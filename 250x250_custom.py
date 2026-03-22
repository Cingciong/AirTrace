#!/usr/bin/env python3
"""
Survey mission controller for PX4 SITL + MAVSDK.

Tryby:
  python survey_mission.py          → survey (główna misja kartograficzna)
  python survey_mission.py test     → 3 losowe przeloty testowe (zigzag)

Geometria:
  • footprint kamery = f(ALT_AGL_M, HFOV, VFOV)
  • line_spacing   = foot_w × (1 − OVERLAP)   ← cross-track (odstęp rzędów)
  • along_spacing  = foot_h × (1 − OVERLAP)   ← along-track (odstęp klatek)
  • SURVEY_SPEED   = stała 1.0 m/s → minimalne przechyły
  • along_overlap weryfikowany w summary po locie

Zakręty POZA obszarem:
  • entry_point = TURNAROUND_EXTRA_M PRZED granicą obszaru
  • exit_point  = TURNAROUND_EXTRA_M ZA granicą obszaru
  • active_windows (nagranie + CSV) włączony TYLKO między start_e a end_e
    (granice obszaru), więc zakręty, hamowanie i przyspieszenie są wycinane.

CSV zawiera:
  timestamp, row_id, gps_lat, gps_lon, gps_alt_agl_m,
  roll_deg, pitch_deg, yaw_deg, ground_speed_mps,
  cross_track_overlap_pct, along_track_overlap_pct
"""

import asyncio
import csv
import math
import os
import random
import re
import shutil
import signal
import subprocess
import threading
import time
from contextlib import suppress
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.gimbal import GimbalMode
from pymavlink import mavutil


# ==========================================================
# PARAMETRY
# ==========================================================
PX4_PATH = "~/PX4-Autopilot"

ALT_AGL_M        = 70.0     # wysokość AGL [m]
AREA_SIZE_M      = 250.0    # bok obszaru [m]
OVERLAP          = 0.70     # wymagane krycie (obie osie)
FPS              = 2.0      # częstotliwość zapisu klatek [Hz]
TELEMETRY_HZ     = 4.0      # próbkowanie telemetrii [Hz]
MAX_TILT_DEG     = 1.0      # max akceptowany przechył klatki [°]

AREA_SHIFT_E_M   = +100.0   # przesunięcie centrum obszaru od home [m E]
AREA_SHIFT_N_M   = +100.0   # przesunięcie centrum obszaru od home [m N]

CAMERA_HFOV_DEG  = 59.49
CAMERA_HFOV_RAD  = math.radians(CAMERA_HFOV_DEG)
CAMERA_ASPECT_W  = 16.0
CAMERA_ASPECT_H  = 9.0

# Prędkość przelotowa — stała niska wartość minimalizująca przechyły
SURVEY_SPEED_MPS = 1.0

# Margines zakrętu poza obszarem [m]
TURNAROUND_EXTRA_M = 30.0

# Czas stabilizacji w punkcie zakrętu (po zatrzymaniu, przed wlotem)
TURN_SETTLE_S    = 2.0

# Tolerancje
ROW_END_TOL_M    = 2.0
TURN_TOL_M       = 3.0

# Losowe przeloty testowe
N_TEST_FLIGHTS   = 3
TEST_WAYPOINTS   = 8

MAVSDK_UDP         = "udpin://0.0.0.0:14540"
MAVLINK_READER_UDP = "udpin:0.0.0.0:14550"


# ==========================================================
# HELPERS
# ==========================================================

def now_str() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 6_371_000.0 * 2.0 * math.asin(math.sqrt(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def compute_vfov(hfov_rad: float, aspect_w: float, aspect_h: float) -> float:
    return 2.0 * math.atan(math.tan(hfov_rad / 2.0) * (aspect_h / aspect_w))


def footprint_wh(alt_m: float, hfov_rad: float, vfov_rad: float) -> Tuple[float, float]:
    w = 2.0 * alt_m * math.tan(hfov_rad / 2.0)
    h = 2.0 * alt_m * math.tan(vfov_rad / 2.0)
    return w, h


# ==========================================================
# GEOMETRIA MISJI
# ==========================================================

class MissionGeometry:
    """Wszystkie obliczenia zależne od parametrów misji — jedno źródło prawdy."""

    def __init__(self):
        self.vfov_rad = compute_vfov(CAMERA_HFOV_RAD, CAMERA_ASPECT_W, CAMERA_ASPECT_H)
        self.foot_w_m, self.foot_h_m = footprint_wh(ALT_AGL_M, CAMERA_HFOV_RAD, self.vfov_rad)

        # Odstęp rzędów i klatek gwarantujący OVERLAP w obu osiach
        self.line_spacing_m  = self.foot_w_m * (1.0 - OVERLAP)   # cross-track
        self.along_spacing_m = self.foot_h_m * (1.0 - OVERLAP)   # along-track

        self.survey_speed_mps = SURVEY_SPEED_MPS

        # Faktyczne krycia przy danej prędkości i FPS
        dist_per_frame = self.survey_speed_mps / FPS
        self.along_overlap_actual = 1.0 - (dist_per_frame / self.foot_h_m)
        self.cross_overlap_actual = 1.0 - (self.line_spacing_m / self.foot_w_m)

    # ---- area bounds ----

    def area_bounds(self) -> Tuple[float, float, float, float]:
        half = AREA_SIZE_M / 2.0
        return (
            -half + AREA_SHIFT_E_M,  # e_min
            +half + AREA_SHIFT_E_M,  # e_max
            -half + AREA_SHIFT_N_M,  # n_min
            +half + AREA_SHIFT_N_M,  # n_max
        )

    # ---- survey segments ----

    def build_survey_segments(self) -> List[Dict[str, Any]]:
        """
        Buduje listę segmentów rzędów.

        Każdy segment:
          entry_e/n  — punkt zakrętu PRZED granicą obszaru
          start_e/n  — granica obszaru (tu włącz nagranie)
          end_e/n    — granica obszaru (tu wyłącz nagranie)
          exit_e/n   — punkt zakrętu ZA granicą obszaru
          yaw        — heading [°]
          row        — numer rzędu (1-based)
        """
        e_min, e_max, n_min, n_max = self.area_bounds()
        spacing = max(2.0, self.line_spacing_m)

        # Środki rzędów wzdłuż N
        row_ns: List[float] = []
        n = n_min + self.foot_h_m / 2.0
        while n <= n_max - self.foot_h_m / 2.0 + 1e-6:
            row_ns.append(n)
            n += spacing
        if not row_ns:
            row_ns = [(n_min + n_max) / 2.0]
        elif (n_max - self.foot_h_m / 2.0) - row_ns[-1] > spacing * 0.35:
            row_ns.append(n_max - self.foot_h_m / 2.0)
        row_ns = [clamp(v, n_min, n_max) for v in row_ns]

        segments: List[Dict[str, Any]] = []
        for i, nrow in enumerate(row_ns):
            ltr = (i % 2 == 0)  # left-to-right
            start_e = e_min if ltr else e_max
            end_e   = e_max if ltr else e_min
            yaw     = 90.0  if ltr else 270.0

            entry_e = start_e - TURNAROUND_EXTRA_M if ltr else start_e + TURNAROUND_EXTRA_M
            exit_e  = end_e   + TURNAROUND_EXTRA_M if ltr else end_e   - TURNAROUND_EXTRA_M

            segments.append({
                "row":     i + 1,
                "yaw":     yaw,
                "entry_e": entry_e, "entry_n": nrow,
                "start_e": start_e, "start_n": nrow,
                "end_e":   end_e,   "end_n":   nrow,
                "exit_e":  exit_e,  "exit_n":  nrow,
            })
        return segments

    # ---- test waypoints ----

    def build_test_waypoints(self, rng: random.Random) -> List[Tuple[float, float]]:
        """Losowe waypoint'y (E, N) wewnątrz obszaru z marginesem footprintu."""
        e_min, e_max, n_min, n_max = self.area_bounds()
        margin = max(self.foot_w_m, self.foot_h_m) / 2.0
        return [
            (rng.uniform(e_min + margin, e_max - margin),
             rng.uniform(n_min + margin, n_max - margin))
            for _ in range(TEST_WAYPOINTS)
        ]

    # ---- summary ----

    def summary_lines(self) -> List[str]:
        return [
            f"alt_agl_m:                {ALT_AGL_M}",
            f"area_size_m:              {AREA_SIZE_M}",
            f"overlap_target:           {OVERLAP:.0%}",
            f"fps:                      {FPS}",
            f"survey_speed_mps:         {self.survey_speed_mps:.2f}",
            f"hfov_deg:                 {CAMERA_HFOV_DEG}",
            f"foot_w_m:                 {self.foot_w_m:.2f}",
            f"foot_h_m:                 {self.foot_h_m:.2f}",
            f"line_spacing_m:           {self.line_spacing_m:.2f}",
            f"along_spacing_m:          {self.along_spacing_m:.2f}",
            f"cross_overlap_actual:     {self.cross_overlap_actual:.1%}",
            f"along_overlap_actual:     {self.along_overlap_actual:.1%}",
        ]


# ==========================================================
# MAVLINK SENSOR READER
# ==========================================================

class MavlinkSensorReader:
    def __init__(self, connection_str: str = MAVLINK_READER_UDP):
        self.connection_str = connection_str
        self._mav = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.connected = False
        self.latest_abs_alt_m: Optional[float] = None
        self.ground_abs_alt_m: Optional[float] = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        with suppress(Exception):
            if self._mav:
                self._mav.close()

    def calibrate_ground(self) -> Optional[float]:
        with self._lock:
            if self.latest_abs_alt_m is not None:
                self.ground_abs_alt_m = self.latest_abs_alt_m
                return self.ground_abs_alt_m
        return None

    def get_agl_m(self) -> float:
        with self._lock:
            if self.latest_abs_alt_m is not None and self.ground_abs_alt_m is not None:
                return self.latest_abs_alt_m - self.ground_abs_alt_m
            return 0.0

    def _run(self):
        try:
            self._mav = mavutil.mavlink_connection(self.connection_str)
            self._mav.wait_heartbeat(timeout=30)
            self.connected = True
            interval_us = int(1e6 / TELEMETRY_HZ)
            with suppress(Exception):
                self._mav.mav.command_long_send(
                    self._mav.target_system, self._mav.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                    interval_us, 0, 0, 0, 0, 0,
                )
            while not self._stop.is_set():
                msg = self._mav.recv_match(type=["GLOBAL_POSITION_INT"], blocking=True, timeout=0.5)
                if msg:
                    with self._lock:
                        self.latest_abs_alt_m = msg.alt / 1000.0
        except Exception:
            self.connected = False


# ==========================================================
# SURVEY RECORDER
# ==========================================================

class SurveyRecorder:
    TELEM_FIELDS = [
        "timestamp", "row_id",
        "gps_lat", "gps_lon", "gps_alt_agl_m",
        "roll_deg", "pitch_deg", "yaw_deg",
        "ground_speed_mps",
        "cross_track_overlap_pct", "along_track_overlap_pct",
    ]
    IMAGE_FIELDS = [
        "image_id", "image_file", "timestamp", "row_id",
        "nearest_telemetry_dt",
        "gps_lat", "gps_lon", "gps_alt_agl_m",
        "roll_deg", "pitch_deg", "yaw_deg",
        "ground_speed_mps",
        "cross_track_overlap_pct", "along_track_overlap_pct",
    ]
    MATCHED_FIELDS = [
        "timestamp", "image_id", "image_file", "image_timestamp",
        "dt_img_telemetry_s", "row_id",
        "gps_lat", "gps_lon", "gps_alt_agl_m",
        "roll_deg", "pitch_deg", "yaw_deg",
        "ground_speed_mps",
        "cross_track_overlap_pct", "along_track_overlap_pct",
    ]

    def __init__(self, folder_base: str, geo: MissionGeometry):
        ts = now_str()
        self.output_dir = f"{folder_base}_{ts}"
        os.makedirs(self.output_dir, exist_ok=True)
        self.img_dir = os.path.join(self.output_dir, "images")
        os.makedirs(self.img_dir, exist_ok=True)

        self.geo  = geo
        self.base = folder_base

        self.telemetry_csv = os.path.join(self.output_dir, f"{folder_base}.csv")
        self.images_csv    = os.path.join(self.output_dir, f"{folder_base}_images.csv")
        self.matched_csv   = os.path.join(self.output_dir, f"{folder_base}_matched.csv")
        self.summary_txt   = os.path.join(self.output_dir, f"{folder_base}_summary.txt")
        self.plot_file     = os.path.join(self.output_dir, f"{folder_base}_trajectory.png")

        self.gst_process: Optional[subprocess.Popen] = None

        self.telemetry_rows:       List[Dict[str, Any]] = []
        self.image_rows:           List[Dict[str, Any]] = []
        self.image_rows_filtered:  List[Dict[str, Any]] = []

        self.t0_wall:     Optional[float] = None
        self.allow_data:  bool            = False
        self._row_id:     int             = 0

        self.active_windows:   List[Tuple[float, float]] = []
        self._window_open_ts:  Optional[float]           = None

        print(f"[REC] Output: {self.output_dir}")

    # ---- time ----

    def start_timebase(self):
        if self.t0_wall is None:
            self.t0_wall = time.time()

    def mission_time(self) -> float:
        return time.time() - self.t0_wall if self.t0_wall else 0.0

    # ---- gate ----

    def set_row_id(self, row_id: int):
        self._row_id = row_id

    def set_allow_data(self, enabled: bool):
        if enabled and not self.allow_data and self.t0_wall is not None:
            self._window_open_ts = self.mission_time()
        if not enabled and self.allow_data and self._window_open_ts is not None:
            t1 = self.mission_time()
            if t1 >= self._window_open_ts:
                self.active_windows.append((self._window_open_ts, t1))
            self._window_open_ts = None
        self.allow_data = enabled
        print(f"[GATE] {'ON ' if enabled else 'OFF'} row={self._row_id}")

    def _in_windows(self, ts: float) -> bool:
        return any(a <= ts <= b for a, b in self.active_windows)

    # ---- capture ----

    def start_capture(self):
        self.start_timebase()
        fps_num = int(FPS * 1000)
        pattern = os.path.join(self.img_dir, "img_%06d.jpg")
        cmd = [
            "gst-launch-1.0", "-e",
            "udpsrc", "port=5600", "buffer-size=2000000",
            "!", "application/x-rtp,encoding-name=H264,payload=96",
            "!", "rtph264depay", "!", "h264parse", "!", "avdec_h264",
            "!", "videorate",
            "!", f"video/x-raw,framerate={fps_num}/1000",
            "!", "jpegenc", "quality=90",
            "!", "multifilesink", f"location={pattern}", "post-messages=true",
        ]
        self.gst_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[REC] Capture @ {FPS:.2f} Hz")

    def stop_capture(self):
        if self.allow_data:
            self.set_allow_data(False)
        if self.gst_process:
            with suppress(Exception):
                self.gst_process.send_signal(signal.SIGINT)
            time.sleep(0.8)
            with suppress(Exception):
                self.gst_process.terminate()
            self.gst_process = None
        print("[REC] Capture stopped")

    # ---- log ----

    def log_telemetry(self, row: Dict[str, Any]):
        """Zawsze loguje — filtrowanie odbywa się w build_image_index."""
        row["row_id"] = self._row_id
        row["cross_track_overlap_pct"] = round(self.geo.cross_overlap_actual * 100, 1)
        row["along_track_overlap_pct"] = round(self.geo.along_overlap_actual * 100, 1)
        self.telemetry_rows.append(row)

    # ---- post-processing ----

    def build_image_index(self):
        """Indeksuje tylko klatki z active_windows, resztę usuwa z dysku."""
        files = sorted(f for f in os.listdir(self.img_dir) if f.lower().endswith(".jpg"))
        rows: List[Dict[str, Any]] = []
        img_id = 0
        for fn in files:
            p = os.path.join(self.img_dir, fn)
            mtime = os.path.getmtime(p)
            ts = mtime - self.t0_wall if self.t0_wall else 0.0
            if self._in_windows(ts):
                rows.append({"image_id": img_id, "image_file": fn, "timestamp": ts})
                img_id += 1
            else:
                with suppress(Exception):
                    os.remove(p)
        self.image_rows = rows

    def filter_images_by_tilt(self, max_dt_s: float = 0.6):
        """Odrzuca klatki z przechyłem > MAX_TILT_DEG lub zbyt daleko od telemetrii."""
        if not self.image_rows or not self.telemetry_rows:
            self.image_rows_filtered = []
            return
        telem = sorted(self.telemetry_rows, key=lambda x: x["timestamp"])
        tts = [t["timestamp"] for t in telem]
        filtered: List[Dict[str, Any]] = []
        j = 0
        for im in sorted(self.image_rows, key=lambda x: x["timestamp"]):
            ts = im["timestamp"]
            while j + 1 < len(tts) and abs(tts[j + 1] - ts) <= abs(tts[j] - ts):
                j += 1
            tr = telem[j]
            dt = abs(tr["timestamp"] - ts)
            ok = (
                dt <= max_dt_s
                and abs(tr["roll_deg"])  <= MAX_TILT_DEG
                and abs(tr["pitch_deg"]) <= MAX_TILT_DEG
            )
            if ok:
                out = dict(im)
                out["nearest_telemetry_dt"] = dt
                for k in ["row_id", "gps_lat", "gps_lon", "gps_alt_agl_m",
                           "roll_deg", "pitch_deg", "yaw_deg",
                           "ground_speed_mps",
                           "cross_track_overlap_pct", "along_track_overlap_pct"]:
                    out[k] = tr.get(k)
                filtered.append(out)
            else:
                with suppress(Exception):
                    os.remove(os.path.join(self.img_dir, im["image_file"]))
        self.image_rows_filtered = filtered

    def match_telemetry_to_images(self, max_dt_s: float = 0.6) -> List[Dict[str, Any]]:
        if not self.telemetry_rows or not self.image_rows_filtered:
            return []
        imgs   = sorted(self.image_rows_filtered, key=lambda x: x["timestamp"])
        img_ts = [i["timestamp"] for i in imgs]
        matched: List[Dict[str, Any]] = []
        j = 0
        for tr in sorted(self.telemetry_rows, key=lambda x: x["timestamp"]):
            t = tr["timestamp"]
            while j + 1 < len(img_ts) and abs(img_ts[j + 1] - t) <= abs(img_ts[j] - t):
                j += 1
            dt = abs(imgs[j]["timestamp"] - t)
            if dt <= max_dt_s:
                matched.append({
                    "timestamp":           tr["timestamp"],
                    "image_id":            imgs[j]["image_id"],
                    "image_file":          imgs[j]["image_file"],
                    "image_timestamp":     imgs[j]["timestamp"],
                    "dt_img_telemetry_s":  dt,
                    **{k: tr.get(k) for k in [
                        "row_id", "gps_lat", "gps_lon", "gps_alt_agl_m",
                        "roll_deg", "pitch_deg", "yaw_deg",
                        "ground_speed_mps",
                        "cross_track_overlap_pct", "along_track_overlap_pct",
                    ]},
                })
        return matched

    # ---- save ----

    @staticmethod
    def _write_csv(path: str, fields: List[str], rows: List[Dict[str, Any]]):
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)

    def save_csvs(self):
        if self.telemetry_rows:
            self._write_csv(self.telemetry_csv, self.TELEM_FIELDS, self.telemetry_rows)
        if self.image_rows_filtered:
            self._write_csv(self.images_csv, self.IMAGE_FIELDS, self.image_rows_filtered)

    def save_matched(self, rows: List[Dict[str, Any]]):
        if rows:
            self._write_csv(self.matched_csv, self.MATCHED_FIELDS, rows)

    def save_summary(self, extra: Optional[List[str]] = None):
        with open(self.summary_txt, "w") as f:
            f.write("MISSION SUMMARY\n")
            f.write(f"time: {datetime.now().isoformat()}\n")
            for line in self.geo.summary_lines():
                f.write(line + "\n")
            f.write(f"active_windows:           {len(self.active_windows)}\n")
            f.write(f"telemetry_rows_total:     {len(self.telemetry_rows)}\n")
            f.write(f"images_raw_in_windows:    {len(self.image_rows)}\n")
            f.write(f"images_after_tilt_filter: {len(self.image_rows_filtered)}\n")
            for line in (extra or []):
                f.write(line + "\n")

    def plot_trajectory(self):
        if not self.telemetry_rows:
            return
        lats = [d["gps_lat"]       for d in self.telemetry_rows]
        lons = [d["gps_lon"]       for d in self.telemetry_rows]
        alts = [d["gps_alt_agl_m"] for d in self.telemetry_rows]
        tss  = [d["timestamp"]     for d in self.telemetry_rows]
        lat0, lon0 = lats[0], lons[0]
        xs = [(lon - lon0) * 111_000 * math.cos(math.radians(lat0)) for lon in lons]
        ys = [(lat - lat0) * 111_000 for lat in lats]

        fig = plt.figure(figsize=(16, 6))
        ax1 = fig.add_subplot(1, 2, 1, projection="3d")
        ax1.plot(xs, ys, alts, lw=1.5, color="tab:blue")
        ax1.scatter(xs[0],  ys[0],  alts[0],  color="green", s=80, label="Start", zorder=5)
        ax1.scatter(xs[-1], ys[-1], alts[-1], color="red",   s=80, label="End",   zorder=5)
        ax1.set_xlabel("East (m)"); ax1.set_ylabel("North (m)"); ax1.set_zlabel("AGL (m)")
        ax1.set_title("3D Trajectory"); ax1.legend()

        ax2 = fig.add_subplot(1, 2, 2)
        ax2.plot(tss, alts, lw=1.5, color="tab:orange")
        ax2.set_xlabel("Time (s)"); ax2.set_ylabel("AGL (m)")
        ax2.set_title("Altitude AGL vs Time"); ax2.grid(True)

        fig.suptitle(self.base, fontsize=14, fontweight="bold")
        plt.tight_layout()
        plt.savefig(self.plot_file, dpi=150)
        plt.close(fig)

    def run_postprocessing(self, extra_summary: Optional[List[str]] = None):
        self.build_image_index()
        self.filter_images_by_tilt(max_dt_s=0.6)
        self.save_csvs()
        self.save_matched(self.match_telemetry_to_images(max_dt_s=0.6))
        self.plot_trajectory()
        self.save_summary(extra=extra_summary)


# ==========================================================
# MISSION CONTROLLER — bazowy
# ==========================================================

class MissionController:
    SPAWN_POSE = "-15,10,0.5,0,0,0"

    def __init__(self, geo: MissionGeometry):
        self.geo = geo
        self.px4_process:    Optional[subprocess.Popen]  = None
        self.sensor_reader:  MavlinkSensorReader         = MavlinkSensorReader()
        self.home_lat:       Optional[float]             = None
        self.home_lon:       Optional[float]             = None
        self.home_abs_alt:   Optional[float]             = None
        self.fov_status:     str                         = "not attempted"
        self._telem_task:    Optional[asyncio.Task]      = None

    # ---- PX4 ----

    def start_px4(self):
        px4_path = os.path.expanduser(PX4_PATH)
        env = {**os.environ, "ROS_DISTRO": "", "ROS_VERSION": "",
               "PX4_GZ_MODEL_POSE": self.SPAWN_POSE}
        self.px4_process = subprocess.Popen(
            ["make", "px4_sitl", "gz_x500_mono_cam_down_baylands"],
            cwd=px4_path, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, env=env,
        )
        for _ in range(70):
            if self.px4_process.poll() is not None:
                raise RuntimeError("PX4 exited during startup.")
            time.sleep(1.0)

    def stop_px4(self):
        if self.px4_process:
            with suppress(Exception):
                self.px4_process.terminate()
            time.sleep(2.0)

    # ---- connect ----

    async def connect(self) -> System:
        drone = System()
        last_err = None
        for _ in range(10):
            try:
                await asyncio.wait_for(drone.connect(system_address=MAVSDK_UDP), timeout=15.0)
                async for st in drone.core.connection_state():
                    if st.is_connected:
                        return drone
            except Exception as e:
                last_err = e
                await asyncio.sleep(2.0)
        raise RuntimeError(f"MAVSDK connect failed: {last_err}")

    # ---- FOV ----

    def _try_gz_service(self) -> bool:
        gz_bin = shutil.which("gz")
        if not gz_bin:
            return False
        svcs = [
            "/world/default/model/x500_mono_cam_down/link/camera_link/sensor/camera/set_camera_info",
            "/world/default/model/x500_mono_cam_down/link/camera_link/sensor/down_camera/set_camera_info",
            "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/set_camera_info",
            "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/down_camera/set_camera_info",
        ]
        req = f"horizontal_fov: {CAMERA_HFOV_RAD}\nwidth: 1920\nheight: 1080\n"
        for s in svcs:
            with suppress(Exception):
                r = subprocess.run(
                    [gz_bin, "service", "-s", s,
                     "--reqtype", "gz.msgs.CameraInfo",
                     "--reptype", "gz.msgs.Boolean",
                     "--timeout", "1000", "--req", req],
                    capture_output=True, text=True, check=False,
                )
                if r.returncode == 0 and "true" in (r.stdout + r.stderr).lower():
                    return True
        return False

    def _patch_sdf(self) -> bool:
        px4_path    = os.path.expanduser(PX4_PATH)
        search_root = os.path.join(px4_path, "Tools", "simulation", "gz", "models")
        if not os.path.isdir(search_root):
            return False
        cands: List[str] = []
        for root, _, files in os.walk(search_root):
            for fn in files:
                if fn.endswith(".sdf") and "x500_mono_cam_down" in fn:
                    cands.append(os.path.join(root, fn))
        if not cands:
            for root, _, files in os.walk(search_root):
                if "x500_mono_cam_down" in root:
                    for fn in files:
                        if fn.endswith(".sdf"):
                            cands.append(os.path.join(root, fn))
        if not cands:
            return False
        cands.sort(key=len)
        sdf_path = cands[0]
        with open(sdf_path, "r", encoding="utf-8") as f:
            txt = f.read()
        bak = sdf_path + ".bak"
        if not os.path.exists(bak):
            with open(bak, "w", encoding="utf-8") as f:
                f.write(txt)
        new_val = f"{CAMERA_HFOV_RAD:.6f}"
        pat = re.compile(r"<horizontal_fov>\s*[^<]+\s*</horizontal_fov>")
        if pat.search(txt):
            txt2 = pat.sub(f"<horizontal_fov>{new_val}</horizontal_fov>", txt, count=1)
        else:
            ci = txt.find("<camera")
            if ci == -1:
                return False
            ip = txt.find(">", ci)
            txt2 = txt[:ip + 1] + f"\n      <horizontal_fov>{new_val}</horizontal_fov>" + txt[ip + 1:]
        with open(sdf_path, "w", encoding="utf-8") as f:
            f.write(txt2)
        return True

    def ensure_camera_fov(self):
        if self._try_gz_service():
            self.fov_status = "set via gz service"; return
        if self._patch_sdf():
            self.fov_status = "patched SDF (restarting PX4)"
            self.stop_px4(); self.start_px4(); return
        self.fov_status = "failed"

    # ---- coordinates ----

    def local_to_global(self, east_m: float, north_m: float) -> Tuple[float, float]:
        lat_per_m = 1.0 / 111_000.0
        lon_per_m = 1.0 / (111_000.0 * math.cos(math.radians(self.home_lat)))
        return self.home_lat + north_m * lat_per_m, self.home_lon + east_m * lon_per_m

    # ---- wait helpers ----

    async def wait_altitude(self, drone: System, target_m: float,
                            tol: float = 2.0, timeout: float = 120.0) -> bool:
        t0 = time.time()
        async for p in drone.telemetry.position():
            if abs(p.relative_altitude_m - target_m) <= tol:
                return True
            if time.time() - t0 > timeout:
                return False
        return False

    async def fly_to(
        self,
        drone:       System,
        lat:         float,
        lon:         float,
        abs_alt:     float,
        yaw:         float,
        pos_tol:     float = TURN_TOL_M,
        stabilize_s: float = 0.0,
        timeout:     float = 600.0,
    ) -> bool:
        """
        goto_location + czekaj na osiągnięcie pozycji.
        Ponawia rozkaz co 5 s zapobiegając zawieszeniu PX4.
        """
        rel_alt = abs_alt - self.home_abs_alt
        await drone.action.goto_location(lat, lon, abs_alt, yaw)
        deadline    = time.time() + timeout
        last_resend = time.time()

        async for p in drone.telemetry.position():
            d  = haversine_m(p.latitude_deg, p.longitude_deg, lat, lon)
            da = abs(p.relative_altitude_m - rel_alt)
            if d <= pos_tol and da <= 2.0:
                if stabilize_s > 0.0:
                    await asyncio.sleep(stabilize_s)
                return True
            now = time.time()
            if now > deadline:
                return False
            if now - last_resend >= 5.0:
                with suppress(Exception):
                    await drone.action.goto_location(lat, lon, abs_alt, yaw)
                last_resend = now
        return False

    # ---- arm / takeoff / land ----

    async def arm(self, drone: System, retries: int = 8):
        for i in range(retries):
            try:
                await drone.action.arm(); return
            except ActionError:
                if i == retries - 1:
                    raise
                await asyncio.sleep(2.5)

    async def takeoff(self, drone: System):
        await self.arm(drone)
        with suppress(Exception):
            await drone.action.set_takeoff_altitude(ALT_AGL_M)
        await drone.action.takeoff()
        await self.wait_altitude(drone, ALT_AGL_M)
        await asyncio.sleep(2.0)
        with suppress(Exception):
            await drone.action.set_maximum_speed(self.geo.survey_speed_mps)
        with suppress(Exception):
            await drone.gimbal.set_mode(GimbalMode.YAW_FOLLOW)
            await drone.gimbal.set_pitch_and_yaw(-90, 0)

    async def land(self, drone: System):
        await drone.action.land()
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                break

    # ---- telemetry loop ----

    async def _telem_loop(self, drone: System, rec: SurveyRecorder):
        latest: Dict[str, Any] = {"pos": None, "att": None, "vel": None}

        async def wp():
            async for p in drone.telemetry.position():
                latest["pos"] = p

        async def wa():
            async for a in drone.telemetry.attitude_euler():
                latest["att"] = a

        async def wv():
            async for v in drone.telemetry.velocity_ned():
                latest["vel"] = v

        tasks = [asyncio.create_task(wp()), asyncio.create_task(wa()), asyncio.create_task(wv())]
        period    = 1.0 / TELEMETRY_HZ
        next_tick = time.perf_counter()
        try:
            while True:
                next_tick += period
                wait = next_tick - time.perf_counter()
                if wait > 0:
                    await asyncio.sleep(wait)
                else:
                    next_tick = time.perf_counter()
                p = latest["pos"]; a = latest["att"]; v = latest["vel"]
                if p is None or a is None or rec.t0_wall is None:
                    continue
                speed = math.sqrt(v.north_m_s ** 2 + v.east_m_s ** 2) if v else 0.0
                rec.log_telemetry({
                    "timestamp":       rec.mission_time(),
                    "gps_lat":         p.latitude_deg,
                    "gps_lon":         p.longitude_deg,
                    "gps_alt_agl_m":   self.sensor_reader.get_agl_m(),
                    "roll_deg":        a.roll_deg,
                    "pitch_deg":       a.pitch_deg,
                    "yaw_deg":         a.yaw_deg,
                    "ground_speed_mps": round(speed, 3),
                })
        except asyncio.CancelledError:
            pass
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    async def start_telemetry(self, drone: System, rec: SurveyRecorder):
        for fn, hz in [
            (drone.telemetry.set_rate_position,       TELEMETRY_HZ),
            (drone.telemetry.set_rate_attitude_euler,  TELEMETRY_HZ),
            (drone.telemetry.set_rate_velocity_ned,    TELEMETRY_HZ),
            (drone.telemetry.set_rate_in_air,          2.0),
        ]:
            with suppress(Exception):
                await fn(hz)
        self._telem_task = asyncio.create_task(self._telem_loop(drone, rec))

    async def stop_telemetry(self):
        if self._telem_task:
            self._telem_task.cancel()
            await asyncio.gather(self._telem_task, return_exceptions=True)
            self._telem_task = None

    # ---- common setup / teardown ----

    async def setup(self, drone: System, rec: SurveyRecorder):
        async for h in drone.telemetry.health():
            if h.is_global_position_ok and h.is_home_position_ok:
                break
        self.sensor_reader.start()
        await asyncio.sleep(2.0)
        async for p in drone.telemetry.position():
            self.home_lat     = p.latitude_deg
            self.home_lon     = p.longitude_deg
            self.home_abs_alt = p.absolute_altitude_m
            break
        self.sensor_reader.calibrate_ground()
        rec.start_timebase()
        await self.start_telemetry(drone, rec)
        rec.start_capture()
        rec.set_allow_data(False)

    async def teardown(self, drone: System, rec: SurveyRecorder):
        rec.stop_capture()
        await self.stop_telemetry()
        self.sensor_reader.stop()


# ==========================================================
# SURVEY MISSION
# ==========================================================

class SurveyMission(MissionController):
    """
    Główna misja kartograficzna.

    Schemat jednego rzędu:
      [entry] → zatrzymaj+stabilizuj → [start | GATE ON]
              → przelot rzędu
              → [end | GATE OFF] → [exit]

    entry i exit leżą TURNAROUND_EXTRA_M poza obszarem.
    """

    def _make_rec(self) -> SurveyRecorder:
        base = (
            f"map_{int(ALT_AGL_M)}m"
            f"_{CAMERA_HFOV_DEG:.1f}fov"
            f"_{int(AREA_SIZE_M)}x{int(AREA_SIZE_M)}m"
            f"_{int(OVERLAP * 100)}pct"
        )
        return SurveyRecorder(folder_base=base, geo=self.geo)

    async def run(self):
        self.start_px4()
        self.ensure_camera_fov()

        drone = await self.connect()
        rec   = self._make_rec()

        await self.setup(drone, rec)
        await self.takeoff(drone)

        segments = self.geo.build_survey_segments()
        abs_alt  = self.home_abs_alt + ALT_AGL_M
        reached  = 0

        for idx, seg in enumerate(segments, start=1):
            rec.set_row_id(idx)
            e_lat,  e_lon  = self.local_to_global(seg["entry_e"], seg["entry_n"])
            s_lat,  s_lon  = self.local_to_global(seg["start_e"], seg["start_n"])
            en_lat, en_lon = self.local_to_global(seg["end_e"],   seg["end_n"])
            x_lat,  x_lon  = self.local_to_global(seg["exit_e"],  seg["exit_n"])
            yaw = seg["yaw"]

            print(f"[ROW {idx}/{len(segments)}] Entry ...")
            rec.set_allow_data(False)
            await self.fly_to(drone, e_lat, e_lon, abs_alt, yaw,
                              pos_tol=TURN_TOL_M, stabilize_s=TURN_SETTLE_S)

            print(f"[ROW {idx}] Wlot → GATE ON")
            await self.fly_to(drone, s_lat, s_lon, abs_alt, yaw, pos_tol=ROW_END_TOL_M)
            rec.set_allow_data(True)

            print(f"[ROW {idx}] Przelot yaw={yaw:.0f}°")
            ok = await self.fly_to(drone, en_lat, en_lon, abs_alt, yaw,
                                   pos_tol=ROW_END_TOL_M, timeout=600.0)
            rec.set_allow_data(False)

            if ok:
                reached += 1
                print(f"[ROW {idx}] ✓  → Exit")
            else:
                print(f"[ROW {idx}] timeout ✗  → Exit")

            await self.fly_to(drone, x_lat, x_lon, abs_alt, yaw, pos_tol=TURN_TOL_M)

        rec.set_allow_data(False)
        await self.land(drone)
        await self.teardown(drone, rec)

        rec.run_postprocessing(extra_summary=[
            f"rows_reached:             {reached}/{len(segments)}",
            f"fov_status:               {self.fov_status}",
        ])

        self.stop_px4()
        print(f"\n[SURVEY] Zakończono. Rzędów: {reached}/{len(segments)}")


# ==========================================================
# TEST MISSION
# ==========================================================

class TestMission(MissionController):
    """
    N_TEST_FLIGHTS losowych zigzag-przelotów nad obszarem.
    Nagranie włączone przez cały przelot (brak podziału na rzędy).
    row_id = numer lotu.
    """

    def _make_rec(self, flight_idx: int) -> SurveyRecorder:
        base = (
            f"test_{flight_idx:02d}"
            f"_{int(ALT_AGL_M)}m"
            f"_{int(AREA_SIZE_M)}x{int(AREA_SIZE_M)}m"
        )
        return SurveyRecorder(folder_base=base, geo=self.geo)

    async def run(self):
        self.start_px4()
        self.ensure_camera_fov()

        drone = await self.connect()
        rng   = random.Random()

        for fi in range(1, N_TEST_FLIGHTS + 1):
            print(f"\n[TEST {fi}/{N_TEST_FLIGHTS}]")
            rec = self._make_rec(fi)

            await self.setup(drone, rec)
            await self.takeoff(drone)

            rec.set_row_id(fi)
            abs_alt   = self.home_abs_alt + ALT_AGL_M
            waypoints = self.geo.build_test_waypoints(rng)

            rec.set_allow_data(True)
            for wi, (e, n) in enumerate(waypoints, start=1):
                lat, lon = self.local_to_global(e, n)
                # Yaw w kierunku następnego waypoint'u
                if wi < len(waypoints):
                    ne_e, ne_n = waypoints[wi]
                    yaw = math.degrees(math.atan2(ne_e - e, ne_n - n)) % 360.0
                else:
                    yaw = 0.0
                print(f"[TEST {fi}] WP {wi}/{len(waypoints)}")
                await self.fly_to(drone, lat, lon, abs_alt, yaw,
                                  pos_tol=ROW_END_TOL_M, timeout=300.0)

            rec.set_allow_data(False)
            await self.land(drone)
            await self.teardown(drone, rec)

            rec.run_postprocessing(extra_summary=[
                f"flight_idx:               {fi}",
                f"waypoints:                {len(waypoints)}",
                f"fov_status:               {self.fov_status}",
            ])

            print(f"[TEST {fi}] Zakończono.")

            # Reset readera między lotami
            self.sensor_reader = MavlinkSensorReader()
            if fi < N_TEST_FLIGHTS:
                await asyncio.sleep(3.0)

        self.stop_px4()
        print(f"\n[TEST] Wszystkie {N_TEST_FLIGHTS} przeloty zakończone.")


# ==========================================================
# ENTRY POINT
# ==========================================================

async def main():
    import sys
    mode = (sys.argv[1] if len(sys.argv) > 1 else "survey").lower()

    geo = MissionGeometry()
    print(f"\n{'=== SURVEY MISSION ===' if mode == 'survey' else '=== TEST FLIGHTS ==='}")
    for line in geo.summary_lines():
        print(" ", line)
    print()

    if mode == "test":
        mc: MissionController = TestMission(geo)
    else:
        mc = SurveyMission(geo)

    try:
        await mc.run()
    except (KeyboardInterrupt, Exception) as e:
        with suppress(Exception):
            mc.sensor_reader.stop()
        with suppress(Exception):
            mc.stop_px4()
        if not isinstance(e, KeyboardInterrupt):
            raise


if __name__ == "__main__":
    asyncio.run(main())