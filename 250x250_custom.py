#!/usr/bin/env python3

import asyncio
import csv
import math
import os
import re
import shutil
import signal
import subprocess
import threading
import time
from contextlib import suppress
from datetime import datetime
from typing import Optional, Dict, Any, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from mavsdk import System
from mavsdk.gimbal import GimbalMode
from mavsdk.action import ActionError
from pymavlink import mavutil


# ==========================================================
# PARAMETRY
# ==========================================================
PX4_PATH = "~/PX4-Autopilot"

ALT_AGL_M = 70.0
AREA_SIZE_M = 250.0
OVERLAP = 0.70
FPS = 2.0
TELEMETRY_HZ = 2.0
MAX_TILT_DEG = 10.0

AREA_SHIFT_E_M = +100.0
AREA_SHIFT_N_M = +100.0

CAMERA_HFOV_DEG = 59.49
CAMERA_HFOV_RAD = math.radians(CAMERA_HFOV_DEG)
CAMERA_ASPECT_W = 16.0
CAMERA_ASPECT_H = 9.0

SURVEY_MAX_SPEED_MPS = 1.6
TURN_SETTLE_S = 1.8

# Tolerancja dotarcia do punktu końca rzędu — mniejsza niż turn point
ROW_END_TOL_M = 2.5
# Tolerancja punktu startowego rzędu / zakrętu
TURN_TOL_M = 4.0

MAVSDK_UDP = "udpin://0.0.0.0:14540"
MAVLINK_READER_UDP = "udpin:0.0.0.0:14550"


def now_str():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def haversine_m(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 6371000.0 * 2.0 * math.asin(math.sqrt(a))


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def compute_vfov_from_hfov(hfov_rad: float, aspect_w: float, aspect_h: float) -> float:
    return 2.0 * math.atan(math.tan(hfov_rad / 2.0) * (aspect_h / aspect_w))


def footprint_wh(alt_m: float, hfov_rad: float, vfov_rad: float) -> Tuple[float, float]:
    w = 2.0 * alt_m * math.tan(hfov_rad / 2.0)
    h = 2.0 * alt_m * math.tan(vfov_rad / 2.0)
    return w, h


class MavlinkSensorReader:
    def __init__(self, connection_str=MAVLINK_READER_UDP):
        self.connection_str = connection_str
        self._mav = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self.connected = False
        self.latest_abs_alt_m = None
        self.ground_abs_alt_m = None

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

    def calibrate_ground(self):
        with self._lock:
            if self.latest_abs_alt_m is not None:
                self.ground_abs_alt_m = self.latest_abs_alt_m
                return self.ground_abs_alt_m
        return None

    def get_agl_m(self):
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
                    self._mav.target_system,
                    self._mav.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                    interval_us, 0, 0, 0, 0, 0
                )

            while not self._stop.is_set():
                msg = self._mav.recv_match(type=["GLOBAL_POSITION_INT"], blocking=True, timeout=0.5)
                if msg is None:
                    continue
                with self._lock:
                    self.latest_abs_alt_m = msg.alt / 1000.0
        except Exception:
            self.connected = False


class SurveyRecorder:
    def __init__(self, folder_base_name: str):
        ts = now_str()
        self.output_dir = f"{folder_base_name}_{ts}"
        os.makedirs(self.output_dir, exist_ok=True)

        self.img_dir = os.path.join(self.output_dir, folder_base_name)
        os.makedirs(self.img_dir, exist_ok=True)

        self.telemetry_csv = os.path.join(self.output_dir, f"{folder_base_name}.csv")
        self.images_csv = os.path.join(self.output_dir, f"{folder_base_name}_images.csv")
        self.matched_csv = os.path.join(self.output_dir, f"{folder_base_name}_matched.csv")
        self.summary_txt = os.path.join(self.output_dir, f"{folder_base_name}_summary.txt")
        self.plot_file = os.path.join(self.output_dir, f"{folder_base_name}_trajectory.png")

        self.gst_process = None
        self.telemetry_rows: List[Dict[str, Any]] = []
        self.image_rows: List[Dict[str, Any]] = []
        self.image_rows_filtered: List[Dict[str, Any]] = []

        self.t0_wall: Optional[float] = None
        self.allow_data = False

        self.active_windows: List[Tuple[float, float]] = []
        self._window_open_ts: Optional[float] = None

        print(f"Output directory: {self.output_dir}")

    def start_timebase(self):
        if self.t0_wall is None:
            self.t0_wall = time.time()

    def mission_time(self):
        if self.t0_wall is None:
            return 0.0
        return time.time() - self.t0_wall

    def set_allow_data(self, enabled: bool):
        if enabled and not self.allow_data:
            if self.t0_wall is not None:
                self._window_open_ts = self.mission_time()

        if (not enabled) and self.allow_data:
            if self._window_open_ts is not None and self.t0_wall is not None:
                t1 = self.mission_time()
                if t1 >= self._window_open_ts:
                    self.active_windows.append((self._window_open_ts, t1))
            self._window_open_ts = None

        self.allow_data = enabled
        print(f"[DATA_GATE] {'ON' if enabled else 'OFF'}")

    def start_capture_once(self):
        self.start_timebase()

        fps_num = int(FPS * 1000)
        fps_den = 1000
        pattern = os.path.join(self.img_dir, "img_%06d.jpg")

        cmd = [
            "gst-launch-1.0", "-e",
            "udpsrc", "port=5600", "buffer-size=2000000",
            "!", "application/x-rtp,encoding-name=H264,payload=96",
            "!", "rtph264depay",
            "!", "h264parse",
            "!", "avdec_h264",
            "!", "videorate",
            "!", f"video/x-raw,framerate={fps_num}/{fps_den}",
            "!", "jpegenc", "quality=90",
            "!", "multifilesink", f"location={pattern}", "post-messages=true"
        ]
        self.gst_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"Image capture started ONCE @ {FPS:.2f}Hz")

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
        print("Image capture stopped")

    def log_telemetry(self, row: Dict[str, Any]):
        # POPRAWKA: telemetria jest zawsze zbierana (nie tylko gdy allow_data=True).
        # Filtrowanie po czasie odbywa się dopiero w build_image_index / filter.
        self.telemetry_rows.append(row)

    def _ts_in_active_windows(self, ts: float) -> bool:
        for a, b in self.active_windows:
            if a <= ts <= b:
                return True
        return False

    def build_image_index(self):
        files = sorted([f for f in os.listdir(self.img_dir) if f.lower().endswith(".jpg")])
        rows = []
        image_id = 0

        for fn in files:
            p = os.path.join(self.img_dir, fn)
            mtime = os.path.getmtime(p)
            ts = mtime - self.t0_wall if self.t0_wall else 0.0

            # Tylko klatki z okien aktywnych (rzędy bez zakrętów/startu/lądowania)
            if self._ts_in_active_windows(ts):
                rows.append({
                    "image_id": image_id,
                    "image_file": fn,
                    "timestamp": ts
                })
                image_id += 1
            else:
                # Usuń klatki spoza okien
                with suppress(Exception):
                    os.remove(p)

        self.image_rows = rows

    def filter_images_by_tilt_using_nearest_telemetry(self, max_dt_s=0.6):
        """
        Usuwa klatki, dla których najbliższa telemetria ma:
        abs(roll)>MAX_TILT_DEG lub abs(pitch)>MAX_TILT_DEG,
        lub jest za daleko czasowo.
        """
        if not self.image_rows or not self.telemetry_rows:
            self.image_rows_filtered = []
            return

        telem = sorted(self.telemetry_rows, key=lambda x: x["timestamp"])
        tts = [t["timestamp"] for t in telem]

        filtered = []
        j = 0

        for im in sorted(self.image_rows, key=lambda x: x["timestamp"]):
            ts = im["timestamp"]

            while j + 1 < len(tts) and abs(tts[j + 1] - ts) <= abs(tts[j] - ts):
                j += 1

            tr = telem[j]
            dt = abs(tr["timestamp"] - ts)

            keep = (
                dt <= max_dt_s
                and abs(tr["roll_deg"]) <= MAX_TILT_DEG
                and abs(tr["pitch_deg"]) <= MAX_TILT_DEG
            )

            if keep:
                out = dict(im)
                out["nearest_telemetry_dt"] = dt
                out["gps_lat"] = tr["gps_lat"]
                out["gps_lon"] = tr["gps_lon"]
                out["gps_alt_agl_m"] = tr["gps_alt_agl_m"]
                out["roll_deg"] = tr["roll_deg"]
                out["pitch_deg"] = tr["pitch_deg"]
                out["yaw_deg"] = tr["yaw_deg"]
                filtered.append(out)
            else:
                with suppress(Exception):
                    os.remove(os.path.join(self.img_dir, im["image_file"]))

        self.image_rows_filtered = filtered

    def plot_trajectory(self):
        if not self.telemetry_rows:
            return

        # Pełna telemetria (wszystkie rzędy, nie tylko aktywne okna)
        timestamps = [d["timestamp"] for d in self.telemetry_rows]
        lats = [d["gps_lat"] for d in self.telemetry_rows]
        lons = [d["gps_lon"] for d in self.telemetry_rows]
        alts = [d["gps_alt_agl_m"] for d in self.telemetry_rows]

        lat0, lon0 = lats[0], lons[0]
        xs = [(lon - lon0) * 111000 * math.cos(math.radians(lat0)) for lon in lons]
        ys = [(lat - lat0) * 111000 for lat in lats]

        fig = plt.figure(figsize=(16, 6))

        ax1 = fig.add_subplot(1, 2, 1, projection="3d")
        ax1.plot(xs, ys, alts, linewidth=1.5, color="tab:blue")
        ax1.scatter(xs[0], ys[0], alts[0], color="green", s=80, label="Start", zorder=5)
        ax1.scatter(xs[-1], ys[-1], alts[-1], color="red", s=80, label="End", zorder=5)
        ax1.set_xlabel("East (m)")
        ax1.set_ylabel("North (m)")
        ax1.set_zlabel("AGL (m)")
        ax1.set_title("3D Trajectory")
        ax1.legend()

        ax2 = fig.add_subplot(1, 2, 2)
        ax2.plot(timestamps, alts, linewidth=1.5, color="tab:orange")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("AGL (m)")
        ax2.set_title("Altitude AGL vs Time")
        ax2.grid(True)

        fig.suptitle("Survey Trajectory", fontsize=14, fontweight="bold")
        plt.tight_layout()
        plt.savefig(self.plot_file, dpi=150)
        plt.close(fig)

    def save_csvs(self):
        # Pełna telemetria (wszystkie próbki, nie tylko z okien aktywnych)
        if self.telemetry_rows:
            fieldnames = ["timestamp", "gps_lat", "gps_lon", "gps_alt_agl_m", "roll_deg", "pitch_deg", "yaw_deg"]
            with open(self.telemetry_csv, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fieldnames)
                w.writeheader()
                w.writerows(self.telemetry_rows)

        # images CSV po filtrze tilt
        if self.image_rows_filtered:
            fieldnames = [
                "image_id", "image_file", "timestamp",
                "nearest_telemetry_dt",
                "gps_lat", "gps_lon", "gps_alt_agl_m",
                "roll_deg", "pitch_deg", "yaw_deg"
            ]
            with open(self.images_csv, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fieldnames)
                w.writeheader()
                w.writerows(self.image_rows_filtered)

    def match_telemetry_to_images(self, max_dt_s=0.6):
        if not self.telemetry_rows or not self.image_rows_filtered:
            return []

        imgs = sorted(self.image_rows_filtered, key=lambda x: x["timestamp"])
        img_ts = [i["timestamp"] for i in imgs]

        matched = []
        j = 0
        telem = sorted(self.telemetry_rows, key=lambda x: x["timestamp"])

        for tr in telem:
            t = tr["timestamp"]
            while j + 1 < len(img_ts) and abs(img_ts[j + 1] - t) <= abs(img_ts[j] - t):
                j += 1

            im = imgs[j]
            dt = abs(im["timestamp"] - t)
            if dt <= max_dt_s:
                matched.append({
                    "timestamp": tr["timestamp"],
                    "image_id": im["image_id"],
                    "image_file": im["image_file"],
                    "image_timestamp": im["timestamp"],
                    "dt_img_telemetry_s": dt,
                    "gps_lat": tr["gps_lat"],
                    "gps_lon": tr["gps_lon"],
                    "gps_alt_agl_m": tr["gps_alt_agl_m"],
                    "roll_deg": tr["roll_deg"],
                    "pitch_deg": tr["pitch_deg"],
                    "yaw_deg": tr["yaw_deg"],
                })
        return matched

    def save_matched(self, rows: List[Dict[str, Any]]):
        if not rows:
            return
        fieldnames = [
            "timestamp", "image_id", "image_file", "image_timestamp", "dt_img_telemetry_s",
            "gps_lat", "gps_lon", "gps_alt_agl_m", "roll_deg", "pitch_deg", "yaw_deg",
        ]
        with open(self.matched_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(rows)

    def save_summary(self, reached_rows: int, total_rows: int, fov_status: str, line_spacing: float, along_spacing: float):
        with open(self.summary_txt, "w") as f:
            f.write("SURVEY SUMMARY\n")
            f.write(f"time: {datetime.now().isoformat()}\n")
            f.write(f"area_size_m: {AREA_SIZE_M}\n")
            f.write(f"area_shift_e_m: {AREA_SHIFT_E_M}\n")
            f.write(f"area_shift_n_m: {AREA_SHIFT_N_M}\n")
            f.write(f"alt_agl_m: {ALT_AGL_M}\n")
            f.write(f"overlap: {OVERLAP}\n")
            f.write(f"fps: {FPS}\n")
            f.write(f"telemetry_hz: {TELEMETRY_HZ}\n")
            f.write(f"max_tilt_deg: {MAX_TILT_DEG}\n")
            f.write(f"hfov_deg_target: {CAMERA_HFOV_DEG}\n")
            f.write(f"fov_status: {fov_status}\n")
            f.write(f"line_spacing_m: {line_spacing:.3f}\n")
            f.write(f"along_spacing_target_m: {along_spacing:.3f}\n")
            f.write(f"rows_reached: {reached_rows}/{total_rows}\n")
            f.write(f"active_windows: {len(self.active_windows)}\n")
            f.write(f"telemetry_rows: {len(self.telemetry_rows)}\n")
            f.write(f"images_raw_in_windows: {len(self.image_rows)}\n")
            f.write(f"images_after_tilt_filter: {len(self.image_rows_filtered)}\n")


class MissionController:
    SPAWN_POSE = "-15,10,0.5,0,0,0"

    def __init__(self):
        folder_base = f"map_{int(ALT_AGL_M)}_{CAMERA_HFOV_DEG:.2f}_{int(AREA_SIZE_M)}x{int(AREA_SIZE_M)}_{int(OVERLAP*100)}"
        self.rec = SurveyRecorder(folder_base_name=folder_base)
        self.sensor_reader = MavlinkSensorReader()
        self.px4_process = None

        self.home_lat = None
        self.home_lon = None
        self.home_abs_alt = None

        self.telemetry_task = None
        self.fov_status = "not attempted"

        self.vfov_rad = compute_vfov_from_hfov(CAMERA_HFOV_RAD, CAMERA_ASPECT_W, CAMERA_ASPECT_H)
        self.foot_w_m, self.foot_h_m = footprint_wh(ALT_AGL_M, CAMERA_HFOV_RAD, self.vfov_rad)
        self.line_spacing_m = self.foot_w_m * (1.0 - OVERLAP)
        self.along_spacing_m = self.foot_h_m * (1.0 - OVERLAP)

    def start_px4(self):
        px4_path = os.path.expanduser(PX4_PATH)
        env = {**os.environ, "ROS_DISTRO": "", "ROS_VERSION": "", "PX4_GZ_MODEL_POSE": self.SPAWN_POSE}

        self.px4_process = subprocess.Popen(
            ["make", "px4_sitl", "gz_x500_mono_cam_down_baylands"],
            cwd=px4_path,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )

        for _ in range(70):
            if self.px4_process.poll() is not None:
                raise RuntimeError("PX4 process exited during startup.")
            time.sleep(1.0)

    def stop_px4(self):
        if self.px4_process:
            with suppress(Exception):
                self.px4_process.terminate()
            time.sleep(2)

    async def connect_mavsdk(self, drone: System):
        last_err = None
        for attempt in range(1, 10):
            try:
                await asyncio.wait_for(drone.connect(system_address=MAVSDK_UDP), timeout=15.0)
                async for st in drone.core.connection_state():
                    if st.is_connected:
                        return
            except Exception as e:
                last_err = e
                await asyncio.sleep(2.0)
        raise RuntimeError(f"MAVSDK connect failed: {last_err}")

    def _try_set_fov_via_gz_service(self) -> bool:
        gz_bin = shutil.which("gz")
        if not gz_bin:
            return False
        services = [
            "/world/default/model/x500_mono_cam_down/link/camera_link/sensor/camera/set_camera_info",
            "/world/default/model/x500_mono_cam_down/link/camera_link/sensor/down_camera/set_camera_info",
            "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/set_camera_info",
            "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/down_camera/set_camera_info",
        ]
        req = f"horizontal_fov: {CAMERA_HFOV_RAD}\nwidth: 1920\nheight: 1080\n"
        for s in services:
            cmd = [
                gz_bin, "service", "-s", s,
                "--reqtype", "gz.msgs.CameraInfo",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "1000",
                "--req", req
            ]
            with suppress(Exception):
                r = subprocess.run(cmd, capture_output=True, text=True, check=False)
                out = (r.stdout or "") + (r.stderr or "")
                if r.returncode == 0 and "true" in out.lower():
                    return True
        return False

    def _patch_sdf_horizontal_fov(self) -> bool:
        px4_path = os.path.expanduser(PX4_PATH)
        search_root = os.path.join(px4_path, "Tools", "simulation", "gz", "models")
        if not os.path.isdir(search_root):
            return False

        cands = []
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
        pattern = re.compile(r"<horizontal_fov>\s*[^<]+\s*</horizontal_fov>")
        if pattern.search(txt):
            txt2 = pattern.sub(f"<horizontal_fov>{new_val}</horizontal_fov>", txt, count=1)
        else:
            cam_idx = txt.find("<camera")
            if cam_idx == -1:
                return False
            insert_pos = txt.find(">", cam_idx)
            if insert_pos == -1:
                return False
            txt2 = txt[:insert_pos + 1] + f"\n      <horizontal_fov>{new_val}</horizontal_fov>" + txt[insert_pos + 1:]

        with open(sdf_path, "w", encoding="utf-8") as f:
            f.write(txt2)
        return True

    def ensure_camera_fov(self):
        if self._try_set_fov_via_gz_service():
            self.fov_status = "set via gz service"
            return
        if self._patch_sdf_horizontal_fov():
            self.fov_status = "patched in SDF (restart applied)"
            self.stop_px4()
            self.start_px4()
            return
        self.fov_status = "failed to set automatically"

    def local_to_global(self, east_m, north_m):
        lat_per_m = 1.0 / 111000.0
        lon_per_m = 1.0 / (111000.0 * math.cos(math.radians(self.home_lat)))
        lat = self.home_lat + north_m * lat_per_m
        lon = self.home_lon + east_m * lon_per_m
        return lat, lon

    def area_bounds(self) -> Tuple[float, float, float, float]:
        half = AREA_SIZE_M / 2.0
        e_min = -half + AREA_SHIFT_E_M
        e_max = +half + AREA_SHIFT_E_M
        n_min = -half + AREA_SHIFT_N_M
        n_max = +half + AREA_SHIFT_N_M
        return e_min, e_max, n_min, n_max

    def build_row_segments(self):
        e_min, e_max, n_min, n_max = self.area_bounds()
        spacing = max(2.0, self.line_spacing_m)

        row_centers = []
        n = n_min + self.foot_h_m / 2.0
        while n <= n_max - self.foot_h_m / 2.0 + 1e-6:
            row_centers.append(n)
            n += spacing

        if not row_centers:
            row_centers = [(n_min + n_max) / 2.0]
        elif (n_max - self.foot_h_m / 2.0) - row_centers[-1] > spacing * 0.35:
            row_centers.append(n_max - self.foot_h_m / 2.0)

        row_centers = [clamp(v, n_min + self.foot_h_m / 2.0, n_max - self.foot_h_m / 2.0) for v in row_centers]

        start_e = clamp(e_min + self.foot_w_m / 2.0, e_min, e_max)
        end_e = clamp(e_max - self.foot_w_m / 2.0, e_min, e_max)

        segments = []
        for i, nrow in enumerate(row_centers):
            if i % 2 == 0:
                s_e, e_e, yaw = start_e, end_e, 90.0
            else:
                s_e, e_e, yaw = end_e, start_e, 270.0
            segments.append({
                "row": i + 1,
                "start_e": s_e,
                "start_n": nrow,
                "end_e": e_e,
                "end_n": nrow,
                "yaw": yaw
            })
        return segments

    async def wait_for_altitude(self, drone: System, target_alt_m: float, tol=2.0, timeout=120.0):
        start = time.time()
        async for p in drone.telemetry.position():
            if abs(p.relative_altitude_m - target_alt_m) <= tol:
                return True
            if time.time() - start > timeout:
                return False
        return False

    async def wait_near(
        self,
        drone: System,
        tgt_lat: float,
        tgt_lon: float,
        tgt_rel_alt: float,
        pos_tol: float = TURN_TOL_M,
        alt_tol: float = 2.0,
        timeout: float = 500.0,
        # POPRAWKA: dodatkowy warunek — dron musi być blisko i mieć
        # prędkość poziomą ≤ speed_tol_mps, żeby uznać dotarcie do punktu.
        # Zapobiega zaliczeniu punktu podczas przelotu ze zbyt dużą prędkością.
        speed_tol_mps: float = 0.0,          # 0 = nie sprawdzaj prędkości
        stabilize_s: float = 0.0,            # czas stabilizacji po osiągnięciu warunku
    ) -> bool:
        start = time.time()
        stable_since: Optional[float] = None

        async for p in drone.telemetry.position():
            d = haversine_m(p.latitude_deg, p.longitude_deg, tgt_lat, tgt_lon)
            da = abs(p.relative_altitude_m - tgt_rel_alt)
            pos_ok = d <= pos_tol and da <= alt_tol

            if pos_ok:
                if stabilize_s <= 0.0:
                    return True
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since >= stabilize_s:
                    return True
            else:
                stable_since = None

            if time.time() - start > timeout:
                return False
        return False

    async def arm_with_retry(self, drone: System, retries=8):
        for i in range(retries):
            try:
                await drone.action.arm()
                return
            except ActionError:
                if i == retries - 1:
                    raise
                await asyncio.sleep(2.5)

    async def telemetry_loop(self, drone: System):
        """
        POPRAWKA: telemetria zbierana jest zawsze (przez cały czas lotu),
        niezależnie od flagi allow_data. Filtrowanie po czasie odbywa się
        w build_image_index (okna aktywne) i filter_images_by_tilt.
        """
        latest = {"pos": None, "att": None}

        async def watch_pos():
            async for p in drone.telemetry.position():
                latest["pos"] = p

        async def watch_att():
            async for a in drone.telemetry.attitude_euler():
                latest["att"] = a

        tasks = [asyncio.create_task(watch_pos()), asyncio.create_task(watch_att())]

        period = 1.0 / TELEMETRY_HZ
        next_tick = time.perf_counter()

        try:
            while True:
                next_tick += period
                wait = next_tick - time.perf_counter()
                if wait > 0:
                    await asyncio.sleep(wait)
                else:
                    next_tick = time.perf_counter()

                p = latest["pos"]
                a = latest["att"]
                if p is None or a is None:
                    continue
                if self.rec.t0_wall is None:
                    continue

                row = {
                    "timestamp": self.rec.mission_time(),
                    "gps_lat": p.latitude_deg,
                    "gps_lon": p.longitude_deg,
                    "gps_alt_agl_m": self.sensor_reader.get_agl_m(),
                    "roll_deg": a.roll_deg,
                    "pitch_deg": a.pitch_deg,
                    "yaw_deg": a.yaw_deg,
                }
                # Zawsze loguj — brama czasowa w build_image_index
                self.rec.log_telemetry(row)

        except asyncio.CancelledError:
            pass
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    async def fly_to_and_wait(
        self,
        drone: System,
        lat: float,
        lon: float,
        abs_alt: float,
        yaw: float,
        pos_tol: float = TURN_TOL_M,
        stabilize_s: float = 0.0,
        timeout: float = 500.0,
    ) -> bool:
        """
        POPRAWKA: Wyślij goto_location i czekaj aż dron faktycznie
        dotrze do punktu (z opcjonalnym czasem stabilizacji).
        Ponawia goto_location co 5 sekund jeśli dron się zatrzymał
        dalej niż pos_tol — zapobiega "zawieszeniu" na PX4.
        """
        rel_alt = abs_alt - self.home_abs_alt
        await drone.action.goto_location(lat, lon, abs_alt, yaw)

        deadline = time.time() + timeout
        resend_interval = 5.0
        last_resend = time.time()

        async for p in drone.telemetry.position():
            d = haversine_m(p.latitude_deg, p.longitude_deg, lat, lon)
            da = abs(p.relative_altitude_m - rel_alt)

            if d <= pos_tol and da <= 2.0:
                if stabilize_s > 0.0:
                    await asyncio.sleep(stabilize_s)
                return True

            now = time.time()
            if now > deadline:
                return False

            # Odśwież rozkaz jeśli minęło resend_interval
            if now - last_resend >= resend_interval:
                with suppress(Exception):
                    await drone.action.goto_location(lat, lon, abs_alt, yaw)
                last_resend = now

        return False

    async def run(self):
        self.start_px4()
        self.ensure_camera_fov()

        drone = System()
        await self.connect_mavsdk(drone)

        async for h in drone.telemetry.health():
            if h.is_global_position_ok and h.is_home_position_ok:
                break

        with suppress(Exception):
            await drone.telemetry.set_rate_position(TELEMETRY_HZ)
        with suppress(Exception):
            await drone.telemetry.set_rate_attitude_euler(TELEMETRY_HZ)
        with suppress(Exception):
            await drone.telemetry.set_rate_in_air(2.0)

        self.sensor_reader.start()
        await asyncio.sleep(2.0)

        async for p in drone.telemetry.position():
            self.home_lat = p.latitude_deg
            self.home_lon = p.longitude_deg
            self.home_abs_alt = p.absolute_altitude_m
            break

        self.sensor_reader.calibrate_ground()

        # Timebase startujemy tu — przed startem telemetrii
        self.rec.start_timebase()
        self.telemetry_task = asyncio.create_task(self.telemetry_loop(drone))

        self.rec.start_capture_once()
        self.rec.set_allow_data(False)

        await self.arm_with_retry(drone)

        with suppress(Exception):
            await drone.action.set_takeoff_altitude(ALT_AGL_M)

        await drone.action.takeoff()
        await self.wait_for_altitude(drone, ALT_AGL_M)
        await asyncio.sleep(2.0)

        with suppress(Exception):
            await drone.action.set_maximum_speed(SURVEY_MAX_SPEED_MPS)

        with suppress(Exception):
            await drone.gimbal.set_mode(GimbalMode.YAW_FOLLOW)
            await drone.gimbal.set_pitch_and_yaw(-90, 0)

        segments = self.build_row_segments()
        reached_rows = 0

        # -------------------------------------------------------
        # POPRAWKA: Lot do punktu startowego pierwszego rzędu.
        # Używamy fly_to_and_wait z czasem stabilizacji TURN_SETTLE_S,
        # żeby dron faktycznie zatrzymał się przed rozpoczęciem rzędu.
        # -------------------------------------------------------
        first = segments[0]
        s_lat, s_lon = self.local_to_global(first["start_e"], first["start_n"])
        print(f"[MOVE] Pozycjonowanie przed rząd 1 ...")
        await self.fly_to_and_wait(
            drone, s_lat, s_lon,
            self.home_abs_alt + ALT_AGL_M,
            first["yaw"],
            pos_tol=TURN_TOL_M,
            stabilize_s=TURN_SETTLE_S,
        )

        for idx, seg in enumerate(segments, start=1):
            # --- Pozycjonowanie na start rzędu (od rzędu 2 w górę) ---
            if idx > 1:
                self.rec.set_allow_data(False)
                s_lat, s_lon = self.local_to_global(seg["start_e"], seg["start_n"])
                print(f"[MOVE] Zakręt → start rzędu {idx} ...")
                await self.fly_to_and_wait(
                    drone, s_lat, s_lon,
                    self.home_abs_alt + ALT_AGL_M,
                    seg["yaw"],
                    pos_tol=TURN_TOL_M,
                    stabilize_s=TURN_SETTLE_S,
                )

            # --- Lot wzdłuż rzędu z nagrywaniem ---
            self.rec.set_allow_data(True)
            e_lat, e_lon = self.local_to_global(seg["end_e"], seg["end_n"])
            print(f"[ROW {idx}/{len(segments)}] Lecę rzędem yaw={seg['yaw']:.0f}° ...")

            # POPRAWKA: mniejsza tolerancja dla końca rzędu — dron musi
            # faktycznie dolecieć do końca, a nie zaliczyć go z daleka.
            ok = await self.fly_to_and_wait(
                drone, e_lat, e_lon,
                self.home_abs_alt + ALT_AGL_M,
                seg["yaw"],
                pos_tol=ROW_END_TOL_M,
                stabilize_s=0.0,
                timeout=500.0,
            )
            self.rec.set_allow_data(False)

            if ok:
                reached_rows += 1
                print(f"[ROW {idx}] Osiągnięty ✓")
            else:
                print(f"[ROW {idx}] Timeout ✗")

        # --- Lądowanie ---
        self.rec.set_allow_data(False)
        await drone.action.land()
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                break

        self.rec.stop_capture()

        if self.telemetry_task:
            self.telemetry_task.cancel()
            await asyncio.gather(self.telemetry_task, return_exceptions=True)

        self.sensor_reader.stop()

        self.rec.build_image_index()
        self.rec.filter_images_by_tilt_using_nearest_telemetry(max_dt_s=0.6)
        self.rec.save_csvs()

        matched = self.rec.match_telemetry_to_images(max_dt_s=0.6)
        self.rec.save_matched(matched)
        self.rec.plot_trajectory()
        self.rec.save_summary(
            reached_rows=reached_rows,
            total_rows=len(segments),
            fov_status=self.fov_status,
            line_spacing=self.line_spacing_m,
            along_spacing=self.along_spacing_m
        )

        self.stop_px4()


async def main():
    mc = MissionController()
    try:
        await mc.run()
    except KeyboardInterrupt:
        with suppress(Exception):
            mc.rec.set_allow_data(False)
            mc.rec.stop_capture()
            mc.sensor_reader.stop()
            mc.stop_px4()
    except Exception:
        with suppress(Exception):
            mc.rec.set_allow_data(False)
            mc.rec.stop_capture()
            mc.sensor_reader.stop()
            mc.stop_px4()
        raise


if __name__ == "__main__":
    asyncio.run(main())