#!/usr/bin/env python3

import asyncio
import subprocess
import os
import signal
import time
import math
import csv
import shutil
import json
import threading
from contextlib import suppress

from mavsdk import System
from mavsdk.gimbal import GimbalMode
from mavsdk.action import ActionError

from pymavlink import mavutil


class RawBaroReader:
    """
    RAW barometr z MAVLink SCALED_PRESSURE.press_abs [hPa].
    Słucha na OSOBNYM porcie (14550), żeby nie kolidować z MAVSDK (14540).
    """
    def __init__(self, connection_str: str = "udpin:0.0.0.0:14550"):
        self.connection_str = connection_str
        self._mav = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self.latest_press_hpa = None
        self.connected = False

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

    def get_latest_pressure_hpa(self):
        with self._lock:
            return self.latest_press_hpa

    def _run(self):
        try:
            self._mav = mavutil.mavlink_connection(self.connection_str)
            self._mav.wait_heartbeat(timeout=30)
            self.connected = True

            with suppress(Exception):
                self._mav.mav.command_long_send(
                    self._mav.target_system,
                    self._mav.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
                    33333,  # µs -> ~30 Hz
                    0, 0, 0, 0, 0
                )

            while not self._stop.is_set():
                msg = self._mav.recv_match(type=["SCALED_PRESSURE"], blocking=True, timeout=0.5)
                if msg is None:
                    continue
                with self._lock:
                    self.latest_press_hpa = float(msg.press_abs)

        except Exception:
            self.connected = False


class DroneRecorder:
    def __init__(self, px4_path="~/PX4-Autopilot", output_dir="mission_data"):
        self.px4_path = os.path.expanduser(px4_path)
        self.timestamp = time.strftime("%Y%m%d_%H%M%S")

        self.output_dir = f"{output_dir}_{self.timestamp}"
        os.makedirs(self.output_dir, exist_ok=True)

        self.video_mkv = os.path.join(self.output_dir, f"video_{self.timestamp}.mkv")
        self.video_mp4 = os.path.join(self.output_dir, f"mission_{self.timestamp}.mp4")
        self.telemetry_csv = os.path.join(self.output_dir, f"telemetry_{self.timestamp}.csv")
        self.summary_file = os.path.join(self.output_dir, f"summary_{self.timestamp}.txt")

        self.px4_process = None
        self.gst_process = None

        self.telemetry_data = []

        self.video_t0_wall = None
        self.mission_start_ts = None
        self.mission_end_ts = None

        print(f"Data directory: {self.output_dir}/")

    def start_mission_window(self):
        if self.mission_start_ts is None:
            self.mission_start_ts = time.time()

    def end_mission_window(self):
        if self.mission_end_ts is None:
            self.mission_end_ts = time.time()

    def start_px4(self):
        print("Starting PX4 simulation...")
        self.px4_process = subprocess.Popen(
            ["make", "px4_sitl", "gz_x500_mono_cam_down_baylands"],
            cwd=self.px4_path,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env={**os.environ, "ROS_DISTRO": "", "ROS_VERSION": ""},
        )
        print("Waiting ~25s for PX4 to boot...")
        time.sleep(25)
        print("PX4 should be ready.")

    def start_recording(self):
        print(f"Starting video recording: {self.video_mkv}")
        gst_cmd = [
            "gst-launch-1.0", "-e",
            "udpsrc", "port=5600", "buffer-size=2000000",
            "!", "application/x-rtp,encoding-name=H264,payload=96",
            "!", "rtph264depay",
            "!", "h264parse",
            "!", "matroskamux",
            "!", "filesink", f"location={self.video_mkv}", "sync=false"
        ]
        self.gst_process = subprocess.Popen(gst_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.video_t0_wall = time.time()
        time.sleep(2)
        print("Video recording started")

    def save_telemetry_point(self, data: dict):
        self.telemetry_data.append(data)

    def _trim_to_mission_window(self):
        if self.mission_start_ts is None or self.mission_end_ts is None:
            return
        self.telemetry_data = [
            d for d in self.telemetry_data
            if self.mission_start_ts <= d["_wall_time"] <= self.mission_end_ts
        ]

    def _drop_internal_fields(self):
        for d in self.telemetry_data:
            d.pop("_wall_time", None)

    def _get_media_duration_seconds(self, media_path: str):
        ffprobe = shutil.which("ffprobe")
        if not ffprobe or not os.path.exists(media_path):
            return None
        cmd = [ffprobe, "-v", "error", "-show_entries", "format=duration", "-of", "json", media_path]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            data = json.loads(result.stdout)
            dur = float(data["format"]["duration"])
            return dur if dur > 0 else None
        except Exception:
            return None

    def _resync_csv_timestamp_to_media_duration(self, media_path: str):
        if not self.telemetry_data:
            return
        media_duration = self._get_media_duration_seconds(media_path)
        if media_duration is None:
            return
        last_csv = self.telemetry_data[-1]["timestamp"]
        if last_csv <= 0:
            return
        scale = media_duration / last_csv
        for row in self.telemetry_data:
            row["timestamp"] *= scale

    def save_all_data(self):
        self._trim_to_mission_window()
        self._drop_internal_fields()

        if self.telemetry_data:
            print(f"Saving telemetry CSV: {self.telemetry_csv}")
            fieldnames = [
                "timestamp",
                "sensor_accelx",
                "sensor_accely",
                "sensor_accelz",
                "sensor_baro",
                "sensor_gps_lat",
                "sensor_gps_lon",
                "sensor_gps_alt",
                "sensor_gyrox",
                "sensor_gyroy",
                "sensor_gyroz",
                "sensor_magx",
                "sensor_magy",
                "sensor_magz",
            ]
            with open(self.telemetry_csv, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.telemetry_data)

        with open(self.summary_file, "w") as f:
            f.write("MISSION SUMMARY\n")
            if self.telemetry_data:
                duration = self.telemetry_data[-1]["timestamp"] - self.telemetry_data[0]["timestamp"]
                f.write(f"Duration: {duration:.2f}s\n")
                f.write(f"Points: {len(self.telemetry_data)}\n")

    def stop_all(self):
        print("\nStopping recording and PX4...")
        self.end_mission_window()

        if self.gst_process:
            with suppress(Exception):
                self.gst_process.send_signal(signal.SIGINT)
            time.sleep(1)
            with suppress(Exception):
                self.gst_process.terminate()

        if self.px4_process:
            with suppress(Exception):
                self.px4_process.terminate()
            time.sleep(2)

        converted_ok = False
        if os.path.exists(self.video_mkv):
            ffmpeg = shutil.which("ffmpeg")
            if ffmpeg:
                result = subprocess.run(
                    [ffmpeg, "-i", self.video_mkv, "-c", "copy", self.video_mp4, "-y"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                converted_ok = (result.returncode == 0 and os.path.exists(self.video_mp4))

        media_for_sync = self.video_mp4 if converted_ok else self.video_mkv
        self._resync_csv_timestamp_to_media_duration(media_for_sync)
        self.save_all_data()


async def set_telemetry_rates(drone: System):
    with suppress(Exception):
        await drone.telemetry.set_rate_position(30.0)
    with suppress(Exception):
        await drone.telemetry.set_rate_imu(30.0)
    with suppress(Exception):
        await drone.telemetry.set_rate_velocity_ned(30.0)
    with suppress(Exception):
        await drone.telemetry.set_rate_in_air(10.0)


async def collect_telemetry_30hz(drone: System, recorder: DroneRecorder, baro_reader: RawBaroReader):
    while recorder.video_t0_wall is None:
        await asyncio.sleep(0.005)

    latest = {"pos": None, "imu": None}

    async def watch_position():
        async for p in drone.telemetry.position():
            latest["pos"] = p

    async def watch_imu():
        async for i in drone.telemetry.imu():
            latest["imu"] = i

    tasks = [asyncio.create_task(watch_position()), asyncio.create_task(watch_imu())]

    period = 1.0 / 30.0
    next_tick = time.perf_counter()

    # trzymamy ostatnią dobrą wartość baro (nie nan)
    last_good_baro = 1013.25

    try:
        while True:
            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                await asyncio.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

            now_wall = time.time()
            if recorder.mission_end_ts is not None and now_wall > recorder.mission_end_ts + 0.2:
                break

            pos = latest["pos"]
            imu = latest["imu"]
            if pos is None:
                continue

            ts = now_wall - recorder.video_t0_wall

            baro_hpa = baro_reader.get_latest_pressure_hpa()
            if baro_hpa is not None:
                last_good_baro = baro_hpa
            baro_out = last_good_baro

            recorder.save_telemetry_point({
                "_wall_time": now_wall,
                "timestamp": ts,
                "sensor_accelx": imu.acceleration_frd.forward_m_s2 if imu else 0.0,
                "sensor_accely": imu.acceleration_frd.right_m_s2 if imu else 0.0,
                "sensor_accelz": imu.acceleration_frd.down_m_s2 if imu else 0.0,
                "sensor_baro": baro_out,
                "sensor_gps_lat": pos.latitude_deg,
                "sensor_gps_lon": pos.longitude_deg,
                "sensor_gps_alt": pos.relative_altitude_m,
                "sensor_gyrox": imu.angular_velocity_frd.forward_rad_s if imu else 0.0,
                "sensor_gyroy": imu.angular_velocity_frd.right_rad_s if imu else 0.0,
                "sensor_gyroz": imu.angular_velocity_frd.down_rad_s if imu else 0.0,
                "sensor_magx": imu.magnetic_field_frd.forward_gauss if imu else 0.0,
                "sensor_magy": imu.magnetic_field_frd.right_gauss if imu else 0.0,
                "sensor_magz": imu.magnetic_field_frd.down_gauss if imu else 0.0,
            })

    except asyncio.CancelledError:
        pass
    finally:
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


def haversine_m(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    return 6371000 * c


async def wait_until_arrived_and_stable(
    drone: System,
    target_lat: float,
    target_lon: float,
    pos_tolerance_m: float = 2.0,
    speed_tolerance_ms: float = 0.8,
    stable_time_s: float = 1.5,
    timeout_s: float = 60.0,
):
    start = time.time()
    stable_since = None

    async for pos in drone.telemetry.position():
        vel = await drone.telemetry.velocity_ned().__anext__()
        dist_m = haversine_m(pos.latitude_deg, pos.longitude_deg, target_lat, target_lon)
        speed_ms = math.sqrt(vel.north_m_s ** 2 + vel.east_m_s ** 2 + vel.down_m_s ** 2)

        if dist_m <= pos_tolerance_m and speed_ms <= speed_tolerance_ms:
            if stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since >= stable_time_s:
                return True
        else:
            stable_since = None

        if time.time() - start > timeout_s:
            return False
    return False


async def arm_with_retry(drone: System, retries: int = 8):
    for attempt in range(1, retries + 1):
        try:
            await drone.action.arm()
            return
        except ActionError as e:
            if attempt == retries:
                raise e
            await asyncio.sleep(2.0)


async def fly_square_mission(recorder: DroneRecorder):
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected")
            break

    print("Waiting for GPS...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS ready")
            break

    await asyncio.sleep(2.0)
    await set_telemetry_rates(drone)

    # pymavlink na OSOBNYM porcie 14550
    print("Starting raw baro reader on port 14550...")
    baro_reader = RawBaroReader(connection_str="udpin:0.0.0.0:14550")
    baro_reader.start()
    await asyncio.sleep(2.0)
    print(f"Baro reader connected: {baro_reader.connected}")

    async for position in drone.telemetry.position():
        center_lat = position.latitude_deg
        center_lon = position.longitude_deg
        altitude_abs = position.absolute_altitude_m
        print(f"Square center: {center_lat:.6f}, {center_lon:.6f}")
        print(f"Initial rel alt (should be ~0): {position.relative_altitude_m:.3f} m")
        break

    recorder.start_mission_window()
    telemetry_task = asyncio.create_task(collect_telemetry_30hz(drone, recorder, baro_reader))

    flight_altitude_rel = 50
    side_length = 200
    wait_time = 2
    num_loops = 1
    flight_altitude_abs = altitude_abs + flight_altitude_rel

    print("Arming...")
    await arm_with_retry(drone)

    await drone.action.set_takeoff_altitude(flight_altitude_rel)
    await drone.action.takeoff()
    await asyncio.sleep(12)

    with suppress(Exception):
        await drone.action.set_maximum_speed(10.0)
    with suppress(Exception):
        await drone.gimbal.set_mode(GimbalMode.YAW_FOLLOW)
        await drone.gimbal.set_pitch_and_yaw(-90, 0)

    lat_per_meter = 1 / 111000
    lon_per_meter = 1 / (111000 * math.cos(math.radians(center_lat)))
    half_side = side_length / 2

    square_points = [
        {"name": "Start", "x": 0, "y": 0, "yaw": 0},
        {"name": "Point 1 (E)", "x": half_side, "y": 0, "yaw": 90},
        {"name": "Point 2 (NE)", "x": half_side, "y": half_side, "yaw": 0},
        {"name": "Point 3 (NW)", "x": -half_side, "y": half_side, "yaw": 270},
        {"name": "Point 4 (SW)", "x": -half_side, "y": -half_side, "yaw": 180},
        {"name": "Point 5 (SE)", "x": half_side, "y": -half_side, "yaw": 90},
        {"name": "Return", "x": 0, "y": 0, "yaw": 0},
    ]

    try:
        for _ in range(num_loops):
            for point in square_points:
                target_lat = center_lat + point["y"] * lat_per_meter
                target_lon = center_lon + point["x"] * lon_per_meter
                await drone.action.goto_location(target_lat, target_lon, flight_altitude_abs, point["yaw"])
                await wait_until_arrived_and_stable(drone, target_lat, target_lon)
                await asyncio.sleep(wait_time)

        await drone.action.goto_location(center_lat, center_lon, flight_altitude_abs, 0)
        await asyncio.sleep(10)
        await drone.action.land()

        async for is_in_air in drone.telemetry.in_air():
            if not is_in_air:
                break
    finally:
        recorder.end_mission_window()
        telemetry_task.cancel()
        await asyncio.gather(telemetry_task, return_exceptions=True)
        baro_reader.stop()


async def main():
    recorder = DroneRecorder(px4_path="~/PX4-Autopilot", output_dir="mission_data")
    try:
        recorder.start_px4()
        recorder.start_recording()
        await fly_square_mission(recorder)
    except Exception as e:
        print(f"Error: {e}")
        recorder.end_mission_window()
    finally:
        recorder.stop_all()
        print(f"\nAll data saved in: {recorder.output_dir}/")


if __name__ == "__main__":
    asyncio.run(main())
