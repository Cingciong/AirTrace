#!/usr/bin/env python3

import asyncio
import subprocess
import os
import signal
import time
import math
import csv
import json
import shutil
from datetime import datetime
from contextlib import suppress

from mavsdk import System
from mavsdk.gimbal import GimbalMode


class DroneRecorder:
    def __init__(self, px4_path="~/PX4-Autopilot", output_dir="mission_data"):
        self.px4_path = os.path.expanduser(px4_path)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.output_dir = f"{output_dir}_{self.timestamp}"
        os.makedirs(self.output_dir, exist_ok=True)

        self.video_mkv = os.path.join(self.output_dir, f"video_{self.timestamp}.mkv")
        self.video_mp4 = os.path.join(self.output_dir, f"mission_{self.timestamp}.mp4")
        self.telemetry_csv = os.path.join(self.output_dir, f"telemetry_{self.timestamp}.csv")
        self.telemetry_json = os.path.join(self.output_dir, f"telemetry_{self.timestamp}.json")
        self.mission_log = os.path.join(self.output_dir, f"mission_{self.timestamp}.log")
        self.summary_file = os.path.join(self.output_dir, f"summary_{self.timestamp}.txt")

        self.px4_process = None
        self.gst_process = None

        self.telemetry_data = []
        self.mission_events = []

        # [ZMIANA 1] Wspólny zegar dla video + telemetry
        self.mission_t0 = None

        print(f"Data directory: {self.output_dir}/")

    def log_event(self, event: str):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.mission_events.append(f"[{timestamp}] {event}")

    def start_px4(self):
        """
        Start PX4 SITL. Removed 'make distclean' to avoid long rebuilds and failures.
        Shows PX4 output to help debugging.
        """
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

        self.log_event("PX4 process started")

        print("Waiting ~25s for PX4 to boot...")
        time.sleep(25)
        print("PX4 should be ready (if it booted successfully).")
        self.log_event("PX4 boot wait finished")

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

        self.gst_process = subprocess.Popen(
            gst_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        # [ZMIANA 2] Ustalamy "czas 0" przy starcie nagrywania
        self.mission_t0 = time.time()

        time.sleep(2)
        print("Video recording started")
        self.log_event("Video recording started")

    def save_telemetry_point(self, data: dict):
        self.telemetry_data.append(data)

    def save_all_data(self):
        if self.telemetry_data:
            print(f"Saving telemetry CSV: {self.telemetry_csv}")
            with open(self.telemetry_csv, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self.telemetry_data[0].keys())
                writer.writeheader()
                writer.writerows(self.telemetry_data)

            print(f"Saving telemetry JSON: {self.telemetry_json}")
            with open(self.telemetry_json, "w") as f:
                json.dump(self.telemetry_data, f, indent=2)

        print(f"Saving mission log: {self.mission_log}")
        with open(self.mission_log, "w") as f:
            f.write("\n".join(self.mission_events))

        self.create_summary()

    def create_summary(self):
        print(f"Creating summary: {self.summary_file}")

        total_points = len(self.telemetry_data)
        duration = 0.0
        max_altitude = 0.0
        max_speed = 0.0
        distance = 0.0

        if self.telemetry_data:
            start_time = self.telemetry_data[0]["timestamp"]
            end_time = self.telemetry_data[-1]["timestamp"]
            duration = float(end_time - start_time)

            max_altitude = max(d["altitude_m"] for d in self.telemetry_data)
            max_speed = max(d["ground_speed_ms"] for d in self.telemetry_data)

            for i in range(1, len(self.telemetry_data)):
                lat1 = math.radians(self.telemetry_data[i - 1]["latitude"])
                lon1 = math.radians(self.telemetry_data[i - 1]["longitude"])
                lat2 = math.radians(self.telemetry_data[i]["latitude"])
                lon2 = math.radians(self.telemetry_data[i]["longitude"])

                dlat = lat2 - lat1
                dlon = lon2 - lon1
                a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
                c = 2 * math.asin(math.sqrt(a))
                distance += 6371000 * c

        with open(self.summary_file, "w") as f:
            f.write("=" * 60 + "\n")
            f.write("MISSION SUMMARY - SQUARE PATTERN\n")
            f.write("=" * 60 + "\n\n")
            f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Directory: {self.output_dir}\n\n")

            f.write("FILES:\n")
            f.write(f"  Video MKV: {os.path.basename(self.video_mkv)}\n")
            f.write(f"  Video MP4: {os.path.basename(self.video_mp4)}\n")
            f.write(f"  Telemetry CSV: {os.path.basename(self.telemetry_csv)}\n")
            f.write(f"  Telemetry JSON: {os.path.basename(self.telemetry_json)}\n")
            f.write(f"  Mission log: {os.path.basename(self.mission_log)}\n\n")

            f.write("STATISTICS:\n")
            f.write(f"  Duration: {duration:.1f} s\n")
            f.write(f"  Telemetry points: {total_points}\n")
            f.write(f"  Max altitude: {max_altitude:.2f} m\n")
            f.write(f"  Max speed: {max_speed:.2f} m/s\n")
            f.write(f"  Distance: {distance:.2f} m\n\n")

            f.write("EVENTS:\n")
            for event in self.mission_events:
                f.write(f"  {event}\n")

        print("\n" + "=" * 60)
        print("MISSION SUMMARY")
        print("=" * 60)
        print(f"Duration: {duration:.1f}s")
        print(f"Points: {total_points}")
        print(f"Max altitude: {max_altitude:.2f}m")
        print(f"Max speed: {max_speed:.2f}m/s")
        print(f"Distance: {distance:.2f}m")
        print("=" * 60)

    def stop_all(self):
        print("\nStopping recording and PX4...")
        self.log_event("Mission ended")

        if self.gst_process:
            with suppress(Exception):
                self.gst_process.send_signal(signal.SIGINT)
            time.sleep(1)
            with suppress(Exception):
                self.gst_process.terminate()
            self.log_event("Video recording stopped")

        if self.px4_process:
            with suppress(Exception):
                self.px4_process.terminate()
            time.sleep(2)
            self.log_event("PX4 stopped")

        self.save_all_data()

        print(f"Video saved: {self.video_mkv}")

        if os.path.exists(self.video_mkv):
            ffmpeg = shutil.which("ffmpeg")
            if not ffmpeg:
                print("ffmpeg not found, skipping conversion to mp4.")
                self.log_event("ffmpeg missing; conversion skipped")
                return

            print("Converting to MP4...")
            result = subprocess.run(
                [ffmpeg, "-i", self.video_mkv, "-c", "copy", self.video_mp4, "-y"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if result.returncode == 0:
                print(f"MP4 saved: {self.video_mp4}")
                self.log_event("Converted MKV -> MP4")
            else:
                print("ffmpeg conversion failed (see ffmpeg logs if enabled).")
                self.log_event("ffmpeg conversion failed")


async def collect_telemetry_10hz(drone: System, recorder: DroneRecorder):
    """
    Stable approach: keep latest values from streams in background tasks (cache),
    and log at fixed 10 Hz.
    """
    # [ZMIANA 3] Czekamy na wspólny czas startu
    while recorder.mission_t0 is None:
        await asyncio.sleep(0.01)

    latest = {"pos": None, "vel": None, "att": None, "bat": None}

    async def watch_position():
        async for p in drone.telemetry.position():
            latest["pos"] = p

    async def watch_velocity():
        async for v in drone.telemetry.velocity_ned():
            latest["vel"] = v

    async def watch_attitude():
        async for a in drone.telemetry.attitude_euler():
            latest["att"] = a

    async def watch_battery():
        async for b in drone.telemetry.battery():
            latest["bat"] = b

    tasks = [
        asyncio.create_task(watch_position()),
        asyncio.create_task(watch_velocity()),
        asyncio.create_task(watch_attitude()),
        asyncio.create_task(watch_battery()),
    ]

    try:
        while True:
            await asyncio.sleep(0.1)  # 10 Hz log rate

            pos = latest["pos"]
            if pos is None:
                continue

            vel = latest["vel"]
            att = latest["att"]
            bat = latest["bat"]

            # [ZMIANA 4] Timestamp liczony od startu video
            current_time = time.time() - recorder.mission_t0

            ground_speed = 0.0
            if vel:
                ground_speed = math.sqrt(vel.north_m_s ** 2 + vel.east_m_s ** 2)

            data = {
                "timestamp": current_time,
                "latitude": pos.latitude_deg,
                "longitude": pos.longitude_deg,
                "altitude_m": pos.relative_altitude_m,
                "absolute_altitude_m": pos.absolute_altitude_m,
                "ground_speed_ms": ground_speed,
                "velocity_north_ms": vel.north_m_s if vel else 0.0,
                "velocity_east_ms": vel.east_m_s if vel else 0.0,
                "velocity_down_ms": vel.down_m_s if vel else 0.0,
                "roll_deg": att.roll_deg if att else 0.0,
                "pitch_deg": att.pitch_deg if att else 0.0,
                "yaw_deg": att.yaw_deg if att else 0.0,
                "battery_voltage": bat.voltage_v if bat else 0.0,
                "battery_remaining": bat.remaining_percent if bat else 0.0,
            }
            recorder.save_telemetry_point(data)

    except asyncio.CancelledError:
        # Normal on shutdown
        pass
    finally:
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


# [ZMIANA 5] Helper: odległość Haversine
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


# [ZMIANA 6] Helper: czekanie aż dron doleci i wyhamuje
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


async def fly_square_mission(recorder: DroneRecorder):
    drone = System()

    # Fix deprecated scheme: use udpin://
    recorder.log_event("Connecting MAVSDK (udpin://0.0.0.0:14540)")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected")
            recorder.log_event("Drone connected")
            break

    print("Waiting for GPS...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS ready")
            recorder.log_event("GPS ready")
            break

    async for position in drone.telemetry.position():
        center_lat = position.latitude_deg
        center_lon = position.longitude_deg
        altitude_abs = position.absolute_altitude_m
        print(f"Square center: {center_lat:.6f}, {center_lon:.6f}")
        recorder.log_event(f"Square center: {center_lat:.6f}, {center_lon:.6f}")
        break

    telemetry_task = asyncio.create_task(collect_telemetry_10hz(drone, recorder))

    flight_altitude_rel = 50  # meters above home (relative)
    side_length = 500          # meters
    wait_time = 8             # seconds at each point
    num_loops = 2

    flight_altitude_abs = altitude_abs + flight_altitude_rel

    print("Arming...")
    await drone.action.arm()
    recorder.log_event("Armed")

    print(f"Taking off to {flight_altitude_rel}m...")
    await drone.action.set_takeoff_altitude(flight_altitude_rel)
    await drone.action.takeoff()
    recorder.log_event(f"Takeoff to {flight_altitude_rel}m")
    await asyncio.sleep(12)
    recorder.log_event("Takeoff completed")

    # Optional: set max speed to make waits more meaningful
    with suppress(Exception):
        await drone.action.set_maximum_speed(8.0)  # m/s
        recorder.log_event("Set maximum speed to 8 m/s")

    try:
        await drone.gimbal.set_mode(GimbalMode.YAW_FOLLOW)
        await drone.gimbal.set_pitch_and_yaw(-90, 0)
        recorder.log_event("Camera pointing down (-90 deg)")
        print("Camera pointing down")
    except Exception as e:
        recorder.log_event(f"Gimbal unavailable: {e}")

    await asyncio.sleep(2)

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

    print(f"\nSquare mission: alt_rel={flight_altitude_rel}m, side={side_length}m, loops={num_loops}")
    recorder.log_event(f"Square mission: {num_loops} loops, side={side_length}m, alt_rel={flight_altitude_rel}m")

    try:
        for loop in range(num_loops):
            print(f"\nLoop {loop + 1}/{num_loops}")
            recorder.log_event(f"Starting square loop {loop + 1}/{num_loops}")

            for point in square_points:
                lat_offset = point["y"] * lat_per_meter
                lon_offset = point["x"] * lon_per_meter

                target_lat = center_lat + lat_offset
                target_lon = center_lon + lon_offset
                yaw = point["yaw"]

                print(f"  {point['name']}: ({point['x']:+.0f}m E, {point['y']:+.0f}m N) yaw={yaw} deg")
                recorder.log_event(f"Waypoint: {point['name']} at ({point['x']:+.1f}, {point['y']:+.1f}) m")

                await drone.action.goto_location(target_lat, target_lon, flight_altitude_abs, yaw)

                # [ZMIANA 7] Najpierw dojazd + wyhamowanie, potem pauza
                arrived = await wait_until_arrived_and_stable(
                    drone,
                    target_lat,
                    target_lon,
                    pos_tolerance_m=2.0,
                    speed_tolerance_ms=0.8,
                    stable_time_s=1.5,
                    timeout_s=60.0,
                )
                recorder.log_event(f"{point['name']} reached_and_stable={arrived}")

                await asyncio.sleep(wait_time)

        recorder.log_event("All square loops completed")

        print("\nReturning to center...")
        await drone.action.goto_location(center_lat, center_lon, flight_altitude_abs, 0)
        recorder.log_event("Final return to home position")
        await asyncio.sleep(10)

        print("Landing...")
        await drone.action.land()
        recorder.log_event("Landing started")

        async for is_in_air in drone.telemetry.in_air():
            if not is_in_air:
                print("Landed")
                recorder.log_event("Landed successfully")
                break

        print("Mission completed")

    finally:
        telemetry_task.cancel()
        await asyncio.gather(telemetry_task, return_exceptions=True)


async def main():
    recorder = DroneRecorder(
        px4_path="~/PX4-Autopilot",
        output_dir="mission_data"
    )

    try:
        recorder.start_px4()
        recorder.start_recording()
        await fly_square_mission(recorder)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        recorder.log_event("Mission interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        recorder.log_event(f"Error: {e}")
    finally:
        recorder.stop_all()
        print(f"\nAll data saved in: {recorder.output_dir}/")


if __name__ == "__main__":
    asyncio.run(main())