#!/usr/bin/env python3

import asyncio
import subprocess
import os
import signal
import time
import math
import csv
import json
from datetime import datetime
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

        print(f"Data directory: {self.output_dir}/")

    def start_px4(self):
        print("Starting PX4 simulation...")

        subprocess.run(
            ["make", "distclean"],
            cwd=self.px4_path,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        self.px4_process = subprocess.Popen(
            ["make", "px4_sitl", "gz_x500_mono_cam_down_baylands"],
            cwd=self.px4_path,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env={**os.environ, 'ROS_DISTRO': '', 'ROS_VERSION': ''}
        )

        print("Waiting 25s for PX4...")
        time.sleep(25)
        print("PX4 ready")
        self.log_event("PX4 started")

    def start_recording(self):
        print(f"Starting video recording: {self.video_mkv}")

        gst_cmd = [
            "gst-launch-1.0", "-e", "-v",
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

        time.sleep(3)
        print("Video recording started")
        self.log_event("Video recording started")

    def log_event(self, event):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.mission_events.append(f"[{timestamp}] {event}")

    def save_telemetry_point(self, data):
        self.telemetry_data.append(data)

    def save_all_data(self):
        if self.telemetry_data:
            print(f"Saving telemetry CSV: {self.telemetry_csv}")
            with open(self.telemetry_csv, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.telemetry_data[0].keys())
                writer.writeheader()
                writer.writerows(self.telemetry_data)

            print(f"Saving telemetry JSON: {self.telemetry_json}")
            with open(self.telemetry_json, 'w') as f:
                json.dump(self.telemetry_data, f, indent=2)

        print(f"Saving mission log: {self.mission_log}")
        with open(self.mission_log, 'w') as f:
            f.write('\n'.join(self.mission_events))

        self.create_summary()

    def create_summary(self):
        print(f"Creating summary: {self.summary_file}")

        total_points = len(self.telemetry_data)
        duration = 0
        max_altitude = 0
        max_speed = 0
        distance = 0

        if self.telemetry_data:
            start_time = self.telemetry_data[0]['timestamp']
            end_time = self.telemetry_data[-1]['timestamp']
            duration = end_time - start_time

            max_altitude = max(d['altitude_m'] for d in self.telemetry_data)
            max_speed = max(d['ground_speed_ms'] for d in self.telemetry_data)

            for i in range(1, len(self.telemetry_data)):
                lat1 = math.radians(self.telemetry_data[i - 1]['latitude'])
                lon1 = math.radians(self.telemetry_data[i - 1]['longitude'])
                lat2 = math.radians(self.telemetry_data[i]['latitude'])
                lon2 = math.radians(self.telemetry_data[i]['longitude'])

                dlat = lat2 - lat1
                dlon = lon2 - lon1
                a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
                c = 2 * math.asin(math.sqrt(a))
                distance += 6371000 * c

        with open(self.summary_file, 'w') as f:
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
            self.gst_process.send_signal(signal.SIGINT)
            time.sleep(2)
            self.gst_process.terminate()
            self.log_event("Video recording stopped")

        if self.px4_process:
            self.px4_process.terminate()
            time.sleep(3)
            self.log_event("PX4 stopped")

        self.save_all_data()

        print(f"Video saved: {self.video_mkv}")

        if os.path.exists(self.video_mkv):
            print("Converting to MP4...")
            result = subprocess.run(
                ["ffmpeg", "-i", self.video_mkv, "-c", "copy",
                 self.video_mp4, "-y"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            if result.returncode == 0:
                print(f"MP4 saved: {self.video_mp4}")


async def collect_telemetry(drone, recorder):
    start_time = time.time()

    async for position in drone.telemetry.position():
        current_time = time.time() - start_time

        try:
            velocity = None
            attitude = None
            battery = None

            async for v in drone.telemetry.velocity_ned():
                velocity = v
                break

            async for a in drone.telemetry.attitude_euler():
                attitude = a
                break

            async for b in drone.telemetry.battery():
                battery = b
                break

            ground_speed = 0
            if velocity:
                ground_speed = math.sqrt(velocity.north_m_s ** 2 + velocity.east_m_s ** 2)

            data = {
                'timestamp': current_time,
                'latitude': position.latitude_deg,
                'longitude': position.longitude_deg,
                'altitude_m': position.relative_altitude_m,
                'absolute_altitude_m': position.absolute_altitude_m,
                'ground_speed_ms': ground_speed,
                'velocity_north_ms': velocity.north_m_s if velocity else 0,
                'velocity_east_ms': velocity.east_m_s if velocity else 0,
                'velocity_down_ms': velocity.down_m_s if velocity else 0,
                'roll_deg': attitude.roll_deg if attitude else 0,
                'pitch_deg': attitude.pitch_deg if attitude else 0,
                'yaw_deg': attitude.yaw_deg if attitude else 0,
                'battery_voltage': battery.voltage_v if battery else 0,
                'battery_remaining': battery.remaining_percent if battery else 0,
            }

            recorder.save_telemetry_point(data)

        except Exception as e:
            pass

        await asyncio.sleep(0.5)


async def fly_square_mission(recorder):
    drone = System()
    await drone.connect(system_address="udp://:14540")

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
        altitude = position.absolute_altitude_m
        print(f"Square center: {center_lat:.6f}, {center_lon:.6f}")
        recorder.log_event(f"Square center: {center_lat:.6f}, {center_lon:.6f}")
        break

    telemetry_task = asyncio.create_task(collect_telemetry(drone, recorder))

    flight_altitude = 50
    side_length = 20
    wait_time = 8
    num_loops = 2

    flight_altitude_abs = altitude + flight_altitude

    print("Arming...")
    await drone.action.arm()
    recorder.log_event("Armed")

    print(f"Taking off to {flight_altitude}m...")
    await drone.action.set_takeoff_altitude(flight_altitude)
    await drone.action.takeoff()
    recorder.log_event(f"Takeoff to {flight_altitude}m")
    await asyncio.sleep(12)
    recorder.log_event("Takeoff completed")

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

    print(f"\nSquare mission: altitude={flight_altitude}m, side={side_length}m, loops={num_loops}")
    recorder.log_event(f"Square mission: {num_loops} loops, side={side_length}m, alt={flight_altitude}m")

    for loop in range(num_loops):
        print(f"\nLoop {loop + 1}/{num_loops}")
        recorder.log_event(f"Starting square loop {loop + 1}/{num_loops}")

        for i, point in enumerate(square_points):
            lat_offset = point["y"] * lat_per_meter
            lon_offset = point["x"] * lon_per_meter

            target_lat = center_lat + lat_offset
            target_lon = center_lon + lon_offset
            yaw = point["yaw"]

            print(f"  {point['name']}: ({point['x']:+.0f}m E, {point['y']:+.0f}m N) Yaw: {yaw} deg")
            recorder.log_event(f"Waypoint: {point['name']} at ({point['x']:+.1f}, {point['y']:+.1f}) m")

            await drone.action.goto_location(
                target_lat,
                target_lon,
                flight_altitude_abs,
                yaw
            )

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
    telemetry_task.cancel()


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
