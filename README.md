# Automated Drone Flight Data Recording System

A fully automated Python application for drone mission execution with integrated video recording and telemetry logging using PX4 SITL, Gazebo, and MAVLink.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [How It Works](#how-it-works)
  - [1. MAVLink Communication and Mission Control](#1-mavlink-communication-and-mission-control)
  - [2. Automated PX4 and Gazebo Launch](#2-automated-px4-and-gazebo-launch)
  - [3. Parallel Video Recording](#3-parallel-video-recording)
  - [4. Asynchronous Telemetry Logging](#4-asynchronous-telemetry-logging)
  - [5. Event Logging](#5-event-logging)
  - [6. Data Persistence](#6-data-persistence)
- [Data Synchronization](#data-synchronization)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Output Files](#output-files)
- [Mission Timeline](#mission-timeline)
- [Technical Notes](#technical-notes)

## Overview

This system autonomously manages the entire workflow of drone mission execution:
- Launches PX4 SITL simulation with Gazebo
- Executes predefined flight missions (square pattern)
- Records camera feed via GStreamer
- Collects real-time sensor data
- Generates comprehensive mission reports

**Single command execution - no manual intervention required.**

## Features

- **Zero-Configuration Launch**: Automatically starts PX4, Gazebo, and GStreamer
- **MAVLink Control**: Full autonomous flight control via MAVSDK
- **Real-time Telemetry**: 2Hz sensor data collection (GPS, IMU, battery)
- **HD Video Recording**: Automatic H.264 video capture from drone camera
- **Data Export**: CSV, JSON, and human-readable formats
- **Process Management**: Graceful startup and shutdown of all subsystems
- **Mission Statistics**: Automated summary with distance, duration, max altitude/speed

## Architecture

The system consists of three parallel subsystems communicating via network protocols:

```
┌─────────────────┐   MAVLink      ┌──────────────┐   Gazebo      ┌──────────────┐
│  Python Script  │───UDP 14540───→│ PX4 Autopilot│←──Transport──→│    Gazebo    │
│    (MAVSDK)     │                │    (SITL)    │               │  Simulator   │
└─────────────────┘                └──────────────┘               └──────┬───────┘
         │                                                                │
         │                                                                │
         │                         ┌──────────────┐   H.264/RTP          │
         └────────────────────────→│  GStreamer   │←───UDP 5600──────────┘
                                   │   Recorder   │
                                   └──────────────┘
```

## How It Works

### 1. MAVLink Communication and Mission Control

**Location:** `fly_square_mission()` function (lines 250-350)

Establishes communication with PX4 autopilot using the MAVLink protocol:

```python
drone = System()
await drone.connect(system_address="udp://:14540")
```

**Connection Sequence:**

1. Creates MAVSDK client instance
2. Connects to PX4 via UDP port 14540
3. Exchanges HEARTBEAT messages to establish link
4. Waits for GPS lock and system ready state

**Mission Control Commands:**

| Python Command | MAVLink Message | Description |
|----------------|-----------------|-------------|
| `drone.action.arm()` | `MAV_CMD_COMPONENT_ARM_DISARM` | Arm motors |
| `drone.action.takeoff()` | `MAV_CMD_NAV_TAKEOFF` | Autonomous takeoff |
| `drone.action.goto_location()` | `SET_POSITION_TARGET_GLOBAL_INT` | Navigate to waypoint |
| `drone.action.land()` | `MAV_CMD_NAV_LAND` | Autonomous landing |

**Protocol Flow:**

```
Python MAVSDK          PX4 Autopilot
     |                       |
     |----HEARTBEAT--------->|  (every 1s)
     |<---HEARTBEAT----------|  (response)
     |<---SYS_STATUS---------|  (system status)
     |                       |
  is_connected = True
```

### 2. Automated PX4 and Gazebo Launch

**Location:** `DroneRecorder.start_px4()` method (lines 37-55)

**Why manual launch is NOT required:**

The code uses `subprocess.Popen()` to programmatically start both PX4 SITL and Gazebo:

```python
self.px4_process = subprocess.Popen(
    ["make", "px4_sitl", "gz_x500_mono_cam_down_baylands"],
    cwd=self.px4_path,  # ~/PX4-Autopilot
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
    env={**os.environ, 'ROS_DISTRO': '', 'ROS_VERSION': ''}
)
```

**Execution Flow:**

```
make px4_sitl gz_x500_mono_cam_down_baylands
    ↓
PX4 Makefile detects "gz_" prefix
    ↓
Launches Gazebo Garden via gz sim CLI
    ↓
Loads "baylands" world
    ↓
Spawns x500 drone model with downward camera
    ↓
Starts PX4 autopilot (SITL binary)
    ↓
PX4 connects to Gazebo via Gazebo Transport
    ↓
Opens MAVLink port 14540 and video port 5600
```

**Key Automation Elements:**

- **`subprocess.Popen()`** - Non-blocking process spawn (runs in background)
- **`cwd=self.px4_path`** - Sets working directory to PX4-Autopilot
- **`env={...}`** - Disables ROS variables to prevent conflicts
- **`stdout/stderr=DEVNULL`** - Suppresses verbose output
- **`time.sleep(25)`** - Waits for complete initialization

**Traditional vs Automated:**

```bash
# Manual approach (3 terminals required):
# Terminal 1:
cd ~/PX4-Autopilot
make px4_sitl gz_x500_mono_cam_down_baylands

# Terminal 2:
python3 drone_script.py

# Terminal 3:
gst-launch-1.0 ...

# Automated approach (single command):
python3 Fly_Orbit.py  # Everything handled automatically
```

### 3. Parallel Video Recording

**Location:** `DroneRecorder.start_recording()` method (lines 57-79)

GStreamer automatically captures video stream from Gazebo simulation:

```python
gst_cmd = [
    "gst-launch-1.0", "-e", "-v",
    "udpsrc", "port=5600", "buffer-size=2000000",
    "!", "application/x-rtp,encoding-name=H264,payload=96",
    "!", "rtph264depay",
    "!", "h264parse",
    "!", "matroskamux",
    "!", "filesink", f"location={self.video_mkv}", "sync=false"
]

self.gst_process = subprocess.Popen(gst_cmd, ...)
```

**Video Pipeline:**

```
┌──────────┐   UDP 5600   ┌────────────┐   MKV   ┌──────────┐
│ Gazebo   │──(H.264/RTP)→│ GStreamer  │────────→│ File MKV │
│ Camera   │              │ (background)│         └──────────┘
└──────────┘              └────────────┘               ↓
                                                  (post-mission)
                                                       ↓
                                              ┌─────────────┐
                                              │   FFmpeg    │
                                              │ MKV → MP4   │
                                              └─────────────┘
```

**Process Lifecycle:**

1. **Start:** GStreamer listens on UDP port 5600
2. **Recording:** Gazebo streams camera feed during entire flight
3. **Stop:** SIGINT triggers graceful file closure
4. **Conversion:** FFmpeg converts MKV to MP4

### 4. Asynchronous Telemetry Logging

**Location:** `collect_telemetry()` function (lines 186-234)

Background task collects sensor data at 2Hz concurrently with mission execution:

```python
async def collect_telemetry(drone, recorder):
    start_time = time.time()  # T0 reference
    
    async for position in drone.telemetry.position():  # 2Hz stream
        current_time = time.time() - start_time
        
        # Gather data from multiple MAVLink streams
        velocity = await get_velocity()    # LOCAL_POSITION_NED
        attitude = await get_attitude()    # ATTITUDE
        battery = await get_battery()      # SYS_STATUS
        
        data = {
            'timestamp': current_time,
            'latitude': position.latitude_deg,
            'longitude': position.longitude_deg,
            'altitude_m': position.relative_altitude_m,
            'ground_speed_ms': math.sqrt(vN² + vE²),
            'roll_deg': attitude.roll_deg,
            'pitch_deg': attitude.pitch_deg,
            'yaw_deg': attitude.yaw_deg,
            'battery_remaining': battery.remaining_percent,
        }
        
        recorder.save_telemetry_point(data)
        await asyncio.sleep(0.5)  # 2Hz sampling
```

**MAVLink Messages Used:**

| Message Type | Frequency | Data Collected |
|--------------|-----------|----------------|
| `GLOBAL_POSITION_INT` | 2Hz | Latitude, Longitude, Altitude |
| `LOCAL_POSITION_NED` | 2Hz | Velocity (North, East, Down) |
| `ATTITUDE` | 2Hz | Roll, Pitch, Yaw |
| `SYS_STATUS` | 1Hz | Battery voltage, remaining % |

**Task Launch:**

```python
telemetry_task = asyncio.create_task(collect_telemetry(drone, recorder))
```

Runs concurrently with mission using Python's asyncio framework.

### 5. Event Logging

**Location:** `DroneRecorder.log_event()` method (lines 81-83)

Timestamps mission-critical events for post-flight analysis:

```python
def log_event(self, event):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    self.mission_events.append(f"[{timestamp}] {event}")
```

**Example Event Timeline:**

```
[2026-01-26 14:32:00.123] PX4 started
[2026-01-26 14:32:28.456] Drone connected
[2026-01-26 14:32:30.789] Armed
[2026-01-26 14:32:35.012] Takeoff to 25m
[2026-01-26 14:32:47.234] Takeoff completed
[2026-01-26 14:32:50.567] Starting square loop 1/2
[2026-01-26 14:33:45.890] All square loops completed
[2026-01-26 14:34:00.123] Landing started
[2026-01-26 14:34:15.456] Landed successfully
```

### 6. Data Persistence

**Location:** `DroneRecorder.save_all_data()` method (lines 88-108)

Triggered during shutdown (`stop_all()`, line 178):

```python
# CSV format (for analysis in Excel/pandas)
with open(self.telemetry_csv, 'w') as f:
    writer = csv.DictWriter(f, fieldnames=...)
    writer.writerows(self.telemetry_data)

# JSON format (for programmatic access)
with open(self.telemetry_json, 'w') as f:
    json.dump(self.telemetry_data, f, indent=2)

# Mission log (human-readable timeline)
with open(self.mission_log, 'w') as f:
    f.write('\n'.join(self.mission_events))

# Summary statistics
self.create_summary()  # Duration, distance, max altitude/speed
```

## Data Synchronization

### Timestamp Sources

| Data Source | Timestamp Type | Reference Point |
|-------------|----------------|-----------------|
| **Telemetry** | `time.time() - start_time` | When `collect_telemetry()` starts |
| **Events** | `datetime.now()` | System clock at each event |
| **Video** | GStreamer PTS | When first UDP packet received |
| **Filenames** | `datetime` at `__init__` | Script start time |

### Synchronization Status

- **Within telemetry data:** Fully synchronized (single `start_time` reference)
- **Between telemetry and events:** Correlation possible via event timestamps
- **Between video and telemetry:** No direct sync - requires manual correlation

### Example Correlation:

```
Mission Log:
[2026-01-26 14:32:30.789] Armed

Telemetry CSV:
timestamp, latitude, longitude, ...
0.5, 47.397742, 8.545594, ...
2.3, 47.397742, 8.545594, ...  ← Approximately "Armed" event
7.5, 47.397743, 8.545595, ...
```

Calculate offset: `T_event - T_telemetry = offset`

## Installation

### Prerequisites

```bash
# Required software
- Python 3.8 or higher
- PX4-Autopilot (SITL)
- Gazebo Garden
- GStreamer 1.0
- FFmpeg
```

### Python Dependencies

```bash
pip install mavsdk
```

### PX4 Setup

```bash
# Clone and build PX4
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl gz_x500_mono_cam_down_baylands  # First build
```

### Verify Installation

```bash
# Test GStreamer
gst-launch-1.0 --version

# Test FFmpeg
ffmpeg -version

# Test Gazebo
gz sim --version
```

## Usage

### Basic Usage

```bash
# Clone repository
git clone <repository-url>
cd <repository-directory>

# Edit PX4 path in code (line 354)
# Ensure: px4_path="~/PX4-Autopilot"

# Run mission
python3 Fly_Orbit.py
```

### Expected Output

```
Data directory: mission_data_20260126_143200/
Starting PX4 simulation...
Waiting 25s for PX4...
PX4 ready
Starting video recording: mission_data_20260126_143200/video_20260126_143200.mkv
Video recording started
Waiting for drone connection...
Drone connected
Waiting for GPS...
GPS ready
Square center: 47.397742, 8.545594
Arming...
Taking off to 25m...
Camera pointing down

Square mission: altitude=25m, side=20m, loops=2

Loop 1/2
  Start: (+0m E, +0m N) Yaw: 0 deg
  Point 1 (E): (+10m E, +0m N) Yaw: 90 deg
  ...

Mission completed

Stopping recording and PX4...
Saving telemetry CSV: ...
Saving telemetry JSON: ...
Saving mission log: ...
Creating summary: ...

MISSION SUMMARY
Duration: 145.3s
Points: 290
Max altitude: 25.12m
Max speed: 5.67m/s
Distance: 163.45m

All data saved in: mission_data_20260126_143200/
```

## Configuration

### Mission Parameters

Edit these values in `fly_square_mission()` function (lines 273-276):

```python
flight_altitude = 25    # Flight altitude in meters
side_length = 20        # Square side length in meters
wait_time = 8          # Seconds to wait at each waypoint
num_loops = 2          # Number of complete square patterns
```

### PX4 Path

Edit line 354 in `main()`:

```python
recorder = DroneRecorder(
    px4_path="~/PX4-Autopilot",  # Change to your PX4 installation path
    output_dir="mission_data"
)
```

### Square Waypoints

Modify waypoints in `square_points` list (lines 306-314):

```python
square_points = [
    {"name": "Start", "x": 0, "y": 0, "yaw": 0},
    {"name": "Point 1 (E)", "x": half_side, "y": 0, "yaw": 90},
    # Add more waypoints as needed
]
```

## Output Files

After mission execution, all data is saved in timestamped directory:

```
mission_data_20260126_143200/
├── video_20260126_143200.mkv          # Raw H.264 video (MKV container)
├── mission_20260126_143200.mp4        # Converted video (MP4 container)
├── telemetry_20260126_143200.csv      # Telemetry data (CSV format)
├── telemetry_20260126_143200.json     # Telemetry data (JSON format)
├── mission_20260126_143200.log        # Event timeline
└── summary_20260126_143200.txt        # Mission statistics
```

### Telemetry CSV Format

```csv
timestamp,latitude,longitude,altitude_m,ground_speed_ms,roll_deg,pitch_deg,yaw_deg,battery_remaining
0.5,47.397742,8.545594,0.0,0.0,0.0,0.0,0.0,100.0
1.0,47.397742,8.545594,2.1,0.1,1.2,0.8,0.0,99.5
1.5,47.397742,8.545594,5.3,0.2,2.5,1.5,0.0,99.0
```

### Summary File Example

```
============================================================
MISSION SUMMARY - SQUARE PATTERN
============================================================

Date: 2026-01-26 14:34:15
Directory: mission_data_20260126_143200

FILES:
  Video MKV: video_20260126_143200.mkv
  Video MP4: mission_20260126_143200.mp4
  Telemetry CSV: telemetry_20260126_143200.csv
  Telemetry JSON: telemetry_20260126_143200.json
  Mission log: mission_20260126_143200.log

STATISTICS:
  Duration: 145.3 s
  Telemetry points: 290
  Max altitude: 25.12 m
  Max speed: 5.67 m/s
  Distance: 163.45 m

EVENTS:
  [2026-01-26 14:32:00.123] PX4 started
  [2026-01-26 14:32:28.456] Drone connected
  ...
```

## Mission Timeline

Typical execution sequence:

```
T=0s     main() starts
T=0s     DroneRecorder.__init__() creates timestamp
T=0s     recorder.start_px4()
           └─ subprocess: PX4 + Gazebo launch
T=25s    PX4 ready (after initialization wait)
T=25s    recorder.start_recording()
           └─ subprocess: GStreamer listening on UDP:5600
T=28s    await fly_square_mission()
           └─ drone.connect("udp://:14540")
T=30s    Drone connected (MAVLink handshake complete)
T=30s    telemetry_task = collect_telemetry()
           └─ start_time = time.time()  [TELEMETRY T0]
           └─ Samples at 2Hz (every 0.5s)
T=32s    await drone.action.arm()
           └─ log_event("Armed")
T=35s    await drone.action.takeoff()
T=47s    Takeoff complete (12s wait)
T=47s    Waypoint navigation begins
...      
T=180s   Mission completed
T=180s   recorder.stop_all()
           └─ SIGINT → GStreamer → finalize MKV
           └─ TERMINATE → PX4
           └─ save_all_data() → CSV, JSON, log
           └─ ffmpeg: MKV → MP4 conversion
```

## Technical Notes

### Why subprocess.Popen() instead of subprocess.run()?

- **`Popen()`** is non-blocking - returns immediately, process runs in background
- **`run()`** blocks until process terminates - would freeze the script
- `Popen()` provides process handle for later termination

### Why 25-second wait after PX4 launch?

- Gazebo requires time to initialize physics engine
- PX4 needs to establish Gazebo Transport connection
- MAVLink port 14540 must be ready before connection attempt
- Insufficient wait causes connection failures

### Why env modification (ROS_DISTRO='')?

- Prevents conflicts if ROS2 is installed on system
- PX4 SITL can have issues with ROS environment variables
- Ensures clean isolated environment for simulation

### Process Termination

All subprocesses are automatically terminated on:
- Normal mission completion
- User interrupt (Ctrl+C)
- Exception/error

Graceful shutdown ensures:
- Video file integrity (proper MKV finalization)
- All telemetry data saved
- Summary statistics generated

## Troubleshooting

### Connection timeout

```
Error: Drone connection timeout
```

**Solution:** Increase wait time in `start_px4()`:
```python
time.sleep(30)  # Increase from 25 to 30 seconds
```

### Video file empty

```
Warning: video_*.mkv is 0 bytes
```

**Solutions:**
- Check Gazebo camera plugin is loaded
- Verify UDP port 5600 is not blocked
- Check GStreamer installation: `gst-launch-1.0 --version`

### MAVLink connection refused

```
Error: Connection refused on port 14540
```

**Solutions:**
- Ensure PX4-Autopilot path is correct
- Check PX4 SITL launched successfully (remove `DEVNULL` temporarily)
- Verify no other process using port 14540: `lsof -i :14540`

### Import error: mavsdk

```
ModuleNotFoundError: No module named 'mavsdk'
```

**Solution:**
```bash
pip install mavsdk
```

## License

[Specify your license here]

## Contributing

[Specify contribution guidelines here]

## Authors

[Specify authors here]

---

**Note:** This system is designed for simulation environments. For real drone deployment, additional safety checks and fail-safes must be implemented.