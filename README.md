# AirTrace - Multi-Phase Video & Motion Analysis System

Complete pipeline for capturing, analyzing, and synchronizing video with drone telemetry data.

---

## 📋 Project Overview

**AirTrace** is a two-phase system:

1. **[Phase 1: Ground Testing](#-phase-1-ground-testing)** - Capture video in terrain and analyze motion using optical flow
2. **[Phase 2: Simulation](#-phase-2-simulation-drone-data)** - Synchronize video with PX4 drone telemetry data

---

# 🔸 Phase 1: Ground Testing

**Location:** `ground_testing/`

### What It Does
Records videos from ground-level tests and uses **sparse optical flow** to track motion and displacement in the terrain.

### Purpose
- Validate motion detection algorithms before drone deployment
- Measure terrain traversal using video analysis
- Test height measurement from video
- Extract motion vectors (Δx, Δy, trajectory)

### Files

| File | Purpose |
|------|---------|
| `main.py` | Core optical flow analysis pipeline |
| `csv.ipynb` | Sensor data processing (acceleration, orientation, GPS) |
| `csv_intepreter.py` | Parse and visualize sensor data from CSV files |
| `height measuring.ipynb` | Altitude estimation from video |
| `main.ipynb` | Interactive workflow notebook |

### How It Works: `main.py`

**Core Pipeline:**

```
Video File (.mov)
    ↓
[Preprocess]
├─ Load .mov file
├─ Resize frames (target height: 360px)
├─ Crop unwanted pixels (200px left)
└─ Select frame range
    ↓
[Sparse Optical Flow]
├─ Detect good features (500+ corners/edges)
├─ Track features frame-to-frame
├─ Calculate motion vectors (Δx, Δy)
└─ Filter by tracking quality
    ↓
[Analysis & Visualization]
├─ Plot Δx per frame
├─ Plot Δy per frame
├─ Compute 2D trajectory
└─ Calculate cumulative displacement
    ↓
Output: Motion plots + trajectory path
```

#### Key Parameters

```python
video_path = "G:/projekt gropwy/22.10.2025/video/raw/720p/5.1.mov"
target_height = 360              # Resize to height
frame_interval = 2               # Process every Nth frame
cut_pixels = 200                 # Crop left edge (px)
start_frame = 210                # Skip first N frames
meters_per_pixel_y = 0.0006      # Scale calibration (m/px)
fps_capture = 1209               # Original capture FPS
```

#### Output Visuals

1. **Δx Plot** - Horizontal motion magnitude per frame (pixels)
2. **Δy Plot** - Vertical motion magnitude per frame (pixels)  
3. **2D Trajectory** - XY path showing cumulative displacement

### Sensor Data Processing: `csv.ipynb`

Processes sensor CSV files with multi-step pipeline:

**Input:** CSV files with accelerometer, orientation, GPS data

**Steps:**
1. **Load Data** - Parse CSV with locale-specific separators (`,` or `;`)
2. **Coordinate Transform** - Convert acceleration from local → global (NED frame)
3. **Filtering** - Apply low-pass Butterworth filter (cutoff: 1 Hz, order: 3)
4. **Integration** - Double integrate: acceleration → velocity → position
5. **Visualization** - Plot in 2D/3D

**Output:** 
- Filtered accelerations in global frame
- Velocity vectors
- Position trajectory
- 3D visualization with acceleration overlay

### Example Ground Test

```python
# ground_testing/main.py

video_path = "5.1.mov"  # Test video
# Reads 5.1.mov
# Detects features in 500+ points
# Computes sparse optical flow
# Outputs:
#   ├─ Δx plot
#   ├─ Δy plot
#   └─ Trajectory plot
```

### Data Types

| Data | Type | Range | Meaning |
|------|------|-------|---------|
| **Δx** | float | -50 to +50 px | Horizontal motion per frame |
| **Δy** | float | -50 to +50 px | Vertical motion per frame |
| **Trajectory** | ndarray (N,2) | varies | Cumulative XY displacement |

---

# 🔶 Phase 2: Simulation (Drone Data)

**Location:** `sim/`

### What It Does
Synchronizes drone video with PX4 autopilot telemetry to create frame-by-frame paired data with attitude and altitude.

### Purpose
- Combine video with drone orientation data
- Create training datasets for ML models
- Analyze drone behavior during flight
- Pair video frames with UAV state (yaw, pitch, roll, altitude)

### Files

| File | Purpose |
|------|---------|
| `main.ipynb` | Main workflow & analysis |
| `utilities/TelementryVideoSync.py` | Core synchronization class |
| `utilities/PX4CSVPlotter.py` | Parse PX4 sensor data |
| `data/1/mp4.mp4` | Drone video recording |
| `data/1/ulg.ulg` | PX4 telemetry log |
| `data/1/csv/` | Converted sensor CSVs |

### How It Works: `TelemetryVideoSync` Class

**Complete Pipeline:**

```
ULog File (ulg.ulg)
    ↓
[read_telemetry()]
├─ Parse ULog with pyulog
├─ Extract sensor data
│   ├─ vehicle_attitude (yaw, pitch, roll)
│   ├─ sensor_gps_0 (lat, lon, alt)
│   ├─ sensor_accel_0 (accelerometer)
│   ├─ sensor_gyro_0 (gyroscope)
│   └─ ... other sensors
└─ Convert to CSV files
    ↓
[load_telemetry()]
├─ Read CSV files (PX4CSVPlotter)
├─ Extract attitude angles
├─ Normalize to [-180, +180]°
├─ Resample to N samples
└─ Store in memory
    ↓
Video File (mp4.mp4)
    ↓
[save_video_to_arrays()]
├─ Open MP4 with OpenCV
├─ Load all frames
├─ Convert BGR → RGB
└─ Store as NumPy array
    ↓
[analyze_telemetry()]
├─ Cut video (time mask):
│   start = VIDEO_START_TIME * FPS
│   end = VIDEO_END_TIME * FPS
├─ Cut telemetry (index mask):
│   [TELEMETRY_START_IDX : TELEMETRY_END_IDX]
├─ Interpolate to match lengths
│   frames.shape[0] == telemetry.shape[0]
└─ Align frame-by-frame
    ↓
[play_telemetry_video()]
├─ Display frames
├─ Overlay telemetry text:
│   - Frame index
│   - Altitude (m)
│   - Angles (°)
│   - Timestamp
└─ Save visualization
```

#### Methods

| Method | Input | Output | Purpose |
|--------|-------|--------|---------|
| `read_telemetry()` | .ulg file | .csv files | Convert ULog to CSV |
| `load_telemetry()` | .csv files | NumPy arrays | Parse & normalize |
| `save_video_to_arrays()` | .mp4 file | NumPy array | Load video frames |
| `analyze_telemetry()` | frames + telemetry | Synchronized pairs | Cut & align |
| `play_telemetry_video()` | synchronized data | Display window | Visualize overlay |
| `debug_detect_motion_video()` | frames | Motion plot | Find motion start/end |

#### Key Parameters

```python
# Telemetry cutting indices
TELEMETRY_START_IDX = 16970        # Start index in telemetry array
TELEMETRY_END_IDX = 63418          # End index in telemetry array

# Video cutting time (seconds)
VIDEO_START_MESS = 22.31860479     # Start time in seconds
VIDEO_END_MESS = 457.5700117       # End time in seconds

# Other options
SAVE_EVERY_N = 1                   # Sample every Nth frame
PLOT_EVERY_N = 10                  # Plot telemetry every Nth frame
```

#### Usage Example

```python
from utilities.TelementryVideoSync import TelemetryVideoSync

sync = TelemetryVideoSync(
    telemetry_start_idx=16970,
    telemetry_end_idx=63418,
    video_start_time=22.31860479,
    video_end_time=457.5700117,
    video_path="data/1/mp4.mp4",
    ulog_path="data/1/ulg.ulg",
    csv_path="data/1/csv",
)

# Step 1: Convert ULog → CSV
sync.read_telemetry()

# Step 2: Load & process all data
sync.analyze_telemetry()

# Step 3: Display synchronized video
sync.play_telemetry_video()
```

### Output Data Structure

Each synchronized frame contains:

```python
{
    'frame': ndarray (H, W, 3)     # Video frame (RGB)
    'frame_idx': int               # Frame number
    'yaw': float                   # Rotation [-180, +180]°
    'pitch': float                 # Nose angle [-180, +180]°
    'roll': float                  # Wing angle [-180, +180]°
    'gps_alt': float              # Altitude (meters)
    'gps_time': float             # Timestamp
}
```

### Angle Definitions

| Angle | Axis | Meaning |
|-------|------|---------|
| **Yaw** | Z (vertical) | Compass heading, 0°=North |
| **Pitch** | Y (lateral) | Nose up (+) / down (-) |
| **Roll** | X (longitudinal) | Right wing up (+) / down (-) |

---

## 🗂️ Complete Project Structure

```
CamTrack/
│
├── 🔸 PHASE 1: Ground Testing
│   ├── ground_testing/
│   │   ├── main.py                    ⭐ Optical flow analysis
│   │   ├── csv.ipynb                  📓 Sensor data processing
│   │   ├── csv_intepreter.py          🔧 CSV parser & viz
│   │   ├── height measuring.ipynb     📐 Altitude estimation
│   │   ├── main.ipynb                 📓 Interactive workflow
│   │   ├── videos/                    🎥 Test video files
│   │   ├── requirements.txt
│   │   └── __pycache__/
│   │
├── 🔶 PHASE 2: Simulation/Drone
│   ├── sim/
│   │   ├── main.ipynb                          📓 Main workflow
│   │   ├── semi_manual_video_sync.ipynb        📓 Manual sync
│   │   ├── semi_manual_video_sync.py           📝 Original sync
│   │   ├── utilities/
│   │   │   ├── TelementryVideoSync.py         ⭐ Core sync class
│   │   │   ├── PX4CSVPlotter.py               🔧 CSV parser
│   │   │   ├── plt.py                        🛠️ Plotting utils
│   │   │   └── __pycache__/
│   │   │
│   │   └── data/
│   │       └── 1/
│   │           ├── mp4.mp4                    🎥 Drone video
│   │           ├── ulg.ulg                    📊 Telemetry log
│   │           └── csv/                       📁 Extracted CSVs
│   │               ├── vehicle_attitude_0.csv
│   │               ├── sensor_gps_0.csv
│   │               ├── sensor_accel_0.csv
│   │               └── ... (other sensors)
│   │
├── .gitignore
├── README.md                                  (this file)
└── README_NEW.md                             (detailed version)
```

---

## 🔄 Complete Workflow Path

```
PROJECT START
  │
  ├─► 🔸 PHASE 1: Ground Testing
  │   │
  │   ├─ Record video in terrain
  │   │   └─ Video files → ground_testing/videos/
  │   │
  │   ├─ Analyze motion (main.py)
  │   │   ├─ Input: .mov video
  │   │   ├─ Process: Sparse optical flow
  │   │   └─ Output: Δx, Δy plots + trajectory
  │   │
  │   ├─ Validate algorithms
  │   │   └─ Review motion detection quality
  │   │
  │   └─ Approve for drone phase
  │       │
  │       ↓
  │
  ├─► 🔶 PHASE 2: Simulation/Drone
  │   │
  │   ├─ Record drone flight + telemetry
  │   │   ├─ Video → data/1/mp4.mp4
  │   │   └─ ULog → data/1/ulg.ulg
  │   │
  │   ├─ Read telemetry (TelemetryVideoSync)
  │   │   ├─ Convert: ULog → CSV
  │   │   └─ Store: data/1/csv/
  │   │
  │   ├─ Process data
  │   │   ├─ Load telemetry (attitudes, altitude)
  │   │   ├─ Load video frames
  │   │   └─ Normalize angles & lengths
  │   │
  │   ├─ Synchronize
  │   │   ├─ Cut video (time mask)
  │   │   ├─ Cut telemetry (index mask)
  │   │   └─ Interpolate → same length
  │   │
  │   ├─ Visualize
  │   │   ├─ Display video with overlay
  │   │   ├─ Show: altitude, angles, timestamp
  │   │   └─ Save output video
  │   │
  │   └─ Generate dataset
  │       └─ Paired frame + telemetry data
  │
  └─► PROJECT END
      (Ready for ML training / analysis)
```

---

## 📊 Phase Comparison

| Feature | Phase 1 (Ground) | Phase 2 (Drone) |
|---------|-----------------|-----------------|
| **Location** | `ground_testing/` | `sim/` |
| **Input** | .mov video | .mp4 video + .ulg log |
| **Analysis Type** | Optical flow | Sensor fusion |
| **Key Method** | Sparse optical flow | Telemetry sync |
| **Output** | Motion vectors | Paired frames + telemetry |
| **Purpose** | Algorithm validation | Dataset creation |
| **Sensors Used** | Video only | IMU, GPS, barometer, video |
| **Output Data** | Δx, Δy, trajectory | Frame + yaw/pitch/roll/alt |

---

## 🚀 Getting Started

### Phase 1: Ground Testing

```bash
cd ground_testing
python main.py
# Output: Motion plots and trajectory
```

### Phase 2: Drone Data Sync

```python
from utilities.TelementryVideoSync import TelemetryVideoSync

sync = TelemetryVideoSync()
sync.read_telemetry()      # ULog → CSV
sync.analyze_telemetry()   # Load & process
sync.play_telemetry_video()  # Display
```

---

## 💾 Dependencies

```
opencv-python      # Video processing
numpy              # Array operations
pandas             # CSV handling
matplotlib         # Plotting
pyulog             # ULog parsing
scipy              # Signal processing
```

Install:
```bash
pip install opencv-python numpy pandas matplotlib pyulog scipy
```

---

## 📝 Notes

- **Phase 1** validates algorithms on ground
- **Phase 2** applies them to drone data
- Data flows sequentially: Phase 1 → Phase 2
- Both phases produce synchronized motion data
