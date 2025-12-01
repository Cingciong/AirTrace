Data analysis guide: [PX4 Flight Log Analyzer](PX4_ANALYSIS.md) \
Data analysis guide: [Gazebo simulation recording view from camera](video_recording.md)

# PX4 with Gazebo and QGroundControl Installation Guide for Linux Mint and Ubuntu
This guide provides clear, step-by-step instructions for installing PX4 Autopilot along with Gazebo simulator and QGroundControl for use on Linux Mint and Ubuntu systems. It also includes instructions on creating executable files for drone simulation with multiple sensors.

---

## Prerequisites

- A machine running Linux Mint or Ubuntu (20.04 LTS, 22.04 LTS recommended)
- Internet connection
- Basic terminal usage skills

---
## Links to Official Repositories & manuals:

- PX4 Autopilot: https://github.com/PX4/PX4-Autopilot
- Gazebo Simulator: https://github.com/osrf/gazebo
- QGroundControl: https://github.com/mavlink/qgroundcontrol

---

- PX4 download and installation manual: https://docs.px4.io/main/en/dev_setup/building_px4
- Gazebo setup manual: https://docs.px4.io/main/en/dev_setup/building_px4
- QGrundcontrol manual: https://docs.qgroundcontrol.com/Stable_V4.3/en/qgc-user-guide/

---

## Step 1: Install PX4 Autopilot

1. Open a terminal.
2. Navigate to your home directory: ~cd
3. Clone the PX4 Autopilot repository recursively:
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
4. Change to the PX4 setup directory:
```
cd PX4-Autopilot/Tools/setup
```
5. Run the Ubuntu setup script:
```
./ubuntu.sh
```
#### This script installs necessary dependencies including Gazebo.
## Step 2: Verify or Install Java (Required for PX4 Setup Script)
1. Check if Java is installed:
```
java -version
```
2. If Java is not found, install OpenJDK 18:
```
sudo apt install openjdk-18-jdk openjdk-18-jre
```
3. Verify installation again:
```
java -version
```
---
## Step 3: Install QGroundControl

1. Download the latest QGroundControl from its official GitHub:
https://github.com/mavlink/qgroundcontrol/releases
#### Download the .AppImage file
2. Navigate to the folder where the .AppImage file has been downloaded (for example: *Downloads* folder):
```commandline
cd ~/Downloads
```
3. Make QGroundControl executable:
```commandline
chmod +x ./QGroundControl.AppImage
```
4. Run QGroundControl:
```commandline
./QGroundControl.AppImage
```
---

## Step 4: Build and Run PX4 Simulation

1. Open terminal and navigate to PX4 folder:
```commandline
cd ~/PX4-Autopilot
```
2. Compile and launch PX4 SITL with Gazebo simulation:
```commandline
make px4_sitl gz_x500_vision
```
3. If QGroundControl isn't already running, from a separate terminal run QGroundControl to connect to the simulated drone:
```commandline
~/Downloads/QGroundControl.AppImage
```
4. QGroundControl will automatically connect to the simulation.

5. Useful video on how to use QGroundcontrol: https://www.youtube.com/watch?v=0d23O_RUOmI
