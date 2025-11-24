#### *Nowa wersja, jeszcze do sprawdzenia nie bijcie jak nie działa/ słabo działa (ver. 2.)*

---
# PX4 with Gazebo and QGroundControl Installation Guide for Linux Mint and Ubuntu

This guide provides clear, step-by-step instructions for installing PX4 Autopilot along with Gazebo simulator and QGroundControl for use on Linux Mint and Ubuntu systems. It also includes instructions on creating executable files for drone simulation with multiple sensors.

---

## Prerequisites

- A machine running Linux Mint or Ubuntu (20.04 LTS, 22.04 LTS recommended)
- Internet connection
- Basic terminal usage skills

---

## Links to Official Repositories

- PX4 Autopilot: https://github.com/PX4/PX4-Autopilot
- Gazebo Simulator: https://github.com/osrf/gazebo
- QGroundControl: https://github.com/mavlink/qgroundcontrol

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
bash ubuntu.sh 
```
#### This script installs necessary dependencies including Gazebo (on Ubuntu 22.04, Gazebo is installed automatically).
### If you are on Linux Mint or Ubuntu 20.04, manually install Gazebo 11 with the following:
```
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gz-garden 
```
#### Update and upgrade your system:
```
sudo apt update && sudo apt upgrade -y
```
---

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

## Step 3: Install and Setup Gazebo Simulator

1. If not installed via PX4 setup script, install Gazebo using:
```
curl -sSL http://get.gazebosim.org | sh
```
2. Add your user to the dialout group for serial access:
```
sudo usermod -a -G dialout $USER
```
3. Remove `modemmanager` if it interferes:
```
sudo apt-get remove modemmanager -y
```
4. Install necessary GStreamer plugins for video streaming:
```
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```
5. Log out and back in to apply group membership changes.

---

## Step 4: Install QGroundControl

1. Download the latest QGroundControl from its official GitHub:
https://github.com/mavlink/qgroundcontrol/releases
2. Navigate to your Downloads folder:
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

## Step 5: Build and Run PX4 Simulation

1. Open terminal and navigate to PX4 folder:
```commandline
cd ~/PX4-Autopilot
```
2. Compile and launch PX4 SITL with Gazebo simulation:
```commandline
make px4_sitl gazebo
```
3. In a separate terminal, run QGroundControl to connect to the simulated drone:
```commandline
~/Downloads/QGroundControl.AppImage
```
4. QGroundControl will automatically connect to the simulation.

---

## Step 6: Create Executable Files for Drone Simulation with Multiple Sensors

1. PX4 simulation supports multiple vehicles and sensor models. To include sensors, modify the startup launch files or models in the `Tools/sitl_gazebo` folder within PX4.
2. You can create a custom executable by writing a Makefile or shell script to build and launch PX4 SITL with your specified models and sensor configurations. Basic example:
```commandline
#!/bin/bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

3. Save this script as `run_simulation.sh`, give it execution permission:
```commandline
chmod +x run_simulation.sh
```
4. Run simulation using:
```commandline
./run_simulation.sh
```

---

## Additional Notes

- Follow the official PX4 Gazebo documentation for advanced simulation features:
https://docs.px4.io/main/en/simulation/gazebo.html
- For troubleshooting, ensure all dependencies are installed and environment variables are correctly set.
- After installation, always verify by running the simulation and connecting QGroundControl successfully.

---

This guide is designed so users can simply copy/paste commands without needing to troubleshoot complexities. Enjoy simulating with PX4, Gazebo, and QGroundControl on your Linux Mint or Ubuntu system!