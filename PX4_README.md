#### *Wersja z Claude (ver. 1.)*

---
# DOWNLOAD & INSTALL SIMULATION PROGRAMS ON LINUX
## Gazebo with PX4:
### Step 1: Install Dependencies

```commandline
sudo apt update
sudo apt upgrade -y
sudo apt install -y git build-essential cmake python3-pip python3-dev
```

### Step 2: Install PX4 Autopilot

```commandline
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
```

### Step 3: Build PX4 for Gazebo Simulation

```commandline
cd ~/PX4-Autopilot
make px4_sitl gz_x500_vision
```

## QGroundControl
***Remove modemmanager (skip if not found - it's optional):***
```commandline
sudo apt remove modemmanager -y 2>/dev/null || echo "modemmanager not installed, skipping..."
```

***change username to yours here:***

```commandline
sudo usermod -a -G dialout $USER
```

***download QGroundControl:***
```commandline
cd ~/Downloads
wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.2/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
```

***run  the program (after creating PX4 drone):***
```commandline
./QGroundControl.AppImage
```

## After Installation:
1. Start PX4 simulation first:
2. Launch QGroundControl:

## Changing home position:
### stop the simulation if it's running and:

```commandline
cd ~/PX4-Autopilot
PX4_HOME_LAT=54.3712178 PX4_HOME_LON=18.6133142 PX4_HOME_ALT=50 make px4_sitl gz_x500_vision
```