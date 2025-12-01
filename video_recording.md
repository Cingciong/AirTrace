# Joystick Control, Camera Viewing and Recording Guide (PX4 + Gazebo + QGroundControl)

This guide explains how to:
- Use a joystick to control your PX4 SITL drone in Gazebo.
- View and record video from the simulated downward‑facing camera using QGroundControl.

It assumes you have already:
- Followed your PX4 + Gazebo + QGroundControl installation guide.
- Started a simulation with a camera‑equipped model using **one** of:
  - `make px4_sitl gz_mono_cam_down`
  - `make px4_sitl gz_mono_cam`  
- Downloaded all required PX4 and QGroundControl files.

For additional background on simulation cameras and video streaming, see:
- PX4 Gazebo Classic video streaming: <https://docs.px4.io/main/en/sim_gazebo_classic/#video-streaming>  
- PX4 camera simulation overview: <https://docs.px4.io/main/en/simulation/#camera-simulation>  
- QGroundControl joystick setup guide: <https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/joystick.html>  

---

## 1. Start PX4 + Gazebo Simulation

1. Open a terminal in your `PX4-Autopilot` directory.
2. Run one of the following to start SITL with a mono camera:
```
make px4_sitl gz_mono_cam_down
```

```
make px4_sitl gz_mono_cam
```

3. Wait until PX4 finishes booting and the Gazebo Sim window appears with your drone in the world.

If these commands run successfully, the simulated camera and video stream are already configured on the PX4 side; you only need to set up QGroundControl as described below.

---

## 2. Connect QGroundControl and Prepare Joystick

1. Start **QGroundControl** (e.g. by running the `.AppImage` on Linux).
2. QGroundControl should automatically connect to the PX4 SITL vehicle.
3. Plug in your joystick/gamepad.

### Joystick setup

The full joystick configuration is covered here and does not need to be duplicated:

- **Follow this guide step‑by‑step:**  
<https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/joystick.html>

In short, you will:
- Enable the joystick input.
- Assign axes to roll, pitch, yaw, and throttle.
- Configure buttons as desired (e.g. arming, mode switches).
- Run the calibration procedure in QGroundControl.

Once calibration is complete, you will be able to fly the simulated drone in Gazebo using the joystick while QGroundControl is connected.

---

## 3. Configure Video in QGroundControl

Because you started PX4 with `gz_mono_cam_down` or `gz_mono_cam` and completed the installation/download steps, the camera stream side is already set up. You just have to tell QGroundControl how to receive and record it.

1. In QGroundControl, open **Application Settings** (gear icon).
2. Go to the **General** tab.
3. In the **Video** section, set the following:

- **Video Source:** `UDP h.264 Video Stream`  
- **UDP Port:** `5600`  

(This matches the default PX4 SITL video stream configuration.)

4. Leave the aspect ratio at the default value (it will match the camera stream).
5. Under **Video Recording**, you can optionally adjust:
- **Max Storage Usage**
- **Video File Format** (e.g. `mkv` or `mp4`, depending on your QGroundControl version)

Use the screenshot you captured as a reference: the video settings in your QGroundControl should match what is shown there (Video Source = *UDP h.264 Video Stream*, Port = *5600*).

---

## 4. Viewing and Recording the Camera

1. Return to the **Fly** view in QGroundControl.
2. Once the video settings are correct and the PX4 SITL camera is streaming, a live video window will appear in the lower‑left area of the interface.
3. A **Record** button (circle icon) will appear on or near the video window.
4. To record:
- Click the **Record** button to start recording the camera feed.
- Fly your mission using the joystick.
- Click **Record** again to stop when finished.

QGroundControl will save the video files to your user’s Documents folder:

- **Linux/macOS/Windows (desktop default):**  
`Documents/QGroundControl/Video/`

You can open this folder with your file manager and find the recorded video files there (named with timestamps).

---

## 5. Quick Checklist

Before flying and recording, make sure:

- [x] PX4 SITL is running with `gz_mono_cam_down` or `gz_mono_cam`.  
- [x] QGroundControl is connected to the simulated vehicle.  
- [x] Joystick is calibrated via the official joystick setup page.  
- [x] In QGroundControl **General → Video**:  
- Video Source = **UDP h.264 Video Stream**  
- UDP Port = **5600**  
- [x] In **Fly** view, live video is visible and the **Record** button appears.  
- [x] After recording, video files are present in `Documents/QGroundControl/Video/`.

With these steps, the user can fully control the drone via joystick and view/record camera footage from the Gazebo simulation through QGroundControl.