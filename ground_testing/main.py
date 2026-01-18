#%% Imports
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import os

#%% Parameters
video_path = "G:/projekt gropwy/22.10.2025/video/raw/720p/5.1.mov"
target_height = 360
frame_interval = 2
cut_pixels = 200
start_frame = 210
fps_capture = 1209  # original capture frame rate
meters_per_pixel_y = 0.0006  # Y-scale in meters
meters_per_pixel_y = meters_per_pixel_y / (1080 / target_height)  # adjust for resized height

#%% Read and preprocess video frames
cap = cv2.VideoCapture(video_path)
frames = []
frame_counter = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    if frame_counter % frame_interval == 0:
        # Resize frame
        h, w = frame.shape[:2]
        scale = target_height / h
        new_width = int(w * scale)
        resized_frame = cv2.resize(frame, (new_width, target_height), interpolation=cv2.INTER_AREA)

        # Crop left side
        cropped_frame = resized_frame[:, cut_pixels:]
        frames.append(cropped_frame)

    frame_counter += 1

cap.release()

# Trim frames starting at start_frame
frames = frames[start_frame:]

print(f"Frames captured after preprocessing: {len(frames)}")
if frames:
    print(f"First frame shape: {frames[0].shape}")

#%% Sparse optical flow function
def sparse_optical_flow(prev_gray, gray):
    prev_points = cv2.goodFeaturesToTrack(
        prev_gray, maxCorners=500, qualityLevel=0.3,
        minDistance=7, blockSize=7
    )
    if prev_points is None:
        return 0.0, 0.0, None

    lk_params = dict(
        winSize=(15, 15),
        maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
    )

    next_points, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_points, None, **lk_params)
    good_new = next_points[status == 1]
    good_old = prev_points[status == 1]

    dx, dy = np.mean(good_new - good_old, axis=0)
    return dx, dy, next_points

#%% Compute motion
gray_frames = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
motion = []

print("Computing sparse optical flow...")
for i in range(1, len(gray_frames)):
    dx, dy, _ = sparse_optical_flow(gray_frames[i - 1], gray_frames[i])
    motion.append([dx, dy])

motion = np.array(motion)
print(f"Sparse motion shape: {motion.shape}")
print("Example (first 5 frames):")
print(motion[:5])

#%% Plot motion per frame
frames_idx = np.arange(1, len(motion) + 1)
motion_dx = motion[:, 0]
motion_dy = motion[:, 1]

# Δx per frame
plt.figure(figsize=(12, 4))
plt.plot(frames_idx, motion_dx, color="blue")
plt.title("Sparse Optical Flow Δx (pixels/frame)")
plt.xlabel("Frame")
plt.ylabel("Δx")
plt.grid(True)
plt.show()

# Δy per frame
plt.figure(figsize=(12, 4))
plt.plot(frames_idx, motion_dy, color="green")
plt.title("Sparse Optical Flow Δy (pixels/frame)")
plt.xlabel("Frame")
plt.ylabel("Δy")
plt.grid(True)
plt.show()

# 2D trajectory
trajectory = np.cumsum(motion, axis=0)
plt.figure(figsize=(6, 6))
plt.plot(trajectory[:, 0], trajectory[:, 1], color="purple")
plt.title("Sparse Optical Flow 2D Trajectory (pixels)")
plt.xlabel("X displacement")
plt.ylabel("Y displacement")
plt.axis("equal")
plt.grid(True)
plt.show()

