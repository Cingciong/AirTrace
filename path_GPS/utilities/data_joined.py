import pandas as pd
import cv2
import os

# === CONFIG ===
csv_path = "../data/train.csv"
frames_dir = "../data/train"
output_video = "output.mp4"
fps = 30

# === STEP 1: Load CSV ===
df = pd.read_csv(csv_path)

# 🔑 Extract numeric part from filename (e.g., "frame_12.png" → 12)
df["frame_num"] = df["img"].str.extract(r'(\d+)').astype(int)

# Sort by numeric frame number
df_sorted = df.sort_values(by="frame_num")

# Optional: save sorted CSV (without helper column if you want)
df_sorted.drop(columns=["frame_num"]).to_csv("sorted.csv", index=False)

# === STEP 2: Build ordered frame list ===
frame_files = []

for filename in df_sorted["img"]:
    filepath = os.path.join(frames_dir, filename)

    if os.path.exists(filepath):
        frame_files.append(filepath)
    else:
        print(f"Warning: Missing {filename}")

# Check if we have frames
if not frame_files:
    raise ValueError("No valid frames found!")

# === STEP 3: Initialize video writer ===
first_frame = cv2.imread(frame_files[0])
height, width, _ = first_frame.shape

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

# === STEP 4: Write frames ===
for frame_path in frame_files:
    frame = cv2.imread(frame_path)

    if frame is None:
        print(f"Skipping unreadable frame: {frame_path}")
        continue

    video.write(frame)

video.release()

print("✅ Video created:", output_video)