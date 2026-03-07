import os
import cv2
import shutil
import pandas as pd
from sklearn.model_selection import train_test_split

videos_dir = "sim_data/mp4"
telemetry_dir = "sim_data/csv"
dataset_dir = "Dataset"
frames_dir = "frames_temp"

os.makedirs(frames_dir, exist_ok=True)
os.makedirs(dataset_dir, exist_ok=True)

all_rows = []
global_frame_index = 0

video_files = sorted([f for f in os.listdir(videos_dir) if f.lower().endswith((".mp4", ".mkv"))])
csv_files = sorted([f for f in os.listdir(telemetry_dir) if f.lower().endswith(".csv")])

for video_file, csv_file in zip(video_files, csv_files):

    video_path = os.path.join(videos_dir, video_file)
    telemetry_path = os.path.join(telemetry_dir, csv_file)

    video = cv2.VideoCapture(video_path)
    fps = video.get(cv2.CAP_PROP_FPS)

    frame_index = 0
    frame_times = []
    frame_names = []

    while True:
        ret, frame = video.read()
        if not ret:
            break
        frame_name = f"{global_frame_index}.jpg"
        cv2.imwrite(os.path.join(frames_dir, frame_name), frame)
        frame_time = frame_index / fps
        frame_times.append(frame_time)
        frame_names.append(frame_name)
        frame_index += 1
        global_frame_index += 1

    video.release()

    df = pd.read_csv(telemetry_path)
    df.columns = df.columns.str.strip().str.lower()

    rename_map = {
        "sensor_acellx": "ax",
        "sensor_acelly": "ay",
        "sensor_acellz": "az",
        "sensor_accelx": "ax",
        "sensor_accely": "ay",
        "sensor_accelz": "az",
        "accel_x": "ax",
        "accel_y": "ay",
        "accel_z": "az",
        "sensor_gyrox": "gx",
        "sensor_gyroy": "gy",
        "sensor_gyroz": "gz",
        "gyro_x": "gx",
        "gyro_y": "gy",
        "gyro_z": "gz",
        "sensor_magx": "mx",
        "sensor_magy": "my",
        "sensor_magz": "mz",
        "mag_x": "mx",
        "mag_y": "my",
        "mag_z": "mz",
        "sensor_baro": "barometer",
        "barometer": "barometer",
        "sensor_gps_lat": "lat",
        "gps_latitude": "lat",
        "lat": "lat",
        "sensor_gps_lon": "lon",
        "gps_longitude": "lon",
        "lon": "lon",
        "sensor_gps_alt": "alt",
        "gps_altitude": "alt",
        "alt": "alt"
    }

    df = df.rename(columns=rename_map)
    df = df.sort_values("timestamp").reset_index(drop=True)

    start_alt = df["alt"].iloc[0]
    df["alt"] = df["alt"] - start_alt

    for i, t in enumerate(frame_times):
        idx = (df["timestamp"] - t).abs().idxmin()
        r = df.loc[idx]
        all_rows.append({
            "img": frame_names[i],
            "ax": r["ax"],
            "ay": r["ay"],
            "az": r["az"],
            "gx": r["gx"],
            "gy": r["gy"],
            "gz": r["gz"],
            "mx": r["mx"],
            "my": r["my"],
            "mz": r["mz"],
            "barometer": r["barometer"],
            "lat": r["lat"],
            "lon": r["lon"],
            "alt": r["alt"]
        })

dataset_df = pd.DataFrame(all_rows)

train_df, temp_df = train_test_split(dataset_df, test_size=0.2, shuffle=False)
val_df, test_df = train_test_split(temp_df, test_size=0.5, shuffle=False)

images_dir = os.path.join(dataset_dir, "images")
labels_dir = os.path.join(dataset_dir, "labels")

os.makedirs(os.path.join(images_dir, "train"), exist_ok=True)
os.makedirs(os.path.join(images_dir, "val"), exist_ok=True)
os.makedirs(os.path.join(images_dir, "test"), exist_ok=True)
os.makedirs(labels_dir, exist_ok=True)

for img in train_df["img"]:
    shutil.copy(os.path.join(frames_dir, img), os.path.join(images_dir, "train", img))

for img in val_df["img"]:
    shutil.copy(os.path.join(frames_dir, img), os.path.join(images_dir, "val", img))

for img in test_df["img"]:
    shutil.copy(os.path.join(frames_dir, img), os.path.join(images_dir, "test", img))

train_df.to_csv(os.path.join(labels_dir, "train.csv"), index=False)
val_df.to_csv(os.path.join(labels_dir, "val.csv"), index=False)
test_df.to_csv(os.path.join(labels_dir, "test.csv"), index=False)

shutil.rmtree(frames_dir)