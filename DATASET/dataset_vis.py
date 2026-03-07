import os
import cv2
import pandas as pd
import numpy as np

videos_dir    = "sim_data/mp4"
telemetry_dir = "sim_data/csv"
output_video  = "telemetry_video.mp4"

FONT    = cv2.FONT_HERSHEY_SIMPLEX
OVERLAY_W = 380
OVERLAY_H = 300

rename_map = {
    "sensor_acellx": "ax", "sensor_acelly": "ay", "sensor_acellz": "az",
    "sensor_accelx": "ax", "sensor_accely": "ay", "sensor_accelz": "az",
    "accel_x": "ax", "accel_y": "ay", "accel_z": "az",
    "sensor_gyrox": "gx", "sensor_gyroy": "gy", "sensor_gyroz": "gz",
    "gyro_x": "gx", "gyro_y": "gy", "gyro_z": "gz",
    "sensor_magx": "mx", "sensor_magy": "my", "sensor_magz": "mz",
    "mag_x": "mx", "mag_y": "my", "mag_z": "mz",
    "sensor_baro": "barometer", "barometer": "barometer",
    "sensor_gps_lat": "lat", "gps_latitude": "lat", "lat": "lat",
    "sensor_gps_lon": "lon", "gps_longitude": "lon", "lon": "lon",
    "sensor_gps_alt": "alt", "gps_altitude": "alt", "alt": "alt",
}

video_files = sorted([f for f in os.listdir(videos_dir) if f.lower().endswith((".mp4", ".mkv"))])
csv_files   = sorted([f for f in os.listdir(telemetry_dir) if f.lower().endswith(".csv")])

first_video = cv2.VideoCapture(os.path.join(videos_dir, video_files[0]))
frame_w = int(first_video.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_h = int(first_video.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps     = first_video.get(cv2.CAP_PROP_FPS)
first_video.release()

fourcc = cv2.VideoWriter_fourcc(*"mp4v")
writer = cv2.VideoWriter(output_video, fourcc, fps, (frame_w, frame_h))

all_telemetry = []
for csv_file in csv_files:
    df = pd.read_csv(os.path.join(telemetry_dir, csv_file))
    df.columns = df.columns.str.strip().str.lower()
    df = df.rename(columns=rename_map)
    df = df.sort_values("timestamp").reset_index(drop=True)
    df["alt"] = df["alt"] - df["alt"].iloc[0]
    all_telemetry.append(df)

combined_telem = pd.concat(all_telemetry, ignore_index=True)
lat_min, lat_max = combined_telem["lat"].min(), combined_telem["lat"].max()
lon_min, lon_max = combined_telem["lon"].min(), combined_telem["lon"].max()
alt_min, alt_max = combined_telem["alt"].min(), combined_telem["alt"].max()
all_gps_pts_by_video = []
for df in all_telemetry:
    pts = list(zip(df["lat"].values, df["lon"].values))
    all_gps_pts_by_video.append(pts)


def draw_telemetry_panel(frame, r):
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (10 + OVERLAY_W, 10 + OVERLAY_H), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    def put(text, line, scale=0.46, thickness=1, clr=(220, 220, 220)):
        cv2.putText(frame, text, (20, 32 + line * 22), FONT, scale, clr, thickness, cv2.LINE_AA)

    put("--- Accelerometer ---", 0, scale=0.42, clr=(180, 180, 255))
    put(f"ax={r['ax']:+.3f}  ay={r['ay']:+.3f}  az={r['az']:+.3f}", 1, scale=0.42)
    put("--- Gyroscope ---",    2, scale=0.42, clr=(180, 255, 180))
    put(f"gx={r['gx']:+.3f}  gy={r['gy']:+.3f}  gz={r['gz']:+.3f}", 3, scale=0.42)
    put("--- Magnetometer ---", 4, scale=0.42, clr=(255, 220, 150))
    put(f"mx={r['mx']:+.3f}  my={r['my']:+.3f}  mz={r['mz']:+.3f}", 5, scale=0.42)
    put("--- Navigation ---",   6, scale=0.42, clr=(255, 180, 255))
    put(f"Baro: {r['barometer']:.2f} Pa", 7, scale=0.42)
    put(f"Lat:  {r['lat']:.6f}", 8, scale=0.42)
    put(f"Lon:  {r['lon']:.6f}", 9, scale=0.42)
    put(f"Alt:  {r['alt']:+.2f} m", 10, scale=0.52, thickness=2, clr=(100, 255, 255))
    cv2.rectangle(frame, (10, 10), (10 + OVERLAY_W, 10 + OVERLAY_H), (80, 80, 80), 1)


def to_mm_px(lat, lon, mm_x0, mm_y0, mm_size, margin=10):
    lat_range = lat_max - lat_min or 1e-6
    lon_range = lon_max - lon_min or 1e-6
    px = int(margin + (lon - lon_min) / lon_range * (mm_size - 2 * margin))
    py = int(margin + (1 - (lat - lat_min) / lat_range) * (mm_size - 2 * margin))
    return (mm_x0 + px, mm_y0 + py)


def draw_minimap(frame, all_gps_pts_by_video, video_idx, frame_telem_idx):
    mm_size = 180
    mm_pad  = 12
    x0 = frame_w - mm_size - mm_pad
    y0 = mm_pad

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + mm_size, y0 + mm_size), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
    cv2.rectangle(frame, (x0, y0), (x0 + mm_size, y0 + mm_size), (100, 100, 100), 1)

    for vi, pts in enumerate(all_gps_pts_by_video):
        color = (60, 60, 60) if vi != video_idx else (120, 120, 120)
        for i in range(1, len(pts)):
            p1 = to_mm_px(pts[i-1][0], pts[i-1][1], x0, y0, mm_size)
            p2 = to_mm_px(pts[i][0],   pts[i][1],   x0, y0, mm_size)
            cv2.line(frame, p1, p2, color, 1, cv2.LINE_AA)

    cur_pts = all_gps_pts_by_video[video_idx]
    for i in range(1, min(frame_telem_idx + 1, len(cur_pts))):
        p1 = to_mm_px(cur_pts[i-1][0], cur_pts[i-1][1], x0, y0, mm_size)
        p2 = to_mm_px(cur_pts[i][0],   cur_pts[i][1],   x0, y0, mm_size)
        cv2.line(frame, p1, p2, (0, 200, 100), 1, cv2.LINE_AA)

    if frame_telem_idx < len(cur_pts):
        cx, cy = to_mm_px(cur_pts[frame_telem_idx][0], cur_pts[frame_telem_idx][1], x0, y0, mm_size)
        cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 0), 1, cv2.LINE_AA)

    cv2.putText(frame, "GPS Track", (x0 + 4, y0 + mm_size + 14),
                FONT, 0.38, (180, 180, 180), 1, cv2.LINE_AA)


def draw_alt_bar(frame, alt):
    bar_w  = 18
    bar_h  = 160
    bar_pad = 14
    x0 = frame_w - bar_w - bar_pad
    y0 = frame_h - bar_h - bar_pad - 20

    alt_range  = alt_max - alt_min or 1e-6
    fill_frac  = max(0.0, min(1.0, (alt - alt_min) / alt_range))
    fill_h     = int(fill_frac * bar_h)

    cv2.rectangle(frame, (x0, y0), (x0 + bar_w, y0 + bar_h), (30, 30, 30), -1)
    if fill_h > 0:
        cv2.rectangle(frame, (x0, y0 + bar_h - fill_h), (x0 + bar_w, y0 + bar_h), (0, 210, 210), -1)
    cv2.rectangle(frame, (x0, y0), (x0 + bar_w, y0 + bar_h), (120, 120, 120), 1)
    cv2.putText(frame, f"{alt:+.1f}m", (x0 - 36, y0 + bar_h + 14),
                FONT, 0.38, (100, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, "ALT", (x0 + 1, y0 - 5),
                FONT, 0.38, (180, 180, 180), 1, cv2.LINE_AA)


total_videos = len(video_files)

for video_idx, (video_file, df) in enumerate(zip(video_files, all_telemetry)):
    video_path = os.path.join(videos_dir, video_file)
    cap = cv2.VideoCapture(video_path)
    video_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"[{video_idx+1}/{total_videos}] {video_file}  ({total_frames} frames @ {video_fps:.1f} fps)")

    frame_index = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame.shape[:2] != (frame_h, frame_w):
            frame = cv2.resize(frame, (frame_w, frame_h))

        t = frame_index / video_fps
        telem_idx = int((df["timestamp"] - t).abs().idxmin())
        r = df.loc[telem_idx]

        draw_telemetry_panel(frame, r)
        draw_minimap(frame, all_gps_pts_by_video, video_idx, telem_idx)
        draw_alt_bar(frame, r["alt"])

        progress = (frame_index + 1) / max(total_frames, 1)
        bar_total_w = frame_w - 20
        filled_w    = int(progress * bar_total_w)
        cv2.rectangle(frame, (10, frame_h - 8), (10 + bar_total_w, frame_h - 2), (50, 50, 50), -1)
        cv2.rectangle(frame, (10, frame_h - 8), (10 + filled_w,    frame_h - 2), (0, 200, 100), -1)

        label = f"Video {video_idx+1}/{total_videos}: {video_file}"
        cv2.putText(frame, label, (10, frame_h - 12),
                    FONT, 0.38, (180, 180, 180), 1, cv2.LINE_AA)

        writer.write(frame)
        frame_index += 1

    cap.release()
    print(f"  Done.")

writer.release()
print(f"\nFinished. Output saved to: {output_video}")