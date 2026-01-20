from pyulog import ULog
import pandas as pd
import os
import cv2
import numpy as np
from utilities.PX4CSVPlotter import PX4CSVPlotter
import matplotlib.pyplot as plt
from tqdm import tqdm


class TelemetryVideoSync:
    def __init__(
        self,
        telemetry_start_idx,
        telemetry_end_idx,
        video_start_time,
        video_end_time,
        video_path,
        ulog_path,
        csv_path,
        save_every_n=1,
        plot_every_n=10
    ):

        self.telemetry_start_idx = telemetry_start_idx
        self.telemetry_end_idx = telemetry_end_idx
        self.video_start_time = video_start_time
        self.video_end_time = video_end_time
        self.video_path = video_path
        self.ulog_path = ulog_path
        self.csv_path = csv_path
        self.fps = 19.5
        self.save_every_n = save_every_n
        self.plot_every_n = plot_every_n

        self.frames = None
        self.gps_alt = None
        self.gps_time = None
        self.yaw = None
        self.pitch = None
        self.roll = None

    def read_telemetry(self):
        ulog = ULog(self.ulog_path)

        MONITOR_CSV = [
            "vehicle_attitude",
            "sensor_accel",
            "sensor_gyro",
            "sensor_mag",
            "sensor_baro",
            "sensor_gps",
            "vehicle_local_position",
            "vehicle_global_position"
        ]

        os.makedirs(self.csv_path, exist_ok=True)

        for data in ulog.data_list:
            if data.name in MONITOR_CSV:
                df = pd.DataFrame(data.data)

                if "timestamp" in df.columns:
                    df["timestamp_s"] = df["timestamp"] * 1e-6

                filename = f"{data.name}_{data.multi_id}.csv"
                filepath = os.path.join(self.csv_path, filename)
                df.to_csv(filepath, index=False)
                print(f"Saved {filename}")

    def save_video_to_arrays(self):
        self.read_fps()
        cap = cv2.VideoCapture(self.video_path)
        frames = []
        frame_idx = 0

        # Get total frames for tqdm progress
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        with tqdm(total=total_frames, desc="Reading frames") as pbar:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                if frame_idx % self.save_every_n == 0:
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frames.append(frame_rgb)

                frame_idx += 1
                pbar.update(1)

        cap.release()

        frames = np.array(frames)
        print("Frames shape:", frames.shape)
        return frames

    def read_fps(self):
        cap = cv2.VideoCapture(self.video_path)
        self.fps = cap.get(cv2.CAP_PROP_FPS)
        cap.release()
        print("Video FPS:", self.fps)

    def load_telemetry(self):
        plotter = PX4CSVPlotter(self.csv_path)
        all_data = plotter.plot_all(plot=False)

        angles = all_data["att"]

        # READ GPS DIRECTLY
        gps_path = os.path.join(self.csv_path, "sensor_gps_0.csv")
        gps = pd.read_csv(gps_path)

        gps_time = gps["timestamp_s"].to_numpy()
        gps_lon = gps["longitude_deg"].to_numpy()
        gps_lat = gps["latitude_deg"].to_numpy()
        gps_alt = gps["altitude_msl_m"].to_numpy()

        yaw_att = np.array(angles[0])
        pitch_att = np.array(angles[1])
        roll_att = np.array(angles[2])
        time_att = np.array(angles[3])

        roll_norm = (roll_att + 180) % 360 - 180
        pitch_norm = (pitch_att + 180) % 360 - 180
        yaw_norm = (yaw_att + 180) % 360 - 180

        M = len(yaw_norm)
        N = M

        idx = np.linspace(0, M - 1, N).astype(int)

        gps_time = np.arange(N)
        yaw_norm = yaw_norm[idx]
        pitch_norm = pitch_norm[idx]
        roll_norm = roll_norm[idx]

        old_len = len(gps_alt)

        x_old = np.linspace(0, 1, old_len)
        x_new = np.linspace(0, 1, N)

        gps_alt = np.interp(x_new, x_old, gps_alt)

        print(gps_alt.shape)
        print(gps_time.shape)
        print(yaw_norm.shape)
        print(pitch_norm.shape)
        print(roll_norm.shape)

        return gps_time, yaw_norm, pitch_norm, roll_norm, gps_alt

    def analyze_telemetry(self):
        gps_time, yaw_norm, pitch_norm, roll_norm, gps_alt = self.load_telemetry()

        frames = self.save_video_to_arrays()

        self.frames = frames[int(self.video_start_time * self.fps):int(self.video_end_time * self.fps)]

        gps_time_cut = gps_time[self.telemetry_start_idx:self.telemetry_end_idx]
        yaw_cut = yaw_norm[self.telemetry_start_idx:self.telemetry_end_idx]
        pitch_cut = pitch_norm[self.telemetry_start_idx:self.telemetry_end_idx]
        roll_cut = roll_norm[self.telemetry_start_idx:self.telemetry_end_idx]
        gps_alt_cut = gps_alt[self.telemetry_start_idx:self.telemetry_end_idx]

        m = len(self.frames)
        n = m

        idx = np.linspace(0, len(yaw_cut) - 1, n).astype(int)
        old_len = len(gps_alt_cut)
        x_old = np.linspace(0, 1, old_len)
        x_new = np.linspace(0, 1, n)

        self.gps_time = gps_time_cut
        self.gps_alt = np.interp(x_new, x_old, gps_alt_cut)
        self.gps_time = np.arange(n)
        self.yaw = yaw_cut[idx]
        self.pitch = pitch_cut[idx]
        self.roll = roll_cut[idx]

        print("gps_time_cut:", self.gps_time.shape)
        print("yaw_cut:", self.yaw.shape)
        print("pitch_cut:", self.pitch.shape)
        print("roll_cut:", self.roll.shape)
        print("gps_alt_cut:", self.gps_alt.shape)
        print("frames_cut:", self.frames.shape)

    ## Debug
    def debug_detect_motion_video(self, tresh_video=1, min_static_frames=2):
        self.read_fps()
        cap = cv2.VideoCapture(self.video_path)


        prev_frame = None
        motion = []

        frame_idx = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if prev_frame is None:
                prev_frame = gray
                frame_idx += 1
                continue

            # compute difference
            diff = cv2.absdiff(gray, prev_frame)
            mean_diff = diff.mean()
            motion.append(mean_diff)

            prev_frame = gray
            frame_idx += 1

        cap.release()

        motion = np.array(motion)

        # --- Detect start ---
        start_frame = np.argmax(motion > tresh_video)

        # --- Detect end ---
        # Find last index where motion exceeds threshold
        last_motion = np.where(motion > tresh_video)[0]
        end_frame = last_motion[-1] + min_static_frames if len(last_motion) else 0

        end_time = end_frame / self.fps
        start_time = start_frame / self.fps

        print("Start frame index:", start_frame)
        print("End frame index:", end_frame)
        print("Start time (s):", start_time)
        print("End time (s):", end_time)
        print("Duration (s):", end_time - start_time)

    def debug_select_window_all(self):
        # Load telemetry
        gps_time, yaw_norm, pitch_norm, roll_norm, gps_alt = self.load_telemetry()

        # Ensure all arrays are same length
        n = min(len(gps_time), len(yaw_norm), len(pitch_norm), len(roll_norm), len(gps_alt))
        gps_time = gps_time[:n]
        yaw_norm = yaw_norm[:n]
        pitch_norm = pitch_norm[:n]
        roll_norm = roll_norm[:n]
        gps_alt = gps_alt[:n]

        # Plot everything on the same time axis
        fig, ax = plt.subplots(figsize=(12, 6))

        ax.plot(gps_time, gps_alt, label="Altitude (m)")
        ax.plot(gps_time, pitch_norm, label="Pitch (deg)")
        ax.plot(gps_time, roll_norm, label="Roll (deg)")
        ax.plot(gps_time, yaw_norm, label="Yaw (deg)")

        ax.set_title("Telemetry (click 2 points to select time window)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Value")
        ax.grid(True)
        ax.legend()

        points = []

        def onclick(event):
            if event.inaxes != ax:
                return

            points.append(event.xdata)
            ax.axvline(event.xdata, color="red", linewidth=1.2)
            fig.canvas.draw()

            if len(points) == 2:
                t_start, t_end = points
                if t_start > t_end:
                    t_start, t_end = t_end, t_start

                idx_start = int(np.searchsorted(gps_time, t_start))
                idx_end = int(np.searchsorted(gps_time, t_end))

                print(f"START TIME: {t_start:.2f} s (idx {idx_start})")
                print(f"END TIME:   {t_end:.2f} s (idx {idx_end})")
                print(f"DURATION:   {t_end - t_start:.2f} s")

                plt.close(fig)

        fig.canvas.mpl_connect("button_press_event", onclick)
        plt.show()

    def play_telemetry_video(self, window_name="Telemetry Video"):

        # ---- safety check ----
        if self.frames is None:
            raise RuntimeError("Frames not loaded or not cut")

        n = min(
            len(self.frames),
            len(self.gps_time),
            len(self.gps_alt),
            len(self.pitch),
            len(self.roll),
        )

        t = 1.0 / self.fps

        for i in range(n):
            frame = self.frames[i].copy()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            text = (
                f"Idx: {i} | "
                f"Time: {self.gps_time[i]:.2f}s | "
                f"Alt: {self.gps_alt[i]:.2f}m | "
                f"Pitch: {self.pitch[i]:.2f} | "
                f"Roll: {self.roll[i]:.2f}"
            )

            cv2.putText(
                frame_bgr,
                text,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            cv2.imshow(window_name, frame_bgr)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            cv2.waitKey(int(t * 1000))

        cv2.destroyAllWindows()
