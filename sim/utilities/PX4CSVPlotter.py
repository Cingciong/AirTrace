import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import matplotlib as mpl

mpl.rcParams.update({
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "savefig.facecolor": "white",
    "axes.edgecolor": "black",
    "axes.labelcolor": "black",
    "xtick.color": "black",
    "ytick.color": "black",
    "text.color": "black",
    "grid.color": "#d3d3d3",
    "grid.linestyle": "--",
    "grid.linewidth": 0.5,
    "axes.prop_cycle": mpl.cycler(
        color=[
            "#377eb8",  # blue
            "#4daf4a",  # green
            "#984ea3",  # purple
            "#ff7f00",  # orange
            "#ffff33",  # yellow
            "#a65628",  # brown
            "#f781bf",  # pink
            "#00ffff",  # cyan
            "#ff00ff",  # magenta
        ]
    )
})

class PX4CSVPlotter:
    MONITOR_CSV = [
        "sensor_accel_0.csv",
        "sensor_gyro_0.csv",
        "sensor_mag_0.csv",
        "sensor_baro_0.csv",
        "sensor_gps_0.csv",
        "vehicle_local_position_0.csv",
        "vehicle_global_position_0.csv"
    ]

    def __init__(self, csv_dir: str):
        self.csv_dir = Path(csv_dir)

    def _load_csv(self, filename):
        path = self.csv_dir / filename
        if not path.exists():
            raise FileNotFoundError(f"CSV not found: {path}")
        df = pd.read_csv(path)

        if "time" not in df.columns:
            if "timestamp" in df.columns:
                df["time"] = df["timestamp"] / 1e6
            else:
                df["time"] = np.arange(len(df))

        return df

    def plot_accelerometer(self, plot=True):
        df = self._load_csv("sensor_accel_0.csv")
        time = df["time"].to_numpy()
        x = df["x"].to_numpy()
        y = df["y"].to_numpy()
        z = df["z"].to_numpy()

        if plot:
            fig, axes = plt.subplots(3, 1, figsize=(14, 10))
            fig.suptitle('Raw Accelerometer Data', fontsize=16)

            axes[0].plot(time, x, linewidth=0.5, alpha=0.7)
            axes[0].set_ylabel('Accel X (m/s²)')
            axes[0].set_title('X-axis Acceleration')
            axes[0].grid(True)

            axes[1].plot(time, y, linewidth=0.5, alpha=0.7)
            axes[1].set_ylabel('Accel Y (m/s²)')
            axes[1].set_title('Y-axis Acceleration')
            axes[1].grid(True)

            axes[2].plot(time, z, linewidth=0.5, alpha=0.7)
            axes[2].set_xlabel('Time (s)')
            axes[2].set_ylabel('Accel Z (m/s²)')
            axes[2].set_title('Z-axis Acceleration')
            axes[2].grid(True)

            plt.tight_layout()
            plt.show()

        return time, x, y, z

    def plot_gyroscope(self, plot=True):
        df = self._load_csv("sensor_gyro_0.csv")
        time = df["time"].to_numpy()
        x = np.degrees(df["x"].to_numpy())
        y = np.degrees(df["y"].to_numpy())
        z = np.degrees(df["z"].to_numpy())

        if plot:
            fig, axes = plt.subplots(3, 1, figsize=(14, 10))
            fig.suptitle('Raw Gyroscope Data', fontsize=16)

            axes[0].plot(time, x, linewidth=0.5, alpha=0.7)
            axes[0].set_ylabel('Roll Rate (°/s)')
            axes[0].set_title('X-axis Rotation Rate')
            axes[0].grid(True)

            axes[1].plot(time, y, linewidth=0.5, alpha=0.7)
            axes[1].set_ylabel('Pitch Rate (°/s)')
            axes[1].set_title('Y-axis Rotation Rate')
            axes[1].grid(True)

            axes[2].plot(time, z, linewidth=0.5, alpha=0.7)
            axes[2].set_xlabel('Time (s)')
            axes[2].set_ylabel('Yaw Rate (°/s)')
            axes[2].set_title('Z-axis Rotation Rate')
            axes[2].grid(True)

            plt.tight_layout()
            plt.show()

        return time, x, y, z

    def plot_magnetometer(self, plot=True):
        df = self._load_csv("sensor_mag_0.csv")
        time = df["time"].to_numpy()
        x = df["x"].to_numpy()
        y = df["y"].to_numpy()
        z = df["z"].to_numpy()
        total_field = np.sqrt(x**2 + y**2 + z**2)
        heading = np.degrees(np.arctan2(y, x))

        if plot:
            fig, axes = plt.subplots(2, 2, figsize=(14, 10))
            fig.suptitle('Raw Magnetometer Data', fontsize=16)

            axes[0, 0].plot(time, x, linewidth=1, label='X')
            axes[0, 0].plot(time, y, linewidth=1, label='Y')
            axes[0, 0].plot(time, z, linewidth=1, label='Z')
            axes[0, 0].set_title('Magnetic Field Components')
            axes[0, 0].legend()
            axes[0, 0].grid(True)

            axes[0, 1].plot(time, total_field, linewidth=1)
            axes[0, 1].set_title('Total Magnetic Field Strength')
            axes[0, 1].grid(True)

            axes[1, 0].plot(x, y, linewidth=0.5, alpha=0.5)
            axes[1, 0].set_title('Magnetic Field XY')
            axes[1, 0].grid(True)

            axes[1, 1].plot(time, heading, linewidth=1)
            axes[1, 1].set_title('Magnetic Heading')
            axes[1, 1].grid(True)

            plt.tight_layout()
            plt.show()

        return time, x, y, z, total_field, heading

    def plot_baro(self, plot=True):
        df = self._load_csv("sensor_baro_0.csv")
        time = df["time"].to_numpy()
        pressure = df["pressure"].to_numpy()
        temperature = df["temperature"].to_numpy()

        if plot:
            fig, axes = plt.subplots(2, 1, figsize=(14, 8))
            fig.suptitle('Raw Barometer Data', fontsize=16)

            axes[0].plot(time, pressure, linewidth=1)
            axes[0].set_title('Pressure')
            axes[0].grid(True)

            axes[1].plot(time, temperature, linewidth=1)
            axes[1].set_title('Temperature')
            axes[1].grid(True)

            plt.tight_layout()
            plt.show()

        return time, pressure, temperature

    def plot_gps(self, plot=True):
        df = self._load_csv("sensor_gps_0.csv")

        time = df["time"].to_numpy()
        lon = df["longitude_deg"].to_numpy()
        lat = df["latitude_deg"].to_numpy()
        alt = df["altitude_msl_m"].to_numpy()

        # Convert GPS to meters (local frame)
        R = 6378137.0
        lat_rad = np.deg2rad(lat)
        lon_rad = np.deg2rad(lon)
        lat0 = lat_rad[0]
        lon0 = lon_rad[0]

        x = (lon_rad - lon0) * R * np.cos(lat0)
        y = (lat_rad - lat0) * R

        # ONLY plot when plot=True
        if plot:
            plt.figure(figsize=(6, 6))
            plt.plot(x, y)
            plt.title("GPS Path (meters)")
            plt.xlabel("East (m)")
            plt.ylabel("North (m)")
            plt.grid(True)
            plt.show()

            plt.figure(figsize=(10, 4))
            plt.plot(time, alt)
            plt.title("Altitude over time")
            plt.xlabel("Time (s)")
            plt.ylabel("Altitude (m)")
            plt.grid(True)
            plt.show()

            fig, axes = plt.subplots(2, 1, figsize=(14, 8))
            fig.suptitle('GPS Data', fontsize=16)

            axes[0].plot(lon, lat, linewidth=1)
            axes[0].set_title('GPS Track')
            axes[0].grid(True)

            axes[1].plot(time, alt, linewidth=1)
            axes[1].set_title('GPS Altitude')
            axes[1].grid(True)

            plt.tight_layout()
            plt.show()

        # RETURN GPS data ALWAYS
        return time, lon, lat, alt

    @staticmethod
    def quat_to_euler(q0, q1, q2, q3):
        roll = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
        pitch = np.arcsin(2*(q0*q2 - q3*q1))
        yaw = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    def plot_attitude_angles(self, plot=True):
        df = self._load_csv("vehicle_attitude_0.csv")
        time = df["time"].to_numpy()
        roll, pitch, yaw = self.quat_to_euler(df['q[0]'], df['q[1]'], df['q[2]'], df['q[3]'])

        if plot:
            fig, axes = plt.subplots(3, 1, figsize=(14, 10))
            fig.suptitle("Attitude Angles (Relative to Ground)", fontsize=16)

            axes[0].plot(time, roll, linewidth=1)
            axes[0].set_title("Roll (deg)")
            axes[0].grid(True)

            axes[1].plot(time, pitch, linewidth=1)
            axes[1].set_title("Pitch (deg)")
            axes[1].grid(True)

            axes[2].plot(time, yaw, linewidth=1)
            axes[2].set_title("Yaw (deg)")
            axes[2].grid(True)

            plt.tight_layout()
            plt.show()

        return time, roll, pitch, yaw

    def plot_all(self, plot=False):
        acc = self.plot_accelerometer(plot=plot)
        gyro = self.plot_gyroscope(plot=plot)
        mag = self.plot_magnetometer(plot=plot)
        baro = self.plot_baro(plot=plot)
        gps = self.plot_gps(plot=plot)
        att = self.plot_attitude_angles(plot=plot)

        return {
            "acc": acc,
            "gyro": gyro,
            "mag": mag,
            "baro": baro,
            "gps": gps,
            "att": att
        }
