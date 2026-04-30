# PX4CSVPlotter.py

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

class PX4CSVPlotter:

    def __init__(self, csv_dir: str):
        self.csv_dir = Path(csv_dir)

    def _load_csv(self, filename):
        path = self.csv_dir / filename
        df = pd.read_csv(path)

        if "time" not in df.columns:
            if "timestamp" in df.columns:
                df["time"] = df["timestamp"] / 1e6
            else:
                df["time"] = np.arange(len(df))

        return df

    def plot_gps(self, plot=True):
        df = self._load_csv("sensor_gps_0.csv")

        time = df["time"].to_numpy()
        lon = df["longitude_deg"].to_numpy()
        lat = df["latitude_deg"].to_numpy()
        alt = df["altitude_msl_m"].to_numpy()

        R = 6378137.0
        lat_rad = np.deg2rad(lat)
        lon_rad = np.deg2rad(lon)
        lat0 = lat_rad[0]
        lon0 = lon_rad[0]

        x = (lon_rad - lon0) * R * np.cos(lat0)
        y = (lat_rad - lat0) * R

        if plot:
            plt.figure()
            plt.plot(x, y)
            plt.grid()
            plt.show()

        return time, lon, lat, alt

    # ADDED -----------------------------------------------------------------------
    @staticmethod
    def deg_to_dms(deg):
        d = int(deg)
        m = int((deg - d) * 60)
        s = (deg - d - m / 60) * 3600
        return f"{d}°{m}'{s:.2f}\""
    # -----------------------------------------------------------------------------

    @staticmethod
    def format_dms_axis(ax):
        from matplotlib.ticker import FuncFormatter
        ax.xaxis.set_major_formatter(FuncFormatter(lambda x, _: PX4CSVPlotter.deg_to_dms(x)))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda y, _: PX4CSVPlotter.deg_to_dms(y)))
        plt.xticks(rotation=30, ha='right')

    @staticmethod
    def quat_to_euler(q0, q1, q2, q3):
        roll = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
        pitch = np.arcsin(2*(q0*q2 - q3*q1))
        yaw = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    def plot_all(self, plot=True):
        time_att, roll, pitch, yaw = self.plot_attitude_angles(plot=plot)

        return {
            "att": (yaw, pitch, roll, time_att),
        }

    def plot_attitude_angles(self, plot=True):
        df = self._load_csv("vehicle_attitude_0.csv")
        time = df["time"].to_numpy()
        roll, pitch, yaw = self.quat_to_euler(df['q[0]'], df['q[1]'], df['q[2]'], df['q[3]'])
        return time, roll, pitch, yaw

    def plot_comparison(self, lat_true, lon_true, lat_est, lon_est):
        R = 6378137.0

        lat0 = np.deg2rad(lat_true[0])
        lon0 = np.deg2rad(lon_true[0])

        def to_xy(lat, lon):
            lat_r = np.deg2rad(lat)
            lon_r = np.deg2rad(lon)
            x = (lon_r - lon0) * R * np.cos(lat0)
            y = (lat_r - lat0) * R
            return x, y

        x_true, y_true = to_xy(lat_true, lon_true)
        x_est, y_est = to_xy(lat_est, lon_est)

        plt.figure()
        plt.plot(x_true, y_true, label="GPS")
        plt.plot(x_est, y_est, label="Estimated")
        plt.legend()
        plt.grid()
        plt.show()