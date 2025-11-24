#!/usr/bin/env python3
"""
Complete PX4 Flight Log Analyzer
Analyzes both fused and raw sensor data from .ulg log files
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog
from pathlib import Path


def load_ulog(log_path):
    """Load a ULog file and return the ULog object"""
    print(f"Loading log file: {log_path}")
    ulog = ULog(log_path)
    print(f"✓ Log loaded successfully")
    print(f"  Duration: {ulog.last_timestamp / 1e6:.2f} seconds")
    return ulog


def list_available_topics(ulog):
    """Print all available data topics in the log"""
    print("\n=== Available Topics ===")
    for idx, dataset in enumerate(ulog.data_list):
        print(f"{idx}: {dataset.name} ({len(dataset.data['timestamp'])} samples)")
    return [d.name for d in ulog.data_list]


def get_data_as_dataframe(ulog, topic_name):
    """Extract a specific topic as a pandas DataFrame"""
    for dataset in ulog.data_list:
        if dataset.name == topic_name:
            data_dict = {}
            data_dict['time'] = dataset.data['timestamp'] / 1e6
            for field in dataset.data.keys():
                if field != 'timestamp':
                    data_dict[field] = dataset.data[field]
            df = pd.DataFrame(data_dict)
            print(f"\n✓ Extracted {topic_name}: {len(df)} samples")
            return df
    print(f"✗ Topic '{topic_name}' not found")
    return None


# ============================================================================
# FUSED DATA ANALYSIS (EKF2 - Combined Sensors)
# ============================================================================

def analyze_position(ulog, output_dir='.'):
    """Analyze and plot position data (FUSED)"""
    print("\n=== Analyzing Position Data (Fused) ===")

    pos_df = get_data_as_dataframe(ulog, 'vehicle_local_position')
    if pos_df is None:
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Position Analysis (Fused Data - EKF2)', fontsize=16)

    axes[0, 0].plot(pos_df['y'], pos_df['x'], 'b-', linewidth=1.5)
    axes[0, 0].plot(pos_df['y'].iloc[0], pos_df['x'].iloc[0], 'go', markersize=10, label='Start')
    axes[0, 0].plot(pos_df['y'].iloc[-1], pos_df['x'].iloc[-1], 'ro', markersize=10, label='End')
    axes[0, 0].set_xlabel('East (m)')
    axes[0, 0].set_ylabel('North (m)')
    axes[0, 0].set_title('2D Flight Path (Top View)')
    axes[0, 0].grid(True)
    axes[0, 0].legend()
    axes[0, 0].axis('equal')

    axes[0, 1].plot(pos_df['time'], -pos_df['z'], 'r-', linewidth=1.5)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Altitude (m)')
    axes[0, 1].set_title('Altitude vs Time')
    axes[0, 1].grid(True)

    axes[1, 0].plot(pos_df['time'], pos_df['vx'], label='Vx (North)', linewidth=1.5)
    axes[1, 0].plot(pos_df['time'], pos_df['vy'], label='Vy (East)', linewidth=1.5)
    axes[1, 0].plot(pos_df['time'], pos_df['vz'], label='Vz (Down)', linewidth=1.5)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Velocity (m/s)')
    axes[1, 0].set_title('Velocity Components')
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    speed = np.sqrt(pos_df['vx'] ** 2 + pos_df['vy'] ** 2 + pos_df['vz'] ** 2)
    axes[1, 1].plot(pos_df['time'], speed, 'g-', linewidth=1.5)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Speed (m/s)')
    axes[1, 1].set_title('Total Speed')
    axes[1, 1].grid(True)

    plt.tight_layout()
    output_file = Path(output_dir) / 'position_analysis.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✓ Position plot saved as '{output_file}'")

    print(f"\nPosition Statistics:")
    print(f"  Max altitude: {-pos_df['z'].min():.2f} m")
    print(f"  Max speed: {speed.max():.2f} m/s")
    print(f"  Total distance: {np.sum(np.sqrt(np.diff(pos_df['x']) ** 2 + np.diff(pos_df['y']) ** 2)):.2f} m")


def analyze_attitude(ulog, output_dir='.'):
    """Analyze and plot attitude (orientation) data (FUSED)"""
    print("\n=== Analyzing Attitude Data (Fused) ===")

    att_df = get_data_as_dataframe(ulog, 'vehicle_attitude')
    if att_df is None:
        return

    roll = np.arctan2(2 * (att_df['q[0]'] * att_df['q[1]'] + att_df['q[2]'] * att_df['q[3]']),
                      1 - 2 * (att_df['q[1]'] ** 2 + att_df['q[2]'] ** 2))
    pitch = np.arcsin(2 * (att_df['q[0]'] * att_df['q[2]'] - att_df['q[3]'] * att_df['q[1]']))
    yaw = np.arctan2(2 * (att_df['q[0]'] * att_df['q[3]'] + att_df['q[1]'] * att_df['q[2]']),
                     1 - 2 * (att_df['q[2]'] ** 2 + att_df['q[3]'] ** 2))

    roll_deg = np.degrees(roll)
    pitch_deg = np.degrees(pitch)
    yaw_deg = np.degrees(yaw)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Attitude Analysis (Fused Data)', fontsize=16)

    axes[0].plot(att_df['time'], roll_deg, 'r-', linewidth=1.5)
    axes[0].set_ylabel('Roll (deg)')
    axes[0].set_title('Roll Angle')
    axes[0].grid(True)

    axes[1].plot(att_df['time'], pitch_deg, 'g-', linewidth=1.5)
    axes[1].set_ylabel('Pitch (deg)')
    axes[1].set_title('Pitch Angle')
    axes[1].grid(True)

    axes[2].plot(att_df['time'], yaw_deg, 'b-', linewidth=1.5)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Yaw (deg)')
    axes[2].set_title('Yaw Angle (Heading)')
    axes[2].grid(True)

    plt.tight_layout()
    output_file = Path(output_dir) / 'attitude_analysis.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✓ Attitude plot saved as '{output_file}'")

    print(f"\nAttitude Statistics:")
    print(f"  Max roll: {np.abs(roll_deg).max():.2f}°")
    print(f"  Max pitch: {np.abs(pitch_deg).max():.2f}°")


def analyze_gps(ulog, output_dir='.'):
    """Analyze GPS data"""
    print("\n=== Analyzing GPS Data ===")

    gps_df = get_data_as_dataframe(ulog, 'vehicle_gps_position')
    if gps_df is None:
        return

    print(f"Available GPS columns: {list(gps_df.columns)}")

    lat_field = None
    lon_field = None
    alt_field = None

    for col in gps_df.columns:
        if 'lat' in col.lower() and 'deg' in col.lower():
            lat_field = col
        elif 'lon' in col.lower() and 'deg' in col.lower():
            lon_field = col
        elif 'alt' in col.lower() and ('msl' in col.lower() or 'ellipsoid' in col.lower()):
            alt_field = col

    if lat_field is None:
        lat_field = 'latitude_deg' if 'latitude_deg' in gps_df.columns else 'lat'
    if lon_field is None:
        lon_field = 'longitude_deg' if 'longitude_deg' in gps_df.columns else 'lon'
    if alt_field is None:
        alt_field = 'altitude_msl_m' if 'altitude_msl_m' in gps_df.columns else 'alt'

    try:
        if lat_field in gps_df.columns:
            if 'deg' in lat_field:
                lat = gps_df[lat_field]
                lon = gps_df[lon_field]
            else:
                lat = gps_df[lat_field] / 1e7
                lon = gps_df[lon_field] / 1e7
        else:
            print("✗ Could not find latitude field in GPS data")
            return

        if alt_field in gps_df.columns:
            if '_m' in alt_field:
                alt = gps_df[alt_field]
            else:
                alt = gps_df[alt_field] / 1000
        else:
            print("Warning: No altitude field found")
            alt = pd.Series([0] * len(lat))
    except KeyError as e:
        print(f"✗ Error accessing GPS field: {e}")
        print(f"Available columns: {list(gps_df.columns)}")
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('GPS Analysis', fontsize=16)

    axes[0, 0].plot(lon, lat, 'b-', linewidth=1.5)
    axes[0, 0].plot(lon.iloc[0], lat.iloc[0], 'go', markersize=10, label='Start')
    axes[0, 0].plot(lon.iloc[-1], lat.iloc[-1], 'ro', markersize=10, label='End')
    axes[0, 0].set_xlabel('Longitude')
    axes[0, 0].set_ylabel('Latitude')
    axes[0, 0].set_title('GPS Track')
    axes[0, 0].grid(True)
    axes[0, 0].legend()

    axes[0, 1].plot(gps_df['time'], alt, 'r-', linewidth=1.5)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('GPS Altitude (m)')
    axes[0, 1].set_title('GPS Altitude')
    axes[0, 1].grid(True)

    sat_field = 'satellites_used' if 'satellites_used' in gps_df.columns else None
    if sat_field:
        axes[1, 0].plot(gps_df['time'], gps_df[sat_field], 'g-', linewidth=1.5)
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Satellites')
        axes[1, 0].set_title('GPS Satellites Used')
        axes[1, 0].grid(True)
    else:
        axes[1, 0].text(0.5, 0.5, 'Satellite data not available',
                        ha='center', va='center', transform=axes[1, 0].transAxes)
        axes[1, 0].set_title('GPS Satellites (N/A)')

    vel_field = 'vel_m_s' if 'vel_m_s' in gps_df.columns else 'vel_ned_valid' if 'vel_ned_valid' in gps_df.columns else None
    if vel_field and 'vel_m_s' in vel_field:
        axes[1, 1].plot(gps_df['time'], gps_df[vel_field], 'b-', linewidth=1.5)
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Speed (m/s)')
        axes[1, 1].set_title('GPS Ground Speed')
        axes[1, 1].grid(True)
    else:
        axes[1, 1].text(0.5, 0.5, 'Velocity data not available',
                        ha='center', va='center', transform=axes[1, 1].transAxes)
        axes[1, 1].set_title('GPS Speed (N/A)')

    plt.tight_layout()
    output_file = Path(output_dir) / 'gps_analysis.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✓ GPS plot saved as '{output_file}'")

    print(f"\nGPS Statistics:")
    print(f"  Start: {lat.iloc[0]:.6f}°N, {lon.iloc[0]:.6f}°E")
    print(f"  End: {lat.iloc[-1]:.6f}°N, {lon.iloc[-1]:.6f}°E")
    if sat_field and sat_field in gps_df.columns:
        print(f"  Avg satellites: {gps_df[sat_field].mean():.1f}")


# ============================================================================
# RAW SENSOR DATA ANALYSIS
# ============================================================================

def analyze_accelerometer(ulog, output_dir='.'):
    """Analyze raw accelerometer data"""
    print("\n=== Analyzing Raw Accelerometer Data ===")

    accel_df = get_data_as_dataframe(ulog, 'sensor_accel')
    if accel_df is None:
        return

    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Raw Accelerometer Data', fontsize=16)

    axes[0].plot(accel_df['time'], accel_df['x'], 'r-', linewidth=0.5, alpha=0.7)
    axes[0].set_ylabel('Accel X (m/s²)')
    axes[0].set_title('X-axis Acceleration (Forward/Backward)')
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='--', linewidth=0.5)

    axes[1].plot(accel_df['time'], accel_df['y'], 'g-', linewidth=0.5, alpha=0.7)
    axes[1].set_ylabel('Accel Y (m/s²)')
    axes[1].set_title('Y-axis Acceleration (Left/Right)')
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(y=0, color='k', linestyle='--', linewidth=0.5)

    axes[2].plot(accel_df['time'], accel_df['z'], 'b-', linewidth=0.5, alpha=0.7)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Accel Z (m/s²)')
    axes[2].set_title('Z-axis Acceleration (Up/Down)')
    axes[2].grid(True, alpha=0.3)
    axes[2].axhline(y=-9.81, color='r', linestyle='--', linewidth=1, label='Gravity')
    axes[2].legend()

    plt.tight_layout()
    output_file = Path(output_dir) / 'raw_accelerometer.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✓ Accelerometer plot saved as '{output_file}'")

    total_accel = np.sqrt(accel_df['x'] ** 2 + accel_df['y'] ** 2 + accel_df['z'] ** 2)
    print(f"\nAccelerometer Statistics:")
    print(f"  Sample rate: ~{len(accel_df) / (accel_df['time'].max() - accel_df['time'].min()):.1f} Hz")
    print(f"  Max total acceleration: {total_accel.max():.2f} m/s²")


def analyze_gyroscope(ulog, output_dir='.'):
    """Analyze raw gyroscope data"""
    print("\n=== Analyzing Raw Gyroscope Data ===")

    gyro_df = get_data_as_dataframe(ulog, 'sensor_gyro')
    if gyro_df is None:
        return

    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Raw Gyroscope Data (Angular Velocity)', fontsize=16)

    axes[0].plot(gyro_df['time'], np.degrees(gyro_df['x']), 'r-', linewidth=0.5, alpha=0.7)
    axes[0].set_ylabel('Roll Rate (°/s)')
    axes[0].set_title('X-axis Rotation Rate (Roll)')
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='--', linewidth=0.5)

    axes[1].plot(gyro_df['time'], np.degrees(gyro_df['y']), 'g-', linewidth=0.5, alpha=0.7)
    axes[1].set_ylabel('Pitch Rate (°/s)')
    axes[1].set_title('Y-axis Rotation Rate (Pitch)')
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(y=0, color='k', linestyle='--', linewidth=0.5)

    axes[2].plot(gyro_df['time'], np.degrees(gyro_df['z']), 'b-', linewidth=0.5, alpha=0.7)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Yaw Rate (°/s)')
    axes[2].set_title('Z-axis Rotation Rate (Yaw)')
    axes[2].grid(True, alpha=0.3)
    axes[2].axhline(y=0, color='k', linestyle='--', linewidth=0.5)

    plt.tight_layout()
    output_file = Path(output_dir) / 'raw_gyroscope.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✓ Gyroscope plot saved as '{output_file}'")

    print(f"\nGyroscope Statistics:")
    print(f"  Sample rate: ~{len(gyro_df) / (gyro_df['time'].max() - gyro_df['time'].min()):.1f} Hz")
    print(f"  Max roll rate: {np.degrees(np.abs(gyro_df['x']).max()):.2f} °/s")
    print(f"  Max pitch rate: {np.degrees(np.abs(gyro_df['y']).max()):.2f} °/s")
    print(f"  Max yaw rate: {np.degrees(np.abs(gyro_df['z']).max()):.2f} °/s")


def analyze_magnetometer(ulog, output_dir='.'):
    """Analyze raw magnetometer data"""
    print("\n=== Analyzing Raw Magnetometer Data ===")

    mag_df = get_data_as_dataframe(ulog, 'sensor_mag')
    if mag_df is None:
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Raw Magnetometer Data (Magnetic Field)', fontsize=16)

    axes[0, 0].plot(mag_df['time'], mag_df['x'], 'r-', linewidth=1, label='X')
    axes[0, 0].plot(mag_df['time'], mag_df['y'], 'g-', linewidth=1, label='Y')
    axes[0, 0].plot(mag_df['time'], mag_df['z'], 'b-', linewidth=1, label='Z')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Magnetic Field (Gauss)')
    axes[0, 0].set_title('Magnetic Field Components')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    total_field = np.sqrt(mag_df['x'] ** 2 + mag_df['y'] ** 2 + mag_df['z'] ** 2)
    axes[0, 1].plot(mag_df['time'], total_field, 'purple', linewidth=1)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Total Field (Gauss)')
    axes[0, 1].set_title('Total Magnetic Field Strength')
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(mag_df['x'], mag_df['y'], 'b-', linewidth=0.5, alpha=0.5)
    axes[1, 0].scatter(mag_df['x'].iloc[0], mag_df['y'].iloc[0], c='green', s=100, marker='o', label='Start')
    axes[1, 0].scatter(mag_df['x'].iloc[-1], mag_df['y'].iloc[-1], c='red', s=100, marker='x', label='End')
    axes[1, 0].set_xlabel('Magnetic X (Gauss)')
    axes[1, 0].set_ylabel('Magnetic Y (Gauss)')
    axes[1, 0].set_title('Magnetic Field XY (Compass View)')
    axes[1, 0].axis('equal')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()

    heading = np.degrees(np.arctan2(mag_df['y'], mag_df['x']))
    axes[1, 1].plot(mag_df['time'], heading, 'orange', linewidth=1)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Heading (°)')
    axes[1, 1].set_title('Magnetic Heading')
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    output_file = Path(output_dir) / 'raw_magnetometer.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✓ Magnetometer plot saved as '{output_file}'")

    print(f"\nMagnetometer Statistics:")
    print(f"  Sample rate: ~{len(mag_df) / (mag_df['time'].max() - mag_df['time'].min()):.1f} Hz")
    print(f"  Avg field strength: {total_field.mean():.4f} Gauss")


def analyze_barometer(ulog, output_dir='.'):
    """Analyze raw barometer data"""
    print("\n=== Analyzing Raw Barometer Data ===")

    baro_df = get_data_as_dataframe(ulog, 'sensor_baro')
    if baro_df is None:
        return

    pressure_field = None
    temp_field = None

    for col in baro_df.columns:
        if 'pressure' in col.lower() and pressure_field is None:
            pressure_field = col
        if 'temp' in col.lower() and temp_field is None:
            temp_field = col

    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle('Raw Barometer Data', fontsize=16)

    if pressure_field and pressure_field in baro_df.columns:
        axes[0].plot(baro_df['time'], baro_df[pressure_field], 'b-', linewidth=1)
        axes[0].set_ylabel('Pressure (mbar/hPa)')
        axes[0].set_title('Atmospheric Pressure')
        axes[0].grid(True, alpha=0.3)
    else:
        axes[0].text(0.5, 0.5, 'Pressure data not available',
                     ha='center', va='center', transform=axes[0].transAxes)

    if temp_field and temp_field in baro_df.columns:
        axes[1].plot(baro_df['time'], baro_df[temp_field], 'r-', linewidth=1)
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Temperature (°C)')
        axes[1].set_title('Air Temperature')
        axes[1].grid(True, alpha=0.3)
    else:
        axes[1].text(0.5, 0.5, 'Temperature data not available',
                     ha='center', va='center', transform=axes[1].transAxes)

    plt.tight_layout()
    output_file = Path(output_dir) / 'raw_barometer.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✓ Barometer plot saved as '{output_file}'")

    if pressure_field and pressure_field in baro_df.columns:
        print(f"\nBarometer Statistics:")
        print(f"  Sample rate: ~{len(baro_df) / (baro_df['time'].max() - baro_df['time'].min()):.1f} Hz")
        print(f"  Avg pressure: {baro_df[pressure_field].mean():.2f} mbar")


# ============================================================================
# CSV EXPORT
# ============================================================================

def export_to_csv(ulog, topic_name, output_file):
    """Export a specific topic to CSV"""
    df = get_data_as_dataframe(ulog, topic_name)
    if df is not None:
        output_path = Path(output_file)
        df.to_csv(output_path, index=False)
        print(f"✓ Exported {topic_name} to {output_path}")


def export_all_data(ulog, output_dir):
    """Export all data to CSV files"""
    print("\n=== Exporting All Data to CSV ===")

    # Fused data
    export_to_csv(ulog, 'vehicle_local_position', output_dir / 'position_data.csv')
    export_to_csv(ulog, 'vehicle_attitude', output_dir / 'attitude_data.csv')
    export_to_csv(ulog, 'vehicle_gps_position', output_dir / 'gps_data.csv')

    # Raw sensor data
    export_to_csv(ulog, 'sensor_accel', output_dir / 'raw_accelerometer.csv')
    export_to_csv(ulog, 'sensor_gyro', output_dir / 'raw_gyroscope.csv')
    export_to_csv(ulog, 'sensor_mag', output_dir / 'raw_magnetometer.csv')
    export_to_csv(ulog, 'sensor_baro', output_dir / 'raw_barometer.csv')


# ============================================================================
# MAIN
# ============================================================================

def main():
    # ---------------------------------------------------------------------
    #                                LOG FILE PATH:
    log_file = "logs/log_6.ulg"
    # ---------------------------------------------------------------------

    if not Path(log_file).exists():
        print(f"Error: Log file not found: {log_file}")
        sys.exit(1)

    # Create output directory
    log_path = Path(log_file)
    log_name = log_path.stem
    output_dir = Path(f"{log_name}_complete_analysis")
    output_dir.mkdir(exist_ok=True)
    print(f"\n📁 Output directory: {output_dir.absolute()}")

    # Load log
    ulog = load_ulog(log_file)

    # List available topics
    topics = list_available_topics(ulog)

    # Perform all analyses
    print("\n" + "=" * 60)
    print("Starting Complete Analysis...")
    print("=" * 60)

    print("\n📊 FUSED DATA ANALYSIS (EKF2 - Combined Sensors)")
    print("-" * 60)
    analyze_position(ulog, output_dir)
    analyze_attitude(ulog, output_dir)
    analyze_gps(ulog, output_dir)

    print("\n🔬 RAW SENSOR DATA ANALYSIS")
    print("-" * 60)
    analyze_accelerometer(ulog, output_dir)
    analyze_gyroscope(ulog, output_dir)
    analyze_magnetometer(ulog, output_dir)
    analyze_barometer(ulog, output_dir)

    # Export all data
    export_all_data(ulog, output_dir)

    print("\n" + "=" * 60)
    print("Complete Analysis Finished!")
    print("=" * 60)
    print(f"\n📁 All files saved in: {output_dir.absolute()}")
    print("\n📊 Generated Plots:")
    print("  Fused Data:")
    print("    - position_analysis.png")
    print("    - attitude_analysis.png")
    print("    - GPS_analysis.png")
    print("  📊 Plots:")
    print("    - raw_accelerometer.png")
    print("    - raw_gyroscope.png")
    print("    - raw_magnetometer.png")
    print("    - raw_barometer.png")
    print("  📄 CSV Data:")
    print("    - raw_accelerometer.csv")
    print("    - raw_gyroscope.csv")
    print("    - raw_magnetometer.csv")
    print("    - raw_barometer.csv")

    plt.show()

if __name__ == "__main__":
    main()