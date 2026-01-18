#%%
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

csv_file = "G:/projekt gropwy/22.10.2025/csv/5.2.csv"
df = pd.read_csv(csv_file, sep=";", decimal=",", header=0)


df['time'] = pd.to_numeric(df['time'], errors='coerce')
for col in ['ax','ay','az','wx','wy','wz','Bx','By','Bz','Azimuth','Pitch','Roll',
            'Latitude','Longitude','Speed (m/s)','Altitude (m)']:
    df[col] = pd.to_numeric(df[col], errors='coerce')

hysteresis = 0.04  # m/s^2

# Apply threshold to each acceleration axis
for col in ['ax', 'ay', 'az']:
    df[col + '_filtered'] = df[col].where(df[col].abs() >= hysteresis, 0)

# Pomijamy pierwsze 10 wierszy
df = df[1:].copy()
df = df[::10].copy()

# Wybieramy punkt odniesienia (np. pierwszy punkt)
lat0 = df['Latitude'].iloc[0]
lon0 = df['Longitude'].iloc[0]

# Konwersja stopni na metry
df['x'] = (df['Longitude'] - lon0) * 111320 * np.cos(np.radians(lat0))
df['y'] = (df['Latitude'] - lat0) * 111320

# Rysowanie trajektorii w metrach
plt.figure(figsize=(10,8))
plt.plot(df['x'], df['y'], marker='o', linestyle='-', markersize=3)
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Trajektoria GPS (metry)')
plt.grid(True)
plt.axis('equal')  # zachowanie proporcji w metrach
plt.show()

len(df)

#%%


plt.figure(figsize=(12,6))
plt.plot(df['time'], df['Azimuth'], label='Azimuth', color='tab:blue')
plt.xlabel('Time [s]')
plt.ylabel('Azimuth [°]')
plt.title('Azimuth over Time')
plt.grid(True)
plt.legend()
plt.show()


#%%


# --- Clean column names just in case ---
df.columns = df.columns.str.strip()

# --- Convert to radians and unwrap for smoothness ---
azimuth_rad = np.deg2rad(df['Azimuth'])
azimuth_unwrapped = np.unwrap(azimuth_rad)

# --- Convert from navigation (0°=North, CW positive) to math convention (0°=East, CCW positive) ---
theta = np.deg2rad(90) - azimuth_unwrapped

# --- Compute sin and cos ---
df['cos_az'] = np.cos(theta)
df['sin_az'] = np.sin(theta)

# --- Plot both over time ---
plt.figure(figsize=(12,6))
plt.plot(df['time'], df['cos_az'], label='cos(Azimuth)', color='tab:blue')
plt.plot(df['time'], df['sin_az'], label='sin(Azimuth)', color='tab:orange')
plt.xlabel('Time [s]')
plt.ylabel('Value')
plt.title('Cosine and Sine of Azimuth over Time')
plt.legend()
plt.grid(True)
plt.show()

#%%
from mpl_toolkits.mplot3d import Axes3D

# --- Clean column names ---
df.columns = df.columns.str.strip()

# --- Prepare azimuth in radians and unwrap ---
azimuth_rad = np.deg2rad(df['Azimuth'])
azimuth_unwrapped = np.unwrap(azimuth_rad)

# Convert to math convention (0°=East, CCW positive)
theta = np.deg2rad(90) - azimuth_unwrapped

# --- Compute cos/sin of azimuth ---
df['cos_az'] = np.cos(theta)
df['sin_az'] = np.sin(theta)

# --- Normalize accelerations to visualize on same scale ---
scale = 1 / df[['ax', 'ay']].abs().max().max() * 0.3  # scale factor for visibility
df['ax_scaled'] = df['ax'] * scale
df['ay_scaled'] = df['ay'] * scale

# --- Create 3D figure ---
fig = plt.figure(figsize=(12,8))
ax = fig.add_subplot(111, projection='3d')

# Plot the azimuth circle path over time
ax.plot(
    df['cos_az'],
    df['sin_az'],
    df['time'],
    color='tab:blue',
    linewidth=2,
    label='Heading (cos/sin)'
)

# Plot acceleration vectors (as arrows or lines)
ax.quiver(
    df['cos_az'], df['sin_az'], df['time'],     # base points
    df['ax_scaled'], df['ay_scaled'], np.zeros_like(df['time']),  # direction (no vertical)
    length=1.0, normalize=False, color='tab:red', alpha=0.7, label='Acceleration'
)

# --- Labels and style ---
ax.set_xlabel('cos(Azimuth)')
ax.set_ylabel('sin(Azimuth)')
ax.set_zlabel('Time [s]')
ax.set_title('3D Azimuth Path with Acceleration Vectors')
ax.legend()
ax.set_box_aspect([1,1,0.5])
plt.show()

#%%

#%%
