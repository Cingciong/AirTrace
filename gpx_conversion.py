import pandas as pd
import os
from xml.etree.ElementTree import Element, SubElement, ElementTree

# --- Step 1: Load your CSV file ---
# Adjust this if your column names differ (e.g., 'lat', 'lon', 'elevation')
# ---------------------------------------------------------------
# CHANGE LOG HERE:
log_name = "log_2"
# ---------------------------------------------------------------
base_path = log_name + "_complete_analysis"
csv_path = os.path.join(base_path,"gps_data.csv")
df = pd.read_csv(csv_path)

# Try to identify coordinate columns automatically
lat_col = [c for c in df.columns if 'lat' in c.lower()][0]
lon_col = [c for c in df.columns if 'lon' in c.lower()][0]
ele_col = next((c for c in df.columns if 'ele' in c.lower() or 'alt' in c.lower()), None)
time_col = next((c for c in df.columns if 'time' in c.lower()), None)

# --- Step 2: Build GPX structure ---
gpx = Element('gpx', version="1.1", creator="DroneSim", xmlns="http://www.topografix.com/GPX/1/1")
trk = SubElement(gpx, 'trk')
name = SubElement(trk, 'name')
name.text = 'Drone Simulation Route'
trkseg = SubElement(trk, 'trkseg')

for _, row in df.iterrows():
    trkpt = SubElement(trkseg, 'trkpt', lat=str(row[lat_col]), lon=str(row[lon_col]))
    if ele_col and not pd.isna(row[ele_col]):
        ele = SubElement(trkpt, 'ele')
        ele.text = str(row[ele_col])
    if time_col and not pd.isna(row[time_col]):
        time = SubElement(trkpt, 'time')
        time.text = str(row[time_col])

# --- Step 3: Save to GPX file ---
output_path = os.path.join(base_path,"drone_route.gpx")
ElementTree(gpx).write(output_path, encoding='utf-8', xml_declaration=True)

print(f"✅ GPX file saved as {output_path}")