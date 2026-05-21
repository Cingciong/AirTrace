import os
import json
import subprocess
import csv
from tqdm import tqdm

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "..", "data")
OUTPUT_DIR = os.path.join(BASE_DIR, "..", "data")
EXIFTOOL_COMMAND = "exiftool"
EXIFTOOL_ARGS = ["-ee", "-G3", "-j"]

def parseMp4ToCsv():
    fileList = [f for f in os.listdir(DATA_DIR) if f.lower().endswith(".mp4")]

    for fileName in tqdm(fileList, desc="Processing MP4 files"):
        mp4Path = os.path.join(DATA_DIR, fileName)
        csvPath = os.path.join(OUTPUT_DIR, fileName.lower().replace(".mp4", ".csv"))

        runArgs = [EXIFTOOL_COMMAND] + EXIFTOOL_ARGS + [mp4Path]
        result = subprocess.run(runArgs, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        try:
            fullData = json.loads(result.stdout)
        except json.JSONDecodeError:
            continue

        if not fullData:
            continue

        csvRows = []
        allKeysSet = set()

        for item in fullData:
            rowDict = {}
            for key, val in item.items():
                cleanKey = key.split(":")[-1]
                rowDict[cleanKey] = str(val)
                allKeysSet.add(cleanKey)
            if rowDict:
                csvRows.append(rowDict)

        if csvRows:
            orderedKeys = sorted(list(allKeysSet))
            with open(csvPath, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=orderedKeys)
                writer.writeheader()
                writer.writerows(csvRows)

if __name__ == "__main__":
    parseMp4ToCsv()