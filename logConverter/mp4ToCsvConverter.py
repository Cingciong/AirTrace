import os
import json
import subprocess
import csv
from tqdm import tqdm

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "..", "data")
OUTPUT_DIR = os.path.join(BASE_DIR, "..", "data")
EXIFTOOL_COMMAND = "exiftool"
EXIFTOOL_ARGS = ["-ee", "-G3", "-j", "-n"]

def parseMp4ToCsv():
    fileList = [f for f in os.listdir(DATA_DIR) if f.lower().endswith(".mp4")]

    for fileName in tqdm(fileList, desc="Processing MP4 files"):
        tqdm.write(f"Currently processing: {fileName}")
        mp4Path = os.path.join(DATA_DIR, fileName)
        csvPath = os.path.join(OUTPUT_DIR, fileName.lower().replace(".mp4", ".csv"))

        runArgs = [EXIFTOOL_COMMAND] + EXIFTOOL_ARGS + [mp4Path]
        result = subprocess.run(runArgs, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        try:
            fullData = json.loads(result.stdout)
        except json.JSONDecodeError:
            continue

        if not fullData or not isinstance(fullData, list):
            continue

        mainDoc = fullData[0]
        csvRows = []
        allKeysSet = set()
        tempRowDict = {}

        for key, val in mainDoc.items():
            parts = key.split(":")
            if len(parts) < 2:
                continue
            
            docTag = parts[0]
            cleanKey = parts[1]

            if not docTag.startswith("Doc"):
                continue

            if cleanKey == "SampleTime" or cleanKey == "SampleDuration":
                if cleanKey == "SampleTime" and ("0:" in str(val) or ":" in str(val)) and len(str(val)) > 4:
                    continue
                
                if cleanKey == "SampleTime":
                    if tempRowDict:
                        csvRows.append(tempRowDict)
                        allKeysSet.update(tempRowDict.keys())
                    tempRowDict = {cleanKey: str(val)}
                else:
                    tempRowDict[cleanKey] = str(val)
            else:
                if tempRowDict:
                    tempRowDict[cleanKey] = str(val)

        if tempRowDict:
            csvRows.append(tempRowDict)
            allKeysSet.update(tempRowDict.keys())

        if csvRows:
            orderedKeys = sorted(list(allKeysSet))
            if "SampleTime" in orderedKeys:
                orderedKeys.remove("SampleTime")
                orderedKeys = ["SampleTime"] + orderedKeys

            with open(csvPath, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=orderedKeys)
                writer.writeheader()
                writer.writerows(csvRows)

if __name__ == "__main__":
    parseMp4ToCsv()