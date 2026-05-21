import os
import re

DATA_DIR = r"..\data"
OUTPUT_DIR = r"..\data"
SUBTITLE_PATTERN = r"(\d{2}:\d{2}:\d{2},\d{3}) --> (\d{2}:\d{2}:\d{2},\d{3})"

def parseSrtToCsv():
    for fileName in os.listdir(DATA_DIR):
        if not fileName.lower().endswith(".srt"):
            continue

        srtPath = os.path.join(DATA_DIR, fileName)
        csvPath = os.path.join(OUTPUT_DIR, fileName.lower().replace(".srt", ".csv"))

        with open(srtPath, "r", encoding="utf-8", errors="ignore") as f:
            content = f.read()

        blocks = content.strip().split("\n\n")
        csvRows = []
        allKeys = ["Target_Time"]

        for block in blocks:
            lines = [line.strip() for line in block.split("\n") if line.strip()]
            if len(lines) < 3:
                continue

            timeMatch = re.match(SUBTITLE_PATTERN, lines[1])
            if not timeMatch:
                continue

            startTime = timeMatch.group(1)
            dataText = " ".join(lines[2:])
            
            rowDict = {"Target_Time": startTime}
            
            foundPairs = re.findall(r"([a-zA-Z_]+)\s*:\s*([^\]\s,\[]+)", dataText)
            for key, val in foundPairs:
                cleanKey = key.strip()
                cleanVal = val.strip()
                rowDict[cleanKey] = cleanVal
                if cleanKey not in allKeys:
                    allKeys.append(cleanKey)
                    
            if len(rowDict) > 1:
                csvRows.append(rowDict)

        if csvRows:
            with open(csvPath, "w", newline="", encoding="utf-8") as f:
                f.write(",".join(allKeys) + "\n")
                for row in csvRows:
                    rowValues = [row.get(key, "") for key in allKeys]
                    f.write(",".join(rowValues) + "\n")

if __name__ == "__main__":
    parseSrtToCsv()