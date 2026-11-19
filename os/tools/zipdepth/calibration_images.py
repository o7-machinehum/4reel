#!/usr/bin/env python3
"""Turn recorded native pairs into Kalibr bag-creater camera folders.

Keeps the measured V4L2 monotonic timestamp of each eye (microseconds -> ns),
and records the command pair ID in pairs.json. Does not invent exposures,
corner observations, calibration parameters, or a wall-clock epoch.
"""
import argparse
import json
from pathlib import Path
import cv2
from import_calibration import ORIENTATION


def main():
    p = argparse.ArgumentParser()
    p.add_argument("recordings", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--unit", required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    for camera in ["cam0", "cam1"]:
        (a.output / camera).mkdir()
    pairs = []
    for directory in sorted(a.recordings.glob("pair-*")):
        status = json.loads((directory / "status.json").read_text())
        if status["unit"] != a.unit or status["orientation"] != ORIENTATION:
            raise ValueError(f"Wrong unit/orientation in {directory}")
        for camera, eye in [("cam0", "left"), ("cam1", "right")]:
            image = cv2.imread(str(directory / f"{eye}.pgm"), cv2.IMREAD_UNCHANGED)
            if image is None or image.shape != (800, 1280) or str(image.dtype) != "uint8":
                raise ValueError(f"Not a native mono calibration capture: {directory}")
            timestamp = int(status[f"{eye}_timestamp_us"])*1000
            target = a.output / camera / f"{timestamp}.png"
            if target.exists() or not cv2.imwrite(str(target), image):
                raise ValueError(f"Duplicate timestamp or write failure: {target}")
        pairs.append(status)
    if not pairs:
        raise ValueError("No completed native calibration pairs found")
    (a.output / "pairs.json").write_text(json.dumps(pairs, indent=2)+"\n")


if __name__ == "__main__":
    main()
