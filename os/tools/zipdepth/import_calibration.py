#!/usr/bin/env python3
"""Validate a real per-unit Kalibr ds-none camchain for the native capture path."""
import argparse
import hashlib
import re
from pathlib import Path
import numpy as np
import yaml

ORIENTATION = "left=1,right=0,hflip=both,right_sensor_rotation=180"


def convert(source, unit, cam0_eye, orientation):
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,63}", unit):
        raise ValueError("unit must be 1..63 ASCII letters, digits, underscores, dots or hyphens")
    if orientation != ORIENTATION:
        raise ValueError("Calibration must use the exact delivered native image orientation")
    chain = yaml.safe_load(source)
    cameras = [chain[f"cam{i}"] for i in range(2)]
    for cam in cameras:
        if cam["camera_model"] != "ds" or cam["distortion_model"] != "none":
            raise ValueError("Expected Kalibr ds-none cameras")
        if cam["resolution"] != [1280, 800] or cam.get("distortion_coeffs", []):
            raise ValueError("Expected native 1280x800, no extra distortion coefficients")
        p = np.array(cam["intrinsics"], dtype=float)
        if p.shape != (6,) or not np.isfinite(p).all():
            raise ValueError("Invalid intrinsic vector")
        xi, alpha, fx, fy, cx, cy = p
        if not (-1 < xi < 3 and 0 < alpha < 1 and fx > 0 and fy > 0 and
                0 <= cx < 1280 and 0 <= cy < 800):
            raise ValueError("Out-of-domain DS intrinsics")
    transform = np.array(cameras[1]["T_cn_cnm1"], dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("Invalid T_cn_cnm1")
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError("Invalid homogeneous transform row")
    r = transform[:3, :3]
    if not np.allclose(r @ r.T, np.eye(3), atol=1e-6) or abs(np.linalg.det(r)-1)>1e-6:
        raise ValueError("Extrinsic rotation must be proper and orthonormal")
    if not 0.04 <= np.linalg.norm(transform[:3, 3]) <= 0.12:
        raise ValueError("Baseline outside rig sanity interval 40..120 mm; check metres and unit")
    if cam0_eye == "right":
        cameras.reverse()
        transform = np.linalg.inv(transform)
    values = [c["intrinsics"] for c in cameras]
    values += [transform[:3, :3].ravel(), transform[:3, 3]]
    return (f"4REEL_DS1 {unit} {hashlib.sha256(source).hexdigest()} {ORIENTATION}\n" +
            "\n".join(" ".join(format(float(x), ".17g") for x in row) for row in values) + "\n")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("camchain", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--unit", required=True)
    p.add_argument("--cam0-eye", choices=["left", "right"], required=True)
    p.add_argument("--orientation", required=True, choices=[ORIENTATION])
    a = p.parse_args()
    result = convert(a.camchain.read_bytes(), a.unit, a.cam0_eye, a.orientation)
    # Exclusive creation prevents accidentally replacing another unit's data.
    with a.output.open("x") as f:
        f.write(result)


if __name__ == "__main__":
    main()
