#!/usr/bin/env python3
"""Capture native stereo poses for Kalibr and import its camera chain.

Requires numpy, opencv-python and PyYAML on the PC. Kalibr runs separately.
cam0 is the dashboard's left camera, cam1 is its right camera.
"""
import argparse
import json
from pathlib import Path
import urllib.error

import cv2
import numpy as np

from calibrate_fisheye import RAW_SIZE, OUTPUT_SIZE, capture_aprilgrid, common_projection, request


def camera_model(entry, topic):
    if not isinstance(entry, dict):
        raise ValueError("Each Kalibr camera entry must be a mapping")
    if entry.get("resolution") != list(RAW_SIZE) or entry.get("rostopic") != topic:
        raise ValueError(f"Expected {topic} at native {RAW_SIZE}, with cam0=left and cam1=right")
    values = np.asarray(entry.get("intrinsics", []), dtype=float)
    distortion = np.asarray(entry.get("distortion_coeffs", []), dtype=float)
    model = entry.get("camera_model")
    if model == "ds" and entry.get("distortion_model") == "none" and values.shape == (6,) and distortion.size == 0:
        xi, alpha, fx, fy, cx, cy = values
        if not 0 <= alpha <= 1:
            raise ValueError("Double sphere alpha must be between 0 and 1")
        weight = alpha / (1 - alpha) if alpha <= .5 else (1 - alpha) / alpha
        domain = 1 + xi * xi + 2 * weight * xi
        if not np.isfinite(domain) or domain <= 0:
            raise ValueError("Invalid double sphere projection domain")
        distortion = np.zeros(4)
        kind = 1
    elif model == "pinhole" and entry.get("distortion_model") == "equidistant" and values.shape == (4,) and distortion.shape == (4,):
        fx, fy, cx, cy = values
        xi = alpha = 0.
        kind = 0
    else:
        raise ValueError("Supported Kalibr models are ds-none and pinhole-equi; models are never silently converted")
    if not np.isfinite(values).all() or not np.isfinite(distortion).all() or not (
            1 <= fx <= 10000 and 1 <= fy <= 10000 and 0 <= cx < RAW_SIZE[0] and 0 <= cy < RAW_SIZE[1]):
        raise ValueError("Invalid native camera intrinsics")
    return {"model": kind, "xi": float(xi), "alpha": float(alpha),
            "K": np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]]), "D": distortion}


def project_map(camera, rectification, projection):
    if camera["model"] == 0:
        return cv2.fisheye.initUndistortRectifyMap(camera["K"], camera["D"], rectification,
            projection, OUTPUT_SIZE, cv2.CV_32FC1)
    yy, xx = np.mgrid[:OUTPUT_SIZE[1], :OUTPUT_SIZE[0]]
    rays = np.stack(((xx - projection[0, 2]) / projection[0, 0],
                     (yy - projection[1, 2]) / projection[1, 1], np.ones_like(xx)), axis=-1)
    native = rays @ rectification
    x, y, z = native[:, :, 0], native[:, :, 1], native[:, :, 2]
    norm = np.linalg.norm(native, axis=-1)
    shifted_z = camera["xi"] * norm + z
    denominator = camera["alpha"] * np.sqrt(x*x + y*y + shifted_z*shifted_z) + (1-camera["alpha"]) * shifted_z
    alpha, xi = camera["alpha"], camera["xi"]
    weight = alpha / (1-alpha) if alpha <= .5 else (1-alpha) / alpha
    fov = (weight + xi) / np.sqrt(1 + xi*xi + 2*weight*xi)
    valid = (denominator > 0) & (z > -fov * norm)
    safe = np.where(valid, denominator, np.nan)
    return camera["K"][0, 0] * x/safe + camera["K"][0, 2], camera["K"][1, 1] * y/safe + camera["K"][1, 2]


def convert_camchain(chain, rms=-1):
    if not isinstance(chain, dict) or set(chain) != {"cam0", "cam1"}:
        raise ValueError("Expected a two-camera Kalibr chain containing cam0 and cam1")
    cameras = [camera_model(chain[f"cam{side}"], f"/cam{side}/image_raw") for side in (0, 1)]
    transform = np.asarray(chain["cam1"].get("T_cn_cnm1", []), dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all() or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError("Invalid cam0-to-cam1 homogeneous transform")
    rotation, translation = transform[:3, :3], transform[:3, 3] * 1000
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6) or abs(np.linalg.det(rotation) - 1) > 1e-6:
        raise ValueError("Kalibr rotation must be right-handed and orthonormal")
    if not .1 <= np.linalg.norm(translation) <= 10000:
        raise ValueError("Invalid stereo baseline; Kalibr translation must be in metres")
    if not np.isfinite(rms) or (rms != -1 and not 0 <= rms <= 3):
        raise ValueError("RMS must be unknown (-1) or a measured value from 0 to 3 native pixels")
    # Rectifying rotations depend only on R/T. The pinhole arguments satisfy
    # OpenCV's API; its camera matrices are discarded, not used to fit lenses.
    reference = np.array([[400., 0, 640], [0, 400, 400], [0, 0, 1.]])
    r1, r2, _, _, _, _, _ = cv2.stereoRectify(reference, np.zeros(5), reference, np.zeros(5),
        RAW_SIZE, rotation, translation, flags=cv2.CALIB_ZERO_DISPARITY)
    rectifications = (r1, r2)
    shifted = r2 @ translation
    if np.linalg.norm(shifted[1:]) > np.linalg.norm(translation) * 1e-6:
        raise ValueError("The board's matcher requires a horizontal stereo baseline")
    baseline = -float(shifted[0])
    camera_tuples = [(camera["K"], camera["D"], rectifications[side]) for side, camera in enumerate(cameras)]
    projection = common_projection(camera_tuples, RAW_SIZE,
        lambda side, matrix: project_map(cameras[side], rectifications[side], matrix))
    values = [projection[0, 0], projection[1, 1], projection[0, 2], projection[1, 2], baseline, rms]
    for side, camera in enumerate(cameras):
        values.extend((camera["model"], camera["xi"], camera["alpha"]))
        for intrinsic, distortion in zip((camera["K"][0, 0], camera["K"][1, 1], camera["K"][0, 2], camera["K"][1, 2]), camera["D"]):
            values.extend((intrinsic, distortion))
        values.extend(rectifications[side].ravel())
    text = "4REELFISHEYE2 1280 800 160 100 " + " ".join(f"{number:.9g}" for number in values) + "\n"
    if len(text.encode()) > 2048:
        raise ValueError("Converted profile exceeds the board upload limit")
    metadata = {"source": "Kalibr", "camera_models": [entry["camera_model"] for entry in (chain["cam0"], chain["cam1"])],
                "baseline_mm": abs(baseline), "rms_pixels": None if rms == -1 else rms,
                "horizontal_fov_degrees": float(np.degrees(2*np.arctan(OUTPUT_SIZE[0]/(2*projection[0, 0])))),
                "rectifications": [matrix.tolist() for matrix in rectifications],
                "projection": projection.tolist(), "camchain": chain}
    return text, metadata


def import_calibration(args):
    try:
        import yaml
    except ImportError as error:
        raise ValueError("Install PyYAML in this Python environment") from error
    try:
        chain = yaml.safe_load(args.camchain.read_text())
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid Kalibr YAML: {error}") from error
    text, metadata = convert_camchain(chain, args.rms_px)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".txt").write_text(text)
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Saved {args.output.with_suffix('.txt')}: baseline {metadata['baseline_mm']:.3f} mm, view {metadata['horizontal_fov_degrees']:.1f} degrees")
    if args.rms_px == -1:
        print("RMS is unknown in camchain.yaml; check Kalibr's report. No zero-error claim is stored.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    grab = commands.add_parser("capture", help="Save stationary native pairs in Kalibr bagcreater layout")
    grab.add_argument("--url", default="http://192.168.77.1:8080")
    grab.add_argument("--output", type=Path, required=True)
    grab.add_argument("--count", type=int, default=40)
    grab.set_defaults(run=capture_aprilgrid, capture_format="4reel-kalibr-static-v1")
    convert = commands.add_parser("import", help="Convert Kalibr's camchain.yaml for the board")
    convert.add_argument("--camchain", type=Path, required=True)
    convert.add_argument("--output", type=Path, required=True)
    convert.add_argument("--rms-px", type=float, default=-1, help="Optional measured reprojection RMS from Kalibr's report")
    convert.set_defaults(run=import_calibration)
    upload = commands.add_parser("upload", help="Install the converted profile on the board")
    upload.add_argument("--url", default="http://192.168.77.1:8080")
    upload.add_argument("--input", type=Path, required=True)
    upload.set_defaults(run=lambda args: print(request(args.url, "/api/fisheye", args.input.read_text()).decode()))
    args = parser.parse_args()
    if hasattr(args, "count") and args.count < 20:
        parser.error("Capture at least 20 diverse stationary poses")
    try:
        args.run(args)
    except (ValueError, KeyError, TypeError, cv2.error, urllib.error.URLError, OSError) as error:
        parser.exit(1, f"Kalibr workflow failed: {error}\n")


if __name__ == "__main__":
    main()
