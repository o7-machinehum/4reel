#!/usr/bin/env python3
"""Capture and calibrate one 4reel fisheye stereo pair on a PC.

Requires numpy and opencv-python. The board needs the fisheye-enabled OS.
"""
import argparse
import json
from pathlib import Path
import struct
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

import cv2
import numpy as np

RAW_SIZE = (1280, 800)
OUTPUT_SIZE = (160, 100)
PAIR_HEADER = struct.Struct("<8sIIQQd")


def request(url, path, data=None):
    headers = {"X-4reel-control": "1"}
    if isinstance(data, dict):
        data = urllib.parse.urlencode(data).encode()
        headers["Content-Type"] = "application/x-www-form-urlencoded"
    elif isinstance(data, str):
        data = data.encode()
        headers["Content-Type"] = "text/plain"
    message = urllib.request.Request(url.rstrip("/") + path, data=data, headers=headers)
    with urllib.request.urlopen(message, timeout=20) as response:
        body = response.read(2_100_000)
        if response.read(1):
            raise ValueError("Board response exceeds the expected frame size")
        return body


def decode_pair(data):
    if len(data) < PAIR_HEADER.size:
        raise ValueError("Truncated native pair header")
    magic, width, height, frame, timestamp, delta = PAIR_HEADER.unpack_from(data)
    if magic != b"4REELC1\0" or (width, height) != RAW_SIZE or not 0 <= delta <= 20:
        raise ValueError("Invalid native stereo pair or excessive camera timing skew")
    size = 1078 + width * height
    if len(data) != PAIR_HEADER.size + 2 * size:
        raise ValueError("Truncated native stereo images")
    images = []
    for side in range(2):
        start = PAIR_HEADER.size + side * size
        image = cv2.imdecode(np.frombuffer(data[start:start + size], np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None or image.shape != (height, width):
            raise ValueError("Invalid native BMP image")
        images.append(image)
    return images, {"frame": frame, "timestamp_us": timestamp, "pair_delta_ms": delta}


def corners(image, pattern):
    found, points = cv2.findChessboardCornersSB(image, pattern,
        flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not found:
        return None
    return points.astype(np.float64).reshape(1, -1, 2)


def common_projection(cameras, size, project_maps=None, max_horizontal_fov=150):
    # Boundary undistortion becomes singular near 180 degrees. Choose a
    # centred rectilinear view by testing forward maps instead. Keep every
    # output pixel inside both native images, with room for bilinear taps.
    width, height = OUTPUT_SIZE
    yy, xx = np.mgrid[:height, :width]
    cx, cy = (width - 1) / 2, (height - 1) / 2

    def projection(focal):
        return np.array([[focal, 0, cx], [0, focal, cy], [0, 0, 1.]])

    def fits(focal):
        rays = np.stack(((xx - cx) / focal, (yy - cy) / focal, np.ones_like(xx)), axis=-1)
        for side, (matrix, distortion, rectification) in enumerate(cameras):
            if (rays @ rectification)[:, :, 2].min() <= 0:
                return False
            if project_maps is None:
                mx, my = cv2.fisheye.initUndistortRectifyMap(matrix, distortion, rectification,
                    projection(focal), OUTPUT_SIZE, cv2.CV_32FC1)
            else:
                mx, my = project_maps(side, projection(focal))
            if not (np.isfinite(mx).all() and np.isfinite(my).all() and
                    mx.min() >= 2 and my.min() >= 2 and mx.max() < size[0] - 3 and my.max() < size[1] - 3):
                return False
            # Match the board's conservative lens mask; avoid the vignetted
            # rim as well as the rectangular sensor boundary.
            radius = min(size) * .45 - 3
            if np.max((mx - matrix[0, 2]) ** 2 + (my - matrix[1, 2]) ** 2) >= radius ** 2:
                return False
        return True

    low = width / (2 * np.tan(np.radians(max_horizontal_fov / 2)))
    high = low
    while not fits(high):
        high *= 2
        if high > 10000:
            raise ValueError("The cameras have no usable common forward view")
    for _ in range(24):
        middle = (low + high) / 2
        if fits(middle):
            high = middle
        else:
            low = middle
    return projection(high * 1.005)


def capture(args):
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.glob("*-left.png")):
        raise ValueError("Use an empty capture directory to avoid mixing different lens setups")
    pattern = (args.columns, args.rows)
    (args.output / "capture.json").write_text(json.dumps({
        "columns": args.columns, "rows": args.rows, "native_size": RAW_SIZE}, indent=2) + "\n")
    request(args.url, "/api/capture", {"enabled": 1, "mode": "calibration"})
    last_frame = 0
    accepted = 0
    try:
        print("Keep the checkerboard still for each capture. Vary its position, distance and tilt.")
        while accepted < args.count:
            input(f"Pose {accepted + 1}/{args.count}: press Enter when the board is stationary: ")
            # Let queued exposures of the moving target leave the camera buffers.
            time.sleep(0.4)
            images, metadata = decode_pair(request(args.url, "/api/calibration/pair.bin"))
            if metadata["frame"] == last_frame:
                print("No new pair yet; try again.")
                continue
            last_frame = metadata["frame"]
            if any(corners(image, pattern) is None for image in images):
                print("Checkerboard was not detected in both cameras; reposition it and try again.")
                continue
            stem = args.output / f"pair-{accepted:03d}"
            for side, image in zip(("left", "right"), images):
                if not cv2.imwrite(f"{stem}-{side}.png", image):
                    raise ValueError("Could not save calibration image")
            stem.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
            accepted += 1
            print(f"Saved pair {metadata['frame']}, camera skew {metadata['pair_delta_ms']:.2f} ms")
    finally:
        request(args.url, "/api/capture", {"enabled": 0})


def capture_aprilgrid(args):
    if args.output.exists() and any(args.output.iterdir()):
        raise ValueError("Use an empty directory for this camera/lens assembly")
    for side in (0, 1):
        (args.output / f"cam{side}").mkdir(parents=True, exist_ok=True)
    metadata = {"format": args.capture_format, "resolution": list(RAW_SIZE),
                "cam0": "left", "cam1": "right", "pairs": [],
                "timestamps": "common pair midpoint for stationary poses"}
    request(args.url, "/api/capture", {"enabled": 1, "mode": "calibration"})
    last_frame = 0
    try:
        print("Vary target position, tilt and distance. Hold it still before pressing Enter.")
        while len(metadata["pairs"]) < args.count:
            pose = len(metadata["pairs"]) + 1
            input(f"Pose {pose}/{args.count}: press Enter when the target is stationary: ")
            time.sleep(.5)
            images, pair = decode_pair(request(args.url, "/api/calibration/pair.bin"))
            if pair["frame"] <= last_frame:
                print("No new camera pair yet; try this pose again.")
                continue
            last_frame = pair["frame"]
            name = f"{pair['timestamp_us'] * 1000:019d}.png"
            for side, image in enumerate(images):
                if not cv2.imwrite(str(args.output / f"cam{side}" / name), image):
                    raise ValueError("Could not save native calibration image")
            metadata["pairs"].append({"image": name, **pair})
            (args.output / "capture.json").write_text(json.dumps(metadata, indent=2) + "\n")
            print(f"Saved native pair {pair['frame']}; skew {pair['pair_delta_ms']:.2f} ms")
    finally:
        request(args.url, "/api/capture", {"enabled": 0})


def solve_points(object_points, left_points, right_points, size=RAW_SIZE):
    if len(object_points) < 20 or len(left_points) != len(object_points) or len(right_points) != len(object_points):
        raise ValueError("At least 20 valid, diverse stereo pairs are required")
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-8)
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW
    intrinsics = []
    errors = []
    for image_points in (left_points, right_points):
        rms, matrix, distortion, _, _ = cv2.fisheye.calibrate(
            object_points, image_points, size, np.eye(3), np.zeros((4, 1)),
            flags=flags, criteria=criteria)
        intrinsics.append((matrix, distortion))
        errors.append(float(rms))
    k1, d1 = intrinsics[0]
    k2, d2 = intrinsics[1]
    stereo = cv2.fisheye.stereoCalibrate(
        object_points, left_points, right_points, k1, d1, k2, d2, size,
        flags=cv2.fisheye.CALIB_FIX_INTRINSIC, criteria=criteria)
    # Recent OpenCV versions also return per-view rvecs/tvecs.
    rms, k1, d1, k2, d2, rotation, translation = stereo[:7]
    errors.append(float(rms))
    if not np.isfinite(errors).all() or max(errors) > 3:
        raise ValueError(f"Reprojection error exceeds 3 native pixels: {errors}")
    return rectified_profile(k1, d1, k2, d2, rotation, translation,
        dict(zip(("left", "right", "stereo"), errors)), size)


def rectified_profile(k1, d1, k2, d2, rotation, translation, errors,
                      size=RAW_SIZE, max_horizontal_fov=150):
    translation = np.asarray(translation).reshape(3, 1)
    r1, r2, _, _, _ = cv2.fisheye.stereoRectify(k1, d1, k2, d2, size,
        rotation, translation, flags=cv2.fisheye.CALIB_ZERO_DISPARITY,
        newImageSize=OUTPUT_SIZE, balance=0, fov_scale=1)
    rectified_translation = (r2 @ translation).ravel()
    if np.linalg.norm(rectified_translation[1:]) > np.linalg.norm(translation) * 1e-6:
        raise ValueError("The board matcher requires a horizontal stereo baseline")
    baseline = -float(rectified_translation[0])
    if not np.isfinite(baseline) or not 0.1 <= abs(baseline) <= 10000:
        raise ValueError("Invalid baseline or rectified projection")
    projection = common_projection(((k1, d1, r1), (k2, d2, r2)), size,
                                   max_horizontal_fov=max_horizontal_fov)
    p1 = np.column_stack((projection, np.zeros(3)))
    p2 = p1.copy()
    p2[0, 3] = -projection[0, 0] * baseline
    q = np.array([[1, 0, 0, -projection[0, 2]], [0, 1, 0, -projection[1, 2]],
                  [0, 0, 0, projection[0, 0]], [0, 0, 1 / baseline, 0]])
    cameras = []
    values = [p1[0, 0], p1[1, 1], p1[0, 2], p1[1, 2], baseline, max(errors.values())]
    for matrix, distortion, rectification in ((k1, d1, r1), (k2, d2, r2)):
        intrinsic_values = [matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2]]
        for value, coefficient in zip(intrinsic_values, distortion.ravel()):
            values.extend((value, coefficient))
        values.extend(rectification.ravel())
        cameras.append({"K": matrix.tolist(), "D": distortion.ravel().tolist(),
                        "rectification": rectification.tolist()})
    text = "4REELFISHEYE1 1280 800 160 100 " + " ".join(f"{value:.9g}" for value in values) + "\n"
    if len(text.encode()) > 2048:
        raise ValueError("Calibration exceeds the board's upload limit")
    profile = {"format": "4REELFISHEYE1", "native_size": list(size), "output_size": list(OUTPUT_SIZE),
        "cameras": cameras, "rotation": rotation.tolist(), "translation_mm": translation.ravel().tolist(),
        "projection": [p1.tolist(), p2.tolist()], "Q": q.tolist(),
        "baseline_mm": abs(baseline), "rms": errors,
        "horizontal_fov_degrees": float(np.degrees(2 * np.arctan(OUTPUT_SIZE[0] / (2 * p1[0, 0]))))}
    return text, profile


def solve(args):
    pattern = (args.columns, args.rows)
    grid = np.zeros((1, args.columns * args.rows, 3), np.float64)
    grid[0, :, :2] = np.mgrid[:args.columns, :args.rows].T.reshape(-1, 2) * args.square_mm
    objects, left, right = [], [], []
    for path in sorted(args.input.glob("*-left.png")):
        mate = path.with_name(path.name.replace("-left.png", "-right.png"))
        points = []
        for image_path in (path, mate):
            image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
            if image is None or image.shape != (RAW_SIZE[1], RAW_SIZE[0]):
                raise ValueError(f"Missing or non-native calibration image: {image_path}")
            points.append(corners(image, pattern))
        if any(value is None for value in points):
            print(f"Skipping pair without a detectable checkerboard: {path.name}")
            continue
        # The sensors have the same mounting orientation; resolve the detector's
        # 180-degree ambiguity on an unmarked checkerboard consistently.
        if np.dot((points[0][0, -1] - points[0][0, 0]), (points[1][0, -1] - points[1][0, 0])) < 0:
            points[1] = points[1][:, ::-1].copy()
        objects.append(grid.copy())
        left.append(points[0])
        right.append(points[1])
    text, profile = solve_points(objects, left, right)
    profile.update({"square_mm": args.square_mm, "pattern": list(pattern), "pairs": len(objects)})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".txt").write_text(text)
    args.output.with_suffix(".json").write_text(json.dumps(profile, indent=2) + "\n")
    print(f"Saved {args.output.with_suffix('.txt')} and .json; baseline {profile['baseline_mm']:.3f} mm")
    print(f"Native reprojection RMS: {profile['rms']}; rectified horizontal field {profile['horizontal_fov_degrees']:.1f} degrees")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    grab = commands.add_parser("capture", help="Capture stationary native checkerboard pairs")
    grab.add_argument("--url", default="http://192.168.77.1:8080")
    grab.add_argument("--output", type=Path, required=True)
    grab.add_argument("--count", type=int, default=25)
    grab.add_argument("--columns", type=int, default=9)
    grab.add_argument("--rows", type=int, default=6)
    grab.set_defaults(run=capture)
    april = commands.add_parser("capture-aprilgrid", help="Capture stationary native AprilGrid pairs")
    april.add_argument("--url", default="http://192.168.77.1:8080")
    april.add_argument("--output", type=Path, required=True)
    april.add_argument("--count", type=int, default=40)
    april.set_defaults(run=capture_aprilgrid, capture_format="4reel-aprilgrid-static-v1")
    fit = commands.add_parser("solve", help="Estimate fisheye intrinsics and stereo geometry")
    fit.add_argument("--input", type=Path, required=True)
    fit.add_argument("--output", type=Path, required=True)
    fit.add_argument("--square-mm", type=float, required=True)
    fit.add_argument("--columns", type=int, default=9)
    fit.add_argument("--rows", type=int, default=6)
    fit.set_defaults(run=solve)
    upload = commands.add_parser("upload", help="Install a solved calibration on its board")
    upload.add_argument("--url", default="http://192.168.77.1:8080")
    upload.add_argument("--input", type=Path, required=True)
    upload.set_defaults(run=lambda args: print(request(args.url, "/api/fisheye", args.input.read_text()).decode()))
    args = parser.parse_args()
    if hasattr(args, "columns") and (args.columns < 3 or args.rows < 3):
        parser.error("Use at least three inner corners along each checkerboard dimension")
    if hasattr(args, "square_mm") and (not np.isfinite(args.square_mm) or args.square_mm <= 0):
        parser.error("Square size must be a positive measurement in millimetres")
    if hasattr(args, "count") and args.count < 20:
        parser.error("Capture at least 20 diverse poses")
    try:
        args.run(args)
    except (ValueError, cv2.error, urllib.error.URLError, OSError) as error:
        parser.exit(1, f"Calibration failed: {error}\n")


if __name__ == "__main__":
    main()
