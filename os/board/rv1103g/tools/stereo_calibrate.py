#!/usr/bin/env python3
"""Calibrate the RV1103G stereo pair and build compact rectification maps.

Runs on a development PC with OpenCV and NumPy, not on the 64 MiB board.
"""

import argparse
from pathlib import Path
import struct
import sys

import cv2
import numpy as np


IMAGE_SUFFIXES = {".png", ".pgm", ".jpg", ".jpeg", ".tif", ".tiff"}
MAP_MAGIC = b"4RMP"
MAP_VERSION = 1
MAP_HEADER = "<4sIIIII"
INVALID_COORD = 65535


def images_by_stem(directory):
    if not directory.is_dir():
        raise ValueError(f"not a directory: {directory}")
    images = {}
    for path in sorted(directory.iterdir()):
        if path.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        if path.stem in images:
            raise ValueError(f"duplicate image stem in {directory}: {path.stem}")
        images[path.stem] = path
    return images


def read_gray(path):
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"cannot read image: {path}")
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if image.dtype != np.uint8:
        image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return image


def find_corners(image, pattern):
    found, corners = cv2.findChessboardCorners(
        image, pattern,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found:
        return None
    return cv2.cornerSubPix(
        image, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )


def write_map(path, map_x, map_y, source_size):
    source_width, source_height = source_size
    if max(source_width, source_height) > INVALID_COORD:
        raise ValueError("source image dimensions exceed uint16 map coordinates")
    height, width = map_x.shape
    valid = (
        np.isfinite(map_x) & np.isfinite(map_y)
        & (map_x >= 0) & (map_y >= 0)
        & (map_x < source_width) & (map_y < source_height)
    )
    # Nearest-neighbour source coordinates. A rounded edge coordinate may lie
    # outside the image even when the original floating-point coordinate did not.
    x = np.rint(np.where(valid, map_x, 0)).astype(np.int32)
    y = np.rint(np.where(valid, map_y, 0)).astype(np.int32)
    valid &= (x < source_width) & (y < source_height)
    pairs = np.full((height, width, 2), INVALID_COORD, dtype="<u2")
    pairs[:, :, 0][valid] = x[valid]
    pairs[:, :, 1][valid] = y[valid]

    with path.open("wb") as output:
        output.write(struct.pack(
            MAP_HEADER, MAP_MAGIC, MAP_VERSION,
            width, height, source_width, source_height,
        ))
        output.write(pairs.tobytes(order="C"))


def calibrate(args):
    outputs = [args.output_dir / name for name in
               ("stereo.yaml", "vo.calib", "left.map", "right.map")]
    existing = [path for path in outputs if path.exists()]
    if existing:
        raise ValueError(f"output already exists: {existing[0]}; choose a new output directory")
    left = images_by_stem(args.left)
    right = images_by_stem(args.right)
    names = sorted(left.keys() & right.keys())
    if len(names) < args.min_pairs:
        raise ValueError(f"only {len(names)} matched image names; need {args.min_pairs}")

    pattern = (args.cols, args.rows)
    grid = np.zeros((args.cols * args.rows, 3), dtype=np.float32)
    grid[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    grid *= args.square_mm / 1000.0  # metres; baseline is then in metres

    object_points, left_points, right_points = [], [], []
    size = None
    for name in names:
        a, b = read_gray(left[name]), read_gray(right[name])
        if a.shape != b.shape:
            raise ValueError(f"different left/right image sizes for {name}")
        if size is None:
            size = (a.shape[1], a.shape[0])
        elif size != (a.shape[1], a.shape[0]):
            raise ValueError(f"image size changed at {name}")
        a_corners, b_corners = find_corners(a, pattern), find_corners(b, pattern)
        if a_corners is None or b_corners is None:
            print(f"skip {name}: chessboard not found in both images")
            continue
        object_points.append(grid.copy())
        left_points.append(a_corners)
        right_points.append(b_corners)

    if len(object_points) < args.min_pairs:
        raise ValueError(
            f"only {len(object_points)} usable stereo pairs; need {args.min_pairs}"
        )
    print(f"calibrating with {len(object_points)} pairs at {size[0]}x{size[1]}")

    left_rms, K1, D1, _, _ = cv2.calibrateCamera(
        object_points, left_points, size, None, None,
    )
    right_rms, K2, D2, _, _ = cv2.calibrateCamera(
        object_points, right_points, size, None, None,
    )
    stereo_rms, K1, D1, K2, D2, R, T, _, _ = cv2.stereoCalibrate(
        object_points, left_points, right_points,
        K1, D1, K2, D2, size,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    output_size = (args.output_width, args.output_height)
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0.0,
        newImageSize=output_size,
    )
    if abs(P2[0, 3]) <= abs(P2[1, 3]):
        raise ValueError("VO requires a horizontally rectified stereo pair")
    if P2[0, 3] >= 0:
        raise ValueError(
            "right image appears to be from the physical left camera; "
            "swap --left and --right (and the VO capture device order)"
        )
    vertical_errors = []
    for a_corners, b_corners in zip(left_points, right_points):
        rect_a = cv2.undistortPoints(a_corners, K1, D1, R=R1, P=P1)
        rect_b = cv2.undistortPoints(b_corners, K2, D2, R=R2, P=P2)
        vertical_errors.extend(np.abs(rect_a[:, 0, 1] - rect_b[:, 0, 1]))
    median_vertical_error = float(np.median(vertical_errors))
    p95_vertical_error = float(np.percentile(vertical_errors, 95))
    map1_x, map1_y = cv2.initUndistortRectifyMap(
        K1, D1, R1, P1, output_size, cv2.CV_32FC1,
    )
    map2_x, map2_y = cv2.initUndistortRectifyMap(
        K2, D2, R2, P2, output_size, cv2.CV_32FC1,
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    yaml_path = args.output_dir / "stereo.yaml"
    fs = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise ValueError(f"cannot create {yaml_path}")
    try:
        for key, value in (
            ("image_width", size[0]), ("image_height", size[1]),
            ("rectified_width", output_size[0]),
            ("rectified_height", output_size[1]),
            ("square_size_m", args.square_mm / 1000.0),
            ("baseline_m", float(np.linalg.norm(T))),
            ("left_rms_px", left_rms), ("right_rms_px", right_rms),
            ("stereo_rms_px", stereo_rms),
            ("rectified_vertical_median_px", median_vertical_error),
            ("rectified_vertical_p95_px", p95_vertical_error),
            ("K1", K1), ("D1", D1), ("K2", K2), ("D2", D2),
            ("R", R), ("T", T), ("R1", R1), ("R2", R2),
            ("P1", P1), ("P2", P2), ("Q", Q),
        ):
            fs.write(key, value)
    finally:
        fs.release()

    write_map(args.output_dir / "left.map", map1_x, map1_y, size)
    write_map(args.output_dir / "right.map", map2_x, map2_y, size)
    with (args.output_dir / "vo.calib").open("w", encoding="ascii") as output:
        output.write(
            f"{P1[0, 0]:.9g} {P1[0, 2]:.9g} {P1[1, 2]:.9g} "
            f"{np.linalg.norm(T):.9g}\n"
        )
    print(f"left/right/stereo RMS: {left_rms:.3f} / {right_rms:.3f} / {stereo_rms:.3f} px")
    print(f"rectified vertical error median/p95: {median_vertical_error:.3f} / {p95_vertical_error:.3f} px")
    print(f"baseline: {np.linalg.norm(T) * 1000:.2f} mm")
    print(f"wrote stereo.yaml, vo.calib, left.map, right.map to {args.output_dir}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", type=Path, required=True, help="left image directory")
    parser.add_argument("--right", type=Path, required=True, help="right image directory")
    parser.add_argument("--cols", type=int, required=True, help="checkerboard inner columns")
    parser.add_argument("--rows", type=int, required=True, help="checkerboard inner rows")
    parser.add_argument("--square-mm", type=float, required=True, help="measured square side")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--output-width", type=int, default=640)
    parser.add_argument("--output-height", type=int, default=400)
    parser.add_argument("--min-pairs", type=int, default=12)
    args = parser.parse_args()
    if min(args.cols, args.rows) < 2 or args.square_mm <= 0:
        parser.error("pattern dimensions must be >= 2 and square-mm must be positive")
    if min(args.output_width, args.output_height, args.min_pairs) <= 0:
        parser.error("output dimensions and min-pairs must be positive")
    try:
        calibrate(args)
    except (OSError, ValueError, cv2.error) as exc:
        print(f"calibration failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
