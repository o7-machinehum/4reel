#!/usr/bin/env python3
"""Fit the 4reel fisheye stereo geometry from paired AprilGrid images."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

from calibrate_fisheye import RAW_SIZE, rectified_profile


def observe(folder, tag_mm):
    # The 4reel 5x5 print numbers tags left to right, starting at the bottom.
    # OpenCV returns the four corners starting at the lower right of this grid.
    tag = tag_mm / 1000
    corners = np.array([[tag, 0, 0], [0, 0, 0], [0, tag, 0], [tag, tag, 0]])
    grid = np.array([corners + [column * tag * 1.3, row * tag * 1.3, 0]
                     for row in range(5) for column in range(5)])
    parameters = cv2.aruco.DetectorParameters()
    parameters.markerBorderBits = 2
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), parameters)
    contrast = cv2.createCLAHE(clipLimit=2., tileGridSize=(8, 8))
    files = [sorted((folder / f"cam{side}").glob("*.png")) for side in range(2)]
    if not files[0] or [p.name for p in files[0]] != [p.name for p in files[1]]:
        raise ValueError("Expected matching native PNG pairs in cam0 and cam1")
    print(f"Detecting tags in {len(files[0])} native pairs", flush=True)
    observations = []
    for left, right in zip(*files):
        pair = []
        for path in (left, right):
            image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if image is None or image.shape != RAW_SIZE[::-1]:
                raise ValueError(f"Expected a native 1280 × 800 image at {path}")
            found = {}
            for view, scale in ((image, 1),
                                (cv2.resize(image, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC), 2),
                                (contrast.apply(image), 1)):
                points, ids, _ = detector.detectMarkers(view)
                if ids is not None:
                    for tag_id, point in zip(ids.ravel(), points):
                        if 0 <= tag_id < len(grid):
                            found.setdefault(int(tag_id), point[0] / scale)
            pair.append((grid[list(found)].reshape(-1, 3),
                         np.asarray(list(found.values()), dtype=float).reshape(-1, 2)))
        observations.append((left.name, pair))
    return observations


def intrinsics(parameters):
    fx, fy, cx, cy, k1, k2 = parameters
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]]), np.array([k1, k2, 0., 0.])


def transform(pose, points):
    return Rotation.from_rotvec(pose[:3]).apply(points) + pose[3:]


def project(points, parameters):
    fx, fy, cx, cy, k1, k2 = parameters
    radius = np.linalg.norm(points[:, :2], axis=1)
    theta = np.arctan2(radius, points[:, 2])
    squared = theta ** 2
    factor = theta * (1 + squared * (k1 + squared * k2)) / np.maximum(radius, 1e-12)
    return points[:, :2] * factor[:, None] * [fx, fy] + [cx, cy]


def initial_pose(points, pixels, parameters):
    matrix, distortion = intrinsics(parameters)
    rays = cv2.fisheye.undistortPoints(pixels.reshape(-1, 1, 2), matrix, distortion).reshape(-1, 2)
    ok, rotation, translation, inliers = cv2.solvePnPRansac(points, rays, np.eye(3), None,
        iterationsCount=500, reprojectionError=4 / parameters[0], flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inliers is None or len(inliers) < 12:
        return None
    pose = np.r_[rotation.ravel(), translation.ravel()]
    if np.isfinite(pose).all() and np.min(transform(pose, points)[:, 2]) > 0:
        return pose
    return None


def fit(initial, observations):
    # Model parameters: six per lens, six for the stereo transform and six
    # per AprilGrid pose. Each residual depends only on its own pose/camera.
    def errors(parameters):
        for index, pair in enumerate(observations):
            pose = parameters[18 + 6*index:24 + 6*index]
            for side, (points, pixels) in enumerate(pair):
                rays = transform(pose, points)
                if side:
                    rays = transform(parameters[12:18], rays)
                yield project(rays, parameters[6*side:6*side + 6]) - pixels

    count = sum(pixels.size for pair in observations for _, pixels in pair)
    jacobian = lil_matrix((count, len(initial)), dtype=np.int8)
    start = 0
    for index, pair in enumerate(observations):
        for side, (_, pixels) in enumerate(pair):
            rows = slice(start, start + pixels.size)
            jacobian[rows, 6*side:6*side + 6] = 1
            if side:
                jacobian[rows, 12:18] = 1
            jacobian[rows, 18 + 6*index:24 + 6*index] = 1
            start += pixels.size
    lower = np.r_[[140, 140, 480, 250, -.5, -.18] * 2, [-np.inf] * (len(initial) - 12)]
    upper = np.r_[[600, 600, 800, 550, .5, .18] * 2, [np.inf] * (len(initial) - 12)]
    result = least_squares(lambda params: np.concatenate([error.ravel() for error in errors(params)]),
        initial, jac_sparsity=jacobian.tocsr(), loss="soft_l1", f_scale=.7,
        x_scale="jac", bounds=(lower, upper), max_nfev=200, ftol=5e-7, xtol=5e-7)
    if not result.success or not np.isfinite(result.x).all():
        raise ValueError(f"Calibration fit did not converge: {result.message}")
    return result.x, list(errors(result.x))


def solve(observations):
    # Nominal 0.76 mm focal length and 3 um pixels seed the lens fit.
    focal = .76 / .003
    nominal = np.array([focal, focal, 640., 400., 0., 0.])
    initialized, stereo_guesses = [], []
    cv2.setRNGSeed(4)
    for name, pair in observations:
        if min(len(points) for points, _ in pair) < 16:
            continue
        left, right = (initial_pose(points, pixels, nominal) for points, pixels in pair)
        if left is None or right is None:
            continue
        rotation = Rotation.from_rotvec(right[:3]) * Rotation.from_rotvec(left[:3]).inv()
        stereo_guesses.append(np.r_[rotation.as_rotvec(), right[3:] - rotation.apply(left[3:])])
        initialized.append((name, pair, left))
    if len(initialized) < 20:
        raise ValueError(f"Only {len(initialized)} pairs had usable poses; capture at least 20 sharp, varied pairs")

    training, held_out = [], []
    for index, entry in enumerate(initialized):
        (held_out if index % 6 == 2 else training).append(entry)
    pairs = [pair for _, pair, _ in training]
    initial = np.r_[nominal, nominal, np.median(stereo_guesses, axis=0),
                    np.concatenate([pose for _, _, pose in training])]
    print(f"Fitting {len(training)} pairs and holding out {len(held_out)} for validation", flush=True)
    parameters, errors = fit(initial, pairs)
    inliers = []
    for index, pair in enumerate(pairs):
        views = []
        for side, (points, pixels) in enumerate(pair):
            keep = np.linalg.norm(errors[2*index + side], axis=1) < 2
            views.append((points[keep], pixels[keep]))
        inliers.append(views)
    parameters, errors = fit(parameters, inliers)
    rms = float(np.sqrt(np.mean(np.sum(np.concatenate(errors) ** 2, axis=1))))
    if rms > 3:
        raise ValueError(f"Native reprojection RMS exceeds 3 px: {rms:.2f}")
    k1, d1 = intrinsics(parameters[:6])
    k2, d2 = intrinsics(parameters[6:12])
    rotation = Rotation.from_rotvec(parameters[12:15]).as_matrix()
    text, profile = rectified_profile(k1, d1, k2, d2, rotation, parameters[15:18] * 1000,
                                      {"stereo": rms}, max_horizontal_fov=110)

    residuals = []
    projection = np.array(profile["projection"][0])[:, :3]
    for _, pair, _ in held_out:
        rows = []
        for side, (points, pixels) in enumerate(pair):
            matrix, distortion = intrinsics(parameters[6*side:6*side + 6])
            rotation = np.array(profile["cameras"][side]["rectification"])
            rectified = cv2.fisheye.undistortPoints(pixels.reshape(-1, 1, 2), matrix, distortion,
                                                    R=rotation, P=projection).reshape(-1, 2)
            rows.append(dict(zip(map(tuple, points), rectified[:, 1])))
        residuals.extend(abs(rows[0][point] - rows[1][point]) for point in rows[0].keys() & rows[1].keys())
    if len(residuals) < 20 or not np.isfinite(residuals).all():
        raise ValueError("Too few shared corners in the validation set")
    p95 = float(np.quantile(residuals, .95))
    if p95 > 1:
        raise ValueError(f"Held-out vertical stereo error exceeds 1 rectified px at p95: {p95:.2f}")
    profile.update(source="4reel AprilGrid fit", fit_pairs=len(training), validation_pairs=len(held_out),
                   validation_corners=len(residuals), rectified_vertical_p95_px=p95)
    return text, profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--tag-mm", type=float, required=True, help="Measured black edge length of one tag")
    parser.add_argument("--output", type=Path, required=True, help="Output stem for the .txt profile and .json report")
    args = parser.parse_args()
    if not np.isfinite(args.tag_mm) or args.tag_mm <= 0:
        parser.error("Tag size must be positive")
    try:
        text, profile = solve(observe(args.input, args.tag_mm))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.with_suffix(".txt").write_text(text)
        args.output.with_suffix(".json").write_text(json.dumps(profile, indent=2) + "\n")
    except (ValueError, cv2.error, OSError) as error:
        parser.exit(1, f"Calibration failed: {error}\n")
    print(f"Saved {args.output.with_suffix('.txt')} with {profile['baseline_mm']:.2f} mm baseline")
    print(f"Native RMS {profile['rms']['stereo']:.2f} px, held-out vertical p95 {profile['rectified_vertical_p95_px']:.2f} px")


if __name__ == '__main__':
    main()
