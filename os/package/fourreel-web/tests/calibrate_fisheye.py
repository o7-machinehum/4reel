"""Synthetic stereo calibration and independent OpenCV/C remapping check."""
import ctypes as ct
import importlib.util
from pathlib import Path
import struct
import sys

import cv2
import numpy as np

spec = importlib.util.spec_from_file_location("calibration_tool", Path(__file__).resolve().parents[3] / "tools/calibrate_fisheye.py")
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

grid = np.zeros((1, 54, 3), np.float64)
grid[0, :, :2] = np.mgrid[:9, :6].T.reshape(-1, 2) * 24
matrix = np.array([[430., 0, 640], [0, 428, 400], [0, 0, 1]])
distortion = np.array([-.03, .005, -.001, .0002], np.float64)
rotation, _ = cv2.Rodrigues(np.array([.002, .01, .003]))
translation = np.array([[-40.], [.2], [.1]])
rng = np.random.default_rng(1234)
objects, left, right, poses = [], [], [], []
for _ in range(35):
    angle = rng.uniform([-.7, -.7, -.3], [.7, .7, .3])
    pose, _ = cv2.Rodrigues(angle)
    position = np.array([rng.uniform(-300, 100), rng.uniform(-220, 100), rng.uniform(330, 850)])
    points_left, _ = cv2.fisheye.projectPoints(grid, angle, position, matrix, distortion)
    right_angle, _ = cv2.Rodrigues(rotation @ pose)
    points_right, _ = cv2.fisheye.projectPoints(grid, right_angle, rotation @ position.reshape(3, 1) + translation, matrix, distortion)
    objects.append(grid.copy())
    left.append(points_left)
    right.append(points_right)
    poses.append((pose.copy(), position.copy()))

text, profile = tool.solve_points(objects, left, right)
assert abs(profile["baseline_mm"] - np.linalg.norm(translation)) < .01
assert max(profile["rms"].values()) < .001
assert 30 < profile["horizontal_fov_degrees"] < 150
# Check epipolar alignment and the metric disparity scale independently.
rectified = []
for side, camera in enumerate(profile["cameras"]):
    rectified.append(cv2.fisheye.undistortPoints((left, right)[side][0],
        np.array(camera["K"]), np.array(camera["D"]),
        R=np.array(camera["rectification"]), P=np.array(profile["projection"][side])).reshape(-1, 2))
np.testing.assert_allclose(rectified[0][:, 1], rectified[1][:, 1], atol=1e-6, rtol=0)
disparity = rectified[0][:, 0] - rectified[1][:, 0]
reprojected = np.column_stack((rectified[0], disparity, np.ones(len(disparity)))) @ np.array(profile["Q"]).T
reprojected = reprojected[:, :3] / reprojected[:, 3:]
expected = (grid.reshape(-1, 3) @ poses[0][0].T + poses[0][1]) @ np.array(profile["cameras"][0]["rectification"]).T
np.testing.assert_allclose(reprojected, expected, atol=1e-3, rtol=0)
try:
    tool.solve_points(objects[:3], left[:3], right[:3])
    raise AssertionError("Accepted insufficient calibration poses")
except ValueError:
    pass

class Camera(ct.Structure):
    _fields_ = [("fx", ct.c_double), ("fy", ct.c_double), ("cx", ct.c_double), ("cy", ct.c_double),
                ("distortion", ct.c_double * 4), ("rotation", ct.c_double * 9),
                ("xi", ct.c_double), ("alpha", ct.c_double), ("model", ct.c_int)]

class Calibration(ct.Structure):
    _fields_ = [("camera", Camera * 2), ("fx", ct.c_double), ("fy", ct.c_double),
                ("cx", ct.c_double), ("cy", ct.c_double), ("baseline_mm", ct.c_double),
                ("rms", ct.c_double), ("valid", ct.c_int)]

library = ct.CDLL(sys.argv[1])
library.fisheye_parse.argtypes = [ct.c_char_p, ct.POINTER(Calibration), ct.c_char_p, ct.c_size_t]
library.fisheye_build_maps.argtypes = [ct.POINTER(Calibration), ct.c_void_p]
library.fisheye_build_map.argtypes = [ct.POINTER(Calibration), ct.c_int, ct.c_uint, ct.c_uint, ct.c_void_p]
library.fisheye_rectify_raw10.argtypes = [ct.c_void_p, ct.c_int, ct.c_void_p, ct.c_uint, ct.c_void_p]
calibration, error = Calibration(), ct.create_string_buffer(160)
assert library.fisheye_parse(text.encode(), ct.byref(calibration), error, len(error)) == 0, error.value
maps = np.empty((2, 100, 160, 2), np.uint16)
library.fisheye_build_maps(ct.byref(calibration), maps.ctypes.data)
for side, camera in enumerate(profile["cameras"]):
    mx, my = cv2.fisheye.initUndistortRectifyMap(np.array(camera["K"]), np.array(camera["D"]),
        np.array(camera["rectification"]), np.array(profile["projection"][side]),
        tool.OUTPUT_SIZE, cv2.CV_32FC1)
    valid = maps[side, :, :, 0] != 65535
    assert valid.mean() > .9
    np.testing.assert_allclose(maps[side, :, :, 0][valid] / 16, 1279 - mx[valid], atol=.064, rtol=0)
    np.testing.assert_allclose(maps[side, :, :, 1][valid] / 16, my[valid], atol=.064, rtol=0)
    neural_map = np.empty((160, 256, 2), np.uint16)
    library.fisheye_build_map(ct.byref(calibration), side, 256, 160, neural_map.ctypes.data)
    projection = np.array(profile["projection"][side])
    projection[:2] *= 1.6
    nx, ny = cv2.fisheye.initUndistortRectifyMap(np.array(camera["K"]), np.array(camera["D"]),
        np.array(camera["rectification"]), projection, (256, 160), cv2.CV_32FC1)
    valid = neural_map[:, :, 0] != 65535
    assert valid.mean() > .9
    np.testing.assert_allclose(neural_map[:, :, 0][valid] / 16, 1279 - nx[valid], atol=.064, rtol=0)
    np.testing.assert_allclose(neural_map[:, :, 1][valid] / 16, ny[valid], atol=.064, rtol=0)

# An independent bilinear reference checks all four RAW10 packing positions,
# horizontal correction, fractional map coordinates and native row padding.
yy, xx = np.mgrid[:800, :1280]
samples = ((xx * 3 + yy * 7) % 1024).astype(np.uint64)
groups = samples.reshape(800, 320, 4)
packed = sum(groups[:, :, i] << (10 * i) for i in range(4))
raw = np.full((800, 1632), 0xcc, np.uint8)
raw[:, :1600] = np.stack([(packed >> (8 * i)).astype(np.uint8) for i in range(5)], axis=-1).reshape(800, 1600)
image = np.empty((100, 160), np.uint8)
library.fisheye_rectify_raw10(maps.ctypes.data, 0, raw.ctypes.data, raw.shape[1], image.ctypes.data)
valid = maps[0, :, :, 0] != 65535
x = (maps[0, :, :, 0][valid] // 16).astype(int)
y = (maps[0, :, :, 1][valid] // 16).astype(int)
dx, dy = maps[0, :, :, 0][valid] % 16, maps[0, :, :, 1][valid] % 16
native = (samples // 4).astype(np.uint32)
expected = ((native[y, x] * (16 - dx) + native[y, x + 1] * dx) * (16 - dy) +
            (native[y + 1, x] * (16 - dx) + native[y + 1, x + 1] * dx) * dy + 128) // 256
np.testing.assert_array_equal(image[valid], expected)
assert not image[~valid].any()

images = [np.full((800, 1280), value, np.uint8) for value in (23, 77)]
payload = struct.pack("<8sIIQQd", b"4REELC1\0", 1280, 800, 42, 1234567, 10.)
for source in images:
    ok, bmp = cv2.imencode(".bmp", source)
    assert ok
    payload += bmp.tobytes()
decoded, metadata = tool.decode_pair(payload)
assert metadata["frame"] == 42 and metadata["timestamp_us"] == 1234567
for source, image in zip(images, decoded):
    np.testing.assert_array_equal(source, image)
for bad in (payload[:20], payload[:-1], b"BADMAGIC" + payload[8:]):
    try:
        tool.decode_pair(bad)
        raise AssertionError("Accepted invalid pair packet")
    except ValueError:
        pass
print("PASS: 35-pose synthetic fisheye calibration, metric baseline, OpenCV/C map agreement and atomic pair decoding")
