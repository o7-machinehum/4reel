"""Check Kalibr import, DS inverse geometry, units, camera order and C profiles."""
import ctypes as ct
from pathlib import Path
import sys
import tempfile
import subprocess

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import kalibr_fisheye as tool

class Camera(ct.Structure):
    _fields_ = [(name, ct.c_double) for name in ("fx", "fy", "cx", "cy")] + [
        ("distortion", ct.c_double * 4), ("rotation", ct.c_double * 9),
        ("xi", ct.c_double), ("alpha", ct.c_double), ("model", ct.c_int)]

class Calibration(ct.Structure):
    _fields_ = [("camera", Camera * 2)] + [(name, ct.c_double) for name in (
        "fx", "fy", "cx", "cy", "baseline_mm", "rms")] + [("valid", ct.c_int)]

library = ct.CDLL(sys.argv[1])
library.fisheye_parse.argtypes = [ct.c_char_p, ct.POINTER(Calibration), ct.c_char_p, ct.c_size_t]
library.fisheye_build_map.argtypes = [ct.POINTER(Calibration), ct.c_int, ct.c_uint, ct.c_uint, ct.c_void_p]

def chain_for(direction):
    rotation, _ = cv2.Rodrigues(np.array([.002, .01, -.003]))
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = [direction * -.04, .0002, .0001]
    chain = {f"cam{side}": {"camera_model": "ds", "intrinsics": [-.18, .59, 313., 312., 640., 400.],
        "distortion_model": "none", "distortion_coeffs": [], "resolution": [1280, 800],
        "rostopic": f"/cam{side}/image_raw"} for side in (0, 1)}
    chain["cam1"]["T_cn_cnm1"] = transform.tolist()
    return chain

for direction in (-1, 1):
    chain = chain_for(direction)
    text, metadata = tool.convert_camchain(chain)
    calibration, error = Calibration(), ct.create_string_buffer(160)
    assert library.fisheye_parse(text.encode(), ct.byref(calibration), error, len(error)) == 0, error.value
    assert calibration.valid and calibration.rms == -1 and metadata["rms_pixels"] is None
    assert abs(calibration.baseline_mm - direction * np.linalg.norm([40, .2, .1])) < 1e-6
    for side in (0, 1):
        camera = calibration.camera[side]
        assert camera.model == 1 and camera.xi == -.18 and camera.alpha == .59
        for width, height in ((160, 100), (256, 160)):
            native = np.empty((height, width, 2), np.uint16)
            library.fisheye_build_map(ct.byref(calibration), side, width, height, native.ctypes.data)
            valid = native[:, :, 0] != 65535
            assert valid.mean() > .99
            # Independently invert DS projection (closed-form unprojection),
            # rotate back to the virtual camera and recover the pixel grid.
            mx = (1279 - native[:, :, 0][valid] / 16 - camera.cx) / camera.fx
            my = (native[:, :, 1][valid] / 16 - camera.cy) / camera.fy
            r2 = mx*mx + my*my
            mz = (1-camera.alpha**2*r2) / (camera.alpha*np.sqrt(1-(2*camera.alpha-1)*r2) + 1-camera.alpha)
            scale = (mz*camera.xi + np.sqrt(mz*mz+(1-camera.xi**2)*r2)) / (mz*mz+r2)
            rays = np.stack((scale*mx, scale*my, scale*mz-camera.xi), axis=-1)
            rays = rays @ np.array(camera.rotation).reshape(3, 3).T
            recovered = np.stack((calibration.fx*rays[:, 0]/rays[:, 2] + calibration.cx,
                                  calibration.fy*rays[:, 1]/rays[:, 2] + calibration.cy), axis=-1)
            yy, xx = np.mgrid[:height, :width]
            expected = np.stack((xx * 160/width, yy * 100/height), axis=-1)[valid]
            np.testing.assert_allclose(recovered, expected, atol=.13, rtol=0)

    # Equidistant chains remain supported without approximating DS parameters.
    for side in (0, 1):
        chain[f"cam{side}"].update(camera_model="pinhole", distortion_model="equidistant",
            intrinsics=[430., 428., 640., 400.], distortion_coeffs=[-.03, .005, -.001, .0002])
    text, metadata = tool.convert_camchain(chain, .4)
    assert library.fisheye_parse(text.encode(), ct.byref(calibration), error, len(error)) == 0, error.value
    assert calibration.rms == .4 and calibration.camera[0].model == 0

bad_inputs = []
for key, value in (("camera_model", "omni"), ("resolution", [640, 400]),
                   ("rostopic", "/wrong_camera"), ("intrinsics", [0., 1.1, 313, 313, 640, 400])):
    bad = chain_for(1)
    bad["cam0"][key] = value
    bad_inputs.append(bad)
bad = chain_for(1)
bad["cam1"]["T_cn_cnm1"][0][0] = -1
bad_inputs.append(bad)
bad = chain_for(1)
bad["cam1"]["T_cn_cnm1"][0][3] = 0
bad["cam1"]["T_cn_cnm1"][1][3] = 0
bad["cam1"]["T_cn_cnm1"][2][3] = 0
bad_inputs.append(bad)
for bad in bad_inputs:
    try:
        tool.convert_camchain(bad)
        raise AssertionError("Accepted invalid camera chain")
    except ValueError:
        pass

import yaml
with tempfile.TemporaryDirectory(prefix="4reel-kalibr-import-") as directory:
    folder = Path(directory)
    chain_file = folder / "camchain.yaml"
    chain_file.write_text(yaml.safe_dump(chain_for(1)))
    result = subprocess.run([sys.executable, str(Path(tool.__file__)), "import", "--camchain",
        str(chain_file), "--output", str(folder / "profile")], capture_output=True, text=True, check=True)
    assert "RMS is unknown" in result.stdout
    assert (folder / "profile.txt").read_text().startswith("4REELFISHEYE2")
print("PASS: Kalibr YAML import, DS inverse projection, signed metre/mm conversion, camera order and invalid-chain rejection")
