import unittest
import numpy as np
import yaml
from import_calibration import convert, ORIENTATION


class ImportTests(unittest.TestCase):
    def fixture(self):
        camera = dict(camera_model="ds", distortion_model="none", distortion_coeffs=[],
                      resolution=[1280, 800], intrinsics=[0, .5, 400, 400, 640, 400])
        transform = np.eye(4)
        transform[0, 3] = -.07923
        return dict(cam0=dict(camera), cam1=dict(camera, T_cn_cnm1=transform.tolist()))

    def run_fixture(self, data, eye="left"):
        return convert(yaml.safe_dump(data).encode(), "TEST-ONLY", eye, ORIENTATION)

    def test_eye_direction(self):
        data = self.fixture()
        self.assertLess(float(self.run_fixture(data).splitlines()[-1].split()[0]), 0)
        self.assertGreater(float(self.run_fixture(data, "right").splitlines()[-1].split()[0]), 0)

    def test_rejections(self):
        for modify in [lambda d: d["cam0"].update(camera_model="pinhole"),
                       lambda d: d["cam0"].update(resolution=[320, 200]),
                       lambda d: d["cam0"].update(intrinsics=[0, .5, 0, 400, 640, 400]),
                       lambda d: d["cam0"].update(intrinsics=[float("nan"), .5, 400, 400, 640, 400]),
                       lambda d: d["cam1"]["T_cn_cnm1"][0].__setitem__(3, 79.23),
                       lambda d: d["cam1"]["T_cn_cnm1"][0].__setitem__(0, -1)]:
            data = self.fixture()
            modify(data)
            with self.assertRaises(ValueError):
                self.run_fixture(data)


if __name__ == "__main__":
    unittest.main()
