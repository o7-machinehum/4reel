# Independent board depth and fisheye calibration

Open `http://192.168.77.1:8080/` after booting the board. Each board operates
independently using its own two cameras.

The **Depth pipeline** selector offers Stereo, ZipDepth, or Stereo + ZipDepth.
Capture starts on demand. Focus mode keeps the existing full-resolution preview
and automatic exposure behaviour.

- Stereo uses the RVE SAD matcher, with CPU fallback. A full fisheye calibration
  rectifies both images and supplies the measured baseline and focal length for
  millimetre depth. The existing two-point calibration remains an approximate
  option before a fisheye profile is installed.
- ZipDepth runs one 256 × 160 inference at up to 2 Hz on the RV1103 NPU.
  The left perspective image comes directly from native RAW10. The worker
  publishes a 160 × 100 relative inverse-depth map and its source frame ID.
- Stereo + ZipDepth sends the same frame's stereo measurements to the worker.
  The worker fits the relative inverse depth to reliable stereo matches and
  fills holes in the supported range. The source preview marks each pixel as
  measured, estimated, or invalid. Weak agreement leaves the holes blank.

Before stereo calibration, neural inference uses an approximate 110°
forward perspective view for the fitted Arducam M32076M20 220° lens: nominal
0.76 mm focal length, 3 µm OV9281 pixels, centred equidistant projection. These
are lens-spec estimates, not measured intrinsics or stereo extrinsics. After
uploading calibration, the left neural view uses the measured geometry.
The dashboard shows the actual perspective input alongside each prediction.
Full fisheye previews remain available separately. This avoids feeding the
model a circular image with large black borders, but cannot guarantee depth
accuracy on every surface or recover unseen regions.
With neural inference active, automatic exposure meters the left perspective
input and the current right stereo view once per inference result. Both
cameras share exposure and gain, and small reflections can clip.

Black pixels indicate rejected stereo matches, nonpositive neural predictions,
or missing image coverage. Reflective and textureless surfaces and moving
objects can produce unreliable results. Check distances against a stationary
target before relying on the measurements. These cameras have independent
clocks; pairs more than 20 ms apart are rejected, but accepted pairs are not
hardware synchronized.

## Calibrate the fitted fisheye lenses

Use the native Python workflow with the [4reel AprilGrid](../../target.pdf).
Print at actual size and measure a black tag edge. The nominal tag edge is
30 mm with a 9 mm gap. Mount the paper flat, focus both cameras and hold the
camera assembly fixed. Move and tilt the target across the usable shared
view, holding it still for each capture.

From `os/`, run these commands on the computer connected to the board.

```sh
python3 -m venv .venv-fisheye
.venv-fisheye/bin/pip install numpy opencv-python scipy
.venv-fisheye/bin/python tools/calibrate_fisheye.py capture-aprilgrid --output aprilgrid-pairs --count 40
.venv-fisheye/bin/python tools/solve_aprilgrid.py --input aprilgrid-pairs --tag-mm 30 --output fisheye
.venv-fisheye/bin/python tools/calibrate_fisheye.py upload --input fisheye.txt
```

The solver also reads previously captured `cam0` and `cam1` PNG directories.
It fits the lens geometry to detected tag corners, checks stereo row alignment
on held-out pairs, and writes a board profile and a JSON report. With the
220° Arducam lenses it uses a rectilinear output capped at 110° to stay clear
of the heavily vignetted rim. Reprojection error and row alignment validate
the geometry on the captured dataset. Check distance against a stationary
target at a measured distance to assess depth accuracy.

## Calibrate with a checkerboard

Calibration depends on the actual lenses, focus settings and camera mounting.
No factory calibration is assumed. Focus both lenses first, keep the assembly
rigid, and recalibrate after changing focus or moving either camera.

On your computer, install Python 3, NumPy and OpenCV:

```sh
python3 -m venv .venv-fisheye
.venv-fisheye/bin/pip install numpy opencv-python
```

Use a flat, accurately printed checkerboard with 9 × 6 **inner corners**. Measure
one square in millimetres. Run these commands from `os/` (substitute the measured
square size for `24`):

```sh
.venv-fisheye/bin/python tools/calibrate_fisheye.py capture --output captures
.venv-fisheye/bin/python tools/calibrate_fisheye.py solve --input captures --square-mm 24 --output fisheye
.venv-fisheye/bin/python tools/calibrate_fisheye.py upload --input fisheye.txt
```

Capture collects 25 native 1280 × 800 stereo pairs. Keep the checkerboard still
when pressing Enter. Use varied distances and tilts and cover the centre and
edges of the shared view. Both cameras must see the complete checkerboard. Use
`--columns` and `--rows` on capture and solve for a different pattern. The tool
requires at least 20 usable poses and rejects reprojection RMS above 3 native
pixels. A low RMS alone does not establish accuracy outside the sampled area.

The solver writes `fisheye.txt` for the board and `fisheye.json` with the full
camera geometry, measured RMS, rectification matrices, projection matrices and
Q matrix. It chooses a centred common forward view, capped at 150 degrees
horizontally, whose pixels fit both native images. The selected field of view
is printed after solving. A rectilinear stereo map cannot retain an entire
180-degree or wider fisheye view.

Upload stops capture and saves `/etc/4reel-web.fisheye` across reboots. You can
also upload the `.txt` file on the dashboard's Calibration tab. Start depth
capture again after upload. Verify several stationary targets at measured
distances, particularly around the edges. The 160 × 100 output and integer
disparity matcher limit distance resolution. The metric image is camera-axis
Z depth in millimetres, rather than radial distance from the lens.

## APIs

Control POSTs require `X-4reel-control: 1`. Examples:

```sh
curl -H 'X-4reel-control: 1' -d 'enabled=1&mode=depth&pipeline=both' http://192.168.77.1:8080/api/capture
curl http://192.168.77.1:8080/api/status
curl -o depth.pgm http://192.168.77.1:8080/api/depth.pgm
```

`/api/neural-left.bmp` exposes the relative inverse-depth preview.
`/api/neural-input-left.bmp` shows the left prediction's source image.
The status `neural` object reports availability, source frame, single-image
inference time and errors. The `fusion` object reports stereo and estimated
pixel counts and scale-fit support for that same frame. Use `/api/fused.bmp`
for the fused depth and source preview and `/api/fused.pgm` for 16-bit depth
in millimetres (0 is invalid). In combined mode, NPU initialization failure leaves stereo
available and displays the error. ZipDepth-only mode reports capture failure
if its model cannot initialize.

`/api/calibration/pair.bin` is available in `mode=calibration`. It contains a
40-byte little-endian header (`<8sIIQQd`): magic `4REELC1\0`, width, height,
frame ID, monotonic capture timestamp in microseconds, and camera skew in ms.
Two native 8-bit paletted BMP images follow, left then right. Each is
1,025,078 bytes. The calibration tool decodes this format; both images come
from the same accepted pair.

`/api/frame.bin` is an atomic depth capture export, with this little-endian header:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | 8 bytes | `4REELD1\0` |
| 8 | uint32 | Version 1 |
| 12, 16 | uint32 | Width 160, height 100 |
| 20 | uint32 | Flags: bit 0 calibrated fisheye metric stereo, bit 1 neural available, bit 2 rectified, bit 3 neural uses estimated lens perspective, bit 4 neural data is left-only |
| 24, 32, 40 | uint64 | Stereo frame ID, capture timestamp (µs), neural source frame ID |
| 48, 56, 64, 72 | float64 | Rectified fx, fy, cx, cy; valid when bit 2 is set |

After the 80-byte header: left then right uint8 images (16,000 bytes each),
left uint16 stereo Z depth in mm (32,000 bytes, 0 invalid), then left and right
float32 neural inverse-depth planes (64,000 bytes each). The reserved right
plane is zero when bit 4 is set. Total: 192,080 bytes.
Check the flags before using metric or neural arrays. Neural results can be
older than the stereo pair; their frame ID is explicit. Timestamps are local
to this board and boot. This format requires no connection to another board.
When bit 3 is set, neural arrays cover the estimated forward perspective view
and have different coordinates from the full fisheye stereo images. Use the
neural input endpoints to inspect that view; do not combine those pixels with
the uncalibrated stereo images as though they shared a projection.

## Profiling

The Profiling tab shows one-second rates and per-call times. `/api/profile`
returns the underlying cumulative counters and CPU and elapsed milliseconds.
CPU time is measured on the thread that performs the work. Elapsed time
includes blocking and scheduling. The NPU driver reports load through
`/proc/rknpu/load` and the RKNN call duration includes the driver wait.
RVE reports timed SAD and cache-sync calls. The stereo total includes both.
The kernel configuration disables CPU hardware performance counters.

## Model and validation

The installed model and provenance are in `/usr/share/4reel-web/`. The package
pins RKNN toolkit/runtime 2.5.0 and the upstream ZipDepth checkpoint. The model
is INT8 quantized using 50 grayscale frames. `tools/export_zipdepth.py` records
checksums and verifies the ONNX export against the trained PyTorch model before
quantization. Its pooling reformulations preserve the learned weighted
attention. Quantization and monochrome fisheye input can affect accuracy;
hardware accuracy and throughput must be checked on the booted board.

The kernel reserves 4 MiB of default CMA for RVE and fallback DMA, plus 24 MiB
for the Rockchip heap shared by CIF camera buffers and the NPU
(`rk_dma_heap_cma=24M`). This fits native stereo capture and the neural model
within the board's 64 MiB RAM. Increasing only the default CMA pool can leave
the vendor heap's separate 32 MiB default reservation unavailable, causing
the NPU driver probe to fail.
The runtime package adapts the vendor archive's 32-bit file-offset calls to
musl's 64-bit ABI so DMA mappings use the correct offset.

Run host regression checks after building the RKNN package (which stages its
header). The PC tests need NumPy and OpenCV:

```sh
FOURREEL_TEST_PYTHON=.venv-fisheye/bin/python bash package/fourreel-web/tests/run.sh
```

The tests cover native RAW10 conversion, calibration persistence, accepted
stereo pairing, signed metric disparity, the neural worker with padded tensors,
synthetic fisheye calibration, Kalibr YAML and double-sphere geometry, and
candidate-window masking before disparity and uniqueness selection.
On the board, `4reel-web --self-test` checks stereo math and
`4reel-web --sad-self-test` checks the RVE hardware. Start ZipDepth capture and
inspect `/api/status` to test the NPU; host emulation cannot validate it.

Model sources: [ZipDepth](https://github.com/fabiotosi92/ZipDepth),
[RKNN toolkit/runtime](https://github.com/airockchip/rknn-toolkit2), and
[OpenCV fisheye calibration](https://docs.opencv.org/4.11.0/db/d58/group__calib3d__fisheye.html).
Lens and sensor specifications: [Arducam M32076M20](https://www.arducam.com/m32076m20-2.html)
and [OMNIVISION OV9281](https://www.ovt.com/products/ov9281/).
