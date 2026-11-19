# R0.2 stereo ZipDepth development image

This image includes the actual official unfold-free checkpoint converted to
RV1103 RKNN, the matching 2.5.0 Mini Runtime, an RGA resize path, native capture,
and a per-unit metric estimator. **It is build-tested, not hardware-validated.**
It must not be described as a proven working 10-pair/s depth camera. No unit
calibration or fabricated measurements are included.

The original dashboard, focus view, exposure controls, accelerometer, and
approximate two-distance depth mode remain available. That dashboard's old
calibration is not a Kalibr calibration and is never used by this pipeline.
The new command owns both cameras while running. It is deliberately not started
automatically, to preserve dashboard access and permit calibration before depth.

## Operation on the board (requires hardware)

Connect through the existing USB network at `192.168.77.1`. Stop the dashboard
while the stereo command owns the devices:

```sh
/etc/init.d/S85fourreel-web stop
mkdir -p /root/calibration-pairs
4reel-stereo --unit YOUR_UNIT_ID --capture --count 20 --output /root/calibration-pairs
```

Move the rig/AprilGrid between captures; collecting multiple batches is fine.
Capture varied positions, ranges, orientations and image coverage. The output
space check refuses a batch that does not fit. Retrieve batches before filling
the small development rootfs. Native pairs are `left.pgm`, `right.pgm` (1280x800,
8-bit mono), plus `status.json`. RAW8 is the default. `--raw10` requests sensor
RAW10 and retains the high eight bits after compact CIF decoding; it does not
claim to preserve all ten bits in the PGM. The old dashboard retains RAW10.

Without calibration, genuine relative depth is available:

```sh
4reel-stereo --unit YOUR_UNIT_ID --count 10 --output /root/relative
```

An absent NPU, rejected model, bad ABI, failed RGA operation, lost/extra frame,
or inconsistent trigger timing fails the command. No synthetic inference is
substituted. Failures go to stderr and the process returns nonzero. Incomplete
directories start with `.pair-`; consumers must ignore them. Only completed
`pair-ID/` directories are published by atomic rename.

For calibrated output:

```sh
4reel-stereo --unit YOUR_UNIT_ID --calibration /etc/4reel/unit.ds \
  --count 10 --output /root/metric
/etc/init.d/S85fourreel-web start
```

Retrieve complete directories over SSH (no custom web API is required):

```sh
ssh root@192.168.77.1 'tar -C /root/metric -cf - pair-*' > metric-pairs.tar
```

Each depth result contains both eyes' `*-relative.pfm`, `*-z-metres.pfm`,
`*-support.pgm`, and a shared status file with pair ID, individual V4L2 sequence,
timestamp and flag values, trigger-command interval, real backend names,
calibration hash, inliers, independent fit parameters, confidence and timing.
`metric_valid` describes whether the estimator accepted that eye's fit; it is
not an accuracy certification. `hardware_accuracy_validated` is always false in
this development version. The command reports measured processing time only
when it actually executes on the device.

## Per-unit calibration

Use the images delivered by this exact capture path, not images independently
rotated by another tool. Physical LEFT is module 1, I2C address 0x60, CIF lvds1;
RIGHT is module 0, address 0x10, CIF lvds0. RIGHT retains the existing sensor-side
180-degree rotation. Both images retain the dashboard's software horizontal
flip. No camera swap is inferred from image content or USB enumeration.

On a workstation, using the Python dependencies below:

```sh
uv run --python "$work/venv/bin/python" os/tools/zipdepth/calibration_images.py \
  downloaded-pairs kalibr-images --unit YOUR_UNIT_ID
kalibr_bagcreater --folder kalibr-images --output-bag unit.bag
kalibr_calibrate_cameras --bag unit.bag --topics /cam0/image_raw /cam1/image_raw \
  --models ds-none ds-none --target measured-aprilgrid.yaml
uv run --python "$work/venv/bin/python" os/tools/zipdepth/import_calibration.py \
  unit-camchain.yaml unit.ds --unit YOUR_UNIT_ID --cam0-eye left \
  --orientation 'left=1,right=0,hflip=both,right_sensor_rotation=180'
```

Use the actual measured target dimensions, review Kalibr's report, and retain
the camchain/report with the unit record. `calibration_images.py` preserves each
eye's measured monotonic capture timestamps; they are not wall-clock/exposure
timestamps. `cam0` is physical left in these folders. If a genuine existing
camchain instead uses cam0=right, `--cam0-eye right` swaps intrinsics and inverts
`T_cn_cnm1`. It does not rotate/mirror an incompatible calibration for you.

Install the generated `unit.ds`, original camchain, and report on the same unit,
for example over SSH into `/etc/4reel/`. The importer validates DS-none, native
resolution, finite intrinsics, proper orthonormal rotation and a gross baseline
sanity interval of 40–120 mm. It binds the file to the supplied unit identifier
and SHA256 of the original camchain. The operator must associate that identifier
with the physical unit; it is not automatically authenticated against OTP.
The runtime refuses a mismatching identifier or malformed file. It uses the
calibrated baseline in metres, never the nominal 79.23 mm as a replacement.

## Geometry, masks and confidence

Coordinates are canonical delivered native pixel centres, x right, y down,
z forward along each eye's optical axis. The Kalibr transform maps LEFT to
RIGHT. Outputs are 320x200, mapping output centre `(x,y)` to native
`(4*x+1.5,4*y+1.5)`. PFM is little-endian float32, bottom row first.

Depth is optical Z in metres, not ray range. Zero is invalid in metric PFM.
Support mask 0 is invalid, 1 is inside the accepted anchors' convex hull and
prediction range in the central native rectangle `[256,1024] x [160,640]`, and
2 is extrapolation. Mask 1 means supported interpolation, not measured ground
truth at every pixel. Peripheral estimates are explicitly extrapolated, and
backward rays are invalid. Relative predictions retain the model's arbitrary
affine inverse-depth scale. Calibration missing, weak texture, insufficient
depth variation, or rejected geometry leaves metric maps entirely invalid.

The bounded CPU backend selects up to 96 grid-distributed corners, searches
128 DS epipolar depth hypotheses at quarter resolution, refines with pyramidal
LK through native resolution, and checks uniqueness, forward/backward error,
cheirality, reprojection error, ray separation and minimum parallax. Separate
deterministic robust affine fits map each eye's prediction to inverse Z. Fits
need at least 12 inliers, 60% support, three image quadrants, and adequate depth
and prediction spread. Confidence combines inlier fraction and residual RMS;
it is a heuristic, not a calibrated probability. RVE SAD remains in the old
dashboard. RVE corner/LK ABI/runtime behavior has not been established on this
board, so the new sparse path reports CPU, never RVE.

## Sensor, accelerator and ABI evidence

* [Linux ov9282](https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ov9282.c)
  supports OV9281 and selects true RAW8 using `0x030d=0x60`, `0x3662=0x07`;
  RAW10 uses `0x50`, `0x05`. Patch 0007 adds Y8 negotiation and both writes to
  the pinned vendor driver, preserving RAW10 and board rotation.
* [OmniVision OV9281 datasheet, section 4.8](https://w2.electrodragon.com/Chip-dat/OmniVision-dat/OV9281-dat/OV9281-DataSheet.pdf)
  describes snapshot wake on rising FSIN, `0x3030` bit 2, frame count `0x303f`,
  and low-power control `0x4f00`. The opt-in control `V4L2_CID_USER_BASE+0x10f0`
  enables one-frame snapshots, FSIN input, and each sensor's separate strobe
  output. It is grabbed during streaming and restored after capture.
* The repository R0.2 MPU schematic identifies PWM8_M1 as package pin 61. The
  pinned Rockchip `rv1106-pinctrl.dtsi` maps it to GPIO1_C4 mux 3. PWM8 is at
  `ff490000`. Capture discovers that PWM by DT identity, not a fixed chip index.
* One PWM burst has a 1 s repeat period and 10 us high time, and is disabled
  within 200 ms after one frame from each eye. Quiet-before/after, timestamp,
  count and skew checks reject ambiguous frames. Pair IDs identify commanded
  bursts; they are not a hardware edge counter. Scope verification of FSIN and
  both separate strobes is still required. Continuous PWM throughput and
  strobe GPIO timestamp capture are not implemented.
* [Double Sphere paper](https://arxiv.org/abs/1807.08957) and
  [Kalibr models](https://github.com/ethz-asl/kalibr/wiki/supported-models)
  define the unprojection/projection and `[xi,alpha,fx,fy,cx,cy]` ordering.
* [Rockchip RGA guide](https://github.com/airockchip/librga/blob/main/docs/Rockchip_Developer_Guide_RGA_EN.md)
  documents RV1103/1106 average downscale and no IOMMU. The helper uses
  contiguous RKNN DMA allocations, Y400-to-Y400 resize, explicit DMA buffer
  CPU access synchronization and 320-wide aligned strides. There is no RGB CSC
  or limited-range luma conversion. It pads 12 zero rows above/below the
  320x200 image. The CPU box-average reference is tested; actual RGA rounding
  and sample grid still need comparison on the board.
* Toolkit and Mini Runtime are both 2.5.0 from the same pinned SDK. The pinned
  kernel RKNPU driver is 0.9.8; Rockchip's 2.5.0 RV1103 quickstart recommends
  >=0.9.2. Runtime checks the actual version before inference. Native NHWC
  output and uint8 input use queried strides and zero-copy bindings, following
  Rockchip's RV1103 C example. Default RKNN cache synchronization stays enabled.

The SDK's ARM shared runtime needs glibc and its uClibc static archive references
uClibc ctype/time ABIs. Neither is linked into musl. The image instead contains a
separate ARM EABI helper with a private Debian glibc 2.36 loader/libc/libm and the
official C variant of librga. Its RPATH is `/opt/4reel-rknn/lib`; the system
loader and musl are untouched. QEMU loader checks resolve the full DSO chain
without contacting devices. This proves loading, not NPU/DMA operation.

One helper keeps one RKNN context and processes left then right. CPU sparse
matching overlaps helper preprocessing/inference over bounded pipes. The next
capture waits for completion; no unbounded frame queue exists. Compiler memory
planning reports 6720 KiB internal plus 6146.94 KiB weights. Additional live
allocations include two 1 MiB native image copies in each process, six capture
buffers (about 6 MiB RAW8/7.3 MiB compact RAW10), RGA source/destination (~1.1
MiB), tensor/output buffers, pyramids (~0.61 MiB), libc, kernel, and dashboard
memory if not stopped. CMA is 32 MiB of the 64 MiB RAM. Peak allocation and
fragmentation on real hardware remain acceptance gates; the compiler estimate
does not prove the whole application fits. ~10 metric pairs/s is only a goal.

## Model provenance and build

Official [fabiotosi92/ZipDepth](https://github.com/fabiotosi92/ZipDepth) revision
`91f3fd21e131641f51e8d35736d1958350180e3a`, checkpoint `zipdepth_base_npu.pth`,
SHA256 `627c04fda584133ead4310074884a4a037061b4c01ba86e73e492ea30fab570d`.
The repository MIT license covers the distributed checkpoint with no separate
checkpoint restriction found; it is retained on the image. RKNN SDK, RGA Apache
and glibc notices are also retained. This is a private development build; no
publication or hardware flashing is part of this workflow.

`export_fixed.py` strictly loads the actual weights and exports fixed
1x3x224x320 ONNX without upstream's learned-attention-to-average substitution.
It tests float equivalence. `convert.py` adds exact mono channel replication,
quantizes for `rv1103` (not RV1103B), and runs the genuine host simulator against
ONNX. Normalization is uint8/255 followed by the model's trained channel-specific
mean/std; it is not applied twice. The actual RKNN artifact is on the image at
`/usr/share/4reel-stereo/zipdepth-mono-rv1103.rknn`.

Calibration corpus: all 54 images of the [ETH3D low-res two-view training
set](https://www.eth3d.net/datasets), real grayscale stereo, resized by area
averaging and padded identically. Source names and SHA256s are in the model
manifest. These rectified images are not the target fisheye/sensor distribution;
they justify a provisional quantized artifact, not deployment accuracy. No
quantization images are redistributed on the image. Recalibrate quantization
and assess error with real target mono captures before deployment.

Use the repository's pinned Buildroot submodule and normal defconfig. Keep all
heavy outputs/caches in a private tmpfs work directory. The pinned build tools
and source IDs are in `prepare_bundle.py` and the manifest. Python environment
used: 3.11, torch 2.4.1+cpu, numpy 1.26.4, onnx 1.17.0, onnxsim 0.4.36,
onnxruntime 1.26.0, rknn-toolkit2 2.5.0, setuptools 75.8.0, opencv 4.11.0.

```sh
uv run --python "$work/venv/bin/python" os/tools/zipdepth/export_fixed.py \
  --upstream "$work/ZipDepth" --output "$work/zipdepth-fixed.onnx"
uv run --python "$work/venv/bin/python" os/tools/zipdepth/convert.py \
  --onnx "$work/zipdepth-fixed.onnx" --images "$work/eth3d" --output "$work/model-fixed"
uv run --python "$work/venv/bin/python" os/tools/zipdepth/prepare_bundle.py \
  --zipdepth "$work/ZipDepth" --rknn "$work/rknn-toolkit2" --rga "$work/librga" \
  --glibc "$work/glibc" --model "$work/model-fixed" --parity "$work/zipdepth-fixed.parity.json" \
  --output output/zipdepth-bundle --hash-list os/package/fourreel-stereo/bundle.sha256
make -C os/buildroot O="$work/build" BR2_EXTERNAL="$PWD/os" \
  BR2_DL_DIR="$work/dl" rv1103g_defconfig
make -C os/buildroot O="$work/build" BR2_DL_DIR="$work/dl" -j12
```

`glibc` is the extraction root of Debian `libc6-armhf-cross` and
`libc6-dev-armhf-cross` version `2.36-8cross1` from
`https://deb.debian.org/debian/pool/main/c/cross-toolchain-base/`.
The bundle is local and ignored; Buildroot checks its tracked SHA256 list before
compiling. Re-conversion may change RKNN serialization hashes; review changes
to the hash list and simulator results together. Missing bundles cause a build
error rather than silently producing a stock image.

## Hardware acceptance work still required

Verify actual boot, the two native streams and RAW8/RAW10 register readback;
scope shared FSIN and both strobes through startup, exposure changes and drops;
check pair-count/timestamp rejection under stress; compare RGA tensors against
the CPU reference; compare on-board RKNN output to the stored host reference;
measure CMA/RSS peaks, sustained FPS and temperature; capture/calibrate this
specific unit, and validate central metric accuracy against measured targets
across depth/texture/lighting and motion. Quantify peripheral extrapolation
separately. RVE corner/LK acceleration, continuous-trigger scheduling and tuning
to 10 pairs/s remain unverified/unimplemented optimizations.

## Low-load live preview and bottleneck diagnostics

For continuous operation without filling the SD card, stop the old dashboard,
then run (add the calibration option when the correct unit file is installed):

```sh
/etc/init.d/S85fourreel-web stop
4reel-stereo --unit YOUR_UNIT_ID --live \
  --calibration /etc/4reel/unit.ds
# Without a unit file, omit --calibration: output remains relative.
```

Open `http://192.168.77.1:8081/` on the connected workstation. The listener binds
all interfaces, like the existing local-device dashboard; use it on the trusted
device network. `--preview-port N` changes the port; `0` disables HTTP entirely.
The original dashboard is unchanged. Stop the stereo process before restarting
`S85fourreel-web`. `--live` does not write pair files and runs until interrupted;
use the original `--output DIR --count N` modes to save full maps/calibration
pairs. Do not combine `--live` with `--output` or `--capture`.

The default browser refresh is **1 Hz**. The server also caps new preview and
statistics generation to once per second across clients. Opening no viewer
means no image snapshots, encoding or preview traffic. Unticking refresh stops
requests; capture and inference continue. Each response embeds a small BMP
mosaic (160x100 per tile): physical left/right mono above their respective depth.
One decimal-string trigger ID covers all four tiles, status and timeline.
Metric output uses a fixed logarithmic 0.15–20 m colour range; relative maps use
independent per-eye ranges and have **no metric scale**. Black means invalid;
grey hatching identifies peripheral metric extrapolation. The existing full
PFM/support files remain the data interface for precise saved-map retrieval.

`GET /api/frame` returns the composite and telemetry together. It requests a
snapshot of the next completed pair and returns the previous completed snapshot
(or `ready:false` initially). This deliberate one-refresh delay avoids waiting
for capture. Input age is displayed, and IDs identify stale snapshots. The main
thread only **trylocks** the snapshot slot and copies buffers when requested;
it never waits for HTTP. A busy viewer causes a skipped preview copy, not a
camera-pipeline wait. The HTTP worker performs reduction, colour mapping, BMP
encoding, JSON/percentiles and socket work. It accepts one client at a time,
limits request reads to 0.5 s and sends to 2 s. Slow or excessive clients cannot
queue pair work. The bitmap response is about 260 KiB at 1 Hz. The preview/ring,
extra second-eye map and transient encoding allocations add approximately
4 MiB of userspace memory; **peak RAM and contention still require board tests**.

With diagnostics on, every processed pair enters a fixed **64-pair ring**, with
at most **64 timestamped events per pair**. Overflow is reported. Stage clocks
use host `CLOCK_MONOTONIC`; CPU time uses `CLOCK_THREAD_CPUTIME_ID` separately
for the main, glibc helper and HTTP threads. The helper response carries its
host timings alongside the same pair ID. Failed capture/inference attempts are
recorded before the existing fail-fast exit; stderr reports the failure, and
no fabricated image is published. The server stops with the process, so this
ring is intentionally not a persistent flight recorder.

The displayed pair includes capture/pairing (including quiet guards, PWM
submission, polling and per-eye dequeue/unpack); per-eye DMA source, RGA resize,
tensor/cache preparation, synchronous NPU call and output copy; sparse pyramid,
corners, epipolar/forward LK, reverse validation and triangulation; independent
fits and map generation; file output when selected; and snapshot/record cost.
Missing stages are unavailable, not zero-duration measurements. Rolling p50/p95
use nearest-rank quantiles over only pairs containing that stage. Repeated
calls within a stage (capture polls or file writes) are summed for that stage.
The ring resets when diagnostics mode changes to avoid mixing disabled intervals
into the rolling rate.

The timeline preserves concurrent scopes. Resource occupancy is the **union**
of intervals for that resource divided by pair wall duration, never the sum of
all stages. CPU scope duration includes scheduling; the separate thread CPU
column measures execution. RGA/NPU durations are **host-observed synchronous API
entry-to-return**, including submit/wait/cache overhead; internal device submit,
wait and completion counters are unavailable. The return is an observed
completion boundary, not a measured interrupt timestamp. RVE is explicitly
unused because sparse stereo uses the bounded CPU backend. The UI identifies
critical-path candidates from response-wait/NPU overlap and main-thread CPU
occupancy, and reports pipe waits; it does not claim device saturation from
those envelopes. Cold-start model initialization can appear as initial submit
backpressure. It is not mislabeled NPU inference time.

There is at most one pair in flight and no pending pair queue: the next trigger
follows completed processing. Submit/response `poll()` wait totals include
syscall overhead. V4L2 sequence gaps are reported per eye and over the ring;
these are observed gaps, not proof of every sensor-side dropped frame. Missing,
extra or ambiguous triggered frames still fail the pipeline. Preview skips are
reported separately from capture gaps. Throughput is completed pairs over the
observed ring wall interval. Input age uses the older monotonic V4L2 timestamp,
not an independently measured exposure time. Time to first/both captures uses
successful dequeue observations; time to first/both depth maps uses helper
output readiness, all relative to the host PWM command. First-eye readiness
does not mean that eye was transmitted before the second eye finished.

The UI toggle, `POST /api/diagnostics/off` / `on`, or startup
`--diagnostics-off` allows an on/off comparison while the pipeline continues.
Off removes stage tracing, ring copies and image snapshots/encoding; minimal
pair-boundary clocks, helper total CPU time and aggregate comparison counters
remain (plus operational timeout clocks and small control HTTP responses). Samples show pair throughput and
main/helper CPU cost per pair in each mode. Off/on groups are observational:
compare the same scene, calibration and settings; a scene change is a confounder.
Preview/HTTP elapsed and CPU costs refer to the preceding request so they never
claim to include their own future transmission. HTTP elapsed includes preview
work; do not add those fields. Trace-record/snapshot cost is shown separately.

### Host-only diagnostic validation

`test_stereo_trace.c` uses explicit known mock intervals to check overlap union,
nearest-rank p50/p95, missing stages, fixed-ring wrap, overflow, disabled tracing
and the musl/glibc protocol sizes. `test_stereo_preview.c` is a separate,
non-installed mock fixture checking matching trigger/eye content, bitmap and
validity encoding, demand/rate limits, nonblocking producer under a held mutex,
and HTTP on/off controls. Mock values are never linked into the production
runner and are not hardware performance evidence. The sparse numerical tests
also cover the staged implementation. The image inspection verifies the HTML,
updated ARM binaries, original services, private libraries and actual model.
Board boot, memory, FSIN, DMA/RGA/NPU operation, real timing and sustained
throughput remain unverified. Nothing in this UI changes that qualification.
