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
calibration hash, inlier count, shared scale factor, scale source pair, scale age and timing.
`metric_valid` describes whether that eye's full map carries metre values; it is
not an accuracy certification. `hardware_accuracy_validated` is always false in
this development version. The command reports measured processing time only
when it actually executes on the device.

On each inference pair the main thread matches the central overlap while the
helper runs ZipDepth. Without unit calibration it compares mean-normalized
patches around grid corners in both eyes and requires a unique match in both
directions. These image correspondences have no depth units. The calibrated
path instead searches the calibrated epipolar curve, refines and checks
matches, triangulates, and estimates one shared scale for the pair. Live
mode loads `/etc/4reel/unit.ds` at startup if present, checking the unit ID.
Install the imported camchain for the physical unit and restart the service.

The **CPU main · stereo** timeline lane shows the stages executed on each pair.
The table reports wall time, main-thread CPU, and rolling p50/p95 for each
stage. The stereo summary gives point counts through rejection and elapsed
overlap with the NPU host calls. Circles on the 1 Hz grayscale preview label
corresponding points from that exact pair. The binary grayscale payload is
unchanged. Its optional `X-Stereo-Matches` header contains space-separated
`left_x,left_y,right_x,right_y` coordinates in logical native orientation.

Live mode meters 640 central raw samples per eye on every synchronized pair.
It commands the same exposure and analogue gain on both sensors between
triggers. The dashboard shows the previous command in sensor lines and gain,
the measured per-eye brightness and clipping, and the metering and control-write
times. Batch capture preserves its existing exposure configuration. The four
live canvases rotate 180° on the viewer. Raw stream and calibration coordinates
keep their native orientation.

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

Rectilinear units may instead use a measured pinhole-radtan calibration. The
runtime identifies the model using the file header, including at the default
live path `/etc/4reel/unit.ds`. A `4REEL_PINHOLE1` file uses the same unit,
source SHA256, orientation, and left-to-right extrinsics as `4REEL_DS1`. Each
intrinsics row is `fx fy cx cy k1 k2 p1 p2` for a 1280 × 800 native image.
The 180° browser rotation is display-only. Compute projections from the recorded
camera pixels before this browser transform.

## Geometry, masks and confidence

Coordinates are canonical delivered native pixel centres, x right, y down,
z forward along each eye's optical axis. The Kalibr transform maps LEFT to
RIGHT. Outputs are 320x200, mapping output centre `(x,y)` to native
`(4*x+1.5,4*y+1.5)`. PFM is little-endian float32, bottom row first.

Depth is optical Z in metres, not ray range. Zero is invalid in metric PFM.
Each scaled pixel has mask value 1. Relative eyes have mask value 0. The matcher
selects one corner per cell in a 5 by 4 grid in the central overlap.
After a failed match or triangulation it tries a different textured quarter
of that cell on the next frame. When all textured quarters have been tried,
the cell starts again with its strongest corner. It searches
at most 20 points per pair, with bidirectional normalized patch correlation,
a local 3×3 peak search after the uniqueness check, and subpixel refinement.
Triangulation takes the midpoint between the closest points on the two rays
and checks that it reprojects within 1.5 native pixels in each eye and that the rays pass within 10 mm.

For each triangulated point, sample each eye's relative inverse depth at its
subpixel coordinates. Multiplying this sample by measured optical Z gives one
scale estimate per eye. Give the physical point one vote using the geometric
mean of its two estimates. Take the median in log space. Accept this scale
when at least three distinct points and a strict majority of usable points
agree within a factor of 1.25 in both eyes. Apply the same positive factor to
both full maps: metres = factor / relative inverse depth. The multiplier
preserves ZipDepth's depth ratios. Its accuracy depends on that shape.

Until the first accepted fit, publish the relative maps. On later rejections
reuse the last accepted scale and report its source pair and age at capture.
The scale resets on process restart. An eye with predictions that cannot be
converted to finite positive metres is published in full relative units.

RVE SAD remains in the old
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

RAW8 inference exports the six CIF buffers with `VIDIOC_EXPBUF` before the
helper starts. Each trigger hands the helper the two buffer descriptors and
row strides. Capture holds both buffers until the helper has returned and
CPU consumers have finished reading them. RGA mirrors each native Y400 plane
into a shared contiguous scratch buffer and averages it down to 320×200.
Separate mirror and resize calls preserve the original input sampling. On the
RV1103G1 bench, fusing them changed the tensor. The two-pass implementation
matched every legacy prediction value for both eyes across three real pairs.

The inference thread owns one RKNN context. Once the left input is ready,
a worker prepares the right eye during the left NPU call. The worker has a
separate resized plane and padded staging area. The inference thread copies
that staging area into the bound RKNN input after the left run completes.
Trace events record each RGA call, and helper thread CPU includes the worker.
Calibrated sparse stereo reads the original captured rows in calibration
coordinates using their stride and horizontal direction. RAW10 and native
calibration capture retain the CPU unpack/copy path.

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
then run (live mode also discovers the unit-matched calibration at this path):

```sh
/etc/init.d/S85fourreel-web stop
4reel-stereo --unit YOUR_UNIT_ID --live \
  --calibration /etc/4reel/unit.ds
# With no unit file, omit --calibration for image-space matches and relative depth.
```

Open `http://192.168.77.1:8081/` on the connected workstation. The listener binds
all interfaces, like the existing local-device dashboard; use it on the trusted
device network. `--preview-port N` changes the port; `0` disables HTTP entirely.
The original dashboard is unchanged. Stop the stereo process before restarting
`S85fourreel-web`. `--live` does not write pair files and runs until interrupted;
use the original `--output DIR --count N` modes to save full maps/calibration
pairs. Do not combine `--live` with `--output` or `--capture`.

The dashboard displays both 320×200 depth maps at the completed-pair rate.
The two native 1280×800 grayscale views and timing diagnostics refresh at
1 Hz. Each section shows its source pair ID. Both eyes within a section share
one camera trigger. The browser reports the displayed depth rate and skipped
pair IDs. On the Ethernet bench configuration, open
`http://192.168.78.1:8081/`.

The browser requests `/api/depth?after=ID` and `/api/gray?native=1&after=ID`, starting
at ID zero. A 204 response means there is no newer image available. A 200
response carries the decimal `X-Pair-Id` and left then right pixel arrays in
top-down row order. Depth uses little-endian float32 at 320×200 per eye.
`X-Metric-Eyes` is a bit mask (left 1, right 2). If either eye is scaled into
metres, the float arrays are followed by two uint8 support masks. Grayscale
uses one byte per pixel. With `native=1`, apply a horizontal mirror to each eye
when the response has `X-Horizontal-Flip: 1`. The existing
`/api/gray?after=ID` endpoint returns the original display orientation.
The browser colour-maps the depth data. Relative
predictions use independent ranges for each eye. Metric depth uses a fixed
logarithmic 0.30–20 m colour range. Distances outside the legend retain their
numeric values. Depth packet headers include X-Stereo-Scale, X-Scale-Source-Pair,
X-Scale-Age-Ms and X-Scale-Inliers. Source pair zero means no scale is available.

At four relative-depth pairs/s plus one grayscale pair/s the image payload is
about 33 Mbit/s. A separate sender handles the larger grayscale transfers.
The capture thread uses nonblocking locks and publishes the latest completed
pair. A slow viewer can skip pairs. Image copies cease when viewer demand
expires, and the pipeline continues when the page is paused or hidden.

The dashboard requests `/api/stats` for the timing sample. Its one-refresh
delay avoids waiting for capture. The older compact BMP mosaic and timing
response is available at `/api/frame`.

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
corners, epipolar patch search, reverse validation and triangulation; independent
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
