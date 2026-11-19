# Calibrating one 4reel stereo board with Kalibr

Kalibr runs on your computer. The board supplies native, unrectified camera
pairs; no ROS installation is required on the board. Use camera calibration
with two cameras. The capture below is not suitable for camera/IMU timing
calibration.

For the fitted 220° Arducam lenses, start with Kalibr's **ds-none** model
(double sphere with no additional distortion). The importer preserves that
model directly. It also supports `pinhole-equi` camera chains; it rejects other
models instead of approximating their parameters. The model still needs a
good calibration dataset and a satisfactory Kalibr report.

## Prepare Kalibr and a target

Use a working [Kalibr installation](https://github.com/ethz-asl/kalibr/wiki/installation).
Upstream also provides `Dockerfile_ros1_20_04`. If using that container, mount
this `os/` directory at `/data` and source `/catkin_ws/devel/setup.bash` before
running the Kalibr commands below. These commands assume Kalibr tools are on
your PATH; they do not install Kalibr automatically.

Kalibr recommends an Aprilgrid: it disambiguates target orientation and can
detect a partially visible target. Generate the grid in your Kalibr environment:

```sh
kalibr_create_target_pdf --type apriltag --nx 5 --ny 5 --tsize 0.03 --tspace 0.3 --tfam t36h11 target
```

Print `target.pdf` at **100% / actual size**, mount it on a flat, rigid board,
and measure the black outer edge of several tags. The supplied
[`kalibr-aprilgrid.yaml`](../../tools/kalibr-aprilgrid.yaml) assumes 30 mm tags
and 9 mm gaps. Update `tagSize` in metres to the actual printed measurement;
`tagSpacing` is gap/tag size, rather than a measurement in metres. Do not use
fit-to-page printing and assume its nominal scale is correct.

Focus both lenses before capturing and keep the camera assembly rigid. Use
even light, avoid glare on the printed target, and keep other AprilTags out
of view. Do not refocus or move either camera relative to the other after
calibration.

## Capture native stationary pairs

On your computer, from `os/`:

```sh
python3 -m venv .venv-fisheye
.venv-fisheye/bin/pip install numpy opencv-python PyYAML
.venv-fisheye/bin/python tools/kalibr_fisheye.py capture --output kalibr-captures --count 40
```

Keep the board fixed. At each prompt, position the target, hold it still,
and press Enter. Vary distance, position and tilt, including the centre and
edges of the usable shared view. Include plenty of poses visible in both
cameras. Inspect the saved images and repeat the capture if they are blurred,
glared out, or mostly miss the target; detection happens later in Kalibr.

This creates `kalibr-captures/cam0/TIMESTAMP.png` for the left camera and
`cam1/TIMESTAMP.png` for the right camera, plus capture metadata. Images are
1280 × 800 monochrome, with native geometry and the existing preview gamma.
They retain the fisheye image and dark rim for calibration. Stereo matching's
rim mask is not applied to calibration images.

The cameras have independent clocks. The board rejects pairs over 20 ms
apart. The exported images share the pair midpoint timestamp, so this
workflow requires **stationary poses**, not a moving target or continuous
visual/inertial calibration. Actual camera skew is saved in `capture.json`.
Capture mode stops when the tool finishes or is interrupted.

## Make the bag and solve

In your Kalibr environment, with the same files available:

```sh
kalibr_bagcreater --folder kalibr-captures --output-bag stereo.bag
kalibr_calibrate_cameras --bag stereo.bag --topics /cam0/image_raw /cam1/image_raw --models ds-none ds-none --target tools/kalibr-aprilgrid.yaml
```

If the repository is mounted at `/data`, first `cd /data` in the container.
Depending on your Kalibr setup, its image extraction and report generation
may need a display or a virtual display. Follow the upstream installation
instructions for your environment.

Review Kalibr's report before installing anything: check detected corners,
reprojection residuals, coverage across the view, and a physically plausible
camera baseline. This integration has synthetic geometry/import tests; it
does not certify a fit from uncollected images. Avoid uploading a failed or
poorly constrained result.

## Import and install

Back in `os/` on your computer:

```sh
.venv-fisheye/bin/python tools/kalibr_fisheye.py import --camchain stereo-camchain.yaml --output fisheye-kalibr
.venv-fisheye/bin/python tools/kalibr_fisheye.py upload --input fisheye-kalibr.txt
```

Use the actual camchain filename printed by your Kalibr run. The importer
requires the native resolution and the exact left/right topic order above.
It converts Kalibr's translation from metres to millimetres and constructs
rectifying rotations and a common forward perspective view inside the usable
lens region. OpenCV is used for the rectifying rotations, not for refitting
Kalibr's intrinsics or stereo transform.

The output is `fisheye-kalibr.txt` for the board and a `.json` file retaining
the camera chain and rectification geometry. `camchain.yaml` does not contain
reprojection RMS; the importer records that value as unknown. If you have a
measured scalar RMS from the report, pass `--rms-px VALUE`; values over 3 native
pixels are rejected. Unknown RMS is displayed as unknown, never as zero error.

The converted `4REELFISHEYE2` profile needs the Kalibr-aware firmware from this
change. Older firmware accepts only `4REELFISHEYE1`. Upload stops capture and
persists the profile across reboots. Start Stereo + ZipDepth again, then
check several stationary targets at independently measured distances.

## Vignetting and SAD

The matcher excludes a conservative central native circle of radius 360 px,
centred on each camera's principal point after calibration (image centre
before calibration). The mask follows the rectification map and includes a
margin for the sampling footprint. Every 4 × 4 SAD candidate must be valid in
both images at that candidate's actual disparity and vertical offset. Invalid
costs cannot win or alter the uniqueness check. CPU census applies the same
rule to its 5 × 5 support window. Raw previews retain the full image.

This removes the dark outer rim from match selection. It is a conservative
geometric crop, not a measured brightness correction. Remaining gradual
falloff inside the usable area needs a separate flat-field measurement using
uniform diffuse illumination and fixed exposure/gain. Avoid aggressively
brightening the nearly black rim, which adds noise without recovering lost
detail. Geometry calibration and shading calibration solve different problems.

References: [Kalibr camera calibration](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration),
[supported models](https://github.com/ethz-asl/kalibr/wiki/supported-models),
[targets](https://github.com/ethz-asl/kalibr/wiki/calibration-targets), and
[shading correction](https://docs.baslerweb.com/shading-correction).
