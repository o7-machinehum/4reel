#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d /tmp/4reel-depth-tests.XXXXXX)
trap 'rm -rf "$test_dir"' EXIT
common=(-std=c11 -O2 -Wall -Wextra -Werror)
sources=(http.c depth.c fisheye.c neural.c fusion.c profile.c)
for name in focus exposure calibration fisheye_api; do
    options=()
    case "$name" in
        exposure) options+=(-Wl,--wrap=open,--wrap=ioctl,--wrap=close);;
        calibration|fisheye_api) options+=(-Wl,--wrap=fopen,--wrap=rename,--wrap=unlink);;
    esac
    gcc "${common[@]}" "tests/$name.c" "${sources[@]}" "${options[@]}" -lm -lpthread -o "$test_dir/$name"
    "$test_dir/$name"
done
gcc "${common[@]}" tests/fisheye.c fisheye.c -lm -o "$test_dir/fisheye"
"$test_dir/fisheye"
gcc "${common[@]}" tests/disparity.c depth.c -lm -o "$test_dir/disparity"
"$test_dir/disparity"
gcc "${common[@]}" -DFOURREEL_RVE tests/disparity.c depth.c rve_sad.c tests/rve_mock.c profile.c -Wl,--wrap=access -lm -lpthread -o "$test_dir/rve"
"$test_dir/rve"
gcc "${common[@]}" tests/masking.c depth.c -lm -o "$test_dir/masking"
"$test_dir/masking"
gcc "${common[@]}" -DFOURREEL_RVE tests/masking.c depth.c rve_sad.c tests/rve_mock.c profile.c -Wl,--wrap=access -lm -lpthread -o "$test_dir/rve-masking"
"$test_dir/rve-masking"
sdk=../../buildroot/output/staging/usr/include
gcc "${common[@]}" -DFOURREEL_RKNN -idirafter "$sdk" tests/neural.c neural.c fusion.c profile.c -lm -lpthread -o "$test_dir/neural"
"$test_dir/neural"
gcc "${common[@]}" tests/rknn_compat.c ../rockchip-rknn/musl_compat.c -lm -o "$test_dir/compat"
"$test_dir/compat"
node tests/depth-legend.mjs
gcc "${common[@]}" -shared -fPIC fisheye.c -lm -o "$test_dir/fisheye.so"
"${FOURREEL_TEST_PYTHON:-python3}" tests/calibrate_fisheye.py "$test_dir/fisheye.so"
"${FOURREEL_TEST_PYTHON:-python3}" tests/kalibr.py "$test_dir/fisheye.so"
