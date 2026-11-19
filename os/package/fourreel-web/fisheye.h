#ifndef FOURREEL_FISHEYE_H
#define FOURREEL_FISHEYE_H

#include <stddef.h>
#include <stdint.h>
#include "depth.h"

#define FISHEYE_RAW_W 1280
#define FISHEYE_RAW_H 800
#define FISHEYE_TEXT_BYTES 2049
#define FISHEYE_INVALID UINT16_MAX
/* Conservative central lens region; excludes the dark outer image circle.
 * This geometric crop is not a measured flat-field correction. */
#define FISHEYE_USABLE_RADIUS (FISHEYE_RAW_H * 0.45)
#define FISHEYE_EQUIDISTANT 0
#define FISHEYE_DOUBLE_SPHERE 1

struct FisheyeCamera {
    double fx, fy, cx, cy;
    double distortion[4];
    double rotation[9];
    double xi, alpha;
    int model;
};

struct FisheyeCalibration {
    struct FisheyeCamera camera[2];
    double fx, fy, cx, cy;
    double baseline_mm;
    double rms;
    int valid;
};

struct FisheyeMaps {
    /* Native, unmirrored RAW10 coordinates in sixteenths of a pixel. */
    uint16_t xy[2][DEPTH_W * DEPTH_H][2];
};

int fisheye_parse(const char *text, struct FisheyeCalibration *calibration,
                  char *error, size_t error_size);
int fisheye_format(const struct FisheyeCalibration *calibration,
                   char *text, size_t size);
void fisheye_build_maps(const struct FisheyeCalibration *calibration,
                        struct FisheyeMaps *maps);
void fisheye_build_map(const struct FisheyeCalibration *calibration, int side,
                       unsigned width, unsigned height, uint16_t (*map)[2]);
void fisheye_remap_raw10(const uint16_t (*map)[2], unsigned count,
                        const uint8_t *raw, unsigned stride, uint8_t *image);
void fisheye_rectify_raw10(const struct FisheyeMaps *maps, int side,
                           const uint8_t *raw, unsigned stride, uint8_t *image);
void fisheye_mask_depth(const struct FisheyeMaps *maps,
                        uint8_t *visual, uint16_t *depth_mm);
void fisheye_build_validity(const struct FisheyeCalibration *calibration,
                            const struct FisheyeMaps *maps,
                            uint8_t valid[2][DEPTH_W * DEPTH_H]);

#endif
