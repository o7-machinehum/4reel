#ifndef FOURREEL_STEREO_METRIC_H
#define FOURREEL_STEREO_METRIC_H
#include <stddef.h>
#include <stdint.h>

#define STEREO_W 1280
#define STEREO_H 800
#define TENSOR_W 320
#define TENSOR_H 224
#define TENSOR_PAD 12
#define STEREO_MAX_ANCHORS 128

/* Kalibr ds: xi, alpha, fx, fy, cx, cy. Pixel centres at integer coordinates.
 * R,t map physical LEFT camera coordinates to RIGHT, t in metres.
 * Coordinates: +x image right, +y down, +z optical forward.
 */
struct DsCamera { double p[6]; };
struct StereoCalibration {
    struct DsCamera eye[2];
    double R[9], t[3];
    char unit[64], source_sha256[65];
};
struct StereoAnchor { double x, y, prediction, inverse_z, weight; };
struct StereoFit {
    double scale, shift, rms, confidence;
    double min_prediction, max_prediction;
    unsigned inliers;
    unsigned hull_size;
    double hull[STEREO_MAX_ANCHORS][2];
    uint8_t accepted[STEREO_MAX_ANCHORS];
};

int stereo_calibration_load(const char *path, const char *unit,
                            struct StereoCalibration *out);
int stereo_calibration_valid(const struct StereoCalibration *c);
int ds_ray(const struct DsCamera *c, double x, double y, double ray[3]);
int ds_project(const struct DsCamera *c, const double p[3], double xy[2]);
/* Reject behind-camera, weak parallax, epipolar error and out-of-range depth.
 * z[0],z[1] are optical-axis metres; no nominal baseline is ever substituted.
 */
int stereo_triangulate(const struct StereoCalibration *c,
                       const double left[2], const double right[2], double z[2]);
int stereo_fit(const struct StereoAnchor *a, unsigned n, struct StereoFit *fit);
/* 0 invalid, 1 interpolation within inlier convex hull, 2 extrapolation.
 * Separate mask prevents periphery being mistaken for supported geometry.
 */
int stereo_support(const struct StereoAnchor *a, unsigned n,
                   const struct StereoFit *fit, double x, double y);
/* CPU reference for RGA average 4x resize, 12 black rows top/bottom.
 * Native mono, no colour conversion or black-level/gamma manipulation.
 */
int stereo_tensor_cpu(const uint8_t *src, size_t bytes, unsigned stride,
                      uint8_t *dst);

#endif
