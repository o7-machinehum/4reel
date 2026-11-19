#ifndef FOURREEL_STEREO_METRIC_H
#define FOURREEL_STEREO_METRIC_H
#include <stddef.h>
#include <stdint.h>

#define STEREO_W 1280
#define STEREO_H 800
#define TENSOR_W 320
#define TENSOR_H 224
#define TENSOR_PAD 12
#define STEREO_MAX_ANCHORS 20

/* Kalibr ds: xi, alpha, fx, fy, cx, cy. Pixel centres at integer coordinates.
 * R,t map physical LEFT camera coordinates to RIGHT, t in metres.
 * Coordinates: +x image right, +y down, +z optical forward.
 */
/* Logical pixel (0,0) in camera/calibration orientation. DMA capture rows
 * may have padding and a negative horizontal step. */
struct StereoImage {
    const uint8_t *origin;
    int stride, xstep;
};

struct CameraModel {
    double p[6]; /* xi, alpha (DS only), fx, fy, cx, cy */
    double distortion[4]; /* pinhole-radtan k1, k2, p1, p2 */
    int pinhole;
};
struct StereoCalibration {
    struct CameraModel eye[2];
    double R[9], t[3];
    char unit[64], source_sha256[65];
};
int stereo_calibration_load(const char *path, const char *unit,
                            struct StereoCalibration *out);
int stereo_calibration_valid(const struct StereoCalibration *c);
int camera_ray(const struct CameraModel *c, double x, double y, double ray[3]);
int camera_project(const struct CameraModel *c, const double p[3], double xy[2]);
/* Reject behind-camera, weak parallax, epipolar error and out-of-range depth.
 * z[0],z[1] are optical-axis metres; no nominal baseline is ever substituted.
 */
int stereo_triangulate(const struct StereoCalibration *c,
                       const double left[2], const double right[2], double z[2]);
/* Each row is one physical point: prediction times measured optical Z, per eye. */
unsigned stereo_scale(const double votes[][2], unsigned n, double *factor);
/* On failure the caller publishes the full relative prediction for this eye. */
int stereo_scale_map(const float *prediction, float *metres, unsigned pixels, double factor);
/* CPU reference for RGA average 4x resize, 12 black rows top/bottom.
 * Native mono, no colour conversion or black-level/gamma manipulation.
 */
int stereo_tensor_cpu(const uint8_t *src, size_t bytes, unsigned stride,
                      uint8_t *dst);

#endif
