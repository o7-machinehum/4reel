#ifndef FOURREEL_DEPTH_H
#define FOURREEL_DEPTH_H

#include <stddef.h>
#include <stdint.h>

#define DEPTH_W 160
#define DEPTH_H 100

/* Two-distance, approximate rectification; not a camera-intrinsic calibration. */
struct Calibration {
	double offset_px;       /* Disparity at infinite distance. */
	double scale_px_mm;     /* Disparity = offset_px + scale_px_mm / distance_mm. */
	double vertical_px;     /* Right-image y minus left-image y. */
	int valid;
};

struct CalPoint {
	double lx, ly, rx, ry;
	double distance_mm;
};

/* Returns 0 on success, -1 with an explanation in error on invalid samples. */
int calibration_solve(const struct CalPoint *near, const struct CalPoint *far,
		      struct Calibration *out, char *error, size_t error_size);

/* visual: 0 invalid, 1 far/blue through 255 near/red.
 * depth_mm: 0 invalid/uncalibrated, otherwise approximate metric distance.
 * Input frames are contiguous 8-bit grayscale. The function is not reentrant.
 */
void depth_compute(const uint8_t *left, const uint8_t *right,
		   const struct Calibration *cal, uint8_t *visual,
		   uint16_t *depth_mm);

/* Returns 0 on success. Exercises the solver and a synthetic stereo pair. */
int depth_self_test(void);

#endif
