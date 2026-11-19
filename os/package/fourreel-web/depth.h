#ifndef FOURREEL_DEPTH_H
#define FOURREEL_DEPTH_H

#include <stddef.h>
#include <stdint.h>

#define DEPTH_W 160
#define DEPTH_H 100
#define MAX_DISPARITY 96
#define STEREO_SEARCH_NEAR_MM 75
#define STEREO_SEARCH_FAR_MM 5000

/* Two-distance, approximate rectification; not a camera-intrinsic calibration. */
struct Calibration {
	double offset_px;       /* Disparity at infinite distance. */
	double scale_px_mm;     /* Signed: disparity = offset_px + scale_px_mm / distance_mm. */
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

/* visual: 0 invalid, 1 far through 255 near.
 * depth_mm: 0 invalid/uncalibrated, otherwise approximate metric distance.
 * Input frames are contiguous 8-bit grayscale. The function is not reentrant.
 */
void depth_compute(const uint8_t *left, const uint8_t *right,
		   const struct Calibration *cal, uint8_t *visual,
		   uint16_t *depth_mm);

/* Validity planes use 0 for excluded pixels and nonzero for usable pixels.
 * Masks apply to entire candidate windows before best/second-best selection. */
void depth_compute_masked(const uint8_t *left, const uint8_t *right,
			 const struct Calibration *cal,
			 const uint8_t *left_valid, const uint8_t *right_valid,
			 uint8_t *visual, uint16_t *depth_mm);

/* Returns 0 on success. Exercises the solver and a synthetic stereo pair. */
int depth_self_test(void);

#endif
