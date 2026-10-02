#include "depth.h"
#include "rve_sad.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#if (defined(__ARM_NEON) || defined(__ARM_NEON__)) && \
	!defined(FOURREEL_DEPTH_FORCE_SCALAR)
#include <arm_neon.h>
#define HAVE_NEON 1
#else
#define HAVE_NEON 0
#endif

#define PIXELS (DEPTH_W * DEPTH_H)
#define CENSUS_RADIUS 2
#define SEARCH_NEAR_MM 75.0
#define SEARCH_FAR_MM 5000.0
#define SEARCH_MARGIN_PX 2

/* One 24-bit descriptor per right-image pixel. No cost volume is allocated. */
static uint32_t right_census[PIXELS];

static double magnitude(double value)
{
	return value < 0 ? -value : value;
}

static int invalid(char *error, size_t size, const char *message)
{
	if (error && size)
		snprintf(error, size, "%s", message);
	return -1;
}

static int point_valid(const struct CalPoint *p)
{
	return p && isfinite(p->lx) && isfinite(p->ly) &&
		isfinite(p->rx) && isfinite(p->ry) &&
		isfinite(p->distance_mm) && p->distance_mm > 0 &&
		p->lx >= 0 && p->lx < DEPTH_W &&
		p->rx >= 0 && p->rx < DEPTH_W &&
		p->ly >= 0 && p->ly < DEPTH_H &&
		p->ry >= 0 && p->ry < DEPTH_H;
}

int calibration_solve(const struct CalPoint *near, const struct CalPoint *far,
		      struct Calibration *out, char *error, size_t error_size)
{
	double near_d, far_d, scale, offset, vertical;

	if (out)
		out->valid = 0;
	if (!out || !point_valid(near) || !point_valid(far))
		return invalid(error, error_size, "Both points must be inside the stereo images and have positive finite distances.");
	if (near->distance_mm >= far->distance_mm)
		return invalid(error, error_size, "Near distance must be smaller than far distance.");

	near_d = near->lx - near->rx;
	far_d = far->lx - far->rx;
	if (magnitude(near_d - far_d) <= 1.0)
		return invalid(error, error_size, "Near and far disparities must differ by more than one pixel; select matching features at distinct distances.");
	vertical = ((near->ry - near->ly) + (far->ry - far->ly)) * 0.5;
	if (vertical < -12.0 || vertical > 12.0 ||
	    magnitude((near->ry - near->ly) - (far->ry - far->ly)) > 4.0)
		return invalid(error, error_size, "Vertical offsets disagree or exceed 12 pixels; align the stereo images first.");

	scale = (near_d - far_d) /
		(1.0 / near->distance_mm - 1.0 / far->distance_mm);
	offset = near_d - scale / near->distance_mm;
	if (!isfinite(scale) || !isfinite(offset) || scale == 0 ||
	    offset < -MAX_DISPARITY || offset > MAX_DISPARITY ||
	    near_d < -MAX_DISPARITY || near_d > MAX_DISPARITY ||
	    far_d < -MAX_DISPARITY || far_d > MAX_DISPARITY)
		return invalid(error, error_size, "Calibration implies an unsupported disparity range (maximum 96 pixels).");

	out->offset_px = offset;
	out->scale_px_mm = scale;
	out->vertical_px = vertical;
	out->valid = 1;
	if (error && error_size)
		error[0] = '\0';
	return 0;
}

static uint32_t census(const uint8_t *image, int x, int y, int *contrast)
{
	uint8_t center = image[y * DEPTH_W + x];
	uint8_t minimum = center, maximum = center;
	uint32_t bits = 0;
	int dx, dy;

	for (dy = -CENSUS_RADIUS; dy <= CENSUS_RADIUS; ++dy) {
		for (dx = -CENSUS_RADIUS; dx <= CENSUS_RADIUS; ++dx) {
			uint8_t value;
			if (!dx && !dy)
				continue;
			value = image[(y + dy) * DEPTH_W + x + dx];
			bits = (bits << 1) | (value < center);
			if (value < minimum)
				minimum = value;
			if (value > maximum)
				maximum = value;
		}
	}
	if (contrast)
		*contrast = maximum - minimum;
	return bits;
}

static inline void consider_match(int disparity, int cost, int *best,
				  int *second, int *best_disparity)
{
	if (cost < *best) {
		*second = *best;
		*best = cost;
		*best_disparity = disparity;
	} else if (cost < *second) {
		*second = cost;
	}
}

static void find_best_match(uint32_t descriptor, int right_y, int x,
			    int min_d, int max_d, int *best, int *second,
			    int *best_disparity)
{
	const uint32_t *row = &right_census[right_y * DEPTH_W];
	int first = min_d;
	int last = max_d;
	int d;

	/* x - d must remain inside the valid census-image border. */
	if (first < x - DEPTH_W + CENSUS_RADIUS + 1)
		first = x - DEPTH_W + CENSUS_RADIUS + 1;
	if (last > x - CENSUS_RADIUS)
		last = x - CENSUS_RADIUS;
	if (first > last)
		return;

	d = first;
#if HAVE_NEON
	{
		const uint32x4_t left4 = vdupq_n_u32(descriptor);

		for (; d + 3 <= last; d += 4) {
			int right_x = x - d;
			uint32x4_t xor4 = veorq_u32(left4,
				vld1q_u32(&row[right_x - 3]));
			uint16x8_t pairs = vpaddlq_u8(
				vcntq_u8(vreinterpretq_u8_u32(xor4)));
			uint32x4_t costs = vpaddlq_u16(pairs);

			/* The load runs left-to-right, opposite to increasing d. */
			consider_match(d, vgetq_lane_u32(costs, 3), best,
				       second, best_disparity);
			consider_match(d + 1, vgetq_lane_u32(costs, 2), best,
				       second, best_disparity);
			consider_match(d + 2, vgetq_lane_u32(costs, 1), best,
				       second, best_disparity);
			consider_match(d + 3, vgetq_lane_u32(costs, 0), best,
				       second, best_disparity);
		}
	}
#endif
	for (; d <= last; ++d) {
		int cost = __builtin_popcount(descriptor ^ row[x - d]);

		consider_match(d, cost, best, second, best_disparity);
	}
}

void depth_compute(const uint8_t *left, const uint8_t *right,
		   const struct Calibration *cal, uint8_t *visual,
		   uint16_t *depth_mm)
{
	int x, y, min_d = 0, max_d = 32, vertical = 0;
	int calibrated = cal && cal->valid;
	double offset = 0;

	if (!left || !right || !visual || !depth_mm)
		return;
	memset(visual, 0, PIXELS);
	memset(depth_mm, 0, PIXELS * sizeof(*depth_mm));
	if (calibrated) {
		double lower;
		double upper;

		offset = cal->offset_px;
		vertical = (int)(cal->vertical_px +
				 (cal->vertical_px >= 0 ? 0.5 : -0.5));
		/* Camera order determines the sign of scale, not physical distance. */
		double near_d = offset + cal->scale_px_mm / SEARCH_NEAR_MM;
		double far_d = offset + cal->scale_px_mm / SEARCH_FAR_MM;
		lower = floor(fmin(near_d, far_d)) - SEARCH_MARGIN_PX;
		upper = ceil(fmax(near_d, far_d)) + SEARCH_MARGIN_PX;
		if (!isfinite(lower) || !isfinite(upper))
			return;
		if (lower < -MAX_DISPARITY)
			min_d = -MAX_DISPARITY;
		else if (lower > MAX_DISPARITY)
			min_d = MAX_DISPARITY;
		else
			min_d = (int)lower;
		if (upper > MAX_DISPARITY)
			max_d = MAX_DISPARITY;
		else if (upper < -MAX_DISPARITY)
			max_d = -MAX_DISPARITY;
		else
			max_d = (int)upper;
		if (min_d > max_d)
			return;
	}

#ifdef FOURREEL_RVE
	if (rve_depth_compute(left, right, min_d, max_d, vertical, cal, visual, depth_mm) == 0)
		return;
#endif
	for (y = CENSUS_RADIUS; y < DEPTH_H - CENSUS_RADIUS; ++y)
		for (x = CENSUS_RADIUS; x < DEPTH_W - CENSUS_RADIUS; ++x)
			right_census[y * DEPTH_W + x] = census(right, x, y, NULL);

	for (y = CENSUS_RADIUS; y < DEPTH_H - CENSUS_RADIUS; ++y) {
		int right_y = y + vertical;
		if (right_y < CENSUS_RADIUS || right_y >= DEPTH_H - CENSUS_RADIUS)
			continue;
		for (x = CENSUS_RADIUS; x < DEPTH_W - CENSUS_RADIUS; ++x) {
			uint32_t descriptor;
			int contrast, best = 25, second = 25, best_d = 0;
			double parallax, distance;
			int index = y * DEPTH_W + x;

			descriptor = census(left, x, y, &contrast);
			if (contrast < 8 || __builtin_popcount(descriptor) < 3 ||
			    __builtin_popcount(descriptor) > 21)
				continue;
			find_best_match(descriptor, right_y, x, min_d, max_d,
					&best, &second, &best_d);
			if (best > 8 || second - best < 3)
				continue;

			parallax = calibrated ? best_d - offset : best_d;
			if (parallax == 0 || (!calibrated && parallax < 0))
				continue;
			if (calibrated) {
				distance = cal->scale_px_mm / parallax;
				if (!isfinite(distance) || distance < 1.0 ||
				    distance > 65535.0)
					continue;
				depth_mm[index] = (uint16_t)(distance + 0.5);
			}
			parallax = magnitude(parallax);
			if (parallax > MAX_DISPARITY)
				parallax = MAX_DISPARITY;
			visual[index] = 1 + (uint8_t)(parallax * 254.0 /
							  MAX_DISPARITY + 0.5);
		}
	}
}

int depth_self_test(void)
{
	static uint8_t left[PIXELS], right[PIXELS], visual[PIXELS];
	static uint16_t depth_mm[PIXELS];
	const struct CalPoint near = { 80, 50, 70, 50, 500 };
	const struct CalPoint far = { 80, 50, 75, 50, 1000 };
	struct Calibration cal = { 0 };
	uint32_t state = 0x12345678;
	int valid = 0;
	int x, y;

	if (calibration_solve(&near, &far, &cal, NULL, 0) ||
	    magnitude(cal.offset_px) > 0.01 ||
	    magnitude(cal.scale_px_mm - 5000.0) > 0.01)
		return -1;
	for (y = 0; y < DEPTH_H; ++y)
		for (x = 0; x < DEPTH_W; ++x) {
			state ^= state << 13;
			state ^= state >> 17;
			state ^= state << 5;
			left[y * DEPTH_W + x] = (uint8_t)state;
		}
	for (y = 0; y < DEPTH_H; ++y)
		for (x = 0; x < DEPTH_W; ++x)
			right[y * DEPTH_W + x] =
				x + 10 < DEPTH_W ? left[y * DEPTH_W + x + 10] : 0;
	depth_compute(left, right, &cal, visual, depth_mm);
	for (y = 0; y < DEPTH_H; ++y)
		for (x = 0; x < DEPTH_W; ++x)
			if (visual[y * DEPTH_W + x] &&
			    depth_mm[y * DEPTH_W + x] == 500)
				valid++;
	return valid >= PIXELS / 8 ? 0 : -1;
}
