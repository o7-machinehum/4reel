#ifndef FOURREEL_VALIDITY_H
#define FOURREEL_VALIDITY_H

#include "depth.h"

/* Summed invalid-pixel counts: a whole matching window must be usable in
 * both images, including the disparity-shifted window in the right image. */
struct StereoValidity {
    uint16_t invalid[2][(DEPTH_W + 1) * (DEPTH_H + 1)];
};

static inline int stereo_window_valid(const struct StereoValidity *validity,
                                      int side, int x, int y, int size)
{
    if (x < 0 || y < 0 || x + size > DEPTH_W || y + size > DEPTH_H)
        return 0;
    if (!validity)
        return 1;
    const uint16_t *sum = validity->invalid[side];
    int top = y * (DEPTH_W + 1), bottom = (y + size) * (DEPTH_W + 1);
    return sum[bottom + x + size] + sum[top + x] ==
           sum[bottom + x] + sum[top + x + size];
}

#endif
