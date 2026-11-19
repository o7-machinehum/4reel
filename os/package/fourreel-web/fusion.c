#include "fusion.h"

#include <math.h>

#define ANCHOR_STEP 4
#define MIN_ANCHORS 24
#define FIT_TRIALS 64

struct Anchor {
    float relative, inverse_mm;
    unsigned region;
};

static int agrees(const struct Anchor *anchor, float slope, float offset)
{
    /* One disparity count can cause a larger percentage error at long range. */
    float tolerance = 0.12f * anchor->inverse_mm + 0.10f;
    return fabsf(anchor->relative * slope + offset - anchor->inverse_mm) <= tolerance;
}

struct FusionStats fuse_depth(const float *relative, const uint16_t *stereo,
                              const uint8_t *valid, uint16_t *depth, uint8_t *source)
{
    struct FusionStats stats = {.fit = FUSION_INSUFFICIENT};
    struct Anchor anchors[DEPTH_W * DEPTH_H / (ANCHOR_STEP * ANCHOR_STEP)];
    for (unsigned i = 0; i < DEPTH_W * DEPTH_H; ++i) {
        depth[i] = stereo[i];
        source[i] = stereo[i] ? FUSION_STEREO : FUSION_INVALID;
        stats.measured += stereo[i] != 0;
    }
    /* Sample at the RVE block stride so each SAD match votes at most once. */
    for (unsigned y = ANCHOR_STEP / 2; y < DEPTH_H; y += ANCHOR_STEP)
        for (unsigned x = ANCHOR_STEP / 2; x < DEPTH_W; x += ANCHOR_STEP) {
            unsigned i = y * DEPTH_W + x;
            if (!valid[i] || stereo[i] < STEREO_SEARCH_NEAR_MM ||
                stereo[i] > STEREO_SEARCH_FAR_MM || !isfinite(relative[i]))
                continue;
            anchors[stats.anchors++] = (struct Anchor){
                relative[i], 1000.f / stereo[i], (y * 3 / DEPTH_H) * 3 + x * 3 / DEPTH_W
            };
        }
    if (stats.anchors < MIN_ANCHORS)
        return stats;

    stats.fit = FUSION_REJECTED;
    unsigned best = 0;
    float slope = 0, offset = 0;
    uint32_t seed = 0x4a13cb21;
    for (unsigned trial = 0; trial < FIT_TRIALS; ++trial) {
        seed = (seed ^ (seed << 13));
        seed ^= seed >> 17;
        seed ^= seed << 5;
        const struct Anchor *a = &anchors[(seed >> 16) % stats.anchors];
        const struct Anchor *b = &anchors[(seed & 0xffff) % stats.anchors];
        float delta = a->relative - b->relative;
        if (fabsf(delta) < 0.00001f)
            continue;
        float m = (a->inverse_mm - b->inverse_mm) / delta;
        if (m <= 0 || fabsf(a->inverse_mm - b->inverse_mm) < 0.25f)
            continue;
        float c = a->inverse_mm - m * a->relative;
        unsigned support = 0;
        for (unsigned i = 0; i < stats.anchors; ++i)
            support += agrees(&anchors[i], m, c);
        if (support > best) {
            best = support;
            slope = m;
            offset = c;
        }
    }
    stats.inliers = best;
    if (best < MIN_ANCHORS || best * 5 < stats.anchors * 3)
        return stats;

    double xsum = 0, ysum = 0, xx = 0, xy = 0;
    for (unsigned i = 0; i < stats.anchors; ++i) {
        const struct Anchor *a = &anchors[i];
        if (!agrees(a, slope, offset))
            continue;
        double x = a->relative, y = a->inverse_mm;
        xsum += x;
        ysum += y;
        xx += x * x;
        xy += x * y;
    }
    double variance = xx - xsum * xsum / best;
    if (variance <= 0)
        return stats;
    slope = (xy - xsum * ysum / best) / variance;
    offset = (ysum - slope * xsum) / best;
    if (slope <= 0 || !isfinite(slope) || !isfinite(offset))
        return stats;

    unsigned regions = 0;
    float lo = INFINITY, hi = -INFINITY;
    stats.inliers = 0;
    for (unsigned i = 0; i < stats.anchors; ++i) {
        const struct Anchor *a = &anchors[i];
        if (!agrees(a, slope, offset))
            continue;
        stats.inliers++;
        regions |= 1U << a->region;
        if (a->relative < lo)
            lo = a->relative;
        if (a->relative > hi)
            hi = a->relative;
    }
    if (stats.inliers < MIN_ANCHORS || stats.inliers * 5 < stats.anchors * 3 ||
        __builtin_popcount(regions) < 3 || (hi - lo) * slope < 0.25f)
        return stats;

    stats.fit = FUSION_FITTED;
    stats.slope = slope;
    stats.offset = offset;
    for (unsigned i = 0; i < DEPTH_W * DEPTH_H; ++i) {
        if (source[i] || !valid[i] || !isfinite(relative[i]) ||
            relative[i] < lo || relative[i] > hi)
            continue;
        float inverse_mm = relative[i] * slope + offset;
        if (inverse_mm < 1000.f / STEREO_SEARCH_FAR_MM ||
            inverse_mm > 1000.f / STEREO_SEARCH_NEAR_MM)
            continue;
        depth[i] = (uint16_t)(1000.f / inverse_mm + 0.5f);
        source[i] = FUSION_ESTIMATED;
        stats.estimated++;
    }
    return stats;
}
