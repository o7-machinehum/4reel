#ifndef FOURREEL_FUSION_H
#define FOURREEL_FUSION_H

#include "depth.h"

enum FusionSource {
    FUSION_INVALID,
    FUSION_STEREO,
    FUSION_ESTIMATED,
};

enum FusionFit {
    FUSION_OFF,
    FUSION_INSUFFICIENT,
    FUSION_REJECTED,
    FUSION_FITTED,
};

struct FusionStats {
    unsigned measured, estimated, anchors, inliers;
    enum FusionFit fit;
    float slope, offset;
};

struct FusionStats fuse_depth(const float *relative, const uint16_t *stereo,
                              const uint8_t *valid, uint16_t *depth, uint8_t *source);

#endif
