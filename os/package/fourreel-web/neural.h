#ifndef FOURREEL_NEURAL_H
#define FOURREEL_NEURAL_H

#include <stdint.h>
#include "fusion.h"
#include "profile.h"

#define NEURAL_W 256
#define NEURAL_H 160
#define NEURAL_PIXELS (NEURAL_W * NEURAL_H)
#define NEURAL_MODEL "/usr/share/4reel-web/zipdepth-rv1103.rknn"

struct NeuralResult {
    uint8_t input[DEPTH_W * DEPTH_H];
    uint8_t visual[DEPTH_W * DEPTH_H];
    float inverse_depth[DEPTH_W * DEPTH_H];
    uint16_t fused_mm[DEPTH_W * DEPTH_H];
    uint8_t source[DEPTH_W * DEPTH_H];
    struct FusionStats fusion;
    uint16_t metric_mm[DEPTH_W * DEPTH_H];
    uint64_t metric_frame, metric_capture_ns;
    uint64_t metric_reference_frame;
    float relative_low, relative_high;
    uint64_t frame;
    double milliseconds;
    int available;
    char error[96];
};

int neural_start(const char *model_path);
void neural_stop(void);
/* Optional stereo depth and left validity are copied from this same frame. */
int neural_submit(const uint8_t *left, const uint16_t *stereo,
                  const uint8_t *valid, uint64_t frame, const struct ProfileCycle *cycle);
void neural_snapshot(struct NeuralResult *result);

struct NeuralProfile {
    uint64_t queue_ns, started, dropped;
    struct ProfileMetric pack, run, decode, fusion;
    struct ProfileCycle cycle;
};
void neural_profile(struct NeuralProfile *profile);

#endif
