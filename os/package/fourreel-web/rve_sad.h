#ifndef FOURREEL_RVE_SAD_H
#define FOURREEL_RVE_SAD_H
#include "depth.h"
#include "validity.h"
#include "profile.h"

struct RveProfile {
    struct ProfileMetric sad, dma, refine;
};
#ifdef FOURREEL_RVE
int rve_sad_self_test(void);
int rve_depth_compute(const uint8_t *left, const uint8_t *right,
                      int min_d, int max_d, int vertical,
                      const struct Calibration *cal,
                      const struct StereoValidity *validity,
                      uint8_t *visual, uint16_t *depth_mm);
const char *rve_sad_status(void);
double rve_sad_milliseconds(void);
unsigned rve_sad_passes(void);
void rve_sad_close(void);
struct RveProfile rve_profile(void);
#else
static inline const char *rve_sad_status(void) { return "CPU census"; }
static inline double rve_sad_milliseconds(void) { return 0; }
static inline unsigned rve_sad_passes(void) { return 0; }
static inline struct RveProfile rve_profile(void) { return (struct RveProfile){0}; }
static inline void rve_sad_close(void) {}
#endif
#endif
