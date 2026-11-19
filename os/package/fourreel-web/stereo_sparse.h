#ifndef FOURREEL_STEREO_SPARSE_H
#define FOURREEL_STEREO_SPARSE_H
#include "stereo_metric.h"
struct StereoMatch { double xy[2][2], z[2], weight; };
/* Bounded portable fallback: grid corners, DS epipolar search at quarter size,
 * pyramidal LK refinement through native resolution, forward/backward and
 * geometric checks. No undocumented RVE ABI is invoked. */
unsigned stereo_sparse(const uint8_t *left,const uint8_t *right,
                       const struct StereoCalibration *cal,
                       struct StereoMatch *matches);
#endif
