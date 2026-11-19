#ifndef FOURREEL_STEREO_SPARSE_H
#define FOURREEL_STEREO_SPARSE_H
#include "stereo_metric.h"
struct Trace;
struct SparseStats;
struct StereoMatch;
/* One bit per previously rejected quarter of each spatial cell. */
struct StereoCornerHistory { uint8_t rejected[STEREO_MAX_ANCHORS]; };
unsigned stereo_match_pair(const struct StereoImage eyes[2], const struct StereoCalibration *cal,
                           struct StereoMatch *matches, struct SparseStats *stats, struct Trace *trace,
                           struct StereoCornerHistory *history);
struct StereoMatch { double xy[2][2], z[2], weight; };
/* Grid corners, bidirectional normalized patch searches at quarter resolution,
 * subpixel peak interpolation and calibrated triangulation. */
unsigned stereo_sparse(const uint8_t *left,const uint8_t *right,
                       const struct StereoCalibration *cal,
                       struct StereoMatch *matches);
unsigned stereo_sparse_views(const struct StereoImage eyes[2],
                       const struct StereoCalibration *cal,struct StereoMatch *matches,
                       struct Trace *trace);
unsigned stereo_sparse_timed(const uint8_t *left,const uint8_t *right,
                       const struct StereoCalibration *cal,struct StereoMatch *matches,
                       struct Trace *trace);
#endif
