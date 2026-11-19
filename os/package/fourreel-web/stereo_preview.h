#ifndef FOURREEL_STEREO_PREVIEW_H
#define FOURREEL_STEREO_PREVIEW_H
#include "stereo_trace.h"
#include "stereo_metric.h"
struct StereoMatch;
struct StereoPreview;
/* NULL disables HTTP/preview. Trace ring remains independent of HTTP. */
struct StereoPreview *stereo_preview_start(unsigned port,int enabled);
int stereo_preview_enabled(struct StereoPreview *p);
/* Capture thread never waits for the HTTP thread. One latest-slot snapshot is
 * copied only for a pending request; rendering/encoding/socket IO run elsewhere. */
void stereo_preview_publish(struct StereoPreview *p,struct PairTrace *pair,
 const struct TraceRing *ring,const struct StereoImage native[2],const float *prediction,
 const float *metric,const uint8_t *support,const int valid[2],int capture,
 const struct StereoMatch *matches,unsigned match_count);
void stereo_preview_stop(struct StereoPreview *p);
#endif
