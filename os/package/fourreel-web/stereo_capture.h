#ifndef FOURREEL_STEREO_CAPTURE_H
#define FOURREEL_STEREO_CAPTURE_H
#include <stdint.h>
#include "stereo_metric.h"
struct PairTrace;
struct Trace;
struct CaptureDmaView {
    int fd[2];
    uint32_t stride[2];
    const uint8_t *data[2];
};
struct CapturePair {
    uint64_t id, trigger_before_us, trigger_after_us;
    int64_t timestamp_us[2];
    uint32_t sequence[2], flags[2], sequence_gap[2];
    uint64_t dequeued_us[2];
};
/* One hardware PWM burst, one full-resolution image per physical eye.
 * Native RAW8 or packed RAW10 is converted to orientation-preserving uint8.
 * Caller stops the dashboard before taking exclusive camera ownership.
 */
int stereo_capture_open(int raw8);
int stereo_capture_enable_ae(void);
int stereo_capture_adjust_ae(const struct StereoImage eyes[2],struct PairTrace *record);
int stereo_capture_pair(uint8_t *left,uint8_t *right,struct CapturePair *pair);
int stereo_capture_pair_timed(uint8_t *left,uint8_t *right,struct CapturePair *pair,struct Trace *trace);
/* RAW8 only. Export after open and before fork/exec of the RGA helper.
 * Exported descriptors belong to capture and stay open until close. The DMA
 * view contains the physical (unflipped) input and is valid until release.
 * Finish all CPU/RGA reads before releasing the pair back to CIF. */
int stereo_capture_export(void);
int stereo_capture_pair_dma(struct CapturePair *pair,struct Trace *trace,struct CaptureDmaView *view);
int stereo_capture_release(void);
void stereo_capture_close(void);
#endif
