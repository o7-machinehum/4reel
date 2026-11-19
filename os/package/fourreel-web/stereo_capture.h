#ifndef FOURREEL_STEREO_CAPTURE_H
#define FOURREEL_STEREO_CAPTURE_H
#include <stdint.h>
struct Trace;
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
int stereo_capture_pair(uint8_t *left,uint8_t *right,struct CapturePair *pair);
int stereo_capture_pair_timed(uint8_t *left,uint8_t *right,struct CapturePair *pair,struct Trace *trace);
void stereo_capture_close(void);
#endif
