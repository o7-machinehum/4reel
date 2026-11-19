#ifndef FOURREEL_STEREO_TRACE_H
#define FOURREEL_STEREO_TRACE_H
#include <stdint.h>
#include <time.h>
/* Fixed wire layout shared by musl and the private glibc helper. All times are
 * host CLOCK_MONOTONIC us; CPU is CLOCK_THREAD_CPUTIME_ID, never device time. */
#define TRACE_EVENTS 64
#define TRACE_RING 64
#define TRACE_PROTOCOL 0x46525432u
#define TRACE_STAGES(X) \
 X(CAPTURE,"capture/pairing") X(QUIET_PRE,"capture quiet-before") \
 X(TRIGGER,"PWM submit/return") X(CAPTURE_WAIT,"capture poll wait") \
 X(UNPACK_L,"left dequeue/unpack") X(UNPACK_R,"right dequeue/unpack") \
 X(QUIET_POST,"capture quiet-after") X(SUBMIT,"helper submit/pipe") \
 X(RECEIVE,"helper receive/wait") X(SOURCE_L,"left DMA source prep") \
 X(RGA_L,"left RGA synchronous call") X(TENSOR_L,"left tensor/cache prep") \
 X(NPU_L,"left NPU synchronous call") X(OUTPUT_L,"left output copy/check") \
 X(SOURCE_R,"right DMA source prep") X(RGA_R,"right RGA synchronous call") \
 X(TENSOR_R,"right tensor/cache prep") X(NPU_R,"right NPU synchronous call") \
 X(OUTPUT_R,"right output copy/check") X(PYRAMID,"sparse pyramids") \
 X(CORNERS,"sparse corners") X(MATCH,"sparse epipolar/forward LK") \
 X(REVERSE,"sparse reverse validation") X(TRIANGULATE,"sparse triangulation") \
 X(FIT_L,"left robust fit") X(FIT_R,"right robust fit") \
 X(MAP_L,"left map generation") X(MAP_R,"right map generation") \
 X(FILES,"pair file publication") X(SNAPSHOT,"on-demand snapshot copy") \
 X(TELEMETRY,"trace record overhead")
enum TraceStage {
#define X(a,b) TS_##a,
 TRACE_STAGES(X)
#undef X
 TS_COUNT
};
enum TraceResource { TR_MAIN, TR_HELPER, TR_RGA, TR_NPU, TR_WAIT, TR_IO, TR_ENVELOPE, TR_RESOURCES };
struct TraceEvent { uint64_t begin, end, cpu; uint32_t stage, resource; };
struct Trace { uint32_t enabled, count, overflow, reserved; struct TraceEvent event[TRACE_EVENTS]; };
struct TraceMark { uint64_t wall,cpu; };
struct RunnerRequest { uint64_t id; uint32_t protocol, enabled; };
struct RunnerResponse { uint64_t id, first_eye_us, complete_us, cpu_us; struct Trace trace; };
_Static_assert(sizeof(struct RunnerRequest)==16,"musl/glibc request ABI");
_Static_assert(sizeof(struct RunnerResponse)==2096,"musl/glibc response ABI");
struct PairTrace {
 uint64_t id, start, end, input_us, trigger_us, first_capture_us, complete_capture_us;
 uint64_t first_eye_us, complete_inference_us, main_cpu_us, helper_cpu_us;
 uint64_t submit_wait_us, receive_wait_us;
 uint32_t sequence_gap[2], success, reserved;
 struct Trace trace;
};
struct TraceRing { struct PairTrace pair[TRACE_RING]; uint32_t next, count; };
static inline uint64_t trace_clock(clockid_t id) {
 struct timespec t; clock_gettime(id,&t); return (uint64_t)t.tv_sec*1000000+t.tv_nsec/1000;
}
static inline struct TraceMark trace_begin(const struct Trace *t) {
 return t && t->enabled ? (struct TraceMark){trace_clock(CLOCK_MONOTONIC),trace_clock(CLOCK_THREAD_CPUTIME_ID)}:(struct TraceMark){0,0};
}
static inline void trace_end(struct Trace *t,struct TraceMark m,unsigned stage,unsigned resource) {
 if(!t || !t->enabled) return;
 struct TraceEvent e={m.wall,trace_clock(CLOCK_MONOTONIC),trace_clock(CLOCK_THREAD_CPUTIME_ID)-m.cpu,stage,resource};
 if(t->count<TRACE_EVENTS) t->event[t->count++]=e; else ++t->overflow;
}
void trace_ring_push(struct TraceRing *ring,const struct PairTrace *p);
/* nearest-rank percentile, missing stages excluded rather than measured as zero */
uint64_t trace_percentile(const struct TraceRing *r,unsigned stage,unsigned percentile,unsigned *samples);
uint64_t trace_union_us(const struct PairTrace *p,unsigned resource);
const char *trace_stage_name(unsigned stage);
#endif
