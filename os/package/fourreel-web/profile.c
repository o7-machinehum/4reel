#include "profile.h"

#include <pthread.h>
#include <string.h>

struct Span {
    uint64_t start, end;
    enum ProfileLane lane;
};

/* Main, camera and RVE operations run in the HTTP event-loop thread. */
static struct Span main_spans[8192];
static struct Span worker_spans[64];
static struct Trace {
    struct Span *spans;
    unsigned capacity, next, count;
} main_trace = {.spans = main_spans, .capacity = sizeof(main_spans) / sizeof(*main_spans)},
  worker_trace = {.spans = worker_spans, .capacity = sizeof(worker_spans) / sizeof(*worker_spans)};
static pthread_mutex_t worker_lock = PTHREAD_MUTEX_INITIALIZER;

void profile_trace(enum ProfileLane lane, uint64_t start, uint64_t end)
{
    if (end <= start)
        return;
    int worker = lane == TIMELINE_ZIPDEPTH_CPU || lane == TIMELINE_NPU;
    struct Trace *trace = worker ? &worker_trace : &main_trace;
    if (worker)
        pthread_mutex_lock(&worker_lock);
    trace->spans[trace->next] = (struct Span){start, end, lane};
    if (++trace->next == trace->capacity)
        trace->next = 0;
    if (trace->count < trace->capacity)
        trace->count++;
    if (worker)
        pthread_mutex_unlock(&worker_lock);
}

static void collect(const struct Trace *trace, struct ProfileRange range, uint64_t begin,
                    uint64_t step, enum ProfileLane first, enum ProfileLane last,
                    uint64_t elapsed[TIMELINE_LANES][PROFILE_TIMELINE_SAMPLES])
{
    if (range.end <= range.start)
        return;
    unsigned index = trace->next;
    for (unsigned i = 0; i < trace->count; ++i) {
        index = index ? index - 1 : trace->capacity - 1;
        const struct Span *span = &trace->spans[index];
        if (span->end <= range.start)
            break;
        if (span->lane < first || span->lane > last)
            continue;
        uint64_t start = span->start > range.start ? span->start : range.start;
        uint64_t end = span->end < range.end ? span->end : range.end;
        while (start < end) {
            unsigned bucket = (start - begin) / step;
            uint64_t next = begin + (bucket + 1) * step;
            if (next > end)
                next = end;
            uint64_t *value = &elapsed[span->lane][bucket];
            *value += next - start;
            if (*value > step)
                *value = step;
            start = next;
        }
    }
}

uint64_t profile_timeline(const struct ProfileCycle *cycle,
                          char rows[TIMELINE_LANES][PROFILE_TIMELINE_SAMPLES + 1])
{
    for (unsigned lane = 0; lane < TIMELINE_LANES; ++lane) {
        memset(rows[lane], '0', PROFILE_TIMELINE_SAMPLES);
        rows[lane][PROFILE_TIMELINE_SAMPLES] = '\0';
    }
    const struct ProfileRange ranges[] = {
        cycle->cameras[0], cycle->cameras[1], cycle->stereo, cycle->neural
    };
    uint64_t begin = UINT64_MAX, end = 0;
    for (unsigned i = 0; i < sizeof(ranges) / sizeof(*ranges); ++i) {
        if (ranges[i].end <= ranges[i].start)
            continue;
        if (ranges[i].start < begin)
            begin = ranges[i].start;
        if (ranges[i].end > end)
            end = ranges[i].end;
    }
    if (end <= begin)
        return 0;

    uint64_t duration = end - begin;
    uint64_t step = (duration + PROFILE_TIMELINE_SAMPLES - 1) / PROFILE_TIMELINE_SAMPLES;
    uint64_t elapsed[TIMELINE_LANES][PROFILE_TIMELINE_SAMPLES] = {{0}};
    for (unsigned side = 0; side < 2; ++side)
        collect(&main_trace, cycle->cameras[side], begin, step,
                TIMELINE_CAMERA_IO, TIMELINE_CAMERA_CPU, elapsed);
    collect(&main_trace, cycle->stereo, begin, step, TIMELINE_STEREO, TIMELINE_STEREO_REFINE, elapsed);
    pthread_mutex_lock(&worker_lock);
    collect(&worker_trace, cycle->neural, begin, step, TIMELINE_ZIPDEPTH_CPU, TIMELINE_NPU, elapsed);
    pthread_mutex_unlock(&worker_lock);
    for (unsigned lane = 0; lane < TIMELINE_LANES; ++lane)
        for (unsigned i = 0; i < PROFILE_TIMELINE_SAMPLES; ++i)
            rows[lane][i] = '0' + (elapsed[lane][i] * 9 + step - 1) / step;
    return duration;
}
