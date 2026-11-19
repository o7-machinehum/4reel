#ifndef FOURREEL_PROFILE_H
#define FOURREEL_PROFILE_H

#include <stdint.h>

enum ProfileLane {
    TIMELINE_CAMERA_IO, TIMELINE_CAMERA_CPU, TIMELINE_STEREO,
    TIMELINE_RVE_SAD, TIMELINE_RVE_DMA, TIMELINE_STEREO_REFINE,
    TIMELINE_ZIPDEPTH_CPU, TIMELINE_NPU,
    TIMELINE_LANES
};

#define PROFILE_TIMELINE_SAMPLES 500

struct ProfileRange {
    uint64_t start, end;
};

struct ProfileCycle {
    uint64_t frame;
    struct ProfileRange cameras[2], stereo, neural;
};

void profile_trace(enum ProfileLane lane, uint64_t start, uint64_t end);
uint64_t profile_timeline(const struct ProfileCycle *cycle,
                          char rows[TIMELINE_LANES][PROFILE_TIMELINE_SAMPLES + 1]);

struct ProfileMetric {
    uint64_t calls, cpu_ns, wall_ns;
};

static inline void profile_merge(struct ProfileMetric *total, struct ProfileMetric sample)
{
    total->calls += sample.calls;
    total->cpu_ns += sample.cpu_ns;
    total->wall_ns += sample.wall_ns;
}

#if defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 199309L
#include <time.h>

struct ProfileStamp {
    uint64_t cpu_ns, wall_ns;
};

static inline uint64_t profile_clock(clockid_t clock)
{
    struct timespec now;
    return clock_gettime(clock, &now) ? 0 :
        (uint64_t)now.tv_sec * 1000000000 + now.tv_nsec;
}

static inline struct ProfileStamp profile_start(void)
{
    uint64_t wall = profile_clock(CLOCK_MONOTONIC);
    uint64_t cpu = profile_clock(CLOCK_THREAD_CPUTIME_ID);
    return (struct ProfileStamp){.cpu_ns = cpu, .wall_ns = wall};
}

static inline uint64_t profile_end(struct ProfileMetric *metric, struct ProfileStamp start)
{
    uint64_t cpu = profile_clock(CLOCK_THREAD_CPUTIME_ID);
    uint64_t wall = profile_clock(CLOCK_MONOTONIC);
    metric->calls++;
    metric->wall_ns += wall - start.wall_ns;
    metric->cpu_ns += cpu - start.cpu_ns;
    return wall;
}

static inline uint64_t profile_end_traced(struct ProfileMetric *metric, struct ProfileStamp start,
                                          enum ProfileLane lane)
{
    uint64_t end = profile_end(metric, start);
    profile_trace(lane, start.wall_ns, end);
    return end;
}

#endif

#endif
