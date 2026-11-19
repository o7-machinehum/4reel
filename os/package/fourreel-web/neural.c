#define _POSIX_C_SOURCE 200809L
#include "neural.h"
#include "image.h"

#include <stdio.h>
#include <string.h>

#ifdef FOURREEL_RKNN
#include <pthread.h>
#include <stdlib.h>
#include <time.h>
#include <rknn_api.h>

#define PIXELS (DEPTH_W * DEPTH_H)
#define NEURAL_DMA_BUDGET (16U * 1024 * 1024)
#define SCALE_CONFIRMATION_NS 500000000

void rknn_musl_init(void);

struct NeuralJob {
    uint8_t image[NEURAL_PIXELS];
    uint16_t stereo[PIXELS];
    uint8_t valid[PIXELS];
    uint64_t frame, queued_ns;
    struct ProfileCycle cycle;
    int fuse;
};

struct DepthScale {
    float slope, offset;
    uint64_t frame, captured_ns;
};

static struct {
    pthread_mutex_t lock;
    pthread_cond_t ready;
    pthread_t thread;
    int started, stop, pending;
    struct NeuralJob jobs[2];
    struct NeuralJob *queued;
    struct NeuralProfile profile;
    uint8_t gamma[256];
    struct NeuralResult result;
    rknn_context context;
    rknn_tensor_mem *input, *output;
    rknn_tensor_attr input_attr, output_attr;
    unsigned output_stride, output_channels;
} worker = {.lock = PTHREAD_MUTEX_INITIALIZER, .ready = PTHREAD_COND_INITIALIZER};

static int fail(const char *message, int code)
{
    snprintf(worker.result.error, sizeof(worker.result.error), "%s (%d)", message, code);
    return -1;
}

static void release_model(void)
{
    if (worker.output)
        rknn_destroy_mem(worker.context, worker.output);
    if (worker.input)
        rknn_destroy_mem(worker.context, worker.input);
    if (worker.context)
        rknn_destroy(worker.context);
    worker.output = worker.input = NULL;
    worker.context = 0;
}

static int load_model(const char *path)
{
    rknn_musl_init();
    int ret = rknn_init(&worker.context, (void *)path, 0, 0, NULL);
    if (ret < 0)
        return fail("ZipDepth model or NPU unavailable", ret);
    rknn_input_output_num count = {0};
    rknn_mem_size memory = {0};
    if (rknn_query(worker.context, RKNN_QUERY_IN_OUT_NUM, &count, sizeof(count)) ||
        count.n_input != 1 || count.n_output != 1 ||
        rknn_query(worker.context, RKNN_QUERY_MEM_SIZE, &memory, sizeof(memory)))
        return fail("unsupported ZipDepth model", -1);
    memset(&worker.input_attr, 0, sizeof(worker.input_attr));
    memset(&worker.output_attr, 0, sizeof(worker.output_attr));
    if (rknn_query(worker.context, RKNN_QUERY_INPUT_ATTR,
                   &worker.input_attr, sizeof(worker.input_attr)) ||
        rknn_query(worker.context, RKNN_QUERY_NATIVE_OUTPUT_ATTR,
                   &worker.output_attr, sizeof(worker.output_attr)))
        return fail("ZipDepth tensor query failed", -1);
    rknn_tensor_attr *input = &worker.input_attr;
    rknn_tensor_attr *output = &worker.output_attr;
    int input_ok = input->n_dims == 4 && input->dims[0] == 1 &&
        ((input->fmt == RKNN_TENSOR_NHWC && input->dims[1] == NEURAL_H &&
          input->dims[2] == NEURAL_W && input->dims[3] == 3) ||
         (input->fmt == RKNN_TENSOR_NCHW && input->dims[1] == 3 &&
          input->dims[2] == NEURAL_H && input->dims[3] == NEURAL_W));
    int output_ok = output->dims[0] == 1 &&
        ((output->fmt == RKNN_TENSOR_NC1HWC2 && output->n_dims == 5 &&
          output->dims[1] == 1 && output->dims[2] == NEURAL_H &&
          output->dims[3] == NEURAL_W &&
          (output->dims[4] == 16 || output->dims[4] == 32)) ||
         (output->fmt == RKNN_TENSOR_NHWC && output->n_dims == 4 &&
          output->dims[1] == NEURAL_H && output->dims[2] == NEURAL_W &&
          output->dims[3] == 1));
    if (!input_ok || !output_ok || output->type != RKNN_TENSOR_INT8 ||
        !isfinite(output->scale) || output->scale <= 0)
        return fail("expected INT8 ZipDepth 256x160 tensors", -1);
    input->type = RKNN_TENSOR_UINT8;
    input->fmt = RKNN_TENSOR_NHWC;
    input->dims[1] = NEURAL_H;
    input->dims[2] = NEURAL_W;
    input->dims[3] = 3;
    input->pass_through = 0;
    if (input->w_stride < NEURAL_W)
        input->w_stride = NEURAL_W;
    if (input->w_stride > NEURAL_W * 2)
        return fail("unsupported ZipDepth input stride", -1);
    unsigned input_bytes = input->w_stride * NEURAL_H * 3;
    worker.output_channels = output->fmt == RKNN_TENSOR_NC1HWC2 ? output->dims[4] : 1;
    worker.output_stride = output->w_stride ? output->w_stride : NEURAL_W;
    uint64_t required = (uint64_t)worker.output_stride * NEURAL_H * worker.output_channels;
    if (worker.output_stride < NEURAL_W || required > output->size_with_stride ||
        (uint64_t)memory.total_weight_size + memory.total_internal_size +
            input_bytes + output->size_with_stride > NEURAL_DMA_BUDGET)
        return fail("ZipDepth exceeds the board memory budget", -1);
    worker.input = rknn_create_mem(worker.context, input_bytes);
    worker.output = rknn_create_mem(worker.context, output->size_with_stride);
    if (!worker.input || !worker.output)
        return fail("ZipDepth DMA allocation failed", -1);
    memset(worker.input->virt_addr, 0, input_bytes);
    if (rknn_set_io_mem(worker.context, worker.input, input) ||
        rknn_set_io_mem(worker.context, worker.output, output))
        return fail("ZipDepth tensor binding failed", -1);
    for (unsigned i = 0; i < 256; ++i)
        worker.gamma[i] = image_display_luma(i);
    return 0;
}

static int infer(const struct NeuralJob *job, struct NeuralResult *result,
                 struct NeuralProfile *profile)
{
    struct ProfileStamp start = profile_start();
    uint8_t *input = worker.input->virt_addr;
    for (unsigned y = 0; y < NEURAL_H; ++y)
        for (unsigned x = 0; x < NEURAL_W; ++x) {
            uint8_t value = worker.gamma[job->image[y * NEURAL_W + x]];
            size_t at = (y * worker.input_attr.w_stride + x) * 3;
            input[at] = input[at + 1] = input[at + 2] = value;
        }
    profile_end_traced(&profile->pack, start, TIMELINE_ZIPDEPTH_CPU);
    start = profile_start();
    rknn_run_extend extend = {.timeout_ms = 1000};
    int ret = rknn_run(worker.context, &extend);
    profile_end_traced(&profile->run, start, TIMELINE_NPU);
    if (ret < 0)
        return ret;
    start = profile_start();
    const int8_t *output = worker.output->virt_addr;
    unsigned histogram[256] = {0};
    for (unsigned y = 0; y < DEPTH_H; ++y)
        for (unsigned x = 0; x < DEPTH_W; ++x) {
            unsigned nx = x * NEURAL_W / DEPTH_W;
            unsigned ny = y * NEURAL_H / DEPTH_H;
            result->input[y * DEPTH_W + x] = job->image[ny * NEURAL_W + nx];
            int value = output[(ny * worker.output_stride + nx) * worker.output_channels];
            histogram[value + 128]++;
            result->inverse_depth[y * DEPTH_W + x] =
                (value - worker.output_attr.zp) * worker.output_attr.scale;
            result->visual[y * DEPTH_W + x] = (uint8_t)(value + 128);
        }
    unsigned sum = 0, low = 0, high = 255;
    for (unsigned i = 0; i < 256; ++i) {
        sum += histogram[i];
        if (sum < PIXELS / 20)
            low = i;
        if (sum >= PIXELS * 19 / 20) {
            high = i;
            break;
        }
    }
    result->relative_low = ((int)low - 128 - worker.output_attr.zp) * worker.output_attr.scale;
    result->relative_high = ((int)high - 128 - worker.output_attr.zp) * worker.output_attr.scale;
    for (unsigned i = 0; i < PIXELS; ++i) {
        int value = result->visual[i];
        if (value < (int)low)
            value = (int)low;
        if (value > (int)high)
            value = (int)high;
        result->visual[i] = result->inverse_depth[i] > 0 && high > low ?
            (uint8_t)(1 + (value - (int)low) * 254 / (int)(high - low)) : 0;
    }
    profile_end_traced(&profile->decode, start, TIMELINE_ZIPDEPTH_CPU);
    return 0;
}

#define BENCHMARK_BANDS 5
struct StereoAnchor {
    uint16_t depth_mm;
    float relative;
};

static int compare_benchmark(const void *a, const void *b)
{
    const struct StereoAnchor *left = a;
    const struct StereoAnchor *right = b;
    return (int)left->depth_mm - (int)right->depth_mm;
}

static int supported_stereo_block(const struct NeuralJob *job, int x, int y)
{
    unsigned i = y * DEPTH_W + x;
    unsigned mm = job->stereo[i];
    if (!job->valid[i] || mm < STEREO_SEARCH_NEAR_MM || mm > STEREO_SEARCH_FAR_MM)
        return 0;
    float inverse = 1000.f / mm;
    float tolerance = fmaxf(0.1f, 0.2f * inverse);
    unsigned agreeing = 0;
    for (int dy = -4; dy <= 4; dy += 4)
        for (int dx = -4; dx <= 4; dx += 4) {
            int nx = x + dx, ny = y + dy;
            if ((!dx && !dy) || nx < 0 || ny < 0 || nx >= DEPTH_W || ny >= DEPTH_H)
                continue;
            unsigned p = ny * DEPTH_W + nx;
            unsigned neighbour = job->stereo[p];
            if (job->valid[p] && neighbour &&
                fabsf(1000.f / neighbour - inverse) <= tolerance && ++agreeing == 2)
                return 1;
        }
    return 0;
}

static int fit_stereo_benchmarks(const struct NeuralJob *job, const struct NeuralResult *result,
                                 float *slope, float *offset)
{
    struct StereoAnchor groups[BENCHMARK_BANDS][(DEPTH_W / 4) * (DEPTH_H / 4)];
    unsigned counts[BENCHMARK_BANDS] = {0};
    for (int y = 2; y < DEPTH_H; y += 4)
        for (int x = 2; x < DEPTH_W; x += 4) {
            unsigned i = y * DEPTH_W + x;
            unsigned colour = result->visual[i];
            if (!colour || !supported_stereo_block(job, x, y))
                continue;
            unsigned band = (colour - 1) * BENCHMARK_BANDS / 255;
            groups[band][counts[band]++] = (struct StereoAnchor){
                job->stereo[i], result->inverse_depth[i]
            };
        }

    struct StereoAnchor anchors[BENCHMARK_BANDS];
    unsigned used = 0;
    unsigned last_mm = UINT16_MAX;
    for (unsigned band = 0; band < BENCHMARK_BANDS; ++band) {
        unsigned count = counts[band];
        if (count < 8)
            continue;
        struct StereoAnchor *points = groups[band];
        qsort(points, count, sizeof(*points), compare_benchmark);
        unsigned low = count / 4, high = count - 1 - low;
        unsigned median = points[count / 2].depth_mm;
        /* Use spatially supported colour groups whose middle half is at
         * one measured depth, with independent near and far anchors. */
        if (points[high].depth_mm * 4 > points[low].depth_mm * 5 ||
            median * 100 >= last_mm * 95)
            continue;
        float sum = 0;
        for (unsigned i = low; i <= high; ++i)
            sum += points[i].relative;
        anchors[used++] = (struct StereoAnchor){median, sum / (high - low + 1)};
        last_mm = median;
    }
    if (used < 2)
        return 0;
    const struct StereoAnchor *far = &anchors[0], *near = &anchors[used - 1];
    float far_inverse = 1000.f / far->depth_mm;
    float delta = 1000.f / near->depth_mm - far_inverse;
    if (delta < 0.2f || near->relative <= far->relative)
        return 0;
    *slope = delta / (near->relative - far->relative);
    *offset = far_inverse - *slope * far->relative;
    for (unsigned i = 1; i + 1 < used; ++i) {
        float inverse = 1000.f / anchors[i].depth_mm;
        if (fabsf(anchors[i].relative * *slope + *offset - inverse) > 0.2f * inverse + 0.1f)
            return 0;
    }
    return isfinite(*slope) && isfinite(*offset);
}

static int scales_agree(const struct DepthScale *a, const struct DepthScale *b,
                        const struct NeuralResult *result)
{
    /* Compare the depths implied at the ends of the same prediction range.
     * A slope and offset can change together while giving similar depths. */
    const float levels[] = {result->relative_low, result->relative_high};
    for (unsigned i = 0; i < sizeof(levels) / sizeof(*levels); ++i) {
        float first = a->slope * levels[i] + a->offset;
        float second = b->slope * levels[i] + b->offset;
        if (fabsf(first - second) > 0.20f * fmaxf(fabsf(first), fabsf(second)) + 0.1f)
            return 0;
    }
    return 1;
}

static void remove_depth_spikes(const uint16_t *input, uint16_t *output)
{
    memcpy(output, input, PIXELS * sizeof(*input));
    for (unsigned y = 1; y < DEPTH_H - 1; ++y)
        for (unsigned x = 1; x < DEPTH_W - 1; ++x) {
            unsigned i = y * DEPTH_W + x;
            unsigned centre = input[i];
            if (!centre)
                continue;
            const unsigned neighbours[] = {
                input[i - 1], input[i + 1], input[i - DEPTH_W], input[i + DEPTH_W]
            };
            unsigned near = UINT16_MAX, far = 0, sum = 0;
            for (unsigned n = 0; n < 4; ++n) {
                if (neighbours[n] < near)
                    near = neighbours[n];
                if (neighbours[n] > far)
                    far = neighbours[n];
                sum += neighbours[n];
            }
            /* Keep boundaries and invalid pixels. Only remove an isolated
             * value when all four neighbours agree within 20%. */
            if (near && far * 5 <= near * 6 &&
                (centre * 2 > far * 3 || centre * 3 < near * 2))
                output[i] = (sum + 2) / 4;
        }
}

static void calibrate_prediction(const struct NeuralJob *job, struct NeuralResult *result,
                                 struct DepthScale *scale, struct DepthScale *pending)
{
    struct DepthScale candidate = {
        result->fusion.slope, result->fusion.offset, job->frame, job->queued_ns
    };
    if (result->fusion.fit == FUSION_FITTED ||
        fit_stereo_benchmarks(job, result, &candidate.slope, &candidate.offset)) {
        if (!scale->frame || scales_agree(scale, &candidate, result) ||
            (pending->frame && candidate.captured_ns - pending->captured_ns <= SCALE_CONFIRMATION_NS &&
             scales_agree(pending, &candidate, result))) {
            *scale = candidate;
            pending->frame = 0;
        } else {
            *pending = candidate;
        }
    }
    if (!scale->frame)
        return;

    uint16_t depth[PIXELS];
    for (unsigned i = 0; i < PIXELS; ++i) {
        float inverse = result->inverse_depth[i] * scale->slope + scale->offset;
        uint16_t mm = 0;
        if (job->valid[i] && inverse >= 1000.f / UINT16_MAX && inverse <= 1000.f)
            mm = (uint16_t)(1000.f / inverse + 0.5f);
        depth[i] = mm;
    }
    remove_depth_spikes(depth, result->metric_mm);
    result->metric_reference_frame = scale->frame;
    result->metric_frame = job->frame;
    result->metric_capture_ns = job->queued_ns;
}

static void *run_worker(void *unused)
{
    (void)unused;
    struct NeuralResult *next = calloc(1, sizeof(*next));
    struct DepthScale scale = {0}, pending = {0};
    pthread_mutex_lock(&worker.lock);
    if (!next) {
        fail("ZipDepth result allocation failed", -1);
        pthread_mutex_unlock(&worker.lock);
        return NULL;
    }
    while (!worker.stop) {
        while (!worker.pending && !worker.stop)
            pthread_cond_wait(&worker.ready, &worker.lock);
        if (worker.stop)
            break;
        const struct NeuralJob *job = worker.queued;
        worker.queued = &worker.jobs[job == &worker.jobs[0]];
        worker.pending = 0;
        uint64_t started = profile_clock(CLOCK_MONOTONIC);
        worker.profile.queue_ns += started - job->queued_ns;
        worker.profile.started++;
        pthread_mutex_unlock(&worker.lock);
        struct NeuralProfile sample = {0};
        int ret = infer(job, next, &sample);
        next->milliseconds = (profile_clock(CLOCK_MONOTONIC) - started) / 1000000.0;
        next->fusion = (struct FusionStats){0};
        if (!ret && job->fuse) {
            struct ProfileStamp fusion_start = profile_start();
            next->fusion = fuse_depth(next->inverse_depth, job->stereo, job->valid,
                                      next->fused_mm, next->source);
            calibrate_prediction(job, next, &scale, &pending);
            profile_end_traced(&sample.fusion, fusion_start, TIMELINE_ZIPDEPTH_CPU);
        }

        pthread_mutex_lock(&worker.lock);
        profile_merge(&worker.profile.pack, sample.pack);
        profile_merge(&worker.profile.run, sample.run);
        profile_merge(&worker.profile.decode, sample.decode);
        profile_merge(&worker.profile.fusion, sample.fusion);
        if (ret) {
            worker.result.available = 0;
            fail("ZipDepth inference failed", ret);
            break;
        }
        worker.profile.cycle = job->cycle;
        worker.profile.cycle.neural = (struct ProfileRange){started, profile_clock(CLOCK_MONOTONIC)};
        next->frame = job->frame;
        next->available = 1;
        worker.result = *next;
    }
    pthread_mutex_unlock(&worker.lock);
    free(next);
    return NULL;
}

int neural_start(const char *model_path)
{
    if (worker.started)
        return 0;
    memset(&worker.result, 0, sizeof(worker.result));
    worker.stop = worker.pending = 0;
    worker.queued = &worker.jobs[0];
    if (load_model(model_path)) {
        release_model();
        return -1;
    }
    pthread_attr_t attributes;
    pthread_attr_init(&attributes);
    int ret = pthread_attr_setstacksize(&attributes, 128 * 1024);
    if (!ret)
        ret = pthread_create(&worker.thread, &attributes, run_worker, NULL);
    pthread_attr_destroy(&attributes);
    if (ret) {
        release_model();
        return fail("ZipDepth worker failed to start", ret);
    }
    worker.started = 1;
    return 0;
}

void neural_stop(void)
{
    if (worker.started) {
        pthread_mutex_lock(&worker.lock);
        worker.stop = 1;
        pthread_cond_signal(&worker.ready);
        pthread_mutex_unlock(&worker.lock);
        pthread_join(worker.thread, NULL);
        worker.started = 0;
    }
    release_model();
    memset(&worker.result, 0, sizeof(worker.result));
    worker.profile.cycle = (struct ProfileCycle){0};
}

int neural_submit(const uint8_t *left, const uint16_t *stereo,
                  const uint8_t *valid, uint64_t frame, const struct ProfileCycle *cycle)
{
    if (!worker.started)
        return 0;
    pthread_mutex_lock(&worker.lock);
    int accepted = !worker.stop && !worker.result.error[0];
    if (accepted) {
        struct NeuralJob *job = worker.queued;
        worker.profile.dropped += worker.pending != 0;
        memcpy(job->image, left, NEURAL_PIXELS);
        job->fuse = stereo && valid;
        if (job->fuse) {
            memcpy(job->stereo, stereo, sizeof(job->stereo));
            memcpy(job->valid, valid, sizeof(job->valid));
        }
        job->frame = frame;
        job->cycle = cycle ? *cycle : (struct ProfileCycle){0};
        job->queued_ns = profile_clock(CLOCK_MONOTONIC);
        worker.pending = 1;
        pthread_cond_signal(&worker.ready);
    }
    pthread_mutex_unlock(&worker.lock);
    return accepted;
}

void neural_snapshot(struct NeuralResult *result)
{
    pthread_mutex_lock(&worker.lock);
    if (result->frame != worker.result.frame ||
        strcmp(result->error, worker.result.error) ||
        result->available != worker.result.available)
        *result = worker.result;
    pthread_mutex_unlock(&worker.lock);
}

void neural_profile(struct NeuralProfile *profile)
{
    pthread_mutex_lock(&worker.lock);
    *profile = worker.profile;
    pthread_mutex_unlock(&worker.lock);
}

#else
void neural_profile(struct NeuralProfile *profile) { memset(profile, 0, sizeof(*profile)); }
int neural_start(const char *model_path) { (void)model_path; return -1; }
void neural_stop(void) {}
int neural_submit(const uint8_t *left, const uint16_t *stereo,
                  const uint8_t *valid, uint64_t frame, const struct ProfileCycle *cycle)
{
    (void)left; (void)stereo; (void)valid; (void)frame; (void)cycle; return 0;
}
void neural_snapshot(struct NeuralResult *result)
{
    memset(result, 0, sizeof(*result));
    snprintf(result->error, sizeof(result->error), "ZipDepth was not compiled into this image");
}
#endif
