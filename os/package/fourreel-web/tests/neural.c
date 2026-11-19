#define _POSIX_C_SOURCE 200809L
#include "../neural.h"
#include "../image.h"
#include <rknn_api.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#define PIXELS (DEPTH_W * DEPTH_H)
#define INPUT_STRIDE 272
#define OUTPUT_STRIDE 288
static rknn_tensor_mem *input, *output;
static int contexts, allocations, allocation_calls, runs;
static int bad_dimensions, oversized, fail_allocation, fail_run, compact;
static unsigned output_shift;
static int output_spike;

void rknn_musl_init(void) {}
int rknn_init(rknn_context *context, void *model, uint32_t size,
              uint32_t flags, rknn_init_extend *extend)
{
    (void)extend;
    assert(model && !size && !flags && !contexts);
    *context = 1;
    contexts++;
    allocation_calls = runs = 0;
    return 0;
}
int rknn_destroy(rknn_context context)
{
    assert(context == 1 && contexts == 1 && !allocations);
    contexts--;
    return 0;
}
int rknn_query(rknn_context context, rknn_query_cmd command, void *data, uint32_t size)
{
    assert(context == 1);
    memset(data, 0, size);
    if (command == RKNN_QUERY_IN_OUT_NUM) {
        rknn_input_output_num *count = data;
        count->n_input = count->n_output = 1;
    } else if (command == RKNN_QUERY_MEM_SIZE) {
        rknn_mem_size *memory = data;
        memory->total_weight_size = oversized ? 17 * 1024 * 1024 : 6200000;
        memory->total_internal_size = 3800000;
    } else {
        rknn_tensor_attr *attribute = data;
        if (command == RKNN_QUERY_INPUT_ATTR) {
            attribute->n_dims = 4;
            attribute->dims[0] = 1;
            attribute->dims[1] = 3;
            attribute->dims[2] = NEURAL_H;
            attribute->dims[3] = NEURAL_W + bad_dimensions;
            attribute->fmt = RKNN_TENSOR_NCHW;
            attribute->w_stride = INPUT_STRIDE;
        } else {
            assert(command == RKNN_QUERY_NATIVE_OUTPUT_ATTR);
            attribute->n_dims = compact ? 4 : 5;
            attribute->dims[0] = 1;
            attribute->dims[1] = compact ? NEURAL_H : 1;
            attribute->dims[2] = compact ? NEURAL_W : NEURAL_H;
            attribute->dims[3] = compact ? 1 : NEURAL_W;
            attribute->dims[4] = compact ? 0 : 16;
            attribute->fmt = compact ? RKNN_TENSOR_NHWC : RKNN_TENSOR_NC1HWC2;
            attribute->type = RKNN_TENSOR_INT8;
            attribute->scale = 0.25f;
            attribute->zp = -128;
            attribute->w_stride = OUTPUT_STRIDE;
            attribute->size_with_stride = NEURAL_H * OUTPUT_STRIDE * (compact ? 1 : 16);
        }
    }
    return 0;
}
rknn_tensor_mem *rknn_create_mem(rknn_context context, uint32_t size)
{
    assert(context == 1);
    if (++allocation_calls == fail_allocation)
        return NULL;
    rknn_tensor_mem *memory = calloc(1, sizeof(*memory));
    assert(memory);
    memory->virt_addr = malloc(size);
    assert(memory->virt_addr);
    memset(memory->virt_addr, 0xa5, size);
    memory->size = size;
    allocations++;
    return memory;
}
int rknn_destroy_mem(rknn_context context, rknn_tensor_mem *memory)
{
    assert(context == 1 && memory && allocations);
    free(memory->virt_addr);
    free(memory);
    allocations--;
    return 0;
}
int rknn_set_io_mem(rknn_context context, rknn_tensor_mem *memory, rknn_tensor_attr *attribute)
{
    assert(context == 1);
    if (attribute->type == RKNN_TENSOR_UINT8) {
        assert(attribute->fmt == RKNN_TENSOR_NHWC && !attribute->pass_through);
        assert(attribute->dims[1] == NEURAL_H && attribute->dims[2] == NEURAL_W && attribute->dims[3] == 3);
        assert(memory->size == INPUT_STRIDE * NEURAL_H * 3);
        input = memory;
    } else output = memory;
    return 0;
}
int rknn_run(rknn_context context, rknn_run_extend *extend)
{
    assert(context == 1 && extend->timeout_ms == 1000);
    if (fail_run) return -7;
    runs++;
    const uint8_t *rgb = input->virt_addr;
    uint8_t expected = image_display_luma(80);
    for (unsigned y = 0; y < NEURAL_H; ++y)
        for (unsigned x = 0; x < INPUT_STRIDE; ++x)
            for (unsigned c = 0; c < 3; ++c)
                assert(rgb[(y * INPUT_STRIDE + x) * 3 + c] == (x < NEURAL_W ? expected : 0));
    memset(output->virt_addr, 0x7f, output->size);
    int8_t *values = output->virt_addr;
    for (unsigned y = 0; y < NEURAL_H; ++y)
        for (unsigned x = 0; x < NEURAL_W; ++x)
            values[(y * OUTPUT_STRIDE + x) * (compact ? 1 : 16)] =
                output_spike && x == 64 * NEURAL_W / DEPTH_W && y == 50 * NEURAL_H / DEPTH_H ?
                -124 : (int)((x + y + output_shift) % 256) - 128;
    return 0;
}
static void wait_result(struct NeuralResult *result, int error)
{
    for (int i = 0; i < 200; ++i) {
        neural_snapshot(result);
        if (error ? !!result->error[0] : result->available) return;
        nanosleep(&(struct timespec){.tv_nsec = 10000000}, NULL);
    }
    assert(!"neural worker failed to publish a result");
}

static void wait_frame(struct NeuralResult *result, uint64_t frame)
{
    for (int i = 0; i < 200; ++i) {
        neural_snapshot(result);
        if (result->frame == frame)
            return;
        nanosleep(&(struct timespec){.tv_nsec = 10000000}, NULL);
    }
    assert(!"neural worker missed requested frame");
}
int main(void)
{
    static uint8_t left[NEURAL_PIXELS], valid[PIXELS];
    static uint16_t stereo[PIXELS];
    static struct NeuralResult result;
    memset(left, 80, sizeof(left));
    memset(valid, 1, sizeof(valid));
    for (unsigned y = 0; y < DEPTH_H; ++y)
        for (unsigned x = 0; x < DEPTH_W; ++x) {
            unsigned value = (x * NEURAL_W / DEPTH_W + y * NEURAL_H / DEPTH_H) % 256;
            if ((x / 4 + y / 4) % 2)
                stereo[y * DEPTH_W + x] = (uint16_t)(1000 / (1.0 + value * 0.02) + 0.5);
        }
    for (compact = 0; compact <= 1; ++compact) {
        memset(&result, 0, sizeof(result));
        assert(!neural_start("mock.rknn") && !neural_start("mock.rknn"));
        struct NeuralProfile before, after;
        neural_profile(&before);
        assert(neural_submit(left, stereo, valid, 42, NULL));
        wait_result(&result, 0);
        assert(result.frame == 42 && result.available && !result.error[0]);
        assert(runs == 1 && result.fusion.fit == FUSION_FITTED);
        assert(fabsf(result.fusion.slope - 0.08f) < 0.001f);
        assert(fabsf(result.fusion.offset - 1.0f) < 0.01f);
        assert(result.fusion.measured == PIXELS / 2 && result.fusion.estimated > PIXELS / 3);
        assert(result.source[50 * DEPTH_W + 52] == FUSION_STEREO);
        assert(result.source[50 * DEPTH_W + 56] == FUSION_ESTIMATED);
        assert(!stereo[50 * DEPTH_W + 56] && result.metric_mm[50 * DEPTH_W + 56]);
        for (unsigned y = 0; y < DEPTH_H; ++y)
            for (unsigned x = 0; x < DEPTH_W; ++x) {
                unsigned value = (x * NEURAL_W / DEPTH_W + y * NEURAL_H / DEPTH_H) % 256;
                assert(result.inverse_depth[y * DEPTH_W + x] == value * 0.25f);
                assert(result.input[y * DEPTH_W + x] == 80);
                int expected_mm = (int)(1000 / (1.0 + value * 0.02) + 0.5);
                assert(abs((int)result.metric_mm[y * DEPTH_W + x] - expected_mm) <= 2);
                if (stereo[y * DEPTH_W + x])
                    assert(result.fused_mm[y * DEPTH_W + x] == stereo[y * DEPTH_W + x]);
            }
        neural_profile(&after);
        assert(after.run.calls == before.run.calls + 1 && after.fusion.calls == before.fusion.calls + 1);

        /* An isolated scale jump must not recolour the frame. The same fit
         * on a second frame is accepted so a sustained scene change works. */
        static uint16_t close_stereo[PIXELS];
        for (unsigned i = 0; i < PIXELS; ++i)
            close_stereo[i] = stereo[i] / 2;
        unsigned sample = 50 * DEPTH_W + 64;
        unsigned prior = result.metric_mm[sample];
        output_spike = 1;
        assert(neural_submit(left, close_stereo, valid, 43, NULL));
        wait_frame(&result, 43);
        assert(result.fusion.fit == FUSION_FITTED && result.metric_reference_frame == 42);
        assert(result.metric_frame == 43);
        assert(abs((int)result.metric_mm[sample] - (int)prior) <= 2);
        output_spike = 0;
        assert(neural_submit(left, close_stereo, valid, 44, NULL));
        wait_frame(&result, 44);
        assert(result.metric_reference_frame == 44 && result.metric_mm[sample] * 3 < prior * 2);

        neural_stop();
        assert(!contexts && !allocations && !neural_submit(left, NULL, NULL, 43, NULL));
    }
    /* A flat prediction cannot set the metric scale. Preserve every
     * measured point and leave every unsupported hole invalid. */
    for (unsigned i = 0; i < PIXELS; ++i)
        result.inverse_depth[i] = 1.0f;
    struct FusionStats fallback = fuse_depth(result.inverse_depth, stereo, valid,
                                             result.fused_mm, result.source);
    assert(fallback.fit == FUSION_REJECTED && !fallback.estimated);
    for (unsigned i = 0; i < PIXELS; ++i) {
        assert(result.fused_mm[i] == stereo[i]);
        assert(result.source[i] == (stereo[i] ? FUSION_STEREO : FUSION_INVALID));
    }
    /* Stable near and far surfaces still provide useful labels when the
     * middle colours have incompatible stereo readings. */
    for (unsigned y = 0; y < DEPTH_H; ++y)
        for (unsigned x = 0; x < DEPTH_W; ++x) {
            unsigned value = (x * NEURAL_W / DEPTH_W + y * NEURAL_H / DEPTH_H) % 256;
            stereo[y * DEPTH_W + x] = value < 45 ? 2233 : value > 200 ? 106 :
                (x / 12 + y / 12) % 2 ? 300 : 3500;
        }
    memset(&result, 0, sizeof(result));
    assert(!neural_start("mock.rknn") && neural_submit(left, stereo, valid, 46, NULL));
    wait_result(&result, 0);
    assert(result.fusion.fit == FUSION_REJECTED);
    assert(result.metric_frame == 46);
    unsigned pixel = 50 * DEPTH_W + 50;
    assert(result.metric_mm[pixel] > 0 && result.metric_mm[pixel] != stereo[pixel]);
    uint16_t known_metric = result.metric_mm[pixel];
    memset(stereo, 0, sizeof(stereo));
    output_shift = 15;
    assert(neural_submit(left, stereo, valid, 47, NULL));
    wait_frame(&result, 47);
    assert(result.frame == 47 && result.fusion.fit == FUSION_INSUFFICIENT);
    assert(result.metric_frame == 47 && result.metric_reference_frame == 46);
    assert(result.metric_mm[pixel] > 0 && result.metric_mm[pixel] < known_metric);
    output_shift = 0;
    neural_stop();

    bad_dimensions = 1;
    assert(neural_start("mock.rknn") < 0 && !contexts && !allocations);
    bad_dimensions = 0;
    oversized = 1;
    assert(neural_start("mock.rknn") < 0 && !contexts && !allocations);
    oversized = 0;
    fail_allocation = 2;
    assert(neural_start("mock.rknn") < 0 && !contexts && !allocations);
    fail_allocation = 0;
    fail_run = 1;
    memset(&result, 0, sizeof(result));
    assert(!neural_start("mock.rknn") && neural_submit(left, NULL, NULL, 44, NULL));
    wait_result(&result, 1);
    assert(!result.available && strstr(result.error, "inference failed"));
    assert(!neural_submit(left, NULL, NULL, 45, NULL));
    neural_stop();
    assert(!contexts && !allocations);
    puts("PASS: neural worker, grayscale input, padded native/compact tensors, one NPU call, same-frame fusion, profiling, memory guards and cleanup");
    return 0;
}
