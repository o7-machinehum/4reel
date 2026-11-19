#define _POSIX_C_SOURCE 200809L
#include "rve_sad.h"
#include "rve_abi.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define BLOCK 4
#define COLS (DEPTH_W / BLOCK)
#define ROWS (DEPTH_H / BLOCK)
#define COUNT (COLS * ROWS)
#define OUT_STRIDE ((COLS + 15) & ~15)
#define IMAGE_BYTES (DEPTH_W * DEPTH_H)
#define COST_BYTES (OUT_STRIDE * ROWS * 2)
_Static_assert(DEPTH_W % 16 == 0 && DEPTH_H % 4 == 0, "SAD dimensions");

static void *memory[3];
static RveImage input[2], output;
static int initialized, state;
static char status[96] = "RVE SAD pending first frame";
static double elapsed_ms;
static unsigned passes;
static struct RveProfile profile;
static uint16_t costs[2 * MAX_DISPARITY + 1][COUNT];

const char *rve_sad_status(void) { return status; }
double rve_sad_milliseconds(void) { return elapsed_ms; }
unsigned rve_sad_passes(void) { return passes; }

struct RveProfile rve_profile(void) { return profile; }

void rve_sad_close(void)
{
    if (initialized)
        RK_MPI_IVE_Deinit();
    initialized = 0;
    if (state > 0) {
        state = 0;
        snprintf(status, sizeof(status), "RVE SAD pending first frame");
    }
    for (int i = 0; i < 3; ++i) {
        if (memory[i])
            RK_MPI_MMZ_Free(memory[i]);
        memory[i] = NULL;
    }
}

static int fail(const char *reason)
{
    snprintf(status, sizeof(status), "CPU fallback: %s", reason);
    fprintf(stderr, "4reel depth: %s\n", status);
    state = -1;
    rve_sad_close();
    return -1;
}

static void image_init(RveImage *image, void *block, int width, int height,
                       int stride, int type)
{
    memset(image, 0, sizeof(*image));
    image->width = width;
    image->height = height;
    image->stride[0] = stride; /* SDK strides count pixels, including U16. */
    image->type = type;
    image->physical[0] = RK_MPI_MB_Handle2PhysAddr(block);
    image->virtual_address[0] = (uintptr_t)RK_MPI_MB_Handle2VirAddr(block);
}

static int submit(void)
{
    /* MB_4X4, SAD-only, full 16-bit cost (0..4080). */
    RveSadControl control = {.mode = 0, .output_mode = 0, .output_bits = 1};
    int32_t handle = 0;
    struct ProfileStamp start = profile_start();
    int ret = RK_MPI_IVE_SAD(&handle, &input[0], &input[1], &output, NULL,
                             &control, true);
    profile_end_traced(&profile.sad, start, TIMELINE_RVE_SAD);
    return ret;
}

static int sync_block(int index, int end)
{
    void *address = RK_MPI_MB_Handle2VirAddr(memory[index]);
    uint32_t size = RK_MPI_MMZ_GetSize(memory[index]);
    struct ProfileStamp start = profile_start();
    int ret = end ? RK_MPI_MMZ_FlushCacheVaddrEnd(address, size, 2) :
                    RK_MPI_MMZ_FlushCacheVaddrStart(address, size, 2);
    profile_end_traced(&profile.dma, start, TIMELINE_RVE_DMA);
    return ret;
}

int rve_sad_self_test(void)
{
    if (state)
        return state > 0 ? 0 : -1;
    if (access("/dev/rve", R_OK | W_OK))
        return fail("RVE device unavailable");
    for (int i = 0; i < 3; ++i)
        if (RK_MPI_MMZ_Alloc(&memory[i], i == 2 ? COST_BYTES : IMAGE_BYTES, 1))
            return fail("CMA allocation or physical-address lookup failed");
    image_init(&input[0], memory[0], DEPTH_W, DEPTH_H, DEPTH_W, 0);
    image_init(&input[1], memory[1], DEPTH_W, DEPTH_H, DEPTH_W, 0);
    image_init(&output, memory[2], COLS, ROWS, OUT_STRIDE, 9);
    if (RK_MPI_IVE_Init())
        return fail("RVE initialization failed");
    initialized = 1;

    /* Compare every returned hardware cost against a CPU reference. Two
     * patterns verify stride, 16-bit output, cache visibility and completion. */
    for (int test = 0; test < 2; ++test) {
        for (int i = 0; i < 3; ++i)
            if (sync_block(i, 0))
                return fail("DMA synchronization failed");
        uint8_t *a = RK_MPI_MB_Handle2VirAddr(memory[0]);
        uint8_t *b = RK_MPI_MB_Handle2VirAddr(memory[1]);
        uint16_t *cost = RK_MPI_MB_Handle2VirAddr(memory[2]);
        for (int i = 0; i < IMAGE_BYTES; ++i) {
            a[i] = test ? (uint8_t)(i * 37 + i / DEPTH_W * 13) : 0;
            b[i] = test ? (uint8_t)(i * 19 + 11) : 255;
        }
        memset(cost, 0xa5, COST_BYTES);
        for (int i = 0; i < 3; ++i)
            if (sync_block(i, 1))
                return fail("DMA synchronization failed");
        if (submit())
            return fail("SAD self-test submission failed");
        if (sync_block(2, 0))
            return fail("SAD self-test synchronization failed");
        int correct = 1;
        for (int y = 0; y < ROWS; ++y)
            for (int x = 0; x < COLS; ++x) {
                unsigned reference = 0;
                for (int dy = 0; dy < BLOCK; ++dy)
                    for (int dx = 0; dx < BLOCK; ++dx) {
                        int i = (y * BLOCK + dy) * DEPTH_W + x * BLOCK + dx;
                        int av = test ? (uint8_t)(i * 37 + i / DEPTH_W * 13) : 0;
                        int bv = test ? (uint8_t)(i * 19 + 11) : 255;
                        reference += av > bv ? av - bv : bv - av;
                    }
                if (cost[y * OUT_STRIDE + x] != reference)
                    correct = 0;
            }
        if (sync_block(2, 1))
            return fail("SAD self-test synchronization failed");
        if (!correct)
            return fail("hardware SAD did not match CPU reference");
    }
    state = 1;
    snprintf(status, sizeof(status), "RVE SAD 4x4 with subpixel refinement and left-right check");
    return 0;
}

static unsigned block_sad(const uint8_t *left, const uint8_t *right, unsigned limit)
{
    unsigned cost = 0;
    for (int y = 0; y < BLOCK; ++y) {
        cost += abs(left[0] - right[0]) + abs(left[1] - right[1]) +
                abs(left[2] - right[2]) + abs(left[3] - right[3]);
        if (cost > limit)
            break;
        left += DEPTH_W;
        right += DEPTH_W;
    }
    return cost;
}

static int consistent_match(const uint8_t *left, const uint8_t *right_block,
                            const struct StereoValidity *validity, int y, int right_x,
                            int min_d, int max_d, int best_d, unsigned best)
{
    /* The RVE scans on the left grid. The reverse check anchors at the exact
     * matched right location, which can lie between right-grid blocks. */
    int first = min_d > -right_x ? min_d : -right_x;
    int last = max_d < DEPTH_W - BLOCK - right_x ? max_d : DEPTH_W - BLOCK - right_x;
    const uint8_t *row = left + y * DEPTH_W;
    for (int d = best_d - 1; d <= best_d + 1; d += 2) {
        int x = right_x + d;
        if (d >= first && d <= last && stereo_window_valid(validity, 0, x, y, BLOCK)) {
            unsigned cost = block_sad(row + x, right_block, best);
            if (cost < best)
                best = cost;
        }
    }
    for (int d = first; d <= last; ++d) {
        int x = right_x + d;
        if (abs(d - best_d) > 1 && stereo_window_valid(validity, 0, x, y, BLOCK) &&
            block_sad(row + x, right_block, best) <= best)
            return 0;
    }
    return 1;
}

static double refined_disparity(int disparity, int min_d, int max_d, int block)
{
    unsigned centre = costs[disparity + MAX_DISPARITY][block];
    if (!centre || disparity == min_d || disparity == max_d)
        return disparity;
    unsigned before = costs[disparity - 1 + MAX_DISPARITY][block];
    unsigned after = costs[disparity + 1 + MAX_DISPARITY][block];
    if (before == UINT16_MAX || after == UINT16_MAX || before + after <= 2 * centre)
        return disparity;
    /* Fit the minimum of a parabola through the three adjacent SAD costs. */
    return disparity + ((double)before - after) / (2 * (before + after - 2 * centre));
}

int rve_depth_compute(const uint8_t *left, const uint8_t *right,
                      int min_d, int max_d, int vertical,
                      const struct Calibration *cal,
                      const struct StereoValidity *validity,
                      uint8_t *visual, uint16_t *depth_mm)
{
    elapsed_ms = 0;
    passes = 0;
    if (rve_sad_self_test())
        return -1;
    uint16_t best[COUNT];
    int16_t disparity[COUNT];
    uint8_t textured[COUNT];
    memset(best, 0xff, sizeof(best));
    memset(disparity, 0, sizeof(disparity));
    for (int y = 0; y < ROWS; ++y)
        for (int x = 0; x < COLS; ++x) {
            int lo = 255, hi = 0;
            for (int dy = 0; dy < BLOCK; ++dy)
                for (int dx = 0; dx < BLOCK; ++dx) {
                    int v = left[(y * BLOCK + dy) * DEPTH_W + x * BLOCK + dx];
                    if (v < lo) lo = v;
                    if (v > hi) hi = v;
                }
            textured[y * COLS + x] = hi - lo >= 12 &&
                stereo_window_valid(validity, 0, x * BLOCK, y * BLOCK, BLOCK);
        }
    if (sync_block(0, 0))
        return fail("left-image DMA synchronization failed");
    memcpy(RK_MPI_MB_Handle2VirAddr(memory[0]), left, IMAGE_BYTES);
    if (sync_block(0, 1))
        return fail("left-image DMA synchronization failed");
    for (int d = min_d; d <= max_d; ++d) {
        uint16_t *plane = costs[d + MAX_DISPARITY];
        memset(plane, 0xff, sizeof(costs[0]));
        if (sync_block(1, 0))
            return fail("right-image DMA synchronization failed");
        uint8_t *shifted = RK_MPI_MB_Handle2VirAddr(memory[1]);
        memset(shifted, 0, IMAGE_BYTES);
        int first_x = d > 0 ? d : 0;
        int last_x = d < 0 ? DEPTH_W + d : DEPTH_W;
        for (int y = 0; y < DEPTH_H; ++y)
            if (y + vertical >= 0 && y + vertical < DEPTH_H)
                memcpy(shifted + y * DEPTH_W + first_x,
                       right + (y + vertical) * DEPTH_W + first_x - d,
                       last_x - first_x);
        if (sync_block(1, 1))
            return fail("right-image DMA synchronization failed");
        uint64_t previous = profile.sad.wall_ns;
        if (submit())
            return fail("hardware SAD submission failed");
        elapsed_ms += (profile.sad.wall_ns - previous) / 1000000.0;
        ++passes;
        if (sync_block(2, 0))
            return fail("cost-buffer DMA synchronization failed");
        const uint16_t *cost = RK_MPI_MB_Handle2VirAddr(memory[2]);
        for (int y = 0; y < ROWS; ++y) {
            if (y * BLOCK + vertical < 0 ||
                (y + 1) * BLOCK + vertical > DEPTH_H)
                continue;
            for (int x = 0; x < COLS; ++x) {
                int i = y * COLS + x;
                if (!textured[i] || x * BLOCK < first_x ||
                    (x + 1) * BLOCK > last_x ||
                    !stereo_window_valid(validity, 1, x * BLOCK - d,
                                         y * BLOCK + vertical, BLOCK))
                    continue;
                uint16_t value = cost[y * OUT_STRIDE + x];
                plane[i] = value;
                if (value < best[i]) {
                    best[i] = value;
                    disparity[i] = d;
                }
            }
        }
        if (sync_block(2, 1))
            return fail("cost-buffer DMA synchronization failed");
    }
    struct ProfileStamp refinement = profile_start();
    for (int y = 0; y < ROWS; ++y)
        for (int x = 0; x < COLS; ++x) {
            int i = y * COLS + x;
            if (best[i] > 384)
                continue;
            int winner = disparity[i];
            unsigned alternative = UINT16_MAX;
            for (int d = min_d; d <= max_d; ++d) {
                unsigned cost = costs[d + MAX_DISPARITY][i];
                if (abs(d - winner) > 1 && cost < alternative)
                    alternative = cost;
            }
            /* Adjacent shifts participate in the same subpixel minimum. */
            if (alternative == UINT16_MAX || alternative - best[i] < 32 ||
                alternative * 10 < best[i] * 12)
                continue;
            int right_x = x * BLOCK - winner;
            const uint8_t *right_block = right + (y * BLOCK + vertical) * DEPTH_W + right_x;
            if (!consistent_match(left, right_block, validity, y * BLOCK, right_x, min_d, max_d,
                                  winner, best[i]))
                continue;
            double parallax = refined_disparity(winner, min_d, max_d, i) -
                              (cal && cal->valid ? cal->offset_px : 0);
            if (fabs(parallax) < 0.5 || (!(cal && cal->valid) && parallax < 0))
                continue;
            uint16_t mm = 0;
            if (cal && cal->valid) {
                double distance = cal->scale_px_mm / parallax;
                if (!isfinite(distance) || distance < 1 || distance > 65535)
                    continue;
                mm = (uint16_t)(distance + 0.5);
            }
            uint8_t color = 1 + (uint8_t)(fmin(fabs(parallax), MAX_DISPARITY) *
                                        254 / MAX_DISPARITY + 0.5);
            for (int dy = 0; dy < BLOCK; ++dy)
                for (int dx = 0; dx < BLOCK; ++dx) {
                    int p = (y * BLOCK + dy) * DEPTH_W + x * BLOCK + dx;
                    visual[p] = color;
                    depth_mm[p] = mm;
                }
        }
    profile_end_traced(&profile.refine, refinement, TIMELINE_STEREO_REFINE);
    return 0;
}
