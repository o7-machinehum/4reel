/* Run with depth.c -lm; add -DFOURREEL_RVE rve_sad.c tests/rve_mock.c
 * -Wl,--wrap=access to exercise the RVE path with emulated SAD costs. */
#include "../depth.h"
#include "../rve_sad.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PIXELS (DEPTH_W * DEPTH_H)
static uint8_t left[PIXELS], right[PIXELS], visual[PIXELS];
static uint16_t mm[PIXELS];

static void fill_texture(uint8_t *image, unsigned random)
{
    for (int i = 0; i < PIXELS; ++i) {
        random ^= random << 13;
        random ^= random >> 17;
        random ^= random << 5;
        image[i] = random;
    }
}

static void check_pair(struct Calibration *cal, int disparity, int vertical,
                       int distance)
{
    fill_texture(left, 0x12345678);
    memset(right, 0, sizeof(right));
    for (int y = 0; y < DEPTH_H; ++y)
        for (int x = 0; x < DEPTH_W; ++x) {
            int rx = x - disparity, ry = y + vertical;
            if (rx >= 0 && rx < DEPTH_W && ry >= 0 && ry < DEPTH_H)
                right[ry * DEPTH_W + rx] = left[y * DEPTH_W + x];
        }
    depth_compute(left, right, cal, visual, mm);
    unsigned valid = 0;
    double parallax = fabs(disparity - cal->offset_px);
    unsigned color = 1 + (unsigned)(fmin(parallax, MAX_DISPARITY) * 254 / MAX_DISPARITY + 0.5);
    for (int i = 0; i < PIXELS; ++i)
        if (mm[i] == distance) {
            assert(visual[i] == color);
            ++valid;
        }
    assert(valid > PIXELS / 4);
}

#ifdef FOURREEL_RVE
static void check_fractional_disparity(void)
{
    const struct Calibration calibration = {.valid = 1, .scale_px_mm = 4466.44482};
    static uint8_t texture[PIXELS];
    fill_texture(texture, 0x12345678);
    memset(left, 0, sizeof(left));
    memset(right, 0, sizeof(right));
    for (int y = 1; y + 1 < DEPTH_H; ++y)
        for (int x = 1; x + 1 < DEPTH_W; ++x) {
            unsigned sum = 0;
            const unsigned weights[] = {1, 2, 1};
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx)
                    sum += texture[(y + dy) * DEPTH_W + x + dx] *
                           weights[dy + 1] * weights[dx + 1];
            left[y * DEPTH_W + x] = (sum + 8) / 16;
        }
    /* Shift the right image 2.6 pixels using linear interpolation. */
    for (int y = 0; y < DEPTH_H; ++y)
        for (int x = 0; x + 3 < DEPTH_W; ++x) {
            int i = y * DEPTH_W + x;
            right[i] = (2 * left[i + 2] + 3 * left[i + 3] + 2) / 5;
        }
    depth_compute(left, right, &calibration, visual, mm);
    unsigned valid = 0, accurate = 0;
    for (int y = 12; y < DEPTH_H - 12; ++y)
        for (int x = 16; x < DEPTH_W - 16; ++x) {
            unsigned depth = mm[y * DEPTH_W + x];
            valid += depth != 0;
            accurate += fabs((double)depth - calibration.scale_px_mm / 2.6) < 150;
        }
    assert(valid > PIXELS / 3 && accurate * 2 > valid);

    depth_compute(left, left, &calibration, visual, mm);
    for (unsigned i = 0; i < PIXELS; ++i)
        assert(!mm[i]);
}

static void check_reverse_match(void)
{
    const struct Calibration calibration = {.valid = 1, .scale_px_mm = 4466.44482};
    fill_texture(left, 0x2468abcd);
    fill_texture(right, 0x942ab3de);
    for (int y = 40; y < 44; ++y)
        for (int dx = 0; dx < 4; ++dx) {
            int row = y * DEPTH_W;
            unsigned sample = 20 + ((y * 4 + dx) * 37) % 200;
            /* Both left blocks point to this right patch. The reversed
             * match is exact only at the second left location. */
            right[row + 72 + dx] = sample;
            left[row + 80 + dx] = sample + 4;
            left[row + 92 + dx] = sample;
        }
    depth_compute(left, right, &calibration, visual, mm);
    assert(!mm[40 * DEPTH_W + 80]);
    assert(mm[40 * DEPTH_W + 92] == 223);
}
#endif

int main(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        /* User's measured disparities, mirrored to exercise both conventions.
         * Distances are synthetic 200/600 mm, not assumptions about their setup. */
        struct CalPoint near = {60, 40, 60 - direction * 37, 42, 200};
        struct CalPoint far = {60, 40, 60 - direction * 18, 42, 600};
        struct Calibration cal;
        assert(calibration_solve(&near, &far, &cal, NULL, 0) == 0);
        assert(cal.valid);
        assert(fabs(cal.scale_px_mm - direction * 5700) < 1e-6);
        assert(fabs(cal.offset_px - direction * 8.5) < 1e-6);
        check_pair(&cal, direction * 37, 2, 200);
        check_pair(&cal, direction * 18, 2, 600);
        check_pair(&cal, direction * 27, 2, 308);

        /* Offset can make raw disparity and scale have opposite signs. */
        cal.offset_px = -direction * 20;
        cal.scale_px_mm = direction * 5000;
        cal.vertical_px = -2;
        check_pair(&cal, -direction * 10, -2, 500);
        cal.offset_px = -direction * 10;
        check_pair(&cal, 0, -2, 500);

        char error[160];
        far.rx = near.rx;
        assert(calibration_solve(&near, &far, &cal, error, sizeof(error)) < 0);
        assert(strstr(error, "differ by more than one pixel"));
        far.rx = near.rx + direction;
        assert(calibration_solve(&near, &far, &cal, error, sizeof(error)) < 0);
        /* Current board calibration predicts 85 px at 10 cm. */
        int anchor = direction > 0 ? 110 : 30;
        near = (struct CalPoint){anchor, 40, anchor - direction * 85, 43, 100};
        far = (struct CalPoint){anchor, 40, anchor - direction * 15, 43, 600};
        assert(calibration_solve(&near, &far, &cal, NULL, 0) == 0);
        assert(fabs(cal.scale_px_mm - direction * 8400) < 1e-6);
        assert(fabs(cal.offset_px - direction) < 1e-6);
        check_pair(&cal, direction * 85, 3, 100);
        check_pair(&cal, direction * 96, 3, 88);
        near.rx = anchor - direction * 97;
        assert(calibration_solve(&near, &far, &cal, error, sizeof(error)) < 0);
        assert(strstr(error, "maximum 96 pixels"));
    }
#ifdef FOURREEL_RVE
    check_fractional_disparity();
    check_reverse_match();
#endif
    assert(depth_self_test() == 0);
    rve_sad_close();
    puts("PASS: both disparity directions, offsets, vertical alignment, metric depth and color");
    return 0;
}
