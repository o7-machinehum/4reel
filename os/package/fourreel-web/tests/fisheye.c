#include "../fisheye.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void)
{
    struct FisheyeCalibration source = {.fx = 80, .fy = 80, .cx = 80,
        .cy = 50, .baseline_mm = 40, .rms = 0.2, .valid = 1};
    for (int side = 0; side < 2; ++side) {
        source.camera[side] = (struct FisheyeCamera){.fx = 300, .fy = 300,
            .cx = 640, .cy = 400, .rotation = {1, 0, 0, 0, 1, 0, 0, 0, 1}};
    }
    char text[FISHEYE_TEXT_BYTES], error[160];
    struct FisheyeCalibration parsed = {0};
    assert(fisheye_format(&source, text, sizeof(text)) > 0);
    assert(!fisheye_parse(text, &parsed, error, sizeof(error)) && parsed.valid);
    assert(parsed.baseline_mm == 40 && parsed.camera[1].rotation[8] == 1);
    source.baseline_mm = -40;
    assert(fisheye_format(&source, text, sizeof(text)) > 0);
    assert(!fisheye_parse(text, &parsed, error, sizeof(error)) && parsed.baseline_mm == -40);
    struct FisheyeCalibration previous = parsed;
    assert(fisheye_parse("4REELFISHEYE1 1280 800 160 100 nan", &parsed, error, sizeof(error)));
    assert(!memcmp(&previous, &parsed, sizeof(parsed)));
    strcat(text, " garbage");
    assert(fisheye_parse(text, &parsed, error, sizeof(error)));
    source.camera[0].rotation[0] = -1;
    assert(fisheye_format(&source, text, sizeof(text)) > 0);
    assert(fisheye_parse(text, &parsed, error, sizeof(error)));
    source.camera[0].rotation[0] = 1;
    source.rms = 4;
    assert(fisheye_format(&source, text, sizeof(text)) > 0);
    assert(fisheye_parse(text, &parsed, error, sizeof(error)));
    source.rms = 0.2;
    assert(fisheye_format(&source, text, 10) < 0);
    source.camera[0].model = source.camera[1].model = FISHEYE_DOUBLE_SPHERE;
    source.camera[0].xi = source.camera[1].xi = -0.18;
    source.camera[0].alpha = source.camera[1].alpha = 0.59;
    source.rms = -1;
    assert(fisheye_format(&source, text, sizeof(text)) > 0 && strstr(text, "4REELFISHEYE2"));
    assert(!fisheye_parse(text, &parsed, error, sizeof(error)));
    assert(parsed.camera[0].model == FISHEYE_DOUBLE_SPHERE && parsed.rms == -1);
    source.camera[0].alpha = 1.1;
    assert(fisheye_format(&source, text, sizeof(text)) > 0);
    assert(fisheye_parse(text, &parsed, error, sizeof(error)));
    source.camera[0].alpha = 0.59;
    source.camera[0].distortion[0] = 0.01;
    assert(fisheye_format(&source, text, sizeof(text)) > 0);
    assert(fisheye_parse(text, &parsed, error, sizeof(error)));
    source.camera[0].distortion[0] = 0;
    source.camera[0].model = source.camera[1].model = FISHEYE_EQUIDISTANT;
    source.camera[0].xi = source.camera[1].xi = 0;
    source.camera[0].alpha = source.camera[1].alpha = 0;
    source.rms = 0.2;

    struct FisheyeMaps *maps = malloc(sizeof(*maps));
    unsigned stride = FISHEYE_RAW_W * 5 / 4 + 32;
    uint8_t *raw = malloc((size_t)stride * FISHEYE_RAW_H);
    assert(maps && raw);
    memset(raw, 0xcc, (size_t)stride * FISHEYE_RAW_H);
    /* Every RAW10 sample is 48 * 4; padding must never be sampled. */
    for (unsigned y = 0; y < FISHEYE_RAW_H; ++y)
        for (unsigned x = 0; x < FISHEYE_RAW_W; x += 4) {
            uint64_t packed = 0;
            for (int i = 0; i < 4; ++i)
                packed |= (uint64_t)192 << (10 * i);
            for (int byte = 0; byte < 5; ++byte)
                raw[y * stride + x / 4 * 5 + byte] = (uint8_t)(packed >> (8 * byte));
        }
    fisheye_build_maps(&source, maps);
    unsigned centre = 50 * DEPTH_W + 80;
    assert(maps->xy[0][centre][0] == (FISHEYE_RAW_W - 1 - 640) * 16);
    assert(maps->xy[0][centre][1] == 400 * 16);
    uint8_t image[DEPTH_W * DEPTH_H];
    fisheye_rectify_raw10(maps, 0, raw, stride, image);
    assert(image[centre] == 48);
    for (unsigned i = 0; i < DEPTH_W * DEPTH_H; ++i)
        assert(image[i] == (maps->xy[0][i][0] == FISHEYE_INVALID ? 0 : 48));
    uint16_t mm[DEPTH_W * DEPTH_H];
    for (unsigned i = 0; i < DEPTH_W * DEPTH_H; ++i)
        mm[i] = 1000;
    maps->xy[1][centre][0] = FISHEYE_INVALID;
    fisheye_mask_depth(maps, image, mm);
    assert(image[centre] == 0 && mm[centre] == 0);
    uint8_t valid[2][DEPTH_W * DEPTH_H];
    fisheye_build_validity(NULL, NULL, valid);
    assert(valid[0][centre] && valid[1][centre]);
    assert(!valid[0][0] && !valid[0][50 * DEPTH_W] && !valid[1][DEPTH_W * DEPTH_H - 1]);
    fisheye_build_validity(&source, maps, valid);
    assert(valid[0][centre] && !valid[1][centre]);
    /* A rectified coordinate can lie within the sensor but outside the
     * useful lens circle; geometric validity must reject that too. */
    maps->xy[0][centre][0] = 0;
    fisheye_build_validity(&source, maps, valid);
    assert(!valid[0][centre]);
    free(raw);
    free(maps);
    puts("PASS: fisheye profiles, RAW10 remapping, lens-rim exclusion and invalid-pixel masking");
    return 0;
}
