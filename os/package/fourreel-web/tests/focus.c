/* Host-side test: gcc -std=c11 -Wall -Wextra -Werror tests/focus.c http.c
 * depth.c -lm -lpthread -o /tmp/4reel-focus-test && /tmp/4reel-focus-test */
#define main dashboard_main
#include "../server.c"
#undef main
#include <assert.h>

static uint32_t get32(const uint8_t *p)
{
    return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

int main(void)
{
    const unsigned stride = 1792;
    uint8_t *raw = malloc(stride * RAW_H);
    assert(raw);
    memset(raw, 0xcc, stride * RAW_H);
    for (unsigned y = 0; y < RAW_H; ++y)
        for (unsigned x = 0; x < RAW_W; x += 4) {
            uint64_t packed = 0;
            for (unsigned i = 0; i < 4; ++i)
                packed |= (uint64_t)((x + i + 7 * y) & 1023) << (10 * i);
            for (unsigned byte = 0; byte < 5; ++byte)
                raw[y * stride + x / 4 * 5 + byte] = packed >> (8 * byte);
        }
    server.cameras[0].fd = server.cameras[1].fd = -1;
    server.focus_mode = server.capture = 1;
    server.frame = 1;
    server.calibration = (struct Calibration){.valid = 1, .scale_px_mm = 5000};
    server.focus_bmp = malloc(FOCUS_BMP_BYTES);
    assert(server.focus_bmp);
    init_bmp(server.focus_bmp, RAW_W, RAW_H, 0);
    focus_unpack(&server, raw, stride);
    assert(get32(server.focus_bmp + 2) == FOCUS_BMP_BYTES);
    assert(get32(server.focus_bmp + 18) == RAW_W);
    assert(get32(server.focus_bmp + 22) == RAW_H);
    for (unsigned y = 0; y < RAW_H; ++y)
        for (unsigned x = 0; x < RAW_W; ++x)
            assert(server.focus_bmp[1078 + (RAW_H - 1 - y) * RAW_W + x] ==
                   ((RAW_W - 1 - x + 7 * y) & 1023) >> 2);
    for (unsigned y = 0; y < DEPTH_H; ++y)
        for (unsigned x = 0; x < DEPTH_W; ++x)
            assert(server.latest[0][y * DEPTH_W + x] ==
                   ((RAW_W - 1 - (x * 8 + 4) + 7 * (y * 8 + 4)) & 1023) >> 2);
    assert(!memcmp(server.latest[0], server.latest[1], PIXELS));

    /* Both depth inputs must have the same horizontal correction as focus. */
    uint8_t reduced[PIXELS];
    downsample_raw10(reduced, raw, stride);
    for (unsigned y = 0; y < DEPTH_H; ++y)
        for (unsigned x = 0; x < DEPTH_W; ++x) {
            unsigned source_x = DEPTH_W - 1 - x;
            unsigned sum = 0;
            for (unsigned dy = 0; dy < 8; ++dy)
                for (unsigned dx = 0; dx < 8; ++dx) {
                    unsigned rx = (source_x & ~1U) * 8 + (source_x & 1) + dx * 2;
                    unsigned ry = (y & ~1U) * 8 + (y & 1) + dy * 2;
                    sum += (rx + 7 * ry) & 1023;
                }
            assert(reduced[y * DEPTH_W + x] == sum >> 8);
        }
    const unsigned scaled_stride = DEPTH_W * 2 + 16;
    uint8_t scaled[DEPTH_H * scaled_stride];
    memset(scaled, 0xcc, sizeof(scaled));
    for (unsigned y = 0; y < DEPTH_H; ++y)
        for (unsigned x = 0; x < DEPTH_W; ++x) {
            scaled[y * scaled_stride + x * 2] = 123;
            scaled[y * scaled_stride + x * 2 + 1] = (x + 3 * y) & 255;
        }
    unpack_raw10(reduced, scaled, scaled_stride);
    for (unsigned y = 0; y < DEPTH_H; ++y)
        for (unsigned x = 0; x < DEPTH_W; ++x)
            assert(reduced[y * DEPTH_W + x] == ((DEPTH_W - 1 - x + 3 * y) & 255));

    struct HttpRequest request = {.path = "/api/focus.bmp", .control_header = 1};
    struct HttpResponse response = {0};
    handle_get(&request, &response, &server);
    assert(response.status == 200 && response.body_len == FOCUS_BMP_BYTES);
    strcpy(request.path, "/api/depth.pgm");
    handle_get(&request, &response, &server);
    assert(response.status == 409);
    strcpy(request.path, "/api/snapshot");
    strcpy(request.body, "slot=near");
    handle_post(&request, &response, &server);
    assert(response.status == 409);
    strcpy(request.path, "/api/capture");
    strcpy(request.body, "enabled=1&mode=invalid");
    handle_post(&request, &response, &server);
    assert(response.status == 400 && server.capture);
    strcpy(request.body, "enabled=1&mode=focus&camera=invalid");
    handle_post(&request, &response, &server);
    assert(response.status == 400 && server.capture);
    status_response(&server, &response);
    assert(response.body_len < sizeof(json));
    assert(strstr(json, "\"mode\":\"focus\""));
    assert(strstr(json, "\"processing_width\":1280,\"processing_height\":800"));

    capture_stop(&server);
    assert(!server.focus_bmp && !server.capture && !server.frame);
    assert(server.calibration.valid && server.calibration.scale_px_mm == 5000);
    strcpy(request.path, "/api/focus.bmp");
    handle_get(&request, &response, &server);
    assert(response.status == 409);
    server.focus_mode = 0;
    status_response(&server, &response);
    assert(strstr(json, "\"processing_width\":160,\"processing_height\":100"));
    assert(depth_self_test() == 0);
    free(raw);
    puts("PASS: full-resolution RAW10/BMP, AE sampling, mode guards, cleanup and depth self-test");
    return 0;
}
