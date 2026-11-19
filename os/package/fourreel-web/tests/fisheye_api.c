#define main dashboard_main
#include "../server.c"
#undef main
#include <assert.h>

FILE *__real_fopen(const char *, const char *);
int __real_rename(const char *, const char *);
int __real_unlink(const char *);
static const char *redirect(const char *path)
{
    if (!strcmp(path, FISHEYE_FILE)) return "fisheye";
    if (!strcmp(path, FISHEYE_FILE ".tmp")) return "fisheye.tmp";
    return path;
}
FILE *__wrap_fopen(const char *path, const char *mode) { return __real_fopen(redirect(path), mode); }
int __wrap_rename(const char *old, const char *next) { return __real_rename(redirect(old), redirect(next)); }
int __wrap_unlink(const char *path) { return __real_unlink(redirect(path)); }

static uint64_t get64(const uint8_t *p)
{
    uint64_t value = 0;
    for (int i = 7; i >= 0; --i)
        value = (value << 8) | p[i];
    return value;
}

int main(void)
{
    char directory[] = "/tmp/4reel-fisheye-api-XXXXXX";
    assert(mkdtemp(directory) && !chdir(directory));
    server.cameras[0].fd = server.cameras[1].fd = -1;
    struct HttpRequest request = {.path = "/api/fisheye", .control_header = 1};
    struct HttpResponse response = {0};
    strcpy(request.body, "invalid calibration");
    handle_post(&request, &response, &server);
    assert(response.status == 400 && !server.fisheye.valid);
    assert(!prepare_neural_maps(&server) && !server.fisheye.valid && !server.calibration.valid);
    for (unsigned i = 0; i < NEURAL_PIXELS; ++i)
        assert((*server.neural_maps)[i][0] != FISHEYE_INVALID);
    free(server.neural_maps);
    server.neural_maps = NULL;
    struct FisheyeCalibration profile = {.fx = 80, .fy = 80, .cx = 80,
        .cy = 50, .baseline_mm = 40, .rms = 0.2, .valid = 1};
    for (int side = 0; side < 2; ++side)
        profile.camera[side] = (struct FisheyeCamera){.fx = 300, .fy = 300,
            .cx = 640, .cy = 400, .rotation = {1,0,0,0,1,0,0,0,1}};
    assert(fisheye_format(&profile, request.body, sizeof(request.body)) > 0);
    handle_post(&request, &response, &server);
    assert(response.status == 200 && server.fisheye.valid && !server.capture);
    assert(server.fisheye_maps && server.calibration.scale_px_mm == 3200);
    status_response(&server, &response);
    assert(strstr(json, "\"fisheye_calibrated\":true"));
    assert(strstr(json, "\"baseline_mm\":40.0000"));
    strcpy(request.path, "/api/calibrate");
    handle_post(&request, &response, &server);
    assert(response.status == 409);
    strcpy(request.path, "/api/capture");
    strcpy(request.body, "enabled=1&mode=depth&pipeline=invalid");
    handle_post(&request, &response, &server);
    assert(response.status == 400 && !server.capture);

    server.calibration_mode = server.capture = 1;
    server.calibration_pending = malloc(2 * FOCUS_BMP_BYTES);
    server.calibration_pair = malloc(CAL_PAIR_HEADER_BYTES + 2 * FOCUS_BMP_BYTES);
    assert(server.calibration_pending && server.calibration_pair);
    for (int i = 0; i < 2; ++i) {
        init_bmp(server.calibration_pending + i * FOCUS_BMP_BYTES, RAW_W, RAW_H, 0);
        server.calibration_pending[i * FOCUS_BMP_BYTES + 1078] = i ? 77 : 23;
        server.cameras[i].fresh = 1;
        server.cameras[i].timestamp_us = 1000000 + i * 10000;
    }
    pair_if_ready(&server);
    assert(server.frame == 1);
    strcpy(request.path, "/api/calibration/pair.bin");
    handle_get(&request, &response, &server);
    assert(response.status == 200 && response.body_len == CAL_PAIR_HEADER_BYTES + 2 * FOCUS_BMP_BYTES);
    const uint8_t *packet = response.body;
    assert(!memcmp(packet, "4REELC1", 8) && get64(packet + 16) == 1);
    assert(packet[CAL_PAIR_HEADER_BYTES + 1078] == 77);
    assert(packet[CAL_PAIR_HEADER_BYTES + FOCUS_BMP_BYTES + 1078] == 23);
    strcpy(request.path, "/api/frame.bin");
    handle_get(&request, &response, &server);
    assert(response.status == 409);
    capture_stop(&server);

    server.capture = 1;
    server.calibration_mode = 0;
    server.frame = 7;
    server.capture_timestamp_us = 123456789;
    server.neural.available = 1;
    server.neural.frame = 6;
    server.neural.inverse_depth[0] = 1.5f;
    server.depth_mm[0] = 1234;
    size_t size;
    packet = make_frame_packet(&server, &size);
    assert(size == 80 + PIXELS * 12 && !memcmp(packet, "4REELD1", 8));
    assert(packet[20] == 23 && get64(packet + 24) == 7);
    assert(get64(packet + 32) == 123456789 && get64(packet + 40) == 6);
    assert(packet[80 + PIXELS * 2] == 0xd2 && packet[81 + PIXELS * 2] == 4);
    const uint8_t *inverse = packet + 80 + PIXELS * 4;
    assert(inverse[0] == 0 && inverse[1] == 0 && inverse[2] == 0xc0 && inverse[3] == 0x3f);
    for (unsigned i = 0; i < PIXELS * 4; ++i)
        assert(inverse[PIXELS * 4 + i] == 0);

    server.depth_mm[1] = 2000;
    server.depth_mm[2] = 1000;
    server.depth_mm[3] = 5000;
    server.depth_mm[4] = 75;
    strcpy(request.path, "/api/depth.bmp");
    handle_get(&request, &response, &server);
    assert(response.status == 200 && response.body_len == BMP_BYTES);
    const uint8_t *preview = response.body;
    assert(!memcmp(preview + 54, (uint8_t[]){0, 0, 0}, 3));
    assert(!memcmp(preview + 54 + 4, (uint8_t[]){52, 45, 225}, 3));
    assert(!memcmp(preview + 54 + 255 * 4, (uint8_t[]){232, 83, 54}, 3));
    const uint8_t *first_row = preview + 1078 + (DEPTH_H - 1) * DEPTH_W;
    assert(first_row[1] == 56 && first_row[2] == 98);
    assert(first_row[3] == 1 && first_row[4] == 255 && first_row[5] == 0);
    assert(strstr(metric_legend_json(), "[5000,\"#e12d34\"]"));
    assert(strstr(metric_legend_json(), "[75,\"#3653e8\"]"));
    preview = make_fused_bmp(&server);
    assert(!memcmp(preview + 54 + 6 * 4, (uint8_t[]){6, 12, 249}, 3));
    assert(!memcmp(preview + 54 + 254 * 4, (uint8_t[]){180, 190, 49}, 3));
    assert(!memcmp(preview + 54 + 255 * 4, (uint8_t[]){75, 181, 240}, 3));

    set_zipdepth_scale(&response);
    assert(!strcmp(response.zipdepth_scale,
                   "[[1,5000],[56,2000],[98,1000],[140,500],[182,250],[213,150],[255,75]]"));
    respond(&response, 200, "image/bmp", preview, BMP_BYTES);
    assert(!response.zipdepth_scale[0]);

    const struct ProfileCycle cycle = {
        .frame = 7, .cameras = {{90000000, 100000000}, {110000000, 120000000}},
        .stereo = {120000000, 150000000}, .neural = {160000000, 190000000},
    };
    char timeline[TIMELINE_LANES][PROFILE_TIMELINE_SAMPLES + 1];
    profile_trace(TIMELINE_CAMERA_IO, 90000000, 100000000);
    profile_trace(TIMELINE_RVE_SAD, 121000000, 121050000);
    profile_trace(TIMELINE_STEREO, 120000000, 150000000);
    profile_trace(TIMELINE_CAMERA_IO, 155000000, 158000000);
    profile_trace(TIMELINE_NPU, 165000000, 175000000);
    assert(profile_timeline(&cycle, timeline) == 100000000);
    assert(!memcmp(timeline[TIMELINE_CAMERA_IO] + 48, "990", 3));
    assert(timeline[TIMELINE_CAMERA_IO][330] == '0');
    assert(timeline[TIMELINE_RVE_SAD][155] == '3');
    assert(!memcmp(timeline[TIMELINE_NPU] + 374, "099", 3));

    assert(!unlink(FISHEYE_FILE));
    free(server.fisheye_maps);
    assert(!chdir("/") && !rmdir(directory));
    puts("PASS: fisheye upload/persistence, calibration mode, atomic native pairs and versioned depth packets");
    return 0;
}
