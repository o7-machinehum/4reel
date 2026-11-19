#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/videodev2.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "depth.h"
#include "rve_sad.h"
#include "http.h"
#include "fisheye.h"
#include "neural.h"
#include "image.h"

#define RAW_W 1280
#define RAW_H 800
#define PIXELS (DEPTH_W * DEPTH_H)
#define MAX_BUFFERS 4
#define BMP_BYTES (14 + 40 + 1024 + PIXELS)
#define FOCUS_BMP_BYTES (14 + 40 + 1024 + RAW_W * RAW_H)
#define FOCUS_INTERVAL_MS 250
#define CAL_PAIR_HEADER_BYTES 40
#define PROCESS_INTERVAL_MS 33
/* Fixed physical labels: the former "Swap cameras" enabled mapping. */
#define LEFT_CAMERA 1
#define RIGHT_CAMERA 0
#define CALIBRATION_FILE "/etc/4reel-web.calibration"
#define INDEX_FILE "/usr/share/4reel-web/index.html"
#define FISHEYE_FILE "/etc/4reel-web.fisheye"
#define NEURAL_INTERVAL_MS 100
#define AE_NEURAL_INTERVAL_MS 500
#define AE_TARGET 128
#define AE_LOW 110
#define AE_HIGH 145
#define AE_CLIP_PERCENT 0.5
#define AE_MOTION_EXPOSURE 900
#define AE_INTERVAL_FRAMES 5

struct CameraBuffer {
    void *data;
    size_t length;
};

struct Camera {
    int fd;
    int streaming;
    unsigned buffer_count;
    unsigned stride;
    unsigned height;
    int scaled;
    struct CameraBuffer buffers[MAX_BUFFERS];
    uint8_t image[PIXELS];
    uint8_t neural_image[NEURAL_PIXELS];
    int neural_ready;
    int fresh;
    int64_t timestamp_us;
    double unpack_ms;
    int64_t last_frame_ms;
    struct ProfileRange profile_range;
};

struct TimingMetric {
    double ms;
    uint64_t samples;
};

struct Diagnostics {
    uint64_t raw_frames[2];
    uint64_t decoded_frames[2];
    uint64_t skipped_frames[2];
    uint64_t sync_dropped[2];
    int64_t last_raw_timestamp_us[2];
    int64_t last_output_us;
    double raw_fps[2];
    double output_fps;
    uint64_t raw_fps_samples[2];
    uint64_t output_fps_samples;
    struct TimingMetric unpack[2];
    struct TimingMetric unpack_work;
    struct TimingMetric depth;
    struct TimingMetric auto_exposure;
    struct TimingMetric process;
    struct TimingMetric http_image;
    struct TimingMetric http_status;
};

enum HttpRequestKind {
    HTTP_REQUEST_OTHER,
    HTTP_REQUEST_STATUS,
    HTTP_REQUEST_IMAGE,
};

enum ProfileStage {
    PROFILE_DEQUEUE_LEFT, PROFILE_DEQUEUE_RIGHT,
    PROFILE_REQUEUE_LEFT, PROFILE_REQUEUE_RIGHT,
    PROFILE_UNPACK_LEFT, PROFILE_UNPACK_RIGHT,
    PROFILE_REMAP, PROFILE_MATCH, PROFILE_AE, PROFILE_SENSOR, PROFILE_QUEUE,
    PROFILE_HTTP_IMAGE, PROFILE_HTTP_STATUS, PROFILE_HTTP_OTHER,
    PROFILE_STAGE_COUNT
};

struct Server {
    struct Camera cameras[2];
    uint8_t latest[2][PIXELS];
    uint8_t snapshots[2][2][PIXELS];
    int snapshot_valid[2];
    uint8_t depth_visual[PIXELS];
    uint16_t depth_mm[PIXELS];
    uint8_t match_valid[2][PIXELS];
    struct Calibration calibration;
    uint64_t frame;
    int64_t last_pair_ms;
    double pair_delta_ms;
    int capture;
    int focus_mode;
    int focus_side;
    uint8_t *focus_bmp;
    int calibration_mode;
    uint8_t *calibration_pending;
    uint8_t *calibration_pair;
    struct FisheyeCalibration fisheye;
    struct FisheyeMaps *fisheye_maps;
    uint16_t (*neural_maps)[NEURAL_PIXELS][2];
    struct NeuralResult neural;
    int depth_engine; /* 0: stereo, 1: ZipDepth, 2: both. */
    int64_t capture_timestamp_us;
    int64_t neural_last_ms;
    int exposure;
    int gain;
    int auto_exposure;
    int ae_level[2];
    double ae_clipped_percent;
    uint64_t ae_last_frame;
    uint64_t ae_neural_frame;
    int64_t ae_neural_last_ms;
    char error[128];
    char accel_path[128];
    struct Diagnostics diagnostics;
    struct ProfileMetric profile[PROFILE_STAGE_COUNT];
    struct ProfileCycle profile_cycle;
    enum HttpRequestKind http_request_kind;
};

static struct Server server;
static uint8_t bmp[BMP_BYTES];
static uint8_t depth_pgm[32 + PIXELS * 2];
static char json[4096];
enum BmpPalette { BMP_LUMA, BMP_DEPTH_COLOURS, BMP_STEREO_DEPTH };
static void init_bmp(uint8_t *out, unsigned width, unsigned height, enum BmpPalette palette);
static volatile sig_atomic_t stopping;
static struct {
    pthread_mutex_t lock;
    pthread_t thread;
    double values[3];
    int valid, stop, started;
} accelerometer = {.lock = PTHREAD_MUTEX_INITIALIZER};

static void on_signal(int signal_number)
{
    (void)signal_number;
    stopping = 1;
}

static int xioctl(int fd, unsigned long request, void *arg)
{
    int result;
    do {
        result = ioctl(fd, request, arg);
    } while (result < 0 && errno == EINTR);
    return result;
}

static int64_t monotonic_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static int64_t monotonic_us(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)now.tv_sec * 1000000 + now.tv_nsec / 1000;
}

static void timing_sample(struct TimingMetric *metric, double milliseconds)
{
    if (!metric->samples)
        metric->ms = milliseconds;
    else
        metric->ms += (milliseconds - metric->ms) / 8.0;
    metric->samples++;
}

static void fps_sample(double *fps, uint64_t *samples, int64_t *last_us,
                       int64_t timestamp_us)
{
    if (*last_us && timestamp_us > *last_us) {
        double current = 1000000.0 / (timestamp_us - *last_us);
        if (!*samples)
            *fps = current;
        else
            *fps += (current - *fps) / 8.0;
        (*samples)++;
    }
    *last_us = timestamp_us;
}

static void diagnostics_reset(struct Server *state)
{
    memset(&state->diagnostics, 0, sizeof(state->diagnostics));
}

static void camera_close(struct Camera *camera)
{
    if (camera->fd >= 0 && camera->streaming) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        xioctl(camera->fd, VIDIOC_STREAMOFF, &type);
    }
    for (unsigned i = 0; i < MAX_BUFFERS; ++i) {
        if (camera->buffers[i].data) {
            munmap(camera->buffers[i].data, camera->buffers[i].length);
            camera->buffers[i].data = NULL;
            camera->buffers[i].length = 0;
        }
    }
    if (camera->fd >= 0)
        close(camera->fd);
    camera->fd = -1;
    camera->streaming = 0;
    camera->buffer_count = 0;
    camera->fresh = 0;
    camera->neural_ready = 0;
    camera->unpack_ms = 0;
}

static int camera_queue(struct Camera *camera, unsigned index)
{
    struct v4l2_buffer buffer = {0};
    struct v4l2_plane plane = {0};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    buffer.m.planes = &plane;
    buffer.length = 1;
    return xioctl(camera->fd, VIDIOC_QBUF, &buffer);
}

static int camera_open(struct Camera *camera, const char *path, int scaled)
{
    unsigned width = scaled ? DEPTH_W : RAW_W;
    unsigned height = scaled ? DEPTH_H : RAW_H;
    camera->scaled = scaled;
    camera->height = height;
    camera->fd = open(path, O_RDWR | O_NONBLOCK);
    if (camera->fd < 0)
        return -1;

    struct v4l2_capability capability = {0};
    if (xioctl(camera->fd, VIDIOC_QUERYCAP, &capability) < 0)
        goto fail;
    unsigned caps = capability.capabilities & V4L2_CAP_DEVICE_CAPS ?
        capability.device_caps : capability.capabilities;
    if (!(caps & V4L2_CAP_STREAMING) || !(caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {
        errno = ENOTSUP;
        goto fail;
    }

    struct v4l2_format format = {0};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    format.fmt.pix_mp.width = width;
    format.fmt.pix_mp.height = height;
    format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_Y10;
    format.fmt.pix_mp.field = V4L2_FIELD_NONE;
    format.fmt.pix_mp.num_planes = 1;
    if (xioctl(camera->fd, VIDIOC_S_FMT, &format) < 0)
        goto fail;
    if (format.fmt.pix_mp.width != width || format.fmt.pix_mp.height != height ||
        format.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_Y10 ||
        format.fmt.pix_mp.num_planes != 1 ||
        format.fmt.pix_mp.plane_fmt[0].bytesperline <
            (scaled ? width * 2 : width * 5 / 4) ||
        format.fmt.pix_mp.plane_fmt[0].sizeimage <
            format.fmt.pix_mp.plane_fmt[0].bytesperline * height) {
        errno = EINVAL;
        goto fail;
    }
    camera->stride = format.fmt.pix_mp.plane_fmt[0].bytesperline;

    struct v4l2_requestbuffers request = {0};
    request.count = 3;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(camera->fd, VIDIOC_REQBUFS, &request) < 0)
        goto fail;
    if (request.count < 3 || request.count > MAX_BUFFERS) {
        errno = ENOMEM;
        goto fail;
    }
    camera->buffer_count = request.count;

    for (unsigned i = 0; i < camera->buffer_count; ++i) {
        struct v4l2_buffer buffer = {0};
        struct v4l2_plane plane = {0};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        buffer.m.planes = &plane;
        buffer.length = 1;
        if (xioctl(camera->fd, VIDIOC_QUERYBUF, &buffer) < 0)
            goto fail;
        void *mapping = mmap(NULL, plane.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, camera->fd, plane.m.mem_offset);
        if (mapping == MAP_FAILED)
            goto fail;
        camera->buffers[i].data = mapping;
        camera->buffers[i].length = plane.length;
        if (camera_queue(camera, i) < 0)
            goto fail;
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (xioctl(camera->fd, VIDIOC_STREAMON, &type) < 0)
        goto fail;
    camera->streaming = 1;
    return 0;

fail:
    {
    int saved_errno = errno;
    camera_close(camera);
    errno = saved_errno;
    return -1;
    }
}

static void unpack_raw10(uint8_t *destination, const uint8_t *source,
                         unsigned stride)
{
    /* Correct both sensors' horizontal mirroring while converting, before
     * preview, calibration and stereo matching. Keep vertical orientation. */
    for (unsigned y = 0; y < DEPTH_H; ++y) {
        const uint8_t *row = source + y * stride;
        for (unsigned x = 0; x < DEPTH_W; ++x) {
            unsigned sample = row[x * 2] | ((unsigned)row[x * 2 + 1] << 8);
            /* CIF returns sums of 64 RAW10 samples, not averaged RAW10.
             * Divide by 64, then discard the sensor's bottom two bits. */
            destination[y * DEPTH_W + DEPTH_W - 1 - x] = (uint8_t)(sample >> 8);
        }
    }
}

/* RV1103/RV1106 has only one CIF scaler (TRM SCL_CH_CTRL bits 31:8 are
 * reserved). Camera 0 uses the scaler; camera 1 uses normal DMA. */
static void downsample_raw10(uint8_t *destination, const uint8_t *source,
                             unsigned stride)
{
    _Static_assert(RAW_W == DEPTH_W * 8 && RAW_H == DEPTH_H * 8,
                   "8x raw downsampling");
    for (int y = 0; y < DEPTH_H; ++y)
        for (int x = 0; x < DEPTH_W; x += 2) {
            unsigned even = 0, odd = 0;
            /* Match the scaler's Bayer-preserving 2x2 output: each sum
             * samples one parity of a 16x16 input tile, even for mono. */
            for (int dy = 0; dy < 8; ++dy) {
                int row = (y & ~1) * 8 + (y & 1) + dy * 2;
                const uint8_t *p = source + row * stride + x * 10;
                for (int group = 0; group < 4; ++group, p += 5) {
                    even += p[0] | ((p[1] & 3U) << 8);
                    odd += (p[1] >> 2) | ((p[2] & 15U) << 6);
                    even += (p[2] >> 4) | ((p[3] & 63U) << 4);
                    odd += (p[3] >> 6) | ((unsigned)p[4] << 2);
                }
            }
            destination[y * DEPTH_W + DEPTH_W - 1 - x] = even >> 8;
            destination[y * DEPTH_W + DEPTH_W - 2 - x] = odd >> 8;
        }
}

static int find_camera_nodes(char paths[2][32], int full_resolution)
{
    const char *names[2] = {
        full_resolution ? "stream_cif_mipi_id0" : "rkcif_scale_ch0",
        "stream_cif_mipi_id0",
    };
    const char *buses[2] = {
        "platform:rkcif-mipi-lvds",
        "platform:rkcif-mipi-lvds1",
    };

    paths[0][0] = paths[1][0] = 0;
    for (int number = 0; number < 128; ++number) {
        char sysfs[64];
        char name[64] = {0};
        char device[32];
        struct v4l2_capability capability = {0};

        snprintf(sysfs, sizeof(sysfs),
                 "/sys/class/video4linux/video%d/name", number);
        FILE *input = fopen(sysfs, "r");
        if (!input)
            continue;
        int read_name = fgets(name, sizeof(name), input) != NULL;
        fclose(input);
        if (!read_name)
            continue;
        name[strcspn(name, "\r\n")] = 0;

        if (strcmp(name, names[0]) && strcmp(name, names[1]))
            continue;
        snprintf(device, sizeof(device), "/dev/video%d", number);
        int fd = open(device, O_RDWR | O_NONBLOCK);
        if (fd < 0)
            continue;
        int queried = xioctl(fd, VIDIOC_QUERYCAP, &capability);
        int saved_errno = errno;
        close(fd);
        errno = saved_errno;
        for (int target = 0; target < 2; ++target)
            if (queried == 0 && !strcmp(name, names[target]) &&
                !strcmp((const char *)capability.bus_info, buses[target]))
                snprintf(paths[target], 32, "%s", device);
    }
    if (!paths[0][0] || !paths[1][0]) {
        errno = ENODEV;
        return -1;
    }
    return 0;
}

static int set_sensor_control(struct Server *state, const char *path, unsigned id, int value)
{
    int fd = open(path, O_RDWR);
    if (fd < 0)
        return -1;
    struct v4l2_control control = {.id = id, .value = value};
    struct ProfileStamp start = profile_start();
    int result = xioctl(fd, VIDIOC_S_CTRL, &control);
    int saved_errno = errno;
    profile_end_traced(&state->profile[PROFILE_SENSOR], start, TIMELINE_CAMERA_IO);
    close(fd);
    errno = saved_errno;
    return result;
}

static int apply_sensor_settings(struct Server *state, int exposure, int gain)
{
    const char *paths[2] = {"/dev/v4l-subdev2", "/dev/v4l-subdev5"};
    for (int i = 0; i < 2; ++i) {
        if (set_sensor_control(state, paths[i], V4L2_CID_EXPOSURE, exposure) < 0 ||
            set_sensor_control(state, paths[i], V4L2_CID_ANALOGUE_GAIN, gain) < 0)
            return -1;
    }
    return 0;
}

static void capture_stop(struct Server *state)
{
    neural_stop();
    memset(&state->neural, 0, sizeof(state->neural));
    state->neural_last_ms = 0;
    state->capture_timestamp_us = 0;
    memset(state->depth_visual, 0, sizeof(state->depth_visual));
    memset(state->depth_mm, 0, sizeof(state->depth_mm));
    camera_close(&state->cameras[0]);
    camera_close(&state->cameras[1]);
    free(state->focus_bmp);
    state->focus_bmp = NULL;
    free(state->calibration_pending);
    free(state->calibration_pair);
    state->calibration_pending = state->calibration_pair = NULL;
    free(state->neural_maps);
    state->neural_maps = NULL;
    state->capture = 0;
    state->frame = 0;
    state->profile_cycle = (struct ProfileCycle){0};
    state->last_pair_ms = 0;
    state->ae_last_frame = 0;
    state->ae_neural_frame = 0;
    state->ae_neural_last_ms = 0;
    state->diagnostics.raw_fps[0] = state->diagnostics.raw_fps[1] = 0;
    state->diagnostics.output_fps = 0;
    state->diagnostics.last_raw_timestamp_us[0] = 0;
    state->diagnostics.last_raw_timestamp_us[1] = 0;
    state->diagnostics.last_output_us = 0;
}

static void update_neural_snapshot(struct Server *state)
{
    uint64_t previous_frame = state->neural.frame;
    neural_snapshot(&state->neural);
    if (state->fisheye.valid && state->neural.available &&
        state->neural.frame != previous_frame)
        for (int i = 0; i < PIXELS; ++i)
            if (state->fisheye_maps->xy[0][i][0] == FISHEYE_INVALID) {
                state->neural.visual[i] = 0;
                state->neural.inverse_depth[i] = 0;
            }
}

static int prepare_neural_maps(struct Server *state)
{
    struct FisheyeCalibration projection = state->fisheye;
    if (!projection.valid) {
        /* Approximate Arducam M32076M20: 0.76 mm EFL, OV9281 3 um pixels.
         * This is a neural-only lens estimate, never a metric calibration.
         * Use a 110-degree forward perspective view of the 220-degree lens. */
        projection.fx = projection.fy = DEPTH_W / (2 * tan(55 * 3.141592653589793 / 180));
        projection.cx = (DEPTH_W - 1) / 2.0;
        projection.cy = (DEPTH_H - 1) / 2.0;
        for (int side = 0; side < 2; ++side)
            projection.camera[side] = (struct FisheyeCamera){
                .fx = 0.76 / 0.003, .fy = 0.76 / 0.003,
                .cx = RAW_W / 2.0, .cy = RAW_H / 2.0,
                .rotation = {1,0,0,0,1,0,0,0,1}};
    }
    state->neural_maps = malloc(sizeof(*state->neural_maps));
    if (!state->neural_maps)
        return -1;
    fisheye_build_map(&projection, 0, NEURAL_W, NEURAL_H, *state->neural_maps);
    return 0;
}

static int capture_start(struct Server *state)
{
    char paths[2][32];

    if (state->capture)
        return 0;
    if (find_camera_nodes(paths, state->focus_mode || state->calibration_mode ||
                           state->fisheye.valid || state->depth_engine) < 0 ||
        apply_sensor_settings(state, state->exposure, state->gain) < 0)
        goto fail;
    if (state->focus_mode) {
        int index = state->focus_side ? RIGHT_CAMERA : LEFT_CAMERA;
        rve_sad_close();
        state->focus_bmp = malloc(FOCUS_BMP_BYTES);
        if (!state->focus_bmp)
            goto fail;
        init_bmp(state->focus_bmp, RAW_W, RAW_H, BMP_LUMA);
        if (camera_open(&state->cameras[index], paths[index], 0) < 0)
            goto fail;
    } else if (camera_open(&state->cameras[0], paths[0],
                           !state->calibration_mode && !state->fisheye.valid && !state->depth_engine) < 0 ||
               camera_open(&state->cameras[1], paths[1], 0) < 0) {
        goto fail;
    }
    if (state->calibration_mode) {
        rve_sad_close();
        state->calibration_pending = malloc(2 * FOCUS_BMP_BYTES);
        state->calibration_pair = malloc(CAL_PAIR_HEADER_BYTES + 2 * FOCUS_BMP_BYTES);
        if (!state->calibration_pending || !state->calibration_pair)
            goto fail;
        for (int side = 0; side < 2; ++side)
            init_bmp(state->calibration_pending + side * FOCUS_BMP_BYTES, RAW_W, RAW_H, BMP_LUMA);
    } else if (!state->focus_mode && state->depth_engine) {
        if (prepare_neural_maps(state))
            goto fail;
        int result = neural_start(NEURAL_MODEL);
        update_neural_snapshot(state);
        if (result && state->depth_engine == 1) {
            snprintf(state->error, sizeof(state->error), "%s", state->neural.error);
            capture_stop(state);
            return -1;
        }
    }
    if (!state->focus_mode && !state->calibration_mode)
        fisheye_build_validity(&state->fisheye, state->fisheye_maps, state->match_valid);
    state->cameras[0].fresh = state->cameras[1].fresh = 0;
    state->cameras[0].last_frame_ms = state->cameras[1].last_frame_ms = monotonic_ms();
    diagnostics_reset(state);
    state->capture = 1;
    state->error[0] = 0;
    return 0;

fail:
    {
        snprintf(state->error, sizeof(state->error), "camera start failed: %s",
                 strerror(errno));
        capture_stop(state);
        return -1;
    }
}

static int image_percentile(const uint8_t *image, unsigned percentile,
                            double *clipped_percent)
{
    uint32_t histogram[256] = {0};
    unsigned samples = 0;
    unsigned clipped = 0;

    for (unsigned y = DEPTH_H / 10; y < DEPTH_H * 9 / 10; y += 2) {
        for (unsigned x = DEPTH_W / 10; x < DEPTH_W * 9 / 10; x += 2) {
            uint8_t value = image[y * DEPTH_W + x];
            histogram[value]++;
            samples++;
            if (value >= 250)
                clipped++;
        }
    }

    unsigned threshold = (samples * percentile + 99) / 100;
    unsigned accumulated = 0;
    for (unsigned value = 0; value < 256; ++value) {
        accumulated += histogram[value];
        if (accumulated >= threshold) {
            *clipped_percent = 100.0 * clipped / samples;
            return (int)value;
        }
    }
    *clipped_percent = 0;
    return 255;
}

static void exposure_gain_for_product(double product, int *exposure, int *gain)
{
    if (product <= AE_MOTION_EXPOSURE * 16.0) {
        *gain = 16;
        *exposure = (int)(product / *gain + 0.5);
    } else if (product <= AE_MOTION_EXPOSURE * 128.0) {
        *exposure = AE_MOTION_EXPOSURE;
        *gain = (int)(product / *exposure + 0.5);
    } else if (product <= 3652.0 * 128.0) {
        *gain = 128;
        *exposure = (int)(product / *gain + 0.5);
    } else {
        *exposure = 3652;
        *gain = (int)(product / *exposure + 0.5);
    }
    if (*exposure < 4)
        *exposure = 4;
    if (*exposure > 3652)
        *exposure = 3652;
    if (*gain < 16)
        *gain = 16;
    if (*gain > 248)
        *gain = 248;
}

static void auto_exposure_update(struct Server *state, int64_t now_ms)
{
    int neural_meter = !state->focus_mode && !state->calibration_mode &&
        state->depth_engine && state->neural.available;
    double clipped[2];
    double factor;
    double minimum_factor = state->focus_mode || neural_meter ? 0.5 : 0.8;
    double maximum_factor = state->focus_mode || neural_meter ? 2.0 : 1.25;
    unsigned interval_frames = state->focus_mode ? 1 : AE_INTERVAL_FRAMES;
    int level;
    int exposure;
    int gain;

    if (!state->auto_exposure)
        return;
    if (neural_meter) {
        /* Keep the exposure feedback cadence independent of inference FPS. */
        if (state->ae_neural_frame == state->neural.frame ||
            (state->ae_neural_last_ms &&
             now_ms - state->ae_neural_last_ms < AE_NEURAL_INTERVAL_MS))
            return;
        state->ae_neural_frame = state->neural.frame;
        state->ae_neural_last_ms = now_ms;
    } else if (state->ae_last_frame &&
               state->frame - state->ae_last_frame < interval_frames) {
        return;
    }
    state->ae_last_frame = state->frame;
    for (int i = 0; i < 2; ++i)
        state->ae_level[i] = image_percentile(neural_meter && i == 0 ? state->neural.input :
                                             state->latest[i], 75, &clipped[i]);
    state->ae_clipped_percent = clipped[0] > clipped[1] ? clipped[0] : clipped[1];
    level = state->ae_level[0] < state->ae_level[1] ?
        state->ae_level[0] : state->ae_level[1];

    /* Focus and neural metering tolerate small reflections; broad clipping
     * still raises the percentile. Stereo-only keeps its highlight limit. */
    if (!state->focus_mode && !neural_meter && state->ae_clipped_percent > AE_CLIP_PERCENT) {
        factor = 0.8;
    } else if (level < AE_LOW || level > AE_HIGH) {
        factor = (double)(AE_TARGET - SENSOR_BLACK_LEVEL) /
                 (level > SENSOR_BLACK_LEVEL ? level - SENSOR_BLACK_LEVEL : 1);
        if (factor < minimum_factor)
            factor = minimum_factor;
        if (factor > maximum_factor)
            factor = maximum_factor;
    } else {
        return;
    }

    exposure_gain_for_product((double)state->exposure * state->gain * factor,
                              &exposure, &gain);
    if (exposure == state->exposure && gain == state->gain)
        return;
    if (apply_sensor_settings(state, exposure, gain) < 0) {
        int saved_errno = errno;
        apply_sensor_settings(state, state->exposure, state->gain);
        state->auto_exposure = 0;
        snprintf(state->error, sizeof(state->error),
                 "auto exposure disabled: %s", strerror(saved_errno));
        return;
    }
    state->exposure = exposure;
    state->gain = gain;
}

static void pair_if_ready(struct Server *state)
{
    struct Camera *a = &state->cameras[0];
    struct Camera *b = &state->cameras[1];
    if (!a->fresh || !b->fresh)
        return;
    int64_t delta = a->timestamp_us - b->timestamp_us;
    if (llabs(delta) > 20000) {
        if (delta < 0) {
            a->fresh = 0;
            a->unpack_ms = 0;
            state->diagnostics.sync_dropped[0]++;
        } else {
            b->fresh = 0;
            b->unpack_ms = 0;
            state->diagnostics.sync_dropped[1]++;
        }
        return;
    }
    double unpack_ms = a->unpack_ms + b->unpack_ms;
    a->fresh = b->fresh = 0;
    a->unpack_ms = b->unpack_ms = 0;
    int64_t now = monotonic_ms();
    int interval = state->calibration_mode ? FOCUS_INTERVAL_MS : PROCESS_INTERVAL_MS;
    if (now - state->last_pair_ms < interval)
        return;
    int64_t process_start_us = monotonic_us();
    const struct Camera *left = &state->cameras[LEFT_CAMERA];
    const struct Camera *right = &state->cameras[RIGHT_CAMERA];
    struct ProfileCycle cycle = {
        .frame = state->frame + 1,
        .cameras = {left->profile_range, right->profile_range},
    };
    memcpy(state->latest[0], left->image, PIXELS);
    memcpy(state->latest[1], right->image, PIXELS);
    int64_t depth_start_us = monotonic_us();
    if (state->calibration_mode) {
        memcpy(state->calibration_pair + CAL_PAIR_HEADER_BYTES,
               state->calibration_pending + LEFT_CAMERA * FOCUS_BMP_BYTES, FOCUS_BMP_BYTES);
        memcpy(state->calibration_pair + CAL_PAIR_HEADER_BYTES + FOCUS_BMP_BYTES,
               state->calibration_pending + RIGHT_CAMERA * FOCUS_BMP_BYTES, FOCUS_BMP_BYTES);
    } else if (state->depth_engine != 1) {
        struct ProfileStamp match_start = profile_start();
        depth_compute_masked(state->latest[0], state->latest[1], &state->calibration,
                             state->match_valid[0], state->match_valid[1],
                             state->depth_visual, state->depth_mm);
        cycle.stereo.start = match_start.wall_ns;
        cycle.stereo.end = profile_end_traced(&state->profile[PROFILE_MATCH], match_start, TIMELINE_STEREO);
    }
    timing_sample(&state->diagnostics.depth,
                  (monotonic_us() - depth_start_us) / 1000.0);
    state->pair_delta_ms = fabs((double)delta) / 1000.0;
    state->last_pair_ms = now;
    state->frame++;
    state->profile_cycle = cycle;
    state->capture_timestamp_us = (a->timestamp_us + b->timestamp_us) / 2;
    if (!state->calibration_mode && state->depth_engine && left->neural_ready &&
        now - state->neural_last_ms >= NEURAL_INTERVAL_MS) {
        int fuse = state->depth_engine == 2 && state->fisheye.valid;
        struct ProfileStamp queue_start = profile_start();
        if (neural_submit(left->neural_image, fuse ? state->depth_mm : NULL,
                          fuse ? state->match_valid[0] : NULL, state->frame, &cycle))
            state->neural_last_ms = now;
        profile_end_traced(&state->profile[PROFILE_QUEUE], queue_start, TIMELINE_CAMERA_CPU);
    }
    if (!state->calibration_mode && state->depth_engine)
        update_neural_snapshot(state);
    int64_t ae_start_us = monotonic_us();
    struct ProfileStamp ae_start = profile_start();
    if (state->auto_exposure) {
        auto_exposure_update(state, now);
        profile_end_traced(&state->profile[PROFILE_AE], ae_start, TIMELINE_CAMERA_CPU);
    }
    timing_sample(&state->diagnostics.auto_exposure,
                  (monotonic_us() - ae_start_us) / 1000.0);
    int64_t completed_us = monotonic_us();
    timing_sample(&state->diagnostics.unpack_work,
                  unpack_ms);
    timing_sample(&state->diagnostics.process,
                  unpack_ms + (completed_us - process_start_us) / 1000.0);
    fps_sample(&state->diagnostics.output_fps,
               &state->diagnostics.output_fps_samples,
               &state->diagnostics.last_output_us, completed_us);
}

/* Native packed RAW10 -> 8-bit BMP for focus and calibration capture.
 * Reverse horizontal order here too, matching the depth inputs. */
static void native_unpack(uint8_t *bitmap, const uint8_t *source, unsigned stride)
{
    for (unsigned y = 0; y < RAW_H; ++y) {
        const uint8_t *p = source + y * stride;
        uint8_t *row = bitmap + 1078 + (RAW_H - 1 - y) * RAW_W;
        for (unsigned x = 0; x < RAW_W; x += 4, p += 5) {
            row[RAW_W - 1 - x] = (p[0] >> 2) | ((p[1] & 3U) << 6);
            row[RAW_W - 2 - x] = (p[1] >> 4) | ((p[2] & 15U) << 4);
            row[RAW_W - 3 - x] = (p[2] >> 6) | ((p[3] & 63U) << 2);
            row[RAW_W - 4 - x] = p[4];
        }
    }
}

static void focus_unpack(struct Server *state, const uint8_t *source, unsigned stride)
{
    native_unpack(state->focus_bmp, source, stride);
    /* Reuse the small AE input arrays; in focus mode meter only the active
     * camera, rather than a stale image from the inactive sensor. */
    for (unsigned y = 0; y < DEPTH_H; ++y)
        for (unsigned x = 0; x < DEPTH_W; ++x)
            state->latest[0][y * DEPTH_W + x] =
                state->focus_bmp[1078 + (RAW_H - 1 - (y * 8 + 4)) * RAW_W + x * 8 + 4];
    memcpy(state->latest[1], state->latest[0], PIXELS);
}

static int camera_requeue(struct Server *state, int index, unsigned buffer, int side)
{
    struct ProfileStamp queue_start = profile_start();
    int result = camera_queue(&state->cameras[index], buffer);
    int saved_errno = errno;
    profile_end_traced(&state->profile[PROFILE_REQUEUE_LEFT + side], queue_start, TIMELINE_CAMERA_IO);
    errno = saved_errno;
    return result;
}

static void camera_drain(struct Server *state, int index)
{
    struct Camera *camera = &state->cameras[index];
    int side = index != LEFT_CAMERA;
    for (;;) {
        struct v4l2_buffer buffer = {0};
        struct v4l2_plane plane = {0};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.m.planes = &plane;
        buffer.length = 1;
        struct ProfileStamp dequeue_start = profile_start();
        int dequeued = xioctl(camera->fd, VIDIOC_DQBUF, &buffer);
        int dequeue_error = errno;
        profile_end_traced(&state->profile[PROFILE_DEQUEUE_LEFT + side], dequeue_start, TIMELINE_CAMERA_IO);
        if (dequeued < 0) {
            if (dequeue_error == EAGAIN)
                return;
            snprintf(state->error, sizeof(state->error), "camera dequeue failed: %s",
                     strerror(dequeue_error));
            capture_stop(state);
            return;
        }
        if (buffer.index >= camera->buffer_count) {
            snprintf(state->error, sizeof(state->error), "invalid camera frame");
            capture_stop(state);
            return;
        }
        int64_t timestamp_us = (int64_t)buffer.timestamp.tv_sec * 1000000 +
                               buffer.timestamp.tv_usec;
        state->diagnostics.raw_frames[index]++;
        fps_sample(&state->diagnostics.raw_fps[index],
                   &state->diagnostics.raw_fps_samples[index],
                   &state->diagnostics.last_raw_timestamp_us[index],
                   timestamp_us);
        if (buffer.flags & V4L2_BUF_FLAG_ERROR) {
            if (camera_requeue(state, index, buffer.index, side) < 0) {
                snprintf(state->error, sizeof(state->error), "camera queue failed: %s",
                         strerror(errno));
                capture_stop(state);
                return;
            }
            continue;
        }
        if (plane.bytesused < (size_t)camera->stride * camera->height ||
            plane.bytesused > camera->buffers[buffer.index].length) {
            snprintf(state->error, sizeof(state->error), "short camera frame");
            capture_stop(state);
            return;
        }
        camera->last_frame_ms = monotonic_ms();
        int interval = state->focus_mode || state->calibration_mode ?
            FOCUS_INTERVAL_MS : PROCESS_INTERVAL_MS;
        if (state->last_pair_ms &&
            monotonic_ms() - state->last_pair_ms < interval) {
            state->diagnostics.skipped_frames[index]++;
            if (camera_requeue(state, index, buffer.index, side) < 0) {
                snprintf(state->error, sizeof(state->error),
                         "camera queue failed: %s", strerror(errno));
                capture_stop(state);
                return;
            }
            continue;
        }
        int64_t unpack_start_us = monotonic_us();
        struct ProfileStamp unpack_start = profile_start();
        camera->last_frame_ms = monotonic_ms();
        camera->neural_ready = 0;
        if (state->focus_mode)
            focus_unpack(state, camera->buffers[buffer.index].data, camera->stride);
        else if (state->calibration_mode) {
            native_unpack(state->calibration_pending + index * FOCUS_BMP_BYTES,
                          camera->buffers[buffer.index].data, camera->stride);
            downsample_raw10(camera->image, camera->buffers[buffer.index].data,
                             camera->stride);
        } else if (state->fisheye.valid)
            fisheye_rectify_raw10(state->fisheye_maps, index == LEFT_CAMERA ? 0 : 1,
                                  camera->buffers[buffer.index].data,
                                  camera->stride, camera->image);
        else if (camera->scaled)
            unpack_raw10(camera->image, camera->buffers[buffer.index].data,
                         camera->stride);
        else
            downsample_raw10(camera->image, camera->buffers[buffer.index].data,
                             camera->stride);
        profile_end_traced(&state->profile[PROFILE_UNPACK_LEFT + side], unpack_start, TIMELINE_CAMERA_CPU);
        if (!side && !state->focus_mode && !state->calibration_mode && state->depth_engine &&
            monotonic_ms() - state->neural_last_ms >= NEURAL_INTERVAL_MS) {
            struct ProfileStamp remap_start = profile_start();
            fisheye_remap_raw10(*state->neural_maps,
                               NEURAL_PIXELS, camera->buffers[buffer.index].data,
                               camera->stride, camera->neural_image);
            camera->neural_ready = 1;
            profile_end_traced(&state->profile[PROFILE_REMAP], remap_start, TIMELINE_CAMERA_CPU);
        }
        double unpack_ms = (monotonic_us() - unpack_start_us) / 1000.0;
        timing_sample(&state->diagnostics.unpack[index], unpack_ms);
        state->diagnostics.decoded_frames[index]++;
        camera->timestamp_us = timestamp_us;
        camera->unpack_ms = unpack_ms;
        camera->fresh = 1;
        if (camera_requeue(state, index, buffer.index, side) < 0) {
            snprintf(state->error, sizeof(state->error), "camera queue failed: %s",
                     strerror(errno));
            capture_stop(state);
            return;
        }
        camera->profile_range = (struct ProfileRange){
            dequeue_start.wall_ns, profile_clock(CLOCK_MONOTONIC)
        };
        if (state->focus_mode) {
            state->last_pair_ms = monotonic_ms();
            state->frame++;
            state->profile_cycle = (struct ProfileCycle){
                .frame = state->frame, .cameras = {camera->profile_range}
            };
            timing_sample(&state->diagnostics.unpack_work, unpack_ms);
            int64_t ae_start_us = monotonic_us();
            if (state->auto_exposure) {
                struct ProfileStamp ae_start = profile_start();
                auto_exposure_update(state, state->last_pair_ms);
                profile_end_traced(&state->profile[PROFILE_AE], ae_start, TIMELINE_CAMERA_CPU);
            }
            timing_sample(&state->diagnostics.auto_exposure,
                          (monotonic_us() - ae_start_us) / 1000.0);
            timing_sample(&state->diagnostics.process,
                          (monotonic_us() - unpack_start_us) / 1000.0);
            fps_sample(&state->diagnostics.output_fps,
                       &state->diagnostics.output_fps_samples,
                       &state->diagnostics.last_output_us, monotonic_us());
        } else {
            pair_if_ready(state);
        }
    }
}

static int save_calibration(const struct Server *state)
{
    const char *temporary = CALIBRATION_FILE ".tmp";
    FILE *output = fopen(temporary, "w");
    if (!output)
        return -1;
    int result = fprintf(output,
                         "4REELCAL3 %d %d %d %.17g %.17g %.17g %d\n",
                         DEPTH_W, DEPTH_H, state->calibration.valid,
                         state->calibration.offset_px,
                         state->calibration.scale_px_mm,
                         state->calibration.vertical_px, 1);
    int failed = result < 0;
    if (!failed && fflush(output) != 0)
        failed = 1;
    if (!failed && fsync(fileno(output)) != 0)
        failed = 1;
    if (fclose(output) != 0)
        failed = 1;
    if (failed) {
        unlink(temporary);
        return -1;
    }
    if (rename(temporary, CALIBRATION_FILE) != 0) {
        unlink(temporary);
        return -1;
    }
    return 0;
}

static void load_calibration(struct Server *state)
{
    FILE *input = fopen(CALIBRATION_FILE, "r");
    if (!input)
        return;
    struct Calibration calibration = {0};
    int width = 0, height = 0, valid = 0, swapped = 0;
    /* v3 coordinates include horizontal correction; reject mirrored v2 data. */
    if (fscanf(input, "4REELCAL3 %d %d %d %lf %lf %lf %d",
               &width, &height, &valid,
               &calibration.offset_px, &calibration.scale_px_mm,
               &calibration.vertical_px, &swapped) == 7 &&
        width == DEPTH_W && height == DEPTH_H &&
        /* Calibration from the old unchecked order needs new snapshots. */
        (valid == 0 || valid == 1) && swapped == 1 &&
        isfinite(calibration.offset_px) &&
        isfinite(calibration.scale_px_mm) &&
        isfinite(calibration.vertical_px) &&
        (!valid || calibration.scale_px_mm != 0)) {
        calibration.valid = valid;
        state->calibration = calibration;
    }
    fclose(input);
}

static int install_fisheye(struct Server *state, const struct FisheyeCalibration *calibration)
{
    struct FisheyeMaps *maps = malloc(sizeof(*maps));
    if (!maps)
        return -1;
    fisheye_build_maps(calibration, maps);
    free(state->fisheye_maps);
    state->fisheye_maps = maps;
    state->fisheye = *calibration;
    state->calibration = (struct Calibration){
        .scale_px_mm = calibration->fx * calibration->baseline_mm, .valid = 1};
    state->snapshot_valid[0] = state->snapshot_valid[1] = 0;
    return 0;
}

static void load_fisheye(struct Server *state)
{
    FILE *input = fopen(FISHEYE_FILE, "r");
    if (!input)
        return;
    char text[FISHEYE_TEXT_BYTES], error[160];
    struct FisheyeCalibration calibration;
    size_t length = fread(text, 1, sizeof(text) - 1, input);
    text[length] = 0;
    int extra = fgetc(input);
    int failed = ferror(input);
    fclose(input);
    if (!failed && extra == EOF &&
        !fisheye_parse(text, &calibration, error, sizeof(error))) {
        if (install_fisheye(state, &calibration))
            fprintf(stderr, "could not allocate fisheye rectification maps\n");
    } else {
        fprintf(stderr, "ignoring invalid fisheye calibration in %s\n", FISHEYE_FILE);
    }
}

static int read_number(const char *path, double *value)
{
    FILE *input = fopen(path, "r");
    if (!input)
        return -1;
    int result = fscanf(input, "%lf", value);
    fclose(input);
    return result == 1 && isfinite(*value) ? 0 : -1;
}

static void find_accelerometer(struct Server *state)
{
    for (int i = 0; i < 16; ++i) {
        char path[160];
        snprintf(path, sizeof(path), "/sys/bus/iio/devices/iio:device%d/name", i);
        FILE *input = fopen(path, "r");
        if (!input)
            continue;
        char name[64] = {0};
        int found = fgets(name, sizeof(name), input) && strstr(name, "lis2hh12");
        fclose(input);
        if (found) {
            snprintf(state->accel_path, sizeof(state->accel_path),
                     "/sys/bus/iio/devices/iio:device%d", i);
            return;
        }
    }
}

static int read_accelerometer(const struct Server *state, double values[3])
{
    if (!state->accel_path[0])
        return -1;
    const char *axis[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
        char path[192];
        double raw, scale;
        snprintf(path, sizeof(path), "%s/in_accel_%s_raw", state->accel_path,
                 axis[i]);
        if (read_number(path, &raw) < 0)
            return -1;
        snprintf(path, sizeof(path), "%s/in_accel_%s_scale", state->accel_path,
                 axis[i]);
        if (read_number(path, &scale) < 0)
            return -1;
        values[i] = raw * scale;
    }
    return 0;
}

/* The IIO driver's one-shot reads sleep for sensor startup on every axis.
 * Never perform them in the HTTP/capture event loop. */
static void *accelerometer_worker(void *argument)
{
    const struct Server *state = argument;
    for (;;) {
        pthread_mutex_lock(&accelerometer.lock);
        int stop = accelerometer.stop;
        pthread_mutex_unlock(&accelerometer.lock);
        if (stop)
            break;
        double values[3];
        int valid = read_accelerometer(state, values) == 0;
        pthread_mutex_lock(&accelerometer.lock);
        accelerometer.valid = valid;
        if (valid)
            memcpy(accelerometer.values, values, sizeof(values));
        pthread_mutex_unlock(&accelerometer.lock);
        struct timespec interval = {.tv_sec = 1};
        nanosleep(&interval, NULL);
    }
    return NULL;
}

static int cached_accelerometer(double values[3])
{
    pthread_mutex_lock(&accelerometer.lock);
    int valid = accelerometer.valid;
    memcpy(values, accelerometer.values, sizeof(accelerometer.values));
    pthread_mutex_unlock(&accelerometer.lock);
    return valid ? 0 : -1;
}

static void put16(uint8_t *out, uint16_t value)
{
    out[0] = value & 255;
    out[1] = value >> 8;
}

static void put32(uint8_t *out, uint32_t value)
{
    put16(out, value & 65535);
    put16(out + 2, value >> 16);
}

static const unsigned metric_distances[] = {
    STEREO_SEARCH_FAR_MM, 2000, 1000, 500, 250, 150, STEREO_SEARCH_NEAR_MM
};

static const uint8_t *metric_depth_indices(void)
{
    static uint8_t indices[STEREO_SEARCH_FAR_MM + 1];
    if (!indices[STEREO_SEARCH_NEAR_MM]) {
        const double far = STEREO_SEARCH_FAR_MM;
        const double scale = 254 / log(far / STEREO_SEARCH_NEAR_MM);
        for (unsigned mm = 1; mm <= STEREO_SEARCH_FAR_MM; ++mm) {
            unsigned distance = mm < STEREO_SEARCH_NEAR_MM ? STEREO_SEARCH_NEAR_MM : mm;
            indices[mm] = 1 + (uint8_t)(scale * log(far / distance) + 0.5);
        }
    }
    return indices;
}

static const uint8_t *metric_bmp_palette(void)
{
    static uint8_t bytes[1024];
    static const uint8_t stops[][3] = {
        {225, 45, 52}, {250, 112, 34}, {247, 206, 51}, {117, 210, 62},
        {41, 200, 159}, {48, 153, 237}, {54, 83, 232}
    };
    const unsigned last_stop = sizeof(stops) / sizeof(*stops) - 1;
    if (!bytes[6]) {
        for (unsigned i = 1; i < 256; ++i) {
            unsigned position = (i - 1) * last_stop;
            unsigned a = position / 254, b = a < last_stop ? a + 1 : a;
            unsigned weight = position % 254;
            for (unsigned channel = 0; channel < 3; ++channel)
                bytes[i * 4 + 2 - channel] = ((254 - weight) * stops[a][channel] +
                    weight * stops[b][channel] + 127) / 254;
        }
    }
    return bytes;
}

static void init_bmp(uint8_t *out, unsigned width, unsigned height, enum BmpPalette palette)
{
    memset(out, 0, 1078);
    out[0] = 'B';
    out[1] = 'M';
    put32(out + 2, 1078 + width * height);
    put32(out + 10, 1078);
    put32(out + 14, 40);
    put32(out + 18, width);
    put32(out + 22, height);
    put16(out + 26, 1);
    put16(out + 28, 8);
    put32(out + 34, width * height);
    put32(out + 46, 256);
    if (palette == BMP_DEPTH_COLOURS) {
        memcpy(out + 54, metric_bmp_palette(), 1024);
        return;
    }
    for (int i = 0; i < 256; ++i) {
        uint8_t *entry = out + 54 + i * 4;
        if (palette == BMP_LUMA) {
            uint8_t bright = image_display_luma((unsigned)i);
            entry[0] = entry[1] = entry[2] = bright;
        } else if (i) {
            entry[0] = i;
            entry[1] = 255 - abs(2 * i - 255);
            entry[2] = 255 - i;
        }
    }
}

static const uint8_t *make_bmp(const uint8_t *image, enum BmpPalette palette)
{
    init_bmp(bmp, DEPTH_W, DEPTH_H, palette);
    for (int y = 0; y < DEPTH_H; ++y)
        memcpy(bmp + 1078 + y * DEPTH_W,
               image + (DEPTH_H - 1 - y) * DEPTH_W, DEPTH_W);
    return bmp;
}

static const uint8_t *make_metric_bmp(const uint16_t *depth)
{
    init_bmp(bmp, DEPTH_W, DEPTH_H, BMP_DEPTH_COLOURS);
    const uint8_t *indices = metric_depth_indices();
    for (unsigned y = 0; y < DEPTH_H; ++y) {
        uint8_t *row = bmp + 1078 + (DEPTH_H - 1 - y) * DEPTH_W;
        for (unsigned x = 0; x < DEPTH_W; ++x) {
            unsigned mm = depth[y * DEPTH_W + x];
            row[x] = indices[mm < STEREO_SEARCH_FAR_MM ? mm : STEREO_SEARCH_FAR_MM];
        }
    }
    return bmp;
}

/* A single image keeps the metric preview and source map on the same frame. */
static const uint8_t *make_fused_bmp(const struct Server *state)
{
    static uint8_t image[1078 + 2 * PIXELS];
    init_bmp(image, 2 * DEPTH_W, DEPTH_H, BMP_STEREO_DEPTH);
    /* Reserve the high palette indices for the source key. */
    image[54 + 254 * 4] = 180;
    image[55 + 254 * 4] = 190;
    image[56 + 254 * 4] = 49;
    image[54 + 255 * 4] = 75;
    image[55 + 255 * 4] = 181;
    image[56 + 255 * 4] = 240;
    double scale = fabs(state->calibration.scale_px_mm);
    for (unsigned y = 0; y < DEPTH_H; ++y) {
        uint8_t *out = image + 1078 + (DEPTH_H - 1 - y) * 2 * DEPTH_W;
        for (unsigned x = 0; x < DEPTH_W; ++x) {
            unsigned i = y * DEPTH_W + x;
            unsigned depth = state->neural.fused_mm[i];
            double parallax = depth ? fmin(scale / depth, MAX_DISPARITY) : 0;
            out[x] = depth ? 1 + (uint8_t)(252 * parallax / MAX_DISPARITY + 0.5) : 0;
            out[x + DEPTH_W] = state->neural.source[i] == FUSION_STEREO ? 254 :
                state->neural.source[i] == FUSION_ESTIMATED ? 255 : 0;
        }
    }
    return image;
}

static const uint8_t *make_depth_pgm(const uint16_t *depth, size_t *length)
{
    int header = snprintf((char *)depth_pgm, 32, "P5\n%d %d\n65535\n",
                          DEPTH_W, DEPTH_H);
    for (int i = 0; i < PIXELS; ++i) {
        uint16_t value = depth[i];
        depth_pgm[header + i * 2] = value >> 8;
        depth_pgm[header + i * 2 + 1] = value & 255;
    }
    *length = header + PIXELS * 2;
    return depth_pgm;
}

static void put64(uint8_t *out, uint64_t value)
{
    put32(out, (uint32_t)value);
    put32(out + 4, (uint32_t)(value >> 32));
}

/* One HTTP response contains one stereo pair and the explicitly identified
 * asynchronous neural result. All scalar fields are little-endian. */
static const uint8_t *make_frame_packet(const struct Server *state, size_t *length)
{
    static uint8_t packet[80 + PIXELS * 12];
    _Static_assert(sizeof(float) == 4 && sizeof(double) == 8, "frame packet scalar sizes");
    memset(packet, 0, 80);
    memcpy(packet, "4REELD1", 7);
    put32(packet + 8, 1);
    put32(packet + 12, DEPTH_W);
    put32(packet + 16, DEPTH_H);
    unsigned flags = (state->fisheye.valid && state->depth_engine != 1 ? 1U : 0U) |
        (state->neural.available ? 2U : 0U) | (state->fisheye.valid ? 4U : 0U) |
        (state->neural.available && !state->fisheye.valid ? 8U : 0U) |
        (state->neural.available ? 16U : 0U);
    put32(packet + 20, flags);
    put64(packet + 24, state->frame);
    put64(packet + 32, (uint64_t)state->capture_timestamp_us);
    put64(packet + 40, state->neural.frame);
    const double projection[] = {state->fisheye.fx, state->fisheye.fy,
                                  state->fisheye.cx, state->fisheye.cy};
    for (unsigned i = 0; i < 4; ++i) {
        uint64_t bits;
        memcpy(&bits, &projection[i], sizeof(bits));
        put64(packet + 48 + i * 8, bits);
    }
    memcpy(packet + 80, state->latest[0], PIXELS);
    memcpy(packet + 80 + PIXELS, state->latest[1], PIXELS);
    for (unsigned i = 0; i < PIXELS; ++i)
        put16(packet + 80 + PIXELS * 2 + i * 2, state->depth_mm[i]);
    for (unsigned i = 0; i < PIXELS; ++i) {
        uint32_t bits;
        memcpy(&bits, &state->neural.inverse_depth[i], sizeof(bits));
        put32(packet + 80 + PIXELS * 4 + i * 4, bits);
    }
    /* Version 1 reserves its second neural plane. Bit 4 identifies left-only data. */
    *length = sizeof(packet);
    return packet;
}

static char index_page[65536];
static size_t index_length;

static int load_index(void)
{
    FILE *input = fopen(INDEX_FILE, "rb");
    if (!input)
        return -1;
    index_length = fread(index_page, 1, sizeof(index_page), input);
    int result = ferror(input) || !feof(input) || !index_length;
    fclose(input);
    return result ? -1 : 0;
}

static int form_value(const struct HttpRequest *request, const char *key,
                      char *value, size_t capacity)
{
    const char *entry = request->body;
    size_t key_length = strlen(key);
    while (*entry) {
        const char *end = strchr(entry, '&');
        if (!end)
            end = entry + strlen(entry);
        if ((size_t)(end - entry) > key_length &&
            !memcmp(entry, key, key_length) && entry[key_length] == '=') {
            const char *start = entry + key_length + 1;
            size_t length = end - start;
            if (!length || length >= capacity)
                return -1;
            memcpy(value, start, length);
            value[length] = 0;
            return 0;
        }
        entry = *end ? end + 1 : end;
    }
    return -1;
}

static int form_int(const struct HttpRequest *request, const char *key, int *value)
{
    char text[32], *end;
    if (form_value(request, key, text, sizeof(text)) < 0)
        return -1;
    errno = 0;
    long parsed = strtol(text, &end, 10);
    if (errno || *end || parsed < INT32_MIN || parsed > INT32_MAX)
        return -1;
    *value = (int)parsed;
    return 0;
}

static int form_double(const struct HttpRequest *request, const char *key,
                       double *value)
{
    char text[64], *end;
    if (form_value(request, key, text, sizeof(text)) < 0)
        return -1;
    errno = 0;
    double parsed = strtod(text, &end);
    if (errno || *end || !isfinite(parsed))
        return -1;
    *value = parsed;
    return 0;
}

static void respond(struct HttpResponse *response, int status,
                    const char *type, const void *body, size_t length)
{
    response->status = status;
    response->content_type = type;
    response->body = body;
    response->body_len = length;
    response->zipdepth_scale[0] = '\0';
}

static void set_zipdepth_scale(struct HttpResponse *response)
{
    const uint8_t *indices = metric_depth_indices();
    char *header = response->zipdepth_scale;
    size_t used = 0;
    for (unsigned i = 0; i < sizeof(metric_distances) / sizeof(*metric_distances); ++i) {
        unsigned mm = metric_distances[i];
        size_t remaining = sizeof(response->zipdepth_scale) - used;
        int length = snprintf(header + used, remaining, "%s[%u,%u]",
                              used ? "," : "[", (unsigned)indices[mm], mm);
        if (length < 0 || (size_t)length + 2 > remaining) {
            header[0] = '\0';
            return;
        }
        used += (size_t)length;
    }
    memcpy(header + used, "]", 2);
}

static const char *metric_legend_json(void)
{
    static char legend[256];
    if (!legend[0]) {
        const uint8_t *indices = metric_depth_indices(), *palette = metric_bmp_palette();
        size_t used = 0;
        for (unsigned i = 0; i < sizeof(metric_distances) / sizeof(*metric_distances); ++i) {
            unsigned mm = metric_distances[i];
            const uint8_t *bgr = palette + 4 * indices[mm];
            size_t remaining = sizeof(legend) - used;
            int length = snprintf(legend + used, remaining, "%s[%u,\"#%02x%02x%02x\"]",
                                  i ? "," : "[", mm, bgr[2], bgr[1], bgr[0]);
            if (length < 0 || (size_t)length + 2 > remaining) {
                legend[0] = '\0';
                return "[]";
            }
            used += (size_t)length;
        }
        memcpy(legend + used, "]", 2);
    }
    return legend;
}

static void respond_json(struct HttpResponse *response, int status,
                         const char *message)
{
    int length = snprintf(json, sizeof(json), "{\"error\":\"%s\"}", message);
    respond(response, status, "application/json", json, (size_t)length);
}

static void respond_ok(struct HttpResponse *response)
{
    static const char ok[] = "{\"ok\":true}";
    respond(response, 200, "application/json", ok, sizeof(ok) - 1);
}

static const char *diagnostic_number(char *buffer, size_t size, double value,
                                     uint64_t samples)
{
    if (!samples)
        return "null";
    snprintf(buffer, size, "%.3f", value);
    return buffer;
}

static void profile_response(const struct Server *state, struct HttpResponse *response)
{
    static const char *const stages[PROFILE_STAGE_COUNT] = {
        [PROFILE_DEQUEUE_LEFT] = "left_dequeue", [PROFILE_DEQUEUE_RIGHT] = "right_dequeue",
        [PROFILE_REQUEUE_LEFT] = "left_requeue", [PROFILE_REQUEUE_RIGHT] = "right_requeue",
        [PROFILE_UNPACK_LEFT] = "left_rectify", [PROFILE_UNPACK_RIGHT] = "right_rectify",
        [PROFILE_REMAP] = "neural_remap", [PROFILE_MATCH] = "stereo",
        [PROFILE_AE] = "exposure", [PROFILE_SENSOR] = "sensor_control",
        [PROFILE_QUEUE] = "neural_submit",
        [PROFILE_HTTP_IMAGE] = "http_image", [PROFILE_HTTP_STATUS] = "http_status",
        [PROFILE_HTTP_OTHER] = "http_other",
    };
    struct NeuralProfile neural;
    neural_profile(&neural);
    struct RveProfile rve = rve_profile();
    struct {
        const char *name;
        struct ProfileMetric metric;
    } accelerators[] = {
        {"rve_sad", rve.sad}, {"rve_dma", rve.dma},
        {"stereo_refine", rve.refine},
        {"neural_queue", {.calls = neural.started, .wall_ns = neural.queue_ns}},
        {"neural_pack", neural.pack}, {"npu_run", neural.run},
        {"neural_decode", neural.decode}, {"fusion", neural.fusion},
    };
    static char report[4096 + TIMELINE_LANES * (PROFILE_TIMELINE_SAMPLES + 3)];
    char load[8] = "null";
    unsigned percent;
    FILE *file = fopen("/proc/rknpu/load", "r");
    if (file) {
        if (fscanf(file, "NPU load: %u%%", &percent) == 1 && percent <= 100)
            snprintf(load, sizeof(load), "%u", percent);
        fclose(file);
    }
    uint64_t now = profile_clock(CLOCK_MONOTONIC);
    int length = snprintf(report, sizeof(report),
        "{\"wall_ms\":%.3f,\"process_cpu_ms\":%.3f,\"main_cpu_ms\":%.3f,"
        "\"npu_load_pct\":%s,\"replaced_jobs\":%" PRIu64 ",\"stages\":{",
        now / 1000000.0,
        profile_clock(CLOCK_PROCESS_CPUTIME_ID) / 1000000.0,
        profile_clock(CLOCK_THREAD_CPUTIME_ID) / 1000000.0,
        load, neural.dropped);
    for (unsigned i = 0; i < PROFILE_STAGE_COUNT + sizeof(accelerators) / sizeof(*accelerators); ++i) {
        unsigned extra = i - PROFILE_STAGE_COUNT;
        const char *name = i < PROFILE_STAGE_COUNT ? stages[i] : accelerators[extra].name;
        struct ProfileMetric value = i < PROFILE_STAGE_COUNT ?
            state->profile[i] : accelerators[extra].metric;
        int added = snprintf(report + length, sizeof(report) - length,
            "%s\"%s\":{\"calls\":%" PRIu64 ",\"cpu_ms\":%.3f,\"wall_ms\":%.3f}",
            i ? "," : "", name, value.calls, value.cpu_ns / 1000000.0, value.wall_ns / 1000000.0);
        if (added < 0 || (size_t)added + 2 >= sizeof(report) - length) {
            respond_json(response, 500, "profiling response exceeded its buffer");
            return;
        }
        length += added;
    }
    const struct ProfileCycle *cycle = neural.cycle.frame ? &neural.cycle : &state->profile_cycle;
    char rows[TIMELINE_LANES][PROFILE_TIMELINE_SAMPLES + 1];
    uint64_t duration = profile_timeline(cycle, rows);
    length += snprintf(report + length, sizeof(report) - length,
                       "},\"timeline\":{\"frame\":%" PRIu64 ",\"duration_ms\":%.3f,\"zipdepth\":%s,\"rows\":[",
                       cycle->frame, duration / 1000000.0, cycle->neural.end ? "true" : "false");
    for (unsigned i = 0; i < TIMELINE_LANES; ++i)
        length += snprintf(report + length, sizeof(report) - length,
                           "%s\"%s\"", i ? "," : "", rows[i]);
    memcpy(report + length, "]}}", 3);
    respond(response, 200, "application/json", report, (size_t)length + 3);
}

static void status_response(const struct Server *state, struct HttpResponse *response)
{
    double accel[3];
    char accel_json[160];
    char metric_json[11][32];
    int left = LEFT_CAMERA;
    int right = RIGHT_CAMERA;
    if (cached_accelerometer(accel) == 0)
        snprintf(accel_json, sizeof(accel_json),
                 "{\"x\":%.5f,\"y\":%.5f,\"z\":%.5f}",
                 accel[0], accel[1], accel[2]);
    else
        snprintf(accel_json, sizeof(accel_json), "null");
    int length = snprintf(json, sizeof(json),
        "{\"capture\":%s,\"frame\":%" PRIu64 ",\"pair_delta_ms\":%.3f,"
        "\"mode\":\"%s\",\"focus_camera\":\"%s\","
        "\"exposure\":%d,\"gain\":%d,\"auto_exposure\":%s,"
        "\"ae_level_left\":%d,\"ae_level_right\":%d,"
        "\"ae_clipped_percent\":%.3f,\"swapped\":true,"
        "\"calibrated\":%s,\"offset_px\":%.5f,"
        "\"scale_px_mm\":%.5f,\"vertical_px\":%.5f,"
        "\"metric_legend\":%s,"
        "\"pipeline\":\"%s\",\"timestamp_us\":%" PRId64 ","
        "\"fisheye_calibrated\":%s,\"fisheye_rms\":%.4f,\"baseline_mm\":%.4f,"
        "\"neural\":{\"available\":%s,\"frame\":%" PRIu64 ",\"pair_ms\":%.3f,\"inference_ms\":%.3f,\"error\":\"%s\","
        "\"projection\":\"%s\",\"hfov_degrees\":%.1f,\"metric\":%s,"
        "\"metric_frame\":%" PRIu64 ",\"metric_age_ms\":%" PRIu64 ","
        "\"metric_reference_frame\":%" PRIu64 "},"
        "\"fusion\":{\"fit\":%d,\"measured\":%u,\"estimated\":%u,\"anchors\":%u,\"inliers\":%u},"
        "\"accel\":%s,\"diagnostics\":{"
        "\"depth_backend\":\"%s\",\"sad_ms\":%.3f,\"sad_passes\":%u,"
        "\"capture_backend\":\"%s\","
        "\"capture_width\":%d,\"capture_height\":%d,"
        "\"processing_width\":%d,\"processing_height\":%d,"
        "\"output_fps\":%s,\"raw_fps_left\":%s,"
        "\"raw_fps_right\":%s,\"unpack_ms_left\":%s,"
        "\"unpack_ms_right\":%s,\"unpack_ms\":%s,"
        "\"depth_ms\":%s,"
        "\"ae_ms\":%s,\"process_ms\":%s,"
        "\"http_image_ms\":%s,\"http_status_ms\":%s,"
        "\"raw_left\":%" PRIu64 ",\"raw_right\":%" PRIu64 ","
        "\"decoded_left\":%" PRIu64 ",\"decoded_right\":%" PRIu64 ","
        "\"skipped_left\":%" PRIu64 ",\"skipped_right\":%" PRIu64 ","
        "\"sync_dropped_left\":%" PRIu64 ","
        "\"sync_dropped_right\":%" PRIu64 "},"
        "\"error\":\"%s\"}",
        state->capture ? "true" : "false", state->frame,
        state->frame && !state->focus_mode ? state->pair_delta_ms : -1.0,
        state->focus_mode ? "focus" : state->calibration_mode ? "calibration" : "depth",
        state->focus_side ? "right" : "left",
        state->exposure, state->gain,
        state->auto_exposure ? "true" : "false",
        state->ae_level[0], state->ae_level[1], state->ae_clipped_percent,
        state->calibration.valid ? "true" : "false",
        state->calibration.offset_px, state->calibration.scale_px_mm,
        state->calibration.vertical_px, metric_legend_json(),
        state->depth_engine == 1 ? "zipdepth" : state->depth_engine == 2 ? "both" : "stereo",
        state->capture_timestamp_us,
        state->fisheye.valid ? "true" : "false", state->fisheye.rms, fabs(state->fisheye.baseline_mm),
        state->neural.available ? "true" : "false", state->neural.frame,
        state->neural.milliseconds, state->neural.milliseconds, state->neural.error,
        state->fisheye.valid ? "calibrated" : "estimated-lens",
        state->fisheye.valid ? 2 * atan(DEPTH_W / (2 * state->fisheye.fx)) * 180 / 3.141592653589793 : 110.0,
        state->neural.available && state->neural.metric_frame ? "true" : "false",
        state->neural.metric_frame,
        state->neural.metric_capture_ns ?
            (profile_clock(CLOCK_MONOTONIC) - state->neural.metric_capture_ns) / 1000000 : 0,
        state->neural.metric_reference_frame,
        state->neural.fusion.fit, state->neural.fusion.measured,
        state->neural.fusion.estimated, state->neural.fusion.anchors, state->neural.fusion.inliers,
        accel_json,
        state->focus_mode ? "Paused for focus" : state->calibration_mode ?
            "Fisheye calibration capture" : state->depth_engine == 1 ? "ZipDepth only" : rve_sad_status(),
        state->focus_mode ? 0 : rve_sad_milliseconds(), state->focus_mode ? 0 : rve_sad_passes(),
        state->focus_mode ? "single camera, native RAW10" :
            state->calibration_mode ? "native stereo calibration" :
            state->fisheye.valid ? "native stereo with fisheye rectification" :
            state->depth_engine ? "native stereo, CPU /8" : "left CPU /8, right CIF /8",
        RAW_W, RAW_H, state->focus_mode || state->calibration_mode ? RAW_W : DEPTH_W,
        state->focus_mode || state->calibration_mode ? RAW_H : DEPTH_H,
        diagnostic_number(metric_json[0], sizeof(metric_json[0]),
                          state->diagnostics.output_fps,
                          state->diagnostics.output_fps_samples),
        diagnostic_number(metric_json[1], sizeof(metric_json[1]),
                          state->diagnostics.raw_fps[left],
                          state->diagnostics.raw_fps_samples[left]),
        diagnostic_number(metric_json[2], sizeof(metric_json[2]),
                          state->diagnostics.raw_fps[right],
                          state->diagnostics.raw_fps_samples[right]),
        diagnostic_number(metric_json[3], sizeof(metric_json[3]),
                          state->diagnostics.unpack[left].ms,
                          state->diagnostics.unpack[left].samples),
        diagnostic_number(metric_json[4], sizeof(metric_json[4]),
                          state->diagnostics.unpack[right].ms,
                          state->diagnostics.unpack[right].samples),
        diagnostic_number(metric_json[5], sizeof(metric_json[5]),
                          state->diagnostics.unpack_work.ms,
                          state->diagnostics.unpack_work.samples),
        diagnostic_number(metric_json[6], sizeof(metric_json[6]),
                          state->diagnostics.depth.ms,
                          state->diagnostics.depth.samples),
        diagnostic_number(metric_json[7], sizeof(metric_json[7]),
                          state->diagnostics.auto_exposure.ms,
                          state->diagnostics.auto_exposure.samples),
        diagnostic_number(metric_json[8], sizeof(metric_json[8]),
                          state->diagnostics.process.ms,
                          state->diagnostics.process.samples),
        diagnostic_number(metric_json[9], sizeof(metric_json[9]),
                          state->diagnostics.http_image.ms,
                          state->diagnostics.http_image.samples),
        diagnostic_number(metric_json[10], sizeof(metric_json[10]),
                          state->diagnostics.http_status.ms,
                          state->diagnostics.http_status.samples),
        state->diagnostics.raw_frames[left],
        state->diagnostics.raw_frames[right],
        state->diagnostics.decoded_frames[left],
        state->diagnostics.decoded_frames[right],
        state->diagnostics.skipped_frames[left],
        state->diagnostics.skipped_frames[right],
        state->diagnostics.sync_dropped[left],
        state->diagnostics.sync_dropped[right],
        state->error);
    respond(response, 200, "application/json", json, (size_t)length);
}

static void handle_post(const struct HttpRequest *request,
                        struct HttpResponse *response, struct Server *state)
{
    if (!request->control_header) {
        respond_json(response, 403, "dashboard control header required");
        return;
    }
    if (!strcmp(request->path, "/api/capture")) {
        int enabled;
        char mode[16] = "depth", side[16];
        if (form_int(request, "enabled", &enabled) < 0 ||
            (enabled != 0 && enabled != 1)) {
            respond_json(response, 400, "enabled must be 0 or 1");
            return;
        }
        if (enabled) {
            form_value(request, "mode", mode, sizeof(mode));
            if (strcmp(mode, "depth") && strcmp(mode, "focus") && strcmp(mode, "calibration")) {
                respond_json(response, 400, "mode must be depth, focus, or calibration");
                return;
            }
            int focus = !strcmp(mode, "focus"), focus_side = 0;
            int calibration_mode = !strcmp(mode, "calibration");
            int engine = state->depth_engine;
            char pipeline[16];
            if (!strcmp(mode, "depth") && !form_value(request, "pipeline", pipeline, sizeof(pipeline))) {
                if (!strcmp(pipeline, "stereo"))
                    engine = 0;
                else if (!strcmp(pipeline, "zipdepth"))
                    engine = 1;
                else if (!strcmp(pipeline, "both"))
                    engine = 2;
                else {
                    respond_json(response, 400, "pipeline must be stereo, zipdepth, or both");
                    return;
                }
            }
            if (focus) {
                if (form_value(request, "camera", side, sizeof(side)) < 0 ||
                    (strcmp(side, "left") && strcmp(side, "right"))) {
                    respond_json(response, 400, "focus camera must be left or right");
                    return;
                }
                focus_side = !strcmp(side, "right");
            }
            if (state->focus_mode != focus || state->focus_side != focus_side ||
                state->calibration_mode != calibration_mode || state->depth_engine != engine)
                capture_stop(state);
            state->focus_mode = focus;
            state->focus_side = focus_side;
            state->calibration_mode = calibration_mode;
            state->depth_engine = engine;
            if (capture_start(state) < 0) {
                respond_json(response, 503, state->error);
                return;
            }
        } else {
            capture_stop(state);
        }
        respond_ok(response);
        return;
    }
    if (!strcmp(request->path, "/api/fisheye")) {
        struct FisheyeCalibration calibration;
        char error[160], text[FISHEYE_TEXT_BYTES];
        if (fisheye_parse(request->body, &calibration, error, sizeof(error))) {
            respond_json(response, 400, error);
            return;
        }
        int length = fisheye_format(&calibration, text, sizeof(text));
        FILE *output = length > 0 ? fopen(FISHEYE_FILE ".tmp", "w") : NULL;
        if (!output) {
            respond_json(response, 500, "could not write fisheye calibration");
            return;
        }
        int failed = fwrite(text, 1, (size_t)length, output) != (size_t)length;
        if (fflush(output))
            failed = 1;
        if (fclose(output))
            failed = 1;
        if (failed || rename(FISHEYE_FILE ".tmp", FISHEYE_FILE)) {
            unlink(FISHEYE_FILE ".tmp");
            respond_json(response, 500, "could not save fisheye calibration");
            return;
        }
        capture_stop(state);
        if (install_fisheye(state, &calibration)) {
            respond_json(response, 503, "could not allocate fisheye rectification maps; restart the service");
            return;
        }
        respond_ok(response);
        return;
    }
    if (!strcmp(request->path, "/api/settings")) {
        int exposure, gain, auto_exposure;
        if (form_int(request, "exposure", &exposure) < 0 ||
            form_int(request, "gain", &gain) < 0 ||
            form_int(request, "auto_exposure", &auto_exposure) < 0 ||
            exposure < 4 || exposure > 3652 || gain < 16 || gain > 248 ||
            (auto_exposure != 0 && auto_exposure != 1)) {
            respond_json(response, 400,
                         "invalid exposure, gain, or auto exposure");
            return;
        }
        if (state->capture && apply_sensor_settings(state, exposure, gain) < 0) {
            apply_sensor_settings(state, state->exposure, state->gain);
            respond_json(response, 503, "could not set sensor controls");
            return;
        }
        state->exposure = exposure;
        state->gain = gain;
        state->auto_exposure = auto_exposure;
        state->ae_last_frame = 0;
        state->ae_neural_frame = 0;
        state->ae_neural_last_ms = 0;
        respond_ok(response);
        return;
    }
    if (!strcmp(request->path, "/api/snapshot")) {
        if (state->focus_mode) {
            respond_json(response, 409, "start stereo depth capture before taking calibration snapshots");
            return;
        }
        char slot[16];
        if (form_value(request, "slot", slot, sizeof(slot)) < 0 ||
            (strcmp(slot, "near") && strcmp(slot, "far"))) {
            respond_json(response, 400, "slot must be near or far");
            return;
        }
        if (!state->capture || !state->frame) {
            respond_json(response, 503, "start capture and wait for a stereo pair");
            return;
        }
        int index = !strcmp(slot, "far");
        memcpy(state->snapshots[index][0], state->latest[0], PIXELS);
        memcpy(state->snapshots[index][1], state->latest[1], PIXELS);
        state->snapshot_valid[index] = 1;
        respond_ok(response);
        return;
    }
    if (!strcmp(request->path, "/api/calibrate")) {
        if (state->fisheye.valid) {
            respond_json(response, 409, "fisheye stereo uses its measured calibration; use the fisheye calibration tool");
            return;
        }
        struct CalPoint near, far;
        if (!state->snapshot_valid[0] || !state->snapshot_valid[1]) {
            respond_json(response, 400, "take both near and far snapshots first");
            return;
        }
        if (form_double(request, "near_mm", &near.distance_mm) < 0 ||
            form_double(request, "near_lx", &near.lx) < 0 ||
            form_double(request, "near_ly", &near.ly) < 0 ||
            form_double(request, "near_rx", &near.rx) < 0 ||
            form_double(request, "near_ry", &near.ry) < 0 ||
            form_double(request, "far_mm", &far.distance_mm) < 0 ||
            form_double(request, "far_lx", &far.lx) < 0 ||
            form_double(request, "far_ly", &far.ly) < 0 ||
            form_double(request, "far_rx", &far.rx) < 0 ||
            form_double(request, "far_ry", &far.ry) < 0) {
            respond_json(response, 400, "missing or invalid calibration measurements");
            return;
        }
        struct Calibration calibration;
        char error[160];
        if (calibration_solve(&near, &far, &calibration, error, sizeof(error)) < 0) {
            respond_json(response, 400, error);
            return;
        }
        struct Calibration previous = state->calibration;
        state->calibration = calibration;
        if (save_calibration(state) < 0) {
            state->calibration = previous;
            respond_json(response, 500, "could not save calibration");
            return;
        }
        if (state->frame && !state->focus_mode)
            depth_compute_masked(state->latest[0], state->latest[1], &state->calibration,
                                 state->match_valid[0], state->match_valid[1],
                                 state->depth_visual, state->depth_mm);
        respond_ok(response);
        return;
    }
    respond_json(response, 404, "unknown API endpoint");
}

static void handle_get(const struct HttpRequest *request,
                       struct HttpResponse *response, struct Server *state)
{
    const char *path = request->path;
    if (!strcmp(path, "/")) {
        respond(response, 200, "text/html; charset=utf-8", index_page,
                index_length);
        return;
    }
    if (!strcmp(path, "/api/status")) {
        state->http_request_kind = HTTP_REQUEST_STATUS;
        if (state->capture && !state->focus_mode && !state->calibration_mode && state->depth_engine)
            update_neural_snapshot(state);
        status_response(state, response);
        return;
    }
    if (!strcmp(path, "/api/profile")) {
        profile_response(state, response);
        return;
    }
    if (!strcmp(path, "/api/fisheye")) {
        static char text[FISHEYE_TEXT_BYTES];
        if (!state->fisheye.valid) {
            respond_json(response, 409, "fisheye calibration has not been uploaded");
            return;
        }
        int length = fisheye_format(&state->fisheye, text, sizeof(text));
        respond(response, 200, "text/plain", text, (size_t)length);
        return;
    }
    if (!strcmp(path, "/api/calibration/pair.bin")) {
        if (!state->capture || !state->calibration_mode || !state->frame || !state->calibration_pair) {
            respond_json(response, 409, "start native calibration capture and wait for a synchronized pair");
            return;
        }
        uint8_t *header = state->calibration_pair;
        memset(header, 0, CAL_PAIR_HEADER_BYTES);
        memcpy(header, "4REELC1", 7);
        put32(header + 8, RAW_W);
        put32(header + 12, RAW_H);
        put64(header + 16, state->frame);
        put64(header + 24, (uint64_t)state->capture_timestamp_us);
        uint64_t bits;
        memcpy(&bits, &state->pair_delta_ms, sizeof(bits));
        put64(header + 32, bits);
        state->http_request_kind = HTTP_REQUEST_IMAGE;
        respond(response, 200, "application/octet-stream", state->calibration_pair,
                CAL_PAIR_HEADER_BYTES + 2 * FOCUS_BMP_BYTES);
        return;
    }
    if (!strcmp(path, "/api/frame.bin")) {
        if (!state->capture || state->focus_mode || state->calibration_mode || !state->frame) {
            respond_json(response, 409, "start depth capture and wait for a stereo pair");
            return;
        }
        if (state->depth_engine)
            update_neural_snapshot(state);
        size_t length;
        const uint8_t *packet = make_frame_packet(state, &length);
        state->http_request_kind = HTTP_REQUEST_IMAGE;
        respond(response, 200, "application/octet-stream", packet, length);
        return;
    }
    if (!strcmp(path, "/api/focus.bmp")) {
        if (!state->capture || !state->focus_mode || !state->frame || !state->focus_bmp) {
            respond_json(response, 409, "start focus capture and wait for an image");
            return;
        }
        state->http_request_kind = HTTP_REQUEST_IMAGE;
        respond(response, 200, "image/bmp", state->focus_bmp, FOCUS_BMP_BYTES);
        return;
    }
    if (state->focus_mode && (!strcmp(path, "/api/left.bmp") ||
        !strcmp(path, "/api/right.bmp") || !strcmp(path, "/api/depth.bmp") ||
        !strcmp(path, "/api/depth.pgm"))) {
        respond_json(response, 409, "stereo depth capture is paused for focus");
        return;
    }
    if (!strcmp(path, "/api/fused.bmp") || !strcmp(path, "/api/fused.pgm")) {
        update_neural_snapshot(state);
        if (!state->capture || state->focus_mode || state->calibration_mode ||
            !state->neural.available || !state->neural.fusion.fit) {
            respond_json(response, 409, "start calibrated Stereo + ZipDepth capture");
            return;
        }
        state->http_request_kind = HTTP_REQUEST_IMAGE;
        if (!strcmp(path, "/api/fused.pgm")) {
            size_t length;
            const uint8_t *data = make_depth_pgm(state->neural.fused_mm, &length);
            respond(response, 200, "application/octet-stream", data, length);
        } else {
            respond(response, 200, "image/bmp", make_fused_bmp(state), 1078 + 2 * PIXELS);
        }
        return;
    }
    if (!strcmp(path, "/api/zipdepth.bmp") || !strcmp(path, "/api/zipdepth.pgm")) {
        update_neural_snapshot(state);
        if (!state->capture || state->focus_mode || state->calibration_mode ||
            !state->neural.available || !state->neural.metric_frame) {
            respond_json(response, 409, "ZipDepth needs consistent stereo samples at different distances");
            return;
        }
        state->http_request_kind = HTTP_REQUEST_IMAGE;
        if (!strcmp(path, "/api/zipdepth.pgm")) {
            size_t length;
            const uint8_t *data = make_depth_pgm(state->neural.metric_mm, &length);
            respond(response, 200, "application/octet-stream", data, length);
        } else {
            respond(response, 200, "image/bmp",
                    make_metric_bmp(state->neural.metric_mm), BMP_BYTES);
            set_zipdepth_scale(response);
        }
        return;
    }
    const uint8_t *image = NULL;
    enum BmpPalette palette = BMP_LUMA;
    if (!strcmp(path, "/api/neural-right.bmp") || !strcmp(path, "/api/neural-input-right.bmp")) {
        respond_json(response, 409, "ZipDepth runs on the left view");
        return;
    }
    if (!strcmp(path, "/api/neural-left.bmp") || !strcmp(path, "/api/neural-input-left.bmp")) {
        update_neural_snapshot(state);
        if (state->capture && !state->focus_mode && !state->calibration_mode && state->neural.available) {
            int input = strstr(path, "input") != NULL;
            image = input ? state->neural.input : state->neural.visual;
            palette = input ? BMP_LUMA : BMP_DEPTH_COLOURS;
        }
    } else if (!strcmp(path, "/api/left.bmp") && state->frame)
        image = state->latest[0];
    else if (!strcmp(path, "/api/right.bmp") && state->frame)
        image = state->latest[1];
    else if (!strcmp(path, "/api/depth.bmp") && state->frame && !state->calibration_mode && state->depth_engine != 1) {
        if (state->calibration.valid) {
            state->http_request_kind = HTTP_REQUEST_IMAGE;
            respond(response, 200, "image/bmp", make_metric_bmp(state->depth_mm), BMP_BYTES);
            return;
        }
        image = state->depth_visual;
        palette = BMP_DEPTH_COLOURS;
    } else if (!strcmp(path, "/api/snapshot/near/left.bmp") &&
               state->snapshot_valid[0])
        image = state->snapshots[0][0];
    else if (!strcmp(path, "/api/snapshot/near/right.bmp") &&
             state->snapshot_valid[0])
        image = state->snapshots[0][1];
    else if (!strcmp(path, "/api/snapshot/far/left.bmp") &&
             state->snapshot_valid[1])
        image = state->snapshots[1][0];
    else if (!strcmp(path, "/api/snapshot/far/right.bmp") &&
             state->snapshot_valid[1])
        image = state->snapshots[1][1];

    if (image) {
        state->http_request_kind = HTTP_REQUEST_IMAGE;
        respond(response, 200, "image/bmp", make_bmp(image, palette), BMP_BYTES);
        return;
    }
    if (!strcmp(path, "/api/depth.pgm")) {
        if (!state->frame || !state->calibration.valid || state->calibration_mode || state->depth_engine == 1) {
            respond_json(response, 409, "calibrated depth is not ready");
            return;
        }
        size_t length;
        const uint8_t *data = make_depth_pgm(state->depth_mm, &length);
        state->http_request_kind = HTTP_REQUEST_IMAGE;
        respond(response, 200, "application/octet-stream", data, length);
        return;
    }
    if (!strncmp(path, "/api/", 5)) {
        respond_json(response, 503, "image not available yet");
        return;
    }
    respond_json(response, 404, "not found");
}

static void handle_request(const HttpRequest *request, HttpResponse *response,
                           void *context)
{
    struct Server *state = context;
    if (!strcmp(request->method, "GET"))
        handle_get(request, response, state);
    else if (!strcmp(request->method, "POST"))
        handle_post(request, response, state);
    else
        respond_json(response, 405, "method not allowed");
}

static int serve(const char *bind_address)
{
    int listener = http_listen(bind_address, 8080);
    if (listener < 0) {
        perror("listen");
        return 1;
    }
    fprintf(stderr, "4reel-web listening on http://%s:8080/\n", bind_address);
    while (!stopping) {
        struct pollfd fds[3] = {{0}};
        nfds_t count = 1;
        fds[0].fd = listener;
        fds[0].events = POLLIN;
        if (server.capture) {
            for (int i = 0; i < 2; ++i) {
                fds[count].fd = server.cameras[i].fd;
                fds[count].events = POLLIN;
                count++;
            }
        }
        int ready = poll(fds, count, 250);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            perror("poll");
            break;
        }
        if (server.capture) {
            for (int i = 0; i < 2 && server.capture; ++i) {
                if (fds[i + 1].revents & (POLLIN | POLLERR | POLLHUP))
                    camera_drain(&server, i);
            }
            for (int i = 0; i < 2 && server.capture; ++i) {
                if (server.cameras[i].fd >= 0 &&
                    monotonic_ms() - server.cameras[i].last_frame_ms > 2000) {
                    snprintf(server.error, sizeof(server.error),
                             "camera %d: no valid frames for 2 seconds", i);
                    capture_stop(&server);
                }
            }
        }
        if (fds[0].revents & POLLIN) {
            int64_t http_start_us = monotonic_us();
            struct ProfileStamp http_start = profile_start();
            server.http_request_kind = HTTP_REQUEST_OTHER;
            http_handle_client(listener, handle_request, &server);
            enum ProfileStage stage = server.http_request_kind == HTTP_REQUEST_IMAGE ?
                PROFILE_HTTP_IMAGE : server.http_request_kind == HTTP_REQUEST_STATUS ?
                PROFILE_HTTP_STATUS : PROFILE_HTTP_OTHER;
            profile_end(&server.profile[stage], http_start);
            double http_ms = (monotonic_us() - http_start_us) / 1000.0;
            if (server.http_request_kind == HTTP_REQUEST_IMAGE)
                timing_sample(&server.diagnostics.http_image, http_ms);
            else if (server.http_request_kind == HTTP_REQUEST_STATUS)
                timing_sample(&server.diagnostics.http_status, http_ms);
        }
    }
    capture_stop(&server);
    rve_sad_close();
    close(listener);
    return 0;
}

int main(int argc, char **argv)
{
    const char *bind_address = "192.168.77.1";
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--bind") && i + 1 < argc)
            bind_address = argv[++i];
        else if (!strcmp(argv[i], "--sad-self-test")) {
#ifdef FOURREEL_RVE
            int result = rve_sad_self_test();
            printf("%s\n", rve_sad_status());
            rve_sad_close();
            return result ? 1 : 0;
#else
            fputs("RVE hardware backend is not compiled in\n", stderr);
            return 1;
#endif
        } else if (!strcmp(argv[i], "--self-test")) {
            if (depth_self_test() < 0) {
                fprintf(stderr, "depth self-test failed\n");
                return 1;
            }
            puts("depth self-test passed");
            return 0;
        } else if (!strcmp(argv[i], "--help")) {
            puts("Usage: 4reel-web [--bind IPv4] [--self-test] [--sad-self-test]");
            return 0;
        } else {
            fprintf(stderr, "unknown argument: %s\n", argv[i]);
            return 1;
        }
    }
    server.cameras[0].fd = server.cameras[1].fd = -1;
    server.exposure = 3000;
    server.gain = 128;
    server.auto_exposure = 1;
    server.depth_engine = 2;
    load_calibration(&server);
    load_fisheye(&server);
    find_accelerometer(&server);
    if (load_index() < 0) {
        fprintf(stderr, "cannot load %s\n", INDEX_FILE);
        return 1;
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);
    if (server.accel_path[0]) {
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        int result = pthread_attr_setstacksize(&attr, 64 * 1024);
        if (!result)
            result = pthread_create(&accelerometer.thread, &attr,
                                    accelerometer_worker, &server);
        pthread_attr_destroy(&attr);
        accelerometer.started = result == 0;
        if (result)
            fprintf(stderr, "accelerometer worker: %s\n", strerror(result));
    }
    int result = serve(bind_address);
    free(server.fisheye_maps);
    if (accelerometer.started) {
        pthread_mutex_lock(&accelerometer.lock);
        accelerometer.stop = 1;
        pthread_mutex_unlock(&accelerometer.lock);
        pthread_join(accelerometer.thread, NULL);
    }
    return result;
}
