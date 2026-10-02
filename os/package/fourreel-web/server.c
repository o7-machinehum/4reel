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

#define RAW_W 1280
#define RAW_H 800
#define PIXELS (DEPTH_W * DEPTH_H)
#define MAX_BUFFERS 4
#define BMP_BYTES (14 + 40 + 1024 + PIXELS)
#define FOCUS_BMP_BYTES (14 + 40 + 1024 + RAW_W * RAW_H)
#define FOCUS_INTERVAL_MS 250
#define PROCESS_INTERVAL_MS 33
/* Fixed physical labels: the former "Swap cameras" enabled mapping. */
#define LEFT_CAMERA 1
#define RIGHT_CAMERA 0
#define CALIBRATION_FILE "/etc/4reel-web.calibration"
#define INDEX_FILE "/usr/share/4reel-web/index.html"
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
    int fresh;
    int64_t timestamp_us;
    double unpack_ms;
    int64_t last_frame_ms;
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

struct Server {
    struct Camera cameras[2];
    uint8_t latest[2][PIXELS];
    uint8_t snapshots[2][2][PIXELS];
    int snapshot_valid[2];
    uint8_t depth_visual[PIXELS];
    uint16_t depth_mm[PIXELS];
    struct Calibration calibration;
    uint64_t frame;
    int64_t last_pair_ms;
    double pair_delta_ms;
    int capture;
    int focus_mode;
    int focus_side;
    uint8_t *focus_bmp;
    int exposure;
    int gain;
    int auto_exposure;
    int ae_level[2];
    double ae_clipped_percent;
    uint64_t ae_last_frame;
    char error[128];
    char accel_path[128];
    struct Diagnostics diagnostics;
    enum HttpRequestKind http_request_kind;
};

static struct Server server;
static uint8_t bmp[BMP_BYTES];
static uint8_t depth_pgm[32 + PIXELS * 2];
static char json[3072];
static void init_bmp(uint8_t *out, unsigned width, unsigned height, int depth_palette);
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

static int set_sensor_control(const char *path, unsigned id, int value)
{
    int fd = open(path, O_RDWR);
    if (fd < 0)
        return -1;
    struct v4l2_control control = {.id = id, .value = value};
    int result = xioctl(fd, VIDIOC_S_CTRL, &control);
    int saved_errno = errno;
    close(fd);
    errno = saved_errno;
    return result;
}

static int apply_sensor_settings(int exposure, int gain)
{
    const char *paths[2] = {"/dev/v4l-subdev2", "/dev/v4l-subdev5"};
    for (int i = 0; i < 2; ++i) {
        if (set_sensor_control(paths[i], V4L2_CID_EXPOSURE, exposure) < 0 ||
            set_sensor_control(paths[i], V4L2_CID_ANALOGUE_GAIN, gain) < 0)
            return -1;
    }
    return 0;
}

static void capture_stop(struct Server *state)
{
    camera_close(&state->cameras[0]);
    camera_close(&state->cameras[1]);
    free(state->focus_bmp);
    state->focus_bmp = NULL;
    state->capture = 0;
    state->frame = 0;
    state->last_pair_ms = 0;
    state->ae_last_frame = 0;
    state->diagnostics.raw_fps[0] = state->diagnostics.raw_fps[1] = 0;
    state->diagnostics.output_fps = 0;
    state->diagnostics.last_raw_timestamp_us[0] = 0;
    state->diagnostics.last_raw_timestamp_us[1] = 0;
    state->diagnostics.last_output_us = 0;
}

static int capture_start(struct Server *state)
{
    char paths[2][32];

    if (state->capture)
        return 0;
    if (find_camera_nodes(paths, state->focus_mode) < 0 ||
        apply_sensor_settings(state->exposure, state->gain) < 0)
        goto fail;
    if (state->focus_mode) {
        int index = state->focus_side ? RIGHT_CAMERA : LEFT_CAMERA;
        rve_sad_close();
        state->focus_bmp = malloc(FOCUS_BMP_BYTES);
        if (!state->focus_bmp)
            goto fail;
        init_bmp(state->focus_bmp, RAW_W, RAW_H, 0);
        if (camera_open(&state->cameras[index], paths[index], 0) < 0)
            goto fail;
    } else if (camera_open(&state->cameras[0], paths[0], 1) < 0 ||
               camera_open(&state->cameras[1], paths[1], 0) < 0) {
        goto fail;
    }
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

static void auto_exposure_update(struct Server *state)
{
    double clipped[2];
    double factor;
    int level;
    int exposure;
    int gain;

    if (!state->auto_exposure ||
        (state->ae_last_frame &&
         state->frame - state->ae_last_frame < AE_INTERVAL_FRAMES))
        return;
    state->ae_last_frame = state->frame;
    for (int i = 0; i < 2; ++i)
        state->ae_level[i] = image_percentile(state->latest[i], 75, &clipped[i]);
    state->ae_clipped_percent = clipped[0] > clipped[1] ? clipped[0] : clipped[1];
    level = state->ae_level[0] < state->ae_level[1] ?
        state->ae_level[0] : state->ae_level[1];

    if (state->ae_clipped_percent > AE_CLIP_PERCENT) {
        factor = 0.8;
    } else if (level < AE_LOW || level > AE_HIGH) {
        factor = (double)(AE_TARGET - 16) / (level > 16 ? level - 16 : 1);
        if (factor < 0.8)
            factor = 0.8;
        if (factor > 1.25)
            factor = 1.25;
    } else {
        return;
    }

    exposure_gain_for_product((double)state->exposure * state->gain * factor,
                              &exposure, &gain);
    if (exposure == state->exposure && gain == state->gain)
        return;
    if (apply_sensor_settings(exposure, gain) < 0) {
        int saved_errno = errno;
        apply_sensor_settings(state->exposure, state->gain);
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
    if (now - state->last_pair_ms < PROCESS_INTERVAL_MS)
        return;
    int64_t process_start_us = monotonic_us();
    const struct Camera *left = &state->cameras[LEFT_CAMERA];
    const struct Camera *right = &state->cameras[RIGHT_CAMERA];
    memcpy(state->latest[0], left->image, PIXELS);
    memcpy(state->latest[1], right->image, PIXELS);
    int64_t depth_start_us = monotonic_us();
    depth_compute(state->latest[0], state->latest[1], &state->calibration,
                  state->depth_visual, state->depth_mm);
    timing_sample(&state->diagnostics.depth,
                  (monotonic_us() - depth_start_us) / 1000.0);
    state->pair_delta_ms = fabs((double)delta) / 1000.0;
    state->last_pair_ms = now;
    state->frame++;
    int64_t ae_start_us = monotonic_us();
    auto_exposure_update(state);
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

/* Native packed RAW10 -> 8-bit BMP, without spatial downsampling. Only one
 * full-size buffer is allocated, and only while focus capture is active.
 * Reverse horizontal order here too, matching the depth inputs. */
static void focus_unpack(struct Server *state, const uint8_t *source, unsigned stride)
{
    for (unsigned y = 0; y < RAW_H; ++y) {
        const uint8_t *p = source + y * stride;
        uint8_t *row = state->focus_bmp + 1078 + (RAW_H - 1 - y) * RAW_W;
        for (unsigned x = 0; x < RAW_W; x += 4, p += 5) {
            row[RAW_W - 1 - x] = (p[0] >> 2) | ((p[1] & 3U) << 6);
            row[RAW_W - 2 - x] = (p[1] >> 4) | ((p[2] & 15U) << 4);
            row[RAW_W - 3 - x] = (p[2] >> 6) | ((p[3] & 63U) << 2);
            row[RAW_W - 4 - x] = p[4];
        }
    }
    /* Reuse the small AE input arrays; in focus mode meter only the active
     * camera, rather than a stale image from the inactive sensor. */
    for (unsigned y = 0; y < DEPTH_H; ++y)
        for (unsigned x = 0; x < DEPTH_W; ++x)
            state->latest[0][y * DEPTH_W + x] =
                state->focus_bmp[1078 + (RAW_H - 1 - (y * 8 + 4)) * RAW_W + x * 8 + 4];
    memcpy(state->latest[1], state->latest[0], PIXELS);
}

static void camera_drain(struct Server *state, int index)
{
    struct Camera *camera = &state->cameras[index];
    for (;;) {
        struct v4l2_buffer buffer = {0};
        struct v4l2_plane plane = {0};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.m.planes = &plane;
        buffer.length = 1;
        if (xioctl(camera->fd, VIDIOC_DQBUF, &buffer) < 0) {
            if (errno == EAGAIN)
                return;
            snprintf(state->error, sizeof(state->error), "camera dequeue failed: %s",
                     strerror(errno));
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
            if (camera_queue(camera, buffer.index) < 0) {
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
        int interval = state->focus_mode ? FOCUS_INTERVAL_MS : PROCESS_INTERVAL_MS;
        if (state->last_pair_ms &&
            monotonic_ms() - state->last_pair_ms < interval) {
            state->diagnostics.skipped_frames[index]++;
            if (camera_queue(camera, buffer.index) < 0) {
                snprintf(state->error, sizeof(state->error),
                         "camera queue failed: %s", strerror(errno));
                capture_stop(state);
                return;
            }
            continue;
        }
        int64_t unpack_start_us = monotonic_us();
        camera->last_frame_ms = monotonic_ms();
        if (state->focus_mode)
            focus_unpack(state, camera->buffers[buffer.index].data, camera->stride);
        else if (camera->scaled)
            unpack_raw10(camera->image, camera->buffers[buffer.index].data,
                         camera->stride);
        else
            downsample_raw10(camera->image, camera->buffers[buffer.index].data,
                             camera->stride);
        double unpack_ms = (monotonic_us() - unpack_start_us) / 1000.0;
        timing_sample(&state->diagnostics.unpack[index], unpack_ms);
        state->diagnostics.decoded_frames[index]++;
        camera->timestamp_us = timestamp_us;
        camera->unpack_ms = unpack_ms;
        camera->fresh = 1;
        if (camera_queue(camera, buffer.index) < 0) {
            snprintf(state->error, sizeof(state->error), "camera queue failed: %s",
                     strerror(errno));
            capture_stop(state);
            return;
        }
        if (state->focus_mode) {
            state->last_pair_ms = monotonic_ms();
            state->frame++;
            timing_sample(&state->diagnostics.unpack_work, unpack_ms);
            int64_t ae_start_us = monotonic_us();
            auto_exposure_update(state);
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

static void init_bmp(uint8_t *out, unsigned width, unsigned height, int depth_palette)
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
    for (int i = 0; i < 256; ++i) {
        uint8_t *entry = out + 54 + i * 4;
        if (depth_palette && i) {
            entry[0] = 255 - i;
            entry[1] = 255 - abs(2 * i - 255);
            entry[2] = i;
        } else if (!depth_palette) {
            uint8_t bright = (uint8_t)(pow((double)i / 255.0, 0.45) * 255.0 + 0.5);
            entry[0] = entry[1] = entry[2] = bright;
        }
    }
}

static const uint8_t *make_bmp(const uint8_t *image, int depth_palette)
{
    init_bmp(bmp, DEPTH_W, DEPTH_H, depth_palette);
    for (int y = 0; y < DEPTH_H; ++y)
        memcpy(bmp + 1078 + y * DEPTH_W,
               image + (DEPTH_H - 1 - y) * DEPTH_W, DEPTH_W);
    return bmp;
}

static const uint8_t *make_depth_pgm(const struct Server *state, size_t *length)
{
    int header = snprintf((char *)depth_pgm, 32, "P5\n%d %d\n65535\n",
                          DEPTH_W, DEPTH_H);
    for (int i = 0; i < PIXELS; ++i) {
        uint16_t value = state->depth_mm[i];
        depth_pgm[header + i * 2] = value >> 8;
        depth_pgm[header + i * 2 + 1] = value & 255;
    }
    *length = header + PIXELS * 2;
    return depth_pgm;
}

static char index_page[49152];
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
        state->focus_mode ? "focus" : "depth", state->focus_side ? "right" : "left",
        state->exposure, state->gain,
        state->auto_exposure ? "true" : "false",
        state->ae_level[0], state->ae_level[1], state->ae_clipped_percent,
        state->calibration.valid ? "true" : "false",
        state->calibration.offset_px, state->calibration.scale_px_mm,
        state->calibration.vertical_px, accel_json,
        state->focus_mode ? "Paused for focus" : rve_sad_status(),
        state->focus_mode ? 0 : rve_sad_milliseconds(), state->focus_mode ? 0 : rve_sad_passes(),
        state->focus_mode ? "single camera, native RAW10" : "left CPU /8, right CIF /8",
        RAW_W, RAW_H, state->focus_mode ? RAW_W : DEPTH_W,
        state->focus_mode ? RAW_H : DEPTH_H,
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
            if (strcmp(mode, "depth") && strcmp(mode, "focus")) {
                respond_json(response, 400, "mode must be depth or focus");
                return;
            }
            int focus = !strcmp(mode, "focus"), focus_side = 0;
            if (focus) {
                if (form_value(request, "camera", side, sizeof(side)) < 0 ||
                    (strcmp(side, "left") && strcmp(side, "right"))) {
                    respond_json(response, 400, "focus camera must be left or right");
                    return;
                }
                focus_side = !strcmp(side, "right");
            }
            if (state->focus_mode != focus || state->focus_side != focus_side)
                capture_stop(state);
            state->focus_mode = focus;
            state->focus_side = focus_side;
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
        if (state->capture && apply_sensor_settings(exposure, gain) < 0) {
            apply_sensor_settings(state->exposure, state->gain);
            respond_json(response, 503, "could not set sensor controls");
            return;
        }
        state->exposure = exposure;
        state->gain = gain;
        state->auto_exposure = auto_exposure;
        state->ae_last_frame = 0;
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
            depth_compute(state->latest[0], state->latest[1], &state->calibration,
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
        status_response(state, response);
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
    const uint8_t *image = NULL;
    int palette = 0;
    if (!strcmp(path, "/api/left.bmp") && state->frame)
        image = state->latest[0];
    else if (!strcmp(path, "/api/right.bmp") && state->frame)
        image = state->latest[1];
    else if (!strcmp(path, "/api/depth.bmp") && state->frame) {
        image = state->depth_visual;
        palette = 1;
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
        if (!state->frame || !state->calibration.valid) {
            respond_json(response, 409, "calibrated depth is not ready");
            return;
        }
        size_t length;
        const uint8_t *data = make_depth_pgm(state, &length);
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
            server.http_request_kind = HTTP_REQUEST_OTHER;
            http_handle_client(listener, handle_request, &server);
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
    load_calibration(&server);
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
    if (accelerometer.started) {
        pthread_mutex_lock(&accelerometer.lock);
        accelerometer.stop = 1;
        pthread_mutex_unlock(&accelerometer.lock);
        pthread_join(accelerometer.thread, NULL);
    }
    return result;
}
