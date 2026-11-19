/* Host test with http.c depth.c -lm -lpthread and
 * -Wl,--wrap=open,--wrap=ioctl,--wrap=close. Sensor controls are mocked. */
#define main dashboard_main
#include "../server.c"
#undef main
#include <assert.h>
#include <stdarg.h>

static unsigned sensor_writes;

int __wrap_open(const char *path, int flags, ...)
{
    assert(!strcmp(path, "/dev/v4l-subdev2") ||
           !strcmp(path, "/dev/v4l-subdev5"));
    assert(flags == O_RDWR);
    return 42;
}

int __wrap_ioctl(int fd, unsigned long request, ...)
{
    assert(fd == 42 && request == VIDIOC_S_CTRL);
    va_list args;
    va_start(args, request);
    const struct v4l2_control *control = va_arg(args, struct v4l2_control *);
    va_end(args);
    if (control->id == V4L2_CID_EXPOSURE)
        assert(control->value >= 4 && control->value <= 3652);
    else {
        assert(control->id == V4L2_CID_ANALOGUE_GAIN);
        assert(control->value >= 16 && control->value <= 248);
    }
    sensor_writes++;
    return 0;
}

int __wrap_close(int fd)
{
    assert(fd == 42);
    return 0;
}

static void scene(int focus, uint8_t level)
{
    memset(&server, 0, sizeof(server));
    memset(server.latest, level, sizeof(server.latest));
    server.auto_exposure = 1;
    server.focus_mode = focus;
    server.exposure = 100;
    server.gain = 16;
    server.frame = 1;
    sensor_writes = 0;
    /* About 1% of the metered area is a saturated reflection. */
    for (unsigned x = 16; x < 80; x += 2)
        server.latest[0][10 * DEPTH_W + x] =
            server.latest[1][10 * DEPTH_W + x] = 255;
}

int main(void)
{
    uint8_t preview[BMP_BYTES];
    init_bmp(preview, DEPTH_W, DEPTH_H, 0);
    for (unsigned i = 0; i <= 16; ++i)
        assert(preview[54 + i * 4] == 0);
    assert(preview[54 + 17 * 4] > 0);
    assert(preview[54 + 255 * 4] == 255);
    for (unsigned i = 1; i < 256; ++i)
        assert(preview[54 + i * 4] >= preview[54 + (i - 1) * 4]);
    init_bmp(preview, DEPTH_W, DEPTH_H, BMP_DEPTH_COLOURS);
    assert(preview[54] == 0 && preview[55] == 0 && preview[56] == 0);
    assert(!memcmp(preview + 54 + 4, (uint8_t[]){52, 45, 225}, 3));
    assert(!memcmp(preview + 54 + 255 * 4, (uint8_t[]){232, 83, 54}, 3));

    /* A small reflection must not darken an already dim focus target. */
    scene(1, 40);
    auto_exposure_update(&server, 5000);
    assert(server.ae_clipped_percent > 0.5);
    assert(server.exposure > 100 && server.gain == 16);
    assert(sensor_writes == 4 && server.auto_exposure);
    int previous = server.exposure;
    server.frame++;
    auto_exposure_update(&server, 5000);
    assert(server.exposure > previous && sensor_writes == 8);

    /* Exposure stays settled despite the same small reflection. */
    scene(1, 128);
    auto_exposure_update(&server, 5000);
    assert(server.exposure == 100 && sensor_writes == 0);

    /* Broad overexposure still darkens the image in focus mode. */
    scene(1, 255);
    auto_exposure_update(&server, 5000);
    assert(server.exposure < 100 && sensor_writes == 4);

    /* Depth retains its strict highlight protection and update interval. */
    scene(0, 40);
    auto_exposure_update(&server, 5000);
    assert(server.exposure < 100 && sensor_writes == 4);
    previous = server.exposure;
    server.frame++;
    auto_exposure_update(&server, 5000);
    assert(server.exposure == previous && sensor_writes == 4);
    server.frame += 4;
    auto_exposure_update(&server, 5000);
    assert(server.exposure < previous && sensor_writes == 8);

    /* Neural AE meters the actual perspective input, tolerates reflections,
     * and never applies the same asynchronous input frame twice. */
    scene(0, 255);
    server.depth_engine = 2;
    server.neural.available = 1;
    server.neural.frame = 7;
    memset(server.neural.input, 40, sizeof(server.neural.input));
    for (unsigned x = 16; x < 80; x += 2)
        server.neural.input[10 * DEPTH_W + x] = 255;
    auto_exposure_update(&server, 5000);
    assert(server.exposure > 100 && server.ae_clipped_percent > 0.5 && sensor_writes == 4);
    previous = server.exposure;
    server.frame += 10;
    auto_exposure_update(&server, 5000);
    assert(server.exposure == previous && sensor_writes == 4);
    server.neural.frame++;
    auto_exposure_update(&server, 5100);
    assert(server.exposure == previous && sensor_writes == 4);
    auto_exposure_update(&server, 5500);
    assert(server.exposure > previous && sensor_writes == 8);

    scene(1, 40);
    server.auto_exposure = 0;
    auto_exposure_update(&server, 5000);
    assert(server.exposure == 100 && sensor_writes == 0);
    scene(1, 16);
    server.exposure = 3652;
    server.gain = 248;
    auto_exposure_update(&server, 5000);
    assert(server.exposure == 3652 && server.gain == 248 && !sensor_writes);
    puts("PASS: preview black level, focus/perspective AE, asynchronous feedback and depth/manual exposure");
    return 0;
}
