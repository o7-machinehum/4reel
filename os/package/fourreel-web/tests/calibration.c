/* Host test with http.c depth.c -lm -lpthread and
 * -Wl,--wrap=fopen,--wrap=rename,--wrap=unlink. Calibration I/O is redirected
 * into a private temporary directory; the host's /etc is never modified. */
#define main dashboard_main
#include "../server.c"
#undef main
#include <assert.h>

FILE *__real_fopen(const char *, const char *);
int __real_rename(const char *, const char *);
int __real_unlink(const char *);

static const char *redirect(const char *path)
{
    if (!strcmp(path, CALIBRATION_FILE)) return "calibration";
    if (!strcmp(path, CALIBRATION_FILE ".tmp")) return "calibration.tmp";
    return path;
}
FILE *__wrap_fopen(const char *path, const char *mode)
{
    return __real_fopen(redirect(path), mode);
}
int __wrap_rename(const char *old, const char *next)
{
    return __real_rename(redirect(old), redirect(next));
}
int __wrap_unlink(const char *path) { return __real_unlink(redirect(path)); }

int main(void)
{
    char directory[] = "/tmp/4reel-calibration-XXXXXX";
    assert(mkdtemp(directory) && chdir(directory) == 0);
    assert(LEFT_CAMERA == 1 && RIGHT_CAMERA == 0);
    for (int sign = -1; sign <= 1; sign += 2) {
        server.calibration = (struct Calibration){8.5 * sign, 5700 * sign, 2, 1};
        assert(save_calibration(&server) == 0);
        memset(&server.calibration, 0, sizeof(server.calibration));
        load_calibration(&server);
        assert(server.calibration.valid);
        assert(server.calibration.scale_px_mm == 5700 * sign);
        assert(server.calibration.offset_px == 8.5 * sign);
        assert(server.calibration.vertical_px == 2);
    }
    struct HttpRequest request = {.path = "/api/settings", .control_header = 1};
    struct HttpResponse response = {0};
    strcpy(request.body, "exposure=100&gain=16&auto_exposure=0");
    handle_post(&request, &response, &server);
    assert(response.status == 200 && server.calibration.valid);
    /* A cached old page cannot change the fixed order, either. */
    strcat(request.body, "&swapped=0");
    handle_post(&request, &response, &server);
    assert(response.status == 200 && server.calibration.valid);
    status_response(&server, &response);
    assert(strstr(json, "\"swapped\":true"));

    server.calibration.valid = 0;
    server.cameras[0].fresh = server.cameras[1].fresh = 1;
    memset(server.cameras[0].image, 20, PIXELS);
    memset(server.cameras[1].image, 80, PIXELS);
    pair_if_ready(&server);
    assert(server.frame == 1);
    assert(server.latest[0][0] == 80 && server.latest[1][0] == 20);

    /* Old calibration for the opposite camera order must not be reapplied. */
    FILE *old = fopen(CALIBRATION_FILE, "w");
    assert(old);
    assert(fprintf(old, "4REELCAL3 160 100 1 0 5000 0 0\n") > 0);
    assert(fclose(old) == 0);
    memset(&server.calibration, 0, sizeof(server.calibration));
    load_calibration(&server);
    assert(!server.calibration.valid);
    /* Matching camera order is not enough: v2 used mirrored coordinates. */
    old = fopen(CALIBRATION_FILE, "w");
    assert(old);
    assert(fprintf(old, "4REELCAL2 160 100 1 -1 -8400 3 1\n") > 0);
    assert(fclose(old) == 0);
    load_calibration(&server);
    assert(!server.calibration.valid);
    assert(unlink(CALIBRATION_FILE) == 0);
    assert(chdir("/") == 0 && rmdir(directory) == 0);
    puts("PASS: signed calibration persistence, fixed camera order and settings API");
    return 0;
}
