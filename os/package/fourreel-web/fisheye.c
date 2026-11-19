#include "fisheye.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int invalid(char *error, size_t size, const char *message)
{
    snprintf(error, size, "%s", message);
    return -1;
}

static int number(const char **cursor, double *value)
{
    char *end;
    *value = strtod(*cursor, &end);
    if (end == *cursor || !isfinite(*value))
        return -1;
    *cursor = end;
    return 0;
}

int fisheye_parse(const char *text, struct FisheyeCalibration *calibration,
                  char *error, size_t error_size)
{
    struct FisheyeCalibration next = {0};
    int version, width, height, output_width, output_height, consumed = 0;
    if (sscanf(text, "4REELFISHEYE%d %d %d %d %d %n", &version, &width, &height,
               &output_width, &output_height, &consumed) != 5 || !consumed ||
        (version != 1 && version != 2) ||
        width != FISHEYE_RAW_W || height != FISHEYE_RAW_H ||
        output_width != DEPTH_W || output_height != DEPTH_H)
        return invalid(error, error_size, "unsupported fisheye calibration format or image size");
    const char *cursor = text + consumed;
    double *projection[] = {&next.fx, &next.fy, &next.cx, &next.cy,
                            &next.baseline_mm, &next.rms};
    for (unsigned i = 0; i < sizeof(projection) / sizeof(projection[0]); ++i)
        if (number(&cursor, projection[i]))
            return invalid(error, error_size, "invalid fisheye projection");
    if (next.fx < 1 || next.fx > 10000 || next.fy < 1 || next.fy > 10000 ||
        next.cx < -DEPTH_W || next.cx > DEPTH_W * 2 ||
        next.cy < -DEPTH_H || next.cy > DEPTH_H * 2 ||
        fabs(next.baseline_mm) < 0.1 || fabs(next.baseline_mm) > 10000 ||
        (next.rms < 0 && !(version == 2 && next.rms == -1)) || next.rms > 3)
        return invalid(error, error_size, "invalid focal length, baseline, or reprojection RMS (maximum 3 px)");
    for (int side = 0; side < 2; ++side) {
        struct FisheyeCamera *camera = &next.camera[side];
        if (version == 2) {
            double model;
            if (number(&cursor, &model) ||
                (model != FISHEYE_EQUIDISTANT && model != FISHEYE_DOUBLE_SPHERE) ||
                number(&cursor, &camera->xi) || number(&cursor, &camera->alpha))
                return invalid(error, error_size, "invalid camera model parameters");
            camera->model = (int)model;
            if (camera->model == FISHEYE_EQUIDISTANT && (camera->xi || camera->alpha))
                return invalid(error, error_size, "equidistant model has no xi or alpha");
            if (camera->model == FISHEYE_DOUBLE_SPHERE) {
                if (camera->alpha < 0 || camera->alpha > 1)
                    return invalid(error, error_size, "double sphere alpha must be between 0 and 1");
                double w = camera->alpha <= 0.5 ? camera->alpha / (1 - camera->alpha) :
                                                  (1 - camera->alpha) / camera->alpha;
                double domain = 1 + camera->xi * camera->xi + 2 * w * camera->xi;
                if (!isfinite(domain) || domain <= 0)
                    return invalid(error, error_size, "invalid double sphere projection domain");
            }
        }
        double *intrinsics[] = {&camera->fx, &camera->fy, &camera->cx, &camera->cy};
        for (unsigned i = 0; i < 4; ++i)
            if (number(&cursor, intrinsics[i]) ||
                number(&cursor, &camera->distortion[i]))
                return invalid(error, error_size, "invalid fisheye intrinsics");
        if (camera->model == FISHEYE_DOUBLE_SPHERE)
            for (int i = 0; i < 4; ++i)
                if (camera->distortion[i])
                    return invalid(error, error_size, "double sphere requires distortion model none");
        if (camera->fx < 1 || camera->fx > 10000 || camera->fy < 1 ||
            camera->fy > 10000 || camera->cx < 0 || camera->cx >= width ||
            camera->cy < 0 || camera->cy >= height)
            return invalid(error, error_size, "fisheye intrinsics are outside the native image");
        for (int i = 0; i < 9; ++i)
            if (number(&cursor, &camera->rotation[i]))
                return invalid(error, error_size, "invalid fisheye rotation");
        const double *r = camera->rotation;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                double product = 0;
                for (int k = 0; k < 3; ++k)
                    product += r[i * 3 + k] * r[j * 3 + k];
                if (fabs(product - (i == j)) > 0.001)
                    return invalid(error, error_size, "fisheye rotation must be orthonormal");
            }
        double determinant = r[0] * (r[4] * r[8] - r[5] * r[7]) -
                             r[1] * (r[3] * r[8] - r[5] * r[6]) +
                             r[2] * (r[3] * r[7] - r[4] * r[6]);
        if (fabs(determinant - 1) > 0.001)
            return invalid(error, error_size, "fisheye rotation must be right-handed");
    }
    while (isspace((unsigned char)*cursor))
        cursor++;
    if (*cursor)
        return invalid(error, error_size, "unexpected data after fisheye calibration");
    next.valid = 1;
    *calibration = next;
    return 0;
}

int fisheye_format(const struct FisheyeCalibration *calibration,
                   char *text, size_t size)
{
    int version = calibration->rms < 0 || calibration->camera[0].model ||
                  calibration->camera[1].model ? 2 : 1;
    int length = snprintf(text, size, "4REELFISHEYE%d %d %d %d %d %.9g %.9g %.9g %.9g %.9g %.9g",
                          version, FISHEYE_RAW_W, FISHEYE_RAW_H, DEPTH_W, DEPTH_H,
                          calibration->fx, calibration->fy, calibration->cx,
                          calibration->cy, calibration->baseline_mm, calibration->rms);
    if (length < 0 || (size_t)length >= size)
        return -1;
    for (int side = 0; side < 2; ++side) {
        const struct FisheyeCamera *camera = &calibration->camera[side];
        if (version == 2) {
            int added = snprintf(text + length, size - (size_t)length, " %d %.9g %.9g",
                                 camera->model, camera->xi, camera->alpha);
            if (added < 0 || (size_t)added >= size - (size_t)length)
                return -1;
            length += added;
        }
        const double intrinsics[] = {camera->fx, camera->fy, camera->cx, camera->cy};
        for (int i = 0; i < 4; ++i) {
            int added = snprintf(text + length, size - (size_t)length, " %.9g %.9g",
                                 intrinsics[i], camera->distortion[i]);
            if (added < 0 || (size_t)added >= size - (size_t)length)
                return -1;
            length += added;
        }
        for (int i = 0; i < 9; ++i) {
            int added = snprintf(text + length, size - (size_t)length, " %.9g", camera->rotation[i]);
            if (added < 0 || (size_t)added >= size - (size_t)length)
                return -1;
            length += added;
        }
    }
    return length;
}

void fisheye_build_map(const struct FisheyeCalibration *calibration, int side,
                       unsigned width, unsigned height, uint16_t (*map)[2])
{
        const struct FisheyeCamera *camera = &calibration->camera[side];
        const double *r = camera->rotation;
        double domain = 0;
        if (camera->model == FISHEYE_DOUBLE_SPHERE) {
            double w = camera->alpha <= 0.5 ? camera->alpha / (1 - camera->alpha) :
                                              (1 - camera->alpha) / camera->alpha;
            domain = (w + camera->xi) / sqrt(1 + camera->xi * camera->xi + 2 * w * camera->xi);
        }
        for (unsigned y = 0; y < height; ++y)
            for (unsigned x = 0; x < width; ++x) {
                uint16_t *point = map[y * width + x];
                point[0] = point[1] = FISHEYE_INVALID;
                double u = (x * (double)DEPTH_W / width - calibration->cx) / calibration->fx;
                double v = (y * (double)DEPTH_H / height - calibration->cy) / calibration->fy;
                /* Undo rectification before applying the native fisheye model. */
                double rx = r[0] * u + r[3] * v + r[6];
                double ry = r[1] * u + r[4] * v + r[7];
                double rz = r[2] * u + r[5] * v + r[8];
                double px, py;
                if (camera->model == FISHEYE_DOUBLE_SPHERE) {
                    double length = sqrt(rx * rx + ry * ry + rz * rz);
                    if (rz <= -domain * length)
                        continue;
                    double shifted_z = camera->xi * length + rz;
                    double second = sqrt(rx * rx + ry * ry + shifted_z * shifted_z);
                    double denominator = camera->alpha * second + (1 - camera->alpha) * shifted_z;
                    if (denominator <= 0)
                        continue;
                    px = camera->fx * rx / denominator + camera->cx;
                    py = camera->fy * ry / denominator + camera->cy;
                } else {
                    if (rz <= 0)
                        continue;
                    double radius = hypot(rx, ry);
                    double theta = atan2(radius, rz);
                    double squared = theta * theta;
                    const double *d = camera->distortion;
                    double distorted = theta * (1 + squared * (d[0] + squared *
                        (d[1] + squared * (d[2] + squared * d[3]))));
                    double scale = radius > 1e-12 ? distorted / radius : 1;
                    px = camera->fx * rx * scale + camera->cx;
                    py = camera->fy * ry * scale + camera->cy;
                }
                /* Calibration images use the dashboard's horizontal correction. */
                px = FISHEYE_RAW_W - 1 - px;
                if (!isfinite(px) || !isfinite(py) || px < 0 || py < 0 ||
                    px >= FISHEYE_RAW_W - 1 || py >= FISHEYE_RAW_H - 1)
                    continue;
                point[0] = (uint16_t)(px * 16);
                point[1] = (uint16_t)(py * 16);
            }
}

void fisheye_build_maps(const struct FisheyeCalibration *calibration,
                        struct FisheyeMaps *maps)
{
    for (int side = 0; side < 2; ++side)
        fisheye_build_map(calibration, side, DEPTH_W, DEPTH_H, maps->xy[side]);
}

static unsigned raw_pixel(const uint8_t *row, unsigned x)
{
    const uint8_t *p = row + x / 4 * 5;
    switch (x & 3) {
    case 0: return (p[0] >> 2) | ((p[1] & 3U) << 6);
    case 1: return (p[1] >> 4) | ((p[2] & 15U) << 4);
    case 2: return (p[2] >> 6) | ((p[3] & 63U) << 2);
    default: return p[4];
    }
}

void fisheye_remap_raw10(const uint16_t (*map)[2], unsigned count,
                        const uint8_t *raw, unsigned stride, uint8_t *image)
{
    for (unsigned i = 0; i < count; ++i) {
        const uint16_t *point = map[i];
        if (point[0] == FISHEYE_INVALID) {
            image[i] = 0;
            continue;
        }
        unsigned x = point[0] >> 4, y = point[1] >> 4;
        unsigned dx = point[0] & 15, dy = point[1] & 15;
        const uint8_t *row = raw + y * stride;
        unsigned top = raw_pixel(row, x) * (16 - dx) + raw_pixel(row, x + 1) * dx;
        row += stride;
        unsigned bottom = raw_pixel(row, x) * (16 - dx) + raw_pixel(row, x + 1) * dx;
        image[i] = (uint8_t)((top * (16 - dy) + bottom * dy + 128) >> 8);
    }
}

void fisheye_rectify_raw10(const struct FisheyeMaps *maps, int side,
                           const uint8_t *raw, unsigned stride, uint8_t *image)
{
    fisheye_remap_raw10(maps->xy[side], DEPTH_W * DEPTH_H, raw, stride, image);
}

void fisheye_mask_depth(const struct FisheyeMaps *maps,
                        uint8_t *visual, uint16_t *depth_mm)
{
    for (int i = 0; i < DEPTH_W * DEPTH_H; ++i)
        if (maps->xy[0][i][0] == FISHEYE_INVALID ||
            maps->xy[1][i][0] == FISHEYE_INVALID)
            visual[i] = depth_mm[i] = 0;
}

void fisheye_build_validity(const struct FisheyeCalibration *calibration,
                            const struct FisheyeMaps *maps,
                            uint8_t valid[2][DEPTH_W * DEPTH_H])
{
    int rectified = calibration && calibration->valid && maps;
    for (int side = 0; side < 2; ++side) {
        double cx = rectified ? calibration->camera[side].cx : FISHEYE_RAW_W / 2.0;
        double cy = rectified ? calibration->camera[side].cy : FISHEYE_RAW_H / 2.0;
        /* Include all bilinear taps, or the full 16x16 raw downsample tile. */
        double radius = FISHEYE_USABLE_RADIUS - (rectified ? 2 : 12);
        for (unsigned y = 0; y < DEPTH_H; ++y)
            for (unsigned x = 0; x < DEPTH_W; ++x) {
                unsigned i = y * DEPTH_W + x;
                valid[side][i] = 0;
                double px = (x + 0.5) * FISHEYE_RAW_W / DEPTH_W;
                double py = (y + 0.5) * FISHEYE_RAW_H / DEPTH_H;
                if (rectified) {
                    const uint16_t *point = maps->xy[side][i];
                    if (point[0] == FISHEYE_INVALID)
                        continue;
                    px = FISHEYE_RAW_W - 1 - point[0] / 16.0;
                    py = point[1] / 16.0;
                }
                double dx = px - cx, dy = py - cy;
                if (dx * dx + dy * dy <= radius * radius)
                    valid[side][i] = 255;
            }
    }
}
