#ifndef FOURREEL_IMAGE_H
#define FOURREEL_IMAGE_H

#include <math.h>
#include <stdint.h>

#define SENSOR_BLACK_LEVEL 16

static inline uint8_t image_display_luma(unsigned value)
{
    double signal = value > SENSOR_BLACK_LEVEL ?
        (double)(value - SENSOR_BLACK_LEVEL) / (255 - SENSOR_BLACK_LEVEL) : 0;
    return (uint8_t)(pow(signal, 0.45) * 255.0 + 0.5);
}

#endif
