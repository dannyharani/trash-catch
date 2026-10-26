#pragma once

#include <stdint.h>
#include <stdbool.h>

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480

typedef struct {
    float h_min, h_max;
    float s_min, s_max;
    float v_min, v_max;
    uint32_t min_area;
} tHsvThreshold;

typedef struct {
    double x, y, z;
    uint16_t u, v;
    uint32_t area_px;
    bool valid;
} tObservation3D;

tObservation3D get_object_from_view(
    const uint8_t *rgb,
    const uint16_t *depth,
    tHsvThreshold thresh
);

void pixel_rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, float *h, float *s, float *v);
