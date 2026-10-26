#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "vision_pipeline.h"

#define TILE_SZ 16
#define GRID_W  (FRAME_WIDTH / TILE_SZ)
#define GRID_H  (FRAME_HEIGHT / TILE_SZ)

void pixel_rgb_to_hsv(uint8_t r_in, uint8_t g_in, uint8_t b_in, float *h, float *s, float *v) {
    float r = r_in / 255.0f;
    float g = g_in / 255.0f;
    float b = b_in / 255.0f;

    float c_max = fmaxf(fmaxf(r, g), b);
    float c_min = fminf(fminf(r, g), b);
    float d = c_max - c_min;

    *v = c_max;
    *s = (c_max == 0.0f) ? 0.0f : (d / c_max);

    if (d == 0.0f) {
        *h = 0.0f;
    } else if (c_max == r) {
        *h = 60.0f * ((g - b) / d);
    } else if (c_max == g) {
        *h = 60.0f * (((b - r) / d) + 2.0f);
    } else {
        *h = 60.0f * (((r - g) / d) + 4.0f);
    }

    if (*h < 0.0f) *h += 360.0f;
}

static inline bool match_hsv(const uint8_t *rgb, size_t idx, tHsvThreshold t) {
    size_t i = idx * 3;
    float h, s, v;
    pixel_rgb_to_hsv(rgb[i], rgb[i + 1], rgb[i + 2], &h, &s, &v);
    return (h >= t.h_min && h <= t.h_max &&
            s >= t.s_min && s <= t.s_max &&
            v >= t.v_min && v <= t.v_max);
}

tObservation3D get_object_from_view(const uint8_t *rgb, const uint16_t *depth, tHsvThreshold thresh) {
    // Bin matches into 16x16 tiles first so random background color hits don't skew the centroid
    uint16_t bins[GRID_H][GRID_W] = {0};
    uint32_t total = 0;

    for (uint32_t v = 0; v < FRAME_HEIGHT; v++) {
        uint32_t ty = v / TILE_SZ;
        for (uint32_t u = 0; u < FRAME_WIDTH; u++) {
            if (match_hsv(rgb, (size_t)v * FRAME_WIDTH + u, thresh)) {
                bins[ty][u / TILE_SZ]++;
                total++;
            }
        }
    }

    tObservation3D obs = {0};
    uint32_t min_area = thresh.min_area ? thresh.min_area : 50;
    if (total < min_area) return obs;

    uint32_t best_sum = 0;
    int best_tx = 0, best_ty = 0;

    for (int ty = 0; ty < GRID_H; ty++) {
        for (int tx = 0; tx < GRID_W; tx++) {
            uint32_t sum = 0;
            for (int dy = -1; dy <= 1; dy++) {
                int ny = ty + dy;
                if (ny < 0 || ny >= GRID_H) continue;
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = tx + dx;
                    if (nx >= 0 && nx < GRID_W) sum += bins[ny][nx];
                }
            }
            if (sum > best_sum) {
                best_sum = sum;
                best_tx = tx;
                best_ty = ty;
            }
        }
    }

    // Moment calculation restricted to local ROI (+/- 5 tiles around peak)
    int u0 = (best_tx - 5) * TILE_SZ;
    int u1 = (best_tx + 6) * TILE_SZ - 1;
    int v0 = (best_ty - 5) * TILE_SZ;
    int v1 = (best_ty + 6) * TILE_SZ - 1;

    if (u0 < 0) u0 = 0;
    if (u1 >= FRAME_WIDTH) u1 = FRAME_WIDTH - 1;
    if (v0 < 0) v0 = 0;
    if (v1 >= FRAME_HEIGHT) v1 = FRAME_HEIGHT - 1;

    uint32_t m00 = 0;
    uint64_t m10 = 0, m01 = 0;

    for (int v = v0; v <= v1; v++) {
        for (int u = u0; u <= u1; u++) {
            if (match_hsv(rgb, (size_t)v * FRAME_WIDTH + u, thresh)) {
                m00++;
                m10 += u;
                m01 += v;
            }
        }
    }

    if (m00 < min_area) return obs;

    obs.u = (uint16_t)(m10 / m00);
    obs.v = (uint16_t)(m01 / m00);
    obs.area_px = m00;

    // Sample 9x9 window around centroid; clamp to closest surface + 15cm to avoid averaging background wall
    const int rad = 4;
    int vc = obs.v, uc = obs.u;

    uint16_t samples[81];
    uint32_t n_samples = 0;
    uint16_t min_d = 0xFFFF;

    for (int pass = 0; pass < 2 && n_samples == 0; pass++) {
        for (int dv = -rad; dv <= rad; dv++) {
            int vs = vc + dv;
            if (vs < 0 || vs >= FRAME_HEIGHT) continue;

            for (int du = -rad; du <= rad; du++) {
                int us = uc + du;
                if (us < 0 || us >= FRAME_WIDTH) continue;

                size_t idx = (size_t)vs * FRAME_WIDTH + us;
                uint16_t d = depth[idx];
                if (d == 0 || d > 6000) continue;

                // First pass only takes HSV-matched pixels; second pass falls back to any valid depth in patch
                if (pass == 0 && !match_hsv(rgb, idx, thresh)) continue;

                samples[n_samples++] = d;
                if (d < min_d) min_d = d;
            }
        }
    }

    if (n_samples == 0) return obs;

    uint64_t d_sum = 0;
    uint32_t d_cnt = 0;
    for (uint32_t i = 0; i < n_samples; i++) {
        if (samples[i] <= min_d + 150) {
            d_sum += samples[i];
            d_cnt++;
        }
    }

    float z_m = ((float)d_sum / (float)d_cnt) * 0.001f;

    // Kinect v1 pinhole intrinsics
    const float fx = 525.0f, fy = 525.0f;
    const float cx = 319.5f, cy = 239.5f;

    obs.x = (obs.u - cx) * z_m / fx;
    obs.y = -((obs.v - cy) * z_m / fy);
    obs.z = z_m;
    obs.valid = true;

    return obs;
}
