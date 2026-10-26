#include "display.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if __has_include(<SDL2/SDL.h>)
#include <SDL2/SDL.h>
#define HAVE_SDL2 1
#elif __has_include(<SDL.h>)
#include <SDL.h>
#define HAVE_SDL2 1
#else
#define HAVE_SDL2 0
#endif

#if HAVE_SDL2
static SDL_Window *s_win = NULL;
static SDL_Renderer *s_ren = NULL;
static SDL_Texture *s_tex = NULL;
static int s_w = 0;
static int s_h = 0;

static SDL_Window *s_ctrl_win = NULL;
static SDL_Renderer *s_ctrl_ren = NULL;
static uint32_t s_ctrl_id = 0;
static uint32_t s_main_id = 0;
static int s_drag_slider = -1;
static uint8_t s_raw_rgb[FRAME_WIDTH * FRAME_HEIGHT * 3];
static bool s_has_raw = false;

#define CTRL_W      500
#define CTRL_H      655
#define NUM_SLIDERS 12
#endif

static const float FX = 525.0f;
static const float FY = 525.0f;
static const float CX = 319.5f;
static const float CY = 239.5f;

static inline bool proj_3d_to_2d(double x, double y, double z, int *u, int *v) {
    if (z <= 0.05) return false;
    *u = (int)round(FX * (float)(x / z) + CX);
    *v = (int)round(-FY * (float)(y / z) + CY);
    return true;
}

static inline void put_px(uint8_t *rgb, int u, int v, uint8_t r, uint8_t g, uint8_t b) {
    if (u < 0 || u >= FRAME_WIDTH || v < 0 || v >= FRAME_HEIGHT) return;
    size_t idx = ((size_t)v * FRAME_WIDTH + (size_t)u) * 3;
    rgb[idx] = r;
    rgb[idx + 1] = g;
    rgb[idx + 2] = b;
}

static void draw_dot(uint8_t *rgb, int cx, int cy, int rad, uint8_t r, uint8_t g, uint8_t b) {
    for (int dy = -rad; dy <= rad; dy++) {
        for (int dx = -rad; dx <= rad; dx++) {
            if (dx * dx + dy * dy <= rad * rad) {
                put_px(rgb, cx + dx, cy + dy, r, g, b);
            }
        }
    }
}

static void draw_crosshair(uint8_t *rgb, int cx, int cy, int sz, uint8_t r, uint8_t g, uint8_t b) {
    for (int d = -sz; d <= sz; d++) {
        for (int t = -1; t <= 1; t++) {
            put_px(rgb, cx + d, cy + t, r, g, b);
            put_px(rgb, cx + t, cy + d, r, g, b);
        }
    }
}

static void draw_box(uint8_t *rgb, int cx, int cy, int hw, int hh, uint8_t r, uint8_t g, uint8_t b) {
    for (int x = cx - hw; x <= cx + hw; x++) {
        put_px(rgb, x, cy - hh, r, g, b);
        put_px(rgb, x, cy + hh, r, g, b);
    }
    for (int y = cy - hh; y <= cy + hh; y++) {
        put_px(rgb, cx - hw, y, r, g, b);
        put_px(rgb, cx + hw, y, r, g, b);
    }
}

static void draw_overlays(
    uint8_t *rgb,
    tObservation3D obs,
    const tKalmanFilter *kf,
    tInterceptPred pred,
    tTuningConfig *cfg
) {
    if (!rgb || !cfg) return;

    if (cfg->show_mask) {
        for (int v = 0; v < FRAME_HEIGHT; v++) {
            for (int u = 0; u < FRAME_WIDTH; u++) {
                size_t idx = ((size_t)v * FRAME_WIDTH + (size_t)u) * 3;
                float h, s, val;
                pixel_rgb_to_hsv(rgb[idx], rgb[idx + 1], rgb[idx + 2], &h, &s, &val);
                if (h >= cfg->thresh.h_min && h <= cfg->thresh.h_max &&
                    s >= cfg->thresh.s_min && s <= cfg->thresh.s_max &&
                    val >= cfg->thresh.v_min && val <= cfg->thresh.v_max) {
                    rgb[idx] = 255;
                    rgb[idx + 1] = 0;
                    rgb[idx + 2] = 255;
                } else {
                    rgb[idx] >>= 1;
                    rgb[idx + 1] >>= 1;
                    rgb[idx + 2] >>= 1;
                }
            }
        }
    }

    // Perspective floor grid adjusted for sensor tilt
    double rad = cfg->tilt_angle_deg * (M_PI / 180.0);
    double ny = cos(rad);
    double nz = sin(rad);

    if (fabs(ny) > 1e-4) {
        for (double z = 1.0; z <= 3.5; z += 0.5) {
            double yf = (cfg->target_floor - nz * z) / ny;
            for (double x = -1.2; x <= 1.2; x += 0.015) {
                int u, v;
                if (proj_3d_to_2d(x, yf, z, &u, &v)) {
                    put_px(rgb, u, v, 80, 160, 255);
                }
            }
        }

        for (double x = -1.2; x <= 1.21; x += 0.4) {
            bool center = (fabs(x) < 0.05);
            for (double z = 0.8; z <= 3.5; z += 0.02) {
                double yf = (cfg->target_floor - nz * z) / ny;
                int u, v;
                if (proj_3d_to_2d(x, yf, z, &u, &v)) {
                    if (center) put_px(rgb, u, v, 255, 220, 60);
                    else put_px(rgb, u, v, 80, 160, 255);
                }
            }
        }
    }

    if (obs.valid) {
        draw_box(rgb, obs.u, obs.v, 4, 4, 255, 200, 0);
        draw_crosshair(rgb, obs.u, obs.v, 10, 255, 0, 0);
    }

    if (kf && kf->initialized && pred.valid && pred.t_impact > 0.0) {
        double x0  = la_get(&kf->x, 0, 0);
        double y0  = la_get(&kf->x, 1, 0);
        double z0  = la_get(&kf->x, 2, 0);
        double vx0 = la_get(&kf->x, 3, 0);
        double vy0 = la_get(&kf->x, 4, 0);
        double vz0 = la_get(&kf->x, 5, 0);

        double gy = GRAVITY_MPS2 * ny;
        double gz = GRAVITY_MPS2 * nz;

        for (double t = 0.0; t <= pred.t_impact; t += 0.02) {
            double xt = x0 + vx0 * t;
            double yt = y0 + vy0 * t - 0.5 * gy * t * t;
            double zt = z0 + vz0 * t - 0.5 * gz * t * t;

            int u, v;
            if (proj_3d_to_2d(xt, yt, zt, &u, &v)) {
                draw_dot(rgb, u, v, 2, 0, 255, 0);
            }
        }
    }

    if (pred.valid) {
        int ui, vi;
        if (proj_3d_to_2d(pred.x_impact, pred.y_impact, pred.z_impact, &ui, &vi)) {
            draw_box(rgb, ui, vi, 8, 8, 0, 255, 255);
            draw_crosshair(rgb, ui, vi, 12, 0, 255, 255);
        }
    }

    // Top-down (X, Z) mini-radar in top-right corner
    const int rw = 136, rh = 136;
    const int rx0 = FRAME_WIDTH - rw - 10;
    const int ry0 = 10;
    const double z_max = 3.8;
    const double x_max = 1.4;

    for (int py = ry0; py < ry0 + rh; py++) {
        for (int px = rx0; px < rx0 + rw; px++) {
            size_t idx = ((size_t)py * FRAME_WIDTH + (size_t)px) * 3;
            rgb[idx] >>= 2;
            rgb[idx + 1] >>= 2;
            rgb[idx + 2] = (uint8_t)((rgb[idx + 2] >> 2) + 20);
        }
    }
    draw_box(rgb, rx0 + rw / 2, ry0 + rh / 2, rw / 2, rh / 2, 80, 160, 255);

    for (int zm = 1; zm <= 3; zm++) {
        int ry = ry0 + rh - (int)round((zm / z_max) * rh);
        for (int px = rx0 + 1; px < rx0 + rw - 1; px += 2) {
            put_px(rgb, px, ry, 50, 90, 140);
        }
    }
    int r_cx = rx0 + rw / 2;
    for (int py = ry0 + 1; py < ry0 + rh - 1; py += 2) {
        put_px(rgb, r_cx, py, 120, 110, 40);
    }

    if (obs.valid) {
        int ox = r_cx + (int)round((obs.x / x_max) * (rw * 0.5));
        int oy = ry0 + rh - (int)round((obs.z / z_max) * rh);
        if (ox >= rx0 + 2 && ox < rx0 + rw - 2 && oy >= ry0 + 2 && oy < ry0 + rh - 2) {
            draw_dot(rgb, ox, oy, 3, 255, 50, 50);
        }

        if (pred.valid) {
            int ix = r_cx + (int)round((pred.x_impact / x_max) * (rw * 0.5));
            int iy = ry0 + rh - (int)round((pred.z_impact / z_max) * rh);
            for (int s = 0; s <= 24; s++) {
                double a = s / 24.0;
                int lx = (int)round(ox + a * (ix - ox));
                int ly = (int)round(oy + a * (iy - oy));
                if (lx > rx0 && lx < rx0 + rw - 1 && ly > ry0 && ly < ry0 + rh - 1) {
                    put_px(rgb, lx, ly, 0, 255, 0);
                }
            }
            if (ix >= rx0 + 3 && ix < rx0 + rw - 3 && iy >= ry0 + 3 && iy < ry0 + rh - 3) {
                draw_box(rgb, ix, iy, 3, 3, 0, 255, 255);
            }
        }
    }
}

#if HAVE_SDL2
static void glyph_5x7(char c, uint8_t rows[7]) {
    memset(rows, 0, 7);
    if (c >= 'a' && c <= 'z') c -= 32;
    switch (c) {
        case 'A': rows[0]=0x0E; rows[1]=0x11; rows[2]=0x11; rows[3]=0x1F; rows[4]=0x11; rows[5]=0x11; rows[6]=0x11; break;
        case 'B': rows[0]=0x1E; rows[1]=0x11; rows[2]=0x11; rows[3]=0x1E; rows[4]=0x11; rows[5]=0x11; rows[6]=0x1E; break;
        case 'C': rows[0]=0x0E; rows[1]=0x11; rows[2]=0x10; rows[3]=0x10; rows[4]=0x10; rows[5]=0x11; rows[6]=0x0E; break;
        case 'D': rows[0]=0x1C; rows[1]=0x12; rows[2]=0x11; rows[3]=0x11; rows[4]=0x11; rows[5]=0x12; rows[6]=0x1C; break;
        case 'E': rows[0]=0x1F; rows[1]=0x10; rows[2]=0x10; rows[3]=0x1E; rows[4]=0x10; rows[5]=0x10; rows[6]=0x1F; break;
        case 'F': rows[0]=0x1F; rows[1]=0x10; rows[2]=0x10; rows[3]=0x1E; rows[4]=0x10; rows[5]=0x10; rows[6]=0x10; break;
        case 'G': rows[0]=0x0E; rows[1]=0x11; rows[2]=0x10; rows[3]=0x17; rows[4]=0x11; rows[5]=0x11; rows[6]=0x0F; break;
        case 'H': rows[0]=0x11; rows[1]=0x11; rows[2]=0x11; rows[3]=0x1F; rows[4]=0x11; rows[5]=0x11; rows[6]=0x11; break;
        case 'I': rows[0]=0x0E; rows[1]=0x04; rows[2]=0x04; rows[3]=0x04; rows[4]=0x04; rows[5]=0x04; rows[6]=0x0E; break;
        case 'J': rows[0]=0x07; rows[1]=0x02; rows[2]=0x02; rows[3]=0x02; rows[4]=0x02; rows[5]=0x12; rows[6]=0x0C; break;
        case 'K': rows[0]=0x11; rows[1]=0x12; rows[2]=0x14; rows[3]=0x18; rows[4]=0x14; rows[5]=0x12; rows[6]=0x11; break;
        case 'L': rows[0]=0x10; rows[1]=0x10; rows[2]=0x10; rows[3]=0x10; rows[4]=0x10; rows[5]=0x10; rows[6]=0x1F; break;
        case 'M': rows[0]=0x11; rows[1]=0x1B; rows[2]=0x15; rows[3]=0x15; rows[4]=0x11; rows[5]=0x11; rows[6]=0x11; break;
        case 'N': rows[0]=0x11; rows[1]=0x19; rows[2]=0x15; rows[3]=0x13; rows[4]=0x11; rows[5]=0x11; rows[6]=0x11; break;
        case 'O': rows[0]=0x0E; rows[1]=0x11; rows[2]=0x11; rows[3]=0x11; rows[4]=0x11; rows[5]=0x11; rows[6]=0x0E; break;
        case 'P': rows[0]=0x1E; rows[1]=0x11; rows[2]=0x11; rows[3]=0x1E; rows[4]=0x10; rows[5]=0x10; rows[6]=0x10; break;
        case 'Q': rows[0]=0x0E; rows[1]=0x11; rows[2]=0x11; rows[3]=0x11; rows[4]=0x15; rows[5]=0x12; rows[6]=0x0D; break;
        case 'R': rows[0]=0x1E; rows[1]=0x11; rows[2]=0x11; rows[3]=0x1E; rows[4]=0x14; rows[5]=0x12; rows[6]=0x11; break;
        case 'S': rows[0]=0x0F; rows[1]=0x10; rows[2]=0x10; rows[3]=0x0E; rows[4]=0x01; rows[5]=0x01; rows[6]=0x1E; break;
        case 'T': rows[0]=0x1F; rows[1]=0x04; rows[2]=0x04; rows[3]=0x04; rows[4]=0x04; rows[5]=0x04; rows[6]=0x04; break;
        case 'U': rows[0]=0x11; rows[1]=0x11; rows[2]=0x11; rows[3]=0x11; rows[4]=0x11; rows[5]=0x11; rows[6]=0x0E; break;
        case 'V': rows[0]=0x11; rows[1]=0x11; rows[2]=0x11; rows[3]=0x11; rows[4]=0x11; rows[5]=0x0A; rows[6]=0x04; break;
        case 'W': rows[0]=0x11; rows[1]=0x11; rows[2]=0x11; rows[3]=0x15; rows[4]=0x15; rows[5]=0x15; rows[6]=0x0A; break;
        case 'X': rows[0]=0x11; rows[1]=0x11; rows[2]=0x0A; rows[3]=0x04; rows[4]=0x0A; rows[5]=0x11; rows[6]=0x11; break;
        case 'Y': rows[0]=0x11; rows[1]=0x11; rows[2]=0x0A; rows[3]=0x04; rows[4]=0x04; rows[5]=0x04; rows[6]=0x04; break;
        case 'Z': rows[0]=0x1F; rows[1]=0x01; rows[2]=0x02; rows[3]=0x04; rows[4]=0x08; rows[5]=0x10; rows[6]=0x1F; break;
        case '0': rows[0]=0x0E; rows[1]=0x11; rows[2]=0x13; rows[3]=0x15; rows[4]=0x19; rows[5]=0x11; rows[6]=0x0E; break;
        case '1': rows[0]=0x04; rows[1]=0x0C; rows[2]=0x04; rows[3]=0x04; rows[4]=0x04; rows[5]=0x04; rows[6]=0x0E; break;
        case '2': rows[0]=0x0E; rows[1]=0x11; rows[2]=0x01; rows[3]=0x02; rows[4]=0x04; rows[5]=0x08; rows[6]=0x1F; break;
        case '3': rows[0]=0x1F; rows[1]=0x02; rows[2]=0x04; rows[3]=0x02; rows[4]=0x01; rows[5]=0x11; rows[6]=0x0E; break;
        case '4': rows[0]=0x02; rows[1]=0x06; rows[2]=0x0A; rows[3]=0x12; rows[4]=0x1F; rows[5]=0x02; rows[6]=0x02; break;
        case '5': rows[0]=0x1F; rows[1]=0x10; rows[2]=0x1E; rows[3]=0x01; rows[4]=0x01; rows[5]=0x11; rows[6]=0x0E; break;
        case '6': rows[0]=0x06; rows[1]=0x08; rows[2]=0x10; rows[3]=0x1E; rows[4]=0x11; rows[5]=0x11; rows[6]=0x0E; break;
        case '7': rows[0]=0x1F; rows[1]=0x01; rows[2]=0x02; rows[3]=0x04; rows[4]=0x08; rows[5]=0x08; rows[6]=0x08; break;
        case '8': rows[0]=0x0E; rows[1]=0x11; rows[2]=0x11; rows[3]=0x0E; rows[4]=0x11; rows[5]=0x11; rows[6]=0x0E; break;
        case '9': rows[0]=0x0E; rows[1]=0x11; rows[2]=0x11; rows[3]=0x0F; rows[4]=0x01; rows[5]=0x02; rows[6]=0x0C; break;
        case '.': rows[5]=0x0C; rows[6]=0x0C; break;
        case '-': rows[3]=0x1F; break;
        case '+': rows[1]=0x04; rows[2]=0x04; rows[3]=0x1F; rows[4]=0x04; rows[5]=0x04; break;
        case ':': rows[1]=0x0C; rows[2]=0x0C; rows[4]=0x0C; rows[5]=0x0C; break;
        case '/': rows[0]=0x01; rows[1]=0x01; rows[2]=0x02; rows[3]=0x04; rows[4]=0x08; rows[5]=0x10; rows[6]=0x10; break;
        case '(': rows[0]=0x02; rows[1]=0x04; rows[2]=0x08; rows[3]=0x08; rows[4]=0x08; rows[5]=0x04; rows[6]=0x02; break;
        case ')': rows[0]=0x08; rows[1]=0x04; rows[2]=0x02; rows[3]=0x02; rows[4]=0x02; rows[5]=0x04; rows[6]=0x08; break;
        case '_': rows[6]=0x1F; break;
        case '[': rows[0]=0x0E; rows[1]=0x08; rows[2]=0x08; rows[3]=0x08; rows[4]=0x08; rows[5]=0x08; rows[6]=0x0E; break;
        case ']': rows[0]=0x0E; rows[1]=0x02; rows[2]=0x02; rows[3]=0x02; rows[4]=0x02; rows[5]=0x02; rows[6]=0x0E; break;
        default: break;
    }
}

static void draw_str(SDL_Renderer *ren, int x, int y, int scale, const char *s, uint8_t r, uint8_t g, uint8_t b) {
    SDL_SetRenderDrawColor(ren, r, g, b, 255);
    for (int cx = x; *s; s++, cx += 6 * scale) {
        uint8_t rows[7];
        glyph_5x7(*s, rows);
        for (int row = 0; row < 7; row++) {
            for (int col = 0; col < 5; col++) {
                if (rows[row] & (1 << (4 - col))) {
                    SDL_Rect rc = { cx + col * scale, y + row * scale, scale, scale };
                    SDL_RenderFillRect(ren, &rc);
                }
            }
        }
    }
}

typedef struct {
    const char *label;
    double min_v;
    double max_v;
    const char *fmt;
} tSliderSpec;

static const tSliderSpec SLIDERS[NUM_SLIDERS] = {
    { "HUE MIN",        0.0,   360.0, "%.1f DEG" },
    { "HUE MAX",        0.0,   360.0, "%.1f DEG" },
    { "SAT MIN",        0.0,   1.0,   "%.2f"     },
    { "SAT MAX",        0.0,   1.0,   "%.2f"     },
    { "VAL MIN",        0.0,   1.0,   "%.2f"     },
    { "VAL MAX",        0.0,   1.0,   "%.2f"     },
    { "MIN AREA PX",    5.0,   500.0, "%.0f PX"  },
    { "SIGMA PROCESS",  0.001, 1.0,   "%.3f"     },
    { "SIGMA MEAS",     0.002, 0.20,  "%.3f M"   },
    { "TARGET Y PLANE", -2.5,  0.5,   "%+.2f M"  },
    { "TILT ANGLE",     -27.0, 27.0,  "%+.0f DEG"},
    { "MAX MISSED FRM", 1.0,   60.0,  "%.0f FRM" }
};

static double get_slider(const tTuningConfig *cfg, int idx) {
    switch (idx) {
        case 0:  return cfg->thresh.h_min;
        case 1:  return cfg->thresh.h_max;
        case 2:  return cfg->thresh.s_min;
        case 3:  return cfg->thresh.s_max;
        case 4:  return cfg->thresh.v_min;
        case 5:  return cfg->thresh.v_max;
        case 6:  return (double)(cfg->thresh.min_area ? cfg->thresh.min_area : 50);
        case 7:  return cfg->sigma_process;
        case 8:  return cfg->sigma_meas;
        case 9:  return cfg->target_floor;
        case 10: return cfg->tilt_angle_deg;
        case 11: return (double)cfg->max_missed_frames;
        default: return 0.0;
    }
}

static void set_slider(tTuningConfig *cfg, int idx, double v) {
    if (v < SLIDERS[idx].min_v) v = SLIDERS[idx].min_v;
    if (v > SLIDERS[idx].max_v) v = SLIDERS[idx].max_v;

    switch (idx) {
        case 0:  cfg->thresh.h_min = (float)v; break;
        case 1:  cfg->thresh.h_max = (float)v; break;
        case 2:  cfg->thresh.s_min = (float)v; break;
        case 3:  cfg->thresh.s_max = (float)v; break;
        case 4:  cfg->thresh.v_min = (float)v; break;
        case 5:  cfg->thresh.v_max = (float)v; break;
        case 6:  cfg->thresh.min_area = (uint32_t)round(v); break;
        case 7:  cfg->sigma_process = v; cfg->kf_dirty = true; break;
        case 8:  cfg->sigma_meas = v;    cfg->kf_dirty = true; break;
        case 9:  cfg->target_floor = v; break;
        case 10: cfg->tilt_angle_deg = v; break;
        case 11: cfg->max_missed_frames = (int)round(v); break;
    }
}

static void update_slider_mouse(tTuningConfig *cfg, int idx, int mx) {
    double t = (mx - 180.0) / 200.0;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    set_slider(cfg, idx, SLIDERS[idx].min_v + t * (SLIDERS[idx].max_v - SLIDERS[idx].min_v));
}

static void render_ctrl_panel(const tTuningConfig *cfg, tObservation3D obs, const tKalmanFilter *kf, tInterceptPred pred) {
    if (!s_ctrl_ren || !cfg) return;

    SDL_SetRenderDrawColor(s_ctrl_ren, 28, 30, 36, 255);
    SDL_RenderClear(s_ctrl_ren);

    SDL_Rect hdr = { 0, 0, CTRL_W, 38 };
    SDL_SetRenderDrawColor(s_ctrl_ren, 42, 46, 56, 255);
    SDL_RenderFillRect(s_ctrl_ren, &hdr);
    draw_str(s_ctrl_ren, 16, 12, 2, "TUNING CONTROL PANEL", 80, 220, 255);

    const int y0 = 52, row_h = 38, bx = 180, bw = 200, bh = 14;

    for (int i = 0; i < NUM_SLIDERS; i++) {
        int y = y0 + i * row_h;
        draw_str(s_ctrl_ren, 16, y + 2, 2, SLIDERS[i].label, 220, 225, 235);

        SDL_Rect trk = { bx, y, bw, bh };
        SDL_SetRenderDrawColor(s_ctrl_ren, 55, 60, 72, 255);
        SDL_RenderFillRect(s_ctrl_ren, &trk);

        double val = get_slider(cfg, i);
        double norm = (val - SLIDERS[i].min_v) / (SLIDERS[i].max_v - SLIDERS[i].min_v);
        if (norm < 0.0) norm = 0.0;
        if (norm > 1.0) norm = 1.0;

        int fw = (int)round(norm * bw);
        SDL_Rect fill = { bx, y, fw, bh };
        if (i <= 5) SDL_SetRenderDrawColor(s_ctrl_ren, 240, 190, 50, 255);
        else if (i == 6) SDL_SetRenderDrawColor(s_ctrl_ren, 180, 110, 255, 255);
        else SDL_SetRenderDrawColor(s_ctrl_ren, 60, 195, 255, 255);
        SDL_RenderFillRect(s_ctrl_ren, &fill);

        SDL_Rect knob = { bx + fw - 5, y - 3, 10, bh + 6 };
        SDL_SetRenderDrawColor(s_ctrl_ren, 255, 255, 255, 255);
        SDL_RenderFillRect(s_ctrl_ren, &knob);

        char buf[32];
        snprintf(buf, sizeof(buf), SLIDERS[i].fmt, val);
        draw_str(s_ctrl_ren, bx + bw + 14, y + 2, 2, buf, 255, 255, 255);
    }

    int by = y0 + NUM_SLIDERS * row_h + 10;

    SDL_Rect mask_btn = { 16, by, 215, 34 };
    if (cfg->show_mask) SDL_SetRenderDrawColor(s_ctrl_ren, 200, 50, 180, 255);
    else SDL_SetRenderDrawColor(s_ctrl_ren, 65, 72, 88, 255);
    SDL_RenderFillRect(s_ctrl_ren, &mask_btn);
    draw_str(s_ctrl_ren, 28, by + 10, 2, cfg->show_mask ? "MASK VIEW: ON" : "MASK VIEW: OFF", 255, 255, 255);

    SDL_Rect rst_btn = { 255, by, 220, 34 };
    SDL_SetRenderDrawColor(s_ctrl_ren, 190, 65, 65, 255);
    SDL_RenderFillRect(s_ctrl_ren, &rst_btn);
    draw_str(s_ctrl_ren, 285, by + 10, 2, "RESET KALMAN", 255, 255, 255);

    int sy = by + 46;
    char line1[64], line2[64];
    snprintf(line1, sizeof(line1), "OBS: %s  AREA: %u PX  KF: %s",
             obs.valid ? "LOCKED" : "NONE", obs.area_px,
             (kf && kf->initialized) ? "ACTIVE" : "IDLE");
    draw_str(s_ctrl_ren, 16, sy, 2, line1, 140, 235, 160);

    if (obs.valid && kf && kf->initialized) {
        double vz = la_get(&kf->x, 5, 0);
        if (pred.valid) {
            snprintf(line2, sizeof(line2), "Z:%.2fM VZ:%+.2f Z_IMP:%.2fM", obs.z, vz, pred.z_impact);
        } else {
            snprintf(line2, sizeof(line2), "POS X:%+.2f Y:%+.2f Z:%.2fM", obs.x, obs.y, obs.z);
        }
    } else {
        snprintf(line2, sizeof(line2), "TIP: CLICK VIDEO TO PICK HSV");
    }
    draw_str(s_ctrl_ren, 16, sy + 22, 2, line2, 80, 220, 255);

    SDL_RenderPresent(s_ctrl_ren);
}
#endif

bool display_init(const char *title, int width, int height) {
#if HAVE_SDL2
    if (SDL_Init(SDL_INIT_VIDEO) < 0) return false;

    s_win = SDL_CreateWindow(
        title ? title : "Trash Catch",
        100, 100, width, height, SDL_WINDOW_SHOWN
    );
    if (!s_win) {
        SDL_Quit();
        return false;
    }
    s_main_id = SDL_GetWindowID(s_win);

    s_ren = SDL_CreateRenderer(s_win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!s_ren) s_ren = SDL_CreateRenderer(s_win, -1, 0);
    if (!s_ren) {
        SDL_DestroyWindow(s_win);
        s_win = NULL;
        SDL_Quit();
        return false;
    }

    s_tex = SDL_CreateTexture(s_ren, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, width, height);
    if (!s_tex) {
        SDL_DestroyRenderer(s_ren);
        SDL_DestroyWindow(s_win);
        s_ren = NULL;
        s_win = NULL;
        SDL_Quit();
        return false;
    }

    s_ctrl_win = SDL_CreateWindow(
        "Parameter Tuning & Controls",
        100 + width + 20, 100, CTRL_W, CTRL_H, SDL_WINDOW_SHOWN
    );
    if (s_ctrl_win) {
        s_ctrl_id = SDL_GetWindowID(s_ctrl_win);
        s_ctrl_ren = SDL_CreateRenderer(s_ctrl_win, -1, SDL_RENDERER_ACCELERATED);
        if (!s_ctrl_ren) s_ctrl_ren = SDL_CreateRenderer(s_ctrl_win, -1, 0);
    }

    s_w = width;
    s_h = height;
    return true;
#else
    (void)title; (void)width; (void)height;
    return true;
#endif
}

void display_destroy(void) {
#if HAVE_SDL2
    if (s_ctrl_ren) { SDL_DestroyRenderer(s_ctrl_ren); s_ctrl_ren = NULL; }
    if (s_ctrl_win) { SDL_DestroyWindow(s_ctrl_win);   s_ctrl_win = NULL; }
    if (s_tex)      { SDL_DestroyTexture(s_tex);       s_tex = NULL; }
    if (s_ren)      { SDL_DestroyRenderer(s_ren);      s_ren = NULL; }
    if (s_win)      { SDL_DestroyWindow(s_win);        s_win = NULL; }
    SDL_Quit();
#endif
}

bool display_poll_events(tTuningConfig *cfg) {
#if HAVE_SDL2
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_QUIT) return false;
        if (ev.type == SDL_WINDOWEVENT && ev.window.event == SDL_WINDOWEVENT_CLOSE) return false;
        if (ev.type == SDL_KEYDOWN && (ev.key.keysym.sym == SDLK_ESCAPE || ev.key.keysym.sym == SDLK_q)) {
            return false;
        }

        if (!cfg) continue;

        if (ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT) {
            if (ev.button.windowID == s_ctrl_id) {
                int mx = ev.button.x, my = ev.button.y;
                const int y0 = 52, row_h = 38;

                for (int i = 0; i < NUM_SLIDERS; i++) {
                    int sy = y0 + i * row_h;
                    if (my >= sy - 6 && my <= sy + 20 && mx >= 170 && mx <= 395) {
                        s_drag_slider = i;
                        update_slider_mouse(cfg, i, mx);
                        break;
                    }
                }

                int by = y0 + NUM_SLIDERS * row_h + 10;
                if (my >= by && my <= by + 34) {
                    if (mx >= 16 && mx <= 231) cfg->show_mask = !cfg->show_mask;
                    else if (mx >= 255 && mx <= 475) cfg->request_reset = true;
                }
            } else if (ev.button.windowID == s_main_id && s_has_raw) {
                int u = ev.button.x, v = ev.button.y;
                if (u >= 0 && u < FRAME_WIDTH && v >= 0 && v < FRAME_HEIGHT) {
                    size_t idx = ((size_t)v * FRAME_WIDTH + (size_t)u) * 3;
                    float h, s, val;
                    pixel_rgb_to_hsv(s_raw_rgb[idx], s_raw_rgb[idx + 1], s_raw_rgb[idx + 2], &h, &s, &val);

                    cfg->thresh.h_min = fmaxf(0.0f, h - 20.0f);
                    cfg->thresh.h_max = fminf(360.0f, h + 20.0f);
                    cfg->thresh.s_min = fmaxf(0.1f, s - 0.25f);
                    cfg->thresh.s_max = 1.0f;
                    cfg->thresh.v_min = fmaxf(0.1f, val - 0.25f);
                    cfg->thresh.v_max = 1.0f;
                    printf("[EYEDROPPER] (%d, %d) -> HSV(%.1f, %.2f, %.2f)\n", u, v, h, s, val);
                }
            }
        } else if (ev.type == SDL_MOUSEBUTTONUP && ev.button.button == SDL_BUTTON_LEFT) {
            if (s_drag_slider == 10) cfg->tilt_dirty = true;
            s_drag_slider = -1;
        } else if (ev.type == SDL_MOUSEMOTION && s_drag_slider >= 0) {
            if (ev.motion.windowID == s_ctrl_id) {
                update_slider_mouse(cfg, s_drag_slider, ev.motion.x);
            }
        }
    }
    return true;
#else
    (void)cfg;
    return true;
#endif
}

void display_render(
    uint8_t *rgb,
    tObservation3D obs,
    const tKalmanFilter *kf,
    tInterceptPred pred,
    tTuningConfig *cfg
) {
    if (!rgb) return;

#if HAVE_SDL2
    memcpy(s_raw_rgb, rgb, sizeof(s_raw_rgb));
    s_has_raw = true;
#endif

    draw_overlays(rgb, obs, kf, pred, cfg);

#if HAVE_SDL2
    if (s_ren && s_tex) {
        SDL_UpdateTexture(s_tex, NULL, rgb, s_w * 3);
        SDL_RenderClear(s_ren);
        SDL_RenderCopy(s_ren, s_tex, NULL, NULL);
        SDL_RenderPresent(s_ren);
    }

    render_ctrl_panel(cfg, obs, kf, pred);
#endif
}
