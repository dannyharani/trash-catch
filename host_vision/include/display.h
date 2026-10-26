#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "vision_pipeline.h"
#include "kalman_filter.h"

typedef struct {
    tHsvThreshold thresh;
    double sigma_process;
    double sigma_meas;
    double target_floor;
    double tilt_angle_deg;
    int max_missed_frames;
    bool show_mask;
    bool kf_dirty;
    bool tilt_dirty;
    bool request_reset;
} tTuningConfig;

bool display_init(const char *title, int width, int height);
void display_destroy(void);
bool display_poll_events(tTuningConfig *cfg);
void display_render(
    uint8_t *rgb,
    tObservation3D obs,
    const tKalmanFilter *kf,
    tInterceptPred pred,
    tTuningConfig *cfg
);
