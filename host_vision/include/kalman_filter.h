#pragma once

#include <stdbool.h>
#include "la.h"
#include "vision_pipeline.h"

#define GRAVITY_MPS2 9.81

typedef struct {
    tMatrix x; // [X, Y, Z, Vx, Vy, Vz]^T
    tMatrix P;
    tMatrix Q;
    tMatrix R;
    tMatrix H;
    double tilt_angle_deg;
    bool initialized;
} tKalmanFilter;

typedef struct {
    double x_impact;
    double y_impact;
    double z_impact;
    double t_impact;
    bool valid;
} tInterceptPred;

void kf_init(tKalmanFilter *kf, double sigma_proc, double sigma_meas);
void kf_set_noise(tKalmanFilter *kf, double sigma_proc, double sigma_meas);
void kf_reset(tKalmanFilter *kf);
void kf_predict(tKalmanFilter *kf, double dt);
void kf_next(tKalmanFilter *kf, tObservation3D obs);
tInterceptPred kf_pred_impact(const tKalmanFilter *kf, double floor_y, double tilt_deg);
