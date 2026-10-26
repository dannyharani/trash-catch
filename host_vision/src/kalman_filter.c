#include "kalman_filter.h"
#include "la.h"
#include <assert.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void kf_init(tKalmanFilter *kf, double sigma_proc, double sigma_meas) {
    assert(kf != NULL);

    kf->initialized = false;
    kf->tilt_angle_deg = 0.0;
    kf->x = la_new(6, 1);
    kf->P = la_new(6, 6);

    kf->H = la_new(3, 6);
    la_set(&kf->H, 0, 0, 1.0);
    la_set(&kf->H, 1, 1, 1.0);
    la_set(&kf->H, 2, 2, 1.0);

    kf_set_noise(kf, sigma_proc, sigma_meas);
}

void kf_set_noise(tKalmanFilter *kf, double sigma_proc, double sigma_meas) {
    if (!kf) return;

    double q_var = sigma_proc * sigma_proc;
    kf->Q = la_new(6, 6);
    // Keep position Q low so frame-to-frame deltas feed into velocity estimates instead of position slack
    for (size_t i = 0; i < 3; i++) {
        la_set(&kf->Q, i, i, q_var * 0.01);
        la_set(&kf->Q, i + 3, i + 3, q_var);
    }

    double r_var = sigma_meas * sigma_meas;
    kf->R = la_new(3, 3);
    for (size_t i = 0; i < 3; i++) {
        la_set(&kf->R, i, i, r_var);
    }
}

void kf_reset(tKalmanFilter *kf) {
    if (!kf) return;
    kf->initialized = false;
    kf->tilt_angle_deg = 0.0;
    kf->x = la_new(6, 1);
    kf->P = la_new(6, 6);
}

void kf_predict(tKalmanFilter *kf, double dt) {
    if (!kf || !kf->initialized || dt <= 0.0) return;

    tMatrix F = la_identity(6);
    la_set(&F, 0, 3, dt);
    la_set(&F, 1, 4, dt);
    la_set(&F, 2, 5, dt);

    kf->x = la_mult(F, kf->x);

    double rad = kf->tilt_angle_deg * (M_PI / 180.0);
    double gy = GRAVITY_MPS2 * cos(rad);
    double gz = GRAVITY_MPS2 * sin(rad);

    *la_at(&kf->x, 1, 0) -= 0.5 * gy * dt * dt;
    *la_at(&kf->x, 2, 0) -= 0.5 * gz * dt * dt;
    *la_at(&kf->x, 4, 0) -= gy * dt;
    *la_at(&kf->x, 5, 0) -= gz * dt;

    tMatrix Ft = la_transpose(F);
    kf->P = la_sum(la_mult(la_mult(F, kf->P), Ft), kf->Q);
}

void kf_next(tKalmanFilter *kf, tObservation3D obs) {
    if (!kf || !obs.valid) return;

    if (!kf->initialized) {
        kf->x = la_new(6, 1);
        la_set(&kf->x, 0, 0, obs.x);
        la_set(&kf->x, 1, 0, obs.y);
        la_set(&kf->x, 2, 0, obs.z);

        kf->P = la_new(6, 6);
        for (size_t i = 0; i < 3; i++) {
            la_set(&kf->P, i, i, 0.01);
            la_set(&kf->P, i + 3, i + 3, 10.0);
        }

        kf->initialized = true;
        return;
    }

    tMatrix z = la_new(3, 1);
    la_set(&z, 0, 0, obs.x);
    la_set(&z, 1, 0, obs.y);
    la_set(&z, 2, 0, obs.z);

    tMatrix y = la_diff(z, la_mult(kf->H, kf->x));
    tMatrix Ht = la_transpose(kf->H);
    tMatrix S = la_sum(la_mult(la_mult(kf->H, kf->P), Ht), kf->R);

    tMatrix S_inv = la_inverse_3x3(S);
    if (S_inv.rows == 0) return;

    tMatrix K = la_mult(la_mult(kf->P, Ht), S_inv);
    kf->x = la_sum(kf->x, la_mult(K, y));

    tMatrix I = la_identity(6);
    kf->P = la_mult(la_diff(I, la_mult(K, kf->H)), kf->P);
}

tInterceptPred kf_pred_impact(const tKalmanFilter *kf, double floor_y, double tilt_deg) {
    tInterceptPred pred = {0};
    if (!kf || !kf->initialized) return pred;

    double x0  = la_get(&kf->x, 0, 0);
    double y0  = la_get(&kf->x, 1, 0);
    double z0  = la_get(&kf->x, 2, 0);
    double vx0 = la_get(&kf->x, 3, 0);
    double vy0 = la_get(&kf->x, 4, 0);
    double vz0 = la_get(&kf->x, 5, 0);

    double rad = tilt_deg * (M_PI / 180.0);
    double ny = cos(rad);
    double nz = sin(rad);

    double y_proj  = ny * y0 + nz * z0;
    double vy_proj = ny * vy0 + nz * vz0;

    double a = 0.5 * ny * GRAVITY_MPS2;
    double b = -vy_proj;
    double c = floor_y - y_proj;

    double disc = b * b - 4.0 * a * c;
    if (disc < 0.0 || fabs(a) < 1e-6) return pred;

    double sd = sqrt(disc);
    double t1 = (-b - sd) / (2.0 * a);
    double t2 = (-b + sd) / (2.0 * a);

    double t = (t2 > 0.0) ? t2 : ((t1 > 0.0) ? t1 : -1.0);
    if (t <= 0.0) return pred;

    pred.t_impact = t;
    pred.x_impact = x0 + vx0 * t;
    pred.z_impact = z0 + vz0 * t - 0.5 * (nz * GRAVITY_MPS2) * t * t;
    pred.y_impact = (fabs(ny) > 1e-6)
        ? ((floor_y - nz * pred.z_impact) / ny)
        : (y0 + vy0 * t - 0.5 * (ny * GRAVITY_MPS2) * t * t);
    pred.valid = true;

    return pred;
}
