#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include "kalman_filter.h"
#include "vision_pipeline.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double rand_norm(void) {
    double u1 = 0.0;
    while (u1 <= 1e-15) {
        u1 = (double)rand() / (double)RAND_MAX;
    }
    double u2 = (double)rand() / (double)RAND_MAX;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

int main(void) {
    srand(42);

    const double x0 = -0.5, y0 = 0.2, z0 = 2.5;
    const double vx0 = 1.2, vy0 = 3.5, vz0 = -1.5;
    const double y_floor = -0.5;
    const double dt = 1.0 / 30.0;
    const double sigma_meas = 0.02;
    const double sigma_proc = 0.01;

    double g = GRAVITY_MPS2;
    double a = 0.5 * g;
    double b = -vy0;
    double c = y_floor - y0;

    double disc = b * b - 4.0 * a * c;
    assert(disc >= 0.0);

    double t_true = (-b + sqrt(disc)) / (2.0 * a);
    double x_true = x0 + vx0 * t_true;
    double z_true = z0 + vz0 * t_true;

    printf("Ground Truth: T=%.4fs | Impact=(%.4fm, %.4fm)\n", t_true, x_true, z_true);
    printf("%-6s | %-19s | %-21s | %-12s | %-6s\n",
           "Frame", "True (X, Z)", "Pred (X, Z)", "Err (cm)", "Valid");
    printf("--------------------------------------------------------------------------\n");

    tKalmanFilter kf;
    kf_init(&kf, sigma_proc, sigma_meas);

    bool ok = true;

    for (int step = 0; step <= 25; step++) {
        double t = step * dt;

        double tx = x0 + vx0 * t;
        double ty = y0 + vy0 * t - 0.5 * g * t * t;
        double tz = z0 + vz0 * t;

        tObservation3D obs = {
            .x = tx + rand_norm() * sigma_meas,
            .y = ty + rand_norm() * sigma_meas,
            .z = tz + rand_norm() * sigma_meas,
            .valid = true
        };

        if (step > 0) kf_predict(&kf, dt);
        kf_next(&kf, obs);

        tInterceptPred pred = kf_pred_impact(&kf, y_floor, 0.0);

        double err_cm = 0.0;
        if (pred.valid) {
            double dx = pred.x_impact - x_true;
            double dz = pred.z_impact - z_true;
            err_cm = hypot(dx, dz) * 100.0;
            printf("%-6d | (%6.3f, %6.3f)   | (%7.3f, %7.3f)     | %10.2f cm  | YES\n",
                   step, x_true, z_true, pred.x_impact, pred.z_impact, err_cm);
        } else {
            printf("%-6d | (%6.3f, %6.3f)   |          N/A        |        N/A   | NO\n",
                   step, x_true, z_true);
        }

        if (step >= 12 && (!pred.valid || err_cm > 5.0)) {
            ok = false;
        }
    }

    printf("RESULT: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
