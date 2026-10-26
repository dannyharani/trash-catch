#include <stdio.h>
#include <windows.h>

#include "display.h"
#include "kalman_filter.h"
#include "kinect_capture.h"
#include "vision_pipeline.h"

int main(void) {
    printf("Initializing Kinect...\n");
    if (!kinect_start()) {
        fprintf(stderr, "Failed to start Kinect\n");
        return 1;
    }

    tTuningConfig cfg = {
        .thresh = {
            .h_min = 35.0f, .h_max = 75.0f,
            .s_min = 0.35f, .s_max = 1.0f,
            .v_min = 0.35f, .v_max = 1.0f,
            .min_area = 50
        },
        .sigma_process = 0.1,
        .sigma_meas = 0.02,
        .target_floor = -0.8,
        .max_missed_frames = 10
    };

    tKalmanFilter kf;
    kf_init(&kf, cfg.sigma_process, cfg.sigma_meas);

    tKinectFrame frame;
    double prev_t = 0.0;
    int missed = 0;

    display_init("Projectile Tracker", FRAME_WIDTH, FRAME_HEIGHT);

    printf("[KINECT] Calibrating tilt...\n");
    kinect_set_tilt_angle(KINECT_TILT_MAX_DEG);
    Sleep(800);
    kinect_set_tilt_angle(KINECT_TILT_MIN_DEG);
    Sleep(800);
    kinect_set_tilt_angle(0.0);
    Sleep(500);

    if (kinect_get_tilt_angle(&cfg.tilt_angle_deg)) {
        printf("[KINECT] Tilt angle: %+.1f deg\n", cfg.tilt_angle_deg);
    }

    while (display_poll_events(&cfg)) {
        if (cfg.kf_dirty) {
            kf_set_noise(&kf, cfg.sigma_process, cfg.sigma_meas);
            cfg.kf_dirty = false;
        }

        if (cfg.tilt_dirty) {
            printf("[MOTOR] Tilt -> %+.0f deg\n", cfg.tilt_angle_deg);
            kinect_set_tilt_angle(cfg.tilt_angle_deg);
            cfg.tilt_dirty = false;
        }

        if (cfg.request_reset) {
            kf_reset(&kf);
            missed = 0;
            cfg.request_reset = false;
        }

        if (!kinect_get_new_frame(&frame)) {
            Sleep(5);
            continue;
        }

        if (prev_t == 0.0) {
            prev_t = frame.timestamp_sec;
            continue;
        }

        double dt = frame.timestamp_sec - prev_t;
        prev_t = frame.timestamp_sec;
        if (dt <= 0.0 || dt > 0.1) continue;

        kf.tilt_angle_deg = cfg.tilt_angle_deg;
        kf_predict(&kf, dt);

        tObservation3D obs = get_object_from_view(frame.rgb, frame.depth, cfg.thresh);
        tInterceptPred pred = {0};

        if (obs.valid) {
            kf_next(&kf, obs);
            missed = 0;

            pred = kf_pred_impact(&kf, cfg.target_floor, cfg.tilt_angle_deg);
            if (pred.valid) {
                printf("[TRACK] t=%.2fs | X=%+.2fm Z=%.2fm\n", pred.t_impact, pred.x_impact, pred.z_impact);
            }
        } else {
            if (++missed > cfg.max_missed_frames && kf.initialized) {
                printf("[RESET] Target lost\n");
                kf_reset(&kf);
            }
            if (kf.initialized) {
                pred = kf_pred_impact(&kf, cfg.target_floor, cfg.tilt_angle_deg);
            }
        }

        display_render(frame.rgb, obs, &kf, pred, &cfg);
    }

    display_destroy();
    kinect_stop();
    return 0;
}
