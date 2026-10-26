#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "vision_pipeline.h"

#define RGB_BUF_SIZE   (FRAME_WIDTH * FRAME_HEIGHT * 3)
#define DEPTH_BUF_SIZE (FRAME_WIDTH * FRAME_HEIGHT)

#define KINECT_TILT_MAX_DEG  27.0
#define KINECT_TILT_MIN_DEG -27.0

typedef struct {
    uint8_t  rgb[RGB_BUF_SIZE];
    uint16_t depth[DEPTH_BUF_SIZE];
    double   timestamp_sec;
} tKinectFrame;

bool kinect_start(void);
void kinect_stop(void);
bool kinect_get_new_frame(tKinectFrame *out);
bool kinect_get_tilt_angle(double *out_deg);
bool kinect_set_tilt_angle(double deg);
