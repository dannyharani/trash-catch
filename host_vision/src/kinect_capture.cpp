#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <NuiApi.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <mutex>

extern "C" {
#include "kinect_capture.h"
}

static std::atomic<bool> s_running(false);
static std::thread s_worker;
static std::mutex s_mtx;

static tKinectFrame s_latest;
static bool s_new_frame = false;

static HANDLE s_color_ev = INVALID_HANDLE_VALUE;
static HANDLE s_depth_ev = INVALID_HANDLE_VALUE;
static HANDLE s_color_stream = INVALID_HANDLE_VALUE;
static HANDLE s_depth_stream = INVALID_HANDLE_VALUE;
static INuiSensor *s_sensor = nullptr;

static double now_sec() {
    using namespace std::chrono;
    static auto t0 = steady_clock::now();
    return duration<double>(steady_clock::now() - t0).count();
}

static void capture_loop() {
    HANDLE evs[2] = { s_color_ev, s_depth_ev };

    static uint8_t rgb_buf[RGB_BUF_SIZE];
    static uint16_t depth_buf[DEPTH_BUF_SIZE];
    static USHORT raw_depth[DEPTH_BUF_SIZE];
    static LONG color_xy[DEPTH_BUF_SIZE * 2];

    bool got_color = false;
    bool got_depth = false;

    while (s_running.load()) {
        DWORD res = WaitForMultipleObjects(2, evs, FALSE, 100);

        if (res == WAIT_OBJECT_0) {
            NUI_IMAGE_FRAME frame;
            if (SUCCEEDED(s_sensor->NuiImageStreamGetNextFrame(s_color_stream, 0, &frame))) {
                INuiFrameTexture *tex = frame.pFrameTexture;
                NUI_LOCKED_RECT rect;
                tex->LockRect(0, &rect, NULL, 0);

                if (rect.Pitch != 0) {
                    const uint8_t *src = static_cast<const uint8_t*>(rect.pBits);
                    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
                        rgb_buf[i * 3 + 0] = src[i * 4 + 2];
                        rgb_buf[i * 3 + 1] = src[i * 4 + 1];
                        rgb_buf[i * 3 + 2] = src[i * 4 + 0];
                    }
                    got_color = true;
                }
                tex->UnlockRect(0);
                s_sensor->NuiImageStreamReleaseFrame(s_color_stream, &frame);
            }
        } else if (res == WAIT_OBJECT_0 + 1) {
            NUI_IMAGE_FRAME frame;
            if (SUCCEEDED(s_sensor->NuiImageStreamGetNextFrame(s_depth_stream, 0, &frame))) {
                INuiFrameTexture *tex = frame.pFrameTexture;
                NUI_LOCKED_RECT rect;
                tex->LockRect(0, &rect, NULL, 0);

                if (rect.Pitch != 0) {
                    std::memcpy(raw_depth, rect.pBits, sizeof(raw_depth));

                    // Align depth to RGB camera space to avoid parallax offset
                    HRESULT hr = s_sensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
                        NUI_IMAGE_RESOLUTION_640x480,
                        NUI_IMAGE_RESOLUTION_640x480,
                        DEPTH_BUF_SIZE,
                        raw_depth,
                        DEPTH_BUF_SIZE * 2,
                        color_xy
                    );

                    if (SUCCEEDED(hr)) {
                        std::memset(depth_buf, 0, sizeof(depth_buf));
                        for (int i = 0; i < DEPTH_BUF_SIZE; i++) {
                            uint16_t d = NuiDepthPixelToDepth(raw_depth[i]);
                            if (!d) continue;

                            LONG cx = color_xy[i * 2];
                            LONG cy = color_xy[i * 2 + 1];
                            // 2x2 splat fills gaps from stretching wider IR FOV onto RGB grid
                            for (LONG dy = 0; dy <= 1; dy++) {
                                LONG y = cy + dy;
                                if (y < 0 || y >= FRAME_HEIGHT) continue;
                                for (LONG dx = 0; dx <= 1; dx++) {
                                    LONG x = cx + dx;
                                    if (x < 0 || x >= FRAME_WIDTH) continue;
                                    size_t idx = (size_t)y * FRAME_WIDTH + (size_t)x;
                                    if (depth_buf[idx] == 0 || d < depth_buf[idx]) {
                                        depth_buf[idx] = d;
                                    }
                                }
                            }
                        }
                    } else {
                        for (int i = 0; i < DEPTH_BUF_SIZE; i++) {
                            depth_buf[i] = NuiDepthPixelToDepth(raw_depth[i]);
                        }
                    }
                    got_depth = true;
                }
                tex->UnlockRect(0);
                s_sensor->NuiImageStreamReleaseFrame(s_depth_stream, &frame);
            }
        }

        if (got_color && got_depth) {
            std::lock_guard<std::mutex> lk(s_mtx);
            std::memcpy(s_latest.rgb, rgb_buf, RGB_BUF_SIZE);
            std::memcpy(s_latest.depth, depth_buf, sizeof(depth_buf));
            s_latest.timestamp_sec = now_sec();
            s_new_frame = true;
            got_color = false;
            got_depth = false;
        }
    }
}

extern "C" bool kinect_start(void) {
    if (s_running.load()) return true;

    int count = 0;
    if (FAILED(NuiGetSensorCount(&count)) || count == 0) return false;
    if (FAILED(NuiCreateSensorByIndex(0, &s_sensor)) || !s_sensor) return false;

    HRESULT hr = s_sensor->NuiInitialize(
        NUI_INITIALIZE_FLAG_USES_COLOR |
        NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX
    );
    if (FAILED(hr)) {
        s_sensor->Release();
        s_sensor = nullptr;
        return false;
    }

    s_color_ev = CreateEvent(NULL, TRUE, FALSE, NULL);
    s_depth_ev = CreateEvent(NULL, TRUE, FALSE, NULL);

    hr = s_sensor->NuiImageStreamOpen(
        NUI_IMAGE_TYPE_COLOR,
        NUI_IMAGE_RESOLUTION_640x480,
        0, 2, s_color_ev, &s_color_stream
    );
    if (FAILED(hr)) {
        kinect_stop();
        return false;
    }

    hr = s_sensor->NuiImageStreamOpen(
        NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
        NUI_IMAGE_RESOLUTION_640x480,
        0, 2, s_depth_ev, &s_depth_stream
    );
    if (FAILED(hr)) {
        kinect_stop();
        return false;
    }

    s_running.store(true);
    s_worker = std::thread(capture_loop);
    return true;
}

extern "C" void kinect_stop(void) {
    if (s_running.load()) {
        s_running.store(false);
        if (s_worker.joinable()) s_worker.join();
    }

    if (s_sensor) {
        s_sensor->NuiShutdown();
        s_sensor->Release();
        s_sensor = nullptr;
    }

    if (s_color_ev != INVALID_HANDLE_VALUE) {
        CloseHandle(s_color_ev);
        s_color_ev = INVALID_HANDLE_VALUE;
    }
    if (s_depth_ev != INVALID_HANDLE_VALUE) {
        CloseHandle(s_depth_ev);
        s_depth_ev = INVALID_HANDLE_VALUE;
    }
}

extern "C" bool kinect_get_new_frame(tKinectFrame *out) {
    if (!out) return false;

    std::lock_guard<std::mutex> lk(s_mtx);
    if (!s_new_frame) return false;

    std::memcpy(out->rgb, s_latest.rgb, RGB_BUF_SIZE);
    std::memcpy(out->depth, s_latest.depth, sizeof(out->depth));
    out->timestamp_sec = s_latest.timestamp_sec;
    s_new_frame = false;
    return true;
}

extern "C" bool kinect_get_tilt_angle(double *out_deg) {
    if (!s_sensor || !out_deg) return false;

    LONG angle = 0;
    if (SUCCEEDED(s_sensor->NuiCameraElevationGetAngle(&angle))) {
        *out_deg = (double)angle;
        return true;
    }
    return false;
}

extern "C" bool kinect_set_tilt_angle(double deg) {
    if (!s_sensor) return false;

    LONG target = static_cast<LONG>(std::round(deg));
    if (target > NUI_CAMERA_ELEVATION_MAXIMUM) target = NUI_CAMERA_ELEVATION_MAXIMUM;
    if (target < NUI_CAMERA_ELEVATION_MINIMUM) target = NUI_CAMERA_ELEVATION_MINIMUM;

    return SUCCEEDED(s_sensor->NuiCameraElevationSetAngle(target));
}
