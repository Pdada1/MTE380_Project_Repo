#include "controller.h"
#include "../alternate_tracking.h"

namespace autolevel {

static uint16_t num_samples = 0;
static uint32_t frame_ms[CAMERA_SAMPLES] = {};
static uint32_t last_frame_ms = 0;
static bool have_frame = false;
static float thetax = 0;
static float thetay = 0;

void reset() {
    num_samples = 0;
    for (int i = 0; i < CAMERA_SAMPLES; ++i) {
        frame_ms[i] = 0;
    }
    have_frame = false;
    thetax = 0;
    thetay = 0;
    reset_controller();
}

void track(BallData& ball_data, ApplyAlternateTilt apply_tilt) {
    const int latest = CAMERA_SAMPLES - 1;
    const uint32_t now_ms = millis();
    if (have_frame && now_ms - last_frame_ms > 300) {
        reset();
    }
    have_frame = true;
    last_frame_ms = now_ms;
    for (int i = 0; i < CAMERA_SAMPLES - 1; ++i) {
        frame_ms[i] = frame_ms[i + 1];
    }
    frame_ms[latest] = now_ms;
    if (num_samples < CAMERA_SAMPLES) {
        ++num_samples;
    }

    if (num_samples >= CAMERA_SAMPLES) {
        const uint32_t elapsed_ms = frame_ms[latest] - frame_ms[latest - 1];
        const float dt = elapsed_ms / 1000.0f;
        if (dt > 0) {
            const float pid_x = pid_get_xtilt(&ball_data, frame_ms);
            const float pid_y = pid_get_ytilt(&ball_data, frame_ms);
            thetax = constrain(thetax + pid_x * dt, -PLANE_ANGLE_MAX, PLANE_ANGLE_MAX);
            thetay = constrain(thetay + pid_y * dt, -PLANE_ANGLE_MAX, PLANE_ANGLE_MAX);
        }
        apply_tilt(thetax, thetay);
        Serial.print(now_ms); Serial.print(' ');
        Serial.print(thetax); Serial.print(' ');
        // Serial.print(thetay); Serial.print(' ');
        Serial.println();
        //Serial.println(micros());
    }
}

}
