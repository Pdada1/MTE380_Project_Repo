#include "controller.h"
#include "LinearFit.h"
#include "ParabolicFit.h"

namespace autolevel {

#define POS_SAMPLES 1
#define VEL_SAMPLES 4
#define ACC_SAMPLES 12 //12 samples

#define AUTOLEVEL_I_LIMIT 0.6f

static float int_ex = 0;
static float int_ey = 0;

static float vel_av(BallData* ball_data, char axis, const uint32_t* frame_ms) {
    float* x;
    switch (axis) {
        case 'x':
            x = ball_data->x;
            break;
        case 'y':
            x = ball_data->y;
            break;
        default:
            return 0;
    }

    const uint32_t start_ms = frame_ms[CAMERA_SAMPLES - VEL_SAMPLES];
    float position[VEL_SAMPLES] = {0};
    float time[VEL_SAMPLES] = {0};
    for (int i = 0; i < VEL_SAMPLES; ++i) {
        const int sample = CAMERA_SAMPLES - VEL_SAMPLES + i;
        position[i] = x[sample];
        time[i] = (frame_ms[sample] - start_ms) / 1000.0f;
    }

    LinearFit<float, float> fit;
    fit.fit(time, position, VEL_SAMPLES);
    return fit.slope();
}

static float inte(BallData* ball_data, char axis, const uint32_t* frame_ms) {
    const int latest = CAMERA_SAMPLES - 1;
    float dt = (frame_ms[latest] - frame_ms[latest-1]) / 1000.0f;
    switch (axis) {
        case 'x': {
            int_ex += vel_av(ball_data, 'x', frame_ms) * dt;
            int_ex = constrain(int_ex, -AUTOLEVEL_I_LIMIT, AUTOLEVEL_I_LIMIT);
            return int_ex;
        }
        case 'y': {
            int_ey += vel_av(ball_data, 'y', frame_ms) * dt;
            int_ey = constrain(int_ey, -AUTOLEVEL_I_LIMIT, AUTOLEVEL_I_LIMIT);
            return int_ey;
        }
        default:
            return 0;
    }
}

static float acc_av(BallData* ball_data, char axis, const uint32_t* frame_ms) {
    float* x;
    switch (axis) {
        case 'x':
            x = ball_data->x;
            break;
        case 'y':
            x = ball_data->y;
            break;
        default:
            return 0;
    }
    
    //Generate error/time arrays
    float pos[ACC_SAMPLES] = {0};
    float t[ACC_SAMPLES] = {0};
    for (int i=0; i<ACC_SAMPLES; i++) {
        pos[i] = x[CAMERA_SAMPLES-ACC_SAMPLES+i]; // originally subtracted xr here too, but that caused setpoint kick, so now its just x
        t[i] = (frame_ms[CAMERA_SAMPLES-ACC_SAMPLES+i] - frame_ms[0]) / 1000.0f;
    }

    //Linear regression to find average acceleration
    ParabolicFit<float, float> pf;
    pf.fit(t, pos, ACC_SAMPLES);

    float acc = 2*pf.a();
    return acc;
}

static float pid_get_tilt(BallData* ball_data, char axis, const uint32_t* frame_ms) {
    const float error = vel_av(ball_data, axis, frame_ms);
    const float integral = inte(ball_data, axis, frame_ms);
    const float derivative = acc_av(ball_data, axis, frame_ms);

    if (axis == 'x') {
        Serial.print(ball_data->x[CAMERA_SAMPLES-1]); Serial.print(' ');
        Serial.print(error, 3); Serial.print(' ');
        Serial.print(integral, 3); Serial.print(' ');
        Serial.print(derivative, 3); Serial.print(' ');
    }
    // else {Serial.print(ball_data->y[CAMERA_SAMPLES-1]); Serial.print(' ');}
    // Serial.print(ball_data->kp, 3); Serial.print(' ');
    // Serial.print(ball_data->ki, 3); Serial.print(' ');
    // Serial.print(ball_data->kd, 3); Serial.print(' ');

    return error * ball_data->kp + integral * ball_data->ki + derivative * ball_data->kd;
}

float pid_get_xtilt(BallData* ball_data, const uint32_t* frame_ms) {
    return pid_get_tilt(ball_data, 'x', frame_ms);
}

float pid_get_ytilt(BallData* ball_data, const uint32_t* frame_ms) {
    return pid_get_tilt(ball_data, 'y', frame_ms);
}

void reset_controller() {
    int_ex = 0;
    int_ey = 0;
}

}
