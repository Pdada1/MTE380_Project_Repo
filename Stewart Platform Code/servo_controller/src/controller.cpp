#include "controller.h"
#include "LinearFit.h" //SimpleRegression library

static float kp = KP;
static float ki = KI;
static float kd = KD;

float int_ex = 0;
float int_ey = 0;
static float saved_int_ex;
static float saved_int_ey;

void pid_set_gains(float next_kp, float next_ki, float next_kd) {
    if (!isfinite(next_kp) || !isfinite(next_ki) || !isfinite(next_kd) ||
        next_kp < 0 || next_ki < 0 || next_kd < 0) {
        return;
    }
    if (next_ki != ki) {
        int_ex = 0;
        int_ey = 0;
    }
    kp = next_kp;
    ki = next_ki;
    kd = next_kd;
}

void pid_save_integral() {
    saved_int_ex = int_ex;
    saved_int_ey = int_ey;
}

void pid_restore_integral() {
    int_ex = saved_int_ex;
    int_ey = saved_int_ey;
}

void pid_reset() {
    int_ex = 0;
    int_ey = 0;
}

float e_av(BallData* ball_data, char axis) {
    float* x;
    float* xr;
    switch (axis) {
        case 'x':
            x = ball_data->x;
            xr = ball_data->xr;
            break;
        case 'y':
            x = ball_data->y;
            xr = ball_data->yr;
            break;
        default:
            return 0;
    }
    
    //Average position over latest POS_SAMPLES
    float sum = 0;
    for (int i=CAMERA_SAMPLES-POS_SAMPLES; i<CAMERA_SAMPLES; i++) {
        sum += x[i] - xr[i];
    }
    return sum/POS_SAMPLES;
}

float de_av(BallData* ball_data, char axis) {
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
    float ex[VEL_SAMPLES] = {0};
    float t[VEL_SAMPLES] = {0};
    for (int i=0; i<VEL_SAMPLES; i++) {
        ex[i] = x[CAMERA_SAMPLES-VEL_SAMPLES+i]; // originally subtracted xr here too, but that caused setpoint kick, so now its just x
        // The serial timestamp is in milliseconds; the original fit used seconds.
        t[i] = (ball_data->t[CAMERA_SAMPLES-VEL_SAMPLES+i] - ball_data->t[0]) * 0.001f;
    }

    //Linear regression to find average velocity
    LinearFit<float, float> lf;
    lf.fit(t, ex, VEL_SAMPLES);

    float slope = lf.slope();

    return slope;
}

float inte(BallData* ball_data, char axis) { //Make sure to only call once (for 'x' and then 'y') per new sample, otherwise integral will be inaccurate
    float e;
    float dt = (ball_data->t[CAMERA_SAMPLES-1] - ball_data->t[CAMERA_SAMPLES-2]) * 0.001f;
    switch (axis) {
        case 'x':
            e = ball_data->x[CAMERA_SAMPLES-1] - ball_data->xr[CAMERA_SAMPLES-1];
            int_ex += e*dt;
            int_ex = constrain(int_ex, -I_LIMIT, I_LIMIT);
            return int_ex;
        case 'y':
            e = ball_data->y[CAMERA_SAMPLES-1] - ball_data->yr[CAMERA_SAMPLES-1];
            int_ey += e*dt;
            int_ey = constrain(int_ey, -I_LIMIT, I_LIMIT);
            return int_ey;
        default:
            return 0;
    }
}

float pid_get_tilt(BallData* ball_data, char axis, uint16_t fresh_samples) { //Gets tilt for axis 'x' or 'y'
    // Use the latest error until enough fresh frames are available to average it.
    const int latest = CAMERA_SAMPLES - 1;
    const float latest_error = axis == 'x' ? ball_data->x[latest] - ball_data->xr[latest]
                                          : ball_data->y[latest] - ball_data->yr[latest];
    const float e = fresh_samples >= POS_SAMPLES ? e_av(ball_data, axis) : latest_error;
    
    // Keep the saved correction on the first frame and skip velocity across the gap.
    const float int_e = fresh_samples >= 2 ? inte(ball_data, axis) : (axis == 'x' ? int_ex : int_ey);
    const float d_e = fresh_samples >= VEL_SAMPLES ? de_av(ball_data, axis) : 0.0f;

    float u = e*kp + int_e*ki + d_e*kd;
    return u;
}

float pid_get_xtilt(BallData* ball_data, uint16_t fresh_samples) {
    return pid_get_tilt(ball_data, 'x', fresh_samples);
}

float pid_get_ytilt(BallData* ball_data, uint16_t fresh_samples) {
    return pid_get_tilt(ball_data, 'y', fresh_samples);
}
