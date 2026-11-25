#include "controller.h"
#include "LinearFit.h" //SimpleRegression library

#define POS_SAMPLES 1
#define VEL_SAMPLES 2

float int_ex = 0;
float int_ey = 0;

// @MM autotune PID params
float kp_a = 4; 
float ki_a = 0; 
float kd_a = 0;



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
    
    // Generate error/time arrays relative to the first velocity sample.
    const uint32_t start_ms = ball_data->frame_ms[CAMERA_SAMPLES-VEL_SAMPLES];
    float ex[VEL_SAMPLES] = {0};
    float t[VEL_SAMPLES] = {0};
    for (int i=0; i<VEL_SAMPLES; i++) {
        ex[i] = x[CAMERA_SAMPLES-VEL_SAMPLES+i]; // originally subtracted xr here too, but that caused setpoint kick, so now its just x
        const uint32_t elapsed_ms = ball_data->frame_ms[CAMERA_SAMPLES-VEL_SAMPLES+i] - start_ms;
        t[i] = elapsed_ms / 1000.0f;
    }

    //Linear regression to find average velocity
    LinearFit<float, float> lf;
    lf.fit(t, ex, VEL_SAMPLES);

    float slope = lf.slope();

    return slope;
}

float inte(BallData* ball_data, char axis) { //Make sure to only call once (for 'x' and then 'y') per new sample, otherwise integral will be inaccurate
    float e;
    // Subtract in unsigned milliseconds so elapsed time survives timestamp rollover.
    const uint32_t elapsed_ms = ball_data->frame_ms[CAMERA_SAMPLES-1] - ball_data->frame_ms[CAMERA_SAMPLES-2];
    const float dt = elapsed_ms / 1000.0f;
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

float pid_get_tilt(BallData* ball_data, char axis, bool autotune) { //Gets tilt for axis 'x' or 'y'
    float e = e_av(ball_data, axis);
    float int_e = inte(ball_data, axis);
    float d_e = de_av(ball_data, axis);
    float u;

    if (autotune) 
        u = e * kp_a + int_e * ki_a + d_e * kd_a;
    else 
        u = e*ball_data->kp + int_e*ball_data->ki + d_e*ball_data->kd;
    



    // Debug every 10 control updates
    // static int count = 0;
    // if (axis == 'x' && ++count >= 10) {
    //     count = 0;

    //     Serial.print("P: ");
    //     Serial.print(p, 3);
    //     Serial.print(" D: ");
    //     Serial.print(d, 3);
    //     Serial.print(" U: ");
    //     Serial.println(u, 3);
    // }

    return u;
}


float pid_get_xtilt(BallData* ball_data, bool autotune) {
    return pid_get_tilt(ball_data, 'x', autotune);
}

float pid_get_ytilt(BallData* ball_data, bool autotune) {
    return pid_get_tilt(ball_data, 'y', autotune);
}



// @ MM


// FIXME refactor code such that kp is only ever set, not scaled
void autotune_set_kp(float kp) {
    kp_a = constrain(kp, KP_MIN, KP_MAX);
}


void autotune_scale_kp(float scale) { 
    kp_a *= scale; 
    kp_a = constrain(kp_a, KP_MIN, KP_MAX);
}

float autotune_get_kp() { 
    return kp_a; 
}

// True when kp_a is pinned at a limit and `ratio` is asking for more in that direction.
// (ratio > 1: ball too slow, wants more kp. ratio < 1: too fast, wants less.)
bool autotune_kp_saturated(float ratio) {
    return (ratio > 1.0f && kp_a >= KP_MAX - 1e-3f) ||
           (ratio < 1.0f && kp_a <= KP_MIN + 1e-3f);
}


void autotune_set_kd(float kd) { 
    kd_a = constrain(kd, KD_MIN, KD_MAX);
}

float autotune_get_kd() { 
    return kd_a; 
}

bool autotune_kd_at_max() { 
    return kd_a >= KD_MAX - 1e-3f; 
}

float autotune_get_ki(){
    return ki_a;
}

void autotune_set_ki(float ki) {
    ki_a = constrain(ki, KI_MIN, KI_MAX);
}

bool autotune_ki_at_max() {
    return ki_a >= KI_MAX - 1e-3f;
}

// Start a ki run from a known integrator state.
void autotune_reset_integral() {
    int_ex = 0;
    int_ey = 0;
}
    