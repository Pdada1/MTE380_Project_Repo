#include "controller.h"
#include "LinearFit.h" //SimpleRegression library
 
#define POS_SAMPLES 1
#define VEL_SAMPLES 2

static constexpr float POSITION_DEADBAND_MM = 15.0f;
static constexpr float POSITION_DEADBAND = POSITION_DEADBAND_MM / S;
static constexpr float PROPORTIONAL_DEADBAND = POSITION_DEADBAND * (2.0f / 3.0f);
static constexpr uint32_t TRACKING_GAP_MS = 250;
static constexpr float INTEGRAL_UNWIND_FACTOR = 4.0f;

float int_ex = 0;
float int_ey = 0;

// @MM autotune PID params
float kp_a = 4; 
float ki_a = 0; 
float kd_a = 0;
 
void pid_reset_integral() {
    int_ex = 0.0f;
    int_ey = 0.0f;
}

float e_av(BallData* ball_data, char axis) {
    if (axis != 'x' && axis != 'y') {
        return 0;
    }
    
    //Average position over latest POS_SAMPLES
    float sum = 0;
    for (int i=CAMERA_SAMPLES-POS_SAMPLES; i<CAMERA_SAMPLES; i++) {
        const float ex = ball_data->x[i] - ball_data->xr[i];
        const float ey = ball_data->y[i] - ball_data->yr[i];
        const float radius = sqrtf(ex * ex + ey * ey);
        if (radius > PROPORTIONAL_DEADBAND) {
            sum += (axis == 'x' ? ex : ey) * (1.0f - PROPORTIONAL_DEADBAND / radius);
        }
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
    // Wait for a full velocity window of fresh samples after a tracking gap.
    for (int i = CAMERA_SAMPLES - VEL_SAMPLES + 1; i < CAMERA_SAMPLES; ++i) {
        if (ball_data->frame_ms[i] - ball_data->frame_ms[i - 1] > TRACKING_GAP_MS) {
            return 0.0f;
        }
    }
    if (ball_data->frame_ms[CAMERA_SAMPLES-1] == start_ms) {
        return 0.0f; // Buffered frames can be handled in the same Arduino millisecond.
    }
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

static float inte(const BallData* ball_data, char axis, float proportional_error) {
    if (axis != 'x' && axis != 'y') {
        return 0.0f;
    }
    float& integral = axis == 'x' ? int_ex : int_ey;
    // Subtract in unsigned milliseconds so elapsed time survives timestamp rollover.
    const uint32_t elapsed_ms = ball_data->frame_ms[CAMERA_SAMPLES-1] - ball_data->frame_ms[CAMERA_SAMPLES-2];
    // Preserve the correction learned before the gap without integrating over missing data.
    if (elapsed_ms > TRACKING_GAP_MS) {
        return integral;
    }
    const float dt = elapsed_ms / 1000.0f;
    const int latest = CAMERA_SAMPLES - 1;
    const float ex = ball_data->x[latest] - ball_data->xr[latest];
    const float ey = ball_data->y[latest] - ball_data->yr[latest];
    // Hold the accumulated bias inside the tolerance; use full error outside it.
    if (ex * ex + ey * ey <= POSITION_DEADBAND * POSITION_DEADBAND) {
        return integral;
    }
    const float error = axis == 'x' ? ex : ey;
    const float p_tilt = proportional_error * ball_data->kp;
    const float i_tilt = integral * ball_data->ki;
    float delta = error * dt;
    // After crossing a fixed target, unwind faster while I overpowers P.
    if (ball_data->fixed_target && i_tilt * error < 0.0f
        && p_tilt * error >= 0.0f && fabsf(i_tilt) > fabsf(p_tilt)) {
        delta = copysignf(fminf(fabsf(delta) * INTEGRAL_UNWIND_FACTOR, fabsf(integral)), delta);
    }
    integral = constrain(integral + delta, -I_LIMIT, I_LIMIT);
    return integral;
}

float pid_get_tilt(BallData* ball_data, char axis, bool autotune) { //Gets tilt for axis 'x' or 'y'
    float e = e_av(ball_data, axis);
    float int_e = inte(ball_data, axis, e);
    float d_e = de_av(ball_data, axis);
    float u;

    if (autotune) 
        u = e * kp_a + int_e * ki_a + d_e * kd_a;
    else 
        u = e*ball_data->kp + int_e*ball_data->ki + d_e*ball_data->kd;
    
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
// FIXME this is identical to pid_reset(), so refactor
void autotune_reset_integral() {
    int_ex = 0;
    int_ey = 0;
}
    
