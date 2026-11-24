#include "controller.h"
#include "LinearFit.h" //SimpleRegression library
#include "ParabolicFit.h"

#define POS_SAMPLES (int)(1) //1 sample
#define VEL_SAMPLES (int)(4) //4 samples
#define ACC_SAMPLES (int)(12) //12 samples

static const double DEG2RAD = PI / 180.0;

float int_ex = 0;
float int_ey = 0;

//Rotates a camera-frame (x,y) pair into the platform frame, i.e. by -theta0.
//(The platform x axis points along (cos(theta0), sin(theta0)) in the camera frame.)
static void rotate_to_platform(float theta0_deg, float x, float y, float* xp, float* yp) {
    float th = theta0_deg * (float)DEG2RAD;
    float c = cos(th);
    float s = sin(th);
    *xp =  c*x + s*y;
    *yp = -s*x + c*y;
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
    
    //Generate error/time arrays
    float ex[VEL_SAMPLES] = {0};
    float t[VEL_SAMPLES] = {0};
    for (int i=0; i<VEL_SAMPLES; i++) {
        ex[i] = x[CAMERA_SAMPLES-VEL_SAMPLES+i] - xr[CAMERA_SAMPLES-VEL_SAMPLES+i]; // originally subtracted xr here too, but that caused setpoint kick, so now its just x
        t[i] = ball_data->t[CAMERA_SAMPLES-VEL_SAMPLES+i] - ball_data->t[0]; //Use difference to keep numbers from getting large for LinearFit
    }

    //Linear regression to find average velocity
    LinearFit<float, float> lf;
    lf.fit(t, ex, VEL_SAMPLES);

    float slope = lf.slope();

    return slope;
}

float vel_av(BallData* ball_data, char axis) {
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
        t[i] = ball_data->t[CAMERA_SAMPLES-VEL_SAMPLES+i] - ball_data->t[0]; //Use difference to keep numbers from getting large for LinearFit
    }

    //Linear regression to find average velocity
    LinearFit<float, float> lf;
    lf.fit(t, ex, VEL_SAMPLES);

    float slope = lf.slope();

    return slope;
}

float inte(BallData* ball_data, char axis, float theta0_deg) { //Make sure to only call once (for 'x' and then 'y') per new sample, otherwise integral will be inaccurate
    float dt = ball_data->t[CAMERA_SAMPLES-1] - ball_data->t[CAMERA_SAMPLES-2];

    //Latest error in the camera frame, rotated into the platform frame. The integral is accumulated from this already-rotated error.
    float ex = ball_data->x[CAMERA_SAMPLES-1] - ball_data->xr[CAMERA_SAMPLES-1];
    float ey = ball_data->y[CAMERA_SAMPLES-1] - ball_data->yr[CAMERA_SAMPLES-1];
    float ex_p, ey_p;
    rotate_to_platform(theta0_deg, ex, ey, &ex_p, &ey_p);

    switch (axis) {
        case 'x':
            int_ex += ex_p*dt;
            int_ex = constrain(int_ex, -I_LIMIT, I_LIMIT);
            return int_ex;
        case 'y':
            int_ey += ey_p*dt;
            int_ey = constrain(int_ey, -I_LIMIT, I_LIMIT);
            return int_ey;
        default:
            return 0;
    }
}

float acc_av(BallData* ball_data, char axis) {
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
        t[i] = ball_data->t[CAMERA_SAMPLES-ACC_SAMPLES+i] - ball_data->t[0]; //Use difference to keep numbers from getting large for LinearFit
    }

    //Linear regression to find average acceleration
    ParabolicFit<float, float> pf;
    pf.fit(t, pos, ACC_SAMPLES);

    float acc = 2*pf.a();

    return acc;
}

float pid_get_tilt(BallData* ball_data, char axis, float theta0_deg) { //Gets tilt for axis 'x' or 'y'
    if (axis != 'x' && axis != 'y') {
        return 0;
    }

    //Error and velocity in the camera frame
    float ex = e_av(ball_data, 'x');
    float ey = e_av(ball_data, 'y');
    float vx = vel_av(ball_data, 'x');
    float vy = vel_av(ball_data, 'y');

    //Rotate both into the platform frame
    float ex_p, ey_p, vx_p, vy_p;
    rotate_to_platform(theta0_deg, ex, ey, &ex_p, &ey_p);
    rotate_to_platform(theta0_deg, vx, vy, &vx_p, &vy_p);

    float e     = (axis == 'x') ? ex_p : ey_p;
    float d_e   = (axis == 'x') ? vx_p : vy_p;
    float int_e = inte(ball_data, axis, theta0_deg); //Integrates the already-rotated error

    float u = e*KP + int_e*KI + d_e*KD;
    return u;
}

float pid_get_xtilt(BallData* ball_data, float theta0_deg) {
    return pid_get_tilt(ball_data, 'x', theta0_deg);
}

float pid_get_ytilt(BallData* ball_data, float theta0_deg) {
    return pid_get_tilt(ball_data, 'y', theta0_deg);
}

void update_shifted_tilt(RotationData* rd, float t_new, float thetax, float thetay) {
    const int hist_n = FIT_HISTORY_SAMPLES;

    //Shift the tilt history and append the new sample
    for (int i = 0; i < hist_n - 1; i++) {
        rd->t[i]      = rd->t[i + 1];
        rd->thetax[i] = rd->thetax[i + 1];
        rd->thetay[i] = rd->thetay[i + 1];
    }
    rd->t[hist_n - 1]      = t_new;
    rd->thetax[hist_n - 1] = thetax;
    rd->thetay[hist_n - 1] = thetay;
    if (rd->num_valid < hist_n) {
        rd->num_valid++;
    }

    //Linearly interpolate the tilt history CONTROLLER_DELAY earlier
    float t_target = t_new - CONTROLLER_DELAY;
    int first_valid = hist_n - rd->num_valid;
    int j = hist_n - 2;
    while (j >= first_valid && rd->t[j] > t_target) {
        j--;
    }
    if (j < first_valid) {
        return;
    }

    float dt = rd->t[j + 1] - rd->t[j];
    float frac = (dt > 1e-9f) ? (t_target - rd->t[j]) / dt : 0.0f;
    frac = constrain(frac, 0.0f, 1.0f);

    rd->thetax_shifted = rd->thetax[j] + (rd->thetax[j + 1] - rd->thetax[j]) * frac;
    rd->thetay_shifted = rd->thetay[j] + (rd->thetay[j + 1] - rd->thetay[j]) * frac;
}
