#include "controller.h"
#include "LinearFit.h" //SimpleRegression library

#define POS_SAMPLES (int)(1) //1 sample
#define VEL_SAMPLES (int)(2) //2 samples

float int_ex = 0;
float int_ey = 0;

// @MM autotune PID params
float kp = KP; //todo find a good conservative value to have as an initial proportion constant
float ki = 0;
float kd = 0;

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
        t[i] = ball_data->t[CAMERA_SAMPLES-VEL_SAMPLES+i] - ball_data->t[0]; //Use difference to keep numbers from getting large for LinearFit
    }

    //Linear regression to find average velocity
    LinearFit<float, float> lf;
    lf.fit(t, ex, VEL_SAMPLES);

    float slope = lf.slope();

    return slope;
}

float inte(BallData* ball_data, char axis) { //Make sure to only call once (for 'x' and then 'y') per new sample, otherwise integral will be inaccurate
    float e;
    float dt = ball_data->t[CAMERA_SAMPLES-1] - ball_data->t[CAMERA_SAMPLES-2];
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
    float p, u, d;

    if (autotune) {
        p = e * kp;
        d = d_e * kd;
        u = p + int_e * ki + d;
    }
    else {
        p = e * KP;
        d = d_e * KD;
        u = p + int_e * KI + d;
    }



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

void set_autotune_pid(float _kp, float _kd, float _ki) {
    kp *= _kp; // FIXME: this function should ideally set the value, not scale it
    kd = _kd;
    ki = _ki;
}

    