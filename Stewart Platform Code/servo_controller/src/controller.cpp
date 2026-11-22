#include "controller.h"
#include "LinearFit.h" //SimpleRegression library
#include "ParabolicFit.h"

#define POS_SAMPLES (int)(1) //1 sample
#define VEL_SAMPLES (int)(4) //4 samples
#define ACC_SAMPLES (int)(12) //12 samples

float int_ex = 0;
float int_ey = 0;

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

float pid_get_tilt(BallData* ball_data, char axis) { //Gets tilt for axis 'x' or 'y'
    float e = e_av(ball_data, axis);
    float int_e = inte(ball_data, axis);
    float d_e = vel_av(ball_data, axis);

    float u = e*KP + int_e*KI + d_e*KD;
    return u;
}

float pid_get_xtilt(BallData* ball_data) {
    return pid_get_tilt(ball_data, 'x');
}

float pid_get_ytilt(BallData* ball_data) {
    return pid_get_tilt(ball_data, 'y');
}

//ROTATION FITTING
void read_rotation_data(RotationData* rotation_data, BallData* ball_data) {
    //Shift data arrays to make room for new sample
    for (int i=0; i<FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1; i++) {
        rotation_data->ax[i] = rotation_data->ax[i+1];
        rotation_data->ay[i] = rotation_data->ay[i+1];
        rotation_data->thetax[i] = rotation_data->thetax[i+1];
        rotation_data->thetay[i] = rotation_data->thetay[i+1];
        rotation_data->t[i] = rotation_data->t[i+1];
    }

    //Add new sample to end of arrays
    rotation_data->ax[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = acc_av(ball_data, 'x') * (S/1000); //Scale back to m/s^2
    rotation_data->ay[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = acc_av(ball_data, 'y') * (S/1000);
    rotation_data->thetax[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = pid_get_xtilt(ball_data);
    rotation_data->thetay[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = pid_get_ytilt(ball_data);
    rotation_data->t[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = ball_data->t[CAMERA_SAMPLES-1];
}

void fit_rotation_data(RotationData* rotation_data) {
    //Check if there are enough samples after CONTROLLER_DELAY to make up FIT_SAMPLES
    int t_start_index = 0; // Index after CONTROLLER_DELAY
    int t_end_index = FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1;
    for (int i=0; i<=t_end_index; i++) {
        t_start_index = i;
        if (rotation_data->t[i] - rotation_data->t[0] >= CONTROLLER_DELAY) {
            break;
        }
    }
    if (t_end_index - t_start_index < FIT_SAMPLES - 1) {
        Serial.println(F("Not enough samples to fit rotation data!"));
        return;
    }
    else {
        t_start_index = t_end_index - FIT_SAMPLES + 1;
    }

    //Window of FIT_SAMPLES
    float t[FIT_SAMPLES] = {0};
    float ax[FIT_SAMPLES] = {0};
    float ay[FIT_SAMPLES] = {0};
    for (int i=0; i<FIT_SAMPLES; i++) {
        t[i] = rotation_data->t[t_start_index+i] - rotation_data->t[t_start_index]; //Use difference to keep numbers from getting large
        ax[i] = rotation_data->ax[t_start_index+i];
        ay[i] = rotation_data->ay[t_start_index+i];
    }

    //Shift thetax and thetay by CONTROLLER_DELAY, then interpolate to match the t[] array
    float thetax[FIT_SAMPLES] = {0};
    float thetay[FIT_SAMPLES] = {0};
    float t_shifted[FIT_SAMPLES] = {0};
    for (int i=0; i<FIT_SAMPLES; i++) {
        t_shifted[i] = t[i] + CONTROLLER_DELAY;
    }
    int sample1 = 0;
    int sample2 = 1;
    for (int i=0; i<FIT_SAMPLES; i++) {
        //Make sample1 and sample2 bracket t[i]
        while (sample2 < FIT_SAMPLES-1 && t_shifted[sample2] < t[i]) {
            sample1++;
            sample2++;
        }

        //Interpolate between sample1 and sample2 to t[i]
        float dt = t_shifted[sample2] - t_shifted[sample1];
        float frac = (dt > 1e-9f) ? (t[i] - t_shifted[sample1]) / dt : 0.0f;

        thetax[i] = rotation_data->thetax[t_start_index+sample1] + (rotation_data->thetax[t_start_index+sample2] - rotation_data->thetax[t_start_index+sample1]) * frac;
        thetay[i] = rotation_data->thetay[t_start_index+sample1] + (rotation_data->thetay[t_start_index+sample2] - rotation_data->thetay[t_start_index+sample1]) * frac;
    }

    //Debugging
    rotation_data->thetax_shifted = thetax[FIT_SAMPLES-1];
    rotation_data->thetay_shifted = thetay[FIT_SAMPLES-1];

    // Now we have ax[],ay[],thetax[],thetay[] compensated for CONTROLLER_DELAY, all at the same time instances.
    // Least squares
}
