#include "controller.h"
#include "LinearFit.h" //SimpleRegression library

#define POS_SAMPLES (int)(CAMERA_SAMPLES/3)
#define VEL_SAMPLES (int)(CAMERA_SAMPLES/1.4)

float int_ex = 0;
float int_ey = 0;

float e_av(BallData* ball_data, char axis) {
    float* x;
    float* xr;
    switch axis {
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
        sum += xr[i] - x[i];
    }
    return sum/POS_SAMPLES;
}

float de_av(BallData* ball_data, char axis) {
    float* x;
    float* xr;
    switch axis {
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
        ex[i] = xr[CAMERA_SAMPLES-VEL_SAMPLES+i] - x[CAMERA_SAMPLES-VEL_SAMPLES+i];
        t[i] = ball_data->t[CAMERA_SAMPLES-VEL_SAMPLES+i];
    }

    //Linear regression to find average velocity
    LinearFit<int, double> lf;
    lf.fit(t, ex, VEL_SAMPLES);

    float slope = lf.slope();

    return slope;
}

float inte(BallData* ball_data, char axis) { //Make sure to only call once (for 'x' and then 'y') per new sample, otherwise integral will be inaccurate
    float e;
    float dt = ball_data->t[CAMERA_SAMPLES-1] - ball_data->t[CAMERA_SAMPLES-2]
    switch axis {
        case 'x':
            e = ball_data->xr[CAMERA_SAMPLES-1] - ball_data->x[CAMERA_SAMPLES-1];
            int_ex += e*dt;
            int_ex = constrain(int_ex, -I_LIMIT, I_LIMIT);
            break;
        case 'y':
            e = ball_data->yr[CAMERA_SAMPLES-1] - ball_data->y[CAMERA_SAMPLES-1];
            int_ey += e*dt;
            int_ey = constrain(int_ey, -I_LIMIT, I_LIMIT);
            break;
        default:
            return 0;
    }
}

float get_tilt_PID(BallData* ball_data, char axis) { //Gets tilt for axis 'x' or 'y'
    e = e_av(ball_data, axis);
    int_e = inte(ball_data, axis);
    d_e = de_av(ball_data, axis);

    float u = e*KP + int_e*KI + d_e*KD;
    return u;
}

float get_xtilt_PID(BallData* ball_data) {
    get_tilt_PID(ball_data, 'x');
}

float get_ytilt_PID(BallData* ball_data) {
    get_tilt_PID(ball_data, 'y');
}