#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "camera_servo.h"

#define KP 4
#define KI 0.3
#define KD 3
#define I_LIMIT 1.0

#define CONTROLLER_DELAY_MAX 2.0
#define CONTROLLER_DELAY 0.55
#define SAMPLE_RATE_APPROX 0.05
#define FIT_SAMPLES 50

struct RotationData { //Struct to keep track of all relevant rotation data
    float ax[FIT_SAMPLES + (int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)] = {0}; //ordered old measurements -> new measurements
    float ay[FIT_SAMPLES + (int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)] = {0};
    float thetax[FIT_SAMPLES + (int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)] = {0};
    float thetay[FIT_SAMPLES + (int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)] = {0};
    float t[FIT_SAMPLES + (int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)] = {0}; //t of each sample [s]
    float msx = 0;
    float msy = 0;
    float theta0 = 0;
    float thetax_shifted = 0;
    float thetay_shifted = 0;
};

float pid_get_xtilt(BallData* ball_data);
float pid_get_ytilt(BallData* ball_data);

float vel_av(BallData* ball_data, char axis);
float acc_av(BallData* ball_data, char axis);

void read_rotation_data(RotationData* rotation_data, BallData* ball_data);
void fit_rotation_data(RotationData* rotation_data);

#endif