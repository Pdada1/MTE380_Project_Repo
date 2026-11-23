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

//Limits for rejecting rotation-fit data. Rejected data -> keep the old msx, msy, theta0.
#define PLATFORM_RADIUS_LIMIT 1.0 //Reject if |(x,y)| > this (x,y are in units of S, so 1 = platform edge)
#define ACC_LIMIT_K 1.5           //K in: reject if |(ax,ay)| > K*5.89*sqrt(sx^2+sy^2)/(1+sx^2+sy^2), sx=sin(thetax), sy=sin(thetay)
#define RMS_LIMIT_DEG 3.0         //Reject if RMS angle between np_calculated and np_measured is above this [deg]

enum FitStatus : uint8_t { //Result of the most recent fit_rotation_data() call
    FIT_OK = 0,                //Fit accepted, msx/msy/theta0 updated
    FIT_NOT_ENOUGH_SAMPLES,
    FIT_OFF_PLATFORM,          //Rejected: |(x,y)| > PLATFORM_RADIUS_LIMIT somewhere in the window
    FIT_ACC_TOO_HIGH,          //Rejected: acceleration too large for the commanded tilt
    FIT_RMS_TOO_HIGH,          //Rejected: RMS angle error above RMS_LIMIT_DEG
    FIT_DEGENERATE             //Rejected: too few usable samples / rotation can't be parameterized
};

struct RotationData { //Struct to keep track of all relevant rotation data
    float ax[FIT_SAMPLES + (int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)] = {0}; //ordered old measurements -> new measurements
    float ay[FIT_SAMPLES + (int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)] = {0};
    float thetax[FIT_SAMPLES + (int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)] = {0};
    float thetay[FIT_SAMPLES + (int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)] = {0};
    float t[FIT_SAMPLES + (int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)] = {0}; //t of each sample [s]
    bool off_platform[FIT_SAMPLES + (int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)] = {false}; //true if the ball was off the platform for this sample
    int num_valid = 0; //Number of real samples stored so far (the arrays start out zero-filled), saturates at the array size
    float msx = 0;
    float msy = 0;
    float theta0 = 0;
    float thetax_shifted = 0;
    float thetay_shifted = 0;
    float rms_deg = 0; //RMS angle between np_calculated and np_measured from the latest fit [deg] (useful for tuning RMS_LIMIT_DEG)
    FitStatus fit_status = FIT_NOT_ENOUGH_SAMPLES;
};

//theta0_deg: current estimate of theta0 [deg], used to rotate the error/velocity/integral into the platform frame
float pid_get_xtilt(BallData* ball_data, float theta0_deg);
float pid_get_ytilt(BallData* ball_data, float theta0_deg);

float vel_av(BallData* ball_data, char axis);
float acc_av(BallData* ball_data, char axis);

//thetax, thetay: the tilt that was just computed by the PID this sample (pass the same values you give to setTilt)
void read_rotation_data(RotationData* rotation_data, BallData* ball_data, float thetax, float thetay);
void fit_rotation_data(RotationData* rotation_data);

#endif