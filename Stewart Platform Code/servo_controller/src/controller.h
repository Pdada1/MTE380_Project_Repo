#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "camera_servo.h"

#define KP 4
#define KI 0.3
#define KD 3
#define I_LIMIT 1.0

#define CONTROLLER_DELAY_MAX 1.0
#define CONTROLLER_DELAY 0.55
#define SAMPLE_RATE_APPROX 0.045

//Tilt history kept for looking back CONTROLLER_DELAY (needs to cover at least CONTROLLER_DELAY_MAX)
#define FIT_HISTORY_SAMPLES ((int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX) + 2)

struct RotationData { //Struct to keep track of tilt history and shifted tilt
    //Tilt history, ordered old -> new, used to look back CONTROLLER_DELAY for each new sample
    float t[FIT_HISTORY_SAMPLES] = {0};      //t of each sample [s]
    float thetax[FIT_HISTORY_SAMPLES] = {0}; //commanded tilt [deg]
    float thetay[FIT_HISTORY_SAMPLES] = {0};
    int num_valid = 0; //Number of real samples in the history arrays (they start out zero-filled). Set to 0 to force a refill.

    float msx = 0;
    float msy = 0;
    float theta0 = 0;
    float thetax_shifted = 0; //Delay-compensated tilt of the newest sample [deg]
    float thetay_shifted = 0;
};

//theta0_deg: current estimate of theta0 [deg], used to rotate the error/velocity/integral into the platform frame
float pid_get_xtilt(BallData* ball_data, float theta0_deg);
float pid_get_ytilt(BallData* ball_data, float theta0_deg);

float vel_av(BallData* ball_data, char axis);
float acc_av(BallData* ball_data, char axis);

void update_shifted_tilt(RotationData* rd, float t_new, float thetax, float thetay);

#endif
