#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "camera_servo.h"

#define I_LIMIT 1.5


// Autotuning limits
#define KD_MIN 0
#define KD_MAX 12
#define KP_MIN 3
#define KP_MAX 8

float pid_get_xtilt(BallData* ball_data, bool autotune);
float pid_get_ytilt(BallData* ball_data, bool autotune);
float e_av(BallData* ball_data, char axis);
float de_av(BallData* ball_data, char axis);


#endif