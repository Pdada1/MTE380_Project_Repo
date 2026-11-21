#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "camera_servo.h"

#define KP 6.0
#define KI 0.0
#define KD 4.5
#define I_LIMIT 0.5

float pid_get_xtilt(BallData* ball_data, bool autotune);
float pid_get_ytilt(BallData* ball_data, bool autotune);
float e_av(BallData* ball_data, char axis);

void set_autotune_pid(float _kp, float _kd, float _ki);

#endif