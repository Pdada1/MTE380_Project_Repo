#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "camera_servo.h"

#define KP 3
#define KI 1
#define KD 4
#define I_LIMIT 3

float pid_get_xtilt(BallData* ball_data);
float pid_get_ytilt(BallData* ball_data);

#endif