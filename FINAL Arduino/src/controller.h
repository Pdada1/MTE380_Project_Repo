#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "camera_servo.h"

#define KP 1
#define KI 1
#define KD 1
#define I_LIMIT 10

float pid_get_xtilt(BallData* ball_data);
float pid_get_ytilt(BallData* ball_data);

#endif