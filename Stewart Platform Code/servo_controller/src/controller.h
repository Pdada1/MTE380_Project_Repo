#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "camera_servo.h"

#define I_LIMIT 1.0

float pid_get_xtilt(BallData* ball_data);
float pid_get_ytilt(BallData* ball_data);

#endif