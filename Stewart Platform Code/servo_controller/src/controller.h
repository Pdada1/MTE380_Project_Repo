#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "camera_servo.h"

#define KP 4.5
#define KI 1.4
#define KD 4.2
#define I_LIMIT 0.5

void pid_set_gains(float kp, float ki, float kd);
void pid_reset();
void pid_save_integral();
void pid_restore_integral();

float pid_get_xtilt(BallData* ball_data);
float pid_get_ytilt(BallData* ball_data);

#endif