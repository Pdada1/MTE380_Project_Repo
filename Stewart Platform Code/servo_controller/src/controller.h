#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "camera_servo.h"

#define KP 4.5
#define KI 0.5
#define KD 3.7
#define I_LIMIT 0.8

void pid_set_gains(float kp, float ki, float kd);
void pid_reset();
void pid_save_integral();
void pid_restore_integral();

float pid_get_xtilt(BallData* ball_data);
float pid_get_ytilt(BallData* ball_data);

#endif