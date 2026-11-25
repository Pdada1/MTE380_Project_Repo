#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#include "camera_servo.h"


void autotune_scale_kp(float scale);
float autotune_get_kp();
void autotune_set_kd(float kd);
float autotune_get_kd();
float autotune_get_ki();
bool autotune_kp_saturated(float ratio);
bool autotune_kd_at_max();
bool autotune_routine(BallData* ballData);
void autotune_set_rise_time(float ms);
void autotune_set_overshoot(float percent);
void autotune_set_e_ss(float radii);
void autotune_set_ki(float ki);
bool autotune_ki_at_max();
void autotune_reset_integral();

#endif