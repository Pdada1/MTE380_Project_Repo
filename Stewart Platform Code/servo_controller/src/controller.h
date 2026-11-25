#ifndef CONTROLLER_H
#define CONTROLLER_H
 
#include "camera_servo.h"

#define I_LIMIT 1.35

// Autotuning limits
#define KD_MIN 0
#define KD_MAX 12
#define KP_MIN 3
#define KP_MAX 8
#define KI_MIN 0
#define KI_MAX 2

float pid_get_xtilt(BallData* ball_data, bool autotune);
float pid_get_ytilt(BallData* ball_data, bool autotune);
float e_av(BallData* ball_data, char axis);
float de_av(BallData* ball_data, char axis);
void pid_reset_integral();
inline void pid_reset() { pid_reset_integral(); }
void autotune_set_kp(float kp);



#endif
