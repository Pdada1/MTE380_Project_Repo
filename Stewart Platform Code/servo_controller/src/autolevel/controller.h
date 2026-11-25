#ifndef AUTOLEVEL_CONTROLLER_H
#define AUTOLEVEL_CONTROLLER_H

#include "camera_servo.h"

namespace autolevel {

float pid_get_xtilt(BallData* ball_data, const uint32_t* frame_ms);
float pid_get_ytilt(BallData* ball_data, const uint32_t* frame_ms);
void reset_controller();

}

#endif
