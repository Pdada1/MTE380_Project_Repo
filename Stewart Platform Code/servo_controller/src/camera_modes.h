#ifndef CAMERA_MODES_H
#define CAMERA_MODES_H

#include "camera_servo.h"

void handle_camera_command(const CameraPacket& packet, BallData& ballData);

#endif