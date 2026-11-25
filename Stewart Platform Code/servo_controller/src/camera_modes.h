#ifndef CAMERA_MODES_H
#define CAMERA_MODES_H
 
#include "camera_servo.h"
 
void handle_camera_command(const CameraPacket& packet, BallData& ballData);
 
// Prepare the sweep service (sample history, integrals, servo pins) from the Autotune command.
void sweep_service_begin(const CameraPacket& command);
 
#endif