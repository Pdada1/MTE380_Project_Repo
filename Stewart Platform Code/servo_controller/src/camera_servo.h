#ifndef CAMERA_SERVO_H
#define CAMERA_SERVO_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_MIN_DUTY 0.5/20
#define SERVO_MAX_DUTY 2.5/20
#define SERVO_RANGE 180

#define SERVO_A_MAX 119.0 // Link touches ground
#define SERVO_A_MIN 47.0 //Links are straight
#define SERVO_A_HORIZ 100.0 // Horizontal position (linkage parallel to gnd)
#define SERVO_B_MAX 131.0
#define SERVO_B_MIN 65.5
#define SERVO_B_HORIZ 113.0
#define SERVO_C_MAX 125.0
#define SERVO_C_MIN 57.5
#define SERVO_C_HORIZ 108.0

#define SHIELD_ADDR 0x40

#define CAMERA_SAMPLES 10

struct BallData { //Struct to keep track of all relevant ball data
    float x[CAMERA_SAMPLES] = {0}; //ordered old measurements -> new measurements
    float y[CAMERA_SAMPLES] = {0};
    float xr[CAMERA_SAMPLES] = {0};
    float yr[CAMERA_SAMPLES] = {0};
    float t[CAMERA_SAMPLES] = {0}; //t of each sample [s]
    uint8_t servo_A_pin = 0xFF;
    uint8_t servo_B_pin = 0xFF;
    uint8_t servo_C_pin = 0xFF;
};

bool camera_data_ready();

void camera_read_data(BallData* ball_data);

void servoInit();

/*!
 *  @brief  Sets the servo angle for a given servo 
 *  @param  servo Servo number 'A'/'B'/'C'
 *  @param  angle_rel_horiz Angle of the servo in degrees relative to the platform
 *  @param  ball_data ballData struct
 */
void setServoAngle(char servo, float angle_rel_horiz, BallData* ball_data);

void setTilt(float thetax, float thetay, BallData* ball_data);

#endif