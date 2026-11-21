#ifndef CAMERA_SERVO_H
#define CAMERA_SERVO_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_MIN_DUTY 0.5/20
#define SERVO_MAX_DUTY 2.5/20
#define SERVO_RANGE 180

// Servo A: index 0 (as labelled on platform)
// Servo B: index 2
// Servo C: index 1
#define SERVO_A_MAX 124.0 // Link touches ground
#define SERVO_A_MIN 65.0 //Links are straight
#define SERVO_A_HORIZ 88.0 // Horizontal position (linkage parallel to gnd)
#define SERVO_B_MAX 113.0
#define SERVO_B_MIN 65.0
#define SERVO_B_HORIZ 88.0
#define SERVO_C_MAX 106.0
#define SERVO_C_MIN 47.0
#define SERVO_C_HORIZ 71.0

// stewart platform dimensions 
#define S 140 // distance from center of platform to pivot point
#define H 100 // height of center point from origin (constrained to be a constant)
#define L_1 66
#define L_11 80
#define L_12 94
#define PLANE_ANGLE_MAX 15

#define SHIELD_ADDR 0x40

#define CAMERA_SAMPLES 3 // Do not change without updating other segments of code!!!
#define POS_SAMPLES 2
#define VEL_SAMPLES 3


struct BallData { //Struct to keep track of all relevant ball data
    uint8_t status;
    float x[CAMERA_SAMPLES] = {0}; //ordered old measurements -> new measurements
    float y[CAMERA_SAMPLES] = {0};
    float xr[CAMERA_SAMPLES] = {0};
    float yr[CAMERA_SAMPLES] = {0};
    uint32_t t[CAMERA_SAMPLES] = {0}; // Sample timestamps in milliseconds
    uint8_t servo_A_pin = 0xFF;
    uint8_t servo_B_pin = 0xFF;
    uint8_t servo_C_pin = 0xFF;
};

bool camera_data_ready();

void camera_read_data(BallData* ball_data);

void servoInit();

bool setServoTrim(uint8_t pin, float offset_deg);
void refreshServoTrim(BallData* ball_data);

bool setTilt(float thetax, float thetay, BallData* ball_data);

#endif