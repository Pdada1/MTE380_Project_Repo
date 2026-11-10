#ifndef CAMERA_SERVO_H
#define CAMERA_SERVO_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_RANGE 180
#define SERVO_A 0
#define SERVO_B 1
#define SERVO_C 2

#define SERVO_A_MIN 0 // Servo limits
#define SERVO_A_MAX 180
#define SERVO_B_MIN 0
#define SERVO_B_MAX 180
#define SERVO_C_MIN 0
#define SERVO_C_MAX 180

#define SERVO_A_NEUT 90 // Neutral positions (platform parralel to gnd)
#define SERVO_B_NEUT 90
#define SERVO_C_NEUT 90

#define SHIELD_ADDR 0x40
Adafruit_PWMServoDriver servo_drive;

#define CAMERA_SAMPLES 10

struct BallData { //Struct to keep track of all relevant ball data
    float x[CAMERA_SAMPLES] = {0}; //ordered old measurements -> new measurements
    float y[CAMERA_SAMPLES] = {0};
    float xr[CAMERA_SAMPLES] = {0};
    float yr[CAMERA_SAMPLES] = {0};
    float t[CAMERA_SAMPLES] = {0}; //t of each sample [s]
};

bool camera_data_ready();

void camera_read_data(BallData* ball_data);

/*!
 *  @brief  Sets the servo angle for a given servo 
 *  @param  num Servo PWM number from 0 to 15
 *  @param  angle Angle of the servo in degrees
 */
void setServoAngle(uint8_t num, float angle);

void setTilt(float thetax, float thetay);

#endif