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
#define SERVO_A_HORIZ 83.0 // Horizontal position (linkage parallel to gnd)
#define SERVO_B_MAX 113.0
#define SERVO_B_MIN 65.0
#define SERVO_B_HORIZ 85.0
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
 
#define CAMERA_SAMPLES 3
 
// Match the status byte sent by track_ball.py.
enum class PacketStatus : uint8_t {
    BallLost = 0,
    Tracking = 1,
    Level = 2,
    Autotune = 3,
    Autolevel = 4,
    Sweep = 5,
    ServoTrim = 6,
    SweepData = 8, // ball/target stream the PC sends while an autotune sweep runs
};
 
// Payload of Python's <3sB3B4fI3f packet, after the three sync bytes.
struct __attribute__((packed)) CameraPacket {
    PacketStatus status;
    uint8_t servo_pins[3];
    float x, y;
    float xr, yr;
    uint32_t timestamp; // Frame time in ms, or the request ID for sweep
    float kp, ki, kd;
};
 
extern CameraPacket latest_camera_packet;
 
struct BallData { //Struct to keep track of all relevant ball data
    float x[CAMERA_SAMPLES] = {0}; //ordered old measurements -> new measurements
    float y[CAMERA_SAMPLES] = {0};
    float xr[CAMERA_SAMPLES] = {0};
    float yr[CAMERA_SAMPLES] = {0};
    uint32_t frame_ms[CAMERA_SAMPLES] = {}; // Python frame timestamps [ms]
    // Gains are populated by the first tracking packet.
    float kp{}, ki{}, kd{};
    uint8_t servo_A_pin = 0;
    uint8_t servo_B_pin = 2;
    uint8_t servo_C_pin = 1;
};
 
bool camera_data_ready();
bool camera_take_command(CameraPacket& packet);
 
void camera_read_data(BallData* ball_data);
 
void servoInit();
 
/*!
 *  @brief  Sets the servo angle for a given servo 
 *  @param  servo Servo number 'A'/'B'/'C'
 *  @param  angle_rel_horiz Angle of the servo in degrees relative to the platform
 *  @param  ball_data ballData struct
 */
void setServoAngle(char servo, float angle_rel_horiz, BallData* ball_data);
 
void apply_servo_trims(const CameraPacket& packet, BallData& ballData);
 
void setTilt(float thetax, float thetay, BallData* ball_data);
 
#endif