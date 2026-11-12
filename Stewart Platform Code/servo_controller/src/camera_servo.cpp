#include "camera_servo.h"

Adafruit_PWMServoDriver servo_drive;

bool camera_data_ready() { //Reads until after the first "bob"1
    bool data_ready = false;

    int bytes_available = Serial.available();
    char char_frame[4] = {0};
    if (bytes_available >= 7+4*sizeof(float)) { //"bob"1 + 3 bytes + 4 floats
        for (int i=0; i<bytes_available-(3+4*sizeof(float)); i++) { //Always leave at least 3 bytes + 4 floats in serial buffer
            char_frame[0] = char_frame[1]; //Track frame of 4 chars to check for bob1
            char_frame[1] = char_frame[2];
            char_frame[2] = char_frame[3];
            char_frame[3] = Serial.read();

            if (char_frame[0] == 'b' && char_frame[1] == 'o' && char_frame[2] == 'b' && char_frame[3] == 1) {
                //Ball position valid
                data_ready = true;
                break;
            }
            else if (char_frame[0] == 'b' && char_frame[1] == 'o' && char_frame[2] == 'b' && char_frame[3] == 0) {
                //Ball position invalid
            }
        }
    }
    return data_ready;
}

void camera_read_data(BallData* ball_data) { //Assumes camera data is ready
    //Shift BallData array by one
    for (int i=0; i<CAMERA_SAMPLES-1; i++) {
        ball_data->x[i] = ball_data->x[i+1];
        ball_data->y[i] = ball_data->y[i+1];
        ball_data->xr[i] = ball_data->xr[i+1];
        ball_data->yr[i] = ball_data->yr[i+1];
        ball_data->t[i] = ball_data->t[i+1];
    }

    //Empty serial buffer, leaving the last valid measurement in ballData
    do {
        ball_data->servo_A_pin = Serial.read();
        ball_data->servo_B_pin = Serial.read();
        ball_data->servo_C_pin = Serial.read();
        Serial.readBytes((uint8_t*)&ball_data->x[CAMERA_SAMPLES-1], sizeof(float));
        Serial.readBytes((uint8_t*)&ball_data->y[CAMERA_SAMPLES-1], sizeof(float));
        Serial.readBytes((uint8_t*)&ball_data->xr[CAMERA_SAMPLES-1], sizeof(float));
        Serial.readBytes((uint8_t*)&ball_data->yr[CAMERA_SAMPLES-1], sizeof(float));
    } while (camera_data_ready());

    //Get time of sample
    ball_data->t[CAMERA_SAMPLES-1] = millis()/(float)1000;
}

void servoInit() {
    servo_drive = Adafruit_PWMServoDriver(SHIELD_ADDR);
    servo_drive.begin();
    servo_drive.setOscillatorFrequency(27000000);
    servo_drive.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

/*!
 *  @brief  Sets the servo angle for a given servo 
 *  @param  servo Servo number 'A'/'B'/'C'
 *  @param  angle_rel_horiz Angle of the servo in degrees relative to the platform
 *  @param  ball_data ballData struct
 */
void setServoAngle(char servo, float angle_rel_horiz, BallData* ball_data)
{
    float angle_servo = 0;
    uint8_t servo_num = 0xFF;
    switch (servo) {
        case 'A':
            servo_num = ball_data->servo_A_pin;
            angle_servo = SERVO_A_HORIZ - angle_rel_horiz;
            angle_servo = constrain(angle_servo, SERVO_A_MIN, SERVO_A_MAX);
            break;
        case 'B':
            servo_num = ball_data->servo_B_pin;
            angle_servo = SERVO_B_HORIZ - angle_rel_horiz;
            angle_servo = constrain(angle_servo, SERVO_B_MIN, SERVO_B_MAX);
            break;
        case 'C':
            servo_num = ball_data->servo_C_pin;
            angle_servo = SERVO_C_HORIZ - angle_rel_horiz;
            angle_servo = constrain(angle_servo, SERVO_C_MIN, SERVO_C_MAX);
            break;
        default: 
            return;
    }

    float period_us = 1000000/(float)SERVO_FREQ;
    float duty = (angle_servo/(float)SERVO_RANGE)*(SERVO_MAX_DUTY-SERVO_MIN_DUTY) + SERVO_MIN_DUTY;
    float pulse_us = duty*period_us;
    servo_drive.writeMicroseconds(servo_num, pulse_us);
}

void setTilt(float thetax, float thetay, BallData* ball_data) { //Sets platform angle relative to neutral position
    
}

