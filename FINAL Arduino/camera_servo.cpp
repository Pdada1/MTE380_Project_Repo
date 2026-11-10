#include "camera_servo.h"

bool camera_data_ready() { //Reads until after the first "bob"
    bool data_ready = false;

    int bytes_available = Serial.available();
    char char_frame[3] = {0};
    if (bytes_available >= 3+4*sizeof(float)) { //"bob" + 4 floats
        for (int i=0; i<bytes_available-4*sizeof(float); i++) { //Always leave at least 4 floats in serial buffer
            char_frame[0] = char_frame[1]; //Track frame of 3 chars to check for bob
            char_frame[1] = char_frame[2];
            char_frame[2] = Serial.read();

            if (char_frame[0] == 'b' && char_frame[1] == 'o' && char_frame[2] == 'b') {
                data_ready = true;
                break;
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
        Serial.readBytes((uint8_t*)&ball_data->x[CAMERA_SAMPLES-1], sizeof(float));
        Serial.readBytes((uint8_t*)&ball_data->y[CAMERA_SAMPLES-1], sizeof(float));
        Serial.readBytes((uint8_t*)&ball_data->xr[CAMERA_SAMPLES-1], sizeof(float));
        Serial.readBytes((uint8_t*)&ball_data->yr[CAMERA_SAMPLES-1], sizeof(float));
    } while (camera_data_ready());

    //Get time of sample
    ball_data->t[CAMERA_SAMPLES-1] = millis()/(float)1000;
}

/*!
 *  @brief  Sets the servo angle for a given servo 
 *  @param  num Servo PWM number from 0 to 15
 *  @param  angle Angle of the servo in degrees
 */
void setServoAngle(uint8_t num, float angle)
{
    switch (num) {
        case SERVO_A:
            angle = constrain(angle, SERVO_A_MIN, SERVO_A_MAX);
            break;
        case SERVO_B:
            angle = constrain(angle, SERVO_B_MIN, SERVO_B_MAX);
            break;
        case SERVO_C:
            angle = constrain(angle, SERVO_C_MIN, SERVO_C_MAX);
            break;
        default:
            return;
    }

    float pulselength_us = 1000000/(float)SERVO_FREQ;
    float pulse_us = (angle/(float)SERVO_RANGE)*pulselength_us;
    servo_drive.writeMicroseconds(num,pulse_us);
}

void setTilt(float thetax, float thetay) { //Sets platform angle relative to neutral position

}

