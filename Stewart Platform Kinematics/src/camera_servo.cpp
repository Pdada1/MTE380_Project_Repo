#include "camera_servo.h"

Adafruit_PWMServoDriver servo_drive;

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

void servoInit() {
    servo_drive = Adafruit_PWMServoDriver(SHIELD_ADDR);
    servo_drive.begin();
    servo_drive.setOscillatorFrequency(27000000);
    servo_drive.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
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

    // Limit plane angle to max of plus/minus 20 deg
    if (thetax > PLANE_ANGLE_MAX)
        thetax = PLANE_ANGLE_MAX;
    else if (thetax < -PLANE_ANGLE_MAX)
        thetax = -PLANE_ANGLE_MAX;

    if (thetay > PLANE_ANGLE_MAX)
        thetay = PLANE_ANGLE_MAX;
    else if (thetay < -PLANE_ANGLE_MAX)
        thetay = -PLANE_ANGLE_MAX;



    // normal vector
    float n[3] = {sin(thetax*PI/180), sin(thetay*PI/180), 0};
    n[2] = sqrt(1 - pow(n[0],2) - pow(n[1],2) ); 
    // setting up the z-component of the normal this way allows 
    // for the input angle to translate 1:1 to the plane angle

    // normalize normal vector
    float mag = 1 / sqrt(pow(n[0],2) + pow(n[1],2) + pow(n[2], 2));
    for (int i = 0; i < 3; i++) {
        n[i] *= mag;
    }

    // orthonormal vectors
    float temp1 = 1 + n[2];    
    float temp2 = -(n[0]*n[1])/temp1;

    float e1[3] = {
        1 - pow(n[0],2)/temp1, 
        temp2, 
        -n[0]
    }; 
    float e2[3] = {
        temp2, 
        1 - pow(n[1],2)/temp1, 
        -n[1]
    }; 

    const float STOT = sqrt(3)/2;

    // Vectors representing the offsets from the center of the platform to the pivot points 
    float p1[3] = {
        S*e2[0], 
        S*e2[1], 
        S*e2[2]
    };
    float p2[3] = {
        S*(STOT*e1[0]-0.5*e2[0]),
        S*(STOT*e1[1]-0.5*e2[1]),
        S*(STOT*e1[2]-0.5*e2[2])
    };
    float p3[3] = {
        S*(-STOT*e1[0]-0.5*e2[0]),
        S*(-STOT*e1[1]-0.5*e2[1]),
        S*(-STOT*e1[2]-0.5*e2[2])
    };
 
    // center point is then calculated from the offsets to determine how much it drifts during plane rotation
    float center[3] = {
        -p1[0],
        -p2[1]-(p2[0]-p1[0])/sqrt(3),
        H
    };

    // sum center and offsets to obtain the locations of the pivot points 
    float X1[3] = {
        center[0] + p1[0],
        center[1] + p1[1],
        center[2] + p1[2]
    };
    float X2[3] = {
        center[0] + p2[0],
        center[1] + p2[1],
        center[2] + p2[2]
    };
    float X3[3] = {
        center[0] + p3[0],
        center[1] + p3[1],
        center[2] + p3[2]
    };

    // Inverse kinematics 
    float d[3] = {
        sqrt(pow(X1[0], 2) + pow(X1[1], 2)) - L_1,
        sqrt(pow(X2[0], 2) + pow(X2[1], 2)) - L_1,
        sqrt(pow(X3[0], 2) + pow(X3[1], 2)) - L_1
    };
    float D[3] = {
        sqrt(pow(d[0], 2) + pow(X1[2], 2)),
        sqrt(pow(d[1], 2) + pow(X2[2], 2)),
        sqrt(pow(d[2], 2) + pow(X3[2], 2))
    };
    float theta1 = (atan2(X1[2], d[0]) - acos((pow(L_11,2) - pow(L_12,2) + pow(D[0],2))/(2*L_11*D[0])))*180/PI;
    float theta2 = (atan2(X2[2], d[1]) - acos((pow(L_11,2) - pow(L_12,2) + pow(D[1],2))/(2*L_11*D[1])))*180/PI;
    float theta3 = (atan2(X3[2], d[2]) - acos((pow(L_11,2) - pow(L_12,2) + pow(D[2],2))/(2*L_11*D[2])))*180/PI;

    // debug
    Serial.print("theta1: ");
    Serial.println(theta1);
    Serial.print("theta2: ");
    Serial.println(theta2);
    Serial.print("theta3: ");
    Serial.println(theta3);


    setServoAngle(0, theta1);
    setServoAngle(1, theta2);
    setServoAngle(2, theta2);
}  

