#include "camera_servo.h"

Adafruit_PWMServoDriver servo_drive;

void servoInit() {
    servo_drive = Adafruit_PWMServoDriver(SHIELD_ADDR);
    servo_drive.begin();
    servo_drive.setOscillatorFrequency(27000000);
    servo_drive.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

/*!
 *  @brief  Sets the servo angle for a given servo to a specific angle
 *  @param  servo_num Servo number
 *  @param  angle_servo Servo angle in degrees
 */
void setServoAngleManual(uint8_t servo_num, float angle_servo)
{
    angle_servo = constrain(angle_servo, 0.0f, (float)SERVO_RANGE);

    float pulse_us = 500.0f +
        (angle_servo / (float)SERVO_RANGE) * 2000.0f;

    servo_drive.writeMicroseconds(servo_num, (uint16_t)pulse_us);
}

/*!
 *  @brief  Sets the servo angle for a given servo relative to the platform
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
    float n[3] = {-tan(thetax*PI/180), -tan(thetay*PI/180), 1};

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
        S*e1[0], 
        S*e1[1], 
        S*e1[2]
    };
    float p2[3] = {
        S*(STOT*e2[0]-0.5*e1[0]),
        S*(STOT*e2[1]-0.5*e1[1]),
        S*(STOT*e2[2]-0.5*e1[2])
    };
    float p3[3] = {
        S*(-STOT*e2[0]-0.5*e1[0]),
        S*(-STOT*e2[1]-0.5*e1[1]),
        S*(-STOT*e2[2]-0.5*e1[2])
    };
 
    // center point is then calculated from the offsets to determine how much it drifts during plane rotation
    float center[3] = {
        -p2[0]-(p2[1]-p1[1])/sqrt(3),
        -p1[1],
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
    // Serial.print("theta1: ");
    // Serial.println(theta1);
    // Serial.print("theta2: ");
    // Serial.println(theta2);
    // Serial.print("theta3: ");
    // Serial.println(theta3);

    setServoAngle('A', theta1, ball_data);
    setServoAngle('B', theta2, ball_data);
    setServoAngle('C', theta3, ball_data);
}

