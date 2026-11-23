#include "camera_servo.h"

Adafruit_PWMServoDriver servo_drive;

void servoInit() {
    servo_drive = Adafruit_PWMServoDriver(SHIELD_ADDR);
    servo_drive.begin();
    servo_drive.setOscillatorFrequency(27000000);
    servo_drive.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

// Trims belong to physical PWM channels. Keep the desired tilt angles so
// trims can be applied immediately even when tracking is paused.
static float trims[16] = {};
static float last_angles[3] = {};
static const float horizontal[3] = {SERVO_A_HORIZ, SERVO_B_HORIZ, SERVO_C_HORIZ};
static const float minimum[3] = {SERVO_A_MIN, SERVO_B_MIN, SERVO_C_MIN};
static const float maximum[3] = {SERVO_A_MAX, SERVO_B_MAX, SERVO_C_MAX};

bool setServoTrim(uint8_t pin, float offset_deg) {
    if (pin >= 16 || !isfinite(offset_deg)) {
        return false;
    }
    const float offset = constrain(offset_deg, -50.0f, 50.0f);
    const bool changed = trims[pin] != offset;
    trims[pin] = offset;
    return changed;
}

static void writeServoAngle(uint8_t axis, uint8_t pin, float relative_deg) {
    const float degrees = constrain(horizontal[axis] - relative_deg + trims[pin],
                                    minimum[axis], maximum[axis]);
    const float period_us = 1000000 / (float)SERVO_FREQ;
    const float duty = (degrees / SERVO_RANGE) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) + SERVO_MIN_DUTY;
    servo_drive.writeMicroseconds(pin, duty * period_us);
}

void refreshServoTrim(BallData* ball_data) {
    writeServoAngle(0, ball_data->servo_A_pin, last_angles[0]);
    writeServoAngle(1, ball_data->servo_B_pin, last_angles[1]);
    writeServoAngle(2, ball_data->servo_C_pin, last_angles[2]);
}

static bool solveTilt(float thetax, float thetay, float angles[3], const BallData* data) {
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
    angles[0] = (atan2(X1[2], d[0]) - acos((pow(L_11,2) - pow(L_12,2) + pow(D[0],2))/(2*L_11*D[0])))*180/PI;
    angles[1] = (atan2(X2[2], d[1]) - acos((pow(L_11,2) - pow(L_12,2) + pow(D[1],2))/(2*L_11*D[1])))*180/PI;
    angles[2] = (atan2(X3[2], d[2]) - acos((pow(L_11,2) - pow(L_12,2) + pow(D[2],2))/(2*L_11*D[2])))*180/PI;

    const uint8_t pins[3] = {data->servo_A_pin, data->servo_B_pin, data->servo_C_pin};
    for (uint8_t axis = 0; axis < 3; ++axis) {
        if (pins[axis] >= 16 || !isfinite(angles[axis])) {
            return false;
        }
        const float degrees = horizontal[axis] - angles[axis] + trims[pins[axis]];
        if (degrees < minimum[axis] || degrees > maximum[axis]) {
            return false;
        }
    }
    return true;
}

// Return true when the requested tilt had to be reduced.
bool setTilt(float thetax, float thetay, BallData* ball_data) {
    if (!isfinite(thetax) || !isfinite(thetay)) {
        return true;
    }

    const float limit = PLANE_ANGLE_MAX;
    const bool axis_limited = fabs(thetax) > limit || fabs(thetay) > limit;
    thetax = constrain(thetax, -limit, limit);
    thetay = constrain(thetay, -limit, limit);

    float angles[3];
    const bool limited = !solveTilt(thetax, thetay, angles, ball_data);
    if (limited) {
        if (!solveTilt(0, 0, angles, ball_data)) {
            return true;
        }
        float low = 0.0f;
        float high = 1.0f;
        for (uint8_t i = 0; i < 5; ++i) {
            const float mid = (low + high) * 0.5f;
            float candidate[3];
            if (solveTilt(mid * thetax, mid * thetay, candidate, ball_data)) {
                low = mid;
                memcpy(angles, candidate, sizeof(angles));
            } else {
                high = mid;
            }
        }
    }

    memcpy(last_angles, angles, sizeof(last_angles));
    refreshServoTrim(ball_data);
    return limited || axis_limited;
}