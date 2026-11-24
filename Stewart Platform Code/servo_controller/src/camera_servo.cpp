#include "camera_servo.h"

Adafruit_PWMServoDriver servo_drive;

// Trim offsets follow the PWM channels even if calibration reorders A/B/C.
static uint8_t trim_channels[3] = {};
static float servo_trims[3] = {};
static bool have_tilt = false;
static float last_tilt_x = 0;
static float last_tilt_y = 0;

void apply_servo_trims(const CameraPacket& packet, BallData& ballData) {
    for (uint8_t i = 0; i < 3; ++i) {
        trim_channels[i] = packet.servo_pins[i];
    }
    servo_trims[0] = packet.x;
    servo_trims[1] = packet.y;
    servo_trims[2] = packet.xr;

    // Before the first pose command, just store the offsets.
    if (have_tilt) {
        setTilt(last_tilt_x, last_tilt_y, &ballData);
    }
}

static float servo_trim(uint8_t channel) {
    for (uint8_t i = 0; i < 3; ++i) {
        if (trim_channels[i] == channel) {
            return servo_trims[i];
        }
    }
    return 0;
}

CameraPacket latest_camera_packet{};
static bool command_pending = false;

bool camera_take_command(CameraPacket& packet) {
    if (!command_pending) {
        return false;
    }
    packet = latest_camera_packet;
    command_pending = false;
    return true;
}

bool camera_data_ready() {
    static uint8_t sync[3] = {};
    static bool have_tracking_frame = false;
    static uint32_t last_frame_ms = 0;
    static PacketStatus last_frame_status = PacketStatus::BallLost;

    // Keep enough bytes buffered for the payload after the last sync byte.
    while (!command_pending && Serial.available() > (int)sizeof(CameraPacket)) {
        sync[0] = sync[1];
        sync[1] = sync[2];
        sync[2] = Serial.read();

        if (sync[0] == 0x0B && sync[1] == 0x00 && sync[2] == 0x0B) {
            Serial.readBytes(reinterpret_cast<uint8_t*>(&latest_camera_packet),
                             sizeof(latest_camera_packet));
            sync[0] = sync[1] = sync[2] = 0;

            // Stop draining here so the command runs before any later tracking packet.
            const PacketStatus status = latest_camera_packet.status;
            const bool tracking_frame = status == PacketStatus::Tracking || status == PacketStatus::Autolevel;
            if (!tracking_frame && status != PacketStatus::BallLost) {
                command_pending = true;
                return false;
            }

            // Use the Python timestamp only to reject resends of the same frame.
            if (status == PacketStatus::Tracking && (!have_tracking_frame || latest_camera_packet.timestamp != last_frame_ms)) {
                last_frame_ms = latest_camera_packet.timestamp;
                last_frame_status = status;
                have_tracking_frame = true;
                return true;
            }
        }
    }
    return false;
}

void camera_read_data(BallData* ball_data) { //Assumes camera data is ready
    //Shift BallData array by one
    for (int i=0; i<CAMERA_SAMPLES-1; i++) {
        ball_data->x[i] = ball_data->x[i+1];
        ball_data->y[i] = ball_data->y[i+1];
        ball_data->xr[i] = ball_data->xr[i+1];
        ball_data->yr[i] = ball_data->yr[i+1];
        ball_data->frame_ms[i] = ball_data->frame_ms[i+1];
    }

    //Empty serial buffer, leaving the last valid measurement in ballData
    do {
        ball_data->servo_A_pin = latest_camera_packet.servo_pins[0];
        ball_data->servo_B_pin = latest_camera_packet.servo_pins[1];
        ball_data->servo_C_pin = latest_camera_packet.servo_pins[2];
        ball_data->x[CAMERA_SAMPLES-1] = latest_camera_packet.x;
        ball_data->y[CAMERA_SAMPLES-1] = latest_camera_packet.y;
        ball_data->xr[CAMERA_SAMPLES-1] = latest_camera_packet.xr;
        ball_data->yr[CAMERA_SAMPLES-1] = latest_camera_packet.yr;
        ball_data->frame_ms[CAMERA_SAMPLES-1] = millis();
        ball_data->kp = latest_camera_packet.kp;
        ball_data->ki = latest_camera_packet.ki;
        ball_data->kd = latest_camera_packet.kd;
    } while (camera_data_ready());

    //Logging
    // Serial.print(F("pins A/B/C: "));
    // Serial.print((int)ball_data->servo_A_pin);  Serial.print('/');
    // Serial.print((int)ball_data->servo_B_pin);  Serial.print('/');
    // Serial.print((int)ball_data->servo_C_pin);

    // Serial.print(F("  x="));  Serial.print(ball_data->x[CAMERA_SAMPLES-1], 2);
    // Serial.print(F(" y="));   Serial.print(ball_data->y[CAMERA_SAMPLES-1], 2);
    // Serial.print(F(" xr="));  Serial.print(ball_data->xr[CAMERA_SAMPLES-1], 2);
    // Serial.print(F(" yr="));  Serial.println(ball_data->yr[CAMERA_SAMPLES-1], 2);
}

void servoInit() {
    servo_drive = Adafruit_PWMServoDriver(SHIELD_ADDR);
    servo_drive.begin();
    servo_drive.setOscillatorFrequency(27000000);
    servo_drive.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

/*!
 *  @brief  Sets the servo angle for a given servo 
 *  @param  channel Servo pin
 *  @param  angle Angle of the servo in degrees
 */
static void write_servo_angle(uint8_t channel, float angle) {
    const float period_us = 1000000 / (float)SERVO_FREQ;
    const float duty = (angle / SERVO_RANGE) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) + SERVO_MIN_DUTY;
    if (channel < 16) {
        servo_drive.writeMicroseconds(channel, duty * period_us);
    }
}

void identify_servo_pins(const CameraPacket& packet) {
    // Use the travel range shared by all three servo calibrations.
    const float low = max(SERVO_A_MIN, max(SERVO_B_MIN, SERVO_C_MIN));
    const float high = min(SERVO_A_MAX, min(SERVO_B_MAX, SERVO_C_MAX));
    // A selector of 16 puts all channels low for the camera baseline.
    const uint8_t selected = packet.servo_pins[0];
    for (uint8_t channel = 0; channel < 16; ++channel) {
        write_servo_angle(channel, channel == selected ? high : low);
    }
}

/*!
 *  @brief  Sets the servo angle for a given servo, relative to horizontal
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
            angle_servo = SERVO_A_HORIZ - angle_rel_horiz + servo_trim(servo_num);
            angle_servo = constrain(angle_servo, SERVO_A_MIN, SERVO_A_MAX);
            break;
        case 'B':
            servo_num = ball_data->servo_B_pin;
            angle_servo = SERVO_B_HORIZ - angle_rel_horiz + servo_trim(servo_num);
            angle_servo = constrain(angle_servo, SERVO_B_MIN, SERVO_B_MAX);
            break;
        case 'C':
            servo_num = ball_data->servo_C_pin;
            angle_servo = SERVO_C_HORIZ - angle_rel_horiz + servo_trim(servo_num);
            angle_servo = constrain(angle_servo, SERVO_C_MIN, SERVO_C_MAX);
            break;
        default: 
            return;
    }

    write_servo_angle(servo_num, angle_servo);
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

    last_tilt_x = thetax;
    last_tilt_y = thetay;
    have_tilt = true;

    setServoAngle('A', theta1, ball_data);
    setServoAngle('B', theta2, ball_data);
    setServoAngle('C', theta3, ball_data);
}

