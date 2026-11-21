
#include "camera_servo.h"
#include "controller.h"

BallData ballData; // Ball data to be fed to the controller (x,y,t,xr,yr)
uint16_t num_samples = 0;

#define SYNC_BYTES 3
#define PACKET_SIZE_BYTES 23
const uint8_t SYNC_WORD[SYNC_BYTES] = {0x0B, 0x00, 0x0B};

// #define CALIBRATE_SERVOS /* Uncomment to enable manual servo calibration mode */
// #define SWEEP_SERVOS /* Uncomment to enable automatic servo sweep mode */

struct CameraPacket {
    uint8_t status;
    uint8_t servo_A_pin;
    uint8_t servo_B_pin;
    uint8_t servo_C_pin;
    float x;
    float y;
    float xr;
    float yr;
};

bool read_serial_port(CameraPacket *packet) {
    static uint8_t buf[PACKET_SIZE_BYTES] = {0};
    static size_t buf_count = 0;

    while (Serial.available() > 0) {
        buf[buf_count++] = (uint8_t)Serial.read();

        if (buf_count >= SYNC_BYTES &&
            memcmp(buf, SYNC_WORD, SYNC_BYTES) != 0) {
            // Sync word not valid, shift buffer by one byte
            memmove(buf, buf + 1, --buf_count);
            continue;
        }

        if (buf_count < PACKET_SIZE_BYTES) {
            // Packet incomplete, continue building it
            continue;
        }

        // Packet complete, extract it
        packet->status = buf[SYNC_BYTES];
        packet->servo_A_pin = buf[SYNC_BYTES + 1];
        packet->servo_B_pin = buf[SYNC_BYTES + 2];
        packet->servo_C_pin = buf[SYNC_BYTES + 3];

        size_t offset = SYNC_BYTES + 4;

        memcpy(&packet->x, buf + offset, sizeof(float));
        offset += sizeof(float);

        memcpy(&packet->y, buf + offset, sizeof(float));
        offset += sizeof(float);

        memcpy(&packet->xr, buf + offset, sizeof(float));
        offset += sizeof(float);

        memcpy(&packet->yr, buf + offset, sizeof(float));

        // Ready to receive next packet
        buf_count = 0;
        return true;
    }

    return false;
}

void run_servo_calibration(void) {
    static int curr_angle1 = SERVO_A_HORIZ;
    static int curr_angle2 = SERVO_B_HORIZ;
    static int curr_angle3 = SERVO_C_HORIZ;

    if (Serial.available() > 0) {
        char key = Serial.read();
        if (key == 'q') curr_angle1 = constrain(curr_angle1 + 1, 0, SERVO_RANGE);
        if (key == 'a') curr_angle1 = constrain(curr_angle1 - 1, 0, SERVO_RANGE);
        if (key == 'w') curr_angle2 = constrain(curr_angle2 + 1, 0, SERVO_RANGE);
        if (key == 's') curr_angle2 = constrain(curr_angle2 - 1, 0, SERVO_RANGE);
        if (key == 'e') curr_angle3 = constrain(curr_angle3 + 1, 0, SERVO_RANGE);
        if (key == 'd') curr_angle3 = constrain(curr_angle3 - 1, 0, SERVO_RANGE);
    }

    setServoAngleManual(0, curr_angle1);
    setServoAngleManual(2, curr_angle2);
    setServoAngleManual(1, curr_angle3);

    Serial.print(curr_angle1);
    Serial.print(' ');
    Serial.print(curr_angle2);
    Serial.print(' ');
    Serial.println(curr_angle3);
    delay(50);
}

void run_servo_sweep(void) {
    BallData sweep_data = {}; // Local ball data, no camera required

    // Set servo pins
    sweep_data.servo_A_pin = 0;
    sweep_data.servo_B_pin = 2;
    sweep_data.servo_C_pin = 1;

    const int SWEEP_ANGLE = 15;
    const int SWEEP_DELAY = 100;

    // Start with platform level
    setTilt(0, 0, &sweep_data);
    delay(1000);

    // Sweep X: 0 -> -15 -> +15 -> 0
    for (int angle = 0; angle >= -SWEEP_ANGLE; angle--) {
        setTilt(angle, 0, &sweep_data);
        delay(SWEEP_DELAY);
    }
    for (int angle = -SWEEP_ANGLE + 1; angle <= SWEEP_ANGLE; angle++) {
        setTilt(angle, 0, &sweep_data);
        delay(SWEEP_DELAY);
    }
    for (int angle = SWEEP_ANGLE - 1; angle >= 0; angle--) {
        setTilt(angle, 0, &sweep_data);
        delay(SWEEP_DELAY);
    }

    delay(500);

    // Sweep Y: 0 -> -15 -> +15 -> 0
    for (int angle = 0; angle >= -SWEEP_ANGLE; angle--) {
        setTilt(0, angle, &sweep_data);
        delay(SWEEP_DELAY);
    }
    for (int angle = -SWEEP_ANGLE + 1; angle <= SWEEP_ANGLE; angle++) {
        setTilt(0, angle, &sweep_data);
        delay(SWEEP_DELAY);
    }
    for (int angle = SWEEP_ANGLE - 1; angle >= 0; angle--) {
        setTilt(0, angle, &sweep_data);
        delay(SWEEP_DELAY);
    }

    // Finish with platform level
    setTilt(0, 0, &sweep_data);
}

void test_servo_response() {
    const float angles1[3] = {
        SERVO_A_HORIZ,
        SERVO_B_HORIZ,
        SERVO_C_HORIZ
    };

    const float angles2[3] = {
        SERVO_A_MAX,
        SERVO_B_MAX,
        SERVO_C_MAX
    };

    const float angles3[3] = {
        SERVO_A_MIN,
        SERVO_B_MIN,
        SERVO_C_MIN
    };

    const uint8_t pins[3] = {0, 2, 1};

    for (int i = 0; i < 3; i++) {
        setServoAngleManual(pins[i], angles1[i]);
    }
    delay(1000);

    for (int i = 0; i < 3; i++) {
        setServoAngleManual(pins[i], angles2[i]);
    }
    delay(1000);

    for (int i = 0; i < 3; i++) {
        setServoAngleManual(pins[i], angles3[i]);
    }
    delay(1000);
}

void setup() {
    Serial.begin(115200);
    servoInit();

#ifdef SWEEP_SERVOS
    while (1) {
        run_servo_sweep();
        test_servo_response();
    }
#endif

#ifdef CALIBRATE_SERVOS
    while (1) {
        run_servo_calibration();
    }
#endif
}

void loop() {
    CameraPacket curr_packet;
    CameraPacket latest_packet;
    bool new_data = false;
    bool level_platform = false;




    // @MM
    static bool use_autotune_pid = false;

    #define T_RISE_DESIRED 1.0 // TODO does this have to be proportional to the distance the set point is from the initial position? 

    static float xr_prev = 0;
    static float yr_prev = 0;
    static float e_prev = 0;

    static float t_0 = 0; // time when set point is first changed 

    // Empty serial buffer, keeping the latest valid measurement
    while (read_serial_port(&curr_packet)) {
        switch (curr_packet.status) {
            case 0x00:
                // Ball not in frame, do nothing
                break;

            case 0x01:
                // Ball data valid, keep latest measurement
                latest_packet = curr_packet;
                new_data = true;
                level_platform = false;
                break;

            case 0x02:
                // Set platform to 'level' state
                new_data = false;
                level_platform = true;
                break;

            case 0x03: // @MM
                // Run frequency response for motor autotuning
                latest_packet = curr_packet;
                level_platform = false;


                // detect if set point has changed
                // if so, change PID parameters, and save when it changed
                // @TODO check if ball has settled prior.
                // @TODO generate xr, yr randomly 
                // thus if you change setpoint while ball hasnt settled, youll have to choose one again 
                if (xr_prev != latest_packet.xr || yr_prev != latest_packet.yr) {
                    use_autotune_pid = true;
                    t_0 = millis() / 1000.0f;
                    e_prev = 0;
              
                    xr_prev = latest_packet.xr;
                    yr_prev = latest_packet.yr;
                }

                break;

            default:
                // Invalid status, do nothing
                break;
        }
    }


    if (level_platform) {
        // Set platform to neutral position
        setTilt(0, 0, &ballData);
        return;
    }


    if (!new_data) {
        // No new camera measurement
        return;
    }

    // Shift existing camera samples to make room for next one
    for (int i = 0; i < CAMERA_SAMPLES - 1; i++) {
        ballData.x[i] = ballData.x[i + 1];
        ballData.y[i] = ballData.y[i + 1];
        ballData.xr[i] = ballData.xr[i + 1];
        ballData.yr[i] = ballData.yr[i + 1];
        ballData.t[i] = ballData.t[i + 1];
    }

    // Store latest camera measurement
    ballData.status = latest_packet.status;
    ballData.servo_A_pin = latest_packet.servo_A_pin;
    ballData.servo_B_pin = latest_packet.servo_B_pin;
    ballData.servo_C_pin = latest_packet.servo_C_pin;

    ballData.x[CAMERA_SAMPLES - 1] = latest_packet.x;
    ballData.y[CAMERA_SAMPLES - 1] = latest_packet.y;
    ballData.xr[CAMERA_SAMPLES - 1] = latest_packet.xr;
    ballData.yr[CAMERA_SAMPLES - 1] = latest_packet.yr;
    ballData.t[CAMERA_SAMPLES - 1] = millis() / 1000.0f;

    // Only run controller after ballData buffers have filled
    if (num_samples < CAMERA_SAMPLES) {
        num_samples++;
    }

    if (use_autotune_pid) {

        float e = sqrt(pow(e_av(&ballData, 'x'), 2) + pow(e_av(&ballData, 'y'), 2));
        float de = e_prev - e;

        if (de > 0) { // ball passes set point
            use_autotune_pid = false;
            float t_r = ballData.t[0] - t_0;
            float kp_scale = t_r/T_RISE_DESIRED;
            Serial.print("time: ");
            Serial.print(ballData.t[0]);
            Serial.print("\tt_r: ");
            Serial.print(t_r);
            Serial.print("\tt_desired: ");
            Serial.print(T_RISE_DESIRED);
            Serial.print("\tkp_scale: ");
            Serial.println(kp_scale);

            set_autotune_pid(kp_scale, 0, 0);
        }

        e_prev = e;
    }

    if (num_samples >= CAMERA_SAMPLES) {
        float thetax = pid_get_xtilt(&ballData, use_autotune_pid);
        float thetay = pid_get_ytilt(&ballData, use_autotune_pid);
        setTilt(thetax, thetay, &ballData);

        // Serial.print(thetax);
        // Serial.print(' ');
        // Serial.println(thetay);
    }
}
