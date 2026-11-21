
#include "camera_servo.h"
#include "controller.h"
#include "freq_response_servo.h"

BallData ballData; // Ball data to be fed to the controller (x,y,t,xr,yr)
uint16_t num_samples = 0;

#define SYNC_BYTES 3
#define PACKET_SIZE_BYTES 39
const uint8_t SYNC_WORD[SYNC_BYTES] = {0x0B, 0x00, 0x0B};

enum Status : uint8_t {
    BALL_LOST = 0,
    TRACKING = 1,
    LEVEL = 2,
    AUTOTUNE = 3,
    AUTOLEVEL = 4,
    SWEEP = 5,
    SERVO_TRIM = 6,
    CALIBRATE_SERVOS = 7,
};

struct CameraPacket {
    uint8_t status;
    uint8_t servo_A_pin;
    uint8_t servo_B_pin;
    uint8_t servo_C_pin;
    float x;
    float y;
    float xr;
    float yr;
    uint32_t frame_ms;
    float gains[3];
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

        offset += sizeof(float);
        memcpy(&packet->frame_ms, buf + offset, sizeof(packet->frame_ms));

        offset += sizeof(packet->frame_ms);
        memcpy(packet->gains, buf + offset, sizeof(packet->gains));

        // Ready to receive next packet
        buf_count = 0;
        return true;
    }

    return false;
}

// Replay the original X and Y sweep without blocking serial input.
void update_servo_sweep(uint32_t elapsed_ms, BallData* data) {
    const uint32_t STEP_MS = 100;
    const int LIMIT = 15;
    const uint32_t AXIS_MS = (4 * LIMIT + 1) * STEP_MS;
    static int previous_step = -1;
    if (elapsed_ms == 0) {
        previous_step = -1;
    }
    int step = 0;
    if (elapsed_ms >= 1000) {
        step = min((elapsed_ms - 1000) / STEP_MS, (2 * AXIS_MS + 500) / STEP_MS) + 1;
    }
    if (elapsed_ms != 0 && step == previous_step) {
        return;
    }
    previous_step = step;

    int x = 0;
    int y = 0;
    if (elapsed_ms >= 1000 && elapsed_ms < 1000 + 2 * AXIS_MS + 500) {
        uint32_t offset = elapsed_ms - 1000;
        const bool y_axis = offset >= AXIS_MS + 500;
        if (y_axis) {
            offset -= AXIS_MS + 500;
        }
        if (offset < AXIS_MS) {
            const int index = offset / STEP_MS;
            const int angle = index <= LIMIT ? -index :
                              index <= 3 * LIMIT ? index - 2 * LIMIT : 4 * LIMIT - index;
            if (y_axis) {
                y = angle;
            } else {
                x = angle;
            }
        }
    }
    setTilt(x, y, data);
}

void setup() {
    Serial.begin(115200);
    servoInit();
}

void loop() {
    static Status active = BALL_LOST;
    static uint32_t sweep_start = 0;
    static uint32_t sweep_id = 0;
    static uint32_t last_command = 0;
    CameraPacket curr_packet;
    CameraPacket latest_packet;
    bool new_data = false;
    bool mode_changed = false;
    bool trim_changed = false;

    // Keep the latest valid measurement while processing sideband trim updates
    // independently of the selected action.
    while (read_serial_port(&curr_packet)) {
        const uint8_t a = curr_packet.servo_A_pin;
        const uint8_t b = curr_packet.servo_B_pin;
        const uint8_t c = curr_packet.servo_C_pin;

        if (curr_packet.status == SERVO_TRIM) {
            trim_changed |= setServoTrim(a, curr_packet.x);
            trim_changed |= setServoTrim(b, curr_packet.y);
            trim_changed |= setServoTrim(c, curr_packet.xr);
            continue;
        }
        if (curr_packet.status > CALIBRATE_SERVOS) {
            continue;
        }

        const Status next = static_cast<Status>(curr_packet.status);
        if (next == TRACKING &&
            (!isfinite(curr_packet.x) || !isfinite(curr_packet.y) ||
             !isfinite(curr_packet.xr) || !isfinite(curr_packet.yr))) {
            continue;
        }

        last_command = millis();
        if (next == SWEEP && (active != SWEEP || curr_packet.frame_ms != sweep_id)) {
            sweep_start = last_command;
            sweep_id = curr_packet.frame_ms;
            mode_changed = true;
        }
        if (active != next) {
            mode_changed = true;
            const bool tracking_or_lost =
                (active == TRACKING || active == BALL_LOST) &&
                (next == TRACKING || next == BALL_LOST);
            if (!tracking_or_lost) {
                num_samples = 0;
            }
            pid_reset();
            active = next;
        }
        if (active == TRACKING) {
            latest_packet = curr_packet;
            new_data = true;
        } else {
            new_data = false;
            mode_changed |= ballData.servo_A_pin != a || ballData.servo_B_pin != b ||
                            ballData.servo_C_pin != c;
            ballData.servo_A_pin = a;
            ballData.servo_B_pin = b;
            ballData.servo_C_pin = c;
        }
    }

    if (active == SWEEP) {
        const uint32_t now = millis();
        if (now - last_command > 500) {
            active = LEVEL;
            setTilt(0, 0, &ballData);
            return;
        }
        update_servo_sweep(mode_changed ? 0 : now - sweep_start, &ballData);
        if (trim_changed) {
            refreshServoTrim(&ballData);
        }
        return;
    }

    switch (active) {
        case LEVEL:
        case CALIBRATE_SERVOS:
            if (mode_changed || trim_changed) {
                setTilt(0, 0, &ballData);
            }
            return;

        case AUTOTUNE:
            set_motor_pins(curr_packet.servo_A_pin,curr_packet.servo_B_pin,curr_packet.servo_C_pin);
            run_sweep_X();
            run_sweep_Y();
            break;
        case AUTOLEVEL:
            // Reserved for future automatic leveling.
            break;

        default:
            break;
    }

    if (trim_changed) {
        refreshServoTrim(&ballData);
    }
    if (!new_data) {
        return;
    }

    if (num_samples > 0) {
        const uint32_t gap = latest_packet.frame_ms - ballData.t[CAMERA_SAMPLES - 1];
        if (gap == 0) {
            return;
        }
        if (gap > 250) {
            // Reset PID state after a long gap, while retaining the valid sample history.
            pid_reset();
        }
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
    ballData.t[CAMERA_SAMPLES - 1] = latest_packet.frame_ms;

    // Only run controller after ballData buffers have filled
    if (num_samples < CAMERA_SAMPLES) {
        num_samples++;
    }

    if (num_samples >= CAMERA_SAMPLES) {
        pid_set_gains(latest_packet.gains[0], latest_packet.gains[1], latest_packet.gains[2]);
        pid_save_integral();
        float thetax = pid_get_xtilt(&ballData);
        float thetay = pid_get_ytilt(&ballData);
        const bool limited = setTilt(thetax, thetay, &ballData);
        if (limited) {
            pid_restore_integral();
        }

        static uint32_t last_log_ms = 0;
        const uint32_t now = millis();
        if (now - last_log_ms >= 250 && Serial.availableForWrite() >= 56) {
            last_log_ms = now;
            // Serial.print(F("dt_ms="));
            // Serial.print(ballData.t[CAMERA_SAMPLES - 1] - ballData.t[CAMERA_SAMPLES - 2]);
            // Serial.print(F(" tilt="));
            // Serial.print(thetax, 2);
            // Serial.print('/');
            // Serial.print(thetay, 2);
            // Serial.print(F(" limited="));
            // Serial.println(limited);
        }
    }
}
