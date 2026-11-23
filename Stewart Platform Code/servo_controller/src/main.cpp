#include "camera_servo.h"
#include "controller.h"
#include "freq_response_servo.h"

BallData ballData; // Ball data to be fed to the controller (x,y,t,xr,yr)

// Counts valid samples in the tracking history, up to CAMERA_SAMPLES.
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
    SWEEP_DATA = 8,   // ball/target stream the PC sends during the autotune sweep
};

// The fields after the sync word, in the same order as the Python packet.
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

// Reads packets from serial. Single-byte PC keys (PROCEED_CHAR, RERUN_CHAR) that arrive
// between packets are stored in *key (if provided) instead of entering the packet buffer.
bool read_serial_port(CameraPacket *packet, uint8_t *key = nullptr) {
    // Static so it keeps partial packets between calls in case the 
    // rest of the bytes arrive later.
    static uint8_t buf[PACKET_SIZE_BYTES] = {0};
    static size_t buf_count = 0;

    while (Serial.available() > 0) {
        const uint8_t incoming = (uint8_t)Serial.read();
        // Single-byte PC keys only count between packets, so a 0x0A inside a packet is safe.
        if (buf_count == 0 &&
            (incoming == PROCEED_CHAR || incoming == RERUN_CHAR || incoming == STOP_CHAR)) {
            if (key != nullptr && *key != STOP_CHAR) {   // a stop request is never overwritten
                *key = incoming;
            }
            continue;
        }
        buf[buf_count++] = incoming;
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

        // Skip the sync bytes, status, and three servo pins to get to the floats.
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

static constexpr uint32_t SWEEP_STEP_MS = 100;
static constexpr int SWEEP_LIMIT = 15;
static constexpr uint32_t SWEEP_AXIS_MS = (4 * SWEEP_LIMIT + 1) * SWEEP_STEP_MS;
static constexpr uint32_t SWEEP_DURATION_MS = 1000 + 2 * SWEEP_AXIS_MS + 500;

// Replay the original X and Y sweep without blocking serial input.
void update_servo_sweep(uint32_t elapsed_ms, BallData* data) {
    static int previous_step = -1;
    if (elapsed_ms == 0) {
        previous_step = -1;
    }
    // Hold level for the first second and advance one step every 100 ms after that.
    int step = 0;
    if (elapsed_ms >= 1000) {
        step = min((elapsed_ms - 1000) / SWEEP_STEP_MS, (2 * SWEEP_AXIS_MS + 500) / SWEEP_STEP_MS) + 1;
    }
    // Only send a new tilt when the sweep reaches a new step or restarts.
    if (elapsed_ms != 0 && step == previous_step) {
        return;
    }
    previous_step = step;

    // Sweep X first, pause at level for 500 ms, then sweep Y and return to level.
    int x = 0;
    int y = 0;
    if (elapsed_ms >= 1000 && elapsed_ms < SWEEP_DURATION_MS) {
        uint32_t offset = elapsed_ms - 1000;
        const bool y_axis = offset >= SWEEP_AXIS_MS + 500;
        if (y_axis) {
            offset -= SWEEP_AXIS_MS + 500;
        }
        if (offset < SWEEP_AXIS_MS) {
            const int index = offset / SWEEP_STEP_MS;
            // For each axis, move from 0 to -LIMIT to +LIMIT and back to 0.
            const int angle = index <= SWEEP_LIMIT ? -index :
                              index <= 3 * SWEEP_LIMIT ? index - 2 * SWEEP_LIMIT : 4 * SWEEP_LIMIT - index;
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

/**
 * Refreshes the servo pins based on the incoming data from the computer
 */
static bool update_servo_pins(const CameraPacket& packet) {
    const bool changed = ballData.servo_A_pin != packet.servo_A_pin ||
                         ballData.servo_B_pin != packet.servo_B_pin ||
                         ballData.servo_C_pin != packet.servo_C_pin;
    ballData.servo_A_pin = packet.servo_A_pin;
    ballData.servo_B_pin = packet.servo_B_pin;
    ballData.servo_C_pin = packet.servo_C_pin;
    return changed;
}

/**
 * Shift a tracking packet into the sample history. Returns false for a repeated frame.
 */
static bool ingest_tracking(const CameraPacket& packet) {
    if (num_samples > 0) {
        const uint32_t gap = packet.frame_ms - ballData.t[CAMERA_SAMPLES - 1];
        // The PC can resend its last frame. Only use it once in the PID history.
        if (gap == 0) {
            return false;
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
    ballData.status = packet.status;
    update_servo_pins(packet);

    ballData.x[CAMERA_SAMPLES - 1] = packet.x;
    ballData.y[CAMERA_SAMPLES - 1] = packet.y;
    ballData.xr[CAMERA_SAMPLES - 1] = packet.xr;
    ballData.yr[CAMERA_SAMPLES - 1] = packet.yr;
    ballData.t[CAMERA_SAMPLES - 1] = packet.frame_ms;

    // Only run controller after ballData buffers have filled
    if (num_samples < CAMERA_SAMPLES) {
        num_samples++;
    }
    return true;
}

/**
 * Update the ball tracking state by refreshing camera samples using the latest packet
 * and tilting the platform accordingly
 */
static void update_tracking(const CameraPacket& packet) {
    if (!ingest_tracking(packet) || num_samples < CAMERA_SAMPLES) {
        return;
    }
    pid_set_gains(packet.gains[0], packet.gains[1], packet.gains[2]);
    // Save the integral so we can undo this step if the tilt hits a limit.
    pid_save_integral();
    float thetax = pid_get_xtilt(&ballData);
    float thetay = pid_get_ytilt(&ballData);
    const bool limited = setTilt(thetax, thetay, &ballData);
    if (limited) {
        pid_restore_integral();
    }
}

/**
 * Used by the blocking sweep loops. Reads serial, feeds the controller with SWEEP_DATA
 * packets, and runs PID on the requested axes ('x', 'y' or 'b' for both), writing the
 * tilts into *tx / *ty. Sets *updated when a new sample was processed.
 * Returns a PC key byte (PROCEED_CHAR or RERUN_CHAR) or 0.
 */
uint8_t sweep_service(char axes, float* tx, float* ty, bool* updated) {
    uint8_t key = 0;
    *updated = false;
    CameraPacket packet;
    while (read_serial_port(&packet, &key)) {
        if (packet.status != SWEEP_DATA ||
            !isfinite(packet.x) || !isfinite(packet.y) ||
            !isfinite(packet.xr) || !isfinite(packet.yr)) {
            continue;
        }
        if (!ingest_tracking(packet) || num_samples < CAMERA_SAMPLES) {
            continue;
        }
        pid_set_gains(packet.gains[0], packet.gains[1], packet.gains[2]);
        if (axes != 'y') {
            *tx = pid_get_xtilt(&ballData);
        }
        if (axes != 'x') {
            *ty = pid_get_ytilt(&ballData);
        }
        *updated = true;
    }
    return key;
}


void loop() {
    // Keep the current operating mode and sweep timing between calls to loop().
    static Status active = BALL_LOST;
    static uint32_t sweep_start = 0;
    static uint32_t sweep_id = 0;

    CameraPacket packet;
    // Pending measurement for this pass through loop(). Each valid tracking packet replaces it.
    CameraPacket latest_tracking;
    bool new_tracking = false;
    bool mode_changed = false;
    bool trim_changed = false;

    // Read all available packets before running the platform. The last accepted mode wins.
    while (read_serial_port(&packet)) {
        const Status next = static_cast<Status>(packet.status);
        bool restart_sweep = false;

        /*
        'next' is the status in the current incoming packet. 'active' is the currently-running mode.

        - switch (next) runs for each packet. It decides whether to accept the packet and what data to keep. 
          For example, a valid TRACKING packet replaces the pending ball measurement. A SERVO_TRIM packet 
          updates the trims and leaves active alone.

        - switch (active) runs once after the serial queue is drained. It decides what the platform should do now. 
          Tracking uses the latest valid measurement, and a sweep can advance even if no new packet arrived.
        
          For example, if two TRACKING packets arrive back to back, switch(next) reads both and keeps 
          the newer measurement. The second switch, switch(active) runs PID once using that measurement.
        */

        // Queue the next actions according to the incoming packet type
        switch (next) {
            case SERVO_TRIM:
                // Trim packets leave the active mode alone.
                trim_changed |= setServoTrim(packet.servo_A_pin, packet.x);
                trim_changed |= setServoTrim(packet.servo_B_pin, packet.y);
                trim_changed |= setServoTrim(packet.servo_C_pin, packet.xr);
                continue;

            case TRACKING:
                // Ignore invalid coordinates and keep any valid measurement already received.
                if (!isfinite(packet.x) || !isfinite(packet.y) ||
                    !isfinite(packet.xr) || !isfinite(packet.yr)) {
                    continue;
                }
                latest_tracking = packet;
                new_tracking = true;
                break;

            case SWEEP:
                // The PC uses frame_ms as a sweep ID. Repeated IDs keep the current sweep going.
                if (active != SWEEP || packet.frame_ms != sweep_id) {
                    sweep_id = packet.frame_ms;
                    mode_changed = true;
                    restart_sweep = true;
                }
                // Fall through to update the servo mapping.
            case BALL_LOST:
            case LEVEL:
            case AUTOTUNE:
            case AUTOLEVEL:
            case CALIBRATE_SERVOS:
                // These modes do not use ball positions. Drop any pending tracking update
                // from this pass through loop(), but keep the servo pin mapping from the packet.
                new_tracking = false;
                mode_changed |= update_servo_pins(packet);
                break;

            default:
                // Includes SWEEP_DATA, which is only used while an autotune sweep is running.
                continue;
        }

        if (restart_sweep) {
            sweep_start = millis();
        }
        if (active != next) {
            // Packet type changed, must adjust action
            mode_changed = true;
            const bool tracking_or_lost = (active == TRACKING || active == BALL_LOST) && (next == TRACKING || next == BALL_LOST);
            if (!tracking_or_lost) {
                // Keep the history when the ball disappears or returns. Start fresh
                // when switching to or from another operating mode.
                num_samples = 0;
            }
            pid_reset();
            active = next;
        }
    }

    // Apply only the active mode. Sweep can advance without new serial data.
    switch (active) {
        case TRACKING:
            if (trim_changed) {
                refreshServoTrim(&ballData);
            }
            if (new_tracking) {
                update_tracking(latest_tracking);
            }
            break;

        case BALL_LOST:
            // Hold the last tilt and apply any new trims while waiting for the ball.
            if (trim_changed) {
                refreshServoTrim(&ballData);
            }
            break;

        case LEVEL:
        case CALIBRATE_SERVOS:
            // Hold neutral tilt and update it if the mode, pin mapping, or trims change.
            if (mode_changed || trim_changed) {
                setTilt(0, 0, &ballData);
            }
            break;

        case AUTOTUNE: {
            // Run both axes from one command without waiting for a serial prompt.
            const Mode previous_mode = current_mode;
            current_mode = Mode::HUMAN_IN_LOOP;

            sweep_aborted = false;
            set_motor_pins(ballData.servo_A_pin, ballData.servo_B_pin, ballData.servo_C_pin);
            run_sweep_X();
            if (!sweep_aborted) {
                run_sweep_Y();
            }
            Serial.println(sweep_aborted ? "SWEEP_ABORTED" : "SWEEP_DONE");

            current_mode = previous_mode;

            // Done autotuning
            active = LEVEL;
            setTilt(0, 0, &ballData);
            break;
        }

        case AUTOLEVEL:
            // Automatic leveling is still a placeholder.

             /* Add future code right here */

            active = LEVEL;
            setTilt(0, 0, &ballData);
            break;

        case SWEEP: {
            const uint32_t now = millis();
            // A single command runs the full sweep, then leaves the platform level.
            if (now - sweep_start >= SWEEP_DURATION_MS) {
                active = LEVEL;
                setTilt(0, 0, &ballData);
                break;
            }
            update_servo_sweep(mode_changed ? 0 : now - sweep_start, &ballData);
            if (trim_changed) {
                refreshServoTrim(&ballData);
            }
            break;
        }

        case SERVO_TRIM:
        case SWEEP_DATA:
            // Handled on receipt and never selected as an active mode.
            break;
    }
}