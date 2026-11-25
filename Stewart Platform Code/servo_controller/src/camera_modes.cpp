#include "camera_modes.h"
#include "freq_response_servo.h"
#include "controller.h"

static constexpr uint32_t SWEEP_STEP_MS = 100;

// Commands can arrive before the first tracking packet supplies servo channels.
static void set_command_channels(const CameraPacket& packet, BallData& ballData) {
    ballData.servo_A_pin = packet.servo_pins[0];
    ballData.servo_B_pin = packet.servo_pins[1];
    ballData.servo_C_pin = packet.servo_pins[2];
}

// Keeps the platform at the level position
static void hold_level(BallData& ballData) {
    setTilt(0, 0, &ballData);
}

// Move from zero to negative limit, then positive limit, then back to zero.
static int sweep_angle(int step) {
    if (step <= PLANE_ANGLE_MAX) {
        return -step;
    }
    if (step <= 3 * PLANE_ANGLE_MAX) {
        return step - 2 * PLANE_ANGLE_MAX;
    }
    return 4 * PLANE_ANGLE_MAX - step;
}

static void run_sweep(BallData& ballData) {
    constexpr int last_step = 4 * PLANE_ANGLE_MAX;

    hold_level(ballData);
    delay(1000);
    for (int step = 0; step <= last_step; ++step) {
        setTilt(sweep_angle(step), 0, &ballData);
        delay(SWEEP_STEP_MS);
    }

    delay(500);
    for (int step = 0; step <= last_step; ++step) {
        setTilt(0, sweep_angle(step), &ballData);
        delay(SWEEP_STEP_MS);
    }
    hold_level(ballData);
}

// ---------------------------------------------------------------------------
// Autotune sweep service
//
// While a frequency sweep blocks in run_sweep(), loop() is not running, so this
// reads the serial port itself. It picks up the PC's single-byte keys
// (PROCEED_CHAR, RERUN_CHAR, STOP_CHAR) and the SweepData ball stream, and runs
// PID on the axis the sweep is not moving so the ball stays on the platform.
// ---------------------------------------------------------------------------
static constexpr size_t SYNC_BYTES = 3;
static const uint8_t SYNC_WORD[SYNC_BYTES] = {0x0B, 0x00, 0x0B};
static constexpr size_t PACKET_BYTES = SYNC_BYTES + sizeof(CameraPacket);

static uint8_t sweep_buf[PACKET_BYTES];
static size_t sweep_buf_count = 0;
static BallData sweep_pid_data;        // sample history used by the PID while sweeping
static uint8_t sweep_samples = 0;      // valid samples in sweep_pid_data, up to CAMERA_SAMPLES

void sweep_service_begin(const CameraPacket& command) {
    sweep_pid_data = BallData();
    sweep_pid_data.servo_A_pin = command.servo_pins[0];
    sweep_pid_data.servo_B_pin = command.servo_pins[1];
    sweep_pid_data.servo_C_pin = command.servo_pins[2];
    sweep_samples = 0;
    sweep_buf_count = 0;
    pid_reset();
}

// Read bytes until a full packet arrives. A key byte between packets is stored in key
// (a stop request is never overwritten) and never enters the packet buffer.
static bool sweep_read_packet(CameraPacket& packet, uint8_t& key) {
    while (Serial.available() > 0) {
        const uint8_t incoming = (uint8_t)Serial.read();
        if (sweep_buf_count == 0 &&
            (incoming == PROCEED_CHAR || incoming == RERUN_CHAR || incoming == STOP_CHAR)) {
            if (key != STOP_CHAR) {
                key = incoming;
            }
            continue;
        }
        sweep_buf[sweep_buf_count++] = incoming;
        if (sweep_buf_count >= SYNC_BYTES && memcmp(sweep_buf, SYNC_WORD, SYNC_BYTES) != 0) {
            // Not a sync word, drop the oldest byte and keep looking.
            memmove(sweep_buf, sweep_buf + 1, --sweep_buf_count);
            continue;
        }
        if (sweep_buf_count < PACKET_BYTES) {
            continue;
        }
        memcpy(&packet, sweep_buf + SYNC_BYTES, sizeof(packet));
        sweep_buf_count = 0;
        return true;
    }
    return false;
}

uint8_t sweep_service(char axes, float* tx, float* ty, bool* updated) {
    uint8_t key = 0;
    *updated = false;
    CameraPacket packet;
    while (sweep_read_packet(packet, key)) {
        if (packet.status == PacketStatus::ServoTrim) {
            // Keep trims working during a sweep.
            apply_servo_trims(packet, sweep_pid_data);
            continue;
        }
        if (packet.status != PacketStatus::SweepData ||
            !isfinite(packet.x) || !isfinite(packet.y) ||
            !isfinite(packet.xr) || !isfinite(packet.yr)) {
            continue;
        }

        if (sweep_samples > 0) {
            const uint32_t gap = packet.timestamp - sweep_pid_data.frame_ms[CAMERA_SAMPLES - 1];
            if (gap == 0) {
                continue;       // the PC resends its last frame, use it once
            }
            if (gap > 250) {
                pid_reset();    // long gap, start the integral again
            }
        }

        for (int i = 0; i < CAMERA_SAMPLES - 1; i++) {
            sweep_pid_data.x[i] = sweep_pid_data.x[i + 1];
            sweep_pid_data.y[i] = sweep_pid_data.y[i + 1];
            sweep_pid_data.xr[i] = sweep_pid_data.xr[i + 1];
            sweep_pid_data.yr[i] = sweep_pid_data.yr[i + 1];
            sweep_pid_data.frame_ms[i] = sweep_pid_data.frame_ms[i + 1];
        }
        sweep_pid_data.servo_A_pin = packet.servo_pins[0];
        sweep_pid_data.servo_B_pin = packet.servo_pins[1];
        sweep_pid_data.servo_C_pin = packet.servo_pins[2];
        sweep_pid_data.x[CAMERA_SAMPLES - 1] = packet.x;
        sweep_pid_data.y[CAMERA_SAMPLES - 1] = packet.y;
        sweep_pid_data.xr[CAMERA_SAMPLES - 1] = packet.xr;
        sweep_pid_data.yr[CAMERA_SAMPLES - 1] = packet.yr;
        sweep_pid_data.frame_ms[CAMERA_SAMPLES - 1] = packet.timestamp;
        sweep_pid_data.kp = packet.kp;
        sweep_pid_data.ki = packet.ki;
        sweep_pid_data.kd = packet.kd;

        if (sweep_samples < CAMERA_SAMPLES) {
            sweep_samples++;
        }
        if (sweep_samples < CAMERA_SAMPLES) {
            continue;
        }

        // Each call advances that axis' integral, so only call the axes being held.
        if (axes != 'y') {
            *tx = pid_get_xtilt(&sweep_pid_data);
        }
        if (axes != 'x') {
            *ty = pid_get_ytilt(&sweep_pid_data);
        }
        *updated = true;
    }
    return key;
}

// Run the frequency sweep on the requested axes ('x', 'y' or 'b' for both) and then hold
// level. only_freq > 0 runs just that frequency instead of every frequency.
static void run_frequency_sweeps(const CameraPacket& packet, BallData& ballData,
                                 char axes, float only_freq) {
    set_command_channels(packet, ballData);
    set_motor_pins(ballData.servo_A_pin, ballData.servo_B_pin, ballData.servo_C_pin);
    sweep_service_begin(packet);

    // The PC prompts before each frequency: Space = go, R = repeat the previous one.
    // PID keeps the ball centred on the axis that is not being swept.
    const Mode previous_mode = current_mode;
    current_mode = Mode::HUMAN_IN_LOOP;
    sweep_aborted = false;
    if (axes != 'y') {
        run_sweep_X(only_freq);
    }
    if (!sweep_aborted && axes != 'x') {
        run_sweep_Y(only_freq);
    }
    Serial.println(sweep_aborted ? "SWEEP_ABORTED" : "SWEEP_DONE");
    current_mode = previous_mode;
    hold_level(ballData);
}

void handle_camera_command(const CameraPacket& packet, BallData& ballData) {
    // Tracking and ball loss are handled by the camera path.
    switch (packet.status) {
        case PacketStatus::Level:
            set_command_channels(packet, ballData);
            hold_level(ballData);
            break;
        case PacketStatus::Sweep:
            set_command_channels(packet, ballData);
            run_sweep(ballData);
            break;
        case PacketStatus::IdentifyServos:
            identify_servo_pins(packet);
            break;
        case PacketStatus::ServoTrim:
            apply_servo_trims(packet, ballData);
            break;
        case PacketStatus::Autotune:
            run_frequency_sweeps(packet, ballData, 'b', 0.0f);
            break;
        case PacketStatus::FreqRun: {
            // One frequency on one axis. The PC puts the axis in x (1 = X, 2 = Y)
            // and the frequency in Hz in y.
            const int axis_code = isfinite(packet.x) ? (int)(packet.x + 0.5f) : 0;
            const float freq = isfinite(packet.y) ? packet.y : 0.0f;
            if ((axis_code != 1 && axis_code != 2) || freq <= 0.0f) {
                Serial.println("BAD_REQUEST");
                Serial.println("SWEEP_ABORTED");
                break;
            }
            run_frequency_sweeps(packet, ballData, axis_code == 1 ? 'x' : 'y', freq);
            break;
        }
        case PacketStatus::Autolevel:
            break;
        default:
            break;
    }
}