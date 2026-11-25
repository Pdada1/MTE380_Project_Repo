#include "camera_modes.h"
#include "freq_response_servo.h"
#include "controller.h"
#include "autotune.h"

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
        case PacketStatus::ServoTrim:
            apply_servo_trims(packet, ballData);
            autotune_set_rise_time(packet.yr);     // trim packet: yr = desired rise time (ms)
            autotune_set_overshoot(packet.kp);     // trim packet: kp = max overshoot (%)
            autotune_set_e_ss(packet.ki);          // trim packet: ki = max steady state error
            break;
        case PacketStatus::Autotune: {
            set_command_channels(packet, ballData);
            set_motor_pins(ballData.servo_A_pin, ballData.servo_B_pin, ballData.servo_C_pin);

            // Run both axes from one command without waiting for a PC prompt.
            const Mode previous_mode = current_mode;
            current_mode = Mode::TRANSIENT_IT;
            run_sweep_X();
            run_sweep_Y();
            current_mode = previous_mode;
            hold_level(ballData);
            break;
        }
        case PacketStatus::Autolevel:
            break;
        default:
            break;
    }
}



