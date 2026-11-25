 
#include "camera_servo.h"
#include "camera_modes.h"
#include "controller.h"
#include "autotune.h"
#include "alternate_tracking.h"
#include "autolevel/controller.h"

BallData ballData; //Ball data to be fed to the controller (x,y,t,xr,yr)
uint16_t num_samples = 0;
static bool autolevel_active = false;

void setup() {
    //Laptop + camera init
    Serial.begin(115200);
 
    servoInit();
}

static void apply_alternate_tilt(float x, float y) {
    setTilt(x, y, &ballData);
}

static void track_alternate(const CameraPacket& packet) {
    // Time alternate control updates on the Arduino.
    const uint32_t received_ms = millis();
    static const uint32_t clock_origin = received_ms;
    static uint32_t previous_frame = 0;
    static PacketStatus previous_mode = PacketStatus::BallLost;
    if (packet.status == previous_mode && packet.timestamp == previous_frame) {
        return;
    }
    previous_frame = packet.timestamp;
    previous_mode = packet.status;

    ballData.servo_A_pin = packet.servo_pins[0];
    ballData.servo_B_pin = packet.servo_pins[1];
    ballData.servo_C_pin = packet.servo_pins[2];
    const AlternateFrame frame = {
        packet.x, packet.y, packet.xr, packet.yr,
        (received_ms - clock_origin) / 1000.0f,
        packet.timestamp
    };
    if (packet.status == PacketStatus::Autorotation) {
        autorotate::track(frame, apply_alternate_tilt);
    } else {
        autotilt::track(frame, apply_alternate_tilt);
    }
}

void loop() {
    if (camera_data_ready()) {
        const CameraPacket frame = latest_camera_packet;
        camera_read_data(&ballData);
        if (frame.status == PacketStatus::Autolevel) {
            if (!autolevel_active) {
                autolevel::reset();
                autolevel_active = true;
            }
            autolevel::track(ballData, apply_alternate_tilt);
        } else {
            if (autolevel_active) {
                autolevel::reset();
                autolevel_active = false;
            }
            num_samples++;
          
            // Run autotune routine: checks if autotune PID parameters should be used
            // FIXME make it so that use_autotune is always false if Autotune_MM is currently not active
            bool use_autotune = autotune_routine(&ballData);


            //Only run the regular controller for ordinary tracking frames.
            if (num_samples >= CAMERA_SAMPLES) {
                float thetax = pid_get_xtilt(&ballData, use_autotune);
                float thetay = pid_get_ytilt(&ballData, use_autotune);
                setTilt(thetax, thetay, &ballData);
            }
        }
    }

    // Handle one-shot camera commands.
    CameraPacket command;
    if (camera_take_command(command)) {
        autolevel::reset();
        autolevel_active = false;
        if (command.status == PacketStatus::Autorotation || command.status == PacketStatus::Autotilt) {
            track_alternate(command);
        } 
        else {
            handle_camera_command(command, ballData);
        }
    }
}