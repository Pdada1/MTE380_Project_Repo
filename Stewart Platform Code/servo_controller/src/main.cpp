
#include "camera_servo.h"
#include "camera_modes.h"
#include "controller.h"

BallData ballData; //Ball data to be fed to the controller (x,y,t,xr,yr)
uint16_t num_samples = 0;

void setup() {
    //Laptop + camera init
    Serial.begin(115200);

    servoInit();
}

void loop() {
    if (camera_data_ready()) {
        camera_read_data(&ballData);
        num_samples++;

        //Only run controller after ballData buffers have filled
        if (num_samples >= CAMERA_SAMPLES) {
            float thetax = pid_get_xtilt(&ballData);
            float thetay = pid_get_ytilt(&ballData);
            setTilt(thetax, thetay, &ballData);
            
            // Serial.print(thetax);
            // Serial.print(' ');
            // Serial.println(thetay);
        }
    }

    // Handle alternative packet modes, e.g. level, autotune, and autolevel
    CameraPacket command;
    if (camera_take_command(command)) {
        handle_camera_command(command, ballData);
    }
}