#include "camera_servo.h"
#include "controller.h"

BallData ballData; //Ball data to be fed to the controller (x,y,t,xr,yr)

void setup() {
    //Laptop + camera init
    Serial.begin(115200);

    //Servo init
    servo_drive = Adafruit_PWMServoDriver(SHIELD_ADDR);
    servo_drive.begin();
    servo_drive.setOscillatorFrequency(27000000);
    servo_drive.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

void loop() {
    if (camera_data_ready()) {
        camera_read_data(&ballData);
        float thetax = get_xtilt_PID(&ballData);
        float thetay = get_ytilt_PID(&ballData);
        setTilt(thetax, thetay);
    }
}