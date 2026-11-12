#include "camera_servo.h"
#include "controller.h"

BallData ballData; //Ball data to be fed to the controller (x,y,t,xr,yr)
uint16_t num_samples = 0;
float thetay = 0;
float thetax = 0;
int ready = 0;

void setup() {
    //Laptop + camera init
    Serial.begin(115200);

    //Servo init
    servoInit();

    // set servos to be at neutral angle
    setTilt(thetax, thetay);
}

void loop() {

    if (Serial.available() >0) {
        
        Serial.print("Enter theta_x (degrees): ");
        String thx = Serial.readStringUntil('\n');
        thx.trim();

        if (thx.length() > 0) {
            thetax = thx.toFloat();
            Serial.print(" Parsed float theta_x: ");
            Serial.println(thetax, 4);
            ready |= 0x1;
        }

        Serial.print("Enter theta_y (degrees): ");
        String thy = Serial.readStringUntil('\n');
        thy.trim();

        if (thy.length() > 0) {
            thetay = thy.toFloat();
            Serial.print(" Parsed float theta_y: ");
            Serial.println(thetay, 4);
            ready |= 0x2;
        }

        // Determine neutral and limits for each motor
        // angle parameter in setServoAngle() relative to horizontal of l_1 l_2 l_3 

        // float thetax = pid_get_xtilt(&ballData);
        //float thetay = pid_get_ytilt(&ballData);

        if (ready == 0x3) {
            setTilt(thetax, thetay);
            ready = 0;
        }
    }
    
}