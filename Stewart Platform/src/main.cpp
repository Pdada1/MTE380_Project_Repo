#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>


#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_MIN 0
#define SERVO_MAX 180 

#define SHIELD_ADDR 0x40
Adafruit_PWMServoDriver servo_drive;

// put function declarations here:
//void setServoAngle(uint8_t,uint16_t);

void setup() {
  Serial.begin(9600);
  servo_drive = Adafruit_PWMServoDriver(SHIELD_ADDR);
  servo_drive.setOscillatorFrequency(27000000);
  servo_drive.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

void loop() {
  // put your main code here, to run repeatedly:
}


/*!
 *  @brief  Sets the servo angle for a given servo 
 *  @param  num Servo PWM number from 0 to 15
 *  @param  angle Angle of the servo in degrees
 */
void setServoAngle(uint8_t num, double angle)
{
  double pulselength=1000000;
  pulselength /= SERVO_FREQ; 
  double pulse = (angle/SERVO_MAX)*pulselength;
  servo_drive.writeMicroseconds(num,pulse);
}

//Essentially waiting on the inverse kinematics from Miles. 
/*!
 *  @brief  Calculates the servo angle for given x and y tilt.  
 *  @param  tiltX
 *  @param  tiltY 
 */
