
#include "camera_servo.h"
#include "controller.h"

BallData ballData; //Ball data to be fed to the controller (x,y,t,xr,yr)
RotationData rotationData; //Rotation data for rotation fitting
uint16_t num_samples = 0;

// #define CALIBRATE_SERVOS /* Uncomment to enable manual servo calibration mode */

#ifdef CALIBRATE_SERVOS
Adafruit_PWMServoDriver servo_drive2;
int curr_angle1 = SERVO_A_HORIZ;
int curr_angle2 = SERVO_B_HORIZ;
int curr_angle3 = SERVO_C_HORIZ;


/*!
 *  @brief  Sets the servo angle for a given servo 
 *  @param  servo Servo number 'A'/'B'/'C'
 *  @param  angle_rel_horiz Angle of the servo in degrees relative to the platform
 *  @param  ball_data ballData struct
 */
void setServoAngleManual(uint8_t servo_num, float angle_servo)
{
    angle_servo = constrain(angle_servo, 0.0f, (float)SERVO_RANGE);

    float pulse_us = 500.0f +
        (angle_servo / (float)SERVO_RANGE) * 2000.0f;

    servo_drive2.writeMicroseconds(servo_num, (uint16_t)pulse_us);
}
#endif

void setup() {
    //Laptop + camera init
    Serial.begin(115200);

    //Servo init
#ifdef CALIBRATE_SERVOS
    servo_drive2 = Adafruit_PWMServoDriver(SHIELD_ADDR);
    servo_drive2.begin();
    servo_drive2.setOscillatorFrequency(27000000);
    servo_drive2.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
#else
    servoInit();
#endif
}

void loop() {
#ifdef CALIBRATE_SERVOS
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

#else
    if (camera_data_ready()) {
        static float last_t = 0;
        camera_read_data(&ballData);
        float now = ballData.t[CAMERA_SAMPLES-1];
        if (now - last_t > 0.3f) {            // dropout: buffers hold stale data
            num_samples = 1;                  // wait for CAMERA_SAMPLES fresh samples
            rotationData.num_valid = 0;       // tilt history is stale too: force a refill
            rotationData.shifted_valid = false;
        }
        last_t = now;
        num_samples++;

        //Only run controller after ballData buffers have filled
        if (num_samples >= CAMERA_SAMPLES) {
            float thetax = pid_get_xtilt(&ballData, 0.0f);
            float thetay = pid_get_ytilt(&ballData, 0.0f);
            thetax = constrain(thetax, -PLANE_ANGLE_MAX, PLANE_ANGLE_MAX);
            thetay = constrain(thetay, -PLANE_ANGLE_MAX, PLANE_ANGLE_MAX);

            //Correct the desired tilt for the fitted msx, msy (no-op until the fit is valid), then limit again.
            //The corrected value is what is actually commanded, so it is also what goes into the tilt history / fit.
            if (wahba_compensate_tilt(&rotationData, &thetax, &thetay)) {
                thetax = constrain(thetax, -PLANE_ANGLE_MAX, PLANE_ANGLE_MAX);
                thetay = constrain(thetay, -PLANE_ANGLE_MAX, PLANE_ANGLE_MAX);
            }
            setTilt(thetax, thetay, &ballData);
            update_shifted_tilt(&rotationData, ballData.t[CAMERA_SAMPLES - 1], thetax, thetay);

            // // t, x, y, ax, ay, sin(thetax), sin(thetay)
            // Serial.print(ballData.t[CAMERA_SAMPLES-1], 3);
            // Serial.print(' ');
            // Serial.print(ballData.x[CAMERA_SAMPLES-1], 3);
            // Serial.print(' ');
            // Serial.print(ballData.y[CAMERA_SAMPLES-1], 3);
            // Serial.print(' ');
            // Serial.print(vel_av(&ballData, 'x'), 3);
            // Serial.print(' ');
            // Serial.print(vel_av(&ballData, 'y'), 3);
            // Serial.print(' ');
            // Serial.print(acc_av(&ballData, 'x'), 3);
            // Serial.print(' ');
            // Serial.print(acc_av(&ballData, 'y'), 3);
            // Serial.print(' ');
            // Serial.print(-sin(thetax*PI/180), 3);
            // Serial.print(' ');
            // Serial.print(-sin(thetay*PI/180), 3);
            // Serial.println(' ');

            //Store (acceleration, delay-shifted tilt) and refit msx, msy with Wahba's problem (theta0 = 0)
            wahba_update(&rotationData, &ballData);

            // t, msx, msy, theta0, combined R^2, rms residual, iterations, status (0 not ready, 1 ok, 2 not converged, 3 failed)
            // Serial.print(ballData.t[CAMERA_SAMPLES-1], 3);  Serial.print(' ');
            // Serial.print(rotationData.msx, 6);              Serial.print(' ');
            // Serial.print(rotationData.msy, 6);              Serial.print(' ');
            // // Serial.print(rotationData.r2, 6);               Serial.print(' ');
            // // Serial.print(rotationData.rms, 6);              Serial.print(' ');
            // // Serial.print(rotationData.fit_iters);           Serial.print(' ');
            // // Serial.print((int)rotationData.fit_status);
            // Serial.println();

            // //Raw data in the python tool's 5-column format: t, ax, ay, -sin(thetax_shifted), -sin(thetay_shifted)
            // Serial.print(ballData.t[CAMERA_SAMPLES-1], 3);              Serial.print(' ');
            // Serial.print(acc_av(&ballData, 'x') * ACC_SCALE, 4);        Serial.print(' ');
            // Serial.print(acc_av(&ballData, 'y') * ACC_SCALE, 4);        Serial.print(' ');
            // Serial.print(-sin(rotationData.thetax_shifted*PI/180), 4);  Serial.print(' ');
            // Serial.println(-sin(rotationData.thetay_shifted*PI/180), 4);

            // Serial.print(ballData.t[CAMERA_SAMPLES-1], 3);              Serial.print(' ');
            // Serial.print(ballData.x[CAMERA_SAMPLES-1], 3);              Serial.print(' ');
            // Serial.print(ballData.y[CAMERA_SAMPLES-1], 3);              Serial.print(' ');
            // Serial.print(-sin(thetax*PI/180), 2);                       Serial.print(' ');
            // Serial.print(-sin(thetay*PI/180), 2);                       Serial.print(' ');
            // Serial.print(acc_av(&ballData, 'x') * (S/(float)1000), 4);  Serial.print(' ');
            // Serial.print(acc_av(&ballData, 'y') * (S/(float)1000), 4);  Serial.print(' ');
            // Serial.print(-sin(rotationData.thetax_shifted*PI/180), 2);  Serial.print(' ');
            // Serial.print(-sin(rotationData.thetay_shifted*PI/180), 2);  Serial.print(' ');
            // Serial.print((int)rotationData.fit_status);                 Serial.print(' ');
            // Serial.print(rotationData.rms_deg, 2);                      Serial.print(' ');
            // Serial.print(rotationData.msx, 4);                          Serial.print(' ');
            // Serial.print(rotationData.msy, 4);                          Serial.print(' ');
            // Serial.print(rotationData.theta0, 2);                       Serial.print(' ');
            // unsigned long fit_us = micros() - t0;
            // Serial.println(fit_us);

        }
    }
#endif
}