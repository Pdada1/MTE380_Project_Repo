#include <Arduino.h>
#include <Servo.h>

const size_t NUM_SENSORS = 8;
const float TARGET = 3.5;
const int SAMPLES = 4;

const int pinA = A0;
const int pinB = A1;
const int pinC = A2;
const int analog_in = A3;

uint16_t sensorValues[NUM_SENSORS];

Servo servo;
const int SERVO_PIN = 5;

/* PID parameters */
float Kp = 5.0;
float Ki = 1.0;
float Kd = 3.0;

float integral = 0.0;
float previousError = 0.0;
unsigned long previousTime = 0;

/* Servo Parameters */
float servoCenter = 135.0;
float servoMin = 110.0;
float servoMax = 160.0;

/**
 * Calibrates the servo center/min/max positions according to the hardware setup
 */
void calibrate_servo_limit(float *angle) {
    Serial.println("Use A/G to move fast, S/F to move slow, and E to save the value");
    servo.write(*angle);

    while (1) {
        if (Serial.available() > 0) {
            char key = Serial.read();

            if (key == 'a' || key == 'A') {
                *angle -= 10;
            }
            if (key == 'g' || key == 'G') {
                *angle += 10;
            }
            if (key == 's' || key == 'S') {
                *angle -= 1;
            }
            if (key == 'f' || key == 'F') {
                *angle += 1;
            }
            if (key == 'e' || key == 'E') {
                break;
            }

            *angle = constrain(*angle, 0, 180);
            servo.write(*angle);

            Serial.print("Angle: ");
            Serial.println(*angle);
        }
    }

}


/**
 * Prints sensor values for debugging purposes
 */
void print_sensor_values(void) {
    for (size_t sensor = 0; sensor < NUM_SENSORS; sensor++) {
        Serial.print(sensorValues[sensor]);

        if (sensor < NUM_SENSORS - 1) {
            Serial.print('\t');
        }
    }

    Serial.println();
}


/**
 * Reads one value from a selected mux channel.
 */
uint16_t read_mux_sensor_once(size_t sensor) {
    digitalWrite(pinA, (sensor >> 0) & 0x01);
    digitalWrite(pinB, (sensor >> 1) & 0x01);
    digitalWrite(pinC, (sensor >> 2) & 0x01);

    delayMicroseconds(200);

    // Discard first ADC conversion after switching mux channel
    analogRead(analog_in);

    return analogRead(analog_in);
}


/**
 * Reads all sensors over multiple complete sweeps,
 * then averages each sensor's readings.
 */
void read_sensor_values(void) {
    uint32_t totals[NUM_SENSORS] = {0};

    // Take complete 8-sensor sweeps
    for (int sample = 0; sample < SAMPLES; sample++) {

        for (size_t sensor = 0; sensor < NUM_SENSORS; sensor++) {
            totals[sensor] += read_mux_sensor_once(sensor);
        }
    }

    // Average each sensor across all sweeps
    for (size_t sensor = 0; sensor < NUM_SENSORS; sensor++) {
        sensorValues[sensor] = totals[sensor] / SAMPLES;
    }
}


/**
 * Estimates the ball's current location using sensor readings.
 */
float estimate_ball_position(void) {

    // Find strongest sensor
    size_t strongest = 0;
    for (size_t i = 1; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > sensorValues[strongest]) {
            strongest = i;
        }
    }

    // Handle left edge
    if (strongest == 0) {
        float a = sensorValues[0];
        float b = sensorValues[1];

        return b / (a + b);
    }

    // Handle right edge
    if (strongest == NUM_SENSORS - 1) {
        float a = sensorValues[NUM_SENSORS - 2];
        float b = sensorValues[NUM_SENSORS - 1];

        return (NUM_SENSORS - 2) + b / (a + b);
    }

    // Compare neighbors of strongest sensor
    size_t left = strongest - 1;
    size_t right = strongest + 1;

    if (sensorValues[left] > sensorValues[right]) {

        // Interpolate between left and strongest
        float a = sensorValues[left];
        float b = sensorValues[strongest];

        return left + b / (a + b);
    }
    else {

        // Interpolate between strongest and right
        float a = sensorValues[strongest];
        float b = sensorValues[right];

        return strongest + b / (a + b);
    }
}


/**
 * PID controller for servo position; aims to move the ball to TARGET.
 */
void servo_PID_move(float position) {
    unsigned long now = micros();

    float dt = (now - previousTime) / 1000000.0;
    previousTime = now;

    if (dt <= 0.0) {
        // No time delta, not ready to move yet
        return;
    }

    float error = TARGET - position;

    // Integral
    integral += error * dt;

    // Derivative
    float derivative = (error - previousError) / dt;
    previousError = error;

    float output =
        Kp * error +
        Ki * integral +
        Kd * derivative;

    float servoCommand = servoCenter + output;

    servoCommand = constrain(
        servoCommand,
        servoMin,
        servoMax
    );

    servo.write(servoCommand);

    Serial.print("Position: ");
    Serial.print(position);

    Serial.print("\tError: ");
    Serial.print(error);

    Serial.print("\tServo: ");
    Serial.println(servoCommand);
}


void setup() {
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pinMode(pinC, OUTPUT);
    pinMode(analog_in, INPUT);

    Serial.begin(115200);

    servo.attach(SERVO_PIN);
    servo.write(servoCenter);

    previousTime = micros();

    // calibrate_servo_limit(&servoCenter);
    // calibrate_servo_limit(&servoMin);
    // calibrate_servo_limit(&servoMax);
}

/**
 * Provides a keyboard interface to tune the PID values
 */
void tune_PID(float increment) {
    if (Serial.available() > 0) {
        char key = Serial.read();
        if (key == 'q') {
            Kp += increment;
        }
        if (key == 'w') {
            Ki += increment;
        }
        if (key == 'e') {
            Kd += increment;
        }
        if (key == 'a') {
            Kp -= increment;
        }
        if (key == 's') {
            Ki -= increment;
        }
        if (key == 'd') {
            Kd -= increment;
        }

        Serial.print("PID: ");
        Serial.print(Kp);
        Serial.print(" ");
        Serial.print(Ki);
        Serial.print(" ");
        Serial.println(Kd);
    }
}


void loop() {
    read_sensor_values();
    float position = estimate_ball_position();
    // print_sensor_values();
    servo_PID_move(position);
    tune_PID(0.2);

    delay(50);
}