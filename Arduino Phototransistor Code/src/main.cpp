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
float Kp = 8.0;
float Ki = 0.0;
float Kd = 0.0;

float integral = 0.0;
float previousError = 0.0;
unsigned long previousTime = 0;

/* Calibrated Servo Parameters */
const float SERVO_CENTER = 90.0;
const float SERVO_MIN = 40.0;
const float SERVO_MAX = 130.0;


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
    integral = constrain(integral, -5.0, 5.0);

    // Derivative
    float derivative = (error - previousError) / dt;
    previousError = error;

    float output =
        Kp * error +
        Ki * integral +
        Kd * derivative;

    float servoCommand = SERVO_CENTER + output;

    servoCommand = constrain(
        servoCommand,
        SERVO_MIN,
        SERVO_MAX
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
    servo.write(SERVO_CENTER);

    previousTime = micros();
}


void loop() {

    read_sensor_values();
    float position = estimate_ball_position();
    // print_sensor_values();
    servo_PID_move(position);

    delay(20);
}