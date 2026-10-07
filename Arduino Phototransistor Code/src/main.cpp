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
float servoCenter = 135;
float servoMin = 110;
float servoMax = 145;

/* Phototransistor params - Store a ambient calibration for every valid servo angle*/
int ambientReading[360][NUM_SENSORS] = {0};

/**
 * Calibrates the servo center/min/max positions according to the hardware setup
 */
void calibrate_servo_limit(float *angle, char *text) {
    Serial.println("---------------------------------------------------------------");
    Serial.println(text);
    Serial.println("Use A/G to move fast, S/F to move slow, and E to save the value");
    Serial.println("---------------------------------------------------------------");

    servo.write(*angle);
    delay(20);

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
 * Prints ambient values at certain servo Angle for debugging purposes
 */
void print_ambient_values(int servoAngle) {
    for (size_t sensor = 0; sensor < NUM_SENSORS; sensor++) {
        Serial.print(ambientReading[servoAngle][sensor]);

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
float estimate_ball_position(int servoAngle) {

    // Find strongest sensor
    size_t strongest = 0;
    for (size_t i = 1; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > sensorValues[strongest]) {
            strongest = i;
        }
    }

    // Handle left edge
    if (strongest == 0) {
        float a = sensorValues[0] - ambientReading[servoAngle][0];
        float b = sensorValues[1] - ambientReading[servoAngle][1];
        return b/(b+a);
    }

    // Handle right edge
    if (strongest == NUM_SENSORS - 1) {
        float a = sensorValues[NUM_SENSORS-1] - ambientReading[servoAngle][NUM_SENSORS-1];
        float b = sensorValues[NUM_SENSORS-2] - ambientReading[servoAngle][NUM_SENSORS-2];
        return NUM_SENSORS-1 - a/(a+b);
    }

    // Parabolic interpolation:
    size_t left = strongest - 1;
    size_t right = strongest + 1;
    float a = sensorValues[left] - ambientReading[servoAngle][left];
    float b = sensorValues[strongest] - ambientReading[servoAngle][strongest];
    float c = sensorValues[right] - ambientReading[servoAngle][right];
    return strongest + (a-b) / (2*a + 2*c -4*b);
}


/**
 * PID controller for servo position; aims to move the ball to TARGET. Returns the angle commanded to the servo.
 */
float servo_PID_move(float position) {
    unsigned long now = micros();

    float dt = (now - previousTime) / 1000000.0;
    previousTime = now;

    if (dt <= 0.0) {
        // No time delta, not ready to move yet
        return 0;
    }

    float error = TARGET - position;

    // Integral
    integral += error * dt;
    //Cap integral so it doesn't accumulate
    if (integral > 50) {
        integral = 50;
    }
    else if (integral < -50) {
        integral = -50;
    }

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

    servo.write((int)servoCommand);

    Serial.print("Position: ");
    Serial.print(position);
    Serial.print("  ");

    Serial.print("\tError: ");
    Serial.print(error);

    Serial.print("\tPID Output: ");
    Serial.print(output);

    Serial.print("\tServo: ");
    Serial.println(servoCommand);

    return servoCommand;
}

void calibrate_phototransistors(int (*ambientValues)[NUM_SENSORS], char* text) {    
    Serial.println(text);
    Serial.println("Press Q to begin phototransistor calibration");
    while(1) {
        if (Serial.available() > 0) {
            char input = Serial.read();
            if (input == 'q' || input == 'Q') {
                break;
            }
        }
    }

    for (int angle=(int)servoMin; angle<(int)servoMax; angle++) {
        servo.write((int)angle);
        delay(100);
        // Read once into sensorValues[]
        read_sensor_values();

        // Copy sensorValues[] into ambientReading[] array
        for (int i=0; i<NUM_SENSORS; i++) {
            ambientValues[(int)angle][i] = sensorValues[i];
        }
    }
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

    calibrate_servo_limit(&servoCenter, "Place tha ball between Q4 and Q5, then tilt the platform until it's steady");
    calibrate_servo_limit(&servoMin, "Tilt the platform until the ball quickly rolls toward Q1");
    calibrate_servo_limit(&servoMax, "Tilt the platform until the ball rolls quickly toward Q8");

    calibrate_phototransistors(ambientReading, "Remove ball and let phototransistors calibrate using ambient lighting");

    Serial.print("Input W to start program");
    while(1) {
        if (Serial.available() > 0) {
            char input = Serial.read();
            if (input == 'w' || input == 'W') {
                break;
            }
        }
    }
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

float prevServoAngle = servoCenter;
void loop() {
    read_sensor_values();
    float position = estimate_ball_position(prevServoAngle);
    print_sensor_values();
    print_ambient_values(prevServoAngle);
    prevServoAngle = servo_PID_move(position);
    tune_PID(0.2);

    delay(50);
}