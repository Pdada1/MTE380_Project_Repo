/*Overall functionality needed
Wait for PC to prompt for calibration through serial message.
Start by moving the plaform at x hertz
Get the camera to record position data for the ball across multiple oscillation cycles of the ball
Prompt from computer to stop data recoding or just use a set amount of time
Reset the platform to level and wait for the computer to tell it to continue?
*/
#include "camera_servo.h"
#include "freq_response_servo.h"

#define MIN_TSTEP 0.05f
#define MAX_ANGLE_DEVIATION 10.0f
#define MAX_FREQ 1.5f       // Hz
#define MIN_FREQ 0.5f       // Hz
#define FREQ_INCREMENT 0.1f // Hz
#define NUM_OSCILLATIONS 10

#define SERVO_0_CH 8
#define SERVO_1_CH 10
#define SERVO_2_CH 9

#define BALL_EXTENT_MM 25.0f
#define G_MM_S2 9810.0f
#define MIN_AMP_DEG 1.0f

#define PROCEED_CHAR 0xC

BallData sweep_ball_data;
unsigned long time_us;

Mode current_mode = Mode::TRANSIENT_IT;

// Amplitude (deg) that keeps ball travel near BALL_EXTENT_MM at this frequency.
float amp_for_freq(float freq)
{
    float w = 2.0f * PI * freq;
    float theta_rad = BALL_EXTENT_MM * w * w / (2.0f * (5.0f / 7.0f) * G_MM_S2);
    float theta_deg = theta_rad * 180.0f / PI;
    return constrain(theta_deg, MIN_AMP_DEG, MAX_ANGLE_DEVIATION);
}

// Generate a sinusoid of angles between Angle Min and Angle Max amplitude as a sin wave of a given freq
// Use the freq in Hz
double generate_sin_angle(float freq, int point, float amp)
{
    return amp * sin(MIN_TSTEP * point * 2 * PI * freq);
}

bool wait_for_pc()
{
    while (!Serial.available())
    {
    } // block until the PC sends anything
    int byte = Serial.read();
    return byte == PROCEED_CHAR;
}

void run_sweep_X()
{
    run_sweep('x');
}

void run_sweep_Y()
{
   run_sweep('y');
}

// Runs the full frequency sweep on one axis ('x' or 'y').
void run_sweep(char axis)
{
    // Round before truncating so float error (e.g. 9.9999 instead of 10) can't drop the last frequency
    const int num_freqs = (int)((MAX_FREQ - MIN_FREQ) / FREQ_INCREMENT + 0.5f) + 1;

    for (int n = 0; n < num_freqs; n++)
    {
        float freq = MIN_FREQ + n * FREQ_INCREMENT;
        setTilt(0, 0, &sweep_ball_data);

        switch (current_mode)
        {
        case Mode::HUMAN_IN_LOOP:
            Serial.print("Start Frequency Run for ");
            Serial.print(freq);
            Serial.println("?");
            while (!wait_for_pc())
            {
            } // start each frequency run
            Serial.print("Starting Frequency Run for ");
            Serial.println(freq);
            break;
        case Mode::PID_TO_STOP:
            // call PID stopping function from camera_servo
            break;
        case Mode::TRANSIENT_IT:
            Serial.print("Starting Frequency Run for ");
            Serial.println(freq);
            break;
        }

        int total_points = (int)(NUM_OSCILLATIONS / (freq * MIN_TSTEP) + 0.5f);
        float amp = amp_for_freq(freq);
        time_us = micros();

        for (int i = 0; i < total_points; i++)
        {
            float angle = generate_sin_angle(freq, i, amp);

            if (axis == 'x')
                setTilt(angle, 0, &sweep_ball_data);
            else
                setTilt(0, angle, &sweep_ball_data);

            time_us += (unsigned long)(MIN_TSTEP * 1e6f); // next deadline, 50,000 us later
            while ((long)(micros() - time_us) < 0)
            {
            }
        }
    }

    setTilt(0, 0, &sweep_ball_data); // leave the platform level
}

void set_motor_pins(uint8_t servo_A_pin, uint8_t servo_B_pin, uint8_t servo_C_pin)
{
    sweep_ball_data.servo_A_pin = servo_A_pin;
    sweep_ball_data.servo_B_pin = servo_B_pin;
    sweep_ball_data.servo_C_pin = servo_C_pin;
}
