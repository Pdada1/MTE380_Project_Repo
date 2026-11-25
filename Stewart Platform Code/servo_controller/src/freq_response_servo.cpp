/*Overall functionality needed
Wait for PC to prompt for calibration through serial message.
Start by moving the plaform at x hertz
Get the camera to record position data for the ball across multiple oscillation cycles of the ball
Prompt from computer to stop data recoding or just use a set amount of time
Reset the platform to level and wait for the computer to tell it to continue?
*/
#include "camera_servo.h"
#include "freq_response_servo.h"
#include "controller.h"

#define MIN_TSTEP 0.05f
#define MAX_ANGLE_DEVIATION 5.0f
#define MAX_FREQ 1.5f       // Hz
#define MIN_FREQ 0.5f       // Hz
#define MIN_SINGLE_FREQ 0.1f // Hz, limits for a single requested run (keep in sync with run.py)
#define MAX_SINGLE_FREQ 2.0f // Hz
#define FREQ_INCREMENT 0.1f // Hz
#define NUM_OSCILLATIONS 10

#define SERVO_0_CH 8
#define SERVO_1_CH 10
#define SERVO_2_CH 9

#define BALL_EXTENT_MM 20.0f
#define G_MM_S2 9810.0f
#define MIN_AMP_DEG 1.0f


BallData sweep_ball_data;
unsigned long time_us;
bool sweep_aborted = false;

Mode current_mode = Mode::HUMAN_IN_LOOP;

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
double generate_cos_angle(float freq, int point, float amp)
{
    return -amp * cos(MIN_TSTEP * point * 2 * PI * freq);
}

// Block until the PC sends PROCEED_CHAR or RERUN_CHAR, keeping the ball centred meanwhile.
uint8_t wait_for_pc(float* tx, float* ty)
{
    bool updated = false;
    while (true)
    {
        const uint8_t key = sweep_service('b', tx, ty, &updated);
        if (updated)
            setTilt(*tx, *ty, &sweep_ball_data);
        if (key == PROCEED_CHAR || key == RERUN_CHAR || key == STOP_CHAR)
            return key;
    }
}

void run_sweep_X(float only_freq)
{
    run_sweep('x', only_freq);
}

void run_sweep_Y(float only_freq)
{
   run_sweep('y', only_freq);
}

// Runs the frequency sweep on one axis ('x' or 'y'). If only_freq > 0, only that
// frequency is run (once) instead of the full MIN_FREQ..MAX_FREQ sweep.
void run_sweep(char axis, float only_freq)
{
    const bool single = only_freq > 0.0f;
    if (single && (only_freq < MIN_SINGLE_FREQ || only_freq > MAX_SINGLE_FREQ))
    {
        Serial.println("BAD_FREQUENCY");
        sweep_aborted = true;
        return;
    }
    const int num_freqs = single ? 1 : (int)((MAX_FREQ - MIN_FREQ) / FREQ_INCREMENT + 0.5f) + 1;
    const char hold_axis = (axis == 'x') ? 'y' : 'x'; // PID keeps the ball on this axis
    float tx = 0, ty = 0;                             // latest PID tilts
    bool updated = false;

    int n = 0;
    while (n < num_freqs)
    {
        float freq = single ? only_freq : MIN_FREQ + n * FREQ_INCREMENT;

        switch (current_mode)
        {
        case Mode::HUMAN_IN_LOOP:
        {
            Serial.print("Start Frequency Run for ");
            Serial.print(freq);
            Serial.println("?");
            // Discard keys pressed before the prompt, but never a stop request
            if (sweep_service('b', &tx, &ty, &updated) == STOP_CHAR)
            {
                sweep_aborted = true;
                break;
            }
            const uint8_t key = wait_for_pc(&tx, &ty);
            if (key == STOP_CHAR)
            {
                sweep_aborted = true;
                break;
            }
            if (key == RERUN_CHAR)
            {
                // Step back so the previous frequency is offered again
                if (n > 0)
                    n--;
                continue;
            }
            Serial.print("Starting Frequency Run for ");
            Serial.println(freq);
            break;
        }
        case Mode::PID_TO_STOP:
            // call PID stopping function from camera_servo
            break;
        case Mode::TRANSIENT_IT:
            Serial.print("Starting Frequency Run for ");
            Serial.println(freq);
            break;
        }

        if (sweep_aborted)
            break;

        const int total_points = (int)(NUM_OSCILLATIONS / (freq * MIN_TSTEP) + 0.5f);
        const float amp = amp_for_freq(freq);

        Serial.print("RUN_START,");
        Serial.print(axis);
        Serial.print(',');
        Serial.print(freq, 2);
        Serial.print(',');
        Serial.println(amp, 3);

        time_us = micros();

        for (int i = 0; i < total_points && !sweep_aborted; i++)
        {
            const float angle = generate_cos_angle(freq, i, amp);
            if (axis == 'x')
                setTilt(angle, ty, &sweep_ball_data);
            else
                setTilt(tx, angle, &sweep_ball_data);

            time_us += (unsigned long)(MIN_TSTEP * 1e6f); // next deadline, 50,000 us later
            while ((long)(micros() - time_us) < 0)
            {
                if (sweep_service(hold_axis, &tx, &ty, &updated) == STOP_CHAR)
                {
                    sweep_aborted = true;
                    break;
                }
                if (updated)
                {
                    // New camera sample: refresh the held axis without waiting for the next step
                    if (axis == 'x')
                        setTilt(angle, ty, &sweep_ball_data);
                    else
                        setTilt(tx, angle, &sweep_ball_data);
                }
            }
        }

        pid_reset(); // recenter from a clean integral
        if (sweep_aborted)
            break;
        Serial.println("RUN_END");
        n++;
    }

    setTilt(0, 0, &sweep_ball_data); // leave the platform level
}

void set_motor_pins(uint8_t servo_A_pin, uint8_t servo_B_pin, uint8_t servo_C_pin)
{
    sweep_ball_data.servo_A_pin = servo_A_pin;
    sweep_ball_data.servo_B_pin = servo_B_pin;
    sweep_ball_data.servo_C_pin = servo_C_pin;
}