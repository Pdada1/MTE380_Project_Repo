#ifndef FREQ_RESPONSE_SERVO_H
#define FREQ_RESPONSE_SERVO_H
#include <stdint.h>

enum class Mode {
    HUMAN_IN_LOOP,
    PID_TO_STOP,
    TRANSIENT_IT
};
extern Mode current_mode; 

void run_sweep_X();
void run_sweep_Y();
void run_sweep_axis(char axis);
void set_motor_pins(uint8_t servo_A_pin, uint8_t servo_B_pin, uint8_t servo_C_pin);

#endif