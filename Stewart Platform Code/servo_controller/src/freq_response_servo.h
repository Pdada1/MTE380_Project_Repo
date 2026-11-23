#ifndef FREQ_RESPONSE_SERVO_H
#define FREQ_RESPONSE_SERVO_H
#include <stdint.h>
 
#define PROCEED_CHAR 0x0A
#define RERUN_CHAR 'r'
#define STOP_CHAR 'x'
 
extern bool sweep_aborted;
 
enum class Mode {
    HUMAN_IN_LOOP,
    PID_TO_STOP,
    TRANSIENT_IT
};
extern Mode current_mode; 
 
void run_sweep(char axis);
void run_sweep_X();
void run_sweep_Y();
void run_sweep_axis(char axis);
void set_motor_pins(uint8_t servo_A_pin, uint8_t servo_B_pin, uint8_t servo_C_pin);
 
// Implemented in camera_modes.cpp. Reads serial packets, feeds the controller and runs PID
// on the requested axes ('x', 'y' or 'b' for both), writing the tilts into *tx / *ty.
// Sets *updated when a new camera sample arrived. Returns a PC key byte or 0.
uint8_t sweep_service(char axes, float* tx, float* ty, bool* updated);
 
#endif