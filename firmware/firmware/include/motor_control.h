#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// Motor configuration
#define NUM_MOTORS 4

// Motor pin definitions
extern const int motor_in1[NUM_MOTORS];
extern const int motor_in2[NUM_MOTORS];

// Global PWM values
extern int pwm_values[NUM_MOTORS];

// Function declarations
void setup_motors(void);
void update_motor_outputs(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H