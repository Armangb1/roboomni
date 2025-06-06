#include "include/motor_control.h"

// Motor Pins
const int motor_in1[NUM_MOTORS] = {5, 18, 4, 15};
const int motor_in2[NUM_MOTORS] = {17, 19, 16, 2};

// Global PWM values
int pwm_values[NUM_MOTORS] = {0};

void setup_motors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motor_in1[i], OUTPUT);  
    pinMode(motor_in2[i], OUTPUT);
    ledcAttach(motor_in1[i], 20000, 16);  // Attach PWM: pin, freq, resolution
  }
}

void update_motor_outputs() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    int pwm = pwm_values[i];
    int speed = abs(pwm);
    if (speed > 65535) speed = 65535;

    digitalWrite(motor_in2[i], pwm >= 0 ? LOW : HIGH);  // IN2 = DIR
    ledcWrite(motor_in1[i], speed);                     // Write PWM to pin directly
  }
}