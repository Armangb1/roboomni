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
    // Setup PWM channel for each motor
    ledcSetup(i, 12000, 12);  // channel, frequency, resolution
    ledcAttachPin(motor_in1[i], i);  // pin, channel  // Attach PWM: pin, freq, resolution
  }
}

void update_motor_outputs() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    int pwm = pwm_values[i];
    int speed = abs(pwm);
    if (speed > 4096) speed = 4096;

    digitalWrite(motor_in2[i], pwm >= 0 ? LOW : HIGH);  // IN2 = DIR
    ledcWrite(i, speed);                     // Write PWM to pin directly
  }
}