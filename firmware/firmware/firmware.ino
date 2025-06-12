// RoboOmni - Main firmware file
// Modularized micro-ROS firmware for omnidirectional robot

#include "include/motor_control.h"
#include "include/encoder.h"
#include "include/ros_interface.h"
#include "include/freertos_tasks.h"

void setup() {
  Serial.begin(460800);
  delay(2000);
  
  // Initialize hardware components
  setup_motors();
  setup_encoders();
  setup_ros();

  // Create FreeRTOS tasks
  create_tasks();
}

void loop() {
  // Nothing here — all handled in FreeRTOS tasks
}