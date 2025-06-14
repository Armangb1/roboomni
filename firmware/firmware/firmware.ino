// RoboOmni - Main firmware file
// Modularized micro-ROS firmware for omnidirectional robot with MPU9250 IMU

#include "include/motor_control.h"
#include "include/encoder.h"
#include "include/ros_interface.h"
#include "include/freertos_tasks.h"
#include "include/mpu9250_sensor.h"

void setup() {
  Serial.begin(921600);
  // Serial.begin(1000000);
  delay(2000);
  
  Serial.println("RoboOmni Starting...");
  
  // Initialize hardware components
  Serial.println("Initializing motors...");
  setup_motors();
  
  Serial.println("Initializing encoders...");
  setup_encoders();
  
  Serial.println("Initializing MPU9250...");
  if (!setup_mpu9250()) {
    Serial.println("MPU9250 initialization failed! Continuing without IMU...");
  }
  
  Serial.println("Initializing ROS interface...");
  setup_ros();

  Serial.println("Creating FreeRTOS tasks...");
  create_tasks();
  
  Serial.println("Setup complete!");
}

void loop() {
  // Nothing here — all handled in FreeRTOS tasks
}