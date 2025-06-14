#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

// Include micro-ROS headers first
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <rosidl_runtime_c/string_functions.h>

// Core ROS objects
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;
extern rcl_timer_t encoder_timer;
extern rcl_timer_t imu_timer;

// ROS objects for encoders
extern rcl_publisher_t encoder_pub;
extern std_msgs__msg__Int32MultiArray encoder_msg;

// ROS objects for PWM commands
extern rcl_subscription_t pwm_sub;
extern std_msgs__msg__Int16MultiArray pwm_msg;

// ROS objects for IMU
extern rcl_publisher_t imu_pub;
extern sensor_msgs__msg__Imu imu_msg;
extern rcl_publisher_t mag_pub;
extern sensor_msgs__msg__MagneticField mag_msg;

extern rmw_qos_profile_t custom_qos;


// Function declarations
void setup_ros(void);
void pwm_callback(const void *msgin);
void publish_encoder_cb(rcl_timer_t *timer, int64_t last_call_time);
void publish_imu_cb(rcl_timer_t *timer, int64_t last_call_time);

#endif // ROS_INTERFACE_H