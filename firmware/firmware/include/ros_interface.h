#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

// Include micro-ROS headers first
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>

// ROS objects
extern rcl_publisher_t joint_pub;
extern sensor_msgs__msg__JointState joint_msg;
extern rcl_subscription_t pwm_sub;
extern std_msgs__msg__Int16MultiArray pwm_msg;
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;
extern rcl_timer_t timer;

// Function declarations
void setup_ros(void);
void pwm_callback(const void *msgin);
void publish_encoder_cb(rcl_timer_t *timer, int64_t last_call_time);

#endif // ROS_INTERFACE_H