#include "include/ros_interface.h"
#include "include/motor_control.h"
#include "include/encoder.h"

// ROS objects
rcl_publisher_t encoder_pub;
std_msgs__msg__Int32MultiArray encoder_msg;
rcl_subscription_t pwm_sub;
std_msgs__msg__Int16MultiArray pwm_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void pwm_callback(const void *msgin)
{
    const std_msgs__msg__Int16MultiArray *msg = (const std_msgs__msg__Int16MultiArray *)msgin;
    for (int i = 0; i < NUM_MOTORS && i < msg->data.size; i++)
    {
        pwm_values[i] = msg->data.data[i];
    }
}

void publish_encoder_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    for (int i = 0; i < NUM_ENCODERS; i++)
    {
        encoder_msg.data.data[i] = encoder_counts[i];
    }
    rcl_publish(&encoder_pub, &encoder_msg, NULL);
}

void setup_ros()
{
    set_microros_transports();

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_node", "", &support);

    // Subscriber
    rclc_subscription_init_default(
        &pwm_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "/pwm_cmds");

    // Publisher
    rclc_publisher_init_default(
        &encoder_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/encoder");

    // Timer
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(10), // 100Hz
        publish_encoder_cb);

    // Executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &pwm_sub, &pwm_msg, &pwm_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    // Init encoder message
    encoder_msg.data.data = (int32_t *)malloc(NUM_ENCODERS * sizeof(int32_t));
    encoder_msg.data.size = NUM_ENCODERS;
    encoder_msg.data.capacity = NUM_ENCODERS;

    // Init pwm message
    pwm_msg.data.data = (int16_t *)malloc(NUM_MOTORS * sizeof(int16_t));
    pwm_msg.data.size = NUM_MOTORS;
    pwm_msg.data.capacity = NUM_MOTORS;
}