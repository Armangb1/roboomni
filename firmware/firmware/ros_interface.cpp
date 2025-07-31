#include "include/ros_interface.h"
#include "include/motor_control.h"
#include "include/encoder.h"
#include "include/mpu9250_sensor.h"

// ROS objects for encoders
rcl_publisher_t encoder_pub;
std_msgs__msg__Int32MultiArray encoder_msg;

// ROS objects for PWM commands
rcl_subscription_t pwm_sub;
std_msgs__msg__Int16MultiArray pwm_msg;


// ROS objects for IMU data (using MultiArray instead of sensor_msgs)
rcl_publisher_t imu_pub;
std_msgs__msg__Float32MultiArray imu_msg;

// ROS objects for magnetometer data (using MultiArray instead of sensor_msgs)
rcl_publisher_t mag_pub;
std_msgs__msg__Float32MultiArray mag_msg;

// Core ROS objects
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t encoder_timer;
rcl_timer_t imu_timer;

rmw_qos_profile_t custom_qos;



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

void publish_imu_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    // IMU data array structure:
    // [0-2]: Linear acceleration (x, y, z) in m/s²
    // [3-5]: Angular velocity (x, y, z) in rad/s
    // [6]: Timestamp seconds
    // [7]: Timestamp nanoseconds
    
    // Get current time
    int64_t time_ns = rmw_uros_epoch_nanos();
    
    // Populate IMU message array
    imu_msg.data.data[0] = mpu9250_data.accel_x;  // Linear acceleration X
    imu_msg.data.data[1] = mpu9250_data.accel_y;  // Linear acceleration Y
    imu_msg.data.data[2] = mpu9250_data.accel_z;  // Linear acceleration Z
    imu_msg.data.data[3] = mpu9250_data.gyro_x;   // Angular velocity X
    imu_msg.data.data[4] = mpu9250_data.gyro_y;   // Angular velocity Y
    imu_msg.data.data[5] = mpu9250_data.gyro_z;   // Angular velocity Z
    imu_msg.data.data[6] = (float)(time_ns / 1000000000);  // Timestamp seconds
    imu_msg.data.data[7] = (float)(time_ns % 1000000000);  // Timestamp nanoseconds
    
    // Publish IMU data
    rcl_publish(&imu_pub, &imu_msg, NULL);
    
    // Magnetometer data array structure:
    // [0-2]: Magnetic field (x, y, z) in Tesla
    // [3]: Timestamp seconds
    // [4]: Timestamp nanoseconds
    
    // Populate magnetic field message array
    mag_msg.data.data[0] = mpu9250_data.mag_x * 1e-6;  // Convert μT to T
    mag_msg.data.data[1] = mpu9250_data.mag_y * 1e-6;  // Convert μT to T
    mag_msg.data.data[2] = mpu9250_data.mag_z * 1e-6;  // Convert μT to T
    mag_msg.data.data[3] = (float)(time_ns / 1000000000);  // Timestamp seconds
    mag_msg.data.data[4] = (float)(time_ns % 1000000000);  // Timestamp nanoseconds
    
    // Publish magnetic field data
    rcl_publish(&mag_pub, &mag_msg, NULL);
}


void setup_ros()
{
    set_microros_transports();

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_node", "", &support);
   
    // Using predefined sensor data QoS profile
    // This profile is optimized for sensor data with:
    // - Best effort reliability for low latency
    // - Keep last history with depth of 5
    // - Volatile durability

    // Subscriber for PWM commands
    
    rclc_subscription_init_default(
        &pwm_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "/pwm_cmds");

    // Publisher for encoder data
    rclc_publisher_init(
        &encoder_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/encoder",
        &rmw_qos_profile_sensor_data);

    // Publisher for IMU data (using Float32MultiArray)
    rclc_publisher_init(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/imu/data",
        &rmw_qos_profile_sensor_data);

    // Publisher for magnetic field data (using Float32MultiArray)
    rclc_publisher_init(
        &mag_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/imu/mag",
        &rmw_qos_profile_sensor_data);

    // Timer for encoder publishing (100Hz)
    rclc_timer_init_default(
        &encoder_timer,
        &support,
        RCL_MS_TO_NS(10),
        publish_encoder_cb);

    // Timer for IMU publishing (100Hz)
    rclc_timer_init_default(
        &imu_timer,
        &support,
        RCL_MS_TO_NS(10),
        publish_imu_cb);

    // Executor
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    rclc_executor_add_subscription(&executor, &pwm_sub, &pwm_msg, &pwm_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &encoder_timer);
    rclc_executor_add_timer(&executor, &imu_timer);

    // Initialize encoder message
    encoder_msg.data.data = (int32_t *)malloc(NUM_ENCODERS * sizeof(int32_t));
    encoder_msg.data.size = NUM_ENCODERS;
    encoder_msg.data.capacity = NUM_ENCODERS;

    // Initialize PWM message
    pwm_msg.data.data = (int16_t *)malloc(NUM_MOTORS * sizeof(int16_t));
    pwm_msg.data.size = NUM_MOTORS;
    pwm_msg.data.capacity = NUM_MOTORS;

    // Initialize IMU message (8 elements: 6 sensor values + 2 timestamp values)
    imu_msg.data.data = (float *)malloc(8 * sizeof(float));
    imu_msg.data.size = 8;
    imu_msg.data.capacity = 8;
    
    // Initialize with zeros
    for (int i = 0; i < 8; i++) {
        imu_msg.data.data[i] = 0.0f;
    }

    // Initialize magnetic field message (5 elements: 3 sensor values + 2 timestamp values)
    mag_msg.data.data = (float *)malloc(5 * sizeof(float));
    mag_msg.data.size = 5;
    mag_msg.data.capacity = 5;
    
    // Initialize with zeros
    for (int i = 0; i < 5; i++) {
        mag_msg.data.data[i] = 0.0f;
    }
}
