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


// ROS objects for IMU
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t mag_pub;
sensor_msgs__msg__MagneticField mag_msg;


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


    // Read sensor data
    // Get current time
    int64_t time_ns = rmw_uros_epoch_nanos();
    
    // Populate IMU message
    imu_msg.header.stamp.sec = time_ns / 1000000000;
    imu_msg.header.stamp.nanosec = time_ns % 1000000000;
    
    // Linear acceleration (already in m/s²)
    imu_msg.linear_acceleration.x = mpu9250_data.accel_x;
    imu_msg.linear_acceleration.y = mpu9250_data.accel_y;
    imu_msg.linear_acceleration.z = mpu9250_data.accel_z;
    
    // Angular velocity (already in rad/s)
    imu_msg.angular_velocity.x = mpu9250_data.gyro_x;
    imu_msg.angular_velocity.y = mpu9250_data.gyro_y;
    imu_msg.angular_velocity.z = mpu9250_data.gyro_z;
    
    // Orientation (not computed, set to zero with high covariance)
    // imu_msg.orientation.x = 0.0;
    // imu_msg.orientation.y = 0.0;
    // imu_msg.orientation.z = 0.0;
    // imu_msg.orientation.w = 1.0;
    
    // Covariance matrices (set to -1 for unknown, or provide estimates)
    // for (int i = 0; i < 9; i++) {
    //     imu_msg.orientation_covariance[i] = -1.0;          // Unknown orientation
    //     imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.001 : 0.0;  // Diagonal
    //     imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.001 : 0.0;  // Diagonal
    // }
    
    // Publish IMU data
    rcl_publish(&imu_pub, &imu_msg, NULL);
    
    // Populate magnetic field message
    mag_msg.header.stamp.sec = time_ns / 1000000000;
    mag_msg.header.stamp.nanosec = time_ns % 1000000000;
    // mag_msg.header.frame_id.data = "imu_link";
    // mag_msg.header.frame_id.size = strlen("imu_link");
    
    // Magnetic field (Tesla) - convert μT to T
    mag_msg.magnetic_field.x = mpu9250_data.mag_x * 1e-6;
    mag_msg.magnetic_field.y = mpu9250_data.mag_y * 1e-6;
    mag_msg.magnetic_field.z = mpu9250_data.mag_z * 1e-6;
    
    // Magnetic field covariance
    // for (int i = 0; i < 9; i++) {
    //     mag_msg.magnetic_field_covariance[i] = (i % 4 == 0) ? 1e-12 : 0.0;  // Diagonal in T²
    // }
    
    // Publish magnetic field data
    rcl_publish(&mag_pub, &mag_msg, NULL);
}

void test_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    // do nothing
}

void setup_ros()
{
    set_microros_transports();

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_node", "", &support);

    // Custom QoS profile for IMU and magnetic field data
    // This profile uses best-effort reliability for lower latency
    // and keeps the last 5 messages in history.
    // It is suitable for high-frequency data like IMU and magnetic field.
    custom_qos = rmw_qos_profile_default;
    custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT; // Change to best-effort for lower latency
    custom_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos.depth = 5; // Keep last 5 messages
    // custom_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;


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
        &custom_qos);

    
    // Publisher for IMU data
    rclc_publisher_init(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data",
        &custom_qos);

    // Publisher for magnetic field data
    rclc_publisher_init(
        &mag_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "/imu/mag",
        &custom_qos);

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

    // Initialize IMU message
    imu_msg.header.frame_id.data = (char *)malloc(10);
    imu_msg.header.frame_id.capacity = 10;
    
    imu_msg.header.frame_id.data = (char *)"imu_link";
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0; // Identity quaternion
    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = -1.0; // Unknown orientation
        imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.001 : 0.0; // Diagonal
        imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.001 : 0.0; // Diagonal
    }


    // Initialize magnetic field message
    mag_msg.header.frame_id.data = (char *)malloc(10);
    mag_msg.header.frame_id.capacity = 10;

    mag_msg.header.frame_id.data = (char *)"imu_link";
    mag_msg.header.frame_id.size = strlen(mag_msg.header.frame_id.data);

    mag_msg.magnetic_field.x = 0.0;
    mag_msg.magnetic_field.y = 0.0;
    mag_msg.magnetic_field.z = 0.0;

    for (int i = 0; i < 9; i++) {
        mag_msg.magnetic_field_covariance[i] = (i % 4 == 0) ? 1e-12 : 0.0; // Diagonal in T²
    }





}