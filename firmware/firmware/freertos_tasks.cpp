#include "include/freertos_tasks.h"
#include "include/ros_interface.h"
#include "include/motor_control.h"
#include "include/mpu9250_sensor.h"

void ros_spin_task(void *pvParameters)
{
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        vTaskDelay(pdMS_TO_TICKS(1)); // 1000Hz
    }
}

void motor_control_task(void *pvParameters)
{
    while (1)
    {
        update_motor_outputs();        // Apply latest PWM
        vTaskDelay(pdMS_TO_TICKS(2)); // 500Hz
    }
}

void imu_read_task(void *pvParameters)
{
    while (1)
    {
        if (mpu9250_ready) {
            read_mpu9250_data();  // Read sensor data into global structure
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // 200Hz - IMU reading rate
    }
}

void create_tasks(void)
{
    // ROS communication task on core 1 (high priority)
    xTaskCreatePinnedToCore(ros_spin_task, "ros_spin", 8000, NULL, 5, NULL, 1);
    
    // Motor control task on core 0 (medium priority)
    xTaskCreatePinnedToCore(motor_control_task, "motor_ctrl", 2048, NULL, 2, NULL, 0);
    
    // IMU reading task on core 0 (lower priority)
    xTaskCreatePinnedToCore(imu_read_task, "imu_read", 2048, NULL, 1, NULL, 0);
}