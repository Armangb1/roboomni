#include "include/freertos_tasks.h"
#include "include/ros_interface.h"
#include "include/motor_control.h"

void ros_spin_task(void *pvParameters)
{
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(5)); // 200Hz
    }
}

void motor_control_task(void *pvParameters)
{
    while (1)
    {
        update_motor_outputs();        // Apply latest PWM
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

void create_tasks(void)
{
    xTaskCreatePinnedToCore(ros_spin_task, "ros_spin", 4000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(motor_control_task, "motor_ctrl", 2048, NULL, 1, NULL, 0);
}