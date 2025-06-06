#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef __cplusplus
extern "C" {
#endif

// Task function declarations
void ros_spin_task(void *pvParameters);
void motor_control_task(void *pvParameters);

// Task creation functions
void create_tasks(void);

#ifdef __cplusplus
}
#endif

#endif // FREERTOS_TASKS_H