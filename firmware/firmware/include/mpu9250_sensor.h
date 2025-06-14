#ifndef MPU9250_SENSOR_H
#define MPU9250_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>  // Add this library: https://github.com/bolderflight/MPU9250

#ifdef __cplusplus
extern "C" {
#endif

// Sensor data structure
typedef struct {
    float accel_x, accel_y, accel_z;    // Acceleration in g
    float gyro_x, gyro_y, gyro_z;       // Angular velocity in deg/s
    float mag_x, mag_y, mag_z;          // Magnetic field in μT
    float temperature;                   // Temperature in °C
} mpu9250_data_t;

// Global sensor data
extern mpu9250_data_t mpu9250_data;
extern bool mpu9250_ready;
extern MPU9250 mpu9250_sensor;

// Function declarations
bool setup_mpu9250(void);
bool read_mpu9250_data(void);
void calibrate_mpu9250(void);

#ifdef __cplusplus
}
#endif

#endif // MPU9250_SENSOR_H