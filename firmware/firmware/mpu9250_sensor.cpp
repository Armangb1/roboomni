#include "include/mpu9250_sensor.h"

// Global sensor data
mpu9250_data_t mpu9250_data = {0};
bool mpu9250_ready = false;

// Create MPU9250 object (I2C interface, address 0x68)
MPU9250 mpu9250_sensor;

bool setup_mpu9250(void)
{
    // Initialize I2C
    Wire.begin();  // SDA=21, SCL=22, 40kHz
    delay(100);
    
    Serial.println("Initializing MPU9250...");
    
    // Start communication with IMU
    int status = mpu9250_sensor.setup(0x68);
    if (status < 0) {
        Serial.println("MPU9250 initialization unsuccessful");
        Serial.print("Status: ");
        Serial.println(status);
        return false;
    }
    
    Serial.println("MPU9250 initialized successfully");
    
    // Optional: Run calibration
    Serial.println("Starting calibration in 3 seconds...");
    delay(3000);
    calibrate_mpu9250();
    
    mpu9250_ready = true;
    return true;
}

bool read_mpu9250_data(void)
{
    if (!mpu9250_ready) return false;
    
    // Read the sensor
    mpu9250_sensor.update();
    
    // Get accelerometer values (m/s²)
    mpu9250_data.accel_x = mpu9250_sensor.getAccX();
    mpu9250_data.accel_y = mpu9250_sensor.getAccY();
    mpu9250_data.accel_z = mpu9250_sensor.getAccZ();
    
    // Get gyroscope values (rad/s)
    mpu9250_data.gyro_x = mpu9250_sensor.getGyroX();
    mpu9250_data.gyro_y = mpu9250_sensor.getGyroY();
    mpu9250_data.gyro_z = mpu9250_sensor.getGyroZ();
    
    // Get magnetometer values (μT)
    mpu9250_data.mag_x = mpu9250_sensor.getMagX();
    mpu9250_data.mag_y = mpu9250_sensor.getMagY();
    mpu9250_data.mag_z = mpu9250_sensor.getMagZ();
    
    // Get temperature (°C)
    mpu9250_data.temperature = mpu9250_sensor.getTemperature();
    
    return true;
}

void calibrate_mpu9250(void)
{
    Serial.println("Starting MPU9250 calibration...");
    Serial.println("Keep the sensor stationary and level!");
    
    // Calibrate gyroscope and accelerometer
    Serial.println("Calibrating gyroscope and accelerometer...");
    mpu9250_sensor.calibrateAccelGyro();
    Serial.println("Gyroscope and accelerometer calibration complete");
    
    // Calibrate magnetometer
    Serial.println("Starting magnetometer calibration...");
    mpu9250_sensor.calibrateMag();

    Serial.println("Magnetometer calibration complete");
    Serial.println("All calibrations finished successfully!");
    
    // Print calibration values for reference
    Serial.println("Calibration values:");
    Serial.print("Accel bias X: "); Serial.println(mpu9250_sensor.getAccBiasX());
    Serial.print("Accel bias Y: "); Serial.println(mpu9250_sensor.getAccBiasY());
    Serial.print("Accel bias Z: "); Serial.println(mpu9250_sensor.getAccBiasZ());
    
    Serial.print("Gyro bias X: "); Serial.println(mpu9250_sensor.getGyroBiasX());
    Serial.print("Gyro bias Y: "); Serial.println(mpu9250_sensor.getGyroBiasY());
    Serial.print("Gyro bias Z: "); Serial.println(mpu9250_sensor.getGyroBiasZ());
    
    Serial.print("Mag bias X: "); Serial.println(mpu9250_sensor.getMagBiasX());
    Serial.print("Mag bias Y: "); Serial.println(mpu9250_sensor.getMagBiasY());
    Serial.print("Mag bias Z: "); Serial.println(mpu9250_sensor.getMagBiasZ());
    
    Serial.print("Mag scale X: "); Serial.println(mpu9250_sensor.getMagScaleX());
    Serial.print("Mag scale Y: "); Serial.println(mpu9250_sensor.getMagScaleY());
    Serial.print("Mag scale Z: "); Serial.println(mpu9250_sensor.getMagScaleZ());
}