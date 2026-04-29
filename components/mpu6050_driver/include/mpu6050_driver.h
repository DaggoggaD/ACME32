#ifndef MPU_6050_DRIVER_H
#define MPU_6050_DRIVER_H

#include "esp_log.h"
#include "driver/i2c_master.h"
#include "i2cController.h"
#include "esp_rom_sys.h"

typedef struct ReadData_Accelerometer{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

} ReadData_Accelerometer;

typedef struct ReadData_Gyroscope{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

} ReadData_Gyroscope;

typedef struct RawReadData_Mpu6050{
    ReadData_Accelerometer accelerometerData;
    ReadData_Gyroscope gyroscopeData;
    int16_t temperature;

} RawReadData_Mpu6050;

typedef struct ReadData_Mpu6050{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float temp_c;

} ReadData_Mpu6050;

esp_err_t wake_up_mpu6050(i2c_master_dev_handle_t device);

esp_err_t wake_up_calibrated_mpu6050(i2c_master_dev_handle_t device);

esp_err_t get_raw_data_mpu6050(i2c_master_dev_handle_t device, RawReadData_Mpu6050* dataOut);

esp_err_t get_data_mpu6050(i2c_master_dev_handle_t device, ReadData_Mpu6050* dataOut);

#endif