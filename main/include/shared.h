#ifndef SHARED_H
#define SHARED_H

#include <stdio.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "i2cController.h"
#include "mpu6050_driver.h"
#include "bmp280.h"
#include "filters.h"
#include "avionics.h"

#define DEBUG 1
#define TELEPLOT 0
#define CYCLES_GROUNDUP_CALIBRATION 100
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_FREQ_HZ          100000

#define BMP280_ADDR                 0x76
#define BMP280_REG_ID               0xD0

#define MPU6050_ADDR                0x68
#define MPU6050_REG_WHO_AM_I        0x75

typedef struct Vector3 {
    float x;
    float y;
    float z;
} Vector3;

typedef struct FlightData{
    Vector3 gyro_dps;
    Vector3 accel_ms2;

    float press;
    float temp;

    uint32_t time;
} FlightData;

typedef struct FlightTelemetry{
    float accelerationUp;
    float height;
    float velocityUp;
    float tiltAngle_deg;
    Vector3 upDir;
    uint32_t time;

} FlightTelemetry;


#endif