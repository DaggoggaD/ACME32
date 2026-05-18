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
#include "RocketSetup.h"
#include "buzzer.h"
#include "filters.h"
#include "avionics.h"

// Components addresses for i2c communications.
// Should not be modified.
#define BMP280_ADDR                 0x76
#define BMP280_REG_ID               0xD0
#define MPU6050_ADDR                0x68
#define MPU6050_REG_WHO_AM_I        0x75
#define I2C_MASTER_FREQ_HZ          100000

typedef enum {
    ServoNorth,
    ServoSouth,
    ServoEast,
    ServoWest
} ServoIndex;

typedef enum {
    S_IDLE,
    S_ARMED,
    S_BOOST,
    S_COAST,
    S_APOGEE,
    S_DESCENT,
    S_LANDED
} FlightState;

typedef struct Vector3 {
    float x;
    float y;
    float z;
} Vector3;

// Stores the output of the sensors
typedef struct ReadingsData{
    Vector3 gyro_dps;
    Vector3 accel_ms2;

    float press;
    float temp;

    uint32_t time;
} ReadingsData;

// Stores the filtered and calculated infos of the
// current flight situation 
typedef struct FlightTelemetry{
    // Position
    float accelerationUp;
    float height;
    float velocityUp;

    // Rotation
    float pitchAngle_deg;
    float yawAngle_deg;
    float tiltAngle_deg;
    Vector3 upDir;
    
    // State
    uint32_t time;
    FlightState FSMstate;

} FlightTelemetry;


#endif