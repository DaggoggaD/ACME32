#include "mpu6050_driver.h"
#define MPU6050_ACCEL_LSB_PER_G  2048.0f
#define MPU6050_GYRO_LSB_PER_DPS 16.4f
#define ACCEL_REG_START 0X3B
#define TEMP_REG_START 0X41
#define OFFSET_CYCLES 100


// MPU6050 Calibration
static uint8_t calibrated = 0;

static float accelOffset_x = 0;
static float accelOffset_y = 0;
static float accelOffset_z = 0;

static float gyroOffset_x = 0;
static float gyroOffset_y = 0;
static float gyroOffset_z = 0;

esp_err_t wake_up_mpu6050(i2c_master_dev_handle_t device){
    uint8_t data[2] = {0x6B, 0x1};
    esp_err_t err = write_i2c(device, data, 2);

    if(err == ESP_OK) ESP_LOGI("MPU6050", "Correctly activated");
    else ESP_LOGE("MPU6050", "Couldn't wake up correctly");

    return err;
}

static void reset_offsets(){
    accelOffset_x = 0;
    accelOffset_y = 0;
    accelOffset_z = 0;

    gyroOffset_x = 0;
    gyroOffset_y = 0;
    gyroOffset_z = 0;
}

static esp_err_t calibrate_axis_mpu6050(i2c_master_dev_handle_t device){
    reset_offsets();
    ReadData_Mpu6050 offData = {0};
    float tempReads[6] = {0};
    
    for (int i = 0; i < OFFSET_CYCLES; i++)
    {
        esp_err_t offErr = get_data_mpu6050(device, &offData);
        if(offErr != ESP_OK){
            ESP_LOGW("MPU6050", "Calibration error");
            return offErr;
        }

        tempReads[0] += offData.accel_x_g;
        tempReads[1] += offData.accel_y_g;
        tempReads[2] += (offData.accel_z_g - 1);

        tempReads[3] += offData.gyro_x_dps;
        tempReads[4] += offData.gyro_y_dps;
        tempReads[5] += offData.gyro_z_dps;


        // if freertos.h is available, use vtaskdelay instead.
        // This is a busy cpu command, wich "burns" 200ms total
        // on each startup. As it's only for initialization, it's fine.
        esp_rom_delay_us(2000);
    }

    accelOffset_x = tempReads[0] / OFFSET_CYCLES;
    accelOffset_y = tempReads[1] / OFFSET_CYCLES;
    accelOffset_z = tempReads[2] / OFFSET_CYCLES;

    gyroOffset_x = tempReads[3] / OFFSET_CYCLES;
    gyroOffset_y = tempReads[4] / OFFSET_CYCLES;
    gyroOffset_z = tempReads[5] / OFFSET_CYCLES;

    calibrated = 1;

    return ESP_OK;
}

esp_err_t wake_up_calibrated_mpu6050(i2c_master_dev_handle_t device){
    uint8_t data[2] = {0x6B, 0x1};
    esp_err_t err = write_i2c(device, data, 2);

    if(err == ESP_OK) ESP_LOGI("MPU6050", "Correctly activated");
    else ESP_LOGE("MPU6050", "Couldn't wake up correctly");

    // Set gyro to higher dps
    uint8_t gyro_cfg[2] = {0x1B, 0x18};
    err |= write_i2c(device, gyro_cfg, 2);

    // Set accelerometer to higher gs.
    uint8_t accel_cfg[2] = {0x1C, 0x18};
    err |= write_i2c(device, accel_cfg, 2);

    if (err != ESP_OK) ESP_LOGW("MPU6050", "Warning: Could not set Full Scale ranges");

    if(calibrated != 0) return err;
    err = calibrate_axis_mpu6050(device);
    if(err!=ESP_OK) ESP_LOGW("MPU6050", "Couldn't calibrate axis correctly");

    return err;
}

esp_err_t get_raw_data_mpu6050(i2c_master_dev_handle_t device, RawReadData_Mpu6050* dataOut){
    uint8_t data[14];

    esp_err_t err = read_sensor_i2c(device, ACCEL_REG_START, data, 14);
    if(err != ESP_OK) {
        ESP_LOGE("MPU6050", "Couldn't read data correctly");
        return err;
    }

    dataOut->accelerometerData.accel_x = (int16_t)(data[0] << 8) | data[1];
    dataOut->accelerometerData.accel_y = (int16_t)(data[2] << 8) | data[3];
    dataOut->accelerometerData.accel_z = (int16_t)(data[4] << 8) | data[5];

    dataOut->temperature = (int16_t)(data[6] << 8) | data[7];

    dataOut->gyroscopeData.gyro_x = (int16_t)(data[8] << 8)  | data[9];
    dataOut->gyroscopeData.gyro_y = (int16_t)(data[10] << 8) | data[11];
    dataOut->gyroscopeData.gyro_z = (int16_t)(data[12] << 8) | data[13];

    return err;
}

esp_err_t get_data_mpu6050(i2c_master_dev_handle_t device, ReadData_Mpu6050* dataOut){
    RawReadData_Mpu6050 raw = {0};
    
    esp_err_t err = get_raw_data_mpu6050(device, &raw);
    if(err!=ESP_OK){
        ESP_LOGE("MPU6050", "Couldn't parse data correctly, invalid entry");
        return err;
    }

    dataOut->accel_x_g = raw.accelerometerData.accel_x / MPU6050_ACCEL_LSB_PER_G - accelOffset_x;
    dataOut->accel_y_g = raw.accelerometerData.accel_y / MPU6050_ACCEL_LSB_PER_G - accelOffset_y;
    dataOut->accel_z_g = raw.accelerometerData.accel_z / MPU6050_ACCEL_LSB_PER_G - accelOffset_z;

    dataOut->temp_c = raw.temperature/340.0 + 36.53;

    dataOut->gyro_x_dps = raw.gyroscopeData.gyro_x / MPU6050_GYRO_LSB_PER_DPS - gyroOffset_x;
    dataOut->gyro_y_dps = raw.gyroscopeData.gyro_y / MPU6050_GYRO_LSB_PER_DPS - gyroOffset_y;
    dataOut->gyro_z_dps = raw.gyroscopeData.gyro_z / MPU6050_GYRO_LSB_PER_DPS - gyroOffset_z;

    return err;
}