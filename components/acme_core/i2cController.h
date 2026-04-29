#ifndef I2C_CONTROLLER_H
#define I2C_CONTROLLER_H

#include "esp_log.h"
#include "driver/i2c_master.h"

#define TIMEOUT_I2C 1000

esp_err_t config_bus_i2c(int sdaPin, int sclPin);

esp_err_t config_device_i2c(i2c_master_dev_handle_t* devHandle, int address, int sclSpeed, int sclWait);

esp_err_t write_i2c(i2c_master_dev_handle_t devHandle, uint8_t* data, size_t dataSize);

esp_err_t read_sensor_i2c(i2c_master_dev_handle_t devHandle, uint8_t address, uint8_t* dataOut, size_t dataOutS);

esp_err_t probe_sensor_i2c(uint16_t address);

#endif