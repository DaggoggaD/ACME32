#include "i2cController.h"

static i2c_master_bus_handle_t i2cBusHandle;
static const char* Tag = "ACME32";

esp_err_t config_bus_i2c(int sdaPin, int sclPin){
    i2c_master_bus_config_t busConfig = {
        .i2c_port = -1,
        .sda_io_num = sdaPin,
        .scl_io_num = sclPin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&busConfig, &i2cBusHandle);

    if(err == ESP_OK) ESP_LOGI(Tag, "Completed bus config correctly");
    else ESP_LOGE(Tag, "Couldn't complete bus configuration correctly");
    return err;
}

esp_err_t config_device_i2c(i2c_master_dev_handle_t* devHandle, int address, int sclSpeed, int sclWait){
    if(devHandle==NULL) {
        ESP_LOGE(Tag, "Null device handle in config_device_i2c");
        return ESP_ERR_NOT_FOUND;
    }
    i2c_device_config_t deviceConfig = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = address,
        .scl_speed_hz = sclSpeed,
        .scl_wait_us = sclWait
    };

    esp_err_t err = i2c_master_bus_add_device(i2cBusHandle, &deviceConfig, devHandle);
    if(err == ESP_OK) ESP_LOGI(Tag, "Completed device configuration at address %d", address);
    else ESP_LOGE(Tag, "Couldn't complete device configuration at address %d", address);
    return err;
}

esp_err_t write_i2c(i2c_master_dev_handle_t devHandle, uint8_t* data, size_t dataSize){
    return i2c_master_transmit(devHandle, data, dataSize, TIMEOUT_I2C);
}

esp_err_t read_sensor_i2c(i2c_master_dev_handle_t devHandle, uint8_t address, uint8_t* dataOut, size_t dataOutS){
    return i2c_master_transmit_receive(devHandle, &address, 1, dataOut, dataOutS, TIMEOUT_I2C);
}

esp_err_t probe_sensor_i2c(uint16_t address){
    esp_err_t err = i2c_master_probe(i2cBusHandle, address, -1); 
    if(err == ESP_OK) {
        ESP_LOGI(Tag, "Device found at address: %d", address);
    }
    else ESP_LOGW(Tag, "Couldn't find device at address %d", address);
    return err;
}