#ifndef BMP_280_H
#define BMP_280_H

#include "esp_log.h"
#include "driver/i2c_master.h"
#include "i2cController.h"
#include "esp_rom_sys.h"

typedef struct {
    float temp_c;
    float press_hpa;
} ReadData_Bmp280;

esp_err_t bmp280_read_calibration_matrix(i2c_master_dev_handle_t device);

esp_err_t wake_up_bmp280(i2c_master_dev_handle_t device);

esp_err_t get_data_bmp280(i2c_master_dev_handle_t device, ReadData_Bmp280* dataOut);

esp_err_t get_ground_pressure(i2c_master_dev_handle_t device, float* groundP);

#endif