#include "bmp280.h"

#define BMP280_ADDR 0x76

#define BMP280_REG_ID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_CTRL_SETTINGS 0xF4
#define BMP280_REG_CONFIG 0xF5

#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_CALIB_START 0x88

#define GROUND_PRESS_CALIBRATION_CYCLES 100

// ====================================
// BOSH READ COMPENSATION, FROM 
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
// ====================================

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_data;

static bmp280_calib_data calib_matrix;
static int32_t t_fine;

static int32_t bmp280_compensate_T(int32_t adc_T, bmp280_calib_data *calib) {
    int32_t var1, var2, T;
    
    var1 = ((((adc_T>>3) - ((int32_t)calib->dig_T1<<1))) * ((int32_t)calib->dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)calib->dig_T1)) * ((adc_T>>4) - ((int32_t)calib->dig_T1))) >> 12) * ((int32_t)calib->dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    
    return T;
}

static uint32_t bmp280_compensate_P(int32_t adc_P, bmp280_calib_data *calib) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib->dig_P6;
    var2 = var2 + ((var1*(int64_t)calib->dig_P5)<<17);
    var2 = var2 + (((int64_t)calib->dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)calib->dig_P3)>>8) + ((var1 * (int64_t)calib->dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calib->dig_P1)>>33;
    
    if (var1 == 0) {
        return 0;
    }
    
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)calib->dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)calib->dig_P8) * p) >> 19;
    
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib->dig_P7)<<4);
    return (uint32_t)p;
}

esp_err_t bmp280_read_calibration_matrix(i2c_master_dev_handle_t device) {
    esp_rom_delay_us(2000);
    uint8_t buf[24];
    
    esp_err_t err = read_sensor_i2c(device, BMP280_REG_CALIB_START, buf, 24);
    if(err != ESP_OK) return err;

    calib_matrix.dig_T1 = (buf[1] << 8) | buf[0];
    calib_matrix.dig_T2 = (int16_t)((buf[3] << 8) | buf[2]);
    calib_matrix.dig_T3 = (int16_t)((buf[5] << 8) | buf[4]);
    calib_matrix.dig_P1 = (buf[7] << 8) | buf[6];
    calib_matrix.dig_P2 = (int16_t)((buf[9] << 8) | buf[8]);
    calib_matrix.dig_P3 = (int16_t)((buf[11] << 8) | buf[10]);
    calib_matrix.dig_P4 = (int16_t)((buf[13] << 8) | buf[12]);
    calib_matrix.dig_P5 = (int16_t)((buf[15] << 8) | buf[14]);
    calib_matrix.dig_P6 = (int16_t)((buf[17] << 8) | buf[16]);
    calib_matrix.dig_P7 = (int16_t)((buf[19] << 8) | buf[18]);
    calib_matrix.dig_P8 = (int16_t)((buf[21] << 8) | buf[20]);
    calib_matrix.dig_P9 = (int16_t)((buf[23] << 8) | buf[22]);

    ESP_LOGI("BMP280", "Loaded calibration Matrix");
    return ESP_OK;
}

// ====================================
// Driver code
// ====================================

esp_err_t wake_up_bmp280(i2c_master_dev_handle_t device){
    uint8_t data[2] = {BMP280_REG_CTRL_SETTINGS, 0x27};

    esp_err_t err = write_i2c(device, data, 2);
    if(err == ESP_OK) ESP_LOGI("BMP280", "Correctly activated");
    return err;
}

esp_err_t get_data_bmp280(i2c_master_dev_handle_t device, ReadData_Bmp280* dataOut) {
    uint8_t buf[6];
    
    esp_err_t err = read_sensor_i2c(device, BMP280_REG_PRESS_MSB, buf, 6);
    if(err != ESP_OK) {
        ESP_LOGE("BPM280", "Couldn't parse data correctly, invalid entry");
        return err;
    }

    int32_t binPress = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    int32_t binTemp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);

    int32_t rawTemp = bmp280_compensate_T(binTemp, &calib_matrix);
    uint32_t rawPress = bmp280_compensate_P(binPress, &calib_matrix);

    dataOut->temp_c = rawTemp / 100.0f;
    
    dataOut->press_hpa = (rawPress / 256.0f) / 100.0f;

    return err;
}

esp_err_t get_ground_pressure(i2c_master_dev_handle_t device, float* groundP){
    float pressureSum = 0;

    for (int i = 0; i < GROUND_PRESS_CALIBRATION_CYCLES; i++)
    {
        ReadData_Bmp280 data = {0};
        esp_err_t err = get_data_bmp280(device, &data);

        if(err != ESP_OK) {
            ESP_LOGE("BMP280", "Couldn't get ground pressure");
            return err;
        }

        pressureSum += data.press_hpa;
        esp_rom_delay_us(2000);
    }
    
    *groundP = pressureSum / GROUND_PRESS_CALIBRATION_CYCLES;
    return ESP_OK;
}