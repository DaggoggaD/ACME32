idf_component_register(SRC_DIRS "src"
                       INCLUDE_DIRS "include"
                       REQUIRES acme_core servos mpu6050_driver bmp280_driver esp_timer processing buzzer esp_driver_mcpwm)