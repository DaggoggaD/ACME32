idf_component_register(SRCS "i2cController.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c spi_flash driver freertos log esp_common)