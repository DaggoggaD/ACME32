#include "../include/shared.h"

static i2c_master_dev_handle_t mpu6050;
static i2c_master_dev_handle_t bmp280;
static const char* Tag = "ACME32";

static QueueHandle_t sensor_data_queue;
static float groundPressure = 0;
static Vector3 groundUp = {0};


static void init_i2c(){
    esp_err_t err = config_bus_i2c(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    err += config_device_i2c(&bmp280, BMP280_ADDR, I2C_MASTER_FREQ_HZ, 0);
    err += config_device_i2c(&mpu6050, MPU6050_ADDR, I2C_MASTER_FREQ_HZ, 0);
    if(err != 0) ESP_LOGE(Tag, "Couldn't configure i2c properly. Check previous errors");
    
    probe_sensor_i2c(BMP280_ADDR);
    probe_sensor_i2c(MPU6050_ADDR);
}

static void wake_up_devices(){
    wake_up_calibrated_mpu6050(mpu6050);

    bmp280_read_calibration_matrix(bmp280);
    wake_up_bmp280(bmp280);

    get_ground_pressure(bmp280, &groundPressure);
    set_ground_direction(mpu6050, CYCLES_GROUNDUP_CALIBRATION, &groundUp);
}

static void task_get_sensor_data(void* params){
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        ReadData_Bmp280 bmpData = {0};
        ReadData_Mpu6050 mpuData = {0};

        esp_err_t bErr = get_data_bmp280(bmp280, &bmpData);
        if(bErr != ESP_OK) ESP_LOGE("Task_get_sensor_data", "Couldn't get bmp280 data");
        
        esp_err_t mErr = get_data_mpu6050(mpu6050, &mpuData);
        if(mErr != ESP_OK) ESP_LOGE("Task_get_sensor_data", "Couldn't get mpu6050 data");
        
        FlightData data = {
            .accel_ms2 = {mpuData.accel_x_g, mpuData.accel_y_g, mpuData.accel_z_g},
            .gyro_dps = {mpuData.gyro_x_dps, mpuData.gyro_y_dps, mpuData.gyro_z_dps},

            .temp = bmpData.temp_c,
            .press = bmpData.press_hpa,
            
            .time = (uint32_t)(esp_timer_get_time() / 1000ULL)
        };

        esp_err_t err = xQueueSend(sensor_data_queue, &data, 10);
        if(err!=pdTRUE) ESP_LOGW("Task_get_sensor_data", "Queue full");

        vTaskDelayUntil(&lastWakeTime, xFrequency);
    }
} 

static void debug_telemetry(FlightTelemetry* data, FlightData* pack, uint8_t* cycleN, int cyclesToPrint){
    if(*cycleN < cyclesToPrint){
        return;
    }
    *cycleN = 0;

    ESP_LOGI(Tag, "GroundUp: (x: %5.1f , y: %5.1f , z: %5.1f) | Velocity: %5.1f | AccelUP: %5.1f | Accel: (x: %5.1f , y: %5.1f , z: %5.1f) | Gyro: (x: %5.1f, y: %5.1f, z: %5.1f) | Altitude: %5.1f | Temp: %2.1f",
        data->upDir.x,
        data->upDir.y,
        data->upDir.z,
        data->velocityUp,
        data->accelerationUp,
        pack->accel_ms2.x,
        pack->accel_ms2.y,
        pack->accel_ms2.z,
        pack->gyro_dps.x,
        pack->gyro_dps.y,
        pack->gyro_dps.z,
        data->height,
        pack->temp
    );
}

static float update_flight_data(uint32_t *lastTime_ms, FlightData* pack, KalmanState* state){
    // Update time
    if (*lastTime_ms == 0) *lastTime_ms = pack->time;
    float dt = (pack->time - *lastTime_ms) / 1000.0f;
    *lastTime_ms = pack->time;

    // Get up acceleration using gyro, update ground position relative to current rotation
    if (dt > 0.0f) update_ground_up(&(pack->gyro_dps), &groundUp, dt);
    float accelerationUp = get_global_acceleration_ms2(&(pack->accel_ms2), &groundUp);

    // Calculate filtered altitude and velocity
    float rawAlt = get_raw_altitude(pack->press, groundPressure);
    get_kalman_data(state, accelerationUp, rawAlt, dt);

    return accelerationUp;
}



static void task_telemetry(void* params) {
    FlightData pack = {0};
    KalmanState state = {0};
    uint32_t lastTime_ms = 0;
    uint8_t i = 0;
    init_kalman(&state, 0);
    
    while (1) {
        if (xQueueReceive(sensor_data_queue, &pack, portMAX_DELAY) == pdTRUE) {
            float accelerationUp = update_flight_data(&lastTime_ms, &pack, &state);

            float tilt_rad = acos(groundUp.z); 
            float tiltAngle = tilt_rad * (180.0f / M_PI);

            FlightTelemetry flightState = {
                .accelerationUp = accelerationUp,
                .height = state.altitude,
                .velocityUp = state.velocity,
                .tiltAngle_deg = tiltAngle,
                .upDir = groundUp,
                .time = pack.time
            };

            #if DEBUG == 1
            #if TELEPLOT == 1
            printf(">Raw:%.2f\n", get_raw_altitude(pack->press, groundPressure););
            printf(">EMA:%.2f\n", get_EMA_altitude(pack.press, groundPressure));
            printf(">Kalman:%.2f\n", state.altitude);
            #endif

            debug_telemetry(&flightState, &pack, &i, 10);
            #endif

            i++;
        }
    }
}

void app_main(void) {
    init_i2c();
    wake_up_devices();

    sensor_data_queue = xQueueCreate(50, sizeof(FlightData));
    if(sensor_data_queue == NULL) {
        ESP_LOGE(Tag, "Couldn't create seansor queue");
        return;
    }

    xTaskCreate(task_get_sensor_data, "Task_sensors", 4096, NULL, 5, NULL);

    xTaskCreate(task_telemetry, "Task_telemetry", 4096, NULL, 2, NULL);

}