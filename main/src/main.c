#include "../include/shared.h"
#include "../include/stateMachine.h"

// Devices
static i2c_master_dev_handle_t mpu6050;
static i2c_master_dev_handle_t bmp280;
static const char* Tag = "ACME32";

// Data Queues
static QueueHandle_t sensorDataQueue;
static QueueHandle_t sdTelemetryQueue;

// Ground state, touched by FAST_CORE only
static float groundPressure = 0;
static Vector3 groundUp = {0};


// =======================================================================================
//                                 Calibration & intialization
// =======================================================================================

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

// =======================================================================================
//                               Telemetry calculations helpers
// =======================================================================================

static float update_flight_data(uint32_t *lastTime_ms, FlightData* pack, KalmanState* state){
    // Updates rocket rotation (groundUp vector3), calculates current altitude and velocity, 
    // returns upwards acceleration. Uses a kalman filter to eliminate noise

    // Update time
    if (*lastTime_ms == 0) *lastTime_ms = pack->time;
    float dt = (pack->time - *lastTime_ms) / 1000.0f;
    *lastTime_ms = pack->time;

    // Gyro deadband filtering. Could be improved with "simple" 1D filter.
    if (fabs(pack->gyro_dps.x) < GYRO_DRIFT_DEADBAND_FILTER) pack->gyro_dps.x = 0.0f;
    if (fabs(pack->gyro_dps.y) < GYRO_DRIFT_DEADBAND_FILTER) pack->gyro_dps.y = 0.0f;
    if (fabs(pack->gyro_dps.z) < GYRO_DRIFT_DEADBAND_FILTER) pack->gyro_dps.z = 0.0f;


    // Get upwar acceleration using gyro, update ground position relative to current rotation
    if (dt > 0.0f) update_ground_up(&(pack->gyro_dps), &groundUp, dt);
    float accelerationUp = get_global_acceleration_ms2(&(pack->accel_ms2), &groundUp);

    // Calculate filtered altitude and velocity
    float rawAlt = get_raw_altitude(pack->press, groundPressure);
    get_kalman_data(state, accelerationUp, rawAlt, dt);

    
    return accelerationUp;
}

static float calculate_tilt_angle(FlightData* pack, FlightState state){
    // When in IDLE, the gyro still has some drift.
    // By not resetting ACME32 for a while, the gyro naturally accumulates error,
    // causing ~15°/minute.
    // To avoid this, we can use the initial groundUp direction (start rocket rotation) to
    // filter out the unwanted drift.

    if(state != S_IDLE){
        float tiltRad = 0.0f;
        #ifdef UP_Y
            tiltRad = acos(groundUp.y); 
        #elif defined(UP_X)
            tiltRad = acos(groundUp.x);
        #elif defined(UP_Z)
            tiltRad = acos(groundUp.z);
        #else
            #error "ERROR: Must define UPWARD direction.in shared.h"
        #endif
        return tiltRad * (180.0f / M_PI);
    }

    float accelMagnitude = sqrt(pack->accel_ms2.x * pack->accel_ms2.x + 
        pack->accel_ms2.y * pack->accel_ms2.y + 
        pack->accel_ms2.z * pack->accel_ms2.z);

    
    // If the raw acceleration (considering gravity's influence) is approximately 1,
    // we're on the ground, so we can filter out the gyroscope's drift influence.
    if (accelMagnitude > 9.4f && accelMagnitude < 10.2f) {
        float ax = pack->accel_ms2.x / accelMagnitude;
        float ay = pack->accel_ms2.y / accelMagnitude;
        float az = pack->accel_ms2.z / accelMagnitude;
        
        // Correction
        float alpha = 0.998f; 
        
        groundUp.x = groundUp.x * alpha + ax * (1.0f - alpha);
        groundUp.y = groundUp.y * alpha + ay * (1.0f - alpha);
        groundUp.z = groundUp.z * alpha + az * (1.0f - alpha);

        float mod = get_vector_module(&groundUp);
        if(mod > 0.0f) {
            groundUp.x /= mod;
            groundUp.y /= mod;
            groundUp.z /= mod;
        }
    }

    // Not the best looking, but still decent for now
    float tiltRad = 0.0f;
    #ifdef UP_Y
        tiltRad = acos(groundUp.y); 
    #elif defined(UP_X)
        tiltRad = acos(groundUp.x);
    #elif defined(UP_Z)
        tiltRad = acos(groundUp.z);
    #else
        #error "ERROR: Must define UPWARD direction in shared.h"
    #endif
    return tiltRad * (180.0f / M_PI);
}


// =======================================================================================
//                                     Logging functions
// =======================================================================================

static void debug_telemetry(FlightTelemetry* data, FlightData* pack, uint8_t* cycleN, int cyclesToPrint){
    // Outputs all available data to terminal.

    // #if here and not insite task_telemetry for code clarity
    #if DEBUG == 1
        #if TELEPLOT == 1
            printf(">Raw:%.2f\n", get_raw_altitude(pack->press, groundPressure););
            printf(">EMA:%.2f\n", get_EMA_altitude(pack.press, groundPressure));
            printf(">Kalman:%.2f\n", state.altitude);
        #endif


        
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

    #endif
}

static void write_to_sd(FlightTelemetry* data){
    // Write to SD. Untill no SD reader is added, simply write to console

    ESP_LOGI("SD WRITER", 
        "Acceleration: %5.1f | Velocity: %5.1f | Height: %5.1f | Rotation (Up dir): (x: %5.1f, y: %5.1f, z: %5.1f) | Titl on up axis: %5.1f | FSM State: %1d",
        data->accelerationUp,
        data->velocityUp,
        data->height,

        data->upDir.x,
        data->upDir.y,
        data->upDir.z,
        data->tiltAngle_deg,
        
        data->FSMstate
    );

}

static void close_sd_file(){
    // Close SD on landing.
}

static void sd_handler(FlightTelemetry* data, uint32_t packetCount){
    // Writes data to sd at appropriate speed based on current state. 

    switch(data->FSMstate) {
        case S_IDLE:
            if (packetCount % 100 == 0) write_to_sd(data);
            break;
        
        case S_BOOST: case S_COAST: case S_APOGEE:
            write_to_sd(data);
            break;
        
        case S_DESCENT:
            if (packetCount % 10 == 0) write_to_sd(data);
            break;

        case S_LANDED:
            close_sd_file();
            break;
    }

}


// =======================================================================================
//                                          Tasks
// =======================================================================================

static void task_get_sensor_data(void* params){
    // Get sensor data, package them in a task_telemetry 
    // understandable format (FlightData), and send them. 
    // Fixed frequency of 1000 * 1/RETRIEVE_SENSOR_DATA_MS.

    const TickType_t xFrequency = pdMS_TO_TICKS(RETRIEVE_SENSOR_DATA_MS);
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
            .accel_ms2 = {mpuData.accel_x_g*9.81f, mpuData.accel_y_g*9.81f, mpuData.accel_z_g*9.81f},
            .gyro_dps = {mpuData.gyro_x_dps, mpuData.gyro_y_dps, mpuData.gyro_z_dps},

            .temp = bmpData.temp_c,
            .press = bmpData.press_hpa,
            
            .time = (uint32_t)(esp_timer_get_time() / 1000ULL)
        };

        esp_err_t err = xQueueSend(sensorDataQueue, &data, 10);
        if(err!=pdTRUE) ESP_LOGW("Task_get_sensor_data", "Queue full");

        vTaskDelayUntil(&lastWakeTime, xFrequency);
    }
} 

static void task_telemetry(void* params) {
    // Retreive data from sensors (task_get_sensor_data), process them and 
    // send Flight Telemetry to the Flight state Machine.
    // No fixed frequency, as it must handle all packages from task_get_sensor_data immediatly.
    // "Wake up" on package arrival

    FlightData pack = {0};
    KalmanState KState = {0};
    FlightState FSMState = S_IDLE;
    uint32_t lastTime_ms = 0;
    uint8_t i = 0;

    init_kalman(&KState, 0);
    
    while (1) {
        if (xQueueReceive(sensorDataQueue, &pack, portMAX_DELAY) == pdTRUE) {

            float accelerationUp = update_flight_data(&lastTime_ms, &pack, &KState);
            float tiltAngle = calculate_tilt_angle(&pack, FSMState);
            
            FlightTelemetry flightState = {
                .accelerationUp = accelerationUp,
                .height = KState.altitude,
                .velocityUp = KState.velocity,
                .tiltAngle_deg = tiltAngle,
                .upDir = groundUp,
                .time = pack.time
            };

            state_handler(&FSMState, &flightState);
            flightState.FSMstate = FSMState;
            xQueueSend(sdTelemetryQueue, &flightState, 0);

            //debug_telemetry(&flightState, &pack, &i, 10);

            i++;
        }
    }
}

static void task_sd_log(void* params){
    // Receive data from Flight state machine, write to sd at
    // rate specified by current Flight State.
    // Wake up on package arrival.

    FlightTelemetry data;
    uint32_t packet_counter = 0;

    while(1){
        if(xQueueReceive(sdTelemetryQueue, &data, portMAX_DELAY) == pdTRUE){
            packet_counter++;
            sd_handler(&data, packet_counter);
        }
    }
}


void app_main(void) {
    init_i2c();
    wake_up_devices();

    sensorDataQueue = xQueueCreate(50, sizeof(FlightData));
    sdTelemetryQueue = xQueueCreate(50, sizeof(FlightTelemetry));

    if(sensorDataQueue == NULL) {
        ESP_LOGE(Tag, "Couldn't create seansor queue");
        return;
    }

    xTaskCreatePinnedToCore(task_get_sensor_data, "Task_sensors", 4096, NULL, 5, NULL, FAST_CORE);
    xTaskCreatePinnedToCore(task_telemetry, "Task_telemetry", 4096, NULL, 2, NULL, FAST_CORE);
    xTaskCreatePinnedToCore(task_sd_log, "Task_sd_logger", 4096, NULL, 0, NULL, SLOW_CORE);
}