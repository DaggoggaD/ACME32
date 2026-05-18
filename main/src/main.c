#include "../include/shared.h"
#include "../include/stateMachine.h"
#include "servos.h"
#include "buzzer.h"
#include "pid.h"


// Devices
static i2c_master_dev_handle_t mpu6050;
static i2c_master_dev_handle_t bmp280;
static const char* Tag = "ACME32";

// Data Queues
static QueueHandle_t sensorDataQueue;
static QueueHandle_t sdTelemetryQueue;

// Ground state, touched by FAST_CORE only
// and modified only by update_flight_data.
static float groundPressure = 0;
static Vector3 groundUp = {0};

PIDController pitchPid;
PIDController yawPid;


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
    ESP_LOGI(Tag, "Starting calibration in 2s"); 
    vTaskDelay(pdMS_TO_TICKS(2000));


    // Sensors initializations
    wake_up_calibrated_mpu6050(mpu6050, (float[3]){UP_X, UP_Y, UP_Z});

    bmp280_read_calibration_matrix(bmp280);
    wake_up_bmp280(bmp280);

    get_ground_pressure(bmp280, &groundPressure);
    set_ground_direction(mpu6050, CYCLES_GROUNDUP_CALIBRATION, &groundUp);


    // Fins initialization and testing
    #if HAS_CONTROL_FINS == 1
    const int servoPins[NUM_FINS] = {
        SERVO_PIN_NORTH,
        SERVO_PIN_SOUTH,
        SERVO_PIN_EAST,
        SERVO_PIN_WEST
    };

    init_servos(servoPins);
    init_pid(&pitchPid, 1.0f, 0.0f, 0.2f, 15.0f, 5.0f);
    init_pid(&yawPid, 1.0f, 0.0f, 0.2f, 15.0f, 5.0f);
    #endif

    // Communication components
    #if HAS_FINDME_BUZZER == 1
    init_buzzer(BUZZ_PIN);
    #endif
    
    #if HAS_SD_READER == 1
    //init_sd_reader();
    #endif

}

// =======================================================================================
//                               Telemetry calculations helpers
// =======================================================================================

static float update_time(uint32_t* lastTime_ms, ReadingsData* pack){
    if (*lastTime_ms == 0) *lastTime_ms = pack->time;
    float dt = (pack->time - *lastTime_ms) / 1000.0f;
    *lastTime_ms = pack->time;
    return dt;
}

static float update_flight_data(ReadingsData* pack, KalmanState* state, float dt){
    // Updates rocket rotation (groundUp vector3), calculates current altitude and velocity, 
    // returns upwards acceleration. Uses a kalman filter to eliminate noise


    // Get upwar acceleration using gyro, update ground position relative to current rotation
    //if (dt > 0.0f) update_ground_up(&(pack->gyro_dps), &groundUp, dt);
    float accelerationUp = get_global_acceleration_ms2(&(pack->accel_ms2), &groundUp);

    // Calculate filtered altitude and velocity
    float rawAlt = get_raw_altitude(pack->press, groundPressure);
    get_kalman_data(state, accelerationUp, rawAlt, dt);
    
    return accelerationUp;
}

static float calculate_tilt_angle(ReadingsData* pack, FlightState state){
    // IDLE prograssive clamping is no longer needed.
    // This function calculates the tilt on the up axis, along the two "floor"
    // directions, to be used in parachute ejection on emergency.
    
    float up_val = 0.0f;
    #if UP_Y == 1
        up_val = groundUp.y;
    #elif UP_X == 1
        up_val = groundUp.x;
    #elif UP_Z == 1
        up_val = groundUp.z;
    #else
        #error "ERROR: Must define UPWARD direction in shared.h"
    #endif

    // Clamping to avoid nans
    if (up_val > 1.0f) up_val = 1.0f;
    if (up_val < -1.0f) up_val = -1.0f;

    float tiltRad = acosf(up_val);
    
    return tiltRad * (180.0f / (float)M_PI);
}

static inline void calculate_pitch_yaw(float *pitchOut, float *yawOut){
    #if UP_Y == 1
    *pitchOut = atan2f(groundUp.z, sqrtf(groundUp.x * groundUp.x + groundUp.y * groundUp.y)) * (180.0f / (float)M_PI);
    *yawOut = atan2f(groundUp.x, sqrtf(groundUp.z * groundUp.z + groundUp.y * groundUp.y)) * (180.0f / (float)M_PI);
    #endif

    #if UP_Z == 1
    *pitchOut = atan2f(groundUp.x, sqrtf(groundUp.y * groundUp.y + groundUp.z * groundUp.z)) * (180.0f / (float)M_PI);
    *yawOut = atan2f(groundUp.y, sqrtf(groundUp.x * groundUp.x + groundUp.z * groundUp.z)) * (180.0f / (float)M_PI);
    #endif
    
    #if UP_X == 1
    *pitchOut = atan2f(groundUp.y, sqrtf(groundUp.z * groundUp.z + groundUp.x * groundUp.x)) * (180.0f / (float)M_PI);
    *yawOut = atan2f(groundUp.z, sqrtf(groundUp.y * groundUp.y + groundUp.x * groundUp.x)) * (180.0f / (float)M_PI);
    #endif

}

// =======================================================================================
//                                     Logging functions
// =======================================================================================

static void write_to_sd(FlightTelemetry* data){
    // Write to SD. Untill no SD reader is added, simply write to console

    ESP_LOGI("SD WRITER", 
        "Acceleration: %5.1f | Velocity: %5.1f | Height: %5.1f | Rotation (Up dir): (x: %5.1f, y: %5.1f, z: %5.1f) | Pitch: %5.1f | Yaw: %5.1f | Titl on up axis: %5.1f | FSM State: %1d",
        data->accelerationUp,
        data->velocityUp,
        data->height,

        data->upDir.x,
        data->upDir.y,
        data->upDir.z,

        data->pitchAngle_deg,
        data->yawAngle_deg,

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
        case S_IDLE: case S_ARMED:
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
        if(bErr != ESP_OK) ESP_LOGE("GetSensorData", "Couldn't get bmp280 data");
        
        esp_err_t mErr = get_data_mpu6050(mpu6050, &mpuData);
        if(mErr != ESP_OK) ESP_LOGE("GetSensorData", "Couldn't get mpu6050 data");
        
        ReadingsData data = {
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

    const float dpsToRads = (M_PI / 180.0f);

    ReadingsData pack = {0};
    KalmanState KState = {0};
    IMUState IState = {0};
    
    FlightState FSMState = S_IDLE;
    uint32_t lastTime_ms = 0;
    uint8_t i = 0;

    init_kalman(&KState, 0);
    init_imu_filter(UP_X, UP_Y, UP_Z, &IState);
    
    while (1) {
        if (xQueueReceive(sensorDataQueue, &pack, portMAX_DELAY) == pdTRUE) {
            float dt = update_time(&lastTime_ms, &pack);

            IMUInput currInput = {
                .accelX = pack.accel_ms2.x,
                .accelY = pack.accel_ms2.y,
                .accelZ = pack.accel_ms2.z,

                .gyroX = pack.gyro_dps.x * dpsToRads,
                .gyroY = pack.gyro_dps.y * dpsToRads,
                .gyroZ = pack.gyro_dps.z * dpsToRads
            };

            // Rocket's rotation
            update_imu_filter(&IState, &currInput, dt);
            get_up_vector(&IState, &groundUp.x, &groundUp.y, &groundUp.z);
            float tiltAngle = calculate_tilt_angle(&pack, FSMState);
            float pitch = 0, yaw = 0;
            calculate_pitch_yaw(&pitch, &yaw);

            // Rocket's position
            float accelerationUp = update_flight_data(&pack, &KState, dt);

            // Send data to state machine
            FlightTelemetry flightState = {
                .accelerationUp = accelerationUp,
                .height = KState.altitude,
                .velocityUp = KState.velocity,
                .pitchAngle_deg = pitch,
                .yawAngle_deg = yaw,
                .tiltAngle_deg = tiltAngle,
                .upDir = groundUp,
                .time = pack.time
            };

            // TEMP fin control
            float pitchOut = update_pid(&pitchPid, 0.0f, pitch, dt);
            float yawOut = update_pid(&yawPid, 0.0f, yaw, dt);

            set_fin_angle(ServoNorth, 90.0f + pitchOut);
            set_fin_angle(ServoSouth, 90.0f - pitchOut);

            set_fin_angle(ServoEast,  90.0f + yawOut);
            set_fin_angle(ServoWest,  90.0f - yawOut);

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

    sensorDataQueue = xQueueCreate(50, sizeof(ReadingsData));
    sdTelemetryQueue = xQueueCreate(50, sizeof(FlightTelemetry));

    if(sensorDataQueue == NULL) {
        ESP_LOGE(Tag, "Couldn't create seansor queue");
        return;
    }

    xTaskCreatePinnedToCore(task_get_sensor_data, "Task_sensors", 4096, NULL, 5, NULL, FAST_CORE);
    xTaskCreatePinnedToCore(task_telemetry, "Task_telemetry", 4096, NULL, 2, NULL, FAST_CORE);
    xTaskCreatePinnedToCore(task_sd_log, "Task_sd_logger", 4096, NULL, 0, NULL, SLOW_CORE);
}