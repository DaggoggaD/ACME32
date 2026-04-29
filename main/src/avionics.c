#include "avionics.h"

float get_vector_module(const Vector3* vec){
    return sqrt(
        vec->x * vec->x + 
        vec->y * vec->y + 
        vec->z * vec->z
    );
}

void update_ground_up(const Vector3* gyro_dps, Vector3* outGroundUp, float dt){
    float wx = DEG_TO_RAD(gyro_dps->x);
    float wy = DEG_TO_RAD(gyro_dps->y);
    float wz = DEG_TO_RAD(gyro_dps->z);

    // Rodrigues formula for ground rotation:

    float w_mag = sqrt(wx*wx + wy*wy + wz*wz);
    if (w_mag < 0.0001f) return; 

    float theta = w_mag * dt;

    float kx = -(wx / w_mag);
    float ky = -(wy / w_mag);
    float kz = -(wz / w_mag);

    float vx = outGroundUp->x;
    float vy = outGroundUp->y;
    float vz = outGroundUp->z;

    float cos_theta = cos(theta);
    float sin_theta = sin(theta);

    float k_dot_v = (kx*vx + ky*vy + kz*vz);

    float cross_x = ky*vz - kz*vy;
    float cross_y = kz*vx - kx*vz;
    float cross_z = kx*vy - ky*vx;

    outGroundUp->x = vx * cos_theta + cross_x * sin_theta + kx * k_dot_v * (1.0f - cos_theta);
    outGroundUp->y = vy * cos_theta + cross_y * sin_theta + ky * k_dot_v * (1.0f - cos_theta);
    outGroundUp->z = vz * cos_theta + cross_z * sin_theta + kz * k_dot_v * (1.0f - cos_theta);

    float mod = get_vector_module(outGroundUp);
    if(mod > 0.0f) {
        outGroundUp->x /= mod;
        outGroundUp->y /= mod;
        outGroundUp->z /= mod;
    }
}

float dot_product(const Vector3* a, const Vector3* b){
    return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

esp_err_t set_ground_direction(i2c_master_dev_handle_t accelerometer, int cycles, Vector3* outGroundUp){
    Vector3 curr = {0};

    for (int i = 0; i < cycles; i++)
    {
        ReadData_Mpu6050 currAccel = {0};

        esp_err_t err = get_data_mpu6050(accelerometer, &currAccel);
        if(err != ESP_OK) {
            ESP_LOGE("Avionics", "Could not configure ground up direction");
            return err;
        }

        curr.x += currAccel.accel_x_g;
        curr.y += currAccel.accel_y_g;
        curr.z += currAccel.accel_z_g;
    }

    curr.x /= cycles;
    curr.y /= cycles;
    curr.z /= cycles;

    float mod = get_vector_module(&curr);
    if(mod > 0.9f && mod < 1.1f){
        outGroundUp->x = curr.x/mod;
        outGroundUp->y = curr.y/mod;
        outGroundUp->z = curr.z/mod;
    }
    else {
        ESP_LOGE("Avionics", "Could not find ground up direction (mod error)");
        return ESP_ERR_NOT_ALLOWED;
    }
    
    return ESP_OK;
}

float get_global_acceleration_ms2(const Vector3* localAcceleration, const Vector3* groundUp){
    float a = dot_product(localAcceleration, groundUp);
    return (a-1.0f)*9.81f;
}