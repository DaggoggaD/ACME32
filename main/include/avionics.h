#ifndef AVIONICS_H
#define AVIONICS_H
#define DEG_TO_RAD(x) ((x) * 3.14159265f / 180.0f)

#include <math.h>
#include "shared.h"
typedef struct Vector3 Vector3;

void update_ground_up(const Vector3* gyro_dps, Vector3* outGroundUp, float dt);

esp_err_t set_ground_direction(i2c_master_dev_handle_t accelerometer, int cycles, Vector3* outGroundUp);

float get_global_acceleration_ms2(const Vector3* localAcceleration, const Vector3* groundUp);

#endif