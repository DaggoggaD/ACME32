#ifndef FILTERS_H
#define FILTERS_H

#include <math.h>

typedef struct IMUInput {
    float gyroX; 
    float gyroY; 
    float gyroZ;
    
    float accelX; 
    float accelY; 
    float accelZ;
} IMUInput;

typedef struct KalmanState {
    float altitude;
    float velocity;
    float acceleration;

    // Propagation matrix
    // 00: altitude uncertainty
    // 11: velocity uncertainty
    // 01, 10 covariance
    float P[2][2];

} KalmanState;

typedef struct IMUState{
    float q0;
    float q1;
    float q2;
    float q3;
} IMUState;

float get_raw_altitude(float currPress, float groundPress);

float get_EMA_altitude(float currPress, float groundPress);

void init_kalman(KalmanState* state, float groundAlt);

void get_kalman_data(KalmanState* state, float accelZ, float baroAlt, float dt);

void init_imu_filter(IMUState* state);

void update_imu_filter(IMUState* state, IMUInput* DataIn, float dt);

void get_up_vector(IMUState* state, float* upX, float* upY, float* upZ);

#endif