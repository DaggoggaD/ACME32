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

// Initializes acceleration, height and velocity to 0. Also sets kalman
// propagation matrix.
void init_kalman(KalmanState* state, float groundAlt);

// Updates kalman states
void get_kalman_data(KalmanState* state, float accelZ, float baroAlt, float dt);

// Initializes imu starting up direction. Only one of x,y,z should be set to 1,
// based on the initial mpu6050 orientation
void init_imu_filter(uint8_t x, uint8_t y, uint8_t z, IMUState* state);

// Implementation found at: https://github.com/dccharacter/AHRS/blob/master/MahonyAHRS.c
void update_imu_filter(IMUState* state, IMUInput* DataIn, float dt);

// Updates the Up vector accounting for the rocket rotation in time
void get_up_vector(IMUState* state, float* upX, float* upY, float* upZ);

#endif