#ifndef FILTERS_H
#define FILTERS_H

#include <math.h>

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

float get_raw_altitude(float currPress, float groundPress);

float get_EMA_altitude(float currPress, float groundPress);

void init_kalman(KalmanState* state, float groundAlt);

void get_kalman_data(KalmanState* state, float accelZ, float baroAlt, float dt);
#endif