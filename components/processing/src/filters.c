#include "filters.h"
#define EMA_ALPHA 0.2

#define NOISE_ALT 0.05f
#define NOISE_VEL 0.1f
#define NOISE_BARO 0.1f

static uint8_t firstRead = 1;
static float altitude = 0;

//=======================================
// EMA
//=======================================

float get_raw_altitude(float currPress, float groundPress) {
    if (groundPress <= 0 || currPress <= 0) return 0.0f;

    float base = currPress / groundPress;
    float exponent = 0.1903f;
    return 44330.0f * (1.0f - pow(base, exponent));
}

float get_EMA_altitude(float currPress, float groundPress){
    float rawAltitude = get_raw_altitude(currPress, groundPress);

    if(firstRead == 1) {
        firstRead = 0;
        altitude = rawAltitude;
        return rawAltitude;
    }

    altitude = EMA_ALPHA * rawAltitude + (1-EMA_ALPHA) * altitude;

    return altitude;
}

//=======================================
// Kallman filter
//=======================================

void init_kalman(KalmanState* state, float groundAlt){
    state->acceleration = 0;
    state->velocity = 0;
    state->altitude = groundAlt;
    
    state->P[0][0] = 1;
    state->P[1][1] = 1;
    state->P[0][1] = 0;
    state->P[1][0] = 0;
}

static void altitude_prediction(KalmanState* state, float dt){
    state->altitude += state->velocity * dt + 0.5f * state->acceleration * dt * dt;
    state->velocity += state->acceleration * dt;

    state->P[0][0] += dt * (dt * state->P[1][1] + state->P[0][1] + state->P[1][0] + NOISE_ALT);

    state->P[0][1] += dt * state->P[1][1];
    state->P[1][0] += dt * state->P[1][1];

    state->P[1][1] += NOISE_VEL * dt;
} 

static void kalman_update(KalmanState* state, float currAlt){
    // Calculate height error with baro and kalman approximation difference
    float altErr = currAlt - state->altitude;
    float totalErr = state->P[0][0] + NOISE_BARO;

    if(totalErr == 0) return;
    
    // Use the total error to update height and velocity gains
    float kalGain[2];
    kalGain[0] = state->P[0][0] / totalErr;
    kalGain[1] = state->P[1][0] / totalErr;

    // Update altitude and velocity
    state->altitude += kalGain[0] * altErr;
    state->velocity += kalGain[1] * altErr;


    // Update the propagation matrix
    float currP00 = state->P[0][0];
    float currP01 = state->P[0][1];
    
    state->P[0][0] -= kalGain[0] * currP00;
    state->P[0][1] -= kalGain[0] * currP01;
    state->P[1][0] -= kalGain[1] * currP00;
    state->P[1][1] -= kalGain[1] * currP01;
}

void get_kalman_data(KalmanState* state, float accelZ, float baroAlt, float dt){
    state->acceleration = accelZ;
    altitude_prediction(state, dt);
    kalman_update(state, baroAlt);
}