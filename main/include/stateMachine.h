#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "shared.h"

#define ACCELERATION_TRANSITION_TO_BOOST 20.0f //ms2
#define ACCELERATION_TRANSITION_TO_COAST 0.0f //ms2
#define VELOCITY_TRANSITION_TO_APOGEE 0.0f //ms
#define MAX_HEIGHT_TRANSITION_TO_LANDED 15.0f //m
#define VELOCITY_TRANSITION_TO_LANDED 2.0f //ms
#define TIME_ELAPSED_TO_LANDED 2000 //s

esp_err_t state_handler(FlightState* currentFlightState, FlightTelemetry* telemetry);

#endif