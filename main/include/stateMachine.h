#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "shared.h"

typedef enum {
    S_IDLE,
    S_BOOST,
    S_COAST,
    S_APOGEE,
    S_DESCENT,
    S_LANDED,
    S_ERR
} FlightState;

#endif