#include "stateMachine.h"

static const char* Tag = "FSM";

static void check_boost_transition(FlightState* state, FlightTelemetry* telemetry){
    static int8_t triggerCount = 0;

    if(telemetry->accelerationUp > ACCELERATION_TRANSITION_TO_BOOST){
        triggerCount++;

        if(triggerCount > 3){
            *state = S_BOOST;
            triggerCount = 0;
            ESP_LOGI(Tag, "Liftoff detected, switching to BOOST state");
        }
    }
    else triggerCount = 0;
}

static void check_coast_transition(FlightState* state, FlightTelemetry* telemetry){
    static int8_t triggerCount = 0;

    if(telemetry->accelerationUp < ACCELERATION_TRANSITION_TO_COAST){
        triggerCount++;

        if(triggerCount > 3){
            *state = S_COAST;
            triggerCount = 0;
            ESP_LOGI(Tag, "Booster shutoff, switching to COAST state");
        }
    }
    else triggerCount = 0;
}

static void check_apogee_transition(FlightState* state, FlightTelemetry* telemetry){
    static int8_t triggerCount = 0;

    if(telemetry->velocityUp <= VELOCITY_TRANSITION_TO_APOGEE){
        triggerCount++;

        if(triggerCount > 3){
            *state = S_APOGEE;
            triggerCount = 0;
            ESP_LOGI(Tag, "Maximum altitude of %5.1f reached, switching to APOGEE state", telemetry->height);
        }
    }
    else triggerCount = 0;
}

static void check_descent_transition(FlightState* state, FlightTelemetry* telemetry){
    *state = S_DESCENT;
    ESP_LOGI(Tag, "Switching to descent state");
}

static void check_landed_transition(FlightState* state, FlightTelemetry* telemetry){
    static uint32_t landingTimerStart = 0;

    if (telemetry->height < MAX_HEIGHT_TRANSITION_TO_LANDED && fabs(telemetry->velocityUp) < VELOCITY_TRANSITION_TO_LANDED) { 
        if (landingTimerStart == 0) landingTimerStart = telemetry->time;
        else if ((telemetry->time - landingTimerStart) > 2000) { 
            *state = S_LANDED;
            ESP_LOGI(Tag, "Touchdown detected, switching to LANDED state");
        }
    } else {
        landingTimerStart = 0;
    }
}

esp_err_t state_handler(FlightState* currentFlightState, FlightTelemetry* telemetry){

    switch (*currentFlightState)
    {
    case S_IDLE:
        // Beep sound to signal idle state, reduced sd telemetry.
        

        check_boost_transition(currentFlightState, telemetry);
        break;
    
    case S_BOOST:
        // Max sd telemtry writing. If available, activate control fins.
        

        check_coast_transition(currentFlightState, telemetry);
        break;

    case S_COAST:
        // Max sd telemtry writing. If available, activate control fins.


        check_apogee_transition(currentFlightState, telemetry);
        break;

    case S_APOGEE:
        // Open parachute.
        
        check_descent_transition(currentFlightState, telemetry);
        break;

    case S_DESCENT:
        // Reduced sd telemetry.

        check_landed_transition(currentFlightState, telemetry);
        break;

    case S_LANDED:
        // Save flight to SD card, start continuos beeping.

        
        break;

    default:
        break;
    }

    return ESP_OK;
}