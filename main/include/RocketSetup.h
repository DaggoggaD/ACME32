#ifndef ROCKET_SETUP_H
#define ROCKET_SETUP_H

// =======================================================================================
//                                    Components setup
// =======================================================================================


// Additional components (WIP, still no pin support)
#define HAS_PARACHUTE 0

#define HAS_FINDME_BUZZER 0
#define BUZZ_PIN 19

#define HAS_CONTROL_FINS 0
#define HAS_THRUST_VECTORING 0

// Sets the direction of the rocket, viewed from the
// MPU6050. On the chip, check the direction of the arrows
// and select the one that points to the sky when the rocket will
// be on the launchpad.
#define UP_X 0
#define UP_Y 0
#define UP_Z 1

// Number of cycles to calibrate the ground upwards directions.
#define CYCLES_GROUNDUP_CALIBRATION 100

// Sets the maximum module for wich the gyro will be updated,
// done to avoid excessive gyro drift on shakeier components.
#define GYRO_DRIFT_DEADBAND_FILTER 1.0f

// How often should the board get data from the components
// (i.e. the frequency of the get_sensor data).
// The frequency correspond to 1000 * 1/RETRIEVE_SENSOR_DATA_MS
// Lower values means higher frequency.
#define RETRIEVE_SENSOR_DATA_MS 10

// I2C pins setup
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21

// =======================================================================================
//                              Task handling and core setup
// =======================================================================================

// Fast core should be used for calculation/sensor data fetching,
// Slow core should be used to write to sd card/html setup display and
// other quality of life addons.
#define FAST_CORE 1
#define SLOW_CORE 0


// =======================================================================================
//                                      VSCode debug
// ======================================================================================= 

// DEBUG will print precise updates at higher frequency (currently non utilized)
// Teleplot is used to display raw altitude (and possibly other data) on a graph in VSCode.
#define DEBUG 1
#define TELEPLOT 0

#endif