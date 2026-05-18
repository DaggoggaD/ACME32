#ifndef ROCKET_SETUP_H
#define ROCKET_SETUP_H

// =======================================================================================
//                                    Components setup
// =======================================================================================


// Additional components

#define HAS_PARACHUTE 0
#define EMERGENCT_DEPLOY_ANGLE_DG 45
#define SERVO_PIN_CHUTE 0

#define HAS_FINDME_BUZZER 1
#define BUZZ_PIN 19

#define HAS_SD_READER 0

#define HAS_CONTROL_FINS 1
#define NUM_FINS 4
#define SERVO_PIN_NORTH 32
#define SERVO_PIN_SOUTH 27
#define SERVO_PIN_EAST  13 
#define SERVO_PIN_WEST  25

#define HAS_THRUST_VECTORING 0

// Sets the direction of the rocket, viewed from the
// MPU6050. On the chip, check the direction of the arrows
// and select the one that points to the sky when the rocket will
// be on the launchpad.
#define UP_X 1
#define UP_Y 0
#define UP_Z 0

// Number of cycles to calibrate the ground upwards directions.
#define CYCLES_GROUNDUP_CALIBRATION 100

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