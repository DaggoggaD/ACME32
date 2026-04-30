# ACME32 - Avionics Control Module for ESP32

![Status: WIP](https://img.shields.io/badge/status-work--in--progress-orange)
![License: Open Source](https://img.shields.io/badge/license-Open%20Source-blue)

ACME32 is a high-performance, real-time Flight Computer and GNC (Guidance, Navigation, and Control) system designed for amateur and high-power model rocketry. Developed using the ESP-IDF framework on FreeRTOS, it leverages the ESP32 dual-core architecture to ensure deterministic timing for critical flight operations and high-frequency sensor fusion.

## Core Features

### Advanced Sensor Fusion and Navigation
The system integrates an MPU6050 (6-DOF IMU) and a BMP280 (Barometer) at a 100Hz sampling rate. It utilizes the Rodrigues rotation formula to maintain a stable "Ground-Up" reference vector, allowing for accurate vertical acceleration extraction and attitude estimation without gimbal lock issues.

### State Estimation
A custom 1D Kalman Filter processes raw atmospheric pressure and global vertical acceleration. This filter provides real-time, low-latency estimates of altitude and velocity, effectively mitigating barometric noise and the Bernoulli effect during high-velocity flight phases.

### Flight State Machine (FSM)
The mission logic is governed by a deterministic Finite State Machine that manages the following flight phases:
- IDLE: Ground calibration, sensor bias compensation, and launch readiness checks.
- BOOST: Detection of rapid acceleration and motor burn monitoring.
- COAST: Inertial ascent and apogee prediction logic.
- APOGEE: Precision triggering for recovery systems (parachutes).
- DESCENT: Monitoring of terminal velocity and descent stability.
- LANDED: Post-flight data preservation and recovery beacon activation.

### Modular Software Architecture
The project follows a modular component-based structure for scalability:
- acme_core: Centralized I2C bus management and device handling.
- bmp280_driver / mpu6050_driver: Specialized low-level drivers for aeronautical sensors.
- processing: Mathematics library containing Kalman filters, kinematics, and vector math.

## Hardware Requirements
- ESP32 Development Board.
- MPU6050 Accelerometer/Gyroscope via I2C.
- BMP280 Barometer via I2C.
- Recommended: Dedicated power regulation for high-torque servos (for TVC or active fin systems).

## Getting Started

### Prerequisites
- ESP-IDF Framework v5.x or higher.
- CMake and Python 3.x.

### Build and Flash
1. Clone the repository:
   git clone https://github.com/DaggoggaD/ACME32.git
2. Navigate to the project directory:
   cd ACME32
3. Build the firmware:
   idf.py build
4. Flash the device and open the monitor:
   idf.py -p [YOUR_PORT] flash monitor

## Project Structure
- components/: Independent libraries for drivers and data processing.
- main/: Core flight tasks, telemetry logic, and FSM implementation.
- main/include/shared.h: Global definitions, structs, and flight constants.

## Future Development
- Implementation of active PID control for Thrust Vectoring (TVC).
- Integration of microSD card logging for high-rate blackbox data.
- Hardware-In-The-Loop (HITL) simulation support for trajectory validation.

## License
This project is open-source. Consult the LICENSE file for usage terms.