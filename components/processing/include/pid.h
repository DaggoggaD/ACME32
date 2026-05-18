#ifndef PID_H
#define PID_H

typedef struct {
    float kp;       // Proportional constant
    float ki;       // Integral constant
    float kd;       // Derivative constant
    
    float integral;
    float previousError;
    
    float maxIntegral;
    float maxOutput;
} PIDController;

// Initializes pid. Configure kp, ki and kd in "RocketSetup.h"
void init_pid(PIDController *pid, float kp, float ki, float kd, float max_out, float max_int);

// Pid retroaction calculation.
float update_pid(PIDController *pid, float setpoint, float measured_value, float dt);

// Reset boost on boost stage, to account for integral degradation on launch wait.
void reset_pid(PIDController *pid);

#endif // PID_H