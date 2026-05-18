#include "pid.h"

void init_pid(PIDController *pid, float kp, float ki, float kd, float maxOut, float maxIntegral) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->maxOutput = maxOut;
    pid->maxIntegral = maxIntegral;
    
    pid->integral = 0.0f;
    pid->previousError = 0.0f;
}

void reset_pid(PIDController *pid) {
    // Call on BOOST/ARMED state to delete integral error accumulation on launch wait.

    pid->integral = 0.0f;
    pid->previousError = 0.0f;
}

float update_pid(PIDController *pid, float setpoint, float measuredValue, float dt) {
    // Calculates necessary fin rotation 

    if (dt <= 0.0001f) return 0.0f;

    // Calculate the error, using setpoint as the desired angle.
    // On custom trajectories, change this value according to the desired
    // direction.
    float error = setpoint - measuredValue;
    float pOut = pid->kp * error;

    // Integral error propagation
    pid->integral += (error * dt);
    
    // On maximum wing rotation, the integral would explode to "infinity".
    // To avoid this, we clamp the result to a max and min.
    if (pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    if (pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
    
    float iOut = pid->ki * pid->integral;

    // Derivative error propagation
    float derivative = (error - pid->previousError) / dt;
    float dOut = pid->kd * derivative;

    pid->previousError = error;
    float total_output = pOut + iOut + dOut;

    // Max fins rotoation clamping
    if (total_output > pid->maxOutput) total_output = pid->maxOutput;
    if (total_output < -pid->maxOutput) total_output = -pid->maxOutput;

    return total_output;
}