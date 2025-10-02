#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float Kp, float Ki, float Kd) 
    : kp(Kp), ki(Ki), kd(Kd), setpoint(0), integral(0), 
      prevError(0), outputMin(-255), outputMax(255), lastTime(0) {
}

void PIDController::setGains(float Kp, float Ki, float Kd) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

void PIDController::setSetpoint(float target) {
    setpoint = target;
}

void PIDController::setOutputLimits(float min, float max) {
    outputMin = min;
    outputMax = max;
}

float PIDController::compute(float input) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
    
    if (lastTime == 0 || dt <= 0) {
        lastTime = currentTime;
        return 0;
    }
    
    // Calculate error
    float error = setpoint - input;
    
    // Proportional term
    float P = kp * error;
    
    // Integral term with anti-windup
    integral += error * dt;
    float I = ki * integral;
    
    // Derivative term
    float derivative = (error - prevError) / dt;
    float D = kd * derivative;
    
    // Calculate output
    float output = P + I + D;
    
    // Apply output limits and anti-windup
    if (output > outputMax) {
        output = outputMax;
        // Anti-windup: prevent integral from growing when saturated
        integral -= error * dt;
    } else if (output < outputMin) {
        output = outputMin;
        // Anti-windup: prevent integral from growing when saturated
        integral -= error * dt;
    }
    
    // Save state for next iteration
    prevError = error;
    lastTime = currentTime;
    
    return output;
}

void PIDController::reset() {
    integral = 0;
    prevError = 0;
    lastTime = 0;
}
