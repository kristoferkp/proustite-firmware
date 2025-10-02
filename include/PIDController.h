#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
private:
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    
    float setpoint;     // Desired value
    float integral;     // Accumulated error
    float prevError;    // Previous error for derivative
    
    float outputMin;    // Minimum output limit
    float outputMax;    // Maximum output limit
    
    unsigned long lastTime;  // Last update time
    
public:
    PIDController(float Kp, float Ki, float Kd);
    
    void setGains(float Kp, float Ki, float Kd);
    void setSetpoint(float target);
    void setOutputLimits(float min, float max);
    
    float compute(float input);
    void reset();
    
    float getSetpoint() const { return setpoint; }
};

#endif
