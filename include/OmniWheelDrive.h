#ifndef OMNI_WHEEL_DRIVE_H
#define OMNI_WHEEL_DRIVE_H

#include "MotorController.h"

class OmniWheelDrive {
private:
    MotorController* motor1;
    MotorController* motor2;
    MotorController* motor3;
    
    float wheelRadius;        // Radius of omni wheels (meters)
    float robotRadius;        // Distance from robot center to wheel (meters)
    
    // Velocity components
    float vx;  // Linear velocity in x direction (forward/backward)
    float vy;  // Linear velocity in y direction (left/right)
    float w;   // Angular velocity (rotation)
    
public:
    OmniWheelDrive(MotorController* m1, MotorController* m2, MotorController* m3,
                   float wheelR = 0.05, float robotR = 0.15);
    
    void begin();
    
    // Set robot velocity in robot frame
    void setVelocity(float vx, float vy, float omega);
    
    // Set individual wheel speeds (for testing)
    void setWheelSpeeds(float w1, float w2, float w3);
    
    // Update all motors
    void update();
    
    // Stop all motors
    void stopAll();
    
    // Get current velocities
    void getCurrentVelocity(float& vx_out, float& vy_out, float& omega_out);
    
private:
    // Inverse kinematics: convert robot velocity to wheel speeds
    void inverseKinematics(float vx, float vy, float omega, 
                          float& w1, float& w2, float& w3);
    
    // Forward kinematics: convert wheel speeds to robot velocity
    void forwardKinematics(float w1, float w2, float w3,
                          float& vx, float& vy, float& omega);
};

#endif
