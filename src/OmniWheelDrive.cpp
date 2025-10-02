#include "OmniWheelDrive.h"
#include <Arduino.h>
#include <math.h>

OmniWheelDrive::OmniWheelDrive(MotorController* m1, MotorController* m2, MotorController* m3,
                               float wheelR, float robotR)
    : motor1(m1), motor2(m2), motor3(m3), 
      wheelRadius(wheelR), robotRadius(robotR),
      vx(0), vy(0), w(0) {
}

void OmniWheelDrive::begin() {
    motor1->begin();
    motor2->begin();
    motor3->begin();
}

void OmniWheelDrive::setVelocity(float vx_in, float vy_in, float omega) {
    vx = vx_in;
    vy = vy_in;
    w = omega;
    
    // Calculate wheel speeds using inverse kinematics
    float w1, w2, w3;
    inverseKinematics(vx, vy, omega, w1, w2, w3);
    
    // Set target speeds for each motor
    motor1->setTargetSpeed(w1);
    motor2->setTargetSpeed(w2);
    motor3->setTargetSpeed(w3);
}

void OmniWheelDrive::setWheelSpeeds(float w1, float w2, float w3) {
    motor1->setTargetSpeed(w1);
    motor2->setTargetSpeed(w2);
    motor3->setTargetSpeed(w3);
}

void OmniWheelDrive::update() {
    motor1->update();
    motor2->update();
    motor3->update();
}

void OmniWheelDrive::stopAll() {
    motor1->stop();
    motor2->stop();
    motor3->stop();
    vx = 0;
    vy = 0;
    w = 0;
}

void OmniWheelDrive::getCurrentVelocity(float& vx_out, float& vy_out, float& omega_out) {
    // Get current wheel speeds
    float w1 = motor1->getCurrentSpeed();
    float w2 = motor2->getCurrentSpeed();
    float w3 = motor3->getCurrentSpeed();
    
    // Calculate robot velocity using forward kinematics
    forwardKinematics(w1, w2, w3, vx_out, vy_out, omega_out);
}

void OmniWheelDrive::inverseKinematics(float vx, float vy, float omega,
                                      float& w1, float& w2, float& w3) {
    /*
     * For 3-wheel omni drive with 120° spacing:
     * Motor 1 at 90° (back, pointing backward in robot frame)
     * Motor 2 at 330° (front-left, 30° from forward)
     * Motor 3 at 210° (front-right, 30° from backward)
     * 
     * Wheel velocity equations:
     * w = (-sin(angle)*vx + cos(angle)*vy + L*omega) / r
     * 
     * where L is the distance from robot center to wheel
     * and r is the wheel radius
     */
    
    const float deg2rad = PI / 180.0;
    
    // Motor 1 at 90° (back)
    float angle1 = 90 * deg2rad;
    w1 = (-sin(angle1) * vx + cos(angle1) * vy + robotRadius * omega) / wheelRadius;
    
    // Motor 2 at 330° (front-left)
    float angle2 = 330 * deg2rad;
    w2 = (-sin(angle2) * vx + cos(angle2) * vy + robotRadius * omega) / wheelRadius;
    
    // Motor 3 at 210° (front-right)
    float angle3 = 210 * deg2rad;
    w3 = (-sin(angle3) * vx + cos(angle3) * vy + robotRadius * omega) / wheelRadius;
}

void OmniWheelDrive::forwardKinematics(float w1, float w2, float w3,
                                      float& vx, float& vy, float& omega) {
    /*
     * Forward kinematics to recover robot velocity from wheel speeds
     * This is the inverse of the matrix used in inverse kinematics
     */
    
    const float deg2rad = PI / 180.0;
    
    // Convert wheel angular velocities to linear velocities
    float v1 = w1 * wheelRadius;
    float v2 = w2 * wheelRadius;
    float v3 = w3 * wheelRadius;
    
    // Motor angles (updated for new configuration)
    float angle1 = 90 * deg2rad;   // back
    float angle2 = 330 * deg2rad;  // front-left
    float angle3 = 210 * deg2rad;  // front-right
    
    // Calculate robot velocities (using pseudo-inverse)
    vx = (-sin(angle1) * v1 - sin(angle2) * v2 - sin(angle3) * v3) * (2.0 / 3.0);
    vy = (cos(angle1) * v1 + cos(angle2) * v2 + cos(angle3) * v3) * (2.0 / 3.0);
    omega = (v1 + v2 + v3) / (3.0 * robotRadius);
}
