#include "OmniWheelDrive.h"
#include <Arduino.h>
#include <math.h>

OmniWheelDrive::OmniWheelDrive(MotorController *m1, MotorController *m2, MotorController *m3,
                               float wheelR, float robotR)
    : motor1(m1), motor2(m2), motor3(m3),
      wheelRadius(wheelR), robotRadius(robotR),
      vx(0), vy(0), w(0), targetVx(0), targetVy(0), targetW(0),
      lastUpdateTime(0)
{
}

void OmniWheelDrive::begin()
{
    motor1->begin();
    motor2->begin();
    motor3->begin();
    lastUpdateTime = millis();
}

void OmniWheelDrive::setVelocity(float vx_in, float vy_in, float omega)
{
    // SAFETY: Input validation - limit maximum velocity commands
    const float MAX_LINEAR_VEL = 2.0;   // m/s - reasonable max for indoor robot
    const float MAX_ANGULAR_VEL = 6.28; // rad/s - one rotation per second

    vx_in = constrain(vx_in, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    vy_in = constrain(vy_in, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    omega = constrain(omega, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

    // Store target velocities (will be ramped in update())
    targetVx = vx_in;
    targetVy = vy_in;
    targetW = omega;
}

void OmniWheelDrive::setWheelSpeeds(float w1, float w2, float w3)
{
    motor1->setTargetSpeed(w1);
    motor2->setTargetSpeed(w2);
    motor3->setTargetSpeed(w3);
}

void OmniWheelDrive::update()
{
    // SAFETY: Apply acceleration limiting to smooth velocity changes
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds

    if (dt > 0 && dt < 1.0) // Sanity check on dt (between 0 and 1 second)
    {
        applyAccelerationLimits(vx, targetVx, MAX_LINEAR_ACCEL, dt);
        applyAccelerationLimits(vy, targetVy, MAX_LINEAR_ACCEL, dt);
        applyAccelerationLimits(w, targetW, MAX_ANGULAR_ACCEL, dt);
    }

    lastUpdateTime = currentTime;

    // Calculate wheel speeds using inverse kinematics (returns rad/s)
    float w1_rad_s, w2_rad_s, w3_rad_s;
    inverseKinematics(vx, vy, w, w1_rad_s, w2_rad_s, w3_rad_s);

    // Convert from rad/s to RPM: RPM = (rad/s) * (60 / 2π)
    const float rad_s_to_rpm = 60.0 / (2.0 * PI);
    float w1_rpm = w1_rad_s * rad_s_to_rpm;
    float w2_rpm = w2_rad_s * rad_s_to_rpm;
    float w3_rpm = w3_rad_s * rad_s_to_rpm;

    // SAFETY: Limit maximum RPM to prevent dangerous speeds
    // Motor spec: 530 RPM no-load @ 12V
    // Setting limit to 70% of max for safety margin and under load
    const float MAX_RPM = 350.0; // ~66% of 530 RPM max
    w1_rpm = constrain(w1_rpm, -MAX_RPM, MAX_RPM);
    w2_rpm = constrain(w2_rpm, -MAX_RPM, MAX_RPM);
    w3_rpm = constrain(w3_rpm, -MAX_RPM, MAX_RPM);

    // Set target speeds for each motor
    motor1->setTargetSpeed(w1_rpm);
    motor2->setTargetSpeed(w2_rpm);
    motor3->setTargetSpeed(w3_rpm);

    // Update individual motor controllers
    motor1->update();
    motor2->update();
    motor3->update();
}

void OmniWheelDrive::stopAll()
{
    motor1->stop();
    motor2->stop();
    motor3->stop();
    vx = 0;
    vy = 0;
    w = 0;
    targetVx = 0;
    targetVy = 0;
    targetW = 0;
}

void OmniWheelDrive::getCurrentVelocity(float &vx_out, float &vy_out, float &omega_out)
{
    // Get current wheel speeds in RPM
    float w1_rpm = motor1->getCurrentSpeed();
    float w2_rpm = motor2->getCurrentSpeed();
    float w3_rpm = motor3->getCurrentSpeed();

    // Convert from RPM to rad/s: rad/s = RPM * (2π / 60)
    const float rpm_to_rad_s = (2.0 * PI) / 60.0;
    float w1_rad_s = w1_rpm * rpm_to_rad_s;
    float w2_rad_s = w2_rpm * rpm_to_rad_s;
    float w3_rad_s = w3_rpm * rpm_to_rad_s;

    // Calculate robot velocity using forward kinematics
    forwardKinematics(w1_rad_s, w2_rad_s, w3_rad_s, vx_out, vy_out, omega_out);
}

void OmniWheelDrive::inverseKinematics(float vx, float vy, float omega,
                                       float &w1, float &w2, float &w3)
{
    /*
     * For 3-wheel omni drive with 120° spacing:
     * Motor 1 at 300° (5π/3) - Front-Left
     * Motor 2 at 60° (π/3) - Front-Right
     * Motor 3 at 180° (π) - Back
     *
     * Wheel velocity equations:
     * w = (-sin(angle)*vx + cos(angle)*vy + L*omega) / r
     */

    const float deg2rad = PI / 180.0;

    // Motor 1 at 300° (Front-Left)
    float angle1 = 300 * deg2rad;
    w1 = (-sin(angle1) * vx + cos(angle1) * vy + robotRadius * omega) / wheelRadius;

    // Motor 2 at 60° (Front-Right)
    float angle2 = 60 * deg2rad;
    w2 = (-sin(angle2) * vx + cos(angle2) * vy + robotRadius * omega) / wheelRadius;

    // Motor 3 at 180° (Back)
    float angle3 = 180 * deg2rad;
    w3 = (-sin(angle3) * vx + cos(angle3) * vy + robotRadius * omega) / wheelRadius;
}

void OmniWheelDrive::forwardKinematics(float w1, float w2, float w3,
                                       float &vx, float &vy, float &omega)
{
    /*
     * Forward kinematics to recover robot velocity from wheel speeds
     * This is the inverse of the matrix used in inverse kinematics
     */

    const float deg2rad = PI / 180.0;

    // Convert wheel angular velocities to linear velocities
    float v1 = w1 * wheelRadius;
    float v2 = w2 * wheelRadius;
    float v3 = w3 * wheelRadius;

    // Motor angles (matching actual hardware configuration)
    float angle1 = 300 * deg2rad; // Front-Left
    float angle2 = 60 * deg2rad;  // Front-Right
    float angle3 = 180 * deg2rad; // Back

    // Calculate robot velocities (using pseudo-inverse)
    vx = (-sin(angle1) * v1 - sin(angle2) * v2 - sin(angle3) * v3) * (2.0 / 3.0);
    vy = (cos(angle1) * v1 + cos(angle2) * v2 + cos(angle3) * v3) * (2.0 / 3.0);
    omega = (v1 + v2 + v3) / (3.0 * robotRadius);
}

// SAFETY: Apply acceleration limits to prevent sudden velocity changes
void OmniWheelDrive::applyAccelerationLimits(float &current, float target, float maxAccel, float dt)
{
    float error = target - current;
    float maxChange = maxAccel * dt;

    if (error > maxChange)
    {
        current += maxChange;
    }
    else if (error < -maxChange)
    {
        current -= maxChange;
    }
    else
    {
        current = target;
    }
}
