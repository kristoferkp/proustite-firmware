#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "PIDController.h"

class MotorController {
private:
    int pwmPin;           // PWM pin for speed control
    int dirPin1;          // Direction pin 1
    int dirPin2;          // Direction pin 2
    int encoderPinA;      // Encoder channel A
    int encoderPinB;      // Encoder channel B
    
    volatile long encoderCount;    // Current encoder count
    float currentSpeed;            // Current speed in RPM or units/sec
    float targetSpeed;             // Target speed
    
    PIDController pid;             // PID controller for this motor
    
    unsigned long lastSpeedCalcTime;
    long lastEncoderCount;
    
    // Hardware specifications
    static constexpr int ENCODER_CPR = 64;      // Encoder counts per revolution (base encoder)
    static constexpr float GEAR_RATIO = 18.75;  // Motor gear ratio
    static constexpr float COUNTS_PER_OUTPUT_REV = ENCODER_CPR * GEAR_RATIO;  // 64 * 18.75 = 1200
    
public:
    MotorController(int pwm, int dir1, int dir2, int encA, int encB, 
                   float Kp, float Ki, float Kd);
    
    void begin();
    void setTargetSpeed(float speed);
    void update();
    void stop();
    
    void handleEncoderA();  // Interrupt handler for encoder A
    void handleEncoderB();  // Interrupt handler for encoder B
    
    float getCurrentSpeed() const { return currentSpeed; }
    float getTargetSpeed() const { return targetSpeed; }
    long getEncoderCount() const { return encoderCount; }
    
    void setPIDGains(float Kp, float Ki, float Kd);
    void resetPID();
};

#endif
