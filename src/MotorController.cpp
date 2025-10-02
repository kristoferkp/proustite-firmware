#include "MotorController.h"
#include <Arduino.h>

MotorController::MotorController(int pwm, int dir1, int dir2, int encA, int encB,
                                 float Kp, float Ki, float Kd)
    : pwmPin(pwm), dirPin1(dir1), dirPin2(dir2),
      encoderPinA(encA), encoderPinB(encB),
      encoderCount(0), currentSpeed(0), targetSpeed(0),
      pid(Kp, Ki, Kd), lastSpeedCalcTime(0), lastEncoderCount(0)
{
}

void MotorController::begin()
{
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);

    // Set PWM frequency (optional, depends on board)
    analogWrite(pwmPin, 0);

    // Initialize direction
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);

    pid.setOutputLimits(-255, 255);
}

// SAFETY: Atomic read of encoder count to prevent race conditions with ISR
long MotorController::getEncoderCountAtomic() const
{
    noInterrupts();
    long count = encoderCount;
    interrupts();
    return count;
}

void MotorController::setTargetSpeed(float speed)
{
    targetSpeed = speed;
    pid.setSetpoint(speed);
}

void MotorController::update()
{
    // Calculate current speed based on encoder
    unsigned long currentTime = millis();
    float dt = (currentTime - lastSpeedCalcTime) / 1000.0;

    if (dt >= 0.05)
    { // Update every 50ms
        // Use atomic read for encoder count
        long currentEncoderCount = getEncoderCountAtomic();
        long countDiff = currentEncoderCount - lastEncoderCount;

        // Convert to RPM at the output shaft
        // countDiff is the encoder counts in time dt
        // Divide by total counts per output revolution (ENCODER_CPR * GEAR_RATIO)
        // Then convert to RPM: revolutions * (60 / dt)
        currentSpeed = (countDiff / COUNTS_PER_OUTPUT_REV) / dt * 60.0; // RPM at output shaft

        lastEncoderCount = currentEncoderCount;
        lastSpeedCalcTime = currentTime;
    }

    // If target speed is zero, stop immediately without PID
    if (abs(targetSpeed) < 0.01)
    {
        analogWrite(pwmPin, 0);
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, LOW);
        return;
    }

    // Compute PID output
    float pidOutput = pid.compute(currentSpeed);

    // Set motor direction and speed
    if (pidOutput >= 0)
    {
        digitalWrite(dirPin1, HIGH);
        digitalWrite(dirPin2, LOW);
        analogWrite(pwmPin, constrain(abs(pidOutput), 0, 255));
    }
    else
    {
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, HIGH);
        analogWrite(pwmPin, constrain(abs(pidOutput), 0, 255));
    }
}

void MotorController::stop()
{
    targetSpeed = 0;
    pid.setSetpoint(0);
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
}

void MotorController::handleEncoderA()
{
    // Read encoder state
    bool stateA = digitalRead(encoderPinA);
    bool stateB = digitalRead(encoderPinB);

    // Determine direction and update count
    if (stateA == stateB)
    {
        encoderCount++;
    }
    else
    {
        encoderCount--;
    }
}

void MotorController::handleEncoderB()
{
    // Read encoder state
    bool stateA = digitalRead(encoderPinA);
    bool stateB = digitalRead(encoderPinB);

    // Determine direction and update count
    if (stateA != stateB)
    {
        encoderCount++;
    }
    else
    {
        encoderCount--;
    }
}
void MotorController::setPIDGains(float Kp, float Ki, float Kd)
{
    pid.setGains(Kp, Ki, Kd);
}

void MotorController::resetPID()
{
    pid.reset();
}
