#include <Arduino.h>
#include <Servo.h>
#include "MotorController.h"
#include "OmniWheelDrive.h"

// ===== PIN CONFIGURATION =====

// Motor 1 - Front-Left at 300° (5π/3)
#define MOTOR1_PWM D9
#define MOTOR1_DIR1 D8
#define MOTOR1_DIR2 D7
#define MOTOR1_ENC_A D11
#define MOTOR1_ENC_B D12

// Motor 2 - Front-Right at 60° (π/3)
#define MOTOR2_PWM D0
#define MOTOR2_DIR1 A3
#define MOTOR2_DIR2 A4
#define MOTOR2_ENC_A A1
#define MOTOR2_ENC_B A0

// Motor 3 - Back at 180° (π)
#define MOTOR3_PWM D1
#define MOTOR3_DIR1 D3
#define MOTOR3_DIR2 A6
#define MOTOR3_ENC_A D4
#define MOTOR3_ENC_B D6

// ===== PID TUNING PARAMETERS =====
// Motor specs: 530 RPM no-load @ 12V, 8.5 kg⋅cm stall torque
// Adjust these values based on your motors and robot characteristics
float motor1_kp = 1.7, motor1_ki = 0.16, motor1_kd = 0;
float motor2_kp = 1.3, motor2_ki = 0.16, motor2_kd = 0;
float motor3_kp = 1.6, motor3_ki = 0.16, motor3_kd = 0;

// ===== ROBOT DIMENSIONS =====
#define WHEEL_RADIUS 0.035 // Wheel radius in meters (35mm)
#define ROBOT_RADIUS 0.115  // Distance from center to wheel in meters (115mm)

// ===== MOTOR & ENCODER SPECIFICATIONS =====
// Encoder: 64 CPR (counts per revolution)
// Gear Ratio: 18.75:1
// Effective output shaft resolution: 64 × 18.75 = 1200 counts/rev
// These values are defined in MotorController.h

// ===== GLOBAL OBJECTS =====
MotorController motor1(MOTOR1_PWM, MOTOR1_DIR1, MOTOR1_DIR2,
                       MOTOR1_ENC_A, MOTOR1_ENC_B, motor1_kp, motor1_ki, motor1_kd);
MotorController motor2(MOTOR2_PWM, MOTOR2_DIR1, MOTOR2_DIR2,
                       MOTOR2_ENC_A, MOTOR2_ENC_B, motor2_kp, motor2_ki, motor2_kd);
MotorController motor3(MOTOR3_PWM, MOTOR3_DIR1, MOTOR3_DIR2,
                       MOTOR3_ENC_A, MOTOR3_ENC_B, motor3_kp, motor3_ki, motor3_kd);

OmniWheelDrive robot(&motor1, &motor2, &motor3, WHEEL_RADIUS, ROBOT_RADIUS);

// ===== SERIAL COMMAND BUFFER =====
String commandBuffer = "";
bool commandComplete = false;

// ===== SAFETY WATCHDOG =====
#define WATCHDOG_TIMEOUT 1000 // Stop motors if no command received for 1 second
unsigned long lastCommandTime = 0;
bool motorsEnabled = false;

// ===== FUNCTION DECLARATIONS =====
void checkWatchdog();
void emergencyStop();
void processCommand(String cmd);
void handleVelocityCommand(String cmd);
void handlePIDCommand(String cmd);
void runAutoTune(int motorId);
void sendStatus();
void printRobotInfo();

// ===== ENCODER INTERRUPT HANDLERS =====
void motor1EncoderA_ISR()
{
  motor1.handleEncoderA();
}

void motor1EncoderB_ISR()
{
  motor1.handleEncoderB();
}

void motor2EncoderA_ISR()
{
  motor2.handleEncoderA();
}

void motor2EncoderB_ISR()
{
  motor2.handleEncoderB();
}

void motor3EncoderA_ISR()
{
  motor3.handleEncoderA();
}

void motor3EncoderB_ISR()
{
  motor3.handleEncoderB();
}

// ===== SETUP =====
void setup()
{
  // Initialize serial communication for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
    ; // Wait for serial or timeout

  Serial.println("=== 3-Wheel Omni Robot PID Controller ===");
  Serial.println("Initializing...");

  // Initialize the robot
  robot.begin();

  // 
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_A), motor1EncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_B), motor1EncoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_A), motor2EncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_B), motor2EncoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_ENC_A), motor3EncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_ENC_B), motor3EncoderB_ISR, CHANGE);

  Serial.println("Initialization complete!");
  Serial.println("\n=== Serial Protocol ===");
  Serial.println("Format: COMMAND[,param1,param2,...]");
  Serial.println("  Raspberry Pi Commands:");
  Serial.println("  VEL,vx,vy,omega  - Set velocity (m/s, m/s, rad/s)");
  Serial.println("                     Example: VEL,0.3,0,0 (moderate forward)");
  Serial.println("                     Recommended range: 0.1 to 1.5 m/s");
  Serial.println("                     Motor max: 530 RPM, limited to 350 RPM");
  Serial.println("  STOP             - Stop all motors");
  Serial.println("  STATUS           - Get robot status");
  Serial.println("  PID,kp,ki,kd     - Update PID gains");
  Serial.println("\nSAFETY: Motors will stop if no command received for 1 second");
  Serial.println();

  delay(1000);

  // Initialize watchdog
  lastCommandTime = millis();
  motorsEnabled = false; // Start with motors disabled
}

// ===== MAIN LOOP =====
void loop()
{
  // Check watchdog timer for safety
  checkWatchdog();

  // Update all motor controllers (PID computation) only if enabled
  if (motorsEnabled)
  {
    robot.update();
  }

  // Read serial data into buffer
  while (Serial.available() > 0)
  {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r')
    {
      if (commandBuffer.length() > 0)
      {
        commandComplete = true;
      }
    }
    else
    {
      commandBuffer += inChar;
    }
  }

  // Process complete command
  if (commandComplete)
  {
    processCommand(commandBuffer);
    commandBuffer = "";
    commandComplete = false;
  }

  // Small delay to prevent overwhelming the CPU
  delay(10);
}

// ===== COMMAND PROCESSING =====
void processCommand(String cmd)
{
  cmd.trim();
  cmd.toUpperCase();

  // Parse structured commands (for Raspberry Pi)
  if (cmd.startsWith("VEL,"))
  {
    handleVelocityCommand(cmd);
  }
  else if (cmd == "STOP")
  {
    emergencyStop();
    Serial.println("OK:STOPPED");
  }
  else if (cmd == "STATUS")
  {
    sendStatus();
  }
  else if (cmd.startsWith("PID,"))
  {
    handlePIDCommand(cmd);
  }
  else if (cmd.startsWith("TUNE,"))
  {
    int comma = cmd.indexOf(',');
    int motorId = cmd.substring(comma + 1).toInt();
    runAutoTune(motorId);
  }
  else
  {
    Serial.println("ERROR:UNKNOWN_COMMAND");
  }
}

void handleVelocityCommand(String cmd)
{
  // Parse: VEL,vx,vy,omega
  int firstComma = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);
  int thirdComma = cmd.indexOf(',', secondComma + 1);

  if (firstComma == -1 || secondComma == -1 || thirdComma == -1)
  {
    Serial.println("ERROR:INVALID_VEL_FORMAT");
    return;
  }

  float vx = cmd.substring(firstComma + 1, secondComma).toFloat();
  float vy = cmd.substring(secondComma + 1, thirdComma).toFloat();
  float omega = cmd.substring(thirdComma + 1).toFloat();

  // Update watchdog and enable motors
  lastCommandTime = millis();
  motorsEnabled = true;

  robot.setVelocity(vx, vy, omega);

  Serial.print("OK:VEL_SET,");
  Serial.print(vx, 3);
  Serial.print(",");
  Serial.print(vy, 3);
  Serial.print(",");
  Serial.println(omega, 3);
}

void handlePIDCommand(String cmd)
{
  // Parse: PID,kp,ki,kd OR PID,motorId,kp,ki,kd
  int commas = 0;
  for (unsigned int i = 0; i < cmd.length(); i++)
  {
    if (cmd[i] == ',')
      commas++;
  }

  if (commas == 3)
  {
    // Global set: PID,kp,ki,kd
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    int thirdComma = cmd.indexOf(',', secondComma + 1);

    float kp = cmd.substring(firstComma + 1, secondComma).toFloat();
    float ki = cmd.substring(secondComma + 1, thirdComma).toFloat();
    float kd = cmd.substring(thirdComma + 1).toFloat();

    motor1.setPIDGains(kp, ki, kd);
    motor2.setPIDGains(kp, ki, kd);
    motor3.setPIDGains(kp, ki, kd);
    
    // Update global vars
    motor1_kp = motor2_kp = motor3_kp = kp;
    motor1_ki = motor2_ki = motor3_ki = ki;
    motor1_kd = motor2_kd = motor3_kd = kd;

    Serial.print("OK:PID_SET_ALL,");
    Serial.print(kp, 3);
    Serial.print(",");
    Serial.print(ki, 3);
    Serial.print(",");
    Serial.println(kd, 3);
  }
  else if (commas == 4)
  {
    // Specific set: PID,motorId,kp,ki,kd
    int c1 = cmd.indexOf(',');
    int c2 = cmd.indexOf(',', c1 + 1);
    int c3 = cmd.indexOf(',', c2 + 1);
    int c4 = cmd.indexOf(',', c3 + 1);

    int motorId = cmd.substring(c1 + 1, c2).toInt();
    float kp = cmd.substring(c2 + 1, c3).toFloat();
    float ki = cmd.substring(c3 + 1, c4).toFloat();
    float kd = cmd.substring(c4 + 1).toFloat();

    if (motorId == 1) { motor1.setPIDGains(kp, ki, kd); motor1_kp = kp; motor1_ki = ki; motor1_kd = kd; }
    else if (motorId == 2) { motor2.setPIDGains(kp, ki, kd); motor2_kp = kp; motor2_ki = ki; motor2_kd = kd; }
    else if (motorId == 3) { motor3.setPIDGains(kp, ki, kd); motor3_kp = kp; motor3_ki = ki; motor3_kd = kd; }
    else { Serial.println("ERROR:INVALID_MOTOR_ID"); return; }

    Serial.print("OK:PID_SET_MOTOR_");
    Serial.print(motorId);
    Serial.print(",");
    Serial.print(kp, 3);
    Serial.print(",");
    Serial.print(ki, 3);
    Serial.print(",");
    Serial.println(kd, 3);
  }
  else
  {
    Serial.println("ERROR:INVALID_PID_FORMAT");
  }
}

void sendStatus()
{
  // Send status in structured format for Raspberry Pi parsing
  float vx, vy, omega;
  robot.getCurrentVelocity(vx, vy, omega);

  Serial.print("STATUS:");
  Serial.print(vx, 3);
  Serial.print(",");
  Serial.print(vy, 3);
  Serial.print(",");
  Serial.print(omega, 3);
  Serial.print(",");
  Serial.print(motor1.getCurrentSpeed(), 2);
  Serial.print(",");
  Serial.print(motor2.getCurrentSpeed(), 2);
  Serial.print(",");
  Serial.print(motor3.getCurrentSpeed(), 2);
  Serial.print(",");
  Serial.print(motor1.getEncoderCount());
  Serial.print(",");
  Serial.print(motor2.getEncoderCount());
  Serial.print(",");
  Serial.println(motor3.getEncoderCount());
}

// ===== HELPER FUNCTIONS =====
void printRobotInfo()
{
  Serial.println("\n=== Robot Status ===");

  Serial.print("Motor 1 - Target: ");
  Serial.print(motor1.getTargetSpeed());
  Serial.print(" RPM, Current: ");
  Serial.print(motor1.getCurrentSpeed());
  Serial.print(" RPM, Encoder: ");
  Serial.println(motor1.getEncoderCount());

  Serial.print("Motor 2 - Target: ");
  Serial.print(motor2.getTargetSpeed());
  Serial.print(" RPM, Current: ");
  Serial.print(motor2.getCurrentSpeed());
  Serial.print(" RPM, Encoder: ");
  Serial.println(motor2.getEncoderCount());

  Serial.print("Motor 3 - Target: ");
  Serial.print(motor3.getTargetSpeed());
  Serial.print(" RPM, Current: ");
  Serial.print(motor3.getCurrentSpeed());
  Serial.print(" RPM, Encoder: ");
  Serial.println(motor3.getEncoderCount());

  float vx, vy, omega;
  robot.getCurrentVelocity(vx, vy, omega);
  Serial.print("Robot Velocity - Vx: ");
  Serial.print(vx);
  Serial.print(" m/s, Vy: ");
  Serial.print(vy);
  Serial.print(" m/s, Omega: ");
  Serial.print(omega);
  Serial.println(" rad/s");

  Serial.print("Motors Enabled: ");
  Serial.println(motorsEnabled ? "YES" : "NO");

  unsigned long timeSinceCommand = millis() - lastCommandTime;
  Serial.print("Time since last command: ");
  Serial.print(timeSinceCommand);
  Serial.println(" ms");

  Serial.println();
}

// ===== SAFETY FUNCTIONS =====
void checkWatchdog()
{
  if (motorsEnabled && (millis() - lastCommandTime > WATCHDOG_TIMEOUT))
  {
    emergencyStop();
    Serial.println("WARNING:WATCHDOG_TIMEOUT - Motors stopped for safety");
  }
}

void emergencyStop()
{
  motorsEnabled = false;
  robot.stopAll();
  // Reset PID controllers to prevent integral windup
  motor1.resetPID();
  motor2.resetPID();
  motor3.resetPID();
}

void runAutoTune(int motorId)
{
  MotorController* targetMotor = nullptr;
  float* targetKp = nullptr;
  float* targetKi = nullptr;
  float* targetKd = nullptr;

  if (motorId == 1) { targetMotor = &motor1; targetKp = &motor1_kp; targetKi = &motor1_ki; targetKd = &motor1_kd; }
  else if (motorId == 2) { targetMotor = &motor2; targetKp = &motor2_kp; targetKi = &motor2_ki; targetKd = &motor2_kd; }
  else if (motorId == 3) { targetMotor = &motor3; targetKp = &motor3_kp; targetKi = &motor3_ki; targetKd = &motor3_kd; }
  else {
    Serial.println("ERROR:INVALID_MOTOR_ID");
    return;
  }

  Serial.print("INFO:STARTING_ITERATIVE_TUNE_MOTOR_"); Serial.println(motorId);
  
  // Stop all motors and disable main loop
  robot.stopAll();
  motorsEnabled = false; 
  
  // Reset PID
  targetMotor->resetPID();
  
  float tunedKp = 0.0;
  float tunedKi = 0.0;
  float tunedKd = 0.0; // We will leave Kd at 0 for this simple tune
  
  float targetRPM = 150.0; // Target speed for tuning
  
  Serial.println("INFO:PHASE_1_TUNING_KP");
  targetMotor->setTargetSpeed(targetRPM);
  
  // Phase 1: Increase Kp until we reach ~70% of target speed
  // This ensures we have enough P-gain to move, but leaves room for I-gain
  for (float kp = 0.1; kp < 10.0; kp += 0.1) {
    targetMotor->setPIDGains(kp, 0, 0);
    
    // Run for 300ms to let speed stabilize
    unsigned long startWait = millis();
    float avgSpeed = 0;
    int samples = 0;
    
    while (millis() - startWait < 300) {
      targetMotor->update();
      if (millis() - startWait > 100) { // Ignore first 100ms transient
         avgSpeed += abs(targetMotor->getCurrentSpeed());
         samples++;
      }
      delay(5);
    }
    if (samples > 0) avgSpeed /= samples;
    
    Serial.print("INFO:Kp="); Serial.print(kp);
    Serial.print(",Speed="); Serial.println(avgSpeed);
    
    if (avgSpeed >= targetRPM * 0.70) {
      tunedKp = kp;
      break;
    }
  }
  
  if (tunedKp == 0) {
     Serial.println("ERROR:TUNE_FAILED_MOTOR_DID_NOT_REACH_SPEED");
     targetMotor->stop();
     return;
  }
  
  Serial.println("INFO:PHASE_2_TUNING_KI");
  
  // Phase 2: Increase Ki until error is minimized
  // We keep the Kp we found
  for (float ki = 0.01; ki < 5.0; ki += 0.05) {
    targetMotor->setPIDGains(tunedKp, ki, 0);
    
    // Run for 500ms to let integrator work
    unsigned long startWait = millis();
    float avgError = 0;
    int samples = 0;
    
    while (millis() - startWait < 500) {
      targetMotor->update();
      if (millis() - startWait > 200) {
         float error = abs(targetRPM - abs(targetMotor->getCurrentSpeed()));
         avgError += error;
         samples++;
      }
      delay(5);
    }
    if (samples > 0) avgError /= samples;
    
    Serial.print("INFO:Ki="); Serial.print(ki);
    Serial.print(",AvgError="); Serial.println(avgError);
    
    // If error is less than 5% of target, we are good
    if (avgError < targetRPM * 0.05) {
      tunedKi = ki;
      break;
    }
  }
  
  // Stop motor
  targetMotor->stop();
  
  // Update global variables
  *targetKp = tunedKp;
  *targetKi = tunedKi;
  *targetKd = tunedKd;
  
  // Apply final values
  targetMotor->setPIDGains(tunedKp, tunedKi, tunedKd);
  
  Serial.print("OK:TUNED_MOTOR_"); Serial.print(motorId);
  Serial.print(",Kp="); Serial.print(tunedKp);
  Serial.print(",Ki="); Serial.print(tunedKi);
  Serial.println(",Kd=0.00");
}