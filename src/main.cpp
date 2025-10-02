#include <Arduino.h>
#include "MotorController.h"
#include "OmniWheelDrive.h"

// ===== PIN CONFIGURATION =====
// Motor 1 - Front-Left at 300° (5π/3)
#define MOTOR1_PWM D9
#define MOTOR1_DIR1 D8
#define MOTOR1_DIR2 D7
#define MOTOR1_ENC_A D12
#define MOTOR1_ENC_B D11

// Motor 2 - Front-Right at 60° (π/3)
#define MOTOR2_PWM D0
#define MOTOR2_DIR1 A3
#define MOTOR2_DIR2 A4
#define MOTOR2_ENC_A A0
#define MOTOR2_ENC_B A1

// Motor 3 - Back at 180° (π)
#define MOTOR3_PWM D1
#define MOTOR3_DIR1 A6
#define MOTOR3_DIR2 D3
#define MOTOR3_ENC_A D6
#define MOTOR3_ENC_B D4

// ===== PID TUNING PARAMETERS =====
// Motor specs: 530 RPM no-load @ 12V, 8.5 kg⋅cm stall torque
// Adjust these values based on your motors and robot characteristics
#define KP 1.5  // Proportional gain - moderate for 530 RPM motors
#define KI 0.3  // Integral gain - helps overcome friction
#define KD 0.02 // Derivative gain - damping for high-speed motors

// ===== ROBOT DIMENSIONS =====
#define WHEEL_RADIUS 0.035 // Wheel radius in meters (35mm)
#define ROBOT_RADIUS 0.14  // Distance from center to wheel in meters (140mm)

// ===== MOTOR & ENCODER SPECIFICATIONS =====
// Encoder: 64 CPR (counts per revolution)
// Gear Ratio: 18.75:1
// Effective output shaft resolution: 64 × 18.75 = 1200 counts/rev
// These values are defined in MotorController.h

// ===== GLOBAL OBJECTS =====
MotorController motor1(MOTOR1_PWM, MOTOR1_DIR1, MOTOR1_DIR2,
                       MOTOR1_ENC_A, MOTOR1_ENC_B, KP, KI, KD);
MotorController motor2(MOTOR2_PWM, MOTOR2_DIR1, MOTOR2_DIR2,
                       MOTOR2_ENC_A, MOTOR2_ENC_B, KP, KI, KD);
MotorController motor3(MOTOR3_PWM, MOTOR3_DIR1, MOTOR3_DIR2,
                       MOTOR3_ENC_A, MOTOR3_ENC_B, KP, KI, KD);

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
void sendStatus();
void handleLegacyCommand(char cmd);
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
  Serial.println("\nRaspberry Pi Commands:");
  Serial.println("  VEL,vx,vy,omega  - Set velocity (m/s, m/s, rad/s)");
  Serial.println("                     Example: VEL,0.3,0,0 (moderate forward)");
  Serial.println("                     Recommended range: 0.1 to 1.5 m/s");
  Serial.println("                     Motor max: 530 RPM, limited to 350 RPM");
  Serial.println("  STOP             - Stop all motors");
  Serial.println("  STATUS           - Get robot status");
  Serial.println("  PID,kp,ki,kd     - Update PID gains");
  Serial.println("\nManual Test Commands (legacy):");
  Serial.println("  w/s - Forward/Backward  |  a/d - Left/Right");
  Serial.println("  q/e - Rotate CCW/CW     |  x - Stop  |  i - Info");
  Serial.println("\nSAFETY: Motors will stop if no command received for 1 second");
  Serial.println("SAFETY: Maximum motor speed limited to 350 RPM (66% of 530 RPM max)");
  Serial.println("SAFETY: Acceleration limiting enabled (2.0 m/s², 3.0 rad/s²)");
  Serial.println("SAFETY: Input velocity validation and clamping enabled");
  Serial.println("SAFETY: Encoder health monitoring active (500ms timeout)");
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

  // SAFETY: Check encoder health and stop if encoders fail
  if (!motor1.isEncoderHealthy() || !motor2.isEncoderHealthy() || !motor3.isEncoderHealthy())
  {
    if (motorsEnabled)
    {
      emergencyStop();
      Serial.println("ERROR:ENCODER_FAILURE - Motors stopped");

      // Report which encoder(s) failed
      if (!motor1.isEncoderHealthy())
        Serial.println("ERROR:Motor 1 encoder unhealthy");
      if (!motor2.isEncoderHealthy())
        Serial.println("ERROR:Motor 2 encoder unhealthy");
      if (!motor3.isEncoderHealthy())
        Serial.println("ERROR:Motor 3 encoder unhealthy");
    }
  }

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
  // Legacy single-character commands (for manual testing)
  else if (cmd.length() == 1)
  {
    handleLegacyCommand(cmd.charAt(0));
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
  // Parse: PID,kp,ki,kd
  int firstComma = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);
  int thirdComma = cmd.indexOf(',', secondComma + 1);

  if (firstComma == -1 || secondComma == -1 || thirdComma == -1)
  {
    Serial.println("ERROR:INVALID_PID_FORMAT");
    return;
  }

  float kp = cmd.substring(firstComma + 1, secondComma).toFloat();
  float ki = cmd.substring(secondComma + 1, thirdComma).toFloat();
  float kd = cmd.substring(thirdComma + 1).toFloat();

  motor1.setPIDGains(kp, ki, kd);
  motor2.setPIDGains(kp, ki, kd);
  motor3.setPIDGains(kp, ki, kd);

  Serial.print("OK:PID_SET,");
  Serial.print(kp, 3);
  Serial.print(",");
  Serial.print(ki, 3);
  Serial.print(",");
  Serial.println(kd, 3);
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
  Serial.print(motor3.getEncoderCount());
  Serial.print(",");
  // SAFETY: Add encoder health status
  Serial.print(motor1.isEncoderHealthy() ? "1" : "0");
  Serial.print(",");
  Serial.print(motor2.isEncoderHealthy() ? "1" : "0");
  Serial.print(",");
  Serial.println(motor3.isEncoderHealthy() ? "1" : "0");
}

void handleLegacyCommand(char cmd)
{
  float vx = 0, vy = 0, omega = 0;

  switch (cmd)
  {
  case 'W':   // Forward
    vx = 0.3; // Increased for better visibility
    Serial.println("OK:FORWARD");
    break;
  case 'S':    // Backward
    vx = -0.3; // Increased for better visibility
    Serial.println("OK:BACKWARD");
    break;
  case 'A':   // Left
    vy = 0.3; // Increased for better visibility
    Serial.println("OK:LEFT");
    break;
  case 'D':    // Right
    vy = -0.3; // Increased for better visibility
    Serial.println("OK:RIGHT");
    break;
  case 'Q':      // Rotate CCW
    omega = 1.0; // Increased for better visibility
    Serial.println("OK:ROTATE_CCW");
    break;
  case 'E':       // Rotate CW
    omega = -1.0; // Increased for better visibility
    Serial.println("OK:ROTATE_CW");
    break;
  case 'X': // Stop
    emergencyStop();
    Serial.println("OK:STOPPED");
    return;
  case 'I': // Info
    printRobotInfo();
    return;
  default:
    Serial.println("ERROR:UNKNOWN_LEGACY_COMMAND");
    return;
  }

  // Update watchdog and enable motors
  lastCommandTime = millis();
  motorsEnabled = true;

  robot.setVelocity(vx, vy, omega);
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

  // SAFETY: Report encoder health
  Serial.print("Motor 1 Encoder: ");
  Serial.println(motor1.isEncoderHealthy() ? "HEALTHY" : "UNHEALTHY");
  Serial.print("Motor 2 Encoder: ");
  Serial.println(motor2.isEncoderHealthy() ? "HEALTHY" : "UNHEALTHY");
  Serial.print("Motor 3 Encoder: ");
  Serial.println(motor3.isEncoderHealthy() ? "HEALTHY" : "UNHEALTHY");

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