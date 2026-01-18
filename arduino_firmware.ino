#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// --- ESC Pin definitions & Constants ---
const int ESC_PIN = 5;
const int ESC_MIN_PULSE     = 1000; // Full Reverse
const int ESC_NEUTRAL_PULSE = 1500; // Stop / Neutral
const int ESC_MAX_PULSE     = 2000; // Full Forward

// --- Global Objects ---
Adafruit_MPU6050 mpu;
Servo escServo;


void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  delay(1000);
  
  // --- MPU6050 SETUP ---
  Serial.println("\n\nMPU6050 Initialization Started");
  
  // Initialize I2C
  Wire.begin();
  delay(100);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1) {
      delay(100);
    }
  }
  
  Serial.println("MPU6050 initialized successfully!");
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // --- ESC SETUP ---
  Serial.println("\nInitializing BLHeli_S 3D ESC...");
  Serial.println("Commands:");
  Serial.println("  'f' or 'F': Forward (Slow)");
  Serial.println("  'r' or 'R': Reverse (Slow)");
  Serial.println("  's' or 'S': Stop");

  // Arduino Servo setup
  // Attach with the specific range your ESC expects
  escServo.attach(ESC_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);

  // --- ARMING SEQUENCE ---
  // 3D ESCs usually arm when they receive the NEUTRAL signal (1500us)
  Serial.println("Arming: Sending Neutral (1500us)...");
  escServo.writeMicroseconds(ESC_NEUTRAL_PULSE);
  
  delay(3000); // Wait for ESC initialization beeps
  Serial.println("ESC Arming complete. Ready for Serial commands.");

  delay(100);
  Serial.println("\nStarting sensor readings and ESC control loop...\n");
}

void loop() {
  // --- ESC CONTROL LOGIC ---
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // Ignore newline or carriage return characters
    if (command != '\n' && command != '\r') {
        int currentPulse = ESC_NEUTRAL_PULSE;
        bool validCommand = true;

        switch (command) {
        case 'f':
        case 'F':
            Serial.println("Command: Forward (Slow)");
            currentPulse = 1550; // Slow forward
            break;

        case 'r':
        case 'R':
            Serial.println("Command: Reverse (Slow)");
            currentPulse = 1410; // Slow reverse
            break;

        case 's':
        case 'S':
            Serial.println("Command: Stop");
            currentPulse = ESC_NEUTRAL_PULSE;
            break;

        default:
            Serial.print("Unknown command: ");
            Serial.println(command);
            validCommand = false;
            break;
        }

        if (validCommand) {
            escServo.writeMicroseconds(currentPulse);
        }
    }
  }

  // --- MPU6050 READING LOGIC ---
  // Get sensor event
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print timestamp
  Serial.print("time: " + String(millis()) + " ms\n");

  // Print accelerometer readings (m/s²)
  Serial.print("accel_x: " + String(a.acceleration.x, 3) + ", ");
  Serial.print("accel_y: " + String(a.acceleration.y, 3) + ", ");
  Serial.print("accel_z: " + String(a.acceleration.z, 3) + "\n");
  
  // Print gyroscope readings (rad/s)
  Serial.print("gyro_x: " + String(g.gyro.x, 3) + ", ");
  Serial.print("gyro_y: " + String(g.gyro.y, 3) + ", ");
  Serial.print("gyro_z: " + String(g.gyro.z, 3) + "\n");
  
  // Print temperature (°C)
  Serial.print("temp_c: " + String(temp.temperature, 2) + "\n");
  
  Serial.println("---");
  
  // Delay before next reading (100ms = 10 Hz)
  delay(100);
}
