#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

// --- MPU6050 Pin definitions ---
#define SDA_PIN 2
#define SCL_PIN 15

// --- ESC Pin definitions & Constants ---
const int ESC_PIN = 13;
const int ESC_MIN_PULSE     = 1000; // Full Reverse
const int ESC_NEUTRAL_PULSE = 1500; // Stop / Neutral
const int ESC_MAX_PULSE     = 2000; // Full Forward

// --- Global Objects ---
Adafruit_MPU6050 mpu;
Servo escServo;

// --- GLOBAL VARIABLES FOR CALIBRATION OFFSETS ---
// Acceleration Biases (m/s²)
float accelBiasX = 0.0;
float accelBiasY = 0.0;
float accelBiasZ = 0.0;

// Gyroscope Biases (rad/s)
float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;
// ------------------------------------------------

/**
 * @brief Performs the Bias/Offset Calibration for both Accelerometer and Gyroscope.
 * The sensor MUST be kept still and flat during this routine.
 */
void calibrateSensor() {
  const int numSamples = 1000; // Number of samples to average
  
  float sumAx = 0.0, sumAy = 0.0, sumAz = 0.0;
  float sumGx = 0.0, sumGy = 0.0, sumGz = 0.0;
  
  Serial.print("Starting calibration: Collecting ");
  Serial.print(numSamples);
  Serial.println(" samples. KEEP SENSOR PERFECTLY STILL!");
  
  sensors_event_t a, g, temp;

  // 1. Collect Samples
  for (int i = 0; i < numSamples; i++) {
    mpu.getEvent(&a, &g, &temp);
    
    sumAx += a.acceleration.x;
    sumAy += a.acceleration.y;
    sumAz += a.acceleration.z;
    
    sumGx += g.gyro.x;
    sumGy += g.gyro.y;
    sumGz += g.gyro.z;
    
    // Print progress dot every 100 samples
    if (i % 100 == 0) Serial.print("."); 

    delay(2); // Small delay
  }
  Serial.println("\nSampling complete.");
  
  // 2. Calculate Averages
  float avgAx = sumAx / numSamples;
  float avgAy = sumAy / numSamples;
  float avgAz = sumAz / numSamples;
  
  float avgGx = sumGx / numSamples;
  float avgGy = sumGy / numSamples;
  float avgGz = sumGz / numSamples;
  
  // 3. Store Accelerometer Biases
  // X and Y biases are the average reading when still.
  accelBiasX = avgAx;
  accelBiasY = avgAy;
  
  // Z bias is the average reading MINUS the expected gravity (9.81 m/s²).
  // This makes the calibrated Z read ~9.81 m/s² when still.
  const float EARTH_GRAVITY = 9.80665; // Standard gravity in m/s²
  accelBiasZ = avgAz - EARTH_GRAVITY;
  
  // 4. Store Gyroscope Biases
  // Gyro biases are simply the average readings when still (should be 0 rad/s).
  gyroBiasX = avgGx;
  gyroBiasY = avgGy;
  gyroBiasZ = avgGz;
  
  Serial.println("Calibration finished. Biases saved:");
  Serial.print("Accel Biases (m/s²): X="); Serial.print(accelBiasX);
  Serial.print(" Y="); Serial.print(accelBiasY);
  Serial.print(" Z="); Serial.println(accelBiasZ);
  Serial.print("Gyro Biases (rad/s): X="); Serial.print(gyroBiasX);
  Serial.print(" Y="); Serial.print(gyroBiasY);
  Serial.print(" Z="); Serial.println(gyroBiasZ);
}


void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  delay(1000);
  
  // --- MPU6050 SETUP ---
  Serial.println("\n\nMPU6050 Initialization Started");
  
  // Initialize I2C with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);
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
  
  // --- RUN CALIBRATION ---
  calibrateSensor();
  // -----------------------
  
  // --- ESC SETUP ---
  Serial.println("\nInitializing BLHeli_S 3D ESC...");
  Serial.println("Commands:");
  Serial.println("  'f' or 'F': Forward (Slow)");
  Serial.println("  'r' or 'R': Reverse (Slow)");
  Serial.println("  's' or 'S': Stop");

  // ESP32Servo setup
  escServo.setPeriodHertz(50); 
  // Attach with the specific range your ESC expects
  escServo.attach(ESC_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);

  // --- ARMING SEQUENCE ---
  // 3D ESCs usually arm when they receive the NEUTRAL signal (1500us)
  Serial.println("Arming: Sending Neutral (1500us)...");
  escServo.writeMicroseconds(ESC_NEUTRAL_PULSE);
  
  delay(3000); // Wait for ESC initialization beeps
  Serial.println("ESC Arming complete. Ready for Serial commands.");

  delay(100);
  Serial.println("\nStarting calibrated sensor readings and ESC control loop...\n");
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
            currentPulse = 1545; // Slow forward
            break;

        case 'r':
        case 'R':
            Serial.println("Command: Reverse (Slow)");
            currentPulse = 1420; // Slow reverse
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
  // Get sensor event (RAW readings)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // --- APPLY CALIBRATION CORRECTION ---
  float cal_accel_x = a.acceleration.x - accelBiasX;
  float cal_accel_y = a.acceleration.y - accelBiasY;
  float cal_accel_z = a.acceleration.z - accelBiasZ; // Should be ~9.81 m/s²
  
  float cal_gyro_x = g.gyro.x - gyroBiasX;
  float cal_gyro_y = g.gyro.y - gyroBiasY;
  float cal_gyro_z = g.gyro.z - gyroBiasZ;
  // ------------------------------------

  // Print timestamp
  Serial.print("time: " + String(millis()) + " ms\n");

  // Print CALIBRATED accelerometer readings (m/s²)
  Serial.print("accel_x: " + String(cal_accel_x, 3) + ", ");
  Serial.print("accel_y: " + String(cal_accel_y, 3) + ", ");
  Serial.print("accel_z: " + String(cal_accel_z, 3) + "\n");
  
  // Print CALIBRATED gyroscope readings (rad/s)
  Serial.print("gyro_x: " + String(cal_gyro_x, 3) + ", ");
  Serial.print("gyro_y: " + String(cal_gyro_y, 3) + ", ");
  Serial.print("gyro_z: " + String(cal_gyro_z, 3) + "\n");
  
  // Print temperature (°C)
  Serial.print("temp_c: " + String(temp.temperature, 2) + "\n");
  
  Serial.println("---");
  
  // Delay before next reading (100ms = 10 Hz)
  delay(100);
}
