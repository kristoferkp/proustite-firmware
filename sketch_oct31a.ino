#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Pin definitions
#define SDA_PIN 2
#define SCL_PIN 15

// MPU6050 instance
Adafruit_MPU6050 mpu;

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
  
  delay(100);
  Serial.println("\nStarting calibrated sensor readings...\n");
}

void loop() {
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
  Serial.print("Time: ");
  Serial.print(millis());
  Serial.println(" ms");
  
  // Print CALIBRATED accelerometer readings (m/s²)
  Serial.print("Accel Calibrated (m/s²): ");
  Serial.print("X=");
  Serial.print(cal_accel_x, 3); // Print with 3 decimal places
  Serial.print(" Y=");
  Serial.print(cal_accel_y, 3);
  Serial.print(" Z=");
  Serial.println(cal_accel_z, 3);
  
  // Print CALIBRATED gyroscope readings (rad/s)
  Serial.print("Gyro Calibrated (rad/s): ");
  Serial.print("X=");
  Serial.print(cal_gyro_x, 3);
  Serial.print(" Y=");
  Serial.print(cal_gyro_y, 3);
  Serial.print(" Z=");
  Serial.println(cal_gyro_z, 3);
  
  // Print temperature (°C)
  Serial.print("Temperature (°C): ");
  Serial.println(temp.temperature, 2);
  
  Serial.println("---");
  
  // Delay before next reading (100ms = 10 Hz)
  delay(100);
}