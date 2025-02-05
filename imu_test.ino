#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Create an MPU6050 object
Adafruit_MPU6050 mpu;

void setup() {
  // Initialize Serial communication for output
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for Serial monitor to open
  }

  // Initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Check wiring");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 initialized successfully");

  // Configure the sensor
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  // Get new sensor events with the Adafruit library
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Print Accelerometer values (in m/s^2)
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s^2, ");
  Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" m/s^2, ");
  Serial.print("Z: "); Serial.print(accel.acceleration.z); Serial.println(" m/s^2");

  // Print Gyroscope values (in degrees per second)
  Serial.print("Gyro X: "); Serial.print(gyro.gyro.x); Serial.print(" °/s, ");
  Serial.print("Y: "); Serial.print(gyro.gyro.y); Serial.print(" °/s, ");
  Serial.print("Z: "); Serial.print(gyro.gyro.z); Serial.println(" °/s");

  // Print Temperature (in °C)
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" °C");

  // Wait for 1 second before the next reading
  delay(1000);
}
