#include <Wire.h>

// The universal hardware address for the MPU6050 sensor
const int MPU_ADDR = 0x68; 

void setup() {
  Serial.begin(115200);
  delay(1000); 

  Serial.println("--- Teensy 4.1 Native Wire I2C MPU6050 Test ---");

  // Initialize the native Teensy I2C bus
  Wire.begin();
  Wire.setClock(100000); // 100kHz standard speed

  // Wake up the MPU6050 by writing to its Power Management Register (0x6B)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Access the PWR_MGMT_1 register
  Wire.write(0);    // Set to 0 to wake up the sensor from sleep mode
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("MPU6050 successfully awoken via direct I2C registers!");
  } else {
    Serial.print("Error: Direct communication failed with code: ");
    Serial.println(error);
    while (1) { delay(10); } // Halt program
  }
}

void loop() {
  // Request 6 bytes of data starting from the raw Gyroscope Register (0x43)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  // Read raw 16-bit high and low bytes for X, Y, and Z axes
  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  // Convert raw sensor integers into readable Degrees Per Second (DPS)
  // At the default sensitivity setting, dividing by 131 gives standard degrees/sec
  float gyroX = rawX / 131.0;
  float gyroY = rawY / 131.0;
  float gyroZ = rawZ / 131.0;

  /* Print out the real-time rotation values */
  Serial.print("Rotation X: "); Serial.print(gyroX);
  Serial.print("  |  Y: ");     Serial.print(gyroY);
  Serial.print("  |  Z: ");     Serial.println(gyroZ);

  delay(50); // Loop update window
}
