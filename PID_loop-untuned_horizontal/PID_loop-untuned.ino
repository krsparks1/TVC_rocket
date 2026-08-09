#include <Servo.h>
#include <Wire.h>

Servo servoX;
Servo servoY;

const int MPU_ADDR = 0x68; 

// =========================================================================
//  1. YOUR VERIFIED HARDWARE CALIBRATION TUNING VARIABLES
// =========================================================================
const int CALIBRATED_CENTER_X = 98; 
const int CALIBRATED_CENTER_Y = 86; 
const float AXIS_Y_MULTIPLIER = 1.5; 
const int STRICT_TILT_LIMIT = 10; 

// =========================================================================
//  2. DRIFT-PROOF COMPLEMENTARY FILTER ALPHA VALUE
// =========================================================================
// 0.98 means 98% Gyro (for speed) and 2% Accelerometer (to kill drift)
const float ALPHA = 0.98; 

// =========================================================================
//  3. THE COMPLETE PID TUNING CONSTANTS
// =========================================================================
const float Kp = 0.55;  
const float Ki = 0.05;  
const float Kd = 0.12;  

// =========================================================================
//  4. INTERNAL PID MATH TRACKING REGISTERS
// =========================================================================
unsigned long lastTime = 0;
float angleX = 0.0, angleY = 0.0;
float errorSumX = 0.0, errorSumY = 0.0;
float lastErrorX = 0.0, lastErrorY = 0.0;

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("--- Teensy 4.1 Drift-Proof TVC Loop ---");

  servoX.attach(1); 
  servoY.attach(2); 

  Wire.begin();
  Wire.setClock(100000); 

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: Sensor lost!");
    while (1) { delay(10); } 
  }

  servoX.write(CALIBRATED_CENTER_X);
  servoY.write(CALIBRATED_CENTER_Y);
  
  delay(1000);
  lastTime = micros(); 
  Serial.println("--> DRIFT-PROOF MODE ACTIVE: Test center snap now.");
}

void loop() {
  // --- A. CALCULATE TIME DELTA ---
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; 
  lastTime = currentTime;
  if (dt <= 0.0) dt = 0.01; 

  // --- B. READ ALL 14 BYTES OF RAW MPU DATA (ACCEL + GYRO) ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Start reading from Accelerometer X Register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Pull 14 bytes sequentially

  // Read Raw Accelerometer values
  int16_t rawAccX = (Wire.read() << 8) | Wire.read();
  int16_t rawAccY = (Wire.read() << 8) | Wire.read();
  int16_t rawAccZ = (Wire.read() << 8) | Wire.read();
  // Skip Temperature bytes
  Wire.read(); Wire.read();
  // Read Raw Gyroscope values
  int16_t rawGyroX = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroY = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroZ = (Wire.read() << 8) | Wire.read();

  // Convert raw values to standard floats (DPS for gyro, Gs for accel)
  float gyroX = rawGyroX / 131.0;
  float gyroY = rawGyroY / 131.0;
  
  // Calculate absolute angle references from Gravity vector via trigonometry
  // atan2 calculates the pitch/yaw lean based purely on acceleration forces
  float accelAngleX = atan2((float)rawAccY, (float)rawAccZ) * 180.0 / PI;
  float accelAngleY = atan2((float)-rawAccX, (float)rawAccZ) * 180.0 / PI;

  // --- C. COMPLEMENTARY FILTER SENSOR FUSION ---
  // Gyro tracks fast changes, while Accel acts as a heavy anchor back to absolute zero
  angleX = ALPHA * (angleX + gyroX * dt) + (1.0 - ALPHA) * accelAngleX;
  angleY = ALPHA * (angleY + gyroY * dt) + (1.0 - ALPHA) * accelAngleY;

  // --- D. EXECUTE COMPLETE PID ALGORITHM ---
  float errorX = 0.0 - angleX;
  float errorY = 0.0 - angleY;

  float pTermX = Kp * errorX;
  float pTermY = Kp * errorY;

  errorSumX += errorX * dt;
  errorSumY += errorY * dt;
  errorSumX = constrain(errorSumX, -10.0, 10.0); 
  errorSumY = constrain(errorSumY, -10.0, 10.0);
  float iTermX = Ki * errorSumX;
  float iTermY = Ki * errorSumY;

  float dTermX = Kd * ((errorX - lastErrorX) / dt);
  float dTermY = Kd * ((errorY - lastErrorY) / dt);
  lastErrorX = errorX;
  lastErrorY = errorY;

  float pidOutputX = pTermX + iTermX + dTermX;
  float pidOutputY = pTermY + iTermY + dTermY;

  float scaledOutputY = pidOutputY * AXIS_Y_MULTIPLIER;

  int finalOffsetX = constrain((int)pidOutputX, -STRICT_TILT_LIMIT, STRICT_TILT_LIMIT);
  int finalOffsetY = constrain((int)scaledOutputY, -STRICT_TILT_LIMIT, STRICT_TILT_LIMIT);

  // --- E. GENERATE HARDWARE COMMAND SIGNALS ---
  int targetServoX = CALIBRATED_CENTER_X + finalOffsetX;
  int targetServoY = CALIBRATED_CENTER_Y + finalOffsetY;

  servoX.write(targetServoX);
  servoY.write(targetServoY);

  // Debug Print: Check your stabilization tracking values
  Serial.print("FilteredTiltX: "); Serial.print(angleX);
  Serial.print(" | ServoX: "); Serial.println(targetServoX);

  delay(10); 
}
