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
const float AXIS_Y_MULTIPLIER = 1.50; 
const int STRICT_TILT_LIMIT = 10; 

const float ALPHA = 0.98; 

// =========================================================================
//  2. DIRECTION INVERSION TOGGLES (CRITICAL FOR FEEDBACK LOOPS)
//  If an axis twitches wildly or moves the WRONG way, change true to false!
// =========================================================================
const bool INVERT_PITCH = true; 
const bool INVERT_YAW   = false; 

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
  Serial.println("--- Custom Vertical Re-Mapped Flight Controller ---");

  servoX.attach(1); 
  servoY.attach(2); 

  Wire.begin();
  Wire.setClock(100000); 

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: MPU6050 link dropped!");
    while (1) { delay(10); } 
  }

  servoX.write(CALIBRATED_CENTER_X);
  servoY.write(CALIBRATED_CENTER_Y);
  
  delay(1000);
  lastTime = micros(); 
  Serial.println("--> SYSTEM LOCKED: Stand rocket vertically upright now.");
}

void loop() {
  // --- A. CALCULATE TIME DELTA ---
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; 
  lastTime = currentTime;
  if (dt <= 0.0) dt = 0.01; 

  // --- B. READ ALL 14 BYTES OF RAW MPU DATA ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); 

  int16_t rawAccX = (Wire.read() << 8) | Wire.read();
  int16_t rawAccY = (Wire.read() << 8) | Wire.read();
  int16_t rawAccZ = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Skip temp bytes
  int16_t rawGyroX = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroY = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroZ = (Wire.read() << 8) | Wire.read();

  // =========================================================================
  //  YOUR MATCHED PHYSICAL AXES CONVERSION MATRIX
  // =========================================================================
  // Pitch is raw X-gyro, Yaw is raw Z-gyro based on your diagnostic results
  float gyroX = (INVERT_PITCH ? -1.0 : 1.0) * (rawGyroX / 131.0);
  float gyroY = (INVERT_YAW   ? -1.0 : 1.0) * (rawGyroZ / 131.0);
  
  // Custom Trigonometry: Because Gravity pulls through Accel-Y, we map 
  // angles relative to Accel-Y instead of Accel-Z to prevent division by zero.
  float accelAngleX = atan2((float)rawAccZ, (float)rawAccY) * 180.0 / PI;
  float accelAngleY = atan2((float)-rawAccX, (float)rawAccY) * 180.0 / PI;

  // --- C. COMPLEMENTARY FILTER SENSOR FUSION ---
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

  // Debug Print: Check active updates
  Serial.print("PitchTilt: "); Serial.print(angleX);
  Serial.print(" | YawTilt: "); Serial.println(angleY);

  delay(10); 
}
