#include <Servo.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

Servo servoX;
Servo servoY;

const int MPU_ADDR = 0x68; 
File flightLog;

// =========================================================================
//  GLOBAL VARIABLE FIX:
//  Declaring fileName out here allows BOTH setup() and loop() to access it!
// =========================================================================
char fileName[16]; 

// =========================================================================
//  1. HARDWARE CALIBRATION & REVERSING TOGGLES
// =========================================================================
const int CALIBRATED_CENTER_X = 98; //92
const int CALIBRATED_CENTER_Y = 86; //95
const float AXIS_Y_MULTIPLIER = 1.5; 
const int STRICT_TILT_LIMIT = 10; 

const float ALPHA = 0.98; 

const bool INVERT_PITCH = true;  
const bool INVERT_YAW   = true; 

// =========================================================================
//  2. PID TUNING CONSTANTS
// =========================================================================
const float Kp = 0.11;  
const float Ki = 0;  
const float Kd = 0.0533;  

// Noise Filter Buffer
const int FILTER_SIZE = 5; 
float historyX[FILTER_SIZE] = {0};
float historyY[FILTER_SIZE] = {0};
int filterIndex = 0;

// Internal math registers
unsigned long lastTime = 0;
float angleX = 0.0, angleY = 0.0;
float errorSumX = 0.0, errorSumY = 0.0;
float lastErrorX = 0.0, lastErrorY = 0.0;

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("--- Teensy 4.1 Auto-Increment Logging Flight Computer ---");

  servoX.attach(1); 
  servoY.attach(2); 

  // --- A. INITIALIZE THE BLACK BOX MICROSD RECORDER ---
  Serial.print("Initializing SD card... ");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("FAILED! Check card formatting.");
    while(1) { delay(10); } 
  }
  Serial.println("SUCCESS.");

  // --- AUTO-INCREMENTING FILE NAME ENGINE ---
  int fileCount = 0;
  while (true) {
    sprintf(fileName, "flight%d.csv", fileCount);
    if (!SD.exists(fileName)) {
      break; 
    }
    fileCount++; 
  }

  Serial.print("Creating log profile: ");
  Serial.println(fileName);

  flightLog = SD.open(fileName, FILE_WRITE);
  if (flightLog) {
    flightLog.println("TimeMS,FilteredTiltX,FilteredTiltY,TargetServoX,TargetServoY");
    flightLog.close(); 
  }

  // --- B. INITIALIZE SENSOR ---
  Wire.begin();
  Wire.setClock(100000); 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: MPU6050 flat sensor missing!");
    while (1) { delay(10); } 
  }

  servoX.write(CALIBRATED_CENTER_X);
  servoY.write(CALIBRATED_CENTER_Y);
  
  delay(1000);
  lastTime = micros(); 
  Serial.println("--> BLACK BOX ACTIVE: System tracking live.");
}

void loop() {
  // --- 1. TIMING AND DELTA ---
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; 
  lastTime = currentTime;
  if (dt <= 0.0) dt = 0.01; 

  // --- 2. PULL REGISTER PACKETS ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  
  // FIXED: Explicit conversion to (uint8_t) forces the compiler to match candidate 2 perfectly
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, true); 

  int16_t rawAccX = (Wire.read() << 8) | Wire.read();
  int16_t rawAccY = (Wire.read() << 8) | Wire.read();
  int16_t rawAccZ = (Wire.read() << 8) | Wire.read();
  
  // Skip temp bytes safely
  Wire.read(); Wire.read(); 
  
  int16_t rawGyroX = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroY = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroZ = (Wire.read() << 8) | Wire.read(); // Now read and used inside logging engine

  // Native flat conversion formulas
  float gyroX = (INVERT_PITCH ? -1.0 : 1.0) * (rawGyroX / 131.0);
  float gyroY = (INVERT_YAW   ? -1.0 : 1.0) * (rawGyroY / 131.0);
  
  float accelAngleX = atan2((float)rawAccY, (float)rawAccZ) * 180.0 / PI;
  float accelAngleY = atan2((float)-rawAccX, (float)rawAccZ) * 180.0 / PI;

  // --- 3. FILTER AND SMOOTH SENSOR DATA ---
  angleX = ALPHA * (angleX + gyroX * dt) + (1.0 - ALPHA) * accelAngleX;
  angleY = ALPHA * (angleY + gyroY * dt) + (1.0 - ALPHA) * accelAngleY;

  historyX[filterIndex] = angleX;
  historyY[filterIndex] = angleY;
  filterIndex = (filterIndex + 1) % FILTER_SIZE;

  float smoothAngleX = 0, smoothAngleY = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    smoothAngleX += historyX[i];
    smoothAngleY += historyY[i];
  }
  smoothAngleX /= FILTER_SIZE;
  smoothAngleY /= FILTER_SIZE;

  // --- 4. PID CALCULATIONS ---
  float errorX = 0.0 - smoothAngleX;
  float errorY = 0.0 - smoothAngleY;

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

  // --- 5. DRIVE SERVOS ---
  int targetServoX = CALIBRATED_CENTER_X + finalOffsetX;
  int targetServoY = CALIBRATED_CENTER_Y + finalOffsetY;

  servoX.write(targetServoX);
  servoY.write(targetServoY);

  // --- 6. REAL-TIME BLACK BOX LOGGING ---
  // FIXED: Using our global variable name to dump unique flight timelines dynamically
  flightLog = SD.open(fileName, FILE_WRITE);
  if (flightLog) {
    flightLog.print(millis()); flightLog.print(",");
    flightLog.print(smoothAngleX); flightLog.print(",");
    flightLog.print(smoothAngleY); flightLog.print(",");
    flightLog.print(targetServoX); flightLog.print(",");
    flightLog.println(targetServoY);
    flightLog.close(); 
  }

  delay(10); 
}
