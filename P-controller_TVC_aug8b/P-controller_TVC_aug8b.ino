#include <Servo.h>
#include <Wire.h>

Servo servoX;
Servo servoY;

// The universal hardware address for your MPU6050 sensor
const int MPU_ADDR = 0x68; 

// =========================================================================
//  YOUR VERIFIED HARDWARE CALIBRATION TUNING VARIABLES
// =========================================================================
const int CALIBRATED_CENTER_X = 98; // Keeps your mount physically straight
const int CALIBRATED_CENTER_Y = 85; 

// Your verified mechanical multiplier that matches the Y-axis range to X
const float AXIS_Y_MULTIPLIER = 1.50; 

// The maximum degrees the computer is allowed to tilt the engine (safety wall)
const int MAX_TVC_TILT_LIMIT = 15; 

// =========================================================================
//  CONTROL PARAMETERS (STABILIZATION SENSITIVITY)
// =========================================================================
// This multiplier controls how aggressively the servos react to movements.
// If the gimbal under-corrects, increase this. If it shakes, decrease it.
const float STABILIZATION_GAIN = 0.45; 

// Timing variables to track the precise execution speed of the loop
unsigned long lastTime = 0;
float angleX = 0.0;
float angleY = 0.0;

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("--- Teensy 4.1 Active TVC Stabilization Loop ---");

  // Attach the control channels to your signal pins
  servoX.attach(1); 
  servoY.attach(2); 

  // Initialize native I2C lines 
  Wire.begin();
  Wire.setClock(100000); 

  // Wake up the MPU6050 by writing to its Power Management Register (0x6B)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.println("Error: Direct I2C sensor link failed!");
    while (1) { delay(10); } 
  }
  Serial.println("MPU6050 Sensor initialized successfully via direct registers.");

  // Command both servos to lock into their home targets on startup
  servoX.write(CALIBRATED_CENTER_X);
  servoY.write(CALIBRATED_CENTER_Y);
  
  delay(1000);
  lastTime = micros(); // Capture starting clock cycle timestamp
  Serial.println("--> ACTIVE FLIGHT MODE STANDBY: Pick up your sensor and tilt it!");
}

void loop() {
  // 1. Calculate time delta (dt) since the last loop iteration
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; // Convert microseconds to seconds
  lastTime = currentTime;

  // 2. Query 6 bytes of data starting from the raw Gyroscope Register (0x43)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  // Read raw 16-bit registers for X, Y, and Z axes
  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  // Convert raw integers into standard Degrees Per Second (DPS)
  float gyroX = rawX / 131.0;
  float gyroY = rawY / 131.0;
  
  // 3. Mathematical Integration: Update relative tilt tracking angles
  angleX += gyroX * dt;
  angleY += gyroY * dt;

  // 4. Calculate Feedback Correction: Oppose the motion with a negative sign
  // Multiplying by STABILIZATION_GAIN translates tilt angles into servo commands
  float correctionX = -angleX * STABILIZATION_GAIN;
  float correctionY = -angleY * STABILIZATION_GAIN;

  // Apply your verified structural Y-axis mechanical scaling factor
  float scaledCorrectionY = correctionY * AXIS_Y_MULTIPLIER;

  // Constrain calculations strictly within your physical 15-degree walls
  int finalOffsetX = constrain((int)correctionX, -MAX_TVC_TILT_LIMIT, MAX_TVC_TILT_LIMIT);
  int finalOffsetY = constrain((int)scaledCorrectionY, -MAX_TVC_TILT_LIMIT, MAX_TVC_TILT_LIMIT);

  // 5. Add corrections to your home coordinates to generate final servo targets
  int targetServoX = CALIBRATED_CENTER_X + finalOffsetX;
  int targetServoY = CALIBRATED_CENTER_Y + finalOffsetY;

  // Send the real-time tracking signals to the hardware
  servoX.write(targetServoX);
  servoY.write(targetServoY);

  // Debug Print: View the active data tracking outputs on your monitor
  Serial.print("Rocket Tilt X: "); Serial.print(angleX);
  Serial.print(" | Servo Target X: "); Serial.println(targetServoX);

  delay(10); // Loop updates at roughly 100Hz (Real-time speed)
}
