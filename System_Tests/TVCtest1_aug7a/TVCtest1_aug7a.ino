#include <Servo.h>
#include <Wire.h>

Servo servoX;
Servo servoY;

// =========================================================================
//  MICRO-ADJUST THESE TWO VALUES TO GET YOUR GIMBAL PERFECTLY CENTERED
// =========================================================================
// If your inner ring tilts slightly left/right or up/down when booted,
// change these values away from 90 (try values between 80 and 100) until straight.
const int CALIBRATED_CENTER_X = 98; 
const int CALIBRATED_CENTER_Y = 85; 

// Expanded test parameter: 15 degrees of sweeping freedom
const int MAX_TVC_TILT_DEGREES = 10; 

// Software limits calculated automatically
const int MIN_LIMIT_X = CALIBRATED_CENTER_X - MAX_TVC_TILT_DEGREES;
const int MAX_LIMIT_X = CALIBRATED_CENTER_X + MAX_TVC_TILT_DEGREES;
const int MIN_LIMIT_Y = CALIBRATED_CENTER_Y - MAX_TVC_TILT_DEGREES;
const int MAX_LIMIT_Y = CALIBRATED_CENTER_Y + MAX_TVC_TILT_DEGREES;

void setup() {
  Serial.begin(115200);
  
  // Initialize native I2C to wake up your functional register loops
  Wire.begin();
  Wire.setClock(100000);

  servoX.attach(1); 
  servoY.attach(2); 

  // Command the servos to stay rigidly at your calibrated home alignment
  servoX.write(CALIBRATED_CENTER_X);
  servoY.write(CALIBRATED_CENTER_Y);
  
  Serial.println(">>> SERVOS ENERGIZED AND LOCKED AT HOME POSITION <<<");
  Serial.println("1. Look at your 3D-printed gimbal rings.");
  Serial.println("2. If they are still crooked, change CALIBRATED_CENTER_X or Y variables, re-upload.");
  Serial.println("3. Once the rings look perfectly centered, click the message bar above, type anything, and press ENTER to start the 15-degree sweep.");

  // Infinite hold loop: Keeps the servos centered until you send a character in the Serial Monitor
  while (Serial.available() == 0) {
    delay(10); 
  }
  Serial.read(); // Clear the text buffer
}

void loop() {
  Serial.println("Executing 15-Degree Safety Sweep...");

  // --- SWEEP AXIS X ---
  for (int angle = CALIBRATED_CENTER_X; angle <= MAX_LIMIT_X; angle++) {
    servoX.write(angle);
    delay(30); 
  }
  for (int angle = MAX_LIMIT_X; angle >= MIN_LIMIT_X; angle--) {
    servoX.write(angle);
    delay(30);
  }
  for (int angle = MIN_LIMIT_X; angle <= CALIBRATED_CENTER_X; angle++) {
    servoX.write(angle);
    delay(30);
  }
  delay(500); 

  // --- SWEEP AXIS Y ---
  for (int angle = CALIBRATED_CENTER_Y; angle <= MAX_LIMIT_Y; angle++) {
    servoY.write(angle);
    delay(30);
  }
  for (int angle = MAX_LIMIT_Y; angle >= MIN_LIMIT_Y; angle--) {
    servoY.write(angle);
    delay(30);
  }
  for (int angle = MIN_LIMIT_Y; angle <= CALIBRATED_CENTER_Y; angle++) {
    servoY.write(angle);
    delay(30);
  }
  delay(1000); 
}
