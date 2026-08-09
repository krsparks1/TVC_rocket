#include <Servo.h>
#include <Wire.h>

Servo servoX;
Servo servoY;

// =========================================================================
//  YOUR VERIFIED CALIBRATION VALUES 
// =========================================================================
const int CALIBRATED_CENTER_X = 98; 
const int CALIBRATED_CENTER_Y = 85; 

// Base radius of your circular sweep (in degrees)
const float BASE_RADIUS_DEGREES = 5.0; 

// =========================================================================
//  SOFTWARE SCALING MULTIPLIER TO FIX THE MECHANICAL MISMATCH
// =========================================================================
// Since your Y-axis is moving less, we use this multiplier to boost its range.
// 1.0 means default movement. 1.25 means it moves 25% MORE. 
// If your Y-axis is still moving too little, increase this value to 1.3 or 1.4.
const float AXIS_Y_MULTIPLIER = 1.50; 

// Controls the rotational speed of the circle loop.
const float SPEED_STEP = 0.25; 

void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  Wire.setClock(100000);

  servoX.attach(1); 
  servoY.attach(2); 

  servoX.write(CALIBRATED_CENTER_X);
  servoY.write(CALIBRATED_CENTER_Y);
  delay(1500)
}

void loop() {
  static float angleRad = 0.0;

  // Calculate standard trigonometric offsets
  float offsetX = cos(angleRad) * BASE_RADIUS_DEGREES;
  float offsetY = sin(angleRad) * BASE_RADIUS_DEGREES;

  // CRITICAL FIX: Multiply the Y-axis offset to compensate for the hardware limitation
  float scaledOffsetY = offsetY * AXIS_Y_MULTIPLIER;

  // Add the adjusted offsets to your verified centers
  int targetX = CALIBRATED_CENTER_X + (int)offsetX;
  int targetY = CALIBRATED_CENTER_Y + (int)scaledOffsetY;

  // Send the balanced positioning updates to the servos
  servoX.write(targetX);
  servoY.write(targetY);

  angleRad += SPEED_STEP;
  if (angleRad >= 2.0 * PI) {
    angleRad = 0.0; 
  }

  delay(15); 
}
