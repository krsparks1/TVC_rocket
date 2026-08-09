#include <Servo.h>

// Create two distinct servo control objects
Servo servoX;
Servo servoY;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Teensy 4.1 Dual-Servo Hardware Sweep Test ---");

  // Mount the software logic to our specific physical PWM signal pins
  servoX.attach(1); // Servo X signal must be wired to Teensy Pin 1
  servoY.attach(2); // Servo Y signal must be wired to Teensy Pin 2

  // Command both servos to snap immediately to their exact center position (90 degrees)
  Serial.println("Centering servos to 90 degrees...");
  servoX.write(90);
  servoY.write(90);
  delay(2000); // Wait 2 seconds to let the user observe the initial centering snap
}

void loop() {
  Serial.println("Starting continuous sweep sequence...");

  // Sweep both servos slowly from 70 degrees up to 110 degrees
  // This simulates a gentle TVC rocket flight correction loop
  for (int angle = 70; angle <= 110; angle++) {
    servoX.write(angle);
    servoY.write(angle);
    delay(15); // Small delay to control the speed of the sweep movement
  }

  delay(200); // Pause briefly at the top end of the movement

  // Sweep both servos back down from 110 degrees to 70 degrees
  for (int angle = 110; angle >= 70; angle--) {
    servoX.write(angle);
    servoY.write(angle);
    delay(15);
  }

  delay(200); // Pause briefly at the bottom end before repeating the loop
}
