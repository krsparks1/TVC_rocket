void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Configure the onboard LED
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
  delay(500);                      // Wait half a second
  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off
  delay(500);                      // Wait half a second
}
