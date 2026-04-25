#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3); 
  pinMode(LED_BUILTIN, OUTPUT); // Initialize digital pin LED_BUILTIN as an output
  
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on
  delay(1000);                       // Wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // Turn the LED off
  delay(1000);                       // Wait for a second
}
// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
