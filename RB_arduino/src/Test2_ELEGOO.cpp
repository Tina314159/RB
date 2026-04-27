#include <Arduino.h>

const int TRIGGER_PIN = 2; // Hardware interrupt pin
volatile bool triggered = false;
const int LED_indicator = 1; 
    //1 = LED will be used for indication (introcude delay)

void onTrigger() {
  triggered = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIGGER_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT); // Initialize digital pin LED_BUILTIN as an output 
  digitalWrite(LED_BUILTIN, HIGH);  
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), onTrigger, RISING);
}

void loop() {
  if (triggered) {
    triggered = false;
    Serial.println("START");  // Python listens for this
    if (LED_indicator == 1) {
      digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on
      delay(1000);                       // Wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // Turn the LED off
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH); 
    }
  }
}
