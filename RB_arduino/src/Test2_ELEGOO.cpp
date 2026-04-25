#include <Arduino.h>

const int TRIGGER_PIN = 2; // Hardware interrupt pin
volatile bool triggered = false;

void onTrigger() {
  triggered = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIGGER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), onTrigger, RISING);
}

void loop() {
  if (triggered) {
    triggered = false;
    Serial.println("START");  // Python listens for this
  }
}
