#include <avr/sleep.h>
#define interruptPin 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, HIGH);
}

void wakeUp() {
  Serial.println("Interrupt Fired!");
  sleep_disable();
  detachInterrupt(0);
}

void go_to_sleep() {
  sleep_enable();
  attachInterrupt(0, wakeUp, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  sleep_cpu();
  Serial.println("Just woke up!");
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(5000);
  Serial.println("Going to sleep.");
  go_to_sleep();
}
