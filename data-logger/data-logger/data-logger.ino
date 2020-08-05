
#include <avr/sleep.h>//this AVR library contains the methods that controls the sleep modes
#define interruptPin 2 //Pin we are going to use to wake up the Arduino
#include <DS3232RTC.h>  //RTC Library https://github.com/JChristensen/DS3232RTC


const int wakeup_interval_hours = 6;

void wakeup(){
    sleep_disable();
    detachInterrupt(0);
}

void go_to_sleep() {
    time_t t = RTC.get();

    byte next_wakeup = (int(hour(t)/wakeup_interval_hours)+1)*wakeup_interval_hours % 24;
    Serial.println("Setting next wakeup at " + String(next_wakeup) + ":00");
    RTC.setAlarm(ALM1_MATCH_HOURS, 0, 0, next_wakeup, 0);
    RTC.alarm(ALARM_1);

    sleep_enable();
    attachInterrupt(0, wakeup, LOW);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    digitalWrite(LED_BUILTIN,LOW);
    Serial.println("Going to sleep at " + String(hour(t)) + ":" + String(minute(t)));
    delay(1000); // wait to allow ops to finish before sleeping
    sleep_cpu();

    digitalWrite(LED_BUILTIN,HIGH);
    t=RTC.get();
    Serial.println("Woke up at " + String(hour(t)) + ":" + String(minute(t)));
}

void setup() {
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(interruptPin, INPUT_PULLUP);
    digitalWrite(LED_BUILTIN,HIGH);

    // Init alarm to null
    RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
    RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
    RTC.alarm(ALARM_1);
    RTC.alarm(ALARM_2);
    RTC.alarmInterrupt(ALARM_1, false);
    RTC.alarmInterrupt(ALARM_2, false);
    RTC.squareWave(SQWAVE_NONE);
    RTC.alarmInterrupt(ALARM_1, true);
}

void loop() {
    delay(5000);//wait 5 seconds before going to sleep. In real senairio keep this as small as posible
    go_to_sleep();
}
