
#include <avr/sleep.h>//this AVR library contains the methods that controls the sleep modes
#define interruptPin 2 //Pin we are going to use to wake up the Arduino
#include <DS3232RTC.h>  //RTC Library https://github.com/JChristensen/DS3232RTC

#include <SPI.h>
#include <SD.h>


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

#define NSAMPLES 10
#define ANALOGINPUTS 3

volatile bool time_to_sample = false;

void sample_interrupt() {
    time_to_sample = true;
}

void sample_loop() {
  int sampling_duration = 10;
  int analog_input[ANALOGINPUTS] = {8, 9, 10};
  char temp_cstr[128];
  bool led_value = false;

  
  File datafile = SD.open("test.csv", FILE_WRITE);

  String header = "DATE";
  for(int i = 0; i < ANALOGINPUTS; ++i) {
    header += ", MU(A" + String(analog_input[i]) + "), STD(A" + String(analog_input[i]) + ")";
  }
  Serial.println(header);
  datafile.println(header);
  
  for(int sampling_elapsed = 0; sampling_elapsed < sampling_duration; ++sampling_elapsed) {
    int samples [ANALOGINPUTS][NSAMPLES] = {0};
    int samples_taken = 0;

    RTC.alarmInterrupt(ALARM_1, false);
    attachInterrupt(0, sample_interrupt, FALLING);
    RTC.squareWave(SQWAVE_1_HZ);

    while(samples_taken < NSAMPLES) {
        if(time_to_sample) {
            time_to_sample = false;
            for(int i = 0; i < ANALOGINPUTS; ++i) {
                samples[i][samples_taken] = analogRead(analog_input[i]);
            }
            led_value = !led_value;
            digitalWrite(LED_BUILTIN, led_value);
            ++samples_taken;
        }
        delay(50);
    }

    unsigned long sample_mean[ANALOGINPUTS] = {0};
    for(int i = 0; i < ANALOGINPUTS; ++i) {
      sample_mean[i] = 0;
      for(int j = 0; j < NSAMPLES; ++j) {
        sample_mean[i] += (unsigned long) samples[i][j];
      }
      sample_mean[i] /= (unsigned long)NSAMPLES;
    }
    unsigned long sample_std[ANALOGINPUTS] = {0};
    for(int i = 0; i < ANALOGINPUTS; ++i) {
      sample_std[i] = 0;
      for(int j = 0; j < NSAMPLES; ++j) {
        sample_std[i] += ((unsigned long)samples[i][j]-sample_mean[i])*((unsigned long)samples[i][j]-sample_mean[i]);
      }
      sample_std[i] = sqrt(sample_std[i]/(unsigned long)NSAMPLES);
    } 
    
    time_t t = RTC.get();
    sprintf(temp_cstr, "%4d-%02d-%02dT%02d:%02d:%02d", year(t), month(t), day(t), hour(t), minute(t), second(t));
    String str(temp_cstr);
    for(int i = 0; i < ANALOGINPUTS; ++i) {
      sprintf(temp_cstr, ", %4d", sample_mean[i]);
      str += temp_cstr;
      sprintf(temp_cstr, ", %4d", sample_std[i]);
      str += temp_cstr;
    }
    
    Serial.println(str);
    datafile.println(str);
  }
  datafile.close();
}

void setup() {
    Serial.begin(9600);
    while(!Serial);
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

    int CS=53;
    if(!SD.begin(CS)) {
      Serial.println("Failed to initialize SD card");
      while(1);
    }
}

void loop() {
    // delay(5000);//wait 5 seconds before going to sleep. In real senairio keep this as small as posible
    // go_to_sleep();
    sample_loop();
    while(1);
}
