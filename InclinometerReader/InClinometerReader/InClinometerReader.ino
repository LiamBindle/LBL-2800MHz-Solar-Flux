
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
    RTC.squareWave(SQWAVE_NONE);
    RTC.alarmInterrupt(ALARM_1, true);

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

#define NSAMPLES 5
#define ANALOGINPUTS 3

volatile bool time_to_sample = false;

void sample_interrupt() {
    time_to_sample = true;
}

void sample_loop() {
    int sampling_duration = 3;
    int analog_input[ANALOGINPUTS] = {8, 9, 10};
    char temp_cstr[128];
    bool led_value = false;
    time_t t = RTC.get();

    int elapsed_days = elapsedDays(t) - 18000;
    sprintf(temp_cstr, "d%dh%d.csv", elapsed_days, hour(t));
    String fname(temp_cstr);
    Serial.println("Opening '" + fname + "'");
    File datafile = SD.open(fname, FILE_WRITE);

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

        t = RTC.get();
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
    Serial.println("Closing '" + fname + "'");
    datafile.close();
    digitalWrite(LED_BUILTIN, LOW);
}

const int INCLINOMETER_PIN = 0;
const int INCLINOMETER_RELAY_PIN = 4;

void set_inclinometer_relay(bool value) {
  digitalWrite(INCLINOMETER_RELAY_PIN, value);
}

float get_zenith() {
  // accuracy is += 0.5
  float reading;

  const float x1 = 141;
  const float y1 = 34.9;
  const float x2 = 446;
  const float y2 = 79.6;
  const float m = (y2-y1)/(x2-x1);
  const float b = y1 - m*x1;

  reading = m * analogRead(INCLINOMETER_PIN) + b;

  return reading;

  // top : 0.701 V -- 141 +- 1 => 34.9 deg
  // bot: 2.19 V -- 446 +- 2 => 79.6 deg

//  Serial.println("Zenith: " + String(reading) + " [deg]");
}

static const int INA = 6;
static const int INB = 7;
static const int PWM = 5;

void set_direction_increase_zenith() {
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
}
void set_direction_decrease_zenith() {
  digitalWrite(INA, LOW);
  digitalWrite(INB, HIGH);
}
void stop_movement() {
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);
  analogWrite(PWM, 0);
}

void set_pwm(int duty_cycle) {
  analogWrite(PWM, duty_cycle);
}

void nudge_ms(unsigned long duration_ms, int high_value=255) {
  analogWrite(PWM, high_value);
  delay(duration_ms);
  analogWrite(PWM, 0);
}

void set_zenith(float target_zenith) {
  float nudge_mode_tol = 10; // [deg]

  Serial.println("Setting zenith to " + String(target_zenith) + " [deg]");
  Serial.println("Current zenith is " + String(get_zenith()) + " [deg]");
  Serial.println("Moving at full speed until within " + String(nudge_mode_tol) + " [deg]");

  
  bool pre_too_low = get_zenith() < target_zenith;
  bool post_too_low;
  while(1) {
    post_too_low = get_zenith() < target_zenith;
    if(post_too_low) {
      set_direction_increase_zenith();
      set_pwm(255);
    } else {
      set_direction_decrease_zenith();
      set_pwm(255);
    }

    if(abs(get_zenith() - target_zenith) < nudge_mode_tol) {
      break;
    } else if (pre_too_low != post_too_low) {
      break;
    }

    delay(20);
  }

  
  Serial.println("Switching to nudge mode");
  Serial.println("Current zenith is " + String(get_zenith()) + " [deg]");

  int nudge_duration = 1024;
  
  while(nudge_duration > 0) {
    pre_too_low = get_zenith() < target_zenith;
    if(pre_too_low) {
      set_direction_increase_zenith();
    } else {
      set_direction_decrease_zenith();
    }

    nudge_ms(nudge_duration);
    delay(500);

    post_too_low = get_zenith() < target_zenith;
    if(pre_too_low != post_too_low) {
      nudge_duration /= 2;
    }
    Serial.println("Finished setting the position");
    Serial.println("Current: " + String(get_zenith()) + " | Target: " + String(target_zenith) + " [deg]");
  }
}

void setup() {
    Serial.begin(9600);
    while(!Serial);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(INCLINOMETER_RELAY_PIN, OUTPUT);
    pinMode(INA, OUTPUT);
    pinMode(INB, OUTPUT);
    pinMode(PWM, OUTPUT);
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

//    int CS=53;
//    if(!SD.begin(CS)) {
//      Serial.println("Failed to initialize SD card");
//      while(1);
//    }
}

void loop() {
    //delay(5000);//wait 5 seconds before going to sleep. In real senairio keep this as small as posible
    //go_to_sleep();
    //sample_loop();
    //while(1);
    set_inclinometer_relay(HIGH);
    delay(1000);
    set_zenith(65);
    while(1);
//    delay(1000);
//
//    set_direction_decrease_zenith();
//    Serial.println("Going up");
//    delay(500);
////    nudge_ms(1000);
//    read_zenith();
////    set_direction_decrease_zenith();
//    Serial.println("Going down");
//    delay(500);
////    nudge_ms(1000);
//    read_zenith();
//    Serial.println("Zenith: " + String(get_zenith()) + " [deg]");
//    delay(1000);
    
}
