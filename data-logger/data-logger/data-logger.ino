
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
    RTC.setAlarm(ALM1_MATCH_HOURS, 0, 30, next_wakeup, 0);
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

#define NSAMPLES 60
#define ANALOGINPUTS 5

volatile bool time_to_sample = false;

void sample_interrupt() {
    time_to_sample = true;
}

void sample_loop() {
    int sampling_duration = 60;
    int analog_input[ANALOGINPUTS] = {1, 2, 3, 4, 5};
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

const int AB_RELAY = 26;
const int FLUX_RECV_RELAY = 28;
const int DISABLE_6M_RELAY = 40;

void enable_relays() {
  digitalWrite(AB_RELAY, HIGH);
  digitalWrite(FLUX_RECV_RELAY, HIGH);
  digitalWrite(DISABLE_6M_RELAY, HIGH);
}

void disable_relays() {
  digitalWrite(AB_RELAY, LOW);
  digitalWrite(FLUX_RECV_RELAY, LOW);
  digitalWrite(DISABLE_6M_RELAY, LOW);
}

time_t compileTime()
{
    const time_t FUDGE(10);    //fudge factor to allow for upload time, etc. (seconds, YMMV)
    const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char compMon[4], *m;

    strncpy(compMon, compDate, 3);
    compMon[3] = '\0';
    m = strstr(months, compMon);

    tmElements_t tm;
    tm.Month = ((m - months) / 3 + 1);
    tm.Day = atoi(compDate + 4);
    tm.Year = atoi(compDate + 7) - 1970;
    tm.Hour = atoi(compTime);
    tm.Minute = atoi(compTime + 3);
    tm.Second = atoi(compTime + 6);

    time_t t = makeTime(tm);
    return t + FUDGE;        //add fudge factor to allow for compile time
}

void setup() {
    Serial.begin(9600);
    while(!Serial);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(interruptPin, INPUT_PULLUP);
    pinMode(AB_RELAY, OUTPUT);
    pinMode(FLUX_RECV_RELAY, OUTPUT);
    pinMode(DISABLE_6M_RELAY, OUTPUT);
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

    //RTC.set(compileTime()); // Don't leave uncommented

    delay(10000);

    int CS=53;
    if(!SD.begin(CS)) {
      Serial.println("Failed to initialize SD card");
      while(1);
    }
}

void loop() {
    enable_relays();
    delay(1000);
    sample_loop();
    disable_relays();
    delay(1000);
    go_to_sleep();
}
