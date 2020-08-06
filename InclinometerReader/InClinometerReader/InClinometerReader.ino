
#include <avr/sleep.h>//this AVR library contains the methods that controls the sleep modes
#define interruptPin 2 //Pin we are going to use to wake up the Arduino
#include <DS3232RTC.h>  //RTC Library https://github.com/JChristensen/DS3232RTC

//#include <SPI.h>
//#include <SD.h>

static const int TEMP_5V_PIN = 1;
static const int TEMP_SENS_PIN = 2;
float get_temperature() {
  digitalWrite(TEMP_5V_PIN, HIGH);
  delay(500);
  float reading = analogRead(TEMP_SENS_PIN) * 5.0 / 1024.0;
  reading = (reading-0.424)/0.00625;
  Serial.println("Temperature reading is " + String(reading) + " [deg C]");
  digitalWrite(TEMP_5V_PIN, LOW);
  return reading;
}

static const int HEATER_RELAY = 10;
void turn_heaters_on() {
  digitalWrite(HEATER_RELAY, HIGH);
}
void turn_heaters_off() {
  digitalWrite(HEATER_RELAY, LOW);
}

inline void warmup_actuator()  {
  float temp = get_temperature();
  Serial.println("Current temperature is " + String(temp) + " [deg C]");
  if(temp < -20 && temp > -50) {
    Serial.println("Turning heaters on");
    turn_heaters_on();
    for(int i = 0; i < 45 && temp < 20; ++i) {
      delay(60000); // 60 seconds
      temp = get_temperature();
      Serial.println("Current temperature is " + String(temp) + " [deg C]");
    }
    Serial.println("Turning heaters off");
    turn_heaters_off();
  } else {
    Serial.println("It is warm enough that the heaters don't need to be turned on");
  }
}

void wakeup(){
    sleep_disable();
    detachInterrupt(0);
}

void go_to_sleep() {
    time_t t = RTC.get();

    byte wake_hour = 15;
    byte wake_minute = 30;
    char temp_cstr[8];
    sprintf(temp_cstr, "%02d:%02d", wake_hour, wake_minute);
    Serial.println("Setting next wakeup at " + String(temp_cstr));
    RTC.setAlarm(ALM1_MATCH_HOURS, 0, wake_minute, wake_hour, 0);
    RTC.alarm(ALARM_1);
    RTC.squareWave(SQWAVE_NONE);
    RTC.alarmInterrupt(ALARM_1, true);

    delay(1000);

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

#define NSAMPLES 15
#define ANALOGINPUTS 3

static const int INCLINOMETER_PIN = 0;

volatile bool time_to_sample = false;

static void sample_interrupt() {
    time_to_sample = true;
}

//static void sample_loop() {
//    int sampling_duration = 1;
//    int analog_input[ANALOGINPUTS] = {INCLINOMETER_PIN, 9, 10};
//    char temp_cstr[32];
//    bool led_value = false;
//    time_t t = RTC.get();
//
//    int elapsed_days = elapsedDays(t) - 18000;
//    sprintf(temp_cstr, "d%dh%d.csv", elapsed_days, hour(t));
//    String fname(temp_cstr);
//    Serial.println("Opening '" + fname + "'");
//    File datafile = SD.open(fname, FILE_WRITE);
//
//    String header = "DATE";
//    for(int i = 0; i < ANALOGINPUTS; ++i) {
//        header += ", MU(A" + String(analog_input[i]) + "), STD(A" + String(analog_input[i]) + ")";
//    }
//    Serial.println(header);
//    datafile.println(header);
//
//    for(int sampling_elapsed = 0; sampling_elapsed < sampling_duration; ++sampling_elapsed) {
//        int samples [ANALOGINPUTS][NSAMPLES] = {0};
//        int samples_taken = 0;
//
//        RTC.alarmInterrupt(ALARM_1, false);
//        attachInterrupt(0, sample_interrupt, FALLING);
//        RTC.squareWave(SQWAVE_1_HZ);
//
//        while(samples_taken < NSAMPLES) {
//            if(time_to_sample) {
//                time_to_sample = false;
//                for(int i = 0; i < ANALOGINPUTS; ++i) {
//                    samples[i][samples_taken] = analogRead(analog_input[i]);
//                }
//                led_value = !led_value;
//                digitalWrite(LED_BUILTIN, led_value);
//                ++samples_taken;
//            }
//            delay(50);
//        }
//
//        unsigned int sample_mean[ANALOGINPUTS] = {0};
//        for(int i = 0; i < ANALOGINPUTS; ++i) {
//            sample_mean[i] = 0;
//            for(int j = 0; j < NSAMPLES; ++j) {
//                sample_mean[i] += (unsigned long) samples[i][j];
//            }
//            sample_mean[i] /= (unsigned long)NSAMPLES;
//        }
//        unsigned int sample_std[ANALOGINPUTS] = {0};
//        for(int i = 0; i < ANALOGINPUTS; ++i) {
//            sample_std[i] = 0;
//            for(int j = 0; j < NSAMPLES; ++j) {
//                sample_std[i] += ((unsigned long)samples[i][j]-sample_mean[i])*((unsigned long)samples[i][j]-sample_mean[i]);
//            }
//            sample_std[i] = sqrt(sample_std[i]/(unsigned long)NSAMPLES);
//        } 
//
//        t = RTC.get();
//        sprintf(temp_cstr, "%4d-%02d-%02dT%02d:%02d:%02d", year(t), month(t), day(t), hour(t), minute(t), second(t));
//        String str(temp_cstr);
//        for(int i = 0; i < ANALOGINPUTS; ++i) {
//            sprintf(temp_cstr, ", %4d", sample_mean[i]);
//            str += temp_cstr;
//            sprintf(temp_cstr, ", %4d", sample_std[i]);
//            str += temp_cstr;
//        }
//
//        Serial.println(str);
//        datafile.println(str);
//    }
//    Serial.println("Closing '" + fname + "'");
//    datafile.close();
//    digitalWrite(LED_BUILTIN, LOW);
//}

static const int INCLINOMETER_RELAY_PIN = 4;

static void set_inclinometer_relay(bool value) {
  digitalWrite(INCLINOMETER_RELAY_PIN, value);
}

static float get_zenith() {
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

static void set_direction_increase_zenith() {
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
}
static void set_direction_decrease_zenith() {
  digitalWrite(INA, LOW);
  digitalWrite(INB, HIGH);
}
static void stop_movement() {
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);
  analogWrite(PWM, 0);
}

static void set_pwm(int duty_cycle) {
  analogWrite(PWM, duty_cycle);
}

static void nudge_ms(unsigned long duration_ms, int high_value=255) {
  analogWrite(PWM, high_value);
  delay(duration_ms);
  analogWrite(PWM, 0);
}

static void set_zenith(float target_zenith) {
  float nudge_mode_tol = 10; // [deg]

  unsigned long timeout = 15*60*1000;
  unsigned long start_time = millis();

  warmup_actuator();

  Serial.println("Setting zenith to " + String(target_zenith) + " [deg]");
  Serial.println("Current zenith is " + String(get_zenith()) + " [deg]");
  Serial.println("Moving at full speed until within " + String(nudge_mode_tol) + " [deg]");

  
  bool pre_too_low = get_zenith() < target_zenith;
  bool post_too_low;
  while(1) {
    post_too_low = get_zenith() < target_zenith;
    if(post_too_low) {
      set_direction_increase_zenith();
    } else {
      set_direction_decrease_zenith();
    }

    if(abs(get_zenith() - target_zenith) < nudge_mode_tol) {
      break;
    } else if (pre_too_low != post_too_low) {
      break;
    } else if (millis() - start_time > timeout) {
      break;
    }

    set_pwm(255);

    delay(20);
  }

  set_pwm(0);

  
  Serial.println("Switching to nudge mode");
  Serial.println("Current zenith is " + String(get_zenith()) + " [deg]");

  int nudge_duration = 1024;
  
  while(nudge_duration > 8) {
    pre_too_low = get_zenith() < target_zenith;
    if(pre_too_low) {
      set_direction_increase_zenith();
    } else {
      set_direction_decrease_zenith();
    }

    nudge_ms(nudge_duration);
    delay(1000);

    post_too_low = get_zenith() < target_zenith;
    if(pre_too_low != post_too_low) {
      nudge_duration /= 2;
    }

    if(millis() - start_time > timeout) {
      nudge_duration = 0;
    }
  }
  Serial.println("Finished setting the position (elapsed time: " + String((float)(millis() - start_time)/1000.0) + " [sec])");
  Serial.println("Current: " + String(get_zenith()) + " | Target: " + String(target_zenith) + " [deg]");
    
}

static double calc_zenith(double latitude, double longitude, double timezone, long y, long m, long d, long h, long t_min, long t_sec) {
    const double DEG2RAD=M_PI/180.0;
    const double RAD2DEG=180.0/M_PI;

    if(m == 1 || m == 2) {
        y -= 1;
        m += 12;
    }
    long A = y/100;
    double B = 2.0 - A + (long)(A/4);
    double time_past_midnight = ((double)h + (double)t_min/60.0 +(double)(t_sec)/3600)/24.0;
    double jul_day = (long)(365.25*(y+4716))+(long)(30.6001*(m+1))+d+B-1524.5-timezone/24.0 + time_past_midnight;
    double jul_century = (jul_day-2451545.0)/36525.0;

    double geo_mean_long_sun = fmod(280.46646+jul_century*(36000.76983+jul_century*0.0003032), 360.0)*DEG2RAD;
    double geo_mean_anom_sun = (357.52911+jul_century*(35999.05029-0.0001537*jul_century))*DEG2RAD;
    double ecc_earth_orbit = 0.016708634-jul_century*(0.000042037+0.0000001267*jul_century);

    double mean_obliq_ecl = 23.0+(26.0+((21.448-jul_century*(46.815+jul_century*(0.00059-jul_century*0.001813))))/60.0)/60.0;
    double obliq_corr = mean_obliq_ecl+0.00256*cos(DEG2RAD*(125.04-1934.136*jul_century));
    double vary = tan(DEG2RAD * obliq_corr/2.0)*tan(DEG2RAD * obliq_corr/2.0);

    double sun_eqn_center = sin(geo_mean_anom_sun)*(1.914602-jul_century*(0.004817+0.000014*jul_century))+sin(2*geo_mean_anom_sun)*(0.019993-0.000101*jul_century)+sin(3*geo_mean_anom_sun)*0.000289;
    double sun_true_long = geo_mean_long_sun*RAD2DEG+sun_eqn_center;
    double sun_app_long = sun_true_long-0.00569-0.00478*sin(DEG2RAD*(125.04-1934.136*jul_century));
    double sun_declin = asin(sin(DEG2RAD*obliq_corr)*sin(DEG2RAD*sun_app_long));

    double eqn_of_time = (vary*sin(2*geo_mean_long_sun)-2*ecc_earth_orbit*sin(geo_mean_anom_sun)+4*ecc_earth_orbit*vary*sin(geo_mean_anom_sun)*cos(2*geo_mean_long_sun)-0.5*vary*vary*sin(4*geo_mean_long_sun)-1.25*ecc_earth_orbit*ecc_earth_orbit*sin(2*geo_mean_anom_sun))*RAD2DEG*4;
    double true_solar_time = fmod(time_past_midnight*1440.0+eqn_of_time+4.0*longitude-60.0*timezone, 1440.0);

    double hour_angle;
    if(true_solar_time/4.0 < 0) {
        hour_angle = true_solar_time/4.0+180.0;
    } else {
        hour_angle = true_solar_time/4.0-180.0;
    }

    double solar_zenith_angle = acos(sin(DEG2RAD*latitude)*sin(sun_declin)+cos(DEG2RAD*latitude)*cos(sun_declin)*cos(DEG2RAD*hour_angle))*RAD2DEG;
    return solar_zenith_angle;

}

#define NZENITHCALCS 180
static double calc_tomorrows_min_zenith() {
    Serial.println("Calculating tomorrows minimum zenith...");
    time_t t = nextMidnight(RTC.get());
    double latitude = 54;
    double longitude = -104.5;
    double timezone=-6;

    int tomorrow_year = year(t);
    int tomorrow_month = month(t);
    int tomorrow_day = day(t);

    char temp_cstr[16];

    float min_zenith=90;
    int min_zenith_hour = 0;
    int min_zenith_minute = 0;
    for(int i = 0; i < NZENITHCALCS; ++i) {
      int i_hour = i/60 + 11;
      int i_minute = i%60;
      double i_zenith = calc_zenith(latitude, longitude, timezone, tomorrow_year, tomorrow_month, tomorrow_day, i_hour, i_minute, 0);
      if(i_zenith < min_zenith) {
        min_zenith=i_zenith;
        min_zenith_hour = i_hour;
        min_zenith_minute = i_minute;
      }
    }

    sprintf(temp_cstr, "%02d:%02d", min_zenith_hour, min_zenith_minute);
    Serial.println("Tomorrows minimum zenith is " + String(min_zenith) + " [deg] at " + String(temp_cstr));
    
    return min_zenith;
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
    pinMode(INCLINOMETER_RELAY_PIN, OUTPUT);
    pinMode(INA, OUTPUT);
    pinMode(INB, OUTPUT);
    pinMode(PWM, OUTPUT);
    pinMode(HEATER_RELAY, OUTPUT);
    pinMode(TEMP_5V_PIN, OUTPUT);
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

    turn_heaters_off();

//    RTC.set(compileTime()); // Don't leave uncommented

//    int CS=9;
//    if(!SD.begin(CS)) {
//      Serial.println("Failed to initialize SD card");
//      while(1);
//    }

    time_t t = RTC.get();
    char temp_cstr[6];
    sprintf(temp_cstr, "%02d:%02d", hour(t), minute(t));
    Serial.println("Powered on at " + String(temp_cstr));
    
    set_inclinometer_relay(HIGH);
    delay(5000);
    Serial.println("Resetting position to 60 [deg]...");
    set_zenith(60.0);
    set_inclinometer_relay(LOW);
}

void loop() {
  float abs_min_zenith = 34.9;
  float abs_max_zenith = 79.6;  
  
  float tomorrows_min_zenith = calc_tomorrows_min_zenith(); // 80 B
  set_inclinometer_relay(HIGH);
  delay(5000);
  Serial.println("Reading current position...");
  float current_zenith = 0;
  const int nsamples = 30;
  for(int i = 0; i < nsamples; ++i) {
    current_zenith += get_zenith();
  }
  current_zenith /= (float)nsamples;
  Serial.println("Current position is " + String(current_zenith) + " [deg]");
  if(abs(current_zenith - tomorrows_min_zenith) > 0.5) {
    if(tomorrows_min_zenith < abs_min_zenith) {
      tomorrows_min_zenith = abs_min_zenith;
    }
    if(tomorrows_min_zenith > abs_max_zenith) {
      tomorrows_min_zenith = abs_max_zenith;
    }
    Serial.println("Difference is greater than 0.5 [deg]. Moving to new position.");
    set_zenith(tomorrows_min_zenith);
  }

  set_inclinometer_relay(LOW);
  delay(1000);
  go_to_sleep();
}
