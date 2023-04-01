// SAMPLE OVER TIME
// First compiled by Amelia Ritger March 24, 2023
// Maybe make a function for READ ADC section? And remove any Serial printing for final publication.
// Remove alarmTrigger and add sleep functionality, maybe move Alarm2 up to setup out of loop since it is a one time thing?

#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#include <Adafruit_ADS1X15.h>
#include "ArduinoLowPower.h"
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
#include <Streaming.h>      // https://github.com/janelia-arduino/Streaming , allows you to use << formatting
#include <Time.h>           // https://github.com/PaulStoffregen/Time
#include "SdFat.h"          // https://github.com/greiman/SDFat

#include "CONFIG.h"
#include "SRC.h"

DS3232RTC rtc;            // Construct RTC
File datafile;            // Construct File for SD card
Adafruit_ADS1015 ads1015; // Construct an ads1015
Adafruit_ADS1115 ads1115; // Construct an ads1115

const int ledPin = LED_BUILTIN;
int ledState = LOW; // ledState used to set the LED
char timestamp[32]; // Current time from the RTC in text format, 32 bytes long
bool alarmTrigger = false; // Create variable set to FALSE for alarm nesting

// Function to print a timestamp ("last modified") callback to the SD card
void SDfileDate(uint16_t* date, uint16_t* time) {
  setSyncProvider(rtc.get);
  *date = FAT_DATE(year(), month(), day());
  *time = FAT_TIME(hour(), minute(), second());
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin 13 as an output (blink)
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Setup SD card
  Serial << "Initializing SD card..." << endl;

  if (!SD.begin(4)) {
    Serial << "sd card initialization failed!" << endl;
    while (1);
  }

  Serial << "sd card initialization done." << endl;

  if (SD.exists(FILE_NAME)) { // If file already exists, don't add a header. If it doesn't exist, then write a header.
    Serial << "File exists, opening file..." << endl;
  } else {
    Serial << "File doesn't exist, creating file and adding header" << endl;
    Serial << "Opening file..." << endl;
    datafile = SD.open(FILE_NAME, FILE_WRITE); // open the file
    Serial << "Writing header..." << endl;
    datafile << "Date,Time,J1 diff (mV),J1 oversampled (mV),J2+ (mV),J3 diff (mV),J4 oversampled (mV), J4 (mV), J4+ (mV),J4- (mV), J4+ oversampled (mV),J4- oversampled (mV)" << endl; // removed "Coin battery (mV)"
    datafile.close(); // close the file
    Serial << "Done." << endl;
  }

  // Initialize the alarms, clear the alarm flags, clear the alarm interrupt flags
  rtc.begin();
  rtc.setAlarm(DS3232RTC::ALM1_MATCH_DATE, 0, 0, 0, 1);
  rtc.setAlarm(DS3232RTC::ALM2_MATCH_DATE, 0, 0, 0, 1);
  rtc.alarm(DS3232RTC::ALARM_1);
  rtc.alarm(DS3232RTC::ALARM_2);
  rtc.alarmInterrupt(DS3232RTC::ALARM_1, false);
  rtc.alarmInterrupt(DS3232RTC::ALARM_2, false);
  rtc.squareWave(DS3232RTC::SQWAVE_NONE);

  rtc.set(compileTime()); // set the RTC time and date to the compile time (see SRC.h)

  time_t t = rtc.get(); // get the current time
  time_t alarmTime = t + ALARM_INTERVAL;    // calculate the alarm time by adding current RTC time with alarm interval
  formatTime(timestamp, t); // reformat time (see SRC.h)
  Serial << F("Current RTC date and time ") << timestamp << endl;   // print the current time to the serial port

  // set the Alarms
  rtc.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0); // set Alarm 1 to occur at sampling interval
  rtc.alarm(DS3232RTC::ALARM_1); // clear the alarm flag
  rtc.setAlarm(DS3232RTC::ALM2_MATCH_DATE, 0, START_MIN, START_HOUR, START_DAY); // set Alarm 2 to occur at specific date and time
  rtc.alarm(DS3232RTC::ALARM_2); // clear the alarm flag

  // Arduino low power mode until the alarm is triggered
  //while (!rtc.alarm(DS3232RTC::ALARM_2)) {
  //  LowPower.sleep(1000); // Sleep for 1 second
  //}

  //time_t startTime = rtc.get();
  //formatTime(timestamp, startTime);
  //Serial << "IT'S TIME TO START SAMPLING! " << timestamp << endl; // print the time when this part of the loop is running

  //Setup ADC 1015 and 1115
  Serial << "Getting differential reading from AIN0 (P) and AIN1 (N)" << endl;
  Serial << "ADC Range: +/- 4.096V (1 bit = 2mV)" << endl;
  ads1015.setGain(ADS1015_GAIN_VAL); // set Gain for ADC 1015
  ads1015.begin(0x48); // Initialize ads1015 at the default address 0x48
  Serial << "Getting differential reading from AIN0 (P) and AIN1 (N)" << endl;
  Serial << "ADC Range: +/- 1.024V (1 bit = 0.03125mV)" << endl;
  ads1115.setGain(ADS1115_GAIN_VAL); // set Gain for ADC 1115
  ads1115.begin(0x49); // Initialize ads1115 at address 0x49
}

void loop()
{
  if (alarmTrigger == false) {
    if (rtc.alarm(DS3232RTC::ALARM_2) ) {    // check alarm flag, clear it if set
      alarmTrigger = true; // set alarm trigger to TRUE to trigger Alarm 1
      time_t t = rtc.get();
      formatTime(timestamp, t);
      Serial << "ALARM_2 " << timestamp << endl; // print the time when this part of the loop is running
    }
  }

  else {
    if (rtc.alarm(DS3232RTC::ALARM_1)) { // check alarm flag (and clear the flag if set)
      time_t t = rtc.get(); // get the current time
      formatTime(timestamp, t);
      time_t alarmTime = t + ALARM_INTERVAL; // calculate the next alarm time
      rtc.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0); // set the alarm
      Serial << "IT'S TIME TO SAMPLE! " << timestamp << endl; // print the time when this part of the loop is running

      SdFile::dateTimeCallback(SDfileDate); // Set file date and time on sd card ("last modified"). Check out this page if having issues: https://arduino.stackexchange.com/questions/39126/how-does-one-set-attributes-for-sd-files
      datafile = SD.open(FILE_NAME, FILE_WRITE); // open the file. note that only one file can be open at a time, so you have to close this one before opening another.

      if (datafile) { // if the file opened okay, write to it:
        Serial << "Writing to SD Card..." << endl;
        time_t TIME = rtc.get();
        datafile << month(TIME) << "/" << day(TIME) << "/" << year(TIME) << "," << hour(TIME) << ":" << minute(TIME) << ":" << second(TIME) << "," ;

        // Read ADC1015 inputs
        int16_t adc1_1, adc1_diff; //adc1_2 for coin batt
        adc1_1 = ads1015.readADC_SingleEnded(2); // Read A2
        Serial << "J2+: " << adc1_1 << "(" << adc1_1 * ADS1015_GAIN_MULT << "mV)" << endl;
        //adc1_2 = ads1015.readADC_SingleEnded(3); // Read A3
        //Serial << "coin batt: " << adc1_2 << "(" << adc1_2 * ADS1015_GAIN_MULT << "mV)" << endl;
        adc1_diff = ads1015.readADC_Differential_0_1(); //Read A0/A1 differential
        Serial << "J3 differential: " << adc1_diff << "(" << adc1_diff * ADS1015_GAIN_MULT << "mV)" << endl;

        // Read ADC1115 inputs
        int16_t adc2_1, adc2_2, adc2_diff;
        adc2_1 = ads1115.readADC_SingleEnded(2); // Read A2
        Serial << "J4+: " << adc2_1 << "(" << adc2_1 * ADS1115_GAIN_MULT << "mV)" << endl;
        adc2_2 = ads1115.readADC_SingleEnded(3); // Read A3
        Serial << "J4-: " << adc2_2 << "(" << adc2_2 * ADS1115_GAIN_MULT << "mV)" << endl;
        adc2_diff = ads1115.readADC_Differential_0_1(); // Read A0/A1 differential
        Serial << "J1 differential: " << adc2_diff << "(" << adc2_diff * ADS1115_GAIN_MULT << "mV)" << endl;

        // Need to add oversampling code here, once it's figured out

        // close the file:
        datafile.close();
      } else {
        // if the file didn't open, print an error:
        Serial << "error writing to file" << endl;
      }

      // Blink LED
      digitalWrite(ledPin, HIGH);
      delay(1000); 
      digitalWrite(ledPin, LOW);

      LowPower.sleep(5000); // sleep for 5 seconds. This will break the Serial, but will still log
    }
  }
}
