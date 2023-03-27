// SAMPLE OVER TIME
// First compiled by Amelia Ritger March 24, 2023

#include <Time.h>           // https://github.com/PaulStoffregen/Time
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
#include <Streaming.h>      // https://github.com/janelia-arduino/Streaming
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "CONFIG.h"
#include "SdFat.h"          // https://github.com/greiman/SDFat
#include "SRC.h"
#include <Adafruit_ADS1X15.h>

DS3232RTC rtc;
File datafile;
Adafruit_ADS1015 ads1015; // Construct an ads1015
Adafruit_ADS1115 ads1115; // Construct an ads1115 

const int ledPin = LED_BUILTIN;
int ledState = LOW; // ledState used to set the LED
char timestamp[32]; // current time from the RTC in text format, 32 bytes long
bool ALARM_TRIGGER = false; // create variable set to FALSE for alarm nesting
float average_samples_one; // here and below, set variables for oversampling
int over_samples_one;
float sum_samples_one;
float average_samples_plus;
int over_samples_plus;
float sum_samples_plus;
float average_samples_minus;
int over_samples_minus;
float sum_samples_minus;

void dateTime(uint16_t* date, uint16_t* time) { // set RTC date and time timestamp to sd card
  DateTime now = rtc.get();
  *date = FAT_DATE(now.year(), now.month(), now.day()); // return date using FAT_DATE macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second()); // return time using FAT_TIME macro to format fields
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin 13 as an output (blink)
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Setup SD card
  Serial.print("Initializing SD card...");
  
  if (!SD.begin(4)) {
    Serial.println("sd card initialization failed!");
    while (1);
  }
  
  Serial.println("sd card initialization done.");
  
  if (SD.exists(FILE_NAME)) { // If file already exists, don't add a header. If it doesn't exist, then write a header.
    Serial.println("File exists, opening file...");
  } else {
    Serial.println("File doesn't exist, creating file and adding header");
    Serial.println("Opening file...");
    datafile = SD.open(FILE_NAME, FILE_WRITE);
    Serial.println("Writing header...");
    datafile.println("Date,Time,J1 diff (mV),J1 oversampled (mV),J2+ (mV),J3 diff (mV),J4 oversampled (mV), J4 (mV), J4+ (mV),J4- (mV), J4+ oversampled (mV),J4- oversampled (mV),Coin battery (mV)");
    datafile.close();
    Serial.println("done.");
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

  //Setup ADC 1015 and 1115
  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 4.096V (1 bit = 2mV)");
  ads1015.setGain(ADS1015_GAIN_VAL); // set Gain for ADC 1015
  ads1015.begin(0x48); // Initialize ads1015 at the default address 0x48
   
  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 1.024V (1 bit = 0.03125mV)");
  ads1115.setGain(ADS1115_GAIN_VAL); // set Gain for ADC 1115
  ads1115.begin(0x49); // Initialize ads1115 at address 0x49
}

void loop()
{ 
  if (rtc.alarm(DS3232RTC::ALARM_2) ) {    // check alarm flag, clear it if set
    ALARM_TRIGGER = true; // set alarm trigger to TRUE to trigger Alarm 1
    time_t t = rtc.get();
    formatTime(timestamp, t);
    Serial << "ALARM_2 " << timestamp << endl; // print the time when this part of the loop is running
    }
    
    if (ALARM_TRIGGER && rtc.alarm(DS3232RTC::ALARM_1)) { // check alarm flag (and clear the flag if set)
      time_t t = rtc.get(); // get the current time
      formatTime(timestamp, t);
      time_t alarmTime = t + ALARM_INTERVAL; // calculate the next alarm time
      rtc.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0); // set the alarm
      Serial << " ALARM_1 " << timestamp << endl; // print the time when this part of the loop is running
     
      //SdFile::dateTimeCallback(dateTime); // Set file date and time on sd card ("last modified")
      //datafile = SD.open(FILE_NAME, FILE_WRITE); // open the file. note that only one file can be open at a time, so you have to close this one before opening another.
      
      //if (datafile){ // if the file opened okay, write to it:
      //  Serial.println("Writing to SD Card...");
        //datafile << timestamp << endl; // print date and time to datafile
    //date
    //tmElements_t tm; // check out SRC.h, may also be able to do time_t t and then day(t)
    //rtc.read(tm);
    //time_t tm;
    //datafile.print(day(t));
    //datafile.print(tm.Year(), DEC);
    //datafile.print('/');
    //datafile.print(tm.Month(), DEC);
    //datafile.print('/');
    //datafile.print(tm.Day(), DEC);
    //datafile.print(',');
    //time
    //datafile.print(tm.Hour(), DEC);
    //datafile.print(':');
    //datafile.print(tm.Minute(), DEC);
    //datafile.print(':');
   // datafile.print(tm.Second(), DEC);
    //datafile.print(',');
    
    // close the file:
    //datafile.close();
  //} //else {
    // if the file didn't open, print an error:
    //Serial.println("error writing to file");
  //}
  
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
    }
}
