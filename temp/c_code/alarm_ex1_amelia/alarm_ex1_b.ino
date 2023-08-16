
#include <Time.h>           // https://github.com/PaulStoffregen/Time
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
#include <Streaming.h>      // https://github.com/janelia-arduino/Streaming
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "CONFIG.h"
#include "SdFat.h"          // https://github.com/greiman/SDFat
#include "SRC.h"

DS3232RTC myRTC;
File datafile;

char timestamp[32]; // current time from the RTC in text format, 32 bytes long
bool ALARM_TRIGGER = false; // create variable set to FALSE for alarm nesting

void dateTime(uint16_t* date, uint16_t* time) { // set RTC date and time timestamp to sd card
  DateTime now = myRTC.get();
  *date = FAT_DATE(now.year(), now.month(), now.day()); // return date using FAT_DATE macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second()); // return time using FAT_TIME macro to format fields
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); 
  Serial.begin(115200);     // initialize digital pin 13 as an output (blink)
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
  myRTC.begin();
  myRTC.setAlarm(DS3232RTC::ALM1_MATCH_DATE, 0, 0, 0, 1);
  myRTC.setAlarm(DS3232RTC::ALM2_MATCH_DATE, 0, 0, 0, 1);
  myRTC.alarm(DS3232RTC::ALARM_1);
  myRTC.alarm(DS3232RTC::ALARM_2);
  myRTC.alarmInterrupt(DS3232RTC::ALARM_1, false);
  myRTC.alarmInterrupt(DS3232RTC::ALARM_2, false);
  myRTC.squareWave(DS3232RTC::SQWAVE_NONE);

  myRTC.set(compileTime()); // set the RTC time and date to the compile time (see SRC.h)
  
  time_t t = myRTC.get(); // get the current time
  time_t alarmTime = t + ALARM_INTERVAL;    // calculate the alarm time by adding current RTC time with alarm interval
  formatTime(timestamp, t); // reformat time (see SRC.h)
  Serial << F("Current RTC date and time ") << timestamp << endl;   // print the current time to the serial port

  // set the Alarms
  myRTC.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0); // set Alarm 1 to occur at sampling interval
  myRTC.alarm(DS3232RTC::ALARM_1); // clear the alarm flag
  myRTC.setAlarm(DS3232RTC::ALM2_MATCH_DATE, 0, START_MIN, START_HOUR, START_DAY); // set Alarm 2 to occur at specific date and time
  myRTC.alarm(DS3232RTC::ALARM_2); // clear the alarm flag
}

void loop()
{ 
  if (myRTC.alarm(DS3232RTC::ALARM_2) ) {    // check alarm flag, clear it if set
    ALARM_TRIGGER = true; // set alarm trigger to TRUE to trigger Alarm 1
    time_t t = myRTC.get();
    formatTime(timestamp, t);
    Serial << "ALARM_2 " << timestamp << endl; // print the time when this part of the loop is running
    }
    
    if (ALARM_TRIGGER && myRTC.alarm(DS3232RTC::ALARM_1)) { // check alarm flag (and clear the flag if set)
      time_t t = myRTC.get(); // get the current time
      formatTime(timestamp, t);
      time_t alarmTime = t + ALARM_INTERVAL; // calculate the next alarm time
      myRTC.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0); // set the alarm
      Serial << " ALARM_1 " << timestamp << endl; // print the time when this part of the loop is running
    }
}
