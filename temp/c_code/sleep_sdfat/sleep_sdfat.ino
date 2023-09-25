#include <SPI.h>
//#include <SD.h>
#include <Wire.h>

#include "ArduinoLowPower.h"
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
#include <Streaming.h>      // https://github.com/janelia-arduino/Streaming , allows you to use << formatting
#include <Time.h>           // https://github.com/PaulStoffregen/Time
//#include <LowPower.h>       // https://github.com/rocketscream/Low-Power
#include "SdFat.h"          // Adafruit fork of SdFat library by https://github.com/greiman/SDFat

#include "CONFIG.h"
#include "SRC.h"

DS3232RTC rtc;            // Construct RTC
SdFat sd;                 // Create the objects to talk to the SD card
SdFile file;          // Construct File for SD card

const int ledPin = LED_BUILTIN;
int ledState = LOW; // ledState used to set the LED
char timestamp[32]; // Current time from the RTC in text format, 32 bytes long
bool alarmTrigger = true; // Create variable set to false for alarm nesting. Set to true if you want immediate start time.
const uint8_t intPin = 10; // RTC provides an alarm signal on this pin

//#define CLOCK_POWER A2;
const uint8_t chipSelect = 4; // Adalogger microSD card chip select pin
int readPin1 = A1; //J1+
int Val1 = 0;
float Volt1 = 0;

void setup() {
  //delay(1000); //delay so we can see normal current draw
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin 13 as an output (blink)
  digitalWrite(LED_BUILTIN, LOW); // turn off LED
  pinMode(readPin1, INPUT);
  //pinMode(chipSelect, OUTPUT);
  //digitalWrite(chipSelect, HIGH); // pullup SD card CS pin
  //pinMode(23, OUTPUT);
  //digitalWrite(23, HIGH); // pullup MOSI pin
  //pinMode(22, INPUT);
  //digitalWrite(22, HIGH); // pullup MISO pin on SD card

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(1000);

  // Setup SD card
  Serial << "Initializing SD..." << endl;

  // change to SPI_FULL_SPEED for more performance.//change to SPI_HALF_SPEED for old cards -half speed like the native library.
  if (!sd.begin(chipSelect, SD_SCK_MHZ(12))) {
    Serial << "sd card initialization failed!" << endl;
    Serial.flush();
    sd.initErrorHalt(); // not being able to save data is a terminal error in all cases
  }

  file.open("datafile.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file!");
    return;
  }
  file.println("Hello world!");
  file.close();

  Serial << "sd card initialization done." << endl;

  rtc.begin();
  rtc.setAlarm(DS3232RTC::ALM1_MATCH_DATE, 0, 0, 0, 1); //initialize alarms
  rtc.setAlarm(DS3232RTC::ALM2_MATCH_DATE, 0, 0, 0, 1);
  rtc.alarm(DS3232RTC::ALARM_1); // clear alarm flags
  rtc.alarm(DS3232RTC::ALARM_2);
  rtc.alarmInterrupt(DS3232RTC::ALARM_1, false); //clear alarm interrupt flags
  rtc.alarmInterrupt(DS3232RTC::ALARM_2, false);
  rtc.squareWave(DS3232RTC::SQWAVE_NONE);

  rtc.set(compileTime()); // set the RTC time and date to the compile time (see SRC.h)

  time_t t = rtc.get(); // get the current time
  time_t alarmTime = t + ALARM_INTERVAL;    // calculate the alarm time by adding current RTC time with alarm interval
  formatTime(timestamp, t); // reformat time (see SRC.h)
  Serial << F("Current RTC date and time ") << timestamp << endl;   // print the current time to the serial port

  // set the Alarms
  rtc.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0); // set Alarm 1 to occur at sampling interval
  rtc.alarm(DS3232RTC::ALARM_1); // ensure alarm flag is cleared
  rtc.alarmInterrupt(DS3232RTC::ALARM_1, true);
  rtc.setAlarm(DS3232RTC::ALM2_MATCH_DATE, 0, START_MIN, START_HOUR, START_DAY); // set Alarm 2 to occur at specific date and time
  rtc.alarm(DS3232RTC::ALARM_2); // ensure alarm flag is cleared
  rtc.alarmInterrupt(DS3232RTC::ALARM_2, true);

  extInterrupt(intPin); //create interrupt source on Interrupt Pin
}

void loop() {
  if (!digitalRead(intPin)) { // check to see if the INT/SQW pin is low, i.e. an alarm has occurred
    if (Serial) { // print if Serial port is available
      Serial << "starting data collection..." << endl;
    }
    // Power up the clock
    //digitalWrite(CLOCK_POWER, HIGH);
    //pinMode(CLOCK_POWER, OUTPUT);
    collectData ();
  }
  Val1 = analogRead(readPin1);
  Volt1 = (3.3 / 1023.) * Val1 * 1000; //convert read to voltage

  file.open("datafile.txt", FILE_WRITE);
  file.println(Volt1);
  file.close();

  while (sd.card() -> isBusy()) {
    ;
  }

  if (Serial) { // end Serial connection if port is available
    Serial.println(Volt1); //print analog value of DS3231 RTC battery
    Serial << "Going to sleep..." << endl;
    Serial.flush();
    Serial.end();
  }
  // Power down the clock
    //pinMode(CLOCK_POWER, INPUT);  
    //digitalWrite(CLOCK_POWER, LOW);

  //UNCOMMENT THE FOLLOWING LINES WHILE DEBUGGING
  //USBDevice.detach(); //safely detach USB prior to sleeping
  LowPower.sleep();
  //USBDevice.attach(); //re-attach the USB, audible sound on Windows
  //delay(1000); // delay to make serial more reliable
  //Serial.begin(115200);
  //while (!Serial); // wait for serial port to connect.
  //Serial.println("awake!");
}

void collectData() {
  if (rtc.alarm(DS3232RTC::ALARM_2) ) {    // check alarm flag, clear it if triggered
    alarmTrigger = true; // set alarm trigger to TRUE to trigger Alarm 1
    time_t t = rtc.get();
    formatTime(timestamp, t);
    Serial << "ALARM_2 " << timestamp << endl; // print the time when this part of the loop is running
  }

  if (alarmTrigger && rtc.alarm(DS3232RTC::ALARM_1)) { // check alarm flag (and clear the flag if triggered)
    time_t t = rtc.get(); // get the current time
    formatTime(timestamp, t);
    time_t alarmTime = t + ALARM_INTERVAL; // calculate the next alarm time
    rtc.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0); // set the next alarm
    Serial << "IT'S TIME TO SAMPLE! " << timestamp << endl; // print the time when this part of the loop is running

    // Blink LED
    digitalWrite(ledPin, HIGH); // turn LED on
    delay(1000); // keep LED on for 1 second
    digitalWrite(ledPin, LOW); // turn LED off
  }
}

void wakeUp() { // Alarm has been triggered
}

void extInterrupt(int intPin) {
  pinMode(intPin, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(intPin), wakeUp, CHANGE);
}
