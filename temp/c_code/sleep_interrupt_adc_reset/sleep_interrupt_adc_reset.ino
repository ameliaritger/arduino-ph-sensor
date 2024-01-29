#include <SPI.h>
//#include <SD.h>
#include <Wire.h>
#include "ADS1X15.h"

#include "ArduinoLowPower.h"
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
#include <Streaming.h>      // https://github.com/janelia-arduino/Streaming , allows you to use << formatting
#include <Time.h>           // https://github.com/PaulStoffregen/Time
//#include <LowPower.h>       // https://github.com/rocketscream/Low-Power

#include "CONFIG.h"
#include "SRC.h"

DS3232RTC rtc;            // Construct RTC
ADS1115 ADS1(0x49);
ADS1015 ADS0(0x48);

const int ledPin = LED_BUILTIN;
int ledState = LOW; // ledState used to set the LED
char timestamp[32]; // Current time from the RTC in text format, 32 bytes long
bool alarmTrigger = true; // Create variable set to false for alarm nesting. Set to true if you want immediate start time.
const byte intPin = 5; // RTC provides an alarm signal on this pin
int readPin1 = A1; //J1+
int Val1 = 0;
float Volt1 = 0;

#define DS3231_I2C_ADDRESS 0x68 // Needed when we cut VCC line from RTC, see https://github.com/EKMallon/Utilities/blob/master/setTme/setTme.ino
#define DS3231_CONTROL_REG 0x0E
const int ADS1x15_I2C_GENERAL = 0x00;
const byte ADS1x15_I2C_RESET = 0x06;
int16_t raw = 0;
float raw_volt = 0;


void setup() {
  //delay(1000); //delay so we can see normal current draw
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin 13 as an output (blink)
  digitalWrite(LED_BUILTIN, LOW); // turn off LED
  pinMode(readPin1, INPUT);

  Serial.begin(9600);
  Wire.begin(); // Start I2C interface
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Setup SD card
  Serial << "Initializing rtc..." << endl;

  rtc.begin();
  clearClockTrigger();
  enableRTCAlarmsonBackupBattery();
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
  //Serial.flush();
  //Serial.end();
}

void loop() {
  if (!digitalRead(intPin)) { // check to see if the INT/SQW pin is low, i.e. an alarm has occurred
    //Serial << "starting data collection..." << endl;
    collectData ();
  }

  //Serial << "Going to sleep..." << endl;
  //Serial.flush();
  //Serial.end();

  //UNCOMMENT THE FOLLOWING LINES WHILE DEBUGGING
  //USBDevice.detach(); //safely detach USB prior to sleeping
  //LowPower.deepSleep();
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

    ADS0.begin();
    ADS1.begin();
    delay(10);
    if (ADS0.isConnected()) {
      ;
    }
    if (ADS1.isConnected()) {
      ;
    }
    ADS0.setGain(1);          //  6.144 volt
    ADS0.setDataRate(4);
    ADS1.setGain(1);          //  6.144 volt
    ADS1.setDataRate(4);

    raw = ADS0.readADC(2);
    raw_volt = ADS0.toVoltage(raw);
    Serial.print("\t");
    Serial.print(raw_volt);
    Serial.print("\t");
    Serial.print(raw*2);
    Serial.print("\t");
    raw = ADS1.readADC(2);
    raw_volt = ADS1.toVoltage(raw);
    Serial.print(raw_volt);
    Serial.print("\t");
    Serial.print(raw*0.125);
    Serial.print("\t");

    adcReset();

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

void clearClockTrigger() {  // from http://forum.arduino.cc/index.php?topic=109062.0
  Wire.beginTransmission(DS3231_I2C_ADDRESS);   //Tell devices on the bus we are talking to the DS3231
  Wire.write(0x0F);               //Tell the device which address we want to read or write
  Wire.endTransmission();         //Before you can write to and clear the alarm flag you have to read the flag first!
  Wire.requestFrom(DS3231_I2C_ADDRESS, 1);      //Read one byte
  Wire.read();
  Wire.beginTransmission(DS3231_I2C_ADDRESS);   //Tell devices on the bus we are talking to the DS3231
  Wire.write(0x0F);               //Status Register: Bit 3: zero disables 32kHz, Bit 7: zero enables the main oscilator
  Wire.write(0b00000000);         //Bit1: zero clears Alarm 2 Flag (A2F), Bit 0: zero clears Alarm 1 Flag (A1F)
  Wire.endTransmission();
  //clockInterrupt = false;         //Finally clear the flag we used to indicate the trigger occurred
}

void enableRTCAlarmsonBackupBattery() {
  Wire.beginTransmission(DS3231_I2C_ADDRESS);      // Attention device at RTC address 0x68
  Wire.write(DS3231_CONTROL_REG);                  // move the memory pointer to CONTROL_REGister
  Wire.endTransmission();                          // complete the ‘move memory pointer’ transaction
  Wire.requestFrom(DS3231_I2C_ADDRESS, 1);         // request data from register
  byte resisterData = Wire.read();                 // byte from registerAddress
  bitSet(resisterData, 1);                         // Change bit 1 to a 6 to disable
  Wire.beginTransmission(DS3231_I2C_ADDRESS);      // Attention device at RTC address 0x68
  Wire.write(DS3231_CONTROL_REG);                  // target the CONTROL_REGister
  Wire.write(resisterData);                        // put changed byte back into CONTROL_REG
  Wire.endTransmission();
}

void adcReset() {
  Wire.beginTransmission(ADS1x15_I2C_GENERAL); //start I2C transmission
  Wire.write(ADS1x15_I2C_RESET); // Send reset command byte
  Wire.endTransmission(); //end I2C transmission
}