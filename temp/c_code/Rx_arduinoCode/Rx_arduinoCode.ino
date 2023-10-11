// SAMPLE OVER TIME with wireless functionality
// First compiled by Amelia Ritger March 24, 2023
// Last edited
// Maybe make a function for READ ADC section?
// Remove any Serial printing for final publication.
// Remove alarmTrigger and add sleep functionality, maybe move Alarm2 up to setup out of loop since it is a one time thing?
// the slave or the receiver

#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#include <Adafruit_ADS1X15.h>
#include "ArduinoLowPower.h"
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
#include <Streaming.h>      // https://github.com/janelia-arduino/Streaming , allows you to use << formatting
#include <Time.h>           // https://github.com/PaulStoffregen/Time
#include "SdFat.h"          // https://github.com/greiman/SDFat
#include <nRF24L01.h>       // https://nrf24.github.io/RF24/
#include <RF24.h>

#include "CONFIG.h"
#include "SRC.h"

DS3232RTC rtc;            // Construct RTC
SdFat sd;                 // Construct SD card
File file;                // Construct File for SD card
Adafruit_ADS1015 ads1015; // Construct Ads1015
Adafruit_ADS1115 ads1115; // Construct Ads1115

// Define pins
const uint8_t ledPin = 13; // Adalogger internal LED pin
const uint8_t chipSelect = 4; // Adalogger microSD card chip select pin
const uint8_t reedPin = 12; // Reed switch signals on this pin
const uint8_t cePin = 6; // NRF24L01 CE pin
const uint8_t csnPin = 10; // NRF24L01 CSN pin
const uint8_t intPin = 10; // RTC provides an alarm signal on this pin

// Set up Radio
const byte radioAddress[5] = {'R', 'x', 'A', 'A', 'A'};
RF24 radio(cePin, csnPin);
unsigned int dataReceived; // this must match dataToSend in Tx SHOULD THIS BE UNSIGNED OR 16T?????
byte ackData[32] = {0}; // the SD card file values to be sent to Tx; 32 bytes max for NRF24L01

// Initialize RTC variables
char timestamp[32]; // Current time from the RTC in text format, 32 bytes long
bool alarmTrigger = true; // Create variable set to false for alarm nesting. Set to true if you want immediate start time.

// Initialize other variables
int ledState = LOW; // ledState used to set the LED
bool newData = false;
bool newSdData = true;
bool triggerState = false;
bool radioPowerState = false;
int reedState;

// Initialize ADC variables
int16_t adc2_1, adc2_2, adc2_diff, adc1_1, adc1_diff; //adc1_2 for coin batt
int oversampleArray[OVERSAMPLE_VALUE]; // generate array for oversampling
int oversampleMin, oversampleMax;
float oversampleMean;
double oversampleSD;

// Initialize millis variables
unsigned long previousMillisRadio = 0;
unsigned long previousMillisReed = 0;
unsigned long previousMillisBlink = 0;
unsigned long blinkInterval = 1000;
unsigned long transmitInterval = 20000; //interval for power to be supplied to NRF24L01 (in ms)

//======================================================================

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  pinMode(ledPin, OUTPUT); // initialize LED pin as an output
  digitalWrite(ledPin, ledState); // turn off LED
  pinMode(reedPin, INPUT_PULLUP); // initialize pullup resistor on Reed switch pin
  digitalWrite(reedPin, HIGH); // Set Reed switch pin to HIGH
  dataReceived = 1000;

  // Setup SD card
  SPI.begin();
  Serial << "Initializing SD card..." << endl;
  if (!sd.begin(chipSelect, SD_SCK_MHZ(12))) {
    Serial << "sd card initialization failed!" << endl;
    sd.initErrorHalt(); // not being able to save data is a terminal error in all cases
  } else {
    Serial << "sd card initialization done." << endl;
  }

  file.open(FILE_NAME, FILE_WRITE);
  if (file) { // If file already exists, don't add a header. If it doesn't exist, then write a header.
    Serial << "File exists, opening file..." << endl;
  } else {
    Serial << "File doesn't exist, creating file and adding header" << endl;
    file << "Date,Time,J1 (mV),J1 oversampled mean (mV),J1 oversampled mean,J1 oversampled min,J1 oversampled max,J1 oversampled sd,J2+ (mV),J3 (mV),J4+ (mV),J4+ oversampled mean (mV),J4+ oversampled mean,J4+ oversampled min,J4+ oversampled max,J4+ oversampled sd,J4- (mV),J4- oversampled mean (mV),J4- oversampled mean,J4- oversampled min,J4- oversampled max,J4- oversampled sd" << endl; // removed "Coin battery (mV)"
  }
  file.close(); // close the file
  while (sd.card() -> isBusy()) {
    ;
  }
  Serial << "Done, file closed." << endl;

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
  rtc.alarmInterrupt(DS3232RTC::ALARM_1, true);
  rtc.setAlarm(DS3232RTC::ALM2_MATCH_DATE, 0, START_MIN, START_HOUR, START_DAY); // set Alarm 2 to occur at specific date and time
  rtc.alarm(DS3232RTC::ALARM_2); // clear the alarm flag
  rtc.alarmInterrupt(DS3232RTC::ALARM_2, true);
  extInterrupt(intPin); //create interrupt source on Interrupt Pin

  //Setup ADC 1015 and 1115
  Serial << "Getting differential reading from AIN0 (P) and AIN1 (N)" << endl;
  Serial << "ADC Range: +/- 0.512V (1 bit = 2mV)" << endl;
  ads1015.setGain(ADS1015_GAIN_VAL); // set Gain for ADC 1015
  ads1015.begin(0x48); // Initialize ads1015 at the default address 0x48
  Serial << "Getting differential reading from AIN0 (P) and AIN1 (N)" << endl;
  Serial << "ADC Range: +/- 1.024V (1 bit = 0.03125mV)" << endl;
  ads1115.setGain(ADS1115_GAIN_VAL); // set Gain for ADC 1115
  ads1115.begin(0x49); // Initialize ads1115 at address 0x49

  //Setup Radio
  radio.begin();
  radio.enableAckPayload(); // DO I NEED TO HAVE THIS, YEAH?
  loadFileData(); // pre-load Ack Payload
  radio.powerDown(); // immediately power down the radio until reed switch trigger

  LowPower.sleep(); // go to sleep until Alarm is triggered. This will break Serial communications, but the loop will still be running.
}

//================

void loop() {
  reedState = digitalRead(reedPin);
  if (reedState == LOW) {
    reedTriggered();
  }
  if (triggerState == true ) {
    radioTriggered();
  }
  if (!digitalRead(intPin)) { // check to see if the INT/SQW pin is low, i.e. an alarm has occurred
    if (Serial) {
      Serial << "starting data collection..." << endl;
    }
    collectData();
  }
}

//======================================================================

void blinkWithoutDelay(); {
  unsigned long currentMillisBlink = millis();
  if (currentMillisBlink - previousMillisBlink >= blinkInterval) {
    previousMillisBlink = currentMillisBlink; // update previousMillis value
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);
  }
}

//======================================================================

void collectData() {
  if (rtc.alarm(DS3232RTC::ALARM_2) ) {    // check alarm flag, clear it if set
    alarmTrigger = true; // set alarm trigger to TRUE to trigger Alarm 1
    time_t t = rtc.get();
    formatTime(timestamp, t);
    if (Serial) { //
      Serial << "ALARM_2 " << timestamp << endl; // print the time when this part of the loop is running
    }
  }

  if (alarmTrigger && rtc.alarm(DS3232RTC::ALARM_1)) { // check alarm flag (and clear the flag if set)
    time_t t = rtc.get(); // get the current time
    formatTime(timestamp, t);
    time_t alarmTime = t + ALARM_INTERVAL; // calculate the next alarm time
    rtc.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0); // set the alarm
    if (Serial) {
      Serial << "IT'S TIME TO SAMPLE! " << timestamp << endl; // print the time when this part of the loop is running
    }
    SdFile::dateTimeCallback(SDfileDate); // Set file date and time on sd card ("last modified"). Check out this page if having issues: https://arduino.stackexchange.com/questions/39126/how-does-one-set-attributes-for-sd-files
    file.open(FILE_NAME, FILE_WRITE); // open the file. note that only one file can be open at a time, so you have to close this one before opening another.

    if (file) { // if the file opened okay, write to it:
      if (Serial) {
        Serial << "Writing to SD Card..." << endl;
      }
      time_t TIME = rtc.get();
      file << month(TIME) << "/" << day(TIME) << "/" << year(TIME) << "," << hour(TIME) << ":" << minute(TIME) << ":" << second(TIME) << "," ;
      readADC();
      // close the file:
      file.close();
      while (sd.card() -> isBusy()) {
        ;
      }
    } else {
      if (Serial) {
        Serial << "error writing to file" << endl; // if the file didn't open, print an error:
      }
    }
    blinkWithoutDelay();     // Blink LED to indicate data is collected and SD card is closed
  }
  if (Serial) { // end the Serial connection if port is available
    Serial << "Going to sleep..." << endl;
    Serial.flush();
    Serial.end();
  }
  LowPower.sleep(); // go to sleep until next Alarm is triggered. This will break Serial communications, but the loop will still be running.
}

//======================================================================

void extInterrupt(int intPin) {
  pinMode(intPin, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(intPin), wakeUp, CHANGE);
}

//======================================================================

void getData() {
  if (radio.available()) {
    radio.read(&dataReceived, sizeof(dataReceived));
    loadFileData();
    newData = true;
  }
}

//======================================================================

void loadFileData() {
  if (newSdData) {
    file.open(FILE_NAME, FILE_READ);
    if (file) {
      int bytesRead = file.readBytes(ackData, sizeof(ackData) - 1); // -1 to remove null terminator
      if (bytesRead > 0) {
        radio.writeAckPayload(1, &ackData, sizeof(ackData)); // load the payload for the next time
      } else { // end of file is reached
        file.close();
        while (sd.card() -> isBusy()) {
          ;
        }
        if (Serial) {
          Serial << "end of file" << endl;
        }
        newSdData = false;
        for (int i = 0; i < sizeof(ackData) / sizeof(ackData[0]); i++) {
          ackData[i] = 0; //replace ackData with 0s
        }
      }

    } else {
      if (Serial) {
        Serial << "no new data to report!" << endl;
      }
      radio.writeAckPayload(1, &ackData, sizeof(ackData)); // load blank message payload
    }
  }
}

//======================================================================

void radioTriggered() {
  getData();
  showData();
  unsigned long currentMillisReed = millis(); // get the current time
  if (currentMillisReed - previousMillisReed > transmitInterval) { // if the radio has been powered on and transmitting for longer than the transmit interval
    digitalWrite(ledPin , LOW); // turn off the LED
    triggerState = false;
    radioPowerState = false;
    radio.powerDown();  // powerDown the radio
  }
}

//======================================================================

void readADC() { ///*****Read ADC Inputs
  adc2_diff = ads1115.readADC_Differential_0_1(); // Read J1 pin (A0/A1 differential)
  oversample(&ads1115, oversampleArray, 0);
  oversampleMean = sampleMean(oversampleArray);
  oversampleMin = sampleMin(oversampleArray);
  oversampleMax = sampleMax(oversampleArray);
  oversampleSD = sampleSD(oversampleArray);
  if (Serial) {
    Serial << "J1: " << oversampleMean << ", " << oversampleMean * ADS1115_GAIN_MULT << "mV" << endl;
  }
  file << adc2_diff * ADS1115_GAIN_MULT << "," << oversampleMean * ADS1115_GAIN_MULT << "," << oversampleMean << "," << oversampleMin << "," << oversampleMax << "," << oversampleSD << ","; //J1

  adc1_1 = ads1015.readADC_SingleEnded(2); // Read J2 pin (A2)
  if (Serial) {
    Serial << "J2: " << adc1_1 << "(" << adc1_1 * ADS1015_GAIN_MULT << "mV)" << endl;
  }
  file << adc1_1 * ADS1015_GAIN_MULT << ","; //J2

  adc1_diff = ads1015.readADC_Differential_0_1(); //Read J3 pin (A0/A1 differential)
  if (Serial) {
    Serial << "J3: " << adc1_diff << "(" << adc1_diff * ADS1015_GAIN_MULT << "mV" << endl;
  }
  file << adc1_diff * ADS1015_GAIN_MULT << ","; //J3

  adc2_1 = ads1115.readADC_SingleEnded(2); // Read J4+ pin (A2)
  oversample(&ads1115, oversampleArray, 2);
  oversampleMean = sampleMean(oversampleArray);
  oversampleMin = sampleMin(oversampleArray);
  oversampleMax = sampleMax(oversampleArray);
  oversampleSD = sampleSD(oversampleArray);
  if (Serial) {
    Serial << "J4+: " << oversampleMean << ", " << oversampleMean * ADS1115_GAIN_MULT << "mV" << endl;
  }
  file << adc2_1 * ADS1115_GAIN_MULT << "," << oversampleMean * ADS1115_GAIN_MULT << "," << oversampleMean << "," << oversampleMin << "," << oversampleMax << "," << oversampleSD << ","; //J4+

  adc2_2 = ads1115.readADC_SingleEnded(3); // Read J4- pin (A3)
  oversample(&ads1115, oversampleArray, 3);
  oversampleMean = sampleMean(oversampleArray);
  oversampleMin = sampleMin(oversampleArray);
  oversampleMax = sampleMax(oversampleArray);
  oversampleSD = sampleSD(oversampleArray);
  if (Serial) {
    Serial << "J4-: " << oversampleMean << ", " << oversampleMean * ADS1115_GAIN_MULT << "mV" << endl;
  }
  file << adc2_2 * ADS1115_GAIN_MULT << "," << oversampleMean * ADS1115_GAIN_MULT << "," << oversampleMean << "," << oversampleMin << "," << oversampleMax << "," << oversampleSD << endl; //J4-
}

//======================================================================

void reedTriggered() {
  if (Serial) {
    Serial << "switch activated!" << endl;
  }
  if (!radioPowerState) { //if this is the first time the Radio is turned on
    radio.powerUp();
    delay(10); // RF24 documentation: "this will take up to 5ms for maximum compatibility"
    setupRadio();
    radioPowerState = true; //variable to only powerUp the radio once
    triggerState = true; // variable to indicate the Reed switch has been triggered
  }
  previousMillisReed = millis(); //store the time the Reed switch was triggered
}

//======================================================================

void setupRadio() {
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  //radio.setPALevel(RF24_PA_LOW); // prevent power supply related problems (RF24_PA_MAX is default)
  radio.enableAckPayload();
  radio.openReadingPipe(1, radioAddress);
  radio.startListening();
}

//======================================================================

void showData() {
  if (newData == true) {
    if (Serial) {
      Serial << "Data received: " <<  dataReceived << endl;
      Serial << "ackPayload sent: " << endl;
      Serial.println(F(ackData)); // HOW DO I FORMAT THIS WITH <<????
    }
    newData = false;
  }
  unsigned long currentMillisRadio = millis();
  if (currentMillisRadio - previousMillisRadio >= dataReceived) {
    previousMillisRadio = currentMillisRadio; // update previousMillis value
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);
  }
}

//======================================================================

void wakeUp() { // Alarm has been triggered
}

//======================================================================

void SDfileDate(uint16_t* date, uint16_t* time) { // print a timestamp ("last modified") callback to the SD card
  setSyncProvider(rtc.get);
  *date = FAT_DATE(year(), month(), day());
  *time = FAT_TIME(hour(), minute(), second());
}

//======================================================================

void oversample(Adafruit_ADS1115 * ads1115Pointer, int oversampleArray[OVERSAMPLE_VALUE], int input) { // Function to oversample ADS1115 pins
  ads1115 = *ads1115Pointer;
  for (int i = 0; i < OVERSAMPLE_VALUE; i++) {
    if ((input == 2) || (input == 3)) {
      int adcValue = ads1115.readADC_SingleEnded(input);
      oversampleArray[i] = adcValue;
    } else {
      int adcValue = ads1115.readADC_Differential_0_1();
      oversampleArray[i] = adcValue;
    }
  }
}
