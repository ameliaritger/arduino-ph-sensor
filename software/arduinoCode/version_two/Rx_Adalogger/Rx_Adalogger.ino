// SAMPLE OVER TIME with wireless functionality
// First compiled by Amelia Ritger March 24, 2023
// Last edited
// Maybe make a function for READ ADC section?
// Remove any Serial printing for final publication.
// Remove alarmTrigger and add sleep functionality, maybe move Alarm2 up to setup out of loop since it is a one time thing?
// the slave or the receiver

#include <SPI.h>
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
#define DS3231_I2C_ADDRESS 0x68 // Needed when we cut VCC line from RTC, see https://github.com/EKMallon/Utilities/blob/master/setTme/setTme.ino
#define DS3231_CONTROL_REG 0x0E

SdFat sd;                 // Construct SD card
File file;                // Construct File for SD card
Adafruit_ADS1015 ads1015; // Construct Ads1015
Adafruit_ADS1115 ads1115; // Construct Ads1115
#define ADS1X15_REG_CONFIG_MODE_SINGLE (0x0100)  // Power down ADCs in single-shot mode

// Define pins
const uint8_t ledPin = 13; // Adalogger internal LED pin
const uint8_t chipSelect = 4; // Adalogger microSD card chip select pin
const uint8_t reedPin = 12; // Reed switch signals on this pin
const uint8_t cePin = 6; // NRF24L01 CE pin
const uint8_t csnPin = 10; // NRF24L01 CSN pin
const uint8_t intPin = 5; // RTC provides an alarm signal on this pin

// Set up Radio
const byte rxAddress[5] = {'R', 'x', 'A', 'A', 'A'};
const byte txAddress[5] = {'T', 'X', 'a', 'a', 'a'};
RF24 radio(cePin, csnPin);
char dataReceived[10]; // type must match dataToSend on the Pro Mini
byte replyData[32] = {0}; // the values to be sent to the Pro Mini; 32 bytes max for NRF24L01

// Initialize RTC variables
char timestamp[32]; // Current time from the RTC in text format, 32 bytes long
bool alarmTrigger = true; // Create variable set to false for alarm nesting. Set to true if you want immediate start time.

// Initialize other variables
int ledState = LOW; // ledState used to set the LED
bool newData = false;
bool newSdData = true;
bool sendingSD = false;
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
unsigned long previousMillisTx = 0;
unsigned long previousMillisReed = 0;
unsigned long previousMillisBlink = 0;
unsigned long txInterval = 10; // transmit data once per X ms
unsigned long blinkInterval = 100;
unsigned long powerInterval = 30000000; //interval for power to be supplied to NRF24L01 (in ms)

//=====================

void SDfileDate(uint16_t* date, uint16_t* time) { // print a timestamp ("last modified") callback to the SD card
  setSyncProvider(rtc.get);
  *date = FAT_DATE(year(), month(), day());
  *time = FAT_TIME(hour(), minute(), second());
}

//=====================

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

//======================================================================

void setup() {
  Wire.begin(); // Start I2C interface

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  pinMode(ledPin, OUTPUT); // initialize LED pin as an output
  digitalWrite(ledPin, ledState); // turn off LED
  pinMode(reedPin, INPUT_PULLUP); // initialize pullup resistor on Reed switch pin
  digitalWrite(reedPin, HIGH); // Set Reed switch pin to HIGH

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
  clearClockTrigger();
  enableRTCAlarmsonBackupBattery();
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

  Serial << "Powering up the radio" << endl;
  //Setup Radio
  radio.begin();
  Serial << "Powering down radio" << endl;
  radio.powerDown(); // immediately power down the radio until reed switch trigger

  //LowPower.sleep(); // go to sleep until Alarm is triggered. This will break Serial communications, but the loop will still be running.
}

//================

void loop() {
  reedState = digitalRead(reedPin);
  if (reedState == LOW) {
    reedActivated();
  }
  if (triggerState) {
    radioTriggered();
  } else { // if radio is not on, then collect data
    if (!digitalRead(intPin)) { // check to see if the INT/SQW pin is low, i.e. an alarm has occurred
      if (Serial) {
        Serial << "starting data collection..." << endl;
      }
      if (!radioPowerState) { // only collect new data if NRF24L01 is powered down
        collectData();
      }
    }
  }
}

//======================================================================

void blinkWithoutDelay() {
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

//================

void checkPowerDown() {
  unsigned long currentMillisReed = millis(); // get the current time
  if (currentMillisReed - previousMillisReed >= powerInterval) { // if the radio has been powered on and transmitting for longer than the interval
    previousMillisReed = currentMillisReed; // update previousMillisValue
    file.close(); // make sure file is closed before powering down (prevent bricking card)
    while (sd.card() -> isBusy()) {
      ;
    }
    triggerState = false;
    radioPowerState = false;
    radio.powerDown();  // powerDown the radio
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
    blinkWithoutDelay(); // Blink LED to indicate data is being collected
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
    digitalWrite(ledPin, LOW); // turn off LED
  }
  if (Serial) { // end the Serial connection if port is available
    Serial << "Going to sleep..." << endl;
    //Serial.flush();
    //Serial.end();
  }
  //LowPower.sleep(); // go to sleep until next Alarm is triggered. This will break Serial communications, but the loop will still be running.
}

//======================================================================

void extInterrupt(int intPin) {
  pinMode(intPin, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(intPin), wakeUp, CHANGE);
}

//======================================================================

void getData() {
  radio.startListening();
  if (radio.available()) {
    radio.read(&dataReceived, sizeof(dataReceived));
    if (isalpha(dataReceived[0])) { // if Tx sent an alpha value
      send();
    } else if (isdigit(dataReceived[0])) { // if Tx sent a digit value
      blinkInterval = atoi(dataReceived); // set blinkInterval to received value
      sendConfirm(); // send confirmation message
    }
    newData = true;
  }
}

//======================================================================

void radioTriggered() {
  if (sendingSD) { // if we're already actively sending SD card data
    sendSdData();
  } else {
    getData();
    showData();
  }
  checkPowerDown();
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

void reedActivated() {
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

//====================

void send() {
  radio.stopListening();
  bool rslt;
  if (newSdData) { // if we still have SD card data to send
    sendingSD = true;
    file.open(FILE_NAME, FILE_READ); // open the file
    if (file) { // if the file opens
      int bytesRead = file.readBytes(replyData, sizeof(replyData) - 1); // -1 to remove null terminator
      if (bytesRead > 0) {
        rslt = radio.write(&replyData, sizeof(replyData));
      } else { // end of file is reached
        file.close();
        while (sd.card() -> isBusy()) {
          ;
        }
        Serial.println("end of file");
        newSdData = false; // indicate there's no more SD data to send
        sendingSD = false; // indicate we're no longer sending SD data
        byte noMoreReplyData[32] = "NO MORE DATA TO REPORT";
        rslt = radio.write(&noMoreReplyData, sizeof(noMoreReplyData));
      }
    }
  } else {
    byte noNewReplyData[32] = "NO NEW DATA TO REPORT";
    rslt = radio.write(&noNewReplyData, sizeof(noNewReplyData));
  }
  radio.startListening(); // check to acknowledge data transfer worked
  if (rslt) {
    Serial.println("  Acknowledge data received");
  } else {
    Serial.println("  Data transfer failed");
  }
}

//================

void sendConfirm() { // send the data received back, to confirm the correct value
  radio.stopListening();
  bool rslt;
  byte confirmData[32];
  for (int i = 0; i < sizeof(dataReceived);  i++) { // iterate to create a 32 byte array from dataReceived
    confirmData[i] = dataReceived[i];
  }
  rslt = radio.write(&confirmData, sizeof(confirmData));
  radio.startListening(); // check to acknowledge data transfer worked
  if (rslt) {
    Serial.println("  Acknowledge data received");
  } else {
    Serial.println("  Data transfer failed");
  }
}

//====================

void sendSdData() {
  unsigned long currentMillisTx = millis();
  if (currentMillisTx - previousMillisTx >= txInterval) {
    send();
    previousMillisTx = millis();
  }
}

//======================================================================

void setupRadio() {
  radio.begin();
  radio.setDataRate(RF24_250KBPS); // slower data rate = higher resistance to noise
  radio.setPALevel(RF24_PA_LOW);
  radio.setCRCLength(RF24_CRC_16); // set CRC length to 16-bit to ensure data quality
  radio.openWritingPipe(txAddress); // NB these are swapped compared to the master
  radio.openReadingPipe(1, rxAddress);
  radio.setRetries(5, 15); // delay(default is 5, max is 15), count(default/max is 15);
}

//======================================================================

void showData() {
  if (!sendingSD) {// if we're not actively transmitting SD card data
    if (newData) {
      if (Serial) {
        Serial << "Data received: " <<  dataReceived << endl;
      }
      newData = false;
    }
  }
}

//======================================================================

void wakeUp() { // Alarm has been triggered
}

//======================================================================

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
