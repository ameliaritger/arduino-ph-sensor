// First compiled by Amelia Ritger March 24, 2023
// Maybe make a function for READ ADC section?
// Remove alarmTrigger and add sleep functionality, maybe move Alarm2 up to setup out of loop since it is a one time thing?
// Make a Serial trigger so Serial only prints if TRUE

#include <SPI.h>
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
#define DS3231_I2C_ADDRESS 0x68 // Needed when we cut VCC line from RTC, see https://github.com/EKMallon/Utilities/blob/master/setTme/setTme.ino
#define DS3231_CONTROL_REG 0x0E
SdFat sd;                 // Create the objects to talk to the SD card
SdFile file;              // Construct File for SD card
Adafruit_ADS1015 ads1015; // Construct an ads1015
Adafruit_ADS1115 ads1115; // Construct an ads1115
#define ADS1X15_REG_CONFIG_MODE_SINGLE (0x0100)  // Power down ADCs in single-shot mode

const int ledPin = LED_BUILTIN;
int ledState = LOW; // ledState used to set the LED
char timestamp[32]; // Current time from the RTC in text format, 32 bytes long
bool alarmTrigger = true; // Create variable set to false for alarm nesting. Set to true if you want immediate start time.
const uint8_t intPin = 5; // RTC provides an alarm signal on this pin
const uint8_t chipSelect = 4; // Adalogger microSD card chip select pin

int16_t adc2_1, adc2_2, adc2_diff, adc1_1, adc1_diff; //adc1_2 for coin batt
int oversampleArray[OVERSAMPLE_VALUE]; // generate array for oversampling
int oversampleMin_J1, oversampleMin_J4p, oversampleMin_J4m, oversampleMax_J1, oversampleMax_J4p, oversampleMax_J4m;
float oversampleMean_J1, oversampleMean_J4p, oversampleMean_J4m;
double oversampleSD_J1, oversampleSD_J4p, oversampleSD_J4m;

// Function to print a timestamp ("last modified") callback to the SD card
void SDfileDate(uint16_t* date, uint16_t* time) {
  setSyncProvider(rtc.get);
  *date = FAT_DATE(year(), month(), day());
  *time = FAT_TIME(hour(), minute(), second());
}

// Function to oversample ADS1115 pins
void oversample(Adafruit_ADS1115* ads1115Pointer, int oversampleArray[OVERSAMPLE_VALUE], int input) {
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

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin 13 as an output (blink)
  digitalWrite(LED_BUILTIN, LOW); // turn off LED

  Wire.begin(); // Start I2C interface
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Setup SD card
  Serial << "Initializing SD card..." << endl;

  if (!sd.begin(chipSelect, SD_SCK_MHZ(12))) {
    Serial << "sd card initialization failed!" << endl;
    Serial.flush();
    sd.initErrorHalt(); // not being able to save data is a terminal error in all cases
  }

  Serial << "sd card initialization done." << endl;

  if (sd.exists(FILE_NAME)) { // If file already exists, don't add a header. If it doesn't exist, then write a header.
    Serial << "File exists, opening file..." << endl;
  } else {
    Serial << "File doesn't exist, creating file and adding header" << endl;
    Serial << "Opening file..." << endl;
    file.open(FILE_NAME, FILE_WRITE); // open the file
    Serial << "Writing header..." << endl;
    file << "Date,Time,J1 (mV),J1 oversampled mean (mV),J1 oversampled mean,J1 oversampled min,J1 oversampled max,J1 oversampled sd,J2+ (mV),J3 (mV),J4+ (mV),J4+ oversampled mean (mV),J4+ oversampled mean,J4+ oversampled min,J4+ oversampled max,J4+ oversampled sd,J4- (mV),J4- oversampled mean (mV),J4- oversampled mean,J4- oversampled min,J4- oversampled max,J4- oversampled sd" << endl; // removed "Coin battery (mV)"
    while (sd.card() -> isBusy()) {
      ; //give SD card time to finish processes
    }
    file.close(); // close the file
    Serial << "Done." << endl;
  }

  // Set up RTC
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
  rtc.alarm(DS3232RTC::ALARM_1); // clear the alarm flag
  rtc.alarmInterrupt(DS3232RTC::ALARM_1, true);
  rtc.setAlarm(DS3232RTC::ALM2_MATCH_DATE, 0, START_MIN, START_HOUR, START_DAY); // set Alarm 2 to occur at specific date and time
  rtc.alarm(DS3232RTC::ALARM_2); // clear the alarm flag
  rtc.alarmInterrupt(DS3232RTC::ALARM_2, true);

  extInterrupt(intPin); //create interrupt source on RTC Interrupt Pin

  //Setup ADC 1015 and 1115
  Serial << "Getting differential reading from AIN0 (P) and AIN1 (N)" << endl;
  Serial << "ADC Range: +/- 0.512V (1 bit = 2mV)" << endl;
  ads1015.setGain(ADS1015_GAIN_VAL); // set Gain for ADC 1015
  ads1015.begin(0x48); // Initialize ads1015 at the default address 0x48
  Serial << "Getting differential reading from AIN0 (P) and AIN1 (N)" << endl;
  Serial << "ADC Range: +/- 1.024V (1 bit = 0.03125mV)" << endl;
  ads1115.setGain(ADS1115_GAIN_VAL); // set Gain for ADC 1115
  ads1115.begin(0x49); // Initialize ads1115 at address 0x49
}

void loop()
{
  if (!digitalRead(intPin)) { // check to see if the INT/SQW pin is low, i.e. an alarm has occurred
    if (Serial) { // print if Serial port is available
      Serial << "starting data collection..." << endl;
    }
    collectData ();
  }

  if (Serial) { // end Serial connection if port is available
    Serial << "Going to sleep..." << endl;
    Serial.flush();
    Serial.end();
  }

  LowPower.deepSleep(); // deep sleep. This will break Serial communications, but the loop will still be running.

}

void collectData() {
  if (rtc.alarm(DS3232RTC::ALARM_2) ) {    // check alarm flag, clear it if set
    alarmTrigger = true; // set alarm trigger to TRUE to trigger Alarm 1
    time_t t = rtc.get();
    formatTime(timestamp, t);
    Serial << "ALARM_2 " << timestamp << endl; // print the time when this part of the loop is running
  }

  if (alarmTrigger && rtc.alarm(DS3232RTC::ALARM_1)) { // check alarm flag (and clear the flag if set)
    time_t t = rtc.get(); // get the current time
    formatTime(timestamp, t);
    time_t alarmTime = t + ALARM_INTERVAL; // calculate the next alarm time
    rtc.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0); // set the alarm
    Serial << "IT'S TIME TO SAMPLE! " << timestamp << endl; // print the time when this part of the loop is running

    ////////////////////////*****Read ADC Inputs*****///////////////////////////////////////////////////////////////////////


    adc2_diff = ads1115.readADC_Differential_0_1(); // Read J1 pin (A0/A1 differential)
    oversample(&ads1115, oversampleArray, 0);
    oversampleMean_J1 = sampleMean(oversampleArray);
    oversampleMin_J1 = sampleMin(oversampleArray);
    oversampleMax_J1 = sampleMax(oversampleArray);
    oversampleSD_J1 = sampleSD(oversampleArray);
    Serial << "J1: " << oversampleMean_J1 << ", " << oversampleMean_J1 * ADS1115_GAIN_MULT << "mV" << endl;

    adc1_1 = ads1015.readADC_SingleEnded(2); // Read J2 pin (A2)
    Serial << "J2: " << adc1_1 << "(" << adc1_1 * ADS1015_GAIN_MULT << "mV)" << endl;

    adc1_diff = ads1015.readADC_Differential_0_1(); //Read J3 pin (A0/A1 differential)
    Serial << "J3: " << adc1_diff << "(" << adc1_diff * ADS1015_GAIN_MULT << "mV" << endl;

    adc2_1 = ads1115.readADC_SingleEnded(2); // Read J4+ pin (A2)
    oversample(&ads1115, oversampleArray, 2);
    oversampleMean_J4p = sampleMean(oversampleArray);
    oversampleMin_J4p = sampleMin(oversampleArray);
    oversampleMax_J4p = sampleMax(oversampleArray);
    oversampleSD_J4p = sampleSD(oversampleArray);
    Serial << "J4+: " << oversampleMean_J4p << ", " << oversampleMean_J4p * ADS1115_GAIN_MULT << "mV" << endl;

    adc2_2 = ads1115.readADC_SingleEnded(3); // Read J4- pin (A3)
    oversample(&ads1115, oversampleArray, 3);
    oversampleMean_J4m = sampleMean(oversampleArray);
    oversampleMin_J4m = sampleMin(oversampleArray);
    oversampleMax_J4m = sampleMax(oversampleArray);
    oversampleSD_J4m = sampleSD(oversampleArray);
    Serial << "J4-: " << oversampleMean_J4m << ", " << oversampleMean_J4m * ADS1115_GAIN_MULT << "mV" << endl;

    ////////////////////////*****Write to SD card*****///////////////////////////////////////////////////////////////////////

    SdFile::dateTimeCallback(SDfileDate); // Set file date and time on sd card ("last modified"). Check out this page if having issues: https://arduino.stackexchange.com/questions/39126/how-does-one-set-attributes-for-sd-files
    file.open(FILE_NAME, FILE_WRITE); // open the file. note that only one file can be open at a time, so you have to close this one before opening another.

    if (file) { // if the file opened okay, write to it:
      Serial << "Writing to SD Card..." << endl;
      time_t TIME = rtc.get();
      file << month(TIME) << "/" << day(TIME) << "/" << year(TIME) << "," << hour(TIME) << ":" << minute(TIME) << ":" << second(TIME) << "," ;

      file << adc2_diff * ADS1115_GAIN_MULT << "," << oversampleMean_J1 * ADS1115_GAIN_MULT << "," << oversampleMean_J1 << "," << oversampleMin_J1 << "," << oversampleMax_J1 << "," << oversampleSD_J1 << ","; //J1
      file << adc1_1 * ADS1015_GAIN_MULT << ","; //J2
      file << adc1_diff * ADS1015_GAIN_MULT << ","; //J3
      file << adc2_1 * ADS1115_GAIN_MULT << "," << oversampleMean_J4p * ADS1115_GAIN_MULT << "," << oversampleMean_J4p << "," << oversampleMin_J4p << "," << oversampleMax_J4p << "," << oversampleSD_J4p << ","; //J4+
      file << adc2_2 * ADS1115_GAIN_MULT << "," << oversampleMean_J4m * ADS1115_GAIN_MULT << "," << oversampleMean_J4m << "," << oversampleMin_J4m << "," << oversampleMax_J4m << "," << oversampleSD_J4m << endl; //J4-

      while (sd.card() -> isBusy()) {
        ; //give SD card time to finish processes
      }
      // close the file:
      file.close();
    } else {
      // if the file didn't open, print an error:
      Serial << "error writing to file" << endl;
    }

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
