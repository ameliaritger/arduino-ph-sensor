#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "SdFat.h"
#include <Adafruit_ADS1X15.h> //ADC

#define FILE_NAME "mydata.txt" // set file name

RTC_DS3231 rtc;
Adafruit_ADS1015 ads1015;  // Construct an ads1015
Adafruit_ADS1115 ads1115;  // Construct an ads1115 

File datafile;
//SdFat sd;
//SdFile myFile;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

int delayTime = 5000; //take a reading every X/1000 seconds

float average_samples;
int over_samples;
float sum_samples;
// set rtc date and time timestamp to sd card

void dateTime(uint16_t* date, uint16_t* time) {
  DateTime now = rtc.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}


void setup()
{
    // initialize digital pin 13 as an output (blink)
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  
  // setup SD card
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // setup RTC module
  //rtc.begin(0x68); RTC address is 0x68
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  
   // When time needs to be re-set on a previously configured device, the
   // following line sets the RTC to the date & time this sketch was compiled
// RUN THIS FIRST to set the time on the RTC, then comment out and re-run to avoid re-setting time to compilation date and time each time 
   //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

   //Setup ADC 1015
    Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
    Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");
    ads1015.begin(0x48); // Initialize ads1015 at the default address 0x48
    
   //Setup ADC 1115
   Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
   Serial.println("ADC Range: +/- 6.144V (1 bit = 0.188mV)"); 
   ads1115.begin(0x49);  // Initialize ads1115 at address 0x49
    ///12.288 / 65536 *1000

}

void loop()
{
  DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

  // Set file date and time ("last modified")
  SdFile::dateTimeCallback(dateTime);
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  datafile = SD.open(FILE_NAME, FILE_WRITE);

// if the file opened okay, write to it:
  if (datafile)
    Serial.println("Writing to SD Card...");
    datafile.print(now.year(), DEC);
    datafile.print('/');
    //datafile.print("\t");
    //datafile.print(" ");
    datafile.print(now.month(), DEC);
    datafile.print('/');
    //datafile.print("\t");
    //datafile.print(" ");
    datafile.print(now.day(), DEC);
    datafile.print(" (");
    datafile.print(daysOfTheWeek[now.dayOfTheWeek()]);
    //datafile.print("\t");
    //datafile.print(" ");
    datafile.print(") ");
    datafile.print("\t");
    datafile.print(now.hour(), DEC);
    datafile.print(':');
    //datafile.print("\t");
    //datafile.print(" ");
    datafile.print(now.minute(), DEC);
    datafile.print(':');
    //datafile.print("\t");
    //datafile.print(" ");
    datafile.print(now.second(), DEC);
    datafile.println();

    datafile.print("  "); //delimiter between timestamps

    // Read ADC1015 inputs
    int16_t adc1_1, adc1_diff;
    adc1_1 = ads1015.readADC_SingleEnded(2); // Read A2
    Serial.print("J2+ : "); Serial.print(adc1_1); Serial.print("("); Serial.print(adc1_1 * 3); Serial.println("mV)");
    adc1_diff = ads1015.readADC_Differential_0_1();
    Serial.print("J3 differential: "); Serial.print(adc1_diff); Serial.print("("); Serial.print(adc1_diff * 3); Serial.println("mV)");


    //over_samples = 256;
    //sum_samples = 0;
    //for(int i = 0; i <= over_samples; i++){
    //  int a = ads1015.readADC_SingleEnded(2);
      //Serial.print(a);
    //  sum_samples += a;
    //}
    //average_samples = sum_samples / over_samples;

    //Serial.print("J2+ oversampled: "); Serial.print(average_samples); Serial.print("("); Serial.print(average_samples * 3); Serial.println("mV)");

    // Write ADC1015 inputs
    datafile.print("J2+ (mV) = ");
    datafile.print(adc1_1 * 3);
    datafile.print(", "); // delimiter between data
    //datafile.print("J2+ oversampled (mV) = ");
    //datafile.println(average_samples * 3);
    datafile.print("J3 diff (mV) = ");
    datafile.println(adc1_diff * 3);

    // Read ADC1115 inputs
    int16_t adc2_1, adc2_2, adc2_diff;
    adc2_1 = ads1115.readADC_SingleEnded(2);
    Serial.print("J4+ : "); Serial.print(adc2_1); Serial.print("("); Serial.print(adc2_1 * 0.1875); Serial.println("mV)");
    adc2_2 = ads1115.readADC_SingleEnded(3);
    Serial.print("J4-: "); Serial.print(adc2_2); Serial.print("("); Serial.print(adc2_2 * 0.1875); Serial.println("mV)");
    
    //over_samples = 256;
    //sum_samples = 0;
    //for(int i = 0; i <= over_samples; i++){
    //  int a = ads1115.readADC_SingleEnded(3);
      //Serial.print(a);
    //  sum_samples += a;
    //}
    //average_samples = sum_samples / over_samples;

    //Serial.print("J4- oversampled: "); Serial.print(average_samples); Serial.print("("); Serial.print(average_samples * 0.1875); Serial.println("mV)");
    adc2_diff = ads1115.readADC_Differential_0_1();
    Serial.print("J1 differential: "); Serial.print(adc2_diff); Serial.print("("); Serial.print(adc2_diff * 0.1875); Serial.println("mV)");

    // Write ADC1015 inputs
    datafile.print("J4+ (mV) = ");
    datafile.print(adc2_1 * 0.1875);
    datafile.print(", "); // delimiter between data
    datafile.print("J4- (mV) = ");
    datafile.print(adc2_2 * 0.1875);
    //datafile.print(", "); // delimiter between data
    //datafile.print("J4- oversampled (mV) = ");
    //datafile.println(average_samples * 0.1875);
    datafile.print("J1 diff (mV) = ");
    datafile.println(adc2_diff * 0.1875);
    
    // close the file:
    datafile.close();
    
    digitalWrite(13, HIGH);   // turn the LED on
    delay(500);              // wait for X/1000 seconds
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(500);              // wait for  X/1000 seconds
  
    delay(delayTime);
}
