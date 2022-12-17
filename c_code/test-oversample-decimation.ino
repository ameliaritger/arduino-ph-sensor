
//edits: 12/13/2022 fixed subtraction error with J4 oversampling calculation; create .csv not .txt; fix oversampling issue re: dividing by sampleRate+1

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "SdFat.h"
#include <Adafruit_ADS1X15.h> //ADC

#define FILE_NAME "over_dec.csv" //KEEP THIS AT OR UNDER 8 CHARACTERS DUMMY
#define ADS1015_GAIN 2
#define ADS1115_GAIN 0.03125
#define ADS1115_GAIN_DEC 0.0078125 //0.00390625 //e.g. sampleRate of 16 == GAIN_SIXTEEN

RTC_DS3231 rtc; //construct RTC
Adafruit_ADS1015 ads1015;  // Construct an ads1015
Adafruit_ADS1115 ads1115;  // Construct an ads1115 

File datafile;

int delayTime = 1500; //take a reading every X/1000 seconds (plus delayBlink time)
int delayBlink = 250; //blink on/off every X/1000 seconds
//int sampleRate = 256; //16, (32) 64, (128) 256
int sampleRate = 16;
int sampleBit = 2; // 4 = 1, 16 = 2, 64 = 3, 256 = 4

//set variables for oversampling
float average_samples_one;
int over_samples_one;
uint16_t sum_samples_one;
uint16_t dec_samples_one;

float average_samples_plus;
int over_samples_plus;
uint16_t sum_samples_plus;
uint16_t dec_samples_plus;

float average_samples_minus;
int over_samples_minus;
uint16_t sum_samples_minus;
uint16_t dec_samples_minus;

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
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.print("Initializing SD card...");
  
  // setup SD card
  if (!SD.begin(4)) {
    Serial.println("sd card initialization failed!");
    while (1);
  }
  Serial.println("sd card initialization done.");

  // If file already exists, don't add a header. If it doesn't exist, then write a header.
  if (SD.exists(FILE_NAME)) {
    Serial.println("File exists, opening file...");
  } else {
    Serial.println("File doesn't exist, creating file and adding header");
    Serial.println("Opening file...");
    datafile = SD.open(FILE_NAME, FILE_WRITE);
    Serial.println("Writing header...");
    datafile.println("Date,Time,J1 diff (mV),J1 oversampled (mV),J2+ (mV),J3 diff (mV),J4 oversampled (mV), J4 (mV), J4+ (mV),J4- (mV), J4+ oversampled (mV),J4- oversampled (mV),J1 decimated (mV),J4+ decimated (mV),J4- decimated (mV),Coin battery (mV))");
    datafile.close();
    Serial.println("done.");
  }

  // setup RTC module
  //rtc.begin(0x68); RTC address is 0x68
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  // RUN THIS FIRST TIME to set the time on the RTC, then comment out and re-upload to avoid re-setting time to compilation date and time each time 
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

   //Setup ADC 1015
    Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
    Serial.println("ADC Range: +/- 4.096V (1 bit = 2mV)");
    ads1015.setGain(GAIN_ONE); //set Gain for ADC 1015
    ads1015.begin(0x48); // Initialize ads1015 at the default address 0x48
    
   //Setup ADC 1115
   Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
   Serial.println("ADC Range: +/- 1.024V (1 bit = 0.03125mV)");
   ads1115.setGain(GAIN_FOUR); //set Gain for ADC 1115
   ads1115.begin(0x49);  // Initialize ads1115 at address 0x49
   ///12.288 / 65536 *1000
}

void loop()
{
  DateTime now = rtc.now();
    //date
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(", ");
    //time
    Serial.print(now.hour(), DEC); 
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

  // Set file date and time on sd card ("last modified")
  SdFile::dateTimeCallback(dateTime);
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  datafile = SD.open(FILE_NAME, FILE_WRITE);

  // if the file opened okay, write to it:
  if (datafile){
    Serial.println("Writing to SD Card...");
    //date
    datafile.print(now.year(), DEC);
    datafile.print('/');
    datafile.print(now.month(), DEC);
    datafile.print('/');
    datafile.print(now.day(), DEC);
    datafile.print(',');
    //time
    datafile.print(now.hour(), DEC);
    datafile.print(':');
    datafile.print(now.minute(), DEC);
    datafile.print(':');
    datafile.print(now.second(), DEC);
    datafile.print(',');

    // Read ADC1015 inputs
    int16_t adc1_1, adc1_2, adc1_diff;
    adc1_1 = ads1015.readADC_SingleEnded(2); // Read A2
    Serial.print("J2+ : "); Serial.print(adc1_1); Serial.print("("); Serial.print(adc1_1 * ADS1015_GAIN); Serial.println("mV)");
    adc1_2 = ads1015.readADC_SingleEnded(3); // Read A3
    Serial.print("coin batt : "); Serial.print(adc1_2); Serial.print("("); Serial.print(adc1_2 * ADS1015_GAIN); Serial.println("mV)");
    adc1_diff = ads1015.readADC_Differential_0_1(); //Read A0/A1 differential
    Serial.print("J3 differential: "); Serial.print(adc1_diff); Serial.print("("); Serial.print(adc1_diff * ADS1015_GAIN); Serial.println("mV)");

    // Read ADC1115 inputs
    int16_t adc2_1, adc2_2, adc2_diff;
    adc2_1 = ads1115.readADC_SingleEnded(2);
    Serial.print("J4+ : "); Serial.print(adc2_1); Serial.print("("); Serial.print(adc2_1 * ADS1115_GAIN); Serial.println("mV)");
    adc2_2 = ads1115.readADC_SingleEnded(3);
    Serial.print("J4-: "); Serial.print(adc2_2); Serial.print("("); Serial.print(adc2_2 * ADS1115_GAIN); Serial.println("mV)");
    adc2_diff = ads1115.readADC_Differential_0_1();
    Serial.print("J1 differential: "); Serial.print(adc2_diff); Serial.print("("); Serial.print(adc2_diff * ADS1115_GAIN); Serial.println("mV)");

    // Oversampling J1+
    over_samples_one = sampleRate; //or do 64/256
    sum_samples_one = 0;
    for(int i = 0; i < over_samples_one; i++){
      int a = ads1115.readADC_Differential_0_1();
      //Serial.print(a);
      sum_samples_one += a;
      dec_samples_one = sum_samples_one >> sampleBit;
    }
    average_samples_one = sum_samples_one / over_samples_one;

    // Oversampling J4+
    over_samples_plus = sampleRate; //or do 64/256
    sum_samples_plus = 0;
    for(int i = 0; i < over_samples_plus; i++){
      int a = ads1115.readADC_SingleEnded(2);
      //Serial.print(a);
      sum_samples_plus += a;
      dec_samples_plus = sum_samples_plus >> sampleBit;
    }
    average_samples_plus = sum_samples_plus / over_samples_plus;

    // Oversampling J4-
    over_samples_minus = sampleRate; //or do 64/256
    sum_samples_minus = 0;
    for(int i = 0; i < over_samples_minus; i++){
      int a = ads1115.readADC_SingleEnded(3);
      //Serial.print(a);
      sum_samples_minus += a;      
      dec_samples_minus = sum_samples_minus >> sampleBit;
    }
    average_samples_minus = sum_samples_minus / over_samples_minus;

    Serial.print("J4+ oversampled: "); Serial.print(average_samples_plus); Serial.print("("); Serial.print(average_samples_plus * ADS1115_GAIN); Serial.println("mV)");
    Serial.print("J4+ decimated: "); Serial.print(dec_samples_plus); Serial.print("("); Serial.print(dec_samples_plus * ADS1115_GAIN_DEC); Serial.println("mV)");
    Serial.print("J4- oversampled: "); Serial.print(average_samples_minus); Serial.print("("); Serial.print(average_samples_minus * ADS1115_GAIN); Serial.println("mV)");
    Serial.print("J4- decimated: "); Serial.print(dec_samples_minus); Serial.print("("); Serial.print(dec_samples_minus * ADS1115_GAIN_DEC); Serial.println("mV)");
    Serial.print("J1 oversampled: "); Serial.print(average_samples_one); Serial.print("("); Serial.print(average_samples_one * ADS1115_GAIN); Serial.println("mV)");
    Serial.print("J1 decimated: "); Serial.print(dec_samples_one); Serial.print("("); Serial.print(dec_samples_one * ADS1115_GAIN_DEC); Serial.println("mV)");

    // Write ADC inputs
    datafile.print(adc2_diff * ADS1115_GAIN); datafile.print(','); //J1+
    datafile.print(average_samples_one * ADS1115_GAIN); datafile.print(','); //J1+ oversampled
    datafile.print(adc1_1 * ADS1015_GAIN); datafile.print(','); //J2+
    datafile.print(adc1_diff * ADS1015_GAIN); datafile.print(','); //J3+
    datafile.print((average_samples_plus * ADS1115_GAIN) - (average_samples_minus * ADS1115_GAIN)); datafile.print(','); //J4 oversampled
    datafile.print((adc2_1 * ADS1115_GAIN) - (adc2_2 * ADS1115_GAIN)); datafile.print(','); //J4
    datafile.print(adc2_1 * ADS1115_GAIN); datafile.print(','); //J4+
    datafile.print(adc2_2 * ADS1115_GAIN); datafile.print(','); //J4-
    datafile.print(average_samples_plus * ADS1115_GAIN); datafile.print(','); //J4+ oversampled
    datafile.print(average_samples_minus * ADS1115_GAIN); datafile.print(','); //J4- oversampled
    datafile.print(dec_samples_one * ADS1115_GAIN_DEC); datafile.print(','); //J1 decimated
    datafile.print(dec_samples_plus * ADS1115_GAIN_DEC); datafile.print(','); //J1 decimated
    datafile.print(dec_samples_minus * ADS1115_GAIN_DEC); datafile.print(','); //J1 decimated    
    datafile.println(adc1_2 * ADS1015_GAIN); //coin batt

    // close the file:
    datafile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error writing to file");
  }
      
    digitalWrite(13, HIGH);   // turn the LED on
    delay(delayBlink);              // wait for X/1000 seconds
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(delayBlink);              // wait for  X/1000 seconds
  
    delay(delayTime);
}
