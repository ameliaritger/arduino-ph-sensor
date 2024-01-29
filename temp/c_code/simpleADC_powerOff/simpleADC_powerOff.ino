#include <SPI.h>
#include <Wire.h>
//#include <Adafruit_ADS1X15.h>
#include "ADS1X15.h"

ADS1015 ADS(0x48); 

const byte ADS1015_I2C_ADDRESS = 0x48;
byte ADS1015_I2C_GENERAL = 0x00; 
byte ADS1015_I2C_RESET = 0x06;

const int ADS_POWER = 12;

void setup () {
pinMode(13, OUTPUT);
pinMode(ADS_POWER, OUTPUT);
Wire.begin();
}

void loop() {
digitalWrite(ADS_POWER, HIGH); 
ADS.begin();
int16_t raw = ADS.readADC(2);
delay(5000);
digitalWrite(ADS_POWER, LOW); 
//adcReset();

digitalWrite(13, HIGH);
delay(500);
digitalWrite(13, LOW);
delay(10000);
}

void adcReset() {
  Wire.beginTransmission(ADS1015_I2C_ADDRESS); //start I2C transmission
  Wire.write(ADS1015_I2C_GENERAL); // Send general call address
  Wire.write(ADS1015_I2C_RESET); // Send reset command byte
  Wire.endTransmission(); //end I2C transmission
}
