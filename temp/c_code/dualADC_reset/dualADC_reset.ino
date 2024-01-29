// Current draw = 0.331 mA when not reading, ~0.42-0.55 mA when reading 

#include <SPI.h>
#include <Wire.h>
#include "ADS1X15.h"

ADS1115 ADS1(0x48);
ADS1015 ADS0(0x49);

const int ADS1x15_I2C_GENERAL = 0x00;
const byte ADS1x15_I2C_RESET = 0x06;
int16_t raw = 0;

void setup () {
  Serial.begin(9600);
  Serial.print("Starting...");
  //pinMode(13, OUTPUT);
  Wire.begin();
}

void loop() {
  //digitalWrite(13, HIGH);
  //delay(500);
  //digitalWrite(13, LOW);
  //delay(500);

  ADS0.begin();
  ADS1.begin();
  delay(10);
  if (ADS0.isConnected())
  if (ADS1.isConnected())
  
  raw = ADS0.readADC(1);
  Serial.print(raw);
  Serial.print("\t");
  raw = ADS0.readADC(2);
  Serial.print(raw);
  Serial.print("\t");
  raw = ADS0.readADC(2);
  Serial.print(raw);
  Serial.print("\t");
  Serial.print("\t");
  raw = ADS1.readADC(1);
  Serial.print(raw);
  Serial.print("\t");
  raw = ADS1.readADC(2);
  Serial.print(raw);
  Serial.print("\t");
  raw = ADS1.readADC(3);
  Serial.print(raw);
  Serial.println();

  adcReset();

  delay(5000);
}

void adcReset() {
  Wire.beginTransmission(ADS1x15_I2C_GENERAL); //start I2C transmission
  Wire.write(ADS1x15_I2C_RESET); // Send reset command byte
  Wire.endTransmission(); //end I2C transmission
}
