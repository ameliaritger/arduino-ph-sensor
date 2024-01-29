// ADS1115 "sleep" = 0.32 uA, "sample" = 0.03 mA
// ADS1015 "sleep" = 0.3 uA, "sample" = 0.002 mA

#include <SPI.h>
#include <Wire.h>
#include "ADS1X15.h"

ADS1115 ADS(0x48);

byte ADS1x15_I2C_GENERAL = 0x00;
byte ADS1x15_I2C_RESET = 0x06;
int16_t raw = 0;

void setup () {
  Serial.begin(9600);
  Serial.print("Starting...");
  pinMode(13, OUTPUT);
  Wire.begin();
}

void loop() {
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);

  ADS.begin();
  raw = ADS.readADC(1);
  Serial.print(raw);
  Serial.print("\t");
  raw = ADS.readADC(2);
  Serial.print(raw);
  Serial.print("\t");
  raw = ADS.readADC(2);
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
