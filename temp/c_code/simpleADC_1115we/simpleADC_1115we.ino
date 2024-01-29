#include <SPI.h>
#include <Wire.h>
#include<ADS1115_WE.h>

#define I2C_ADDRESS 0x48

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

const byte ADS1015_I2C_ADDRESS = 0x48;
byte ADS1015_I2C_GENERAL = 0x00;
byte ADS1015_I2C_RESET = 0x06;

void setup () {
  pinMode(13, OUTPUT);
  Wire.begin();
  if (!adc.init()) {
  }
  adc.setVoltageRange_mV(ADS1115_RANGE_6144); //comment line/change parameter to change range
}

void loop() {
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);

  //ads1015.begin(ADS1015_I2C_ADDRESS);
  //int16_t adc1_1, adc1_2, adc1_diff;
  //adc1_1 = ads1015.readADC_SingleEnded(2);
  //adc1_2 = ads1015.readADC_SingleEnded(3);
  //adc1_diff = ads1015.readADC_Differential_0_1();
  float voltage = 0.0;
  //adcReset();
  voltage = readChannel(ADS1115_COMP_1_GND);

  delay(10000);
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){}
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

void adcReset() {
  Wire.beginTransmission(ADS1015_I2C_ADDRESS); //start I2C transmission
  Wire.write(ADS1015_I2C_GENERAL); // Send general call address
  Wire.write(ADS1015_I2C_RESET); // Send reset command byte
  Wire.endTransmission(); //end I2C transmission
}
