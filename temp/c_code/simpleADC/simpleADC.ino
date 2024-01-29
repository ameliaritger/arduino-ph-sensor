#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1015 ads1015;

void setup () {
pinMode(13, OUTPUT);
ads1015.begin();
}

void loop() {
digitalWrite(13, HIGH);
delay(500);
digitalWrite(13, LOW);
delay(500);

int16_t adc1_1, adc1_2, adc1_diff;
adc1_1 = ads1015.readADC_SingleEnded(2);
adc1_2 = ads1015.readADC_SingleEnded(3);
adc1_diff = ads1015.readADC_Differential_0_1();

delay(10000);
}
