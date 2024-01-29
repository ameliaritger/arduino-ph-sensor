#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1015 ads1015;

void setup(void)
{
  Serial.begin(9600);
  delay(5000);
  Serial.println("Hello!");

  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");
  while (!ads1015.begin()) {
    Serial.println("ads not yet connected"); // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("ads connected"); // wait for serial port to connect. Needed for native USB port only
}

void loop(void)
{
  int16_t results;

  results = ads1015.readADC_Differential_2_3();
  Serial.print("Differential: "); Serial.print(results); Serial.print("("); Serial.print(results * 3); Serial.println("mV)");

  delay(1000);
}
