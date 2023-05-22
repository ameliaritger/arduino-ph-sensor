#ifndef config_h
#define config_h

#define FILE_NAME "032023.csv" //KEEP THIS AT OR UNDER 8 CHARACTERS DUMMY

// Change gain values. See table below.
#define ADS1015_GAIN_VAL GAIN_ONE // Recommended: GAIN_ONE
#define ADS1015_GAIN 2
#define ADS1115_GAIN_VAL GAIN_ONE //Recommended: GAIN_FOUR
#define ADS1115_GAIN 0.03125

const long interval = 5000;           // interval at which to perform function (milliseconds)
int sampleRate = 16; //DEFAULT: 64. options: 16, 64, and 256 //add 1000,2000,7000 (seconds) to delayTime for total sampling rate

#endif

  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
