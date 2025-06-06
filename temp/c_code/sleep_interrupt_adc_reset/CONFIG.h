#ifndef config_h
#define config_h

#define FILE_NAME "newtest.csv" //KEEP THIS AT OR UNDER 8 CHARACTERS, not including '.csv' extension

constexpr time_t ALARM_INTERVAL {20};   // alarm interval (in seconds)
constexpr time_t START_DAY {27};   // set the day of the month start time (1-31)
constexpr time_t START_HOUR {1};   // set the hours start time (0-23)
constexpr time_t START_MIN {57};   // set the minutes start time (0-59)

#define OVERSAMPLE_VALUE 64 // set oversampling value. DEFAULT: 64. options: 16, 64, and 256 //add 1000,2000,7000 (seconds) to delayTime for total sampling rate

// Set ADC gain values. DO NOT CHANGE, see table below for reference.
#define ADS1015_GAIN_VAL GAIN_ONE   // Recommended: GAIN_EIGHT
#define ADS1015_GAIN_MULT 0.25             // Change this number based on gain selected.
#define ADS1115_GAIN_VAL GAIN_ONE  // Recommended: GAIN_FOUR
#define ADS1115_GAIN_MULT 0.03125          // Change this number based on gain selected.

//                                                                ADS1015  ADS1115
//                                                                -------  -------
// ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

#endif
