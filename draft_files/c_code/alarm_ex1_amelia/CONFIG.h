#ifndef config_h
#define config_h

#define FILE_NAME "test3.csv" //KEEP THIS AT OR UNDER 8 CHARACTERS, not including '.csv' extension

constexpr time_t ALARM_INTERVAL {10};   // alarm interval (in seconds)
constexpr time_t START_DAY {24};   // set the day of the month start time (1-31)
constexpr time_t START_HOUR {18};   // set the hours start time (0-23)
constexpr time_t START_MIN {05};   // set the minutes start time (0-59)

#endif
