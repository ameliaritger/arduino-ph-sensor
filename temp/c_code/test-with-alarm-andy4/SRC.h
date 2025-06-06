#ifndef src_h
#define src_h

void printDateTime(time_t t) {
  Serial << ((day(t) < 10) ? "0" : "") << _DEC(day(t));
  Serial << monthShortStr(month(t)) << _DEC(year(t)) << ' ';
  Serial << ((hour(t) < 10) ? "0" : "") << _DEC(hour(t)) << ':';
  Serial << ((minute(t) < 10) ? "0" : "") << _DEC(minute(t)) << ':';
  Serial << ((second(t) < 10) ? "0" : "") << _DEC(second(t));
}

// Calculate mean value from oversampling
float sampleMean(int oversampleArray[OVERSAMPLE_VALUE]) { // or replace this with int oversampleArray[]
  long sum = 0L; //
  for (int i = 0; i < OVERSAMPLE_VALUE; i++) {
    sum += oversampleArray[i];
    //Serial.println(oversampleArray[i]);
  }
  float mean = sum / OVERSAMPLE_VALUE;
  return mean;
}

// Calculate minimum value from oversampling
int sampleMin(int oversampleArray[OVERSAMPLE_VALUE]) { // or replace this with int* oversampleArrayPointer
  int minVal = oversampleArray[0]; // Initialize an array to store min value
  for (int i = 0; i < OVERSAMPLE_VALUE; i++) {
    minVal = min(oversampleArray[i], minVal);
    //Serial.println(oversampleArray[i]);
  }
  return minVal;
}

// Calculate maximum value from oversampling
int sampleMax(int oversampleArray[OVERSAMPLE_VALUE]) { // or replace this with int oversampleArray[OVERSAMPLE_VALUE]
  int maxVal = oversampleArray[0]; // Initialize an array to store min value
  for (int i = 0; i < OVERSAMPLE_VALUE; i++) {
    maxVal = max(oversampleArray[i], maxVal);
  }
  return maxVal;
}

// function to return the compile date and time as a time_t value
time_t compileTime() {
  const time_t FUDGE(15);    //fudge factor to allow for upload time, etc. (seconds, YMMV)
  const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char compMon[4], *m;

  strncpy(compMon, compDate, 3);
  compMon[3] = '\0';
  m = strstr(months, compMon);

  tmElements_t tm;
  tm.Month = ((m - months) / 3 + 1);
  tm.Day = atoi(compDate + 4);
  tm.Year = atoi(compDate + 7) - 1970;
  tm.Hour = atoi(compTime);
  tm.Minute = atoi(compTime + 3);
  tm.Second = atoi(compTime + 6);

  time_t t = makeTime(tm);
  return t + FUDGE;        //add fudge factor to allow for compile time
}

// format a time_t value, return the formatted string in buf (must be at least 25 bytes)
void formatTime(char *buf, time_t t)
{
  //char m[4];    // temporary storage for month string (DateStrings.cpp uses shared buffer)
  //strcpy(m, monthShortStr(month(t)));
  sprintf(buf, "%.2d/%.2d/%d %.2d:%.2d:%.2d",
          day(t), month(t), year(t), hour(t), minute(t), second(t));
}


class TimeSpan {
  public:
    TimeSpan(int32_t seconds = 0);
    TimeSpan(int16_t days, int8_t hours, int8_t minutes, int8_t seconds);
    TimeSpan(const TimeSpan &copy);

    /*!
        @brief  Number of days in the TimeSpan
                e.g. 4
        @return int16_t days
    */
    int16_t days() const {
      return _seconds / 86400L;
    }
    /*!
        @brief  Number of hours in the TimeSpan
                This is not the total hours, it includes the days
                e.g. 4 days, 3 hours - NOT 99 hours
        @return int8_t hours
    */
    int8_t hours() const {
      return _seconds / 3600 % 24;
    }
    /*!
        @brief  Number of minutes in the TimeSpan
                This is not the total minutes, it includes days/hours
                e.g. 4 days, 3 hours, 27 minutes
        @return int8_t minutes
    */
    int8_t minutes() const {
      return _seconds / 60 % 60;
    }
    /*!
        @brief  Number of seconds in the TimeSpan
                This is not the total seconds, it includes the days/hours/minutes
                e.g. 4 days, 3 hours, 27 minutes, 7 seconds
        @return int8_t seconds
    */
    int8_t seconds() const {
      return _seconds % 60;
    }
    /*!
        @brief  Total number of seconds in the TimeSpan, e.g. 358027
        @return int32_t seconds
    */
    int32_t totalseconds() const {
      return _seconds;
    }

    TimeSpan operator+(const TimeSpan &right) const;
    TimeSpan operator-(const TimeSpan &right) const;

  protected:
    int32_t _seconds; ///< Actual TimeSpan value is stored as seconds
};


class DateTime {
  public:
    DateTime(uint32_t t = 946684800); // Unixtime for 2000-01-01 00:00:00, useful for initialization
    DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour = 0,
             uint8_t min = 0, uint8_t sec = 0);
    DateTime(const DateTime &copy);
    DateTime(const char *date, const char *time);
    DateTime(const __FlashStringHelper *date, const __FlashStringHelper *time);
    DateTime(const char *iso8601date);
    bool isValid() const;
    char *toString(char *buffer) const;

    /*!
        @brief  Return the year.
        @return Year (range: 2000--2099).
    */
    uint16_t year() const {
      return 2000U + yOff;
    }
    /*!
        @brief  Return the month.
        @return Month number (1--12).
    */
    uint8_t month() const {
      return m;
    }
    /*!
        @brief  Return the day of the month.
        @return Day of the month (1--31).
    */
    uint8_t day() const {
      return d;
    }
    /*!
        @brief  Return the hour
        @return Hour (0--23).
    */
    uint8_t hour() const {
      return hh;
    }

    uint8_t twelveHour() const;
    /*!
        @brief  Return whether the time is PM.
        @return 0 if the time is AM, 1 if it's PM.
    */
    uint8_t isPM() const {
      return hh >= 12;
    }
    /*!
        @brief  Return the minute.
        @return Minute (0--59).
    */
    uint8_t minute() const {
      return mm;
    }
    /*!
        @brief  Return the second.
        @return Second (0--59).
    */
    uint8_t second() const {
      return ss;
    }

    uint8_t dayOfTheWeek() const;

    /* 32-bit times as seconds since 2000-01-01. */
    uint32_t secondstime() const;

    /* 32-bit times as seconds since 1970-01-01. */
    uint32_t unixtime(void) const;

    /*!
        Format of the ISO 8601 timestamp generated by `timestamp()`. Each
        option corresponds to a `toString()` format as follows:
    */
    enum timestampOpt {
      TIMESTAMP_FULL, //!< `YYYY-MM-DDThh:mm:ss`
      TIMESTAMP_TIME, //!< `hh:mm:ss`
      TIMESTAMP_DATE  //!< `YYYY-MM-DD`
    };
    String timestamp(timestampOpt opt = TIMESTAMP_FULL) const;

    DateTime operator+(const TimeSpan &span) const;
    DateTime operator-(const TimeSpan &span) const;
    TimeSpan operator-(const DateTime &right) const;
    bool operator<(const DateTime &right) const;

    /*!
        @brief  Test if one DateTime is greater (later) than another.
        @warning if one or both DateTime objects are invalid, returned value is
          meaningless
        @see use `isValid()` method to check if DateTime object is valid
        @param right DateTime object to compare
        @return True if the left DateTime is later than the right one,
          false otherwise
    */
    bool operator>(const DateTime &right) const {
      return right < *this;
    }

    /*!
        @brief  Test if one DateTime is less (earlier) than or equal to another
        @warning if one or both DateTime objects are invalid, returned value is
          meaningless
        @see use `isValid()` method to check if DateTime object is valid
        @param right DateTime object to compare
        @return True if the left DateTime is earlier than or equal to the
          right one, false otherwise
    */
    bool operator<=(const DateTime &right) const {
      return !(*this > right);
    }

    /*!
        @brief  Test if one DateTime is greater (later) than or equal to another
        @warning if one or both DateTime objects are invalid, returned value is
          meaningless
        @see use `isValid()` method to check if DateTime object is valid
        @param right DateTime object to compare
        @return True if the left DateTime is later than or equal to the right
          one, false otherwise
    */
    bool operator>=(const DateTime &right) const {
      return !(*this < right);
    }
    bool operator==(const DateTime &right) const;

    /*!
        @brief  Test if two DateTime objects are not equal.
        @warning if one or both DateTime objects are invalid, returned value is
          meaningless
        @see use `isValid()` method to check if DateTime object is valid
        @param right DateTime object to compare
        @return True if the two objects are not equal, false if they are
    */
    bool operator!=(const DateTime &right) const {
      return !(*this == right);
    }

  protected:
    uint8_t yOff; ///< Year offset from 2000
    uint8_t m;    ///< Month 1-12
    uint8_t d;    ///< Day 1-31
    uint8_t hh;   ///< Hours 0-23
    uint8_t mm;   ///< Minutes 0-59
    uint8_t ss;   ///< Seconds 0-59
};

#endif
