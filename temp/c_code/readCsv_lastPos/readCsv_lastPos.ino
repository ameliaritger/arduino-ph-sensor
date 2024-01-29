#include <SPI.h>
#include <SdFat.h>

#define CS_PIN  4

SdFat sd;
File file;

const int16_t byteSize = 32;
int lastPosition = 0;
bool newData = true;

void setup() {
  Serial.begin(9600);

  // Wait for USB Serial
  while (!Serial) {
    ;
  }
  Serial.println("Type any character to start");
  while (!Serial.available()) {
    yield();
  }
  // Initialize the SD.
  if (!sd.begin(CS_PIN, SD_SCK_MHZ(12))) {
    sd.initErrorHalt(&Serial);
    return;
  }
}

void loop() {
  if (newData) {
    char buf[byteSize]; // create an array large enough to capture the max number of bites found on a line (excluding the header)
    file.open("filename.csv", FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file!");
      return;
    }
    else {
      file.seek(lastPosition); // Go to the most recent file position
      memset(buf, '\0', sizeof(buf)); // clear previous data
      int bytesRead = file.readBytesUntil('\n', buf, sizeof(buf)); // read until the next line
      Serial.println(buf); // print the line
      //Serial.print(bytesRead);
      lastPosition = file.position(); // update line position
      if (bytesRead == 0) {
        newData = false;
      }
    }
    file.close();
    while (sd.card() -> isBusy()) {
      ;
    }
    Serial.println(F("Done"));
    delay(10);
  }
  else {
    Serial.println("no new data to report!");
    delay(5000);
  }
}
