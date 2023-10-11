#include <SPI.h>
#include <SdFat.h>

#define CS_PIN  4

SdFat sd;
File file;
const int16_t byteSize = 32; // max number of bytes that can be transmitted over nRF24L01 modules

int lastPosition = 0;
bool newData = true;
int i = 0;
byte buf[byteSize] = {0};

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
  file.open("filename.csv", FILE_READ);
  if (file) {
    int bytesRead = file.readBytes(buf, sizeof(buf));
    if (bytesRead > 0) {
      buf[bytesRead-1] = '\0'; // remove null terminator
    Serial.println(F(buf));
    } else { // end of file is reached
      Serial.println("end of file");
      file.close();
      while (sd.card() -> isBusy()) {
        ;
      }
    }
  }
  Serial.println("next loop!");
  delay(1000);
}

void readFile() {
  file.open("filename.csv", FILE_READ);
  while (file.available()) {
    buf[i] = file.read();
    i++;
    if (i == 32) {
      Serial.print(F(buf));
      Serial.println("");
      delay(100);
      //Serial.println("resetting!");
      i = 0;
    }
  }
  file.close();
  while (sd.card() -> isBusy()) {
    ;
  }
}
