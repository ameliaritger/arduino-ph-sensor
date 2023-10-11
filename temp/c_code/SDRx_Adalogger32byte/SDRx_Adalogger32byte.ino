// SimpleRxAckPayload- the slave or the receiver
//only the Adalogger works!
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include "SdFat.h"

#define CE_PIN 6
#define CSN_PIN 10

//const int8_t byteSize = 32; // max number of bytes that can be transmitted over nRF24L01 modules (plus 1 for the null terminator)

const byte radioAddress[5] = {'R', 'x', 'A', 'A', 'A'};

RF24 radio(CE_PIN, CSN_PIN);

char dataReceived[10]; // this must match dataToSend in the TX
byte ackData[32] = {0}; // the values to be sent to the master
bool newData = false;

#define filename "filename.csv"
SdFat sd; // Create the objects to talk to the SD card
File file; // Construct File for SD card
const uint8_t chipSelect = 4; // Adalogger microSD card chip select pin
int lastPosition = 0;
bool newSdData = true;
int loopPos = 0;
int i = 0;

//==============

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("SimpleRxAckPayload Starting");

  SPI.begin();
  if (!sd.begin(chipSelect, SD_SCK_MHZ(12))) {
    Serial.println("sd card initialization failed!");
    sd.initErrorHalt(); // not being able to save data is a terminal error in all cases
  }
  else {
    Serial.println("SD card OK");
  }
  // Generate initial ackData
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  //radio.setPALevel(RF24_PA_LOW); // or MIN
  radio.enableAckPayload();
  radio.openReadingPipe(1, radioAddress);
  loadFileData();
  radio.startListening();
}

//==========

void loop() {
  getData();
  showData();
}

//================

void getData() {
  if (radio.available()) {
    radio.read(&dataReceived, sizeof(dataReceived));
    loadFileData();
    newData = true;
  }
}

//================

void loadFileData() {
  if (newSdData) {
    file.open("filename.csv", FILE_READ);
    if (file) {
      int bytesRead = file.readBytes(ackData, sizeof(ackData) - 1); // -1 to remove null terminator
      if (bytesRead > 0) {
        radio.writeAckPayload(1, &ackData, sizeof(ackData)); // load the payload for the next time
      } else { // end of file is reached
        file.close();
        while (sd.card() -> isBusy()) {
          ;
        }
        Serial.println("end of file");
        newSdData = false;
        for (int i = 0; i < sizeof(ackData) / sizeof(ackData[0]); i++) {
          ackData[i] = 0; //replace ackData with 0s
        }
      }
    } else {
      Serial.println("no new data to report!");
      radio.writeAckPayload(1, &ackData, sizeof(ackData)); // load the blank message payload
    }
  }
}

//================

void showData() {
  if (newData == true) {
    Serial.print("Data received: ");
    Serial.println(dataReceived);
    Serial.print("ackPayload sent: ");
    Serial.println(F(ackData));
    newData = false;
  }
}
