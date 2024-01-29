// SimpleRxAckPayload- the slave or the receiver
//only the Adalogger works!
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include "SdFat.h"

#define CE_PIN 6
#define CSN_PIN 10
#define LED_PIN 13

const int16_t byteSize = 32; // max number of bytes that can be transmitted over nRF24L01 modules

const byte radioAddress[5] = {'R', 'x', 'A', 'A', 'A'};

RF24 radio(CE_PIN, CSN_PIN);

char dataReceived[10]; // this must match dataToSend in the TX
char ackData[byteSize]; // the values to be sent to the master
bool newData = false;

#define filename "filename.csv"
SdFat sd;                 // Create the objects to talk to the SD card
File file;          // Construct File for SD card
const uint8_t chipSelect = 4; // Adalogger microSD card chip select pin
int lastPosition = 0;
bool newSdData = true;
bool setUp = true;
int loopPos = 0;

//==============

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("SimpleRxAckPayload Starting");
  pinMode(LED_PIN, OUTPUT); // Set LED pin to output mode

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
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, radioAddress);
  radio.enableAckPayload();
  radio.startListening();
  loadFileData();
  radio.writeAckPayload(1, &ackData, sizeof(ackData)); // pre-load data
  setUp = false; // set flag to false
}

//==========

void loop() {
  getData();
  showData();
  //ackData[byteSize] = {}; // clear the previous ackPayloadData
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
    file.open(filename, FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file!");
      return;
    }
    else {
      file.seek(lastPosition); // Go to the most recent file position
      memset(ackData, '\0', sizeof(ackData)); // clear the previous data
      int bytesRead = file.readBytesUntil('\n', ackData, sizeof(ackData)); // read until the next line
      lastPosition = file.position(); // update line position
      if (bytesRead == 0) {
        newSdData = false;
      }
    }
    file.close();
    while (sd.card() -> isBusy()) {
      ;
    }
    if (!setUp) { //if we are not in the setup
      radio.writeAckPayload(1, &ackData, sizeof(ackData)); // load the payload for the next time
      delay(10);
    }
  }
  else {
    Serial.println("no new data to report");
  }
  loopPos = loopPos + 1;
  if (loopPos > 50) {
    lastPosition = 0;
    newSdData = true;
  }
}

//================

void showData() {
  if (newData == true) {
    Serial.print("Data received: ");
    Serial.println(dataReceived);
    Serial.print("ackPayload sent: ");
    Serial.println(ackData);
    Serial.println();
    newData = false;
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}
