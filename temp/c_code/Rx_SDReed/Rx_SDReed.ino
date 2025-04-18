//the slave or the receiver
//only the Adalogger works!
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include "SdFat.h"

#define CE_PIN 6
#define CSN_PIN 10
#define REED_PIN 12
#define LED_PIN 13

// Set up Radio
const byte radioAddress[5] = {'R', 'x', 'A', 'A', 'A'};
RF24 radio(CE_PIN, CSN_PIN);

// Initialize variables
unsigned long dataReceived; // type must match dataToSend in Tx
byte ackData[32] = {0}; // the values to be sent to the master; 32 bytes max for NRF24L01
const uint16_t dataDivisible = 1000; // minimum divisible value for dataReceived value
int ledState = LOW;
int reedState;
bool newData = false;
bool triggerState = false;
bool radioPowerState = false;

// Initialize SD card variables
#define filename "filename.csv"
SdFat sd; // Create the objects to talk to the SD card
File file; // Construct File for SD card
const uint8_t chipSelect = 4; // Adalogger microSD card chip select pin
bool newSdData = true;

// Initialize millis variables
unsigned long previousMillisRadio = 0;
unsigned long previousMillisReed = 0;
unsigned long powerInterval = 50000; //interval for power to be supplied to NRF24L01 (in ms) after Reed switch triggered

//==============

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  pinMode(LED_PIN, OUTPUT); // Set LED pin to output mode
  pinMode(REED_PIN, INPUT_PULLUP); // Init pullup resistor on Reed pin
  digitalWrite(REED_PIN, HIGH); // Set Reed pin to HIGH
  dataReceived = 1000; // Set initial blink interval (to be changed by Tx)

  SPI.begin();
  Serial.println("Initializing SD card...");
  if (!sd.begin(chipSelect, SD_SCK_MHZ(12))) {
    Serial.println("sd card initialization failed!");
    sd.initErrorHalt(); // not being able to save data is a terminal error in all cases
  }
  else {
    Serial.println("SD card OK");
  }

  Serial.println("Initializing radio");
  // Generate initial ackData
  radio.begin();
  //radio.setDataRate(RF24_250KBPS);
  //radio.setPALevel(RF24_PA_LOW); // or MIN
  //radio.enableAckPayload(); // DO I NEED TO INIT THIS HERE, YEAH?
  ///radio.openReadingPipe(1, radioAddress);
  loadFileData(); // pre-load Ack Payload
  Serial.println("Powering down radio");
  radio.powerDown(); // immediately power down the radio until reed switch trigger
  //radio.startListening();
}

//==========

void loop() {
  //digitalWrite(LED_PIN, LOW); // turn off LED
  reedState = digitalRead(REED_PIN);
  if (reedState == LOW) {
    Serial.println("switch activated!");
    if (!radioPowerState) { //if this is the first time the Radio is turned on
      radio.powerUp();
      delay(10); // RF24 documentation: "this will take up to 5ms for maximum compatibility"
      setupRadio();
      radioPowerState = true; //variable to only powerUp the radio once
      triggerState = true; // variable to indicate the Reed switch has been triggered
    }
    previousMillisReed = millis(); //store the time the Reed switch was triggered
  }
  if (triggerState == true ) {
    getData();
    showData();
    unsigned long currentMillisReed = millis(); // get the current time
    if (currentMillisReed - previousMillisReed >= powerInterval) { // if the radio has been powered on and transmitting for longer than the interval
      previousMillisReed = currentMillisReed; // update previousMillisValue
      //digitalWrite(LED_PIN , LOW); // turn off the LED
      file.close(); // make sure file is closed (prevent bricking card)
      while (sd.card() -> isBusy()) {
        ;
      }
      triggerState = false;
      radioPowerState = false;
      radio.powerDown();  // powerDown the radio
    }
  }
  blinkWithoutDelay(); // blink at the interval dictated by dataReceived
}

//================

void getData() {
  if (radio.available()) {
    unsigned long previousDataReceived = dataReceived; // save previous dataReceived value
    radio.read(&dataReceived, sizeof(dataReceived));
    if (abs(dataReceived - previousDataReceived) < (previousDataReceived * 0.1)) { // if the difference between old dataReceived value and new dataReceived value is greater than 10%, overwrite dataReceived
      if (dataReceived % dataDivisible != 0) { // if dataReceived is not a multiple of 1000
        dataReceived = ((previousDataReceived + dataDivisible - 1) / dataDivisible * dataDivisible); // make it a multiple of 1000
      } else { // if it is a multiple of 1000 already
        dataReceived = previousDataReceived;
      }
    }
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

//=================

void blinkWithoutDelay() {
  unsigned long currentMillisRadio = millis();
  if (currentMillisRadio - previousMillisRadio >= dataReceived) {
    previousMillisRadio = currentMillisRadio; // update previousMillis value
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(LED_PIN, ledState);
  }
}

//================

void setupRadio() {
  radio.begin();
  radio.setDataRate(RF24_250KBPS); // slower data rate = higher resistance to noise
  //radio.setPALevel(RF24_PA_LOW); // LOW prevents power supply related problems (RF24_PA_MAX is default)
  radio.setCRCLength(RF24_CRC_16); // set CRC length to 16-bit to ensure data quality
  radio.enableAckPayload();
  radio.openReadingPipe(1, radioAddress);
  radio.startListening();
}
