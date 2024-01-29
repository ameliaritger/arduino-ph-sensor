// Swap roles ADALOGGER

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include "SdFat.h"

#define CE_PIN   6
#define CSN_PIN 10
#define REED_PIN 12
#define LED_PIN 13

const byte rxAddress[5] = {'R', 'x', 'A', 'A', 'A'};
const byte txAddress[5] = {'T', 'X', 'a', 'a', 'a'};

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

char dataReceived[10]; // must match dataToSend on the Pro Mini
byte replyData[32] = {0}; // the values to be sent to the Pro Mini; 32 bytes max for NRF24L01

bool newData = false;
bool triggerState = false;
bool radioPowerState = false;
bool sendingSD = false;
int reedState;
int ledState = LOW;
int blinkInterval = 1000; // start off blinking every second

// Initialize SD card variables
#define filename "filename.csv"
SdFat sd; // Create the objects to talk to the SD card
File file; // Construct File for SD card
const uint8_t chipSelect = 4; // Adalogger microSD card chip select pin
bool newSdData = true;

unsigned long previousMillisTx;
unsigned long txInterval = 10; // transmit data once per X ms
unsigned long previousMillisReed = 0;
unsigned long previousMillisBlink = 0;
unsigned long powerInterval = 3000000; //interval for power to be supplied to NRF24L01 (in ms) after Reed switch triggered

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(LED_PIN, OUTPUT); // Set LED pin to output mode
  pinMode(REED_PIN, INPUT_PULLUP); // initialize pullup resistor on Reed switch pin
  digitalWrite(REED_PIN, HIGH); // Set Reed switch pin to HIGH

  SPI.begin();
  Serial.println("Initializing SD card...");
  if (!sd.begin(chipSelect, SD_SCK_MHZ(12))) {
    Serial.println("sd card initialization failed!");
    sd.initErrorHalt(); // not being able to save data is a terminal error in all cases
  }
  else {
    Serial.println("SD card OK");
  }

  Serial.println("SlaveSwapRoles Starting");
  radio.begin();
  Serial.println("Powering down radio");
  radio.powerDown(); // immediately power down the radio until reed switch trigger

}

//====================

void loop() {
  blinkWithoutDelay(); // blink at the interval dictated by dataReceived regardless of reed switch/radio power status
  reedState = digitalRead(REED_PIN);
  if (reedState == LOW) { // if Reed switch is triggered
    reedActivated();
  }
  if (triggerState) { // if Reed switch has been triggered and the Radio is powered up
    if (sendingSD) { // if we're already actively sending SD card data
      sendSdData();
    } else {
      getData();
      showData();
    }
    checkPowerDown();
  }
  //blinkWithoutDelay(); // blink at the interval dictated by dataReceived regardless of reed switch/radio power status
}

//================

void reedActivated() {
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

//================

void sendSdData() {
  unsigned long currentMillisTx = millis();
  if (currentMillisTx - previousMillisTx >= txInterval) {
    send();
    previousMillisTx = millis();
  }
}

//================

void checkPowerDown() {
  unsigned long currentMillisReed = millis(); // get the current time
  if (currentMillisReed - previousMillisReed >= powerInterval) { // if the radio has been powered on and transmitting for longer than the interval
    previousMillisReed = currentMillisReed; // update previousMillisValue
    file.close(); // make sure file is closed before powering down (prevent bricking card)
    while (sd.card() -> isBusy()) {
      ;
    }
    triggerState = false;
    radioPowerState = false;
    radio.powerDown();  // powerDown the radio
  }
}

//================

void getData() {
  radio.startListening();
  if (radio.available()) {
    radio.read( &dataReceived, sizeof(dataReceived) );
    if (isalpha(dataReceived[0])) { // if Tx sent an alpha value
      send();
    } else if (isdigit(dataReceived[0])) { // if Tx sent a digit value
      blinkInterval = atoi(dataReceived); // set blinkInterval to received value
      sendConfirm(); // send confirmation message
    }
    newData = true;
  }
}

//================

void showData() {
  if (!sendingSD) {// if we're not actively transmitting SD card data
    if (newData) {
      Serial.print("Data received: ");
      Serial.println(dataReceived);
      newData = false;
    }
  }
}

//====================

void send() {
  radio.stopListening();
  bool rslt;
  if (newSdData) { // if we still have SD card data to send
    sendingSD = true;
    file.open("filename.csv", FILE_READ); // open the file
    if (file) { // if the file opens
      int bytesRead = file.readBytes(replyData, sizeof(replyData) - 1); // -1 to remove null terminator
      if (bytesRead > 0) {
        rslt = radio.write(&replyData, sizeof(replyData));
      } else { // end of file is reached
        file.close();
        while (sd.card() -> isBusy()) {
          ;
        }
        Serial.println("end of file");
        newSdData = false; // indicate there's no more SD data to send
        sendingSD = false; // indicate we're no longer sending SD data
        byte noMoreReplyData[32] = "NO MORE DATA TO REPORT";
        rslt = radio.write(&noMoreReplyData, sizeof(noMoreReplyData));
      }
    }
  } else {
    byte noNewReplyData[32] = "NO NEW DATA TO REPORT";
    rslt = radio.write(&noNewReplyData, sizeof(noNewReplyData));
  }
  radio.startListening(); // check to acknowledge data transfer worked
  if (rslt) {
    Serial.println("  Acknowledge data received");
  } else {
    Serial.println("  Data transfer failed");
  }
}

//====================

void sendConfirm() { // send the data received back, to confirm the correct value 
  radio.stopListening();
  bool rslt;
  byte confirmData[32];
  for (int i = 0; i < sizeof(dataReceived);  i++) { // iterate to create a 32 byte array from dataReceived
    confirmData[i] = dataReceived[i];
  }
  rslt = radio.write(&confirmData, sizeof(confirmData));
  radio.startListening(); // check to acknowledge data transfer worked
  if (rslt) {
    Serial.println("  Acknowledge data received");
  } else {
    Serial.println("  Data transfer failed");
  }
}

//====================

void blinkWithoutDelay() {
  unsigned long currentMillisBlink = millis();
  if (currentMillisBlink - previousMillisBlink >= blinkInterval) {
    previousMillisBlink = currentMillisBlink; // update previousMillis value
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(LED_PIN, ledState);
  }
}

//=================

void setupRadio() {
  radio.begin();
  radio.setDataRate(RF24_250KBPS); // slower data rate = higher resistance to noise
  radio.setPALevel(RF24_PA_LOW);
  radio.setCRCLength(RF24_CRC_16); // set CRC length to 16-bit to ensure data quality
  radio.openWritingPipe(txAddress); // NB these are swapped compared to the master
  radio.openReadingPipe(1, rxAddress);
  radio.setRetries(5, 15); // delay(default is 5, max is 15), count(default/max is 15);
  //radio.startListening();
}
