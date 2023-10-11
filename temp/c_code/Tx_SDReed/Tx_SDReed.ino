//the master or the transmitter
//only the ProMini works!
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 8
#define CSN_PIN 10
#define LED_PIN 5

// Set up Radio
const byte radioAddress[5] = {'R', 'x', 'A', 'A', 'A'};
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

// Initialize variables
unsigned int dataToSend = 1000; // send initial data value to transmit; must match dataReceived in Rx
String serialInput;
byte ackData[32]; // to hold the values coming from Rx; 32 bytes max for NRF24L01
bool newData = false; // to indicate if there is new data from Rx

int ledState = LOW; // ledState used to set the LED
unsigned long previousMillis;
unsigned long txIntervalMillis = 1000; // transmit data once per second
unsigned long previousMillisBlink = 0;
unsigned long blinkInterval = 1000;

//===============

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  pinMode(LED_PIN, OUTPUT); // Set LED pin to output mode

  Serial.println("Radio Starting...");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  //radio.setPALevel(RF24_PA_LOW);
  radio.enableAckPayload();
  radio.setRetries(5, 5); // delay, count
  radio.openWritingPipe(radioAddress);

  Serial.println("Enter a blink interval (in ms):");
}

//=============

void loop() {
  unsigned long currentMillis = millis();
  if (Serial.available() > 0) { // if there is new data to input from the Serial
    serialInput = Serial.readString();
    dataToSend = serialInput.toInt();
  }
  if (currentMillis - previousMillis >= txIntervalMillis) {
    send();
    previousMillis = millis();
    Serial.println("Enter a blink interval (in ms):");
  }
  showData();
}

//================

void send() {
  bool rslt;
  rslt = radio.write(&dataToSend, sizeof(dataToSend)); // Always use sizeof() as it gives the size as the number of bytes.
  Serial.print("Data Sent: ");
  Serial.println(dataToSend);
  if (rslt) {
    if (radio.isAckPayloadAvailable()) {
      txIntervalMillis = 5; // speed up radio communications to receive SD card data
      radio.read(&ackData, sizeof(ackData));
      newData = true;
    }
    else {
      Serial.println("  Acknowledge but no data ");
      txIntervalMillis = 1000; // slow down radio communications if no new data to receive
    }
  }
  else {
    Serial.println("  Tx failed");
  }
}

//=================

void showData() {
  if (newData == true) {
    Serial.print("Data received: ");
    Serial.write(ackData, strlen(ackData));
    Serial.println("");
    newData = false;
    blinkWithoutDelay(); // blink to acknowledge end of loop
  }
}

//=================

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
