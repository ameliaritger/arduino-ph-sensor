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
const unsigned int dataToSend = 1000; // send initial data value to transmit; must match dataReceived in Rx
String serialInput;
byte ackData[32]; // to hold the values coming from Rx; 32 bytes max for NRF24L01
bool newData = false; // to indicate if there is new data from Rx

unsigned long previousMillis;
unsigned long txIntervalMillis = 1000; // transmit data once per second

//===============

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  pinMode(LED_BUILTIN, OUTPUT); // Set LED pin to output mode

  Serial.println("Radio Starting...");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  //radio.setPALevel(RF24_PA_LOW); // or MIN
  radio.enableAckPayload();
  radio.setRetries(5, 5); // delay, count
  radio.openWritingPipe(radioAddress);

  Serial.println("Enter a blink interval (in ms):");
}

//=============

void loop() {
  unsigned long currentMillis = millis();
  if (Serial.available() > 0) {
    serialInput = Serial.readString();
    dataToSend = serialInput.toInt();
    if (currentMillis - previousMillis >= txIntervalMillis) {
      send();
      previousMillis = millis();
      Serial.println("Enter a blink interval (in ms):");
    }
    showData();
  }
}

//================

void send() {
  bool rslt;
  rslt = radio.write(&dataToSend, sizeof(dataToSend)); // Always use sizeof() as it gives the size as the number of bytes.
  Serial.print("Data Sent: ");
  Serial.println(dataToSend);
  if (rslt) {
    if (radio.isAckPayloadAvailable()) {
      txIntervalMillis = 5; // bump up the speed of radio communication to receive data
      radio.read(&ackData, sizeof(ackData));
      newData = true;
    }
    else {
      Serial.println("  Acknowledge but no data ");
      txIntervalMillis = 1000; // slow down radio communications if no new data to receive
    }
    updateMessage();
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
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }
}

//================

void updateMessage() {
  // so you can see that new data is being sent
  txNum += 1;
  if (txNum > '9') {
    txNum = '0';
  }
  dataToSend[8] = txNum;
}
