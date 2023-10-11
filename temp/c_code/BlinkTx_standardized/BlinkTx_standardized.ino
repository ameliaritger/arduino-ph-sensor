// SimpleTx - the master or the transmitter

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Define NRF24L01 pins
#define CE_PIN 8 // 6 on adalogger, 8 on pro mini
#define CSN_PIN 10

int radioData = 1000; // set initial data value to transmit
String serialInput;
unsigned long previousMillis;
unsigned int transmitInterval = 1000; // transmit data once per second

const byte slaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

void setup() {

  Serial.begin(9600);
  Serial.println("SimpleTx Starting");
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5); // delay, count
  radio.openWritingPipe(slaveAddress); //Set NRF24L01 to transmit mode
  pinMode(LED_BUILTIN, OUTPUT); // Set LED pin to output mode
  Serial.println("Enter a blink interval (in ms):");
}

//====================

void loop() {
  unsigned long currentMillis = millis();
  if (Serial.available() > 0) {
    serialInput = Serial.readString();
    radioData = serialInput.toInt();
    if (currentMillis - previousMillis >= transmitInterval) {
      send();
      previousMillis = millis();
      Serial.println("Enter a blink interval (in ms):");
    }
  }
}

//====================

void send() {

  bool rslt;
  rslt = radio.write(&radioData, sizeof(radioData));
  // Always use sizeof() as it gives the size as the number of bytes.
  // For example if dataToSend was an int sizeof() would correctly return 2

  Serial.print("Data Sent ");
  Serial.println(radioData);
  if (rslt) {
    Serial.println("  Acknowledge received");
  }
  else {
    Serial.println("  Tx failed");
  }
}
