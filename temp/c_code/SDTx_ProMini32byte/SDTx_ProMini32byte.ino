// SimpleTxAckPayload - the master or the transmitter
//only the ProMini works!
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 8
#define CSN_PIN 10
#define LED_PIN 5

//const int8_t byteSize = 32;

const byte radioAddress[5] = {'R', 'x', 'A', 'A', 'A'};

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

char dataToSend[10] = "Message 0"; // this must match dataReceived in the RX
char txNum = '0';
byte ackData[32]; // to hold the values coming from the slave
bool newData = false;

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 1000; // send once per Xms

//===============

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  Serial.println("Radio Starting...");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  //radio.setPALevel(RF24_PA_LOW); // or MIN
  radio.enableAckPayload();
  radio.setRetries(5, 5); // delay, count
  radio.openWritingPipe(radioAddress);
}

//=============

void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= txIntervalMillis) {
    send();
  }
  showData();
}

//================

void send() {
  bool rslt;
  rslt = radio.write( &dataToSend, sizeof(dataToSend) );
  // Always use sizeof() as it gives the size as the number of bytes.
  // For example if dataToSend was an int sizeof() would correctly return 2
  Serial.print("Data Sent: ");
  Serial.println(dataToSend);
  if (rslt) {
    if (radio.isAckPayloadAvailable()) {
      txIntervalMillis = 5; // bump up the speed of radio communications
      radio.read(&ackData, sizeof(ackData));
      newData = true;
    }
    else {
      Serial.println("  Acknowledge but no data ");
      txIntervalMillis = 1000; // slow down radio communications if no new data to report
    }
    updateMessage();
  }
  else {
    Serial.println("  Tx failed");
  }
  prevMillis = millis();
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