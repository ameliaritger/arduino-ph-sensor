// SlaveSwapRoles

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define CE_PIN   6
#define CSN_PIN 10

const byte slaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};
const byte masterAddress[5] = {'T', 'X', 'a', 'a', 'a'};

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

char dataReceived[10]; // must match dataToSend in master
int16_t replyData[2] = {109, -4000}; // the two values to be sent to the master
bool newData = false;
bool optionA = false;

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 1000; // send once per second


void setup() {

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("SlaveSwapRoles Starting");

  radio.begin();
  radio.setDataRate( RF24_250KBPS );

  radio.openWritingPipe(masterAddress); // NB these are swapped compared to the master
  radio.openReadingPipe(1, slaveAddress);

  radio.setRetries(5, 5); // delay, count
  radio.startListening();

}

//====================

void loop() {
  getData();
  showData();
  send();
}

//====================

void send() {
  if (newData == true) {
    radio.stopListening();
    bool rslt;
    if (optionA) {
      int16_t replyDataAlt[2] = {120, -4010}; // the two values to be sent to the master
      rslt = radio.write( &replyDataAlt, sizeof(replyDataAlt) );
      optionA = false;
    } else {
      rslt = radio.write( &replyData, sizeof(replyData) );
      optionA = true;
    }
    radio.startListening();

    Serial.print("Reply Sent ");
    Serial.print(replyData[0]);
    Serial.print(", ");
    Serial.println(replyData[1]);

    if (rslt) {
      Serial.println("Acknowledge Received");
      updateReplyData();
    }
    else {
      Serial.println("Tx failed");
    }
    Serial.println();
    newData = false;
  }
}

//================

void getData() {

  if ( radio.available() ) {
    radio.read( &dataReceived, sizeof(dataReceived) );
    newData = true;
  }
}

//================

void showData() {
  if (newData == true) {
    Serial.print("Data received ");
    Serial.println(dataReceived);
  }
}

//================

void updateReplyData() {
  replyData[0] -= 1;
  replyData[1] -= 1;
  if (replyData[0] < 100) {
    replyData[0] = 109;
  }
  if (replyData[1] < -4009) {
    replyData[1] = -4000;
  }
}
