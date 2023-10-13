// Swap roles PRO MINI

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define CE_PIN   8
#define CSN_PIN 10

const byte rxAddress[5] = {'R', 'x', 'A', 'A', 'A'};
const byte txAddress[5] = {'T', 'X', 'a', 'a', 'a'};


RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

char dataToSend[10];
char txNum = '0';
byte dataReceived[32]; // to hold the data from the slave - must match replyData[] in the slave
bool newData = false;
bool receiveSdData = false;

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 10; // send once per 10 ms

//============

void setup() {

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  Serial.println("MasterSwapRoles Starting");

  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.setPALevel(RF24_PA_LOW);
  radio.setCRCLength(RF24_CRC_16); // set CRC length to 16-bit to ensure data quality
  radio.openWritingPipe(rxAddress);
  radio.openReadingPipe(1, txAddress);
  radio.setRetries(3, 5); // delay, count
  //send(); // to get things started
  prevMillis = millis(); // set clock
  Serial.println("Enter a new blink interval");
}

//=============

void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= txIntervalMillis) {
    if (receiveSdData) { // if we are actively receiving SD Data
      prevMillis = millis(); // skip ahead to get data
    } else {
      if (Serial.available() > 0) {
        Serial.readBytesUntil('\n', dataToSend, sizeof(dataToSend));
        Serial.println(dataToSend);
        send();
        if (isalpha(dataToSend[0])) { // if sent an alpha value
          receiveSdData == true; // flag to listen to receive SD data!
        }
        for (int i = 0; i < sizeof(dataToSend) / sizeof(dataToSend[0]); i++) {
          dataToSend[i] = 0; //reset dataToSend
        }
        Serial.println("Enter a new blink interval");
      }
      prevMillis = millis();
    }
  }
  getData();
  showData();
}


//====================

void send() {
  radio.stopListening();
  bool rslt;
  rslt = radio.write(&dataToSend, sizeof(dataToSend));
  radio.startListening();
  Serial.print("Data Sent: ");
  Serial.print(dataToSend);
  if (rslt) {
    Serial.println("  Acknowledge received");
  }
  else {
    Serial.println("  Tx failed");
  }
}

//================

void getData() {
  if (radio.available()) {
    radio.read(&dataReceived, sizeof(dataReceived));
    newData = true;
  }
}

//================

void showData() {
  if (newData == true) {
    Serial.print("Data received: ");
    Serial.write(dataReceived, strlen(dataReceived));
    Serial.println("");
    newData = false;
  }
}
