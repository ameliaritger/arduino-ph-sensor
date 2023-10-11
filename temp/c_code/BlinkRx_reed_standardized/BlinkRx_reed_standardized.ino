// SimpleRx - the slave or the receiver

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Define NRF24L01 pins
#define CE_PIN 6  // 6 on adalogger, 8 on pro mini
#define CSN_PIN 10
#define REED_PIN 12

int radioData; // must be the same as on Tx
unsigned long previousMillisRadio = 0;
unsigned long previousMillisReed = 0;
unsigned long transmitInterval = 20000; //interval for power to be supplied (ms)
int ledState;
int reedState;
bool triggerState;
bool radioPowerState;
bool newData;

const byte thisSlaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

//===========

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("SimpleRx Starting");
  pinMode(LED_BUILTIN, OUTPUT); // Set LED pin to output mode

  pinMode(REED_PIN, INPUT_PULLUP);
  digitalWrite(REED_PIN, HIGH); // Set Reed pin
  radioData = 1000;
  radio.begin(); //initialize the radio (so it's not floating)
  radio.powerDown(); // immediately power down the radio until reed switch trigger
  triggerState = false;
  radioPowerState = false;
  newData = false;
}

//=============

void loop() {
  reedState = digitalRead(REED_PIN);
  if (reedState == LOW) {
    Serial.println("switch activated");
    if (!radioPowerState) { //if this is the first time the Radio is turned on
      radio.powerUp();
      delay(10); // RF24 documentation: "this will take up to 5ms for maximum compatibility"
      setupRadio();
      radioPowerState = true; //variable to only powerUp the radio once
      triggerState = true; // variable to indicate the reed switch has been triggered
    }
    previousMillisReed = millis(); //store current time
  }
  if (triggerState == true ) {
    getData();
    showData();
    unsigned long currentMillisReed = millis(); // get the current time
    if (currentMillisReed - previousMillisReed > transmitInterval) { // if the radio has been powered on and transmitting for longer than the set transmit interval
      digitalWrite(LED_BUILTIN , LOW); // turn off the LED
      triggerState = false;
      radioPowerState = false;
      radio.powerDown();  // powerDown the radio
    }
  }
}

//==============

void setupRadio() {
  radio.begin();
  radio.setDataRate(RF24_250KBPS); // Can increase to 1 Mbps or 2 Mbps
  radio.setPALevel(RF24_PA_LOW); // prevent power supply related problems (RF24_PA_MAX is default)
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.startListening();
}

//==============

void getData() {
  if (radio.available()) {
    radio.read(&radioData, sizeof(radioData));
    newData = true;
  }
}

//==============

void showData() {
  if (newData == true) {
    Serial.print("Data received ");
    Serial.println(radioData);
  }
  unsigned long currentMillisRadio = millis();
  if (currentMillisRadio - previousMillisRadio >= radioData) {
    previousMillisRadio = currentMillisRadio; // update previousMillis value
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(LED_BUILTIN, ledState);
  }
  newData = false;
}
