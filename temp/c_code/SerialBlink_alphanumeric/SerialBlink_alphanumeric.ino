int ledState = LOW;

int interval = 1000;
char user_data[10];
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Enter a new blink interval");
}

void loop() {
  if (Serial.available() > 0) {
    Serial.readBytesUntil('\n', user_data, sizeof(user_data));
    // Check if input is an alpha or numeric
    if (isalpha(user_data[0])) {
      interval = 100;
      Serial.println("input was a character!");
    } else if (isdigit(user_data[0])) {
      interval = atoi(user_data);
      Serial.println("input was a digit!");
    }
  }
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (ledState == LOW) {
      ledState = HIGH;
      //Serial.println("LED ON");
    } else {
      ledState = LOW;
      //Serial.println("LED OFF");
    }
    digitalWrite(LED_BUILTIN, ledState);
  }
}
