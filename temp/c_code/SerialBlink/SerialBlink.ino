int ledState = LOW;

int interval;
String user_data;
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  interval = 1000;
  Serial.println("Enter a new blink interval");
}
void loop() {
  if (Serial.available() > 0) {
    user_data = Serial.readString();
    interval = user_data.toInt();
    Serial.println(interval);
    Serial.println("here!");
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (ledState == LOW) {
      ledState = HIGH;
      Serial.println("LED ON");
    } else {
      ledState = LOW;
      Serial.println("LED OFF");
    }
    digitalWrite(LED_BUILTIN, ledState);
    Serial.println("here?");
  }
}
