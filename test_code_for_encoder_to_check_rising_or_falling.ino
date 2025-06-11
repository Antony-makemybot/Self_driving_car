const int sensorPin = 2;  // Encoder OUT connected to D2
int lastState = LOW;

void setup() {
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  Serial.println("Starting sensor state monitor...");
}

void loop() {
  int currentState = digitalRead(sensorPin);

  if (currentState != lastState) {
    if (currentState == HIGH) {
      Serial.println("RISING edge detected (LOW → HIGH)");
    } else {
      Serial.println("FALLING edge detected (HIGH → LOW)");
    }
    lastState = currentState;
    delay(50); // debounce; adjust if needed
  }
}
