🧪 Goal
We’ll:

Read the digital state of the sensor pin (HIGH/LOW)

Print state changes to the Serial Monitor

Spin the wheel slowly and observe the output

Decide whether to use RISING or FALLING for clean, one-pulse-per-slot counting

🔧 Simple Test Code
Connect your encoder sensor’s OUT pin to D2 on your Nano (or Uno), then upload this sketch:

cpp
Copy
Edit
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
🛠 How to Use
Open the Serial Monitor

Slowly rotate the motor shaft or slotted disk by hand

Watch the output:

If you see:

python-repl
Copy
Edit
RISING edge detected
FALLING edge detected
RISING edge detected
...
That means your signal is working properly.

Count how many RISING and FALLING you get per full rotation.

📌 What to Look For
If RISING edges are clean and spaced, use:

cpp
Copy
Edit
attachInterrupt(digitalPinToInterrupt(2), countLeft, RISING);
If FALLING edges are more consistent, use:

cpp
Copy
Edit
attachInterrupt(digitalPinToInterrupt(2), countLeft, FALLING);
Don’t use both unless you want to count twice as many ticks per revolution (which is rare and not necessary for your use case).

🧼 Pro Tips
Use manual rotation or set PWM speed low (e.g., analogWrite(pin, 80))

If the output is too noisy (switching rapidly), you may need to:

Add a 10k pull-down resistor on the signal pin

Add a delay(5–10ms) to debounce in test code only (not in actual ISR)
