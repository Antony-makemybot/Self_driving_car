// code for nano with speed groover module 
//without MPU6050 sensor
// === Function Prototypes ===
void countLeft();
void countRight();
void driveForward(int speed);

// === Motor Control Pins (L298N) ===
const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;

const int ENB = 10;
const int IN3 = 8;
const int IN4 = 9;

// === Encoder Sensor Pins ===
const int leftSensorPin = 2;   // ✅ Must be D2
const int rightSensorPin = 3;  // ✅ Must be D3

// === Encoder Variables ===
volatile long leftTicks = 0;
volatile long rightTicks = 0;

unsigned long lastMillis = 0;
float leftRPM = 0;
float rightRPM = 0;

// === Disk Specs ===
const int pulsesPerRevolution = 20;

void setup() {
  Serial.begin(9600);

  // Motor pin setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Sensor pin setup
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(leftSensorPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightSensorPin), countRight, RISING);

  // Start motors
  driveForward(120);  // start with lower speed for testing
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= 1000) {
    noInterrupts();
    long left = leftTicks;
    long right = rightTicks;
    leftTicks = 0;
    rightTicks = 0;
    interrupts();

    leftRPM = (left / (float)pulsesPerRevolution) * 60.0;
    rightRPM = (right / (float)pulsesPerRevolution) * 60.0;

    Serial.print("Left RPM: ");
    Serial.print(leftRPM);
    Serial.print(" | Right RPM: ");
    Serial.println(rightRPM);

    lastMillis = currentMillis;
  }
}

// === Interrupt Handlers ===
void countLeft() {
  leftTicks++;
}

void countRight() {
  rightTicks++;
}

// === Motor Control ===
void driveForward(int speed) {
  // Left motor
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  // Right motor
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}
