#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;

const int ENB = 10;
const int IN3 = 6;
const int IN4 = 5;

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set motors forward by default
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void loop() {
  // Read incoming speed command
  if (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    int leftSpeed, rightSpeed;
    if (sscanf(inputString.c_str(), "%d,%d", &leftSpeed, &rightSpeed) == 2) {
      // Read gyro Z-axis (yaw rotation rate)
      int16_t gx, gy, gz;
      mpu.getRotation(&gx, &gy, &gz);

      // Convert to degrees/sec (approx. using sensitivity = 131 LSB/deg/s)
      float yawRate = gz / 131.0;

      // Apply correction factor to smooth turn
      // If yawRate is high (fast turn), slow down motors a bit
      float correctionFactor = constrain(1.0 - abs(yawRate) / 200.0, 0.7, 1.0);

      // Adjust speeds with correction
      leftSpeed = leftSpeed * correctionFactor;
      rightSpeed = rightSpeed * correctionFactor;

      // Limit PWM range
      leftSpeed = constrain(map(leftSpeed, 0, 100, 0, 255), 0, 255);
      rightSpeed = constrain(map(rightSpeed, 0, 100, 0, 255), 0, 255);

      // Set motor speeds
      analogWrite(ENA, leftSpeed);
      analogWrite(ENB, rightSpeed);

      // Debug print
      Serial.print("Gyro Z: "); Serial.print(yawRate);
      Serial.print(" | Left: "); Serial.print(leftSpeed);
      Serial.print(" | Right: "); Serial.println(rightSpeed);
    }

    inputString = "";
    stringComplete = false;
  }
}
