#include <Wire.h>
#include <MPU6050.h>

// MPU6050 object
MPU6050 mpu;

// Motor pins (L298N)
#define IN1 5
#define IN2 6
#define IN3 9
#define IN4 10
#define ENA 3
#define ENB 4

// PID constants — tune as needed
float Kp = 0.5, Ki = 0.05, Kd = 0.1;
float previousError = 0, integral = 0;

// Base motor speed
int baseSpeed = 180; // range: 0–255

// Control mode
char currentCommand = 'S';

void setup() {
  Serial.begin(9600);

  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Motor pins setup
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  stopMotors();
  Serial.println("Ready.");
}

void loop() {
  // Read incoming command from Pi
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R' || cmd == 'S') {
      currentCommand = cmd;
    }
  }

  if (currentCommand == 'S') {
    stopMotors();
    return;
  }

  // Get yaw rate (gyro Z)
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  float yawRate = gz / 131.0; // in degrees/sec

  // Compute PID correction
  float targetYaw = 0.0; // desired yaw = 0 (go straight)
  float error = targetYaw - yawRate;
  integral += error;
  float derivative = error - previousError;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  int speedLeft = constrain(baseSpeed + output, 0, 255);
  int speedRight = constrain(baseSpeed - output, 0, 255);

  // Handle motor direction
  switch (currentCommand) {
    case 'F':
      moveForward(speedLeft, speedRight);
      break;
    case 'B':
      moveBackward(speedLeft, speedRight);
      break;
    case 'L':
      turnLeft();
      break;
    case 'R':
      turnRight();
      break;
  }

  delay(30); // control loop delay
}

void moveForward(int leftSpeed, int rightSpeed) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);   // Left forward
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);   // Right forward
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right backward
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void turnLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left backward
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);   // Right forward
  analogWrite(ENA, baseSpeed);
  analogWrite(ENB, baseSpeed);
}

void turnRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);   // Left forward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right backward
  analogWrite(ENA, baseSpeed);
  analogWrite(ENB, baseSpeed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
