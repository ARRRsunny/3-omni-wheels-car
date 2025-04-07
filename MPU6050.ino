#include <MPU6050_tockn.h>
#include <SoftwareSerial.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

// Bluetooth Setup
SoftwareSerial Serial_receive(9, 8);  // RX, TX

// Motor Pins (L298N)
#define MOTOR1_EN_PIN 5
#define MOTOR1_IN1_PIN 6
#define MOTOR1_IN2_PIN 7

#define MOTOR2_EN_PIN 3
#define MOTOR2_IN1_PIN 2
#define MOTOR2_IN2_PIN 4

#define MOTOR3_EN_PIN 11
#define MOTOR3_IN2_PIN 12
#define MOTOR3_IN1_PIN 10

#define Trigger_PIN 13

// PID Parameters
double Kp = 1.5, Ki = 0.3, Kd = 0.2;
double pidError, lastError, integral;
double targetAngle = 0;  // Target heading angle

// Motor Control
int baseSpeed = 0;
int v1, v2, v3;
char movement = '0';
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(9600);
  Serial_receive.begin(9600);
  Wire.begin();
  
  // Initialize MPU6050
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);  // Calibrate gyro (keep robot still during startup)

  // Motor Control Pins
  pinMode(MOTOR1_IN1_PIN, OUTPUT);
  pinMode(MOTOR1_IN2_PIN, OUTPUT);
  pinMode(MOTOR1_EN_PIN, OUTPUT);

  pinMode(MOTOR2_IN1_PIN, OUTPUT);
  pinMode(MOTOR2_IN2_PIN, OUTPUT);
  pinMode(MOTOR2_EN_PIN, OUTPUT);

  pinMode(MOTOR3_IN1_PIN, OUTPUT);
  pinMode(MOTOR3_IN2_PIN, OUTPUT);
  pinMode(MOTOR3_EN_PIN, OUTPUT);

  pinMode(Trigger_PIN, OUTPUT);
  
  stopMotors();
  Serial.println("READY");
}

void loop() {
  // Bluetooth handling
  if (Serial_receive.available()) {
    handleBluetooth();
  }

  // Gyro update and PID calculation
  mpu6050.update();
  if(millis() - lastUpdate > 20) {  // Update at ~50Hz
    double currentAngle = mpu6050.getAngleZ();
    calculatePID(currentAngle);
    lastUpdate = millis();
  }

  // Execute movement with PID correction
  executeMovement();
  
  delay(10);
}

void handleBluetooth() {
  int inByte = Serial_receive.read();
  if (strchr("012345678RLKB", inByte)) {
    movement = inByte;
    baseSpeed = (movement == 'R' || movement == 'L') ? 190 : 255;
    // Reset PID when new command arrives
    pidError = 0;
    integral = 0;
    lastError = 0;
  } else {
    movement = '0';
    baseSpeed = 0;
  }
  Serial.println(movement);
}

void calculatePID(double currentAngle) {
  double error = currentAngle - targetAngle;
  integral += error;
  double derivative = error - lastError;
  double pidOutput = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  // Apply correction to motors
  v1 += pidOutput;
  v2 += pidOutput;
  v3 += pidOutput;

  // Constrain speeds
  v1 = constrain(v1, -255, 255);
  v2 = constrain(v2, -255, 255);
  v3 = constrain(v3, -255, 255);
}

void executeMovement() {
  switch(movement) {
    case '0': stopMotors(); break;
    case '1': drive_motor(0,        -baseSpeed, baseSpeed); break;  // Forward-right
    case '2': drive_motor(-baseSpeed, 0,       baseSpeed); break;  // Backward-right
    case '3': hori_move(3); break;                                // Special movement 1
    case '4': drive_motor(-baseSpeed, baseSpeed, 0); break;       // Backward-left
    case '5': drive_motor(0,         baseSpeed, -baseSpeed); break; // Forward-left
    case '6': drive_motor(baseSpeed, 0,        -baseSpeed); break;  // Forward
    case '7': hori_move(7); break;                                // Special movement 2
    case '8': drive_motor(baseSpeed, -baseSpeed, 0); break;       // Backward
    case 'R': drive_motor(baseSpeed, baseSpeed, baseSpeed); break;  // Rotate right
    case 'L': drive_motor(-baseSpeed, -baseSpeed, -baseSpeed); break; // Rotate left
    case 'K': digitalWrite(Trigger_PIN, HIGH); break;
    case 'B': digitalWrite(Trigger_PIN, LOW); break;
    default: stopMotors();
  }
}

void hori_move(int d_movement) {
  switch(d_movement) {
    case 3:  // Diagonal correction
      drive_motor(-250 + v1, 0 + v2, 250 + v3);
      delay(15);
      drive_motor(-250 + v1, 250 + v2, 0 + v3);
      delay(15);
      break;
    case 7:
      drive_motor(250 + v1, -250 + v2, 0 + v3);
      delay(15);
      drive_motor(250 + v1, 0 + v2, -250 + v3);
      delay(15);
      break;
  }
}

void drive_motor(int speed1, int speed2, int speed3) {
  setMotor(speed1, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, MOTOR1_EN_PIN);
  setMotor(speed2, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, MOTOR2_EN_PIN);
  setMotor(speed3, MOTOR3_IN1_PIN, MOTOR3_IN2_PIN, MOTOR3_EN_PIN);
}

void setMotor(int speed, int in1, int in2, int en) {
  speed = constrain(speed, -255, 255);
  digitalWrite(in1, speed > 0 ? HIGH : LOW);
  digitalWrite(in2, speed > 0 ? LOW : HIGH);
  analogWrite(en, abs(speed));
}

void stopMotors() {
  drive_motor(0, 0, 0);
  v1 = v2 = v3 = 0;
}