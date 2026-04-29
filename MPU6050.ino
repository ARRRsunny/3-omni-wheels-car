#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

const int R_SPEED = 150;
const int MAX_SPEED = 255;
const int HORI_SPEED = 250;
const long INTERVAL = 20; 

const float Kp = 5.0;
const float Ki = 0.05;
const float Kd = 1.2;
const float DEADZONE = 0.75;
float lastError = 0, integral = 0;
float targetAngle = 0;

// Pin Definitions
const byte TRIGGER_PIN = 13;
const byte RX_PIN = 9;
const byte TX_PIN = 8;

struct Motor {
  byte en, in1, in2;
};


const Motor M1 = {11, 12, 10};
const Motor M2 = {5, 7, 6};
const Motor M3 = {3, 4, 2};

SoftwareSerial Serial_receive(RX_PIN, TX_PIN);

unsigned long previousMillis = 0;
char movement = '0';
bool alternateHori = false;

void setMotor(const Motor& m, int speed) {
  digitalWrite(m.in1, speed > 0 ? LOW : HIGH);
  digitalWrite(m.in2, speed > 0 ? HIGH : LOW);
  analogWrite(m.en, abs(speed));
}

void drive(int v1, int v2, int v3) {
  setMotor(M1, v1);
  setMotor(M2, v2);
  setMotor(M3, v3);
}

void apply_corrected_movement(char cmd) {
  float currentAngle = mpu.getAngleZ();
  float error = targetAngle - currentAngle;


  if (abs(error) < DEADZONE) {
    error = 0;
    integral = 0; 
  }

  integral += error * (INTERVAL / 1000.0);
  float derivative = (error - lastError) / (INTERVAL / 1000.0);
  lastError = error;

  int correction = (int)((error * Kp) + (integral * Ki) + (derivative * Kd));

  
  switch (cmd) {
    case '0': drive(0, 0, 0); break;
    case '1': drive(correction, correction-MAX_SPEED, correction + MAX_SPEED); break;
    case '2': drive(correction-MAX_SPEED, correction, correction +MAX_SPEED); break;
    case '4': drive(correction-MAX_SPEED,correction + MAX_SPEED, correction); break;
    case '5': drive(correction,correction + MAX_SPEED, correction-MAX_SPEED); break;
    case '6': drive(correction+MAX_SPEED, correction, correction-MAX_SPEED); break;
    case '8': drive(correction+MAX_SPEED, correction-MAX_SPEED, correction); break;
    case 'R': drive(R_SPEED, R_SPEED, R_SPEED); break;
    case 'L': drive(-R_SPEED, -R_SPEED, -R_SPEED); break;
    case 'K': digitalWrite(TRIGGER_PIN, HIGH); break;
    case 'B': digitalWrite(TRIGGER_PIN, LOW); break;
    
    case '3': 
      alternateHori ? drive(-HORI_SPEED, 0, HORI_SPEED) : drive(-HORI_SPEED, HORI_SPEED, 0);
      alternateHori = !alternateHori;
      break;
    case '7': 
      alternateHori ? drive(HORI_SPEED, -HORI_SPEED, 0) : drive(HORI_SPEED, 0, -HORI_SPEED);
      alternateHori = !alternateHori;
      break;
      
    default: drive(0, 0, 0); break;
  }
}



void setup() {
  Serial.begin(9600);
  Serial_receive.begin(9600);

  const Motor motors[] = {M1, M2, M3};
  for (int i = 0; i < 3; i++) {
    pinMode(motors[i].en, OUTPUT);
    pinMode(motors[i].in1, OUTPUT);
    pinMode(motors[i].in2, OUTPUT);
    drive(0, 0, 0);
  }
  pinMode(TRIGGER_PIN, OUTPUT);


  Wire.begin();
  byte status = mpu.begin();
  while(status != 0){ }
  delay(1000);
  mpu.calcOffsets();

  
  Serial.println("SYSTEM READY");
}

void loop() {
  mpu.update();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;

    if (Serial_receive.available()) {
      int inByte = Serial_receive.read();

      if (strchr("012345678RLKB", inByte)) {
        movement = inByte;
        targetAngle = mpu.getAngleZ();
      }
    }

    apply_corrected_movement(movement);
  }
}
