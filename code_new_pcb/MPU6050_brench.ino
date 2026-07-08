#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050_light.h>

const int R_SPEED = 150;
const int MAX_SPEED = 255;
const int HORI_SPEED = 250;
const long INTERVAL = 20; 
const float Kp = 5.0;
const float Ki = 0.09;
const float Kd = 1.0;
const float MAX_INTEGRAL = 100.0;
float targetAngle = 0;
float angleIntegral = 0;
float lastAngleError = 0;


const byte TRIGGER_PIN = 13;
const byte RX_PIN = 9;
const byte TX_PIN = 8;

struct Motor {
  byte en, in1, in2;
};


const Motor M1 = {11, 12, 10};
const Motor M2 = {5, 7, 6};
const Motor M3 = {3, 4, 2};

MPU6050 mpu(Wire);
SoftwareSerial Serial_receive(RX_PIN, TX_PIN);

unsigned long previousMillis = 0;
char movement = '0';
char pre_movement = '0';
bool alternateHori = false;
bool cor_sta = true;


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

constexpr float DT = INTERVAL * 0.001f;

float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

int calculatePIDCorrection(float currentAngle) {
  const float error = normalizeAngle(targetAngle - currentAngle);
  angleIntegral = constrain(angleIntegral + error * DT, -MAX_INTEGRAL, MAX_INTEGRAL);
  const float derivative = (error - lastAngleError) / DT;
  lastAngleError = error;
  const int correction = (int)((Kp * error) + (Ki * angleIntegral) + (Kd * derivative));
  return constrain(correction, -MAX_SPEED, MAX_SPEED);
}

inline void driveCorrection(int correction, int8_t offset1, int8_t offset2, int8_t offset3) {
  drive(
    constrain(correction + offset1 * MAX_SPEED, -MAX_SPEED, MAX_SPEED),
    constrain(correction + offset2 * MAX_SPEED, -MAX_SPEED, MAX_SPEED),
    constrain(correction + offset3 * MAX_SPEED, -MAX_SPEED, MAX_SPEED)
  );
}

void apply_corrected_movement(char cmd) {
  const float currentAngle = mpu.getAngleZ();
  const int correction = cor_sta ? -calculatePIDCorrection(currentAngle) : 0;
  //Serial.println(correction);

  switch (cmd) {
    case '0':
      drive(0, 0, 0);
      break;
    case '1':
      driveCorrection(correction, 0, -1, +1);
      break;
    case '2':
      driveCorrection(correction, -1, 0, +1);
      break;
    case '4':
      driveCorrection(correction, -1, +1, 0);
      break;
    case '5':
      driveCorrection(correction, 0, +1, -1);
      break;
    case '6':
      driveCorrection(correction, +1, 0, -1);
      break;
    case '8':
      driveCorrection(correction, +1, -1, 0);
      break;
    case 'R':
      drive(R_SPEED, R_SPEED, R_SPEED);
      break;
    case 'L':
      drive(-R_SPEED, -R_SPEED, -R_SPEED);
      break;
    case 'K':
      cor_sta = true;
      digitalWrite(TRIGGER_PIN, HIGH);
      break;
    case 'B':
      cor_sta = false;
      digitalWrite(TRIGGER_PIN, LOW);
      break;
    case '3':
      if (alternateHori) {
        drive(-HORI_SPEED, 0, HORI_SPEED);
      } else {
        drive(-HORI_SPEED, HORI_SPEED, 0);
      }
      alternateHori = !alternateHori;
      break;
    case '7':
      if (alternateHori) {
        drive(HORI_SPEED, -HORI_SPEED, 0);
      } else {
        drive(HORI_SPEED, 0, -HORI_SPEED);
      }
      alternateHori = !alternateHori;
      break;
    default:
      drive(0, 0, 0);
      break;
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
  }
  drive(0, 0, 0);
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

        if (strchr("012345678RL", movement)) {
          if (pre_movement != movement) {
            pre_movement = movement;
            targetAngle = mpu.getAngleZ();
            angleIntegral = 0;
            lastAngleError = 0;
          }
        }
      }
    }

    apply_corrected_movement(movement);
  }
}