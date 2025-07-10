#include <SoftwareSerial.h>
#include <Servo.h>

char movement = '0';

#define MOTOR1_EN_PIN 5
#define MOTOR1_IN1_PIN 6
#define MOTOR1_IN2_PIN 7         

#define MOTOR2_EN_PIN 3
#define MOTOR2_IN1_PIN 2
#define MOTOR2_IN2_PIN 4

#define MOTOR3_EN_PIN 11
#define MOTOR3_IN1_PIN 13                    
#define MOTOR3_IN2_PIN 12

#define PWM_PIN 10

int PWM = 0;

Servo myservo;

SoftwareSerial Serial_receive(9, 8);  // RX, TX

unsigned long previousMillis = 0;
const long interval = 20; // 20ms interval

void drive_motor(int v_1, int v_2, int v_3) {
  setMotorPinState(v_1, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
  setMotorPinState(v_2, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
  setMotorPinState(v_3, MOTOR3_IN1_PIN, MOTOR3_IN2_PIN);

  analogWrite(MOTOR1_EN_PIN, abs(v_1));
  analogWrite(MOTOR2_EN_PIN, abs(v_2));
  analogWrite(MOTOR3_EN_PIN, abs(v_3));
}

void case_sw(char M) {                    
  switch (M) {
    case '0':
      drive_motor(0, 0, 0);
      break;
    case '1':
      drive_motor(0, -255, 255);
      break;
    case '2':
      drive_motor(-255, 0, 255);
      break;
    case '3':
      hori_move(3);
      break;
    case '4':
      drive_motor(-255, 255, 0);
      break;
    case '5':
      drive_motor(0, 255, -255);
      break;
    case '6':
      drive_motor(255, 0, -255);
      break;
    case '7':
      hori_move(7);
      break;
    case '8':
      drive_motor(255, -255, 0);
      break;
    case 'R':
      drive_motor(190, 190, 190);
      break;
    case 'L':
      drive_motor(-190, -190, -190);
      break;
    default:
      drive_motor(0, 0, 0);
  }
}

void hori_move(int d_movement) {
  switch (d_movement) {
    case 3:
      drive_motor(-250, 0, 250);
      delay(15);
      drive_motor(-250, 250, 0);
      delay(15);
      break;
    case 7:
      drive_motor(250, -250, 0);
      delay(15);
      drive_motor(250, 0, -250);
      delay(15);
      break;
  }
}

void setMotorPinState(int value, int in1Pin, int in2Pin) {
  digitalWrite(in1Pin, value > 0 ? LOW : HIGH);
  digitalWrite(in2Pin, value > 0 ? HIGH : LOW);
}

void setup() {
  Serial.begin(9600);
  Serial_receive.begin(9600);

  pinMode(MOTOR1_IN1_PIN, OUTPUT);
  pinMode(MOTOR1_IN2_PIN, OUTPUT);
  pinMode(MOTOR1_EN_PIN, OUTPUT);

  pinMode(MOTOR2_IN1_PIN, OUTPUT);
  pinMode(MOTOR2_IN2_PIN, OUTPUT);
  pinMode(MOTOR2_EN_PIN, OUTPUT);
  
  pinMode(MOTOR3_IN1_PIN, OUTPUT);
  pinMode(MOTOR3_IN2_PIN, OUTPUT);
  pinMode(MOTOR3_EN_PIN, OUTPUT);


  analogWrite(MOTOR1_EN_PIN, 0);
  digitalWrite(MOTOR1_IN1_PIN, LOW);  
  digitalWrite(MOTOR1_IN2_PIN, LOW);
  
  analogWrite(MOTOR2_EN_PIN, 0);
  digitalWrite(MOTOR2_IN1_PIN, LOW); 
  digitalWrite(MOTOR2_IN2_PIN, LOW);

  analogWrite(MOTOR3_EN_PIN, 0);
  digitalWrite(MOTOR3_IN1_PIN, LOW);  
  digitalWrite(MOTOR3_IN2_PIN, LOW);

  myservo.attach(PWM_PIN);

  Serial.println("READY");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (Serial_receive.available()) {
      String data = Serial_receive.readStringUntil('\n');
      data.trim();
      int commaIndex = data.indexOf(',');
      if (commaIndex != -1) {
        movement = data.charAt(2);
        String intStr = data.substring(commaIndex + 1);
        PWM = intStr.toInt();
      }
    }
    //Serial.print(movement);
    //Serial.print(" ");
    //Serial.println(PWM);
    myservo.write(PWM);
    case_sw(movement);
  }
}