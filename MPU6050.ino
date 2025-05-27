#include <SoftwareSerial.h>
#include <Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

char movement = '0';
int XValue = 0;
int YValue = 0;
int setang = 0;
int angle_bias = 0;
int speed_offset = 0;
bool Butstate = false;

const int max_angle_bias = 50;
const int base_speed = 225;
const int rotate_speed = 180;
const int adjustable_range = 255 - base_speed; 

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

MPU6050 mpu6050(Wire);
SoftwareSerial Serial_receive(9,8);  //RX,TX



unsigned long previousMillis = 0;
const long interval = 20; // 20ms interval

void drive_motor(int v_1, int v_2, int v_3) {
  int V_1 = v_1-speed_offset;
  int V_2 = v_2-speed_offset;
  int V_3 = v_3-speed_offset;
  setMotorPinState(V_1, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);       //setting of the orientation 
  setMotorPinState(V_2, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
  setMotorPinState(V_3, MOTOR3_IN1_PIN, MOTOR3_IN2_PIN);
  /*
  Serial.print(V_1);
  Serial.print(' ');
  Serial.print(V_2);
  Serial.print(' ');
  Serial.print(V_3);
  */
  analogWrite(MOTOR1_EN_PIN, abs(V_1));             //setting speed
  analogWrite(MOTOR2_EN_PIN, abs(V_2));
  analogWrite(MOTOR3_EN_PIN, abs(V_3));
}

void case_sw(char M) {                    
  switch (M) {
    case '0':
      drive_motor(0, 0, 0);
      break;

    case '1':
      drive_motor(0, -base_speed, base_speed);
      break;

    case '2':
      drive_motor(-base_speed, 0, base_speed);
      break;

    case '3':
      drive_motor(-base_speed, 0, base_speed);
      delay(15);
      drive_motor(-base_speed, base_speed, 0);
      delay(15);
      break;

    case '4':
      drive_motor(-base_speed, base_speed, 0);
      break;

    case '5':
      drive_motor(0, base_speed, -base_speed);
      break;

    case '6':
      drive_motor(base_speed, 0, -base_speed);
      break;

    case '7':
      drive_motor(base_speed, -base_speed, 0);
      delay(15);
      drive_motor(base_speed, 0, -base_speed);
      delay(15);
      break;

    case '8':
      drive_motor(base_speed, -base_speed, 0);
      break;
    case 'R':
      drive_motor(rotate_speed, rotate_speed, rotate_speed);
      break;

    case 'L':
      drive_motor(-rotate_speed, -rotate_speed, -rotate_speed);
      break;
    case 'K':
      digitalWrite(Trigger_PIN, HIGH);
      Butstate = true;
      break;
    case 'B':
      digitalWrite(Trigger_PIN, LOW);
      Butstate = false;
      break;
    default:
      drive_motor(0, 0, 0);
  }
}



void setMotorPinState(int value, int in1Pin, int in2Pin) {            //function of setting the orientation
  digitalWrite(in1Pin, value >= 0 ? 0 : value < 0 ? 1:0);
  digitalWrite(in2Pin, value > 0 ? 1 : value <= 0 ? 0:0);
}

void setup() {
  Serial.begin(9600);  // Serial monitor

  Serial_receive.begin(9600);  // UART2, baud rate: 115200,BT


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

  analogWrite(MOTOR1_EN_PIN, 0);
  digitalWrite(MOTOR1_IN1_PIN, 0);  
  digitalWrite(MOTOR1_IN2_PIN, 0);
  
  analogWrite(MOTOR2_EN_PIN, 0);
  digitalWrite(MOTOR2_IN1_PIN, 0); 
  digitalWrite(MOTOR2_IN2_PIN, 0);

  analogWrite(MOTOR3_EN_PIN, 0);
  digitalWrite(MOTOR3_IN1_PIN, 0);  
  digitalWrite(MOTOR3_IN2_PIN, 0);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  Serial.println("READY");
}

void cal_bias_offset(){
  speed_offset = map(constrain(angle_bias,-max_angle_bias,max_angle_bias),-max_angle_bias,max_angle_bias,-adjustable_range,adjustable_range);
}

void checkangle(){
  mpu6050.update();
  int angle = mpu6050.getAngleZ();
  if(Butstate==true){
    angle_bias = setang - angle;
  } else if(Butstate==false){
    angle_bias = 0;
    setang = angle;
  }
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (Serial_receive.available()) {
      int inByte = Serial_receive.read();
      if (strchr("012345678RLKB", inByte)) {
        movement = inByte;
        checkangle();
        cal_bias_offset();
      } else {
        movement = '0';
      }
    }

    /*
    Serial.println(' ');
    Serial.print(movement);
    Serial.print(' ');
    Serial.print(Butstate);
    Serial.print(' ');
    Serial.print(angle_bias);
    Serial.print(' ');
    Serial.print(speed_offset);
    Serial.print(' ');
    */
    case_sw(movement);
  }
}