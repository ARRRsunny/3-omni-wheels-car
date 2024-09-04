#include <SoftwareSerial.h>
#include <Servo.h>


char movement = '0';
int XValue = 0;
int YValue = 0;


#define MOTOR1_EN_PIN 5
#define MOTOR1_IN1_PIN 6
#define MOTOR1_IN2_PIN 7

#define MOTOR2_EN_PIN 3
#define MOTOR2_IN1_PIN 4
#define MOTOR2_IN2_PIN 2

#define MOTOR3_EN_PIN 11                    
#define MOTOR3_IN2_PIN 12
#define MOTOR3_IN1_PIN 10




#define Trigger_PIN 13

SoftwareSerial Serial_receive(9,8);  //RX,TX

int v_1;        //speed of each wheel
int v_2;
int v_3;



 
void drive_motor(int v_1, int v_2, int v_3) {
  setMotorPinState(v_1, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);       //setting of the orientation 
  setMotorPinState(v_2, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
  setMotorPinState(v_3, MOTOR3_IN1_PIN, MOTOR3_IN2_PIN);


  analogWrite(MOTOR1_EN_PIN, abs(v_1));             //setting speed
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
    case 'K':
      digitalWrite(Trigger_PIN, HIGH);
      break;
    case 'B':
      digitalWrite(Trigger_PIN, LOW);
      break;
    default:
      drive_motor(0, 0, 0);
  }
}

void hori_move(int d_movement) {                  //special movement due the 3 wheels
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

void setMotorPinState(int value, int in1Pin, int in2Pin) {            //function of setting the orientation
  digitalWrite(in1Pin, value > 0 ? 0 : value < 0 ? 1: value == 0 ? 0:0);
  digitalWrite(in2Pin, value > 0 ? 1 : value < 0 ? 0: value == 0 ? 0:0);
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




  Serial.println("READY");
}


void loop() {

  if (Serial_receive.available()) {
    int inByte = Serial_receive.read();
    if (inByte == '0' ||inByte == '1' ||inByte == '2'||inByte == '3' ||inByte == '4' ||inByte == '5'||inByte == '6'||inByte == '7'||inByte == '8'||inByte == 'R'||inByte == 'L'||inByte == 'K'||inByte == 'B'){
      movement = inByte;
      
    } else {
      movement = '0';
    }
  }
  Serial.println(movement);
  case_sw(movement);


  delay(20);

}