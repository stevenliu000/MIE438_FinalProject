#include <SoftwareSerial.h>
#include "AlphaBot.h"
#include <Servo.h>
#include <Wire.h> 
#include <Adafruit_L3GD20.h>

AlphaBot Car1 = AlphaBot();
Servo myservo; 

// Pin Assignment--------------------------------------------------------------------
SoftwareSerial HM10(0,1); // RX, TX 
int leftWheel = 2, rightWheel = 3;
int ECHO = 12, TRIG = 11;
int SERVO = 10;
#define GYRO_CS A1 // labeled CS
#define GYRO_DO A0 // labeled SA0
#define GYRO_DI A4  // labeled SDA
#define GYRO_CLK A5 // labeled SCL
Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
//--------------------------------------------------------------------------------

//Global Variables-------------------------------------------------------------
int bluetoothInfo;

//PID Control
double leftDistance = 0, rightDistance = 0;
int currLeft, currRight;

//ultrasonic sensor
double ultrasonicDistance = 0;

//servo motor
int servoAngle = 0;

//gyro sensor
double gyroAngle = 0;


//------------------------------------------------------------------------------


// Software setup-------------------------------------------------------------
void UltrasonicConfig()
{
  pinMode(ECHO, INPUT);                   // Define the ultrasonic echo input pin
  pinMode(TRIG, OUTPUT);                  // Define the ultrasonic trigger input pin
}

void setup() {
  myservo.attach(SERVO,800,2700);
  UltrasonicConfig();
  Serial.begin(9600);
  HM10.begin(9600);
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
  Car1.SetSpeed(100);       //Speed:0 - 255
}
//-----------------------------------------------------------------------------------


//function definition-------------------------------------------------------------
bool bluetoothReadCommand(char command);
double updateDistance();
bool wallFollowing();
bool straightFollowing();
void ServoTo90();
bool ServoToTargetAngle(int angle);
void ultrasonicDistanceUpdate();
int AngleSweep();
inline void GyroToZero();
void GyroAngleUpdate(double parameter);
//------------------------------------------------------------------------------------


//main loop-----------------------------------------------------------------------
void loop() { 
  //state 1 Waiting for Start Command 
  ServoTo90();
  ServoToTargetAngle(20);
  bluetoothReadCommand('s');

  //int angle = AngleSweep();
  //Serial.println(angle);
  //ServoToTargetAngle(angle);
  //bluetoothReadCommand('s');
  wallFollowing();
  //ultrasonicDistanceUpdate();
  //Serial.println(ultrasonicDistance);
}
//------------------------------------------------------------------------------------

//Helper functions----------------------------------------------------------------------------------
int AngleSweep() {
  int pos;
  double distanceArray[181];
  double startAngle = 181;
  double endAngle = -1;
  
  ServoToTargetAngle(0);
  for (pos = 0; pos <= 180; pos += 1) { 
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    ultrasonicDistanceUpdate();
    distanceArray[pos] = ultrasonicDistance;
    servoAngle = pos;
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  ServoToTargetAngle(90);
  for (pos = 0; pos<=180;pos++){
    Serial.println(distanceArray[pos]);
  }
  for(pos = 1; pos<= 180; pos+= 1){
    if (distanceArray[pos]-distanceArray[pos-1]>40){
      endAngle = pos;
    }
    else if (distanceArray[pos]-distanceArray[pos-1]<-40){
      startAngle = pos;
    }
  }
  Serial.println(endAngle);
   Serial.println(startAngle);
    return (endAngle+startAngle)/2;
}

bool wallFollowing(){
 int flag = 0;
 int i,j;
 double lastError = 0;
 double error = 0;
 double control;
 double kp=1,ki=0,kd=0;
 double integral, derivative;
 double desiredDistance = 15;

 delay(1000);
 Car1.MotorRun(120,150);       //Car turn right for 1s; left motor speed:250,right motor speed:0
 for (int m = 0;m<200 ;m++){
  ultrasonicDistanceUpdate();
  error = ultrasonicDistance - desiredDistance;
  integral = (error+lastError)/2;
  derivative = error - lastError;
  control = error*kp + integral*ki + derivative*kd;
  lastError = error;
  control = min(control,50);
  Serial.println(control);
  Car1.MotorRun(180+control,100-control);       //Car turn right for 1s; left motor speed:250,right motor speed:
  delay(100);
 }
  Car1.Brake();
}

bool straightFollowing(){
 int flag = 0;
 int i,j;
 double lastError = 0;
 double Error = 0;
 double Control;
 double kp=1,ki=0.01,kd=1;
 double integral, derivative;
 
 delay(1000);
 currLeft = digitalRead(leftWheel);
 currRight = digitalRead(rightWheel);
 Car1.MotorRun(120,120);       //Car turn right for 1s; left motor speed:250,right motor speed:0
 delay(1000);
 for (int m = 0;m<80 ;m++){  
  for (int t = 0;t<100;t++){
    delay(1);
    updateDistance();
    }
  Error = leftDistance*5-rightDistance*5;
  integral = (Error+lastError)/2;
  derivative = Error - lastError;
  Control = Error*kp + integral*ki + derivative*kd;
  lastError = Error;
  Car1.MotorRun(120-Control,120+Control);       //Car turn right for 1s; left motor speed:250,right motor speed:
 }
  Serial.println(leftDistance);
  Serial.println(rightDistance);
  Car1.Brake();
  delay(5000);
  leftDistance = 0;
  rightDistance = 0;
}

inline void GyroToZero(){
   gyroAngle = 0;
}

void GyroAngleUpdate(double parameter){
  gyro.read();
  gyroAngle += (parameter)*((int)gyro.data.z-1);
  delay(100);
}

void ultrasonicDistanceUpdate()                      // Measure the distance 
{
  //update the distance read of ultrasonic sensor
  digitalWrite(TRIG, LOW);                // set trig pin low 2μs
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);               // set trig pin 10μs , at last 10us 
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);                // set trig pin low
  ultrasonicDistance = pulseIn(ECHO, HIGH) / 58;  // Read echo pin high level time(us)
  if (ultrasonicDistance>110) ultrasonicDistance = 110;
  //ultrasonicDistance = ultrasonicDistance / 58;                
  //Y m =（X s * 344）/ 2; 
  //X s =（ 2 * Y m）/ 344;
  //X s = 0.0058 * Y m;
  //cm = us /58    
}  

bool bluetoothReadCommand(char command){
  // wait for user to press a certain button on screen
  int c;
  Serial.write("Press: ");
  Serial.write(command);
  Serial.write("\n");
  while(1){
   if (HM10.available()) {
      c = HM10.read(); 
    if (c == command)
    {
      Serial.write("Command Accepted!\n");
      bluetoothInfo = c;
      break;
    }
    else if (c != bluetoothInfo){
      Serial.write("Command not Accepted!\n");
      bluetoothInfo = c;
      Serial.write("Press: ");
      Serial.write(command);
      Serial.write("\n");
    }
   }
  }
}

void ServoTo90(){
  for (int i; i <=15; i++){
    myservo.write(90);
    delay(100);
  }
  servoAngle = 90;
}

bool ServoToTargetAngle(int angle){
  int pos;
  if (servoAngle <= angle){
    for (pos = servoAngle; pos <= angle; pos += 1) {
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    servoAngle = angle;
  }
  else{
    for (pos = servoAngle; pos >= angle; pos -= 1) {
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    servoAngle = angle;
  }
}

double updateDistance(){
  int i;
  i = digitalRead(leftWheel); //读取码盘状态（0或1）
  if(i != currLeft){ //每次状态发生跳变的时候，即和初始状态j不同时，码盘读数增加1
    leftDistance += 1;
    currLeft = i;
  }
   i = digitalRead(rightWheel); //读取码盘状态（0或1）
  if(i != currRight){ //每次状态发生跳变的时候，即和初始状态j不同时，码盘读数增加1
    rightDistance += 1;
    currRight = i;
  }
}
//----------------------------------------------------------------------------------

