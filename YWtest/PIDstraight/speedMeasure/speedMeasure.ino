#include "AlphaBot.h"

AlphaBot Car1 = AlphaBot();
int leftWheel = 2, rightWheel = 3;
int flag = 0;
int i,j;
int n[100];
double s;
int num = 0;
int currLeft, currRight;
double leftDistance = 0, rightDistance = 0;
double lastError = 0;
double Error = 0;
double Control;
double kp=1,ki=0.01,kd=1;
double integral, derivative;

double updateDistance();

void setup() {
  Serial.begin(9600);
  Car1.SetSpeed(100);       //Speed:0 - 255
}

void loop() {
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
  Car1.MotorRun(120-Control,120+Control);       //Car turn right for 1s; left motor speed:250,right motor speed:
 }
  Serial.println(leftDistance);
  Serial.println(rightDistance);
  Car1.Brake();
  delay(5000);
  leftDistance = 0;
  rightDistance = 0;
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

