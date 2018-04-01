#include <SoftwareSerial.h>
#include "AlphaBot.h"
AlphaBot Car1 = AlphaBot();

// Pin Assignment--------------------------------------------------------------------
SoftwareSerial HM10(0,1); // RX, TX 
int leftWheel = 2, rightWheel = 3;
int ECHO = 12, TRIG = 11;
//--------------------------------------------------------------------------------

//Global Variables-------------------------------------------------------------
int bluetoothInfo;

//PID Control
double leftDistance = 0, rightDistance = 0;
int currLeft, currRight;

//ultrasonic sensor
double ultrasonicDistance = 0;

//------------------------------------------------------------------------------


// Software setup-------------------------------------------------------------
void UltrasonicConfig()
{
  pinMode(ECHO, INPUT);                   // Define the ultrasonic echo input pin
  pinMode(TRIG, OUTPUT);                  // Define the ultrasonic trigger input pin
}

void setup() {
  UltrasonicConfig();  
  Serial.begin(9600);
  HM10.begin(9600);
  Car1.SetSpeed(100);       //Speed:0 - 255
}
//-----------------------------------------------------------------------------------


//function definition-------------------------------------------------------------
bool bluetoothReadCommand(char command);
double updateDistance();
bool wallFollowing();
bool straightFollowing();
//------------------------------------------------------------------------------------


//main loop-----------------------------------------------------------------------
void loop() { 
  //state 1 Waiting for Start Command 
  bluetoothReadCommand('s');
  wallFollowing();
  //ultrasonicDistanceUpdate();
  Serial.println(ultrasonicDistance);
}
//------------------------------------------------------------------------------------

//Helper functions----------------------------------------------------------------------------------

bool wallFollowing(){
 int flag = 0;
 int i,j;
 double lastError = 0;
 double error = 0;
 double control;
 double kp=1,ki=0.01,kd=1;
 double integral, derivative;
 double desiredDistance = 10;

 delay(1000);
 Car1.MotorRun(120,120);       //Car turn right for 1s; left motor speed:250,right motor speed:0
 for (int m = 0;m<200 ;m++){
  ultrasonicDistanceUpdate();
  error = ultrasonicDistance - desiredDistance;
  integral = (error+lastError)/2;
  derivative = error - lastError;
  control = error*kp + integral*ki + derivative*kd;
  lastError = error;
  Serial.println(control);
  Car1.MotorRun(120-control*5,120+control*5);       //Car turn right for 1s; left motor speed:250,right motor speed:
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

void ultrasonicDistanceUpdate()                      // Measure the distance 
{
  //update the distance read of ultrasonic sensor
  digitalWrite(TRIG, LOW);                // set trig pin low 2μs
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);               // set trig pin 10μs , at last 10us 
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);                // set trig pin low
  ultrasonicDistance = pulseIn(ECHO, HIGH) / 58;  // Read echo pin high level time(us)
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

