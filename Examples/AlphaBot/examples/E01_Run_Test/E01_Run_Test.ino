/*************************************** 
Waveshare AlphaBot Car Run Test

CN: www.waveshare.net/wiki/AlphaBot
EN: www.waveshare.com/wiki/AlphaBot
****************************************/
#include "AlphaBot.h"

AlphaBot Car1 = AlphaBot();

void setup()
{
  Car1.SetSpeed(250);       //Speed:0 - 255
}

void loop()
{  
  delay(1000);
  Car1.Forward(1000);     //Car run forward for 1s
  Car1.Brake();
  
//  delay(1000);
//  Car1.Backward(1000);  //Car run backward for 1s
//  Car1.Brake();

//  delay(1000);
//  Car1.Left(1000);      //Car turn left for 1s
//  Car1.Brake();

//  delay(1000);
//  Car1.Right(1000);     //Car turn right for 1s
//  Car1.Brake();

//  delay(1000);
//  Car1.LeftCircle(1000);  //Car left circle for 1s
//  Car1.Brake();

//  delay(1000);
//  Car1.RightCircle(1000);  //Car right circle for 1s
//  Car1.Brake();
  
//  delay(1000);  
//  Car1.MotorRun(250,250);  //Car run forward for 1s; left motor speed:250,right motor speed:250
//  delay(1000);
//  Car1.Brake();

//  delay(1000);  
//  Car1.MotorRun(-250,-250);  //Car run backward for 1s; left motor speed:250,right motor speed:250
//  delay(1000);
//  Car1.Brake();
  
//  delay(1000);  
//  Car1.MotorRun(0,250);       //Car left circle for 1s; left motor speed:0,right motor speed:250
//  delay(1000);
//  Car1.Brake();

//  delay(1000);  
//  Car1.MotorRun(250,0);       //Car turn right for 1s; left motor speed:250,right motor speed:0
//  delay(1000);
//  Car1.Brake();
}
