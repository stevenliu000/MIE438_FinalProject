/*************************************** 
Waveshare AlphaBot mjpg-AlphaBot-PID

Command Line:
-----------------------------------------------------------------------------------------------------
{"Car":"Forward"}
{"Car":"Backward"}
{"Car":"Left"}
{"Car":"Right"}

Expended Command
{"Car":"Forward","Time":"1000"}   ------   Car run forward 1000ms
{"Car":"SetSpeed","Value":[250,200]} ------  Set speed as 250 on left motor and 200 on right motor
-----------------------------------------------------------------------------------------------------
{"LCD":"Display","Line1":"Waveshare","Line2":"Waveshare"}

CN: www.waveshare.net/wiki/AlphaBot
EN: www.waveshare.com/wiki/AlphaBot
****************************************/
#include "AlphaBot.h"
#include "ArduinoJson.h"
#include<Servo.h>
#include "PID_v1.h"

#define START '{'
#define END '}'
bool Started = false;
bool Ended = false;
char inData[80];
byte Num;
#define MaxSpeed 16
AlphaBot Car1 = AlphaBot();
Servo servo1;
Servo servo2;
const int CNTL = 0;  //ping D2
const int CNTR = 1;  //ping D3

int countL=0,countR=0;
double SetpointL,InputL,OutputL;
double SetpointR,InputR,OutputR;
PID PIDL(&InputL,&OutputL,&SetpointL,25,8,0,DIRECT);
PID PIDR(&InputR,&OutputR,&SetpointR,25,8,0,DIRECT);

void countLeft()
{
  countL++;
}
void countRight()
{
  countR++;
}

void setup()
{
  Serial.begin(115200);
  attachInterrupt(0,countLeft,CHANGE);
  attachInterrupt(1,countRight,CHANGE);
  InputL = 0;
  SetpointL = 10;
  InputR = 0;
  SetpointR = 10;
  PIDL.SetMode(AUTOMATIC);
  PIDR.SetMode(AUTOMATIC);
  Serial.println("start!");
  servo1.attach(9);
  servo2.attach(10);
  servo1.write(90);
  servo1.write(90);
  //ArduinoCar.Forward();
}

void loop()
{ 
  while(Serial.available() > 0)
 {
   char ch = Serial.read();
   if(ch == START)
   {
     Num = 0;
     inData[Num++] = ch;
     Started = true;
     Ended = false;
   }else if(ch == END)
   {
     inData[Num++] = ch;
     inData[Num] = '\0';
     Ended = true;
   }else if(Started && !Ended)
   {
     inData[Num++] = ch;
   }else
   {
     inData[Num++] = ch;
   }
  }
  
   if(Started && Ended)
   {
     StaticJsonBuffer<80> jsonBuffer;
     JsonObject& DecodeData = jsonBuffer.parseObject(inData);
     
     if (!DecodeData.success())
    {
      Serial.println("parseObject() failed");
      Started = false;
      Ended = false;
      return;
    }
    
    const char* Car = DecodeData["Car"];
    const char* SERVO = DecodeData["Servo"];
    
    if(Car)
    {
      if(strcmp(Car,"Forward") == 0)         //{"Car":"Forward"}
        Car1.Forward();
      else if(strcmp(Car,"Backward") == 0)   //{"Car":"Backward"}
        Car1.Backward();
      else if(strcmp(Car,"Left") == 0)       //{"Car":"Left"}
        Car1.Left();
      else if(strcmp(Car,"Right") == 0)      //{"Car":"Right"}
        Car1.Right();
      else if(strcmp(Car,"SetSpeed") == 0)   //{"Car":"SetSpeed","Value":[250,200]}
      {
        byte LSpeed = DecodeData["Value"][0];
        byte RSpeed = DecodeData["Value"][1];
        SetpointL = LSpeed * MaxSpeed / 255;
        SetpointR = RSpeed * MaxSpeed / 255;
        Serial.println("Speed");
        Serial.println(LSpeed);
        Serial.println(RSpeed);
      }
      else
        Car1.Brake();
      
      unsigned int Time = DecodeData["Time"];
      if(Time > 0)
      {
        delay(Time);
        Car1.Brake();
      }
    }
    else if (SERVO)                        //{"Servo:"Servo1","Angle":180}  {"Servo":"Servo1","Angle":180}
    {
      byte Angle;
      if(strcmp(SERVO,"Servo1") == 0)
      {
        Angle = DecodeData["Angle"]; 
        if(Angle)
        {
          servo1.write(180 - Angle);
        }
      }
      else if(strcmp(SERVO,"Servo2") == 0)
      {
         Angle = DecodeData["Angle"]; 
        if(Angle)
        {
          servo2.write(180 - Angle);
        }
      }
    }
    
    Started = false;
    Ended = false;
   }

  InputL = countL;
  if(PIDL.Compute() == true)
  {
    countL = 0;
  }
  Car1.SetLSpeed(OutputL);
  
  InputR = countR;
  if(PIDR.Compute() == true)
  {
    countR = 0;
  }
  Car1.SetRSpeed(OutputL);
}



