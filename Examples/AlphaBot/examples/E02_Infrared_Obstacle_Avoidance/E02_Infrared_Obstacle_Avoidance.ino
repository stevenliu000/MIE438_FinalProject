/*************************************** 
Waveshare AlphaBot Car Infrared Obstacle Avoidance

CN: www.waveshare.net/wiki/AlphaBot
EN: www.waveshare.com/wiki/AlphaBot
****************************************/
#include "AlphaBot.h"

int LSensorPin = 7;
int RSensorPin = 8;

int LSensor;              //Left Infrared Proximity Sensor signal value
int RSensor;              //Right Infrared Proximity Sensor signal value

AlphaBot Car1 = AlphaBot();

void ProximityConfig()
{
  pinMode(RSensorPin, INPUT);   //Define the input pin of Right Infrared Proximity Sensor
  pinMode(LSensorPin, INPUT);   //Define the input pin of Left Infrared Proximity Sensor
}

void setup()
{
  ProximityConfig();
  Car1.SetSpeed(150);
//  Serial.begin(9600);
}

void loop()
{
  while(1)
  {
    RSensor = digitalRead(RSensorPin);            //LOW means signal, HIGH means no signal 
    LSensor = digitalRead(LSensorPin);            //LOW means signal, HIGH means no signal 
    
//    Serial.print("RSensor: ");
//    Serial.println(RSensor);
//    Serial.print("LSensor: ");
//    Serial.println(LSensor);
    
    if (LSensor == HIGH && RSensor == HIGH)       //If no obstacle in front, run forward
      Car1.Forward();                             
    else if (LSensor == HIGH && RSensor == LOW)   //If the right sensor has a signal,turn left
      Car1.Left();
    else if (RSensor == HIGH && LSensor == LOW)   //If the left sensor has a signal,turn right	
      Car1.Right();
    else                                          //Otherwise,Backward for 5ms and then turn left for 5ms
   {
      Car1.Backward();
      delay(5);
      Car1.LeftCircle();
      delay(5);
    }
  }
  
}


