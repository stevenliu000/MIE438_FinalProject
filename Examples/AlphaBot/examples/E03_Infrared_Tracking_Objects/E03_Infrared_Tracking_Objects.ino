/*************************************** 
Waveshare AlphaBot Car Infrared Tracking Objects

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
}

void loop()
{
  while(1)
  {
    RSensor = digitalRead(RSensorPin);           //LOW means signal, HIGH means no signal
    LSensor = digitalRead(LSensorPin);           //LOW means signal, HIGH means no signal
    
    if (LSensor == LOW && RSensor == LOW)        //If obstacle in front, follow it
      Car1.Forward();
    else if (LSensor == HIGH && RSensor == LOW)  //If the right sensor has a signal,turn right
      Car1.Right();
    else if (RSensor == HIGH && LSensor == LOW)  //If the left sensor has a signal,turn left  
      Car1.Left();
    else                                         //If There is no obstacle, stop
      Car1.Brake();
  }
}


