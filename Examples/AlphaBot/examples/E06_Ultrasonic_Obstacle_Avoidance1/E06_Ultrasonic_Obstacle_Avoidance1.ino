/*************************************** 
Waveshare AlphaBot Car Ultrasonic Obstacle Avoidance (without SG90 servo)

CN: www.waveshare.net/wiki/AlphaBot
EN: www.waveshare.com/wiki/AlphaBot
****************************************/
#include "AlphaBot.h"

int ECHO = 12;
int TRIG = 11;

int Distance = 0;
AlphaBot Car1 = AlphaBot();

void UltrasonicConfig()
{
  pinMode(ECHO, INPUT);                   // Define the ultrasonic echo input pin
  pinMode(TRIG, OUTPUT);                  // Define the ultrasonic trigger input pin
}

void Distance_test()                      // Measure the distance 
{
  digitalWrite(TRIG, LOW);                // set trig pin low 2μs
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);               // set trig pin 10μs , at last 10us 
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);                // set trig pin low
  float Fdistance = pulseIn(ECHO, HIGH);  // Read echo pin high level time(us)
  
  Fdistance = Fdistance / 58;                
  //Y m =（X s * 344）/ 2; 
  //X s =（ 2 * Y m）/ 344;
  //X s = 0.0058 * Y m;
  //cm = us /58
  
  Serial.print("Fdistance:");            //output distance (Unit: cm)
  Serial.println(Fdistance);       
  Distance = Fdistance;
}  

void Distance_display()                   //display distance
{
  if((2<Distance)&(Distance<400))         //Ultrasonic range ranging 2cm to 400cm
  {
    Serial.print("Distance:");           //print distance
    Serial.print(Distance);        
    Serial.print("cm\n\n");        
  }
  else
  {
    Serial.print("!!! Out of range\n");     
  }
  delay(250);
}

void setup()
{
  UltrasonicConfig();
  Car1.SetSpeed(100);
  Serial.begin(9600);                    //Initialize the serial port
  Serial.println("Start\n");
}

void loop()
{
    Distance_test();                      //Measure the distance ahead
    Distance_display();                   //display distance
    if(Distance < 40)                     //The value is the distance to the obstacle and can be set according to the actual situation
      while(Distance < 40)
      {
        Car1.Right();                     //turn right
        Distance_test();
        Distance_display();
      }
    else
       Car1.Forward();                    //there is no objtacle ,go stre
}


