/* WaveShare ARPICAR Run Forward/Backward/Left/Right Test
   
   ARPICAR run forward,backward,left right and so on..
   
   Created 25th June 2016
           by Xinwu Lin
           
   CN: http://www.waveshare.net/
   EN: http://www.waveshare.com/
*/
#include "ArpiBot.h"

int IR1 = 13;
int IR2 = 12;
int IR3 = 11;
int IR4 = 10;
int IR5 = 9;

char S1, S2, S3, S4, S5;
ArpiBot ArduinoCar = ArpiBot();

void TrackerConfig()
{
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
}

void setup()
{
  ArduinoCar.MotorConfig();
  TrackerConfig();
  ArduinoCar.SetSpeed(250);
}

void loop()
{
  S1 = digitalRead(IR1);
  S2 = digitalRead(IR2);
  S3 = digitalRead(IR3);
  S4 = digitalRead(IR4);
  S5 = digitalRead(IR5);
    
  if (S2 == 0)
    ArduinoCar.MotorRun(150,-20);
  else if (S4 == 0)
    ArduinoCar.MotorRun(250,-80);
  else if (S3 == 0)
    ArduinoCar.MotorRun(-20,150);
  else if (S5 == 0)
    ArduinoCar.MotorRun(-80,250);
  else if (S1 == 0)
    ArduinoCar.Forward();
}


