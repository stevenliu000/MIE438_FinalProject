/*************************************** 
Waveshare AlphaBot Car Infrared Remote control

CN: www.waveshare.net/wiki/AlphaBot
EN: www.waveshare.com/wiki/AlphaBot
****************************************/
#include "AlphaBot.h"

#define KEY2 0x18                 //Key:2 
#define KEY8 0x52                 //Key:8 
#define KEY4 0x08                 //Key:4 
#define KEY6 0x5A                 //Key:6 
#define KEY1 0x0C                 //Key:1 
#define KEY3 0x5E                 //Key:3 
#define KEY5 0x1C                 //Key:5
#define SpeedDown 0x07            //Key:VOL-
#define SpeedUp 0x15              //Key:VOL+
#define ResetSpeed 0x09           //Key:EQ
#define Repeat 0xFF               //press and hold the key

#define IR  4                     //he infrare remote receiver pin 
unsigned long n = 0;
AlphaBot ArduinoCar = AlphaBot();

unsigned char t[30];
unsigned char results;
char IR_decode(unsigned char code);
void translateIR();

void setup()
{
  Serial.begin(115200);
  pinMode(IR, INPUT);
  ArduinoCar.SetSpeed(150);
}

void loop()
{
  if (IR_decode(&results))
  {
    translateIR();
  }

    if (millis() - n > 200)
  {
    ArduinoCar.Brake();
  }
}

/*-----( Declare User-written Functions )-----*/
void translateIR() // takes action based on IR code received
// describing KEYES Remote IR codes
{
  n = millis(); ;
  switch (results)
  {
    case KEY1:
      ArduinoCar.LeftCircle();
      break;

    case KEY2:
      ArduinoCar.Forward();
      break;

    case KEY3:
      ArduinoCar.RightCircle();
      break;

    case KEY4:
      ArduinoCar.Left();
      break;

    case KEY5:
      ArduinoCar.Brake();
      break;

    case KEY6:
      ArduinoCar.Right();
      break;

    case KEY8:
      ArduinoCar.Backward();
      break;

    case Repeat:
      break;

    default:
      ArduinoCar.Brake();
  }// End Case
} //END translateIR

char IR_decode(unsigned char * code)
{
  char flag = 0;
  unsigned int count = 0;
  unsigned char i, index, cnt = 0, data[4] = {0, 0, 0, 0};
  if (digitalRead(IR) == LOW)
  {
    count = 0;
    while (digitalRead(IR) == LOW && count++ < 200)  //9ms
      delayMicroseconds(60);
    t[0] = count;
    count = 0;
    while (digitalRead(IR) == HIGH && count++ < 80)   //4.5ms
      delayMicroseconds(60);
    t[1] = count;
    for (i = 0; i < 32; i++)
    {
      count = 0;
      while (digitalRead(IR) == LOW && count++ < 15) //0.56ms
        delayMicroseconds(60);
      count = 0;
      while (digitalRead(IR) == HIGH && count++ < 40) //0: 0.56ms; 1: 1.69ms
        delayMicroseconds(60);
      if (count > 20)data[index] |= (1 << cnt);
      if (cnt == 7)
      {
        cnt = 0;
        index++;
      }
      else cnt++;
    }
    if (data[0] + data[1] == 0xFF && data[2] + data[3] == 0xFF) //check
    {
      code[0] = data[2];
      Serial.println(code[0], HEX);
      flag = 1;
    }
    if (data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFF && data[3] == 0xFF)
    {
      code[0] = 0xFF;
      Serial.println("rep");
      flag = 1;
    }
  }
  return flag;
}
