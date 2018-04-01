/*************************************** 
Waveshare AlphaBot Car Bluetooth Control

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

///////////////////////////////////////////////////////////////////////////////////////////////////
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 7
#define OLED_SA0   8
Adafruit_SSD1306 display(OLED_RESET, OLED_SA0);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////

#define START '{'
#define END '}'
bool Started = false;
bool Ended = false;

AlphaBot Car1 = AlphaBot();

char inData[80];
byte Num;

void setup()
{
  Serial.begin(115200);
  Serial.println("start!");
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
    const char* LCD = DecodeData["LCD"];
    
    if(Car)
    {
      if(strcmp(Car,"Forward") == 0)                 //{"Car":"Forward"}
        Car1.Forward();
      else if(strcmp(Car,"Backward") == 0)           //{"Car":"Backward"}
        Car1.Backward();
      else if(strcmp(Car,"Left") == 0)               //{"Car":"Left"}
        Car1.Left();
      else if(strcmp(Car,"Right") == 0)              //{"Car":"Right"}
        Car1.Right();
      else if(strcmp(Car,"SetSpeed") == 0)           //{"Car":"SetSpeed","Value":[250,200]}
      {
        byte LSpeed = DecodeData["Value"][0];
        byte RSpeed = DecodeData["Value"][1];
        Car1.SetSpeed(LSpeed,RSpeed);
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
    else if (LCD)                                     //{"LCD":"Display","Line1":"Waveshare","Line2":"Waveshare"}
    {
      //define the lcd command here
      display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
      display.clearDisplay();
      
      if(strcmp(LCD,"Display") == 0)
      {
        const char* Line1 = DecodeData["Line1"]; 
        const char* Line2 = DecodeData["Line2"];
        if(Line1)
        {
            display.setTextSize(1);
            display.setTextColor(WHITE);
            display.setCursor(0,0);
            display.println(Line1);
        }
        
        if(Line2)
        {
          display.setTextSize(2);
          display.setTextColor(WHITE);
          display.setCursor(15,20);
          display.println(Line2);
        }
      }
      
      display.display();
      delay(3000);
      display.clearDisplay();
    }
    Started = false;
    Ended = false;
   }
}



