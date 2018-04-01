#include <SoftwareSerial.h>
#include "AlphaBot.h"
#include <Servo.h>

// Pin setup--------------------------------------------------------------------
// Class Initialization 
SoftwareSerial HM10(0,1); // RX, TX 
// Connect HM10      Arduino Uno
//     Pin 1/TXD          Pin 7
//     Pin 2/RXD          Pin 8
AlphaBot Car1 = AlphaBot();
//--------------------------------------------------------------------------------




//Global Variables-------------------------------------------------------------
int bluetoothInfo;
//------------------------------------------------------------------------------





// Software setup-------------------------------------------------------------
void setup() {  
  Serial.begin(9600);
  HM10.begin(9600);
  Car1.SetSpeed(100);       //Speed:0 - 255
}
//-----------------------------------------------------------------------------------





//function definition-------------------------------------------------------------
bool bluetoothReadCommand(char command);
//------------------------------------------------------------------------------------


//main loop-----------------------------------------------------------------------
void loop() { 
  //state 1 Waiting for Start Command 
  bluetoothReadCommand('s');
  
}
//------------------------------------------------------------------------------------

//Helper functions----------------------------------------------------------------------------------
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
    }
   }
  }
}
//----------------------------------------------------------------------------------

