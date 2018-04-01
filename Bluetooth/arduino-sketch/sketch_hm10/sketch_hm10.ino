#include <SoftwareSerial.h>

SoftwareSerial HM10(8,7); // RX, TX  
// Connect HM10      Arduino Uno
//     Pin 1/TXD          Pin 7
//     Pin 2/RXD          Pin 8

void setup() {  
  Serial.begin(9600);
  // If the baudrate of the HM-10 module has been updated,
  // you may need to change 9600 by another value
  // Once you have found the correct baudrate,
  // you can update it using AT+BAUDx command 
  // e.g. AT+BAUD0 for 9600 bauds
  HM10.begin(9600);
}

void loop() {  
  int c;
  
  if (HM10.available()) {
    c = HM10.read();  
    if (c != 0)
    {
      // Non-zero input means "turn on LED".
      Serial.write(c);
    }
  }

  if (Serial.available()){
    c = Serial.read();
    HM10.write(c);
  }
  
}
