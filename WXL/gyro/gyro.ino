///*************************************************** 
//  This is an example for the Adafruit Triple-Axis Gyro sensor
//
//  Designed specifically to work with the Adafruit L3GD20 Breakout 
//  ----> https://www.adafruit.com/products/1032
//
//  These sensors use I2C or SPI to communicate, 2 pins (I2C) 
//  or 4 pins (SPI) are required to interface.
//
//  Adafruit invests time and resources providing this open source code, 
//  please support Adafruit and open-source hardware by purchasing 
//  products from Adafruit!
//
//  Written by Kevin "KTOWN" Townsend for Adafruit Industries.  
//  BSD license, all text above must be included in any redistribution
// ****************************************************/
#include <Wire.h> 
#include <Adafruit_L3GD20.h>

// Comment this next line to use SPI
//#define USE_I2C

#ifdef USE_I2C
  // The default constructor uses I2C
  Adafruit_L3GD20 gyro;
#else
  // To use SPI, you have to define the pins
  #define GYRO_CS A1 // labeled CS
  #define GYRO_DO A0 // labeled SA0
  #define GYRO_DI A4  // labeled SDA
  #define GYRO_CLK A5 // labeled SCL
  Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
#endif

float angle = 0;

void setup() 
{
  Serial.begin(9600);
  
  // Try to initialise and warn if we couldn't detect the chip
   if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
}

void loop() 
{
  gyro.read();
  angle += (0.11)*((int)gyro.data.z-1);
  //Serial.print("X: "); Serial.print((int)gyro.data.x);   Serial.print(" ");
  //Serial.print("Y: "); Serial.print((int)gyro.data.y);   Serial.print(" ");
  //Serial.print("Z: "); Serial.println((int)gyro.data.z); Serial.print(" ");
  Serial.println(angle);
  delay(100);
}




//
//#include <Wire.h> 
//#include <Adafruit_L3GD20.h>
////#define GYRO_FS_DPS 250
////#define GYRO_ABS_SAMPLE_MAX 0x7fff 
//
//// Comment this next line to use SPI
////#define USE_I2C
//
//#ifdef USE_I2C
//  // The default constructor uses I2C
//  Adafruit_L3GD20 gyro;
//#else
//  // To use SPI, you have to define the pins
//  #define GYRO_CS 8 // labeled CS
//  #define GYRO_DO 9 // labeled SA0
//  #define GYRO_DI 18  // labeled SDA
//  #define GYRO_CLK 19 // labeled SCL
//  Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
//#endif
//
//float x_angle = 0; 
//float y_angle = 0; 
//float z_angle = 0;  
//void setup() 
//{
//  Serial.begin(9600);
//
//  // Try to initialise and warn if we couldn't detect the chip
//  // if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
//  if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
//  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
//  {
//    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
//    while (1);
//  }
//}
//
//void loop() 
//{
//  x_angle += gyro.data.x;
//  y_angle += gyro.data.y;
//  z_angle += gyro.data.z;
//  gyro.read();
//  Serial.print("X: "); Serial.print((int)x_angle);   Serial.print(" ");
//  Serial.print("Y: "); Serial.print((int)y_angle);   Serial.print(" ");
//  Serial.print("Z: "); Serial.println((int)z_angle); Serial.print(" ");
//  delay(100);
//}
