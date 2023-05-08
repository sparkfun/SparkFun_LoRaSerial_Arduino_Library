/*
  LoRaSerial Example - Get Firmware Version

  Written by: Nathan Seidle
  Date: July 19, 2022

  This example shows how to connect to and read the firmware version from a radio.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! 
  https://www.sparkfun.com/products/20029

  License: MIT Please see LICENSE.md for full details
*/

//The library supports hardware serial or software serial
//If you wish to use SoftwareSerial, it must be included before the LoRaSerial Library h
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(2, 3); // RX, TX

#include "SparkFun_LoRaSerial_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_LoRaSerial_Arduino_Library

//ESP32 UART1
//HardwareSerial mySerial(1); //ESP32 TX on 33, RX on 32
//const int pin_espTX = 33;
//const int pin_espRX = 32;

//ESP32 UART2
HardwareSerial mySerial(2); //ESP32 TX on 17, RX on 16
const int pin_espTX = 17;
const int pin_espRX = 16;

LoRaSerial myRadio;

void setup()
{
  Serial.begin(115200);
  delay(500); //Wait for ESP32 serial
  Serial.println("Serial configuration of LoRaSerial");

  //Some microcontroller Serial.begin functions need the pin numbers, ie ESP32 UART1
  //setSerialPins can be removed if your platform doesn't need pin numbers when calling Serial.begin()
  myRadio.setSerialPins(pin_espRX, pin_espTX); //RX, TX
  
  myRadio.enableDebugging();

  mySerial.begin(57600);

  myRadio.begin(mySerial, 57600);

  myRadio.setSerialSpeed(57600);  
}

void loop()
{

}
