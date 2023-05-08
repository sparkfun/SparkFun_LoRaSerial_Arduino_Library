/*
  LoRaSerial Example - Enter training mode

  Written by: Nathan Seidle
  Date: July 19, 2022

  This example shows how to put a radio into train mode. This is the easiest and fastest
  way to pair two radios together!
  
  Upon entering training mode, the device will first attempt to another near by radio. 
  If another LoRaSerial radio is detected, a randomly generated netID and AES key 
  will be exchanged and the radios will be paired.  
  If no radio is detected, the device will sit in waiting mode, indicated by bouncing LEDs.

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
HardwareSerial mySerial(1); //ESP32 TX on 33, RX on 32
const int pin_espTX = 33;
const int pin_espRX = 32;

//ESP32 UART2
//HardwareSerial mySerial(2); //ESP32 TX on 17, RX on 16
//const int pin_espTX = 17;
//const int pin_espRX = 16;

LoRaSerial myRadio;

void setup()
{
  Serial.begin(115200);
  delay(500); //Wait for ESP32 serial
  Serial.println("Serial configuration of LoRaSerial");

  //Some microcontroller Serial.begin functions need the pin numbers, ie ESP32 UART1
  //setSerialPins can be removed if your platform doesn't need pin numbers when calling Serial.begin()
  myRadio.setSerialPins(pin_espRX, pin_espTX); //RX, TX

  //if (myRadio.begin(mySerial) == false) //Pass Serial port to LoRaSerial library
  //if (myRadio.begin(mySerial, 9600) == false) //Pass Serial port to LoRaSerial library, set radio serial comm to 9600bps
  if (myRadio.begin(mySerial, 9600, false) == false) //Pass Serial port to LoRaSerial library, set radio serial comm to 9600bps, don't exit command mode
  {
    Serial.println(F("LoRaSerial failed to connect. Freezing..."));
    while (true);
  }

  //Be default, the radio exits command mode after connection. This requires 2 seconds before we can send more commands.
  //To avoid waiting for 2 seconds, use the myRadio.begin(loraSerial, 9600, false) method

  Serial.println(F("LoRaSerial connected!"));

  float loraFirmwareVersion = myRadio.getVersion();
  Serial.print("loraFirmwareVersion: ");
  Serial.println(loraFirmwareVersion, 1);

  if (myRadio.enterTrainingMode() == true)
    Serial.println("Radio waiting to be trained");
  else
    Serial.println("Training mode failed");
}

void loop()
{

}
