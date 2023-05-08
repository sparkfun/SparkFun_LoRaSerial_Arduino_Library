/*
  LoRaSerial Example - Set LoRa bandwidth, spread factor, and coding rate

  Written by: Nathan Seidle
  Date: July 19, 2022

  This example shows how to manually set the LoRa settings. This is not normally needed
  nor is it recommended.

  Please use the AirSpeed setting to set the bandwidth, spread factor, and coding rate.
  Please see the AirSpeed table: https://learn.sparkfun.com/tutorials/loraserial-hookup-guide/all#airspeed-table

  Setting custom LoRa settings requires setting the AirSpeed to 0 first. Otherwise, AirSpeed takes precedence and
  the settings will not take effect.

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

  if (myRadio.setAirSpeed(0) == true)
    Serial.println("Set Success");
  else
    Serial.println("Set Failure");

  int airSpeed = myRadio.getAirSpeed();
  Serial.print("airSpeed (should be 0): ");
  Serial.println(airSpeed);
  
  if (myRadio.setBandwidth(500.0) == true)
    Serial.println("Set Success");
  else
    Serial.println("Set Failure - Is Airspeed set to 0?");

  float bandwidth = myRadio.getBandwidth();
  Serial.print("bandwidth (should be 500.0): ");
  Serial.println(bandwidth, 2);

  if (myRadio.setSpreadFactor(9) == true)
    Serial.println("Set Success");
  else
    Serial.println("Set Failure - Is Airspeed set to 0?");

  int spreadFactor = myRadio.getSpreadFactor();
  Serial.print("spreadFactor (should be 9): ");
  Serial.println(spreadFactor);

  if (myRadio.setCodingRate(8) == true)
    Serial.println("Set Success");
  else
    Serial.println("Set Failure - Is Airspeed set to 0?");

  int codingRate = myRadio.getCodingRate();
  Serial.print("codingRate (should be 8): ");
  Serial.println(codingRate);
  
  //By default, this sketch does not save settings
  //if (myRadio.saveSettings() == true) //Takes 6ms for radio to save settings
  //  Serial.println("Settings saved");
  //else
  //  Serial.println("Settings failed to save");
}

void loop()
{

}
