/*
  LoRaSerial Example - Set Frequency, dwell time, transmit power, number of channels, frequency hop

  Written by: Nathan Seidle
  Date: July 19, 2022

  This example shows how to set the radio upper/lower frequency, dwell time, maximum transmit power,
  the number of channels to use, and whether or not to use FHSS (frequency hopping spread spectrum)

  The dummy values in this sketch are for reference only, they are the default values.

  Be careful, you can set settings that may violate regulatory bodies.

  By default, this sketch does not save settings.

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

  if (myRadio.setTxPower(30) == true)
    Serial.println("Set Success");
  else
    Serial.println("Set Failure");

  uint8_t txPower = myRadio.getTxPower();
  Serial.print("txPower (should be 30): ");
  Serial.println(txPower);

  if (myRadio.setFrequencyMin(902.0) == true)
    Serial.println("Set Success");
  else
    Serial.println("Set Failure");

  float freqMin = myRadio.getFrequencyMin();
  Serial.print("freqMin (should be 902.000): ");
  Serial.println(freqMin, 3);

  if (myRadio.setFrequencyMax(928.0) == true)
    Serial.println("Set Success");
  else
    Serial.println("Set Failure");

  float freqMax = myRadio.getFrequencyMax();
  Serial.print("freqMax (should be 928.0): ");
  Serial.println(freqMax, 3);

  if (myRadio.setNumberOfChannels(50) == true)
    Serial.println("Set Success");
  else
    Serial.println("Set Failure");

  uint8_t numberOfChannels = myRadio.getNumberOfChannels();
  Serial.print("numberOfChannels (should be 50): ");
  Serial.println(numberOfChannels);

  //Be careful when turning off Frequency hopping. This will likely put the radio
  //into a state that violates FCC and other governmental organizational rules
  if (myRadio.setFrequencyHop(true) == true)
    Serial.println("Set Success");
  else
    Serial.println("Set Failure");

  bool frequencyHop = myRadio.getFrequencyHop();
  Serial.print("frequencyHop (should be 1): ");
  Serial.println(frequencyHop);

  if (myRadio.setMaxDwellTime(400) == true)
    Serial.println("Set Success");
  else
    Serial.println("Set Failure");

  int maxDwellTime = myRadio.getMaxDwellTime();
  Serial.print("maxDwellTime (should be 400): ");
  Serial.println(maxDwellTime);



  //AT&W command
//  if (myRadio.saveSettings() == true) //Takes 6ms for radio to save settings
//    Serial.println("Settings saved");
//  else
//    Serial.println("Settings failed to save");
}

void loop()
{

}
