/*
  Arduino library for the SparkFun LoRaSerial serial radio modem
  By: Nathan Seidle
  June 16th, 2022

  Based extensively on the:
  Arduino Library for the SARA R5
  Written by Paul Clark @ SparkFun Electronics, October 19th, 2020

  This Arduino library provides mechanisms to initialize and use
  the LoRaSerial radio over either a SoftwareSerial or hardware serial port.

  Please see LICENSE.md for the license information
*/

#ifndef SPARKFUN_LORASERIAL_ARDUINO_LIBRARY_H
#define SPARKFUN_LORASERIAL_ARDUINO_LIBRARY_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef ARDUINO_ARCH_AVR                    // Arduino AVR boards (Uno, Pro Micro, etc.)
#define LORASERIAL_SOFTWARE_SERIAL_ENABLED // Enable software serial
#endif

#ifdef ARDUINO_ARCH_SAMD                    // Arduino SAMD boards (SAMD21, etc.)
#define LORASERIAL_SOFTWARE_SERIAL_ENABLEDx // Disable software serial
#endif

#ifdef ARDUINO_ARCH_APOLLO3                 // Arduino Apollo boards (Artemis module, RedBoard Artemis, etc)
#define LORASERIAL_SOFTWARE_SERIAL_ENABLEDx // Disable software serial (no longer supported with v2 of Apollo3)
// Note: paulvha has provided software serial support for v2 of the Apollo3 / Artemis core.
//       Further details are available at:
//       https://github.com/paulvha/apollo3/tree/master/SoftwareSerial
#endif

#ifdef ARDUINO_ARCH_STM32                  // STM32 based boards (Disco, Nucleo, etc)
#define LORASERIAL_SOFTWARE_SERIAL_ENABLED // Enable software serial
#endif

#ifdef ARDUINO_ARCH_ESP32 // ESP32 based boards
// Check to see if ESP Software Serial has been included
// Note: you need to #include <SoftwareSerial.h> at the very start of your script,
// _before_ the #include <SparkFun_LoRaSerial_Arduino_Library.h>, for this to work.
#if __has_include(<SoftwareSerial.h>)
#define LORASERIAL_SOFTWARE_SERIAL_ENABLED // Enable software serial
#else
#define LORASERIAL_SOFTWARE_SERIAL_ENABLEDx // Disable software serial
#endif
#endif

#ifdef ARDUINO_ARCH_ESP8266 // ESP8266 based boards
// Check to see if ESP Software Serial has been included
// Note: you need to #include <SoftwareSerial.h> at the very start of your script,
// _before_ the #include <SparkFun_LoRaSerial_Arduino_Library.h>, for this to work.
#if __has_include(<SoftwareSerial.h>)
#define LORASERIAL_SOFTWARE_SERIAL_ENABLED // Enable software serial
#else
#define LORASERIAL_SOFTWARE_SERIAL_ENABLEDx // Disable software serial
#endif
#endif

#ifdef LORASERIAL_SOFTWARE_SERIAL_ENABLED
#include <SoftwareSerial.h> // SoftwareSerial.h is guarded. It is OK to include it twice.
#endif

// Timing
// Note: On ESP32, Serial.available is not instantaneous
// https://gitter.im/espressif/arduino-esp32?at=5e25d6370a1cf54144909c85
// If the UART sees the receive pin inactive for two byte periods it triggers an interrupt to empty the fifo.
// At 9600 baud, hwAvailable takes [number of bytes received + 2] * 1ms = 11ms for '\r\nERROR\r\n' before it indicates that data is being received.
// At 115200 baud, hwAvailable takes [number of bytes received + 2] * .087ms = ~1ms before it indicates that data is being received.
#define LORASERIAL_STANDARD_RESPONSE_TIMEOUT 75

#define LORASERIAL_AESKEY_TIMEOUT 75 // AES Key is 32 characters, at 9600bps it can take ~32ms

// Base AT Commands
const char LORASERIAL_COMMAND_MODE[] = "+++";
const char LORASERIAL_COMMAND_AT[] = "AT";
const char LORASERIAL_COMMAND_VCSTATUS[] = "A";
const char LORASERIAL_COMMAND_BREAKLINK[] = "B";
const char LORASERIAL_COMMAND_VCCONNECT[] = "C";
const char LORASERIAL_COMMAND_FACTORYRESET[] = "F";
const char LORASERIAL_COMMAND_GENERATEAESID[] = "G";
const char LORASERIAL_COMMAND_RADIOVERSION[] = "I";
const char LORASERIAL_COMMAND_EXITCOMMANDMODE[] = "O";
const char LORASERIAL_COMMAND_TRAININGMODE[] = "T";
const char LORASERIAL_COMMAND_SAVESETTINGS[] = "W";
const char LORASERIAL_COMMAND_REBOOT[] = "Z";

// Radio Commands
const char LORASERIAL_COMMAND_AIRSPEED[] = "AirSpeed";
const char LORASERIAL_COMMAND_AUTOTUNE[] = "AutoTune";
const char LORASERIAL_COMMAND_BANDWIDTH[] = "Bandwidth";
const char LORASERIAL_COMMAND_CLIENTFINDPARTNER[] = "ClientFindPartner";
const char LORASERIAL_COMMAND_CODINGRATE[] = "CodingRate";
const char LORASERIAL_COMMAND_DATASCRAMBLING[] = "DataScrambling";
const char LORASERIAL_COMMAND_ENABLECRC16[] = "EnableCRC16";
const char LORASERIAL_COMMAND_ENCRYPTDATA[] = "EncryptData";
const char LORASERIAL_COMMAND_ENCRYPTIONKEY[] = "EncryptionKey";
const char LORASERIAL_COMMAND_FRAMESTOYIELD[] = "FramesToYield";
const char LORASERIAL_COMMAND_FREQUENCYHOP[] = "FrequencyHop";
const char LORASERIAL_COMMAND_FREQUENCYMAX[] = "FrequencyMax";
const char LORASERIAL_COMMAND_FREQUENCYMIN[] = "FrequencyMin";
const char LORASERIAL_COMMAND_HEARTBEATTIMEOUT[] = "HeartBeatTimeout";
const char LORASERIAL_COMMAND_MAXDWELLTIME[] = "MaxDwellTime";
const char LORASERIAL_COMMAND_NETID[] = "NetID";
const char LORASERIAL_COMMAND_NUMBEROFCHANNELS[] = "NumberOfChannels";
const char LORASERIAL_COMMAND_OPERATINGMODE[] = "OperatingMode";
const char LORASERIAL_COMMAND_OVERHEADTIME[] = "OverheadTime";
const char LORASERIAL_COMMAND_PREAMBLELENGTH[] = "PreambleLength";
const char LORASERIAL_COMMAND_SELECTLEDUSE[] = "SelectLedUse";
const char LORASERIAL_COMMAND_SERVER[] = "Server";
const char LORASERIAL_COMMAND_SPREADFACTOR[] = "SpreadFacetor";
const char LORASERIAL_COMMAND_SYNCWORD[] = "SyncWord";
const char LORASERIAL_COMMAND_TRAININGKEY[] = "TrainingKey";
const char LORASERIAL_COMMAND_TXPOWER[] = "TxPower";
const char LORASERIAL_COMMAND_TXTORXUSEC[] = "TxToRxUsec";
const char LORASERIAL_COMMAND_VERIFYRXNETID[] = "VerifyRxNetId";

// Info Commands
const char LORASERIAL_COMMAND_SETTABLEPARAMETERS[] = "I0";
const char LORASERIAL_COMMAND_BOARDVARIANT[] = "I1";
const char LORASERIAL_COMMAND_VERSION[] = "I2";
const char LORASERIAL_COMMAND_RSSI[] = "I3";
const char LORASERIAL_COMMAND_RANDOM[] = "I4";
const char LORASERIAL_COMMAND_MAXTHROUGHPUT[] = "I5";
const char LORASERIAL_COMMAND_AESKEY[] = "I6";
const char LORASERIAL_COMMAND_CHANNEL[] = "I7";
const char LORASERIAL_COMMAND_UNIQUEID[] = "I8";
const char LORASERIAL_COMMAND_DATAGRAMSIZE[] = "I9";
const char LORASERIAL_COMMAND_RADIOMETRICS[] = "I10";
const char LORASERIAL_COMMAND_MYVC[] = "I11";

// Serial Commands
const char LORASERIAL_COMMAND_COPYSERIAL[] = "CopySerial";
const char LORASERIAL_COMMAND_ECHO[] = "Echo";
const char LORASERIAL_COMMAND_FLOWCONTROL[] = "FlowControl";
const char LORASERIAL_COMMAND_INVERTCTS[] = "InvertCts";
const char LORASERIAL_COMMAND_INVERTRTS[] = "InvertRts";
const char LORASERIAL_COMMAND_SERIALDELAY[] = "SerialDelay";
const char LORASERIAL_COMMAND_SERIALSPEED[] = "SerialSpeed";
const char LORASERIAL_COMMAND_USBSERIALWAIT[] = "UsbSerialWait";

// Response
const char LORASERIAL_RESPONSE_OK[] = "OK\r\n";
const char LORASERIAL_RESPONSE_ERROR[] = "ERROR\r\n";
#define LORASERIAL_RESPONSE_OK_OR_ERROR NULL

#define NUM_SUPPORTED_BAUD 8
// Baud rates sorted in most likely to reduce autobaud discovery time
const unsigned long LORASERIAL_SUPPORTED_BAUD[NUM_SUPPORTED_BAUD] =
    {
        57600,
        9600,
        115200,
        2400,
        4800,
        14400,
        19200,
        38400,
};
#define LORASERIAL_DEFAULT_BAUD_RATE 57600

// The minimum memory allocation for responses from sendCommandWithResponse
// This needs to be large enough to hold the response you're expecting plus and URC's that may arrive during the timeout
#define minimumResponseAllocation 128

typedef enum
{
  LORASERIAL_ERROR_INVALID = -1,        // -1
  LORASERIAL_ERROR_SUCCESS = 0,         // 0
  LORASERIAL_ERROR_OUT_OF_MEMORY,       // 1
  LORASERIAL_ERROR_TIMEOUT,             // 2
  LORASERIAL_ERROR_UNEXPECTED_PARAM,    // 3
  LORASERIAL_ERROR_UNEXPECTED_RESPONSE, // 4
  LORASERIAL_ERROR_NO_RESPONSE,         // 5
  LORASERIAL_ERROR_ERROR                // 6
} LoRaSerial_error_t;
#define LORASERIAL_SUCCESS LORASERIAL_ERROR_SUCCESS

class LoRaSerial : public Print
{
public:
  // Constructor
  LoRaSerial(uint8_t maxInitRetries = 0);

  ~LoRaSerial();

  // Begin -- initialize module and ensure it's connected
#ifdef LORASERIAL_SOFTWARE_SERIAL_ENABLED
  bool begin(SoftwareSerial &softSerial, unsigned long baud = 9600, bool exitCommandMode = true);
#endif
  bool begin(HardwareSerial &hardSerial, unsigned long baud = 9600, bool exitCommandMode = true);

  void setSerialPins(uint8_t rxPin, uint8_t txPin); // Needed for some microcontroller Serial.begin functions, ie ESP32 UART1

  // Debug prints
  void enableDebugging(Print &debugPort = Serial);   // Turn on debug printing. If user doesn't specify then Serial will be used.
  void enableAtDebugging(Print &debugPort = Serial); // Turn on AT debug printing. If user doesn't specify then Serial will be used.

  LoRaSerial_error_t getData(const char *command, char *response, bool addQMark = false, unsigned long commandTimeout = LORASERIAL_STANDARD_RESPONSE_TIMEOUT);
  LoRaSerial_error_t setData(const char *command, int settingValue, unsigned long commandTimeout = LORASERIAL_STANDARD_RESPONSE_TIMEOUT);
  LoRaSerial_error_t setData(const char *command, float settingValue, unsigned long commandTimeout = LORASERIAL_STANDARD_RESPONSE_TIMEOUT);
  LoRaSerial_error_t setData(const char *command, const char *settingValue, unsigned long commandTimeout = LORASERIAL_STANDARD_RESPONSE_TIMEOUT);

  // General AT Commands
  bool enterCommandMode(void);
  bool exitCommandMode(void);
  bool at(void);
  bool reset(void);
  bool saveSettings(void);
  bool factoryReset(void);
  bool enterTrainingMode(void);

  // ATIx information
  float getVersion(void); // Return -999 if error
  int getRSSI(void);      // Return -999 if error
  byte getRandom(void);   // 0 to 255
  uint16_t getMaxThroughput(void);

  // Radio Commands
  uint16_t getAirSpeed(void);
  bool setAirSpeed(uint16_t airSpeed);
  bool getAutoTune(void);
  bool setAutoTune(bool autoTune);
  float getBandwidth(void);
  bool setBandwidth(float bandwidth);
  //bool getClientFindPartner(void);
  //bool setClientFindPartner(bool autoTune);
  uint8_t getCodingRate(void);
  bool setCodingRate(uint8_t codingRate);


  // Parameters
  uint16_t getSerialSpeed(void);
  uint8_t getNetworkID(void);
  uint8_t getOperatingMode(void);
  bool getEncryptData(void);
  void getAESKey(char *hexKey);
  bool getDataScrambling(void);
  uint8_t getTxPower(void);
  float getFrequencyMin(void);
  float getFrequencyMax(void);
  uint8_t getNumberOfChannels(void);
  bool getFrequencyHop(void);
  uint16_t getMaxDwellTime(void);
  uint8_t getSpreadFactor(void);
  uint8_t getSyncWord(void);
  uint16_t getPreambleLength(void);
  bool getEcho(void);
  uint16_t getHeartBeatTimeout(void);
  bool getFlowControl(void);

  bool setSerialSpeed(uint16_t baudRate);
  bool setNetworkID(uint8_t networkID);
  bool setOperatingMode(uint8_t operatingMode);
  bool setEncryptData(bool encryptData);
  bool setAESKey(const char *hexKey);
  bool setDataScrambling(bool dataScrambling);
  bool setTxPower(uint8_t txPower);
  bool setFrequencyMin(float frequencyMin);
  bool setFrequencyMax(float frequencyMax);
  bool setNumberOfChannels(uint8_t numberOfChannels);
  bool setFrequencyHop(bool frequencyHop);
  bool setMaxDwellTime(uint16_t maxDwellTime);
  bool setSpreadFactor(uint8_t spreadFactor);
  bool setSyncWord(uint8_t syncWord);
  bool setPreambleLength(uint16_t preambleLength);
  bool setEcho(bool echo);
  bool setHeartBeatTimeout(uint16_t heartBeatTimeout);
  bool setFlowControl(bool flowControl);

  // Direct write/print to cell serial port
  virtual size_t write(uint8_t c);
  virtual size_t write(const char *str);
  virtual size_t write(const char *buffer, size_t size);

private:
  HardwareSerial *_hardSerial;
#ifdef LORASERIAL_SOFTWARE_SERIAL_ENABLED
  SoftwareSerial *_softSerial;
#endif

  Print *_debugPort;          // The stream to send debug messages to if enabled. Usually Serial.
  bool _printDebug = false;   // Flag to print debugging variables
  Print *_debugAtPort;        // The stream to send debug messages to if enabled. Usually Serial.
  bool _printAtDebug = false; // Flag to print debugging variables

  unsigned long _baud;
  uint8_t _maxInitRetries;
  bool _inCommandMode = false;

#define _RXBuffSize 128
  const unsigned long _rxWindowMillis = 2; // 1ms is not quite long enough for a single char at 9600 baud. millis roll over much less often than micros. See notes in .cpp re. ESP32!
  char *_loraRXBuffer;                     // Allocated in LoRaSerial::begin
  unsigned long _lastSerialComm = 0;       // Command mode cannot be entered if serial traffic has occured in the last 2000ms
  bool _firstComm = true;                  // Assume the radio has been on for awhile and we can immediately enter command mode
  uint8_t _rxPin = 255;                    // ESP32 UART 1 requires special begin functions. rx/tx pin #s must be set before calling begin.
  uint8_t _txPin = 255;                    // ESP32 UART 1 requires special begin functions. rx/tx pin #s must be set before calling begin.

  bool init(unsigned long baud);

  virtual void beginSerial(unsigned long baud);
  bool autobaud(unsigned long desiredBaud);

  // Send command with an expected (potentially partial) response, store entire response
  LoRaSerial_error_t sendCommandWithResponse(const char *command, const char *expectedResponse,
                                             char *responseDest, unsigned long commandTimeout, int destSize = minimumResponseAllocation, bool at = true);

  // Send a command -- prepend AT if at is true
  void sendCommand(const char *command, bool at);

  // UART Functions
  int hwAvailable(void);
  size_t hwWriteData(const char *buff, int len);
  size_t hwWrite(const char c);
  size_t hwPrint(const char *s);

  int readAvailable(char *inString);
  char readChar(void);

  //* void setTimeout(unsigned long timeout);
  //* bool find(char *target);

  char *loraSerial_calloc_char(size_t num);
  uint8_t numPlaces(int32_t n);
};

#endif // SPARKFUN_LORASERIAL_ARDUINO_LIBRARY_H