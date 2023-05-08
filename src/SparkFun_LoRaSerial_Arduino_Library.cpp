/*
  Arduino library for the SparkFun LoRaSerial serial radio modem
  By: Nathan Seidle
  February 22nd, 2023

  Based extensively on the:
  Arduino Library for the SARA R5
  Written by Paul Clark @ SparkFun Electronics, October 19th, 2020

  This Arduino library provides mechanisms to initialize and use
  the LoRaSerial radio over either a SoftwareSerial or hardware serial port.

  Please see LICENSE.md for the license information
*/

#include "SparkFun_LoRaSerial_Arduino_Library.h"

LoRaSerial::LoRaSerial(uint8_t maxInitRetries)
{
#ifdef LORASERIAL_SOFTWARE_SERIAL_ENABLED
    _softSerial = NULL;
#endif
    _hardSerial = NULL;
    _maxInitRetries = maxInitRetries;
    _baud = 0;
    _inCommandMode = false;

    _debugAtPort = NULL;
    _debugPort = NULL;
    _printDebug = false;

    _loraRXBuffer = NULL;

    _lastSerialComm = 0;
}

LoRaSerial::~LoRaSerial(void)
{
    if (NULL != _loraRXBuffer) {
        delete[] _loraRXBuffer;
        _loraRXBuffer = NULL;
    }
}

#ifdef LORASERIAL_SOFTWARE_SERIAL_ENABLED
bool LoRaSerial::begin(SoftwareSerial& softSerial, unsigned long baud, bool exitComm)
{
    if (NULL == _loraRXBuffer) {
        _loraRXBuffer = new char[_RXBuffSize];
        if (NULL == _loraRXBuffer) {
            if (_printDebug == true)
                _debugPort->println(F("begin: not enough memory for _loraRXBuffer!"));
            return false;
        }
    }
    memset(_loraRXBuffer, 0, _RXBuffSize);

    _softSerial = &softSerial;

    if (init(baud) == false)
        return false;

    if (exitComm)
        exitCommandMode();

    return true;
}
#endif

bool LoRaSerial::begin(HardwareSerial& hardSerial, unsigned long baud, bool exitComm)
{
    if (NULL == _loraRXBuffer) {
        _loraRXBuffer = new char[_RXBuffSize];
        if (NULL == _loraRXBuffer) {
            if (_printDebug == true)
                _debugPort->println(F("begin: not enough memory for _loraRXBuffer!"));
            return false;
        }
    }
    memset(_loraRXBuffer, 0, _RXBuffSize);

    _hardSerial = &hardSerial;

    if (init(baud) == false)
        return false;

    if (exitComm)
        exitCommandMode();

    return true;
}

// Needed for some microcontroller Serial.begin functions, ie ESP32 UART1
void LoRaSerial::setSerialPins(uint8_t rxPin, uint8_t txPin)
{
    _rxPin = rxPin;
    _txPin = txPin;
}

// Calling this function with nothing sets the debug port to Serial
// You can also call it with other streams like Serial1, SerialUSB, etc.
void LoRaSerial::enableDebugging(Print& debugPort)
{
    _debugPort = &debugPort;
    _printDebug = true;
}

// Calling this function with nothing sets the debug port to Serial
// You can also call it with other streams like Serial1, SerialUSB, etc.
void LoRaSerial::enableAtDebugging(Print& debugPort)
{
    _debugAtPort = &debugPort;
    _printAtDebug = true;
}

size_t LoRaSerial::write(uint8_t c)
{
    return hwWrite(c);
}

size_t LoRaSerial::write(const char* str)
{
    return hwPrint(str);
}

size_t LoRaSerial::write(const char* buffer, size_t size)
{
    return hwWriteData(buffer, size);
}

// High level AT commands

//+++ - Responds with OK if first time entering command mode, or error if already in command mode.
// If our internal variable is not in command mode, attempt to enter command mode
bool LoRaSerial::enterCommandMode(void)
{
    if (_inCommandMode == false) {
        if (millis() - _lastSerialComm < 2200 && _firstComm == false) {
            if (_printDebug == true)
                _debugPort->println(F("Delay before trying commandMode"));
        }

        // LoRaSerial won't respond to +++ unless serial traffic has stopped for 2000ms
        while (millis() - _lastSerialComm < 2200 && _firstComm == false)
            delay(1);

        _firstComm = false;

        // serialRadio.begin(9600);
        //     _hardSerial->begin(9600, SERIAL_8N1, _rxPin, _txPin); //ESP RX, TX
        //     _hardSerial->println("hi");
        //
        //     while (1)
        //     {
        //       if (_hardSerial->available())
        //       {
        //         Serial.print("I heard: ");
        //         Serial.write(_hardSerial->read());
        //         Serial.println();
        //       }
        //     }

        char response[minimumResponseAllocation];
        LoRaSerial_error_t err = sendCommandWithResponse(LORASERIAL_COMMAND_MODE, LORASERIAL_RESPONSE_OK_OR_ERROR,
            response, LORASERIAL_STANDARD_RESPONSE_TIMEOUT, minimumResponseAllocation, false); // No AT portion to command

        if (err == LORASERIAL_SUCCESS || err == LORASERIAL_ERROR_ERROR) {
            if (_printDebug == true)
                _debugPort->println(F("Command mode successful"));
            _inCommandMode = true;
        } else {
            // If the device did not respond to +++, device may already be in command mode
            // and thus requires commands to be terminated. +++\r will get error
            if (_printDebug == true)
                _debugPort->println(F("Send termination"));

            err = sendCommandWithResponse("\r", LORASERIAL_RESPONSE_OK_OR_ERROR,
                response, LORASERIAL_STANDARD_RESPONSE_TIMEOUT, minimumResponseAllocation, false); // No AT portion to command

            _lastSerialComm = millis();

            if (err == LORASERIAL_SUCCESS || err == LORASERIAL_ERROR_ERROR) {
                if (_printDebug == true)
                    _debugPort->println(F("Command mode successful"));
                _inCommandMode = true;
            } else {
                if (_printDebug == true)
                    _debugPort->println(F("Command mode failed"));
            }
        }
    }
    return (_inCommandMode);
}

// ATO - Exit command mode. Responds OK then exits command mode.
bool LoRaSerial::exitCommandMode(void)
{
    //If we are already out of command mode, don't send anything
    if (_inCommandMode == false)
        return (true);

    char response[minimumResponseAllocation];
    if (getData(LORASERIAL_COMMAND_EXITCOMMANDMODE, response) == LORASERIAL_SUCCESS) // Send command and get response
    {
        _inCommandMode = false;
        _lastSerialComm = millis();
        Serial.println("Out of command mode");
        return (true);
    }
    return (false);
}

// AT - Should respond with OK if we are in command mode
bool LoRaSerial::at(void)
{
    char response[minimumResponseAllocation];
    if (getData("", response) == LORASERIAL_SUCCESS) // Send command and get response
        return (true);
    return (false);
}

// ATZ - responds with OK before resetting radio. This will exit command mode
// Radio takes 1300ms to reset from command to radio reporting 'LRS'.
bool LoRaSerial::reset(void)
{
    char response[minimumResponseAllocation];
    if (getData(LORASERIAL_COMMAND_REBOOT, response) == LORASERIAL_SUCCESS) // Send command and get response
    {
        _inCommandMode = false;
        _lastSerialComm = millis();
        Serial.println("Out of command mode");
        return (true);
    }
    return (false);
}

// AT&W - stores all settings to NVM. Stays in command mode.
// NVM record takes 5.5ms.
bool LoRaSerial::saveSettings(void)
{
    char response[minimumResponseAllocation];
    if (getData(LORASERIAL_COMMAND_SAVESETTINGS, response) == LORASERIAL_SUCCESS) // Send command and get response
        return (true);
    return (false);
}

// AT&F - reset all settings to factory default, store to NVM, and respond with OK.
// Stays in command mode
// Command takes ~4ms to respond OK.
bool LoRaSerial::factoryReset(void)
{
    char response[minimumResponseAllocation];
    if (getData(LORASERIAL_COMMAND_FACTORYRESET, response) == LORASERIAL_SUCCESS) // Send command and get response
        return (true);
    return (false);
}

// ATT Enter training mode
bool LoRaSerial::enterTrainingMode(void)
{
    char response[minimumResponseAllocation];
    if (getData(LORASERIAL_COMMAND_TRAININGMODE, response) == LORASERIAL_SUCCESS) // Send command and get response
        return (true);
    return (false);
}






//Radio Commands
// AT-AirSpeed?
uint16_t LoRaSerial::getAirSpeed(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_AIRSPEED, response, true); // Send command and get response, add '?'

    int settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// AT-AirSpeed=9600
bool LoRaSerial::setAirSpeed(uint16_t airSpeed)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_AIRSPEED, (int)airSpeed);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// AT-AutoTune?
bool LoRaSerial::getAutoTune(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_AUTOTUNE, response, true); // Send command and get response, add '?'

    bool settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// AT-AutoTune=1
bool LoRaSerial::setAutoTune(bool autoTune)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_AUTOTUNE, autoTune);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// AT-Bandwidth?
float LoRaSerial::getBandwidth(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_BANDWIDTH, response, true); // Send command and get response, add '?'

    float settingValue = atof(response); // Convert char to float
    return (settingValue);
}

// AT-Bandwidth=31.25
bool LoRaSerial::setBandwidth(float bandwidth)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_BANDWIDTH, bandwidth);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

















// ATIx Information gets

// ATI, ATI0, ATI1 - Skip, human readable

// ATI2
float LoRaSerial::getVersion(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_VERSION, response); // Send command and get response

    char* ptr;
    float settingValue = strtod(response, &ptr); // Convert char to float
    return (settingValue);
}

// ATI3
int LoRaSerial::getRSSI(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_RSSI, response); // Send command and get response

    int settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATI4
// Based on RSSI. 0 to 255
byte LoRaSerial::getRandom(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_RANDOM, response); // Send command and get response

    int settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATI5
// Given spread factor, bandwidth, coding rate and frame size, return most bytes we can push per second
uint16_t LoRaSerial::getMaxThroughput(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_MAXTHROUGHPUT, response); // Send command and get response

    uint16_t settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATSx Parameter gets/sets

// ATS0?
uint16_t LoRaSerial::getSerialSpeed(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_SERIALSPEED, response, true); // Send command and get response, add '?'

    int settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS0=9600
bool LoRaSerial::setSerialSpeed(uint16_t baud)
{
    int b = 0;

    // Error check -- ensure supported baud
    for (; b < NUM_SUPPORTED_BAUD; b++) {
        if (LORASERIAL_SUPPORTED_BAUD[b] == baud)
            break;
    }
    if (b >= NUM_SUPPORTED_BAUD)
        return (false);

    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_SERIALSPEED, (int)baud);

    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}


// AT-NetID?
uint8_t LoRaSerial::getNetworkID(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_NETID, response, true); // Send command and get response, add '?'

    uint8_t settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// AT-NetID=42
bool LoRaSerial::setNetworkID(uint8_t networkID)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_NETID, networkID);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// AT-OperatingMode?
uint8_t LoRaSerial::getOperatingMode(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_OPERATINGMODE, response, true); // Send command and get response, add '?'

    bool settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS-OperatingMode=2
bool LoRaSerial::setOperatingMode(uint8_t operatingMode)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_OPERATINGMODE, operatingMode);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS4?
bool LoRaSerial::getEncryptData(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_ENCRYPTDATA, response, true); // Send command and get response, add '?'

    bool settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS4=1
bool LoRaSerial::setEncryptData(bool encryptData)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_ENCRYPTDATA, encryptData);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// AT-EncryptionKey?
// 128 bit, 16 byte HEX value, 32 bytes printed
void LoRaSerial::getAESKey(char* hexKey)
{
    char response[minimumResponseAllocation];
    // getData(LORASERIAL_COMMAND_ENCRYPTIONKEY, response, true); //Reponse not terminated in v1.0 so we use ATI6 instead
    getData(LORASERIAL_COMMAND_ENCRYPTIONKEY, response, false, LORASERIAL_AESKEY_TIMEOUT); // No '?', increase timeout wait

    strncpy(hexKey, response, 33); // 32 bytes + terminator
}

// AT-EncryptionKey=00112233445566778899AABBCCDDEEFF
bool LoRaSerial::setAESKey(const char* hexKey)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_ENCRYPTIONKEY, hexKey, LORASERIAL_AESKEY_TIMEOUT); // Increase wait time for ESP32s
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS6?
bool LoRaSerial::getDataScrambling(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_DATASCRAMBLING, response, true); // Send command and get response, add '?'

    bool settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS6=1
bool LoRaSerial::setDataScrambling(bool dataScrambling)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_DATASCRAMBLING, dataScrambling);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS7?
uint8_t LoRaSerial::getTxPower(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_TXPOWER, response, true); // Send command and get response, add '?'

    int settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS7=25
bool LoRaSerial::setTxPower(uint8_t txPower)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_TXPOWER, txPower);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS8?
float LoRaSerial::getFrequencyMin(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_FREQUENCYMIN, response, true); // Send command and get response, add '?'

    float settingValue = atof(response); // Convert char to float
    return (settingValue);
}

// ATS8=903.500
bool LoRaSerial::setFrequencyMin(float minFrequency)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_FREQUENCYMIN, minFrequency);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS9?
float LoRaSerial::getFrequencyMax(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_FREQUENCYMAX, response, true); // Send command and get response, add '?'

    float settingValue = atof(response); // Convert char to float
    return (settingValue);
}

// ATS9=910.125
bool LoRaSerial::setFrequencyMax(float maxFrequency)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_FREQUENCYMAX, maxFrequency);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS10?
uint8_t LoRaSerial::getNumberOfChannels(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_NUMBEROFCHANNELS, response, true); // Send command and get response, add '?'

    int settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS10=0
bool LoRaSerial::setNumberOfChannels(uint8_t numberOfChannels)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_NUMBEROFCHANNELS, numberOfChannels);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS11?
bool LoRaSerial::getFrequencyHop(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_FREQUENCYHOP, response, true); // Send command and get response, add '?'

    bool settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS11=0
bool LoRaSerial::setFrequencyHop(bool frequencyHop)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_FREQUENCYHOP, frequencyHop);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS12?
uint16_t LoRaSerial::getMaxDwellTime(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_MAXDWELLTIME, response, true); // Send command and get response, add '?'

    int settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS12=2000
bool LoRaSerial::setMaxDwellTime(uint16_t maxDwellTime)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_MAXDWELLTIME, (int)maxDwellTime);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}



// ATS14?
uint8_t LoRaSerial::getSpreadFactor(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_SPREADFACTOR, response, true); // Send command and get response, add '?'

    int settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS14=11
bool LoRaSerial::setSpreadFactor(uint8_t spreadFactor)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_SPREADFACTOR, spreadFactor);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS15?
uint8_t LoRaSerial::getCodingRate(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_CODINGRATE, response, true); // Send command and get response, add '?'

    int settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS15=8
bool LoRaSerial::setCodingRate(uint8_t codingRate)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_CODINGRATE, codingRate);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS16?
uint8_t LoRaSerial::getSyncWord(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_SYNCWORD, response, true); // Send command and get response, add '?'

    int settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS16=0xBC
bool LoRaSerial::setSyncWord(uint8_t syncWord)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_SYNCWORD, syncWord);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS17?
uint16_t LoRaSerial::getPreambleLength(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_PREAMBLELENGTH, response, true); // Send command and get response, add '?'

    int settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS17=44
bool LoRaSerial::setPreambleLength(uint16_t preambleLength)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_PREAMBLELENGTH, (int)preambleLength);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS21?
bool LoRaSerial::getEcho(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_ECHO, response, true); // Send command and get response, add '?'

    bool settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS21=1
bool LoRaSerial::setEcho(bool echo)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_ECHO, echo);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS22?
uint16_t LoRaSerial::getHeartBeatTimeout(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_HEARTBEATTIMEOUT, response, true); // Send command and get response, add '?'

    int settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS22=6500
bool LoRaSerial::setHeartBeatTimeout(uint16_t heartBeatTimeout)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_HEARTBEATTIMEOUT, (int)heartBeatTimeout);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}

// ATS23?
bool LoRaSerial::getFlowControl(void)
{
    char response[minimumResponseAllocation];
    getData(LORASERIAL_COMMAND_FLOWCONTROL, response, true); // Send command and get response, add '?'

    bool settingValue = atoi(response); // Convert char to int
    return (settingValue);
}

// ATS23=1
bool LoRaSerial::setFlowControl(bool flowControl)
{
    LoRaSerial_error_t err = setData(LORASERIAL_COMMAND_FLOWCONTROL, flowControl);
    if (err == LORASERIAL_SUCCESS)
        return (true);
    return (false);
}



// Given a command, get a char response
// If echo is on, unit will respond with 'AT-SERIALSPEED=9600\r\nOK\r\n'
// If echo is off, unit will respond with '\r\nOK\r\n'
// So do a string search first, it may work or fail
// Then do a sscanf to pull out char
LoRaSerial_error_t LoRaSerial::getData(const char* command, char* response, bool addQMark, unsigned long commandTimeout)
{
    LoRaSerial_error_t err;
    if (enterCommandMode() == false) // If needed, calls sendCommandWithResponse with +++
        return (LORASERIAL_ERROR_ERROR);

    char* stringToSend = loraSerial_calloc_char(strlen(command) + 2);
    if (stringToSend == NULL)
        return LORASERIAL_ERROR_OUT_OF_MEMORY;

    if (addQMark == false)
        sprintf(stringToSend, "%s", command);
    else
        sprintf(stringToSend, "%s?", command);

    err = sendCommandWithResponse(stringToSend, LORASERIAL_RESPONSE_OK_OR_ERROR,
        response, commandTimeout);

    char data[50] = { '\0' }; // AES key is 33 bytes

    // Serial.print("getData response: ");
    // Serial.println(response);

    // Deal with echo
    char* searchPtr = strstr(response, stringToSend);
    int variablesFound = 0;
    if (searchPtr != NULL) {
        variablesFound = sscanf(searchPtr + strlen(stringToSend) + 2, "%s", data);
    } else {
        // Serial.println("No AT command found");
        variablesFound = sscanf(response, "%s", data);
    }

    if (variablesFound > 0) {
        if (_printDebug == true) {
            _debugPort->print(F("data: "));
            _debugPort->println(data);
        }

        strcpy(response, data); // Hand the parsed data back to the caller
    } else {
        if (_printDebug == true)
            _debugPort->println(F("No variables found"));
    }

    free(stringToSend);

    return (err);
}

// Given a command and a value, return err value
// Echo doesn't matter since sendCommandWithReponse scans for OK or ERROR
LoRaSerial_error_t LoRaSerial::setData(const char* command, int settingValue, unsigned long commandTimeout)
{
    LoRaSerial_error_t err;
    if (enterCommandMode() == false) // If needed, calls sendCommandWithResponse with +++
        return (LORASERIAL_ERROR_ERROR);

    char response[minimumResponseAllocation];

    char* stringToSend = loraSerial_calloc_char(strlen("-") + strlen(command) + strlen("=") + numPlaces(settingValue) + strlen("\n\r")); //-SerialSpeed=57600\r\n
    if (stringToSend == NULL)
        return LORASERIAL_ERROR_OUT_OF_MEMORY;

    sprintf(stringToSend, "-%s=%d", command, settingValue);

    err = sendCommandWithResponse(stringToSend, LORASERIAL_RESPONSE_OK_OR_ERROR,
        response, commandTimeout); // Prepend command with AT

    free(stringToSend);

    return (err);
}

// Given a command and a value, return err value
// Echo doesn't matter since sendCommandWithReponse scans for OK or ERROR
LoRaSerial_error_t LoRaSerial::setData(const char* command, float settingValue, unsigned long commandTimeout)
{
    LoRaSerial_error_t err;
    if (enterCommandMode() == false) // If needed, calls sendCommandWithResponse with +++
        return (LORASERIAL_ERROR_ERROR);

    char response[minimumResponseAllocation];

    char* stringToSend = loraSerial_calloc_char(strlen(command) + 7 + 2); // 915.123 = 7 chars
    if (stringToSend == NULL)
        return LORASERIAL_ERROR_OUT_OF_MEMORY;

    // sprintf work-around
    char floatString[8];

    if (settingValue < 10)
        dtostrf(settingValue, 5, 3, floatString); // 1.234
    else if (settingValue < 100)
        dtostrf(settingValue, 6, 3, floatString); // 12.345
    else
        dtostrf(settingValue, 7, 3, floatString); // 123.456

    sprintf(stringToSend, "%s=%s", command, floatString);

    err = sendCommandWithResponse(stringToSend, LORASERIAL_RESPONSE_OK_OR_ERROR,
        response, commandTimeout); // Prepend command with AT

    free(stringToSend);

    return (err);
}

// Given a command and a value, return err value
// Echo doesn't matter since sendCommandWithReponse scans for OK or ERROR
LoRaSerial_error_t LoRaSerial::setData(const char* command, const char* settingValue, unsigned long commandTimeout)
{
    LoRaSerial_error_t err;
    if (enterCommandMode() == false) // If needed, calls sendCommandWithResponse with +++
        return (LORASERIAL_ERROR_ERROR);

    char response[minimumResponseAllocation];

    char* stringToSend = loraSerial_calloc_char(strlen(command) + strlen(settingValue) + 2);
    if (stringToSend == NULL)
        return LORASERIAL_ERROR_OUT_OF_MEMORY;

    sprintf(stringToSend, "%s=%s", command, settingValue);

    err = sendCommandWithResponse(stringToSend, LORASERIAL_RESPONSE_OK_OR_ERROR,
        response, commandTimeout); // Prepend command with AT

    free(stringToSend);

    return (err);
}

/////////////
// Private //
/////////////

bool LoRaSerial::init(unsigned long baud)
{
    int retries = _maxInitRetries;
    bool successfullyConnected = false;

    beginSerial(baud); // Start hardware at a given baud

    do {
        if (_printDebug == true)
            _debugPort->println(F("init: Begin module init."));

        // Quick test of the serial connection
        if (enterCommandMode() == true) {
            successfullyConnected = true;
            break; // Responded with 'OK' or 'ERROR'
        }

        // If that fails, begin autobaud process
        if (_printDebug == true)
            _debugPort->println(F("init: Attempting autobaud connection to module."));

        if (autobaud(baud) == true) {
            successfullyConnected = true;
            Serial.println("Autobaud worked!");
            break; // Responded 'OK'
        }
    } while ((retries--) && (successfullyConnected == false));

    // we tried but failed
    if (successfullyConnected == false) {
        if (_printDebug == true)
            _debugPort->println(F("init: Module failed to init. Exiting."));
        return (false);
    }

    if (_printDebug == true)
        _debugPort->println(F("init: Module responded successfully."));

    _baud = baud;

    return (true);
}

void LoRaSerial::beginSerial(unsigned long baud)
{
    if (_hardSerial != NULL) {
        _hardSerial->end();

        if (_rxPin != 255 && _txPin != 255)
            _hardSerial->begin(baud, SERIAL_8N1, _rxPin, _txPin); // ESP RX, TX
        else
            _hardSerial->begin(baud);
    }
#ifdef LORASERIAL_SOFTWARE_SERIAL_ENABLED
    else if (_softSerial != NULL) {
        _softSerial->end();
        _softSerial->begin(baud);
    }
#endif
}

// Begins serial at various baud rates
// Sends +++ to enter command mode, then AT looking for response
//+++ may respond with OK (entered command most first time) or ERROR (already in command mode)
bool LoRaSerial::autobaud(unsigned long desiredBaud)
{
    for (int b = 0; b < NUM_SUPPORTED_BAUD; b++) {
        if (_printDebug == true) {
            _debugPort->print(F("Pinging device at "));
            _debugPort->print(LORASERIAL_SUPPORTED_BAUD[b]);
            _debugPort->println(F("bps"));
        }

        beginSerial(LORASERIAL_SUPPORTED_BAUD[b]); // Move UART to this guessed baud rate

        if (setSerialSpeed(desiredBaud) == true) // This will enter command mode if the baud rate matches
        {
            if (_printDebug == true) {
                _debugPort->println(F("Device responded!"));
            }

            // Device entered command mode and responded with OK at the LORASERIAL_SUPPORTED_BAUD[b] baud
            // Save settings and reset so that the device moves to the new serial speed
            saveSettings();
            Serial.println("A");

            reset();
            Serial.println("B");

            delay(2000); // Wait for radio to reboot
            Serial.println("C");

            beginSerial(desiredBaud); // Move UART to the desired baud rate
            Serial.println("D");

            while (hwAvailable())
                readChar();

            return (true);
        }
    }
    return (false); // We failed to communicate
}

// Issue command with or without AT
// Scan for expected response. Give up after commandTimeout.
LoRaSerial_error_t LoRaSerial::sendCommandWithResponse(
    const char* command, const char* expectedResponse, char* responseDest,
    unsigned long commandTimeout, int destSize, bool at)
{
    bool found = false;
    bool error = false;
    int responseIndex = 0;
    int errorIndex = 0;
    int destIndex = 0;
    unsigned int charsRead = 0;
    int responseLen = 0;
    int errorLen = 0;
    const char* expectedError = NULL;
    bool printedSomething = true;

    if (_printDebug == true) {
        _debugPort->print(F("sendCommandWithResponse() Command: "));
        _debugPort->println(String(command));
    }

    memset(responseDest, 0, destSize);

    sendCommand(command, at);
    unsigned long startTime = millis();

    //  _hardSerial->println();
    //  while (1)
    //  {
    //    if (_hardSerial->available())
    //    {
    //      Serial.print("I heard: ");
    //      Serial.write(_hardSerial->read());
    //      Serial.println();
    //    }
    //  }

    printedSomething = false;

    if (expectedResponse == LORASERIAL_RESPONSE_OK_OR_ERROR) {
        expectedResponse = LORASERIAL_RESPONSE_OK;
        expectedError = LORASERIAL_RESPONSE_ERROR;
        responseLen = sizeof(LORASERIAL_RESPONSE_OK) - 1;
        errorLen = sizeof(LORASERIAL_RESPONSE_ERROR) - 1;
    } else {
        responseLen = (int)strlen(expectedResponse);
    }

    while ((found == false) && ((millis() - startTime) < commandTimeout)) {
        if (hwAvailable() > 0) // hwAvailable can return -1 if the serial port is NULL
        {
            char c = readChar();

            if (_printDebug == true) {
                if (printedSomething == false) {
                    //          _debugPort->print(F("sendCommandWithResponse() Response: "));
                    _debugPort->print(F("\n\rI heard: <"));
                    printedSomething = true;
                }
                _debugPort->write(c);
            }

            // Record this character into response
            if (responseDest != NULL) {
                if (destIndex < destSize) // Only add this char to response if there is room for it
                    responseDest[destIndex] = c;

                destIndex++;

                if (destIndex == destSize) {
                    if (_printDebug == true)
                        _debugPort->print(F("sendCommandWithResponse() Panic! responseDest is full!"));
                }
            }

            charsRead++;

            // Compare this character against expected error
            if ((errorIndex < errorLen) && (c == expectedError[errorIndex])) {
                if (++errorIndex == errorLen) {
                    error = true;
                    found = true;
                }
            } else {
                errorIndex = ((errorIndex < errorLen) && (c == expectedError[0])) ? 1 : 0;
            }

            // Compare this character against expected response
            if ((responseIndex < responseLen) && (c == expectedResponse[responseIndex])) {
                if (++responseIndex == responseLen) {
                    found = true;
                }
            } else {
                responseIndex = ((responseIndex < responseLen) && (c == expectedResponse[0])) ? 1 : 0;
            }
        } else {
            yield();
        }
    }

    if (_printDebug == true)
        if (printedSomething)
            _debugPort->println(F(">"));

    // If timeout, terminate array
    if ((millis() - startTime) > commandTimeout) {
        if (responseDest != NULL && destIndex < destSize)
            responseDest[destIndex] = '\0';
        else {
            if (_printDebug == true)
                _debugPort->println(F("responseDest NULL"));
        }

        if (_printDebug == true) {
            _debugPort->println(F("Response timeout"));
        }
    }

    if (found) {
        if ((true == _printDebug) && ((NULL != responseDest) || (NULL != expectedResponse))) {
            if (error)
                _debugPort->println(F("ERROR Found"));
            else {
                _debugPort->print(F("Found expected response: "));
                _debugPort->println(expectedResponse);
            }
        }
        return error ? LORASERIAL_ERROR_ERROR : LORASERIAL_SUCCESS;
    } else if (charsRead == 0) {
        if (_printDebug == true)
            _debugPort->println(F("No Response"));
        return LORASERIAL_ERROR_NO_RESPONSE;
    } else {
        if ((true == _printDebug) && (NULL != responseDest)) {
            _debugPort->print(F("Non-OK/ERROR Response: "));
            _debugPort->println(responseDest);
        }
        return LORASERIAL_ERROR_UNEXPECTED_RESPONSE;
    }
}

void LoRaSerial::sendCommand(const char* command, bool at)
{
    if (at) {
        hwPrint(LORASERIAL_COMMAND_AT);
        hwPrint(command);
        // hwPrint("\r\n"); //Disrupts software serial
        hwPrint("\r");
    } else {
        hwPrint(command);
    }
}

int LoRaSerial::hwAvailable(void)
{
    if (_hardSerial != NULL) {
        return _hardSerial->available();
    }
#ifdef LORASERIAL_SOFTWARE_SERIAL_ENABLED
    else if (_softSerial != NULL) {
        return _softSerial->available();
    }
#endif

    return -1;
}

size_t LoRaSerial::hwWriteData(const char* buff, int len)
{
    if ((true == _printAtDebug) && (NULL != buff) && (0 < len)) {
        _debugAtPort->write(buff, len);
    }
    if (_hardSerial != NULL) {
        return _hardSerial->write((const uint8_t*)buff, len);
    }
#ifdef LORASERIAL_SOFTWARE_SERIAL_ENABLED
    else if (_softSerial != NULL) {
        return _softSerial->write((const uint8_t*)buff, len);
    }
#endif
    return (size_t)0;
}

size_t LoRaSerial::hwWrite(const char c)
{
    if (true == _printAtDebug) {
        _debugAtPort->write(c);
    }
    if (_hardSerial != NULL) {
        return _hardSerial->write(c);
    }
#ifdef LORASERIAL_SOFTWARE_SERIAL_ENABLED
    else if (_softSerial != NULL) {
        return _softSerial->write(c);
    }
#endif

    return (size_t)0;
}

size_t LoRaSerial::hwPrint(const char* s)
{
    if ((true == _printAtDebug) && (NULL != s)) {
        _debugAtPort->print(s);
    }
    if (_hardSerial != NULL) {
        return _hardSerial->print(s);
    }
#ifdef LORASERIAL_SOFTWARE_SERIAL_ENABLED
    else if (_softSerial != NULL) {
        return _softSerial->print(s);
    }
#endif

    return (size_t)0;
}

int LoRaSerial::readAvailable(char* inString)
{
    int len = 0;

    if (_hardSerial != NULL) {
        while (_hardSerial->available()) {
            char c = (char)_hardSerial->read();
            if (inString != NULL) {
                inString[len++] = c;
            }
        }
        if (inString != NULL) {
            inString[len] = 0;
        }
        // if (_printDebug == true)
        //   _debugPort->println(inString);
    }
#ifdef LORASERIAL_SOFTWARE_SERIAL_ENABLED
    else if (_softSerial != NULL) {
        while (_softSerial->available()) {
            char c = (char)_softSerial->read();
            if (inString != NULL) {
                inString[len++] = c;
            }
        }
        if (inString != NULL) {
            inString[len] = 0;
        }
    }
#endif

    return len;
}

char LoRaSerial::readChar(void)
{
    char ret = 0;

    if (_hardSerial != NULL) {
        ret = (char)_hardSerial->read();
    }
#ifdef LORASERIAL_SOFTWARE_SERIAL_ENABLED
    else if (_softSerial != NULL) {
        ret = (char)_softSerial->read();
    }
#endif

    return ret;
}

char* LoRaSerial::loraSerial_calloc_char(size_t num)
{
    return (char*)calloc(num, sizeof(char));
}

// Return the character length of a given number
// https://stackoverflow.com/questions/1068849/how-do-i-determine-the-number-of-digits-of-an-integer-in-c
uint8_t LoRaSerial::numPlaces(int32_t n)
{
    if (n < 0)
        n = -n;
    if (n < 10)
        return 1;
    if (n < 100)
        return 2;
    if (n < 1000)
        return 3;
    if (n < 10000)
        return 4;
    if (n < 100000)
        return 5;
    if (n < 1000000)
        return 6;
    if (n < 10000000)
        return 7;
    if (n < 100000000)
        return 8;
    if (n < 1000000000)
        return 9;
    /*      2147483647 is 2^31-1 - add more ifs as needed
       and adjust this final return as well. */
    return 10;
}