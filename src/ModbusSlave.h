/**
 * Copyright (c) 2015, Yaacov Zamir <kobi.zamir@gmail.com>
 * Copyright (c) 2017, Andrew Voznytsa <andrew.voznytsa@gmail.com>, FC_WRITE_REGISTER and FC_WRITE_MULTIPLE_COILS support
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF  THIS SOFTWARE.
 */

#ifndef MODBUSSLAVE_H
#define MODBUSSLAVE_H
#include <Arduino.h>

#define MODBUS_MAX_BUFFER 256
#define MODBUS_INVALID_UNIT_ADDRESS 255
#define MODBUS_DEFAULT_UNIT_ADDRESS 1
#define MODBUS_CONTROL_PIN_NONE -1

// CRC Calc with CRC Lookup Table. Save CPU Cicles.
// #define CRC_LTABLE_CALC


#if defined (ESP32) || defined (ESP8266)
  #define SERIAL_BUFFER_SIZE 256
#endif

#if defined CRC_LTABLE_CALC
static const uint16_t wCRCTable[] PROGMEM = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};
#endif

/**
 * Modbus function codes
 */
enum
{
  FC_INVALID = 0,
  FC_READ_COILS = 1,
  FC_READ_DISCRETE_INPUT = 2,
  FC_READ_HOLDING_REGISTERS = 3,
  FC_READ_INPUT_REGISTERS = 4,
  FC_WRITE_COIL = 5,
  FC_WRITE_REGISTER = 6,
  FC_READ_EXCEPTION_STATUS = 7,
  FC_WRITE_MULTIPLE_COILS = 15,
  FC_WRITE_MULTIPLE_REGISTERS = 16
};

enum
{
  CB_READ_COILS = 0,
  CB_READ_DISCRETE_INPUTS,
  CB_READ_HOLDING_REGISTERS,
  CB_READ_INPUT_REGISTERS,
  CB_WRITE_COILS,
  CB_WRITE_HOLDING_REGISTERS,
  CB_READ_EXCEPTION_STATUS,
  CB_MAX
};

enum
{
  COIL_OFF = 0x0000,
  COIL_ON = 0xff00
};

enum
{
  STATUS_OK = 0,
  STATUS_ILLEGAL_FUNCTION,
  STATUS_ILLEGAL_DATA_ADDRESS,
  STATUS_ILLEGAL_DATA_VALUE,
  STATUS_SLAVE_DEVICE_FAILURE,
  STATUS_ACKNOWLEDGE,
  STATUS_SLAVE_DEVICE_BUSY,
  STATUS_NEGATIVE_ACKNOWLEDGE,
  STATUS_MEMORY_PARITY_ERROR,
  STATUS_GATEWAY_PATH_UNAVAILABLE,
  STATUS_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND,
};

typedef uint8_t (*ModbusCallback)(uint8_t, uint16_t, uint16_t);

/**
 * @class ModbusSlave
 */
class ModbusSlave
{
public:
  ModbusSlave(uint8_t unitAddress = MODBUS_DEFAULT_UNIT_ADDRESS);
  uint8_t getUnitAddress();
  void setUnitAddress(uint8_t unitAddress);
  ModbusCallback cbVector[CB_MAX];

private:
  uint8_t _unitAddress = MODBUS_DEFAULT_UNIT_ADDRESS;
};

/**
 * @class Modbus
 */
class Modbus
{
public:
  Modbus(uint8_t unitAddress = MODBUS_DEFAULT_UNIT_ADDRESS, int transmissionControlPin = MODBUS_CONTROL_PIN_NONE);
  Modbus(ModbusSlave *slaves, uint8_t numberOfSlaves, int transmissionControlPin = MODBUS_CONTROL_PIN_NONE);
  Modbus(Stream &serialStream, uint8_t unitAddress = MODBUS_DEFAULT_UNIT_ADDRESS, int transmissionControlPin = MODBUS_CONTROL_PIN_NONE);
  Modbus(Stream &serialStream, ModbusSlave *slaves, uint8_t numberOfSlaves, int transmissionControlPin = MODBUS_CONTROL_PIN_NONE);

  void begin(uint64_t boudRate);
  void setUnitAddress(uint8_t unitAddress);
  void enable();
  void disable();
  uint8_t poll();

  bool readCoilFromBuffer(int offset);
  uint16_t readRegisterFromBuffer(int offset);
  uint8_t writeExceptionStatusToBuffer(int offset, bool status);
  uint8_t writeCoilToBuffer(int offset, bool state);
  uint8_t writeDiscreteInputToBuffer(int offset, bool state);
  uint8_t writeRegisterToBuffer(int offset, uint16_t value);
  uint8_t writeArrayToBuffer(int offset, uint16_t *str, uint8_t length);

  uint8_t readFunctionCode();
  uint8_t readUnitAddress();
  bool readEnabled();
  bool isBroadcast();

  uint64_t getTotalBytesSent();
  uint64_t getTotalBytesReceived();

  // This cbVector is a pointer to cbVector of the first slave, to allow shorthand syntax:
  //     Modbus slave(SLAVE_ID, CTRL_PIN);
  //     slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
  // Instead of the complete:
  //     ModbusSlave slaves[1] = { ModbusSlave(ID_SLAVE_1) };
  //     Modbus modbus(slaves, 1);
  //     slaves[0].cbVector[CB_WRITE_COILS] = writeDigitalOut;
  ModbusCallback *cbVector;

private:
  ModbusSlave *_slaves = new ModbusSlave();
  uint8_t _numberOfSlaves = 1;

  bool _enabled = true;

  Stream &_serialStream;

#if defined(SERIAL_TX_BUFFER_SIZE) && !defined (ESP32) && !defined (ESP8266)
  int _serialTransmissionBufferLength = SERIAL_TX_BUFFER_SIZE;
#else
  int _serialTransmissionBufferLength = SERIAL_BUFFER_SIZE;
#endif

  int _transmissionControlPin = MODBUS_CONTROL_PIN_NONE;

  uint16_t _halfCharTimeInMicroSecond;
  uint64_t _lastCommunicationTime;

  uint8_t _requestBuffer[MODBUS_MAX_BUFFER];
  uint16_t _requestBufferLength = 0;
  bool _isRequestBufferReading = false;

  uint8_t _responseBuffer[MODBUS_MAX_BUFFER];
  uint16_t _responseBufferLength = 0;
  bool _isResponseBufferWriting = false;
  uint16_t _responseBufferWriteIndex = 0;

  uint64_t _totalBytesSent = 0;
  uint64_t _totalBytesReceived = 0;

  bool relevantAddress(uint8_t unitAddress);
  bool readRequest();
  bool validateRequest();
  uint8_t createResponse();
  uint8_t executeCallback(uint8_t slaveAddress, uint8_t callbackIndex, uint16_t address, uint16_t length);
  uint16_t writeResponse();
  uint16_t reportException(uint8_t exceptionCode);
  uint16_t calculateCRC(uint8_t *buffer, int length);
};
#endif
