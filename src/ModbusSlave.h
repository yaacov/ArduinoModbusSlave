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

  Stream &_serialStream;

#if defined(SERIAL_TX_BUFFER_SIZE)
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
