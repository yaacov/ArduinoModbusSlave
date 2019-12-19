/**
 * Copyright (c) 2015, Yaacov Zamir <kobi.zamir@gmail.com>
 * Copyright (c) 2017, Andrew Voznytsa <andrew.voznytsa@gmail.com>, FC_WRITE_REGISTER and FC_WRITE_MULTIPLE_COILS support
 * Copyright (c) 2019, Soroush Falahati <soroush@falahai.net>, total communication rewrite, setUnitAddress(), FC_READ_EXCEPTION_STATUS support, general refactoring
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

#include <string.h>
#include "ModbusSlave.h"

/**
 * ---------------------------------------------------
 *                CONSTANTS AND MACROS
 * ---------------------------------------------------
 */

#define MODBUS_FRAME_SIZE 4
#define MODBUS_CRC_LENGTH 2

#define MODBUS_ADDRESS_INDEX 0
#define MODBUS_FUNCTION_CODE_INDEX 1
#define MODBUS_DATA_INDEX 2

#define MODBUS_BROADCAST_ADDRESS 0
#define MODBUS_ADDRESS_MIN 1
#define MODBUS_ADDRESS_MAX 247

#define MODBUS_HALF_SILENCE_MULTIPLIER 3
#define MODBUS_FULL_SILENCE_MULTIPLIER 7

#define readUInt16(arr, index) word(arr[index], arr[index + 1])
#define readCRC(arr, length) word(arr[(length - MODBUS_CRC_LENGTH) + 1], arr[length - MODBUS_CRC_LENGTH])

/**
 * ---------------------------------------------------
 *                  PUBLIC METHODS
 * ---------------------------------------------------
 */

/**
 * Initialize a modbus slave object.
 *
 * @param unitAddress the modbus slave unit address.
 */
ModbusSlave::ModbusSlave(uint8_t unitAddress)
{
    ModbusSlave::setUnitAddress(unitAddress);
}

/**
 * Get the modbus slaves unit address.
 */
uint8_t ModbusSlave::getUnitAddress()
{
    return _unitAddress;
}

/**
 * Sets the modbus slaves unit address.
 *
 * @param unitAddress the modbus slaves unit address.
 */
void ModbusSlave::setUnitAddress(uint8_t unitAddress)
{
    if (unitAddress < MODBUS_ADDRESS_MIN || unitAddress > MODBUS_ADDRESS_MAX)
    {
        return;
    }
    _unitAddress = unitAddress;
}

/**
 * Initialize the modbus object.
 *
 * @param unitAddress the modbus slave unit address.
 * @param transmissionControlPin the digital out pin to be used for RS485.
 * transmission control.
 */
Modbus::Modbus(uint8_t unitAddress, int transmissionControlPin)
    : Modbus(Serial, unitAddress, transmissionControlPin)
{
}

/**
 * Initialize the modbus object.
 *
 * @param serialStream the serial stream used for the modbus communication.
 * @param unitAddress the modbus slave unit address.
 * @param transmissionControlPin the digital out pin to be used for RS485.
 * transmission control.
 */
Modbus::Modbus(Stream &serialStream, uint8_t unitAddress, int transmissionControlPin)
    : _serialStream(serialStream)
{
    // set modbus slave unit id
    _slaves[0].setUnitAddress(unitAddress);
    cbVector = _slaves[0].cbVector;

    // set transmission control pin for RS485 communications.
    _transmissionControlPin = transmissionControlPin;
}

/**
 * Initialize the modbus object.
 *
 * @param slaves Pointer to an array of ModbusSlaves.
 * @param numberOfSlaves The number of ModbusSlaves in the array.
 * @param transmissionControlPin the digital out pin to be used for RS485.
 * transmission control.
 */
Modbus::Modbus(ModbusSlave* slaves, uint8_t numberOfSlaves, int transmissionControlPin)
    : Modbus(Serial, slaves, numberOfSlaves, transmissionControlPin)
{
}

/**
 * Initialize the modbus object.
 *
 * @param serialStream the serial stream used for the modbus communication.
 * @param slaves Pointer to an array of ModbusSlaves.
 * @param numberOfSlaves The number of ModbusSlaves in the array.
 * @param transmissionControlPin the digital out pin to be used for RS485.
 * transmission control.
 */
Modbus::Modbus(Stream &serialStream, ModbusSlave* slaves, uint8_t numberOfSlaves, int transmissionControlPin)
    : _serialStream(serialStream)
{
    // set modbus slaves
    _slaves = slaves;
    _numberOfSlaves = numberOfSlaves;
    cbVector = _slaves[0].cbVector;

    // set transmission control pin for RS485 communications.
    _transmissionControlPin = transmissionControlPin;
}

/**
 * Sets the modbus slaves unit address.
 *
 * @param unitAddress the modbus slaves unit address.
 */
void Modbus::setUnitAddress(uint8_t unitAddress)
{
    _slaves[0].setUnitAddress(unitAddress);
}


/**
 * Gets the total number of bytes sent.
 *
 * @return the number of bytes.
 */
uint64_t Modbus::getTotalBytesSent() {
    return _totalBytesSent;
}

/**
 * Gets the total number of bytes received.
 *
 * @return the number of bytes.
 */
uint64_t Modbus::getTotalBytesReceived() {
    return _totalBytesReceived;
}

/**
 * Begins initializing the serial stream and preparing to read request messages.
 *
 * @param boudrate the serial port boudrate.
 */
void Modbus::begin(uint64_t boudrate)
{
    // initialize transmission control pin state
    if (_transmissionControlPin > MODBUS_CONTROL_PIN_NONE)
    {
        pinMode(_transmissionControlPin, OUTPUT);
        digitalWrite(_transmissionControlPin, LOW);
    }

    // disable serial stream timeout and cleans the buffer
    _serialStream.setTimeout(0);
    _serialStream.flush();
    _serialTransmissionBufferLength = _serialStream.availableForWrite();

    // calculate half char time time from the serial's baudrate
    if (boudrate > 19200)
    {
        _halfCharTimeInMicroSecond = 250; // 0.5T
    }
    else
    {
        _halfCharTimeInMicroSecond = 5000000 / boudrate; // 0.5T
    }

    // set the last received time to 3.5T on the future to ignore
    // request currently in the middle of transmission
    _lastCommunicationTime = micros() + (_halfCharTimeInMicroSecond * MODBUS_FULL_SILENCE_MULTIPLIER);

    // sets the request buffer length to zero
    _requestBufferLength = 0;
}

/**
 * Checks if we have a complete request, parses the request, executes the
 * corresponding registered callback and writes the response.
 *
 * @return the number of bytes written as response
 */
uint8_t Modbus::poll()
{
    // if we are in the writing, let it end first
    if (_isResponseBufferWriting) 
    {
        return Modbus::writeResponse();
    }

    // wait for one complete request packet
    if (!Modbus::readRequest())
    {
        return 0;
    }

    // prepare output buffer
    memset(_responseBuffer, 0, MODBUS_MAX_BUFFER);
    _responseBuffer[MODBUS_ADDRESS_INDEX] = _requestBuffer[MODBUS_ADDRESS_INDEX];
    _responseBuffer[MODBUS_FUNCTION_CODE_INDEX] = _requestBuffer[MODBUS_FUNCTION_CODE_INDEX];
    _responseBufferLength = MODBUS_FRAME_SIZE;

    // validate request
    if (!Modbus::validateRequest())
    {
        return 0;
    }

    // execute request and fill the response
    uint8_t status = Modbus::createResponse();

    // check if the callback execution failed
    if (status != STATUS_OK)
    {
        return Modbus::reportException(status);
    }

    // writes the response being created
    return Modbus::writeResponse();
}

/**
 * Reads and returns the current request message's function code
 *
 * @return a byte containing the current request message function code
 */
uint8_t Modbus::readFunctionCode()
{
    if (_requestBufferLength >= MODBUS_FRAME_SIZE && !_isRequestBufferReading)
    {
        return _requestBuffer[MODBUS_FUNCTION_CODE_INDEX];
    }
    return FC_INVALID;
}

/**
 * Reads and returns the current request message's target unit address
 *
 * @return a byte containing the current request message unit address
 */
uint8_t Modbus::readUnitAddress()
{
    if ((_requestBufferLength >= MODBUS_FRAME_SIZE) && !_isRequestBufferReading)
    {
        return _requestBuffer[MODBUS_ADDRESS_INDEX];
    }
    return MODBUS_INVALID_UNIT_ADDRESS;
}

/**
 * Returns a boolean value indicating if the request currently being processed 
 * is a broadcast message and therefore does not need a response.
 *
 * @return true if the current request message is a broadcase message; otherwise
 * false
 */
bool Modbus::isBroadcast()
{
    return Modbus::readUnitAddress() == MODBUS_BROADCAST_ADDRESS;
}

/**
 * Read coil state from input buffer.
 *
 * @param offset the coil offset from first coil in this buffer.
 * @return the coil state from buffer (true / false)
 */
bool Modbus::readCoilFromBuffer(int offset)
{
    if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] == FC_WRITE_COIL)
    {
        if (offset == 0)
        {
            // (2 x coilAddress, 1 x value)
            return readUInt16(_requestBuffer, MODBUS_DATA_INDEX + 2) == COIL_ON;
        }
        return false;
    }
    else if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] == FC_WRITE_MULTIPLE_COILS)
    {
        // (2 x firstCoilAddress, 2 x coilsCount, 1 x valueBytes, n x values)
        uint16_t index = MODBUS_DATA_INDEX + 5 + (offset / 8);
        uint8_t bitIndex = offset % 8;

        // check offset
        if (index < _requestBufferLength - MODBUS_CRC_LENGTH)
        {
            return bitRead(_requestBuffer[index], bitIndex);
        }
    }
    return false;
}

/**
 * Read register value from input buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @return the register value from buffer.
 */
uint16_t Modbus::readRegisterFromBuffer(int offset)
{
    if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] == FC_WRITE_REGISTER)
    {
        if (offset == 0)
        {
            // (2 x coilAddress, 2 x value)
            return readUInt16(_requestBuffer, MODBUS_DATA_INDEX + 2);
        }
    }
    else if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] == FC_WRITE_MULTIPLE_REGISTERS)
    {
        // (2 x firstRegisterAddress, 2 x registersCount, 1 x valueBytes, n x values)
        uint16_t index = MODBUS_DATA_INDEX + 5 + (offset * 2);

        // check offset
        if (index < _requestBufferLength - MODBUS_CRC_LENGTH)
        {
            return readUInt16(_requestBuffer, index);
        }
    }
    return 0;
}

/**
 * Writes exception status to buffer
 *
 * @param offset the exception status flag.
 * @param the exception status flag (true / false)
 */
uint8_t Modbus::writeExceptionStatusToBuffer(int offset, bool status)
{
    // check function code
    if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] != FC_READ_EXCEPTION_STATUS) {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    // (1 x values)
    uint16_t index = MODBUS_DATA_INDEX;
    uint8_t bitIndex = offset % 8;

    // check offset
    if (index >= _responseBufferLength - MODBUS_CRC_LENGTH)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    if (status)
    {
        bitSet(_responseBuffer[index], bitIndex);
    }
    else
    {
        bitClear(_responseBuffer[index], bitIndex);
    }

    return STATUS_OK;
}

/**
 * Writes coil state to output buffer.
 *
 * @param offset the coil offset from first coil in this buffer.
 * @param state the coil state to write into buffer (true / false)
 */
uint8_t Modbus::writeCoilToBuffer(int offset, bool state)
{
    // check function code
    if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] != FC_READ_DISCRETE_INPUT && 
        _requestBuffer[MODBUS_FUNCTION_CODE_INDEX] != FC_READ_COILS) {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    // (1 x valueBytes, n x values)
    uint16_t index = MODBUS_DATA_INDEX + 1 + (offset / 8);
    uint8_t bitIndex = offset % 8;

    // check offset
    if (index >= _responseBufferLength - MODBUS_CRC_LENGTH)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    if (state)
    {
        bitSet(_responseBuffer[index], bitIndex);
    }
    else
    {
        bitClear(_responseBuffer[index], bitIndex);
    }
    
    return STATUS_OK;
}

/**
 * Writes digital input to output buffer.
 *
 * @param offset the input offset from first input in this buffer.
 * @param state the digital input state to write into buffer (true / false)
 */
uint8_t Modbus::writeDiscreteInputToBuffer(int offset, bool state)
{
    return Modbus::writeCoilToBuffer(offset, state);
}

/**
 * Writes register value to output buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @param value the register value to write into buffer.
 */
uint8_t Modbus::writeRegisterToBuffer(int offset, uint16_t value)
{
    // check function code
    if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] != FC_READ_HOLDING_REGISTERS && 
        _requestBuffer[MODBUS_FUNCTION_CODE_INDEX] != FC_READ_INPUT_REGISTERS) {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    // (1 x valueBytes, n x values)
    uint16_t index = MODBUS_DATA_INDEX + 1 + (offset * 2);

    // check offset
    if ((index + 2) > (_responseBufferLength - MODBUS_CRC_LENGTH))
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    _responseBuffer[index] = value >> 8;
    _responseBuffer[index + 1] = value & 0xff;
    
    return STATUS_OK;
}

/**
 * Writes arbitrary string of uint8_t to output buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @param str the string to write into buffer.
 * @param length the string length.
 * @return STATUS_OK in case of success, STATUS_ILLEGAL_DATA_ADDRESS
 *      if data doesn't fit in buffer
 */
uint8_t Modbus::writeStringToBuffer(int offset, uint8_t *str, uint8_t length)
{
    // (1 x valueBytes, n x values)
    uint8_t index = MODBUS_DATA_INDEX + 1 + (offset * 2);

    // check string length.
    if ((index + length) > _responseBufferLength - MODBUS_CRC_LENGTH)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    memcpy(_responseBuffer + index, str, length);

    return STATUS_OK;
}

/**
 * ---------------------------------------------------
 *                  PRIVATE METHODS
 * ---------------------------------------------------
 */

/**
 * Reads a new request from the serial stream and fills the request buffer
 *
 * @return true if the buffer is filled with a request and is ready to
 * be processed; otherwise false.
 */
bool Modbus::readRequest()
{
    /**
     * Read one data packet and report when received completely
     */
    uint16_t lenght = _serialStream.available();
    if (lenght > 0)
    {
        // if not yet started reading
        if (!_isRequestBufferReading)
        {
            // And it already took 1.5T from the last message
            if ((micros() - _lastCommunicationTime) > (_halfCharTimeInMicroSecond * MODBUS_HALF_SILENCE_MULTIPLIER))
            {
                // start reading and clear buffer
                _requestBufferLength = 0;
                _isRequestBufferReading = true;
            }
            else
            {
                // discard data
                _serialStream.read();
            }
        }

        // if already in reading
        if (_isRequestBufferReading)
        {
            if (_requestBufferLength == MODBUS_MAX_BUFFER)
            {
                // buffer is already full; stop reading
                _isRequestBufferReading = false;
            }

            // add new bytes to buffer
            lenght = min(lenght, MODBUS_MAX_BUFFER - _requestBufferLength);
            lenght = _serialStream.readBytes(_requestBuffer + _requestBufferLength, MODBUS_MAX_BUFFER - _requestBufferLength);

            // if this is the first read, check the address to reject irrelevant requests
            if (_requestBufferLength == 0 && lenght > MODBUS_ADDRESS_INDEX && !Modbus::relevantAddress(_requestBuffer[MODBUS_ADDRESS_INDEX]))
            {
                // bad address, stop reading
                _isRequestBufferReading = false;
            }

            // move byte pointer forward
            _requestBufferLength += lenght;
            _totalBytesReceived += lenght;
        }

        // save the time of last received byte(s)
        _lastCommunicationTime = micros();

        // wait for more data
        return false;
    }
    else
    {
        // if we are in reading but no data is available for 1.5T; this request is completed
        if (_isRequestBufferReading && ((micros() - _lastCommunicationTime) > (_halfCharTimeInMicroSecond * MODBUS_HALF_SILENCE_MULTIPLIER)))
        {
            // allow for new requests to be processed
            _isRequestBufferReading = false;
        }
        else
        {
            // otherwise, wait
            return false;
        }
    }

    return !_isRequestBufferReading && (_requestBufferLength >= MODBUS_FRAME_SIZE);
}

/**
 * Returns if one of the slaves has serves the given address.
 *
 * @param unitAddress The address that was received.
 */
bool Modbus::relevantAddress(uint8_t unitAddress)
{
    if (unitAddress == MODBUS_BROADCAST_ADDRESS)
        return true;
    for (uint8_t i = 0; i < _numberOfSlaves; ++i)
    {
        if (_slaves[i].getUnitAddress() == unitAddress)
            return true;
    }
    return false;
}

/**
 * Validates the request message currently in the input buffer.
 *
 * @return true if the request is valid; otherwise false
 */
bool Modbus::validateRequest()
{
    // minimum buffer size (1 x Address, 1 x Function, n x Data, 2 x CRC)
    uint16_t expected_requestBufferSize = MODBUS_FRAME_SIZE;
    // check data validity based on the function code
    switch (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX])
    {
    case FC_READ_EXCEPTION_STATUS:
        // broadcast is not supported
        if (_requestBuffer[MODBUS_ADDRESS_INDEX] == MODBUS_BROADCAST_ADDRESS)
        {
            // ignore
            return false;
        }
        break;
    case FC_READ_COILS:             // read coils (digital read)
    case FC_READ_DISCRETE_INPUT:    // read input state (digital read)
    case FC_READ_HOLDING_REGISTERS: // read holding registers (analog read)
    case FC_READ_INPUT_REGISTERS:   // read input registers (analog read)
        // broadcast is not supported
        if (_requestBuffer[MODBUS_ADDRESS_INDEX] == MODBUS_BROADCAST_ADDRESS)
        {
            // ignore
            return false;
        }
        // (2 x Index, 2 x Count)
        expected_requestBufferSize += 4;
        break;
    case FC_WRITE_COIL:     // write coils (digital write)
    case FC_WRITE_REGISTER: // write regosters (digital write)
        // (2 x Index, 2 x Count)
        expected_requestBufferSize += 4;
        break;
    case FC_WRITE_MULTIPLE_COILS:
    case FC_WRITE_MULTIPLE_REGISTERS:
        // (2 x Index, 2 x Count, 1 x Bytes)
        expected_requestBufferSize += 5;
        if (_requestBufferLength >= expected_requestBufferSize)
        {
            // (n x Bytes)
            expected_requestBufferSize += _requestBuffer[6];
        }
        break;
    default:
        // unknown command
        Modbus::reportException(STATUS_ILLEGAL_FUNCTION);
        return false;
    }

    if (_requestBufferLength < expected_requestBufferSize)
    {
        // data is smaller than expected, ignore
        return false;
    }

    // set correct data size
    _requestBufferLength = expected_requestBufferSize;

    // check crc
    uint16_t crc = readCRC(_requestBuffer, _requestBufferLength);
    if (Modbus::calculateCRC(_requestBuffer, _requestBufferLength - MODBUS_CRC_LENGTH) != crc)
    {
        // ignore
        return false;
    }

    return true;
}

/**
 * Fills the output buffer with the response to the request already in the
 * input buffer.
 *
 * @return the status code representing the success of this operation
 */
uint8_t Modbus::createResponse()
{
    uint16_t firstAddress;
    uint16_t addressesLength;
    uint8_t callbackIndex;
    uint16_t requestUnitAddress = _requestBuffer[MODBUS_ADDRESS_INDEX];

    /**
     * Match the function code with a callback and execute it
     * as well as preparing the response buffer
     */
    switch (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX])
    {
    case FC_READ_EXCEPTION_STATUS:
        // add response data length to output buffer length
        _responseBufferLength += 1;

        // execute callback and return the status code
        return Modbus::executeCallback(requestUnitAddress, CB_READ_EXCEPTION_STATUS, 0, 8);
    case FC_READ_COILS:          // read coils (digital out state)
    case FC_READ_DISCRETE_INPUT: // read input state (digital in)
        // read the the first input address and the number of inputs
        firstAddress = readUInt16(_requestBuffer, MODBUS_DATA_INDEX);
        addressesLength = readUInt16(_requestBuffer, MODBUS_DATA_INDEX + 2);

        // calculate response data length and add to output buffer length
        _responseBuffer[MODBUS_DATA_INDEX] = (addressesLength / 8) + (addressesLength % 8 != 0);
        _responseBufferLength += 1 + _responseBuffer[MODBUS_DATA_INDEX];

        // execute callback and return the status code
        callbackIndex = _requestBuffer[MODBUS_FUNCTION_CODE_INDEX] == FC_READ_COILS ? CB_READ_COILS : CB_READ_DISCRETE_INPUTS;
        return Modbus::executeCallback(requestUnitAddress, callbackIndex, firstAddress, addressesLength);
    case FC_READ_HOLDING_REGISTERS: // read holding registers (analog out state)
    case FC_READ_INPUT_REGISTERS:   // read input registers (analog in)
        // read the starting address and the number of inputs
        firstAddress = readUInt16(_requestBuffer, MODBUS_DATA_INDEX);
        addressesLength = readUInt16(_requestBuffer, MODBUS_DATA_INDEX + 2);

        // calculate response data length and add to output buffer length
        _responseBuffer[MODBUS_DATA_INDEX] = 2 * addressesLength;
        _responseBufferLength += 1 + _responseBuffer[MODBUS_DATA_INDEX];

        // execute callback and return the status code
        callbackIndex = _requestBuffer[MODBUS_FUNCTION_CODE_INDEX] == FC_READ_HOLDING_REGISTERS ? CB_READ_HOLDING_REGISTERS : CB_READ_INPUT_REGISTERS;
        return Modbus::executeCallback(requestUnitAddress, callbackIndex, firstAddress, addressesLength);
    case FC_WRITE_COIL: // write one coil (digital out)
        // read the address
        firstAddress = readUInt16(_requestBuffer, MODBUS_DATA_INDEX);

        // add response data length to output buffer length
        _responseBufferLength += 4;
        // copy parts of the request data that need to be in the response data
        memcpy(_responseBuffer + MODBUS_DATA_INDEX, _requestBuffer + MODBUS_DATA_INDEX, _responseBufferLength - MODBUS_FRAME_SIZE);

        // execute callback and return the status code
        return Modbus::executeCallback(requestUnitAddress, CB_WRITE_COILS, firstAddress, 1);
    case FC_WRITE_REGISTER:
        // read the address
        firstAddress = readUInt16(_requestBuffer, MODBUS_DATA_INDEX);

        // add response data length to output buffer length
        _responseBufferLength += 4;
        // copy parts of the request data that need to be in the response data
        memcpy(_responseBuffer + MODBUS_DATA_INDEX, _requestBuffer + MODBUS_DATA_INDEX, _responseBufferLength - MODBUS_FRAME_SIZE);

        // execute callback and return the status code
        return Modbus::executeCallback(requestUnitAddress, CB_WRITE_HOLDING_REGISTERS, firstAddress, 1);
    case FC_WRITE_MULTIPLE_COILS: // write coils (digital out)
        // read the starting address and the number of outputs
        firstAddress = readUInt16(_requestBuffer, MODBUS_DATA_INDEX);
        addressesLength = readUInt16(_requestBuffer, MODBUS_DATA_INDEX + 2);

        // add response data length to output buffer length
        _responseBufferLength += 4;
        // copy parts of the request data that need to be in the response data
        memcpy(_responseBuffer + MODBUS_DATA_INDEX, _requestBuffer + MODBUS_DATA_INDEX, _responseBufferLength - MODBUS_FRAME_SIZE);
        
        // execute callback and return the status code
        return Modbus::executeCallback(requestUnitAddress, CB_WRITE_COILS, firstAddress, addressesLength);
    case FC_WRITE_MULTIPLE_REGISTERS: // write holding registers (analog out)
        // read the starting address and the number of outputs
        firstAddress = readUInt16(_requestBuffer, MODBUS_DATA_INDEX);
        addressesLength = readUInt16(_requestBuffer, MODBUS_DATA_INDEX + 2);

        // add response data length to output buffer length
        _responseBufferLength += 4;
        // copy parts of the request data that need to be in the response data
        memcpy(_responseBuffer + MODBUS_DATA_INDEX, _requestBuffer + MODBUS_DATA_INDEX, _responseBufferLength - MODBUS_FRAME_SIZE);

        // execute callback and return the status code
        return Modbus::executeCallback(requestUnitAddress, CB_WRITE_HOLDING_REGISTERS, firstAddress, addressesLength);
    default:
        return STATUS_ILLEGAL_FUNCTION;
    }
}

/**
 * Executes a callback
 *
 * @return the status code representing the success of this operation
 */
uint8_t Modbus::executeCallback(uint8_t slaveAddress, uint8_t callbackIndex, uint16_t address, uint16_t length)
{
    for (uint8_t i = 0; i < _numberOfSlaves; ++i)
    {
        if (_slaves[i].getUnitAddress() == slaveAddress)
        {
            if (_slaves[i].cbVector[callbackIndex])
            {
                return _slaves[i].cbVector[callbackIndex](Modbus::readFunctionCode(), address, length);
            }
            else
            {
                return STATUS_ILLEGAL_FUNCTION;
            }
        }
    }
    return STATUS_ILLEGAL_FUNCTION;
}

/**
 * Writes the output buffer to serial stream
 *
 * @return The number of bytes written
 */
uint16_t Modbus::writeResponse()
{
    /**
     * Validate
     */

    // check if there is a response and this is supposed to be the first write
    if (_responseBufferWriteIndex == 0 && _responseBufferLength >= MODBUS_FRAME_SIZE) { 
        // set status as writing
        _isResponseBufferWriting = true;
    } 

    // check if we are not in writing or the address is broadcast
    if (!_isResponseBufferWriting || _responseBuffer[MODBUS_ADDRESS_INDEX] == MODBUS_BROADCAST_ADDRESS) { 
        // cleanup and ignore
        _isResponseBufferWriting = false;
        _responseBufferWriteIndex = 0;
        _responseBufferLength = 0;
        return 0;
    }

    /**
     * Preparing
     */

    // if this is supposed to be the first write
    if (_responseBufferWriteIndex == 0) {
        // if we still need to wait
        if ((micros() - _lastCommunicationTime) <= (_halfCharTimeInMicroSecond * MODBUS_HALF_SILENCE_MULTIPLIER))
        {
            // ignore
            return 0;
        }

        // calculate and fill crc
        uint16_t crc = Modbus::calculateCRC(_responseBuffer, _responseBufferLength - MODBUS_CRC_LENGTH);
        _responseBuffer[_responseBufferLength - MODBUS_CRC_LENGTH] = crc & 0xff;
        _responseBuffer[(_responseBufferLength - MODBUS_CRC_LENGTH) + 1] = crc >> 8;

        // enter transmission mode
        if (_transmissionControlPin > MODBUS_CONTROL_PIN_NONE) {
            digitalWrite(_transmissionControlPin, HIGH);
        }
    }

    /**
     * Transmit
     */

    // send buffer
    uint16_t length = 0;
    if (_serialTransmissionBufferLength > 0) {
        uint16_t length = min(
            _serialStream.availableForWrite(), 
            _responseBufferLength - _responseBufferWriteIndex
        );

        if (length > 0) {
            length = _serialStream.write(
                _responseBuffer + _responseBufferWriteIndex, 
                length
            );
            _responseBufferWriteIndex += length;
            _totalBytesSent += length;
        } 
        
        if (_serialStream.availableForWrite() < _serialTransmissionBufferLength)
        { 
            // still waiting for write to complete
            _lastCommunicationTime = micros();
            return length;
        }

        // if buffer reports as empty; make sure it really is 
        // (`Serial` removes bytes from buffer before sending them)
        _serialStream.flush();
    } else {
        // compatibility for badly written software serials; aka AltSoftSerial
        length = _responseBufferLength - _responseBufferWriteIndex;

        if (length > 0) {
            length = _serialStream.write(_responseBuffer, length);
            _serialStream.flush();
        }

        _responseBufferWriteIndex += length;
        _totalBytesSent += length;
    }

    if (_responseBufferWriteIndex >= _responseBufferLength &&
        (micros() - _lastCommunicationTime) > (_halfCharTimeInMicroSecond * MODBUS_HALF_SILENCE_MULTIPLIER)) {

        // end transmission
        if (_transmissionControlPin > MODBUS_CONTROL_PIN_NONE) {
            digitalWrite(_transmissionControlPin, LOW);
        }

        // cleanup
        _isResponseBufferWriting = false;
        _responseBufferWriteIndex = 0;
        _responseBufferLength = 0;
    }

    return length;
}

/**
 * Fills the output buffer with an exception in regard to the request already 
 * in the input buffer and writes the response. No need to do it later.
 *
 * @param exceptionCode the status code to report.
 * @return the number of bytes written
 */
uint16_t Modbus::reportException(uint8_t exceptionCode)
{
    // we don't respond to broadcast messages
    if (_requestBuffer[MODBUS_ADDRESS_INDEX] == MODBUS_BROADCAST_ADDRESS)
    {
        return 0;
    }
    _responseBufferLength = MODBUS_FRAME_SIZE + 1;
    _responseBuffer[MODBUS_FUNCTION_CODE_INDEX] |= 0x80;
    _responseBuffer[MODBUS_DATA_INDEX] = exceptionCode;

    return Modbus::writeResponse();
}

/**
 * Calculates the CRC of the passed byte array from zero up to the
 * passed length.
 *
 * @param buffer the byte array containing the data.
 * @param length the length of the byte array.
 *
 * @return the calculated CRC as an unsigned 16 bit integer.
 */
uint16_t Modbus::calculateCRC(uint8_t *buffer, int length)
{
    int i, j;
    uint16_t crc = 0xFFFF;
    uint16_t tmp;

    // calculate crc
    for (i = 0; i < length; i++)
    {
        crc = crc ^ buffer[i];

        for (j = 0; j < 8; j++)
        {
            tmp = crc & 0x0001;
            crc = crc >> 1;
            if (tmp)
            {
                crc = crc ^ 0xA001;
            }
        }
    }

    return crc;
}
