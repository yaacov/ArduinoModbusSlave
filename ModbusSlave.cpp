/**
 * Copyright (c) 2015, Yaacov Zamir <kobi.zamir@gmail.com>
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

#include <inttypes.h>
#include <string.h>
#include "Arduino.h"
#include "Print.h"
#include "ModbusSlave.h"

/**
 * Init the modbus object.
 *
 * @param unitID the modbus slave id.
 * @param ctrlPin the digital out pin for RS485 read/write control.
 */
Modbus::Modbus(uint8_t _unitID, int _ctrlPin) {
    // set modbus slave unit id
    unitID = _unitID;
    
    // set control pin for 485 write.
    ctrlPin = _ctrlPin;
}

/**
 * Begin serial port and set timeout.
 *
 * @param boud the serial port boud rate.
 */
void Modbus::begin(int boud) {
    // set control pin
    if (ctrlPin) {
        pinMode(ctrlPin, OUTPUT);
    }
    
    // open port and set the timeout for 3.5 chars.
    Serial.begin(boud);
    setT35(boud);
}


/**
 * Calculate end of buffer time and set port timeout.
 *
 * Modbus states that a baud rate higher than 19200 must use a fixed 750 us 
 * for inter character time out and 1.75 ms for a frame delay.
 * For baud rates below 19200 the timeing is more critical and has to be calculated.
 * E.g. 9600 baud in a 10 bit packet is 960 characters per second
 * In milliseconds this will be 960 characters per 1000ms. So for 1 character
 * 1000ms/960characters is 1.04167ms per character and finally modbus states an
 * intercharacter must be 1.5T or 1.5 times longer than a normal character and thus
 * 1.5T = 1.04167ms * 1.5 = 1.5625ms. A frame delay is 3.5T.
 *
 * @param boud the serial line boud rate.
 */
void Modbus::setT35(int boud) {
    int T35;
    
    if (boud > 19200) {
        T35 = 2; // 1.750 ~ 2 ms
    } else {
        T35 = 35000/boud; // 35000/9600 ~ 4 ms
    }
    
    // default port timeout is 1000ms, T35 is ~4 ms
    Serial.setTimeout(T35);
}

/** 
 * Calculate buffer CRC16 
 *
 * @param buf the data buffer.
 * @param length the length of the buffer without CRC.
 *
 * @return the calculated CRC16.
 */
uint16_t Modbus::calcCRC(uint8_t *buf, int length) {
    int i, j;
    uint16_t crc = 0xFFFF;
    uint16_t tmp;
    
    // calculate crc16
    for (i = 0; i < length; i++) {
        crc = crc ^ buf[i];
        
        for (j = 0; j < 8; j++) {
            tmp = crc & 0x0001;
            crc = crc >> 1;
            if (tmp) {
              crc = crc ^ 0xA001;
            }
        }
    }
    
    return crc;
}

/**
 * wait for end of frame, parse request and answer it.
 */
int Modbus::poll() {
    int lengthIn;
    int lengthOut;
    uint16_t crc;
    uint16_t address;
    uint16_t length;
    uint16_t status;
    uint8_t fc;
    
    /**
     * Read one data frame:
     */
     
    // if we have data in buffer
    // read until end of buffer timeout.
    if (Serial.available()) {
        lengthIn = Serial.readBytes(bufIn, MAX_BUFFER);
    } else {
        return 0;
    }
    
    /**
     * Validate buffer.
     */
    // check minimum length.
    if (lengthIn < 8) return 0;
    
    // check unit-id
    if (bufIn[0] != unitID) return 0;
    
    // check crc.
    crc = word(bufIn[lengthIn - 1], bufIn[lengthIn - 2]);
    if (calcCRC(bufIn, lengthIn - 2) != crc) return 0;
    
    /**
     * Parse command
     */
    fc = bufIn[1];
    switch (fc) {
        case FC_READ_COILS: // read coils (digital out state)
        case FC_READ_DISCRETE_INPUT: // read input state (digital in)
            address = word(bufIn[2], bufIn[3]); // coil to set.
            length = word(bufIn[4], bufIn[5]);
            
            // sanity check.
            if (length > MAX_BUFFER) return 0;
            
            // check command length.
            if (lengthIn != 8) return 0;
            
            // build valid empty answer.
            lengthOut = 3 + (length - 1) / 8 + 1 + 2;
            bufOut[2] = (length - 1) / 8 + 1;
            
            // clear data out.
            memset(bufOut + 3, 0, bufOut[2]);
            
            // if we have uset callback.
            if (cbVector[CB_READ_COILS]) {
                cbVector[CB_READ_COILS](fc, address, length);
            }
            break;
        case FC_READ_HOLDING_REGISTERS: // read holding registers (analog out state)
        case FC_READ_INPUT_REGISTERS: // read input registers (analog in)
            address = word(bufIn[2], bufIn[3]); // first register.
            length = word(bufIn[4], bufIn[5]); // number of registers to read.
            
            // sanity check.
            if (length > MAX_BUFFER) return 0;
            
            // check command length.
            if (lengthIn != 8) return 0;
            
            // build valid empty answer.
            lengthOut = 3 + 2 * length + 2;
            bufOut[2] = 2 * length;
            
            // clear data out.
            memset(bufOut + 3, 0, bufOut[2]);
            
            // if we have uset callback.
            if (cbVector[CB_READ_REGISTERS]) {
                cbVector[CB_READ_REGISTERS](fc, address, length);
            }
            break;
        case FC_WRITE_COIL: // write one coil (digital out)
            address = word(bufIn[2], bufIn[3]); // coil to set
            status = word(bufIn[4], bufIn[5]); // 0xff00 - on, 0x0000 - off
            
            // check command length.
            if (lengthIn != 8) return 0;
            
            // build valid empty answer.
            lengthOut = 8;
            memcpy(bufOut + 2, bufIn + 2, 4);
            
            // if we have uset callback.
            if (cbVector[CB_WRITE_COIL]) {
                cbVector[CB_WRITE_COIL](fc, address, status == COIL_ON);
            }
            break;
        case FC_WRITE_MULTIPLE_REGISTERS: // write holding registers (analog out)
            address = word(bufIn[2], bufIn[3]); // first register
            length = word(bufIn[4], bufIn[5]); // number of registers to set
            
            // sanity check.
            if (length > MAX_BUFFER) return 0;
            
            // check command length
            if (lengthIn != (7 + length * 2 + 2)) return 0;
            
            // build valid empty answer.
            lengthOut = 8;
            memcpy(bufOut + 2, bufIn + 2, 4);
            
            // if we have uset callback
            if (cbVector[CB_WRITE_MULTIPLE_REGISTERS]) {
                cbVector[CB_WRITE_MULTIPLE_REGISTERS](fc, address, length);
            }
            break;
        default:
            // unknown command
            return 0;
            break;
    }
    
    /**
     * Build answer
     */
    bufOut[0] = unitID;
    bufOut[1] = fc;
    
    // add crc
    crc = calcCRC(bufOut, lengthOut - 2);
    bufOut[lengthOut - 2] = crc & 0xff;
    bufOut[lengthOut - 1] = crc >> 8;
    
    /**
     * Transmit
     */
    if (ctrlPin) {
        // set rs485 control pin to write
        digitalWrite(ctrlPin, HIGH);
        
        // send buffer
        Serial.write(bufOut, lengthOut);
        
        // wait until serial port register (UCSRnA) 
        // transfer complete bit (TXCn) is on
        // and then set rs485 control pin to read
        // [ on SoftwareSerial use delay ? ]
        while (!(UCSR0A & (1 << TXC0)));
        digitalWrite(ctrlPin, LOW);
    } else {
        // just send the buffer.
        Serial.write(bufOut, lengthOut);
    }
    
    return lengthOut;
}

/**
 * Read register value from input buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @return the reguster value from buffer.
 */
uint16_t Modbus::readRegisterFromBuffer(int offset) {
    int address = 7 + offset * 2;
    
    return word(bufIn[address], bufIn[address + 1]);
}

/**
 * Write coil state to output buffer.
 *
 * @param offset the coil offset from first coil in this buffer.
 * @param state the coil state to write into buffer (true / false)
 */
void Modbus::writeCoilToBuffer(int offset, uint16_t state) {
    int address = 3 + offset / 8;
    int bit = offset % 8;
    
    if (state == HIGH) {
        bitSet(bufOut[address], bit);
    } else if (state) {
        bitClear(bufOut[address], bit);
    }
}

/**
 * Write register value to output buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @param value the register value to write into buffer.
 */
void Modbus::writeRegisterToBuffer(int offset, uint16_t value) {
    int address = 3 + offset * 2;
    
    bufOut[address] = value >> 8;
    bufOut[address + 1] = value & 0xff;
}

/**
 * Write arbitrary string of uint8_t to output buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @param str the string to write into buffer.
 * @param length the string length.
 */
void Modbus::writeStringToBuffer(int offset, uint8_t *str, uint8_t length) {
    int address = 3 + offset * 2;
    
    // check string length.
    if ((address + length) >= MAX_BUFFER) return;
    
    memcpy(bufOut + address, str, length);
}

