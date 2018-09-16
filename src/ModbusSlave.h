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

#define MAX_BUFFER 64

/**
 * Modbus function codes
 */
enum {
  FC_READ_COILS = 1,
  FC_READ_DISCRETE_INPUT = 2,
  FC_READ_HOLDING_REGISTERS = 3,
  FC_READ_INPUT_REGISTERS = 4,
  FC_WRITE_COIL = 5,
  FC_WRITE_REGISTER = 6,
  FC_WRITE_MULTIPLE_COILS = 15,
  FC_WRITE_MULTIPLE_REGISTERS = 16
};

enum {
  CB_MIN = 0,
  CB_READ_COILS = CB_MIN,
  CB_READ_REGISTERS,
  CB_WRITE_COILS,
  CB_WRITE_REGISTERS,
  CB_MAX
};

enum {
  COIL_OFF = 0x0000,
  COIL_ON = 0xff00
};

enum {
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
  STATUS_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND
};

typedef uint8_t (*CallBackFunc)(uint8_t, uint16_t, uint16_t);

/**
 * @class Modbus
 */
class Modbus {
public:
    Modbus(uint8_t unitID, int ctrlPin);
    Modbus(Stream &serial, uint8_t unitID, int ctrlPin);
    void begin(unsigned long boud);
    int poll();
    int readCoilFromBuffer(int offset);
    uint16_t readRegisterFromBuffer(int offset);
    void writeCoilToBuffer(int offset, int state);
    void writeRegisterToBuffer(int offset, uint16_t value);
    uint8_t writeStringToBuffer(int offset, uint8_t *str, uint8_t length);

    CallBackFunc cbVector[CB_MAX];
private:
    Stream &serial;
    uint32_t timeout;
    uint32_t last_receive_time;
    uint16_t calcCRC(uint8_t *buf, int length);

    int ctrlPin = -1;
    uint8_t unitID;
    uint8_t lengthIn;
    uint8_t bufIn[MAX_BUFFER];
    uint8_t bufOut[MAX_BUFFER];
};
#endif
