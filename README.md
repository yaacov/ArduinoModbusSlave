# ModbusSlave

##### ModbusSlave library for Arduino

This modbus slave library uses callbacks to handle modbus requests for one or multiple slave ids.
Handler functions are called on modbus request, and the users can implement them in their sketch.

### ModbusSlave is fun and easy to use
Register a handler function:
```c
slave.cbVector[CB_READ_INPUT_REGISTERS] = ReadAnalogIn;
```
Implement it:
```c
void ReadAnalogIn(uint8_t fc, uint16_t address, uint16_t length) {
    for (int i = 0; i < length; i++)
        slave.writeRegisterToBuffer(i, analogRead(address + i));
}
```
And thats it, your sketch is modbus enabled. (see the full examples for more detail)

----

- [Install](#install)
- [Competabilty](#competabilty)
- [Callback vector](#callback-vector)
      - [Multiple Slaves](#multiple-slaves)
      - [Slots](#slots)
      - [Handler function](#handler-function)
      - [Function codes](#function-codes)
      - [Reading and writing to the request buffer](#reading-and-writing-to-the-request-buffer)
- [Examples](#examples)
      - [handle "Force Single Coil" as arduino digitalWrite](#handle-force-single-coil-as-arduino-digitalwrite)
      - [handle "Read Input Registers" as arduino analogRead](#handle-read-input-registers-as-arduino-analogread)

----

### Install

Download the zip package, and install it into your Arduino IDE. See the Arduino tutorial about installing 3rd party libraries: https://www.arduino.cc/en/Guide/Libraries#toc4

### Competabilty

###### This class implements:

* FC1 "Read Coil Status"
* FC2 "Read Input Status"
* FC3 "Read Holding Registers"
* FC4 "Read Input Registers"
* FC5 "Force Single Coil"
* FC6 "Preset Single Register"
* FC15 "Force Multiple Coils"
* FC16 "Preset Multiple Registers"

### Serial port

* The default serial port is Serial, but any class that inhirets from Stream can be used.
To set a different Serial class, explicitly set the Stream in the Modbus class constuctor.

### Callback vector

Users register handler functions into the callback vector of the slave.

###### Multiple Slaves

This can be done independently for one or multiple slaves with different IDs.

###### Slots

The callback vector has 4 slots for request handlers:

* slave.cbVector[CB_READ_COILS] - called on FC1
* slave.cbVector[CB_READ_DISCRETE_INPUTS] - called on FC2
* slave.cbVector[CB_READ_HOLDING_REGISTERS] - called on FC3
* slave.cbVector[CB_READ_INPUT_REGISTERS] - called on FC4
* slave.cbVector[CB_WRITE_COILS] - called on FC5 and FC15
* slave.cbVector[CB_WRITE_HOLDING_REGISTERS] - called on FC6 and FC16

###### Handler function

Handler functions must return unit8_t and take:
* uint8_t  fc - request function code
* uint16_t address - first register / first coil address
* uint16_t length - length of data

Return codes:

*  STATUS_OK = 0,
*  STATUS_ILLEGAL_FUNCTION,
*  STATUS_ILLEGAL_DATA_ADDRESS,
*  STATUS_ILLEGAL_DATA_VALUE,
*  STATUS_SLAVE_DEVICE_FAILURE,
*  STATUS_ACKNOWLEDGE,
*  STATUS_SLAVE_DEVICE_BUSY,
*  STATUS_NEGATIVE_ACKNOWLEDGE,
*  STATUS_MEMORY_PARITY_ERROR,
*  STATUS_GATEWAY_PATH_UNAVAILABLE,
*  STATUS_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND

###### Function codes

* FC_READ_COILS = 1
* FC_READ_DISCRETE_INPUT = 2
* FC_READ_REGISTERS = 3
* FC_READ_INPUT_REGISTERS = 4
* FC_WRITE_COIL = 5
* FC_WRITE_REGISTER = 6
* FC_WRITE_MULTIPLE_COILS = 15
* FC_WRITE_MULTIPLE_REGISTERS = 16

----

###### Reading and writing to the request / response buffer

* int readCoilFromBuffer(int offset) : read one coil value from the request buffer.
* uint16_t readRegisterFromBuffer(int offset) : read one register value from the request buffer.
* void writeCoilToBuffer(int offset, int state) : write one coil state into the answer buffer.
* void writeRegisterToBuffer(int offset, uint16_t value) : write one register value into the answer buffer.

----

### Examples

----
###### handle "Force Single Coil" as arduino digitalWrite
```c
#include <ModbusSlave.h>

// implicitly set stream to use the Serial serialport
Modbus slave(1, 8); // [stream = Serial,] slave id = 1, rs485 control-pin = 8

void setup() {
    // register one handler functions
    // if a callback handler is not assigned to a modbus command 
    // the default handler is called. 
    // default handlers return a valid but empty replay.
    slave.cbVector[CB_WRITE_COILS] = writeDigitlOut;
    
    // start slave at baud 9600 on Serial
    Serial.begin( 9600 ); // baud = 9600
    slave.begin( 9600 );
}

void loop() {
    // listen for modbus commands con serial port
    slave.poll();
}

// Handel Force Single Coil (FC=05)
uint8_t writeDigitlOut(uint8_t fc, uint16_t address, uint16_t length) {
    if (slave.readCoilFromBuffer(0) == HIGH) {
        digitalWrite(address, HIGH);
    } else {
        digitalWrite(address, LOW);
    }
    return STATUS_OK;
}

```

----
###### handle "Read Input Registers" as arduino analogRead
```c
#include <ModbusSlave.h>

// explicitly set stream to use the Serial serialport
Modbus slave(Serial, 1, 8); // stream = Serial, slave id = 1, rs485 control-pin = 8

void setup() {
    // register handler functions
    slave.cbVector[CB_READ_INPUT_REGISTERS] = ReadAnalogIn;
    
    // start slave at baud 9600 on Serial
    Serial.begin( 9600 ); // baud = 9600
    slave.begin( 9600 );
}

void loop() {
    // listen for modbus commands con serial port
    slave.poll();
}

// Handel Read Input Registers (FC=04)
uint8_t ReadAnalogIn(uint8_t fc, uint16_t address, uint16_t length) {
    // write registers into the answer buffer
    for (int i = 0; i < length; i++) {
      slave.writeRegisterToBuffer(i, analogRead(address + i));
    }
    return STATUS_OK;
}

```

