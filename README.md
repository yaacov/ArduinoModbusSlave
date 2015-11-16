# arduino-modbus-slave
Modbus-Slave library for Arduino

This modbus slave library uses callbacks to handle modbus requests.
Handler functions are called on modbus request, and the users can handle the
requests.
 
### Competabilty

###### This class implements:

* FC1 "Read Coil Status"
* FC3 "Read Holding Registers"
* FC4 "Read Input Registers"
* FC5 "Force Single Coil"
* FC16 "Preset Multiple Registers"

### Examples

###### handle "Force Single Coil" as arduino digitalWrite
```c
#include <ModbusSlave.h>

Modbus slave(1, 8); // slave id = 0, rs485 control-pin = 8

void setup() {
    // register handler functions
    slave.cbVector[CB_WRITE_COIL] = writeDigitlOut;
    
    // start slave at baud 9600 on Serial
    slave.begin( 9600 ); // baud = 9600
}

void loop() {
    // listen for modbus commands con serial port
    slave.poll();
}

// Handel Force Single Coil (FC=05)
void writeDigitlOut(uint8_t fc, uint16_t address, uint16_t status) {
    if (status == COIL_ON) {
        digitalWrite(address, HIGH);
    } else if (status == COIL_OFF) {
        digitalWrite(address, LOW);
    }
}

```
