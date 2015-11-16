/**
 *  Modbus slave example
 */

#include <ModbusSlave.h>

/**
 *  Modbus object declaration
 */
Modbus slave(1, 8); // slave id = 0, rs485 control-pin = 8

void setup() {
    pinMode(13, OUTPUT);
    
    /* register handler functions
     * into the modbus slave callback vector.
     */
    slave.cbVector[CB_READ_COILS] = NULL;
    slave.cbVector[CB_WRITE_COIL] = writeDigitlOut;
    slave.cbVector[CB_READ_REGISTERS] = ReadAnalogIn;
    slave.cbVector[CB_WRITE_MULTIPLE_REGISTERS] = NULL;
    
    // start slave at baud 9600 on Serial
    slave.begin( 9600 ); // baud = 9600
}

void loop() {
    /* listen for modbus commands con serial port
     *
     * on a request, handle the request.
     * if the request has a user handler function registered in cbVector
     * call the user handler function.
     */ 
    slave.poll();
}

/**
 * Handel Read Input Registers (FC=04)
 *
 * write back the values from analog in pins (input registers).
 *
 * handler functions must return void and take:
 *      uint8_t  fc - function code
 *      uint16_t address - first register/coil address
 *      uint16_t length/status - length of data / coil status
 */
void ReadAnalogIn(uint8_t fc, uint16_t address, uint16_t length) {
    for (int i = 0; i < length; i++) {
      slave.writeRegisterToBuffer(i, analogRead(address + i));
    }
}

/**
 * Handel Force Single Coil (FC=05)
 *
 * set digital output pins (coils) on and off
 */
void writeDigitlOut(uint8_t fc, uint16_t address, uint16_t status) {
    if (status == COIL_ON) {
        digitalWrite(address, HIGH);
    } else if (status == COIL_OFF) {
        digitalWrite(address, LOW);
    }
}

