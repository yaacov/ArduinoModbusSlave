/**
 *  Modbus slave example
 */

#include <ModbusSlave.h>

/**
 * Simulate the Holding Registers
 */
#define MAX_PROG_REG 10
uint16_t programMem[MAX_PROG_REG];

/**
 *  Modbus object declaration
 */
Modbus slave(1, 8); // slave id = 1, rs485 control-pin = 8

void setup() {
    /* set some pins for output
     */
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    
    /* register handler functions
     * into the modbus slave callback vector.
     */
    slave.cbVector[CB_READ_COILS] = readDigitalIn;
    slave.cbVector[CB_WRITE_COIL] = writeDigitlOut;
    slave.cbVector[CB_READ_REGISTERS] = readAnalogIn;
    slave.cbVector[CB_WRITE_MULTIPLE_REGISTERS] = writeProgram;
    
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
 * Handel Read Input Status (FC=02/01)
 * write back the values from digital in pins (input status).
 *
 * handler functions must return void and take:
 *      uint8_t  fc - function code
 *      uint16_t address - first register/coil address
 *      uint16_t length/status - length of data / coil status
 */
void readDigitalIn(uint8_t fc, uint16_t address, uint16_t length) {
    if (fc == FC_READ_COILS) {
        // read digital input
        for (int i = 0; i < length; i++) {
            slave.writeCoilToBuffer(i, digitalRead(address + i) ? COIL_ON : COIL_OFF);
        }
    }
    
    if (fc == FC_READ_DISCRETE_INPUT) {
        // read digital input
        for (int i = 0; i < length; i++) {
            slave.writeCoilToBuffer(i, digitalRead(address + i) ? COIL_ON : COIL_OFF);
        }
    }
}

/**
 * Handel Read Input Registers (FC=04/03)
 * write back the values from analog in pins (input registers).
 *
 * handler functions must return void and take:
 *      uint8_t  fc - function code
 *      uint16_t address - first register/coil address
 *      uint16_t length/status - length of data / coil status
 */
void readAnalogIn(uint8_t fc, uint16_t address, uint16_t length) {
    if (fc == FC_READ_HOLDING_REGISTERS) {
        // read program memory
        for (int i = 0; i < length, (address + i) < MAX_PROG_REG; i++) {
            slave.writeRegisterToBuffer(i, programMem[address + i]);
        }
    }
    
    if (fc == FC_READ_INPUT_REGISTERS) {
        // read analog input
        for (int i = 0; i < length; i++) {
            slave.writeRegisterToBuffer(i, analogRead(address + i));
        }
    }
}

/**
 * Handel Force Single Coil (FC=05)
 * set digital output pins (coils) on and off
 */
void writeDigitlOut(uint8_t fc, uint16_t address, uint16_t status) {
    if (status == COIL_ON) {
        digitalWrite(address, HIGH);
    } else if (status == COIL_OFF) {
        digitalWrite(address, LOW);
    }
}

/**
 * Handel Write Registers (FC=16)
 * write data into program.
 *
 * handler functions must return void and take:
 *      uint8_t  fc - function code
 *      uint16_t address - first register/coil address
 *      uint16_t length/status - length of data / coil status
 */
void writeProgram(uint8_t fc, uint16_t address, uint16_t length) {
    // write to program memory
    // read program memory
    for (int i = 0; i < length, (address + i) < MAX_PROG_REG; i++) {
        programMem[address + i] = slave.readRegisterFromBuffer(i);
    }
}

