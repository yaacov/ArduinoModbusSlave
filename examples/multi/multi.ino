/*
    Modbus multiple slave example.

    Read and write to two different slaves using Modbus serial connection.
    
    This sketch show how to use the callback vector for reading and
    writing to multiple slaves.
    
    * Communicate with two slaves holding the addresses 1 and 3.
	* Set the callback functions of the slaves indipendently.

    Created 19 08 2019
    By Tobias Schaffner

    https://github.com/yaacov/ArduinoModbusSlave
*/

#include <ModbusSlave.h>

#define NUMBER_OF_SLAVES  2
#define ID_SLAVE_1        1
#define ID_SLAVE_2        3

uint16_t memorySlave1[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
uint16_t memorySlave2[] = { 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 };

// Initialize the array of slaves
ModbusSlave slaves[NUMBER_OF_SLAVES] = { ModbusSlave(ID_SLAVE_1), ModbusSlave(ID_SLAVE_2) };

// Create the Modbus Object with the slave array 
Modbus modbus(slaves, NUMBER_OF_SLAVES);

void setup() {
    // register handler functions for the slaves
    slaves[0].cbVector[CB_READ_HOLDING_REGISTERS] = readMemorySlave1;
    slaves[0].cbVector[CB_WRITE_HOLDING_REGISTERS] = writeMemorySlave1;
    slaves[1].cbVector[CB_READ_HOLDING_REGISTERS] = readMemorySlave2;
    slaves[1].cbVector[CB_WRITE_HOLDING_REGISTERS] = writeMemorySlave2;
	
    // start slave at baud 9600 on Serial
    Serial.begin( 9600 );
    modbus.begin( 9600 );
}

void loop() {
    // listen for modbus commands con serial port
    modbus.poll();
}

/**
 * Handel Read Holding Registers (FC=03)
 */
uint8_t readMemorySlave1(uint8_t fc, uint16_t address, uint16_t length) {
    Serial.println("In read memory Slave 1");
    // write registers into the answer buffer
    for (uint8_t i = 0; i < length; ++i) {
        modbus.writeRegisterToBuffer(i, memorySlave1[address + i]);
    }
    return STATUS_OK;
}

/**
 * Handle Write Holding Register(s) (FC=06, FC=16)
 */
uint8_t writeMemorySlave1(uint8_t fc, uint16_t address, uint16_t length) {
    Serial.println("In write memory Slave 1");
    // set digital pin state(s).
    for (uint8_t i = 0; i < length; ++i) {
        memorySlave1[address + i] = modbus.readRegisterFromBuffer(i);
    }
    return STATUS_OK;
}

/**
 * Handel Read Holding Registers (FC=03)
 */
uint8_t readMemorySlave2(uint8_t fc, uint16_t address, uint16_t length) {
    Serial.println("In read memory Slave 2");
    // write registers into the answer buffer
    for (uint8_t i = 0; i < length; ++i) {
        modbus.writeRegisterToBuffer(i, memorySlave2[address + i]);
    }
    return STATUS_OK;
}

/**
 * Handle Write Holding Register(s) (FC=06, FC=16)
 */
uint8_t writeMemorySlave2(uint8_t fc, uint16_t address, uint16_t length) {
    Serial.println("In write memory Slave 2");
    // set digital pin state(s).
    for (uint8_t i = 0; i < length; ++i) {
        memorySlave2[address + i] = modbus.readRegisterFromBuffer(i);
    }
    return STATUS_OK;
}