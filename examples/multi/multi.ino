/*
    Modbus multiple slave example.

    Read and write to two different slaves using Modbus serial connection.
    
    This sketch shows you how to use the callback vector for reading and
    writing to multiple slaves.
    
    * Communicate with two slaves with the addresses 1 and 3.
	* Set the callback functions of the slaves independently.

    Created 19-08-2019
    By Tobias Schaffner

    Updated 31-03-2020
    By Yorick Smilda

    https://github.com/yaacov/ArduinoModbusSlave
*/

#include <ModbusSlave.h>

#define NUMBER_OF_SLAVES 2
#define ID_SLAVE_1 1
#define ID_SLAVE_2 3

#define SERIAL_BAUDRATE 9600 // Change to the baudrate you want to use for Modbus communication.
#define SERIAL_PORT Serial   // Serial port to use for RS485 communication, change to the port you're using.

uint16_t memory_slave1[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
uint16_t memory_slave2[] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

// You shouldn't have to change anything below this to get this example to work

uint8_t memory_slave1_size = sizeof(memory_slave1) / sizeof(memory_slave1[0]); // Get the size of the input_pins array
uint8_t memory_slave2_size = sizeof(memory_slave2) / sizeof(memory_slave2[0]); // Get the size of the output_pins array

// Initialize an array of slaves.
ModbusSlave slaves[NUMBER_OF_SLAVES] = {ModbusSlave(ID_SLAVE_1), ModbusSlave(ID_SLAVE_2)};

// Create an Modbus Object and pass the array of slaves.
Modbus modbus(SERIAL_PORT, slaves, NUMBER_OF_SLAVES);

void setup()
{
    // Register functions to call when a certain function code is received.
    slaves[0].cbVector[CB_READ_HOLDING_REGISTERS] = readMemorySlave1;
    slaves[0].cbVector[CB_WRITE_HOLDING_REGISTERS] = writeMemorySlave1;
    slaves[1].cbVector[CB_READ_HOLDING_REGISTERS] = readMemorySlave2;
    slaves[1].cbVector[CB_WRITE_HOLDING_REGISTERS] = writeMemorySlave2;

    // Set the serial port and slave to the given baudrate.
    SERIAL_PORT.begin(SERIAL_BAUDRATE);
    modbus.begin(SERIAL_BAUDRATE);
}

void loop()
{
    // Listen for modbus requests on the serial port.
    // When a request is received it's going to get validated.
    // And if there is a function registered to the received function code, this function will be executed.
    modbus.poll();
}

// Modbus handler functions
// The handler functions must return an uint8_t and take the following parameters:
//     uint8_t  fc - function code
//     uint16_t address - first register/coil address
//     uint16_t length/status - length of data / coil status

// Handle the function code Read Holding Registers (FC=03).
uint8_t readMemorySlave1(uint8_t fc, uint16_t address, uint16_t length)
{
    // Check if the requested addresses exist in the array.
    if (address > memory_slave1_size || (address + length) > memory_slave1_size)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    Serial.println(F("Reading memory from Slave 1"));

    // Write the memory array into the send buffer.
    for (uint8_t i = 0; i < length; ++i)
    {
        modbus.writeRegisterToBuffer(i, memory_slave1[address + i]);
    }
    return STATUS_OK;
}

// Handle the function codes Write Holding Register(s) (FC=06, FC=16)
uint8_t writeMemorySlave1(uint8_t fc, uint16_t address, uint16_t length)
{
    // Check if the requested addresses exist in the array
    if (address > memory_slave1_size || (address + length) > memory_slave1_size)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    Serial.println(F("Writing memory on Slave 1"));

    // Write the received data into the memory array.
    for (uint8_t i = 0; i < length; ++i)
    {
        memory_slave1[address + i] = modbus.readRegisterFromBuffer(i);
    }
    return STATUS_OK;
}

// Handle the function code Read Holding Registers (FC=03).
uint8_t readMemorySlave2(uint8_t fc, uint16_t address, uint16_t length)
{
    // Check if the requested addresses exist in the array.
    if (address > memory_slave2_size || (address + length) > memory_slave2_size)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    Serial.println(F("Reading memory from Slave 2"));

    // Write the memory array into the send buffer.
    for (uint8_t i = 0; i < length; ++i)
    {
        modbus.writeRegisterToBuffer(i, memory_slave2[address + i]);
    }
    return STATUS_OK;
}

// Handle the function codes Write Holding Register(s) (FC=06, FC=16)
uint8_t writeMemorySlave2(uint8_t fc, uint16_t address, uint16_t length)
{
    // Check if the requested addresses exist in the array
    if (address > memory_slave2_size || (address + length) > memory_slave2_size)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    Serial.println(F("Writing memory on Slave 2"));

    // Write the received data into the memory array.
    for (uint8_t i = 0; i < length; ++i)
    {
        memory_slave2[address + i] = modbus.readRegisterFromBuffer(i);
    }
    return STATUS_OK;
}