/*
    Modbus slave example.

    Control and Read Arduino I/Os using Modbus serial connection.
    
    This sketch show how to use the callback vector for reading and
    controlling Arduino I/Os.
    
    * Control digital pins mode using holding registers 0 .. 50.
    * Control digital output pins as modbus coils.
    * Read digital inputs as discreet inputs.
    * Read analog inputs as input registers.
    * Write and Read EEPROM as holding registers.

    Created 08-12-2015
    By Yaacov Zamir

    Updated 31-03-2020
    By Yorick Smilda

    https://github.com/yaacov/ArduinoModbusSlave

*/

#include <EEPROM.h>
#include <ModbusSlave.h>

#define SLAVE_ID 1           // The Modbus slave ID, change to the ID you want to use.
#define RS485_CTRL_PIN 8     // Change to the pin the RE/DE pin of the RS485 controller is connected to.
#define SERIAL_BAUDRATE 9600 // Change to the baudrate you want to use for Modbus communication.
#define SERIAL_PORT Serial   // Serial port to use for RS485 communication, change to the port you're using.

// The position in the array determines the address. Position 0 will correspond to Coil, Discrete input or Input register 0.
uint8_t digital_pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}; // Add the pins you want to read as a Discrete input.
uint8_t analog_pins[] = {A0, A1, A2, A3, A4, A5};                  // Add the pins you want to read as a Input register.

// The EEPROM layout is as follows
// The first 50 bytes are reserved for storing digital pin pinMode_setting
// Byte 51 and up are free to write any uint16_t to.

// You shouldn't have to change anything below this to get this example to work

uint8_t digital_pins_size = sizeof(digital_pins) / sizeof(digital_pins[0]); // Get the size of the digital_pins array
uint8_t analog_pins_size = sizeof(analog_pins) / sizeof(analog_pins[0]);    // Get the size of the analog_pins array

// Modbus object declaration
Modbus slave(SERIAL_PORT, SLAVE_ID, RS485_CTRL_PIN);

void setup()
{
    // Set the defined digital pins to the value stored in EEPROM.
    for (uint16_t i = 0; i < digital_pins_size; i++)
    {
        uint8_t pinMode_setting;
        // Get the pinMode_setting of this digital pin from the EEPROM.
        EEPROM.get(i, pinMode_setting);

        pinMode(digital_pins[i], pinMode_setting);
    }

    // Set the defined analog pins to input mode.
    for (int i = 0; i < analog_pins_size; i++)
    {
        pinMode(analog_pins[i], INPUT);
    }

    // Register functions to call when a certain function code is received.
    slave.cbVector[CB_READ_COILS] = readDigital;
    slave.cbVector[CB_READ_DISCRETE_INPUTS] = readDigital;
    slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
    slave.cbVector[CB_READ_INPUT_REGISTERS] = readAnalogIn;
    slave.cbVector[CB_READ_HOLDING_REGISTERS] = readMemory;
    slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeMemory;

    // Set the serial port and slave to the given baudrate.
    SERIAL_PORT.begin(SERIAL_BAUDRATE);
    slave.begin(SERIAL_BAUDRATE);
}

void loop()
{
    // Listen for modbus requests on the serial port.
    // When a request is received it's going to get validated.
    // And if there is a function registered to the received function code, this function will be executed.
    slave.poll();
}

// Modbus handler functions
// The handler functions must return an uint8_t and take the following parameters:
//     uint8_t  fc - function code
//     uint16_t address - first register/coil address
//     uint16_t length/status - length of data / coil status

// Handle the function codes Read Input Status (FC=01/02) and write back the values from the digital pins (input status).
uint8_t readDigital(uint8_t fc, uint16_t address, uint16_t length)
{
    // Check if the requested addresses exist in the array
    if (address > digital_pins_size || (address + length) > digital_pins_size)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    // Read the digital inputs.
    for (int i = 0; i < length; i++)
    {
        // Write the state of the digital pin to the response buffer.
        slave.writeCoilToBuffer(i, digitalRead(digital_pins[address + i]));
    }

    return STATUS_OK;
}

// Handle the function code Read Holding Registers (FC=03) and write back the values from the EEPROM (holding registers).
uint8_t readMemory(uint8_t fc, uint16_t address, uint16_t length)
{
    // Read the requested EEPROM registers.
    for (int i = 0; i < length; i++)
    {
        // Below 50 is reserved for pinModes, above 50 is free to use.
        if (address + i <= 50)
        {
            uint8_t value;

            // Read a value from the EEPROM.
            EEPROM.get((address + i), value);

            // Write the pinMode from EEPROM to the response buffer.
            slave.writeRegisterToBuffer(i, value);
        }
        else
        {
            uint16_t value;

            // Read a value from the EEPROM.
            EEPROM.get(address + (i * 2), value);

            // Write the value from EEPROM to the response buffer.
            slave.writeRegisterToBuffer(i, value);
        }
    }

    return STATUS_OK;
}

// Handle the function code Read Input Registers (FC=04) and write back the values from the analog input pins (input registers).
uint8_t readAnalogIn(uint8_t fc, uint16_t address, uint16_t length)
{
    // Check if the requested addresses exist in the array
    if (address > analog_pins_size || (address + length) > analog_pins_size)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    // Read the analog inputs
    for (int i = 0; i < length; i++)
    {
        // Write the state of the analog pin to the response buffer.
        slave.writeRegisterToBuffer(i, analogRead(analog_pins[address + i]));
    }

    return STATUS_OK;
}

// Handle the function codes Force Single Coil (FC=05) and Force Multiple Coils (FC=15) and set the digital output pins (coils).
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length)
{
    // Check if the requested addresses exist in the array
    if (address > digital_pins_size || (address + length) > digital_pins_size)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    // Set the output pins to the given state.
    for (int i = 0; i < length; i++)
    {
        // Write the value in the input buffer to the digital pin.
        digitalWrite(digital_pins[address + i], slave.readCoilFromBuffer(i));
    }

    return STATUS_OK;
}

// Handle the function codes Write Holding Register(s) (FC=06, FC=16) and write data to the eeprom.
uint8_t writeMemory(uint8_t fc, uint16_t address, uint16_t length)
{
    // Write the received data to EEPROM.
    for (int i = 0; i < length; i++)
    {
        if (address + i <= 50)
        {
            // Check if the requested addresses exist in the array.
            if (address + i > digital_pins_size)
            {
                return STATUS_ILLEGAL_DATA_ADDRESS;
            }

            // Read the value from the input buffer.
            uint8_t value = slave.readRegisterFromBuffer(i);

            // Check if the value is 0 (INPUT) or 1 (OUTPUT).
            if (value != INPUT && value != OUTPUT)
            {
                return STATUS_ILLEGAL_DATA_VALUE;
            }

            // Store the received value in the EEPROM.
            EEPROM.put(address + i, value);

            // Set the pinmode to the received value.
            pinMode(digital_pins[address + i], value);
        }
        else
        {
            // Read the value from the input buffer.
            uint16_t value = slave.readRegisterFromBuffer(i);

            // Store the received value in the EEPROM.
            EEPROM.put(address + (i * 2), value);
        }
    }

    return STATUS_OK;
}
