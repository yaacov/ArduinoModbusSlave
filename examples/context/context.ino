/*
    Modbus slave setCallbackContext() example.
    

	This slave is basically useless, as it has only 
	10 Holding Registers to read / write.
	
	Only purpose is to demonstrate how the Modbus Slave stuff may be
	encapsulated in some class.


    Created 07-02-2023
    By Werner Panocha

	

    https://github.com/yaacov/ArduinoModbusSlave
*/


// Dirty trick to use a C++ class w/o making a library ...
#include "SlaveClass.h"			// Header
#include "SlaveClass.cc.h"		// The .cc part also flagged as a header



#define SLAVE_ID 1

// RS485 Modbus Interface 
#define RS485_BAUDRATE 	9600 						// Baudrate for Modbus communication.
#define RS485_SERIAL 	Serial2   					// Serial port for Modbus communication.
#define RS485_CTRL_PIN	MODBUS_CONTROL_PIN_NONE 	// GPIO number for Control pin (optionally)

// The Modbus slave hided in a class
SlaveClass *slaveInstance;

// ---------------------------------------------------------------------
void setup()
// ---------------------------------------------------------------------
{
	// Use this for debug messages etc.
	Serial.begin(115200);
	Serial.print("\nModbus server\n");
	
	slaveInstance = new SlaveClass(&(RS485_SERIAL), RS485_BAUDRATE, RS485_CTRL_PIN, SLAVE_ID);
	
}

// ---------------------------------------------------------------------
void loop()
// ---------------------------------------------------------------------
{

	// Will trigger registered callback in slave 
	// if appropriate message was received.
	// Slave will do it's work ...
    slaveInstance->perform();
    
    // In our trivial example, it flags when a register was changed
    if(slaveInstance->updateAvailable())
		slaveInstance->showRegisters();
}

// eof
