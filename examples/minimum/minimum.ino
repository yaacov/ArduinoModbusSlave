/*
    Modbus slave minimalistic example.

	This slave is basically useless, as it has only 
	10 Holding Registers to read / write.
	
	Just as a template or starting point for basic tests
	to get familiar with this library w/o any external hardware.
	
 
    Werner Panocha, February 2023

 

    https://github.com/yaacov/ArduinoModbusSlave
*/

#include <ModbusSlave.h>


#define SLAVE_ID 1

// RS485 Modbus Interface 
#define RS485_BAUDRATE 	9600 						// Baudrate for Modbus communication.
#define RS485_SERIAL 	Serial2   					// Serial port for Modbus communication.
#define RS485_CTRL_PIN	MODBUS_CONTROL_PIN_NONE 	// GPIO number for Control pin (optionally)

// Pointer to Modbus slave object
Modbus *slave;

// A set of holding registers with predefined values
uint16_t holding_regs[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

// Number of available registers
const uint8_t num_holding_regs = sizeof(holding_regs) / sizeof(holding_regs[0]);

// ---------------------------------------------------------------------
// Handle the function code Read Holding Registers (FC=03) 
// ---------------------------------------------------------------------
uint8_t readHoldingRegs(uint8_t fc, uint16_t address, uint16_t length, void *context)
{
	Serial.print("Read Register(s)\n");
	if ((address + length) > num_holding_regs)
		return STATUS_ILLEGAL_DATA_ADDRESS;
    
	for (int i = 0; i < length; i++){
		slave->writeRegisterToBuffer(i, holding_regs[address + i]);
	}
	return STATUS_OK;
}

// ---------------------------------------------------------------------
// Handle the function codes Write Holding Register(s) (FC=06, FC=16) 
// ---------------------------------------------------------------------
uint8_t writeHoldingRegs(uint8_t fc, uint16_t address, uint16_t length, void *context)
{
	Serial.print("Write Register(s)\n");
	if ((address + length) > num_holding_regs)
		return STATUS_ILLEGAL_DATA_ADDRESS;
    
	for (int i = 0; i < length; i++){
		holding_regs[address + i] = slave->readRegisterFromBuffer(i);
	}
	return STATUS_OK;
}


// ---------------------------------------------------------------------
void setup()
// ---------------------------------------------------------------------
{
	// Use this for debug messages etc.
	Serial.begin(115200);
	Serial.print("\nModbus server\n");
	
	// Initialize slave instance
	slave = new Modbus(RS485_SERIAL, SLAVE_ID, RS485_CTRL_PIN);
	
	// Minimalistic slave functionality
	slave->cbVector[CB_READ_HOLDING_REGISTERS] 	= readHoldingRegs;
	slave->cbVector[CB_WRITE_HOLDING_REGISTERS] = writeHoldingRegs;
	
	// Set the serial port and slave to the given baudrate.
	RS485_SERIAL.begin(RS485_BAUDRATE);
	slave->begin(RS485_BAUDRATE);
}

// ---------------------------------------------------------------------
void loop()
// ---------------------------------------------------------------------
{
	// Listen for modbus requests on the serial port.
	// Will trigger registered callback if appropriate message was received
    slave->poll();
}

// eof
