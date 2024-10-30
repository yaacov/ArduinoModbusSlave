// SlaveClass.cc
// Encapsulate a Modbus Slave in a class, just for the fun of doing it.
//
// This is where setCallbackContext() comes into place, to allow
// for callbacks into such a class.
//
//	As I'm not an C++ expert, chances are that this is not
//	the smartest approach ...
//
// Werner Panocha, February 2023

#include "SlaveClass.h"

// Constructor
SlaveClass::SlaveClass(Stream *serialStream, unsigned int baud, int transmissionControlPin, uint8_t slaveId) {

	// Initialize RTU slave kernel instance 
	_slave = new Modbus(*serialStream, slaveId, transmissionControlPin);
	
	// Set callback context as a reference to this actual instance of SlaveClass
	_slave->setCallbackContext((void *) this);	// <<===
	
	// Set the callbacks to static helper methods of the SlaveClass
	// This may look not elegant, but not all C-Compilers / platforms  have the capability
	// for linking class member functions as callback vectors.
	// And to be honest, I always need Google to find the right syntax for this (if any)
	_slave->cbVector[CB_READ_HOLDING_REGISTERS] = SlaveClass::_readHoldingRegs;
	_slave->cbVector[CB_WRITE_HOLDING_REGISTERS] = SlaveClass::_writeHoldingRegs;
	
	// Start serial port and RTU kernel
	static_cast<HardwareSerial *>(serialStream)->begin(baud);
	_slave->begin(baud);
	
	Serial.print("Slave class initialized\n");
	Serial.print("Use Modbus FC's 03, 06, 16 to play with the Holding Registers\n");
	showRegisters();
}

// ---------------------------------------------------------------------
// Periodic Modbus RTU kernel call
// ---------------------------------------------------------------------
void SlaveClass::perform(void){
	_slave->poll();
}

// ---------------------------------------------------------------------
// Check for register update
// ---------------------------------------------------------------------
bool SlaveClass::updateAvailable(void){
	
	if(_updateReceived){
		// Only one signal per update
		_updateReceived = false;
		return(true);
	} 
	else {
		return(false);
	}
}

// ---------------------------------------------------------------------
// Show current register content
// ---------------------------------------------------------------------
void SlaveClass::showRegisters(void){
	char buf[128];
	int len = 0;
	for(int i = 0; i < _num_holding_regs; i++){
		len +=	snprintf(buf + len, sizeof(buf)-(len + 1), "   %d:%04X", i, _holding_regs[i]);
	}
	Serial.print("regs");
	Serial.println(buf);
}

// ---------------------------------------------------------------------
// Instance methods invoked on callback
// ---------------------------------------------------------------------
uint8_t SlaveClass::readHoldingRegs(uint8_t fc, uint16_t address, uint16_t length){
	Serial.print("Read Register(s)\n");
	if ((address + length) > _num_holding_regs)
		return STATUS_ILLEGAL_DATA_ADDRESS;
    
	for (int i = 0; i < length; i++){
		_slave->writeRegisterToBuffer(i, _holding_regs[address + i]);
	}
	return STATUS_OK;
}

uint8_t SlaveClass::writeHoldingRegs(uint8_t fc, uint16_t address, uint16_t length){
	Serial.print("Write Register(s)\n");
	if ((address + length) > _num_holding_regs)
		return STATUS_ILLEGAL_DATA_ADDRESS;
    
	for (int i = 0; i < length; i++){
		_holding_regs[address + i] = _slave->readRegisterFromBuffer(i);
	}
	_updateReceived = true;
	return STATUS_OK;
}


// ---------------------------------------------------------------------
// Static callback helper methods
// ---------------------------------------------------------------------
// The Modbus kernel provides a pointer to the associated instance of SlaveClass.
// So we can forward the request to the desired instance method
uint8_t SlaveClass::_readHoldingRegs(uint8_t fc, uint16_t address, uint16_t length, void *context){
	return ((SlaveClass *)context)->readHoldingRegs(fc, address, length);
}
uint8_t SlaveClass::_writeHoldingRegs(uint8_t fc, uint16_t address, uint16_t length, void *context){
	return ((SlaveClass *)context)->writeHoldingRegs(fc, address, length);
}
