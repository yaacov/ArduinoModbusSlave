#ifndef SlaveClass_h
#define SlaveClass_h
#include "Arduino.h" 

#include <ModbusSlave.h>
class SlaveClass {
public:
	SlaveClass(Stream *serialStream, unsigned int baud, int transmissionControlPin, uint8_t slaveId);

	void perform(void);
	bool updateAvailable(void);
	void showRegisters(void);
	
	
private:
	// A set of holding registers with predefined values
	const uint8_t 	_num_holding_regs = 10;
	uint16_t 		_holding_regs[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
	bool			_updateReceived = false;
	
	// Instance methods connected to kernel callback
	uint8_t readHoldingRegs(uint8_t fc, uint16_t address, uint16_t length);
	uint8_t writeHoldingRegs(uint8_t fc, uint16_t address, uint16_t length);
	
	// Modbus RTU kernel
	Modbus *_slave;
	// Hooks for kernel callback
	static uint8_t _readHoldingRegs(uint8_t fc, uint16_t address, uint16_t length, void *context);
	static uint8_t _writeHoldingRegs(uint8_t fc, uint16_t address, uint16_t length, void *context);
	
};
#endif
