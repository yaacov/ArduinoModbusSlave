/**
 *  Modbus slave example
 */

#include <ModbusSlave.h>

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
    slave.cbVector[CB_WRITE_COIL] = writeDigitlOut;
    
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
 * Handel Force Single Coil (FC=05)
 * set digital output pins (coils) on and off
 *
 * handler functions must return void and take:
 *      uint8_t  fc - function code
 *      uint16_t address - first register/coil address
 *      uint16_t length/status - length of data / coil status
 */
void writeDigitlOut(uint8_t fc, uint16_t address, uint16_t status) {
    if (status == COIL_ON) {
        digitalWrite(address, HIGH);
    } else if (status == COIL_OFF) {
        digitalWrite(address, LOW);
    }
}

