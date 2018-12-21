/*
    Modbus slave simple example

    Control and Read Arduino I/Os using Modbus serial connection.

    This sketch show how to use the callback vector for reading and
    controleing Arduino I/Os.

    * Controls digital output pins as modbus coils.
    * Reads digital inputs state as discreet inputs.
    * Reads analog inputs as input registers.

    The circuit: ( see: ./extras/ModbusSetch.pdf )
    * An Arduino.
    * 2 x LEDs, with 220 ohm resistors in series.
    * A switch connected to a digital input pin.
    * A potentiometer connected to an analog input pin.
    * A RS485 module (Optional) connected to RX/TX and a digital control pin.

    Created 8 12 2015
    By Yaacov Zamir

    https://github.com/yaacov/ArduinoModbusSlave

*/

#include <ModbusSlave.h>

/* slave id = 1, rs485 control-pin = 8, baud = 9600
 */
#define SLAVE_ID 1
#define CTRL_PIN 8
#define BAUDRATE 9600

/**
 *  Modbus object declaration
 */
Modbus slave(SLAVE_ID, CTRL_PIN);

void setup() {
    /* set some pins for output
     */
    pinMode(10, INPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);

    /* register handler functions
     * into the modbus slave callback vector.
     */
    slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
    slave.cbVector[CB_READ_DISCRETE_INPUTS] = readDigitalIn;
    slave.cbVector[CB_READ_INPUT_REGISTERS] = readAnalogIn;

    // set Serial and slave at baud 9600.
    Serial.begin( BAUDRATE );
    slave.begin( BAUDRATE );
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
 * Handle Force Single Coil (FC=05) and Force Multiple Coils (FC=15)
 * set digital output pins (coils).
 */
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
    // set digital pin state(s).
    for (int i = 0; i < length; i++) {
        digitalWrite(address + i, slave.readCoilFromBuffer(i));
    }

    return STATUS_OK;
}

/**
 * Handel Read Input Status (FC=02)
 * write back the values from digital in pins (input status).
 *
 * handler functions must return void and take:
 *      uint8_t  fc - function code
 *      uint16_t address - first register/coil address
 *      uint16_t length/status - length of data / coil status
 */
uint8_t readDigitalIn(uint8_t fc, uint16_t address, uint16_t length) {
    // read digital input
    for (int i = 0; i < length; i++) {
        slave.writeCoilToBuffer(i, digitalRead(address + i));
    }

    return STATUS_OK;
}

/**
 * Handel Read Input Registers (FC=04)
 * write back the values from analog in pins (input registers).
 */
uint8_t readAnalogIn(uint8_t fc, uint16_t address, uint16_t length) {
    // read analog input
    for (int i = 0; i < length; i++) {
        slave.writeRegisterToBuffer(i, analogRead(address + i));
    }

    return STATUS_OK;
}

