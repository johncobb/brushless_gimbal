/*
 * i2c.h
 *
 *  Created on: Oct 22, 2014
 *      Author: jcobb
 */

#ifndef I2C_H_
#define I2C_H_

#include "../util/defines.h"

//#define I2C_DEBUG	1

#define BUFFER_LENGTH	32

extern uint8_t rxBuffer[BUFFER_LENGTH];
extern uint8_t rxBufferIndex;
extern uint8_t rxBufferLength;

extern uint8_t txAddress;
extern uint8_t txBuffer[BUFFER_LENGTH];
extern uint8_t txBufferIndex;
extern uint8_t txBufferLength;

extern uint8_t transmitting;



void i2c_begin();
void i2c_begin_address(uint8_t address);
uint8_t i2c_request_from(uint8_t address, uint8_t quantity, uint8_t stop);

void i2c_begin_transmission(uint8_t address);
uint8_t i2c_end_transmission(uint8_t sendStop);

uint8_t i2c_available();
size_t i2c_write_byte(uint8_t data);
size_t i2c_write(uint8_t *data, size_t quantity);
uint8_t i2c_read();
uint8_t i2c_peek();

#endif /* I2C_H_ */
