/*
 * i2c.c
 *
 *  Created on: Oct 22, 2014
 *      Author: jcobb
 */



#include "twi/twi.h"
#include "i2c.h"

uint8_t rxBuffer[BUFFER_LENGTH];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t txAddress = 0;
uint8_t txBuffer[BUFFER_LENGTH];
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;

uint8_t transmitting = 0;



void i2c_begin()
{
	rxBufferIndex = 0;
	rxBufferLength = 0;

	txBufferIndex = 0;
	txBufferLength = 0;

	twi_init();
}

void i2c_begin_address(uint8_t address)
{
	twi_setAddress(address);
	/*
	 * TODO: NEED TO RESEARCH INTERRUPTS
	twi_attachSlaveTxEvent(onRequestService);
	twi_attachSlaveRxEvent(onReceiveService);
	*/
	i2c_begin();
}

void i2c_begin_transmission(uint8_t address)
{
	// set transmission flag
	transmitting = 1;
	// set slave address
	txAddress = address;
	// reset buffer vars
	txBufferIndex = 0;
	txBufferLength = 0;
}

uint8_t i2c_request_from(uint8_t address, uint8_t quantity, uint8_t stop)
{
	// sanity check
	if(quantity > BUFFER_LENGTH)
	{
		quantity = BUFFER_LENGTH;
	}
	// perform blocking read into buffer
	uint8_t read = twi_readFrom(address, rxBuffer, quantity, stop);
	// reset buffer vars
	rxBufferIndex = 0;
	rxBufferLength = read;

	return read;
}

uint8_t i2c_end_transmission(uint8_t sendStop)
{
	// transmit buffer (blocking)
	uint8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1, sendStop);
	// reset buffer vars
	txBufferIndex = 0;
	txBufferLength = 0;
	// set transmission flag
	transmitting = 0;

	return ret;
}

// Private function i2c_write_data
size_t i2c_write_byte(uint8_t data)
{

	if(transmitting)
	{
		// in master transmitter mode
		// ignore if buffer full
		if(txBufferLength >=  BUFFER_LENGTH)
		{
			// TODO: setWriteError
			// create a callback singaler
			return 0;
		}
		// put byte in tx buffer
		txBuffer[txBufferIndex] = data;
		++txBufferIndex;
		txBufferLength = txBufferIndex;
	}
	else
	{
		twi_transmit(&data, 1);
	}
	return 1; // Placeholder
}

size_t i2c_write(uint8_t *data, size_t quantity)
{
	if(transmitting)
	{
		// in master transmitter mode
		for (size_t i=0; i<quantity; ++i)
		{
			i2c_write_byte(data[i]);
		}
	}
	else
	{
		// in slave send mode
		// reply to master
		twi_transmit(data, quantity);
	}

	return quantity;
}

uint8_t i2c_available()
{
	return rxBufferLength - rxBufferIndex;
}

uint8_t i2c_read()
{
	uint8_t value = -1;

	if(rxBufferIndex < rxBufferLength){
		value = rxBuffer[rxBufferIndex];
		++rxBufferIndex;
	}

	return value;
}
uint8_t i2c_peek()
{
	uint8_t value = -1;

	if(rxBufferIndex < rxBufferLength)
	{
		value = rxBuffer[rxBufferIndex];
	}

	return value;
}



