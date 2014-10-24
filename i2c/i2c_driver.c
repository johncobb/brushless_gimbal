/*
 * i2c_driver.c
 *
 *  Created on: Oct 22, 2014
 *      Author: jcobb
 */


#include <stdlib.h>
#include <avr/pgmspace.h>
#include "../util/clock.h"
#include "../util/log.h"
#include "twi/twi.h"
#include "i2c_driver.h"
#include "i2c.h"

#ifdef I2C_DEBUG
static const char _tag[] PROGMEM = "i2c_driver: ";
#endif


uint16_t readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

uint8_t readByte(uint8_t address, uint8_t regAddr, uint8_t *data, uint16_t timeout)
{
	return readBytes(address, regAddr, 1, data, timeout);
}


uint8_t readBytes(uint8_t address, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout)
{

#ifdef I2C_DEBUG
	LOG("I2C (0x");
	LOGHEX(&address, 1);
	LOG(") reading ");
	LOG("%d", length);
	LOG(" byte(s) from 0x");
	LOGHEX(&regAddr, 1);
	LOG("...");
#endif

	uint8_t count = 0;
	//clock_time_t time = clock_time();
	clock_time_t future = clock_time() + timeout;

	for (uint8_t i=0; i< length; i+= min(length, BUFFER_LENGTH))
	{
		i2c_begin_transmission(address);
		i2c_write_byte(regAddr);
		i2c_end_transmission(1);

		i2c_request_from(address, (uint8_t) min(length - i, BUFFER_LENGTH), 1);

		while(i2c_available() > 0)
		{
#ifdef I2C_DEBUG
			LOGHEX(&data[count], 1);
			if(count + 1 < length) LOG(" ");
#endif
			count++;
			// break on timeout
			if(clock_time() > future)
			{
#ifdef I2C_DEBUG
				LOG("\r\nreadBytes: timeout occured: (%d)ms\r\n", timeout);
#endif
				break;
			}

			data[count] = i2c_read();
		}

	}

#ifdef I2C_DEBUG
	LOG(" done. (");
	LOG("%d", count);
	LOG(" read)\r\n");
#endif

	return count;
}


bool writeByte(uint8_t address, uint8_t regAddr, uint8_t data)
{
	return writeBytes(address, regAddr, 1, &data);
}

bool writeBytes(uint8_t address, uint8_t regAddr, uint8_t length, uint8_t *data)
{
#ifdef I2C_DEBUG
	LOG("I2C (0x");
	LOGHEX(&address, 1);
	LOG(") writting ");
	LOG("%d", length);
	LOG(" byte(s) to 0x");
	LOGHEX(&regAddr, 1);
	LOG("...");
#endif

	uint8_t status = 0;

	i2c_begin_transmission(address);
	i2c_write_byte(regAddr);

	for(uint8_t i=0; i<length; i++)
	{
		i2c_write_byte(data[i]);
#ifdef I2C_DEBUG
		LOGHEX(&data[i], 1);
		if (i + 1 < length) LOG(" ");
#endif
	}

	status = i2c_end_transmission(1);

#ifdef I2C_DEBUG
	LOG(" status: %d", status);
	LOG(" done.\r\n");

#endif

	return status == 0;
}

bool writeBit(uint8_t address, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
	uint8_t b;
	readByte(address, regAddr, &b, readTimeout);
	b = (data |= 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return writeByte(address, regAddr, b);
}

bool writeBits(uint8_t address, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value

	uint8_t b;

	// TODO: REVIEW ADDITIONAL TIMEOUT VARIALBE ON READBYTE
	if(readByte(address, regAddr, &b, readTimeout) != 0)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		b &= ~(mask); // zero all important bits in existing byte
		b |= data; // combine data with existing byte
		return writeByte(address, regAddr, b);
	}
	else
		return false;
}

uint8_t readBit(uint8_t address, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout)
{
	uint8_t b;
	uint8_t count = readByte(address, regAddr, &b, timeout);
	*data = b & (1 << bitNum);
	return count;
}

uint8_t readBits(uint8_t address, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout)
{

    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted

	uint8_t count;
	uint8_t b;

	if((count = readByte(address, regAddr, &b, timeout)) != 0)
	{
		uint8_t mask = ((1 << length) -1) << (bitStart - length + 1);
		b  &= mask;
		b >>= (bitStart - length + 1);
		*data = b;
	}
	return count;
}
