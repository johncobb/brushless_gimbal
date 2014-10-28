/*
 * eeprom.c
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */


#include "eeprom.h"


uint8_t read(int address)
{
	return eeprom_read_byte((unsigned char *) address);
}

void write(int address, uint8_t value)
{
	eeprom_write_byte((unsigned char *) address, value);
}
