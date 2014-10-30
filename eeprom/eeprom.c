/*
 * eeprom.c
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */


#include "eeprom.h"


uint8_t eeprom_read(int address)
{
	return eeprom_read_byte((unsigned char *) address);
}

void eeprom_write(int address, uint8_t value)
{
	eeprom_write_byte((unsigned char *) address, value);
}

