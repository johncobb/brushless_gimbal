/*
 * eeprom.h
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include <avr/eeprom.h>

uint8_t eeprom_read(int address);
void eeprom_write(int address, uint8_t value);

#endif /* EEPROM_H_ */
