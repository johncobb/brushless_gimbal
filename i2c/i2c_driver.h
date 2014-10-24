/*
 * i2c_driver.h
 *
 *  Created on: Oct 22, 2014
 *      Author: jcobb
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include "../util/defines.h"

#define I2CDEV_DEFAULT_READ_TIMEOUT     1000

uint8_t readByte(uint8_t address, uint8_t regAddr, uint8_t *data, uint16_t timeout);
uint8_t readBytes(uint8_t address, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout);

bool writeByte(uint8_t address, uint8_t regAddr, uint8_t data);
bool writeBytes(uint8_t address, uint8_t regAddr, uint8_t length, uint8_t *data);

bool writeBit(uint8_t address, uint8_t regAddr, uint8_t bitNum, uint8_t data);
bool writeBits(uint8_t address, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);

uint8_t readBit(uint8_t address, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout);
uint8_t readBits(uint8_t address, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout);
#endif /* I2C_DRIVER_H_ */
