/*
 * I2C.c
 *
 * Created: 8/12/2021 2:57:30 AM
 */ 

/*#include "i2c.h"

#define prescaler 1

void I2C_Init(unsigned long SCL_Clock)
{
	// Try a lower clock speed, e.g., 50 kHz
	TWBR = (uint8_t)(((F_CPU / 50000) - 16) / (2 * prescaler));
	if (prescaler == 1)
	TWSR = 0;
	else if (prescaler == 4)
	TWSR = 1;
	else if (prescaler == 16)
	TWSR = 2;
	else if (prescaler == 64)
	TWSR = 3;
}


uint8_t I2C_Start(uint8_t address, uint8_t direction)
{
	// Send START condition
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));

	// Check if START was sent successfully
	uint8_t status = TWSR & 0xF8;
	if (status != 0x08 && status != 0x10)  // 0x10 is for repeated start
	return status;  // Return the actual status for debugging

	// Send device address
	TWDR = (address << 1) | direction;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));

	// Check if address was sent successfully
	status = TWSR & 0xF8;
	if (status != 0x18 && status != 0x40)
	return status;  // Return the actual status for debugging

	return I2C_SUCCESS;
}

void I2C_Stop(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	while (TWCR & (1 << TWSTO));
}

uint8_t I2C_Write(uint8_t data)
{
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));

	if ((TWSR & 0xF8) != 0x28)
	return I2C_ERROR;

	return I2C_SUCCESS;
}

uint8_t I2C_Read(uint8_t ack)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (ack << TWEA);
	while (!(TWCR & (1 << TWINT)));

	return TWDR;
}

uint8_t I2C_ReadNack(void)
{
	return I2C_Read(I2C_NACK);
}*/

#include "i2c.h"

#define prescaler 1

void I2C_Init(unsigned long SCL_Clock)
{
	// Try a lower clock speed, e.g., 50 kHz
	TWBR = (uint8_t)(((F_CPU / 50000) - 16) / (2 * prescaler));
	if (prescaler == 1)
	TWSR = 0;
	else if (prescaler == 4)
	TWSR = 1;
	else if (prescaler == 16)
	TWSR = 2;
	else if (prescaler == 64)
	TWSR = 3;
}


uint8_t I2C_Start(uint8_t address, uint8_t direction)
{
	// Send START condition
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));

	// Check if START was sent successfully
	uint8_t status = TWSR & 0xF8;
	if (status != 0x08 && status != 0x10)  // 0x10 is for repeated start
	return status;  // Return the actual status for debugging

	// Send device address
	TWDR = (address << 1) | direction;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));

	// Check if address was sent successfully
	status = TWSR & 0xF8;
	if (status != 0x18 && status != 0x40)
	return status;  // Return the actual status for debugging

	return I2C_SUCCESS;
}

void I2C_Stop(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	while (TWCR & (1 << TWSTO));
}

uint8_t I2C_Write(uint8_t data)
{
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));

	if ((TWSR & 0xF8) != 0x28)
	return I2C_ERROR;

	return I2C_SUCCESS;
}

uint8_t I2C_Read(uint8_t ack)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (ack << TWEA);
	while (!(TWCR & (1 << TWINT)));

	return TWDR;
}

uint8_t I2C_ReadNack(void)
{
	return I2C_Read(I2C_NACK);
}