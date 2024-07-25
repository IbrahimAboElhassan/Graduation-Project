/*
 * I2C.h
 *
 * Created: 8/12/2021 2:51:14 AM
 */ 

/*#ifndef I2C_H_
#define I2C_H_

#include "CPU_CONFIGURATION.h"
#include <stdint.h>

#define I2C_READ    1
#define I2C_WRITE   0

#define I2C_ACK     1
#define I2C_NACK    0

#define I2C_SUCCESS 0
#define I2C_ERROR   1

void I2C_Init(unsigned long SCL_Clock);
uint8_t I2C_Start(uint8_t address, uint8_t direction);
void I2C_Stop(void);
uint8_t I2C_Write(uint8_t data);
uint8_t I2C_Read(uint8_t ack);
uint8_t I2C_ReadNack(void);

#endif /* I2C_H_ */

#ifndef I2C_H_
#define I2C_H_

#include "CPU_CONFIGURATION.h"
#include <stdint.h>

#define I2C_READ    1
#define I2C_WRITE   0

#define I2C_ACK     1
#define I2C_NACK    0

#define I2C_SUCCESS 0
#define I2C_ERROR   1

void I2C_Init(unsigned long SCL_Clock);
uint8_t I2C_Start(uint8_t address, uint8_t direction);
void I2C_Stop(void);
uint8_t I2C_Write(uint8_t data);
uint8_t I2C_Read(uint8_t ack);
uint8_t I2C_ReadNack(void);

#endif /* I2C_H_ */