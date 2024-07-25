/*
 * BMP180.c
 *
 * Created: 3/27/2024 11:51:09 PM
 *  Author: Ibrahim
 */ 

#include "BMP180.h"
#include "I2C.h"
#include <util/delay.h>
#include <math.h>

// BMP180 EEPROM Calibration Data
static BMP180_EEPROM calibData;
static BMP180_OSS current_oss;
static float referencePressure = 1013.25;//

// Read 16-bit value from BMP180 register
static int16_t BMP180_ReadInt(uint8_t addr) {
	int16_t value;
	if (I2C_Start(BMP180_I2C_ADDR, I2C_WRITE) != I2C_SUCCESS) return 0;
	if (I2C_Write(addr) != I2C_SUCCESS) return 0;
	if (I2C_Start(BMP180_I2C_ADDR, I2C_READ) != I2C_SUCCESS) return 0;
	value = ((int16_t)I2C_Read(I2C_ACK) << 8) | I2C_Read(I2C_NACK);
	I2C_Stop();
	return value;
}


int8_t BMP180_Init(BMP180_OSS oss) {
	I2C_Init(50000);
	delay_ms(100);

	uint8_t chip_id;
	if (I2C_Start(BMP180_I2C_ADDR, I2C_WRITE) != I2C_SUCCESS) return -1;
	if (I2C_Write(0xD0) != I2C_SUCCESS) return -2;
	if (I2C_Start(BMP180_I2C_ADDR, I2C_READ) != I2C_SUCCESS) return -3;
	chip_id = I2C_Read(I2C_NACK);
	I2C_Stop();
	if (chip_id != 0x55) return -4;
	
	// Read calibration data from EEPROM
	calibData.AC1 = BMP180_ReadInt(0xAA);
	if (calibData.AC1 == 0) return -5;
	calibData.AC2 = BMP180_ReadInt(0xAC);
	calibData.AC3 = BMP180_ReadInt(0xAE);
	calibData.AC4 = BMP180_ReadInt(0xB0);
	calibData.AC5 = BMP180_ReadInt(0xB2);
	calibData.AC6 = BMP180_ReadInt(0xB4);
	calibData.B1 = BMP180_ReadInt(0xB6);
	calibData.B2 = BMP180_ReadInt(0xB8);
	calibData.MB = BMP180_ReadInt(0xBA);
	calibData.MC = BMP180_ReadInt(0xBC);
	calibData.MD = BMP180_ReadInt(0xBE);

	current_oss = oss;
	return 0; // Success
}

// Read uncompensated temperature from BMP180
static int32_t BMP180_ReadUT(void) {
	if (I2C_Start(BMP180_I2C_ADDR, I2C_WRITE) != I2C_SUCCESS) return 0;
	if (I2C_Write(BMP180_CONTROL_REG) != I2C_SUCCESS) return 0;
	if (I2C_Write(BMP180_CMD_TEMP) != I2C_SUCCESS) return 0;
	I2C_Stop();
	delay_ms(BMP180_DELAY_TEMP);
	return BMP180_ReadInt(BMP180_MSB_REG);
}

// Read uncompensated pressure from BMP180
static int32_t BMP180_ReadUP(void) {
	uint8_t cmd;
	uint8_t delay;

	switch (current_oss) {
		case BMP180_ULTRA:
		cmd = BMP180_CMD_PRES + (3 << 6);
		delay = 26;
		break;
		case BMP180_HIGH:
		cmd = BMP180_CMD_PRES + (2 << 6);
		delay = 14;
		break;
		case BMP180_STANDARD:
		cmd = BMP180_CMD_PRES + (1 << 6);
		delay = 8;
		break;
		case BMP180_LOW:
		default:
		cmd = BMP180_CMD_PRES;
		delay = 5;
		break;
	}

	if (I2C_Start(BMP180_I2C_ADDR, I2C_WRITE) != I2C_SUCCESS) return 0;
	if (I2C_Write(BMP180_CONTROL_REG) != I2C_SUCCESS) return 0;
	if (I2C_Write(cmd) != I2C_SUCCESS) return 0;
	I2C_Stop();
	delay_ms(delay);

	if (I2C_Start(BMP180_I2C_ADDR, I2C_READ) != I2C_SUCCESS) return 0;
	int32_t up = ((int32_t)I2C_Read(I2C_ACK) << 16) | ((int32_t)I2C_Read(I2C_ACK) << 8) | I2C_Read(I2C_NACK);
	I2C_Stop();

	up >>= (8 - current_oss);
	return up;
}

// Calculate true temperature
float BMP180_ReadTemperature(void) {
	int32_t ut = BMP180_ReadUT();
	if (ut == 0) return 0; // Error reading temperature

	int32_t x1 = ((ut - (int32_t)calibData.AC6) * (int32_t)calibData.AC5) >> 15;
	int32_t x2 = ((int32_t)calibData.MC << 11) / (x1 + calibData.MD);
	int32_t b5 = x1 + x2;
	float temperature = ((b5 + 8) >> 4) / 10.0;
	return temperature;
}

// Calculate true pressure
float BMP180_ReadPressure(void) {
	int32_t ut = BMP180_ReadUT();
	int32_t up = BMP180_ReadUP();
	if (ut == 0 || up == 0) return 0; // Error reading temperature or pressure

	int32_t x1 = ((ut - (int32_t)calibData.AC6) * (int32_t)calibData.AC5) >> 15;
	int32_t x2 = ((int32_t)calibData.MC << 11) / (x1 + calibData.MD);
	int32_t b5 = x1 + x2;

	int32_t b6 = b5 - 4000;
	x1 = (calibData.B2 * (b6 * b6 >> 12)) >> 11;
	x2 = calibData.AC2 * b6 >> 11;
	int32_t x3 = x1 + x2;
	int32_t b3 = (((calibData.AC1 * 4 + x3) << current_oss) + 2) >> 2;

	x1 = calibData.AC3 * b6 >> 13;
	x2 = (calibData.B1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	uint32_t b4 = (calibData.AC4 * (uint32_t)(x3 + 32768)) >> 15;
	uint32_t b7 = ((uint32_t)up - b3) * (50000 >> current_oss);

	int32_t p;
	if (b7 < 0x80000000) {
		p = (b7 * 2) / b4;
		} else {
		p = (b7 / b4) * 2;
	}

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;

	//return (float)p / 100.0;
	return (float)p;
}

// Calculate altitude
float BMP180_CalculateAltitude(float pressure_hPa, float seaLevelPressure_hPa) {
	return 44330.0 * (1.0 - pow(pressure_hPa / seaLevelPressure_hPa, 0.19029495718));
}

//
void BMP180_SetReferencePressure(float pressure_hPa) {
	referencePressure = pressure_hPa;
}

float BMP180_CalculateRelativePressure(float currentPressure_hPa) {
	return currentPressure_hPa - referencePressure;
}
//

// Calculate relative altitude based on the reference pressure
float BMP180_CalculateRelativeAltitude(float currentPressure_hPa) {
	return 44330.0 * (1.0 - pow(currentPressure_hPa / referencePressure, 0.19029495718));
}//

// Perform soft reset
void BMP180_SoftReset(void) {
	if (I2C_Start(BMP180_I2C_ADDR, I2C_WRITE) != I2C_SUCCESS) return;
	I2C_Write(BMP180_SOFT_RESET_REG);
	I2C_Write(BMP180_SOFT_RESET);
	I2C_Stop();
	delay_ms(100); // Wait for reset to complete
}

void delay_ms(uint16_t ms)
{
	uint16_t delay_count = F_CPU / 4000;

	volatile uint16_t i;
	while (ms) {
		for (i = 0; i < delay_count; i++) {
			__asm__ volatile ("nop");
		}
		ms--;
	}
}