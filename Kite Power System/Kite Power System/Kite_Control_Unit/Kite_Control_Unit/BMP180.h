/*
 * BMP180.h
 *
 * Created: 3/27/2024 11:51:23 PM
 *  Author: Ibrahim
 */ 

/*
#ifndef BMP180_H_
#define BMP180_H_
#include <stdint.h>

#define BMP180_I2C_ADDR					0x77
#define BMP180_I2C_TIMEOUT				1000
#define BMP180_CONTROL_REG				0xf4
#define BMP180_MSB_REG					0xf6
#define BMP180_LSB_REG					0xf7
#define BMP180_XLSB_REG					0xf8
#define BMP180_SOFT_RESET_REG			0xe0
#define BMP180_SOFT_RESET				0xb6

typedef enum BMP180_OSS {
	BMP180_LOW, 
	BMP180_STANDARD,
	BMP180_HIGH,
    BMP180_ULTRA,
} BMP180_OSS;

typedef enum BMP180_CalIndex {
	BMP180_INDEX_AC1,
	BMP180_INDEX_AC2,
	BMP180_INDEX_AC3,
	BMP180_INDEX_AC4,
	BMP180_INDEX_AC5,
	BMP180_INDEX_AC6,
	BMP180_INDEX_B1,
	BMP180_INDEX_B2,
	BMP180_INDEX_MB,
	BMP180_INDEX_MC,
	BMP180_INDEX_MD,
} BMP180_CalIndex;

typedef struct BMP180_EEPROM {
	short BMP180_AC1;
	short BMP180_AC2;
	short BMP180_AC3;
	unsigned short BMP180_AC4;
	unsigned short BMP180_AC5;
	unsigned short BMP180_AC6;
	short BMP180_B1;
	short BMP180_B2;
	short BMP180_MB;
	short BMP180_MC;
	short BMP180_MD;
} BMP180_EEPROM;

extern const uint8_t BMP180_EEPROM_ADDR_MSB[11];
extern const uint8_t BMP180_EEPROM_ADDR_LSB[11];

extern const uint8_t BMP180_CMD_TEMP;
extern const uint8_t BMP180_DELAY_TEMP;
extern const uint8_t BMP180_CMD_PRES;
extern const uint8_t BMP180_DELAY_PRES;


void BMP180_Init(uint8_t mode, BMP180_OSS oss);
float BMP180_TemperatureCompensation(uint8_t raw_temp_data[2], uint8_t raw_pressure_data[3]);
float BMP180_MeasurePressure();
float BMP180_CalculateAltitude(float compensated_pressure_hPa);


#endif /* BMP180_H_ */


/*
 * BMP180.h
 * Created: 3/27/2024 11:51:23 PM
 * Author: Ibrahim
 */

#ifndef BMP180_H_
#define BMP180_H_

#include <stdint.h>

// BMP180 I2C Address
#define BMP180_I2C_ADDR                 0x77
#define BMP180_I2C_TIMEOUT              1000

// BMP180 Registers
#define BMP180_CONTROL_REG              0xF4
#define BMP180_MSB_REG                  0xF6
#define BMP180_LSB_REG                  0xF7
#define BMP180_XLSB_REG                 0xF8
#define BMP180_SOFT_RESET_REG           0xE0
#define BMP180_SOFT_RESET               0xB6

// BMP180 Commands and Delays
#define BMP180_CMD_TEMP                 0x2E
#define BMP180_CMD_PRES                 0x34
#define BMP180_DELAY_TEMP               5
#define BMP180_DELAY_PRES               8

// BMP180 OSS (Oversampling Settings)
typedef enum {
	BMP180_LOW,
	BMP180_STANDARD,
	BMP180_HIGH,
	BMP180_ULTRA,
} BMP180_OSS;

// BMP180 Calibration Data Structure
typedef struct {
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
} BMP180_EEPROM;

// Function prototypes
int8_t BMP180_Init(BMP180_OSS oss);
float BMP180_ReadTemperature(void);
float BMP180_ReadPressure(void);
float BMP180_CalculateAltitude(float pressure_hPa, float seaLevelPressure_hPa);
//
void BMP180_SetReferencePressure(float pressure_hPa);
float BMP180_CalculateRelativeAltitude(float currentPressure_hPa);
//
float BMP180_CalculateRelativePressure(float currentPressure_hPa);
//
void BMP180_SoftReset(void);

#endif /* BMP180_H_ */