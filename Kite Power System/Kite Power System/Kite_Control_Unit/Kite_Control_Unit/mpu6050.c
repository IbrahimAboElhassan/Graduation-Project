/*#include "mpu6050.h"

void MPU6050_Init(void) {
	I2C_Start(MPU6050_ADDR, I2C_WRITE);
	I2C_Write(0x6B); // Power management register
	I2C_Write(0x00); // Wake up MPU6050
	I2C_Stop();
}

void MPU6050_Read_Accel(int16_t* ax, int16_t* ay, int16_t* az) {
	I2C_Start(MPU6050_ADDR, I2C_WRITE);
	I2C_Write(0x3B); // Starting register for accelerometer data
	I2C_Stop();

	I2C_Start(MPU6050_ADDR, I2C_READ); // Read mode

	*ax = (I2C_Read(I2C_ACK) << 8) | I2C_Read(I2C_ACK);
	*ay = (I2C_Read(I2C_ACK) << 8) | I2C_Read(I2C_ACK);
	*az = (I2C_Read(I2C_ACK) << 8) | I2C_ReadNack();

	I2C_Stop();
}*/

/*#include "mpu6050.h"

void MPU6050_Init(void) {
	uint8_t status;

	status = I2C_Start(MPU6050_ADDR, I2C_WRITE);
	if (status != I2C_SUCCESS) {
		LCD_Write_String((Uint8t*)"Start fail");
		while (1); // Stay here to debug
	}
	status = I2C_Write(0x6B); // Power management register
	if (status != I2C_SUCCESS) {
		LCD_Write_String((Uint8t*)"Write fail");
		while (1);
	}
	status = I2C_Write(0x00); // Wake up MPU6050
	if (status != I2C_SUCCESS) {
		LCD_Write_String((Uint8t*)"Write fail");
		while (1);
	}
	I2C_Stop();
}

void MPU6050_Read_Accel(int16_t* ax, int16_t* ay, int16_t* az) {
	uint8_t status;

	status = I2C_Start(MPU6050_ADDR, I2C_WRITE);
	if (status != I2C_SUCCESS) {
		LCD_Write_String((Uint8t*)"Start fail");
		while (1);
	}
	status = I2C_Write(0x3B); // Starting register for accelerometer data
	if (status != I2C_SUCCESS) {
		LCD_Write_String((Uint8t*)"Write fail");
		while (1);
	}
	I2C_Stop();

	status = I2C_Start(MPU6050_ADDR, I2C_READ); // Read mode
	if (status != I2C_SUCCESS) {
		// Handle error
		while (1);
	}

	*ax = (I2C_Read(I2C_ACK) << 8) | I2C_Read(I2C_ACK);
	*ay = (I2C_Read(I2C_ACK) << 8) | I2C_Read(I2C_ACK);
	*az = (I2C_Read(I2C_ACK) << 8) | I2C_ReadNack();

	I2C_Stop();
}*/

/*#include "mpu6050.h"
#include "LCD.h"
#include <util/delay.h>

AccelCalibration calibration = {0};

Uint8t MPU6050_Init(void) {
	Uint8t status;

	status = I2C_Start(MPU6050_ADDR, I2C_WRITE);
	if (status != I2C_SUCCESS) {
		return 1; // Error: Start condition failed
	}
	
	status = I2C_Write(0x6B); // Power management register
	if (status != I2C_SUCCESS) {
		return 2; // Error: Write to register failed
	}
	
	status = I2C_Write(0x00); // Wake up MPU6050
	if (status != I2C_SUCCESS) {
		return 3; // Error: Wake up failed
	}
	
	I2C_Stop();
	return 0; // Success
}

Uint8t MPU6050_Read_Accel(float* ax, float* ay, float* az) {
	int16_t raw_ax, raw_ay, raw_az;
	Uint8t status;

	status = I2C_Start(MPU6050_ADDR, I2C_WRITE);
	if (status != I2C_SUCCESS) return 1;

	status = I2C_Write(0x3B); // Starting register for accelerometer data
	if (status != I2C_SUCCESS) return 2;

	I2C_Stop();

	status = I2C_Start(MPU6050_ADDR, I2C_READ);
	if (status != I2C_SUCCESS) return 3;

	raw_ax = (I2C_Read(I2C_ACK) << 8) | I2C_Read(I2C_ACK);
	raw_ay = (I2C_Read(I2C_ACK) << 8) | I2C_Read(I2C_ACK);
	raw_az = (I2C_Read(I2C_ACK) << 8) | I2C_ReadNack();

	I2C_Stop();

	*ax = (float)raw_ax / 16384.0 - calibration.x_offset;
	*ay = (float)raw_ay / 16384.0 - calibration.y_offset;
	*az = (float)raw_az / 16384.0 - calibration.z_offset;

	return 0;
}

void MPU6050_Calibrate(void) {
	int32_t x_sum = 0, y_sum = 0, z_sum = 0;
	float ax, ay, az;
	
	LCD_Clear();
	LCD_Write_String((Uint8t*)"Calibrating...");
	
	for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
		MPU6050_Read_Accel(&ax, &ay, &az);
		x_sum += (int32_t)(ax * 16384.0);
		y_sum += (int32_t)(ay * 16384.0);
		z_sum += (int32_t)(az * 16384.0);
		_delay_ms(10);
	}
	
	calibration.x_offset = (float)x_sum / CALIBRATION_SAMPLES / 16384.0;
	calibration.y_offset = (float)y_sum / CALIBRATION_SAMPLES / 16384.0;
	calibration.z_offset = (float)z_sum / CALIBRATION_SAMPLES / 16384.0 - 1.0; // Assuming Z-axis is vertical
	
	LCD_Clear();
	LCD_Write_String((Uint8t*)"Calibration done");
	_delay_ms(1000);
}*/


#include "mpu6050.h"
#include "LCD.h"
#include <math.h>
#include <util/delay.h>

AccelCalibration calibration = {0};

Uint8t MPU6050_Init(void) {
	Uint8t status;

	status = I2C_Start(MPU6050_ADDR, I2C_WRITE);
	if (status != I2C_SUCCESS) return 1;

	status = I2C_Write(0x6B); // Power management register
	if (status != I2C_SUCCESS) return 2;

	status = I2C_Write(0x00); // Wake up MPU6050
	if (status != I2C_SUCCESS) return 3;

	I2C_Stop();

	// Configure gyroscope range (optional)
	status = I2C_Start(MPU6050_ADDR, I2C_WRITE);
	if (status != I2C_SUCCESS) return 4;

	status = I2C_Write(0x1B); // Gyroscope configuration register
	if (status != I2C_SUCCESS) return 5;

	status = I2C_Write(0x00); // Set full scale range to ±250 degrees/second
	if (status != I2C_SUCCESS) return 6;

	I2C_Stop();

	return 0; // Success
}

Uint8t MPU6050_Read_Accel(int16_t* ax, int16_t* ay, int16_t* az) {
	Uint8t status;
	Uint8t buffer[6];

	status = I2C_Start(MPU6050_ADDR, I2C_WRITE);
	if (status != I2C_SUCCESS) return 1;

	status = I2C_Write(0x3B); // Starting register for accelerometer data
	if (status != I2C_SUCCESS) return 2;

	I2C_Stop();

	status = I2C_Start(MPU6050_ADDR, I2C_READ);
	if (status != I2C_SUCCESS) return 3;

	for (int i = 0; i < 5; i++) {
		buffer[i] = I2C_Read(I2C_ACK);
	}
	buffer[5] = I2C_ReadNack();

	I2C_Stop();

	*ax = (buffer[0] << 8 | buffer[1]) - (int16_t)(calibration.x_offset * 16384.0);
	*ay = (buffer[2] << 8 | buffer[3]) - (int16_t)(calibration.y_offset * 16384.0);
	*az = (buffer[4] << 8 | buffer[5]) - (int16_t)(calibration.z_offset * 16384.0);

	return 0;
}

Uint8t MPU6050_Read_Gyro(int16_t* gx, int16_t* gy, int16_t* gz) {
	Uint8t status;
	Uint8t buffer[6];

	status = I2C_Start(MPU6050_ADDR, I2C_WRITE);
	if (status != I2C_SUCCESS) return 1;

	status = I2C_Write(0x43); // Starting register for gyroscope data
	if (status != I2C_SUCCESS) return 2;

	I2C_Stop();

	status = I2C_Start(MPU6050_ADDR, I2C_READ);
	if (status != I2C_SUCCESS) return 3;

	for (int i = 0; i < 5; i++) {
		buffer[i] = I2C_Read(I2C_ACK);
	}
	buffer[5] = I2C_ReadNack();

	I2C_Stop();

	*gx = (buffer[0] << 8 | buffer[1]);
	*gy = (buffer[2] << 8 | buffer[3]);
	*gz = (buffer[4] << 8 | buffer[5]);

	return 0;
}
//
void MPU6050_Get_Angles(float* pitch, float* roll) {
	int16_t ax, ay, az;
	MPU6050_Read_Accel(&ax, &ay, &az);
	
	// Convert raw values to g forces
	float accel_x = ax / 16384.0;
	float accel_y = ay / 16384.0;
	float accel_z = az / 16384.0;
	
	// Calculate pitch and roll
	*pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 180.0 / M_PI;
	*roll = atan2(accel_y, accel_z) * 180.0 / M_PI;
}

float MPU6050_Get_Pitch(void) {
	int16_t ax, ay, az;
	MPU6050_Read_Accel(&ax, &ay, &az);
	
	// Convert raw values to g forces
	float accel_x = ax / 16384.0;
	float accel_z = az / 16384.0;
	
	// Calculate pitch
	float pitch = atan2(-accel_x, accel_z) * 180.0 / M_PI;
	
	return pitch;
}

float MPU6050_Get_Roll(void) {
	int16_t ax, ay, az;
	MPU6050_Read_Accel(&ax, &ay, &az);
	
	// Convert raw values to g forces
	float accel_y = ay / 16384.0;
	float accel_z = az / 16384.0;
	
	// Calculate roll
	float roll = atan2(accel_y, accel_z) * 180.0 / M_PI;
	
	return roll;
}
//
void MPU6050_Calibrate(void) {
	int32_t x_sum = 0, y_sum = 0, z_sum = 0;
	float ax, ay, az;
	
	//LCD_Clear();
	//LCD_Write_String((Uint8t*)"Calibrating...");
	
	for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
		MPU6050_Read_Accel(&ax, &ay, &az);
		x_sum += (int32_t)(ax * 16384.0);
		y_sum += (int32_t)(ay * 16384.0);
		z_sum += (int32_t)(az * 16384.0);
		_delay_ms(10);
	}
	
	calibration.x_offset = (float)x_sum / CALIBRATION_SAMPLES / 16384.0;
	calibration.y_offset = (float)y_sum / CALIBRATION_SAMPLES / 16384.0;
	calibration.z_offset = (float)z_sum / CALIBRATION_SAMPLES / 16384.0 - 1.0; // Assuming Z-axis is vertical
	
	//LCD_Clear();
	//LCD_Write_String((Uint8t*)"Calibration done");
	//_delay_ms(1000);
}