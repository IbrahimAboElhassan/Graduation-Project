/*
 * mpu6050.h
 *
 * Created: 3/26/2024 12:11:54 AM
 *  Author: ibrahim
 */ 


/*#ifndef MPU6050_H_
#define MPU6050_H_

#include "i2c.h"

#define XG_OFFS_TC          0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define YG_OFFS_TC          0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define ZG_OFFS_TC          0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define X_FINE_GAIN         0x03 //[7:0] X_FINE_GAIN
#define Y_FINE_GAIN         0x04 //[7:0] Y_FINE_GAIN
#define Z_FINE_GAIN         0x05 //[7:0] Z_FINE_GAIN
#define XA_OFFS_H           0x06 //[15:0] XA_OFFS
#define XA_OFFS_L_TC        0x07
#define YA_OFFS_H           0x08 //[15:0] YA_OFFS
#define YA_OFFS_L_TC        0x09
#define ZA_OFFS_H           0x0A //[15:0] ZA_OFFS
#define ZA_OFFS_L_TC        0x0B
#define XG_OFFS_USRH        0x13 //[15:0] XG_OFFS_USR
#define XG_OFFS_USRL        0x14
#define YG_OFFS_USRH        0x15 //[15:0] YG_OFFS_USR
#define YG_OFFS_USRL        0x16
#define ZG_OFFS_USRH        0x17 //[15:0] ZG_OFFS_USR
#define ZG_OFFS_USRL        0x18
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define FF_THR              0x1D
#define FF_DUR              0x1E
#define MOT_THR             0x1F
#define MOT_DUR             0x20
#define ZRMOT_THR           0x21
#define ZRMOT_DUR           0x22
#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define DMP_INT_STATUS      0x39
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define MOT_DETECT_STATU    0x61
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C
#define BANK_SEL            0x6D
#define MEM_START_ADDR      0x6E
#define MEM_R_W             0x6F
#define DMP_CFG_1           0x70
#define DMP_CFG_2           0x71
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75

// Pin AD0 connected to ground
#define MPU6050_R           0b11010001
#define MPU6050_W           0b11010000

extern float Acc_x, Acc_y, Acc_z;
extern float Gyro_x, Gyro_y, Gyro_z;

void Gyro_Init(void);
void MPU_Start_Loc();
void Read_RawValue();


#endif /* MPU6050_H_ */

/*#ifndef MPU6050_H_
#define MPU6050_H_

#include "i2c.h"

#define MPU6050_R 0b11010001
#define MPU6050_W 0b11010000

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define INT_ENABLE 0x38

extern float Acc_x, Acc_y, Acc_z;
extern float Gyro_x, Gyro_y, Gyro_z;

void Gyro_Init(void);
void MPU_Start_Loc(void);
void Read_RawValue(void);

#endif /* MPU6050_H_ */


/*#ifndef MPU6050_H_
#define MPU6050_H_

#include "i2c.h"

#define MPU6050_ADDR 0x68

void MPU6050_Init(void);
void MPU6050_Read_Accel(int16_t* ax, int16_t* ay, int16_t* az);

#endif*/

/*#ifndef MPU6050_H_
#define MPU6050_H_

#include "i2c.h"
#include "CPU_CONFIGURATION.h" // Include this to get your type definitions

#define MPU6050_ADDR 0x68
#define CALIBRATION_SAMPLES 100

typedef struct {
	float x_offset;
	float y_offset;
	float z_offset;
} AccelCalibration;

extern AccelCalibration calibration;

Uint8t MPU6050_Init(void);
Uint8t MPU6050_Read_Accel(float* ax, float* ay, float* az);
void MPU6050_Calibrate(void);

#endif*/

#ifndef MPU6050_H_
#define MPU6050_H_

#include "i2c.h"
#include "CPU_CONFIGURATION.h"

#define MPU6050_ADDR 0x68
#define CALIBRATION_SAMPLES 100

typedef struct {
	float x_offset;
	float y_offset;
	float z_offset;
} AccelCalibration;

extern AccelCalibration calibration;

Uint8t MPU6050_Init(void);
Uint8t MPU6050_Read_Accel(int16_t* ax, int16_t* ay, int16_t* az);
Uint8t MPU6050_Read_Gyro(int16_t* gx, int16_t* gy, int16_t* gz);
//
//void MPU6050_Get_Angles(float* pitch, float* roll);

float MPU6050_Get_Pitch(void);
float MPU6050_Get_Roll(void);
//
void MPU6050_Calibrate(void);

#endif