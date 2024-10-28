/*
 * MPU_6050_IMU.h
 */

#ifndef INC_MPU_6050_IMU_H_
#define INC_MPU_6050_IMU_H_

// Constants

#define RAD2DEG				57.2957795131

// Defines

#define WHO_AM_I_6050_ANS	0x68
#define WHO_AM_I			0x75
#define AD0_LOW				0x68
#define AD0_HIGH			0x69
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C
#define PWR_MGMT_1			0x6B
#define ACCEL_XOUT_H		0x3B
#define I2C_TIMOUT_MS		1000

// Full scale ranges

enum gyroscopeFullScaleRange {
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange {
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

// Function prototypes

void MPU_6050_Init(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt);
void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints);
void MPU_calibrateAccel(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints);
void MPU_calcAttitude(I2C_HandleTypeDef *I2Cx);
void MPU_readRawData(I2C_HandleTypeDef *I2Cx);
void MPU_readProcessedData(I2C_HandleTypeDef *I2Cx);
HAL_StatusTypeDef MPU_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale);
HAL_StatusTypeDef MPU_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale);

#endif /* INC_MPU_6050_IMU_H_ */
