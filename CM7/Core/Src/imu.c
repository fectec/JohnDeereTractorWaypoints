/*
 * imu.c
 */

#include "main.h"
#include "imu.h"
#include "kalman_imu.h"
#include "printf.h"

// Variables

static uint8_t mpu_6050_addr;
static float dt_val, tau_val;
static float aScaleFactor, gScaleFactor;

RawData rawData;
SensorData sensorData;
AccelOffset accelOffset;
GyroOffset gyroOffset;
Attitude attitude;

static KalmanFilter kf_ax, kf_ay, kf_az;
static KalmanFilter kf_gx, kf_gy, kf_gz;

void IMU_SetupRoutine(I2C_HandleTypeDef *I2Cx)
{
	MPU_6050_Init(I2Cx, AD0_LOW, AFSR_2G, GFSR_250DPS, 0.98, 0.004);

    // Initialize Kalman filters

	// Slightly more process noise, higher measurement noise

    Kalman_Init(&kf_ax, 0.05f, 0.5f, 0.0f);
    Kalman_Init(&kf_ay, 0.05f, 0.5f, 0.0f);
    Kalman_Init(&kf_az, 0.05f, 0.5f, 0.0f);

    // Lower process noise for gyro, moderate measurement noise

    Kalman_Init(&kf_gx, 0.01f, 0.3f, 0.0f);
    Kalman_Init(&kf_gy, 0.01f, 0.3f, 0.0f);
    Kalman_Init(&kf_gz, 0.01f, 0.3f, 0.0f);
}

void MPU_6050_Init(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt)
{
    uint8_t select = 0;

    // Save values

    mpu_6050_addr = addr << 1;
    tau_val = tau;
    dt_val = dt;

    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(I2Cx, (mpu_6050_addr + 0), 1, I2C_TIMOUT_MS);

    if (ret == HAL_OK) {
        printf("The device is OK\n\r");
    }
    else {
        printf("The device is not ready\n\r");
    }

    // Quit sleep mode and enable temperature sensor

    select = 0x00;

    ret = HAL_I2C_Mem_Write(I2Cx, (mpu_6050_addr + 0), PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);

    if (ret == HAL_OK) {
        printf("Out of sleep mode and temperature sensor on is OK\n\r");
    }
    else {
        printf("Sleep mode and temperature sensor error\n\r");
    }

    // Set the full scale ranges

    ret = MPU_WriteAccFullScaleRange(I2Cx, aScale);

    if (ret == HAL_OK) {
        printf("Accelerometer scale is OK\n\r");
    }
    else {
        printf("Accelerometer scale not ready\n\r");
    }

    ret = MPU_WriteGyroFullScaleRange(I2Cx, gScale);

    if (ret == HAL_OK) {
        printf("Gyroscope scale is OK\n\r");
    }
    else {
        printf("Gyroscope scale not ready\n\r");
    }

    // Calibrate the accelerometer and the gyroscope

    MPU_CalibrateAccel(I2Cx, 100);		// Calibrate with 100 samples
    MPU_CalibrateGyro(I2Cx, 100);		// Calibrate with 100 samples
}

HAL_StatusTypeDef MPU_WriteAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale)
{
    // Variable initialization

    uint8_t select;
    HAL_StatusTypeDef ret = HAL_OK;

    // Set the value

    switch (aScale) {

		case AFSR_2G:
			aScaleFactor = 16384.0;
			select = 0x00;
			ret = HAL_I2C_Mem_Write(I2Cx, (mpu_6050_addr + 0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
			break;
		case AFSR_4G:
			aScaleFactor = 8192.0;
			select = 0x08;
			ret = HAL_I2C_Mem_Write(I2Cx, (mpu_6050_addr + 0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
			break;
		case AFSR_8G:
			aScaleFactor = 4096.0;
			select = 0x10;
			ret = HAL_I2C_Mem_Write(I2Cx, (mpu_6050_addr + 0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
			break;
		case AFSR_16G:
			aScaleFactor = 2048.0;
			select = 0x18;
			ret = HAL_I2C_Mem_Write(I2Cx, (mpu_6050_addr + 0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
			break;
		default:
			aScaleFactor = 16384.0;
			select = 0x00;
			ret = HAL_I2C_Mem_Write(I2Cx, (mpu_6050_addr + 0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
			break;
    }

    return ret;
}

HAL_StatusTypeDef MPU_WriteGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale)
{
    // Variable initialization

    uint8_t select;
    HAL_StatusTypeDef ret = HAL_OK;

    // Set the value

    switch (gScale) {
		case GFSR_250DPS:
			gScaleFactor = 131.0;
			select = 0x00;
			ret = HAL_I2C_Mem_Write(I2Cx, (mpu_6050_addr + 0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
			break;
		case GFSR_500DPS:
			gScaleFactor = 65.5;
			select = 0x08;
			ret = HAL_I2C_Mem_Write(I2Cx, (mpu_6050_addr + 0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
			break;
		case GFSR_1000DPS:
			gScaleFactor = 32.8;
			select = 0x10;
			ret = HAL_I2C_Mem_Write(I2Cx, (mpu_6050_addr + 0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
			break;
		case GFSR_2000DPS:
			gScaleFactor = 16.4;
			select = 0x18;
			ret = HAL_I2C_Mem_Write(I2Cx, (mpu_6050_addr + 0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
			break;
		default:
			gScaleFactor = 131.0;
			select = 0x00;
			ret = HAL_I2C_Mem_Write(I2Cx, (mpu_6050_addr + 0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
			break;
    }

    return ret;
}

void MPU_ReadRawData(I2C_HandleTypeDef *I2Cx)
{
    // Initialization buffer

    uint8_t buf[14];

    // Subroutine for reading the raw data

    HAL_I2C_Mem_Read(I2Cx, (mpu_6050_addr + 1), ACCEL_XOUT_H, 1, buf, 14, I2C_TIMOUT_MS);

    // Bit shift the data

    rawData.ax = buf[0] << 8 | buf[1];
    rawData.ay = buf[2] << 8 | buf[3];
    rawData.az = buf[4] << 8 | buf[5];
    rawData.gx = buf[8] << 8 | buf[9];
    rawData.gy = buf[10] << 8 | buf[11];
    rawData.gz = buf[12] << 8 | buf[13];
}

SensorData MPU_ReadProcessedData(I2C_HandleTypeDef *I2Cx)
{
    // Get raw values from the IMU

    MPU_ReadRawData(I2Cx);

    // Compensate for accelerometer offset

    sensorData.ax = (rawData.ax - accelOffset.x) * (9.81 / aScaleFactor);
    sensorData.ay = (rawData.ay - accelOffset.y) * (9.81 / aScaleFactor);
    sensorData.az = (rawData.az - accelOffset.z) * (9.81 / aScaleFactor);

    // Compensate for gyroscope offset

    sensorData.gx = (rawData.gx - gyroOffset.x) / gScaleFactor;
    sensorData.gy = (rawData.gy - gyroOffset.y) / gScaleFactor;
    sensorData.gz = (rawData.gz - gyroOffset.z) / gScaleFactor;

    // Apply Kalman filter to each axis

    sensorData.ax = Kalman_Update(&kf_ax, sensorData.ax);
    sensorData.ay = Kalman_Update(&kf_ay, sensorData.ay);
    sensorData.az = Kalman_Update(&kf_az, sensorData.az);

    sensorData.gx = Kalman_Update(&kf_gx, sensorData.gx);
    sensorData.gy = Kalman_Update(&kf_gy, sensorData.gy);
    sensorData.gz = Kalman_Update(&kf_gz, sensorData.gz);

    // DEBUG:

    // printf("ax, ay, az = [%f, %f, %f]\n\r", sensorData.ax, sensorData.ay, sensorData.az);
    // printf("gx, gy, gz = [%f, %f, %f]\n\r", sensorData.gx, sensorData.gy, sensorData.gz);

    return sensorData;
}

void MPU_CalibrateAccel(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints)
{
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;

    for (uint16_t i = 0; i < numCalPoints; i++)
    {
        MPU_ReadRawData(I2Cx);

        sum_ax += rawData.ax;
        sum_ay += rawData.ay;
        sum_az += rawData.az;

        HAL_Delay(10);							// Small delay between readings
    }

    accelOffset.x = (sum_ax / numCalPoints);
    accelOffset.y = (sum_ay / numCalPoints);
    accelOffset.z = (sum_az / numCalPoints);	// Gravity compensation
}

void MPU_CalibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints)
{
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (uint16_t i = 0; i < numCalPoints; i++)
    {
        MPU_ReadRawData(I2Cx);

        sum_gx += rawData.gx;
        sum_gy += rawData.gy;
        sum_gz += rawData.gz;

        HAL_Delay(10); 							// Small delay between readings
    }

    gyroOffset.x = (sum_gx / numCalPoints);
    gyroOffset.y = (sum_gy / numCalPoints);
    gyroOffset.z = (sum_gz / numCalPoints);
}
