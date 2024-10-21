/*
 * MPU_6050.c
 */

#include "main.h"
#include "printf.h"
#include "MPU_6050.h"

// Structures

typedef struct {
    int16_t ax, ay, az, gx, gy, gz;
} RawData;

typedef struct {
    float ax, ay, az, gx, gy, gz;
} SensorData;

typedef struct {
    float x, y, z;
} AccelOffset;

typedef struct {
    float x, y, z;
} GyroOffset;

typedef struct {
    float r, p, y;
} Attitude;

// Variables

static uint8_t mpu6050_addr;
static float dt_val, tau_val;
static float aScaleFactor, gScaleFactor;

RawData rawData;
SensorData sensorData;
AccelOffset accelOffset;
GyroOffset gyroOffset;
Attitude attitude;

void mpu6050_init(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt)
{
    uint8_t select = 0;

    // Save values

    mpu6050_addr = addr << 1;
    tau_val = tau;
    dt_val = dt;

    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(I2Cx, (mpu6050_addr + 0), 1, I2C_TIMOUT_MS);

    if (ret == HAL_OK)
    {
        printf("The device is OK\n\r");
    }
    else
    {
        printf("The device is not ready \n\r");
    }

    // Quit sleep mode and enable temperature sensor

    select = 0x00;

    ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr + 0), PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);

    if (ret == HAL_OK)
    {
        printf("Out of sleep mode and temp sensor on is OK\n\r");
    }
    else
    {
        printf("Sleep mode and temp sensor error \n\r");
    }

    // Set the full scale ranges

    ret = MPU_writeAccFullScaleRange(I2Cx, aScale);

    if (ret == HAL_OK)
    {
        printf("Acc scale is OK\n\r");
    }
    else
    {
        printf("Acc scale not ready \n\r");
    }

    ret = MPU_writeGyroFullScaleRange(I2Cx, gScale);

    if (ret == HAL_OK)
    {
        printf("Gyro scale is OK\n\r");
    }
    else
    {
        printf("Gyro scale not ready \n\r");
    }

    // Calibrate the accelerometer and the gyroscope

    MPU_calibrateAccel(I2Cx, 100);		// Calibrate with 100 samples
    MPU_calibrateGyro(I2Cx, 100);		// Calibrate with 100 samples
}

HAL_StatusTypeDef MPU_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale)
{
    // Variable init

    uint8_t select;
    HAL_StatusTypeDef ret = HAL_OK;

    // Set the value

    switch (aScale) {
    case AFSR_2G:
        aScaleFactor = 16384.0;
        select = 0x00;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr + 0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_4G:
        aScaleFactor = 8192.0;
        select = 0x08;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr + 0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_8G:
        aScaleFactor = 4096.0;
        select = 0x10;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr + 0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_16G:
        aScaleFactor = 2048.0;
        select = 0x18;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr + 0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        aScaleFactor = 16384.0;
        select = 0x00;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr + 0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
    return ret;
}

HAL_StatusTypeDef MPU_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale)
{
    // Variable init

    uint8_t select;
    HAL_StatusTypeDef ret = HAL_OK;

    // Set the value

    switch (gScale)
    {
    case GFSR_250DPS:
        gScaleFactor = 131.0;
        select = 0x00;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr + 0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_500DPS:
        gScaleFactor = 65.5;
        select = 0x08;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr + 0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_1000DPS:
        gScaleFactor = 32.8;
        select = 0x10;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr + 0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_2000DPS:
        gScaleFactor = 16.4;
        select = 0x18;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr + 0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        gScaleFactor = 131.0;
        select = 0x00;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr + 0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }

    return ret;
}

void MPU_readRawData(I2C_HandleTypeDef *I2Cx)
{
    // Init buffer

    uint8_t buf[14];

    // Subroutine for reading the raw data

    HAL_I2C_Mem_Read(I2Cx, (mpu6050_addr + 1), ACCEL_XOUT_H, 1, buf, 14, I2C_TIMOUT_MS);

    // Bit shift the data

    rawData.ax = buf[0] << 8 | buf[1];
    rawData.ay = buf[2] << 8 | buf[3];
    rawData.az = buf[4] << 8 | buf[5];
    rawData.gx = buf[8] << 8 | buf[9];
    rawData.gy = buf[10] << 8 | buf[11];
    rawData.gz = buf[12] << 8 | buf[13];
}

void MPU_readProcessedData(I2C_HandleTypeDef *I2Cx)
{
    // Get raw values from the IMU

    MPU_readRawData(I2Cx);

    // Compensate for accelerometer offset

    sensorData.ax = (rawData.ax - accelOffset.x) * (9.81 / aScaleFactor);
    sensorData.ay = (rawData.ay - accelOffset.y) * (9.81 / aScaleFactor);
    sensorData.az = (rawData.az - accelOffset.z) * (9.81 / aScaleFactor);

    // Compensate for gyro offset

    sensorData.gx = (rawData.gx - gyroOffset.x) / gScaleFactor;
    sensorData.gy = (rawData.gy - gyroOffset.y) / gScaleFactor;
    sensorData.gz = (rawData.gz - gyroOffset.z) / gScaleFactor;

    printf("X-axis accelerometer is ax,ay,az = [%f, %f, %f]\n\r", sensorData.ax, sensorData.ay, sensorData.az);
    printf("X-axis gyroscope is     gx,gy,gz = [%f, %f, %f]\n\r", sensorData.gx, sensorData.gy, sensorData.gz);
}

void MPU_calibrateAccel(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints)
{
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;

    for (uint16_t i = 0; i < numCalPoints; i++)
    {
        MPU_readRawData(I2Cx);

        sum_ax += rawData.ax;
        sum_ay += rawData.ay;
        sum_az += rawData.az;

        HAL_Delay(10);							// Small delay between readings
    }

    accelOffset.x = (sum_ax / numCalPoints);
    accelOffset.y = (sum_ay / numCalPoints);
    accelOffset.z = (sum_az / numCalPoints);	// Gravity compensation
}

void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints)
{
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (uint16_t i = 0; i < numCalPoints; i++)
    {
        MPU_readRawData(I2Cx);

        sum_gx += rawData.gx;
        sum_gy += rawData.gy;
        sum_gz += rawData.gz;

        HAL_Delay(10); 							// Small delay between readings
    }

    gyroOffset.x = (sum_gx / numCalPoints);
    gyroOffset.y = (sum_gy / numCalPoints);
    gyroOffset.z = (sum_gz / numCalPoints);
}
