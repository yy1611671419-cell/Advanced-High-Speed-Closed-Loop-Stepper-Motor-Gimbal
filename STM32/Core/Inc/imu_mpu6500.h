#ifndef __IMU_MPU6500_H
#define __IMU_MPU6500_H

#include "stm32h7xx_hal.h"
#include "protocol.h"

#define MPU_ADDR_LO    0x68
#define MPU_ADDR_HI    0x69

#define MPU_REG_SMPLRT    0x19
#define MPU_REG_CONFIG    0x1A
#define MPU_REG_GYRO_CFG  0x1B
#define MPU_REG_ACCEL_CFG 0x1C
#define MPU_REG_ACCEL_CFG2 0x1D
#define MPU_REG_INT_PIN   0x37
#define MPU_REG_INT_EN    0x38
#define MPU_REG_INT_STAT  0x3A
#define MPU_REG_ACCEL_XH  0x3B
#define MPU_REG_TEMP_H    0x41
#define MPU_REG_GYRO_XH   0x43
#define MPU_REG_PWR_MGMT1 0x6B
#define MPU_REG_PWR_MGMT2 0x6C
#define MPU_REG_WHOAMI    0x75

#define MPU_WHOAMI_ID     0x70

#define MPU_GYRO_250DPS   0x00
#define MPU_GYRO_500DPS   0x08
#define MPU_GYRO_1000DPS  0x10
#define MPU_GYRO_2000DPS  0x18

#define MPU_ACCEL_2G      0x00
#define MPU_ACCEL_4G      0x08
#define MPU_ACCEL_8G      0x10
#define MPU_ACCEL_16G     0x18

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;
    volatile uint8_t data_ready;
    int16_t raw_accel[3];
    int16_t raw_gyro[3];
    int16_t raw_temp;
    float accel[3];
    float gyro[3];
    float temperature;
    float accel_bias[3];
    float gyro_bias[3];
} MPU6500_t;

uint8_t MPU6500_Init(MPU6500_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);
uint8_t MPU6500_CheckID(MPU6500_t *dev);
void MPU6500_Reset(MPU6500_t *dev);
void MPU6500_SetGyroRange(MPU6500_t *dev, uint8_t range);
void MPU6500_SetAccelRange(MPU6500_t *dev, uint8_t range);
void MPU6500_SetSampleRate(MPU6500_t *dev, uint16_t rate_hz);
void MPU6500_ReadAll(MPU6500_t *dev);
void MPU6500_Convert(MPU6500_t *dev);
void MPU6500_Calibrate(MPU6500_t *dev, uint16_t samples);

#endif
