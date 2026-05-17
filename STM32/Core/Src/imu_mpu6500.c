#include "imu_mpu6500.h"
#include <math.h>

static void MPU6500_WriteReg(MPU6500_t *dev, uint8_t reg, uint8_t val)
{
    HAL_I2C_Mem_Write(dev->hi2c, dev->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static uint8_t MPU6500_ReadReg(MPU6500_t *dev, uint8_t reg)
{
    uint8_t val = 0;
    HAL_I2C_Mem_Read(dev->hi2c, dev->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    return val;
}

static void MPU6500_ReadRegs(MPU6500_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(dev->hi2c, dev->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

uint8_t MPU6500_CheckID(MPU6500_t *dev)
{
    uint8_t id = MPU6500_ReadReg(dev, MPU_REG_WHOAMI);
    return (id == MPU_WHOAMI_ID) ? 1 : 0;
}

void MPU6500_Reset(MPU6500_t *dev)
{
    MPU6500_WriteReg(dev, MPU_REG_PWR_MGMT1, 0x80);
    HAL_Delay(100);
    MPU6500_WriteReg(dev, MPU_REG_PWR_MGMT1, 0x01);
    HAL_Delay(10);
}

void MPU6500_SetGyroRange(MPU6500_t *dev, uint8_t range)
{
    MPU6500_WriteReg(dev, MPU_REG_GYRO_CFG, range & 0x18);
}

void MPU6500_SetAccelRange(MPU6500_t *dev, uint8_t range)
{
    MPU6500_WriteReg(dev, MPU_REG_ACCEL_CFG, range & 0x18);
}

void MPU6500_SetSampleRate(MPU6500_t *dev, uint16_t rate_hz)
{
    uint8_t divider = (1000 / rate_hz) - 1;
    MPU6500_WriteReg(dev, MPU_REG_SMPLRT, divider);
}

uint8_t MPU6500_Init(MPU6500_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    dev->hi2c = hi2c;
    dev->addr = addr;
    dev->data_ready = 0;
    memset(dev->accel_bias, 0, sizeof(dev->accel_bias));
    memset(dev->gyro_bias, 0, sizeof(dev->gyro_bias));

    MPU6500_Reset(dev);

    if (!MPU6500_CheckID(dev)) {
        return 0;
    }

    MPU6500_WriteReg(dev, MPU_REG_PWR_MGMT1, 0x01);
    MPU6500_WriteReg(dev, MPU_REG_PWR_MGMT2, 0x00);

    MPU6500_SetGyroRange(dev, MPU_GYRO_2000DPS);
    MPU6500_SetAccelRange(dev, MPU_ACCEL_8G);
    MPU6500_SetSampleRate(dev, 1000);

    MPU6500_WriteReg(dev, MPU_REG_CONFIG, 0x03);
    MPU6500_WriteReg(dev, MPU_REG_ACCEL_CFG2, 0x03);

    MPU6500_WriteReg(dev, MPU_REG_INT_PIN, 0x10);
    MPU6500_WriteReg(dev, MPU_REG_INT_EN, 0x01);

    HAL_Delay(100);
    return 1;
}

void MPU6500_ReadAll(MPU6500_t *dev)
{
    uint8_t buf[14];
    MPU6500_ReadRegs(dev, MPU_REG_ACCEL_XH, buf, 14);

    dev->raw_accel[0] = ((int16_t)buf[0] << 8) | buf[1];
    dev->raw_accel[1] = ((int16_t)buf[2] << 8) | buf[3];
    dev->raw_accel[2] = ((int16_t)buf[4] << 8) | buf[5];
    dev->raw_temp = ((int16_t)buf[6] << 8) | buf[7];
    dev->raw_gyro[0] = ((int16_t)buf[8] << 8) | buf[9];
    dev->raw_gyro[1] = ((int16_t)buf[10] << 8) | buf[11];
    dev->raw_gyro[2] = ((int16_t)buf[12] << 8) | buf[13];

    MPU6500_Convert(dev);
    dev->data_ready = 1;
}

void MPU6500_Convert(MPU6500_t *dev)
{
    dev->accel[0] = ((float)dev->raw_accel[0] / 4096.0f) - dev->accel_bias[0];
    dev->accel[1] = ((float)dev->raw_accel[1] / 4096.0f) - dev->accel_bias[1];
    dev->accel[2] = ((float)dev->raw_accel[2] / 4096.0f) - dev->accel_bias[2];

    dev->gyro[0] = ((float)dev->raw_gyro[0] / 16.4f) - dev->gyro_bias[0];
    dev->gyro[1] = ((float)dev->raw_gyro[1] / 16.4f) - dev->gyro_bias[1];
    dev->gyro[2] = ((float)dev->raw_gyro[2] / 16.4f) - dev->gyro_bias[2];

    dev->temperature = ((float)dev->raw_temp / 333.87f) + 21.0f;
}

void MPU6500_Calibrate(MPU6500_t *dev, uint16_t samples)
{
    float sum_accel[3] = {0, 0, 0};
    float sum_gyro[3] = {0, 0, 0};

    for (uint16_t i = 0; i < samples; i++) {
        MPU6500_ReadAll(dev);
        sum_accel[0] += (float)dev->raw_accel[0] / 4096.0f;
        sum_accel[1] += (float)dev->raw_accel[1] / 4096.0f;
        sum_accel[2] += ((float)dev->raw_accel[2] / 4096.0f) - 1.0f;
        sum_gyro[0] += (float)dev->raw_gyro[0] / 16.4f;
        sum_gyro[1] += (float)dev->raw_gyro[1] / 16.4f;
        sum_gyro[2] += (float)dev->raw_gyro[2] / 16.4f;
        HAL_Delay(2);
    }

    dev->accel_bias[0] = sum_accel[0] / samples;
    dev->accel_bias[1] = sum_accel[1] / samples;
    dev->accel_bias[2] = sum_accel[2] / samples;
    dev->gyro_bias[0] = sum_gyro[0] / samples;
    dev->gyro_bias[1] = sum_gyro[1] / samples;
    dev->gyro_bias[2] = sum_gyro[2] / samples;
}
