#include "mpu6050.h"

extern I2C_HandleTypeDef hi2c1;

/* MPU6050 Configuration */

#define MPU6050_ADDR       (0x68 << 1)

#define PWR_MGMT_1         0x6B
#define GYRO_CONFIG        0x1B
#define GYRO_ZOUT_H        0x47


HAL_StatusTypeDef MPU6050_Init(void)
{
    uint8_t data;

    /* Wake up MPU6050 */
    data = 0x00;

    if (HAL_I2C_Mem_Write(&hi2c1,
                          MPU6050_ADDR,
                          PWR_MGMT_1,
                          1,
                          &data,
                          1,
                          100) != HAL_OK)
        return HAL_ERROR;

    /* Set gyro range to +-250 deg/s */
    data = 0x00;

    if (HAL_I2C_Mem_Write(&hi2c1,
                          MPU6050_ADDR,
                          GYRO_CONFIG,
                          1,
                          &data,
                          1,
                          100) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}


HAL_StatusTypeDef MPU6050_Read_Gyro_Z(float *gyro_z_dps)
{
    uint8_t buffer[2];

    int16_t raw;

    if (HAL_I2C_Mem_Read(&hi2c1,
                         MPU6050_ADDR,
                         GYRO_ZOUT_H,
                         1,
                         buffer,
                         2,
                         100) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Combine two bytes */
    raw = (int16_t)((buffer[0] << 8) | buffer[1]);

    /* Convert raw value to deg/s */
    *gyro_z_dps = (float)raw / 131.0f;

    return HAL_OK;
}