#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

HAL_StatusTypeDef MPU6050_Init(void);

HAL_StatusTypeDef MPU6050_Read_Gyro_Z(float *gyro_z_dps);

#endif