#ifndef SERVOLIB_H
#define SERVOLIB_H

#include "main.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;

    float min_angle;
    float max_angle;

    uint16_t min_pulse;
    uint16_t max_pulse;

} servo_t;


HAL_StatusTypeDef Servo_Init(servo_t *servo,
                             TIM_HandleTypeDef *htim,
                             uint32_t channel);

void Servo_SetAngle(servo_t *servo, float angle);

#endif