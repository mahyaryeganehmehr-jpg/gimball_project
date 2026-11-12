#include "servolib.h"

HAL_StatusTypeDef Servo_Init(servo_t *servo,
                             TIM_HandleTypeDef *htim,
                             uint32_t channel)
{
    if (servo == NULL || htim == NULL) return HAL_ERROR;

    servo->htim      = htim;
    servo->channel   = channel;
    servo->min_angle = 0.0f;
    servo->max_angle = 180.0f;
    servo->min_pulse = 50;
    servo->max_pulse = 2300;

    return HAL_TIM_PWM_Start(htim, channel);
}

void Servo_SetAngle(servo_t *servo, float angle)
{
    if (angle < servo->min_angle) angle = servo->min_angle;
    if (angle > servo->max_angle) angle = servo->max_angle;

    float pulse = servo->min_pulse +
                  (angle - servo->min_angle) *
                  (servo->max_pulse - servo->min_pulse) /
                  (servo->max_angle - servo->min_angle);

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, (uint16_t)pulse);
}