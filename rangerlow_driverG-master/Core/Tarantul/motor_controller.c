//
// Created by glebt on 17.05.2025.
//

#include"motor_controller.h"

#include <math.h>

uint8_t map_float_to_uint8(float input) {
    if (input < -1.0f) input = -1.0f;
    if (input > 1.0f) input = 1.0f;

    return (uint8_t)((input + 1.0f) * 127.5f);
}

void MotorController_SetSpeed(const MotorController_t* motor,const float speed)
{
    const uint16_t pwm = (uint16_t)fabsf(speed*65535);

    if (speed > 0.0f)
    {
        __HAL_TIM_SET_COMPARE(motor->motor_htim, motor->forward_ch, pwm);
        __HAL_TIM_SET_COMPARE(motor->motor_htim, motor->backward_ch, 0);
    }
    else if (speed < 0.0f)
    {
        __HAL_TIM_SET_COMPARE(motor->motor_htim, motor->forward_ch, 0);
        __HAL_TIM_SET_COMPARE(motor->motor_htim, motor->backward_ch, pwm);
    } else
    {
        __HAL_TIM_SET_COMPARE(motor->motor_htim, motor->forward_ch, 0);
        __HAL_TIM_SET_COMPARE(motor->motor_htim, motor->backward_ch, 0);
    }
}

void MotorController_UpdateEnc(MotorController_t* motor)
{
    if (motor == NULL || motor->enc_htim == NULL)
    {
        return;
    }
    motor->counter = (int32_t) motor->enc_htim->CNT;
}

int32_t MotorController_GetCounter(const MotorController_t* motor)
{

    return motor->counter;
}