#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
// #include "stm32f1xx_hal_tim.h"

typedef struct
{
    uint8_t id;
    TIM_HandleTypeDef *motor_htim;
    TIM_TypeDef  *enc_htim;
    uint8_t forward_ch;
    uint8_t backward_ch;
    volatile int32_t counter;
} MotorController_t;


void MotorController_SetSpeed(const MotorController_t* motor,const float speed);

void MotorController_UpdateEnc(MotorController_t* motor);

int32_t MotorController_GetCounter(const MotorController_t* motor);

#endif //MOTOR_CONTROLLER_H
