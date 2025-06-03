/**
 * @file peltier.h
 * @author julien
 * @date 2025-04-05
 * @brief peltier stuff
 */

#ifndef __PELTIER_H
#define __PELTIER_H

#include <stdint.h>
#include "stm32f4xx.h"
// Peltier info structure
typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t channel;
} PL_Peltier_Handler;

void PL_Peltier_Init(PL_Peltier_Handler *peltier, TIM_HandleTypeDef *timer, TIM_HandleTypeDef *timer_ref, uint32_t ch, uint32_t ch_ref);

void PL_Peltier_SetCycle(PL_Peltier_Handler *peltier, float ratio);

#endif // __PELTIER_H
