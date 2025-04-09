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
struct peltier_s
{
    TIM_HandleTypeDef *timer;
    TIM_HandleTypeDef *timer_ref;
    uint32_t channel;
    uint32_t channel_ref;
};

void Peltier_Init(TIM_HandleTypeDef *timer, TIM_HandleTypeDef *timer_ref, uint32_t ch, uint32_t ch_ref);

void Peltier_SetCycle(float ratio);

#endif // __PELTIER_H
