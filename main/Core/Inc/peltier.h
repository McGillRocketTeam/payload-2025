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

void Peltier_Init(int period, TIM_HandleTypeDef *timer, uint32_t ch);

void Peltier_SetCycle(float ratio);


#endif // __PELTIER_H
