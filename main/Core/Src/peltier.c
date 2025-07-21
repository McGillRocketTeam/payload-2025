/**
 * peltier.c
 *
 *  Created on April 5, 2025
 *  Author: Julien
 *
 */

#include "peltier.h"
#include "enabled.h"

/* Global Variables */

/**
 * @brief Initializes PWM on desire TIM Channel and on reference TIM Channel
 *
 * @param (PL_Peltier_Handler*) Pointer to Peltier cooler handle
 * @param (TIM_HandleTypeDef*) Pointer to the timer handle
 * @param (uint32_t) Channel used for PWM
 */

bool PL_Peltier_Init(PL_Peltier_Handler *peltier, TIM_HandleTypeDef *timer, TIM_HandleTypeDef *timer_ref, uint32_t ch, uint32_t ch_ref)
{
#ifdef PELTIER_ENABLED
	// Save Values
	peltier->timer = timer;
	peltier->channel = ch;

	// Initialize PWM on correct Timer and Channel
	HAL_StatusTypeDef ref_started = HAL_TIM_PWM_Start(timer_ref, ch_ref);
	HAL_StatusTypeDef started = HAL_TIM_PWM_Start(timer, ch);
	__HAL_TIM_SET_COMPARE(timer_ref, ch_ref, timer_ref->Init.Period);
	return ref_started == HAL_OK && started == HAL_OK;
#else  // PELTIER_ENABLED
	return true;
#endif // PELTIER_ENABLED
}

/**
 * @brief Setting a new duty cycle, from 0-1
 *
 * @param (PL_Peltier_Handler*) Pointer to Peltier cooler handle
 * @param (float) Desired Duty Cycle ratio from 0-1. 0 -> 0%, 1 -> 100%
 */
void PL_Peltier_SetCycle(PL_Peltier_Handler *peltier, float ratio)
{
#ifdef PELTIER_ENABLED
	if (ratio < 0)
	{
		ratio = 0;
	}
	else if (ratio > 1)
	{
		ratio = 1;
	}
	// Multiply to correct counter period scale, and inverse value (since it is inversely proportional) ie: 20% Power -> 80/100 Duty Cycle
	int fraction = (int)((1 - ratio) * peltier->timer->Init.Period);

	// Set new duty cycle
	__HAL_TIM_SET_COMPARE(peltier->timer, peltier->channel, fraction);
#endif // PELTIER_ENABLED
}
