/*
 * peltier.c
 *
 *  Created on April 5, 2025
 *  Author: Julien
 *
 */

#include "peltier.h"
/* Global Variables */
struct peltier_s peltier_values;

void Peltier_Init(TIM_HandleTypeDef timer, TIM_HandleTypeDef timer_ref, uint32_t ch, uint32_t ch_ref)
{
	/*
	 * @brief Initializes PWM on desire TIM Channel and on reference TIM Channel
	 *
	 * @param (TIM_HandleTypeDef*) Pointer to the timer handle
	 * @param (TIM_HandleTypeDef*) Pointer to the reference timer handle
	 * @param (uint32_t) Channel used for PWM
	 * @param (uint32_t) Channel used for PWM on reference timer
	 */

	// Save Values
	peltier_values.timer = timer;
	peltier_values.timer_ref = timer_ref;
	peltier_values.channel = ch;
	peltier_values.channel_ref = ch_ref;

	// Initialize PWM on correct Timer and Channel
	HAL_TIM_PWM_Start(&timer_ref, ch_ref);
	HAL_TIM_PWM_Start(&timer, ch);
}

void Peltier_SetCycle(float ratio)
{
	/*
	 * @brief Setting a new duty cycle, from 0-1
	 *
	 * @param (float) Desired Duty Cycle ratio from 0-1. 0 -> 0%, 1 -> 100%
	 */

	// Multiply to correct counter period scale, and inverse value (since it is inversely proportional) ie: 20% Power -> 80/100 Duty Cycle
	int fraction = (peltier_values.timer.Init.Period) - (ratio * peltier_values.timer.Init.Period);

	// Set new duty cycle
	__HAL_TIM_SET_COMPARE(&peltier_values.timer, peltier_values.channel, fraction);
}
