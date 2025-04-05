/*
 * peltier.c
 *
 *  Created on April 5, 2025
 *  Author: Julien
 *
 */
TIM_HandleTypeDef *htim;  // CHosen Timer
uint32_t channel; // Chosen Channel (1-4)
int counter_period; // htim.Init.Period in main.c


void Peltier_Init(int period, TIM_HandleTypeDef *timer, uint32_t ch){
	/*
	 * @brief Initializes PWM on desire TIM Channel
	 *
	 * @param (int) period - Counter Period
	 * @param (TIM_HandleTypeDef *) timer - Chosen Timer
	 * @param (uint32_t) ch - Chosen Channel
	 */

	// Save Values
    htim = timer;
    channel = ch;
    counter_period = period;

    // Initialize PWM on correct Timer and Channel
    HAL_TIM_PWM_Start(&htim, channel);
}

void Peltier_SetCycle(float ratio){
	/*
	 * @brief Setting a new duty cycle, from 0-1
	 *
	 * @param (float) Desired Duty Cycle ratio from 0-1. 0 -> 0%, 1 -> 100%
	 */

	// Multiply to correct counter period scale, and inverse value (since it is inversely proportional) ie: 20% Power -> 80/100 Duty Cycle
	int fraction = (counter_period - (ratio * counter_period));

	// Set new duty cycle
	__HAL_TIM_SET_COMPARE(&htim, channel, fraction);
}
