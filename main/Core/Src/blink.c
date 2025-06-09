/*
 * blink.c
 *
 *  Created on: Jun 8, 2025
 *      Author: akash
 */

#include "blink.h"
#include "enabled.h"

void blink_slow(PL_Blink_Handler *blink);

void PL_Blink_Init(PL_Blink_Handler *blink, TIM_HandleTypeDef *htim, GPIO_TypeDef *port, uint16_t pin)
{
#if BLINK_ENABLED
    blink->htim = htim;
    blink->port = port;
    blink->pin = pin;
    blink->count = 0;
#endif
}

bool PL_Blink_Start(PL_Blink_Handler *blink)
{
#if BLINK_ENABLED
    // Reset the blink count
    blink->count = 0; 
    // Start the timer for blinking
    return HAL_TIM_Base_Start_IT(blink->htim) == HAL_OK;
#else
    return true;
#endif
}

bool PL_Blink_Stop(PL_Blink_Handler *blink)
{
#if BLINK_ENABLED
    // Reset the blink count
    blink->count = 0;
    // Ensure the LED is turned off
    HAL_GPIO_WritePin(blink->port, blink->pin, GPIO_PIN_RESET);
    // Stop the timer for blinking
    return HAL_TIM_Base_Stop_IT(blink->htim) == HAL_OK;
#else
    return true;
#endif
}

void PL_Blink_Toggle(PL_Blink_Handler *blink)
{
#if BLINK_ENABLED
#if FINAL_BUILD
    if (HAL_GetTick() <= INIT_BLINK_TIME)
    {
    	// Blink fast (every time this function is called)
        HAL_GPIO_TogglePin(blink->port, blink->pin);
    }
    else
    {
    	blink_slow(blink);
    }
#else
    blink_slow(blink);
#endif
#endif
}

void blink_slow(PL_Blink_Handler *blink)
{
#if BLINK_ENABLED
	// Only blink the LED every certain number of times the timer interrupt is triggered
    if (blink->count == 0)
    {
        HAL_GPIO_TogglePin(blink->port, blink->pin);
    }
    blink->count++;
    blink->count %= BLINK_COUNTER_PERIOD;
#endif
}
