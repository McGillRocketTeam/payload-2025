/*
 * blink.h
 *
 *  Created on: Jun 8, 2025
 *      Author: akash
 */

#ifndef INC_BLINK_H_
#define INC_BLINK_H_

#include <stdbool.h>
#include "stm32f4xx.h"

#define INIT_BLINK_TIME 5000 // milliseconds
#define BLINK_COUNTER_PERIOD 10

typedef struct {
    TIM_HandleTypeDef *htim;   // Pointer to the timer handle
    GPIO_TypeDef *port;  // Pointer to the GPIO port
    uint16_t pin;        // GPIO pin number
    uint8_t count; // Counter for the number of blinks
} PL_Blink_Handler;

/**
 * @brief Initializes the blinking handler.
 * @param blink Pointer to the `PL_Blink_Handler` structure.
 * @param htim Pointer to the timer instance used trigger blinking
 * @param port Pointer to the GPIO port where the LED is connected.
 * @param pin GPIO pin number where the LED is connected.
 */
void PL_Blink_Init(PL_Blink_Handler *blink, TIM_HandleTypeDef *htim, GPIO_TypeDef *port, uint16_t pin);
/**
 * @brief Starts the blinking routine by starting the timer.
 * @param blink Pointer to the `PL_Blink_Handler` structure.
 * @return true if the timer was started successfully, false otherwise.
 */
bool PL_Blink_Start(PL_Blink_Handler *blink);
/**
 * @brief Stops the blinking routine by stopping the timer.
 * @param blink Pointer to the `PL_Blink_Handler` structure.
 */
bool PL_Blink_Stop(PL_Blink_Handler *blink);
/**
 * @brief Toggles the blinking state of the LED, taking into account whether the final build initialization time has passed.
 * @param blink Pointer to the `PL_Blink_Handler` structure.
 * @return true if the LED was actually toggled successfully, false if skipped.
 * @note This function should be called periodically, typically in a timer interrupt.
 */
bool PL_Blink_Toggle(PL_Blink_Handler *blink);

#endif /* INC_BLINK_H_ */
