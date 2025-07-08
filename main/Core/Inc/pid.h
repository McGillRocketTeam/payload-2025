/**
 * @file pid.h
 * @brief Header file for PID controller implementation. 
 * Based on Simone Bertoni's PID controller implementation.
 * @author Simone Bertoni, Akash Shah
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32f446xx.h"

struct PID
{
    float Kp;               // Proportional gain constant
    float Ki;               // Integral gain constant
    float Kd;               // Derivative gain constant
    float Kaw;              // Anti-windup gain constant
    float T_C;              // Time constant for derivative filtering
    float max;              // Max command
    float min;              // Min command
    float max_rate;         // Max rate of change of the command
    float integral;         // Integral term
    float t_0;              // Previous time
    float err_prev;         // Previous error
    float deriv_prev;       // Previous derivative
    float command_sat_prev; // Previous saturated command
    float command_prev;     // Previous command
};

/**
 * @brief Step function for a PID controller.
 * This function calculates the control output based on the current measurement, setpoint, and time.
 * @param pid Pointer to the PID struct containing the controller parameters.
 * @param value Current measurement of the process variable.
 * @param setpoint Desired value of the process variable.
 * @param t1 Current time in milliseconds.
 * @return `command_sat`: the control output of the PID controller (saturated based on max. min, max_rate)
 */
float PID_step(struct PID *pid, float value, float setpoint, uint32_t t1);

#endif /* INC_PID_H_ */
