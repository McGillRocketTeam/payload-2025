/**
 * @file pid.c
 * @brief PID controller implementation.
 * Based on Simone Bertoni's PID controller implementation.
 * @author Simone Bertoni, Akash Shah
 */

#include "pid.h"

float PID_step(struct PID *pid, float measurement, float setpoint, uint32_t t1)
{
    float err;
    float command;
    float command_sat;
    float deriv_filt;
    float dt;

    /* Calculate time step */
    dt = (t1 - pid->t_0) / 1000.0f; // Convert milliseconds to seconds
    // Skip this step if no time has passed to avoid division by zero
    if (dt == 0)
    {
    	// Return the previous command
    	return pid->command_sat_prev;
    }
    pid->t_0 = t1; // Update previous time

    /* Error calculation */
    err = setpoint - measurement;

    /* Integral term calculation - including anti-windup */
    pid->integral += pid->Ki * err * dt + pid->Kaw * (pid->command_sat_prev - pid->command_prev) * dt;

    /* Derivative term calculation using filtered derivative method */
    deriv_filt = (err - pid->err_prev + pid->T_C * pid->deriv_prev) / (dt + pid->T_C);
    pid->err_prev = err;
    pid->deriv_prev = deriv_filt;

    /* Summing the 3 terms */
    command = pid->Kp * err + pid->integral + pid->Kd * deriv_filt;

    /* Remember command at previous step */
    pid->command_prev = command;

    /* Saturate command */
    if (command > pid->max)
    {
        command_sat = pid->max;
    }
    else if (command < pid->min)
    {
        command_sat = pid->min;
    }
    else
    {
        command_sat = command;
    }

    /* Apply rate limiter */
    if (command_sat > pid->command_sat_prev + pid->max_rate * dt)
    {
        command_sat = pid->command_sat_prev + pid->max_rate * dt;
    }
    else if (command_sat < pid->command_sat_prev - pid->max_rate * dt)
    {
        command_sat = pid->command_sat_prev - pid->max_rate * dt;
    }
    else
    {
        /* No action */
    }

    /* Remember saturated command at previous step */
    pid->command_sat_prev = command_sat;

    return command_sat;
}
