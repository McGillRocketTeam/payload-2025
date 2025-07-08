#include "pid.h"

float PID_step(struct PID *pid, float measurement, float setpoint)
{
    /* This function implements a PID controller.
     *
     * Inputs:
     *   measurement: current measurement of the process variable
     *   setpoint: desired value of the process variable
     *   pid: a pointer to a PID struct containing the controller parameters
     *
     * Returns:
     *   command_sat: the control output of the PID controller (saturated based on max. min, max_rate)
     */

    float err;
    float command;
    float command_sat;
    float deriv_filt;

    /* Error calculation */
    err = setpoint - measurement;

    /* Integral term calculation - including anti-windup */
    pid->integral += pid->Ki*err*pid->T + pid->Kaw*(pid->command_sat_prev - pid->command_prev)*pid->T;
    
    /* Derivative term calculation using filtered derivative method */
    deriv_filt = (err - pid->err_prev + pid->T_C*pid->deriv_prev)/(pid->T + pid->T_C);
    pid->err_prev = err;
    pid->deriv_prev = deriv_filt;

    /* Summing the 3 terms */
    command = pid->Kp*err + pid->integral + pid->Kd*deriv_filt;

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
    if (command_sat > pid->command_sat_prev + pid->max_rate*pid->T)
    {
        command_sat = pid->command_sat_prev + pid->max_rate*pid->T;
    }
    else if (command_sat < pid->command_sat_prev - pid->max_rate*pid->T)
    {
        command_sat = pid->command_sat_prev - pid->max_rate*pid->T;
    }
    else
    {
        /* No action */
    }

    /* Remember saturated command at previous step */
    pid->command_sat_prev = command_sat;

    return command_sat;
}