struct PID
{
    float Kp;               // Proportional gain constant
    float Ki;               // Integral gain constant
    float Kd;               // Derivative gain constant
    float Kaw;              // Anti-windup gain constant
    float T_C;              // Time constant for derivative filtering
    float T;                // Time step
    float max;              // Max command
    float min;              // Min command
    float max_rate;         // Max rate of change of the command
    float integral;         // Integral term
    float err_prev;         // Previous error
    float deriv_prev;       // Previous derivative
    float command_sat_prev; // Previous saturated command
    float command_prev;     // Previous command
};

float PID_step(struct PID *pid, float value, float setpoint);