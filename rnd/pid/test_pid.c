#include <stdio.h>
#include <math.h>

#include "pid.h"

#define LENGTH 1200
#define TIME_STEP 0.1

struct Object
{
    float m;               // Mass of the object
    float k;               // Damping constant
    float F_max;           // Max force applied to the object
    float F_min;           // Min force applied to the object
    float T;               // Time step
    float v;               // Velocity of the object
    float z;               // Position of the object
};

float Object_Step(struct Object *obj, float F){

    /* This function updates the position of an object in 1D based on the applied force F and
     * the object's mass, viscous damping coefficient k, max/min forces, and time step T.
     *
     * Inputs:
     *   F: the force applied to the object
     *   obj: a pointer to an object struct containing its properties (mass, damping, etc.)
     *
     * Returns:
     *   z: the position of the object in meters
     */

    /* Declare variables for the derivative dv/dt and the saturated force command */
    float dv_dt;
    float F_sat;

    /* Apply saturation to the input force */
    if (F > obj->F_max)
    {
        F_sat = obj->F_max;
    }
    else if (F < obj->F_min)
    {
        F_sat = obj->F_min;
    }
    else
    {
        F_sat = F;
    }

    /* Calculate the derivative dv/dt using the input force and the object's velocity and properties */
    dv_dt = (F_sat - obj->k*obj->v)/obj->m;

    /* Update the velocity and position of the object by integrating the derivative using the time step T */
    obj->v += dv_dt*obj->T;
    obj->z += obj->v*obj->T;

    /* Return the updated position of the object */
    return obj->z;
}

int main()
{
    // Current simulation time
    float t = 0;

    // Iteration counter
    int i = 0;

    // Setpoint and output of the first control loop
    float command1 = 0;
    float stp1 = 100;
    float z1 = 0;

    // Setpoint and output of the second control loop
    float command2 = 0;
    float stp2 = 50;
    float z2 = 0;

    // PID controller parameters for the first control loop
    struct PID pid1 = {1, 0.1, 5, 0.1, 1, TIME_STEP, 100, -100, 40, 0, 0, 0, 0, 0};

    // Object parameters for the first control loop
    struct Object obj1 = {10, 0.5, 100, -100, TIME_STEP, 0, 0};

    // PID controller parameters for the second control loop
    struct PID pid2 = {1.8, 0.3, 7, 0.3, 1, TIME_STEP, 100, -100, 40, 0, 0, 0, 0, 0};

    // Object parameters for the second control loop
    struct Object obj2 = {10, 0.5, 100, -100, TIME_STEP, 0, 0};

    // Open a file for logging simulation data
    FILE *file = fopen("pid_data.csv", "w");

    fprintf(file, "%s,%s,%s,%s,%s,%s,%s\n", "t", "Setpoint 1", "PID Output 1", "Response 1", "Setpoint 2", "PID Output 2", "Response 2");

    /* Implement iteration using a while loop */
    while(i < LENGTH)
    {
        /* Change setpoint at t = 60 seconds */
        if (t < 60)
        {
            stp1 = 100;
            stp2 = 50;
        }
        else
        {
            stp1 = 200;
            stp2 = 150;
        }
        
        // Execute the first control loop
        command1 = PID_step(&pid1, z1, stp1);
        z1 = Object_Step(&obj1, command1);

        // Execute the second control loop
        command2 = PID_step(&pid2, z2, stp2);
        z2 = Object_Step(&obj2, command2);

        // Log the current time and control loop values to the file
        fprintf(file, "%f,%f,%f,%f,%f,%f,%f\n", t, stp1, command1, z1, stp2, command2, z2);

        // Increment the time and iteration counter
        t = t + TIME_STEP;
        i = i + 1;
    }

    // Close the file and exit the program
    fclose(file);
}