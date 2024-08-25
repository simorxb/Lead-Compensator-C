#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define LENGTH 200
#define TIME_STEP 0.1

struct Lead_Compensator
{
    const float kl;             // Lead compensator gain
    const float tau_p;          // Pole time constant
    const float tau_z;          // Zero time constant
    const float T;              // Time step
    const float max;            // Max command
    const float min;            // Min command
    const float max_rate;       // Max rate of change of the command
    float error_prev;           // Previous error
    float command_prev;         // Previous command
    float command_sat_prev;     // Previous saturated command
};

struct Object
{
    const float m;              // Mass of the object
    const float k;              // Damping constant
    const float F_max;          // Max force applied to the object
    const float F_min;          // Min force applied to the object
    const float T;              // Time step
    float v;                    // Velocity of the object
    float z;                    // Position of the object
};

float Lead_Compensator_Step(struct Lead_Compensator *lead_compensator, float measurement, float setpoint)
{
    /* This function implements a lead compensator.
     *
     * Inputs:
     *   measurement: current measurement of the process variable
     *   setpoint: desired value of the process variable
     *   lead_compensator: a pointer to a lead compensator struct containing the controller parameters
     *
     * Returns:
     *   command_sat: the control output of the lead compensator (saturated based on max, min, max_rate)
     */

    float err;
    float command;
    float command_sat;

    /* Error calculation */
    err = setpoint - measurement;

    /* Calculate command using lead compensator equation */
    command = (lead_compensator->kl * (lead_compensator->T * err + lead_compensator->tau_z * (err - lead_compensator->error_prev)) + 
              lead_compensator->tau_p * lead_compensator->command_prev)/(lead_compensator->T + lead_compensator->tau_p);

    /* Update error and output storage for next iteration */
    lead_compensator->error_prev = err;
    lead_compensator->command_prev = command;

    /* Saturate command */
    if (command > lead_compensator->max)
    {
        command_sat = lead_compensator->max;
    }
    else if (command < lead_compensator->min)
    {
        command_sat = lead_compensator->min;
    }
    else
    {
        command_sat = command;
    }

    /* Apply rate limiter */
    if (command_sat > lead_compensator->command_sat_prev + lead_compensator->max_rate * lead_compensator->T)
    {
        command_sat = lead_compensator->command_sat_prev + lead_compensator->max_rate * lead_compensator->T;
    }
    else if (command_sat < lead_compensator->command_sat_prev - lead_compensator->max_rate * lead_compensator->T)
    {
        command_sat = lead_compensator->command_sat_prev - lead_compensator->max_rate * lead_compensator->T;
    }
    else
    {
        /* No action */
    }

    /* Remember saturated command at previous step */
    lead_compensator->command_sat_prev = command_sat;

    return command_sat;
}

float Object_Step(struct Object *obj, float F, float Fd){

    /* This function updates the position of an object in 1D based on the applied force F and
     * the object's mass, viscous damping coefficient k, max/min forces, disturbance force Fd, and time step T.
     *
     * Inputs:
     *   F: the force applied to the object
     *   Fd: the disturbance force
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
    dv_dt = (F_sat - obj->k*obj->v - Fd)/obj->m;

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
    float command = 0;
    float stp = 1;
    float z = 0;

    // State feedback controller initialisation
    struct Lead_Compensator lead_compensator = {0.4, 1.0, 18.0, TIME_STEP, 10, -10, 100, 0, 0, 0};

    // Object parameters for the first control loop
    struct Object obj = {10, 0.5, 10, -10, TIME_STEP, 0, 0};

    // Open a file for logging simulation data
    FILE *file = fopen("data.txt", "w");

    /* Implement iteration using a while loop */
    while(i < LENGTH)
    {
        
        // Execute the first control loop
        command = Lead_Compensator_Step(&lead_compensator, z, stp);
        z = Object_Step(&obj, command, 0);

        // Log the current time and control loop values to the file
        fprintf(file, "%f %f %f %f\n", t, command, z, stp);

        // Increment the time and iteration counter
        t = t + TIME_STEP;
        i = i + 1;
    }

    // Close the file and exit the program
    fclose(file);
    exit(0);
}
