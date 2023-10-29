/*
 * pid_controller.c
 *
 *  Created on: Oct 11, 2023
 *      Author: Rijin
 */

#include "PID.h"
#include <math.h>

/* Initialize all calculated controller variables to zero. */
void PIDController_Init(PIDController * controller)
{
	controller->integral = 0.0f;
	controller->derivative = 0.0f;
	controller->prev_error = 0.0f;
	controller->motor_output = 0.0f;
}

/* Calculate all Proportional, Integral, and Derivative values based on
   set pitch angle and IMU reading every loop. */
void PIDController_Update(PIDController *controller, float setpoint, float imu_reading)
{
	// Error from current IMU reading and set pitch angle
	float error = fabs(setpoint - imu_reading);

	// Compute proportional
	float proportional = controller->kp * error;
	controller->proportional = proportional;

	// Compute integral
	controller->integral += error;
	float integral = controller->ki * controller->integral;

	/* Integral Clamping/Windup */
	if (integral < 100) {
	    integral = 100;
	    controller->integral = 100;
	} else if (integral > 200) {
	    integral = 200;
	    controller->integral = 200;
	}

	// Compute derivative
	controller->derivative = (error - controller->prev_error)/((controller->sampling_time)/1000.0f);
	float derivative = controller->kd * controller->derivative;

	float motor_output = fabs(proportional + derivative);

	/* Output Clamping */
    if (motor_output < 600) {
        motor_output = 600;
    } else if (motor_output > 1000) {
        motor_output = 	1000;
    }

	// Set motor output based on the values
	controller->motor_output = motor_output;

	// Store current error as previous
	controller->prev_error = error;
}

