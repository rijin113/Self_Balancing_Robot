/*
 * pid_controller.c
 *
 *  Created on: Oct 11, 2023
 *      Author: Rijin
 */

#include "PID.h"

void PIDController_Init(PIDController * controller)
{
	controller->integral = 0.0f;
	controller->derivative = 0.0f;

	controller->prev_error = 0.0f;

	controller->motor_output = 0.0f;
}

void PIDController_Update(PIDController *controller, float setpoint, float imu_reading)
{
	float error = setpoint - imu_reading;

	float proportional = (controller->kp) * error;

	controller->integral += error;
	float integral = controller->ki * controller->integral;

	controller->derivative = (error - controller->prev_error)/((controller->sampling_time)/1000.0f);
	float derivative = controller->kd * controller->derivative;

	controller->motor_output = proportional + integral + derivative;

	controller->prev_error = error;
}

