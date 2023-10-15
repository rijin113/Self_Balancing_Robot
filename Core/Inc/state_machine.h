/*
 * Header file for State Machine
 *
 *  Created on: October 15, 2023
 *      Author: Rijin
 */

#include <stdio.h>
#include "mpu6050.h"
#include "pid.h"

typedef enum
{
	ROBOT_IDLE = 0,
	ROBOT_START,
	ROBOT_FORWARD,
	ROBOT_BACKWARD,
	ROBOT_BALANCED,
	ROBOT_STOP
}Robot_State;

Robot_State robot_idle_state();
Robot_State robot_start_state(accel_data * accel);
Robot_State robot_forward_state(accel_data * accel);
Robot_State robot_backward_state(accel_data * accel);
Robot_State robot_stopped_state();
/* Optional State */
Robot_State robot_balanced_state();
