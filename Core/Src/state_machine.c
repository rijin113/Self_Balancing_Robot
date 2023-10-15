/*
 * state_machine.c
 *
 *  Created on: Oct. 15, 2023
 *      Author: Rijin
 */

#include "state_machine.h"
#include "main.h"
#include "math.h"

//extern TIM_HandleTypeDef htim3;
//int toggle = 0;

// Starting State
Robot_State robot_idle_state(PIDController * controller)
{
	// wait for a button press to change into Start state
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, 0);
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, 0);

//	if(1)
//	{
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, fabs(controller->motor_output*100.0f));
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, fabs(controller->motor_output*100.0f));
//		return ROBOT_START;
//	}
//	return ROBOT_IDLE;
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, fabs(controller->motor_output*100.0f));
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, fabs(controller->motor_output*100.0f));
//	return ROBOT_START;
}

//Robot_State robot_start_state(accel_data * accel)
//{
//	if((accel->pitch_angle) < 0)
//	{
//		return ROBOT_BACKWARD;
//	}
//	else if((accel->pitch_angle) > 0)
//	{
//		return ROBOT_FORWARD;
//	}
//	return ROBOT_START;
//}
//
//Robot_State robot_forward_state(accel_data * accel)
//{
//	// Drive backwards to counter the change
//	if(accel->pitch_angle > 0 && toggle == 1)
//	{
//		// Left Motor
//		HAL_GPIO_TogglePin(GPIOA, PA9_D8_OUT_Pin);
//		HAL_GPIO_TogglePin(PC7_D9_OUT_GPIO_Port, PC7_D9_OUT_Pin);
//
//		// Right Motor
//		HAL_GPIO_TogglePin(GPIOA, PA8_D7_OUT_Pin);
//		HAL_GPIO_TogglePin(PB10_D6_OUT_GPIO_Port, PB10_D6_OUT_Pin);
//		toggle = 0;
//	}
//
//	if(accel->pitch_angle < 0)
//	{
//		return ROBOT_BACKWARD;
//	}
//
////	if (1)
////	{
////		return ROBOT_STOP;
////	}
//	return ROBOT_FORWARD;
//}
//
//Robot_State robot_backward_state(accel_data * accel)
//{
//	// Drive forward to counter the change
//	if(accel->pitch_angle > 0 && toggle == 1)
//	{
//		// Left Motor
//		HAL_GPIO_TogglePin(GPIOA, PA9_D8_OUT_Pin);
//		HAL_GPIO_TogglePin(PC7_D9_OUT_GPIO_Port, PC7_D9_OUT_Pin);
//
//		// Right Motor
//		HAL_GPIO_TogglePin(GPIOA, PA8_D7_OUT_Pin);
//		HAL_GPIO_TogglePin(PB10_D6_OUT_GPIO_Port, PB10_D6_OUT_Pin);
//		toggle = 0;
//	}
//
//	if(accel->pitch_angle > 0)
//	{
//		return ROBOT_FORWARD;
//	}
//
////	if (1)
////	{
////		return ROBOT_STOP;
////	}
//
//	return ROBOT_BACKWARD;
//}
//
//Robot_State robot_stopped_state()
//{
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, 0);
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, 0);
//	return ROBOT_IDLE;
//}
//
///* Optional State Implementation */
//Robot_State robot_balanced_state()
//{
//	return ROBOT_BALANCED;
//}

