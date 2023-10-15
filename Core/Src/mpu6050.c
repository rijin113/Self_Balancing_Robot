/*
 * Source file for MPU6050 Configuration
 *
 *  Created on: Sep. 28, 2023
 *      Author: Rijin
 */

#include "mpu6050.h"
#include "main.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

void mpu6050_init()
{
  uint8_t set_gyro_range = GYRO_RANGE_500;
  uint8_t set_accel_range = ACCEL_RANGE_4G;
  uint8_t set_sleep_mode = SLEEP_MODE_OFF;

  HAL_StatusTypeDef device_ready = HAL_I2C_IsDeviceReady(&hi2c1, SENSOR_ADDR, 1, 100);
  HAL_StatusTypeDef gyro_ready = HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDR, GYRO_CONFIG_REG, 1, &set_gyro_range, 1, 100);
  HAL_StatusTypeDef acc_ready = HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDR, ACCEL_CONFIG_REG, 1, &set_accel_range, 1, 100);
  HAL_StatusTypeDef sleep_ready = HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDR, SLEEP_MODE_REG, 1, &set_sleep_mode, 1, 100);

  if(device_ready == HAL_OK && gyro_ready == HAL_OK && acc_ready == HAL_OK && sleep_ready == HAL_OK)
  {
    printf("Sensor is ready and configured.\n\r");
  }
  else
  {
    printf("Device is not ready.\n\r");
  }
}

/* Note: Sensitivity will change depending on the range configured for
 * the gyro and the accelerometer. Currently, it is not user configurable.
 */

void mpu6050_read_gyro(gyro_data * gyro)
{
//	gyro_data gyro;

    /* Gyroscope Measurements */
    // x-axis
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, GYRO_XOUT_HIGH, 1, &gyro->x_data[0], 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, GYRO_XOUT_LOW, 1, &gyro->x_data[1], 1, 100);

    gyro->x_val_raw = ((int16_t)gyro->x_data[0] << 8) | gyro->x_data[1];
    gyro->x_val = (((double)gyro->x_val_raw)/GYRO_SENSITIVITY);

    // y-axis
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, GYRO_YOUT_HIGH, 1, &gyro->y_data[0], 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, GYRO_YOUT_LOW, 1, &gyro->y_data[1], 1, 100);

    gyro->y_val_raw = ((int16_t)gyro->y_data[0] << 8) | gyro->y_data[1];
    gyro->y_val = (((double)gyro->y_val_raw)/GYRO_SENSITIVITY);

    // z-axis
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, GYRO_ZOUT_HIGH, 1, &gyro->z_data[0], 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, GYRO_ZOUT_LOW, 1, &gyro->z_data[1], 1, 100);

    gyro->z_val_raw = ((int16_t)gyro->z_data[0] << 8) | gyro->z_data[1];
    gyro->z_val = (((double)gyro->z_val_raw)/GYRO_SENSITIVITY);

}

void mpu6050_read_accel(accel_data * accel)
{
//	accel_data accel;

    /* Accelerometer Measurements */
	// x-axis
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, ACCEL_XOUT_HIGH, 1, &accel->x_data[0], 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, ACCEL_XOUT_LOW, 1, &accel->x_data[1], 1, 100);

    accel->x_val_raw = ((int16_t)accel->x_data[0] << 8) | accel->x_data[1];
    accel->x_val = (((double)accel->x_val_raw)*10.0)/ACCEL_SENSITIVITY;

    // y-axis
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, ACCEL_YOUT_HIGH, 1, &accel->y_data[0], 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, ACCEL_YOUT_LOW, 1, &accel->y_data[1], 1, 100);

    accel->y_val_raw = ((int16_t)accel->y_data[0] << 8) | accel->y_data[1];
    accel->y_val = (((double)accel->y_val_raw)*10.0)/ACCEL_SENSITIVITY;

    // z-axis
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, ACCEL_ZOUT_HIGH, 1, &accel->z_data[0], 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, ACCEL_ZOUT_LOW, 1, &accel->z_data[1], 1, 100);

    accel->z_val_raw = ((int16_t)accel->z_data[0] << 8) | accel->z_data[1];
    accel->z_val = (((double)accel->z_val_raw)*10.0)/ACCEL_SENSITIVITY;

    accel->roll_angle = atan(accel->y_val/sqrt((accel->x_val*accel->x_val)+(accel->z_val*accel->z_val))) * 1/(3.142/180);
    accel->pitch_angle = (-atan(accel->x_val/sqrt((accel->y_val*accel->y_val)+(accel->z_val*accel->z_val))) * 1/(3.142/180))+5.0;

}

