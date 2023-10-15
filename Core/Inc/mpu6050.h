/*
 * Header file for MPU6050 configuration
 *
 *  Created on: Sep. 28, 2023
 *      Author: Rijin
 */

#include <stdio.h>

#ifndef MPU6050_H_
#define MPU6050_H_

#define SENSOR_ADDR 0x68 << 1

/* Registers for the MPU6050*/

/* General config (Sampling Rate, Bandwidth, Delay, Sensitivity) */
#define SENSOR_CONFIG_REG 0x1A
#define ACCEL_SENSITIVITY 8192.0
#define GYRO_SENSITIVITY 65.5

/* Gyroscope Config */
#define GYRO_CONFIG_REG 0x1B
#define GYRO_RANGE_250 0x00
#define GYRO_RANGE_500 0x08
#define GYRO_RANGE_1000 0x09
#define GYRO_RANGE_2000 0xA

/* Gyroscope Measurements */
#define GYRO_XOUT_HIGH 0x43
#define GYRO_XOUT_LOW 0x44

#define GYRO_YOUT_HIGH 0x45
#define GYRO_YOUT_LOW 0x46

#define GYRO_ZOUT_HIGH 0x47
#define GYRO_ZOUT_LOW 0x48

/* Accelerometer Config */
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_RANGE_4G 0x08
#define ACCEL_RANGE_8G 0x09
#define ACCEL_RANGE_16G 0xA

/* Accelerometer Measurements */
#define ACCEL_XOUT_HIGH 0x3B
#define ACCEL_XOUT_LOW 0x3C

#define ACCEL_YOUT_HIGH 0x3D
#define ACCEL_YOUT_LOW 0x3E

#define ACCEL_ZOUT_HIGH 0x3F
#define ACCEL_ZOUT_LOW 0x40

/* Sleep Mode Config */
#define SLEEP_MODE_REG 0x6B
#define SLEEP_MODE_ON  0x40
#define SLEEP_MODE_OFF 0x00

/* Accelerometer & Gyroscope Data Initialization */
struct data_init {
    uint8_t x_data[2];
    int16_t x_val_raw;
    float x_val;

    uint8_t y_data[2];
    int16_t y_val_raw;
    float y_val;

    uint8_t z_data[2];
    int16_t z_val_raw;
    float z_val;

    float pitch_angle;
    float roll_angle;
};

typedef struct data_init gyro_data;
typedef struct data_init accel_data;

/* MPU6050 Sensor Functions */
void mpu6050_init();
void mpu6050_read_gyro();
void mpu6050_read_accel();

#endif /* MPU6050_H_ */
