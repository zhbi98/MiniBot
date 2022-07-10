
#ifndef __MPU6050_I2C_H__
#define __MPU6050_I2C_H__

#include "stm32f1xx_hal.h"

#define MPU6050_SCL_GPIO           GPIOB
#define MPU6050_SCL_PIN            GPIO_PIN_10
#define MPU6050_SCL_CLOCK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE();

#define MPU6050_SDA_GPIO           GPIOB
#define MPU6050_SDA_PIN            GPIO_PIN_11
#define MPU6050_SDA_CLOCK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE();

#define MPU6050_SDA_H()    HAL_GPIO_WritePin(MPU6050_SDA_GPIO, MPU6050_SDA_PIN, GPIO_PIN_SET)
#define MPU6050_SDA_L()    HAL_GPIO_WritePin(MPU6050_SDA_GPIO, MPU6050_SDA_PIN, GPIO_PIN_RESET)

#define MPU6050_SCL_H()    HAL_GPIO_WritePin(MPU6050_SCL_GPIO, MPU6050_SCL_PIN, GPIO_PIN_SET)
#define MPU6050_SCL_L()    HAL_GPIO_WritePin(MPU6050_SCL_GPIO, MPU6050_SCL_PIN, GPIO_PIN_RESET)

#define READ_MPU6050_SDA() HAL_GPIO_ReadPin(MPU6050_SDA_GPIO, MPU6050_SDA_PIN)

static void mpu6050_sleep_us(unsigned int us);
extern void mpu6050_i2c_sleep_time_test();
extern void mpu6050_i2c_gpio_init();
extern void mpu6050_sda_out_mode();
extern void mpu6050_sda_in_mode();
extern void mpu6050_i2c_start();
extern void mpu6050_i2c_stop();
extern unsigned char mpu6050_i2c_wait_ack();
extern void mpu6050_i2c_ack();
extern void mpu6050_i2c_nack();
extern void mpu6050_i2c_send_byte(unsigned char a_byte);
extern unsigned char mpu6050_i2c_read_byte(unsigned char ack);

#endif
