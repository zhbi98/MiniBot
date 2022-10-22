
#ifndef __QMC5883_I2C_H__
#define __QMC5883_I2C_H__

#include "stm32f1xx_hal.h"
#include "time.h"

#define QMC5883_SCL_GPIO           GPIOA
#define QMC5883_SCL_PIN            GPIO_PIN_7
#define QMC5883_SCL_CLOCK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE();

#define QMC5883_SDA_GPIO           GPIOA
#define QMC5883_SDA_PIN            GPIO_PIN_6
#define QMC5883_SDA_CLOCK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE();

#define QMC5883_SDA_H()    HAL_GPIO_WritePin(QMC5883_SDA_GPIO, QMC5883_SDA_PIN, GPIO_PIN_SET)
#define QMC5883_SDA_L()    HAL_GPIO_WritePin(QMC5883_SDA_GPIO, QMC5883_SDA_PIN, GPIO_PIN_RESET)

#define QMC5883_SCL_H()    HAL_GPIO_WritePin(QMC5883_SCL_GPIO, QMC5883_SCL_PIN, GPIO_PIN_SET)
#define QMC5883_SCL_L()    HAL_GPIO_WritePin(QMC5883_SCL_GPIO, QMC5883_SCL_PIN, GPIO_PIN_RESET)

#define READ_QMC5883_SDA() HAL_GPIO_ReadPin(QMC5883_SDA_GPIO, QMC5883_SDA_PIN)

static void qmc5883_sleep_us(unsigned int us);
extern void qmc5883_i2c_sleep_time_test();
extern void qmc5883_i2c_gpio_init();
extern void qmc5883_sda_out_mode();
extern void qmc5883_sda_in_mode();
extern void qmc5883_i2c_start();
extern void qmc5883_i2c_stop();
extern unsigned char qmc5883_i2c_wait_ack();
extern void qmc5883_i2c_ack();
extern void qmc5883_i2c_nack();
extern void qmc5883_i2c_send_byte(unsigned char a_byte);
extern unsigned char qmc5883_i2c_read_byte(unsigned char ack);

#endif
