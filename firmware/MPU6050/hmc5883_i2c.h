
#ifndef __HMC5883_I2C_H__
#define __HMC5883_I2C_H__

#include "stm32f1xx_hal.h"
#include "time.h"

#define HMC5883_SCL_GPIO           GPIOA
#define HMC5883_SCL_PIN            GPIO_PIN_7
#define HMC5883_SCL_CLOCK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE();

#define HMC5883_SDA_GPIO           GPIOA
#define HMC5883_SDA_PIN            GPIO_PIN_6
#define HMC5883_SDA_CLOCK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE();

#define HMC5883_SDA_H()    HAL_GPIO_WritePin(HMC5883_SDA_GPIO, HMC5883_SDA_PIN, GPIO_PIN_SET)
#define HMC5883_SDA_L()    HAL_GPIO_WritePin(HMC5883_SDA_GPIO, HMC5883_SDA_PIN, GPIO_PIN_RESET)

#define HMC5883_SCL_H()    HAL_GPIO_WritePin(HMC5883_SCL_GPIO, HMC5883_SCL_PIN, GPIO_PIN_SET)
#define HMC5883_SCL_L()    HAL_GPIO_WritePin(HMC5883_SCL_GPIO, HMC5883_SCL_PIN, GPIO_PIN_RESET)

#define READ_HMC5883_SDA() HAL_GPIO_ReadPin(HMC5883_SDA_GPIO, HMC5883_SDA_PIN)

static void hmc5883_sleep_us(unsigned int us);
extern void hmc5883_i2c_sleep_time_test();
extern void hmc5883_i2c_gpio_init();
extern void hmc5883_sda_out_mode();
extern void hmc5883_sda_in_mode();
extern void hmc5883_i2c_start();
extern void hmc5883_i2c_stop();
extern unsigned char hmc5883_i2c_wait_ack();
extern void hmc5883_i2c_ack();
extern void hmc5883_i2c_nack();
extern void hmc5883_i2c_send_byte(unsigned char a_byte);
extern unsigned char hmc5883_i2c_read_byte(unsigned char ack);

#endif
