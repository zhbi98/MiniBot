
#ifndef __LV8731V_H__
#define __LV8731V_H__

#include "stm32f1xx_hal.h"

#define L_STEP_GPIO           GPIOB 
#define L_STEP_PIN            GPIO_PIN_9
#define L_STEP_CLOCK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

#define L_AT1_GPIO            GPIOC
#define L_AT1_PIN             GPIO_PIN_6
#define L_AT1_CLOCK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()

#define L_AT2_GPIO            GPIOB
#define L_AT2_PIN             GPIO_PIN_0
#define L_AT2_CLOCK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()

#define L_FR_GPIO             GPIOB
#define L_FR_PIN              GPIO_PIN_15
#define L_FR_CLOCK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()

#define R_STEP_GPIO           GPIOA
#define R_STEP_PIN            GPIO_PIN_1
#define R_STEP_CLOCK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define R_AT1_GPIO            GPIOA
#define R_AT1_PIN             GPIO_PIN_5
#define R_AT1_CLOCK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define R_AT2_GPIO            GPIOA
#define R_AT2_PIN             GPIO_PIN_4
#define R_AT2_CLOCK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define R_FR_GPIO             GPIOB
#define R_FR_PIN              GPIO_PIN_1
#define R_FR_CLOCK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()

extern void lv8731v_init();
extern void lv8731_R_speed(int speed);
extern void lv8731_L_speed(int speed);
extern void lv8731_R_dir(unsigned char dir);
extern void lv8731_L_dir(unsigned char dir);

#endif
