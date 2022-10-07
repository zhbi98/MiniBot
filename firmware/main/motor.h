
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f1xx_hal.h"

extern void PWM_tim2_init();
extern void PWM_tim4_init();
extern void PWM_tim2_pulse_set(int pwm);
extern void PWM_tim4_pulse_set(int pwm);

#endif
