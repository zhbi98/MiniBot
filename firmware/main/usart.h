
#ifndef __USART_H__
#define __USART_H__

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef UartHandle;

extern void usart_init();
static void Error_Handler();
extern void usart_send_string(const unsigned char * string);
extern void usart_send_fmt_string(unsigned char * format, ...);

#endif
