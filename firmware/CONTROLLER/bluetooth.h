
#ifndef __BLUE_TOOTH_H__
#define __BLUE_TOOTH_H__

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef BLE_UartHandle;

extern void ble_usart_init();
extern void ble_usart_gpio_init();
extern void ble_init();
extern void ble_usart_send_string(const unsigned char * string);
extern void ble_usart_send_fmt_string(unsigned char * format, ...);
static void Error_Handler();

#endif
