
#ifndef __BLUE_TOOTH_H__
#define __BLUE_TOOTH_H__

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include "log.h"

#define BLE_APP 1

struct _ble_speed {
	int FB;
	int LR;
};

extern UART_HandleTypeDef BLE_UartHandle;
extern struct _ble_speed ble_speed;

extern void ble_usart_init();
extern void ble_usart_gpio_init();
extern void ble_init();
extern void ble_usart_send_string(const unsigned char * string);
extern void ble_usart_send_fmt_string(unsigned char * format, ...);
static void Error_Handler();
extern void BLE_receive(unsigned char ble_data);

#endif
