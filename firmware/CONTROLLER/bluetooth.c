
#include "bluetooth.h"

UART_HandleTypeDef BLE_UartHandle;
unsigned char BLE_RevByte;
unsigned char BLE_Revcnt = 0;
unsigned char BLE_aRxBuffer[32];

void ble_usart_init()
{
    BLE_UartHandle.Instance        = USART2;
    BLE_UartHandle.Init.BaudRate   = 9600;
    BLE_UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    BLE_UartHandle.Init.StopBits   = UART_STOPBITS_1;
    BLE_UartHandle.Init.Parity     = UART_PARITY_NONE;
    BLE_UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    BLE_UartHandle.Init.Mode       = UART_MODE_TX_RX;

    if (HAL_UART_DeInit(&BLE_UartHandle) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UART_Init(&BLE_UartHandle) != HAL_OK)
    {
        Error_Handler();
    }
    // HAL_UART_Receive_IT(&BLE_UartHandle, &BLE_RevByte, 1);
    __HAL_UART_ENABLE_IT(&BLE_UartHandle, UART_IT_RXNE);
}

void ble_usart_gpio_init()
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* Enable USART1 clock */
    __HAL_RCC_USART2_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_2;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*##-3- Configure the NVIC for UART ########################################*/
    /* NVIC for USART */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void ble_init()
{
    /**
     * USART clock must be initialized first,
     * Otherwise, USART can't initialized.
     */
    ble_usart_gpio_init();
    ble_usart_init();
}

void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&BLE_UartHandle, UART_FLAG_RXNE) != RESET) {
        
        HAL_UART_Receive(&BLE_UartHandle, &BLE_RevByte, 1, 1000);

        BLE_aRxBuffer[BLE_Revcnt] = BLE_RevByte;
        BLE_Revcnt++;
        if (Revcnt >= 32)
        {
            BLE_Revcnt = 0;
        }
    }
}

void ble_usart_send_string(const unsigned char * string)
{
    unsigned int i = 0;

    while (string[i] != '\0')
        i++;

    while (HAL_UART_Transmit(&BLE_UartHandle, (unsigned char *)string, i, 0XFFFF) != HAL_OK);
}

void ble_usart_send_fmt_string(unsigned char * format, ...)
{
    unsigned char value[128];

    va_list parameter_pointer;
    va_start(parameter_pointer, format);

    vsprintf(value, format, parameter_pointer);
    
    va_end(parameter_pointer);

    ble_usart_send_string(value);
}

static void Error_Handler()
{
    while(1)
    {
        /* Error if LED2 is slowly blinking (1 sec. period) */
    }
}

/**
 * STM32 HAL USART Refer
 * https://www.cnblogs.com/UnfriendlyARM/p/10321838.html
 * https://www.cnblogs.com/wt88/p/9624115.html
 * https://zhuanlan.zhihu.com/p/346217940
 */
