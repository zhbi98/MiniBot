
#include "usart.h"

UART_HandleTypeDef UartHandle;
unsigned char RevByte;
unsigned char Revcnt = 0;
unsigned char aRxBuffer[32];

void usart_init()
{
    UartHandle.Instance        = USART1;
    UartHandle.Init.BaudRate   = 115200;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;

    if (HAL_UART_DeInit(&UartHandle) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_UART_Receive_IT(&UartHandle, &RevByte, 1);

    RetargetInit(&UartHandle);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* Enable USART1 clock */
    __HAL_RCC_USART1_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*##-3- Configure the NVIC for UART ########################################*/
    /* NVIC for USART */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    /* Set transmission flag: transfer complete */
    if (UartHandle->Instance == USART1) {

    }
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    /* Set transmission flag: transfer complete */
    if (UartHandle->Instance == USART1) {
        aRxBuffer[Revcnt] = RevByte;
        Revcnt++;
        if (Revcnt >= 32)
        {
            Revcnt = 0;
        }
    }
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&UartHandle);
    HAL_UART_Receive_IT(&UartHandle, &RevByte, 1); // Restart USART interrupts
}

void usart_send_string(const unsigned char * string)
{
    unsigned int i = 0;

    while (string[i] != '\0')
        i++;
    if (i <= 0)
        return;

    while (HAL_UART_Transmit(&UartHandle, (unsigned char *)string, i, 0XFFFF) != HAL_OK);
}

void usart_send_fmt_string(unsigned char * format, ...)
{
    unsigned char value[128];

    va_list parameter_pointer;
    va_start(parameter_pointer, format);

    vsprintf(value, format, parameter_pointer);
    
    va_end(parameter_pointer);

    usart_send_string(value);
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
 * https://www.cnblogs.com/wt88/p/9624115.html
 * https://zhuanlan.zhihu.com/p/346217940
 */
