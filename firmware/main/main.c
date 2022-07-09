
#include "stm32f1xx_hal.h"
#include "time.h"
#include "MPU6050.h"
#include "usart.h"

/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle;
TIM_HandleTypeDef    TimHandle2;
__IO uint16_t cnt = 255;
int16_t AccelGyro[6]={ 0 };
extern unsigned char aRxBuffer[];

void SystemClock_Config(void);
static void Error_Handler(void);
static void pwm2_gpio_init();
static void pwm4_gpio_init();

int main()
{
    HAL_Init();
    SystemClock_Config();
#if 0
    MPU6050_Initialize();
#endif
    usart_init();
    /***********************************************
     *            TIM2 CONFIG
     **********************************************/
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    TimHandle.Instance = TIM2;
    TimHandle.Init.Prescaler = 720 - 1;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandle.Init.Period = 0xFFFF;
    TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&TimHandle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    pwm2_gpio_init();
    HAL_TIM_OC_Start_IT(&TimHandle, TIM_CHANNEL_2);
    /***********************************************
     *            TIM4 CONFIG
     **********************************************/
    TimHandle2.Instance = TIM4;
    TimHandle2.Init.Prescaler = 720 - 1;
    TimHandle2.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandle2.Init.Period = 0xFFFF;
    TimHandle2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TimHandle2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&TimHandle2) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&TimHandle2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    pwm4_gpio_init();
    HAL_TIM_OC_Start_IT(&TimHandle2, TIM_CHANNEL_4);

    for (;;) {
        /* Insert delay 100 ms */
        HAL_Delay(50);
        cnt--;
        if (cnt <= 50)
            cnt = 50;
#if 0
        MPU6050_GetRawAccelGyro(AccelGyro);
        uint16_t a = AccelGyro[0];
        uint16_t b = AccelGyro[1];
        uint16_t c = AccelGyro[2];
        uint16_t d = AccelGyro[3];
        uint16_t e = AccelGyro[4];
        uint16_t f = AccelGyro[5];
#endif
        // usart_send_string("Hello, MiNiBot!!!");
        // usart_send_fmt_string("MiNiBot:%d", 2010);
        // usart_send_string(aRxBuffer);
    }
    return 0;
}

void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef clkinitstruct = {0};
    RCC_OscInitTypeDef oscinitstruct = {0};
    
    /* Configure PLL ------------------------------------------------------*/
    /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
    /* Enable HSI and activate PLL with HSi_DIV2 as source */
    oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE/*RCC_OSCILLATORTYPE_HSI*/;
    oscinitstruct.HSEState        = RCC_HSE_ON/*RCC_HSE_OFF*/;
    oscinitstruct.LSEState        = RCC_LSE_OFF;
    oscinitstruct.HSIState        = RCC_HSI_ON;
    oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
    oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
    oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE/*RCC_PLLSOURCE_HSI_DIV2*/;
    oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9/*RCC_PLL_MUL16*/;
    if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
    {
        /* Initialization Error */
        while(1); 
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
         clocks dividers */
    clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
    clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
    if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
    {
        /* Initialization Error */
        while(1); 
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    if(htim_base->Instance==TIM2)
    {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();
        /* TIM2 interrupt Init */
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
    }
    else if(htim_base->Instance==TIM4)
    {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM4_CLK_ENABLE();
        /* TIM2 interrupt Init */
        HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // if (htim == &TimHandle) {
    //     HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
    // }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &TimHandle) {
        __IO uint16_t count = 0; // 因为捕获比较寄存器的值是16位的设置的值不能超过65535 如果超过那么会自动减65535(溢出)
        count = __HAL_TIM_GetCounter(htim); // 获取捕获比较寄存器的值
        __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,(count+cnt) % 0xFFFF); // 重新设置捕获比较寄存器的值
    }
    else if (htim == &TimHandle2) {
        __IO uint16_t count2 = 0; // 因为捕获比较寄存器的值是16位的设置的值不能超过65535 如果超过那么会自动减65535(溢出)
        count2 = __HAL_TIM_GetCounter(htim); // 获取捕获比较寄存器的值
        __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,(count2+cnt) % 0xFFFF); // 重新设置捕获比较寄存器的值
    }
}

static void Error_Handler(void)
{
    while (1)
    {
    }
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TimHandle);
}

void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TimHandle2);
}

static void pwm2_gpio_init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();                // 开启 GPIOC 时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};       // 结构体
    GPIO_InitStruct.Pin = GPIO_PIN_1;            // PC13
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP/*GPIO_MODE_OUTPUT_PP*/;   // 推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;           // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_GPIOB_CLK_ENABLE();                // 开启 GPIOC 时钟

    // GPIO_InitTypeDef GPIO_InitStruct = {0};       // 结构体
    GPIO_InitStruct.Pin = GPIO_PIN_9;            // PC13
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP/*GPIO_MODE_OUTPUT_PP*/;   // 推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;           // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void pwm4_gpio_init()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();                // 开启 GPIOC 时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};       // 结构体
    GPIO_InitStruct.Pin = GPIO_PIN_9;            // PC13
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP/*GPIO_MODE_OUTPUT_PP*/;   // 推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;           // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
