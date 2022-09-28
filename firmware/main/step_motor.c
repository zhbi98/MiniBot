
#include "step_motor.h"

/* TIM handle declaration */
TIM_HandleTypeDef TimHandle2;
TIM_HandleTypeDef TimHandle4;
__IO uint16_t cnt = 255;

static void pwm2_gpio_init();
static void pwm4_gpio_init();
static void Error_Handler(void);

void step_motor_pwm_init()
{
    /***********************************************
     *            TIM2 CONFIG
     **********************************************/
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    TimHandle2.Instance = TIM2;
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
    if (HAL_TIM_OC_ConfigChannel(&TimHandle2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    pwm2_gpio_init();
    HAL_TIM_OC_Start_IT(&TimHandle2, TIM_CHANNEL_2);
    /***********************************************
     *            TIM4 CONFIG
     **********************************************/
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    TimHandle4.Instance = TIM4;
    TimHandle4.Init.Prescaler = 720 - 1;
    TimHandle4.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandle4.Init.Period = 0xFFFF;
    TimHandle4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TimHandle4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&TimHandle4) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&TimHandle4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    pwm4_gpio_init();
    HAL_TIM_OC_Start_IT(&TimHandle4, TIM_CHANNEL_4);
}

static void Error_Handler(void)
{
    while (1)
    {
    }
}

void TIM2_IRQHandler(void)
{
    __IO uint16_t count = 0; // 因为捕获比较寄存器的值是16位的设置的值不能超过65535 如果超过那么会自动减65535(溢出)
    count = __HAL_TIM_GetCounter(&TimHandle2); // 获取捕获比较寄存器的值
    __HAL_TIM_SET_COMPARE(&TimHandle2,TIM_CHANNEL_2,(count+cnt) % 0xFFFF); // 重新设置捕获比较寄存器的值, %0xFFFF 防止溢出

}

void TIM4_IRQHandler(void)
{
    __IO uint16_t count = 0; // 因为捕获比较寄存器的值是16位的设置的值不能超过65535 如果超过那么会自动减65535(溢出)
    count = __HAL_TIM_GetCounter(&TimHandle4); // 获取捕获比较寄存器的值
    __HAL_TIM_SET_COMPARE(&TimHandle4,TIM_CHANNEL_4,(count+cnt) % 0xFFFF); // 重新设置捕获比较寄存器的值, %0xFFFF 防止溢出
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
