
#include "motor.h"

/* TIM handle declaration */
TIM_HandleTypeDef Tim2Handle;
TIM_HandleTypeDef Tim4Handle;

__IO uint16_t tim2cnt = 255;
__IO uint16_t tim4cnt = 255;

static void TIM2_gpio_init();
static void TIM4_gpio_init();
static void Error_Handler(void);

void TIM2_init()
{
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};


    Tim2Handle.Instance = TIM2;
    Tim2Handle.Init.Prescaler = 720 - 1;
    Tim2Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    Tim2Handle.Init.Period = 0xFFFF;
    Tim2Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    Tim2Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&Tim2Handle) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_OC_ConfigChannel(&Tim2Handle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    TIM2_gpio_init();

    HAL_TIM_OC_Start_IT(&Tim2Handle, TIM_CHANNEL_2);
}

void TIM4_init()
{
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};


    Tim4Handle.Instance = TIM4;
    Tim4Handle.Init.Prescaler = 720 - 1;
    Tim4Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    Tim4Handle.Init.Period = 0xFFFF;
    Tim4Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    Tim4Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&Tim4Handle) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_OC_ConfigChannel(&Tim4Handle, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }

    TIM4_gpio_init();

    HAL_TIM_OC_Start_IT(&Tim4Handle, TIM_CHANNEL_4);
}

void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&Tim2Handle, TIM_IT_CC2) != RESET) {
        __IO uint16_t count = 0; // 因为捕获比较寄存器的值是 16 位的设置的值不能超过 65535 如果超过那么会自动减 65535(溢出)
        count = __HAL_TIM_GetCounter(&Tim2Handle); // 获取捕获比较寄存器的值
        __HAL_TIM_SET_COMPARE(&Tim2Handle,TIM_CHANNEL_2,(count+tim2cnt) % 0xFFFF); // 重新设置捕获比较寄存器的值, %0xFFFF 防止溢出

        __HAL_TIM_CLEAR_IT(&Tim2Handle, TIM_IT_CC2);
    }
}

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&Tim4Handle, TIM_IT_CC4) != RESET) {
        __IO uint16_t count = 0; // 因为捕获比较寄存器的值是 16 位的设置的值不能超过 65535 如果超过那么会自动减 65535(溢出)
        count = __HAL_TIM_GetCounter(&Tim4Handle); // 获取捕获比较寄存器的值
        __HAL_TIM_SET_COMPARE(&Tim4Handle,TIM_CHANNEL_4,(count+tim4cnt) % 0xFFFF); // 重新设置捕获比较寄存器的值, %0xFFFF 防止溢出

        __HAL_TIM_CLEAR_IT(&Tim4Handle, TIM_IT_CC4);
    }
}

static void TIM2_gpio_init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();                                  // 开启 GPIOA 时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};                        // 结构体
    GPIO_InitStruct.Pin = GPIO_PIN_1;                              // PA1
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP/*GPIO_MODE_OUTPUT_PP*/; // 推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;                            // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;                  // 高速
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void TIM4_gpio_init()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();                                  // 开启 GPIOB 时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};                        // 结构体
    GPIO_InitStruct.Pin = GPIO_PIN_9;                              // PB9
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP/*GPIO_MODE_OUTPUT_PP*/; // 推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;                            // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;                  // 高速
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void Error_Handler(void)
{
    while (1)
    {
    }
}
