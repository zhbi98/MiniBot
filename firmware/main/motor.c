
#include "motor.h"

/* TIM handle declaration */
TIM_HandleTypeDef Tim2Handle;
TIM_HandleTypeDef Tim4Handle;

const short cnt_list[100];
__IO uint16_t tim2cnt = 50;
__IO uint16_t tim4cnt = 50;

static void PWM_tim2_gpio_init();
static void PWM_tim4_gpio_init();
static void Error_Handler(void);

/*********************************************************
 *         RIGHT MOTOR, PWM MINI 120Hz, MAX 1800Hz
 ********************************************************/
void PWM_tim2_init()
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

    PWM_tim2_gpio_init();

    HAL_TIM_OC_Start_IT(&Tim2Handle, TIM_CHANNEL_2);
}

/*********************************************************
 *         LEFT MOTOR, PWM MINI 120Hz, MAX 1800Hz
 ********************************************************/
void PWM_tim4_init()
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

    PWM_tim4_gpio_init();

    HAL_TIM_OC_Start_IT(&Tim4Handle, TIM_CHANNEL_4);
}

void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&Tim2Handle, TIM_IT_CC2) != RESET) {
        __IO uint16_t count = 0; // 因为捕获比较寄存器的值是 16 位的设置的值不能超过 65535 如果超过那么会自动减 65535(溢出)
        count = __HAL_TIM_GetCounter(&Tim2Handle); // 获取捕获比较寄存器的值
        __HAL_TIM_SET_COMPARE(&Tim2Handle, TIM_CHANNEL_2, (count + cnt_list[tim2cnt]) % 0xFFFF); // 重新设置捕获比较寄存器的值, %0xFFFF 防止溢出

        __HAL_TIM_CLEAR_IT(&Tim2Handle, TIM_IT_CC2);
    }
}

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&Tim4Handle, TIM_IT_CC4) != RESET) {
        __IO uint16_t count = 0; // 因为捕获比较寄存器的值是 16 位的设置的值不能超过 65535 如果超过那么会自动减 65535(溢出)
        count = __HAL_TIM_GetCounter(&Tim4Handle); // 获取捕获比较寄存器的值
        __HAL_TIM_SET_COMPARE(&Tim4Handle, TIM_CHANNEL_4, (count + cnt_list[tim4cnt]) % 0xFFFF); // 重新设置捕获比较寄存器的值, %0xFFFF 防止溢出

        __HAL_TIM_CLEAR_IT(&Tim4Handle, TIM_IT_CC4);
    }
}

static void PWM_tim2_gpio_init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();                                  // 开启 GPIOA 时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};                        // 结构体
    GPIO_InitStruct.Pin = GPIO_PIN_1;                              // PA1
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP/*GPIO_MODE_OUTPUT_PP*/; // 推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;                            // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;                  // 高速
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void PWM_tim4_gpio_init()
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

void PWM_tim2_pulse_set(int pwm)
{
    tim2cnt = pwm;
}

void PWM_tim4_pulse_set(int pwm)
{
    tim4cnt = pwm;
}

const short cnt_list[100] = {
    416,    373,    338,    309,    285,
    264,    246,    230,    217,    204,
    193,    183,    175,    167,    159,
    152,    146,    141,    135,    130,
    126,    122,    118,    114,    110,
    107,    104,    101,    98, 96,
    93, 91, 89, 86, 84,
    82, 81, 79, 77, 75,
    74, 72, 71, 70, 68,
    67, 66, 65, 63, 62,
    61, 60, 59, 58, 57,
    56, 56, 55, 54, 53,
    52, 51, 51, 50, 49,
    49, 48, 47, 47, 46,
    46, 45, 44, 44, 43,
    43, 42, 42, 41, 41,
    40, 40, 39, 39, 39,
    38, 38, 37, 37, 37,
    36, 36, 35, 35, 35,
    34, 34, 34, 33, 33,
};

#if 0
#include <stdio.h>

int main()
{
    printf("const short cnt_list[100] = {\n");

    unsigned int j = 0;

    for (unsigned int i = 0; i < 100; i++) {
        printf("\t%d,", (int)(100000 / (120 + 13.8 * i) / 2));
        
        j++;
        if (j == 5) {
            printf("\n");
            j = 0;
        }
    }

    printf("};");

   return 0;
}

const short cnt_list[100] = {
    416,    373,    338,    309,    285,
    264,    246,    230,    217,    204,
    193,    183,    175,    167,    159,
    152,    146,    141,    135,    130,
    126,    122,    118,    114,    110,
    107,    104,    101,    98, 96,
    93, 91, 89, 86, 84,
    82, 81, 79, 77, 75,
    74, 72, 71, 70, 68,
    67, 66, 65, 63, 62,
    61, 60, 59, 58, 57,
    56, 56, 55, 54, 53,
    52, 51, 51, 50, 49,
    49, 48, 47, 47, 46,
    46, 45, 44, 44, 43,
    43, 42, 42, 41, 41,
    40, 40, 39, 39, 39,
    38, 38, 37, 37, 37,
    36, 36, 35, 35, 35,
    34, 34, 34, 33, 33,
};
#endif
