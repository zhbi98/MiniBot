
#include "lv8731v.h"

/* TIM handle declaration */
TIM_HandleTypeDef Tim2Handle;
TIM_HandleTypeDef Tim4Handle;

__IO uint16_t TIM2_cnt = 0;
__IO uint16_t TIM4_cnt = 0;

static void lv8731v_gpio_init();
static void lv8731v_R_timer_init();
static void lv8731v_L_timer_init();
static void lv8731v_R_timer_gpio_init();
static void lv8731v_L_timer_gpio_init();
static void Error_Handler(void);

static void lv8731v_gpio_init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    L_STEP_CLOCK_ENABLE();
    GPIO_InitStruct.Pin = L_STEP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(L_STEP_GPIO, &GPIO_InitStruct);

    L_AT1_CLOCK_ENABLE();
    GPIO_InitStruct.Pin = L_AT1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(L_AT1_GPIO, &GPIO_InitStruct);

    L_AT2_CLOCK_ENABLE();
    GPIO_InitStruct.Pin = L_AT2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(L_AT2_GPIO, &GPIO_InitStruct);

    L_FR_CLOCK_ENABLE();
    GPIO_InitStruct.Pin = L_FR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(L_FR_GPIO, &GPIO_InitStruct);

    R_STEP_CLOCK_ENABLE();
    GPIO_InitStruct.Pin = R_STEP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(R_STEP_GPIO, &GPIO_InitStruct);

    R_AT1_CLOCK_ENABLE();
    GPIO_InitStruct.Pin = R_AT1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(R_AT1_GPIO, &GPIO_InitStruct);

    R_AT2_CLOCK_ENABLE();
    GPIO_InitStruct.Pin = R_AT2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(R_AT2_GPIO, &GPIO_InitStruct);

    R_FR_CLOCK_ENABLE();
    GPIO_InitStruct.Pin = R_FR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(R_FR_GPIO, &GPIO_InitStruct);

    HAL_GPIO_WritePin(L_AT1_GPIO, L_AT1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L_AT2_GPIO, L_AT2_PIN, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(L_ST_GPIO, L_ST_PIN, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(L_OE_GPIO, L_OE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L_FR_GPIO, L_FR_PIN, GPIO_PIN_SET);
    
    HAL_GPIO_WritePin(R_AT1_GPIO, R_AT1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(R_AT2_GPIO, R_AT2_PIN, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(R_ST_GPIO, R_ST_PIN, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(R_OE_GPIO, R_OE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(R_FR_GPIO, R_FR_PIN, GPIO_PIN_RESET);
}

/*********************************************************
 *         RIGHT MOTOR, PWM MINI 120Hz, MAX 6000Hz
 ********************************************************/
static void lv8731v_R_timer_init()
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
    Tim2Handle.Init.Prescaler = 30/*720*/ - 1;
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

    lv8731v_R_timer_gpio_init();

    HAL_TIM_OC_Start_IT(&Tim2Handle, TIM_CHANNEL_2);
}

/*********************************************************
 *         LEFT MOTOR, PWM MINI 120Hz, MAX 6000Hz
 ********************************************************/
static void lv8731v_L_timer_init()
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
    Tim4Handle.Init.Prescaler = 30/*720*/ - 1;
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

    lv8731v_L_timer_gpio_init();

    HAL_TIM_OC_Start_IT(&Tim4Handle, TIM_CHANNEL_4);
}

void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&Tim2Handle, TIM_IT_CC2) != RESET) {
        __IO uint16_t count = 0; // 因为捕获比较寄存器的值是 16 位的设置的值不能超过 65535 如果超过那么会自动减 65535(溢出)
        count = __HAL_TIM_GetCounter(&Tim2Handle); // 获取捕获比较寄存器的值
        __HAL_TIM_SET_COMPARE(&Tim2Handle, TIM_CHANNEL_2, (count + TIM2_cnt) % 0xFFFF); // 重新设置捕获比较寄存器的值, %0xFFFF 防止溢出

        __HAL_TIM_CLEAR_IT(&Tim2Handle, TIM_IT_CC2);
    }
}

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&Tim4Handle, TIM_IT_CC4) != RESET) {
        __IO uint16_t count = 0; // 因为捕获比较寄存器的值是 16 位的设置的值不能超过 65535 如果超过那么会自动减 65535(溢出)
        count = __HAL_TIM_GetCounter(&Tim4Handle); // 获取捕获比较寄存器的值
        __HAL_TIM_SET_COMPARE(&Tim4Handle, TIM_CHANNEL_4, (count + TIM4_cnt) % 0xFFFF); // 重新设置捕获比较寄存器的值, %0xFFFF 防止溢出

        __HAL_TIM_CLEAR_IT(&Tim4Handle, TIM_IT_CC4);
    }
}

static void lv8731v_R_timer_gpio_init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();                                  // 开启 GPIOA 时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};                        // 结构体
    GPIO_InitStruct.Pin = R_STEP_PIN/*GPIO_PIN_1*/;                // PA1
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP/*GPIO_MODE_OUTPUT_PP*/; // 推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;                            // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;                  // 高速
    HAL_GPIO_Init(R_STEP_GPIO/*GPIOA*/, &GPIO_InitStruct);
}

static void lv8731v_L_timer_gpio_init()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();                                  // 开启 GPIOB 时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};                        // 结构体
    GPIO_InitStruct.Pin = L_STEP_PIN/*GPIO_PIN_9*/;                // PB9
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP/*GPIO_MODE_OUTPUT_PP*/; // 推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;                            // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;                  // 高速
    HAL_GPIO_Init(L_STEP_GPIO/*GPIOB*/, &GPIO_InitStruct);
}

static void Error_Handler(void)
{
    while (1)
    {
    }
}

void lv8731v_init()
{
    lv8731v_gpio_init();
    lv8731v_R_timer_init();
    lv8731v_L_timer_init();
}

#define TIM_CLK    2400000U // 2.4MHz
#define PULSE_MAX  5500U    // 5500 * 2.4Hz = 13.2KHZ

void lv8731_R_speed(unsigned int speed)
{
    if (speed > PULSE_MAX)
        speed = PULSE_MAX;

    TIM2_cnt = (speed == 0) ? (int)(TIM_CLK / 2) : (int)(1000000U / speed / 2.0);
}

void lv8731_L_speed(unsigned int speed)
{
    if (speed > PULSE_MAX)
        speed = PULSE_MAX;

    TIM4_cnt = (speed == 0) ? (int)(TIM_CLK / 2) : (int)(1000000U / speed / 2.0);
}

void lv8731_R_dir(unsigned char dir)
{
    GPIO_PinState state = dir == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(R_FR_GPIO, R_FR_PIN, state);
}

void lv8731_L_dir(unsigned char dir)
{
    GPIO_PinState state = dir == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(L_FR_GPIO, L_FR_PIN, state);
}

#if 0
#include <stdio.h>

#define TIM_CLK    100000U // 100KHz
#define PULSE_MINI 100U    // 100Hz
#define PULSE_MAX  6000U   // 6000HZ
#define PULSE_INCR 59.9F   // (PULSE_MAX - PULSE_MINI) / 100

void timer_counter_value()
{
    unsigned int j = 0;

    printf("const short cnt_list[100] = {\n\t");

    for (unsigned int i = 0; i < 100; i++) {
        // target_speed = PULSE_MINI + PULSE_INCR * i
        printf("%-3d, ", (int)(TIM_CLK / (PULSE_MINI + PULSE_INCR * i) / 2));
        j++;
        if (j == 5) {
            printf("\n\t");
            j = 0;
        }
    }

    printf("\n};");
}
#endif
