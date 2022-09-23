
# MiNiBot 

## 浮点打印

直接打印

```
void printDouble(double v, int decimalDigits)
{
    int i = 1;
    int intPart, fractPart;

    for (; decimalDigits != 0; i *= 10, decimalDigits--);
    intPart = (int)v;

    fractPart = (int)((v - (double)(int)v) * i);
    if (fractPart < 0) fractPart *= -1;
    printf("%d.%d", intPart, fractPart);
}
```

返回值的模式

```
char * Float2String(float value)
{
    static char * str[16];

    int decimalDigits = 3;
    int i = 1;
    int intPart, fractPart;

    for (; decimalDigits != 0; i *= 10, decimalDigits--);
    intPart = (int)value;

    fractPart = (int)((value - (double)(int)value) * i);
    if (fractPart < 0) fractPart *= -1;
    // printf("%i.%i", intPart, fractPart);

    sprintf((char*)str, "%d.%d", intPart, fractPart);

    return str;
}
```

## 步进电机控制 IO 映射

```
#define L_STEP_GPIO  GPIOB
#define L_STEP_PIN   GPIO_Pin_9
#define L_STEP_CLOCK RCC_APB2Periph_GPIOB

#define L_AT1_GPIO   GPIOC
#define L_AT1_PIN    GPIO_Pin_6
#define L_AT1_CLOCK  RCC_APB2Periph_GPIOC

#define L_AT2_GPIO   GPIOB
#define L_AT2_PIN    GPIO_Pin_0
#define L_AT2_CLOCK  RCC_APB2Periph_GPIOB

#define L_FR_GPIO    GPIOB
#define L_FR_PIN     GPIO_Pin_15
#define L_FR_CLOCK   RCC_APB2Periph_GPIOB

#define R_STEP_GPIO  GPIOA
#define R_STEP_PIN   GPIO_Pin_1
#define R_STEP_CLOCK RCC_APB2Periph_GPIOA

#define R_AT1_GPIO   GPIOA
#define R_AT1_PIN    GPIO_Pin_5
#define R_AT1_CLOCK  RCC_APB2Periph_GPIOA

#define R_AT2_GPIO   GPIOA
#define R_AT2_PIN    GPIO_Pin_4
#define R_AT2_CLOCK  RCC_APB2Periph_GPIOA

#define R_FR_GPIO    GPIOB
#define R_FR_PIN     GPIO_Pin_1
#define R_FR_CLOCK   RCC_APB2Periph_GPIOB
```

## 定时器比较模式

使用定时器比较模式来生成可变频率的 PWM 信号。

```
#include "stm32f1xx_hal.h"

TIM_HandleTypeDef    TimHandle;

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
static void Error_Handler(void);
void TIM2_IRQHandler(void);
static void pwm2_gpio_init();

int main()
{
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

    for (;;) {}

    return 0;
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
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &TimHandle) {
        __IO uint16_t count = 0; // 因为捕获比较寄存器的值是16位的设置的值不能超过65535 如果超过那么会自动减65535(溢出)
        count = __HAL_TIM_GetCounter(htim); // 获取捕获比较寄存器的值
        __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,(count+cnt) % 0xFFFF); // 重新设置捕获比较寄存器的值
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

static void pwm2_gpio_init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();                // 开启 GPIOC 时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};       // 结构体
    GPIO_InitStruct.Pin = GPIO_PIN_1;            // PC13
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP/*GPIO_MODE_OUTPUT_PP*/;   // 推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;           // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
```
