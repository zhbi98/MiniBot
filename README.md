
# MiNiBot 

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

#include "stm32f1xx_hal.h"
#include "time.h"

#define TIMx                           TIM2
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM2_CLK_ENABLE()


/* Definition for TIMx's NVIC */
// #define TIMx_IRQn                      TIM3_IRQn
// #define TIMx_IRQHandler                TIM3_IRQHandler

/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle;
uint32_t uwPrescalerValue = 0;

void SystemClock_Config(void);
void test_gpio_init();
static void Error_Handler(void);

int main()
{
    HAL_Init();
    SystemClock_Config();
    test_gpio_init();

    TIMx_CLK_ENABLE();
    /* Set TIMx instance */
    TimHandle.Instance = TIMx;

    /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
    */
    TimHandle.Init.Period            = 7200 - 1;
    TimHandle.Init.Prescaler         = 10 - 1;
    TimHandle.Init.ClockDivision     = 0;
    TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    TimHandle.Init.RepetitionCounter = 0;
    TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE/*TIM_AUTORELOAD_PRELOAD_DISABLE*/;

    if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
    // HAL_NVIC_SetPriority(TIMx_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(TIMx_IRQn);
    /*##-2- Start the TIM Base generation in interrupt mode ####################*/
    /* Start Channel1 */
    if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }

      TIM_OC_InitTypeDef sConfigOC = {0};
      
      sConfigOC.OCMode = TIM_OCMODE_PWM1;
      sConfigOC.Pulse = 3600;
      sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
      sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
      
      if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)       //初始化PWM
      {
        Error_Handler();
      }
      if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)     //配置TIM3通道4
      {
        Error_Handler();
      }
      HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2);
      // HAL_TIM_MspPostInit(&TimHandle);  //调用定时器底层驱动，时钟使能，引脚配置

    int i = 0;
    for (;;) {
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        /* Insert delay 100 ms */
        HAL_Delay(500);
        // TIM1->CCR1 = i;
        // i+=10;
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

void test_gpio_init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();                // 开启 GPIOC 时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};       // 结构体
    GPIO_InitStruct.Pin = GPIO_PIN_1;            // PC13
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP/*GPIO_MODE_OUTPUT_PP*/;   // 推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;           // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &TimHandle) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
    }
}

static void Error_Handler(void)
{
  while (1)
  {
  }
}

extern TIM_HandleTypeDef    TimHandle;
void TIMx_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}

// HAL 多个定时器需要在函数 `HAL_TIM_PeriodElapsedCallback` 中添加判断：
// ```
//     if (htim == &htim1) {
//         // 如果说定时器1的中断
//     } else if(htim == &htim2) {
//         // 如果是定时器2的中断
//     }
// ```

// HAL PWM 配置详细查看：
// https://wenku.baidu.com/view/ece8b006bfd126fff705cc1755270722192e59da.html

```
