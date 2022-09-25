
#include "stm32f1xx_hal.h"
#include <string.h>
#include <math.h>
#include "time.h"
#include "log.h"
#include "ftos.h"
#include "retarget.h"

#include "MPU6050_I2C.h"
#include "MPU6050.h"

void SystemClock_Config(void);
void TIM3_init();

/**
 * Y:tan(pitch) = tan(Axz) = Rx/Rz
 * X:tan(roll)  = tan(Ayz) = Ry/Rz
 */

int main()
{
    short aacx, aacy, aacz;
    short gyrox, gyroy, gyroz;
    short temp;
    float pitch, roll, yaw;

    HAL_Init();
    SystemClock_Config();
    usart_init();
    RetargetInit(&UartHandle);
    TIM3_init();
    MPU_Init();

    for (;;) {
        /* Insert delay 100 ms */
        // HAL_Delay(200);
        MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
        temp = MPU_Get_Temperature();
        roll = atan2(aacy/16384.0, aacz/16384.0) * 180.0 / 3.14;
        // pitch = atan2(aacx, aacz) * 180.0 / 3.14;
        pitch = -atan2(aacx/16384.0, sqrt((aacy/16384.0) * (aacy/16384.0) + (aacz/16384.0) * (aacz/16384.0))) * 180.0 / 3.14;

        // printf("ACC:x=%04d,y=%04d,z=%04d\n", aacx, aacy, aacz);
        // printf("GRO:x=%04d,y=%04d,z=%04d\n", gyrox, gyroy, gyroz);

        // printf("ACCX :%s\n", double_string(aacx / 16384.0, 3));
        // printf("ACCY :%s\n", double_string(aacy / 16384.0, 3));
        // printf("ACCZ :%s\n", double_string(aacz / 16384.0, 3));
        // printf("PITCH:%s\n", double_string(pitch, 3));
        // printf("ROLL :%s\n", double_string(roll, 3));
        send_mpu6050_data(aacx, aacy, aacz, gyrox, gyroy, gyroz);
        send_dmp_data(aacx, aacy, aacz, gyrox, gyroy, gyroz, 
            (int)(roll * 100), (int)(pitch * 100), (int)(yaw * 10));
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

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef * tim_baseHandle)
{
    if (tim_baseHandle->Instance == TIM3) {
        /* USER CODE BEGIN TIM3_MspInit 0 */

        /* USER CODE END TIM3_MspInit 0 */
        /* TIM3 clock enable */
        __HAL_RCC_TIM3_CLK_ENABLE();

        /* TIM3 interrupt Init */
        HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
        /* USER CODE BEGIN TIM3_MspInit 1 */

        /* USER CODE END TIM3_MspInit 1 */
    }
}

TIM_HandleTypeDef Tim3Handle;

void TIM3_init()
{
    /***********************************************
     *            TIM3 CONFIG
     **********************************************/
    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    Tim3Handle.Instance               = TIM3;
    Tim3Handle.Init.Prescaler         = 7200 - 1;
    Tim3Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    Tim3Handle.Init.Period            = 100;
    Tim3Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    Tim3Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&Tim3Handle) != HAL_OK) {
        // Error_Handler();
    }

    HAL_TIM_Base_Start_IT(&Tim3Handle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &Tim3Handle) {
        // printf("%s\n", "TIM3");
    }
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&Tim3Handle);
}
