
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
    MPU_Init();

    for (;;) {
        /* Insert delay 100 ms */
        HAL_Delay(200);
        MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
        temp = MPU_Get_Temperature();
        pitch = atan2(aacy, aacz) * 180.0 / 3.14;

        // printf("ACC:x=%04d,y=%04d,z=%04d\n", aacx, aacy, aacz);
        // printf("GRO:x=%04d,y=%04d,z=%04d\n", gyrox, gyroy, gyroz);

        printf("ACCX:%s\n",  double_string(aacx / 16384.0, 3));
        printf("ACCY:%s\n",  double_string(aacy / 16384.0, 3));
        printf("ACCZ:%s\n",  double_string(aacz / 16384.0, 3));
        printf("ANGLE:%s\n", double_string(pitch, 3));
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
