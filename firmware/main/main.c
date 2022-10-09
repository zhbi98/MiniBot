
#include <string.h>
#include <math.h>

#include "stm32f1xx_hal.h"
#include "time.h"
#include "log.h"
#include "ftos.h"
#include "retarget.h"

#include "MPU6050_I2C.h"
#include "MPU6050.h"
#include "kalman_filter.h"
#include "ano_tech.h"
#include "lv8731v.h"
#include "task.h"

void SystemClock_Config(void);
void TIM3_init();

unsigned int last_count = 0;
unsigned int count = 0;
unsigned int deltaT = 0;

float Med_Angle=-1.00;      // 机械中值，能使得小车真正平衡住的角度 
float Target_Speed=0;     // 期望速度。---二次开发接口，用于控制小车前进后退及其速度。

int Vertical_out,Velocity_out,Turn_out; // 直立环&速度环&转向环的输出变量

float 
  Vertical_Kp=90,
  Vertical_Kd=1.2;      // 直立环Kp、Kd

uint32_t Task_01()
{
    /**
     * Y:tan(pitch) = tan(Axz) = Rx/Rz
     * X:tan(roll)  = tan(Ayz) = Ry/Rz
     */
    read_mpu_data();
    angles.roll = atan2(accel.y, accel.z) * 180.0 / 3.14;
    // pitch = atan2(accel.ax, accel.az) * 180.0 / 3.14;
    angles.pitch = -atan2(accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)) * 180.0 / 3.14;

    deltaT = count - last_count;
    last_count = count;
    roll_kalman_Filter.dt  = deltaT * 10 / 1000.0;
    pitch_kalman_Filter.dt = deltaT * 10 / 1000.0;
    yaw_kalman_Filter.dt   = deltaT * 10 / 1000.0;

    kalman_filter(&roll_kalman_Filter, angles.roll, gyro.x, &angles.roll, &gyro.x);
    kalman_filter(&pitch_kalman_Filter, angles.pitch, gyro.y, &angles.pitch, &gyro.y);
    kalman_filter(&yaw_kalman_Filter, angles.yaw, gyro.z, &angles.yaw, &gyro.z);
    return 0;
}

int Vertical(float Med, float Angle, float gyro_Y) 
{
  int PWM_out;
  
  PWM_out = Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);
  
  return (int)PWM_out;
} 

uint32_t Task_02()
{
    int speed = Vertical(0,angles.roll,gyro.x);
    
    if (speed < 0) {
        lv8731_L_dir(1); // Fall back
        lv8731_R_dir(0); // Fall back
        speed = abs(speed);
    } else {
        lv8731_L_dir(0); // Forward
        lv8731_R_dir(1); // Forward
        speed = abs(speed);
    }

    lv8731_R_speed(speed);
    lv8731_L_speed(speed);
    return 0;
}

uint32_t Task_03()
{
    printf("Task_03\n");
    return 0;
}

int main()
{
    HAL_Init();
    SystemClock_Config();
    usart_init();
    RetargetInit(&UartHandle);
    TIM3_init();
    lv8731v_init();
    MPU_Init();

    SCH_Add_Task(Task_01, 0, 1);
    SCH_Add_Task(Task_02, 0, 1);
    SCH_Add_Task(Task_03, 0, 100);

    for (;;) {
        /* Insert delay 100 ms */
        // HAL_Delay(100);
        SCH_Dispatch_Tasks();
#if 0
        send_sensor_data(
            accel.raw_data_x, accel.raw_data_y, accel.raw_data_z, 
            gyro.raw_data_x, gyro.raw_data_y, gyro.raw_data_z
        );
        send_status_data(
            accel.raw_data_x, accel.raw_data_y, accel.raw_data_z, 
            gyro.raw_data_x, gyro.raw_data_y, gyro.raw_data_z, 
            (int)(angles.roll * 100), 
            (int)(angles.pitch * 100), 
            (int)(angles.yaw * 10)
        );
#endif
#if 0
        ANO_DT_Send_Senser(
            accel.raw_data_x,
            accel.raw_data_y,
            accel.raw_data_z,
            gyro.raw_data_x,
            gyro.raw_data_y,
            gyro.raw_data_z,
            0,
            0,
            0,
            0
        );
        ANO_DT_Send_Status(
            -angles.roll, 
            angles.pitch, 
            -angles.yaw, 
            0, 
            0, 
            0);
#endif
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
        TIMX_IRQHandler_user();
        count++;
    }
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&Tim3Handle);
}
