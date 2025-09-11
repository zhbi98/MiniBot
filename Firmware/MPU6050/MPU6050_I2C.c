
#include "MPU6050_I2C.h"

static void mpu6050_sleep_us(unsigned int us)
{
    volatile unsigned int i;

    /**
     * precise delay is best for software i2c bus, but
     * how to determine if the delay is accurate?
     *
     * IO can be set to output mode, so output a 
     * square wave through this IO, and then use an 
     * oscilloscope to measure the period of the 
     * square wave to know.
     * 
     * Example:
     * MCU_IO_OUTPUT_H();
     * delay_us(2);
     * MCU_IO_OUTPUT_L();
     * delay_us(2);
     */
    for (; us > 0; us--)
        for (i = 6; i > 0; i--);
}

void mpu6050_i2c_sleep_time_test()
{
    MPU6050_SCL_L();
    mpu6050_sleep_us(1);
    MPU6050_SCL_H();
    mpu6050_sleep_us(1);
}

void mpu6050_i2c_gpio_init()
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    MPU6050_SCL_CLOCK_ENABLE();

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SCL
    GPIO_InitStruct.Pin = MPU6050_SCL_PIN;
    HAL_GPIO_Init(MPU6050_SCL_GPIO, &GPIO_InitStruct);

    MPU6050_SDA_CLOCK_ENABLE();

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SDA
    GPIO_InitStruct.Pin = MPU6050_SDA_PIN;
    HAL_GPIO_Init(MPU6050_SDA_GPIO, &GPIO_InitStruct);
}

void mpu6050_sda_out_mode()
{
    // gpio_init(MPU6050_SDA_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, MPU6050_SDA_PIN);
    GPIO_InitTypeDef  GPIO_InitStruct;

    HAL_GPIO_DeInit(MPU6050_SDA_GPIO, MPU6050_SDA_PIN);

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SDA
    GPIO_InitStruct.Pin = MPU6050_SDA_PIN;
    HAL_GPIO_Init(MPU6050_SDA_GPIO, &GPIO_InitStruct);
}

void mpu6050_sda_in_mode()
{
    // gpio_init(MPU6050_SDA_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, MPU6050_SDA_PIN);
    GPIO_InitTypeDef  GPIO_InitStruct;

    HAL_GPIO_DeInit(MPU6050_SDA_GPIO, MPU6050_SDA_PIN);

    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SDA
    GPIO_InitStruct.Pin = MPU6050_SDA_PIN;
    HAL_GPIO_Init(MPU6050_SDA_GPIO, &GPIO_InitStruct);
}

void mpu6050_i2c_start()
{
    mpu6050_sda_out_mode();
    MPU6050_SDA_H();
    MPU6050_SCL_H();
    mpu6050_sleep_us(4);
    MPU6050_SDA_L();
    mpu6050_sleep_us(4);
    MPU6050_SCL_L();
}

void mpu6050_i2c_stop()
{
    mpu6050_sda_out_mode();
    MPU6050_SCL_L();
    MPU6050_SDA_L();
    mpu6050_sleep_us(4);
    MPU6050_SCL_H();
    mpu6050_sleep_us(1);
    MPU6050_SDA_H();
    mpu6050_sleep_us(4);
}

unsigned char mpu6050_i2c_wait_ack()
{
    unsigned char times = 0;

    mpu6050_sda_in_mode();
    MPU6050_SDA_H();
    mpu6050_sleep_us(1);     
    MPU6050_SCL_H();
    mpu6050_sleep_us(1); 

    while (READ_MPU6050_SDA()) {
        // times++;
        if (times > 255) {
            mpu6050_i2c_stop();
            MPU6050_SCL_L();
            return 1;
        }
    }
    MPU6050_SCL_L();
    return 0;  
}

void mpu6050_i2c_ack()
{
    MPU6050_SCL_L();
    mpu6050_sda_out_mode();
    MPU6050_SDA_L();
    mpu6050_sleep_us(2);
    MPU6050_SCL_H();
    mpu6050_sleep_us(2);
    MPU6050_SCL_L();
}

void mpu6050_i2c_nack()
{
    MPU6050_SCL_L();
    mpu6050_sda_out_mode();
    MPU6050_SDA_H();
    mpu6050_sleep_us(2);
    MPU6050_SCL_H();
    mpu6050_sleep_us(2);
    MPU6050_SCL_L();
}

void mpu6050_i2c_send_byte(unsigned char a_byte)
{                        
    unsigned char send;

    mpu6050_sda_out_mode();
    MPU6050_SCL_L();
    for (send = 0; send < 8; send++) {              
        if ((a_byte & 0x80) >> 7)
            MPU6050_SDA_H();
        else
            MPU6050_SDA_L();
        a_byte <<= 1;
        mpu6050_sleep_us(2);
        MPU6050_SCL_H();
        mpu6050_sleep_us(2); 
        MPU6050_SCL_L();
        mpu6050_sleep_us(2);
    }    
} 

unsigned char mpu6050_i2c_read_byte(unsigned char ack)
{
    unsigned char i, receive = 0;

    mpu6050_sda_in_mode();
    for (i = 0; i < 8; i++) {
        MPU6050_SCL_L();
        mpu6050_sleep_us(2);
        MPU6050_SCL_H();
        receive <<= 1;
        if (READ_MPU6050_SDA())
            receive++;
        mpu6050_sleep_us(1); 
    }

    if (!ack)
        mpu6050_i2c_nack();
    else
        mpu6050_i2c_ack();
    return receive;
}
