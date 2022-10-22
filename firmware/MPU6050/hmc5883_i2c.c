
#include "hmc5883_i2c.h"

static void hmc5883_sleep_us(unsigned int us)
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

void hmc5883_i2c_sleep_time_test()
{
    HMC5883_SCL_L();
    hmc5883_sleep_us(1);
    HMC5883_SCL_H();
    hmc5883_sleep_us(1);
}

void hmc5883_i2c_gpio_init()
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    HMC5883_SCL_CLOCK_ENABLE();

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SCL
    GPIO_InitStruct.Pin = HMC5883_SCL_PIN;
    HAL_GPIO_Init(HMC5883_SCL_GPIO, &GPIO_InitStruct);

    HMC5883_SDA_CLOCK_ENABLE();

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SDA
    GPIO_InitStruct.Pin = HMC5883_SDA_PIN;
    HAL_GPIO_Init(HMC5883_SDA_GPIO, &GPIO_InitStruct);
}

void hmc5883_sda_out_mode()
{
    // gpio_init(HMC5883_SDA_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, HMC5883_SDA_PIN);
    GPIO_InitTypeDef  GPIO_InitStruct;

    HAL_GPIO_DeInit(HMC5883_SDA_GPIO, HMC5883_SDA_PIN);

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SDA
    GPIO_InitStruct.Pin = HMC5883_SDA_PIN;
    HAL_GPIO_Init(HMC5883_SDA_GPIO, &GPIO_InitStruct);
}

void hmc5883_sda_in_mode()
{
    // gpio_init(HMC5883_SDA_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, HMC5883_SDA_PIN);
    GPIO_InitTypeDef  GPIO_InitStruct;

    HAL_GPIO_DeInit(HMC5883_SDA_GPIO, HMC5883_SDA_PIN);

    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SDA
    GPIO_InitStruct.Pin = HMC5883_SDA_PIN;
    HAL_GPIO_Init(HMC5883_SDA_GPIO, &GPIO_InitStruct);
}

void hmc5883_i2c_start()
{
    hmc5883_sda_out_mode();
    HMC5883_SDA_H();
    HMC5883_SCL_H();
    hmc5883_sleep_us(4);
    HMC5883_SDA_L();
    hmc5883_sleep_us(4);
    HMC5883_SCL_L();
}

void hmc5883_i2c_stop()
{
    hmc5883_sda_out_mode();
    HMC5883_SCL_L();
    HMC5883_SDA_L();
    hmc5883_sleep_us(4);
    HMC5883_SCL_H();
    hmc5883_sleep_us(1);
    HMC5883_SDA_H();
    hmc5883_sleep_us(4);
}

unsigned char hmc5883_i2c_wait_ack()
{
    unsigned char times = 0;

    hmc5883_sda_in_mode();
    HMC5883_SDA_H();
    hmc5883_sleep_us(1);     
    HMC5883_SCL_H();
    hmc5883_sleep_us(1); 

    while (READ_HMC5883_SDA()) {
        // times++;
        if (times > 255) {
            hmc5883_i2c_stop();
            HMC5883_SCL_L();
            return 1;
        }
    }
    HMC5883_SCL_L();
    return 0;  
}

void hmc5883_i2c_ack()
{
    HMC5883_SCL_L();
    hmc5883_sda_out_mode();
    HMC5883_SDA_L();
    hmc5883_sleep_us(2);
    HMC5883_SCL_H();
    hmc5883_sleep_us(2);
    HMC5883_SCL_L();
}

void hmc5883_i2c_nack()
{
    HMC5883_SCL_L();
    hmc5883_sda_out_mode();
    HMC5883_SDA_H();
    hmc5883_sleep_us(2);
    HMC5883_SCL_H();
    hmc5883_sleep_us(2);
    HMC5883_SCL_L();
}

void hmc5883_i2c_send_byte(unsigned char a_byte)
{                        
    unsigned char send;

    hmc5883_sda_out_mode();
    HMC5883_SCL_L();
    for (send = 0; send < 8; send++) {              
        if ((a_byte & 0x80) >> 7)
            HMC5883_SDA_H();
        else
            HMC5883_SDA_L();
        a_byte <<= 1;
        hmc5883_sleep_us(2);
        HMC5883_SCL_H();
        hmc5883_sleep_us(2); 
        HMC5883_SCL_L();
        hmc5883_sleep_us(2);
    }    
} 

unsigned char hmc5883_i2c_read_byte(unsigned char ack)
{
    unsigned char i, receive = 0;

    hmc5883_sda_in_mode();
    for (i = 0; i < 8; i++) {
        HMC5883_SCL_L();
        hmc5883_sleep_us(2);
        HMC5883_SCL_H();
        receive <<= 1;
        if (READ_HMC5883_SDA())
            receive++;
        hmc5883_sleep_us(1); 
    }

    if (!ack)
        hmc5883_i2c_nack();
    else
        hmc5883_i2c_ack();
    return receive;
}
