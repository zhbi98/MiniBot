
#include "qmc5883_i2c.h"

static void qmc5883_sleep_us(unsigned int us)
{
    unsigned int i;

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

void qmc5883_i2c_sleep_time_test()
{
    QMC5883_SCL_L();
    qmc5883_sleep_us(1);
    QMC5883_SCL_H();
    qmc5883_sleep_us(1);
}

void qmc5883_i2c_gpio_init()
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    QMC5883_SCL_CLOCK_ENABLE();

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SCL
    GPIO_InitStruct.Pin = QMC5883_SCL_PIN;
    HAL_GPIO_Init(QMC5883_SCL_GPIO, &GPIO_InitStruct);

    QMC5883_SDA_CLOCK_ENABLE();

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SDA
    GPIO_InitStruct.Pin = QMC5883_SDA_PIN;
    HAL_GPIO_Init(QMC5883_SDA_GPIO, &GPIO_InitStruct);
}

void qmc5883_sda_out_mode()
{
    // gpio_init(QMC5883_SDA_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, QMC5883_SDA_PIN);
    GPIO_InitTypeDef  GPIO_InitStruct;

    HAL_GPIO_DeInit(QMC5883_SDA_GPIO, QMC5883_SDA_PIN);

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SDA
    GPIO_InitStruct.Pin = QMC5883_SDA_PIN;
    HAL_GPIO_Init(QMC5883_SDA_GPIO, &GPIO_InitStruct);
}

void qmc5883_sda_in_mode()
{
    // gpio_init(QMC5883_SDA_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, QMC5883_SDA_PIN);
    GPIO_InitTypeDef  GPIO_InitStruct;

    HAL_GPIO_DeInit(QMC5883_SDA_GPIO, QMC5883_SDA_PIN);

    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SDA
    GPIO_InitStruct.Pin = QMC5883_SDA_PIN;
    HAL_GPIO_Init(QMC5883_SDA_GPIO, &GPIO_InitStruct);
}

void qmc5883_i2c_start()
{
    qmc5883_sda_out_mode();
    QMC5883_SDA_H();
    QMC5883_SCL_H();
    qmc5883_sleep_us(4);
    QMC5883_SDA_L();
    qmc5883_sleep_us(4);
    QMC5883_SCL_L();
}

void qmc5883_i2c_stop()
{
    qmc5883_sda_out_mode();
    QMC5883_SCL_L();
    QMC5883_SDA_L();
    qmc5883_sleep_us(4);
    QMC5883_SCL_H();
    qmc5883_sleep_us(1);
    QMC5883_SDA_H();
    qmc5883_sleep_us(4);
}

unsigned char qmc5883_i2c_wait_ack()
{
    unsigned char times = 0;

    qmc5883_sda_in_mode();
    QMC5883_SDA_H();
    qmc5883_sleep_us(1);     
    QMC5883_SCL_H();
    qmc5883_sleep_us(1); 

    while (READ_QMC5883_SDA()) {
        // times++;
        if (times > 255) {
            qmc5883_i2c_stop();
            QMC5883_SCL_L();
            return 1;
        }
    }
    QMC5883_SCL_L();
    return 0;  
}

void qmc5883_i2c_ack()
{
    QMC5883_SCL_L();
    qmc5883_sda_out_mode();
    QMC5883_SDA_L();
    qmc5883_sleep_us(2);
    QMC5883_SCL_H();
    qmc5883_sleep_us(2);
    QMC5883_SCL_L();
}

void qmc5883_i2c_nack()
{
    QMC5883_SCL_L();
    qmc5883_sda_out_mode();
    QMC5883_SDA_H();
    qmc5883_sleep_us(2);
    QMC5883_SCL_H();
    qmc5883_sleep_us(2);
    QMC5883_SCL_L();
}

void qmc5883_i2c_send_byte(unsigned char a_byte)
{                        
    unsigned char send;

    qmc5883_sda_out_mode();
    QMC5883_SCL_L();
    for (send = 0; send < 8; send++) {              
        if ((a_byte & 0x80) >> 7)
            QMC5883_SDA_H();
        else
            QMC5883_SDA_L();
        a_byte <<= 1;
        qmc5883_sleep_us(2);
        QMC5883_SCL_H();
        qmc5883_sleep_us(2); 
        QMC5883_SCL_L();
        qmc5883_sleep_us(2);
    }    
} 

unsigned char qmc5883_i2c_read_byte(unsigned char ack)
{
    unsigned char i, receive = 0;

    qmc5883_sda_in_mode();
    for (i = 0; i < 8; i++) {
        QMC5883_SCL_L();
        qmc5883_sleep_us(2);
        QMC5883_SCL_H();
        receive <<= 1;
        if (READ_QMC5883_SDA())
            receive++;
        qmc5883_sleep_us(1); 
    }

    if (!ack)
        qmc5883_i2c_nack();
    else
        qmc5883_i2c_ack();
    return receive;
}
