/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     qmc5883.c
 * @说明     QMC5883地磁传感器驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include "qmc5883.h"
// #include "drv_i2c_soft.h"

#define QMC5883L_Addr     0x1A
#define QMC5883L_HX_L     0x00
#define QMC5883L_HX_H     0x01
#define QMC5883L_HY_L     0x02
#define QMC5883L_HY_H     0x03
#define QMC5883L_HZ_L     0x04
#define QMC5883L_HZ_H     0x05
#define QMC5883L_CTR1     0x09
#define QMC5883L_SRPERIOD 0x0B

/**********************************************************************************************************
*函 数 名: Soft_I2c_Single_Write
*功能说明: 单个寄存器写入
*形    参: 设备号 从机地址 寄存器地址 写入数据
*返 回 值: 写入状态
**********************************************************************************************************/
bool Soft_I2c_Single_Write(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data)
{
    // if(!Soft_I2c_Start(deviceNum))
    //     return false;
    qmc5883_i2c_start();

    //Soft_I2c_SendByte(deviceNum, SlaveAddress);
    qmc5883_i2c_send_byte(SlaveAddress);

    // if(!Soft_I2c_WaitAck(deviceNum))
    // {
    //     Soft_I2c_Stop(deviceNum);
    //     return false;
    // }
    if(qmc5883_i2c_wait_ack())  //等待应答
    {
        qmc5883_i2c_stop();
        return false;
    }

    // Soft_I2c_SendByte(deviceNum, REG_Address);
    qmc5883_i2c_send_byte(REG_Address);

    // Soft_I2c_WaitAck(deviceNum);
    qmc5883_i2c_wait_ack();

    // Soft_I2c_SendByte(deviceNum, REG_data);
    qmc5883_i2c_send_byte(REG_data);

    // Soft_I2c_WaitAck(deviceNum);
    qmc5883_i2c_wait_ack();

    // Soft_I2c_Stop(deviceNum);
    qmc5883_i2c_stop();

    return true;
}

/**********************************************************************************************************
*函 数 名: Soft_I2c_Single_Read
*功能说明: 单个寄存器读取
*形    参: 设备号 从机地址 寄存器地址
*返 回 值: 读出数据
**********************************************************************************************************/
uint8_t Soft_I2C_Single_Read(uint8_t SlaveAddress, uint8_t REG_Address)
{
    uint8_t REG_data;

    // if(!Soft_I2c_Start(deviceNum))
    //     return false;
    qmc5883_i2c_start();

    // Soft_I2c_SendByte(deviceNum, SlaveAddress);
    qmc5883_i2c_send_byte(SlaveAddress);

    // if(!Soft_I2c_WaitAck(deviceNum))
    // {
    //     Soft_I2c_Stop(deviceNum);
    //     return false;
    // }
    if(qmc5883_i2c_wait_ack())  //等待应答
    {
        qmc5883_i2c_stop();
        return false;
    }

    // Soft_I2c_SendByte(deviceNum, (uint8_t)REG_Address);
    qmc5883_i2c_send_byte((uint8_t)REG_Address);

    // Soft_I2c_WaitAck(deviceNum);
    qmc5883_i2c_wait_ack();

    // Soft_I2c_Start(deviceNum);
    qmc5883_i2c_start();

    // Soft_I2c_SendByte(deviceNum, SlaveAddress+1);
    qmc5883_i2c_send_byte(SlaveAddress + 1);

    // Soft_I2c_WaitAck(deviceNum);
    qmc5883_i2c_wait_ack();

    // REG_data = Soft_I2c_ReadByte(deviceNum);
    REG_data = qmc5883_i2c_read_byte(0);

    // Soft_I2c_NoAck(deviceNum);
    // qmc5883_i2c_nack(); // qmc5883_i2c_read_byte(0) 内部支持发送 nack

    // Soft_I2c_Stop(deviceNum);
    qmc5883_i2c_stop();

    return REG_data;
}

/**********************************************************************************************************
*函 数 名: Soft_I2C_Multi_Read
*功能说明: 多个寄存器连续读取
*形    参: 设备号 从机地址 寄存器地址 读出缓冲区指针 读出长度
*返 回 值: 读取状态
**********************************************************************************************************/
bool Soft_I2C_Multi_Read(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t * ptChar, uint8_t size)
{
    uint8_t i;

    if(size < 1)
        return false;
    // if(!Soft_I2c_Start(deviceNum))
    //     return false;
    qmc5883_i2c_start();

    // Soft_I2c_SendByte(deviceNum, SlaveAddress);
    qmc5883_i2c_send_byte(SlaveAddress);

    // if(!Soft_I2c_WaitAck(deviceNum))
    // {
    //     Soft_I2c_Stop(deviceNum);
    //     return false;
    // }
    if(qmc5883_i2c_wait_ack())  //等待应答
    {
        qmc5883_i2c_stop();
        return false;
    }

    // Soft_I2c_SendByte(deviceNum, REG_Address);
    qmc5883_i2c_send_byte(REG_Address);

    // if(!Soft_I2c_WaitAck(deviceNum))
    // {
    //     Soft_I2c_Stop(deviceNum);
    //     return false;
    // }
    if(qmc5883_i2c_wait_ack())  //等待应答
    {
        qmc5883_i2c_stop();
        return false;
    }

    // Soft_I2c_Start(deviceNum);
    qmc5883_i2c_start();

    // Soft_I2c_SendByte(deviceNum, SlaveAddress+1);
    qmc5883_i2c_send_byte(SlaveAddress + 1);

    // if(!Soft_I2c_WaitAck(deviceNum))
    // {
    //     Soft_I2c_Stop(deviceNum);
    //     return false;
    // }
    if(qmc5883_i2c_wait_ack())  //等待应答
    {
        qmc5883_i2c_stop();
        return false;
    }

    for(i=1; i<size; i++)
    {
        // *ptChar++ = Soft_I2c_ReadByte(deviceNum);
        // Soft_I2c_Ack(deviceNum); // qmc5883_i2c_read_byte(1) 内部支持发送 ack
        *ptChar++ = qmc5883_i2c_read_byte(1);
    }
    // *ptChar++ = Soft_I2c_ReadByte(deviceNum);
    // Soft_I2c_NoAck(deviceNum); // qmc5883_i2c_read_byte(0) 内部支持发送 nack
    *ptChar++ = qmc5883_i2c_read_byte(0);

    // Soft_I2c_Stop(deviceNum);
    qmc5883_i2c_stop();

    return true;
}

/**********************************************************************************************************
*函 数 名: QMC5883_WriteReg
*功能说明: 往QMC5883的寄存器写入一个字节的数据
*形    参: 寄存器地址 写入数据
*返 回 值: 无
**********************************************************************************************************/
static void QMC5883_WriteReg(uint8_t REG_Address, uint8_t REG_data)
{
    Soft_I2c_Single_Write(QMC5883L_Addr, REG_Address, REG_data);
}

/**********************************************************************************************************
*函 数 名: QMC5883_ReadReg
*功能说明: 读取QMC5883寄存器的数据
*形    参: 寄存器地址
*返 回 值: 寄存器数据
**********************************************************************************************************/
static uint8_t QMC5883_ReadReg(uint8_t REG_Address)
{
    return Soft_I2C_Single_Read(QMC5883L_Addr, REG_Address);
}

/**********************************************************************************************************
*函 数 名: QMC5883_MultiRead
*功能说明: 连续读取QMC5883寄存器的数据
*形    参: 寄存器地址 读出缓冲区 长度
*返 回 值: 成功标志位
**********************************************************************************************************/
static bool QMC5883_MultiRead(uint8_t REG_Address, uint8_t* buffer, uint8_t length)
{
    return Soft_I2C_Multi_Read(QMC5883L_Addr, REG_Address, buffer, length);
}

/**********************************************************************************************************
*函 数 名: QMC5883_Detect
*功能说明: 检测QMC5883是否存在
*形    参: 无
*返 回 值: 存在状态
**********************************************************************************************************/
bool QMC5883_Detect(void)
{
    QMC5883_WriteReg(QMC5883L_SRPERIOD, 0x01);
    QMC5883_WriteReg(QMC5883L_CTR1, 0x0D); // Full Range:2Guass, Output Data Rate:200Hz
    sleep_ms(50);

    if(QMC5883_ReadReg(QMC5883L_CTR1) == 0x0D)
        return true;
    else
        return false;
}

/**********************************************************************************************************
*函 数 名: QMC5883_Init
*功能说明: QMC5883寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void QMC5883_Init(void)
{
    qmc5883_i2c_gpio_init();

    QMC5883_WriteReg(QMC5883L_SRPERIOD, 0x01);
    sleep_ms(5);
    QMC5883_WriteReg(QMC5883L_Addr, 0x20, 0x40);
    sleep_ms(5);
    QMC5883_WriteReg(QMC5883L_Addr, 0x21, 0x01);
    sleep_ms(5);
    QMC5883_WriteReg(QMC5883L_CTR1, 0x0D); // Full Range:2Guass, Output Data Rate:200Hz
}

/**********************************************************************************************************
*函 数 名: QMC5883_Read
*功能说明: 读取地磁传感器数据
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void QMC5883_Read(int16_t * mx, int16_t * my, int16_t * mz)
{
    uint8_t buffer[6];
    
    if (!QMC5883_MultiRead(QMC5883L_HX_L, buffer, 6))
        return;

    *mx = (int16_t)buffer[1] << 8 | buffer[0];
    *my = (int16_t)buffer[3] << 8 | buffer[2];
    *mz = (int16_t)buffer[5] << 8 | buffer[4];
}
