
#include "ano_tech.h"

/**
 * Example:
 * 
 * MPU_Init();
 * while (mpu_dmp_init());
 * 
 * MPU_Get_Accelerometer(&acc.raw_x, &acc.raw_y, &acc.raw_z);
 * MPU_Get_Gyroscope(&gyro.raw_x, &gyro.raw_y, &gyro.raw_z);
 * temp = MPU_Get_Temperature();
 *
 * if (!mpu_dmp_get_data(&pitch, &roll, &yaw)) {
 *     send_sensor_data(
 *         acc.raw_x, acc.raw_y, acc.raw_z, 
 *         gyro.raw_x, gyro.raw_y, gyro.raw_z
 *     );
 *     send_status_data(
 *         acc.raw_x, acc.raw_y, acc.raw_z,
 *         gyro.raw_x, gyro.raw_y, gyro.raw_z,
 *         (int)(roll * 100),
 *         (int)(pitch * 100),
 *         (int)(yaw * 10)
 *     );
 * }
 */

void send_frames(unsigned char f_code, unsigned char * data, unsigned char length)
{
    unsigned char buf[32];

    if (length > 28)
        return;

    buf[length + 3] = 0;

    buf[0] = 0x88;
    buf[1] = 0xAA;
    buf[1] = f_code;
    buf[2] = length;

    for (unsigned char i = 0; i < length; i++)
        buf[i + 3] = data[i];
    for (unsigned char i = 0; i < length + 3; i++)
        buf[length + 3] = buf[length + 3] + buf[i];
    for (unsigned char i = 0; i < length + 4; i++)
        USART_SEND_DATA(&buf[i], 1);
}

void send_sensor_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
    unsigned char buf[12];

    buf[0]  = (aacx >> 8) & 0xFF;
    buf[1]  = aacx & 0xFF;
    buf[2]  = (aacy >> 8) & 0xFF;
    buf[3]  = aacy & 0xFF;
    buf[4]  = (aacz >> 8) & 0xFF;
    buf[5]  = aacz & 0xFF; 
    buf[6]  = (gyrox >> 8) & 0xFF;
    buf[7]  = gyrox & 0xFF;
    buf[8]  = (gyroy >> 8) & 0xFF;
    buf[9]  = gyroy & 0xFF;
    buf[10] = (gyroz >> 8) & 0xFF;
    buf[11] = gyroz & 0xFF;

    send_frames(0xA1, buf, 12);
}

void send_status_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
{
    unsigned char buf[28]; 

    memset(buf, 0, 28);

    buf[0]  = (aacx >> 8) & 0xFF;
    buf[1]  = aacx & 0xFF;

    buf[2]  = (aacy >> 8) & 0xFF;
    buf[3]  = aacy & 0xFF;

    buf[4]  = (aacz >> 8) & 0xFF;
    buf[5]  = aacz & 0xFF;

    buf[6]  = (gyrox >> 8) & 0xFF;
    buf[7]  = gyrox & 0xFF;

    buf[8]  = (gyroy >> 8) & 0xFF;
    buf[9]  = gyroy & 0xFF;

    buf[10] = (gyroz >> 8) & 0xFF;
    buf[11] = gyroz & 0xFF;

    buf[18] = (roll >> 8) & 0xFF;
    buf[19] = roll & 0xFF;

    buf[20] = (pitch >> 8) & 0xFF;
    buf[21] = pitch & 0xFF;

    buf[22] = (yaw >> 8) & 0xFF;
    buf[23] = yaw & 0xFF;

    send_frames(0xAF, buf, 28);
}

/**
 * Example:
 * 
 * ANO_DT_Send_Senser(
 *     acc.raw_x,
 *     acc.raw_y,
 *     acc.raw_z,
 *     gyro.raw_x,
 *     gyro.raw_y,
 *     gyro.raw_z,
 *     0,
 *     0,
 *     0,
 *     0
 * );
 *
 * ANO_DT_Send_Status(
 *     -angles.roll, 
 *     angles.pitch, 
 *     -angles.yaw, 
 *     0, 
 *     0, 
 *     0
 * );
 */

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

unsigned char data_to_send[50]; // Wait send data buf

void ANO_DT_Send_Senser(short a_x, short a_y, short a_z, short g_x, short g_y, short g_z, short m_x, short m_y, short m_z, int bar)
{
    unsigned char _cnt = 0;
    short _temp;
    
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0;

    _temp = a_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = a_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = a_z;    
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    
    _temp = g_x;    
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = g_y;    
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = g_z;    
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    
    _temp = m_x;    
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = m_y;    
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = m_z;    
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    
    data_to_send[3] = _cnt - 4;
    
    unsigned char sum = 0;
    for(unsigned char i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    
    // ANO_DT_Send_Data(data_to_send, _cnt);
    USART_SEND_DATA(data_to_send, _cnt);
}

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int alt, unsigned char fly_model, unsigned char armed)
{
    unsigned char _cnt = 0;
    short _temp;
    int _temp2 = alt;
    
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0;
    
    _temp = (int)(angle_rol * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(angle_pit * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(angle_yaw * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    
    data_to_send[_cnt++] = BYTE3(_temp2);
    data_to_send[_cnt++] = BYTE2(_temp2);
    data_to_send[_cnt++] = BYTE1(_temp2);
    data_to_send[_cnt++] = BYTE0(_temp2);
    
    data_to_send[_cnt++] = fly_model;
    
    data_to_send[_cnt++] = armed;
    
    data_to_send[3] = _cnt - 4;
    
    unsigned char sum = 0;
    for(unsigned char i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    
    // Usart2_Send(data_to_send, _cnt);
    USART_SEND_DATA(data_to_send, _cnt);
}
