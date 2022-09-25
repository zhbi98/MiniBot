
#include "ano_tech.h"

/**
 * Example:
 * 
 * MPU_Init();
 * while (mpu_dmp_init());
 * 
 * for (;;) {
 *     MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
 *     MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
 *     temp = MPU_Get_Temperature();
 *
 *     if (!mpu_dmp_get_data(&pitch, &roll, &yaw)) {
 *         send_mpu6050_data(
 *             aacx, aacy, aacz, 
 *             gyrox, gyroy, gyroz
 *         );
 *         send_dmp_data(
 *             aacx, aacy, aacz,
 *             gyrox, gyroy, gyroz,
 *             (int)(roll * 100),
 *             (int)(pitch * 100),
 *             (int)(yaw * 10)
 *         );
 *     }
 * }
 */

void send_format_data(unsigned char f_code, unsigned char * data, unsigned char length)
{
    unsigned char buf[32];

    if (length > 28)
        return;

    buf[length + 3] = 0;

    buf[0] = 0x88;
    buf[1] = 0xaa;

    buf[1] = f_code;
    buf[2] = length;

    for (unsigned char i = 0; i < length; i++)
        buf[i + 3] = data[i];

    for (unsigned char i = 0; i < length + 3; i++)
        buf[length + 3] = buf[length + 3] + buf[i];

    for (unsigned char i = 0; i < length + 4; i++)
        // usart_send_byte(CURRENT_USART, buf[i]);
        while (HAL_UART_Transmit(&UartHandle, &buf[i], 1, 0XFFFF) != HAL_OK);
}

void send_mpu6050_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
    unsigned char buf[12];

    buf[0] = (aacx >> 8) & 0xff;
    buf[1] = aacx & 0xff;
    buf[2] = (aacy >> 8) & 0xff;
    buf[3] = aacy & 0xff;
    buf[4] = (aacz >> 8) & 0xff;
    buf[5] = aacz & 0xff; 
    buf[6] = (gyrox >> 8) & 0xff;
    buf[7] = gyrox & 0xff;
    buf[8] = (gyroy >> 8) & 0xff;
    buf[9] = gyroy & 0xff;
    buf[10]= (gyroz >> 8) & 0xff;
    buf[11]= gyroz & 0xff;

    send_format_data(0xa1, buf, 12);
}

void send_dmp_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
{
    unsigned char buf[28]; 

    memset(buf, 0, 28);

    buf[0]  = (aacx >> 8) & 0xff;
    buf[1]  = aacx & 0xff;

    buf[2]  = (aacy >> 8) & 0xff;
    buf[3]  = aacy & 0xff;

    buf[4]  = (aacz >> 8) & 0xff;
    buf[5]  = aacz & 0xff;

    buf[6]  = (gyrox >> 8) & 0xff;
    buf[7]  = gyrox & 0xff;

    buf[8]  = (gyroy >> 8) & 0xff;
    buf[9]  = gyroy & 0xff;

    buf[10] = (gyroz >> 8) & 0xff;
    buf[11] = gyroz & 0xff;

    buf[18] = (roll >> 8) & 0xff;
    buf[19] = roll & 0xff;

    buf[20] = (pitch >> 8) & 0xff;
    buf[21] = pitch & 0xff;

    buf[22] = (yaw >> 8) & 0xff;
    buf[23] = yaw & 0xff;

    send_format_data(0xaf, buf, 28);
}
