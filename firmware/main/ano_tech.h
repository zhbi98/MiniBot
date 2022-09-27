
#ifndef __ANO_TECH_H__
#define __ANO_TECH_H__

#include "stm32f1xx_hal.h"
#include "ano_tech.h"
#include "usart.h"
#include <string.h>

extern void send_format_data(unsigned char f_code, unsigned char * data, unsigned char length);
extern void send_mpu6050_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz);
extern void send_dmp_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw);

extern void ANO_DT_Send_Senser(short a_x,short a_y,short a_z,short g_x,short g_y,short g_z,short m_x,short m_y,short m_z,int bar);
extern void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int alt, unsigned char fly_model, unsigned char armed);

#endif
