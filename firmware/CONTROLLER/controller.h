
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <math.h>
#include "lv8731v.h"
#include "log.h"
#include "ftos.h"
#include "kalman_filter.h"

#define BALANCE_ANGLE 0.0f/*0.67f*/
#define FILTER_COUNT 20
#define SPEED_SCALE 53.33f

#define ANGLE_KP 720.0f
#define ANGLE_KD 0.0f

extern  unsigned int   sys_tick_cnt;
#define SYS_INC_TICK() sys_tick_cnt++
#define GET_SYS_TICK() sys_tick_cnt

extern float speed_buf[];

extern void motor_update(short left_speed, short right_speed);
extern int vertical(float med, float angle, float gyro_y);

#endif
