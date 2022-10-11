
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <math.h>
#include "lv8731v.h"
#include "log.h"
#include "ftos.h"

#define MEDIAN 0

struct _angle_pid {
    float kp;
    float kd;
};

struct _speed_pid {
    float kp;
    float ki;
};

extern int angle_controller(float Med, float Angle, float gyro_Y);
extern int speed_controller();
extern void motor_update_speed(int L_speed, int R_speed);

extern  unsigned int   sys_tick_cnt;
#define SYS_INC_TICK() sys_tick_cnt++
#define GET_SYS_TICK() sys_tick_cnt

#endif
