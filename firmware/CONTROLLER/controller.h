
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <math.h>
#include "lv8731v.h"

#define MEDIAN -0.5

struct _pid_angle {
    float kp;
    float kd;
};

extern unsigned int sys_tick_cnt;

extern int PID_Angle(float target, float angle);
extern int PID_Speed();
extern void motor_driver(int L_speed, int R_speed);

extern unsigned int get_sys_tick();

#endif
