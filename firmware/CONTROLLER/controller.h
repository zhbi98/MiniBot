
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <math.h>
#include "lv8731v.h"
#include "log.h"
#include "ftos.h"
#include "kalman_filter.h"

#define BALANCE_ANGLE 0.67f

struct _angle_pid {
    float kp;
    float kd;
    float last_err;
    float current_err;
    float deltaT;
};

struct _speed_pid {
    float kp;
    float ki;
    float last_err;
    float current_err;
    float deltaT;
};

extern int balance_angle_control(float target_angle, float angle, float gyrox);
extern int balance_speed_control();
extern void motor_update_speed(int L_speed, int R_speed);

extern  unsigned int   sys_tick_cnt;
#define SYS_INC_TICK() sys_tick_cnt++
#define GET_SYS_TICK() sys_tick_cnt

#endif
