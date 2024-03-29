
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <math.h>
#include "lv8731v.h"
#include "log.h"
#include "ftos.h"
#include "bluetooth.h"

#define BALANCE_ANGLE 0.55f
#define SPEED_FILTER_COUNT 20

#define PLUSE_TO_RPM(pluse, angle, div) \
	((float)pluse * (float)(angle / div) / 360.0 * 60.0)

#define ANGLE_KP 300.0f
#define ANGLE_KD 0.0f

#define SPEED_KP 16.6f
#define SPEED_KI (SPEED_KP / 200.0)

#define TURN_KP 10.0f

#define SPEED_MAX 5500

extern  unsigned int   sys_tick_cnt;
#define SYS_INC_TICK() sys_tick_cnt++
#define GET_SYS_TICK() sys_tick_cnt

struct _speed {
	int left_speed;
	int right_speed;
};

extern float speed_buf[];
extern struct _speed motor_speed;

extern float speed_average_filter(int left_speed, int right_speed);
extern void motor_update(short left_speed, short right_speed);
extern int vertical(float med, float angle, float gyro_y);
extern int velocity(int left_speed, int right_speed);
extern int turn(int left_speed, int right_speed, float gyro_y);

#endif
