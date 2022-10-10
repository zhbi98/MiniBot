
#include "controller.h"

struct _pid_angle pid_angle = {
    .kp = 200.0,
    .kd = 1.9,
};

unsigned int sys_tick_cnt = 0;

int PID_Angle(float target, float angle)
{
    float out;

    out = pid_angle.kp * (angle - target) + pid_angle.kd * (angle - target);
    return (int)out;
}

int PID_Speed()
{

}

void motor_driver(int L_speed, int R_speed)
{
    // speed < 0 : Fall back
    // speed > 0 : Forward

    (L_speed < 0) ? lv8731_L_dir(1) : lv8731_L_dir(0);
    (R_speed < 0) ? lv8731_R_dir(0) : lv8731_R_dir(1);

    L_speed = (L_speed < 0) ? -L_speed : L_speed;
    R_speed = (R_speed < 0) ? -R_speed : R_speed;

    lv8731_R_speed(R_speed);
    lv8731_L_speed(L_speed);
}

unsigned int get_sys_tick()
{
    return sys_tick_cnt;
}
