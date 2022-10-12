
#include "controller.h"

unsigned int sys_tick_cnt = 0;

struct _angle_pid angle_pid = {
    .kp = -650.0,
    .kd = 12.0,
};

struct _speed_pid speed_pid = {
    .kp = 00.0,
    .ki = 0.1,
};

int balance_angle_control(float target_angle, float angle, float gyrox)
{
    static unsigned int last_tick = 0;
    unsigned int tick = 0;
    static float last_angle = 0.0;
    float pulse = 0.0;

    tick = GET_SYS_TICK() - last_tick;
    last_tick = GET_SYS_TICK();
    angle_pid.deltaT = tick * 5.0 / 1000.0;
    if (angle_pid.deltaT < 0.005)
        angle_pid.deltaT = 0.005;

    angle_pid.current_err = target_angle - angle;
    pulse = angle_pid.kp * angle_pid.current_err + angle_pid.kd * gyrox /*(angle - last_angle) / angle_pid.deltaT*/;

    // info("Gyrox1:%s", double_string(gyrox, 2));
    // info("Gyrox2:%s", double_string((angle - last_angle) / angle_pid.deltaT, 2));

    last_angle = angle;
    angle_pid.last_err = angle_pid.current_err;

    return (int)pulse;
}

int balance_speed_control()
{

}

void motor_update_speed(int L_speed, int R_speed)
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
