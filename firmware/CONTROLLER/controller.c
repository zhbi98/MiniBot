
#include "controller.h"

unsigned int sys_tick_cnt = 0;

float speed_buf[FILTER_COUNT];

float average_filter(int left_speed, int right_speed)
{
    double speed_sum = 0;

    for (unsigned int i = 1; i < FILTER_COUNT; i++)
        speed_buf[i - 1] = speed_buf[i];

    speed_buf[FILTER_COUNT - 1] = (left_speed + right_speed) / 2.0 / SPEED_SCALE;

    for (unsigned int i = 0 ; i < FILTER_COUNT; i++)
        speed_sum += speed_buf[i];

    return speed_sum / FILTER_COUNT;
}

void motor_update(short left_speed, short right_speed)
{
    unsigned int left_speed_abs, right_speed_abs;

    // speed < 0 : Fall back
    // speed > 0 : Forward

    (left_speed < 0) ? lv8731_L_dir(1) : lv8731_L_dir(0);
    (right_speed < 0) ? lv8731_R_dir(0) : lv8731_R_dir(1);

    left_speed_abs  = (left_speed < 0) ? left_speed * (-1) : left_speed * 1;
    right_speed_abs = (right_speed < 0) ? right_speed * (-1) : right_speed * 1;

    lv8731_L_speed(left_speed_abs);
    lv8731_R_speed(right_speed_abs);
}

int vertical(float med, float angle, float gyro_y)
{
    int pwm_out;

    pwm_out = ANGLE_KP * (angle - med) + ANGLE_KD * (gyro_y - 0);

    return pwm_out;
} 
