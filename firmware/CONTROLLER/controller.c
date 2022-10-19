
#include "controller.h"

unsigned int sys_tick_cnt = 0;

float speed_buf[SPEED_FILTER_COUNT];
struct _speed motor_speed;

float speed_average_filter(int left_speed, int right_speed)
{
    double speed_sum = 0;

    for (unsigned int i = 1; i < SPEED_FILTER_COUNT; i++)
        speed_buf[i - 1] = speed_buf[i];

    speed_buf[SPEED_FILTER_COUNT - 1] = PLUSE_TO_RPM(
                                    (left_speed + right_speed) * 2.4 / 2.0, 
                                    1.8, 
                                    16);

    for (unsigned int i = 0 ; i < SPEED_FILTER_COUNT; i++)
        speed_sum += speed_buf[i];

    return speed_sum / SPEED_FILTER_COUNT;
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
    int vertical;

    vertical = ANGLE_KP * (angle - med) + ANGLE_KD * (gyro_y - 0);

    return vertical;
} 

int velocity(int left_speed, int right_speed)
{  
    static float speed = 0.0;
    static float speed_out = 0.0;
    static float speed_sum = 0.0;

    speed_out = speed_average_filter(left_speed, right_speed);

    speed *= 0.7;
    speed += speed_out * 0.3;
    speed_sum += speed;

    speed_sum = (speed_sum >  6000.0) ?  6000.0 : speed_sum;
    speed_sum = (speed_sum < -6000.0) ? -6000.0 : speed_sum;

    int velocity = (speed - 0) * SPEED_KP + speed_sum * SPEED_KI;

    return velocity;
}

int turn(int left_speed, int right_speed, float gyro_y)
{
    int turn;

    turn = (gyro_y - 0) * TURN_KP;

    return turn;
}
