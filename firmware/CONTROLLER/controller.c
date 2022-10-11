
#include "controller.h"

unsigned int sys_tick_cnt = 0;

const struct _angle_pid angle_pid = {
    .kp = 300.0,
    .kd = 5.0,
};

const struct _speed_pid speed_pid = {
    .kp = 00.0,
    .ki = 0.1,
};

int angle_controller(float Med, float Angle, float gyro_Y)
{
  int PWM_out;
  
  PWM_out = angle_pid.kp * (Angle - Med) + angle_pid.kd * (gyro_Y - 0);
  
  return (int)PWM_out;
}

int speed_controller()
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
