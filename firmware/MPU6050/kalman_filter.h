
#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "MPU6050_I2C.h"
#include "MPU6050.h"
#include "time.h"

struct _accelerometer {
    short raw_data_x;
    short raw_data_y;
    short raw_data_z;
    float x;
    float y;
    float z;
};

struct _gyro {
    short raw_data_x;
    short raw_data_y;
    short raw_data_z;
    float x;
    float y;
    float z;
};

struct _angles {
    float pitch;
    float roll;
    float yaw;
};

struct _recursive {
    float current_estimate;
    float last_estimste;
    float K_gain;
};

struct kalman_filter_t {
    float dt;        // dt 的取值为 kalman 滤波器采样时间
    float angle;     // 角度
    float angle_dot; // 角速度
    float P[2][2];
    float Pdot[4];
    float Q_angle;   // 角度数据置信度
    float Q_gyro;    // 角速度数据置信度
    float R_angle;
    float C_0;
    float q_bias;
    float angle_err;
    float PCt_0;
    float PCt_1;
    float E;
    float K_0;
    float K_1;
    float t_0;
    float t_1;
};

extern struct _accelerometer accel;
extern struct _gyro gyro;
extern struct _angles angles;

extern struct kalman_filter_t roll_kalman_Filter;
extern struct kalman_filter_t pitch_kalman_Filter;
extern struct kalman_filter_t yaw_kalman_Filter;

extern void read_mpu_data();
extern void kalman_filter(struct kalman_filter_t * kf, float angle_m, float gyro_m, float *angle_f, float *angle_dot_f);

#endif
