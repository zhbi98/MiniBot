
#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include <math.h>
#include "MPU6050_I2C.h"
#include "MPU6050.h"
#include "time.h"
#include "log.h"

#define ACC  (2.0 / 32768.0) // Accelerometer full range 2g
#define GYRO (2000.0 / 32768.0) // Gyro full range 2000 deg

#define GYROX_BIAS 110
#define GYROY_BIAS 3
#define GYROZ_BIAS -5

struct _acc {
    short raw_x;
    short raw_y;
    short raw_z;
    float x;
    float y;
    float z;
};

struct _gyro {
    short raw_x;
    short raw_y;
    short raw_z;
    float x;
    float y;
    float z;
};

struct _angle {
    float pitch;
    float roll;
    float yaw;
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

extern struct _acc acc;
extern struct _gyro gyro;
extern struct _angle angle;

extern struct kalman_filter_t roll_kalman_Filter;
extern struct kalman_filter_t pitch_kalman_Filter;
extern struct kalman_filter_t yaw_kalman_Filter;

extern void check_gyro_bias();
extern void read_mpu_data();
extern void kalman_filter(struct kalman_filter_t * kf, float angle_m, float gyro_m, float *angle_f, float *angle_dot_f);
extern void attitude_angle_update();

#endif
