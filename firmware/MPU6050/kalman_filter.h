
#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include <math.h>
#include "MPU6050_I2C.h"
#include "MPU6050.h"
#include "time.h"
#include "log.h"
#include "controller.h"

#define PI 3.1416f

#define ACC  (2.0 / 32768.0) // Accelerometer full range 2g
#define GYRO (2000.0 / 32768.0) // Gyro full range 2000 deg

#if 1 /** MPU MODULE */
#define ACCX_BIAS 0
#define ACCY_BIAS 0
#define ACCZ_BIAS 0

#define GYROX_BIAS 110
#define GYROY_BIAS 3
#define GYROZ_BIAS -5
#endif

#if 0 /** MINIBOT */
#define ACCX_BIAS 0
#define ACCY_BIAS 0
#define ACCZ_BIAS 0

#define GYROX_BIAS -8
#define GYROY_BIAS -6
#define GYROZ_BIAS -3
#endif

struct _mpu_raw {
    short accx;
    short accy;
    short accz;

    short gyrox;
    short gyroy;
    short gyroz;
};

struct _mpu_data {
    float accx;
    float accy;
    float accz;

    float gyrox;
    float gyroy;
    float gyroz;
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

extern struct _mpu_raw mpu_raw;
extern struct _mpu_data mpu_data;
extern struct _angle angle;

extern struct kalman_filter_t roll_kalman;
extern struct kalman_filter_t pitch_kalman;
extern struct kalman_filter_t yaw_kalman;

extern void kalman_filter(struct kalman_filter_t * kf, float angle_m, float gyro_m, float *angle_f, float *angle_dot_f);

extern void mpu_sensor_check_gyro_bias(unsigned char check);
extern void mpu_sensor_update_raw(struct _mpu_raw * raw);
extern void mpu_sensor_update_data(struct _mpu_raw * raw, struct _mpu_data * data);
extern void mpu_sensor_update_angle(struct _angle * angle);
extern void mpu_sensor_update_attitude_angle(struct _angle * angle, struct _mpu_data * mpu_data);

#endif
