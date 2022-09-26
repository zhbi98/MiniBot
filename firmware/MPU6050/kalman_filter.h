
#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "MPU6050_I2C.h"
#include "MPU6050.h"

struct _accelerometer {
    short _ax;
    short _ay;
    short _az;
    float ax;
    float ay;
    float az;
};

struct _gyro {
    short _gx;
    short _gy;
    short _gz;
    float gx;
    float gy;
    float gz;
};

struct _angles {
    float pitch;
    float roll;
    float yaw;
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
