
#include "kalman_filter.h"

struct _accelerometer accel = {0};
struct _gyro gyro = {0};
struct _angles angles = {0};

#define ACCE_CONVERT (2.0 / 32768.0) // Accelerometer full range 2g
#define GYRO_CONVERT (2000.0 / 32768.0) // Gyro full range 2000 deg

void read_mpu_data()
{
    MPU_Get_Accelerometer(
        &accel.raw_data_x,
        &accel.raw_data_y,
        &accel.raw_data_z
    );
    MPU_Get_Gyroscope(
        &gyro.raw_data_x,
        &gyro.raw_data_y,
        &gyro.raw_data_z
    );

    MPU_Get_Temperature();

    accel.raw_data_x = accel.raw_data_x - 1000;
    accel.raw_data_y = accel.raw_data_y + 220;
    accel.raw_data_z = accel.raw_data_z + 100;

    gyro.raw_data_x = gyro.raw_data_x - 109;
    gyro.raw_data_y = gyro.raw_data_y - 2;
    gyro.raw_data_z = gyro.raw_data_z + 10;

    // Accelerometer full range 2g
    accel.x = accel.raw_data_x * ACCE_CONVERT /* 2 * (accel._ax / 32768.0) */;
    accel.y = accel.raw_data_y * ACCE_CONVERT /* 2 * (accel._ay / 32768.0) */;
    accel.z = accel.raw_data_z * ACCE_CONVERT /* 2 * (accel._az / 32768.0) */;
    // Gyro full range 2000 deg
    gyro.x = gyro.raw_data_x * GYRO_CONVERT /* 2000 * (gyro._gx / 32768.0) */;
    gyro.y = gyro.raw_data_y * GYRO_CONVERT /* 2000 * (gyro._gy / 32768.0) */;
    gyro.z = gyro.raw_data_z * GYRO_CONVERT /* 2000 * (gyro._gz / 32768.0) */;
}

#if 0
struct _recursive ax_recursive = {.K_gain = 0.4};
struct _recursive ay_recursive = {.K_gain = 0.4};
struct _recursive az_recursive = {.K_gain = 0.4};
struct _recursive gx_recursive = {.K_gain = 0.4};
struct _recursive gy_recursive = {.K_gain = 0.4};
struct _recursive gz_recursive = {.K_gain = 0.4};
#endif

float recursive_processing(struct _recursive * recursive, float Y_meas)
{
    recursive->current_estimate = recursive->last_estimste + recursive->K_gain * (Y_meas - recursive->last_estimste);

    recursive->last_estimste = recursive->current_estimate;

    return recursive->current_estimate;
}

struct kalman_filter_t roll_kalman_Filter = {
    .dt      = 0.01,
    .P       = {
        {1, 0}, 
        {0, 1}
    },
    .Pdot    = {0, 0, 0, 0},
    .Q_angle = 0.001, 
    .Q_gyro  = 0.005,
    .R_angle = 0.5,
    .C_0     = 1,
};

struct kalman_filter_t pitch_kalman_Filter = {
    .dt      = 0.01,
    .P       = {
        {1, 0}, 
        {0, 1}
    },
    .Pdot    = {0, 0, 0, 0},
    .Q_angle = 0.001, 
    .Q_gyro  = 0.005,
    .R_angle = 0.5,
    .C_0     = 1,
};

struct kalman_filter_t yaw_kalman_Filter = {
    .dt      = 0.01,
    .P       = {
        {1, 0}, 
        {0, 1}
    },
    .Pdot    = {0, 0, 0, 0},
    .Q_angle = 0.001, 
    .Q_gyro  = 0.005,
    .R_angle = 0.5,
    .C_0     = 1,
};

void kalman_filter(struct kalman_filter_t * kf, float angle_m, float gyro_m, float *angle_f, float *angle_dot_f)
{
    kf->angle += (gyro_m - kf->q_bias) * kf->dt;

    kf->Pdot[0] = kf->Q_angle - kf->P[0][1] - kf->P[1][0];
    kf->Pdot[1] = -kf->P[1][1];
    kf->Pdot[2] = -kf->P[1][1];
    kf->Pdot[3] = kf->Q_gyro;

    kf->P[0][0] += kf->Pdot[0] * kf->dt;
    kf->P[0][1] += kf->Pdot[1] * kf->dt;
    kf->P[1][0] += kf->Pdot[2] * kf->dt;
    kf->P[1][1] += kf->Pdot[3] * kf->dt;

    kf->angle_err = angle_m - kf->angle;

    kf->PCt_0 = kf->C_0 * kf->P[0][0];
    kf->PCt_1 = kf->C_0 * kf->P[1][0];

    kf->E = kf->R_angle + kf->C_0 * kf->PCt_0;

    kf->K_0 = kf->PCt_0 / kf->E;
    kf->K_1 = kf->PCt_1 / kf->E;

    kf->t_0 = kf->PCt_0;
    kf->t_1 = kf->C_0 * kf->P[0][1];

    kf->P[0][0] -= kf->K_0 * kf->t_0;
    kf->P[0][1] -= kf->K_0 * kf->t_1;
    kf->P[1][0] -= kf->K_1 * kf->t_0;
    kf->P[1][1] -= kf->K_1 * kf->t_1;
        
    kf->angle += kf->K_0 * kf->angle_err;
    kf->q_bias += kf->K_1 * kf->angle_err;
    kf->angle_dot = gyro_m - kf->q_bias;

    *angle_f = kf->angle;
    *angle_dot_f = kf->angle_dot;
}
