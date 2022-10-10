
#include "kalman_filter.h"

struct _acc acc = {0};
struct _gyro gyro = {0};
struct _angle angle = {0};

void check_gyro_bias()
{
    int data[3] = {0};

    info("Place the gyroscope horizontally and still.");

    for (unsigned char check = 0; check < 255; check++) {
        MPU_Get_Gyroscope(&gyro.raw_x, &gyro.raw_y, &gyro.raw_z);

        data[0] += gyro.raw_x;
        data[1] += gyro.raw_y;
        data[2] += gyro.raw_z;

        sleep_ms(50);
    }

    info("gyrox:%d", (int)(data[0] / 255 + 0.5));
    info("gyroy:%d", (int)(data[1] / 255 + 0.5));
    info("gyroz:%d", (int)(data[2] / 255 + 0.5));
}

void get_mpu_raw_data()
{
    MPU_Get_Accelerometer(
        &acc.raw_x,
        &acc.raw_y,
        &acc.raw_z
    );
    MPU_Get_Gyroscope(
        &gyro.raw_x,
        &gyro.raw_y,
        &gyro.raw_z
    );

    MPU_Get_Temperature();

    gyro.raw_x = gyro.raw_x - GYROX_BIAS;
    gyro.raw_y = gyro.raw_y - GYROY_BIAS;
    gyro.raw_z = gyro.raw_z - GYROZ_BIAS;

    // Accelerometer full range 2g
    acc.x = acc.raw_x * ACC /* 2 * (acc._ax / 32768.0) */;
    acc.y = acc.raw_y * ACC /* 2 * (acc._ay / 32768.0) */;
    acc.z = acc.raw_z * ACC /* 2 * (acc._az / 32768.0) */;
    // Gyro full range 2000 deg
    gyro.x = gyro.raw_x * GYRO /* 2000 * (gyro._gx / 32768.0) */;
    gyro.y = gyro.raw_y * GYRO /* 2000 * (gyro._gy / 32768.0) */;
    gyro.z = gyro.raw_z * GYRO /* 2000 * (gyro._gz / 32768.0) */;
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

void attitude_angle_update()
{
    /**
     * Y:tan(pitch) = tan(Axz) = Rx/Rz
     * X:tan(roll)  = tan(Ayz) = Ry/Rz
     */
    get_mpu_raw_data();

    angle.roll = atan2(acc.y, acc.z) * 180.0 / 3.1415;
    angle.pitch = -atan2(acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)) * 180.0 / 3.1415;
    // pitch = atan2(accel.ax, accel.az) * 180.0 / 3.14;

    kalman_filter(&roll_kalman_Filter, 
        angle.roll, 
        gyro.x, 
        &angle.roll, 
        &gyro.x
    );
    kalman_filter(&pitch_kalman_Filter, 
        angle.pitch, 
        gyro.y, 
        &angle.pitch, 
        &gyro.y
    );
#if 0
    kalman_filter(&yaw_kalman_Filter, 
        angle.yaw, 
        gyro.z, 
        &angle.yaw, 
        &gyro.z
    );
#endif
}
