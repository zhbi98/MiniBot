
#include "kalman_filter.h"

struct _recursive ax_recursive = {.K_gain = 0.2};
struct _recursive ay_recursive = {.K_gain = 0.2};
struct _recursive az_recursive = {.K_gain = 0.2};

float recursive_processing(struct _recursive * recursive, float Y_meas)
{
    recursive->current_estimate = recursive->last_estimste + recursive->K_gain * (Y_meas - recursive->last_estimste);

    recursive->last_estimste = recursive->current_estimate;

    return recursive->current_estimate;
}

struct _mpu_raw mpu_raw = {0};
struct _mpu_data mpu_data = {0};
struct _angle angle = {0};

void mpu_sensor_check_gyro_bias(unsigned char check)
{
    short temp[3] = {0};
    int data[3] = {0};

    if (!check) return;

    info("Place the gyroscope horizontally and still.");

    for (unsigned char read = 0; read < 255; read++) {
        MPU_Get_Gyroscope(&temp[0], &temp[1], &temp[2]);

        data[0] += temp[0];
        data[1] += temp[1];
        data[2] += temp[2];

        sleep_ms(50);
    }

    temp[0] = (int)(data[0] / 255.0 + 0.5);
    temp[1] = (int)(data[1] / 255.0 + 0.5);
    temp[2] = (int)(data[2] / 255.0 + 0.5);

    info("check success.");
}

void mpu_sensor_update_raw(struct _mpu_raw * raw)
{
    short data[7] = {0};

    MPU_Get_Accelerometer(&data[0], &data[1], &data[2]);
    MPU_Get_Gyroscope(&data[3], &data[4], &data[5]);
    data[6] = MPU_Get_Temperature();

    raw->accx  = data[0] - ACCX_BIAS;
    raw->accy  = data[1] - ACCY_BIAS;
    raw->accz  = data[2] - ACCZ_BIAS;

    raw->gyrox = data[3] - GYROX_BIAS;
    raw->gyroy = data[4] - GYROY_BIAS;
    raw->gyroz = data[5] - GYROZ_BIAS;
}

void mpu_sensor_update_data(struct _mpu_raw * raw, struct _mpu_data * data)
{
    // Accelerometer full range 2g
    data->accx = raw->accx * ACC_SCALE /* 2 * (acc._ax / 32768.0) */;
    data->accy = raw->accy * ACC_SCALE /* 2 * (acc._ay / 32768.0) */;
    data->accz = raw->accz * ACC_SCALE /* 2 * (acc._az / 32768.0) */;
    // Gyro full range 2000 deg
    data->gyrox = raw->gyrox * GYRO_SCALE /* 2000 * (gyro._gx / 32768.0) */;
    data->gyroy = raw->gyroy * GYRO_SCALE /* 2000 * (gyro._gy / 32768.0) */;
    data->gyroz = raw->gyroz * GYRO_SCALE /* 2000 * (gyro._gz / 32768.0) */;
#if 0
    data->accx  = recursive_processing(&ax_recursive, data->accx);
    data->accy  = recursive_processing(&ay_recursive, data->accy);
    data->accz  = recursive_processing(&az_recursive, data->accz);
#endif
}

void mpu_sensor_update_angle(struct _mpu_data * data, struct _angle * angle)
{
    /**
     * Y:tan(pitch) = tan(Axz) = Rx/Rz
     * X:tan(roll)  = tan(Ayz) = Ry/Rz
     */
    angle->roll = atan2(
        data->accy, 
        data->accz
    ) * 180.0 / PI;

    angle->pitch = -atan2(
        data->accx, 
        sqrt(data->accy * data->accy + data->accz * data->accz)
    ) * 180.0 / PI;
    // pitch = atan2(accx, accz) * 180.0 / 3.14;
}

struct kalman_filter_t roll_kalman = {
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

struct kalman_filter_t pitch_kalman = {
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

struct kalman_filter_t yaw_kalman = {
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

void mpu_sensor_update_attitude_angle(struct _mpu_data * mpu_data, struct _angle * angle)
{
    static unsigned int last_tick = 0;
    unsigned int tick_incr = 0;

    tick_incr = GET_SYS_TICK() - last_tick;
    last_tick = GET_SYS_TICK();

    roll_kalman.dt = tick_incr * 5.0 / 1000.0;
    pitch_kalman.dt = roll_kalman.dt;
    yaw_kalman.dt = roll_kalman.dt;

    kalman_filter(&roll_kalman, 
        angle->roll, 
        mpu_data->gyrox, 
        &angle->roll, 
        &mpu_data->gyrox
    );
    kalman_filter(&pitch_kalman, 
        angle->pitch, 
        mpu_data->gyroy, 
        &angle->pitch, 
        &mpu_data->gyroy
    );
#if 0
    kalman_filter(&yaw_kalman, 
        angle->yaw, 
        mpu_data->gyroz, 
        &angle->yaw, 
        &mpu_data->gyroz
    );
#endif
}
