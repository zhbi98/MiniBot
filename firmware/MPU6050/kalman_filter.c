
#include "kalman_filter.h"

struct _accelerometer accel = {0};
struct _gyro gyro = {0};
struct _angles angles = {0};

#define ACCE_CONVERT (2.0 / 32768.0) // Accelerometer full range 2g
#define GYRO_CONVERT (2000.0 / 32768.0) // Gyro full range 2000 deg

void read_mpu_data()
{
    MPU_Get_Accelerometer(
        &accel._ax,
        &accel._ay,
        &accel._az
    );
    MPU_Get_Gyroscope(
        &gyro._gx,
        &gyro._gy,
        &gyro._gz
    );

    MPU_Get_Temperature();

    // Accelerometer full range 2g
    accel.ax = accel._ax * ACCE_CONVERT /* 2 * (accel._ax / 32768.0) */;
    accel.ay = accel._ay * ACCE_CONVERT /* 2 * (accel._ay / 32768.0) */;
    accel.az = accel._az * ACCE_CONVERT /* 2 * (accel._az / 32768.0) */;
    // Gyro full range 2000 deg
    gyro.gx = gyro._gx * GYRO_CONVERT /* 2000 * (gyro._gx / 32768.0) */;
    gyro.gy = gyro._gy * GYRO_CONVERT /* 2000 * (gyro._gy / 32768.0) */;
    gyro.gz = gyro._gz * GYRO_CONVERT /* 2000 * (gyro._gz / 32768.0) */;
}

struct _recursive {
    float current_estimate;
    float last_estimste;
    float K_gain;
};

struct _recursive ax_recursive = {.K_gain = 0.4};
struct _recursive ay_recursive = {.K_gain = 0.4};
struct _recursive az_recursive = {.K_gain = 0.4};
struct _recursive gx_recursive = {.K_gain = 0.4};
struct _recursive gy_recursive = {.K_gain = 0.4};
struct _recursive gz_recursive = {.K_gain = 0.4};

float recursive_processing(struct _recursive * recursive, float Y_meas)
{
    recursive->current_estimate = recursive->last_estimste + recursive->K_gain * (Y_meas - recursive->last_estimste);

    recursive->last_estimste = recursive->current_estimate;

    return recursive->current_estimate;
}

void mpu_data_filter()
{
    accel.ax = recursive_processing(&ax_recursive, accel.ax);
    accel.ay = recursive_processing(&ay_recursive, accel.ay);
    accel.az = recursive_processing(&az_recursive, accel.az);
    gyro.gx  = recursive_processing(&gx_recursive, gyro.gx);
    gyro.gy  = recursive_processing(&gy_recursive, gyro.gy);
    gyro.gz  = recursive_processing(&gz_recursive, gyro.gz);
}
