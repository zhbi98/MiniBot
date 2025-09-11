
#include "qmc5883_cal.h"

struct _mag_raw mag_raw = {0};
struct _mag_data mag_data = {0};

void mag_sensor_update_raw(struct _mag_raw * raw)
{
    short data[3] = {0};

    QMC5883_Read(&data[0], &data[1], &data[2]);

    raw->magx = data[0] - MAGX_BIAS;
    raw->magy = data[1] - MAGY_BIAS;
    raw->magz = data[2] - MAGZ_BIAS;
}

void mag_sensor_data_filter(struct _mag_data * data)
{
    static struct _mag_data _data = {0};

    data->magx = _data.magx * 0.7 + data->magx * 0.3;
    data->magy = _data.magy * 0.7 + data->magy * 0.3;
    data->magz = _data.magz * 0.7 + data->magz * 0.3;

    _data = *data;
}

void mag_sensor_update_data(struct _mag_raw * raw, struct _mag_data * data)
{
    // Mag full range 2G
    data->magx = raw->magx * MAG_TO_GAUSS /* 2 * (mag._ax / 32768.0) */;
    data->magy = raw->magy * MAG_TO_GAUSS /* 2 * (mag._ay / 32768.0) */;
    data->magz = raw->magz * MAG_TO_GAUSS /* 2 * (mag._az / 32768.0) */;

    mag_sensor_data_filter(data);
}

void mag_sensor_update_angle(struct _mag_data * data, float * angle)
{
    *angle = atan2(
        data->magy, 
        data->magz
    ) * 180.0 / PI;

    *angle = (*angle < 0.0) ? *angle + 360.0 : *angle;
    *angle = 360.0 - *angle;
}
