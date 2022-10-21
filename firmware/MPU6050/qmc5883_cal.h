
#ifndef __QMC5883_CAL_H__
#define __QMC5883_CAL_H__

#include <math.h>
#include "qmc5883.h"

#define PI 3.1416f

#define MAGX_BIAS 0
#define MAGY_BIAS 0
#define MAGZ_BIAS 0

#define MAG_TO_GAUSS (2.0f / 32768.0f) // Mag full range 2 GUSS

struct _mag_raw {
    short magx;
    short magy;
    short magz;
};

struct _mag_data {
    float magx;
    float magy;
    float magz;
};

extern struct _mag_raw mag_raw;
extern struct _mag_data mag_data;

extern void mag_sensor_update_raw(struct _mag_raw * raw);
extern void mag_sensor_data_filter(struct _mag_data * data);
extern void mag_sensor_update_data(struct _mag_raw * raw, struct _mag_data * data);
extern void mag_sensor_update_angle(struct _mag_data * data, float * angle);

#endif
