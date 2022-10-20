
#ifndef __QMC5883_H__
#define	__QMC5883_H__

#include <stdbool.h>
#include "qmc5883_i2c.h"
#include "time.h"

bool QMC5883_Detect(void);
void QMC5883_Init(void);
void QMC5883_Read(int16_t * mx, int16_t * my, int16_t * mz);

#endif
