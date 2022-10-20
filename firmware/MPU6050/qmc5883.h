#ifndef __QMC5883_H
#define	__QMC5883_H

// #include "mathTool.h"
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "qmc5883_i2c.h"
#include "log.h"
#include "time.h"

bool QMC5883_Detect(void);
void QMC5883_Init(void);
void QMC5883_Update(void);
void QMC5883_Read(/*Vector3f_t* mag*/);


#endif

// https://haas.iot.aliyun.com/aliosthings/compass.html
// https://zhuanlan.zhihu.com/p/386862265
