
#ifndef __IMU_H__
#define __IMU_H__

#include <ICM_20948.h>

extern bool is_inited_imu;
extern ICM_20948_I2C *imu;

void imu_init(void);

#endif /* __IMU_H__ */