#ifndef __IMU_H__
#define __IMU_H__

#include <ICM_20948.h>

ICM_20948_I2C *getIMU(void);

void imu_init(void);
void imu_loop(uint8_t &BTN_state);

#endif /* __IMU_H__ */