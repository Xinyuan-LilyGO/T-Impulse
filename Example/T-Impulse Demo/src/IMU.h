
#ifndef __IMU_H__
#define __IMU_H__

#include <ICM_20948.h>

extern ICM_20948_I2C *imu;

void IMUInit(void);
void IMULoop(uint8_t &BTN_state);

#endif /* __IMU_H__ */