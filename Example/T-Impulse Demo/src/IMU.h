/*
 * @Author: your name
 * @Date: 2021-11-02 11:13:03
 * @LastEditTime: 2021-11-12 10:43:34
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \wristband-S76G\src\IMU.h
 */

#ifndef __IMU_H__
#define __IMU_H__

#include <ICM_20948.h>

ICM_20948_I2C *getIMU(void);

void IMUInit(void);
void IMULoop(uint8_t &BTN_state);

#endif /* __IMU_H__ */