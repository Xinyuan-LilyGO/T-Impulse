/*
 * @Author: your name
 * @Date: 2021-11-02 11:12:50
 * @LastEditTime: 2021-11-02 11:22:28
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \wristband-S76G\src\IMU.CPP
 */

#include "IMU.h"

ICM_20948_I2C *imu = nullptr;

void IMUInit(void)
{
    imu = new ICM_20948_I2C();

    imu->begin();
    if (imu->status != ICM_20948_Stat_Ok)
    {
        Serial.println("setup imu sensor FAIL");
        return;
    }
    Serial.println("ICM_20948_Stat_Ok");
}
