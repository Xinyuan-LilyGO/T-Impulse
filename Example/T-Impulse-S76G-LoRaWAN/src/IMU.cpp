#include "IMU.h"

bool is_inited_imu = true;
ICM_20948_I2C *imu = nullptr;

void imu_init(void)
{
    imu = new ICM_20948_I2C();

    imu->begin();
    if (imu->status != ICM_20948_Stat_Ok)
    {
        Serial.println("setup imu sensor FAIL");
        is_inited_imu = false;
        return;
    }
    Serial.println("ICM_20948_Stat_Ok");
}
