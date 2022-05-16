
#include "IMU.h"
#include <Wire.h>
#include "config.h"
#include "oled.h"
ICM_20948_I2C *imu = nullptr;

#define Y_HIGH 20
#define Y_PIXEL 10

ICM_20948_I2C *getIMU(void)
{
    return imu;
}

void imu_init(void)
{
    imu = new ICM_20948_I2C();

    imu->begin(Wire, ICM20948_ADDR, ICM20948_INT_PIN);
    if (imu->status != ICM_20948_Stat_Ok)
    {
        Serial.println("setup imu sensor FAIL");
        return;
    }
    Serial.println("ICM_20948_Stat_Ok");
}

void imu_loop(uint8_t &BTN_state)
{
    static uint32_t Millis;
    CLEAN_MENU;

    if (imu->dataReady())
    {
        imu->getAGMT();

        getOled()->drawStr(2, 20, "x   y   z");
        // x
        uint8_t x_h = constrain(abs(imu->accX()) / 150, 0, Y_PIXEL);
        Serial.printf("imu->accX() : %f\n", imu->accX());
        getOled()->drawBox(12, (imu->accX() > 0 ? Y_HIGH - x_h : Y_HIGH), 6, x_h);
        // y
        uint8_t y_h = constrain(abs(imu->accY()) / 150, 0, Y_PIXEL);
        getOled()->drawBox(32, (imu->accY() > 0 ? Y_HIGH - y_h : Y_HIGH), 6, y_h);
        // z
        uint8_t z_h = constrain(abs(imu->accZ()) / 150, 0, Y_PIXEL);
        getOled()->drawBox(54, (imu->accZ() < 0 ? Y_HIGH - z_h : Y_HIGH), 6, z_h);
    }
}