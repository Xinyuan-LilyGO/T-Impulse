/*
 * @Author: your name
 * @Date: 2021-11-02 09:33:08
 * @LastEditTime: 2021-11-12 10:45:09
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \wristband-S76G\src\main.cpp
 */
#include <Wire.h> // Only needed for Arduino 1.6.5 and earlier
#include <SPI.h>
#include "loramac.h"
#include "config.h"
#include "oled.h"
#include "gps.h"
#include "IMU.h"
#include "Bat.h"
#include "menu.h"
#include "lorawan.h"
#include "STM32LowPower.h"

void BoardInit(bool Anima)
{
    pinMode(PWR_1_8V_PIN, OUTPUT);
    digitalWrite(PWR_1_8V_PIN, HIGH);

    Serial.begin(115200);

    Wire.setSCL(IICSCL);
    Wire.setSDA(IICSDA);
    Wire.begin();

    gps_init();
    bat_init();
    pinMode(BAT_VOLT_PIN, INPUT_ANALOG);
    oled_init(Anima);
    imu_init();
    rtc_init();
    LoRaInit();
    MenuInit();
}

void setup()
{
    LowPower.begin();
    LowPower.attachInterruptWakeup(
        TOUCH_PAD_PIN, [] {}, RISING, DEEP_SLEEP_MODE);
    BoardInit(true);
    IIC_Scan();
    getRTC().setTime(0, 0, 0);
    Serial.println("LoRaWan Demo");
}

void loop()
{
    LoRaLoop();
    bat_loop();
    gps_loop();
    MenuLoop();
    SysLoop();
}
