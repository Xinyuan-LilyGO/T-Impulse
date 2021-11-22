/*
 * @Author: your name
 * @Date: 2021-11-04 15:40:41
 * @LastEditTime: 2021-11-12 10:44:15
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\RTC.cpp
 */

#include "RTC.h"
#include "gps.h"
#include "oled.h"

#define TimeZone 8
STM32RTC &rtc = STM32RTC::getInstance();

STM32RTC &getRTC(void)
{
    return rtc;
}

void RTCInit(void)
{
    rtc.begin();
}

void RTCLoop(uint8_t &BTN_state)
{

    if (getGPS()->time.isUpdated() && isGPSenable())
    {
        rtc.setTime(getGPS()->time.hour() < 16 ? getGPS()->time.hour() + TimeZone : 24 - getGPS()->time.hour() + TimeZone, getGPS()->time.minute(), getGPS()->time.second());
    }

    CLEAN_MENU;

    getOled()->setFont(u8g2_font_fub20_tn);
    getOled()->setCursor(0, 32);
    getOled()->print(rtc.getHours());

    getOled()->setFont(u8g2_font_profont17_tr);
    getOled()->setCursor(32, 23);
    getOled()->print(rtc.getMinutes());
    getOled()->drawStr(55, 23, "M");

    getOled()->setFont(u8g2_font_nokiafc22_tu);
    getOled()->setCursor(50, 32);
    getOled()->print(rtc.getSeconds());
    getOled()->drawStr(40, 32, "S");
}