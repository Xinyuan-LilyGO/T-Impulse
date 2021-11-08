/*
 * @Author: your name
 * @Date: 2021-11-04 15:40:41
 * @LastEditTime: 2021-11-08 16:14:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\RTC.cpp
 */

#include "RTC.h"
#include "gps.h"
#include "oled.h"

#define TimeZone 8
STM32RTC &rtc = STM32RTC::getInstance();

void RTCInit(void)
{
    rtc.begin();
}

void RTCLoop(uint8_t &BTN_state)
{

    if (gps->time.isUpdated() && isGPSenable())
    {
        rtc.setTime(gps->time.hour() < 16 ? gps->time.hour() + TimeZone : 24 - gps->time.hour() + TimeZone, gps->time.minute(), gps->time.second());
    }

    CLEAN_MENU;

    u8g2->setFont(u8g2_font_fub20_tn);
    u8g2->setCursor(0, 32);
    u8g2->print(rtc.getHours());

    u8g2->setFont(u8g2_font_profont17_tr);
    u8g2->setCursor(32, 23);
    u8g2->print(rtc.getMinutes());
    u8g2->drawStr(55, 23, "M");

    u8g2->setFont(u8g2_font_nokiafc22_tu);
    u8g2->setCursor(50, 32);
    u8g2->print(rtc.getSeconds());
    u8g2->drawStr(40, 32, "S");
}