/*
 * @Author: your name
 * @Date: 2021-11-04 15:40:48
 * @LastEditTime: 2021-11-06 10:41:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\RTC.h
 */
#ifndef __RTC_H__
#define __RTC_H__

#include <STM32RTC.h>

extern STM32RTC &rtc;

void RTCInit(void);
void RTCLoop(uint8_t &BTN_state);

#endif /* __RTC_H__ */