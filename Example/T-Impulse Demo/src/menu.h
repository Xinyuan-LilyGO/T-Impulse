/*
 * @Author: your name
 * @Date: 2021-11-05 11:48:25
 * @LastEditTime: 2021-11-08 15:58:55
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\menu.h
 */
#ifndef __MENU_H__
#define __MENU_H__

#include "oled.h"
#include "btn.h"
#include "RTC.h"
#include "IMU.h"
#include "Bat.h"
#include "gps.h"
#include "lorawan.h"
#include "Sys.h"
#include "msg.h"

struct menu_entry_type
{
    const uint8_t *font;
    uint16_t icon;
    void (*loop)(uint8_t &);
};

void MenuInit(void);
void Title_Commit(char *Str);

void MenuLoop(void);

#endif /* __MENU_H__ */