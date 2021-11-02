/*
 * @Author: your name
 * @Date: 2021-11-02 10:55:22
 * @LastEditTime: 2021-11-02 11:01:45
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \wristband-S76G\src\oled.h
 */

#ifndef __OLED_H__
#define __OLED_H__

#include <U8g2lib.h>

void OledInit(void);
extern U8G2_SSD1306_64X32_1F_F_HW_I2C *u8g2;

#endif /* __OLED_H__ */