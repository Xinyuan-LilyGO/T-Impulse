/*
 * @Author: your name
 * @Date: 2021-11-08 08:53:04
 * @LastEditTime: 2021-11-12 10:47:02
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\Sys.h
 */
#pragma once

#include "Bat.h"
#include <Wire.h>
#include "menu.h"

void setAutoSleepCountTime(uint32_t time);

void SysLoop(void);
void SysOledLoop(uint8_t &BTN_state);
void IIC_Scan(void);
