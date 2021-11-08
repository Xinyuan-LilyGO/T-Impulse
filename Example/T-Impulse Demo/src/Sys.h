/*
 * @Author: your name
 * @Date: 2021-11-08 08:53:04
 * @LastEditTime: 2021-11-08 09:14:23
 * @LastEditors: your name
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\Sys.h
 */
#pragma once

#include "Bat.h"
#include <Wire.h>
#include "menu.h"

extern uint32_t AutoSleepCountTime;

void SysLoop(void);
void SysOledLoop(uint8_t &BTN_state);
void IIC_Scan(void);
