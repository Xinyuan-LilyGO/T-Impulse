/*
 * @Author: your name
 * @Date: 2021-11-06 17:39:25
 * @LastEditTime: 2021-11-06 18:46:58
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\lorawan.h
 */

#pragma once

#include "loramac.h"
#include "oled.h"
#include "config.h"
#include "menu.h"

void LoRaInit(void);
void LoRaLoop(void);
void LoRaOledLoop(uint8_t &BTN_state);
void LoRa_Sleep(void);
