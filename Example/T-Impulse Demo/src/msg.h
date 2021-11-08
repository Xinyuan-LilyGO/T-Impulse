/*
 * @Author: your name
 * @Date: 2021-11-08 11:42:57
 * @LastEditTime: 2021-11-08 16:10:18
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\msg.h
 */

#pragma once

#include "menu.h"
#include "oled.h"

void Msg_Commit(char *Str, uint8_t len);
void MsgOledLoop(uint8_t &BTN_state);
