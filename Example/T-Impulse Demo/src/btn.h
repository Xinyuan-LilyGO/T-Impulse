/*
 * @Author: your name
 * @Date: 2021-11-05 14:53:28
 * @LastEditTime: 2021-11-05 14:58:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\btn.h
 */

#pragma once

#include "Arduino.h"
#include "config.h"
#include "OneButton.h"

extern OneButton *button;
void btnInit(void);