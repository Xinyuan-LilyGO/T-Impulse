/*
 * @Author: your name
 * @Date: 2021-11-02 09:47:37
 * @LastEditTime: 2021-11-08 15:55:45
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\loramac.h
 */
#pragma once

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include "config.h"
#include <CayenneLPP.h>
#include "oled.h"
#include "IMU.h"
#include "gps.h"
#include "Bat.h"
#include "menu.h"
#include "msg.h"

void setupLMIC(void);
void loopLMIC(void);
