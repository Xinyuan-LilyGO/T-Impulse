/*
 * @Author: your name
 * @Date: 2021-11-02 11:27:48
 * @LastEditTime: 2021-11-02 11:38:36
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \wristband-S76G\src\Bat.h
 */
#ifndef __BAT_H__
#define __BAT_H__

#include <stm32l073xx.h>
#include "stm32l0xx_hal.h"
#include "stm32_def.h"

void Bat_Init(void);
void Bat_loop(void);
extern uint32_t Volt;

#endif /* __BAT_H__ */