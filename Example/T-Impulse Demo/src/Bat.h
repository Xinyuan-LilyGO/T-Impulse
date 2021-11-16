
#ifndef __BAT_H__
#define __BAT_H__

#include <stm32l073xx.h>
#include "stm32l0xx_hal.h"
#include "stm32_def.h"

void Bat_Init(void);
void Bat_loop(void);
void PowerTitle(void);
void PowerDown(void);

extern uint32_t Volt;

#endif /* __BAT_H__ */