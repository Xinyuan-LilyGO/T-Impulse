#ifndef __BAT_H__
#define __BAT_H__

#include <stm32l073xx.h>
#include "stm32l0xx_hal.h"
#include "stm32_def.h"

void bat_init(void);
void bat_loop(void);
void PowerTitle(void);
void PowerDown(void);

uint32_t getVolt(void);

#endif /* __BAT_H__ */