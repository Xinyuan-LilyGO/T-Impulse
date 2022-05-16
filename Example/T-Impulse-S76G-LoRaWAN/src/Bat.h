
#ifndef __BAT_H__
#define __BAT_H__

#include <stm32l073xx.h>
#include "stm32l0xx_hal.h"
#include "stm32_def.h"

void bat_init(void);
void bat_loop(void);
extern uint32_t Volt;

#endif /* __BAT_H__ */