
#pragma once

#include "loramac.h"
#include "oled.h"
#include "config.h"
#include "menu.h"

void LoRaInit(void);
void LoRaLoop(void);
void LoRaOledLoop(uint8_t &BTN_state);
void LoRa_Sleep(void);
