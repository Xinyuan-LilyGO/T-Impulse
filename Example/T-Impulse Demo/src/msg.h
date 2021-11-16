#pragma once

#include "menu.h"
#include "oled.h"

void Msg_Commit(char *Str, uint8_t len);
void MsgOledLoop(uint8_t &BTN_state);
