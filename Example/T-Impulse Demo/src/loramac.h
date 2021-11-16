
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
