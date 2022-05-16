#ifndef __OLED_H__
#define __OLED_H__

#include <U8g2lib.h>

void oled_init(bool Anima);
void oled_Loop(void);
void OledSleep(void);

typedef U8G2_SSD1306_64X32_1F_F_HW_I2C OledClass;

OledClass *getOled(void);

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define CLEAN_TITLE                       \
    {                                     \
        getOled()->setDrawColor(0x00);    \
        getOled()->drawBox(0, 0, 64, 10); \
        getOled()->setDrawColor(0xff);    \
    }

#define CLEAN_MENU                         \
    {                                      \
        getOled()->setDrawColor(0x00);     \
        getOled()->drawBox(0, 10, 64, 22); \
        getOled()->setDrawColor(0xff);     \
    }

#endif /* __OLED_H__ */