
#ifndef __OLED_H__
#define __OLED_H__

#include <U8g2lib.h>

void OledInit(bool Anima);
void OledLoop(void);
void OledSleep(void);

extern U8G2_SSD1306_64X32_1F_F_HW_I2C *u8g2;

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define CLEAN_TITLE                  \
    {                                \
        u8g2->setDrawColor(0x00);    \
        u8g2->drawBox(0, 0, 64, 10); \
        u8g2->setDrawColor(0xff);    \
    }

#define CLEAN_MENU                    \
    {                                 \
        u8g2->setDrawColor(0x00);     \
        u8g2->drawBox(0, 10, 64, 22); \
        u8g2->setDrawColor(0xff);     \
    }

#endif /* __OLED_H__ */