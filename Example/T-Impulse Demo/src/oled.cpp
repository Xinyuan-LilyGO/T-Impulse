#include "oled.h"
#include "config.h"
#include "RTC.h"

OledClass *u8g2 = nullptr;

OledClass *getOled(void)
{
    return u8g2;
}

void oled_init(bool Anima)
{
    u8g2 = new OledClass(U8G2_R0, OLED_RESET, IICSCL, IICSDA);

    u8g2->begin();
    u8g2->setContrast(0x00);
    u8g2->clearBuffer();
    u8g2->setFontMode(1); // Transparent
    u8g2->setFontDirection(0);
    if (Anima)
    {
        u8g2->setFont(u8g2_font_fub14_tf);
        u8g2->drawStr(2, 18, "LilyGo");
        u8g2->sendBuffer();
        for (int i = 0; i < 0xFF; i++)
        {
            u8g2->setContrast(i);
            u8g2->drawFrame(2, 25, 60, 6);
            u8g2->drawBox(3, 26, (uint8_t)(0.231 * i), 5);
            u8g2->sendBuffer();
        }
    }
    else
    {
        u8g2->setContrast(0xFF);
    }

    /*     for (int i = 0; i < 0xFF; i++)
        {
            u8g2->setContrast(0xFF - i);
            delay(10);
        } */
}

void OledSleep(void)
{
    u8g2->sleepOn();
    u8g2 = nullptr;
}