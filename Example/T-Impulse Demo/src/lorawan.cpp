/*
 * @Author: your name
 * @Date: 2021-11-06 17:39:18
 * @LastEditTime: 2021-11-11 11:42:34
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\lorawan.cpp
 */

#include "lorawan.h"

bool LoRa_enable = false;

void LoRaInit(void)
{
    SPI.setMISO(LORA_MISO);
    SPI.setMOSI(LORA_MOSI);
    SPI.setSCLK(LORA_SCK);
    SPI.begin();

    pinMode(RADIO_ANT_SWITCH_RXTX, OUTPUT);
    digitalWrite(RADIO_ANT_SWITCH_RXTX, HIGH);

    setupLMIC();
}

void LoRa_Sleep(void)
{
    LMIC_shutdown();
}

void LoRaLoop(void)
{
    if (LoRa_enable)
    {
        loopLMIC();
    }
}

void LoRaOledLoop(uint8_t &BTN_state)
{
    static uint8_t select = 0;
    CLEAN_MENU;

    char optionbuf[10];
    sprintf(optionbuf, "[%s]lora en", LoRa_enable ? "*" : " ");
    getOled()->setFont(u8g2_font_nokiafc22_tr);
    getOled()->drawStr(10, 19, optionbuf);

    //光标
    getOled()->setFont(u8g2_font_open_iconic_all_1x_t);
    getOled()->drawGlyph(0, select + 22, 0x4E); //->

    switch (BTN_state)
    {
    case 0x01:
        break;
    case 0x02:
        LoRa_enable = !LoRa_enable;
        LoRa_enable ? LoRaInit() : LoRa_Sleep();
        LoRa_enable ? Title_Commit("LoRa begin!") : Title_Commit("LoRa shutdown!");
        break;
    default:
        break;
    }
}
