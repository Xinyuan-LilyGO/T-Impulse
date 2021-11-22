/*
 * @Author: your name
 * @Date: 2021-11-08 08:53:04
 * @LastEditTime: 2021-11-12 10:47:23
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%A
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\Sys.cpp
 */
#include "Sys.h"

#define FirstHeight 9
#define RowHeight 10

bool AutoSleep_en = false;
uint32_t TriggerSleepTime = 10; // S
uint32_t AutoSleepCountTime = 0;

void setAutoSleepCountTime(uint32_t time)
{
    AutoSleepCountTime = time;
}

void IIC_Scan(void)
{
    byte error, address;
    int nDevices = 0;
    Serial.println("Scanning...");
    Wire.setSCL(IICSCL);
    Wire.setSDA(IICSDA);
    Wire.begin();
    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");
            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}
void SysLoop(void)
{
    static uint32_t Millis;
    if (millis() - Millis > 1000)
    {
        AutoSleepCountTime++;
        if (TriggerSleepTime < AutoSleepCountTime && AutoSleep_en)
        {
            AutoSleepCountTime = 0;
            PowerDown();
        }
        Millis = millis();
    }
}

void SysOledLoop(uint8_t &BTN_state)
{
    static uint8_t select = 0;
    char optionbuf[20];
    CLEAN_MENU;

    sprintf(optionbuf, "[%s]Auto Sleep", AutoSleep_en ? "*" : " ");
    getOled()->setFont(u8g2_font_nokiafc22_tr);
    getOled()->drawStr(10, FirstHeight + RowHeight, optionbuf);

    sprintf(optionbuf, "tri time[%dS]", TriggerSleepTime);
    getOled()->setFont(u8g2_font_nokiafc22_tr);
    getOled()->drawStr(10, FirstHeight + (RowHeight * 2), optionbuf);

    //光标
    getOled()->setFont(u8g2_font_open_iconic_all_1x_t);
    getOled()->drawGlyph(0, FirstHeight + (select * RowHeight) + RowHeight, 0x4E); //->

    switch (BTN_state)
    {
    case 0x01:
        select = ++select % 2;
        break;
    case 0x02:
        if (select == 0)
        {
            AutoSleep_en = !AutoSleep_en;
            AutoSleep_en ? Title_Commit("AutoSleep ON!") : Title_Commit("AutoSleep OFF!");
        }
        else if (select == 1)
        {
            TriggerSleepTime = TriggerSleepTime < 60 ? TriggerSleepTime + 10 : 10;
        }
        break;
    default:
        break;
    }
}