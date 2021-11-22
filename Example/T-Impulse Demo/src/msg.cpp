/*
 * @Author: your name
 * @Date: 2021-11-08 11:42:53
 * @LastEditTime: 2021-11-11 11:45:07
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\msg.cpp
 */

#include "msg.h"

#define FirstHeight 16
#define RowHeight 8

char msg1[100] = {"Hello LilyGo!"};
char msg2[100] = {"Massage 1!"};
char msg3[100] = {"Massage 2!"};

char *msg[3] = {msg1, msg2, msg3};

uint8_t msgRank = 3;

void Msg_Commit(char *Str, uint8_t len)
{
    if (msgRank < 3)
    {
        memset(msg[msgRank], '\0', strlen(msg[msgRank]));
        memcpy(msg[msgRank], Str, len);
        msgRank++;
    }
    else
    {
        memset(msg[0], '\0', strlen(msg[0]));
        memcpy(msg[0], msg[1], strlen(msg[1]));
        memset(msg[1], '\0', strlen(msg[1]));
        memcpy(msg[1], msg[2], strlen(msg[2]));

        memset(msg[2], '\0', strlen(msg[2]));
        memcpy(msg[2], Str, len);
    }
}

void MsgOledLoop(uint8_t &BTN_state)
{
    static int x;
    CLEAN_MENU;

    // MSG显示
    int width = 0;
    for (int i = 0; i < 3; i++)
    {
        if (getOled()->getStrWidth(msg[i]) > width)
        {
            width = getOled()->getStrWidth(msg[i]);
        }
    }
    width = -(width + 100);
    if (x > width)
    {
        getOled()->setFont(u8g2_font_IPAandRUSLCD_tr);
        for (int i = 0; i < msgRank; i++)
        {
            getOled()->drawStr(x + 64, FirstHeight + i * RowHeight, msg[i]);
        }
        x--;
    }
    else
    {
        x = 0;
    }

    //序号
    getOled()->setDrawColor(0x00);
    getOled()->drawBox(0, 8, 11, 24);
    getOled()->setDrawColor(0xff);

    getOled()->setFont(u8g2_font_nokiafc22_tr);
    getOled()->drawStr(0, 16, "1.");
    getOled()->drawStr(0, 24, "2.");
    getOled()->drawStr(0, 32, "3.");

    switch (BTN_state)
    {
    case 0x01:
        break;
    case 0x02:

        msgRank > 0 ? msgRank -= 1 : msgRank = 0;
        memset(msg[0], '\0', strlen(msg[0]));
        memcpy(msg[0], msg[1], strlen(msg[1]));

        memset(msg[1], '\0', strlen(msg[1]));
        memcpy(msg[1], msg[2], strlen(msg[2]));

        memset(msg[2], '\0', strlen(msg[2]));
        break;
    default:
        break;
    }
}