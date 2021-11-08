/*
 * @Author: your name
 * @Date: 2021-11-08 11:42:53
 * @LastEditTime: 2021-11-08 16:09:01
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
        if (u8g2->getStrWidth(msg[i]) > width)
        {
            width = u8g2->getStrWidth(msg[i]);
        }
    }
    width = -(width + 100);
    if (x > width)
    {
        u8g2->setFont(u8g2_font_IPAandRUSLCD_tr);
        for (int i = 0; i < msgRank; i++)
        {
            u8g2->drawStr(x + 64, FirstHeight + i * RowHeight, msg[i]);
        }
        x--;
    }
    else
    {
        x = 0;
    }

    //序号
    u8g2->setDrawColor(0x00);
    u8g2->drawBox(0, 8, 11, 24);
    u8g2->setDrawColor(0xff);

    u8g2->setFont(u8g2_font_nokiafc22_tr);
    u8g2->drawStr(0, 16, "1.");
    u8g2->drawStr(0, 24, "2.");
    u8g2->drawStr(0, 32, "3.");

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