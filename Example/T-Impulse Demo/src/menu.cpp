#include "menu.h"

#define MAX_APP 5

char Title_Str[50] = {"LilyGo title"};

bool btn_quit = true;
bool btn_join = false;
bool btn_next = false;

bool InApp = false;
uint32_t focus_num = 0;

void MenuInit(void)
{
    btnInit();
    button->attachClick([]
                        {   
                            setAutoSleepCountTime(0);
                            btn_next = true;
                            Serial.println("Click"); });
    button->attachDoubleClick([]
                              {
                                  setAutoSleepCountTime(0);
                                  btn_join = true;
                                  Serial.println("DoubleClick"); });
    button->attachLongPressStart([]
                                 {
                                     if(!InApp)
                                     {
                                         PowerDown();
                                     }
                                     setAutoSleepCountTime(0);
                                     btn_quit = true;
                                     InApp = false;
                                     Serial.println("LongPress"); });
}

void Title_Commit(char *Str)
{
    memset(Title_Str, '\0', strlen(Title_Str));
    memcpy(Title_Str, Str, strlen(Str));
}

struct menu_entry_type menu_entry_list[] =
    {
        {u8g2_font_open_iconic_all_2x_t, 0x7B, RTCLoop},          // clock
        {u8g2_font_open_iconic_all_2x_t, 0xAF, GPSMenuLoop},      // gps
        {u8g2_font_open_iconic_all_2x_t, 0xF8, LoRaOledLoop},     // lora
        {u8g2_font_open_iconic_all_2x_t, 0x107, imu_loop},         // IMU
        {u8g2_font_open_iconic_all_2x_t, 0x87, MsgOledLoop},      // massage
        {u8g2_font_open_iconic_embedded_2x_t, 0x48, SysOledLoop}, // sys
};

void to_right(void)
{
    static int x = 30;
    if (x > 0)
    {
        CLEAN_MENU;

        getOled()->setFont(menu_entry_list[focus_num].font);
        getOled()->drawGlyph(x - 20, 29, menu_entry_list[focus_num].icon);

        int Candidate = (focus_num + 1) <= MAX_APP ? focus_num + 1 : 0;
        getOled()->setFont(menu_entry_list[Candidate].font);
        getOled()->drawGlyph(x + 10, 29, menu_entry_list[Candidate].icon);

        Candidate = (Candidate + 1) <= MAX_APP ? Candidate + 1 : 0;
        getOled()->setFont(menu_entry_list[Candidate].font);
        getOled()->drawGlyph(x + 40, 29, menu_entry_list[Candidate].icon);

        x--;
    }
    else
    {
        focus_num < MAX_APP ? focus_num++ : focus_num = 0;
        btn_next = false;
        x = 30;
        Serial.printf("focus_num : %d\n", focus_num);
    }
}

void MenuLoop(void)
{
    static uint32_t Millis1, Millis2;
    static int x;

    if (millis() - Millis1 > 15)
    {
        button->tick();
        if (x > 0)
        {
            getOled()->setFont(u8g2_font_IPAandRUSLCD_tr);

            CLEAN_TITLE;
            getOled()->drawStr(x - 128, 8, Title_Str);

            PowerTitle();
            x--;
        }
        else
        {
            x = 0xFF;
        }
        Millis1 = millis();
    }

    if (millis() - Millis2 > 5)
    {
        if (!btn_quit && !btn_join && !btn_next && !InApp)
        {
            getOled()->drawRFrame(7, 10, 23, 22, 3);
        }
        else if (InApp)
        {
            uint8_t btn = btn_next ? 0x01 : 0x00;
            btn |= btn_join ? 0x02 : 0x00;
            menu_entry_list[focus_num].loop(btn);
            btn_join = btn_next = false;
        }
        else if (btn_join)
        {
            InApp = true;
            btn_join = btn_next = false;
        }
        else if (btn_next)
        {
            to_right();
            getOled()->drawRFrame(7, 10, 23, 22, 3);
        }
        else if (btn_quit)
        {
            CLEAN_MENU;

            getOled()->setFont(menu_entry_list[focus_num].font);
            getOled()->drawGlyph(10, 29, menu_entry_list[focus_num].icon);

            int Candidate = (focus_num + 1) < MAX_APP ? focus_num + 1 : 0;
            getOled()->setFont(menu_entry_list[Candidate].font);
            getOled()->drawGlyph(40, 29, menu_entry_list[Candidate].icon);

            getOled()->drawRFrame(7, 10, 23, 22, 3);

            btn_quit = false;
        }

        Millis2 = millis();
    }

    getOled()->sendBuffer();
}