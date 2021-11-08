/*
 * @Author: your name
 * @Date: 2021-11-05 14:53:22
 * @LastEditTime: 2021-11-05 15:41:57
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \T-Impulse-S76G-LoRaWAN\src\btn.cpp
 */
#include "btn.h"

OneButton *button = nullptr;

void btnInit(void)
{

    pinMode(TTP223_VDD_PIN, OUTPUT);
    digitalWrite(TTP223_VDD_PIN, HIGH);

    pinMode(TouchPad, INPUT);
    button = new OneButton(TouchPad, false);
}

