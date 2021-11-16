
#include "btn.h"

OneButton *button = nullptr;

void btnInit(void)
{

    pinMode(TTP223_VDD_PIN, OUTPUT);
    digitalWrite(TTP223_VDD_PIN, HIGH);

    pinMode(TouchPad, INPUT);
    button = new OneButton(TouchPad, false);
}

