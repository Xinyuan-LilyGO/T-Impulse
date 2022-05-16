#include "btn.h"

OneButton *button = nullptr;

void btnInit(void)
{

    pinMode(TTP223_VDD_PIN, OUTPUT);
    digitalWrite(TTP223_VDD_PIN, HIGH);

    pinMode(TOUCH_PAD_PIN, INPUT);
    button = new OneButton(TOUCH_PAD_PIN, false);
}

