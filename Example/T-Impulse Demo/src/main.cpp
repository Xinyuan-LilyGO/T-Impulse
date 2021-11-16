
#include <Wire.h> // Only needed for Arduino 1.6.5 and earlier
#include <SPI.h>
#include "loramac.h"
#include "config.h"
#include "oled.h"
#include "gps.h"
#include "IMU.h"
#include "Bat.h"
#include "menu.h"
#include "lorawan.h"
#include "STM32LowPower.h"

void BoardInit(bool Anima)
{
    pinMode(PwrSwitch1_8V, OUTPUT);
    digitalWrite(PwrSwitch1_8V, HIGH);

    Serial.begin(115200);

    Wire.setSCL(IICSCL);
    Wire.setSDA(IICSDA);
    Wire.begin();

    GPS_Init();
    Bat_Init();
    pinMode(BatteryVol, INPUT_ANALOG);
    OledInit(Anima);
    IMUInit();
    RTCInit();
    LoRaInit();
    MenuInit();
}

void setup()
{
    LowPower.begin();
    LowPower.attachInterruptWakeup(
        TouchPad, [] {}, RISING, DEEP_SLEEP_MODE);
    BoardInit(true);
    IIC_Scan();
    rtc.setTime(0, 0, 0);
    Serial.println("LoRaWan Demo");
}

void loop()
{
    LoRaLoop();
    Bat_loop();
    GPS_loop();
    MenuLoop();
    SysLoop();
}
