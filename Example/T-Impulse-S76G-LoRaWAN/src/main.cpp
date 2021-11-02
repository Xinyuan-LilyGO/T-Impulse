/*
 * @Author: your name
 * @Date: 2021-11-02 09:33:08
 * @LastEditTime: 2021-11-02 14:49:43
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \wristband-S76G\src\main.cpp
 */
#include <Wire.h> // Only needed for Arduino 1.6.5 and earlier
#include <SPI.h>
#include "loramac.h"
#include "config.h"
#include "oled.h"
#include "gps.h"
#include "IMU.h"
#include "Bat.h"

void LoraWanInit(void)
{
    SPI.setMISO(LORA_MISO);
    SPI.setMOSI(LORA_MOSI);
    SPI.setSCLK(LORA_SCK);
    SPI.begin();

    pinMode(RADIO_ANT_SWITCH_RXTX, OUTPUT);
    pinMode(GPS_EN, OUTPUT);

    digitalWrite(RADIO_ANT_SWITCH_RXTX, HIGH);
    digitalWrite(GPS_EN, HIGH);
}

void BoardInit(void)
{
    pinMode(PwrSwitch1_8V, OUTPUT);
    digitalWrite(PwrSwitch1_8V, HIGH);
    pinMode(PwrSwitchGPS, OUTPUT);
    digitalWrite(PwrSwitchGPS, HIGH);
    Serial.begin(115200);

    Wire.setSCL(IICSCL);
    Wire.setSDA(IICSDA);
    Wire.begin();

    GPS_Init();
    OledInit();
    IMUInit();

    Bat_Init();
    pinMode(BatteryVol, INPUT_ANALOG);

    LoraWanInit();

    pinMode(TTP223_VDD_PIN, OUTPUT);
    pinMode(TouchPad, INPUT);
    digitalWrite(TTP223_VDD_PIN, HIGH);
}

void setup()
{
    BoardInit();
    delay(500);
    Serial.println("LoRaWan Demo");
    setupLMIC();
}

void loop()
{
    loopLMIC();
    Bat_loop();
    GPS_loop();
}
