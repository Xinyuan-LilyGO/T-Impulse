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
    pinMode(PWR_1_8V_PIN, OUTPUT);
    digitalWrite(PWR_1_8V_PIN, HIGH);
    pinMode(PWR_GPS_PIN, OUTPUT);
    digitalWrite(PWR_GPS_PIN, HIGH);
    Serial.begin(115200);

    Wire.setSCL(IICSCL);
    Wire.setSDA(IICSDA);
    Wire.begin();

    gps_init();
    oled_init();
    imu_init();
    bat_init();
    pinMode(BAT_VOLT_PIN, INPUT_ANALOG);

    LoraWanInit();

    pinMode(TTP223_VDD_PIN, OUTPUT);
    pinMode(TOUCH_PAD_PIN, INPUT);
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
    bat_loop();
    gps_loop();
}
