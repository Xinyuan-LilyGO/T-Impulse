/*
 * @Author: your name
 * @Date: 2021-11-02 11:27:42
 * @LastEditTime: 2021-11-08 18:08:30
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \wristband-S76G\src\Bat.cpp
 */
#include <Arduino.h>
#include "Bat.h"
#include "config.h"
#include "oled.h"
#include "gps.h"
#include "lorawan.h"
#include "STM32LowPower.h"
#include "main.h"

ADC_HandleTypeDef hadc;
uint32_t Volt = 0;

static void MX_ADC_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    hadc.Instance = ADC1;
    hadc.Init.OversamplingMode = DISABLE;
    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ContinuousConvMode = ENABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerFrequencyMode = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
        Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_14;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
}

void Bat_Init(void)
{
    MX_ADC_Init();
    HAL_ADC_Start(&hadc);
}

void Bat_loop(void)
{
    static uint32_t vot, Millis;
    static uint8_t i;
    if (i < 20)
    {
        i++;
        vot += HAL_ADC_GetValue(&hadc);
    }
    else
    {
        Volt = vot / i;
        vot = 0;
        i = 0;
    }

    if (millis() - Millis > 5000)
    {
        if (HAL_ADC_GetValue(&hadc) < 2150)
        {
            u8g2->clearBuffer();
            u8g2->setFont(u8g2_font_battery19_tn);
            u8g2->drawGlyph(22, 4, 0x61);
            u8g2->sendBuffer();
            delay(2000);
            PowerDown();
        }
        Millis = millis();
    }
}

void PowerTitle(void)
{
    u8g2->setDrawColor(0x00);
    u8g2->drawBox(56, 0, 9, 10);
    u8g2->setDrawColor(0xff);

    u8g2->drawFrame(57, 2, 6, 4);
    u8g2->drawVLine(63, 3, 2);

    uint8_t vol = (constrain(Volt, 2100, 2500) - 2100) / 100;
    u8g2->drawFrame(58, 3, vol, 2);
}

void PowerDown(void)
{
    GPS_Sleep();
    LoRa_Sleep();
    OledSleep();

    gps = nullptr;
    button = nullptr;
    u8g2 = nullptr;

    HAL_ADC_Stop(&hadc);
    HAL_ADC_DeInit(&hadc);
    __HAL_RCC_GPIOC_CLK_DISABLE();

    LowPower.deepSleep();
    BoardInit(false);
}