#pragma once

#define DEBUG(x) Serial.print(x);
#define DEBUGLN(x) Serial.println(x);
//GPS
#define PIN_GPS_RST PB2
#define PIN_GPS_RX PC11
#define PIN_GPS_TX PC10
#define PIN_GPS_LEVEL_SHIFTER_EN PC6
#define GPS_BAUD_RATE 115200
//LORA
#define PIN_LORA_RST PB10
#define PIN_LORA_DIO0 PB11
#define PIN_LORA_MOSI PB15
#define PIN_LORA_MISO PB14
#define PIN_LORA_SCK PB13
#define PIN_LORA_NSS PB12
#define LoRa_frequency 433E6
#define PIN_RADIO_ANT_SWITCH_RXTX PA1
//SSD1306
#define PIN_SDA PB7 
#define PIN_SCL PB6   
#define PIN_OLED_RESET PA8 // Reset pin # (or -1 if sharing Arduino reset pin)
//TOUCH
#define PIN_TTP223_VDD PA2
#define PIN_TOUCH_BTN PA0
//ICM20948
#define ICM20948_ADDR 0x69

#define PIN_SHOCK PA5

#define PIN_PWR_ON_GPS PA3
#define PIN_PWR_ON_1_8V PB0
#define PIN_BAT_VOLT PC4
