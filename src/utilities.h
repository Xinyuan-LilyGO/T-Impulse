#pragma once
/*!
 * Board MUC pins definitions , Built-in pin connection
 */
#define RADIO_RESET                                 PB10
#define RADIO_MOSI                                  PB15
#define RADIO_MISO                                  PB14
#define RADIO_SCLK                                  PB13
#define RADIO_NSS                                   PB12

#define RADIO_DIO_0                                 PB11
#define RADIO_DIO_1                                 PC13
#define RADIO_DIO_2                                 PB9
#define RADIO_DIO_3                                 PB4
#define RADIO_DIO_4                                 PB3
#define RADIO_DIO_5                                 PA15

#define RADIO_OSC_PIN                               PC1
#define RADIO_ANT_SWITCH_RXTX                       PA1 //1:Rx, 0:Tx
#define LORA_BAND                                   915E6


#define UART_TX                                     PA9
#define UART_RX                                     PA10

#define GPS_RST                                     PB2
#define GPS_RX                                      PC11
#define GPS_TX                                      PC10
#define GPS_LEVEL_SHIFTER_EN                        PC6
#define GPS_BAUD_RATE                               115200
#define GPS_1PPS_INTPUT                             PB5

/******************
*
*   External pin connection
*
* ****************/
#define TTP223_VDD_PIN                              PA2
#define TTP223_TP_PIN                               PA0 //PA3
#define ADC_PIN                                     PC4
#define CHARGE_PIN                                  PB8//PA3//PA0
#define IND_LED                                     PA5
#define I2C_SCL                                     PB6
#define I2C_SDA                                     PB7
#define OLED_RESET                                  PA8

#define ICM20948_INT                                PA2//PB0


#define SSD1306_ADDRRESS                            0x3C

#define DBG_PORT Serial

#ifdef DBG_PORT
#define LOG(fmt, ...) (DBG_PORT.printf("[%d](%s): " fmt "\n", __LINE__,__func__, ##__VA_ARGS__))
#define LOG_WRITE(c)    DBG_PORT.write(c)
#else
#define LOG(fmt, ...)   //(printf("[%d](%s): " fmt "\n", __LINE__,__func__, ##__VA_ARGS__))
#define LOG_WRITE(c)    // {char ch = c;SEGGER_RTT_Write(0, (char *)&ch, 1);}
#endif



