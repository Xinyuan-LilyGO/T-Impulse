/*
 * @Author: your name
 * @Date: 2021-11-02 10:50:08
 * @LastEditTime: 2021-11-02 11:18:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \wristband-S76G\src\gps.cpp
 */

#include <TinyGPS++.h>
#include "config.h"

TinyGPSPlus *gps = nullptr;
HardwareSerial gpsPort(GPS_RX, GPS_TX);

void GPS_WaitAck(String cmd, String arg = "")
{
    while (1)
    {
        if (arg != "")
        {
            gpsPort.print(cmd);
            gpsPort.print(" ");
            gpsPort.println(arg);
        }
        else
        {
            gpsPort.println(cmd);
        }
        String ack = "";
        uint32_t smap = millis() + 500;
        while (millis() < smap)
        {
            if (gpsPort.available() > 0)
            {
                ack = gpsPort.readStringUntil('\n');
                String acc = "[" + cmd.substring(1) + "] " + "Done";
                if (ack.startsWith(acc))
                {
                    return;
                }
            }
        }
    }
}
void GPS_Init(void)
{
    gps = new TinyGPSPlus();
    gpsPort.begin(GPS_BAUD_RATE);
    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, HIGH);
    pinMode(GPS_RST, GPIO_PULLUP);
    // Set  Reset Pin as 0
    digitalWrite(GPS_RST, LOW);
    // Scope shows 1.12s (Low Period)
    delay(200);
    // Set  Reset Pin as 1
    digitalWrite(GPS_RST, HIGH);
    delay(500);

    GPS_WaitAck("@GSTP");
    GPS_WaitAck("@BSSL", "0x2EF");
    GPS_WaitAck("@GSOP", "1 1000 0");
    GPS_WaitAck("@GNS", "0x03");
    //! Start GPS connamd
    GPS_WaitAck("@GSR");
}

void GPS_loop(void)
{
    while (gpsPort.available() > 0)
    {
        gps->encode(gpsPort.read());
    }
    if (gps->charsProcessed() < 10)
    {
        Serial.println(F("WARNING: No GPS data.  Check wiring."));
    }
}