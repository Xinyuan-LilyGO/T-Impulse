
#include <TinyGPS++.h>
#include "config.h"
#include "oled.h"
#include "menu.h"

TinyGPSPlus *gps = nullptr;

HardwareSerial gpsPort(GPS_RX, GPS_TX);
bool GPS_enable = true;

static char buf1[300];

bool isGPSenable(void) { return GPS_enable; }

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
    pinMode(PwrSwitchGPS, OUTPUT);
    digitalWrite(PwrSwitchGPS, HIGH);

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

    memset(buf1, '\0', strlen(buf1));
    // if (gps->location.isUpdated() && gps->satellites.isUpdated())
    {
        sprintf(buf1, "SATELLITES Fix Age=%d   LOCATION  Raw Lat=%s%d  [+%d billionths],  Raw Long=%s%d [+%d billionths],  Lat=%f  Long=%f ALTITUDE  Fix Age=%dms Raw=%d Meters=%d KM=%d  Feet=%d",
                gps->satellites.value(),
                gps->location.rawLat().negative ? "-" : "+",
                gps->location.rawLat().deg,
                gps->location.rawLat().billionths,
                gps->location.rawLng().negative ? "-" : "+",
                gps->location.rawLng().deg,
                gps->location.rawLng().billionths,
                gps->location.lat(),
                gps->location.lng(),
                gps->altitude.age(),
                gps->altitude.value(),
                gps->altitude.miles(),
                gps->altitude.kilometers(),
                gps->altitude.feet());
    }
}

void GPS_Sleep(void)
{
    pinMode(GPS_EN, OUTPUT);
    pinMode(PwrSwitchGPS, OUTPUT);

    digitalWrite(GPS_EN, LOW);
    digitalWrite(PwrSwitchGPS, LOW);
}

void GPS_loop(void)
{
    while (gpsPort.available() > 0)
    {
        gps->encode(gpsPort.read());
    }
    if (gps->charsProcessed() < 10 && GPS_enable)
    {
        Serial.println(F("WARNING: No GPS data.  Check wiring."));
    }
}

void GPSMenuLoop(uint8_t &BTN_state)
{
    static uint8_t select = 0;
    static int x;

    CLEAN_MENU;

    char optionbuf[10];
    sprintf(optionbuf, "[%s]gps en", GPS_enable ? "*" : " ");
    u8g2->setFont(u8g2_font_nokiafc22_tr);
    u8g2->drawStr(10, 19, optionbuf);

    //光标
    u8g2->setFont(u8g2_font_open_iconic_all_1x_t);
    u8g2->drawGlyph(0, select + 22, 0x4E); //->

    if (GPS_enable)
    {
        if (gps->location.isUpdated() && gps->satellites.isUpdated() && gps->altitude.isUpdated())
        {
            memset(buf1, '\0', sizeof(buf1));
            sprintf(buf1, "SATELLITES Fix Age=%d   LOCATION  Raw Lat=%s%d  [+%d billionths],  Raw Long=%s%d [+%d billionths],  Lat=%f  Long=%f ALTITUDE  Fix Age=%dms Raw=%d Meters=%d KM=%d  Feet=%d",
                    gps->satellites.value(),
                    gps->location.rawLat().negative ? "-" : "+",
                    gps->location.rawLat().deg,
                    gps->location.rawLat().billionths,
                    gps->location.rawLng().negative ? "-" : "+",
                    gps->location.rawLng().deg,
                    gps->location.rawLng().billionths,
                    gps->location.lat(),
                    gps->location.lng(),
                    gps->altitude.age(),
                    gps->altitude.value(),
                    gps->altitude.miles(),
                    gps->altitude.kilometers(),
                    gps->altitude.feet());
        }

        int width = -(u8g2->getStrWidth(buf1) + 200);
        if (x > width)
        {
            u8g2->setDrawColor(0x00);
            u8g2->drawBox(0, 20, 64, 12);
            u8g2->setDrawColor(0xff);

            u8g2->setFont(u8g2_font_IPAandRUSLCD_tr);

            u8g2->drawStr(x + 64, 32, buf1);

            x--;
        }
        else
        {
            x = 0;
        }
    }

    switch (BTN_state)
    {
    case 0x01:
        break;
    case 0x02:
        GPS_enable = !GPS_enable;
        GPS_enable ? GPS_Init() : GPS_Sleep();
        GPS_enable ? Title_Commit("GPS begin!") : Title_Commit("GPS shutdown!");
        break;
    default:
        break;
    }

    // u8g2->sendBuffer();
}