#include <Arduino.h>
#include <HardwareSerial.h>

#define PwrSwitch1_8V PB0
#define PwrSwitchGPS PA3

#define GPS_TX PC10
#define GPS_EN PC6
#define GPS_1PPS PB5
#define GPS_RST PB2
#define GPS_RX PC11
#define GPS_BAUD_RATE 115200

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

void setup(void)
{
    pinMode(PwrSwitch1_8V, OUTPUT);
    digitalWrite(PwrSwitch1_8V, HIGH);

    pinMode(PwrSwitchGPS, OUTPUT);
    digitalWrite(PwrSwitchGPS, HIGH);

    Serial.begin(115200);
    delay(5000);

    Serial.println("GPS TEST!!");

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

void loop(void)
{
    static uint32_t Millis;
    if(millis() - Millis>500)
    {
      Serial.printf("millis[%lu]\n",millis());
      Millis = millis();
    }
    
    while (gpsPort.available() > 0)
    {
        Serial.write(gpsPort.read());
    }
    
}
