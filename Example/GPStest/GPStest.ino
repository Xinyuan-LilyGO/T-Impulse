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

bool GPS_WaitAck(String cmd, long tries=-1, String arg = "")
{
    while (tries>0 || tries==-1)
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
                Serial.println("Something on wire, reading");
                ack = gpsPort.readStringUntil('\n');
                String acc = "[" + cmd.substring(1) + "] " + "Done";
                if (ack.startsWith(acc))
                {
                    return true;
                }
            }
        }
        if(tries>0){
          tries--;
        }
    }
    if(tries==0){
      Serial.print("Command failed: ");
      Serial.print(cmd);
      Serial.println("");
      return false;
    } else {
      return true;
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


    GPS_WaitAck("@GSTP", 10);
    GPS_WaitAck("@BSSL", 10, "0x2EF");
    GPS_WaitAck("@GSOP", 10, "1 1000 0");
    GPS_WaitAck("@GNS", 10, "0x03");
    //! Start GPS connamd
    GPS_WaitAck("@GSR", 10);

}

void loop(void)
{
    static uint32_t Millis;
    if(millis() - Millis>1000)
    {
      Serial.printf("millis[%lu]\n",millis());
      Serial.println("");
      Millis = millis();
      Serial.println("Sending @VER");
      gpsPort.println("@VER");
    }

    while (gpsPort.available() > 0)
    {
        Serial.write(gpsPort.read());
    }
    
}
