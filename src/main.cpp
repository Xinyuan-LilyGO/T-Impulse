/*
*   S76G Wristband full function test firmware ,
*   Sleep power consumption is as low as about 0.7mA
*   Written on December 12, 24 Created by Lewis he
* */
/*************************************************
*
*   ___              _          _____  ______       _____
*  / _ \            (_)        /  ___||___  /      |  __ \
* / /_\ \  ___  ___  _   ___   \ `--.    / / __  __| |  \/
* |  _  | / __|/ __|| | / _ \   `--. \  / /  \ \/ /| | __
* | | | || (__ \__ \| || (_) | /\__/ /./ /    >  < | |_\ \
* \_| |_/ \___||___/|_| \___/  \____/ \_/    /_/\_\ \____/
*
************************************************/

#include <STM32LowPower.h>
#include <ICM_20948.h>
#include <LoRa.h>
#include <U8g2lib.h>
#include <TinyGPS++.h>
#include "utilities.h"

#ifdef LED_BUILTIN
#undef LED_BUILTIN
#define LED_BUILTIN     PA3
#endif

HardwareSerial  SerialGPS(GPS_RX, GPS_TX);
U8G2_SSD1306_64X32_1F_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ OLED_RESET);
ICM_20948_I2C imu;
TinyGPSPlus      gps;
bool                 sleepIn = false;
uint8_t              funcSelectIndex = 0;
uint32_t            counter = 0;
uint32_t            last = 0;
uint32_t            blinkMillis = 0;

bool touched();
void showIMUInfo(const char *prefix, float x, float y, float z);
void showGPSInfo();
void loopGPS();
void loopMag();
void loopAccel();
void loopGyr();
void loopLoRa();
void sleepWristband();
void loopSender();
void loopReciver();

const uint8_t index_max = 7;
typedef void (*funcCallBackTypedef)(void);
funcCallBackTypedef LilyGoCallBack[] = {loopGPS, loopAccel, loopGyr, loopMag, loopSender, loopReciver, sleepWristband};

/*************************************************
*  _            ______
* | |           | ___ \
* | |      ___  | |_/ /  __ _
* | |     / _ \ |    /  / _` |
* | |____| (_) || |\ \ | (_| |
* \_____/ \___/ \_| \_| \__,_|
*
************************************************/

bool isrReceived = false;

void setRadioDirection(bool rx)
{
    isrReceived = rx;
    digitalWrite(RADIO_ANT_SWITCH_RXTX, rx ? HIGH : LOW);
}

#define REG_LR_TCXO                     0x4B
#define RFLR_TCXO_TCXOINPUT_MASK        0xEF
#define RFLR_TCXO_TCXOINPUT_ON          0x10
#define RFLR_TCXO_TCXOINPUT_OFF         0x00  // Default

bool setupLoRa(void)
{
    SPI.setMISO(RADIO_MISO);
    SPI.setMOSI(RADIO_MOSI);
    SPI.setSCLK(RADIO_SCLK);
    SPI.begin();
    LoRa.setSPI(SPI);
    LoRa.setPins(RADIO_NSS, RADIO_RESET, RADIO_DIO_0);
    if (!LoRa.begin(LORA_BAND)) {
        Serial.println( "setupLoRa FAIL" );
        return false;
    }
    //! Initialize Radio ant switch pin
    pinMode(RADIO_ANT_SWITCH_RXTX, OUTPUT);
    //! Lora ANT Switch 1:Rx, 0:Tx
    setRadioDirection(true);

    pinMode(PC1, INPUT);
    digitalWrite(PC1, HIGH);
    if ( digitalRead(PC1) == 0 ) {
        uint8_t tcxo =  LoRa.readRegister( REG_LR_TCXO ) & RFLR_TCXO_TCXOINPUT_MASK;
        LoRa.writeRegister(REG_LR_TCXO, tcxo | RFLR_TCXO_TCXOINPUT_OFF);

        __HAL_RCC_GPIOD_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_Struct;
        // for LoRa sx1276 TCXO OE Pin
        GPIO_Struct.Pin = GPIO_PIN_7;
        GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_Struct.Pull = GPIO_NOPULL;
        GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOD, &GPIO_Struct);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

    }
    return true;
}

void sleepLoRa()
{
    LoRa.end();

    GPIO_InitTypeDef GPIO_Struct;
    __HAL_RCC_GPIOD_CLK_ENABLE();
    // for LoRa sx1276 TCXO OE Pin
    GPIO_Struct.Pin = GPIO_PIN_7;
    GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Struct.Pull = GPIO_NOPULL;
    GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_Struct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
}

void loopSender()
{
    if (isrReceived) {
        setRadioDirection(false);
    }
    if (millis() - last > 3000) {

        u8g2.clearBuffer();
        u8g2.setCursor(0, 8);
        u8g2.print("Sending:");
        u8g2.setCursor(0, 20);
        u8g2.print("hello ");
        u8g2.print(counter);
        u8g2.sendBuffer();


        // send packet
        LoRa.beginPacket();
        LoRa.print("hello ");
        LoRa.print(counter++);
        LoRa.endPacket();
        last = millis();
    }
}

void loopReciver()
{
    if (!isrReceived) {
        setRadioDirection(true);
        u8g2.clearBuffer();
        u8g2.drawStr(0, 12, "Start list");
        u8g2.sendBuffer();
    }
    // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // received a packet
        Serial.print("Received packet '");

        String recv = "";
        // read packet
        while (LoRa.available()) {
            recv += (char)LoRa.read();
        }

        Serial.println(recv);

        // print RSSI of packet
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());

        u8g2.clearBuffer();
        u8g2.setCursor(0, 8);
        u8g2.print("Reciver:");
        u8g2.setCursor(0, 20);
        u8g2.print(recv);
        u8g2.setCursor(0, 32);
        u8g2.print("RSSI:");
        u8g2.print(LoRa.packetRssi());
        u8g2.sendBuffer();

    }
}

/**********************************************
*  _____ ______  _____
* |  __ \| ___ \/  ___|
* | |  \/| |_/ /\ `--.
* | | __ |  __/  `--. \
* | |_\ \| |    /\__/ /
*  \____/\_|    \____/
*
************************************************/

void loopGPS()
{
    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }

    if (gps.location.isUpdated()) {

        showGPSInfo();
        Serial.print(F("LOCATION   Fix Age="));
        Serial.print(gps.location.age());
        Serial.print(F("ms Raw Lat="));
        Serial.print(gps.location.rawLat().negative ? "-" : "+");
        Serial.print(gps.location.rawLat().deg);
        Serial.print("[+");
        Serial.print(gps.location.rawLat().billionths);
        Serial.print(F(" billionths],  Raw Long="));
        Serial.print(gps.location.rawLng().negative ? "-" : "+");
        Serial.print(gps.location.rawLng().deg);
        Serial.print("[+");
        Serial.print(gps.location.rawLng().billionths);
        Serial.print(F(" billionths],  Lat="));
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(" Long="));
        Serial.println(gps.location.lng(), 6);
    }

    else if (gps.date.isUpdated()) {
        Serial.print(F("DATE       Fix Age="));
        Serial.print(gps.date.age());
        Serial.print(F("ms Raw="));
        Serial.print(gps.date.value());
        Serial.print(F(" Year="));
        Serial.print(gps.date.year());
        Serial.print(F(" Month="));
        Serial.print(gps.date.month());
        Serial.print(F(" Day="));
        Serial.println(gps.date.day());
    }

    else if (gps.time.isUpdated()) {
        Serial.print(F("TIME       Fix Age="));
        Serial.print(gps.time.age());
        Serial.print(F("ms Raw="));
        Serial.print(gps.time.value());
        Serial.print(F(" Hour="));
        Serial.print(gps.time.hour());
        Serial.print(F(" Minute="));
        Serial.print(gps.time.minute());
        Serial.print(F(" Second="));
        Serial.print(gps.time.second());
        Serial.print(F(" Hundredths="));
        Serial.println(gps.time.centisecond());
    }

    else if (gps.speed.isUpdated()) {
        Serial.print(F("SPEED      Fix Age="));
        Serial.print(gps.speed.age());
        Serial.print(F("ms Raw="));
        Serial.print(gps.speed.value());
        Serial.print(F(" Knots="));
        Serial.print(gps.speed.knots());
        Serial.print(F(" MPH="));
        Serial.print(gps.speed.mph());
        Serial.print(F(" m/s="));
        Serial.print(gps.speed.mps());
        Serial.print(F(" km/h="));
        Serial.println(gps.speed.kmph());
    }

    else if (gps.course.isUpdated()) {
        Serial.print(F("COURSE     Fix Age="));
        Serial.print(gps.course.age());
        Serial.print(F("ms Raw="));
        Serial.print(gps.course.value());
        Serial.print(F(" Deg="));
        Serial.println(gps.course.deg());
    }

    else if (gps.altitude.isUpdated()) {
        Serial.print(F("ALTITUDE   Fix Age="));
        Serial.print(gps.altitude.age());
        Serial.print(F("ms Raw="));
        Serial.print(gps.altitude.value());
        Serial.print(F(" Meters="));
        Serial.print(gps.altitude.meters());
        Serial.print(F(" Miles="));
        Serial.print(gps.altitude.miles());
        Serial.print(F(" KM="));
        Serial.print(gps.altitude.kilometers());
        Serial.print(F(" Feet="));
        Serial.println(gps.altitude.feet());
    }

    else if (gps.satellites.isUpdated()) {
        Serial.print(F("SATELLITES Fix Age="));
        Serial.print(gps.satellites.age());
        Serial.print(F("ms Value="));
        Serial.println(gps.satellites.value());
    }

    else if (gps.hdop.isUpdated()) {
        Serial.print(F("HDOP       Fix Age="));
        Serial.print(gps.hdop.age());
        Serial.print(F("ms raw="));
        Serial.print(gps.hdop.value());
        Serial.print(F(" hdop="));
        Serial.println(gps.hdop.hdop());
    }

    else if (millis() - last > 5000) {
        Serial.println();
        if (gps.location.isValid()) {
            static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
            double distanceToLondon =
                TinyGPSPlus::distanceBetween(
                    gps.location.lat(),
                    gps.location.lng(),
                    LONDON_LAT,
                    LONDON_LON);
            double courseToLondon =
                TinyGPSPlus::courseTo(
                    gps.location.lat(),
                    gps.location.lng(),
                    LONDON_LAT,
                    LONDON_LON);

            Serial.print(F("LONDON     Distance="));
            Serial.print(distanceToLondon / 1000, 6);
            Serial.print(F(" km Course-to="));
            Serial.print(courseToLondon, 6);
            Serial.print(F(" degrees ["));
            Serial.print(TinyGPSPlus::cardinal(courseToLondon));
            Serial.println(F("]"));
        }


        u8g2.clearBuffer();
        u8g2.setFontMode(1);
        u8g2.setFont(u8g2_font_5x8_t_cyrillic);
        u8g2.setCursor(0, 8);
        u8g2.print("DIAGS:");
        u8g2.print(gps.charsProcessed());
        u8g2.setCursor(0, 20);
        u8g2.print("Fix:");
        u8g2.print(gps.sentencesWithFix());
        u8g2.setCursor(0, 32);
        u8g2.print("Passed:");
        u8g2.print(gps.passedChecksum());
        // u8g2.setCursor(45, 32);
        // u8g2.print("3D");
        u8g2.sendBuffer();

        Serial.print(F("DIAGS      Chars="));
        Serial.print(gps.charsProcessed());
        Serial.print(F(" Sentences-with-Fix="));
        Serial.print(gps.sentencesWithFix());
        Serial.print(F(" Failed-checksum="));
        Serial.print(gps.failedChecksum());
        Serial.print(F(" Passed-checksum="));
        Serial.println(gps.passedChecksum());

        if (gps.charsProcessed() < 10)
            Serial.println(F("WARNING: No GPS data.  Check wiring."));

        last = millis();
        Serial.println();
    }
}

void waitAck()
{
    uint32_t smap = millis() + 2000;
    while (millis() < smap) {
        while (SerialGPS.available() > 0) {
            Serial.write(SerialGPS.read());
        }
    }
}

void waitAck(String cmd, String arg = "")
{
    while (1) {
        if (arg != "") {
            SerialGPS.print(cmd);
            SerialGPS.print(" ");
            SerialGPS.println(arg);
        } else {
            SerialGPS.println(cmd);
        }
        String ack = "";
        uint32_t smap = millis() + 500;
        while (millis() < smap) {
            if (SerialGPS.available() > 0) {
                ack = SerialGPS.readStringUntil('\n');
                String acc = "[" + cmd.substring(1) + "] " + "Done";
                if (ack.startsWith(acc)) {
                    return;
                }
            }
        }
    }
}

void sleepGPS(void)
{
    waitAck("@GSTP");
    waitAck("@BUP");
    waitAck("@SLP", "2");
    delay(5);
    SerialGPS.end();
}

void setupGPS()
{
    SerialGPS.setRx(GPS_RX);
    SerialGPS.setTx(GPS_TX);
    SerialGPS.begin(GPS_BAUD_RATE);

    //! Added 1pps intput
    pinMode(GPS_1PPS_INTPUT, INPUT);

    pinMode(GPS_LEVEL_SHIFTER_EN, OUTPUT);
    digitalWrite(GPS_LEVEL_SHIFTER_EN, HIGH);

    pinMode(GPS_RST, OUTPUT);
    //Set  Reset Pin as 0
    digitalWrite(GPS_RST, LOW);
    //Scope shows 1.12s (Low Period)
    delay(200);
    //Set  Reset Pin as 1
    digitalWrite(GPS_RST, HIGH);
    delay(100);

    //! Start GPS connamd
    waitAck("@GSR");
}

void showGPSInfo()
{
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    // u8g2.setFont(u8g2_font_crox2t_tf);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 8);
    u8g2.print("lat:");
    u8g2.print(gps.location.lat());
    u8g2.setCursor(0, 20);
    u8g2.print("lng:");
    u8g2.print(gps.location.lng());
    u8g2.setCursor(0, 32);
    u8g2.print("sate:");
    u8g2.print(gps.satellites.value());
    u8g2.setCursor(45, 32);
    u8g2.print(gps.sentencesWithFix() ? "2D" : "3D");
    u8g2.sendBuffer();
}

/*********************************************
*   _____  _      _____ ______
* |  _  || |    |  ___||  _  \
* | | | || |    | |__  | | | |
* | | | || |    |  __| | | | |
* \ \_/ /| |____| |___ | |/ /
*  \___/ \_____/\____/ |___/
************************************************/
bool oledFind = false;

void sleepOLED()
{
    u8g2.sleepOn();
}

void wakeupOLED()
{
    u8g2.sleepOff();
}

bool setupOLED(void)
{
    Wire.beginTransmission(SSD1306_ADDRRESS);
    if (Wire.endTransmission() == 0) {
        u8g2.begin();
        u8g2.clearBuffer();
        u8g2.setFontMode(1);
        u8g2.setFont(u8g2_font_cu12_tr);
        u8g2.setCursor(10, 15);
        u8g2.print(F("SoftRF"));
        u8g2.setCursor(10, 30);
        u8g2.print(F("LilyGo"));
        u8g2.sendBuffer();
        Serial.println("Device FIND");
        oledFind = true;
    } else {
        Serial.println( "setupOLED FAIL" );
        oledFind = false;
    }
    return oledFind;
}

/**********************************************
*  _____  _____  _   _  _____  _____ ______
* /  ___||  ___|| \ | |/  ___||  _  || ___ \
* \ `--. | |__  |  \| |\ `--. | | | || |_/ /
*  `--. \|  __| | . ` | `--. \| | | ||    /
* /\__/ /| |___ | |\  |/\__/ /\ \_/ /| |\ \
* \____/ \____/ \_| \_/\____/  \___/ \_| \_|
*
**********************************************/

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
    float aval = abs(val);
    if (val < 0) {
        Serial.print("-");
    } else {
        Serial.print(" ");
    }
    for ( uint8_t indi = 0; indi < leading; indi++ ) {
        uint32_t tenpow = 0;
        if ( indi < (leading - 1) ) {
            tenpow = 1;
        }
        for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
            tenpow *= 10;
        }
        if ( aval < tenpow) {
            Serial.print("0");
        } else {
            break;
        }
    }
    if (val < 0) {
        Serial.print(-val);
    } else {
        Serial.print(val);
    }
}

void printScaledAGMT( ICM_20948_AGMT_t agmt)
{
    Serial.print("Scaled. Acc (mg) [");
    printFormattedFloat( imu.accX(), 5, 2 );
    Serial.print(",");
    printFormattedFloat( imu.accY(), 5, 2 );
    Serial.print(",");
    printFormattedFloat( imu.accZ(), 5, 2 );
    Serial.print(" ], Gyr(DPS)[");
    printFormattedFloat( imu.gyrX(), 5, 2 );
    Serial.print(", ");
    printFormattedFloat( imu.gyrY(), 5, 2 );
    Serial.print(", ");
    printFormattedFloat( imu.gyrZ(), 5, 2 );
    Serial.print("], Mag(uT)[ ");
    printFormattedFloat( imu.magX(), 5, 2 );
    Serial.print(", ");
    printFormattedFloat( imu.magY(), 5, 2 );
    Serial.print(", ");
    printFormattedFloat( imu.magZ(), 5, 2 );
    Serial.print("], Tmp(C)[ ");
    printFormattedFloat( imu.temp(), 5, 2 );
    Serial.print("]");
    Serial.println();
}

bool setupSensor()
{
    imu.begin();
    if ( imu.status != ICM_20948_Stat_Ok ) {
        Serial.println( "setupSensor FAIL" );
        return false;
    }
    Serial.println("ICM_20948_Stat_Ok");
    return true;
}

void loopAccel()
{
    if ( imu.dataReady() ) {
        imu.getAGMT();                // The values are only updated when you call 'getAGMT'
        printScaledAGMT( imu.agmt);
        showIMUInfo("acc", imu.accX(), imu.accY(), imu.accZ());
        delay(200);
    }
}

void loopGyr()
{
    if ( imu.dataReady() ) {
        imu.getAGMT();                // The values are only updated when you call 'getAGMT'
        printScaledAGMT( imu.agmt);
        showIMUInfo("gyr", imu.gyrX(), imu.gyrY(), imu.gyrZ());
        delay(200);
    }
}

void loopMag()
{
    if ( imu.dataReady() ) {
        imu.getAGMT();                // The values are only updated when you call 'getAGMT'
        printScaledAGMT( imu.agmt);
        showIMUInfo("mag", imu.magX(), imu.magY(), imu.magZ());
        delay(200);
    }
}

void sleepSensor()
{
    imu.sleep(true);
    imu.stopMagnetometer();
    // imu.lowPower(true);
}

void wakeupSensor()
{
    imu.sleep(false);
    imu.startupMagnetometer();
    // imu.lowPower(false);
}


void showIMUInfo(const char *prefix, float x, float y, float z)
{
    u8g2.clearBuffer();
    // u8g2.setFontMode(1);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 8);
    u8g2.print(prefix);
    u8g2.print("X:");
    u8g2.print(x, 2);
    u8g2.setCursor(0, 20);
    u8g2.print(prefix);
    u8g2.print("Y:");
    u8g2.print(y, 2);
    u8g2.setCursor(0, 32);
    u8g2.print(prefix);
    u8g2.print("Z:");
    u8g2.print(z, 2);
    u8g2.sendBuffer();
}

/**********************************************
*  _____  _____  _   _  _____  _   _
* |_   _||  _  || | | |/  __ \| | | |
*   | |  | | | || | | || /  \/| |_| |
*   | |  | | | || | | || |    |  _  |
*   | |  \ \_/ /| |_| || \__/\| | | |
*   \_/   \___/  \___/  \____/\_| |_/
*
************************************************/
uint8_t touchStatus = 0;
bool touchPressed = 0;
uint32_t firstPressMillis = 0;
const uint32_t LONGPRESSMILLIS = 3000;

void enableTouch()
{
    digitalWrite(TTP223_VDD_PIN, HIGH);
}

bool touched()
{
    if (digitalRead(TTP223_TP_PIN)) {
        delay(200);
        //Wait for release
        while (digitalRead(TTP223_TP_PIN));
        return true;
    }
    return false;
}

uint8_t  touchPoll()
{
    uint8_t event = 0;
    if (!touched()) {
        if (touchPressed) {
            touchPressed = false;
            firstPressMillis = 0;
            touchStatus = 0;
        }
        return event;
    }

    switch (touchStatus) {
    case 0:
        touchPressed = true;
        firstPressMillis = millis();
        touchStatus++;
        event = 1;
        break;
    case 1:
        if (millis() - firstPressMillis > LONGPRESSMILLIS) {
            event = 2;
        }
        break;
    default:
        break;
    }
    return event;
}

/**********************************************
* ___  ___  ___   _____  _   _
* |  \/  | / _ \ |_   _|| \ | |
* | .  . |/ /_\ \  | |  |  \| |
* | |\/| ||  _  |  | |  | . ` |
* | |  | || | | | _| |_ | |\  |
* \_|  |_/\_| |_/ \___/ \_| \_/
*
************************************************/
void deviceScan(void)
{
    uint8_t err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
            nDevices++;
        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}

void repetitionsIncrease()
{
    // This function will be called once on device wakeup
    // You can do some little operations here (like changing variables which will be used in the loop)
    // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
}

void boardInit()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TTP223_VDD_PIN, OUTPUT);
    pinMode(TTP223_TP_PIN, INPUT);

    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, HIGH);
    delay(80);
    digitalWrite(OLED_RESET, LOW);
    delay(10);
    digitalWrite(OLED_RESET, HIGH);

#ifndef USBCON
    Serial.setRx(UART_RX);
    Serial.setTx(UART_TX);
#endif
    Serial.begin(9600);

    Wire.setSCL(I2C_SCL);
    Wire.setSDA(I2C_SDA);
    Wire.begin();
}


void sleepPeripherals()
{
    Serial.println("sleep Sensor");
    sleepSensor();
    Serial.println("sleep LoRa .");
    sleepGPS();
    Serial.println("sleep GPS .");
    sleepLoRa();
    sleepOLED();
    Serial.println("sleep OLED .");
    Serial.end();
    Wire.end();
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(LED_BUILTIN, INPUT_FLOATING);
}

void wakeupPeripherals()
{
    setupOLED();
    setupSensor();
    setupLoRa();
    wakeupSensor();
    setupGPS();
}

void sleepWristband()
{
    u8g2.clearBuffer();
    u8g2.setCursor(0, 8);
    u8g2.print("sleepMCU");
    u8g2.sendBuffer();

    sleepPeripherals();

    LowPower.deepSleep();

    boardInit();

    Serial.print("Wakeup:");
    Serial.println(millis());

    wakeupPeripherals();
    sleepIn = true;
}



void setup()
{
    boardInit();

    deviceScan();

    LowPower.begin();

    LowPower.attachInterruptWakeup(TTP223_TP_PIN, repetitionsIncrease, RISING);

    Serial.println("LowPower.begin()");
    setupSensor();
    setupOLED();
    setupLoRa();
    setupGPS();
    enableTouch();
}


void loop()
{
    if (millis() - blinkMillis > 1000) {
        blinkMillis = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    if (touched() || sleepIn) {
        sleepIn = false;
        funcSelectIndex++;
        funcSelectIndex %= index_max;
        Serial.print("funcSelectIndex:");
        Serial.println(funcSelectIndex);
    }
    if (LilyGoCallBack[funcSelectIndex]) {
        LilyGoCallBack[funcSelectIndex]();
    }
}





