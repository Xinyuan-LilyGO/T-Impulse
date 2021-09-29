#include <Wire.h> // Only needed for Arduino 1.6.5 and earlier
#include <SPI.h>
#include <LoRa.h>      //https://github.com/sandeepmistry/arduino-LoRa
#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus
#include <U8g2lib.h>   //https://github.com/olikraus/u8g2
#include <ICM_20948.h>
#include "STM32LowPower.h"

#define DEBUG(x) Serial.print(x);
#define DEBUGLN(x) Serial.println(x);
//GPS
#define GPS_RST PB2
#define GPS_RX PC11
#define GPS_TX PC10
#define GPS_LEVEL_SHIFTER_EN PC6
#define GPS_BAUD_RATE 115200
//LORA
#define LORA_RST PB10
#define LORA_DIO0 PB11
#define LORA_MOSI PB15
#define LORA_MISO PB14
#define LORA_SCK PB13
#define LORA_NSS PB12
#define LoRa_frequency 868E6
#define RADIO_ANT_SWITCH_RXTX PA1
//SSD1306
#define MySDA PB7      //ICM20948
#define MySCL PB6      //ICM20948
#define OLED_RESET PA8 // Reset pin # (or -1 if sharing Arduino reset pin)
//TOUCH
#define TTP223_VDD_PIN PA2
#define TouchPad PA0
//ICM20948
#define ICM20948_ADDR 0x69

#define PwrSwitch1_8V PA4
#define BatteryVol PC4

void GPS(void);
void Accel(void);
void Accel2(void);
void Gyr(void);
void Mag(void);
void Sender(void);
void Reciver(void);
void BatteryVoltage(void);

U8G2_SSD1306_64X32_1F_F_HW_I2C u8g2(U8G2_R0, OLED_RESET, MySCL, MySDA);
TinyGPSPlus gps;
HardwareSerial gpsPort(GPS_RX, GPS_TX);
ICM_20948_I2C imu;

const uint8_t index_max = 8;
typedef void (*funcCallBackTypedef)(void);
funcCallBackTypedef LilyGoCallBack[] = {GPS, Accel, Accel2, Gyr, Mag, Sender, Reciver, BatteryVoltage};
uint8_t funcSelectIndex = 0;
 
uint32_t Millis = 0;
uint32_t Last = 0;
uint32_t LoRa_Count = 0;
uint8_t GPS_count = 0;
double Lat = 0, Long = 0;
int Value = 0;
/***
 *      _______               _     
 *     |__   __|             | |    
 *        | | ___  _   _  ___| |__  
 *        | |/ _ \| | | |/ __| '_ \ 
 *        | | (_) | |_| | (__| | | |
 *        |_|\___/ \__,_|\___|_| |_|
 *                                  
 *                                  
 */
//Back to press time
int TouchCallback(void)
{
  int TouchMillis = millis();
  if (digitalRead(TouchPad))
  {
    while (digitalRead(TouchPad))
    {
      if ((int)millis() - TouchMillis > 3000)
        break;
    }
    return ((int)millis() - TouchMillis);
  }
  return 0;
}
/***
 *       _____ _____ _____  __ ____   ___    __     ____  _      ______ _____  
 *      / ____/ ____|  __ \/_ |___ \ / _ \  / /    / __ \| |    |  ____|  __ \ 
 *     | (___| (___ | |  | || | __) | | | |/ /_   | |  | | |    | |__  | |  | |
 *      \___ \\___ \| |  | || ||__ <| | | | '_ \  | |  | | |    |  __| | |  | |
 *      ____) |___) | |__| || |___) | |_| | (_) | | |__| | |____| |____| |__| |
 *     |_____/_____/|_____/ |_|____/ \___/ \___/   \____/|______|______|_____/ 
 *                                                                             
 *                                                                             
 */
void SSD1306_Init(void)
{
  u8g2.begin();
  u8g2.setContrast(0x00);
  u8g2.clearBuffer();
  u8g2.setFontMode(1); // Transparent
  u8g2.setFontDirection(0);
  u8g2.setFont(u8g2_font_fub14_tf);
  u8g2.drawStr(2, 18, "LilyGo");
  u8g2.sendBuffer();
  for (int i = 0; i < 0xFF; i++)
  {
    u8g2.setContrast(i);
    u8g2.drawFrame(2, 25, 60, 6);
    u8g2.drawBox(3, 26, (uint8_t)(0.231 * i), 5);
    u8g2.sendBuffer();
  }
  for (int i = 0; i < 0xFF; i++)
  {
    u8g2.setContrast(0xFF - i);
    delay(10);
  }
}
void SSD1306_Sleep(void)
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
  u8g2.drawStr(12, 8, "Sleep");
  u8g2.sendBuffer();
  delay(3000);
  u8g2.sleepOn();
}
/***
 *      _           _____       
 *     | |         |  __ \      
 *     | |     ___ | |__) |__ _ 
 *     | |    / _ \|  _  // _` |
 *     | |___| (_) | | \ \ (_| |
 *     |______\___/|_|  \_\__,_|
 *                              
 *                              
 */

bool isrReceived = false;

void setRadioDirection(bool rx)
{
  isrReceived = rx;
  digitalWrite(RADIO_ANT_SWITCH_RXTX, rx ? HIGH : LOW);
}

#define REG_LR_TCXO 0x4B
#define RFLR_TCXO_TCXOINPUT_MASK 0xEF
#define RFLR_TCXO_TCXOINPUT_ON 0x10
#define RFLR_TCXO_TCXOINPUT_OFF 0x00 // Default

bool LoRa_Init(void)
{
  SPI.setMISO(LORA_MISO);
  SPI.setMOSI(LORA_MOSI);
  SPI.setSCLK(LORA_SCK);
  SPI.begin();
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LoRa_frequency))
  {
    DEBUGLN(F("Starting LoRa failed!"));
    return false;
  }
  //! Initialize Radio ant switch pin
  pinMode(RADIO_ANT_SWITCH_RXTX, OUTPUT);
  //! Lora ANT Switch 1:Rx, 0:Tx
  setRadioDirection(true);
  pinMode(PC1, INPUT);
  delay(10);
  int pin_state = digitalRead(PC1);
  if (pin_state == false)
  {
    uint8_t tcxo = LoRa.readRegister(REG_LR_TCXO) & RFLR_TCXO_TCXOINPUT_MASK;
    LoRa.writeRegister(REG_LR_TCXO, tcxo | RFLR_TCXO_TCXOINPUT_OFF);

    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_Struct;
    // for LoRa sx1276 TCXO OE Pin
    GPIO_Struct.Pin = GPIO_PIN_7;
    GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Struct.Pull = GPIO_NOPULL;
    GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_Struct);

    /* HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    delay(5); */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    delay(10);
  }
  return true;
}
void LoRa_Sleep(void)
{
  LoRa.end();
  DEBUGLN("LoRa Sleep");
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
void Sender(void)
{
  if (isrReceived)
  {
    setRadioDirection(false);
  }
  if ((millis() - Millis) > 500)
  {
    LoRa.beginPacket();
    LoRa.print("LilyGo ");
    LoRa.print(LoRa_Count);
    LoRa.endPacket();

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
    u8g2.drawStr(0, 8, "LoRa Send");
    u8g2.setCursor(0, 20);
    u8g2.print("LilyGo ");
    u8g2.print(LoRa_Count++);
    u8g2.sendBuffer();
    Millis = millis();
  }
}

void Reciver(void)
{
  static int RSSI;
  static String recv;
  if (!isrReceived)
  {
    setRadioDirection(true);
  }

  int packetSize = LoRa.parsePacket();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
  u8g2.drawStr(0, 8, "LoRa Rec");
  u8g2.setCursor(0, 16);
  u8g2.print("RSSI:");
  if (packetSize)
  {
    // received a packet
    DEBUG("Received packet '");

    recv = "";
    // read packet
    while (LoRa.available())
    {
      recv += (char)LoRa.read();
    }
    RSSI = LoRa.packetRssi();
    DEBUGLN(recv);
    // print RSSI of packet
    DEBUG("' with RSSI ");
    DEBUGLN(RSSI);
  }
  u8g2.print(RSSI);
  u8g2.setCursor(0, 24);
  u8g2.println(recv);
  u8g2.sendBuffer();
}

/***
 *       _____ _____   _____ 
 *      / ____|  __ \ / ____|
 *     | |  __| |__) | (___  
 *     | | |_ |  ___/ \___ \ 
 *     | |__| | |     ____) |
 *      \_____|_|    |_____/ 
 *                           
 *                           
 */
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
  gpsPort.begin(GPS_BAUD_RATE);
  pinMode(GPS_LEVEL_SHIFTER_EN, OUTPUT);
  digitalWrite(GPS_LEVEL_SHIFTER_EN, HIGH);
  pinMode(GPS_RST, GPIO_PULLUP);
  //Set  Reset Pin as 0
  digitalWrite(GPS_RST, LOW);
  //Scope shows 1.12s (Low Period)
  delay(200);
  //Set  Reset Pin as 1
  digitalWrite(GPS_RST, HIGH);
  delay(100);

  GPS_WaitAck("@GSTP");
  GPS_WaitAck("@BSSL", "0x2EF");
  GPS_WaitAck("@GSOP", "1 1000 0");
  GPS_WaitAck("@GNS", "0x03");
  //! Start GPS connamd
  GPS_WaitAck("@GSR");
}

void GPS_Sleep(void)
{
  gpsPort.flush();
  GPS_WaitAck("@GSTP");
  GPS_WaitAck("@SLP", "2");
  DEBUGLN("GPS SLEEP!!");
  gpsPort.end();
}

void GPS(void)
{

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
  u8g2.drawStr(0, 8, "GPS test");

  switch (GPS_count)
  {
  case 0:
    u8g2.drawDisc(58, 4, 4, U8G2_DRAW_UPPER_RIGHT);
    GPS_count++;
    break;
  case 1:
    u8g2.drawDisc(58, 4, 4, U8G2_DRAW_UPPER_LEFT);
    GPS_count++;
    break;
  case 2:
    u8g2.drawDisc(58, 4, 4, U8G2_DRAW_LOWER_LEFT);
    GPS_count++;
    break;
  default:
    u8g2.drawDisc(58, 4, 4, U8G2_DRAW_LOWER_RIGHT);
    GPS_count = 0;
    break;
  }
  u8g2.setCursor(0, 16);
  u8g2.print(F("Fix Val="));
  u8g2.setCursor(48, 16);
  u8g2.print(Value);

  u8g2.setCursor(0, 24);
  u8g2.print(F("Lat ="));
  u8g2.setCursor(34, 24);
  u8g2.print(Lat);

  u8g2.setCursor(0, 32);
  u8g2.print(F("Long ="));
  u8g2.setCursor(42, 32);
  u8g2.print(Long);
  u8g2.sendBuffer();
  // Dispatch incoming characters
  while (gpsPort.available() > 0)
  {
    gps.encode(gpsPort.read());
  }

  if (gps.charsProcessed() < 10)
  {
    DEBUGLN(F("WARNING: No GPS data.  Check wiring."));
  }

  if (gps.location.isUpdated())
  {
    DEBUG(F("LOCATION   Fix Age="));
    DEBUG(gps.location.age());
    DEBUG(F("ms Raw Lat="));
    DEBUG(gps.location.rawLat().negative ? "-" : "+");
    DEBUG(gps.location.rawLat().deg);
    DEBUG("[+");
    DEBUG(gps.location.rawLat().billionths);
    DEBUG(F(" billionths],  Raw Long="));
    DEBUG(gps.location.rawLng().negative ? "-" : "+");
    DEBUG(gps.location.rawLng().deg);
    DEBUG("[+");
    DEBUG(gps.location.rawLng().billionths);
    DEBUG(F(" billionths],  Lat="));
    DEBUG(gps.location.lat());
    Lat = gps.location.lat();
    DEBUG(F(" Long="));
    DEBUGLN(gps.location.lng());
    Long = gps.location.lng();
  }
  if (gps.date.isUpdated())
  {
    DEBUG(F("DATE       Fix Age="));
    DEBUG(gps.date.age());
    DEBUG(F("ms Raw="));
    DEBUG(gps.date.value());
    DEBUG(F(" Year="));
    DEBUG(gps.date.year());
    DEBUG(F(" Month="));
    DEBUG(gps.date.month());
    DEBUG(F(" Day="));
    DEBUGLN(gps.date.day());
  }
  if (gps.time.isUpdated())
  {
    DEBUG(F("TIME       Fix Age="));
    DEBUG(gps.time.age());
    DEBUG(F("ms Raw="));
    DEBUG(gps.time.value());
    DEBUG(F(" Hour="));
    DEBUG(gps.time.hour());
    DEBUG(F(" Minute="));
    DEBUG(gps.time.minute());
    DEBUG(F(" Second="));
    DEBUG(gps.time.second());
    DEBUG(F(" Hundredths="));
    DEBUGLN(gps.time.centisecond());
  }
  if (gps.speed.isUpdated())
  {
    DEBUG(F("SPEED      Fix Age="));
    DEBUG(gps.speed.age());
    DEBUG(F("ms Raw="));
    DEBUG(gps.speed.value());
    DEBUG(F(" Knots="));
    DEBUG(gps.speed.knots());
    DEBUG(F(" MPH="));
    DEBUG(gps.speed.mph());
    DEBUG(F(" m/s="));
    DEBUG(gps.speed.mps());
    DEBUG(F(" km/h="));
    DEBUGLN(gps.speed.kmph());
  }
  if (gps.course.isUpdated())
  {
    DEBUG(F("COURSE     Fix Age="));
    DEBUG(gps.course.age());
    DEBUG(F("ms Raw="));
    DEBUG(gps.course.value());
    DEBUG(F(" Deg="));
    DEBUGLN(gps.course.deg());
  }
  if (gps.altitude.isUpdated())
  {
    DEBUG(F("ALTITUDE   Fix Age="));
    DEBUG(gps.altitude.age());
    DEBUG(F("ms Raw="));
    DEBUG(gps.altitude.value());
    DEBUG(F(" Meters="));
    DEBUG(gps.altitude.meters());
    DEBUG(F(" Miles="));
    DEBUG(gps.altitude.miles());
    DEBUG(F(" KM="));
    DEBUG(gps.altitude.kilometers());
    DEBUG(F(" Feet="));
    DEBUGLN(gps.altitude.feet());
  }
  if (gps.satellites.isUpdated())
  {
    DEBUG(F("SATELLITES Fix Age="));
    DEBUG(gps.satellites.age());
    DEBUG(F("ms Value="));
    DEBUGLN(gps.satellites.value());
    Value = gps.satellites.value();
  }
  else if (gps.hdop.isUpdated())
  {
    DEBUG(F("HDOP       Fix Age="));
    DEBUG(gps.hdop.age());
    DEBUG(F("ms Value="));
    DEBUGLN(gps.hdop.value());
  }
}
/***
 *      _____ _____ __  __ ___   ___   ___  _  _   ___  
 *     |_   _/ ____|  \/  |__ \ / _ \ / _ \| || | / _ \ 
 *       | || |    | \  / |  ) | | | | (_) | || || (_) |
 *       | || |    | |\/| | / /| | | |\__, |__   _> _ < 
 *      _| || |____| |  | |/ /_| |_| |  / /   | || (_) |
 *     |_____\_____|_|  |_|____|\___/  /_/    |_| \___/ 
 *                                                      
 *                                                      
 */
bool ICM20948_Init(void)
{
  imu.begin();
  if (imu.status != ICM_20948_Stat_Ok)
  {
    Serial.println("setupSensor FAIL");
    return false;
  }
  Serial.println("ICM_20948_Stat_Ok");
  return true;
}

void ICM20948_Sleep(void)
{
  imu.sleep(true);
  DEBUGLN("ICM20948 SLEEP!!");
}
/***
 *                       
 *         /\            
 *        /  \   ___ ___ 
 *       / /\ \ / __/ __|
 *      / ____ \ (_| (__ 
 *     /_/    \_\___\___|
 *                       
 *                       
 */
void Accel(void)
{
  if (millis() - Millis > 100)
  {
    if (imu.dataReady())
    {
      imu.getAGMT();
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
      u8g2.drawStr(0, 8, "Acc (mg)");
      u8g2.setCursor(0, 16);
      u8g2.print("x=");
      DEBUG("ACC(mg)  X=");
      u8g2.print(imu.accX());
      DEBUG(imu.accX());
      u8g2.setCursor(0, 24);
      u8g2.print("y=");
      DEBUG("  Y=");
      u8g2.print(imu.accY());
      DEBUG(imu.accY());
      u8g2.setCursor(0, 32);
      u8g2.print("z=");
      DEBUG("  Z=");
      u8g2.print(imu.accZ());
      DEBUGLN(imu.accZ());
      u8g2.sendBuffer();
      Millis = millis();
    }
  }
}

void Accel2(void)
{

  if (imu.dataReady())
  {
    imu.getAGMT();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
    u8g2.drawStr(2, 20, "x   y   z");
    //x
    u8g2.drawBox(12, (imu.accX() > 0 ? 17 - (abs(imu.accX()) / 64) : 16), 6, abs(imu.accX()) / 64);
    //y
    u8g2.drawBox(32, (imu.accY() > 0 ? 17 - (abs(imu.accY()) / 64) : 16), 6, abs(imu.accY()) / 64);
    //z
    u8g2.drawBox(54, (imu.accZ() > 0 ? 17 - (abs(imu.accZ()) / 64) : 16), 6, abs(imu.accZ()) / 64);
    u8g2.sendBuffer();
  }
}
/***
 *       _____            
 *      / ____|           
 *     | |  __ _   _ _ __ 
 *     | | |_ | | | | '__|
 *     | |__| | |_| | |   
 *      \_____|\__, |_|   
 *              __/ |     
 *             |___/      
 */
void Gyr(void)
{
  if (millis() - Millis > 100)
  {
    if (imu.dataReady())
    {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
      u8g2.drawStr(0, 8, "Gyr (DPS)");
      imu.getAGMT();
      u8g2.setCursor(0, 16);
      u8g2.print("x=");
      DEBUG("Gyr (DPS)  X=");
      u8g2.print(imu.gyrX());
      DEBUG(imu.gyrX());
      u8g2.setCursor(0, 24);
      u8g2.print("y=");
      DEBUG("  Y=");
      u8g2.print(imu.gyrY());
      DEBUG(imu.gyrY());
      u8g2.setCursor(0, 32);
      u8g2.print("z=");
      DEBUG("  Z=");
      u8g2.print(imu.gyrZ());
      DEBUGLN(imu.gyrZ());
      u8g2.sendBuffer();
      Millis = millis();
    }
  }
}
/***
 *      __  __             
 *     |  \/  |            
 *     | \  / | __ _  __ _ 
 *     | |\/| |/ _` |/ _` |
 *     | |  | | (_| | (_| |
 *     |_|  |_|\__,_|\__, |
 *                    __/ |
 *                   |___/ 
 */
void Mag(void)
{
  if (millis() - Millis > 100)
  {
    if (imu.dataReady())
    {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
      u8g2.drawStr(0, 8, "Mag (uT)");
      imu.getAGMT();
      u8g2.setCursor(0, 16);
      u8g2.print("x=");
      DEBUG("Mag (uT)  X=");
      u8g2.print(imu.magX());
      DEBUG(imu.magX());
      u8g2.setCursor(0, 24);
      u8g2.print("y=");
      DEBUG("  Y=");
      u8g2.print(imu.magY());
      DEBUG(imu.magY());
      u8g2.setCursor(0, 32);
      u8g2.print("z=");
      DEBUG("  Z=");
      u8g2.print(imu.magZ());
      DEBUGLN(imu.magZ());
      u8g2.sendBuffer();
      Millis = millis();
    }
  }
}

/***
 *     __      __   _ _                   
 *     \ \    / /  | | |                  
 *      \ \  / /__ | | |_ __ _  __ _  ___ 
 *       \ \/ / _ \| | __/ _` |/ _` |/ _ \
 *        \  / (_) | | || (_| | (_| |  __/
 *         \/ \___/|_|\__\__,_|\__, |\___|
 *                              __/ |     
 *                             |___/      
 */
void BatteryVoltage(void)
{
  if (millis() - Millis > 100)
  {
    int Vot = analogRead(BatteryVol) * 2;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
    u8g2.drawStr(17, 8, "VBATT");
    u8g2.setCursor(20, 24);
    u8g2.print(Vot * 3.3 / 1024);
    u8g2.sendBuffer();
    Millis = millis();
  }
}

/***
 *       _____ _                 
 *      / ____| |                
 *     | (___ | | ___  ___ _ __  
 *      \___ \| |/ _ \/ _ \ '_ \ 
 *      ____) | |  __/  __/ |_) |
 *     |_____/|_|\___|\___| .__/ 
 *                        | |    
 *                        |_|    
 */
void Sleep(void)
{
  GPS_Sleep();
  ICM20948_Sleep();
  LoRa_Sleep();
  SSD1306_Sleep();
  DEBUGLN("MCU Sleep");
  pinMode(PwrSwitch1_8V, OUTPUT);
  digitalWrite(PwrSwitch1_8V, LOW);
  Serial.end();
  SPI.end();
  Wire.end();

  pinMode(MySDA, OUTPUT);
  pinMode(MySCL, OUTPUT);
  digitalWrite(MySDA, HIGH);
  digitalWrite(MySCL, HIGH);

  pinMode(BatteryVol, INPUT_ANALOG);

  pinMode(LORA_MISO, INPUT_ANALOG);
  pinMode(LORA_MOSI, INPUT_ANALOG);
  pinMode(LORA_SCK, INPUT_ANALOG);
  pinMode(LORA_NSS, INPUT_ANALOG);
  pinMode(LORA_DIO0, INPUT_ANALOG);
  pinMode(LORA_RST, INPUT_ANALOG);

  pinMode(GPS_RST, INPUT_ANALOG);
  pinMode(GPS_RX, INPUT_ANALOG);
  pinMode(GPS_TX, INPUT_ANALOG);
  pinMode(PB8, INPUT_ANALOG);
  pinMode(PA11, INPUT_ANALOG);
  pinMode(PA12, INPUT_ANALOG);
  pinMode(GPS_LEVEL_SHIFTER_EN, INPUT_ANALOG);

  LowPower.deepSleep();
  BoardInit();
}

void BoardInit(void)
{
  pinMode(PwrSwitch1_8V, OUTPUT);
  digitalWrite(PwrSwitch1_8V, HIGH);
  Serial.begin(115200);

  SSD1306_Init();
  ICM20948_Init();
  LoRa_Init();
  GPS_Init();

  pinMode(BatteryVol, INPUT_ANALOG);
  pinMode(TTP223_VDD_PIN, OUTPUT);
  pinMode(TouchPad, INPUT);
  digitalWrite(TTP223_VDD_PIN, HIGH);
}

void repetitionsIncrease(void)
{
}

void setup()
{
  LowPower.begin();
  LowPower.attachInterruptWakeup(TouchPad, repetitionsIncrease, RISING, DEEP_SLEEP_MODE);
  BoardInit();
}

void loop()
{
  int Touch = TouchCallback();
  if (Touch != 0)
    DEBUGLN(Touch);
  if (Touch > 3000) //Long press for 3 seconds to enter sleep mode
  {
    Sleep();
  }
  else if ((Touch > 100 ? true : false))
  {
    funcSelectIndex++;
    funcSelectIndex %= index_max;
    Serial.print("funcSelectIndex:");
    Serial.println(funcSelectIndex);
  }

  if (LilyGoCallBack[funcSelectIndex])
  {
    LilyGoCallBack[funcSelectIndex]();
  }
  u8g2.setContrast(0xFF);
}
