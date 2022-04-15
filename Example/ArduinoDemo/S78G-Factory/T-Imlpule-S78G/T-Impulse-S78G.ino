#include "Arduino.h"
#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus
#include <U8g2lib.h>   //https://github.com/olikraus/u8g2
#include "RadioLib.h"
#include <ICM_20948.h>
#include "STM32LowPower.h"
#include <STM32RTC.h>
#include "OneButton.h"
#include "pin_config.h"

void GPS(void);
void Accel(void);
void Accel2(void);
void Gyr(void);
void Mag(void);
void MagCalibration(void);
void Sender(void);
void Reciver(void);
void BatteryVoltage(void);
void Rtc(void);

SX1278 radio = nullptr;
U8G2_SSD1306_64X32_1F_F_HW_I2C u8g2(U8G2_R0, PIN_OLED_RESET, PIN_SCL, PIN_SDA);
TinyGPSPlus gps;
HardwareSerial gpsPort(PIN_GPS_RX, PIN_GPS_TX);
ICM_20948_I2C imu;
/*
Note: the value of WHO_AM_I register address 0x00 in ICM20648 is 0XE0
In the process of using ICM20948, everything is normal except without MAG.
*/
STM32RTC &rtc = STM32RTC::getInstance();
OneButton touch(PIN_TOUCH_BTN, false);

uint8_t index_max;
typedef void (*funcCallBackTypedef)(void);
funcCallBackTypedef LilyGoCallBack[] = {Rtc, Sender, Reciver, GPS, BatteryVoltage, Accel, Accel2, Gyr};
uint8_t funcSelectIndex = 0;

uint32_t Millis = 0;
uint32_t Last = 0;
uint32_t LoRa_Count = 0;
uint8_t GPS_count = 0;
double Lat = 0, Long = 0;
int Value = 0;
static bool isShock;
static uint32_t ShockMillis;

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
  u8g2.drawFrame(2, 25, 60, 6);
  for (int i = 0; i < 0xFF; i++)
  {
    u8g2.setContrast(i);
    u8g2.drawBox(3, 26, (uint8_t)(0.231 * i), 5);
    u8g2.sendBuffer();
  }
  for (int i = 0; i < 0xFF; i++)
  {
    u8g2.setContrast(0xFF - i);
    delay(3);
  }
  Serial.println("SSD1306 done");
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
bool isrReceived = false;

void setRadioDirection(bool rx)
{
  isrReceived = rx;
  digitalWrite(PIN_RADIO_ANT_SWITCH_RXTX, rx ? HIGH : LOW);
}

#define REG_LR_TCXO 0x4B
#define RFLR_TCXO_TCXOINPUT_MASK 0xEF
#define RFLR_TCXO_TCXOINPUT_ON 0x10
#define RFLR_TCXO_TCXOINPUT_OFF 0x00 // Default

bool LoRa_Init(void)
{
  radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO0, PIN_LORA_RST, PC13, SPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));
  int state = radio.begin(433.0);
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
  //! Initialize Radio ant switch pin
  pinMode(PIN_RADIO_ANT_SWITCH_RXTX, OUTPUT);
  //! Lora ANT Switch 1:Rx, 0:Tx
  setRadioDirection(true);
  pinMode(PC1, INPUT);
  delay(10);
  int pin_state = digitalRead(PC1);
  if (pin_state == false)
  {
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_Struct;
    // for LoRa sx1276 TCXO OE Pin
    GPIO_Struct.Pin = GPIO_PIN_7;
    GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Struct.Pull = GPIO_NOPULL;
    GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_Struct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    delay(10);
  }
  Serial.println("Lora done");
  return true;
}
void LoRa_Sleep(void)
{
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
    String lorasend = "LilyGo " + String(LoRa_Count);
    radio.transmit(lorasend);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
    u8g2.drawStr(0, 8, "LoRa Send");
    u8g2.setCursor(0, 20);
    Serial.print("LilyGo ");
    Serial.println(LoRa_Count);
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

  int packetSize = radio.receive(recv);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
  u8g2.drawStr(0, 8, "LoRa Rec");
  u8g2.setCursor(0, 16);
  u8g2.print("RSSI:");
  if (packetSize == RADIOLIB_ERR_NONE)
  {
    // received a packet
    DEBUG("Received packet '");
    Serial.println(recv);
    // recv = "";
    //  read packet
    /*     while (LoRa.available())
    {
      recv += (char)LoRa.read();
    } */
    // RSSI = LoRa.packetRssi();
    RSSI = radio.getRSSI();
    DEBUGLN(recv);
    // print RSSI of packet
    DEBUG("' with RSSI ");
    DEBUGLN(RSSI);
  }
  u8g2.print(RSSI);

  if (recv.length() < 15)
  {
    u8g2.setCursor(0, 24);
    u8g2.println(recv);
  }
  else
  {

    u8g2.setCursor(0, 24);
    u8g2.println(recv);
    u8g2.setCursor(0, 32);
    u8g2.println(recv.substring(15));
  }
  u8g2.sendBuffer();
}

uint8_t CalcWeek(uint16_t _year, uint8_t _mon, uint8_t _day)
{
  uint8_t y, c, m, d;
  int16_t w;
  if (_mon >= 3)
  {
    m = _mon;
    y = _year % 100;
    c = _year / 100;
    d = _day;
  }
  else
  {
    m = _mon + 12;
    y = (_year - 1) % 100;
    c = (_year - 1) / 100;
    d = _day;
  }

  w = y + y / 4 + c / 4 - 2 * c + ((uint16_t)26 * (m + 1)) / 10 + d - 1;
  if (w == 0)
  {
    w = 7;
  }
  else if (w < 0)
  {
    w = 7 - (-w) % 7;
  }
  else
  {
    w = w % 7;
  }
  return w;
}

void Rtc(void)
{
  if ((millis() - Millis) > 100)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
    u8g2.setCursor(0, 8);
    u8g2.printf("%02d/%02d/%02d ", rtc.getDay(), rtc.getMonth(), rtc.getYear());
    u8g2.setCursor(0, 24);
    u8g2.printf("%02d:%02d:%02d", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    u8g2.sendBuffer();

    Millis = millis();
  }
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
bool GPS_WaitAck(String cmd, String arg = "")
{
  gpsPort.flush();
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
  // String ack = "";
  uint32_t smap = millis() + 500;
  while (millis() < smap)
  {
    if (gpsPort.available() > 0)
    {
      // ack = gpsPort.readStringUntil('\n');
      // String acc = "[" + cmd.substring(1) + "] " + "Done";
      // if (ack.startsWith(acc))
      if (gpsPort.find("Done"))
      {
        return true;
      }
    }
  }
  Serial.println("GPS Send cmd Fail");
  return false;
}
void GPS_Init(void)
{
  gpsPort.begin(GPS_BAUD_RATE);
  pinMode(PIN_GPS_LEVEL_SHIFTER_EN, OUTPUT);
  digitalWrite(PIN_GPS_LEVEL_SHIFTER_EN, HIGH);
  pinMode(PIN_GPS_RST, GPIO_PULLUP);
  // Set  Reset Pin as 0
  digitalWrite(PIN_GPS_RST, LOW);
  // Scope shows 1.12s (Low Period)
  delay(200);
  // Set  Reset Pin as 1
  digitalWrite(PIN_GPS_RST, HIGH);
  delay(500);

  GPS_WaitAck("@GSTP");
  GPS_WaitAck("@BSSL", "0x2EF");
  GPS_WaitAck("@GSOP", "1 1000 0");
  GPS_WaitAck("@GNS", "0x03");
  //! Start GPS connamd
  GPS_WaitAck("@GSR");
  Serial.println("GPS done");
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
  if (millis() - Millis > 100)
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
    u8g2.print(F("Lat="));
    u8g2.setCursor(26, 24);
    u8g2.print(Lat);

    u8g2.setCursor(0, 32);
    u8g2.print(F("Long="));
    u8g2.setCursor(34, 32);
    u8g2.print(Long);
    u8g2.sendBuffer();
    Millis = millis();
  }
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
    rtc.setDate(CalcWeek(gps.date.year(), gps.date.month(), gps.date.day()), gps.date.day(), gps.date.month(), gps.date.year() - 2000);
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
    rtc.setTime(gps.time.hour() < 16 ? gps.time.hour() + 8 : 24 - gps.time.hour() + 8, gps.time.minute(), gps.time.second());
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

bool ICM20648_Init(void)
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

void ICM20648_Sleep(void)
{
  imu.sleep(true);
  DEBUGLN("ICM20648 SLEEP!!");
}

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
    // x
    u8g2.drawBox(12, (imu.accX() > 0 ? 17 - (abs(imu.accX()) / 64) : 16), 6, abs(imu.accX()) / 64);
    // y
    u8g2.drawBox(32, (imu.accY() > 0 ? 17 - (abs(imu.accY()) / 64) : 16), 6, abs(imu.accY()) / 64);
    // z
    u8g2.drawBox(54, (imu.accZ() < 0 ? 17 - (abs(imu.accZ()) / 64) : 16), 6, abs(imu.accZ()) / 64);
    u8g2.sendBuffer();
  }
}

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

ADC_HandleTypeDef hadc;
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
uint32_t Vot = 0, Millis_t = 0;
uint8_t i = 0;
void BatteryVoltage(void)
{
  HAL_ADC_Start(&hadc);
  if (millis() - Millis_t > 5 && i < 20)
  {
    i++;
    Vot += HAL_ADC_GetValue(&hadc);
    Millis_t = millis();
  }
  if (millis() - Millis > 500)
  {
    Vot /= i;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_IPAandRUSLCD_tr);
    u8g2.drawStr(17, 8, "VBATT");
    u8g2.setCursor(20, 24);
    u8g2.print((Vot * 3.3 * 2) / 4096);
    u8g2.print("V");
    u8g2.sendBuffer();
    i = 0;
    Vot = 0;
    Millis = millis();
  }
}

void Sleep(void)
{
  GPS_Sleep();
  ICM20648_Sleep();
  LoRa_Sleep();
  SSD1306_Sleep();
  DEBUGLN("MCU Sleep");
  pinMode(PIN_PWR_ON_1_8V, OUTPUT);
  digitalWrite(PIN_PWR_ON_1_8V, LOW);

  LowPower.deepSleep();
  BoardInit();
}

void BoardInit(void)
{
  Serial.begin(115200);
  index_max = sizeof(LilyGoCallBack) / sizeof(funcCallBackTypedef);
  delay(1000);
  pinMode(PIN_PWR_ON_1_8V, OUTPUT);
  pinMode(PIN_PWR_ON_GPS, OUTPUT);
  digitalWrite(PIN_PWR_ON_1_8V, HIGH);
  digitalWrite(PIN_PWR_ON_GPS, HIGH);

  pinMode(PIN_SHOCK, OUTPUT);
  digitalWrite(PIN_SHOCK, LOW);

  SPI.setMISO(PIN_LORA_MISO);
  SPI.setMOSI(PIN_LORA_MOSI);
  SPI.setSCLK(PIN_LORA_SCK);
  SPI.begin();

  Wire.setSCL(PIN_SCL);
  Wire.setSDA(PIN_SDA);
  Wire.begin();
  rtc.begin();

  GPS_Init();
  MX_ADC_Init();
  SSD1306_Init();
  LoRa_Init();
  ICM20648_Init();

  scan_iic();

  touch.attachClick([]
                    {
    isShock = true;
    ShockMillis = millis();
    digitalWrite(PIN_SHOCK, HIGH);
    funcSelectIndex++;
    funcSelectIndex %= index_max;
    Serial.print("funcSelectIndex:");
    Serial.println(funcSelectIndex); });

  touch.attachLongPressStart([]
                             { Sleep(); });

  pinMode(PIN_BAT_VOLT, INPUT_ANALOG);
  pinMode(PIN_TOUCH_BTN, INPUT);
  pinMode(PIN_TTP223_VDD, OUTPUT);
  digitalWrite(PIN_TTP223_VDD, HIGH);
}

void repetitionsIncrease(void)
{
}

void setup()
{
  LowPower.begin();
  LowPower.attachInterruptWakeup(PIN_TOUCH_BTN, repetitionsIncrease, RISING, DEEP_SLEEP_MODE);
  BoardInit();
}

void loop()
{
  if (isShock)
  {
    if (millis() - ShockMillis > 200)
    {
      isShock = false;
      digitalWrite(PIN_SHOCK, LOW);
    }
  }

  if (LilyGoCallBack[funcSelectIndex])
  {
    LilyGoCallBack[funcSelectIndex]();
  }
  u8g2.setContrast(0xFF);
  touch.tick();
}

void scan_iic(void)
{
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning for I2C devices ...");
  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    }
    else if (error != 2)
    {
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found");
  }
}