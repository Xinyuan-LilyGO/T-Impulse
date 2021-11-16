
#ifndef __GPS_H__
#define __GPS_H__

#include <TinyGPS++.h>

void GPS_Init(void);
void GPS_loop(void);
extern TinyGPSPlus *gps;
bool isGPSenable(void);
void GPSMenuLoop(uint8_t &BTN_state);
void GPS_Sleep(void);
#endif /* __GPS_H__ */