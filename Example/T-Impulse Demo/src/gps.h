#ifndef __GPS_H__
#define __GPS_H__

#include <TinyGPS++.h>

void gps_init(void);
void gps_loop(void);
TinyGPSPlus *getGPS(void);
bool isGPSenable(void);
void GPSMenuLoop(uint8_t &BTN_state);
void GPS_Sleep(void);
#endif /* __GPS_H__ */