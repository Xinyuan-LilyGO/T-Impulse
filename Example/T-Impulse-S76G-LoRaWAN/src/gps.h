#ifndef __GPS_H__
#define __GPS_H__

#include <TinyGPS++.h>

void gps_init(void);
void gps_loop(void);
extern TinyGPSPlus *gps;

#endif /* __GPS_H__ */