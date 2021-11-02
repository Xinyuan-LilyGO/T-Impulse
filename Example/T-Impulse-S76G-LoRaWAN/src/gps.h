/*
 * @Author: your name
 * @Date: 2021-11-02 10:50:14
 * @LastEditTime: 2021-11-02 11:19:44
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \wristband-S76G\src\gps.h
 */

#ifndef __GPS_H__
#define __GPS_H__

#include <TinyGPS++.h>

void GPS_Init(void);
void GPS_loop(void);
extern TinyGPSPlus *gps;

#endif /* __GPS_H__ */