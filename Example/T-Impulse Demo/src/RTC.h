#ifndef __RTC_H__
#define __RTC_H__

#include <STM32RTC.h>

STM32RTC &getRTC(void);

void rtc_init(void);
void RTCLoop(uint8_t &BTN_state);

#endif /* __RTC_H__ */