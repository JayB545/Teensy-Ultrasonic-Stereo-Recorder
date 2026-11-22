/*
 * rtc.h - NEW file, not from Walter's project.
 *
 * Teensy.cxx does `#include "rtc.h"` and calls rtc_get()/rtc_set()/
 * time2date()/date2time()/encodeTimestamp() (used by setAlarm()/
 * doHibernate()/rtc_get_datetime()/rtc_set_datetime() - none of which
 * this minimal test actually calls, but Teensy.cxx references them at
 * file scope, so they must resolve at link time).
 *
 * Walter's own rtc.cxx (for an external RTC chip on the RP2040/2350
 * boards) wasn't part of what was shared with us, and isn't relevant
 * here anyway - the Teensy 4.1 has its own on-chip RTC. rtcImpl.cpp
 * implements these against that on-chip RTC via TimeLib, purely so the
 * unmodified parts of Teensy.cxx/adc.cxx link and build; nothing here
 * affects the SAI/DMA/ADC signal path under test.
 */
#ifndef RTC_H
#define RTC_H

#include "Teensy.h"  // datetime_t

void time2date(uint32_t t, datetime_t *dt, int epochYear);
uint32_t date2time(const datetime_t *dt, int epochYear);
void encodeTimestamp(char *out, datetime_t *dt);
extern char startTime[32];

#endif
