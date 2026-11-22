/*
 * rtcImpl.cpp - NEW file, not from Walter's project. See rtc.h.
 *
 * Minimal implementations of rtc_get()/rtc_set() (declared in Teensy.h)
 * and time2date()/date2time()/encodeTimestamp() (declared in rtc.h),
 * using the Teensy 4.1's on-chip RTC via TimeLib. Only exists to satisfy
 * the linker for code paths this test never exercises (setAlarm(),
 * doHibernate(), rtc_get_datetime(), rtc_set_datetime()) - none of it
 * touches the SAI/DMA/ADC path under test.
 */
#include <Arduino.h>
#include <TimeLib.h>
#include "Teensy.h"
#include "rtc.h"

char startTime[32] = {0};

// rtc_get()/rtc_set() are deliberately NOT defined here (2026-10-31).
// Teensy's own core already provides functions with exactly these names -
// Teensy3Clock.get() and .set() are thin wrappers that call the core's
// rtc_get()/rtc_set(). The versions that used to live here called
// Teensy3Clock.get()/.set() themselves, and because they were linked in
// ahead of the core's, Teensy3Clock.get() called back into this file:
// rtc_get() -> now() -> Teensy3Clock.get() -> rtc_get() -> ... an endless
// recursion that overflowed the stack (CrashReport: data access violation,
// "Stack problem", right after "about to sync RTC"). Walter's Teensy.cpp
// still calls rtc_get()/rtc_set(); those now resolve to the core's own
// implementations, which read/write the on-chip RTC directly. The TimeLib
// hookup (setSyncProvider) is done once in TUFR_4.1.ino's setup().

void time2date(uint32_t t, datetime_t *dt, int /*epochYear*/) {
  tmElements_t tm;
  breakTime((time_t)t, tm);
  dt->year  = tmYearToCalendar(tm.Year);
  dt->month = tm.Month;
  dt->day   = tm.Day;
  dt->dotw  = tm.Wday - 1; // TimeLib: 1=Sunday -> datetime_t: 0=Sunday
  dt->hour  = tm.Hour;
  dt->min   = tm.Minute;
  dt->sec   = tm.Second;
}

uint32_t date2time(const datetime_t *dt, int /*epochYear*/) {
  tmElements_t tm;
  tm.Year   = CalendarYrToTm(dt->year);
  tm.Month  = dt->month;
  tm.Day    = dt->day;
  tm.Hour   = dt->hour;
  tm.Minute = dt->min;
  tm.Second = dt->sec;
  return (uint32_t)makeTime(tm);
}

void encodeTimestamp(char *out, datetime_t *dt) {
  sprintf(out, "%04d%02d%02d_%02d%02d%02d",
          dt->year, dt->month, dt->day, dt->hour, dt->min, dt->sec);
}
