/*
 * TUFR_4.2.ino  -  Teensy Ultrasonic Field Recorder, version 4.2
 *
 * 4.2 = 4.1 (2026-11-09a) + optional lossless FLAC output (setup line 4,
 * tufr_flac.c/.h). WAV recording is unchanged and remains the default.
 *
 * Records 16-bit WAV (or FLAC) files to a microSD card from the WMXZ-EU ADC_V2 module
 * (TLV320ADC6140) on a Teensy 4.1: 1, 2 or 4 channels at 48/96/192 kHz, 1-2 channels
 * at 384 kHz, 1 channel at 768 kHz. Every deployment setting comes from
 * TUFR_Setup.txt on the SD card (line-by-line format: see readConfigFile() and
 * the TUFR_Setup.txt shipped with this sketch).
 *
 * Features
 *   - Continuous back-to-back files, or a record/wait duty cycle. Waits of
 *     HIBERNATE_MIN_WAIT_S seconds or more power the chip down with an RTC
 *     alarm wake (hibernate, under 1 mA) and restart the board; shorter waits
 *     idle at 24 MHz.
 *   - Optional daily recording window by RTC time (may cross midnight) and an
 *     optional scheduled start date/time. Outside the window the board
 *     hibernates in chunks of at most one hour.
 *   - ADC protection: the ADC is verified after every power-up, re-started if
 *     it drops out, and power-cycled completely (hibernate) if that does not
 *     help, rather than write silent files or stop. A hardware watchdog
 *     restarts the chip if the program ever stops responding. See the "ADC /
 *     data-stream protection" and "Unattended recovery" blocks below.
 *   - TUFR_log.txt telemetry log next to the WAV files (config, per-file
 *     write statistics, ADC events, halts).
 *
 * Output: TUFR_YYYYMMDD_HHMMSS.WAV (or .FLAC), one file per record interval.
 *
 * LED: fast flicker = recording; solid on = stopped cleanly (button), safe to
 * remove the SD card. Blink codes: 4 (repeating) = stopped by the shutdown
 * button while waiting (a real stop). Faults blink their code three times and
 * then power-cycle (60 s, later 10 min) and try again: 2 = SD card problem
 * (missing, unreadable, full, writes failing, or a file cannot be created:
 * insert or try another card; the log says which), 5 = ADC not responding.
 * (No audio from the ADC is a power cycle without blinks, logged NO_DATA.)
 *
 * Credits
 *   The SAI/DMA capture driver and queue are Walter Zimmer's, from his
 *   microPAM Teensy 4.1 project (Teensy.cpp, Teensy.h, adc.cpp, adc.h,
 *   process.h, and the Queue class in queueProcess.cpp), with these changes:
 *   the BCLK:FSYNC ratio is computed from the channel count, and channel
 *   count and gain are runtime parameters rather than compile-time macros.
 *   The ADC itself is configured by adcRuntime.cpp (runtime channel count,
 *   power-cycle retries, health checks). rtc.h/rtcImpl.cpp supply the few
 *   time functions his driver links against.
 *
 * Copyright (c) 2026, Walter Zimmer and Jay Barlow
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <Arduino.h>
#include <SdFat.h>
#include <SPI.h>
#include <TimeLib.h>
#include "global.h"
#include "Teensy.h"
#include "adc.h"
#include "process.h"
#include "rtc.h"
#include "adcRuntime.h"
#include "tufr_flac.h"
#include <Wire.h>
// For PIN_SHUTDOWN - continuous recording (below) now stops on the same
// shutdown button TUFR_4.0.ino uses, instead of a fixed file count.
#include "TUFR_Config.h"
// Hibernate uses Walter Zimmer's method (his Teensy.cpp doHibernate(): set the
// SNVS RTC alarm, then the "turn off power" bit) - NOT the Snooze library.
// Bench finding 2026-11-03: Snooze.hibernate() woke after ~1 s once Wire.begin()
// had run in the same boot, and left an initialized SD card drawing ~35 mA.
// This method slept the full time with I2C and the SD card active, at 0.75 mA.

// ---- RTC ----
// 1 = the board has a coin cell on VBAT and the Teensy's on-chip RTC is used
// (time survives power loss and hibernate wakes; RTC sync and scheduled start
// are compared against real hardware time). 0 = no coin cell: only TimeLib's
// software clock is used and Teensy3Clock is never touched. (An earlier hard
// fault blamed on a missing battery was really the rtc_get()/rtc_set() name
// clash described in rtcImpl.cpp - never define functions with those names.)
#define RTC_HARDWARE_PRESENT 1

#define RTC_DEFAULT_YEAR   2026
#define RTC_DEFAULT_MONTH  1
#define RTC_DEFAULT_DAY    1
#define RTC_DEFAULT_HOUR   0
#define RTC_DEFAULT_MINUTE 0
#define RTC_DEFAULT_SECOND 0

// ---- Duty-cycle waits ----
// Waits shorter than HIBERNATE_MIN_WAIT_S resume in place: acqStop(), idle at
// WAIT_CLOCK_FREQ, then acqStart() again, with no reboot. set_arm_clock() is
// the Teensy core's runtime clock switch (not exposed via Teensy.h).
extern "C" uint32_t set_arm_clock(uint32_t frequency);
#define WAIT_CLOCK_FREQ 24000000UL   // lowest stable clock, same constant TUFR_4.0.ino uses

// NOTE: an in-place resume must call acqStart() ONLY - not acqInit() (it would
// leak a DMA channel per cycle via dma.begin()) and not i2s_setup() (it
// re-runs set_audioClock() on an already-locked PLL, whose unbounded lock
// wait can hang). gFsHz/gNchan never change, and acqStop() only clears the
// receiver enable bits, so acqStart() alone (i2s_start() + adcRuntimeInit())
// is sufficient.

#if RTC_HARDWARE_PRESENT
// TimeLib sync provider: reads the Teensy 4.1's on-chip RTC. (Do not name
// anything rtc_get/rtc_set - those names belong to the Teensy core; see
// rtcImpl.cpp for the recursion bug that caused.)
static time_t rtcSyncProvider() { return (time_t)Teensy3Clock.get(); }
#endif

// ---- Hibernate waits ----
// A duty-cycle wait of HIBERNATE_MIN_WAIT_S seconds or more uses
// a hibernate (snvsHibernate(), under 1 mA) instead of the clock-throttled
// busy-wait above (tens of mA). hibernate() ends in a full RESTART, not a resume - setup()
// runs again from the top, same as a cold boot, so the capture chain
// (acqInit()/acqStart()) is rebuilt from scratch and none of the in-place
// PLL/DMA resume concerns described above apply. Shorter waits keep the old
// in-place path because a restart costs a few seconds of full-power boot.
//
// State that must survive the restart is passed through a small marker file
// on the SD card (not chip registers): "<unix time at sleep> <wait seconds> <file index>".
// setup() reads and deletes it; its presence means "this is a wake, not a
// cold power-on" (skip the start-up-only steps: Serial wait, scheduled-start
// wait) and gives the software clock a value to continue from when there is
// no RTC battery (RTC_HARDWARE_PRESENT==0), so filenames/log timestamps keep
// advancing instead of restarting from the default time.
// KNOWN LIMITS: the shutdown button is only checked while awake (hold it
// through the wake to stop); a power loss during a hibernate leaves a stale
// marker, so the next power-on is treated as a wake (harmless: it just
// resumes the duty cycle).
#define HIBERNATE_MIN_WAIT_S 30
#define HIBERNATE_MARKER_FILE "TUFR_HIB.TXT"
// Scheduled-start (delayed start) wait: hibernate in chunks of at most
// SCHED_HEARTBEAT_S seconds and give the LED one bright flash of SCHED_FLASH_MS
// each time the board wakes, so it is visibly alive. When the start time is
// SCHED_INPLACE_S seconds away or less, the last stretch is waited out in place
// (a hibernate costs a few seconds of boot, so it isn't worth it that close).
#define SCHED_HEARTBEAT_S 57   // + about 3 s of boot per wake = a flash roughly every 60 s
#define SCHED_FLASH_MS    500
#define SCHED_INPLACE_S   40

// ---- Deployment settings (from TUFR_Setup.txt - see readConfigFile()) ----
// Defaults apply if the file is missing or a line is unreadable.
static uint8_t  gNchan     = 2;
static uint32_t gFsHz      = 192000;  // ADC sample rate; clamped to a valid rate by readConfigFile()
static uint32_t gOutFsHz   = 192000;  // rate of the WAV file (= gFsHz, or half of it with the -768 decimation)
static bool     gDecimReq  = false;   // setup rate was negative (-768): sample at 768 kHz, write 384 kHz
static int8_t   gGainVal   = 20;      // 0-42, ADC register units
static bool     gFlac      = false;   // setup line 4: WAV (default) or FLAC (lossless)
// ADC decimation filter: -1 auto (linear phase at 384 kHz, ultra-low latency at every other
// rate), 0 linear, 1 low latency, 2 ultra-low. No longer in the setup file (testing done);
// change it here if it ever needs testing again.
static int8_t   gFiltSel   = -1;
static int32_t  gHpfHz     = -1;      // high-pass: -1/-2/-3 = ADC presets 0.00025/0.002/0.008 x fS, 0 = off, >0 = cutoff Hz
static uint32_t gDurationS = 60;      // record time per file, seconds
static uint32_t gWaitS     = 0;       // duty-cycle wait between recordings, seconds (0 = continuous)
static uint32_t gClockMHz  = 60;      // CPU clock while recording; 0 = leave as the IDE's board menu set it
static time_t   gScheduledStartUnix = 0; // 0 = disabled, start immediately
// Daily recording window, RTC time of day as HHMM: start is inclusive, stop
// is exclusive, start > stop means the window crosses midnight (e.g. 2000 0600 = 8pm to 6am), "0000 2400" = off.
static int      gDailyStartHHMM = 0;
static int      gDailyStopHHMM  = 2400;
static bool     gUseDailySchedule = false;
#define DAILY_SLEEP_CHUNK_S 3600UL  // longest single hibernate while waiting for the window; re-checks the clock hourly

// ---- ADC / data-stream protection ----
// Bench history: on the motherboard at 384 kHz / 2 ch the ADC would accept its
// configuration, then stop answering on I2C within about 100 ms of start-up and
// output only zeros (cause not found; a start-up supply transient is suspected).
// Recording silent files without telling anyone is the worst outcome, so three
// layers now stand between that and a deployment:
//   1. haltIfAdcUnhealthy(): after every ADC power-up the ADC must acknowledge
//      I2C and read back the channel-enable registers it was given
//      (adcRuntimeInit() itself retries up to 3 times with full ADC
//      power-cycles). Still bad -> LED 5 blinks (three times), then a full
//      power cycle (build 13d, see "Unattended recovery").
//   2. settleAdc(): waits 150 ms after init; if the ADC no longer answers it is
//      power-cycled and re-initialized (up to 3 times) before recording starts,
//      and again after every wake from a wait.
//   3. Dead-stream detector (loop()): DEAD_STREAM_RECOVER_MS of all-zero audio
//      -> log the ADC registers and I2S flags, power-cycle the ADC and restart
//      capture with the file left open. After DEAD_STREAM_INPLACE_TRIES
//      attempts in one file: full power cycle (build 13d).
// Blink codes: see the header comment at the top of this file.
#define TUFR_VERSION "4.2 (2026-11-14b)"

// 1 = reserve each file's whole length when it is created (preAllocate), 0 = let the file
// grow as it is written. The 11-04d log showed the preallocation step growing from ~200 ms to
// ~300 ms as the card filled, which is longer than the queue can hold: that caused the
// only mid-file misses (at 57.27 s, exactly when the next file is preallocated). 0 removes
// that stall altogether (and the truncate at close); the ordinary 20-60 ms write stalls that
// remain are well inside the queue. Set back to 1 if the mid-file write times get worse.
#define TUFR_PREALLOCATE 0

// Power-saving experiment: switch the USB device off once recording has started,
// but only when no serial monitor is attached (!Serial), so debugging over USB still
// works. 0 = leave USB alone; 1 = detach from the bus and power down the USB PHY;
// 2 = also stop the 480 MHz USB PLL (largest saving, but if anything else turned out
// to use that PLL, set this to 1). Everything useful is in the SD log; USB serial
// output after this point goes nowhere. Each hibernate wake is a full reboot, so USB
// is back for the first seconds of every boot.
#define TUFR_USB_OFF 2

#define DEAD_STREAM_RECOVER_MS 500UL   // this long of all-zero blocks = dead stream
#define DEAD_STREAM_INPLACE_TRIES 3    // in-place ADC restarts per file, then a full power cycle
static uint8_t gRecoveries = 0;        // in-place restarts in the current file

// ---- Unattended recovery (build 2026-11-13d) ----
// Nothing that goes wrong in the field should stop the recorder for good; only
// the shutdown button does. Three layers:
//  1. Hardware watchdog (RTWDOG): if the program stops responding for
//     WDT_TIMEOUT_S (e.g. an I2C or SD call that never returns), the chip
//     restarts; that boot logs WATCHDOG_RESET and does a full power cycle (3).
//  2. Dead stream: up to DEAD_STREAM_INPLACE_TRIES in-place ADC restarts per
//     file, with growing pauses (0.3, 2, 10 s), then a full power cycle (3).
//  3. Full power cycle: close the file (audio so far is kept), log POWER_CYCLE,
//     hibernate POWER_CYCLE_SHORT_S (the first POWER_CYCLE_SHORT_TRIES times
//     in a row), then POWER_CYCLE_LONG_S, and start again as after a duty-cycle
//     wake. Hibernation switches off the Teensy's 3.3 V, so the ADC (including
//     its IOVD, which comes from that 3.3 V) is completely unpowered: an ADC
//     power-cycle in place cannot do that. Faults that used to halt with an LED
//     code (2 SD card, 3 file open, 5 ADC not answering, 6 no audio) now show
//     the code three times and power-cycle. The count resets once a file closes
//     with live audio.
#define TUFR_WATCHDOG 1                // 0 = no watchdog
#define WDT_TIMEOUT_S 10
#define TUFR_WDT_TEST 0                // bench test: 1 = stop feeding the watchdog 30 s into the
                                       // first file after a cold start (expect WDT_TEST; about
                                       // 10 s later the chip restarts and powers off for 60 s;
                                       // the next boot logs WATCHDOG_RESET and records on).
                                       // Set back to 0 for the field.
#define POWER_CYCLE_SHORT_S 60
#define POWER_CYCLE_LONG_S 600
#define POWER_CYCLE_SHORT_TRIES 3
static uint32_t gPowerCycles = 0;      // power cycles in a row (carried across the hibernate in the marker file)
static bool gColdBoot = true;          // false after a hibernate wake
static bool gAfterWatchdog = false;    // this boot follows a watchdog restart (+ power cycle)
// "The watchdog fired" has to survive the power-off that follows it. The SNVS
// general-purpose registers did not keep a value through that on the bench
// (builds 13d/14a, 2026-11-14), but the RTC alarm register (SNVS_LPTAR) must,
// since it is what wakes the board. So a watchdog power-off picks its wake
// time so that (wake second % 64) == WDT_WAKE_TAG; a boot with no hibernate
// marker that starts within WDT_WAKE_WINDOW_S of such an alarm is a watchdog
// wake. (Normal hibernates always leave a marker file, so they never match.)
#define WDT_WAKE_TAG 37u
#define WDT_WAKE_WINDOW_S 30u
#define TUFR_WDT_TEST_DONE_FILE "TUFR_WDT_DONE.TXT"   // the test runs once; delete this file to run it again
static uint32_t gHibTag = 0xFFFFFFFFu;                 // next hibernate: align the wake second to this tag

// ---- SD card space and health (build 14a) ----
// Every SD problem (missing, unreadable, full, writes failing, file cannot be
// created) gives the same LED code (2: "insert or try another SD card") and a
// power cycle, so a replaced card is picked up within 60 s (10 min after three
// cycles). The log says which problem it was.
static uint64_t gFreeBytes = 0;        // free space, measured at boot, then counted down
static uint32_t gClusterBytes = 32768;
static uint8_t  gSdErrRun = 0;         // consecutive failed SD writes
#define SD_WRITE_FAIL_RUN 8            // this many in a row (about 0.3 s of audio): card problem
static uint64_t fileNeedBytes() {      // largest a file can get (FLAC <= WAV) + log growth + margin
  return (uint64_t)gOutFsHz * gNchan * 2u * gDurationS + 65536u + 2u * gClusterBytes + 1048576u;
}
static void powerCycleRecover(const char* why);
static void haltBlink(int code);
static void closeCurrentFile(bool deferLogs);
static void discardPreparedFile();

// RTWDOG (WDOG3) of the i.MX RT1062, clocked from the 32.768 kHz low-power
// clock with a /256 prescaler: 128 ticks per second.
#define TUFR_RTWDOG_CS    (*(volatile uint32_t *)0x400BC000)
#define TUFR_RTWDOG_CNT   (*(volatile uint32_t *)0x400BC004)
#define TUFR_RTWDOG_TOVAL (*(volatile uint32_t *)0x400BC008)
#define TUFR_RTWDOG_WIN   (*(volatile uint32_t *)0x400BC00C)
static void wdtConfigure(bool enable) {
#if TUFR_WATCHDOG
  CCM_CCGR5 |= (3u << 4);                          // WDOG3 register clock on
  __disable_irq();
  TUFR_RTWDOG_CNT = 0xD928C520;                    // unlock (32-bit command)
  for (int i = 0; i < 1000 && !(TUFR_RTWDOG_CS & (1u << 11)); i++) {}   // wait for ULK
  TUFR_RTWDOG_WIN = 0;
  TUFR_RTWDOG_TOVAL = enable ? (uint32_t)WDT_TIMEOUT_S * 128u : 0xFFFFu;
  TUFR_RTWDOG_CS = (1u << 13)                      // CMD32EN: 32-bit unlock/refresh words
                 | (1u << 12)                      // PRES: clock / 256
                 | (1u << 8)                       // CLK = 1: 32.768 kHz low-power clock
                 | (1u << 5)                       // UPDATE: may be reconfigured later
                 | (1u << 1)                       // keep counting while the CPU waits (WFI)
                 | (enable ? (1u << 7) : 0u);      // EN
  __enable_irq();
  for (int i = 0; i < 1000 && !(TUFR_RTWDOG_CS & (1u << 10)); i++) {}   // wait for RCS
#else
  (void)enable;
#endif
}
static inline void wdtFeed() {
#if TUFR_WATCHDOG
  TUFR_RTWDOG_CNT = 0xB480A602;                    // refresh
#endif
}
// delay() that keeps the watchdog fed (for waits longer than a second or so)
static void wdtDelay(uint32_t ms) {
  while (ms) { uint32_t d = ms > 100 ? 100 : ms; delay(d); ms -= d; wdtFeed(); }
}
static void recoverDeadStream();
static void adcAliveReport(const char* label);
static void settleAdc(const char* where);
// gZeroRunStartMs: millis() when the current run of all-zero blocks began
//                  (0 = the last block was live).
// gLastLiveMs:     ms since the file started at the last live block; printed in
//                  each file's stats line as liveUntilMs=.
static uint32_t gZeroRunStartMs = 0;
static uint32_t gLastLiveMs = 0;
static void haltIfAdcUnhealthy(const char* where);
static void dumpDeadStreamDiagnostics();

char status_text[7][16] = {"DO_START", "CLOSED", "RECORDING", "MUST_STOP", "JUST_STOPPED", "STOPPED", "MUST_HIBERNATE"};
volatile uint32_t fsamp = FSAMP;

// SD access uses SdFat (SdFs / FsFile / SdioConfig(FIFO_SDIO)). Declared ahead of
// readConfigFile(), which needs sd in scope to open TUFR_Setup.txt.
static SdFs sd;
// Two file handles so the NEXT file can be opened and preallocated while the current
// one is still recording (gap-free continuous mode): gFiles[gCur] is the file being
// written, gFiles[gCur ^ 1] the prepared next one. Everything below says "outFile".
static FsFile gFiles[2];
static uint8_t gCur = 0;
#define outFile gFiles[gCur]
static int16_t gLastWords[4];        // first samples of the most recent block (dead-stream diagnostics)
static uint32_t bytesWritten = 0;
static uint32_t startMs = 0;
static uint64_t startTicks = 0;   // hardware RTC (32768 Hz) at file start - see rtcTicks32k()
static uint32_t gFileStartAcq = 0; // acq_count (DMA block number) when the file was opened
static bool doneFlag = false;

// ---- Rolling files ----
// Back-to-back gDurationS-length files are written continuously until
// PIN_SHUTDOWN is pressed. Capture (acqInit/acqStart) keeps running underneath;
// only the destination FILE rotates. Each file: preAllocate -> header -> write
// -> truncate to real size -> patch header -> close.
static uint32_t gFileIndex = 0;
static char gFilename[40];

// Per-file write-timing stats (logged when each file closes). SD write stalls
// were the first thing these caught, so they stay.
static uint32_t writeTimeMinUs, writeTimeMaxUs, writeTimeSumUs, writeTimeCount;
static uint32_t sdWriteErrors;

static void resetWriteTimingStats() {
  writeTimeMinUs = 0xFFFFFFFF;
  writeTimeMaxUs = 0;
  writeTimeSumUs = 0;
  writeTimeCount = 0;
  sdWriteErrors = 0;
}

// ---- TUFR_Setup.txt parsing ----
// readNextLine() returns the next non-blank line (trailing "# comments" are left
// in place; sscanf stops at the numbers).
bool readNextLine(FsFile &file, char* buffer, int maxLen) {
  int i = 0;
  bool contentFound = false;
  while (file.available()) {
    char c = file.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (contentFound) break;
      else continue;
    }
    if (i < maxLen - 1) {
       if (!contentFound && isspace(c)) continue;
       contentFound = true;
       buffer[i++] = c;
    }
  }
  buffer[i] = '\0';
  return contentFound;
}

// Snaps a requested sample rate to the nearest one the hardware can sustain for
// this channel count: 48/96/192 kHz at any count, 384 kHz for 1-2 channels,
// 768 kHz for 1 channel. All of the 384 kHz/2ch, 768 kHz/1ch and 192 kHz/4ch
// cases run the same ~24.6 MHz bit clock. At 768 kHz/1ch each queue block
// fills in about 21 ms, so MAX_QUEUE (5 blocks) absorbs roughly 100 ms of SD
// write stall - watch acq_missed in the log at that rate.
uint32_t clampSampleRate(uint32_t requestedHz, uint8_t nchan) {
  const uint32_t VALID[5] = {48000, 96000, 192000, 384000, 768000};
  uint32_t best = 96000;
  uint32_t bestDiff = 0xFFFFFFFF;
  for (int i = 0; i < 5; i++) {
    uint32_t v = VALID[i];
    if (v == 384000 && nchan > 2) continue; // 384kHz only valid for 1-2 channels
    if (v == 768000 && nchan > 1) continue;  // 768kHz only valid for 1 channel
    uint32_t diff = (requestedHz > v) ? (requestedHz - v) : (v - requestedHz);
    if (diff < bestDiff) { bestDiff = diff; best = v; }
  }
  return best;
}

// TUFR_Setup.txt line order. Anything after "#" on a line is a comment;
// blank lines are skipped.
//   1  channels             1, 2 or 4 (sequential from input 1); 3 is recorded as 4
//   2  sample rate          kHz (192) or Hz (192000); -768 = 768 kHz decimated to 384 kHz files
//   3  record, wait         minutes; wait 0 = continuous
//   4  file format          WAV or FLAC (a word). If line 4 starts with a number
//                           instead, the format line is taken as missing: WAV,
//                           and that number is the gain. Any other word: WAV,
//                           logged as SETUP_FORMAT_UNKNOWN.
//   5  analog gain          0-42, ADC register units
//   6  high-pass filter     -1 / -2 / -3 = ADC presets 0.00025 / 0.002 / 0.008 x fS (-1 is the
//                           default; 96 / 768 / 3072 Hz at 384 kHz), 0 = off, >0 = cutoff in Hz
//   7  daily window         HHMM HHMM start stop (RTC time); 0000 2400 = always on
//   8  clock speed          MHz while recording
//   9  RTC sync             YYYY MM DD HH MM SS, applied only if later than the RTC
//  10  scheduled start      YYYY MM DD HH MM SS; zeros or a past time = start now
enum { SU_CHAN, SU_RATE, SU_DUTY, SU_FMT, SU_GAIN, SU_HPF, SU_DAILY, SU_CLOCK, SU_RTC, SU_SCHED, SU_COUNT };

void readConfigFile() {
  FsFile configFile = sd.open("TUFR_Setup.txt", O_READ);
  if (!configFile) {
    Serial.println("TUFR_Setup.txt not found - using defaults");
    return;
  }

  // Read the non-blank lines in order. The format line is the only one that
  // is a word; if line 4 is a number, the format line is missing and that
  // line (and everything after it) moves down one setting.
  char lines[SU_COUNT][160];
  bool have[SU_COUNT];
  for (int f = 0; f < SU_COUNT; f++) { have[f] = false; lines[f][0] = '\0'; }
  bool fmtMissing = false;
  for (int f = 0; f < SU_COUNT; f++) {
    char buf[160];
    if (!readNextLine(configFile, buf, sizeof(buf))) break;
    if (f == SU_FMT && !isalpha((unsigned char)buf[0])) {
      fmtMissing = true;            // no format line: this is the gain line
      f = SU_GAIN;
    }
    strcpy(lines[f], buf);
    have[f] = true;
  }
  configFile.close();
#define SL(f) (lines[f])
#define SH(f) (have[f])

  // RTC sync first, so every log line below is timestamped with the corrected
  // time. Applied only if AHEAD of the RTC, so a stale line never moves the
  // clock backward over a value already set correctly in the field.
  if (SH(SU_RTC)) {
    int yr, mo, dy, hr, mn, sc;
    if (sscanf(SL(SU_RTC), "%d %d %d %d %d %d", &yr, &mo, &dy, &hr, &mn, &sc) == 6) {
      tmElements_t tm;
      tm.Year = yr - 1970; tm.Month = mo; tm.Day = dy;
      tm.Hour = hr; tm.Minute = mn; tm.Second = sc;
      time_t setupTime = makeTime(tm);
#if RTC_HARDWARE_PRESENT
      if (setupTime > Teensy3Clock.get()) {
        Teensy3Clock.set(setupTime);
        setTime(setupTime);
        writeTelemetryLog("RTC_SYNC", "Updated from Config");
      }
#else
      // No coin cell: Teensy3Clock must not be touched, so apply directly.
      setTime(setupTime);
      writeTelemetryLog("RTC_SYNC", "Updated from Config (software clock only, no RTC hardware)");
#endif
    }
  }

  // File format: the word WAV or FLAC (any case). Missing line = WAV.
  gFlac = false;
  if (SH(SU_FMT)) {
    char word[8];
    int k = 0;
    for (const char* s = SL(SU_FMT); isalpha((unsigned char)*s) && k < 7; s++) word[k++] = (char)toupper(*s);
    word[k] = '\0';
    if (strcmp(word, "FLAC") == 0) gFlac = true;
    else if (strcmp(word, "WAV") != 0) {
      Serial.print("Setup line 4: unknown file format \""); Serial.print(word); Serial.println("\" - recording WAV");
      writeTelemetryLog("SETUP_FORMAT_UNKNOWN", "Setup line 4 is not WAV or FLAC - recording WAV");
    }
  } else if (fmtMissing) {
    writeTelemetryLog("SETUP_FORMAT_MISSING", "No WAV/FLAC line in TUFR_Setup.txt - recording WAV");
  }

  // Channels 1, 2 or 4. A request for 3 records 4: with 3 channels a 32 KB
  // block (16384 samples) is not a whole number of sample frames, so in
  // continuous WAV mode each new file would start on a different channel.
  if (SH(SU_CHAN)) {
    int nch;
    if (sscanf(SL(SU_CHAN), "%d", &nch) == 1 && nch >= 1 && nch <= 4) {
      if (nch == 3) {
        nch = 4;
        Serial.println("3 channels requested - recording 4 (input 4 as well)");
        writeTelemetryLog("CHANNELS_3_TO_4", "3 channels requested; recording 4 so channel order stays fixed across files");
      }
      gNchan = (uint8_t)nch;
    }
  }

  // Sample rate in kHz or Hz (e.g. "192" or "192000")
  if (SH(SU_RATE)) {
    long v;
    if (sscanf(SL(SU_RATE), "%ld", &v) == 1 && v != 0) {
      gDecimReq = (v < 0);          // -768: ADC at 768 kHz, file written at 384 kHz
      if (v < 0) v = -v;
      gFsHz = (v < 10000) ? (uint32_t)v * 1000UL : (uint32_t)v;
    }
  }

  // Record minutes, wait minutes. The configured wait is used as-is (an
  // in-place resume or a hibernate restart costs only a few seconds).
  if (SH(SU_DUTY)) {
    int r = 1, w = 0;
    if (sscanf(SL(SU_DUTY), "%d %d", &r, &w) >= 1 && r > 0) gDurationS = (uint32_t)r * 60;
    if (w > 0) gWaitS = (uint32_t)w * 60;
  }

  // Analog gain 0-42
  if (SH(SU_GAIN)) {
    int g;
    if (sscanf(SL(SU_GAIN), "%d", &g) == 1) {
      if (g < 0) g = 0;
      if (g > 42) g = 42;
      gGainVal = (int8_t)g;
    }
  }

  // High-pass: -1/-2/-3 = fixed presets, 0 = off, >0 = cutoff in Hz
  // (applied in adcRuntimeSetHpf() below once the final sample rate is known).
  if (SH(SU_HPF)) {
    long h;
    if (sscanf(SL(SU_HPF), "%ld", &h) == 1) {
      if (h < -3) h = -1;
      if (h > 100000) h = 100000;
      gHpfHz = (int32_t)h;
    }
  }

  // Daily recording window "HHMM HHMM" (start stop) in RTC time; "0000 2400"
  // (or a blank/invalid line) = off. Invalid values (hour > 24, minutes > 59,
  // start of 2400) are ignored and logged rather than risking a window that
  // never opens.
  if (SH(SU_DAILY)) {
    int startV, stopV;
    if (sscanf(SL(SU_DAILY), "%d %d", &startV, &stopV) == 2) {
      bool valid = startV >= 0 && startV < 2400 && stopV >= 0 && stopV <= 2400 &&
                   (startV % 100) < 60 && (stopV % 100) < 60;
      if (!valid) {
        writeTelemetryLog("DAILY_INVALID", "Daily window ignored: times must be HHMM, 0000-2400");
      } else if (!(startV == 0 && stopV == 2400)) {
        gDailyStartHHMM = startV;
        gDailyStopHHMM = stopV;
        gUseDailySchedule = true;
      }
    }
  }

  // Clock speed in MHz: applied via set_arm_clock() before capture starts and
  // again after each in-place wait.
  if (SH(SU_CLOCK)) {
    int ck;
    if (sscanf(SL(SU_CLOCK), "%d", &ck) == 1 && ck > 0) gClockMHz = (uint32_t)ck;
  }

  // Scheduled start (yr mo dy hr mn sc); yr == 0 (an all-zero line) = disabled.
  if (SH(SU_SCHED)) {
    int yr, mo, dy, hr, mn, sc;
    if (sscanf(SL(SU_SCHED), "%d %d %d %d %d %d", &yr, &mo, &dy, &hr, &mn, &sc) == 6 && yr > 0) {
      tmElements_t tm;
      tm.Year = yr - 1970; tm.Month = mo; tm.Day = dy;
      tm.Hour = hr; tm.Minute = mn; tm.Second = sc;
      gScheduledStartUnix = makeTime(tm);
    }
  }
#undef SL
#undef SH

  uint32_t clamped = clampSampleRate(gFsHz, gNchan);
  if (clamped != gFsHz) {
    Serial.print("Requested fs "); Serial.print(gFsHz);
    Serial.print("Hz clamped to "); Serial.print(clamped); Serial.println("Hz");
    writeTelemetryLog("FS_CLAMPED", "Sample rate adjusted to what this channel count supports");
  }
  gFsHz = clamped;
  // Decimation is only for 768 kHz on one channel; anything else records normally.
  queue_decimate = gDecimReq && gFsHz == 768000 && gNchan == 1;
  if (gDecimReq && !queue_decimate) {
    Serial.println("Negative rate (decimation) needs 768 kHz and 1 channel - ignored");
    writeTelemetryLog("DECIMATE_IGNORED", "Setup rate -768 needs 1 channel; recording without decimation");
  }
  gOutFsHz = queue_decimate ? gFsHz / 2 : gFsHz;
  // -768 with FLAC: the decimation filter (DMA interrupt) plus the encoder
  // need more than 60 MHz (bench: 63 % CPU at 60 MHz with quiet input), so
  // raise a lower clock setting to 100 MHz. (0 = IDE clock, left alone.)
  if (queue_decimate && gFlac && gClockMHz > 0 && gClockMHz < 100) {
    Serial.print("-768 with FLAC: clock raised from "); Serial.print(gClockMHz); Serial.println(" to 100 MHz");
    writeTelemetryLog("CLOCK_RAISED", "-768 with FLAC needs 100 MHz; clock setting raised to 100");
    gClockMHz = 100;
  }
  adcRuntimeSetFilter(gFiltSel, gFsHz);
  adcRuntimeSetHpf(gHpfHz, gFsHz); // needs the final sample rate; used by every ADC init

  Serial.print("TUFR_Setup.txt loaded - nchan="); Serial.print(gNchan);
  Serial.print(" fs="); Serial.print(gFsHz);
  Serial.print("Hz gain="); Serial.print(gGainVal);
  Serial.print(" highpass="); Serial.print(adcRuntimeHpfInfo());
  Serial.print(" record="); Serial.print(gDurationS); Serial.print("s");
  Serial.print(" wait="); Serial.print(gWaitS); Serial.print("s");
  Serial.print(" clockMHz="); Serial.print(gClockMHz);
  Serial.print(" format="); Serial.print(gFlac ? "FLAC" : "WAV");
  Serial.print(" scheduledStart="); Serial.print((unsigned long)gScheduledStartUnix);
  Serial.print(" daily=");
  if (gUseDailySchedule) { Serial.print(gDailyStartHHMM); Serial.print("-"); Serial.println(gDailyStopHHMM); }
  else Serial.println("off");
}

// ---- Duty-cycle sleep helpers ----
static void shutdownAdcAndCapture() {
  acqStop(); // i2s_stop() + adcRuntimeShutdown()
  // Drive the I2S pins low so they don't float or leak current while the ADC is
  // powered down.
  pinMode(PIN_I2S_DATA,  OUTPUT); digitalWrite(PIN_I2S_DATA,  LOW);
  pinMode(PIN_I2S_BCLK,  OUTPUT); digitalWrite(PIN_I2S_BCLK,  LOW);
  pinMode(PIN_I2S_LRCLK, OUTPUT); digitalWrite(PIN_I2S_LRCLK, LOW);
}

// Called when a file closes and gWaitS > 0, instead of starting the next
// file immediately. Returns true if recording should resume (and the
// caller should call startNewFile()); false if the shutdown button was
// pressed during the wait (caller should set doneFlag and stop, same as
// the normal shutdown path).
// Writes the marker, powers the ADC/capture down and hibernates for `secs`
// seconds. On success the chip restarts and this never returns. Returns false
// only if the marker couldn't be written (caller then uses an in-place
// wait, which needs no state across a restart).
// captureRunning: true when acqInit()/acqStart() have run (normal shutdown
// path); false when called early in setup() before capture exists (e.g.
// outside the daily window at power-up) - then acqStop() must NOT be called,
// and the ADC enable pins are just driven low directly.
// Marker file: "unix_time wait_s file_index kind power_cycles". kind 0 = duty-cycle /
// daily-window sleep, 1 = scheduled-start wait (the wake keeps waiting for the start
// time), 2 = power-cycle recovery (the wake starts recording as after a duty cycle).
static bool writeHibMarker(uint32_t unixT, uint32_t secs, uint32_t fileIdx, uint32_t kind) {
  FsFile m = sd.open(HIBERNATE_MARKER_FILE, O_WRONLY | O_CREAT | O_TRUNC);
  if (!m) return false;
  m.print((unsigned long)unixT); m.print(' ');
  m.print((unsigned long)secs); m.print(' ');
  m.print((unsigned long)fileIdx); m.print(' ');
  m.print((unsigned long)kind); m.print(' ');
  m.println((unsigned long)gPowerCycles);
  m.close();
  return true;
}

// Hold the ADC off and the I2S pins from floating while asleep.
static void parkPinsForSleep() {
  pinMode(PIN_ADC_EN, OUTPUT);    digitalWrite(PIN_ADC_EN, LOW);
  pinMode(PIN_ADC_SHDNZ, OUTPUT); digitalWrite(PIN_ADC_SHDNZ, LOW);
  pinMode(PIN_I2S_DATA,  OUTPUT); digitalWrite(PIN_I2S_DATA,  LOW);
  pinMode(PIN_I2S_BCLK,  OUTPUT); digitalWrite(PIN_I2S_BCLK,  LOW);
  pinMode(PIN_I2S_LRCLK, OUTPUT); digitalWrite(PIN_I2S_LRCLK, LOW);
}

// Walter Zimmer's hibernate (Teensy.cpp doHibernate()): set the SNVS (RTC)
// alarm to now+secs, then set the "turn off power" bit. The chip powers down,
// the RTC alarm powers it back up, and it boots from setup() like a cold start.
// Bench result 2026-11-03 (SleepBisect2 variant 17): slept the full 60 s at
// 0.75 mA with the SD card initialized, Wire.begin() already run and 60 MHz.
// Does not return when it works; if it returns (power-off had no effect) the
// caller restarts the chip.
static void snvsHibernate(uint32_t secs) {
  uint32_t tmp = SNVS_LPCR;
  SNVS_LPSR |= 1;
  asm volatile("DSB");
  SNVS_LPCR &= ~0x2u;                      // disable the alarm (LPTA_EN)
  while (SNVS_LPCR & 0x2u);
  __disable_irq();
  uint32_t lsb, msb;
  do { msb = SNVS_LPSRTCMR; lsb = SNVS_LPSRTCLR; }
  while ((SNVS_LPSRTCLR != lsb) | (SNVS_LPSRTCMR != msb));
  uint32_t target = ((msb << 17) | (lsb >> 15)) + secs; // absolute RTC seconds
  if (gHibTag < 64u) target += (gHibTag + 64u - (target & 63u)) & 63u;  // watchdog tag: wake second % 64 == tag
  SNVS_LPTAR = target;
  while (SNVS_LPTAR != target);
  SNVS_LPCR = tmp | 0x2u;                  // alarm on
  while (!(SNVS_LPCR & 0x2u));
  __enable_irq();
  SNVS_LPCR |= (1 << 6);                   // turn off power
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) asm("wfi"); // power should drop within microseconds
}

static void hibernateNow(uint32_t secs) {
  snvsHibernate(secs);
  // Not expected: the chip should have powered down. Restart so the
  // marker-driven wake path in setup() runs rather than resuming with a
  // half-powered-down capture chain.
  Serial.println("checkpoint: hibernate did not power down - forcing a restart");
  writeTelemetryLog("HIBERNATE_RETURNED", "unexpected return - forcing restart");
  delay(50);
  SCB_AIRCR = 0x05FA0004;
  while (1) {}
}

// Writes the marker, powers the ADC/capture down and hibernates for `secs`
// seconds. On success the chip powers down and restarts at the end of the
// wait, so this never returns. Returns false only if the marker couldn't be
// written (caller then uses an in-place wait, which needs no state across a
// restart).
// captureRunning: true when acqInit()/acqStart() have run (normal shutdown
// path); false when called early in setup() before capture exists (e.g.
// outside the daily window at power-up) - then acqStop() must NOT be called.
static bool hibernateAndRestart(uint32_t secs, bool captureRunning, uint32_t kind) {
  if (!writeHibMarker((uint32_t)now(), secs, (uint32_t)gFileIndex, kind)) {
    writeTelemetryLog("HIBERNATE_SKIP", "could not create marker file - using normal wait");
    return false;
  }

  char msg[64];
  sprintf(msg, "Hibernating for %lus", (unsigned long)secs);
  if (kind == 0) writeTelemetryLog("HIBERNATE_START", msg); // heartbeat sleeps would flood the log
  Serial.print("checkpoint: "); Serial.println(msg);
  Serial.flush();
  delay(20);

  if (captureRunning) shutdownAdcAndCapture();
  parkPinsForSleep();
  hibernateNow(secs);
  return true; // unreachable
}

// Reads and deletes the marker. Returns true if this boot is a hibernate wake.
static bool readAndClearHibernateMarker(uint32_t* sleepUnix, uint32_t* waitS, uint32_t* fileIdx, uint32_t* kind) {
  *kind = 0;
  if (!sd.exists(HIBERNATE_MARKER_FILE)) return false;
  bool ok = false;
  FsFile m = sd.open(HIBERNATE_MARKER_FILE, O_RDONLY);
  if (m) {
    char buf[64] = {0};
    int n = m.read(buf, sizeof(buf) - 1);
    m.close();
    unsigned long a = 0, b = 0, c = 0, d = 0, e = 0;
    int got = (n > 0) ? sscanf(buf, "%lu %lu %lu %lu %lu", &a, &b, &c, &d, &e) : 0;
    if (got >= 3) {
      *sleepUnix = a; *waitS = b; *fileIdx = c; *kind = (got >= 4) ? d : 0;
      gPowerCycles = (got >= 5) ? e : 0;
      ok = true;
    }
  }
  sd.remove(HIBERNATE_MARKER_FILE);
  return ok;
}

// ---- Daily recording window ----
// Checked against TimeLib's clock (which follows the RTC). Resolution is one
// minute; the stop minute itself is outside the window.
static bool isWithinDailySchedule() {
  int cur = hour() * 100 + minute();
  if (gDailyStartHHMM < gDailyStopHHMM) return (cur >= gDailyStartHHMM && cur < gDailyStopHHMM);
  return (cur >= gDailyStartHHMM || cur < gDailyStopHHMM); // crosses midnight
}

// Seconds from now until the window next opens (only meaningful while
// outside the window; always 1..86400).
static uint32_t secondsUntilWindowOpens() {
  uint32_t nowS   = (uint32_t)hour() * 3600UL + (uint32_t)minute() * 60UL + (uint32_t)second();
  uint32_t startS = (uint32_t)(gDailyStartHHMM / 100) * 3600UL + (uint32_t)(gDailyStartHHMM % 100) * 60UL;
  uint32_t d = (startS + 86400UL - nowS) % 86400UL;
  return d ? d : 1;
}

// Outside the recording window: power everything down and hibernate toward
// the next window start. A hibernate is capped at DAILY_SLEEP_CHUNK_S; each
// wake re-checks the clock in setup() and, if still outside the window,
// goes straight back to sleep (no ADC start, ~seconds of boot per hour).
// Does not return when it sleeps. Returns false if the shutdown button was
// pressed (caller should stop). If the marker file can't be written it falls
// back to waiting in place at the low clock speed, then reboots when the
// window opens, like TUFR_4.0.ino's handleDailySleep().
static bool dailySleepUntilWindowOpens(bool captureRunning) {
  if (digitalRead(PIN_SHUTDOWN) == LOW) {
    writeTelemetryLog("HALT_USER", "Shutdown before daily sleep");
    return false;
  }
  uint32_t untilOpen = secondsUntilWindowOpens();
  uint32_t chunk = untilOpen > DAILY_SLEEP_CHUNK_S ? DAILY_SLEEP_CHUNK_S : untilOpen;
  char msg[80];
  sprintf(msg, "Outside window %04d-%04d, opens in %lus", gDailyStartHHMM, gDailyStopHHMM, (unsigned long)untilOpen);
  writeTelemetryLog("DAILY_SLEEP", msg);
  Serial.print("checkpoint: "); Serial.println(msg);

  hibernateAndRestart(chunk, captureRunning, 0); // does not return when it sleeps

  // Fallback (marker file couldn't be written): wait in place, then reboot.
  if (captureRunning) shutdownAdcAndCapture();
  set_arm_clock(WAIT_CLOCK_FREQ);
  delay(10);
  unsigned long lastBlink = millis();
  while (!isWithinDailySchedule()) {
    wdtFeed();
    if (digitalRead(PIN_SHUTDOWN) == LOW) {
      writeTelemetryLog("HALT_USER", "Shutdown during daily sleep");
      return false;
    }
    if ((millis() - lastBlink) > 10000) {
      digitalWrite(LED_BUILTIN, HIGH); delay(10); digitalWrite(LED_BUILTIN, LOW);
      lastBlink = millis();
    }
    delay(100);
  }
  writeTelemetryLog("DAILY_WAKE", "Window opened, rebooting");
  delay(100);
  SCB_AIRCR = 0x05FA0004;
  while (1) {}
  return true; // unreachable
}

static bool enterDutyCycleWait() {
  if (gWaitS >= HIBERNATE_MIN_WAIT_S) {
    if (digitalRead(PIN_SHUTDOWN) == LOW) {
      writeTelemetryLog("HALT_USER", "Shutdown before hibernate");
      return false;
    }
    hibernateAndRestart(gWaitS, true, 0); // does not return unless the marker file couldn't be written
  }
  writeTelemetryLog("WAIT_START", "Entering Duty Cycle Sleep");
  Serial.print("checkpoint: entering duty-cycle wait for "); Serial.print(gWaitS); Serial.println("s");
  shutdownAdcAndCapture();
  set_arm_clock(WAIT_CLOCK_FREQ);
  delay(10);

  unsigned long waitStart = millis();
  unsigned long lastBlink = millis();
  bool shutdownPressed = false;
  while ((millis() - waitStart) < gWaitS * 1000UL) {
    wdtFeed();
    if (digitalRead(PIN_SHUTDOWN) == LOW) { shutdownPressed = true; break; }
    if ((millis() - lastBlink) > 10000) {
      digitalWrite(LED_BUILTIN, HIGH); delay(10); digitalWrite(LED_BUILTIN, LOW);
      lastBlink = millis();
    }
    delay(100);
  }

  // Restore normal clock speed before touching the ADC/I2S again, whether
  // we're about to resume recording or about to halt.
  if (gClockMHz > 0) set_arm_clock(gClockMHz * 1000000UL);
  delay(10);

  if (shutdownPressed) {
    writeTelemetryLog("HALT_USER", "Shutdown during Duty Cycle wait");
    Serial.println("checkpoint: shutdown button pressed during duty-cycle wait");
    return false;
  }

  writeTelemetryLog("WAIT_END", "Resuming recording");
  Serial.println("checkpoint: duty-cycle wait complete - resuming (acqStart only)");
  // Resume in place (no reboot). acqStart() ONLY - see the NOTE near the top of
  // this file: a bench hang on the 4th duty cycle was traced to calling
  // i2s_setup() again here, whose set_audioClock() waits forever for a lock bit
  // on an audio PLL that was never powered down.
  acqStart();
  haltIfAdcUnhealthy("resume after wait"); // does not return if the ADC isn't responding
  settleAdc("resume after wait");
  writeTelemetryLog("RESUME_DONE", "acqStart() returned (no i2s_setup()/set_audioClock() - see comment)");
  return true;
}

// ---- Telemetry log (TUFR_log.txt) ----
void writeTelemetryLog(const char* event, const char* val) {
  FsFile logFile = sd.open("TUFR_log.txt", O_RDWR | O_CREAT | O_AT_END);
  if (logFile) {
    logFile.print(year()); logFile.print("-"); logFile.print(month()); logFile.print("-"); logFile.print(day());
    logFile.print(" "); logFile.print(hour()); logFile.print(":"); logFile.print(minute()); logFile.print(":"); logFile.print(second());
    logFile.print(", Event: "); logFile.print(event);
    logFile.print(", Val: "); logFile.println(val);
    logFile.close();
  }
}

// ---- Supply voltage (Vin) from the divider on PIN_BATT (pin 22, A8) ----
// Same idea as TUFR 4.0's readBatteryVoltage(), but with no delay() calls so it can
// run during recording: one throw-away read to charge the sample capacitor, then the
// average of 8 reads (12-bit, 3.3 V reference). Takes well under 1 ms.
static float readVin() {
  analogReadResolution(12);
  (void)analogRead(PIN_BATT);
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(PIN_BATT);
  return ((float)sum / 8.0f) * (3.3f / 4095.0f) * TUFR_VIN_DIVIDER;
}

// ---- SD file modify/create timestamp callback - ported verbatim from
// TUFR_4.0.ino, registered via FsDateTime::setCallback() in setup() below,
// so files on the card carry correct dates instead of SdFat's default
// (2000-01-01). ----
void dateTime(uint16_t* date, uint16_t* time) {
  *date = FS_DATE(year(), month(), day());
  *time = FS_TIME(hour(), minute(), second());
}

// ---- Minimal 44-byte PCM WAV header, patched with real sizes on close ----
// Now uses runtime gFsHz/gNchan (from TUFR_Setup.txt) instead of the old
// compile-time FS_HZ/NCHAN_I2S constants, so the header correctly
// reflects whatever the config file actually requested.
static void writeWavHeader(FsFile &f, uint32_t dataBytes) {
  uint32_t byteRate = gOutFsHz * gNchan * 2;
  uint16_t blockAlign = gNchan * 2;
  // 0xFFFFFFFF = "length unknown" (placeholder at file open): a file cut off by
  // a reset or power loss is then read to its end instead of as empty.
  uint32_t riffSize = (dataBytes == 0xFFFFFFFFu) ? 0xFFFFFFFFu : 36 + dataBytes;

  f.seek(0);
  f.write((const uint8_t*)"RIFF", 4);
  f.write((uint8_t*)&riffSize, 4);
  f.write((const uint8_t*)"WAVE", 4);
  f.write((const uint8_t*)"fmt ", 4);
  uint32_t fmtLen = 16; f.write((uint8_t*)&fmtLen, 4);
  uint16_t audioFmt = 1; f.write((uint8_t*)&audioFmt, 2);
  uint16_t nch = gNchan; f.write((uint8_t*)&nch, 2);
  f.write((uint8_t*)&gOutFsHz, 4);
  f.write((uint8_t*)&byteRate, 4);
  f.write((uint8_t*)&blockAlign, 2);
  uint16_t bitsPerSample = 16; f.write((uint8_t*)&bitsPerSample, 2);
  f.write((const uint8_t*)"data", 4);
  f.write((uint8_t*)&dataBytes, 4);
}

// ---- FLAC output (setup line 4 = FLAC) ----
// The encoder (tufr_flac.c) turns each 32 KB queue block into FLAC frames of
// 2048 samples per channel; the frames collect in gFlacStage and go to the SD
// card in 32 KB writes, the same write size as WAV (just fewer of them). The
// file starts with a 512-byte header (STREAMINFO + padding), so every 32 KB
// write stays sector-aligned; the header is rewritten at close with the
// sample count, like the WAV header. A file cut off by power loss still
// decodes up to its last complete frame.
// Encoding runs in loop(), not in the DMA interrupt; the queue (~300 ms)
// absorbs the time it takes. The per-file FLAC_STATS log line reports the
// compression and the CPU share the encoder used (EncCPU) - keep that well
// under ~60 % (raise line 8, the clock, if it is not).
// RAM (all in RAM1/DTCM, reserved even when recording WAV): encoder 26.8 KB +
// frame buffer 16.4 KB + staging 48.4 KB = about 92 KB. If the compile reports
// RAM1 is short, build tufr_flac.c with -DTUFR_FLAC_MAX_BLOCK=1024 (halves the
// encoder and frame buffer; files about 0.3 % larger).
#define FLAC_STAGE_BYTES 32768u
static tufr_flac_t gFlacEnc;
static uint8_t  gFlacFrame[TUFR_FLAC_FRAME_MAX(TUFR_FLAC_MAX_BLOCK, TUFR_FLAC_MAX_CH)];
static uint8_t  gFlacStage[FLAC_STAGE_BYTES + sizeof(gFlacFrame)];
static uint32_t gFlacFill = 0;           // bytes waiting in gFlacStage
static uint64_t gPcmBytes = 0;           // audio bytes given to the encoder this file (= WAV data size)
static uint64_t gFlacEncCycles = 0;      // CPU cycles spent encoding this file (SD writes excluded)
static uint32_t gFlacWriteCycles = 0;    // cycles inside SD writes during the current push
static uint64_t gFlacFileCycles = 0;     // elapsed CPU cycles this file (for EncCPU)
static uint32_t gFlacLastCyc = 0;

static void writeFlacHeader(FsFile &f, const tufr_flac_t *st) {
  uint8_t h[TUFR_FLAC_HEADER_BYTES];
  tufr_flac_header(st, h);
  f.seek(0);
  f.write(h, sizeof(h));
}

// One SD write of n staged bytes, with the same timing statistics as WAV writes.
static void flacWriteStaged(uint32_t n) {
  uint32_t c0 = ARM_DWT_CYCCNT;
  uint32_t t0 = micros();
  size_t w = outFile.write(gFlacStage, n);
  uint32_t dt = micros() - t0;
  gFlacWriteCycles += ARM_DWT_CYCCNT - c0;
  if (dt < writeTimeMinUs) writeTimeMinUs = dt;
  if (dt > writeTimeMaxUs) writeTimeMaxUs = dt;
  writeTimeSumUs += dt;
  writeTimeCount++;
  if (w == n) { bytesWritten += n; gSdErrRun = 0; } else { sdWriteErrors++; if (gSdErrRun < 255) gSdErrRun++; }
}

// Encoder output sink: collect frames, write every full 32 KB.
static int flacEmit(const uint8_t *d, size_t len, void *ctx) {
  (void)ctx;
  memcpy(gFlacStage + gFlacFill, d, len);
  gFlacFill += len;
  if (gFlacFill >= FLAC_STAGE_BYTES) {
    flacWriteStaged(FLAC_STAGE_BYTES);
    gFlacFill -= FLAC_STAGE_BYTES;
    if (gFlacFill) memmove(gFlacStage, gFlacStage + FLAC_STAGE_BYTES, gFlacFill);
  }
  return 1;
}

// Encode one queue block (nsamp interleaved samples) into the current file.
static void flacWriteBlock(const int16_t *blk, uint32_t nsamp) {
  uint32_t now = ARM_DWT_CYCCNT;
  gFlacFileCycles += (uint32_t)(now - gFlacLastCyc);
  gFlacLastCyc = now;
  gFlacWriteCycles = 0;
  uint32_t c0 = ARM_DWT_CYCCNT;
  if (!tufr_flac_push(&gFlacEnc, blk, nsamp, gFlacFrame, flacEmit, NULL)) sdWriteErrors++;
  gFlacEncCycles += (uint32_t)(ARM_DWT_CYCCNT - c0) - gFlacWriteCycles;
  gPcmBytes += (uint64_t)nsamp * 2u;
}

// Per-file encoder counters; keepCarry = this file continues a gap-free stream.
static void flacStartFile(bool keepCarry) {
  if (keepCarry) tufr_flac_new_file(&gFlacEnc);
  else tufr_flac_init(&gFlacEnc, gOutFsHz, gNchan, TUFR_FLAC_MAX_BLOCK);
  gFlacFill = 0;
  gPcmBytes = 0;
  gFlacEncCycles = 0;
  gFlacFileCycles = 0;
  gFlacLastCyc = ARM_DWT_CYCCNT;
}

// Placeholder header for a new (or prepared) file: sample count "unknown".
static void writeFileHeaderPlaceholder(FsFile &f) {
  if (gFlac) {
    tufr_flac_t t;
    tufr_flac_init(&t, gOutFsHz, gNchan, TUFR_FLAC_MAX_BLOCK);
    writeFlacHeader(f, &t);
  } else {
    writeWavHeader(f, 0xFFFFFFFFu);   // "length unknown" until closeCurrentFile() patches it
  }
}

// ---- Rolling file open/close - ported from TUFR_4.0.ino's startNewFile()/
// closeCurrentFile() (see the comment by gFileIndex's declaration above
// for what's simplified vs. the real firmware). ----
// Wall-clock time from the chip's RTC counter (32768 ticks per second). millis()
// is NOT used to time files: (older builds:) queue.pull() copied a 64 KB block with interrupts
// disabled, which at low CPU clocks (24 MHz: ~2 ms) swallows SysTick interrupts,
// so millis() ran about 5% slow and 60 s files held 63 s of audio. The RTC counter
// runs from its own crystal and cannot lose ticks. Without an RTC battery
// (RTC_HARDWARE_PRESENT==0) the SNVS counter is never touched, so millis() is used.
static uint64_t rtcTicks32k() {
#if RTC_HARDWARE_PRESENT
  uint32_t msb, lsb;
  do { msb = SNVS_LPSRTCMR; lsb = SNVS_LPSRTCLR; }
  while ((SNVS_LPSRTCLR != lsb) | (SNVS_LPSRTCMR != msb));
  return ((uint64_t)msb << 32) | lsb;
#else
  return (uint64_t)millis() * 32768ULL / 1000ULL;
#endif
}

// ---- Deferred log lines (continuous-mode file rotation) ----
// While a file is being closed/opened nothing drains the capture queue (about 85 ms
// of audio), so every SD log write made inside that window lengthens the gap. The
// rotation stores its log lines here instead and loop() writes them one at a time
// once the new file is running and the queue is empty.
struct PendingLog { char ev[48]; char val[240]; };
static PendingLog gPend[4];
static int gPendN = 0;
static void logDeferred(const char* ev, const char* val) {
  if (gPendN < 4) {
    strncpy(gPend[gPendN].ev, ev, sizeof(gPend[0].ev) - 1);  gPend[gPendN].ev[sizeof(gPend[0].ev) - 1] = 0;
    strncpy(gPend[gPendN].val, val, sizeof(gPend[0].val) - 1); gPend[gPendN].val[sizeof(gPend[0].val) - 1] = 0;
    gPendN++;
  } else {
    writeTelemetryLog(ev, val);
  }
}
static void flushOneDeferredLog() {
  if (gPendN <= 0) return;
  writeTelemetryLog(gPend[0].ev, gPend[0].val);
  for (int i = 1; i < gPendN; i++) gPend[i - 1] = gPend[i];
  gPendN--;
}
static void flushAllDeferredLogs() { while (gPendN > 0) flushOneDeferredLog(); }

static uint32_t gOpenMs = 0, gPreallocMs = 0;   // this file's open / preAllocate time
static uint32_t gTruncMs = 0, gCloseMs = 0;     // last close: truncate, header patch + close
static uint32_t gEndMs = 0;                     // recording length when closeCurrentFile() began
static uint32_t ticksToMs(uint64_t t) { return (uint32_t)((t * 1000ULL) >> 15); }

// ---- Gap-free file rotation (continuous mode) ----
// Opening a file (~130 ms) and preallocating it (~190 ms) used to happen at the join,
// longer than the queue could cover. Now they happen a few seconds EARLIER, one at a
// time, each only when the queue has just been drained (so the whole queue is headroom
// and each step is shorter than it). At the join only the old file's close (~25 ms)
// remains, and the new file is simply adopted by startNewFile().
static uint8_t  gNextStage = 0;      // 0 = nothing done, 1 = opened, 2 = preallocated + header written (or given up)
static bool     gNextOpened = false, gNextReady = false;
static char     gNextName[40];
static uint32_t gNextOpenMs = 0, gNextPreMs = 0;

static void discardPreparedFile() {
  if (gNextOpened) {
    FsFile &nf = gFiles[gCur ^ 1];
    nf.close();
    sd.remove(gNextName);
  }
  gNextOpened = false; gNextReady = false; gNextStage = 0;
}

static void prepareNextFileSteps() {
  if (gNextStage >= 2) return;
  if (queue.available() > 0) return;      // wait until drained: all of the queue is headroom
  uint32_t durMs = gDurationS * 1000UL;
  uint32_t el = ticksToMs(rtcTicks32k() - startTicks);
  uint32_t lead1 = durMs * 2 / 5; if (lead1 > 6000) lead1 = 6000;   // open this long before the join
  uint32_t lead2 = lead1 / 2;                                         // preallocate this long before it
  if (gNextStage == 0) {
    if (el + lead1 < durMs) return;
    if (gFreeBytes < 2 * fileNeedBytes()) { gNextStage = 2; return; } // no room for another file: the join checks again
    if (gUseDailySchedule && !isWithinDailySchedule()) { gNextStage = 2; return; } // window closed: no next file
    time_t tn = now() + (durMs > el ? (durMs - el + 500) / 1000 : 0);  // the name the file will have at the join
    sprintf(gNextName, "TUFR_%04d%02d%02d_%02d%02d%02d.%s", year(tn), month(tn), day(tn), hour(tn), minute(tn), second(tn), gFlac ? "FLAC" : "WAV");
    uint64_t t0 = rtcTicks32k();
    gFiles[gCur ^ 1] = sd.open(gNextName, O_RDWR | O_CREAT | O_TRUNC);
    gNextOpenMs = ticksToMs(rtcTicks32k() - t0);
    if (!gFiles[gCur ^ 1]) { gNextStage = 2; return; }  // could not: fall back to opening at the join
    gNextOpened = true;
    gNextStage = 1;
    return;
  }
  if (el + lead2 < durMs) return;
  FsFile &nf = gFiles[gCur ^ 1];
  gNextPreMs = 0;
#if TUFR_PREALLOCATE
  uint64_t expectedBytes = (uint64_t)gOutFsHz * gNchan * 2 * gDurationS + 44 + 65536;
  uint64_t t0 = rtcTicks32k();
  nf.preAllocate(expectedBytes);
  gNextPreMs = ticksToMs(rtcTicks32k() - t0);
#endif
  writeFileHeaderPlaceholder(nf); // placeholder, patched when this file is closed
  gNextReady = true;
  gNextStage = 2;
}

static void cardFull() {
  char m[100];
  snprintf(m, sizeof(m), "%lu MB free, a %lu s file needs up to %lu MB - insert or try another SD card",
           (unsigned long)(gFreeBytes >> 20), (unsigned long)gDurationS, (unsigned long)(fileNeedBytes() >> 20));
  Serial.println(m);
  writeTelemetryLog("CARD_FULL", m);   // may not fit any more either
  discardPreparedFile();
  haltBlink(2);                        // 2 blinks x3, then a power cycle (does not return)
}

static void startNewFile() {
  if (gFreeBytes < fileNeedBytes()) cardFull();
  bool adopted = gNextReady;   // continuous mode with the next file prepared in advance
  if (gNextReady) {
    // Prepared during the previous file: just make it the current one.
    gCur ^= 1;
    strcpy(gFilename, gNextName);
    gOpenMs = gNextOpenMs; gPreallocMs = gNextPreMs;
    gNextOpened = false; gNextReady = false; gNextStage = 0;
    Serial.print("checkpoint: "); Serial.print(gFilename); Serial.println(" prepared in advance");
  } else {
    gNextOpened = false; gNextStage = 0;
  // RTC-based filename, matching TUFR_4.0.ino's startNewFile() exactly
  // (previously an incrementing index - TUFR_TEST_NNNN.WAV - now that the
  // RTC is hooked and synced from TUFR_Setup.txt above). gFileIndex is
  // still tracked, but only for the "N file(s)" count in the shutdown
  // message, not for naming.
  sprintf(gFilename, "TUFR_%04d%02d%02d_%02d%02d%02d.%s", year(), month(), day(), hour(), minute(), second(), gFlac ? "FLAC" : "WAV");
  uint64_t tOpen0 = rtcTicks32k();
  outFile = sd.open(gFilename, O_RDWR | O_CREAT | O_TRUNC);
  gOpenMs = ticksToMs(rtcTicks32k() - tOpen0);
  if (!outFile) {
    Serial.print("sd.open FAILED for "); Serial.print(gFilename);
    Serial.println(" - LED 2 blinks, then a power cycle");
    writeTelemetryLog("SD_OPEN_FAILED", gFilename);
    haltBlink(2);
  }

  // Reserve contiguous space up front, exactly as TUFR_4.0.ino does and
  // for the same reason - avoids a mid-recording FAT-extend stall. Must
  // happen before any data is written (preAllocate() needs an empty file).
  bool preallocOk = true;
  gPreallocMs = 0;
#if TUFR_PREALLOCATE
  uint64_t expectedBytes = (uint64_t)gOutFsHz * gNchan * 2 * gDurationS + 44 + 65536;
  uint64_t tPre0 = rtcTicks32k();
  preallocOk = outFile.preAllocate(expectedBytes);
  gPreallocMs = ticksToMs(rtcTicks32k() - tPre0);
#endif
  uint32_t preallocUs = gPreallocMs * 1000UL;
  Serial.print("checkpoint: "); Serial.print(gFilename);
  Serial.print(" preAllocate "); Serial.print(preallocOk ? "OK" : "FAIL");
  Serial.print(", "); Serial.print(preallocUs); Serial.println("us");

  writeFileHeaderPlaceholder(outFile); // placeholder, patched in closeCurrentFile()
  }
  bytesWritten = 0;
  gRecoveries = 0;     // each file gets its own in-place ADC restarts
  if (gFlac) flacStartFile(adopted);  // a prepared (continuous) file continues the stream
  gLastLiveMs = 0;      // (gZeroRunStartMs deliberately NOT reset: a stream that
                        // died before the file boundary is still dead after it)
  resetWriteTimingStats();
  startMs = millis();
  // A prepared file continues the audio stream exactly where the last one ended, so its
  // clock starts at the previous file's scheduled end - not "now", which is ~25 ms later
  // (the old file's close) and would make every file that much longer.
  if (adopted) startTicks += (uint64_t)gDurationS * 32768ULL;
  else startTicks = rtcTicks32k();
  gFileStartAcq = acq_count; // misses logged before this are "gap" misses (file rotation)
}

// Reads one ADC register over I2C (address 0x4C). Returns -1 if it did not answer.
static int adcRegRead(uint8_t reg) {
  Wire.beginTransmission(0x4C);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)0x4C, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

// Called once, at the moment the recording has been all zeros for
// DEAD_STREAM_RECOVER_MS. Tells apart "the ADC lost its configuration / reset"
// (registers back at power-up defaults: 0x02=0x00, 0x73=0xF0, 0x74=0x00,
// 0x75=0x00), "the ADC is still configured but silent" (registers still
// 0x81/0xC0/0xC0/0xE0), and "the ADC does not answer at all" (-1), and shows
// whether the Teensy's I2S receiver flagged an error.
static void dumpDeadStreamDiagnostics() {
  char m[200];
  int r02 = adcRegRead(0x02), r15 = adcRegRead(0x15), r73 = adcRegRead(0x73);
  int r74 = adcRegRead(0x74), r75 = adcRegRead(0x75), r76 = adcRegRead(0x76);
  sprintf(m, "ADC regs 0x02=%02X 0x15=%02X 0x73=%02X 0x74=%02X 0x75=%02X 0x76=%02X (-1 = no answer; working: 81/74/C0/C0/E0/C0)",
          r02 & 0xFFFF, r15 & 0xFFFF, r73 & 0xFFFF, r74 & 0xFFFF, r75 & 0xFFFF, r76 & 0xFFFF);
  Serial.println(m);
  writeTelemetryLog("DEAD_STREAM_ADC", m);
  // I2S receiver status: bit18 FEF = FIFO overrun, bit19 SEF = sync error,
  // bit20 WSF = word start, bit31 RE = receiver enabled (should be 1).
  sprintf(m, "I2S1_RCSR=%08lX (FEF=%d SEF=%d WSF=%d RE=%d) liveUntilMs=%lu acq_missed=%lu qavail=%d",
          (unsigned long)I2S1_RCSR, (int)((I2S1_RCSR >> 18) & 1), (int)((I2S1_RCSR >> 19) & 1),
          (int)((I2S1_RCSR >> 20) & 1), (int)((I2S1_RCSR >> 31) & 1),
          (unsigned long)gLastLiveMs, (unsigned long)acq_missed, queue.available());
  Serial.println(m);
  writeTelemetryLog("DEAD_STREAM_SAI", m);
  sprintf(m, "last samples (16-bit): %04lX %04lX %04lX %04lX",
          (unsigned long)(uint16_t)gLastWords[0], (unsigned long)(uint16_t)gLastWords[1],
          (unsigned long)(uint16_t)gLastWords[2], (unsigned long)(uint16_t)gLastWords[3]);
  Serial.println(m);
  writeTelemetryLog("DEAD_STREAM_RAW", m);
  char hm[240];
  adcRuntimeHpfDump(hm, sizeof(hm)); // 0x6B and the high-pass coefficients as the ADC holds them
  Serial.println(hm);
  writeTelemetryLog("DEAD_STREAM_HPF", hm);
}

// Recovery from a dead data stream (2026-11-03): the new-build bench logs showed
// the ADC stops answering I2C and outputs zeros within ~100 ms of capture
// starting at 384 kHz/2ch on the motherboard. Instead of giving up, power-cycle
// the ADC (acqStop() = I2S off + ADC shutdown) and bring it back up exactly as
// the duty-cycle resume does (acqStart() = I2S on + full ADC init, which itself
// retries up to 3 times). The open file keeps recording through the gap.
// After DEAD_STREAM_INPLACE_TRIES tries in one file: full power cycle (see
// "Unattended recovery").
static void recoverDeadStream() {
  gRecoveries++;
  if (adcRuntimeHpfIsCustom()) {
    // A bad programmable high-pass setting must not cost the recording: fall back
    // to the ADC's fixed default preset for the rest of this run.
    adcRuntimeSetHpf(-1, gFsHz);
    writeTelemetryLog("HPF_FALLBACK", "dead stream with the programmable high-pass: using the default preset (0.00025 x fS) from now on");
  }
  char m[96];
  sprintf(m, "dead stream: in-place restart %u of %u - power-cycling ADC, restarting capture",
          (unsigned)gRecoveries, (unsigned)DEAD_STREAM_INPLACE_TRIES);
  Serial.println(m);
  writeTelemetryLog("DEAD_STREAM_RECOVER", m);
  acqStop();
  // Pause before powering up again: lets the ADC rail discharge (0.3 s) and,
  // on later tries, gives a transient such as an input overload time to pass.
  static const uint16_t pauseMs[DEAD_STREAM_INPLACE_TRIES] = {300, 2000, 10000};
  wdtDelay(pauseMs[(gRecoveries - 1) < DEAD_STREAM_INPLACE_TRIES ? (gRecoveries - 1) : DEAD_STREAM_INPLACE_TRIES - 1]);
  queue.reset();       // discard zero blocks still queued
  if (gFlac) tufr_flac_pad_frame(&gFlacEnc, gFlacFrame, flacEmit, NULL); // restarted stream begins a new FLAC frame
  acqStart();
  haltIfAdcUnhealthy("dead-stream recovery"); // halts (5 blinks) if the ADC won't come back
  gZeroRunStartMs = 0; // judge the restarted stream afresh
}

// v4 bench log (2026-11-03): at 384 kHz/2ch the ADC answered right after init
// but not 100 ms later (cold start), and the dead-stream recovery then fixed
// it ~half a second into the file. This does the same thing BEFORE recording
// starts: wait 150 ms, and if the ADC no longer answers, power-cycle it and
// re-init (up to 3 times). Falls through if it never settles - the dead-stream
// recovery in loop() is still the safety net.
static void settleAdc(const char* where) {
  for (uint8_t t = 0; t < 3; t++) {
    delay(150);
    if (adcRegRead(0x73) >= 0) return; // still answering: settled
    char m[100];
    sprintf(m, "ADC stopped answering within 150 ms of init at %s - power-cycling (try %u of 3)",
            where, (unsigned)(t + 1));
    Serial.println(m);
    writeTelemetryLog("ADC_SETTLE_RETRY", m);
    acqStop();
    delay(300);
    queue.reset(); // capture is stopped here, so this is safe
    acqStart();
    haltIfAdcUnhealthy(where);
  }
}

#if TUFR_USB_OFF && defined(__IMXRT1062__)
extern "C" volatile uint8_t usb_configuration;
static void usbPowerDown() {
  if (Serial) return;                              // a terminal is open: keep USB for the user
  writeTelemetryLog("USB_OFF", TUFR_USB_OFF >= 2 ? "USB detached, PHY and USB PLL powered down" : "USB detached, PHY powered down");
  USB1_USBCMD &= ~USB_USBCMD_RS;                   // detach from the bus
  usb_configuration = 0;                           // core now treats Serial writes as "no host": they return at once
  USBPHY1_PWD = 0xFFFFFFFF;                        // power down all PHY blocks
  USBPHY1_CTRL_SET = USBPHY_CTRL_CLKGATE;
#if TUFR_USB_OFF >= 2
  CCM_ANALOG_PLL_USB1_CLR = CCM_ANALOG_PLL_USB1_EN_USB_CLKS | CCM_ANALOG_PLL_USB1_ENABLE | CCM_ANALOG_PLL_USB1_POWER;
#endif
}
#endif

// One-line check that the ADC still answers, used at start-up to find WHICH
// event (ADC init finishing, a 100 ms wait, opening the first file) kills it.
static void adcAliveReport(const char* label) {
  int a = adcRegRead(0x73), b = adcRegRead(0x76);
  char m[110];
  sprintf(m, "ADC check %s: 0x73=%d 0x76=%d (working: 192 192; -1 = no answer) t=%lums",
          label, a, b, (unsigned long)millis());
  Serial.println(m);
}

static void closeCurrentFile(bool deferLogs) {
  if (!outFile) return;
  flushAllDeferredLogs(); // lines still pending from the previous rotation (a stop within the first 1.5 s)
  gEndMs = ticksToMs(rtcTicks32k() - startTicks); // real recording length

  // Pre-allocation reserves more than was actually written; trim back to
  // the real size before patching the final header, same as production.
  uint64_t tC0 = rtcTicks32k();
  if (gFlac) {
    // last (shorter) frame and whatever is still staged
    tufr_flac_flush(&gFlacEnc, gFlacFrame, flacEmit, NULL);
    if (gFlacFill) { flacWriteStaged(gFlacFill); gFlacFill = 0; }
  }
#if TUFR_PREALLOCATE
  outFile.truncate((gFlac ? TUFR_FLAC_HEADER_BYTES : 44) + bytesWritten);
#endif
  uint64_t tC1 = rtcTicks32k();
  outFile.seek(0);
  if (gFlac) writeFlacHeader(outFile, &gFlacEnc);
  else writeWavHeader(outFile, bytesWritten);
  outFile.close();
  gTruncMs = ticksToMs(tC1 - tC0);
  gCloseMs = ticksToMs(rtcTicks32k() - tC1);
  {
    uint64_t used = ((uint64_t)bytesWritten + 512u + gClusterBytes - 1u) / gClusterBytes * gClusterBytes + gClusterBytes; // + log growth
    gFreeBytes = (gFreeBytes > used) ? gFreeBytes - used : 0;
  }
  if (gLastLiveMs > 10000) gPowerCycles = 0;   // 10 s of live audio in this file: healthy again

  uint32_t avgWriteUs = writeTimeCount ? (writeTimeSumUs / writeTimeCount) : 0;
  uint32_t minWriteUs = writeTimeCount ? writeTimeMinUs : 0;
  char statsMsg[192];
  float vinNow = readVin(); // under recording load: the most telling value for a weak battery
  sprintf(statsMsg, "Vin = %.2f, SDErrors=%lu, Writes=%lu, WriteUs(min/avg/max)=%lu/%lu/%lu, Bytes=%lu, acq_missed=%lu, liveUntilMs=%lu",
          vinNow,
          (unsigned long)sdWriteErrors, (unsigned long)writeTimeCount,
          (unsigned long)minWriteUs, (unsigned long)avgWriteUs, (unsigned long)writeTimeMaxUs,
          (unsigned long)bytesWritten, (unsigned long)acq_missed, (unsigned long)gLastLiveMs);
  Serial.print(gFilename); Serial.print(": "); Serial.println(statsMsg);
  if (deferLogs) logDeferred(gFilename, statsMsg); else writeTelemetryLog(gFilename, statsMsg);

  if (gFlac) {
    // Ratio = FLAC file size / the WAV file it replaces. EncCycPerSample and
    // EncCPU = encoder cost (SD write time excluded); Verbatim frames = blocks
    // that would not compress (noise near full scale: gain may be too high).
    uint64_t samples = gPcmBytes / 2u;
    double ratio = gPcmBytes ? 100.0 * (double)(bytesWritten + TUFR_FLAC_HEADER_BYTES) / (double)(gPcmBytes + 44) : 0.0;
    double cyc = samples ? (double)gFlacEncCycles / (double)samples : 0.0;
    double cpu = gFlacFileCycles ? 100.0 * (double)gFlacEncCycles / (double)gFlacFileCycles : 0.0;
    uint32_t nSub = gFlacEnc.n_constant + gFlacEnc.n_verbatim + gFlacEnc.n_fixed[0] + gFlacEnc.n_fixed[1] +
                    gFlacEnc.n_fixed[2] + gFlacEnc.n_fixed[3] + gFlacEnc.n_fixed[4];
    char flacMsg[230];
    snprintf(flacMsg, sizeof(flacMsg), "Ratio=%.1f%%, PCMBytes=%lu, Frames=%lu, EncCycPerSample=%.1f, EncCPU=%.0f%% at %lu MHz, subframes const=%lu verbatim=%lu fixed0-4=%lu/%lu/%lu/%lu/%lu of %lu",
            ratio, (unsigned long)gPcmBytes, (unsigned long)gFlacEnc.frame_number, cyc, cpu,
            (unsigned long)(F_CPU_ACTUAL / 1000000UL),
            (unsigned long)gFlacEnc.n_constant, (unsigned long)gFlacEnc.n_verbatim,
            (unsigned long)gFlacEnc.n_fixed[0], (unsigned long)gFlacEnc.n_fixed[1], (unsigned long)gFlacEnc.n_fixed[2],
            (unsigned long)gFlacEnc.n_fixed[3], (unsigned long)gFlacEnc.n_fixed[4], (unsigned long)nSub);
    Serial.print("  "); Serial.println(flacMsg);
    if (deferLogs) logDeferred("FLAC_STATS", flacMsg); else writeTelemetryLog("FLAC_STATS", flacMsg);
  }

  // Where did the misses fall? acq_missAt[] holds the DMA block number of the first
  // 16 misses; convert to time since this file was opened. "gap" = during the file
  // rotation (before this file was open), "early" = first 2 s, "late" = last 2 s,
  // "close" = after the recording ended (during this file's close), "mid" = in between.
  // One missed push drops a block, and on overflow Walter's
  // queue also discards what was queued, so each miss can be tens of ms of audio.
  if (acq_missed > 0) {
    uint32_t nLog = acq_missed < 16 ? acq_missed : 16;
    float blockMs = 1000.0f * (float)NBUF_I2S / ((float)gNchan * (float)gFsHz);
    long endMs = (long)gEndMs;   // actual recording length
    uint32_t nGap = 0, nEarly = 0, nMid = 0, nLate = 0, nClose = 0;
    long firstMs = 0, lastMs = 0;
    for (uint32_t i = 0; i < nLog; i++) {
      long ms = (long)((float)(int32_t)(acq_missAt[i] - gFileStartAcq) * blockMs);
      if (i == 0) firstMs = ms;
      lastMs = ms;
      if (ms < 0) nGap++;
      else if (ms < 2000) nEarly++;
      else if (ms >= endMs) nClose++;
      else if (ms >= endMs - 2000) nLate++;
      else nMid++;
    }
    char missMsg[240];
    sprintf(missMsg, "misses=%lu (first %lu located): gap=%lu early=%lu mid=%lu late=%lu close=%lu, first at %ldms, last at %ldms; this file open=%lums prealloc=%lums, closing it: truncate=%lums header+close=%lums",
            (unsigned long)acq_missed, (unsigned long)nLog,
            (unsigned long)nGap, (unsigned long)nEarly, (unsigned long)nMid, (unsigned long)nLate, (unsigned long)nClose,
            firstMs, lastMs, (unsigned long)gOpenMs, (unsigned long)gPreallocMs, (unsigned long)gTruncMs, (unsigned long)gCloseMs);
    Serial.print("  "); Serial.println(missMsg);
    if (deferLogs) logDeferred("MISS_DETAIL", missMsg); else writeTelemetryLog("MISS_DETAIL", missMsg);
  }

  // FIX (#1): acq_missed is Walter's own global counter (queueProcess.cpp)
  // and was never reset here, so every file's log line was showing the
  // cumulative total since boot, not that file's own drop count - e.g. the
  // "acq_missed=2" printed identically for files 0001 AND 0002 in an
  // earlier test wasn't 2 new misses in file 2, it was the same 2 misses
  // from file 0's startup still being reported. Reset it right after
  // logging so each file's number is really its own.
  acq_missed = 0;
}

// Visible-without-a-host diagnostics: LED_BUILTIN (pin 13, on-board - no
// wiring needed) blinks a distinct, repeating count for each halt reason,
// so a failure is diagnosable even with no Serial Monitor open. Recording
// itself also gets a fast heartbeat blink (see loop()) so "is it actually
// running" doesn't require a host connection either.
static void blinkCode(int code) {
  for (int ii = 0; ii < code; ii++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(300); // slowed 2x (2026-11-01) so the count is easy to read
    digitalWrite(LED_BUILTIN, LOW);  delay(300);
  }
}
// Code 4 (shutdown button) stops for good. Every other code is a fault: the
// code is shown three times, then the recorder power-cycles and tries again.
static void haltBlink(int code) {
  pinMode(LED_BUILTIN, OUTPUT);
  if (code == 4) {
    wdtConfigure(false);           // a deliberate stop: no watchdog restart
    while (1) { blinkCode(code); delay(2000); }
  }
  for (int r = 0; r < 3; r++) { wdtFeed(); blinkCode(code); wdtDelay(2000); }
  char why[48];
  snprintf(why, sizeof(why), "fault, LED code %d", code);
  powerCycleRecover(why);          // does not return
}

// Full power cycle (see "Unattended recovery"): keep the audio recorded so far,
// then hibernate, which switches off the Teensy's 3.3 V and so the whole ADC.
// Does not return.
static void powerCycleRecover(const char* why) {
  wdtFeed();
  gPowerCycles++;
  uint32_t secs = (gPowerCycles <= POWER_CYCLE_SHORT_TRIES) ? POWER_CYCLE_SHORT_S : POWER_CYCLE_LONG_S;
  char m[180];
  snprintf(m, sizeof(m), "%s - closing file, full power-off for %lus (power cycle %lu in a row)",
           why, (unsigned long)secs, (unsigned long)gPowerCycles);
  Serial.println(m);
  writeTelemetryLog("POWER_CYCLE", m);
  uint32_t cycles = gPowerCycles;
  closeCurrentFile(false);         // no-op if no file is open
  gPowerCycles = cycles;           // (closing a file with live audio would reset it)
  discardPreparedFile();
  wdtFeed();
  // No acqStop(): it talks to the ADC over I2C, which may be what is stuck.
  // hibernateAndRestart(.., false, ..) parks the ADC enable pins low and powers off.
  if (!hibernateAndRestart(secs, false, 2)) {
    // Marker file could not be written (SD trouble): power off anyway; the
    // wake is then an ordinary cold start.
    parkPinsForSleep();
    hibernateNow(secs);
  }
  while (1) {}                     // not reached (the watchdog would restart us)
}

// Stops everything and blinks 5 if the ADC didn't acknowledge on I2C or its
// channel-enable registers didn't read back as written (adcRuntimeHealthy()).
// Called after every acqStart(). Never returns when unhealthy.
static void haltIfAdcUnhealthy(const char* where) {
  if (adcRuntimeInitAttempts() > 1) {
    char rmsg[64];
    sprintf(rmsg, "ADC needed %u attempts at %s", (unsigned)adcRuntimeInitAttempts(), where);
    writeTelemetryLog("ADC_RETRY", rmsg);
  }
  if (adcRuntimeHealthy()) return;
  char msg[110];
  sprintf(msg, "ADC not responding correctly at %s (I2CErr=%u, config readback %s) - power cycle",
          where, (unsigned)adcRuntimeLastI2CError(), adcRuntimeConfigOk() ? "ok" : "MISMATCH");
  Serial.println(msg);
  writeTelemetryLog("ADC_FAULT", msg);
  // No acqStop() here: it talks to the ADC over I2C, which is what failed.
  // haltBlink() shows 5 blinks, then power-cycles everything (does not return).
  haltBlink(5);
}

// One bright LED flash (SCHED_FLASH_MS) that also watches the shutdown button, so
// holding the button through a heartbeat wake stops the board.
static void heartbeatFlash() {
  digitalWrite(LED_BUILTIN, HIGH);
  uint32_t t0 = millis();
  while ((millis() - t0) < SCHED_FLASH_MS) {
    if (digitalRead(PIN_SHUTDOWN) == LOW) {
      digitalWrite(LED_BUILTIN, LOW);
      writeTelemetryLog("HALT_USER", "Shutdown during scheduled-start wait");
      Serial.println("Done - shutdown button pressed during scheduled-start wait.");
      haltBlink(4);
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  // Reset status register, read before anything else: 0x1 = power-on type reset
  // (what a real hibernate wake or a USB plug-in should give), 0x2 = software
  // reset (e.g. the forced restart after an unexpected hibernate() return).
  uint32_t resetStatus = SRC_SRSR;
  SRC_SRSR = resetStatus; // clear the sticky reset-cause bits (write 1 to clear) so the next boot reads its own
  wdtConfigure(true);   // from here on, a hang anywhere restarts the chip
  // Restarted by the watchdog (SRC_SRSR bit 7 = RTWDOG, bit 4 = WDOG1/2): the
  // program had stopped responding, possibly inside an SD card or I2C call. A
  // watchdog restart leaves the ADC (IOVD) and the SD card powered, so power
  // everything off at once - before touching the card - and log it on the
  // next boot (recognised by the wake time, see WDT_WAKE_TAG).
  if (resetStatus & ((1u << 7) | (1u << 4))) {
    gHibTag = WDT_WAKE_TAG;              // the wake time itself marks this as a watchdog power-off
    parkPinsForSleep();
    hibernateNow(POWER_CYCLE_SHORT_S);   // 60-123 s; does not return; wakes without a marker
  }
  // LED usable from the very first instant, so a hang anywhere below still
  // leaves it in a known, meaningful state instead of just off/floating.
  pinMode(PIN_BATT, INPUT); // Vin divider (analog); readVin() does the rest
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  // Same pin/mode TUFR_4.0.ino uses - active-low button, checked in loop()
  // below to stop continuous recording cleanly (close current file first).
  pinMode(PIN_SHUTDOWN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(300); // SD card settle time before the first command (cold power-on)

  // The SD card comes up BEFORE the banner and the up-to-3 s wait for a Serial
  // monitor, because the hibernate-wake marker file lives on the card and must
  // be checked first - a wake restart must not burn ~3 s of full-power boot
  // waiting for a USB host that isn't there. A failed sd.begin() still halts
  // with 2 blinks, just after the banner.
  bool sdOk = sd.begin(SdioConfig(FIFO_SDIO));
  uint32_t hibSleepUnix = 0, hibWaitS = 0, hibFileIdx = 0, hibKind = 0;
  bool resumed = sdOk && readAndClearHibernateMarker(&hibSleepUnix, &hibWaitS, &hibFileIdx, &hibKind);
  bool schedWake = resumed && hibKind == 1; // heartbeat wake during a scheduled-start wait

  gColdBoot = !resumed;
  if (!resumed) {
    while (millis() < 3000) if (Serial) break;
    delay(200);
  }
  wdtFeed();
  Serial.println("\n*** TUFR " TUFR_VERSION " ***");
  // __DATE__/__TIME__ are filled in by the compiler, so they change on every
  // compile: if they are not what you just built, the board has old firmware.
  Serial.println("Compiled " __DATE__ " " __TIME__);
  Serial.print("Reset status SRC_SRSR = 0x"); Serial.println((unsigned long)resetStatus, HEX);
  printCrashReport(); // forensics IF last boot ended in a hard fault, not a true hang
  Serial.println("checkpoint: banner printed");
  if (!sdOk) {
    Serial.println("sd.begin FAILED - LED 2 blinks, then a power cycle (insert or try another SD card)");
    haltBlink(2);
  }
  Serial.println("checkpoint: sd.begin ok");
  if (resumed) Serial.println("checkpoint: HIBERNATE WAKE - continuing duty cycle (start-up waits skipped)");

#if RTC_HARDWARE_PRESENT
  // Hook TimeLib to the Teensy's on-chip RTC now that Serial is already up
  // and printing (so any problem here still leaves "banner printed"/
  // "sd.begin ok" on the serial monitor, instead of dead silence). rtc_get()
  // (rtcImpl.cpp) does the actual setSyncProvider() hookup lazily, the
  // first time it's called - this is that first call.
  Serial.println("checkpoint: about to sync RTC (setSyncProvider)");
  setSyncProvider(rtcSyncProvider); // TimeLib now follows the on-chip RTC (syncs immediately)
  setSyncInterval(1); // re-read the RTC every second: between syncs TimeLib counts with millis(),
                      // which can lag (see rtcTicks32k()), and the default 300 s interval made
                      // log/file timestamps lag and then jump forward
  Serial.println("checkpoint: RTC synced ok");
#else
  // No coin cell on VBAT (RTC_HARDWARE_PRESENT==0) - Teensy3Clock is never
  // touched at all (confirmed on this hardware: even just reading it
  // without a battery hard-faults the board and reboots it). Software
  // clock set directly to the fixed default instead.
  tmElements_t tm;
  tm.Year = RTC_DEFAULT_YEAR - 1970;
  tm.Month = RTC_DEFAULT_MONTH; tm.Day = RTC_DEFAULT_DAY;
  tm.Hour = RTC_DEFAULT_HOUR; tm.Minute = RTC_DEFAULT_MINUTE; tm.Second = RTC_DEFAULT_SECOND;
  setTime(makeTime(tm));
  // After a hibernate wake, continue from the time the marker recorded
  // (sleep time + wait) instead of restarting at the default time.
  if (resumed) setTime((time_t)(hibSleepUnix + hibWaitS));
  Serial.println("checkpoint: RTC_HARDWARE_PRESENT==0 - using default start time, Teensy3Clock not touched");
  if (!resumed) writeTelemetryLog("RTC_ABSENT", "RTC_HARDWARE_PRESENT==0, using default start time");
#endif

  // Correct SD file modify/create timestamps from here on.
  FsDateTime::setCallback(dateTime);

  readConfigFile(); // loads all deployment settings from TUFR_Setup.txt; also applies the
                     // RTC sync datetime if it is newer than the RTC (RTC_SYNC log event)

  if (resumed) {
#if !RTC_HARDWARE_PRESENT
    // readConfigFile()'s RTC sync handling just overwrote the software clock
    // with the config's sync time (it has no RTC to compare against) - put
    // the post-wake time back.
    setTime((time_t)(hibSleepUnix + hibWaitS));
#endif
    gFileIndex = hibFileIdx;
    // How long did the RTC say we were asleep? Should be about hibWaitS. (With
    // RTC_HARDWARE_PRESENT 0 the clock is just set from the marker, so this is
    // meaningless there.)
    long rtcElapsed = (long)now() - (long)hibSleepUnix;
    char wakeMsg[120];
    sprintf(wakeMsg, "Wake: asked for %lus, RTC says %lds elapsed, reset status 0x%lX, file index %lu",
            (unsigned long)hibWaitS, rtcElapsed, (unsigned long)resetStatus, (unsigned long)hibFileIdx);
    Serial.print("checkpoint: "); Serial.println(wakeMsg);
    if (!schedWake) writeTelemetryLog("HIBERNATE_WAKE", wakeMsg);
  }

  if (!schedWake) writeTelemetryLog("BOOT_VERSION", "TUFR " TUFR_VERSION);

  // The previous boot was restarted by the watchdog and powered off at once
  // (see the top of setup()): log it now that the card and clock are up.
  // Woken from a watchdog power-off? (no marker, alarm second carries the tag,
  // and the alarm was only moments ago)
  if (!resumed) {
    uint32_t lsb, msb;
    do { msb = SNVS_LPSRTCMR; lsb = SNVS_LPSRTCLR; }
    while ((SNVS_LPSRTCLR != lsb) | (SNVS_LPSRTCMR != msb));
    uint32_t rtcS = (msb << 17) | (lsb >> 15);
    uint32_t alarmS = SNVS_LPTAR;
    if ((alarmS & 63u) == WDT_WAKE_TAG && rtcS >= alarmS && rtcS - alarmS <= WDT_WAKE_WINDOW_S) {
      char wm[140];
      snprintf(wm, sizeof(wm), "Program had stopped responding: watchdog restart, then a full power-off; recording resumes (woke %lus after the alarm)",
               (unsigned long)(rtcS - alarmS));
      writeTelemetryLog("WATCHDOG_RESET", wm);
      gAfterWatchdog = true;
    }
  }
  char configLog[240];
  char fsTxt[40];
  if (queue_decimate) sprintf(fsTxt, "%lu(ADC %lu, decimated 2:1)", (unsigned long)gOutFsHz, (unsigned long)gFsHz);
  else sprintf(fsTxt, "%lu", (unsigned long)gFsHz);
  sprintf(configLog, "Ch:%u Fs:%sHz Gain:%d HPF:%s Filter:%s Duration:%lus Wait:%lus ClockMHz:%lu Format:%s Vin = %.2f",
          gNchan, fsTxt, gGainVal, adcRuntimeHpfInfo(), adcRuntimeFilterName(), (unsigned long)gDurationS,
          (unsigned long)gWaitS, (unsigned long)gClockMHz, gFlac ? "FLAC" : "WAV", readVin());
  if (!schedWake) writeTelemetryLog("CONFIG", configLog);

  // ---- Scheduled start ----
  // Compared against TimeLib's now(), which readConfigFile() has just synced
  // from the RTC / setup file. A 15 s guard band skips the wait loop for a start
  // time that is already here or in the very recent past.
  // First boot: wait if the start is more than 15 s away. After a heartbeat wake
  // (schedWake): keep waiting until the start time itself, so recording starts on time.
  if (gScheduledStartUnix > 0 &&
      ((!resumed && gScheduledStartUnix > now() + 15) ||
       (schedWake && gScheduledStartUnix > now()))) {
    if (!resumed) {
      writeTelemetryLog("SCHEDULED_WAIT_START", "Waiting for scheduled start time");
      Serial.print("checkpoint: scheduled start in the future - waiting until ");
      Serial.println((unsigned long)gScheduledStartUnix);
    }
    heartbeatFlash(); // bright flash at the start of the wait and on every wake
    uint32_t remaining = (uint32_t)(gScheduledStartUnix - now());
    if (remaining > SCHED_INPLACE_S) {
      // Hibernate (about 1 mA) until about 20 s before the start time, at most
      // SCHED_HEARTBEAT_S at a time; the next wake flashes and goes back to sleep.
      uint32_t chunk = remaining - 20;
      if (chunk > SCHED_HEARTBEAT_S) chunk = SCHED_HEARTBEAT_S;
      hibernateAndRestart(chunk, false, 1); // does not return unless the marker can't be written
    }
    // Close to the start time (or no marker file): wait out the rest in place.
    set_arm_clock(WAIT_CLOCK_FREQ);
    delay(10);
    unsigned long lastBlink = millis();
    while (now() < gScheduledStartUnix) {
      wdtFeed();
      if (digitalRead(PIN_SHUTDOWN) == LOW) {
        writeTelemetryLog("HALT_USER", "Shutdown during scheduled-start wait");
        Serial.println("Done - shutdown button pressed during scheduled-start wait.");
        haltBlink(4);
      }
      if ((millis() - lastBlink) > 10000) {
        digitalWrite(LED_BUILTIN, HIGH); delay(10); digitalWrite(LED_BUILTIN, LOW);
        lastBlink = millis();
      }
      delay(100);
    }
    if (gClockMHz > 0) set_arm_clock(gClockMHz * 1000000UL);
    delay(10);
    writeTelemetryLog("SCHEDULED_WAIT_END", "Scheduled start time reached");
    Serial.println("checkpoint: scheduled start time reached - proceeding");
  } else if (gClockMHz > 0) {
    // No scheduled-start wait: apply the configured clock speed now, before
    // acqInit().
    set_arm_clock(gClockMHz * 1000000UL);
    delay(10);
  }

  // ---- Daily recording window ----
  // Checked on every boot - cold power-on, scheduled-start arrival, and every
  // hibernate wake - BEFORE the ADC/capture chain is started, so waking
  // outside the window costs only a few seconds of boot, not an ADC power-up.
  if (gUseDailySchedule && !isWithinDailySchedule()) {
    if (!dailySleepUntilWindowOpens(false)) {
      Serial.println("Done - shutdown button pressed outside the daily window.");
      haltBlink(4);
    }
  }

  // Free space on the card, measured once per boot and counted down as files
  // close (scanning the allocation table every file would cost time at the joins).
  wdtFeed();
  gClusterBytes = sd.bytesPerCluster();
  if (gClusterBytes == 0) gClusterBytes = 32768;
  gFreeBytes = (uint64_t)sd.freeClusterCount() * gClusterBytes;
  wdtFeed();
  {
    char sm[110];
    snprintf(sm, sizeof(sm), "%lu MB free, cluster %lu bytes, room for at least %lu files of %lu s (WAV size)",
             (unsigned long)(gFreeBytes >> 20), (unsigned long)gClusterBytes,
             (unsigned long)(gFreeBytes / fileNeedBytes()), (unsigned long)gDurationS);
    writeTelemetryLog("SD_SPACE", sm);
  }
  if (gFreeBytes < fileNeedBytes()) cardFull();

  queue.reset();
  process_init();
  Serial.println("checkpoint: queue reset");

  // Same order as Walter's own acqSetup(): BCLK/FSYNC running (acqInit) before
  // the ADC is powered up and configured (acqStart). The ADC has no MCLK pin and
  // derives its clocks from BCLK, so BCLK must exist before its PLL initializes.
  // The BCLK:FSYNC ratio (ovr) is computed from the channel count in Teensy.cpp.
  if (gFsHz > FSAMP) gFsHz = FSAMP;
  fsamp = gFsHz; // kept in sync for anything else that reads the extern fsamp global
  Serial.print("checkpoint: requesting fs="); Serial.print(gFsHz);
  Serial.print("Hz, nchan="); Serial.print(gNchan);
  Serial.print(", gain="); Serial.println(gGainVal);
  Serial.println("checkpoint: acqInit() (audio PLL lock wait - if nothing prints after this, it is stuck there)");
  digitalWrite(LED_BUILTIN, HIGH); // solid ON = "inside acqInit() / acqStart()"
  wdtFeed();
  acqInit(gFsHz, gNchan, gGainVal);
  Serial.println("checkpoint: acqInit() returned");
  acqStart();
  Serial.println("checkpoint: acqStart() returned");
  haltIfAdcUnhealthy("start-up"); // does not return if the ADC isn't responding
  if (!resumed) {
    writeTelemetryLog("HPF_LOAD", adcRuntimeHpfInfo()); // how the high-pass setup went
    if (adcRuntimeHpfDiag()[0]) writeTelemetryLog("HPF_DIAG", adcRuntimeHpfDiag());
    { char hp[240]; adcRuntimeHpfDump(hp, sizeof(hp)); writeTelemetryLog("HPF_POST", hp); } // registers after the ADC is running
  }
  digitalWrite(LED_BUILTIN, LOW);
  adcAliveReport("right after init");
  settleAdc("start-up");   // re-does the ADC power-up if it dies within 150 ms of init
  adcAliveReport("after settle    ");
  // Discard whatever piled up in the queue during init/settle (not recording
  // yet); pull() is interrupt-safe, unlike queue.reset() while capture runs.
  while (queue.peek()) queue.release();
  wdtFeed();

  Serial.print("Recording continuously in back-to-back "); Serial.print(gDurationS);
  Serial.print("s files at "); Serial.print(gOutFsHz);
  Serial.print("Hz, "); Serial.print(gNchan);
  Serial.println("ch until the shutdown button is pressed...");

  // Capture (acqInit/acqStart above) is already running continuously;
  // startNewFile() just opens the first destination file for it. Only the
  // file rotates on the gDurationS timer in loop() below - capture itself
  // is never stopped/restarted between files, matching production.
  startNewFile();
  adcAliveReport("after 1st file opened");
#if TUFR_USB_OFF && defined(__IMXRT1062__)
  usbPowerDown();
#endif
}

void loop() {
  wdtFeed();
#if TUFR_WDT_TEST
  if (gColdBoot && !gAfterWatchdog && gFileIndex == 0 && !doneFlag && ticksToMs(rtcTicks32k() - startTicks) > 30000 &&
      !sd.exists(TUFR_WDT_TEST_DONE_FILE)) {
    FsFile d = sd.open(TUFR_WDT_TEST_DONE_FILE, O_WRONLY | O_CREAT | O_TRUNC);
    if (d) { d.println("WDT test done; delete this file to run it again"); d.close(); }
    writeTelemetryLog("WDT_TEST", "TUFR_WDT_TEST: stopped feeding the watchdog - expect a restart in about 10 s, then 60 s off");
    while (1) {}
  }
#endif
  if (doneFlag) {
    // Steady-on LED = stopped cleanly, current file closed.
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    return;
  }

  // Checked first, same as TUFR_4.0.ino's loop() - close the file currently
  // being written (patches its real header, truncates the preallocated
  // slack) before halting, so a button press never leaves a corrupt/
  // oversized file behind.
  if (digitalRead(PIN_SHUTDOWN) == LOW) {
    closeCurrentFile(false);
    discardPreparedFile();
    acqStop();
    doneFlag = true;
    writeTelemetryLog("HALT_USER", "Shutdown button pressed");
    Serial.print("Done - stopped by shutdown button after ");
    Serial.print(gFileIndex + 1); Serial.println(" file(s).");
    return;
  }

  // Fast heartbeat while actively recording - visible proof it's running
  // even with no computer/Serial Monitor attached.
  digitalWrite(LED_BUILTIN, (millis() / 100) % 2);

  int16_t *blk = queue.peek();     // oldest finished block, read in place (no copy)
  if (blk) {
    size_t nBytes = NBUF_DISK * sizeof(int16_t);
    { // cheap: stops scanning at the first live sample of each block
      bool live = false;
      for (uint32_t i = 0; i < NBUF_DISK; i++) {
        if (blk[i] != 0) { live = true; break; }
      }
      for (int i = 0; i < 4; i++) gLastWords[i] = blk[i];
      if (live) {
        gZeroRunStartMs = 0;
        gLastLiveMs = (uint32_t)(((rtcTicks32k() - startTicks) * 1000ULL) >> 15);
      } else if (gZeroRunStartMs == 0) {
        gZeroRunStartMs = millis() ? millis() : 1;
      }
    }

    if (gFlac) {
      flacWriteBlock(blk, NBUF_DISK);   // encodes; writes 32 KB to the card whenever that much is ready
      queue.release();
    } else {
    uint32_t t0 = micros();
    size_t w = outFile.write((uint8_t*)blk, nBytes);
    uint32_t dt = micros() - t0;
    if (dt < writeTimeMinUs) writeTimeMinUs = dt;
    if (dt > writeTimeMaxUs) writeTimeMaxUs = dt;
    writeTimeSumUs += dt;
    writeTimeCount++;
    queue.release();

    if (w == nBytes) { bytesWritten += nBytes; gSdErrRun = 0; }
    else { sdWriteErrors++; if (gSdErrRun < 255) gSdErrRun++; }
    }
    if (gSdErrRun >= SD_WRITE_FAIL_RUN) {
      char m[100];
      snprintf(m, sizeof(m), "%u SD writes in a row failed (%lu in this file) - insert or try another SD card",
               (unsigned)gSdErrRun, (unsigned long)sdWriteErrors);
      writeTelemetryLog("SD_WRITE_FAILED", m);
      gSdErrRun = 0;
      haltBlink(2);                     // 2 blinks x3, then a power cycle (does not return)
    }
  }

  // Continuous mode: open and preallocate the next file ahead of the join.
  if (gWaitS == 0 && !doneFlag) prepareNextFileSteps();

  // Deferred rotation log lines: one SD write per pass, once the new file has been
  // running 1.5 s (queue empty, so a ~30 ms log write can't overflow it).
  if (gPendN > 0 && ticksToMs(rtcTicks32k() - startTicks) > 1500) flushOneDeferredLog();

  // Dead-stream check: DEAD_STREAM_RECOVER_MS of all-zero audio means the data
  // path is dead even if the chip still answers on I2C (any live input, even a
  // terminated one, produces some noise). See "ADC / data-stream protection".
  if (gZeroRunStartMs != 0 && (millis() - gZeroRunStartMs) >= DEAD_STREAM_RECOVER_MS) {
    // Look at the hardware BEFORE anything touches it.
    dumpDeadStreamDiagnostics();
    if (gRecoveries < DEAD_STREAM_INPLACE_TRIES) {
      recoverDeadStream(); // power-cycle the ADC and restart capture; the file stays open
    } else {
      writeTelemetryLog("NO_DATA", "Recording still all zeros after in-place ADC restarts");
      powerCycleRecover("dead stream: in-place restarts did not help"); // does not return
    }
  }

  if (!doneFlag && (rtcTicks32k() - startTicks) >= (uint64_t)gDurationS * 32768ULL) {
    // Continuous mode with the next file opening straight away: defer this file's log
    // lines so the SD writes don't lengthen the gap (see logDeferred()).
    bool nextFileNow = (gWaitS == 0) && !(gUseDailySchedule && !isWithinDailySchedule());
    closeCurrentFile(nextFileNow);
    if (!nextFileNow) discardPreparedFile();   // duty-cycle wait or daily window closed: no continuous next file
    gFileIndex++;
    if (gUseDailySchedule && !isWithinDailySchedule()) {
      // The window closed during this file (a file is never cut short, so
      // recording can run up to one file length past the stop time). Sleep
      // until it opens again; this replaces the duty-cycle wait.
      if (!dailySleepUntilWindowOpens(true)) {
        doneFlag = true;
        Serial.print("Done - stopped by shutdown button at the end of the daily window after ");
        Serial.print(gFileIndex); Serial.println(" file(s).");
      }
      return;
    }
    if (gWaitS > 0) {
      // Duty-cycle mode: power down between recordings instead of rotating
      // straight to the next file.
      if (enterDutyCycleWait()) {
        writeTelemetryLog("RESUME_STARTNEWFILE", "About to call startNewFile()");
        startNewFile();
        writeTelemetryLog("RESUME_STARTNEWFILE_DONE", "startNewFile() returned");
      } else {
        doneFlag = true;
        Serial.print("Done - stopped by shutdown button during duty-cycle wait after ");
        Serial.print(gFileIndex); Serial.println(" file(s).");
      }
    } else {
      startNewFile(); // continuous mode (gWaitS==0) - next file opens immediately,
                       // capture never paused
    }
  }
}
