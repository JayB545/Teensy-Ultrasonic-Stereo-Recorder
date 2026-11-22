/* microPAM
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
 *
 * =====================================================================
 * THIS FILE IS A MODIFIED COPY, for TUFR's "run Walter's actual SAI/DMA/
 * ADC driver, unmodified, at our hardest failing config" ground-truth
 * sketch (TUFR 4.1). It is NOT the original global.h - see the three
 * change-markers below ("TUFR TEST CHANGE") for the only edits made
 * to Walter's original file. Everything else is copied verbatim.
 * =====================================================================
 */

#ifndef GLOBAL_H
#define GLOBAL_H

  // TUFR TEST CHANGE (1 of 3): Walter's original line here is
  //   #include "../config.h"
  // - a PlatformIO project file (his own per-deployment settings) that
  // isn't part of what he shared with us. The only two things later in
  // this file that actually come from it are FSAMP and PROC_MODE (plus
  // NCHAN_ADC, used only to define an unused ICH macro). Inlined directly
  // here instead of reconstructing his config.h:
  // TASK 2 (Jay's list, 2026-10-29): raised from 384000 to 768000 to allow
  // 768kHz on a single channel. This is a safe extension of the fix noted
  // just below (FSAMP decoupled from the compile-time NCHAN_I2S macro) -
  // clampSampleRate() in TUFR_4.1.ino is what actually enforces the
  // real, channel-count-correct ceiling at runtime (768kHz for 1ch only,
  // 384kHz for <=2ch, 192kHz otherwise); this is just the absolute
  // hardware ceiling nothing may exceed regardless of channel count.
  // Electrically this isn't new territory: BCLK = fs*nchan*MBIT, so
  // 768kHz/1ch (ovr=32) produces the exact same ~24.576MHz BCLK as the
  // already-tested-clean 384kHz/2ch (ovr=64) and 192kHz/4ch (ovr=128)
  // configs - same bit clock rate, same PLL math (set_audioClock()'s n2
  // depends only on fs*ovr, which is identical across all three).
  #define FSAMP     768000   // ceiling; NCHAN_I2S==2 below re-clamps to this exact value anyway
  #define PROC_MODE 0        // raw passthrough: queue.push(acq_buffer) with no compression/DSP - see process.cxx
  #define NCHAN_ADC 4        // physical channel count the ADC6140 exposes; only gates an unused ICH define

  #define Program "microPAM_V4"
  #define Version "4.3.0" // 28-06-2026

  #define T_4_1   0
  #define RP_2040 1
  #define RP_2350 2

  #if defined __IMXRT1062__
    #define MCU T_4_1
  #elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER)
    #define MCU RP_2040
  #elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2350_HSTX)
    #define MCU RP_2350
  #endif

  #define NBUF 1024

  // for acq

  // for acq/adc hardware
  #if MCU==T_4_1
    #define NPORT_I2S   1
    // TUFR TEST CHANGE (2 of 3): Walter's original line here is
    //   #define NCHAN_I2S   4
    // TEMPORARILY SET BACK TO 4 to verify the ovr=NCHAN_I2S*MBIT fix
    // (Teensy.cpp's setAudioFrequency()) generalizes correctly beyond the
    // 2ch case already tested clean - this is Walter's own original
    // shipped default, so it doubles as a regression check that the fix
    // didn't disturb his proven-working config (ovr should evaluate to
    // the same 128 it was hardcoded to before, byte-for-byte).
    #define NCHAN_I2S   4
    #define NCHAN_ACQ   4
    #define NBUF_I2S    (2*NBUF)
    // TUFR TEST CHANGE (3 of 3): Walter's original lines here are
    //   #define ADC_SHDNZ   3
    //   #define ADC_EN      2
    // - his own board's pin choice, not TUFR's. Changed to match
    // TUFR_Config.h's actual wiring (PIN_ADC_SHDNZ=6, PIN_ADC_EN=9) so
    // this test runs on the existing TUFR board with no jumper changes.
    // Important: leaving this at Walter's original pin 2 would have been
    // a real conflict, not just a mismatch - TUFR's PIN_SHUTDOWN is ALSO
    // pin 2 (the shutdown-button input), so adc_init()/adc_exit() driving
    // ADC_EN as a digital output would have driven straight onto whatever
    // the shutdown button/circuit has wired to that same pin.
    #define ADC_SHDNZ   6
    #define ADC_EN      9
    #define mWire       Wire
    #define USB_POWER   1
    #define MAX_QUEUE   14   // 16-bit blocks of NBUF_DISK samples, 32 KB (~21 ms) each: 448 KB of RAM2, ~300 ms of audio
    #define USE_EXT_RTC   0
    #if PROC_MODE==3 || PROC_MODE==4
      #define NAVG      1 // could change this
      #define NCHAN_PROC 3
      #define NBUF_PROC (NCHAN_PROC*NBUF_I2S/NCHAN_I2S)
    #else
      #define NAVG      1
      #define NCHAN_PROC NCHAN_ACQ
      #define NBUF_PROC  NBUF_I2S
    #endif

  #elif MCU==RP_2040
    #define NPORT_I2S   1
    #define NCHAN_I2S   1
    #define NCHAN_ACQ   1
    #define NCHAN_PROC  1
    #define NBUF_I2S    NBUF
    #define NBUF_PROC   NBUF
    #define ADC_EN      5
    #define ADC_SHDNZ   6
    #define mWire       Wire
    #define MAX_QUEUE     5
    #define USE_EXT_RTC   1
    #define XRTC_INT_PIN  15
    #define SD_CS         23

  #elif MCU==RP_2350
    #define NPORT_I2S   1
    #define NCHAN_I2S   4
    #define NCHAN_ACQ   4
    #define NCHAN_PROC  4
    #define NBUF_I2S    NBUF
    #define NBUF_PROC   NBUF
    #define ADC_EN      5
    #define ADC_SHDNZ   6
    #define mWire       Wire
    #define MAX_QUEUE   225
    #define USE_EXT_RTC   1
    #define XRTC_INT_PIN  29
    #define SD_CS         25
  #endif

  // limit processing mode for other than Teensy4.1
  #if MCU != T_4_1
    #if PROC_MODE==2 || PROC_MODE==3 || PROC_MODE==4
      #undef PROC_MODE
      #define PROC_MODE 1
    #endif
    #define NAVG        1
    #define USB_POWER   0
  #endif

  // limit sampling frequency of ADC
  //
  // BUG FOUND (TUFR ground-truth testing): this block ties the compile-time
  // FSAMP ceiling to the compile-time NCHAN_I2S macro - fine in Walter's
  // original, where channel count WAS a compile-time choice, but TUFR now
  // picks channel count at runtime from TUFR_Setup.txt (see gRxNchan in
  // Teensy.cpp and readConfigFile() in TUFR_4.1.ino), so NCHAN_I2S
  // here is just whatever channel count happened to be compiled in last -
  // currently 4, left over from an earlier regression test. With the old
  // code, that silently capped FSAMP at 192000 regardless of the ACTUAL
  // runtime channel count, so a config-file request for 2ch/384kHz got
  // knocked down to 192kHz by TUFR_4.1.ino's `if (gFsHz > FSAMP)
  // gFsHz = FSAMP;` line before acqInit() ever saw it - a silent, clean
  // 2:1 rate error, exactly like the ovr bug, but caused by leftover
  // build config rather than a driver bug. (Confirmed: the "384kHz"
  // recording that came back was a clean, correctly-pitched 192kHz file -
  // not corruption, not a wiring/trace-length issue.)
  //
  // Fixed by decoupling: FSAMP's compile-time ceiling is now just the
  // hardware's absolute max (768kHz, raised from 384kHz for Task 2 - see
  // the #define above) regardless of NCHAN_I2S. The REAL, channel-count-
  // correct limit (768kHz for 1ch only, 384kHz for <=2ch, 192kHz
  // otherwise) is already enforced properly at runtime by
  // clampSampleRate() in TUFR_4.1.ino, which takes the actual
  // runtime nchan - this compile-time value should never again be the
  // thing doing that job. (This clamp itself must track the #define above
  // - it silently re-capped FSAMP back down to 384000 the first time Task
  // 2 raised the #define without updating this block too, reproducing
  // exactly the kind of silent rate-ceiling bug this whole section exists
  // to prevent - caught before it shipped.)
  #if FSAMP>768000
    #undef FSAMP
    #define FSAMP 768000
  #endif
  #if NCHAN_I2S==2
    #undef  NCHAN_ACQ
    #define NCHAN_ACQ 2
  #endif
  //
  #if MCU==RP_2040
    #if FSAMP>192000
      #undef FSAMP
      #define FSAMP 192000
    #endif
  #endif

  // for adc
  #define AGAIN 20
  #define DGAIN 0
  #define PGAIN 0

  // Detection parameters (PROC_MODE 2, Teensy 4.1 only)
  #ifndef DETECT_ALPHA
    #define DETECT_ALPHA 0.01f
  #endif
  #ifndef DETECT_THR
    #define DETECT_THR   3.0f
  #endif

  // for processing
  #define SHIFT 12
  #if NCHAN_ADC < NCHAN_I2S
    #define ICH 0
  #endif

  // for filing (and queue)
  #define NBUF_DISK (8*NBUF_I2S)

  // for XRTC
  #define XRTC_SDA   2
  #define XRTC_SCL   3

  // program states
  enum status_t  {DO_START, CLOSED, RECORDING, MUST_STOP, JUST_STOPPED, STOPPED, MUST_HIBERNATE};
  extern char status_text[][16];

  // setting MCU clock speed according to sampling frequency
  #if FSAMP<=96000
    #define CLK_MULT 4  // (48 MHz)
  #elif FSAMP<=192000
    #define CLK_MULT 8  // (96 MHz)
  #else
    #define CLK_MULT 12 // (144 MHz)
  #endif

  #include <cstdint>

  // in rp2040.cxx/Teensy.cxx
  extern volatile uint32_t fsamp;  // sampling frequency (kHz) needed inter alia for filing

  #define MBIT 32

  #if MCU==T_4_1
    #define __not_in_flash_func(func) func
  #endif

  #define MONITOR 1
#endif
