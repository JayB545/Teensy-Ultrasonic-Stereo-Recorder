/*
 * adcRuntime.cpp
 *
 * Field-configurable (1-4 channel) initialization for the WMXZ-EU ADC_V2
 * module (TLV320ADC6140), for the TUFR ultrasonic field recorder.
 *
 * =====================================================================
 * WHAT THIS FILE IS, AND HOW IT RELATES TO WALTER ZIMMER'S adc.cxx
 * =====================================================================
 * TUFR ships Walter Zimmer's adc.cxx/adc.h unmodified, alongside this
 * file. adc.cxx's own adc_init() picks the ADC's active channel count
 * (1, 2 or 4) via the NCHAN_I2S preprocessor macro, decided when the
 * firmware is COMPILED. TUFR needs channel count (1, 2, 3 or 4) to be
 * chosen from TUFR_Setup.txt on the SD card at BOOT time, in the field,
 * without recompiling - something a preprocessor macro cannot do.
 *
 * This file reimplements just the channel/slot-configuration portion of
 * adc_init() - the per-channel-count chanMask/chmap tables and the I2C
 * register writes they drive - as a function of a runtime `nchan`
 * argument, using the same register addresses, bit fields and default
 * values as adc.cxx. It does not call adc.cxx's adc_init()/adc_exit()
 * (which cannot take a runtime channel count, and which also call
 * usbPowerSetup()/usbPowerOff()/usbPowerExit() from Walter's own
 * Teensy.cxx - functions TUFR does not use, since it manages USB power
 * itself). adc.cxx's `again` variable and setAGain()/adcStatus() helpers,
 * which are not tied to channel count, ARE still used as-is from adc.h.
 *
 * One deliberate difference from adc.cxx's own channel choices: for 1
 * and 2 channels, adc.cxx enables physical inputs IN2 (mono) or IN2+IN3
 * (stereo) - not IN1 or IN1+IN2. That was Walter's choice for his own
 * board/use case, for reasons we didn't ask him about (channel-noise
 * differences between the ADC's input pins, most likely, but unconfirmed
 * - he's being asked). For TUFR, Jay Barlow chose the simpler, more
 * intuitive field convention instead: channels are always the FIRST
 * `nchan` physical inputs, in order (1ch = IN1; 2ch = IN1+IN2; 3ch =
 * IN1+IN2+IN3; 4ch = IN1-IN4). If Walter's reason turns out to matter
 * for signal quality, this mapping is the one place that needs to change.
 *
 * =====================================================================
 * ORIGINAL adc.cxx LICENSE (reproduced per its terms, since this file is
 * a derivative work of it)
 * =====================================================================
 * microPAM
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
#include <Wire.h>
#include <math.h>
#include "adcRuntime.h"
#include "TUFR_Config.h"
// Note: adc.h/adc.cxx are shipped in this project folder unmodified, for
// reference and attribution, but are NOT compiled into the firmware - see
// the file-header comment above for why. Every register write below is a
// direct, faithful reproduction of the corresponding write in adc.cxx.

// ---- I2C address / register map (identical to adc.cxx) ----
// ADDRESS L,L: 0x4C ; H,L: 0x4D; L,H: 0x4E; H,H: 0x4F  (TUFR uses a single
// ADC_V2 board strapped to 0x4C, matching NPORT_I2S==1 in global.h)
static const uint8_t ADCRT_I2C_ADDRESS = 0x4C;
static const uint8_t ADCRT_REGS[4] = {0x3C, 0x41, 0x46, 0x4B}; // CH1..CH4 cfg blocks

static uint8_t gLastI2CError = 0;
uint8_t adcRuntimeLastI2CError() { return gLastI2CError; }

// --- ASI_STS (0x15) poll-for-lock diagnostic - see adcRuntime.h for the
// full rationale. Kept as plain static state (not local to
// adcRuntimeInit()) so the getters below can report it after the fact. ---
static const uint8_t ADCRT_ASI_STS_REG = 0x15;
static const uint8_t ADCRT_ASI_STS_GOOD_MASK = 0x30; // thin (3-point) correlation, not a confirmed datasheet bit field
static const uint8_t ADCRT_ASI_STS_MAX_ATTEMPTS = 20; // 20 * 5ms = 100ms cap
static const uint8_t ADCRT_ASI_STS_RETRY_DELAY_MS = 5;

// Config readback check (added 2026-11-01 after a PCB with a cold ground joint
// recorded silent files without any warning): after the channel setup,
// IN_CH_EN (0x73) and ASI_OUT_CH_EN (0x74) are read back and must equal the
// chanMask that was written. A chip that isn't really responding (no power,
// bad ground, stuck bus) reads back 0 and fails. adcrtWrite() results are
// otherwise discarded, so this is the only check that the writes took.
static bool gConfigOk = false;
bool adcRuntimeConfigOk() { return gConfigOk; }
bool adcRuntimeHealthy() { return gLastI2CError == 0 && gConfigOk; }

static uint8_t gAsiStsAttempts = 0;   // 0 == pattern never seen within the cap
static uint8_t gAsiStsFinalValue = 0; // raw register value on the last attempt, always valid

uint8_t adcRuntimeAsiStsAttempts() { return gAsiStsAttempts; }
uint8_t adcRuntimeAsiStsFinalValue() { return gAsiStsFinalValue; }

// Minimal local I2C helpers. adc.cxx's own i2c_class is private to that
// file (not exposed via adc.h), so it can't be reused directly; these are
// straight equivalents of its read()/write() methods.
static bool adcrtWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static uint8_t adcrtRead(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom((int)addr, 1) < 1) return 0;
  return Wire.read();
}

// ---- High-pass filter (DSP_CFG0 = P0_R107 = 0x6B, HPF_SEL bits 1:0) ----
// HPF_SEL 1/2/3 = fixed first-order presets at 0.00025 / 0.002 / 0.008 x fS.
// HPF_SEL 0 = programmable first-order IIR whose three 32-bit coefficients live
// in Page 4, registers 0x48-0x53 (N0, N1, D1, each big-endian byte 1..4, reset
// values = all-pass: N0 = 0x7FFFFFFF, N1 = D1 = 0). The datasheet says coefficients
// must be written before the channel is powered up, so adcRuntimeInitOnce() loads
// them before the channel-enable (0x73) and power-up (0x75) writes.
//
// ASSUMED transfer function (not confirmed - the datasheet text available here
// has no worked example):  H(z) = (N0 + N1 z^-1) / (2^31 - D1 z^-1), coefficients
// 32-bit two's complement, with N0 = (1+a)/2, N1 = -N0, D1 = a, a = exp(-2 pi fc/fS),
// each scaled by 2^31. If the sign convention of D1 is the other way round the
// recording will be saturated noise (gain of about 1/(1-a) at high frequencies);
// in that case set TUFR_HPF_D1_SIGN to -1 in TUFR_Config.h.
static uint8_t gDeciFilt = 2;        // DECI_FILT (0x6B bits 5:4): 0 linear phase, 1 low latency, 2 ultra-low latency
static uint8_t gHpfSel = 1;          // HPF_SEL value written to 0x6B bits 1:0
static bool    gHpfLoadCoef = false; // true when HPF_SEL = 0 (coefficients must be written)
static bool    gHpfOff = false;      // setup value 0: leave the reset (all-pass) coefficients alone
static char    gHpfDiag[240] = "";   // readbacks from the last coefficient load, for the log
static uint8_t gHpfCoef[12] = {0x7F,0xFF,0xFF,0xFF, 0,0,0,0, 0,0,0,0}; // N0, N1, D1 (all-pass)
static char    gHpfBase[48] = "preset 1 (0.00025 x fS)"; // what was asked for
static char    gHpfInfo[112] = "preset 1 (0.00025 x fS)"; // ... plus how the coefficient load went

static void hpfPut32(uint8_t* d, double x) {
  double v = floor(x * 2147483648.0 + 0.5);
  if (v > 2147483647.0) v = 2147483647.0;
  if (v < -2147483648.0) v = -2147483648.0;
  uint32_t u = (uint32_t)(int32_t)v;
  d[0] = (uint8_t)(u >> 24); d[1] = (uint8_t)(u >> 16); d[2] = (uint8_t)(u >> 8); d[3] = (uint8_t)u;
}

static void hpfSelect(int32_t hpfHz, uint32_t fsHz) {
  gHpfLoadCoef = false;
  gHpfOff = false;
  if (hpfHz < 0 || fsHz == 0) {
    // -1, -2, -3 = the ADC's fixed presets 1, 2, 3 (0.00025, 0.002, 0.008 x fS)
    static const double PRESET[3] = {0.00025, 0.002, 0.008};
    int k = (hpfHz <= -3) ? 3 : (hpfHz == -2 ? 2 : 1);
    gHpfSel = (uint8_t)k;
    snprintf(gHpfBase, sizeof(gHpfBase), "preset %d (%.0f Hz)", k, PRESET[k - 1] * (double)fsHz);
    return;
  }
  gHpfSel = 0;
  gHpfLoadCoef = true;
  gHpfOff = (hpfHz == 0);
  const uint8_t allpass[12] = {0x7F,0xFF,0xFF,0xFF, 0,0,0,0, 0,0,0,0};
  memcpy(gHpfCoef, allpass, 12);
  if (hpfHz == 0) { snprintf(gHpfBase, sizeof(gHpfBase), "off"); return; }

  double fc = (double)hpfHz;
  double fmax = (double)fsHz / 10.0;
  if (fc < 1.0) fc = 1.0;
  if (fc > fmax) fc = fmax;
  double a = exp(-2.0 * 3.14159265358979323846 * fc / (double)fsHz);
  double n0 = (1.0 + a) / 2.0;
  hpfPut32(&gHpfCoef[0], n0);
  hpfPut32(&gHpfCoef[4], -n0);
  hpfPut32(&gHpfCoef[8], (double)TUFR_HPF_D1_SIGN * a);
  snprintf(gHpfBase, sizeof(gHpfBase), "custom %.0f Hz", fc);
}
void adcRuntimeSetHpf(int32_t hpfHz, uint32_t fsHz) {
  hpfSelect(hpfHz, fsHz);
  strncpy(gHpfInfo, gHpfBase, sizeof(gHpfInfo) - 1);
  gHpfInfo[sizeof(gHpfInfo) - 1] = '\0';
}

// ---- Loading the programmable coefficients ----
// Bench finding 2026-11-05 (build 05d): writing the 12 coefficient bytes as 12
// separate single-byte I2C transactions left only the LAST byte of each 32-bit
// coefficient in the ADC (read back N0=0000006B for 7F7A816B; even the all-pass
// 7FFFFFFF became 000000FF), so the filter passed nothing and the stream was all
// zeros. Each coefficient apparently has to be written as one multi-byte
// transaction. Now: one 12-byte burst (auto-increment), read back and compared;
// if that doesn't stick, three 4-byte bursts; if that doesn't either, the
// default fixed preset is used instead so the recording still works.
static bool adcrtWriteBurst(uint8_t addr, uint8_t reg, const uint8_t* d, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  for (uint8_t i = 0; i < n; i++) Wire.write(d[i]);
  return (Wire.endTransmission() == 0);
}

static bool hpfCoefReadBackMatches() { // Page 4 must already be selected
  for (uint8_t i = 0; i < 12; i++)
    if (adcrtRead(ADCRT_I2C_ADDRESS, 0x48 + i) != gHpfCoef[i]) return false;
  return true;
}

static void hex12(char* out, const uint8_t* b) {
  for (uint8_t i = 0; i < 12; i++) { sprintf(out + 2 * i, "%02X", b[i]); }
}

static void hpfReadAll(uint8_t* b) { // Page 4 must already be selected
  for (uint8_t i = 0; i < 12; i++) b[i] = adcrtRead(ADCRT_I2C_ADDRESS, 0x48 + i);
}

// Experimental loader (2026-11-05 f). Build 05d/05e bench logs: after writing the
// coefficients the ADC held only the last byte of each (N0=0000006B for 7F7A816B),
// by single-byte writes and (05e) by 12-byte and 4-byte bursts alike, and the
// stream was all zeros. This build records, in gHpfDiag (logged as HPF_DIAG):
//   def = the 12 bytes right after power-up, before anything is written (should be
//         7FFFFFFF 00000000 00000000 if the readback is faithful)
//   A   = readback after a 12-byte burst
//   B   = readback after three 4-byte bursts
//   C   = readback after powering up only the PLL (0x75 = 0x20) and then a 12-byte burst
// and uses the first method whose readback matches. Setup value 0 ("off") writes
// nothing and leaves the reset coefficients in place.
static void hpfLoadCoefficients() {
  uint8_t b[12];
  char h[26];
  const char* how = "";
  bool ok = false;
  gHpfDiag[0] = '\0';
  adcrtWrite(ADCRT_I2C_ADDRESS, 0x00, 0x04);
  hpfReadAll(b); hex12(h, b);
  snprintf(gHpfDiag, sizeof(gHpfDiag), "def=%s", h);
  if (gHpfOff) {
    adcrtWrite(ADCRT_I2C_ADDRESS, 0x00, 0x00);
    snprintf(gHpfInfo, sizeof(gHpfInfo), "off (reset coefficients left in place)");
    return;
  }
  // 05f bench result: a 12-byte burst keeps only the last byte of each word;
  // one 4-byte burst per coefficient word reads back correctly.
  // 06a bench result: the FIRST write after power-up kept only the last byte of
  // each word (B=000000A4...), but in 05f a second write over a first one stuck.
  // So write the three words up to three times, reading back after each pass.
  for (uint8_t pass = 0; pass < 4 && !ok; pass++) {
    for (uint8_t c = 0; c < 3; c++) adcrtWriteBurst(ADCRT_I2C_ADDRESS, 0x48 + 4 * c, &gHpfCoef[4 * c], 4);
    hpfReadAll(b); hex12(h, b);
    snprintf(gHpfDiag + strlen(gHpfDiag), sizeof(gHpfDiag) - strlen(gHpfDiag), " P%u=%s", pass + 1, h);
    ok = (memcmp(b, gHpfCoef, 12) == 0);
  }
  how = "repeated 4-byte bursts";
  adcrtWrite(ADCRT_I2C_ADDRESS, 0x00, 0x00);
  if (ok) {
    snprintf(gHpfInfo, sizeof(gHpfInfo), "%s, coefficients loaded by %s and verified", gHpfBase, how);
  } else {
    // The coefficients would not stick: use the fixed default preset so the
    // recording is not lost (HPF_SEL 1 = 0.00025 x fS).
    gHpfSel = 1;
    gHpfLoadCoef = false;
    snprintf(gHpfInfo, sizeof(gHpfInfo), "%s: coefficient load FAILED, using preset 1", gHpfBase);
  }
}
// Decimation filter type. setting: -1 = automatic (linear phase at 384 kHz, where the
// datasheet gives it twice the flat bandwidth of the ultra-low-latency filter;
// ultra-low latency at every other rate), 0 = linear phase, 1 = low latency,
// 2 = ultra-low latency.
void adcRuntimeSetFilter(int8_t setting, uint32_t fsHz) {
  if (setting < 0 || setting > 2) setting = (fsHz == 384000) ? 0 : 2;
  gDeciFilt = (uint8_t)setting;
}
const char* adcRuntimeFilterName() {
  return gDeciFilt == 0 ? "linear" : (gDeciFilt == 1 ? "lowlat" : "ultralow");
}

const char* adcRuntimeHpfInfo() { return gHpfInfo; }
const char* adcRuntimeHpfDiag() { return gHpfDiag; }
bool adcRuntimeHpfIsCustom() { return gHpfSel == 0; }

// Reads back DSP_CFG0 (0x6B) and, for the programmable filter, the 12 coefficient
// bytes from Page 4, and prints them next to what was written, e.g.
// "0x6B=21 page4 read N0=7F7A816B N1=80857E95 D1=7EF502D5 wrote N0=... N1=... D1=...".
void adcRuntimeHpfDump(char* out, size_t n) {
  int len = snprintf(out, n, "0x6B=%02X", adcrtRead(ADCRT_I2C_ADDRESS, 0x6B));
  if (gHpfSel != 0 || len < 0 || (size_t)len >= n) return;
  uint8_t b[12];
  adcrtWrite(ADCRT_I2C_ADDRESS, 0x00, 0x04);
  for (uint8_t i = 0; i < 12; i++) b[i] = adcrtRead(ADCRT_I2C_ADDRESS, 0x48 + i);
  adcrtWrite(ADCRT_I2C_ADDRESS, 0x00, 0x00);
  snprintf(out + len, n - len,
           " page4 read N0=%02X%02X%02X%02X N1=%02X%02X%02X%02X D1=%02X%02X%02X%02X"
           " wrote N0=%02X%02X%02X%02X N1=%02X%02X%02X%02X D1=%02X%02X%02X%02X",
           b[0],b[1],b[2],b[3], b[4],b[5],b[6],b[7], b[8],b[9],b[10],b[11],
           gHpfCoef[0],gHpfCoef[1],gHpfCoef[2],gHpfCoef[3], gHpfCoef[4],gHpfCoef[5],gHpfCoef[6],gHpfCoef[7],
           gHpfCoef[8],gHpfCoef[9],gHpfCoef[10],gHpfCoef[11]);
}

// One complete power-up + configure attempt (the original adcRuntimeInit()).
// Returns false only if the ADC didn't acknowledge its I2C address; whether the
// configuration actually took is reported separately by gConfigOk.
static bool adcRuntimeInitOnce(uint8_t nchan, int8_t againVal, int adcEnPin, int adcShdnzPin) {
  if (nchan < 1) nchan = 1;
  if (nchan > 4) nchan = 4;
  gConfigOk = false; // set true below only if the readback check passes

  // --- Power-up sequence (mirrors adc.cxx's acqPower()/adcReset()/adcStart()) ---
  // VIN_EN enables the board's onboard 3.3V LDO. SHDNZ is the ADC6140's own
  // reset/shutdown pin and must not be released until the LDO output has
  // settled - see the 100 ms gap below, taken directly from adc.cxx's timing.
  pinMode(adcEnPin, OUTPUT);
  digitalWrite(adcEnPin, HIGH);
  delay(100);

  pinMode(adcShdnzPin, OUTPUT);
  digitalWrite(adcShdnzPin, LOW);   // hold in reset
  delay(100);
  digitalWrite(adcShdnzPin, HIGH);  // release reset

  Wire.begin();
  delay(100);
  Wire.setClock(100000);

  Wire.beginTransmission(ADCRT_I2C_ADDRESS);
  uint8_t err = Wire.endTransmission();
  gLastI2CError = err;
  if (err != 0) {
    return false; // ADC did not ack on I2C - not found / not powered; see adcRuntimeLastI2CError()
  }

  adcrtWrite(ADCRT_I2C_ADDRESS, 0x02, 0x81); // 1.8V AREG, not sleep

  adcrtWrite(ADCRT_I2C_ADDRESS, 0x07, (3 << 4)); // TDM; 32-bit; rising-edge xmit; zero-fill
  adcrtWrite(ADCRT_I2C_ADDRESS, 0x08, 0x00);     // TX_offset 0

  // Programmable high-pass coefficients (only when HPF_SEL = 0), written before
  // any channel is enabled. Page 4, then back to page 0 so every later write
  // (and the readbacks below) go to the normal registers.
  // (06b log: coefficients written here were replaced by the preset-1 values
  // (about 96 Hz) once the ADC powered up / 0x6B was written, so they are now
  // loaded after power-up, below.)

  // --- Runtime channel/slot map ---
  // Slot assignment: physical input channel i (0-based) -> ASI slot i.
  // The first `nchan` physical inputs are used, in order, and packed into
  // the first `nchan` output slots - i.e. IN1 is always slot 0, IN2 slot 1,
  // and so on, regardless of how many channels are active. This differs
  // from adc.cxx's own 1/2-channel tables; see the file header above.
  for (uint8_t slot = 0; slot < 4; slot++) {
    adcrtWrite(ADCRT_I2C_ADDRESS, 0x0B + slot, slot);
  }

  // chanMask enables the first `nchan` input channels, MSB = channel 1
  // (P0_R115 input enable / P0_R116 ASI output slot enable - same bit
  // layout adc.cxx uses for its NCHAN_I2S==4 case).
  uint8_t chanMask = 0;
  for (uint8_t i = 0; i < nchan; i++) chanMask |= (uint8_t)(0x80 >> i);

  adcrtWrite(ADCRT_I2C_ADDRESS, 0x73, chanMask); // enable input channels 1..nchan
  adcrtWrite(ADCRT_I2C_ADDRESS, 0x74, chanMask); // enable ASI output slots 0..nchan-1

  adcrtWrite(ADCRT_I2C_ADDRESS, 0x75, 0xE0); // power up MIC/ADC/PLL (P0_R117)

  adcrtWrite(ADCRT_I2C_ADDRESS, 0x3B, (6 << 4) | (0 << 0)); // MICBIAS=AVDD(3.3V), VREF=2.75V

  adcrtWrite(ADCRT_I2C_ADDRESS, 0x6B, (gDeciFilt << 4) | gHpfSel); // ultra-low latency filter; HPF per setup line 5 (default 0.00025*fs)

  for (uint8_t ch = 0; ch < 4; ch++) {
    uint8_t base = ADCRT_REGS[ch];
    adcrtWrite(ADCRT_I2C_ADDRESS, base + 0, 0x88);                     // line in, 20 kOhm
    adcrtWrite(ADCRT_I2C_ADDRESS, base + 1, (uint8_t)(againVal << 2)); // analog gain
    adcrtWrite(ADCRT_I2C_ADDRESS, base + 2, 201);                      // digital gain (0 dB, DGAIN=0)
    adcrtWrite(ADCRT_I2C_ADDRESS, base + 3, 0x80);                     // 0 dB decimation gain correction
    adcrtWrite(ADCRT_I2C_ADDRESS, base + 4, 0x00);
  }

  // Programmable high-pass: load coefficients now that the ADC is running and
  // HPF_SEL = 0 is already set; falls back to preset 1 if they will not stick.
  if (gHpfLoadCoef) {
    hpfLoadCoefficients();
    if (gHpfSel != 0) adcrtWrite(ADCRT_I2C_ADDRESS, 0x6B, (gDeciFilt << 4) | gHpfSel);
  }

  delay(20); // let settings settle before any readback (matches adc.cxx)

  gConfigOk = (adcrtRead(ADCRT_I2C_ADDRESS, 0x73) == chanMask) &&
              (adcrtRead(ADCRT_I2C_ADDRESS, 0x74) == chanMask);

  // --- ASI_STS poll-for-lock (see adcRuntime.h) - replaces the old
  // single-shot 0x15 readback with a short retry loop, so we capture how
  // many attempts (if any) it took to see the 0x30 pattern, instead of
  // just whatever happened to be there on one read. ---
  {
    uint8_t asiSts = 0;
    uint8_t attempt;
    for (attempt = 1; attempt <= ADCRT_ASI_STS_MAX_ATTEMPTS; attempt++) {
      asiSts = adcrtRead(ADCRT_I2C_ADDRESS, ADCRT_ASI_STS_REG);
      if ((asiSts & ADCRT_ASI_STS_GOOD_MASK) == ADCRT_ASI_STS_GOOD_MASK) break;
      delay(ADCRT_ASI_STS_RETRY_DELAY_MS);
    }
    gAsiStsFinalValue = asiSts;
    gAsiStsAttempts = (attempt <= ADCRT_ASI_STS_MAX_ATTEMPTS) ? attempt : 0;
  }

#if TUFR_DEBUG_SERIAL
  Serial.printf("ADC init: nchan=%u chanMask=0x%02X gain=%d\n", nchan, chanMask, againVal);
  Serial.print("  0x15: "); Serial.print(gAsiStsFinalValue, HEX);
  Serial.print("  (attempts to reach 0x30 pattern: "); Serial.print(gAsiStsAttempts);
  Serial.println(gAsiStsAttempts == 0 ? ", NEVER)" : ")");
  Serial.print("  0x73: "); Serial.println(adcrtRead(ADCRT_I2C_ADDRESS, 0x73), HEX);
  Serial.print("  0x74: "); Serial.println(adcrtRead(ADCRT_I2C_ADDRESS, 0x74), HEX);
  Serial.print("  0x76: "); Serial.println(adcrtRead(ADCRT_I2C_ADDRESS, 0x76), HEX);
#endif

  return true;
}

// Power-up with retry (added 2026-11-01). On the PCB, a COLD start (USB
// re-plug) left the ADC acknowledging its address but failing every register
// read and write; a warm start worked, and so did a full ADC power-cycle
// (regulator off ~0.5 s, then up again) on the second attempt. Suspected cause
// is the extra 100 uF mic-bias capacitor slowing the first power-up. Whatever
// the cause, a cold start is the normal case in the field, so: if the ADC
// doesn't answer or the channel-enable readback doesn't match, power-cycle it
// fully and try again, up to ADCRT_MAX_INIT_ATTEMPTS times in total.
#define ADCRT_MAX_INIT_ATTEMPTS 3
static uint8_t gInitAttempts = 0;
uint8_t adcRuntimeInitAttempts() { return gInitAttempts; }

bool adcRuntimeInit(uint8_t nchan, int8_t againVal, int adcEnPin, int adcShdnzPin) {
  bool ack = false;
  gInitAttempts = 0;
  for (uint8_t attempt = 1; attempt <= ADCRT_MAX_INIT_ATTEMPTS; attempt++) {
    gInitAttempts = attempt;
    ack = adcRuntimeInitOnce(nchan, againVal, adcEnPin, adcShdnzPin);
    if (ack && gConfigOk) break;
    if (attempt < ADCRT_MAX_INIT_ATTEMPTS) {
#if TUFR_DEBUG_SERIAL
      Serial.print("ADC init attempt "); Serial.print(attempt);
      Serial.println(" failed - power-cycling the ADC board and retrying");
#endif
      Wire.end();
      digitalWrite(adcShdnzPin, LOW);
      digitalWrite(adcEnPin, LOW);
      delay(500);
    }
  }
  return ack; // false only if the LAST attempt got no I2C ack; a config mismatch is reported via adcRuntimeHealthy()
}

void adcRuntimeShutdown(int adcEnPin, int adcShdnzPin) {
  digitalWrite(adcShdnzPin, LOW);
  digitalWrite(adcEnPin, LOW);
}

void adcRuntimeSetGain(int8_t againVal) {
  for (uint8_t ch = 0; ch < 4; ch++) {
    adcrtWrite(ADCRT_I2C_ADDRESS, ADCRT_REGS[ch] + 1, (uint8_t)(againVal << 2));
  }
}
