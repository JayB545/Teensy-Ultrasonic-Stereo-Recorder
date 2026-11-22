/*
 * adcRuntime.h
 *
 * Field-configurable (1-4 channel) initialization for the WMXZ-EU ADC_V2
 * module (TLV320ADC6140), for the TUFR ultrasonic field recorder.
 *
 * See adcRuntime.cpp for a full explanation of how this relates to, and
 * differs from, Walter Zimmer's adc.cxx/adc.h.
 */
#ifndef ADC_RUNTIME_H
#define ADC_RUNTIME_H

#include <stdint.h>
#include <stddef.h>

// Initializes the ADC_V2 board for `nchan` channels (1-4), using physical
// inputs IN1..IN(nchan) in order (IN1 first). `againVal` is the analog gain
// register value (0-42, same units/scale as Walter's `again`, see adc.h).
// `adcEnPin` drives the board's VIN_EN (LDO enable); `adcShdnzPin` drives
// SHDNZ (ADC reset/shutdown). Returns false if the ADC did not ack on I2C -
// see adcRuntimeLastI2CError() to distinguish why.
bool adcRuntimeInit(uint8_t nchan, int8_t againVal, int adcEnPin, int adcShdnzPin);

// The raw Wire.endTransmission() result code from the detection probe in
// the most recent adcRuntimeInit() call: 0 = success (ADC init proceeded
// normally - this function only has a meaningful non-zero value after
// adcRuntimeInit() returned false). Per the Wire library: 1 = data too
// long for buffer, 2 = NACK on transmit of address (nothing answered at
// all - classic "not powered" or "not connected" signature), 3 = NACK on
// transmit of data, 4 = other error, 5 = timeout (bus stuck - e.g. SDA or
// SCL shorted/stuck low). Logged alongside HALT_ADC_NOT_FOUND so a field
// failure shows which of these it was instead of just pass/fail.
uint8_t adcRuntimeLastI2CError();

// Result of the most recent adcRuntimeInit(): adcRuntimeConfigOk() is true if
// IN_CH_EN (0x73) and ASI_OUT_CH_EN (0x74) read back equal to what was just
// written; adcRuntimeHealthy() is true only if the ADC also acknowledged its
// I2C address (adcRuntimeLastI2CError() == 0). Either being false means the
// chip is not really responding and any recording would be silent.
bool adcRuntimeConfigOk();
bool adcRuntimeHealthy();
// How many power-up attempts the most recent adcRuntimeInit() needed (1 = first
// try worked; up to 3, with a full ADC power-cycle between tries).
uint8_t adcRuntimeInitAttempts();

// Powers the ADC board back down (SHDNZ low, then VIN_EN low). Call before
// sleep/duty-cycle wait states and before a safe shutdown.
void adcRuntimeShutdown(int adcEnPin, int adcShdnzPin);

// High-pass filter selection (setup-file line 5). Call once BEFORE the first
// adcRuntimeInit() (the setting is kept and re-applied on every later init).
//   -1 : the ADC's fixed preset 1, 0.00025 x fS (96 Hz at 384 kHz) - the default
//   -2 : fixed preset 2, 0.002 x fS (768 Hz at 384 kHz)
//   -3 : fixed preset 3, 0.008 x fS (3072 Hz at 384 kHz)
//    0 : high-pass off (the programmable filter is loaded as all-pass)
//   >0 : cutoff in Hz, programmable first-order filter with coefficients
//        computed from fsHz; clamped to 1 Hz .. fsHz/10.
void adcRuntimeSetHpf(int32_t hpfHz, uint32_t fsHz);
// Decimation filter: -1 automatic (linear phase at 384 kHz, ultra-low latency otherwise), 0 linear, 1 low latency, 2 ultra-low latency.
void adcRuntimeSetFilter(int8_t setting, uint32_t fsHz);
const char* adcRuntimeFilterName();
// Short text describing what was selected, e.g. "preset 1 (96 Hz)", "off",
// "custom 20 Hz". For logging.
const char* adcRuntimeHpfInfo();
// Readbacks from the last programmable-coefficient load (diagnostic; empty if none).
const char* adcRuntimeHpfDiag();
// True if the programmable filter (HPF_SEL = 0: "off" or a custom cutoff) is selected.
bool adcRuntimeHpfIsCustom();
// Diagnostic: text with the ADC's 0x6B and (for the programmable filter) the
// coefficient bytes read back from Page 4, next to the bytes that were written.
void adcRuntimeHpfDump(char* out, size_t n);

// Updates the analog gain on all 4 channel config blocks without a full
// re-init. Safe to call at any time after adcRuntimeInit() has succeeded.
void adcRuntimeSetGain(int8_t againVal);

// =====================================================================
// DIAGNOSTIC: ASI_STS (register 0x15) poll-for-lock, added to test the
// newest corruption hypothesis with hard, persistent data instead of a
// one-off Serial print.
//
// Per TI's TLV320ADC6140 datasheet, register 0x15 (ASI_STS - "Audio
// Serial Interface Status") "captures the device auto detect result for
// the FSYNC frequency and the BCLK to FSYNC ratio" - i.e. it reflects
// whether the ADC's own clock-recovery logic has successfully locked
// onto the BCLK/FSYNC Teensy is driving it with, not just an echo of a
// config register. Three prior one-off readings (2 corrupted-audio
// cases, 1 clean) showed bits 0x30 set only in the clean case - a
// real but thin (3-point) correlation, not a confirmed bit-field
// meaning (TI's public datasheet text available to us doesn't include
// the detailed bit tables for this register).
//
// adcRuntimeInit() now polls this register (with a short delay between
// reads, capped at a fixed attempt count) right after channel config,
// looking for that 0x30 pattern, instead of reading it once. These two
// getters expose the result of that poll so the caller can log it
// persistently (e.g. via writeTelemetryLog()) on every run, turning the
// 3-point correlation into an ongoing, cross-run dataset:
//
//   adcRuntimeAsiStsAttempts(): how many reads it took to see the 0x30
//     pattern (1 = present on the very first read after config settled).
//     0 means the pattern was NEVER seen within the attempt cap - i.e.
//     the poll timed out still "unlocked" by this working definition.
//   adcRuntimeAsiStsFinalValue(): the actual register value on the last
//     read attempt (whether or not the 0x30 pattern was ever matched),
//     so the raw byte is always available even when the pattern guess
//     turns out to be wrong.
//
// Valid only after adcRuntimeInit() has been called at least once.
uint8_t adcRuntimeAsiStsAttempts();
uint8_t adcRuntimeAsiStsFinalValue();

#endif
