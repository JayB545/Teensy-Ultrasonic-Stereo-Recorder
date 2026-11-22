/*
 * TUFR_Config.h - board wiring and build-time switches for TUFR 4.0
 * (ADC_V2 / TLV320ADC6140 version)
 */
#ifndef TUFR_CONFIG_H
#define TUFR_CONFIG_H

// Set to 1 to keep USB/serial alive throughout operation, for field
// diagnostics (config readback, overrun counts, SD write timing). Set to
// 0 for lowest power consumption (matches TUFR 2.4's default behavior).
#ifndef TUFR_DEBUG_SERIAL
#define TUFR_DEBUG_SERIAL 1
#endif

// Programmable high-pass filter (setup line 5): sign of the D1 coefficient. The
// datasheet's transfer function was not available, so +1 is an assumption. If a
// custom cutoff gives saturated noise instead of audio, try -1.
#ifndef TUFR_HPF_D1_SIGN
#define TUFR_HPF_D1_SIGN 1
#endif

// ---- Pins carried over unchanged from TUFR 2.4 ----
#define PIN_LED         13
#define PIN_SHUTDOWN    2
#define PIN_BATT        A8   // = pin 22, mid-point of the battery-voltage divider

// Vin sense: the divider on PIN_BATT is read and logged as "Vin = x.xx" (volts at the
// board's supply input) with CONFIG and with every file's stats line, so an unplanned
// stop from a depleted battery shows up as a falling Vin in TUFR_log.txt.
// TUFR_VIN_DIVIDER = (Rtop + Rbottom) / Rbottom: 2.0 for two equal resistors (10k/10k).
// The pin must never see more than 3.3 V, so a 1:1 divider measures at most 6.6 V.
// For the 5-20 V input of the planned buck board use a ratio of about 7 (e.g. 10k over
// 1.8k, ratio 6.56 -> 21.6 V full scale) and set this to the real value.
#ifndef TUFR_VIN_DIVIDER
#define TUFR_VIN_DIVIDER 2.0f
#endif

// ---- ADC_V2 / TLV320ADC6140 control pins ----
// PIN_ADC_EN drives the board's VIN_EN (onboard 3.3V LDO enable) - this is
// the same physical pin (9) that previously drove the Pololu switch.
// PIN_ADC_SHDNZ drives SHDNZ (the ADC6140's own reset/shutdown pin) and is
// new - the WM8782 had no equivalent.
#define PIN_ADC_EN      9
#define PIN_ADC_SHDNZ   6

// ---- I2S (SAI1) pins - unchanged from TUFR 2.4's wiring ----
// These are fixed by the Teensy 4.1's SAI1 peripheral pinout, not
// arbitrary GPIOs, so they cannot be moved without also changing the
// register setup in saiCapture.cpp.
#define PIN_I2S_BCLK    21   // SAI1_RX_BCLK  (Teensy drives this - master)
#define PIN_I2S_LRCLK   20   // SAI1_RX_SYNC  (Teensy drives this - master)
#define PIN_I2S_DATA     8   // SAI1_RX_DATA0 (from ADC SDOUT)

// ---- I2C (ADC configuration bus) ----
// Uses the default `Wire` (pins 18/19), matching mWire=Wire in Walter's
// global.h for the T_4_1 target.

#endif
