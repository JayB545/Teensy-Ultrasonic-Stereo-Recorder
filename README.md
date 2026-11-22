# TUFR 4.2 - Teensy Ultrasonic Field Recorder

Firmware 4.2, build 2026-11-14b.

Low-power field recorder for ultrasonic sound (bats, cetaceans). A Teensy 4.1 records 16-bit WAV or lossless FLAC files to a microSD card from a WMXZ-EU ADC_V2 module (TLV320ADC6140): 1, 2 or 4 channels at 48/96/192 kHz, 1-2 channels at 384 kHz, 1 channel at 768 kHz or `-768` (768 kHz sampled, 384 kHz files). Every deployment setting is read from `TUFR_Setup.txt` on the SD card; nothing needs recompiling.

**Users: see the TUFR 4.2 User Manual** for the setup file, filter responses, gain, file formats, scheduling, LED codes and the log. This README is for whoever builds or modifies the firmware.

## Building

- Arduino IDE with Teensyduino, board **Teensy 4.1**, Tools > Optimize **"Faster"**.
- Libraries: **SdFat** (the Teensyduino copy), **Time** (TimeLib), **Wire**. No Snooze library.
- The sketch folder must be named `TUFR_4.2` (Arduino requires the folder to match `TUFR_4.2.ino`).
- RAM: the FLAC encoder reserves about 92 KB of RAM1 even in WAV mode. If the compile reports RAM1 short, build `tufr_flac.c` with `-DTUFR_FLAC_MAX_BLOCK=1024` (files about 0.3 % larger).
- Uploading with the Teensy Loader sets the RTC to the computer's clock.
- Before a field release check that `TUFR_WDT_TEST` is `0`.

## Hardware notes

- MotherBoard: BarlowAGAD TUFR Bridgeboard v3a (the v4 motherboard is in design). Pin assignments are in `TUFR_Config.h` and `TUFR_pin_table.html`: ADC_EN 9, SHDNZ 6, I2C SDA 18 / SCL 19, BCLK 21, LRCLK 20, data 8, shutdown button 2, LED 13, Vin divider 22 (A8). In the current version, components are hand-soldered onto the bare motherboard.
- ADC Module: In the current version, the ADC module is manufactured fully populated (except for the pin headers).
- **IOVD**: the ADC module's IOVD pad must be connected to the Teensy 3.3 V (without it: cold-start dropouts at 384 kHz and HF spikes).
- **USB power**: cut the Teensy's VUSB-VIN pad. USB then carries data only; bench work with the serial monitor needs battery or bench-supply power. `Vin` in the log is then the buck output (about 4.1 V); below about 3.8 V the batteries are running out.
- Input pins must stay within the ADC's supply rails (0 to 3.3 V). Consider a series resistor and clamp diodes per input if a pre-amp runs from a higher supply.

## Compile-time settings

| Setting | Where | Default | What it does |
| --- | --- | --- | --- |
| `TUFR_WATCHDOG` | `TUFR_4.2.ino` | 1 | Hardware watchdog (RTWDOG) on |
| `WDT_TIMEOUT_S` | `TUFR_4.2.ino` | 10 | Watchdog timeout, seconds (bench-confirmed) |
| `TUFR_WDT_TEST` | `TUFR_4.2.ino` | 0 | Bench test: 1 = stop feeding the watchdog 30 s into the first file (runs once; delete `TUFR_WDT_DONE.TXT` to repeat). **0 for the field.** |
| `DEAD_STREAM_RECOVER_MS` | `TUFR_4.2.ino` | 500 | All-zero audio this long = dead stream |
| `DEAD_STREAM_INPLACE_TRIES` | `TUFR_4.2.ino` | 3 | In-place ADC restarts per file before a full power cycle |
| `POWER_CYCLE_SHORT_S`, `_LONG_S`, `_SHORT_TRIES` | `TUFR_4.2.ino` | 60, 600, 3 | Power-off time for recovery: 60 s, then 10 min after three in a row |
| `SD_WRITE_FAIL_RUN` | `TUFR_4.2.ino` | 8 | Consecutive failed SD writes that count as a failing card |
| `TUFR_PREALLOCATE` | `TUFR_4.2.ino` | 0 | Preallocate the next file (off: preallocation caused mid-file misses) |
| `TUFR_USB_OFF` | `TUFR_4.2.ino` | 2 | Power down USB PHY and PLL while recording if no serial monitor is open (saves about 13 mA) |
| `gFiltSel` | `TUFR_4.2.ino` | -1 | ADC decimation filter: -1 auto (linear phase at 384 kHz, ultra-low latency elsewhere), 0 linear, 1 low latency, 2 ultra-low latency. Not in the setup file. |
| `TUFR_HPF_D1_SIGN` | `TUFR_Config.h` | 1 | Sign convention of the programmable high-pass (1 is correct) |
| `TUFR_VIN_DIVIDER` | `TUFR_Config.h` | 2.0 | (Rtop + Rbottom) / Rbottom of the Vin divider on pin 22 |
| `TUFR_FLAC_MAX_BLOCK` | `tufr_flac.h` | 2048 | FLAC frame length per channel |
| `MAX_QUEUE` | `global.h` | 14 | Audio queue, 32 KB slots (about 300 ms at 384 kHz x 2 ch) |

## How it works

- **Capture**: Walter Zimmer's SAI/DMA driver fills a 14-slot queue of 16-bit samples (32 KB per slot); `loop()` writes slots to the card, or encodes them to FLAC first. In `-768` mode a 43-tap half-band FIR in the DMA interrupt halves the rate.
- **Gap-free files**: in continuous mode the next file is opened about 6 s before the join, so only the old file's close (about 25 ms) falls at the join; each file starts at the previous file's scheduled end, so files are exactly the set length. The FLAC encoder carries its state across files.
- **FLAC** (`tufr_flac.c/.h`, written for TUFR, no libFLAC, no malloc): fixed predictors of order 0-4 chosen per channel per frame, partitioned Rice coding (up to 64 partitions), 2048-sample frames, constant/verbatim subframes, a 512-byte header so every 32 KB SD write stays sector-aligned, header rewritten at close. About 33 cycles per sample on quiet input, 40-48 on loud input.
- **Waits** (duty cycle, daily window, scheduled start) use Walter Zimmer's SNVS hibernate: the 3.3 V regulator is switched off (0.75 mA) and the RTC alarm restarts the board; a marker file `TUFR_HIB.TXT` tells the restart it is a wake.
- **Unattended recovery**: nothing but the shutdown button stops the recorder for good.
  1. Dead stream (0.5 s of zeros): up to 3 in-place ADC restarts per file, pauses 0.3, 2 and 10 s.
  2. Full power cycle (hibernate 60 s, then 10 min after three in a row; count reset after a file with 10 s of live audio) when that fails, when the ADC does not answer (5 blinks), or on any SD problem (2 blinks: missing, full, failing writes, file not created). Hibernation also removes IOVD power, which an in-place ADC restart cannot.
  3. Watchdog: a hang of 10 s restarts the chip, which powers off at once (before touching the SD card) for 60-123 s. The wake time is chosen so that the alarm second modulo 64 is 37; the next boot recognises that and logs `WATCHDOG_RESET`. (The SNVS general-purpose registers did not keep a value through the power-off on this board, so they are not used.)
- **SD space**: free space is measured once per boot (`SD_SPACE`) and counted down as files close; a file is started only if a full-size WAV file would fit.
- **Setup file**: ten lines in a fixed order; the file-format line is a word (`WAV`/`FLAC`); if line 4 is a number the format line is treated as missing (WAV). 3 channels are recorded as 4 (16384-sample blocks are not a whole number of 3-channel frames). `-768` with FLAC raises the clock to 100 MHz.

## Files

| File | Role |
| --- | --- |
| `TUFR_4.2.ino` | Main sketch: setup file, file handling, FLAC staging, waits, hibernate, recovery, watchdog |
| `tufr_flac.c/.h` | Lossless FLAC encoder |
| `Teensy.cpp/.h`, `adc.cpp/.h`, `process.h`, `queueProcess.cpp`, `global.h` | Walter Zimmer's capture driver (modifications noted in the sketch header) |
| `adcRuntime.cpp/.h` | ADC configuration, channel count, high-pass, retries, health checks |
| `rtc.h`, `rtcImpl.cpp` | Time functions for the capture driver (never define `rtc_get`/`rtc_set` here; they belong to the Teensy core) |
| `TUFR_Config.h` | Pins, Vin divider, high-pass sign |
| `TUFR_Setup.txt` | Template setup file for the SD card |
| `TUFR_pin_table.html` | Printable pin table |

The PC test harness for the encoder (`TUFR_flac_pc_test`) ships beside this folder.

## Status (2026-11-14)

Bench-validated on Bridgeboard v3:

- WAV and FLAC at 384 kHz x 1 and 2 ch, 192 kHz x 4 ch, `-768`; continuous files joined without gaps; 60 MHz with no mid-file misses, including loud white noise on all channels (worst case 62 % CPU).
- High-pass presets and programmable corners; frequency response at 96, 192, 384 kHz and `-768` (see the manual).
- Setup file 13b format, 3 to 4 channels, `CLOCK_RAISED`.
- Recovery: watchdog restart, power-off and `WATCHDOG_RESET` logging; missing SD card (2 blinks, recording resumes when a card is inserted).
- Hibernate between recordings at 0.75 mA; recording 87 mA at 384 kHz x 2 ch WAV.

### Bench tests still to do

1. Full card: a nearly full card should end with complete files, `CARD_FULL` and 2 blinks.
2. A WAV file cut off by a reset should open in Audacity and PAMGuard (placeholder header "length unknown").
3. Recording current, FLAC vs WAV at 60 MHz and at other rates and channel counts (the manual's battery figures cover only 384 kHz x 2 ch WAV).
4. One recording on the towed array with the 40 dB pre-amp and about 10 dB ADC gain: headroom and FLAC ratio.
5. Recording at 48 kHz (anti-alias response not measured).

### Known open issues

- An input overload (1.5 Vrms) once stopped the ADC's output entirely (seen twice on 2026-11-13, not reproduced since); recovery now covers it.
- Small spikes at 468 Hz and multiples (source unknown), a 32.77 kHz spur (RTC crystal), 60 Hz pickup.
- The first file of a 768 kHz run can lose up to about 1 s to an SD stall at start-up (before the file opens).

## Version history

- **4.2 (2026-11-12 to 11-14)**: optional FLAC; encoder 3x faster (13a); setup file reordered with the format as a word, decimation-filter line removed (13b); 3 channels recorded as 4, `-768` + FLAC at 100 MHz (13c); unattended recovery and watchdog (13d); SD-card checks (14a); watchdog wake recognised by its alarm time (14b).
- **4.1 (2026-11-03 to 11-09)**: Walter Zimmer's capture driver; ADC protection; SNVS hibernate; daily window and scheduled start; gap-free continuous files; programmable high-pass; `-768` decimation; Vin logging; USB power-down while recording.
- **4.0**: first version with its own SAI capture (garbled at 384 kHz x 2 ch).

Credits: capture driver and hibernate method (c) Walter Zimmer, microPAM project (https://github.com/WMXZ-EU/LC-MARE/tree/main/microPAM_V3); ADC configuration sequence from his `adc.cxx`. Adapted by Jay Barlow with Claude (Anthropic).
