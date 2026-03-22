# TUFR: Teensy Ultrasonic Field Recorder

TUFR is an low-power, high-fidelity, stereo acoustic field recorder designed for continuous or scheduled ultrasonic monitoring (such as bat echolocation or bird vocalizations). 

Powered by the Teensy 4.1 microcontroller, TUFR receives signals from an external I2S ADC module and records 192kHz, 16-bit audio directly to an ExFAT/FAT32 MicroSD card. By dropping the CPU clock to 24MHz and utilizing a hardware power switch for the ADC and microphone, TUFR achieves long battery life for long-term remote deployments.

## 🌟 Key Features

* **High-Fidelity Ultrasonic Capture:** Gapless 1-minute stereo WAV files recorded at 192kHz (via 3-wire I2S module in Master Mode).
* **"Purity Mode" Audio:** Raw, un-equalized PCM data is streamed straight to the SD card for maximum fidelity and zero CPU overhead.
* **Ultra-Low Power:** Draws ~100mA while actively recording and drops to ~32mA during sleep states. A single 3400mAh 18650 lithium cell yields ~12 hours of continuous recording, scaling linearly with larger packs.
* **Smart Duty Cycle & Daily Scheduling:** Record continuously, set a duty cycle (e.g., 1 min on / 5 mins off), or restrict recording to specific daily windows (e.g., 20:00 to 06:00) with automatic midnight-crossing logic.
* **Robust File Management:** Supports high-capacity ExFAT SD cards via 4-bit SDIO. Includes correct Windows/Mac "Date Modified" file timestamps.
* **Field-Ready Telemetry:** Logs SD write errors, boot reasons, and smoothed battery voltage to a persistent `TUFR_log.txt` file.
* **Battery Protection:** Safely finalizes files and enters a permanent deep-halt if the lithium cell drops below 3.0V.

## 🛠️ Hardware Requirements

* **Microcontroller:** Teensy 4.1 (with attached CR coin cell for RTC backup).
* **ADC Module:** WM8782 (or equivalent) 24-bit/192kHz ADC as I2S master (runs on 3.3V using Vcc pin labeled for 5V). Jumper 24.576MHz pin to MCLK pin.
* **Power Supply:** 5V (e.g. output from an 18650 battery pack (via Boost Converter) or an 18650 UPS Hat). Power must be supplied to the `VIN` pin.
* **Power Switch:** Pololu 2810 High-Side P-Channel MOSFET Switch (to kill 3.3V ADC & microphone power during sleep).
* **Storage:** High-speed MicroSD Card (ExFAT or FAT32, tested up to 512GB, should be OK at 2TB).

## 🔌 Wiring Guide

| Teensy 4.1 Pin | Peripheral / Component | Notes |
| :--- | :--- | :--- |
| **VIN (5V)** | 18650 5V Boost Output | Primary system power. |
| **3.3V** | Pololu-switched power supply | ADC & microphone pre-amp power. |
| **GND** | Common Ground | Tie all components to a common ground. |
| **VBAT** | 3V CR Coin Cell | Keeps the RTC running when main power is off. |
| **Pin 9** | Pololu Switch "ON" Pin | High=ADC On, Low=ADC Off (Sleep). |
| **Pin 2** | Shutdown Button | Connect to GND via momentary pushbutton. |
| **Pin 22 (A8)** | Battery Voltage Divider | Connect to the center of a 1:1 voltage divider (e.g., two 10kΩ resistors) measuring the *raw* battery voltage (B+). |
| **Pin 7** | I2S TX | Audio Data Out (Anti-ghosting pull-down). |
| **Pin 8** | I2S RX | Audio Data In (From ADC `DATA` or `DOUT`). |
| **Pin 20** | I2S LRCLK | Word Select / Left-Right Clock. |
| **Pin 21** | I2S BCLK | Bit Clock. |
| **Pin 13** | Onboard LED | Status indicator. |

*Note: The voltage divider must tap the raw Lithium-Ion voltage (max 4.2V), **not** the 5V regulated output, to accurately trigger the 3.0V low-battery cutoff.*

## ⚙️ Configuration (`TUFR_Setup.txt`)

TUFR is configured entirely via a plain text file on the root of the SD card named `TUFR_Setup.txt`. This allows researchers to change deployment parameters in the field without recompiling code.

You can copy and paste the following template directly into your `TUFR_Setup.txt` file. The recorder will parse the values and ignore the `#` comments. The file **must** contain exactly 6 lines.

```text
1 0                     # Record Minutes, Wait Minutes (e.g., 1 0 is continuous)
24                      # Clock Speed (24 is lowest power, 600 is max)
S                       # Mode: S (Stereo), L (Mono-Left), R (Mono-Right)
2026 03 12 17 40 00     # Optional: Force Time Sync (YYYY MM DD HH MM SS) (if later than current clock time)
2026 03 10 06 00 00     # Optional: Scheduled Start Date/Time (will start immediately if zero or in the past)
0000 2400               # Start/End times
```

## ⚙️ Line-by-Line Breakdown of Setup File:

**Duty Cycle:** Sets the recording duration and sleep duration.

**Clock Speed:** Sets the Teensy's CPU frequency in MHz during recording. 24 is recommended for optimal power savings (higher if you get write errors).

**Audio Mode:** Determines how the I2S channels are written to the WAV file.

**RTC Sync:** Updates the Teensy's internal Real-Time Clock if the provided timestamp is newer than the currently saved time. Disconnect RTC battery to reset to earlier time.

**Scheduled Start:** Forces the recorder into an ultra-low-power sleep state until the specific date and time are reached. Immediate start if scheduled start is earlier than current time.

**Daily Schedule:** Restricts recording to a specific daily time window using military time (e.g., 2000 0600 records overnight). Use 0000 2400 to run 24/7.

## ⚙️ LED Status Indicators

* **Solid On (Boot):** Initializing SD card and reading configuration.

* **Brief Flash (Every 3s):** Actively recording audio.

* **Faint Flash (Every 10s):** System is sleeping (Duty Cycle Wait or Daily Schedule Wait).

* **Continuous Fast Strobe:** Critical Error (SD card failed to mount, or SD card is full).

* **1 Slow Flash loop:** User-triggered Safe Shutdown (via Pin 2). It is now safe to remove the SD card.

* **2 Slow Flashes loop:** Low Battery Cutoff triggered. Battery must be recharged.

## ⚙️ Dependencies

To compile this sketch in the Arduino IDE (Board = Teensy 4.1), you will need the following libraries:

* **Audio** (Native Teensy Library)

* **SdFat** (For ExFAT and high-speed SDIO support)

* **Time** (TimeLib for RTC management)

## Recording quality
* **Noise floor** from 100Hz to 96kHz is typically below -120dB re: full scale when input leads are shorted:
<img width="1000" height="400" alt="NoiseFloor dB re FullScale" src="https://github.com/user-attachments/assets/042ddf66-632f-4f5a-93bb-d991a18450d9" />
