/* * TEENSY 4.1 ULTRASONIC FIELD RECORDER (TUFR): MODEL 2.4
 * * Designed by Jay Barlow with coding by Gemini AI
 * * Features: 
 * - Records gapless sequential 1-minute WAV files on SD card
 * - 3-wire I2S input from ADC in master mode (192kHz)
 * - Purity Mode: Raw un-equalized data straight to SD
 * - Smart Duty Cycle with 24MHz low-power wait state
 * - Pololu P-Channel Power Switch for ADC module (Pin 9)
 * - Sortable Date/time naming of WAV files & FAT Timestamps
 * - Config-driven RTC sync and Scheduled Start times
 * - Daily Recording Window (e.g., 2000 to 0600)
 * - Battery supply voltage monitoring on Pin 22 (A8)
 * - Low Battery Cutoff & Safe Shutdown Switch (Pin 2)
 * - SD Write Error Tracking & Telemetry Logging
 * - Sleep-state shutdown sensing & Voltage smoothing
 */

#undef AUDIO_SAMPLE_RATE_EXACT
#define AUDIO_SAMPLE_RATE_EXACT 192000.0f

#include <Audio.h>
#include <SdFat.h>
#include <TimeLib.h>
#include <ctype.h> 

const char* FIRMWARE_VERSION = "TUFR_2.4_Release";

// --- CLOCK SPEED CONTROL ---
extern "C" uint32_t set_arm_clock(uint32_t frequency);
const uint32_t WAIT_CLOCK_FREQ = 24000000;

// --- AUDIO SYSTEM (PURITY MODE - NO EQ) ---
AudioInputI2Sslave       i2s_in;
AudioRecordQueue         queueLeft;
AudioRecordQueue         queueRight;
AudioConnection          patchCord1(i2s_in, 0, queueLeft, 0);
AudioConnection          patchCord2(i2s_in, 1, queueRight, 0);

SdFs sd;
FsFile frec;

// --- PINS ---
const int LED_PIN = 13;
const int SHUTDOWN_PIN = 2;
const int BATT_PIN = A8;       
const int ADC_POWER_PIN = 9;   

// I2S Pins
const int PIN_I2S_TX = 7;
const int PIN_I2S_RX = 8;
const int PIN_I2S_LRCLK = 20;
const int PIN_I2S_BCLK = 21;

const float LOW_BAT_THRESHOLD = 3.0;
const float VOLTAGE_DIVIDER_RATIO = 2.0;
const uint32_t STABILIZATION_DELAY_MS = 3000; 

// --- CONFIGURABLE GLOBALS ---
unsigned long recordTimeMillis = 60000; 
unsigned long waitTimeMillis = 0;
uint32_t recordClockMHz = 24;      
char audioMode = 'S';               
time_t scheduledStartUnix = 0;

// Daily Schedule
int dailyStartHHMM = 0;    
int dailyStopHHMM = 2400;  
bool useDailySchedule = false;

// Internal Tracking
unsigned long sessionStartMillis = 0;
unsigned long lastBattCheckMillis = 0;
uint32_t bytesWrittenInCurrentFile = 0;
char filename[64];
uint32_t sdWriteErrors = 0;

// LED Timers
unsigned long lastRecBlink = 0;
bool isLedOn = false;
unsigned long ledOnStartTime = 0;

const uint32_t SAMPLE_RATE = 192000;

// Forward Declarations
void nukeUsbMagic(); 
void enterWaitModeAndReset(); 
void enterLowPowerHalt();
void handleRecordLed();
time_t getTeensy3Time();
void dateTime(uint16_t* date, uint16_t* time); // NEW: SD Timestamp Callback
void writeTelemetryLog(const char* event, const char* val);
void logResetReason();
void errorFlash(int count);
void triggerBadCardMode();
void readConfigFile();
bool readNextLine(FsFile &file, char* buffer, int maxLen);
void closeCurrentFile(); 
void writeWavHeader(FsFile &file, uint32_t dataSize); 
void processAudioStereo();
void processAudioMonoL();
void processAudioMonoR();
void handleFileEnd();
bool isWithinDailySchedule();
void handleDailySleep();
float readBatteryVoltage(); 

// Reset Registers
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

// === SD CARD TIMESTAMP CALLBACK ===
void dateTime(uint16_t* date, uint16_t* time) {
  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(year(), month(), day());
  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(hour(), minute(), second());
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SHUTDOWN_PIN, INPUT_PULLUP);
  pinMode(BATT_PIN, INPUT);
  
  nukeUsbMagic();
  setSyncProvider(getTeensy3Time); 

  digitalWrite(LED_PIN, HIGH); delay(500); digitalWrite(LED_PIN, LOW); delay(200);

  if (!sd.begin(SdioConfig(FIFO_SDIO))) {
    triggerBadCardMode();
  }

  // Set the callback to timestamp files correctly in Windows/Mac
  FsDateTime::setCallback(dateTime);

  readConfigFile();

  writeTelemetryLog("BOOT_VERSION", FIRMWARE_VERSION);
  char configLog[128];
  sprintf(configLog, "Rec:%lum Wait:%lum Clk:%luMHz Mode:%c Sched:%04d-%04d", 
          recordTimeMillis/60000, waitTimeMillis/60000, recordClockMHz, audioMode, dailyStartHHMM, dailyStopHHMM);
  writeTelemetryLog("CONFIG", configLog);
  logResetReason();

  // --- PRE-RECORDING WAIT CHECKS ---
  time_t now = Teensy3Clock.get();
  if (scheduledStartUnix > now + 15) {
      writeTelemetryLog("SCHEDULED_WAIT", "Waiting for absolute start time");
      pinMode(ADC_POWER_PIN, OUTPUT);
      digitalWrite(ADC_POWER_PIN, LOW); 
      set_arm_clock(WAIT_CLOCK_FREQ);
      delay(10);
      
      unsigned long lastWaitBlink = millis();
      while (Teensy3Clock.get() < scheduledStartUnix) {
          if (digitalRead(SHUTDOWN_PIN) == LOW) {
              writeTelemetryLog("HALT_USER", "Shutdown during Schedule Wait");
              enterLowPowerHalt(); 
              while(1) { errorFlash(1); } 
          }
          if (millis() - lastWaitBlink > 10000) {
              digitalWrite(LED_PIN, HIGH); delay(10); digitalWrite(LED_PIN, LOW);
              lastWaitBlink = millis();
          }
          delay(100);
      }
      writeTelemetryLog("SCHEDULED_START", "Waking Up");
  }

  // 2. Daily Schedule Check
  if (useDailySchedule && !isWithinDailySchedule()) {
       handleDailySleep(); 
  }

  // --- READY TO RECORD ---
  set_arm_clock(recordClockMHz * 1000000);
  delay(10); 

  pinMode(ADC_POWER_PIN, OUTPUT);
  digitalWrite(ADC_POWER_PIN, HIGH); 
  delay(STABILIZATION_DELAY_MS); 

  AudioMemory(800); 

  if (audioMode == 'S') {
    queueLeft.begin(); queueRight.begin();
  } else if (audioMode == 'L') {
    queueLeft.begin();
  } else if (audioMode == 'R') {
    queueRight.begin();
  } else {
    audioMode = 'S';
    queueLeft.begin(); queueRight.begin();
  }
  
  AudioInterrupts();
  
  sessionStartMillis = millis();
  startNewFile();
  nukeUsbMagic();
  
  lastBattCheckMillis = millis(); 
}

void loop() {
  if (digitalRead(SHUTDOWN_PIN) == LOW) {
      closeCurrentFile();
      writeTelemetryLog("HALT_USER", "Shutdown Button Pressed");
      enterLowPowerHalt(); 
      while(1) { errorFlash(1); } 
  }

  unsigned long currentMillis = millis();

  if (currentMillis - lastBattCheckMillis > 60000) {
     char voltStr[10];
     float volts = readBatteryVoltage(); 
     dtostrf(volts, 4, 2, voltStr);
     if (volts > 1.0 && volts < LOW_BAT_THRESHOLD) {
        closeCurrentFile();
        writeTelemetryLog("HALT_LOW_BATT", voltStr);
        enterLowPowerHalt();
        while(1) { errorFlash(2); }
     }
     lastBattCheckMillis = currentMillis;
  }
  
  handleRecordLed();

  if (audioMode == 'S') {
    if (queueLeft.available() >= 1 && queueRight.available() >= 1) processAudioStereo();
  } else if (audioMode == 'L') {
    if (queueLeft.available() >= 1) processAudioMonoL();
  } else if (audioMode == 'R') {
    if (queueRight.available() >= 1) processAudioMonoR();
  }
  
  unsigned long elapsed = millis() - sessionStartMillis;
  if (elapsed >= recordTimeMillis) {
      handleFileEnd();
  }
}

// === VOLTAGE SMOOTHING ===
float readBatteryVoltage() {
    analogRead(BATT_PIN); 
    delay(2);
    float sum = 0;
    for(int i = 0; i < 5; i++) {
        sum += analogRead(BATT_PIN);
        delay(2);
    }
    return (sum / 5.0) * (3.3 / 1023.0) * VOLTAGE_DIVIDER_RATIO;
}

// === DAILY SCHEDULE LOGIC ===

bool isWithinDailySchedule() {
    int currentHHMM = (hour() * 100) + minute();

    if (dailyStartHHMM < dailyStopHHMM) {
        return (currentHHMM >= dailyStartHHMM && currentHHMM < dailyStopHHMM);
    } else {
        return (currentHHMM >= dailyStartHHMM || currentHHMM < dailyStopHHMM);
    }
}

void handleDailySleep() {
    writeTelemetryLog("DAILY_SLEEP", "Outside recording window");
    AudioNoInterrupts();
    queueLeft.clear();
    queueRight.clear();
    
    digitalWrite(ADC_POWER_PIN, LOW); 
    
    pinMode(PIN_I2S_TX, OUTPUT);    digitalWrite(PIN_I2S_TX, LOW);
    pinMode(PIN_I2S_MCLK, OUTPUT);  digitalWrite(PIN_I2S_MCLK, LOW);
    pinMode(PIN_I2S_RX, OUTPUT);    digitalWrite(PIN_I2S_RX, LOW);
    pinMode(PIN_I2S_BCLK, OUTPUT);  digitalWrite(PIN_I2S_BCLK, LOW);
    pinMode(PIN_I2S_LRCLK, OUTPUT); digitalWrite(PIN_I2S_LRCLK, LOW);

    set_arm_clock(WAIT_CLOCK_FREQ);
    delay(10); 
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    unsigned long lastWaitBlink = millis();

    while (!isWithinDailySchedule()) {
        if (digitalRead(SHUTDOWN_PIN) == LOW) {
            writeTelemetryLog("HALT_USER", "Shutdown during Daily Sleep");
            enterLowPowerHalt(); 
            while(1) { errorFlash(1); } 
        }
        
        if (millis() - lastWaitBlink > 10000) {
            digitalWrite(LED_PIN, HIGH); delay(10); digitalWrite(LED_PIN, LOW);
            lastWaitBlink = millis();
        }
        delay(100); 
    }
    
    writeTelemetryLog("DAILY_WAKE", "Window opened, rebooting");
    delay(100); 
    WRITE_RESTART(0x05FA0004); 
    while(1);
}

// === FILE & POWER UTILS ===

void enterLowPowerHalt() {
    AudioNoInterrupts();
    digitalWrite(ADC_POWER_PIN, LOW); 
    set_arm_clock(WAIT_CLOCK_FREQ);   
    delay(10);
    pinMode(LED_PIN, OUTPUT);
}

void handleFileEnd() {
  char voltStr[10];
  float endVoltage = readBatteryVoltage(); 
  dtostrf(endVoltage, 4, 2, voltStr);
  
  closeCurrentFile();

  char logMsg[64];
  sprintf(logMsg, "Volts: %s, Errors: %lu", voltStr, sdWriteErrors);
  writeTelemetryLog(filename, logMsg);
  
  sdWriteErrors = 0;

  uint32_t freeClusters = sd.freeClusterCount();
  float mbytes = (float)freeClusters * sd.vol()->sectorsPerCluster() * 0.000512;
  if (mbytes < 100) {
      writeTelemetryLog("HALT_SD_FULL", "0");
      enterLowPowerHalt();
      while(true) errorFlash(6);
  }
  
  if (useDailySchedule && !isWithinDailySchedule()) {
      handleDailySleep(); 
  } else if (waitTimeMillis > 0) {
      enterWaitModeAndReset();
  } else {
      startNewFile();
      sessionStartMillis = millis();
  }
}

// === CONFIG PARSER ===
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

void readConfigFile() {
  FsFile configFile = sd.open("TUFR_Setup.txt", O_READ);
  if (!configFile) return; 

  char lineBuffer[512];

  if (readNextLine(configFile, lineBuffer, 512)) {
    int r = 1, w = 0;
    int count = sscanf(lineBuffer, "%d %d", &r, &w);
    if (count >= 1) recordTimeMillis = (unsigned long)r * 60000;
    if (count >= 2) {
        unsigned long w_ms = (unsigned long)w * 60000;
        if (w_ms >= STABILIZATION_DELAY_MS) {
            w_ms -= STABILIZATION_DELAY_MS;
        } else {
            w_ms = 0;
        }
        waitTimeMillis = w_ms;
    }
  }

  if (readNextLine(configFile, lineBuffer, 512)) {
    int ck = 60;
    if (sscanf(lineBuffer, "%d", &ck) == 1) {
      if (ck < 24) ck = 24;
      recordClockMHz = ck;
    }
  }

  if (readNextLine(configFile, lineBuffer, 512)) {
    char m = toupper(lineBuffer[0]);
    if (m == 'S' || m == 'L' || m == 'R') {
      audioMode = m;
    }
  }
  
  if (readNextLine(configFile, lineBuffer, 512)) {
    int yr, mo, dy, hr, mn, sc;
    if (sscanf(lineBuffer, "%d %d %d %d %d %d", &yr, &mo, &dy, &hr, &mn, &sc) == 6) {
        tmElements_t tm;
        tm.Year = yr - 1970;
        tm.Month = mo;
        tm.Day = dy;
        tm.Hour = hr;
        tm.Minute = mn;
        tm.Second = sc;
        time_t setupTime = makeTime(tm);
        
        if (setupTime > Teensy3Clock.get()) {
            Teensy3Clock.set(setupTime);
            setTime(setupTime);
            writeTelemetryLog("RTC_SYNC", "Updated from Config");
        }
    }
  }

  if (readNextLine(configFile, lineBuffer, 512)) {
    int yr, mo, dy, hr, mn, sc;
    if (sscanf(lineBuffer, "%d %d %d %d %d %d", &yr, &mo, &dy, &hr, &mn, &sc) == 6) {
        if (yr == 0) {
            scheduledStartUnix = 0;
        } else {
            tmElements_t tm;
            tm.Year = yr - 1970;
            tm.Month = mo;
            tm.Day = dy;
            tm.Hour = hr;
            tm.Minute = mn;
            tm.Second = sc;
            scheduledStartUnix = makeTime(tm);
        }
    }
  }

  if (readNextLine(configFile, lineBuffer, 512)) {
      int startH, stopH;
      if (sscanf(lineBuffer, "%d %d", &startH, &stopH) == 2) {
          dailyStartHHMM = startH;
          dailyStopHHMM = stopH;
          if (!(dailyStartHHMM == 0 && dailyStopHHMM == 2400)) {
              useDailySchedule = true;
          }
      }
  }
  
  configFile.close();
}

// === AUDIO PROCESSING ===

void processAudioStereo() {
  int16_t *bufferL = queueLeft.readBuffer();
  int16_t *bufferR = queueRight.readBuffer();
  int16_t interleaved[256]; 
  for (int i = 0; i < 128; i++) {
    interleaved[i * 2] = bufferL[i];
    interleaved[i * 2 + 1] = bufferR[i];
  }
  
  size_t written = frec.write((byte*)interleaved, 512); 
  if (written != 512) sdWriteErrors++;
  else bytesWrittenInCurrentFile += 512;
  
  queueLeft.freeBuffer();
  queueRight.freeBuffer();
}

void processAudioMonoL() {
  int16_t *bufferL = queueLeft.readBuffer();
  size_t written = frec.write((byte*)bufferL, 256); 
  if (written != 256) sdWriteErrors++;
  else bytesWrittenInCurrentFile += 256;
  queueLeft.freeBuffer();
}

void processAudioMonoR() {
  int16_t *bufferR = queueRight.readBuffer();
  size_t written = frec.write((byte*)bufferR, 256);
  if (written != 256) sdWriteErrors++;
  else bytesWrittenInCurrentFile += 256;
  queueRight.freeBuffer();
}

void handleRecordLed() {
    unsigned long now = millis();
    if (!isLedOn && (now - lastRecBlink > 3000)) {
        digitalWrite(LED_PIN, HIGH);
        isLedOn = true;
        ledOnStartTime = now;
        lastRecBlink = now;
    }
    if (isLedOn && (now - ledOnStartTime > 10)) {
        digitalWrite(LED_PIN, LOW);
        isLedOn = false;
    }
}

void nukeUsbMagic() {
  USBPHY1_CTRL &= ~USBPHY_CTRL_ENAUTOCLR_CLKGATE; 
  USBPHY1_CTRL |= USBPHY_CTRL_CLKGATE;            
  USBPHY1_PWD = 0xFFFFFFFF;                       
  USBPHY2_CTRL &= ~USBPHY_CTRL_ENAUTOCLR_CLKGATE;
  USBPHY2_CTRL |= USBPHY_CTRL_CLKGATE;
  USBPHY2_PWD = 0xFFFFFFFF;
  CCM_CCGR1 &= ~CCM_CCGR1_ENET(CCM_CCGR_ON);
}

void enterWaitModeAndReset() {
    writeTelemetryLog("WAIT_START", "Entering Duty Cycle Sleep");
    AudioNoInterrupts();
    queueLeft.clear();
    queueRight.clear();
    
    digitalWrite(ADC_POWER_PIN, LOW); 
    
    pinMode(PIN_I2S_TX, OUTPUT);    digitalWrite(PIN_I2S_TX, LOW);
    pinMode(PIN_I2S_MCLK, OUTPUT);  digitalWrite(PIN_I2S_MCLK, LOW);
    pinMode(PIN_I2S_RX, OUTPUT);    digitalWrite(PIN_I2S_RX, LOW);
    pinMode(PIN_I2S_BCLK, OUTPUT);  digitalWrite(PIN_I2S_BCLK, LOW);
    pinMode(PIN_I2S_LRCLK, OUTPUT); digitalWrite(PIN_I2S_LRCLK, LOW);

    set_arm_clock(WAIT_CLOCK_FREQ);
    delay(10); 
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    unsigned long startWait = millis();
    unsigned long lastWaitBlink = millis();

    while (millis() - startWait < waitTimeMillis) {
        if (digitalRead(SHUTDOWN_PIN) == LOW) {
            writeTelemetryLog("HALT_USER", "Shutdown during Duty Cycle");
            enterLowPowerHalt(); 
            while(1) { errorFlash(1); } 
        }
        
        if (millis() - lastWaitBlink > 10000) {
            digitalWrite(LED_PIN, HIGH); delay(10); digitalWrite(LED_PIN, LOW);
            lastWaitBlink = millis();
        }
        delay(100);
    }
    
    writeTelemetryLog("WAIT_END_RESET", "Rebooting for next cycle");
    delay(100); 
    WRITE_RESTART(0x05FA0004);
    while(1);
}

void logResetReason() {
  uint16_t status = SRC_SRSR; 
  const char* reason = "UNKNOWN";
  if (status & 0x01) reason = "Cold Boot";   
  else if (status & 0x10) reason = "Warm Reboot"; 
  writeTelemetryLog("RESET_SOURCE", reason);
}

void startNewFile() {
  sprintf(filename, "TUFR_%04d%02d%02d_%02d%02d%02d.WAV", year(), month(), day(), hour(), minute(), second());
  frec = sd.open(filename, O_RDWR | O_CREAT | O_TRUNC);
  if (frec) {
    writeWavHeader(frec, 0);
    bytesWrittenInCurrentFile = 0;
  } else {
    triggerBadCardMode();
  }
}

void closeCurrentFile() {
  if (frec) {
    frec.seek(0);
    writeWavHeader(frec, bytesWrittenInCurrentFile);
    frec.close();
  }
}

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

void writeWavHeader(FsFile &file, uint32_t dataSize) {
  uint32_t fileSize = dataSize + 36;
  uint16_t numChannels = (audioMode == 'S') ? 2 : 1;
  uint16_t bitsPerSample = 16;
  uint32_t byteRate = SAMPLE_RATE * numChannels * (bitsPerSample / 8);
  uint16_t blockAlign = numChannels * (bitsPerSample / 8);
  
  uint8_t header[44];
  memcpy(header, "RIFF", 4); memcpy(header + 4, &fileSize, 4);
  memcpy(header + 8, "WAVEfmt ", 8);
  uint32_t subChunk1Size = 16; memcpy(header + 16, &subChunk1Size, 4);
  uint16_t audioFormat = 1; memcpy(header + 20, &audioFormat, 2);
  memcpy(header + 22, &numChannels, 2); memcpy(header + 24, &SAMPLE_RATE, 4);
  memcpy(header + 28, &byteRate, 4); memcpy(header + 32, &blockAlign, 2);
  memcpy(header + 34, &bitsPerSample, 2); memcpy(header + 36, "data", 4);
  memcpy(header + 40, &dataSize, 4);
  file.write(header, 44);
}

void triggerBadCardMode() {
  enterLowPowerHalt();
  while(1) { digitalWrite(LED_PIN, HIGH); delay(200); digitalWrite(LED_PIN, LOW); delay(200); }
}

void errorFlash(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH); delay(200); digitalWrite(LED_PIN, LOW); delay(200);
  }
  delay(1000);
}

time_t getTeensy3Time() { return Teensy3Clock.get(); }