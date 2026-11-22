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
 */
 
#include "Arduino.h"
#include "global.h"

#if MCU==T_4_1
  #include <CrashReport.h>
  #include "teensy.h"
  #include "adc.h"
  #include "rtc.h"
  #include "process.h"
  // TUFR ground-truth testing: adc.cxx's own adc_init()/adc_exit() pick
  // channel count via the compile-time NCHAN_I2S macro, so they can't
  // take a runtime channel count from TUFR_Setup.txt. adcRuntime.cpp is
  // TUFR's existing, already-verified (register-for-register match
  // against adc.cxx) runtime equivalent - see its own file header - used
  // below in place of adc_init()/adc_exit() for exactly that reason.
  // adc.h is left included too (harmless - setAGain()/adcStatus() are
  // still available if wanted), just no longer used for init/shutdown.
  #include "adcRuntime.h"
  #include "TUFR_Config.h"

    /*******************************************************************************/
  uint32_t getPSRAMSize(void) {return 0;}

  void printCrashReport(void)
  {   // Teensy has a crash report
      if(CrashReport) Serial.print(CrashReport);
  }

    // use usb host 5V power (has 100uF capacitor)
    void usbPowerInit()
    {
      IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40 = 5;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40 = 0x0008; // slow speed, weak 150 ohm drive
      GPIO8_GDIR |= 1<<26;
    }
    void usbPowerExit()
    {
      IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40 = 0; // disable
      GPIO8_GDIR &= ~(1<<26);
    }

    void usbPowerOn()  { GPIO8_DR_SET = 1<<26; }
    void usbPowerOff() { GPIO8_DR_CLEAR = 1<<26; }

    void usbPowerSetup(void)
    {
      usbPowerInit();
      usbPowerOn();
      delay(100);
    }

  /****************misc**********************************************************/
  extern "C" uint32_t set_arm_clock(uint32_t frequency); // clockspeed.c
  void set_MCU_clock(int32_t mcu_factor)
  {
    set_arm_clock(mcu_factor*12'000'000);
  }

  void lowPowerInit(void)
  { // keep memory powered during sleep
    CCM_CGPR |= CCM_CGPR_INT_MEM_CLK_LPM;
    // keep cpu clock on in wait mode (required for systick to trigger wake-up)
    CCM_CLPCR &= ~(CCM_CLPCR_ARM_CLK_DIS_ON_LPM | CCM_CLPCR_LPM(3));
    // set SoC low power mode to wait mode
    CCM_CLPCR |= CCM_CLPCR_LPM(1);
    // ensure above config is done before executing WFI
    asm volatile("dsb");
  }

/************************************ADC***************************************************/

  #define MSYNC   1
  #define MDIV    1      // MCLK divider (MCLK = 2*MDIV*BCLK)

  void process(int32_t * buffer);
  static uint16_t acq_isrunning=0;

  PROGMEM
  void set_audioClock(int nfact, int32_t nmult, uint32_t ndiv) // sets PLL4
  {
    CCM_ANALOG_PLL_AUDIO = CCM_ANALOG_PLL_AUDIO_BYPASS | CCM_ANALOG_PLL_AUDIO_ENABLE
            | CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(2) // 2: 1/4; 1: 1/2; 0: 1/1
            | CCM_ANALOG_PLL_AUDIO_DIV_SELECT(nfact);

    CCM_ANALOG_PLL_AUDIO_NUM   = nmult & CCM_ANALOG_PLL_AUDIO_NUM_MASK;
    CCM_ANALOG_PLL_AUDIO_DENOM = ndiv & CCM_ANALOG_PLL_AUDIO_DENOM_MASK;
    
    CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_POWERDOWN;  //Switch on PLL
    while (!(CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_LOCK)) {}; //Wait for pll-lock
    
    const int div_post_pll = 1; // other values: 2,4
    CCM_ANALOG_MISC2 &= ~(CCM_ANALOG_MISC2_DIV_MSB | CCM_ANALOG_MISC2_DIV_LSB);
    if(div_post_pll>1) CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_LSB;
    if(div_post_pll>3) CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_MSB;
    
    CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_BYPASS;   //Disable Bypass
  }

  // gRxNchan: runtime channel count, replacing the compile-time NCHAN_I2S
  // macro in the two places that actually need it (here, and RCR4 below).
  // Set by i2s_setup() before every call into setAudioFrequency() -
  // including the one from acqModifyFrequency(), which has no nchan
  // parameter of its own and so relies on this having already been set.
  static uint8_t gRxNchan = 4;
  static int8_t  gAdcGain = 0;

  void setAudioFrequency(int fs)
  { if(!acq_isrunning) return; // change frequency only when i2s is running

    // FIX (TUFR ground-truth testing, see TUFR_4.1.ino): ovr was
    // hardcoded to 128 here, described only as "from ADC manual." Working
    // backward through this function's math (see below) shows ovr is not
    // an ADC-wide constant at all - it IS the BCLK:FSYNC ratio this
    // function programs, which for TDM is just nchan*bits-per-slot by
    // definition (that's what "how many bit-clocks per frame sync" means).
    // 128 happens to be the right value ONLY for this driver's shipped
    // default (NCHAN_I2S==4, MBIT==32: 4*32=128) - the exact config this
    // was presumably last tuned/verified against. For any other channel
    // count it silently produces the wrong BCLK, and because the error is
    // a clean integer ratio (128 / actual_ratio), the symptom is a
    // perfectly-formed recording at the wrong pitch/rate rather than
    // obvious noise - e.g. at NCHAN_I2S==2 (actual ratio 64) this produced
    // real audio at exactly 2x the requested sample rate, confirmed
    // empirically via a 10kHz test tone recording as 5kHz.
    //
    // n1/n2/C below convert the target BCLK ratio (ovr) into the PLL4
    // fractional-multiply (c0/c1/c2, via set_audioClock) and integer
    // post-divider (CCM_CS1CDR PRED/PODF) needed to hit it; that math
    // doesn't care what ovr represents, so making ovr correct here is the
    // only change needed - everything below is unmodified from the
    // original, including the n1/2 halving (which is a real, necessary
    // part of getting from the doubled prescaler used to compute C down
    // to the PRED register's actual divide-by-2-less value in TDM mode,
    // not a channel-count hack).
    // FURTHER CHANGE (incorporating TUFR_Setup.txt): NCHAN_I2S was a
    // compile-time constant; TUFR needs channel count chosen at boot, so
    // this now reads the runtime value i2s_setup() stashed in gRxNchan
    // instead. Everything else about the fix above (ovr = nchan*MBIT) is
    // unchanged.
    int ovr = gRxNchan * MBIT;  // BCLK:FSYNC ratio = TDM slots * bits/slot
    int n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
    int n2 = 1 + (24000000 * 27) / (fs * ovr * n1);
    Serial.printf("fs=%d, ovr=%d, n1=%d, n2=%d\r\n", fs, ovr, n1, n2);

    double C = ((double)fs * ovr * n1 * n2) / 24000000;
    Serial.print("C: "); Serial.println(C);

    int c0 = C;
    int c2 = 10'000;
    int c1 = C * c2 - (c0 * c2);
    set_audioClock(c0, c1, c2);

  	n1 = n1 / 2; // halves the C-computation prescaler down to the PRED
                 // register's actual divide ratio in TDM mode - unrelated
                 // to the ovr fix above, kept as-is (see comment above)

      // clear SAI1_CLK register locations
    CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI1_CLK_SEL_MASK))
        | CCM_CSCMR1_SAI1_CLK_SEL(2); // &0x03 // (0,1,2): PLL3PFD0, PLL5, PLL4
    CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
        | CCM_CS1CDR_SAI1_CLK_PRED(n1-1) // &0x07
        | CCM_CS1CDR_SAI1_CLK_PODF(n2-1); // &0x3f
    // Select MCLK
    IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1
      & ~(IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL_MASK))
      | (IOMUXC_GPR_GPR1_SAI1_MCLK_DIR | IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL(0));

  }

  void i2s_setup(uint32_t fsamp, uint8_t nchan)
  {
    CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON);

    // if receiver is enabled, do nothing
    if (I2S1_RCSR & I2S_RCSR_RE) return;
    //PLL:
    int fs = fsamp;

    // Runtime channel count (TUFR_Setup.txt), replacing the compile-time
    // NCHAN_I2S macro - see gRxNchan's declaration above setAudioFrequency().
    gRxNchan = nchan;

    acq_isrunning=1;
    setAudioFrequency(fs);

    CORE_PIN23_CONFIG = 3;  //1:MCLK
    CORE_PIN21_CONFIG = 3;  //1:RX_BCLK
    CORE_PIN20_CONFIG = 3;  //1:RX_SYNC

  	CORE_PIN8_CONFIG  = 3;  //1:RX_DATA0
  	IOMUXC_SAI1_RX_DATA0_SELECT_INPUT = 2;

    I2S1_RMR = 0;
    //I2S1_RCSR = (1<<25); //Reset
    I2S1_RCR1 = I2S_RCR1_RFW(4);
    I2S1_RCR2 = I2S_RCR2_SYNC(0) //| I2S_RCR2_BCP  
              | (I2S_RCR2_BCD | I2S_RCR2_DIV((MDIV-1)) | I2S_RCR2_MSEL(1));
    I2S1_RCR3 = I2S_RCR3_RCE;
    I2S1_RCR4 = I2S_RCR4_FRSZ((nchan-1)) | I2S_RCR4_SYWD((MSYNC-1)) | I2S_RCR4_MF
              | I2S_RCR4_FSE | I2S_RCR4_FSP | I2S_RCR4_FSD;
    I2S1_RCR5 = I2S_RCR5_WNW((MBIT-1)) | I2S_RCR5_W0W((MBIT-1)) | I2S_RCR5_FBT((MBIT-1));

    I2S1_RCSR = I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE | I2S_RCSR_FR;
  }

  /******************************************************************************************/
  #include <DMAChannel.h>

  static DMAChannel dma;
  DMAMEM  __attribute__((aligned(32))) static  uint32_t i2s_buffer[2*NBUF_I2S];
  
  static void acq_isr(void);

  void dma_setup(void)
  {
    dma.begin(true); // Allocate the DMA channel first

    dma.TCD->SADDR = (void *)((uint32_t)&I2S1_RDR0);
    dma.TCD->SOFF = 0;
    dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE((MBIT/16)) | DMA_TCD_ATTR_DSIZE((MBIT/16));
    dma.TCD->NBYTES_MLNO = (MBIT/8);
    dma.TCD->SLAST = 0;
    dma.TCD->DADDR = i2s_buffer;
    dma.TCD->DOFF = (MBIT/8);
    dma.TCD->CITER_ELINKNO = 2*NBUF_I2S;
    dma.TCD->DLASTSGA = -sizeof(i2s_buffer);
    dma.TCD->BITER_ELINKNO = dma.TCD->CITER_ELINKNO;
    dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;

    dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_RX);

    dma.attachInterrupt(acq_isr, 0x60);	
    dma.enable();
  }

  DMAMEM int32_t acq_buffer[NBUF_I2S];

  #define IMXRT_CACHE_ENABLED 2 // 0=disabled, 1=WT, 2= WB
  static void acq_isr(void)
  {
    uint32_t daddr;
    int32_t *src;

    daddr = (uint32_t)(dma.TCD->DADDR);
    dma.clearInterrupt();

    if (daddr < (uint32_t) &i2s_buffer[NBUF_I2S]) 
    {
      // DMA is receiving to the first half of the buffer
      // need to remove data from the second half
      src = (int32_t *)&i2s_buffer[NBUF_I2S];
    }
    else
    {
    // DMA is receiving to the second half of the buffer
    // need to remove data from the first half
      src = (int32_t *)&i2s_buffer[0];
    }
    #if IMXRT_CACHE_ENABLED >=1
        arm_dcache_delete((void*)src, sizeof(i2s_buffer) / 2);
    #endif

    //process(src);
    memcpy(acq_buffer,(const void *)src,4*NBUF_I2S);
    process(acq_buffer);
  }

  /*-------------Utilities----------------------------*/
  void acqModifyFrequency(uint32_t fsamp)
  {
    // stop I2S
    I2S1_RCSR &= ~(I2S_RCSR_RE | I2S_RCSR_BCE);
    setAudioFrequency(fsamp);
    //restart I2S
    I2S1_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE;
  }

  void i2s_start(void){  I2S1_RCSR |=  (I2S_RCSR_RE | I2S_RCSR_BCE);}
  void i2s_stop(void){   I2S1_RCSR &= ~(I2S_RCSR_RE | I2S_RCSR_BCE);}

  void acqInit(uint32_t fs, uint8_t nchan, int8_t gainVal)
  {
    gAdcGain = gainVal; // stashed for acqStart()'s adcRuntimeInit() call below
    i2s_setup(fs, nchan);
    dma_setup();
  }

  void acqStart(void)
  {
    i2s_start();
    // adc_init() (adc.cxx, compile-time NCHAN_I2S) replaced with TUFR's
    // adcRuntimeInit() (runtime nchan/gain) - see the adcRuntime.h include
    // above for why. gRxNchan/gAdcGain were set by acqInit()/i2s_setup().
    if (!adcRuntimeInit(gRxNchan, gAdcGain, ADC_EN, ADC_SHDNZ)) {
      Serial.print("ADC init FAILED, I2CErr="); Serial.println(adcRuntimeLastI2CError());
    }
  }

  void acqStop(void)
  {
    i2s_stop();
    adcRuntimeShutdown(ADC_EN, ADC_SHDNZ); // was adc_exit() - see acqStart()
  }

  void dma_exit(void){}


    /*---------------hibernate -------------------*/
   void doReset(void)  { *(uint32_t *)0xE000ED0C =  0x5FA0004; }
   void doReboot(void) { *(uint32_t *)0xE000ED0C =  0x5FA0004;}
    void powerDown(void)
    {
      SNVS_LPCR |= (1 << 6); // turn off power
      while (1) asm("wfi");      
    }

    void stopSystem(void)
    { //shutting down power
    //stopSD();
    adcRuntimeShutdown(ADC_EN, ADC_SHDNZ); // was adc_exit() - see acqStart()
    //stopUSB();
    }

    #define SNVS_LPCR_LPTA_EN_MASK    (0x2U)

    void doHibernate(uint32_t secs)
    {
        uint32_t tmp = SNVS_LPCR; // save control register
        SNVS_LPSR |= 1;
        asm volatile("DSB");

        // disable alarm
        SNVS_LPCR &= ~SNVS_LPCR_LPTA_EN_MASK;
        while (SNVS_LPCR & SNVS_LPCR_LPTA_EN_MASK);

        __disable_irq();

        //get Time:
        uint32_t lsb, msb;
        do {
          msb = SNVS_LPSRTCMR;
          lsb = SNVS_LPSRTCLR;
        } while ( (SNVS_LPSRTCLR != lsb) | (SNVS_LPSRTCMR != msb) );
        uint32_t secso = (msb << 17) | (lsb >> 15);

        // if alarm is not in future do not hibernate
        if(secs <= secso) return;

        //set alarm
        SNVS_LPTAR = secs;
        while (SNVS_LPTAR != secs);

        // restore control register and set alarm
        SNVS_LPCR = tmp | SNVS_LPCR_LPTA_EN_MASK; 
        while (!(SNVS_LPCR & SNVS_LPCR_LPTA_EN_MASK));

    //    NVIC_CLEAR_PENDING(IRQ_SNVS_ONOFF);
    //    attachInterruptVector(IRQ_SNVS_ONOFF, &call_back);
    //    NVIC_SET_PRIORITY(IRQ_SNVS_ONOFF, 255); //lowest priority
    //    asm volatile ("dsb"); //make sure to write before interrupt-enable
    //    NVIC_ENABLE_IRQ(IRQ_SNVS_ONOFF);
        __enable_irq();
      
        SNVS_LPCR |= (1 << 6); // turn off power
        while (1) asm("wfi");  
    }


  extern char startTime[];

    void setAlarm(uint32_t secs)
    {
        uint32_t tmp = SNVS_LPCR; // save control register
        SNVS_LPSR |= 1;
        asm volatile("DSB");

        // disable alarm
        SNVS_LPCR &= ~SNVS_LPCR_LPTA_EN_MASK;
        while (SNVS_LPCR & SNVS_LPCR_LPTA_EN_MASK);

        __disable_irq();
        //set alarm
        SNVS_LPTAR = secs;
        while (SNVS_LPTAR != secs);

        // restore control register and set alarm
        SNVS_LPCR = tmp | SNVS_LPCR_LPTA_EN_MASK; 
        while (!(SNVS_LPCR & SNVS_LPCR_LPTA_EN_MASK));
        __enable_irq();

      datetime_t tm;
      time2date(secs, &tm, 1970);
      encodeTimestamp(startTime,&tm);
    }

   void goDormant(void) 
    {
      powerDown();
    }

  void rtc_init(void) {}
  bool rtc_is_running(void) {return 0;}

  bool rtc_get_datetime(datetime_t *t)
  {
    time2date(rtc_get(), t, 1970);
    return 1;
  }

  bool rtc_set_datetime(const datetime_t *t)
  {
    rtc_set(date2time(t,1970));
    return 1;
  }

  char uid_strng[10]; 
  void getUID(void) { sprintf(uid_strng,"%08lX",(HW_OCOTP_MAC0 & 0xFFFFFFFF)); }

  void neo_pixel_show(uint16_t r, uint16_t g, uint16_t b) {}

  void systemInit(void)
  { lowPowerInit();
  }
#endif