/* microPAM
 * Copyright (c) 2026, Walter Zimmer
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

#ifndef PROCESS_H
#define PROCESS_H

// Lock-free single-producer / single-consumer ring of 16-bit PCM blocks.
// push() runs in the DMA interrupt and stores the top 16 bits of each 32-bit
// TDM word (that is all the WAV files keep). The main loop reads a finished
// block IN PLACE with peek(), writes it to the SD card, then calls release().
// Nothing is copied with interrupts off, and the interrupt never waits for
// the main loop. When the ring is full only the newest incoming DMA block is
// dropped (acq_missed++), not the whole queue as in Walter's original.
class Queue
{ volatile int16_t head, tail;   // head: slot being filled (ISR); tail: oldest finished slot (main loop)
  int32_t cnt;                   // samples already stored in slot "head"

  public:
  Queue() {reset();}

  void reset(void);              // only with capture stopped
  int push(uint32_t *data, int ndat);
  int16_t* peek(void);           // oldest finished block (NBUF_DISK samples) or NULL
  void release(void);            // done with the block peek() returned
  int available(void);           // number of finished blocks waiting
};

extern Queue queue;
extern bool queue_decimate;   // true: push() low-pass filters and halves the sample rate (setup rate -768)

void process_init(void);
void process(int32_t *buffer);

#if defined(USE_SYNTH_SIGNAL) && defined(__IMXRT1062__)
  void synth_signal_fill(float fsamp);
#endif

extern uint32_t acq_missed;
extern uint32_t acq_missAt[16];
extern uint32_t acq_count;
extern uint32_t process_max_us;

#if MCU==T_4_1
  extern uint32_t dsp_isr_max_us;
  void dsp_init(void);
  void dsp_trigger(int32_t *acq_buffer);
  int32_t *spectrum_power(int32_t *buffer);
  int32_t *dsp_apply(int32_t *buffer);

  extern float Imax;
  extern float Dmax;
  extern float Dmean;
  extern float Dsnr;
  extern float Dpeak;
#else
  // On RP targets DSP is not compiled; provide zero constants so shared code
  // referencing these symbols compiles without changes and emits no linker symbols.
  static inline void dsp_init(void) {}
  static inline void dsp_trigger(int32_t *) {}
  static inline int32_t *spectrum_power(int32_t *b) { return b; }
  static inline int32_t *dsp_apply(int32_t *b) { return b; }
  #define Imax  0.0f
  #define Dmax  0.0f
  #define Dmean 0.0f
  #define Dsnr  0.0f
  #define Dpeak 0.0f
#endif

#endif // PROCESS_H
