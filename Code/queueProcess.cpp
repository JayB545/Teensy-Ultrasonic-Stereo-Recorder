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

/*
 * queueProcess.cpp
 *
 * The Queue class implementation below started as a verbatim copy of
 * Walter's own process.cxx (its "Queue" section) - see the FIX note below
 * for the one deliberate change since made to it, found and applied during
 * TUFR's own bench testing, not part of the original. process() below is a
 * deliberately trimmed copy of process.cxx's process(): only the
 * PROC_MODE==0 branch (raw passthrough - queue.push(acq_buffer), no
 * compression/DSP/classifier) is kept, since PROC_MODE is fixed to 0 in
 * this test's global.h and the other branches pull in process.cxx's
 * compressData()/dsp_trigger()/classifier.h, which aren't needed to
 * answer the one question this test asks: is the raw TDM signal path
 * (SAI/DMA/ADC) clean at 2ch/384kHz.
 *
 * 2026-11-04 REWRITE of the Queue (this file and process.h): blocks are now
 * 16-bit PCM (push() keeps the top 16 bits of each 32-bit TDM word), the main
 * loop reads a finished block in place (peek()/release()) instead of copying
 * 64 KB with interrupts off, and a full ring drops only the newest DMA block.
 * Producer and consumer never touch the same slot, so no interrupt masking or
 * lock is needed (the earlier spin-lock deadlock and the noInterrupts() copy
 * that stole SysTick ticks are both gone). Sized by MAX_QUEUE in global.h.
 */
#include <Arduino.h>
#include "global.h"
#include "process.h"

#define NBLOCK NBUF_DISK
// 16-bit blocks: a slot is 32 KB = about 21 ms of 384 kHz x 2 ch audio (the
// same time span a 64 KB raw 32-bit slot held), so the same RAM2 holds twice
// the audio. MAX_QUEUE (global.h) slots, one of them always being filled.
DMAMEM int16_t queue_buffer[MAX_QUEUE][NBLOCK];

static_assert(NBLOCK % NBUF_I2S == 0, "a queue block must hold a whole number of DMA blocks");
static_assert(NBLOCK % (NBUF_I2S / 2) == 0, "a queue block must hold a whole number of decimated DMA blocks");

#define INC(x) (((x) + 1) % MAX_QUEUE)

// ---- Optional 2:1 decimation (setup rate "-768": ADC at 768 kHz, 1 channel,
// file written at 384 kHz). Runs in the DMA interrupt inside push(). A 43-tap
// linear-phase half-band FIR (Kaiser, beta 7): flat to 150 kHz (ripple < 0.01 dB),
// at least 70 dB down above 234 kHz, so what the ADC passes between 192 and
// 345 kHz (including its high-frequency noise hump near 240 kHz) is removed
// instead of folding into the band. Every other tap is zero, so only 12
// multiplies per output sample. Q15 coefficients; the filter history carries
// across blocks and files, so there is no seam. Designed with scipy
// (see TUFR_hpf notes in the README).
bool queue_decimate = false;
#define DEC_N     43
#define DEC_HIST  (DEC_N - 1)
#define DEC_C     ((DEC_N - 1) / 2)
static const int32_t DEC_CENTER = 16384;          // 0.5 in Q15
static const int32_t DEC_H[11] = {10354,-3253,1732,-1030,622,-366,203,-103,45,-16,3}; // taps at offsets 1,3,...,21 from the centre
static int16_t dec_x[DEC_HIST + NBUF_I2S];
static int16_t dec_out[NBUF_I2S / 2];
static_assert(NBUF_I2S % 2 == 0, "decimation needs an even DMA block");

static void decReset(void) {
  memset(dec_x, 0, sizeof(dec_x));
}

// Filters one DMA block (mono, 32-bit words, top 16 bits used); returns the
// number of output samples left in dec_out.
static int __not_in_flash_func(decimateBlock)(const uint32_t *data, int ndat) {
  int16_t *x = dec_x;
  for (int i = 0; i < ndat; i++) x[DEC_HIST + i] = (int16_t)(data[i] >> 16);
  int nout = 0;
  for (int s0 = 0; s0 < ndat; s0 += 2) {
    const int16_t *c = &x[s0 + DEC_C];
    int32_t acc = DEC_CENTER * c[0];
    for (int j = 0; j < 11; j++) {
      int k = 2 * j + 1;
      acc += DEC_H[j] * ((int32_t)c[-k] + (int32_t)c[k]);
    }
    acc = (acc + 16384) >> 15;
    if (acc > 32767) acc = 32767;
    if (acc < -32768) acc = -32768;
    dec_out[nout++] = (int16_t)acc;
  }
  memmove(x, x + ndat, DEC_HIST * sizeof(int16_t)); // keep the newest 42 inputs
  return nout;
}

void Queue::reset(void) {
  head = 0;
  tail = 0;
  cnt = 0;
  decReset();
}

int __not_in_flash_func(Queue::push)(uint32_t *data, int ndat) {
  // Producer only (DMA interrupt). Slot "head" is never slot "tail", so the
  // consumer never reads what is being written here.
  // With decimation the filter always sees every input block, even when the
  // ring is full and the output is dropped, so its history stays continuous.
  bool dec = queue_decimate;
  if (dec) ndat = decimateBlock(data, ndat);
  if (cnt >= NBLOCK) {                 // current slot is full: move on if a slot is free
    int16_t nh = INC(head);
    if (nh == tail) return 0;          // ring full: drop this DMA block only
    head = nh;
    cnt = 0;
  }
  int16_t *dst = &queue_buffer[head][cnt];
  if (dec) memcpy(dst, dec_out, ndat * sizeof(int16_t));
  else for (int ii = 0; ii < ndat; ii++) dst[ii] = (int16_t)(data[ii] >> 16);
  cnt += ndat;
  if (cnt >= NBLOCK) {                 // publish a finished block right away
    int16_t nh = INC(head);
    if (nh != tail) {
      asm volatile("dsb" ::: "memory");  // block contents visible before head moves
      head = nh;
      cnt = 0;
    }
  }
  return 1;
}

int16_t* __not_in_flash_func(Queue::peek)(void) {
  if (tail == head) return NULL;
  return queue_buffer[tail];
}

void __not_in_flash_func(Queue::release)(void) {
  if (tail != head) tail = INC(tail);
}

int __not_in_flash_func(Queue::available)(void) {
  return (head + MAX_QUEUE - tail) % MAX_QUEUE;
}

Queue queue;

// ---- process() : PROC_MODE 0 only (raw passthrough) ----
#define NBUF_ACQ NBUF_I2S

uint32_t acq_missed = 0;
uint32_t acq_missAt[16];   // acq_count (block number) of the first 16 misses since acq_missed was last reset
uint32_t acq_count  = 0;
uint32_t process_max_us = 0;

void __not_in_flash_func(process)(int32_t *acq_buffer) {
  acq_count++;
  uint32_t to = micros();

  if (!queue.push((uint32_t*)acq_buffer, NBUF_ACQ)) {
    if (acq_missed < 16) acq_missAt[acq_missed] = acq_count;
    acq_missed++;
  }

  uint32_t dt = micros() - to;
  if (dt > process_max_us) process_max_us = dt;
}

void process_init(void) {}
