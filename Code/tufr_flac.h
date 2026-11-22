// tufr_flac.h -- small static-memory FLAC encoder for TUFR (Teensy 4.1)
//
// 16-bit PCM, 1-8 channels, any sample rate up to 1,048,575 Hz.
// FIXED predictors (order 0-4, chosen per channel per frame), partitioned
// Rice coding (partition order 0-6, Rice parameter chosen per partition),
// CONSTANT subframes for digital silence, VERBATIM fallback.
// No malloc, no stdio, no libFLAC. Output is a normal (non-subset when
// fs > 655350 Hz) FLAC stream that libFLAC, ffmpeg, Audacity and PAMGuard
// decode.
//
// File layout written by this module:
//   [ 512-byte header: "fLaC" + STREAMINFO + PADDING ]  <- one SD sector
//   [ frames ... ]
// The header is exactly 512 bytes so every later 32 KB write stays
// sector-aligned. At file close, call tufr_flac_header() again and rewrite
// the first 512 bytes (same pattern as the WAV header rewrite). If the
// recorder loses power before close, the file still decodes: STREAMINFO
// then says "total samples unknown" and frame sizes 0, which is legal.
//
// Usage per file:
//   tufr_flac_t enc;
//   tufr_flac_init(&enc, fs, nch, 2048);
//   uint8_t hdr[TUFR_FLAC_HEADER_BYTES];
//   tufr_flac_header(&enc, hdr);  file.write(hdr, 512);
//   for each block of interleaved samples (multiple of blocksize frames,
//   except possibly the last):
//       n = tufr_flac_encode(&enc, samples, nframes, out, outCap);
//       append out[0..n) to the SD staging buffer
//   close: tufr_flac_header(&enc, hdr); seek(0); write(hdr,512)
//
// Copyright: written for the TUFR project, 2026. Free to use and modify.

#ifndef TUFR_FLAC_H
#define TUFR_FLAC_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TUFR_FLAC_MAX_BLOCK
#define TUFR_FLAC_MAX_BLOCK     2048   // samples per channel per frame (4096 saves ~0.3 % size, doubles RAM)
#endif
#define TUFR_FLAC_MAX_CH        4
#define TUFR_FLAC_HEADER_BYTES  512
#ifndef TUFR_FLAC_MAX_PORDER
#define TUFR_FLAC_MAX_PORDER    6      // 2^6 = 64 Rice partitions max (8 = finer, slower, no gain measured)
#endif

// Worst-case encoded bytes for one frame (verbatim + headers), use this to
// size output buffers: blocksize * nch * 2 + 64.
#define TUFR_FLAC_FRAME_MAX(bs, nch)  ((size_t)(bs) * (nch) * 2u + 64u)

typedef struct {
    uint32_t fs;
    uint8_t  nch;
    uint16_t blocksize;
    uint64_t total_samples;   // per channel, encoded so far
    uint32_t frame_number;
    uint32_t min_frame;       // bytes
    uint32_t max_frame;
    // per-frame statistics (optional, for the log)
    uint32_t n_constant, n_verbatim, n_fixed[5];
    uint32_t carry;           // samples waiting in the internal part-frame buffer
} tufr_flac_t;

// Output sink for the streaming API: return 1 on success, 0 to abort.
typedef int (*tufr_flac_emit_t)(const uint8_t *data, size_t len, void *ctx);

void   tufr_flac_init(tufr_flac_t *e, uint32_t fs, uint8_t nch, uint16_t blocksize);

// Writes exactly TUFR_FLAC_HEADER_BYTES bytes into hdr. Call once at file
// open (counts are zero = "unknown") and again at close for the final values.
void   tufr_flac_header(const tufr_flac_t *e, uint8_t *hdr);

// Encode nframes (<= e->blocksize) interleaved 16-bit frames as ONE FLAC
// frame. A frame shorter than blocksize is only allowed as the last frame
// of a file. Returns bytes written to out (0 on error).
size_t tufr_flac_encode_frame(tufr_flac_t *e, const int16_t *pcm,
                              uint32_t nframes, uint8_t *out, size_t outCap);

// Convenience: encode any number of frames as consecutive FLAC frames of
// blocksize (the last one may be short). Returns total bytes written, or 0
// if out would overflow (size out with TUFR_FLAC_FRAME_MAX per frame).
size_t tufr_flac_encode(tufr_flac_t *e, const int16_t *pcm,
                        uint32_t nframes, uint8_t *out, size_t outCap);

// Streaming API (what the sketch uses). Push any number of interleaved
// samples (need not be whole sample frames); every completed FLAC frame is
// encoded into 'frame' (TUFR_FLAC_FRAME_MAX bytes) and passed to emit().
// Samples that do not fill a whole FLAC frame wait in an internal buffer
// until the next push (3-channel recordings straddle the 32 KB queue blocks).
// Returns 0 if encoding or emit() failed.
int tufr_flac_push(tufr_flac_t *e, const int16_t *pcm, uint32_t nsamp,
                   uint8_t *frame, tufr_flac_emit_t emit, void *ctx);
// At file close: encode the whole sample frames that are left as a final,
// shorter FLAC frame. An incomplete sample frame (3-channel recordings: the
// last 1-2 samples) stays in the buffer and starts the next file, so channel
// order is never shifted across a join.
int tufr_flac_flush(tufr_flac_t *e, uint8_t *frame, tufr_flac_emit_t emit, void *ctx);
// Next file of the same gap-free stream: zero the per-file counters, keep the
// carried samples. (tufr_flac_init() starts a fresh stream and drops them.)
void tufr_flac_new_file(tufr_flac_t *e);
// Stream restarted inside a file (dead-stream recovery): pad the waiting
// part-frame with zeros to a full frame and encode it, so the restarted
// stream begins on channel 1 and the file keeps fixed-size frames.
int tufr_flac_pad_frame(tufr_flac_t *e, uint8_t *frame, tufr_flac_emit_t emit, void *ctx);

// RAM (all static, ordinary globals = Teensy DTCM): 4 * MAX_BLOCK (residual)
// + 2 * MAX_BLOCK * MAX_CH (part-frame) + 0.6 KB (partition sums) + 2.3 KB (CRC
// tables) = 26.9 KB at MAX_BLOCK 2048. Plus the caller's frame buffer
// (TUFR_FLAC_FRAME_MAX = 16.4 KB at 2048 x 4 ch) and its SD staging buffer.

#ifdef __cplusplus
}
#endif
#endif
