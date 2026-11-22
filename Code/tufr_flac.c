// tufr_flac.c -- see tufr_flac.h
#include "tufr_flac.h"
#include <string.h>

// The predictor order is chosen from every 4th block of 64 samples, spread
// over the frame (1 = every sample: same files, about 25 % slower).
#ifndef TUFR_FLAC_SELECT_DIV
#define TUFR_FLAC_SELECT_DIV 4
#endif

// ---------------------------------------------------------------- buffers
// On Teensy 4.x ordinary globals land in DTCM (RAM1, zero wait state),
// which is what we want for the inner loops. 32 KB total.
static int32_t  s_x[TUFR_FLAC_MAX_BLOCK];     // one channel, de-interleaved; then its folded residual (in place)
#define s_u ((uint32_t*)s_x)
static int16_t  s_carry[TUFR_FLAC_MAX_BLOCK * TUFR_FLAC_MAX_CH]; // partial frame between push() calls
static uint64_t s_psum[1u << TUFR_FLAC_MAX_PORDER];

static uint8_t  s_crc8[256];
static uint16_t s_crc16[4][256];   // slice-by-4 (2 KB)
static int      s_tables_ok = 0;

static void make_tables(void)
{
    for (int i = 0; i < 256; i++) {
        uint8_t c = (uint8_t)i;
        for (int b = 0; b < 8; b++) c = (uint8_t)((c & 0x80) ? (c << 1) ^ 0x07 : (c << 1));
        s_crc8[i] = c;
        uint16_t d = (uint16_t)(i << 8);
        for (int b = 0; b < 8; b++) d = (uint16_t)((d & 0x8000) ? (d << 1) ^ 0x8005 : (d << 1));
        s_crc16[0][i] = d;
    }
    // s_crc16[k][i] = CRC of byte i followed by k zero bytes
    for (int k = 1; k < 4; k++)
        for (int i = 0; i < 256; i++) {
            uint16_t p = s_crc16[k - 1][i];
            s_crc16[k][i] = (uint16_t)((p << 8) ^ s_crc16[0][p >> 8]);
        }
    s_tables_ok = 1;
}

// CRC-16 (poly 0x8005, MSB first), 4 bytes per step
static uint16_t crc16(const uint8_t *q, size_t n)
{
    uint32_t c = 0;
    while (n >= 4) {
        uint32_t x = ((uint32_t)q[0] << 24 | (uint32_t)q[1] << 16 | (uint32_t)q[2] << 8 | q[3]) ^ (c << 16);
        c = (uint32_t)s_crc16[3][x >> 24] ^ s_crc16[2][(x >> 16) & 0xFF] ^
            s_crc16[1][(x >> 8) & 0xFF] ^ s_crc16[0][x & 0xFF];
        q += 4; n -= 4;
    }
    while (n--) c = (uint16_t)((c << 8) ^ s_crc16[0][((c >> 8) ^ *q++) & 0xFF]);
    return (uint16_t)c;
}

// ------------------------------------------------------------- bit writer
// 32-bit accumulator (64-bit shifts cost several instructions on the M7).
// 'acc' holds 32-free pending bits in its low end; full words are stored
// big-endian. Values passed to bw_put must already fit in 'bits' bits, and
// bits must be 1..31.
typedef struct {
    uint8_t *p, *end;
    uint32_t acc;
    int      free;    // 1..32 free bit positions in the current word
    int      ovf;
} bw_t;

static inline void bw_store(bw_t *b, uint32_t w)
{
    if (b->p + 4 > b->end) { b->ovf = 1; return; }
    w = __builtin_bswap32(w);
    memcpy(b->p, &w, 4);            // unaligned word store (fine on Cortex-M7)
    b->p += 4;
}

static inline void bw_put(bw_t *b, uint32_t v, int bits)
{
    if (bits < b->free) {
        b->acc = (b->acc << bits) | v;
        b->free -= bits;
    } else {                         // here free <= bits <= 31, so free < 32
        int r = bits - b->free;      // bits that spill into the next word
        bw_store(b, (b->acc << b->free) | (v >> r));
        b->acc = v;                  // its low r bits are the pending ones
        b->free = 32 - r;
    }
}

// pad with zeros to a byte boundary and write out everything pending
static void bw_align_flush(bw_t *b)
{
    int used = 32 - b->free;
    if (used == 0) return;
    int nbytes = (used + 7) >> 3;
    uint32_t w = b->acc << b->free;  // left-justify (free < 32 here)
    if (b->p + nbytes > b->end) { b->ovf = 1; return; }
    for (int i = 0; i < nbytes; i++) *b->p++ = (uint8_t)(w >> (24 - 8 * i));
    b->acc = 0; b->free = 32;
}

static inline void bw_rice(bw_t *b, uint32_t u, int k)
{
    uint32_t q = u >> k;
    uint32_t len = q + 1u + (uint32_t)k;
    uint32_t low = u & ((1u << k) - 1u);            // k <= 30
    if (len <= 31u) {
        bw_put(b, (1u << k) | low, (int)len);       // q zeros, a one, k bits
    } else {                                        // rare: long unary run
        while (q >= 31u) { bw_put(b, 0, 31); q -= 31u; }
        bw_put(b, 1u, (int)q + 1);
        if (k) bw_put(b, low, k);
    }
}


// Hot loop: Rice-code u[0..n) with parameter k. Writer state lives in locals
// (registers) for the whole partition instead of going through *b per sample.
static void bw_rice_block(bw_t *b, const uint32_t *u, uint32_t n, int k)
{
    uint32_t acc = b->acc; int fr = b->free;
    uint8_t *p = b->p; uint8_t *const end = b->end;
    const uint32_t lowmask = (1u << k) - 1u, one = 1u << k;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t v = u[i], q = v >> k;
        uint32_t len = q + 1u + (uint32_t)k;
        if (len <= 31u) {
            uint32_t code = one | (v & lowmask);
            if ((int)len < fr) { acc = (acc << len) | code; fr -= (int)len; }
            else {
                int r = (int)len - fr;
                uint32_t w = (acc << fr) | (code >> r);
                if (p + 4 > end) { b->ovf = 1; return; }
                w = __builtin_bswap32(w); memcpy(p, &w, 4); p += 4;
                acc = code; fr = 32 - r;
            }
        } else {                                    // rare long unary run: slow path
            b->acc = acc; b->free = fr; b->p = p;
            bw_rice(b, v, k);
            acc = b->acc; fr = b->free; p = b->p;
        }
    }
    b->acc = acc; b->free = fr; b->p = p;
}

// --------------------------------------------------------------- helpers
static int blocksize_code(uint32_t n)
{
    switch (n) {
        case 192: return 1;
        case 576: return 2;  case 1152: return 3; case 2304: return 4; case 4608: return 5;
        case 256: return 8;  case 512: return 9;  case 1024: return 10; case 2048: return 11;
        case 4096: return 12; case 8192: return 13; case 16384: return 14; case 32768: return 15;
    }
    return (n <= 256) ? 6 : 7;
}

static int rate_code(uint32_t fs)
{
    switch (fs) {
        case 88200: return 1;  case 176400: return 2; case 192000: return 3;
        case 8000: return 4;   case 16000: return 5;  case 22050: return 6;
        case 24000: return 7;  case 32000: return 8;  case 44100: return 9;
        case 48000: return 10; case 96000: return 11;
    }
    if (fs % 1000u == 0 && fs / 1000u <= 255u) return 12;
    if (fs <= 65535u) return 13;
    if (fs % 10u == 0 && fs / 10u <= 65535u) return 14;   // 384 kHz -> 38400
    return 0;                                             // 768 kHz: from STREAMINFO
}

// ---------------------------------------------------------------- public
void tufr_flac_init(tufr_flac_t *e, uint32_t fs, uint8_t nch, uint16_t blocksize)
{
    if (!s_tables_ok) make_tables();
    memset(e, 0, sizeof(*e));
    e->fs = fs;
    e->nch = (nch < 1) ? 1 : (nch > TUFR_FLAC_MAX_CH ? TUFR_FLAC_MAX_CH : nch);
    if (blocksize < 16) blocksize = 16;
    if (blocksize > TUFR_FLAC_MAX_BLOCK) blocksize = TUFR_FLAC_MAX_BLOCK;
    e->blocksize = blocksize;
    e->min_frame = 0xFFFFFFu;
}

void tufr_flac_header(const tufr_flac_t *e, uint8_t *h)
{
    memset(h, 0, TUFR_FLAC_HEADER_BYTES);
    h[0] = 'f'; h[1] = 'L'; h[2] = 'a'; h[3] = 'C';
    // STREAMINFO block header: not last, type 0, length 34
    h[4] = 0x00; h[5] = 0; h[6] = 0; h[7] = 34;
    uint8_t *s = h + 8;
    s[0] = (uint8_t)(e->blocksize >> 8); s[1] = (uint8_t)e->blocksize;   // min block
    s[2] = (uint8_t)(e->blocksize >> 8); s[3] = (uint8_t)e->blocksize;   // max block
    uint32_t mn = (e->frame_number && e->min_frame != 0xFFFFFFu) ? e->min_frame : 0;
    uint32_t mx = e->frame_number ? e->max_frame : 0;
    s[4] = (uint8_t)(mn >> 16); s[5] = (uint8_t)(mn >> 8); s[6] = (uint8_t)mn;
    s[7] = (uint8_t)(mx >> 16); s[8] = (uint8_t)(mx >> 8); s[9] = (uint8_t)mx;
    // 20 bits rate, 3 bits ch-1, 5 bits bps-1, 36 bits total samples
    uint64_t v = ((uint64_t)(e->fs & 0xFFFFFu) << 44)
               | ((uint64_t)((e->nch - 1) & 7u) << 41)
               | ((uint64_t)15u << 36)
               | (e->total_samples & 0xFFFFFFFFFull);
    for (int i = 0; i < 8; i++) s[10 + i] = (uint8_t)(v >> (56 - 8 * i));
    // s[18..33] MD5 = 0 (not computed: "unknown", allowed by the spec)
    // PADDING block: last, type 1, fills the rest of the 512-byte header
    uint32_t padlen = TUFR_FLAC_HEADER_BYTES - 8 - 34 - 4;   // 466
    h[42] = 0x80 | 0x01;
    h[43] = (uint8_t)(padlen >> 16); h[44] = (uint8_t)(padlen >> 8); h[45] = (uint8_t)padlen;
}


// Rice parameter for a partition of nj folded residuals summing to S:
// the largest k (0..30) with nj * 2^k <= S, i.e. floor(log2(S / nj)).
// Uses count-leading-zeros (one instruction on the M7) instead of a
// bit-by-bit search; the CLZ estimate is exact or one too high.
static inline int rice_k(uint64_t S, uint32_t nj)
{
    if (S < (uint64_t)nj) return 0;
    int k = (63 - __builtin_clzll(S)) - (31 - __builtin_clz(nj));
    if (k > 30) return 30;           // then nj * 2^30 <= S already holds
    if (((uint64_t)nj << k) > S) k--;
    return k;
}

// Encode one channel of the frame as a subframe. src points at this channel's
// first sample in the interleaved block, stride = number of channels.
static void encode_subframe(tufr_flac_t *e, bw_t *b, uint32_t n,
                           const int16_t *src, uint32_t stride)
{
    int32_t *x = s_x;

    // Pass 1: de-interleave into s_x and, in the same loop, sum |residual|
    // for the five FIXED predictors (pick the order with the smallest sum).
    // A frame with all samples equal becomes a CONSTANT subframe.
    int order = 0;
    {
        const int16_t *p = src;
        int32_t x0 = *p, x1, x2, x3;
        x[0] = x0;
        if (n < 8) {
            uint32_t i;
            for (i = 1; i < n; i++) x[i] = p[(size_t)i * stride];
            for (i = 1; i < n && x[i] == x0; i++) ;
            if (i == n) goto constant;
            goto verbatim;
        }
        x1 = p[stride]; x2 = p[2 * (size_t)stride]; x3 = p[3 * (size_t)stride];
        x[1] = x1; x[2] = x2; x[3] = x3;
        int32_t a = x3, d1 = x3 - x2;
        int32_t d2 = d1 - (x2 - x1);
        int32_t d3 = d2 - ((x2 - x1) - (x1 - x0));
        uint32_t s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
        const int head_const = (x1 == x0 && x2 == x0 && x3 == x0);
        p += 4 * (size_t)stride;
        // Choose the order from every TUFR_FLAC_SELECT_DIV-th block of 64
        // samples (spread over the whole frame, so a short call anywhere in
        // the frame is seen); the other blocks are only copied.
        uint32_t same = 1;
        for (uint32_t i0 = 4, blk = 0; i0 < n; blk++) {
            uint32_t i1 = (blk + 1) * 64u; if (i1 > n) i1 = n;
            if (TUFR_FLAC_SELECT_DIV <= 1 || n < 256 || blk % TUFR_FLAC_SELECT_DIV == 0) {
                if (i0 > 4) {    // restart the difference chain from the samples before i0
                    int32_t y1 = x[i0-1], y2 = x[i0-2], y3 = x[i0-3], y4 = x[i0-4];
                    a = y1; d1 = y1 - y2; d2 = d1 - (y2 - y3); d3 = d2 - ((y2 - y3) - (y3 - y4));
                }
                for (uint32_t i = i0; i < i1; i++, p += stride) {
                    int32_t v = *p;
                    x[i] = v;
                    int32_t e1 = v - a;     a = v;
                    int32_t e2 = e1 - d1;   d1 = e1;
                    int32_t e3 = e2 - d2;   d2 = e2;
                    int32_t e4 = e3 - d3;   d3 = e3;
                    s0 += (uint32_t)(v  < 0 ? -v  : v);
                    s1 += (uint32_t)(e1 < 0 ? -e1 : e1);
                    s2 += (uint32_t)(e2 < 0 ? -e2 : e2);
                    s3 += (uint32_t)(e3 < 0 ? -e3 : e3);
                    s4 += (uint32_t)(e4 < 0 ? -e4 : e4);
                }
            } else {
                for (uint32_t i = i0; i < i1; i++, p += stride) { int32_t v = *p; x[i] = v; same &= (v == x0); }
            }
            i0 = i1;
        }
        if (!same) s1 |= 1u;   // not constant
        if (s1 == 0 && head_const) goto constant;   // every sample equals x[0]
        uint32_t best = s0;
        if (s1 < best) { best = s1; order = 1; }
        if (s2 < best) { best = s2; order = 2; }
        if (s3 < best) { best = s3; order = 3; }
        if (s4 < best) { best = s4; order = 4; }
    }

    // finest legal partition order
    int pmax = 0;
    while (pmax < TUFR_FLAC_MAX_PORDER &&
           (n & ((1u << (pmax + 1)) - 1u)) == 0 &&
           (n >> (pmax + 1)) > (uint32_t)order)
        pmax++;
    uint32_t np = 1u << pmax, plen = n >> pmax;

    // Pass 2: residual (folded to unsigned) written over s_x in place, with
    // the per-partition sums at pmax. Forward loop; the previous original
    // samples are kept in registers, so x[i] can be overwritten at once.
    {
        uint32_t i = (uint32_t)order;
        int32_t p1 = order >= 1 ? x[order - 1] : 0;
        int32_t p2 = order >= 2 ? x[order - 2] : 0;
        int32_t p3 = order >= 3 ? x[order - 3] : 0;
        int32_t p4 = order >= 4 ? x[order - 4] : 0;
        for (uint32_t j = 0; j < np; j++) {
            uint32_t i1 = (j + 1) * plen;
            uint32_t s = 0;              // partition <= 2048 samples of < 2^21: fits
            switch (order) {
            case 0:
                for (; i < i1; i++) { int32_t r = x[i];
                    uint32_t u = ((uint32_t)r << 1) ^ (uint32_t)(r >> 31); s_u[i] = u; s += u; }
                break;
            case 1:
                for (; i < i1; i++) { int32_t v = x[i], r = v - p1; p1 = v;
                    uint32_t u = ((uint32_t)r << 1) ^ (uint32_t)(r >> 31); s_u[i] = u; s += u; }
                break;
            case 2:
                for (; i < i1; i++) { int32_t v = x[i], r = v - 2 * p1 + p2; p2 = p1; p1 = v;
                    uint32_t u = ((uint32_t)r << 1) ^ (uint32_t)(r >> 31); s_u[i] = u; s += u; }
                break;
            case 3:
                for (; i < i1; i++) { int32_t v = x[i], r = v - 3 * (p1 - p2) - p3; p3 = p2; p2 = p1; p1 = v;
                    uint32_t u = ((uint32_t)r << 1) ^ (uint32_t)(r >> 31); s_u[i] = u; s += u; }
                break;
            default:
                for (; i < i1; i++) { int32_t v = x[i], r = v - 4 * (p1 + p3) + 6 * p2 + p4;
                    p4 = p3; p3 = p2; p2 = p1; p1 = v;
                    uint32_t u = ((uint32_t)r << 1) ^ (uint32_t)(r >> 31); s_u[i] = u; s += u; }
                break;
            }
            s_psum[j] = s;
        }
    }

    // Walk partition orders from fine to coarse, keep the cheapest; remember
    // the Rice parameters of the best one.
    static uint8_t kp[1u << TUFR_FLAC_MAX_PORDER];
    uint64_t best_bits = ~0ull; int best_p = 0, best_rice2 = 0;
    for (int p = pmax; p >= 0; p--) {
        uint32_t cnt = 1u << p, len = n >> p;
        uint64_t bits = 0; int kmax = 0;
        uint8_t ktmp[1u << TUFR_FLAC_MAX_PORDER];
        for (uint32_t j = 0; j < cnt; j++) {
            uint32_t nj = (j == 0) ? len - (uint32_t)order : len;
            uint64_t S = s_psum[j];
            int k = rice_k(S, nj);
            ktmp[j] = (uint8_t)k;
            if (k > kmax) kmax = k;
            bits += (uint64_t)nj * (uint64_t)(k + 1) + (S >> k);
        }
        bits += (uint64_t)cnt * (kmax > 14 ? 5u : 4u);
        if (bits < best_bits) {
            best_bits = bits; best_p = p; best_rice2 = (kmax > 14);
            memcpy(kp, ktmp, cnt);
        }
        if (p > 0)  // merge pairs for the next coarser order
            for (uint32_t j = 0; j < cnt / 2; j++) s_psum[j] = s_psum[2*j] + s_psum[2*j+1];
    }

    // compare with VERBATIM (estimate is an upper bound on the real size)
    if (8 + 16u * (uint32_t)order + 6 + best_bits >= 8 + 16ull * n) goto verbatim;

    // FIXED subframe
    bw_put(b, (uint32_t)(0x10 | (order << 1)), 8);
    for (int i = 0; i < order; i++) bw_put(b, (uint32_t)x[i] & 0xFFFFu, 16);  // warm-up: untouched
    bw_put(b, best_rice2 ? 1u : 0u, 2);
    bw_put(b, (uint32_t)best_p, 4);
    {
        uint32_t cnt = 1u << best_p, len = n >> best_p;
        for (uint32_t j = 0; j < cnt; j++) {
            uint32_t i0 = (j == 0) ? (uint32_t)order : j * len, i1 = (j + 1) * len;
            bw_put(b, kp[j], best_rice2 ? 5 : 4);
            bw_rice_block(b, &s_u[i0], i1 - i0, kp[j]);
        }
    }
    e->n_fixed[order]++;
    return;

constant:
    bw_put(b, 0x00, 8);
    bw_put(b, (uint32_t)s_x[0] & 0xFFFFu, 16);
    e->n_constant++;
    return;

verbatim:     // s_x may hold residuals now: take the samples from the source
    bw_put(b, 0x02, 8);
    for (uint32_t i = 0; i < n; i++) bw_put(b, (uint16_t)src[(size_t)i * stride], 16);
    e->n_verbatim++;
}

size_t tufr_flac_encode_frame(tufr_flac_t *e, const int16_t *pcm,
                              uint32_t n, uint8_t *out, size_t cap)
{
    if (n == 0 || n > e->blocksize || cap < 32) return 0;
    bw_t b = { out, out + cap, 0, 32, 0 };

    // ---- frame header
    int bsc = (n == e->blocksize) ? blocksize_code(n) : (n <= 256 ? 6 : 7);
    if (bsc != 6 && bsc != 7 && (uint32_t)n != e->blocksize) bsc = (n <= 256 ? 6 : 7);
    int rc = rate_code(e->fs);
    bw_put(&b, 0xFFF8, 16);                       // sync + reserved + fixed-blocksize (byte 2 = 0xF8)
    bw_put(&b, (uint32_t)((bsc << 4) | rc), 8);
    bw_put(&b, (uint32_t)(((e->nch - 1) << 4) | (4 << 1)), 8);   // independent, 16-bit
    // frame number, UTF-8 style
    uint32_t fn = e->frame_number;
    if (fn < 0x80) bw_put(&b, fn, 8);
    else {
        int nb = (fn < 0x800) ? 2 : (fn < 0x10000) ? 3 : (fn < 0x200000) ? 4 :
                 (fn < 0x4000000) ? 5 : 6;
        uint32_t lead = (0xFF00u >> nb) & 0xFFu;
        bw_put(&b, lead | (fn >> (6 * (nb - 1))), 8);
        for (int i = nb - 2; i >= 0; i--) bw_put(&b, 0x80u | ((fn >> (6 * i)) & 0x3Fu), 8);
    }
    if (bsc == 6) bw_put(&b, n - 1, 8);
    if (bsc == 7) bw_put(&b, n - 1, 16);
    if (rc == 12) bw_put(&b, e->fs / 1000u, 8);
    if (rc == 13) bw_put(&b, e->fs, 16);
    if (rc == 14) bw_put(&b, e->fs / 10u, 16);
    // header is byte aligned here; flush to compute CRC-8
    bw_align_flush(&b);
    if (b.ovf) return 0;
    uint8_t c8 = 0;
    for (uint8_t *q = out; q < b.p; q++) c8 = s_crc8[c8 ^ *q];
    bw_put(&b, c8, 8);

    // ---- subframes
    for (uint32_t ch = 0; ch < e->nch; ch++) {
        encode_subframe(e, &b, n, pcm + ch, e->nch);
        if (b.ovf) return 0;
    }

    // ---- footer
    bw_align_flush(&b);
    if (b.ovf || b.p + 2 > b.end) return 0;
    uint16_t c16 = crc16(out, (size_t)(b.p - out));
    *b.p++ = (uint8_t)(c16 >> 8);
    *b.p++ = (uint8_t)c16;

    size_t len = (size_t)(b.p - out);
    if (len < e->min_frame) e->min_frame = (uint32_t)len;
    if (len > e->max_frame) e->max_frame = (uint32_t)len;
    e->frame_number++;
    e->total_samples += n;
    return len;
}

size_t tufr_flac_encode(tufr_flac_t *e, const int16_t *pcm,
                        uint32_t nframes, uint8_t *out, size_t cap)
{
    size_t total = 0;
    while (nframes) {
        uint32_t n = nframes > e->blocksize ? e->blocksize : nframes;
        size_t w = tufr_flac_encode_frame(e, pcm, n, out + total, cap - total);
        if (w == 0) return 0;
        total += w;
        pcm += (size_t)n * e->nch;
        nframes -= n;
    }
    return total;
}

// ------------------------------------------------------- streaming API
int tufr_flac_push(tufr_flac_t *e, const int16_t *pcm, uint32_t nsamp,
                   uint8_t *frame, tufr_flac_emit_t emit, void *ctx)
{
    const uint32_t full = (uint32_t)e->blocksize * e->nch;   // samples per frame
    const size_t cap = TUFR_FLAC_FRAME_MAX(e->blocksize, e->nch);
    while (nsamp) {
        if (e->carry == 0 && nsamp >= full) {
            // whole frame straight from the caller's buffer (no copy)
            size_t w = tufr_flac_encode_frame(e, pcm, e->blocksize, frame, cap);
            if (!w || !emit(frame, w, ctx)) return 0;
            pcm += full; nsamp -= full;
            continue;
        }
        uint32_t take = full - e->carry;
        if (take > nsamp) take = nsamp;
        memcpy(&s_carry[e->carry], pcm, take * sizeof(int16_t));
        e->carry += take; pcm += take; nsamp -= take;
        if (e->carry == full) {
            size_t w = tufr_flac_encode_frame(e, s_carry, e->blocksize, frame, cap);
            e->carry = 0;
            if (!w || !emit(frame, w, ctx)) return 0;
        }
    }
    return 1;
}

int tufr_flac_flush(tufr_flac_t *e, uint8_t *frame, tufr_flac_emit_t emit, void *ctx)
{
    uint32_t nfr = e->carry / e->nch;
    uint32_t rem = e->carry - nfr * e->nch;   // 1..nch-1 samples of an incomplete sample frame
    int ok = 1;
    if (nfr) {
        size_t w = tufr_flac_encode_frame(e, s_carry, nfr, frame,
                                          TUFR_FLAC_FRAME_MAX(e->blocksize, e->nch));
        ok = (w && emit(frame, w, ctx)) ? 1 : 0;
    }
    // keep the incomplete sample frame: it continues in the next file
    if (rem) memmove(s_carry, &s_carry[nfr * e->nch], rem * sizeof(int16_t));
    e->carry = rem;
    return ok;
}

void tufr_flac_new_file(tufr_flac_t *e)
{
    uint32_t carry = e->carry;
    tufr_flac_init(e, e->fs, e->nch, e->blocksize);
    e->carry = carry;
}

int tufr_flac_pad_frame(tufr_flac_t *e, uint8_t *frame, tufr_flac_emit_t emit, void *ctx)
{
    if (e->carry == 0) return 1;
    uint32_t full = (uint32_t)e->blocksize * e->nch;
    memset(&s_carry[e->carry], 0, (full - e->carry) * sizeof(int16_t));
    e->carry = full;
    size_t w = tufr_flac_encode_frame(e, s_carry, e->blocksize, frame,
                                      TUFR_FLAC_FRAME_MAX(e->blocksize, e->nch));
    e->carry = 0;
    return (w && emit(frame, w, ctx)) ? 1 : 0;
}
