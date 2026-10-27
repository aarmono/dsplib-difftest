/*
 * R12 differential harness: the caller-ID detectors band_pass / DTMF_MTD_detect / dtmf_modem / reset_dtmf (DTMF caller ID), CID_FSD_demodulate / CID_MTD_detect (FSK caller
 * ID: Bell 202 and V.23) and FPM_div_32 - real dsplibs.o vs src/cid/cid_detect.c + src/fpm/fpm_div32.c + src/cid/cid_tables.c.  Library test/r04_common.h (hashed canonical output, coverage
 * counters exit 2, 3 = harness violation); the typed objects of src/cid/cid.h on both sides (identical layout at -m32), the 32-bit BLOB run is the x86-64 golden.  The callee
 * FPM_iir_filt is the REAL one of each side (blob: dsplibs.o; port: src/fpm/fpm_iir.c); dsplibs_debug_printf is the stub's (the debug lines are part of the diffed text).
 * All signals are integer-only (Bhaskara sine, no libm).  There is no nullmodem scenario for caller ID: this harness is the only test of these functions.
 *
 * Scenarios (each its own process):
 *   0 FPM_div_32: 0 (debug level 0 .. 3), every power of two and its neighbours, normalised-negative inputs, the inputs whose mantissa index is 128 (one past the table), random values
 *     with random leading-zero counts; outputs left untouched when x == 0;
 *   1 CID_MTD_detect: tone sweeps 100 .. 4000 Hz in steps (the Bell 202 / V.23 mark and space tones included), amplitudes 30 .. 32000, noise, silence, random and saturated data, block
 *     lengths 0 .. 200, fs 9600 / 8000 / 7200 / 0, biquad state carried across blocks and poked;
 *   2 CID_FSD_demodulate (+ CID_MTD_detect on the same blocks): realistic Bell 202 (1200 / 2200 Hz) and V.23 (1300 / 2100 Hz) caller-ID messages (channel seizure, mark, framed
 *     SDMF / MDMF bytes with checksum), random bit streams, noisy and level-varying channels, DC offset, silence, saturation, fs 8000 / 9600 / 7200, blocks of 1 .. 192 samples,
 *     long runs (the threshold counters reach their 0x80 / 0x10 / 0x100 limits), zero and poked initial state;
 *   3 band_pass: blocks of tones / noise / silence / bursts in every energy class (the 20000 / 10000 / 600 / 300 / 150 counters, the x6 / x3 / x2 / x1 scalings, mode 0 .. 5, fs 9600 /
 *     8000, minEn poked, n = -3 .. 198), DC offsets, the in-place output hashed;
 *   4 DTMF_MTD_detect: all 16 DTMF digits at 8000 / 9600 Hz, twist, level, noise, single tones, silence, block lengths 40 .. 192, filter states carried / poked;
 *   5 dtmf_modem end to end: caller-ID DTMF sequences (start tone 'D', digits, 'C'), random tone sequences, onsets inside a frame (the half-block stash path), noise, glitches, the
 *     time-outs (3 s, 300 ms), mode 0 .. 4, fs 8000 / 9600, frame lengths 160 / 192 / odd / random; reset_dtmf from the blob and from the port;
 *   7 (R13 closing the R12 harness gaps) boundary sweeps in unit steps (CID_MTD_detect, DTMF_MTD_detect, band_pass energy / ratio decisions), CID_FSD_demodulate with n == 0,
 *     dtmf_modem digit stores outside the object (nDig far outside digits[]: the blob writes into harness slack, the port drops the store);
 *   6 dtmf_modem from poked states: phase 0 .. 3, nDig -1 .. 21, cnt, lastDig, fails, halfFlag, samples around the limits, then a few blocks; reset_dtmf over junk objects (debug 0 / 2).
 */
#include "../r04_common.h"
#include "cid/cid.h"

extern short FPM_iir_filt(short seed, const short *coefs, short *state, short count);

enum {
  CV_DIV_ZERO = 0, CV_DIV_OK, CV_DIV_NEG, CV_DIV_IDX128, CV_DIV_SH31, CV_DIV_DBG,
  CV_MTD_ON = 10, CV_MTD_OFF, CV_MTD_9600, CV_MTD_8000,
  CV_FSD_BITS = 20, CV_FSD_ONE, CV_FSD_ZERO, CV_FSD_9600, CV_FSD_8000, CV_FSD_7200, CV_FSD_MEAN, CV_FSD_THR16, CV_FSD_THR128, CV_FSD_LONG, CV_FSD_BELL, CV_FSD_V23,
  CV_BP_ON = 40, CV_BP_OFF, CV_BP_S6, CV_BP_S3, CV_BP_S2, CV_BP_S1, CV_BP_C20, CV_BP_C10, CV_BP_MODE2, CV_BP_MODE3, CV_BP_MODE4, CV_BP_MIN100, CV_BP_NPOS_ZERO, CV_BP_NNEG,
  CV_BP_CL1, CV_BP_CL2, CV_BP_CL3, CV_BP_CL4,
  CV_DT_DIG0 = 60, /* 60 .. 75: digit codes 0 .. 15 detected by DTMF_MTD_detect */
  CV_DT_NONE = 76, CV_DT_9600, CV_DT_8000,
  CV_DM_R1 = 80, CV_DM_R2, CV_DM_R3, CV_DM_RM1, CV_DM_DIGIT, CV_DM_C, CV_DM_20, CV_DM_HALF, CV_DM_TIMEOUT, CV_DM_FAIL, CV_DM_PH0, CV_DM_PH1, CV_DM_PH2, CV_DM_PH3,
  CV_DM_START, CV_DM_INVALID, CV_DM_MODE1, CV_DM_MODE0, CV_DM_9600, CV_DM_DBG, CV_DM_RESET, CV_DM_BADDIG, CV_DM_ONSET, CV_DM_SHORT, CV_DM_NOISE,
  CV_RS_RUN = 110, CV_RS_DBG,
  CV_GAP_OUT = 120, CV_GAP_IN, CV_GAP_N0, CV_GAP_MTD, CV_GAP_DT, CV_GAP_BP
};

static const char *g_what = "";
/* ------------------------------------------------------------------------------------------------ integer-only signals */
static int isin(unsigned phase)
{
  unsigned x = phase & 0x7fffu, u = (x * (32768u - x)) >> 15, y = (16u * u << 15) / ((5u << 15) - 4u * u);
  return (phase & 0x8000u) ? -(int)y : (int)y;
}
static unsigned inc_of(int f, int fs) { return (unsigned)((((unsigned long long)(unsigned)f) << 32) / (unsigned)fs); }
static short clip16(long long v) { return (short)(v > 32767 ? 32767 : v < -32768 ? -32768 : v); }
#define SIGMAX 600000
static short g_sig[SIGMAX];
static unsigned g_sn;
static unsigned g_ph1, g_ph2;
static void put(long long v) { if (g_sn < SIGMAX) g_sig[g_sn++] = clip16(v); }
static void gen_noise(unsigned n, int amp)
{
  unsigned i;
  for (i = 0; i < n; i++) { int r = (int)(rnd() >> 8) % (2 * amp + 1) - amp; put(r); }
}
static void gen_tone2(unsigned n, int f1, int f2, int a1, int a2, int fs, int noise, int dc)
{
  unsigned i, i1 = inc_of(f1, fs), i2 = inc_of(f2, fs);
  for (i = 0; i < n; i++) {
    long long v = (long long)a1 * isin(g_ph1 >> 16) / 32768 + (long long)a2 * isin(g_ph2 >> 16) / 32768 + dc;
    if (noise) { int r = (int)(rnd() >> 8) % (2 * noise + 1) - noise; v += r; }
    put(v);
    g_ph1 += i1;
    g_ph2 += i2;
  }
}
static const int dtmf_lo[4] = { 697, 770, 852, 941 }, dtmf_hi[4] = { 1209, 1336, 1477, 1633 };
/* digit codes as DTMF_MTD_detect reports them: the 4 x 4 table row-major */
static const short dtmf_code[16] = { 1, 2, 3, 10, 4, 5, 6, 11, 7, 8, 9, 12, 14, 0, 15, 13 };
static void gen_dtmf(unsigned n, int code, int fs, int a1, int a2, int noise)
{
  int k;
  for (k = 0; k < 16; k++) if (dtmf_code[k] == code) break;
  if (k == 16) { gen_noise(n, noise ? noise : 1); return; }
  gen_tone2(n, dtmf_lo[k / 4], dtmf_hi[k % 4], a1, a2, fs, noise, 0);
}

/* ------------------------------------------------------------------------------------------------ object dumps (layout independent) */
static void dump_dtmf(const char *tag, int show, const CidDtmf *d)
{
  char dig[48];
  int i;
  const char *bp = d->blkp == NULL ? "null" : d->blkp == d->blk ? "blk" : "other";
  for (i = 0; i < 20; i++) snprintf(dig + 2 * i, 3, "%02x", (unsigned char)d->digits[i]);
  emit(show, "%s D st0=%d w4=%d s8=%d last=%u cnt=%d nd=%d ph=%d half=%d fl=%d min=%u bp=%s fs=%u mode=%d dig=%s i=%d,%d,%d,%d,%d,%d,%d,%d s354=%d,%d,%d s=%d",
       tag, d->state0, d->w004, d->s008, d->lastDig, d->cnt, d->nDig, d->phase, d->halfFlag, d->fails, d->minEn, bp, d->fs, d->mode, dig, d->iirBand[0], d->iirBand[1],
       d->iirLow0[0], d->iirLow0[1], d->iirLow1[0], d->iirLow1[1], d->s35c[0], d->s35c[1], d->s354, d->s356, d->s35c[0], d->samples);
  emit(show, "%s M %016llx %016llx %016llx", tag, fnv_shorts(&d->iirMtd[0][0], 16), fnv_shorts(d->blk, 300), fnv_shorts(d->stash, 100));
}
static void dump_fsk(const char *tag, int show, const CidFsk *d)
{
  /* the members the R12 detectors use, by name (the object grew with R13 and its layout differs on x86-64: no raw bytes) */
  emit(show, "%s F fs=%u lp=%d,%d lpf=%016llx hist=%016llx cur=%u np=%d nn=%d thr=%d mean=%d neg=%d pos=%d run=%d,%d,%d i1=%d,%d i2=%d,%d", tag, d->fs, d->lpfPos, d->pos,
       fnv_shorts(d->lpf, 17), fnv_shorts(d->hist, 5), d->cur, d->nPos, d->nNeg, d->thr, d->mean, d->negAcc, d->posAcc, d->run, d->runAlt, d->quiet, d->iirMtd1[0], d->iirMtd1[1],
       d->iirMtd2[0], d->iirMtd2[1]);
}

/* ------------------------------------------------------------------------------------------------ 0: FPM_div_32 */
static void div_call(const char *tag, int show, unsigned x)
{
  short m = 0x5a5a, s = 0x2b2b;
  int r = FPM_div_32(x, &m, &s);
  emit(show, "%s x=%08x r=%d m=%d s=%d", tag, x, r, m, s);
  if (r == 0) {
    unsigned y = x;
    int sh = 0;
    cov_hit(CV_DIV_OK);
    if ((int)x < 0) cov_hit(CV_DIV_NEG);
    if (s == 31) cov_hit(CV_DIV_SH31);
    while ((int)y >= 0) { y <<= 1; sh++; }
    if (((y >> 16) + 0x80u) >> 8 == 0x100u) cov_hit(CV_DIV_IDX128);
  } else {
    cov_hit(CV_DIV_ZERO);
  }
}
static void sc_div(void)
{
  int t, k, lvl;
  cov_def(CV_DIV_ZERO, "div32: x = 0"); cov_def(CV_DIV_OK, "div32: ok"); cov_def(CV_DIV_NEG, "div32: x negative"); cov_def(CV_DIV_IDX128, "div32: mantissa index 128");
  cov_def(CV_DIV_SH31, "div32: shift 31"); cov_def(CV_DIV_DBG, "div32: debug line");
  for (lvl = 0; lvl < 4; lvl++) {
    dsplibs_debug_level = (unsigned)lvl;
    emit(1, "level %d", lvl);
    div_call("Z", 1, 0);
    if (lvl > 1) cov_hit(CV_DIV_DBG);
  }
  dsplibs_debug_level = 0;
  for (k = 0; k < 32; k++) {
    unsigned p = 1u << k;
    div_call("P", k < 6, p);
    div_call("P-", 0, p - 1u);
    div_call("P+", 0, p + 1u);
    div_call("N", 0, 0u - p);
    div_call("N+", 0, (0u - p) + 1u);
    div_call("H", 0, p | (p >> 1));
    div_call("HH", 0, (p | (p >> 1) | (p >> 2)) & 0xffffffffu);
  }
  for (k = 0; k < 17; k++) { /* the inputs of index 128 and the entries around it */
    unsigned lo = 0xff00u + (unsigned)k * 16u;
    int sh;
    for (sh = 0; sh < 16; sh++) div_call("I", 0, (lo << 16 >> sh) | 0x7fu);
  }
  for (t = 0; t < 60000; t++) {
    unsigned x = rnd();
    int sh = (int)rmod(32);
    char tag[8];
    x >>= sh;
    if (rpct(10)) x |= 0xffff0000u >> rmod(16);
    if (rpct(10)) x = ~x;
    snprintf(tag, sizeof tag, "R");
    div_call(tag, t < 20, x);
  }
  for (t = 0; t < 4096; t++) div_call("S", 0, ((unsigned)t << 20) | (rnd() & 0xfffffu)); /* the whole table index range at every shift */
}

/* ------------------------------------------------------------------------------------------------ 1: CID_MTD_detect */
static CidFsk g_f;
static const int g_fsk_fs[4] = { 9600, 8000, 7200, 0 };
static void fsk_init(int fs, int poke)
{
  memset(&g_f, 0, sizeof g_f);
  g_f.fs = (unsigned short)fs;
  if (poke) {
    g_f.iirMtd1[0] = rshort(); g_f.iirMtd1[1] = rshort(); g_f.iirMtd2[0] = rshort(); g_f.iirMtd2[1] = rshort();
  }
}
static void sc_mtd(void)
{
  int t, c;
  cov_def(CV_MTD_ON, "cid_mtd: detects (0)"); cov_def(CV_MTD_OFF, "cid_mtd: no tone (1)"); cov_def(CV_MTD_9600, "cid_mtd: 9600"); cov_def(CV_MTD_8000, "cid_mtd: 8000");
  for (t = 0; t < 4000; t++) {
    int fs = g_fsk_fs[t % 4 == 3 ? rmod(4) : (unsigned)(t % 3)];
    int rate = fs ? fs : 8000;
    int f = rpct(40) ? (rpct(50) ? 1200 : 2200) + (rpct(50) ? 0 : rrange(-60, 60)) : rpct(30) ? (rpct(50) ? 1300 : 2100) : rrange(100, 4000);
    int amp = rpct(10) ? 32767 : rpct(15) ? rrange(30, 200) : rrange(200, 20000);
    int noise = rpct(25) ? amp / 4 : 0;
    char tag[16];
    snprintf(tag, sizeof tag, "M%d", t);
    fsk_init(fs, rpct(30));
    g_sn = 0;
    g_ph1 = rnd();
    for (c = 0; c < 6; c++) {
      int kind = (int)rmod(10);
      unsigned n = (unsigned)(rpct(10) ? (int)rmod(3) : rpct(70) ? rrange(80, 192) : rrange(1, 200));
      int r;
      g_sn = 0;
      if (kind < 6) gen_tone2(n, f, 0, amp, 0, rate, noise, 0);
      else if (kind == 6) gen_noise(n, amp);
      else if (kind == 7) { unsigned i; for (i = 0; i < n; i++) put(rshort()); }
      else if (kind == 8) gen_tone2(n, f, 0, 3 * amp, 0, rate, 0, 0);
      else gen_tone2(n, f, 0, 0, 0, rate, 0, 0);
      r = CID_MTD_detect(g_sig, (short)n, &g_f);
      emit(t < 10, "%s c=%d n=%u f=%d fs=%d -> %d", tag, c, n, f, fs, r);
      if (r == 0) cov_hit(CV_MTD_ON); else cov_hit(CV_MTD_OFF);
      if (fs == 9600) cov_hit(CV_MTD_9600);
      if (fs == 8000) cov_hit(CV_MTD_8000);
      dump_fsk(tag, 0, &g_f);
    }
    dump_fsk(tag, t < 10, &g_f);
  }
  /* negative / zero lengths */
  for (t = -3; t <= 0; t++) { fsk_init(9600, 1); emit(1, "NEG n=%d -> %d", t, CID_MTD_detect(g_sig, (short)t, &g_f)); dump_fsk("NEG", 1, &g_f); }
}

/* ------------------------------------------------------------------------------------------------ 2: CID_FSD_demodulate */
static unsigned char g_bits[200000];
static unsigned g_nbits;
static void bits_push(int b) { if (g_nbits < sizeof g_bits) g_bits[g_nbits++] = (unsigned char)b; }
static void bits_byte(int v)
{
  int i;
  bits_push(0);
  for (i = 0; i < 8; i++) bits_push((v >> i) & 1);
  bits_push(1);
}
/* a caller-ID message (SDMF type 4 or MDMF type 0x80), the way the telephone network sends it */
static void bits_message(int mdmf)
{
  unsigned char msg[80];
  int n = 0, i, sum = 0, len;
  const char *date = "10051230";
  char num[16];
  int nn = rrange(4, 12);
  for (i = 0; i < nn; i++) num[i] = (char)('0' + rmod(10));
  num[nn] = 0;
  if (!mdmf) {
    msg[n++] = 0x04;
    msg[n++] = (unsigned char)(8 + nn);
    for (i = 0; i < 8; i++) msg[n++] = (unsigned char)date[i];
    for (i = 0; i < nn; i++) msg[n++] = (unsigned char)num[i];
  } else {
    msg[n++] = 0x80;
    msg[n++] = (unsigned char)(2 + 8 + 2 + nn);
    msg[n++] = 1; msg[n++] = 8;
    for (i = 0; i < 8; i++) msg[n++] = (unsigned char)date[i];
    msg[n++] = 2; msg[n++] = (unsigned char)nn;
    for (i = 0; i < nn; i++) msg[n++] = (unsigned char)num[i];
  }
  len = n;
  for (i = 0; i < len; i++) sum += msg[i];
  msg[n++] = (unsigned char)(-sum);
  for (i = 0; i < 300 / 2; i++) { bits_push(0); bits_push(1); } /* channel seizure */
  for (i = 0; i < rrange(60, 200); i++) bits_push(1); /* mark */
  for (i = 0; i < n; i++) bits_byte(msg[i]);
  for (i = 0; i < 40; i++) bits_push(1);
}
static void gen_fsk(int fs, int fm, int fsp, int amp, int noise, int dc, int baud)
{
  unsigned i, ph = g_ph1, im = inc_of(fm, fs), is = inc_of(fsp, fs);
  unsigned tb = 0;
  unsigned long long cnt = 0;
  for (i = 0; i < g_nbits; ) {
    int b = g_bits[i];
    long long v = (long long)amp * isin(ph >> 16) / 32768 + dc;
    if (noise) { int r = (int)(rnd() >> 8) % (2 * noise + 1) - noise; v += r; }
    put(v);
    ph += b ? im : is;
    tb += (unsigned)baud;
    if (tb >= (unsigned)fs) { tb -= (unsigned)fs; i++; }
    cnt++;
    if (g_sn >= SIGMAX - 2) break;
  }
  g_ph1 = ph;
}
static void fsk_poke(void)
{
  int i;
  memset(&g_f, 0, sizeof g_f);
  g_f.pos = (short)rmod(5);
  g_f.lpfPos = (short)rmod(17);
  for (i = 0; i < 5; i++) g_f.hist[i] = (short)(rshort() >> 2);
  for (i = 0; i < 17; i++) g_f.lpf[i] = (short)(rshort() >> 2);
  g_f.cur = (unsigned short)rmod(2);
  g_f.nPos = (short)rmod(140);
  g_f.nNeg = (short)(rpct(50) ? rmod(20) : (rpct(50) ? 0x7f + rmod(3) : 0xff + rmod(3)));
  g_f.thr = rpct(50) ? 0 : rrange(-300, 3000);
  g_f.mean = rpct(40) ? 0 : rrange(0, 4000);
  g_f.negAcc = rrange(-100000, 0);
  g_f.posAcc = rrange(0, 200000);
  g_f.run = (short)rrange(-8, 9);
  g_f.runAlt = (short)rrange(-8, 5);
  g_f.quiet = (short)rrange(-8, 30);
}
static void fsd_stream(const char *tag, int show, int fs, int blk, int poke, int *nbitsOut)
{
  unsigned pos = 0, total = 0;
  static unsigned short out[400];
  unsigned long long h = 1469598103934665603ULL;
  int calls = 0;
  if (poke) { fsk_poke(); g_f.fs = (unsigned short)fs; }
  else { memset(&g_f, 0, sizeof g_f); g_f.fs = (unsigned short)fs; }
  while (pos < g_sn) {
    unsigned n = (unsigned)(blk > 0 ? blk : rrange(1, 192));
    unsigned short r;
    int mt;
    unsigned i;
    if (pos + n > g_sn) n = g_sn - pos;
    memset(out, 0xee, sizeof out);
    r = CID_FSD_demodulate(g_sig + pos, out, (short)n, &g_f);
    if (r > n) die3("more bits than samples");
    for (i = 0; i < r; i++) { h ^= out[i]; h *= 1099511628211ULL; if (out[i] == 1) cov_hit(CV_FSD_ONE); else if (out[i] == 0) cov_hit(CV_FSD_ZERO); }
    if (out[r] != 0xeeee) die3("wrote past the count");
    h ^= r; h *= 1099511628211ULL;
    total += r;
    mt = CID_MTD_detect(g_sig + pos, (short)n, &g_f);
    h ^= (unsigned)mt; h *= 1099511628211ULL;
    if (calls < 4 || (calls & 63) == 0) dump_fsk(tag, 0, &g_f);
    pos += n;
    calls++;
  }
  dump_fsk(tag, show, &g_f);
  emit(show, "%s fs=%d blk=%d poke=%d samples=%u bits=%u calls=%d h=%016llx", tag, fs, blk, poke, g_sn, total, calls, h);
  if (total) cov_hit(CV_FSD_BITS);
  if (g_f.mean) cov_hit(CV_FSD_MEAN);
  if (g_f.nNeg >= 0x10) cov_hit(CV_FSD_THR16);
  if (g_f.nNeg >= 0x100 || (unsigned char)g_f.nNeg == 0x80) cov_hit(CV_FSD_THR128);
  if (g_sn > 100000) cov_hit(CV_FSD_LONG);
  if (fs == 9600) cov_hit(CV_FSD_9600); else if (fs == 8000) cov_hit(CV_FSD_8000); else cov_hit(CV_FSD_7200);
  if (nbitsOut) *nbitsOut = (int)total;
}
static void sc_fsd(void)
{
  static const int blks[] = { 160, 192, 80, 1, 17, 40, 0, 0, 160 };
  int t;
  cov_def(CV_FSD_BITS, "fsd: bits produced"); cov_def(CV_FSD_ONE, "fsd: a one bit"); cov_def(CV_FSD_ZERO, "fsd: a zero bit"); cov_def(CV_FSD_9600, "fsd: 9600");
  cov_def(CV_FSD_8000, "fsd: 8000"); cov_def(CV_FSD_7200, "fsd: 7200"); cov_def(CV_FSD_MEAN, "fsd: positive mean set"); cov_def(CV_FSD_THR16, "fsd: negative counter >= 16");
  cov_def(CV_FSD_THR128, "fsd: negative counter at 0x80 / 0x100"); cov_def(CV_FSD_LONG, "fsd: stream > 100000 samples"); cov_def(CV_FSD_BELL, "fsd: Bell 202 message");
  cov_def(CV_FSD_V23, "fsd: V.23 message");
  for (t = 0; t < 140; t++) {
    int fs = g_fsk_fs[t % 3];
    int kind = t % 14;
    int bell = (t & 1) == 0;
    int fm = bell ? 1200 : 1300, fsp = bell ? 2200 : 2100;
    int amp = rpct(15) ? rrange(200, 1500) : rpct(15) ? 32000 : rrange(2000, 16000);
    int noise = (kind == 3 || kind == 9) ? amp / 3 : (kind == 5 ? amp / 8 : 0);
    int dc = kind == 6 ? rrange(-3000, 3000) : 0;
    int blk = blks[rmod(sizeof blks / sizeof blks[0])];
    char tag[24];
    int gotbits = 0;
    snprintf(tag, sizeof tag, "S%d", t);
    g_sn = 0; g_nbits = 0; g_ph1 = rnd();
    if (kind != 12) gen_noise((unsigned)rrange(100, 3000), rpct(50) ? 3 : 40); /* silence before the message */
    if (kind == 7 || kind == 13) { /* random bits */
      int i, nb = rrange(500, 20000);
      for (i = 0; i < nb; i++) bits_push(rpct(50));
    } else if (kind == 12) { /* many messages, long */
      int m;
      for (m = 0; m < 6; m++) bits_message(m & 1);
    } else {
      bits_message(kind & 1);
      if (kind == 2) bits_message(0);
    }
    gen_fsk(fs, fm, fsp, kind == 4 ? 32767 : amp, noise, dc, 1200);
    if (kind == 8) { unsigned i; for (i = 0; i < g_sn; i++) g_sig[i] = clip16((long long)g_sig[i] * 4); } /* saturation */
    if (kind == 10) { unsigned i; for (i = 0; i < g_sn; i++) g_sig[i] = clip16((long long)g_sig[i] * (long long)(1 + (i >> 11) % 8) / 4); } /* level changes */
    gen_noise((unsigned)rrange(100, 2000), 5);
    fsd_stream(tag, t < 12, fs, blk, (t % 5) == 4, &gotbits);
    if (bell) cov_hit(CV_FSD_BELL); else cov_hit(CV_FSD_V23);
  }
  /* long streams: the counters at their limits */
  for (t = 0; t < 4; t++) {
    int fs = g_fsk_fs[t % 3], i;
    char tag[24];
    snprintf(tag, sizeof tag, "L%d", t);
    g_sn = 0; g_nbits = 0; g_ph1 = rnd();
    for (i = 0; i < 12000; i++) bits_push(rpct(50));
    gen_fsk(fs, 1200, 2200, 9000, rpct(50) ? 2000 : 0, 0, 1200);
    fsd_stream(tag, t < 2, fs, t < 2 ? 160 : 0, t & 1, NULL);
  }
  /* non-FSK inputs */
  for (t = 0; t < 30; t++) {
    int fs = g_fsk_fs[t % 3];
    char tag[24];
    unsigned i;
    snprintf(tag, sizeof tag, "X%d", t);
    g_sn = 0; g_ph1 = rnd();
    switch (t % 6) {
      case 0: gen_noise(30000, 8000); break;
      case 1: gen_tone2(30000, rrange(200, 3500), 0, 9000, 0, fs, 0, 0); break;
      case 2: for (i = 0; i < 20000; i++) put(rshort()); break;
      case 3: for (i = 0; i < 20000; i++) put((i / 5) & 1 ? 32767 : -32768); break;
      case 4: gen_noise(20000, 2); break;
      default: gen_tone2(30000, 1700, 0, 12000, 0, fs, 500, 1000); break;
    }
    fsd_stream(tag, t < 6, fs, t % 3 == 0 ? 160 : 0, t & 1, NULL);
  }
}

/* ------------------------------------------------------------------------------------------------ 3: band_pass */
static CidDtmf g_d;
static void dtmf_clear(int fs, int mode, int poke)
{
  memset(&g_d, 0, sizeof g_d);
  g_d.fs = (unsigned short)fs;
  g_d.mode = (short)mode;
  g_d.minEn = 1;
  if (poke) {
    g_d.minEn = (unsigned short)(rpct(30) ? rmod(8) : rmod(300));
    g_d.iirBand[0] = (short)(rshort() >> 4); g_d.iirBand[1] = (short)(rshort() >> 4);
  }
}
static short g_bb[400];
static void bp_model(const short *x, int n)
{
  int i, c20 = 0, c10 = 0;
  for (i = 0; i < n; i++) { if (x[i] > 20000) c20++; if (x[i] > 10000) c10++; }
  if (n > 0 && (n >> 7) < c20) cov_hit(CV_BP_C20); else if (n > 0 && (n >> 7) < c10) cov_hit(CV_BP_C10);
}
static void sc_bp(void)
{
  int t, c;
  cov_def(CV_BP_ON, "bp: detects"); cov_def(CV_BP_OFF, "bp: no"); cov_def(CV_BP_S6, "bp: x6"); cov_def(CV_BP_S3, "bp: x3"); cov_def(CV_BP_S2, "bp: x2"); cov_def(CV_BP_S1, "bp: x1");
  cov_def(CV_BP_C20, "bp: >20000 count scaling"); cov_def(CV_BP_C10, "bp: >10000 count scaling"); cov_def(CV_BP_MODE2, "bp: mode 2"); cov_def(CV_BP_MODE3, "bp: mode 3");
  cov_def(CV_BP_MODE4, "bp: mode 4"); cov_def(CV_BP_MIN100, "bp: n > 100 sets minEn"); cov_def(CV_BP_NPOS_ZERO, "bp: n <= 0"); cov_def(CV_BP_CL1, "bp: energy class 0x38a4");
  cov_def(CV_BP_CL2, "bp: energy class 0x1fdc"); cov_def(CV_BP_CL3, "bp: energy class 0x1378"); cov_def(CV_BP_CL4, "bp: energy class 0xa9f");
  for (t = 0; t < 6000; t++) {
    int fs = rpct(50) ? 9600 : (rpct(80) ? 8000 : 7200);
    int mode = rpct(50) ? (int)rmod(5) : (int)rmod(7);
    int poke = rpct(40);
    int f = rpct(50) ? (rpct(50) ? 1209 : 697) + rrange(-20, 600) : rrange(100, 3900);
    int amp = rpct(8) ? 32767 : rpct(8) ? rrange(1, 40) : (int[]){100, 300, 600, 1200, 3000, 8000, 12000, 22000, 31000}[rmod(9)];
    int dc = rpct(15) ? rrange(-4000, 4000) : 0;
    int noise = rpct(30) ? amp / 3 : 0;
    char tag[16];
    snprintf(tag, sizeof tag, "B%d", t);
    dtmf_clear(fs, mode, poke);
    g_ph1 = rnd();
    for (c = 0; c < 5; c++) {
      int n = rpct(5) ? rrange(-3, 3) : rpct(60) ? (rpct(50) ? 80 : 96) : rrange(1, 198);
      unsigned i;
      int r, kind = (int)rmod(10);
      g_sn = 0;
      if (n > 0) {
        if (kind < 5) gen_tone2((unsigned)n, f, 0, amp, 0, fs, noise, dc);
        else if (kind == 5) gen_noise((unsigned)n, amp);
        else if (kind == 6) for (i = 0; i < (unsigned)n; i++) put(rshort());
        else if (kind == 7) gen_tone2((unsigned)n, f, f * 2 + 31, amp, amp / 2, fs, 0, dc);
        else if (kind == 8) { gen_tone2((unsigned)n, f, 0, amp, 0, fs, 0, 0); for (i = 0; i < (unsigned)n; i++) if (rpct(6)) g_sig[i] = (short)(rpct(50) ? 25000 : -22000); }
        else gen_tone2((unsigned)n, f, 0, 0, 0, fs, 0, dc);
        for (i = 0; i < (unsigned)n; i++) g_bb[i] = g_sig[i];
      }
      if (rpct(15)) g_d.minEn = (unsigned short)rmod(400);
      bp_model(g_bb, n);
      r = band_pass(g_bb, (short)n, &g_d);
      emit(t < 12, "%s c=%d n=%d f=%d amp=%d fs=%d mode=%d -> %d out=%016llx min=%u", tag, c, n, f, amp, fs, mode, r, n > 0 ? fnv_shorts(g_bb, (unsigned)n) : 0ULL, g_d.minEn);
      dump_dtmf(tag, 0, &g_d);
      if (r) cov_hit(CV_BP_ON); else cov_hit(CV_BP_OFF);
      if (n <= 0) cov_hit(CV_BP_NPOS_ZERO);
      if (n > 100 && r) cov_hit(CV_BP_MIN100);
      if (mode == 2) cov_hit(CV_BP_MODE2); else if (mode == 3) cov_hit(CV_BP_MODE3); else if (mode == 4) cov_hit(CV_BP_MODE4);
    }
    dump_dtmf(tag, t < 12, &g_d);
  }
  /* the scaling classes and counters, built directly: counts of |x| > 600 / 300 / 150 and > 20000 / 10000 */
  for (t = 0; t < 4000; t++) {
    int n = rrange(8, 192), i;
    int big = rrange(0, 6), mid = rrange(0, 6), small = rrange(0, 6), hi20 = rpct(20) ? (n >> 7) + rrange(1, 3) : 0, hi10 = rpct(20) ? (n >> 7) + rrange(1, 3) : 0;
    char tag[16];
    int r;
    snprintf(tag, sizeof tag, "K%d", t);
    for (i = 0; i < n; i++) g_bb[i] = (short)rrange(-100, 100);
    for (i = 0; i < big; i++) g_bb[rmod((unsigned)n)] = (short)(rpct(50) ? rrange(601, 1500) : -rrange(601, 1500));
    for (i = 0; i < mid; i++) g_bb[rmod((unsigned)n)] = (short)(rpct(50) ? rrange(301, 600) : -rrange(301, 600));
    for (i = 0; i < small; i++) g_bb[rmod((unsigned)n)] = (short)(rpct(50) ? rrange(151, 300) : -rrange(151, 300));
    for (i = 0; i < hi20; i++) g_bb[rmod((unsigned)n)] = (short)rrange(20001, 32767);
    for (i = 0; i < hi10; i++) g_bb[rmod((unsigned)n)] = (short)rrange(10001, 20000);
    if (rpct(10)) g_bb[rmod((unsigned)n)] = -32768;
    bp_model(g_bb, n);
    dtmf_clear(rpct(50) ? 9600 : 8000, (int)rmod(5), rpct(50));
    r = band_pass(g_bb, (short)n, &g_d);
    emit(t < 8, "%s n=%d -> %d out=%016llx", tag, n, r, fnv_shorts(g_bb, (unsigned)n));
    dump_dtmf(tag, 0, &g_d);
    if (big == 0 && mid == 0) cov_hit(CV_BP_S1);
  }
  /* scaling classes derived from the outcome by value probes (counters in the harness cannot see inside): count which blocks the classes apply to by a model */
  for (t = 0; t < 2000; t++) {
    int n = rrange(8, 192), i, a = 0, b = 0, cc = 0;
    int hi = rpct(50) ? 0 : 1;
    for (i = 0; i < n; i++) {
      int m = rrange(-60, 60);
      g_bb[i] = (short)m;
    }
    if (hi) for (i = 0; i < rrange(1, 12); i++) g_bb[rmod((unsigned)n)] = (short)((rpct(50) ? 1 : -1) * rrange(100, 800));
    {
      /* model of the post-DC amplitudes only to classify the case: the mean is ~0 here */
      for (i = 0; i < n; i++) {
        int m = g_bb[i] < 0 ? -g_bb[i] : g_bb[i];
        if (m > 0x258) { a++; b++; cc++; } else if (m > 0x12c) { a++; b++; } else if (m > 0x96) a++;
      }
      if (a <= 2) cov_hit(CV_BP_S6); else if (b <= 2) cov_hit(CV_BP_S3); else if (cc <= 2) cov_hit(CV_BP_S2); else cov_hit(CV_BP_S1);
    }
    dtmf_clear(8000, (int)rmod(5), 0);
    emit(0, "Q%d -> %d out=%016llx", t, band_pass(g_bb, (short)n, &g_d), fnv_shorts(g_bb, (unsigned)n));
    dump_dtmf("Q", 0, &g_d);
  }
  for (t = 0; t < 300; t++) { /* big blocks: the count thresholds (n >> 7) */
    int n = rrange(129, 198), i, k = rrange(0, 4);
    for (i = 0; i < n; i++) g_bb[i] = (short)rrange(-500, 500);
    for (i = 0; i < k; i++) g_bb[rmod((unsigned)n)] = (short)rrange(10001, 32767);
    dtmf_clear(9600, (int)rmod(5), 1);
    emit(0, "H%d -> %d out=%016llx", t, band_pass(g_bb, (short)n, &g_d), fnv_shorts(g_bb, (unsigned)n));
    dump_dtmf("H", t < 3, &g_d);
  }
  /* energy classes: tone amplitude sweep at a fixed block (class boundaries u = 0x157 / 0x2af / 0x55f) */
  for (t = 20; t <= 32000; t += (t < 2000 ? 7 : 61)) {
    int n = 160;
    unsigned i;
    unsigned long long e = 0;
    int r;
    g_sn = 0; g_ph1 = 0;
    gen_tone2((unsigned)n, 1209, 0, t, 0, 8000, 0, 0);
    for (i = 0; i < (unsigned)n; i++) { g_bb[i] = g_sig[i]; }
    dtmf_clear(8000, t % 5, t & 1);
    for (i = 0; i < (unsigned)n; i++) e += (unsigned long long)((int)g_bb[i] * (int)g_bb[i]);
    r = band_pass(g_bb, (short)n, &g_d);
    emit(0, "E%d -> %d", t, r);
    dump_dtmf("E", 0, &g_d);
    { unsigned u = (unsigned)((e + 0x20000u) >> 18); if (u > 0x55f) cov_hit(CV_BP_CL4); else if (u > 0x2af) cov_hit(CV_BP_CL3); else if (u > 0x157) cov_hit(CV_BP_CL2); else cov_hit(CV_BP_CL1); }
  }
}

/* ------------------------------------------------------------------------------------------------ 4: DTMF_MTD_detect */
static void sc_dmtd(void)
{
  int t, c, k;
  cov_def(CV_DT_NONE, "dtmf_mtd: -9"); cov_def(CV_DT_9600, "dtmf_mtd: 9600"); cov_def(CV_DT_8000, "dtmf_mtd: 8000");
  for (k = 0; k < 16; k++) { static char nm[16][32]; snprintf(nm[k], sizeof nm[k], "dtmf_mtd: digit code %d", k); cov_def(CV_DT_DIG0 + k, nm[k]); }
  for (t = 0; t < 6000; t++) {
    int fs = (t % 3 == 0) ? 9600 : (t % 3 == 1) ? 8000 : (rpct(50) ? 8000 : 7777);
    int mode = (int)rmod(4);
    int poke = rpct(30);
    int a1 = rpct(10) ? rrange(100, 800) : rrange(1500, 9000), a2 = rpct(25) ? a1 / 2 + rrange(0, 600) : a1;
    int noise = rpct(25) ? a1 / 3 : 0;
    char tag[16];
    snprintf(tag, sizeof tag, "T%d", t);
    dtmf_clear(fs, mode, 0);
    if (poke) for (c = 0; c < 8; c++) { g_d.iirMtd[c][0] = (short)(rshort() >> 3); g_d.iirMtd[c][1] = (short)(rshort() >> 3); }
    if (poke) { g_d.iirLow0[0] = rshort() >> 3; g_d.iirLow1[1] = rshort() >> 3; }
    g_ph1 = rnd(); g_ph2 = rnd();
    for (c = 0; c < 4; c++) {
      int code = rpct(80) ? dtmf_code[rmod(16)] : rrange(-2, 20);
      unsigned n = (unsigned)(rpct(70) ? rrange(80, 192) : rrange(1, 200));
      int r, kind = (int)rmod(12);
      g_sn = 0;
      if (kind < 7) gen_dtmf(n, code, fs == 7777 ? 8000 : fs, a1, a2, noise);
      else if (kind == 7) gen_noise(n, a1);
      else if (kind == 8) gen_tone2(n, rrange(300, 2000), 0, a1, 0, fs == 7777 ? 8000 : fs, 0, 0);
      else if (kind == 9) gen_tone2(n, dtmf_lo[rmod(4)], dtmf_lo[rmod(4)], a1, a2, fs == 7777 ? 8000 : fs, 0, 0);
      else if (kind == 10) gen_tone2(n, dtmf_hi[rmod(4)], dtmf_hi[rmod(4)], a1, a2, fs == 7777 ? 8000 : fs, 0, 0);
      else gen_noise(n, 2);
      r = DTMF_MTD_detect(g_sig, (short)n, &g_d);
      emit(t < 12, "%s c=%d n=%u code=%d fs=%d -> %d", tag, c, n, code, fs, r);
      if (r == -9) cov_hit(CV_DT_NONE); else if (r >= 0 && r < 16) cov_hit(CV_DT_DIG0 + r);
      if (fs == 9600) cov_hit(CV_DT_9600);
      if (fs == 8000) cov_hit(CV_DT_8000);
      dump_dtmf(tag, 0, &g_d);
    }
    dump_dtmf(tag, t < 6, &g_d);
  }
  for (t = -2; t <= 0; t++) { dtmf_clear(8000, 0, 1); emit(1, "NEG n=%d -> %d", t, DTMF_MTD_detect(g_sig, (short)t, &g_d)); dump_dtmf("NEG", 1, &g_d); }
}

/* ------------------------------------------------------------------------------------------------ 5, 6: dtmf_modem */
static int g_dm_stats[8];
static void dm_cov(int r, int preFails, unsigned preLast)
{
  if (r == 3) { cov_hit(CV_DM_C); if (dsplibs_debug_level > 1) cov_hit(CV_DM_DBG); }
  if (g_d.phase == 3) cov_hit(CV_DM_TIMEOUT);
  if (g_d.fails == preFails + 1 && g_d.lastDig == preLast && preLast != 0xffffu) cov_hit(CV_DM_INVALID);
  if (g_d.mode == 1) cov_hit(CV_DM_MODE1); else cov_hit(CV_DM_MODE0);
  if (g_d.fs == 9600) cov_hit(CV_DM_9600);
  if (r == 1) cov_hit(CV_DM_R1); else if (r == 2) cov_hit(CV_DM_R2); else if (r == 3) cov_hit(CV_DM_R3); else if (r == -1) cov_hit(CV_DM_RM1);
  switch (g_d.phase) { case 0: cov_hit(CV_DM_PH0); break; case 1: cov_hit(CV_DM_PH1); break; case 2: cov_hit(CV_DM_PH2); break; case 3: cov_hit(CV_DM_PH3); break; default: break; }
  if (g_d.halfFlag) cov_hit(CV_DM_HALF);
  if (g_d.nDig > 0) cov_hit(CV_DM_DIGIT);
  if (g_d.nDig == 0x14) cov_hit(CV_DM_20);
  if (g_d.fails > 5) cov_hit(CV_DM_FAIL);
}
static void dm_define(int e2e)
{
  cov_def(CV_DM_R1, "dtmf_modem: result 1"); cov_def(CV_DM_R2, "dtmf_modem: result 2"); cov_def(CV_DM_R3, "dtmf_modem: result 3 ('C')"); cov_def(CV_DM_RM1, "dtmf_modem: result -1");
  cov_def(CV_DM_DIGIT, "dtmf_modem: digits stored"); cov_def(CV_DM_C, "dtmf_modem: 'C' seen"); cov_def(CV_DM_20, "dtmf_modem: 20 digits"); cov_def(CV_DM_HALF, "dtmf_modem: half-block stash path");
  cov_def(CV_DM_TIMEOUT, "dtmf_modem: time-out phase 3"); cov_def(CV_DM_FAIL, "dtmf_modem: > 5 failed blocks"); cov_def(CV_DM_PH0, "dtmf_modem: phase 0"); cov_def(CV_DM_PH1, "dtmf_modem: phase 1");
  cov_def(CV_DM_PH2, "dtmf_modem: phase 2"); cov_def(CV_DM_PH3, "dtmf_modem: phase 3"); if (e2e) cov_def(CV_DM_START, "dtmf_modem: mode 1 start 'D'"); cov_def(CV_DM_MODE1, "dtmf_modem: mode 1");
  cov_def(CV_DM_MODE0, "dtmf_modem: mode != 1"); cov_def(CV_DM_9600, "dtmf_modem: 9600"); if (e2e) cov_def(CV_DM_RESET, "dtmf_modem: reset_dtmf");
  cov_def(CV_DM_INVALID, "dtmf_modem: invalid digit code twice");
  if (e2e) { cov_def(CV_DM_DBG, "dtmf_modem: debug line"); cov_def(CV_DM_SHORT, "dtmf_modem: short glitch tones"); cov_def(CV_DM_NOISE, "dtmf_modem: noise bursts"); }
}
/* a caller-ID DTMF message: silence, 'D' (13), digits, 'C' (12) with tone / gap lengths in samples */
static void dtmf_message(int fs, int ndig, int withStart, int withEnd, int rand_codes, int amp, int noise)
{
  int i;
  int tone = fs * rrange(50, 110) / 1000, gap = fs * rrange(45, 110) / 1000;
  gen_noise((unsigned)rrange(fs / 20, fs / 3), rpct(50) ? 4 : 60);
  if (withStart) {
    gen_dtmf((unsigned)tone, 13, fs, amp, amp, noise);
    gen_noise((unsigned)gap, noise ? noise : 4);
  }
  for (i = 0; i < ndig; i++) {
    int code = rand_codes ? (rpct(70) ? dtmf_code[rmod(16)] : rrange(-1, 18)) : (int)rmod(10);
    int tn = tone + (rpct(10) ? rrange(-tone / 2, tone / 2) : 0), gp = gap + (rpct(10) ? rrange(-gap / 2, gap / 2) : 0);
    int tw = rpct(20) ? amp / 2 : amp;
    if (rpct(5)) { gen_dtmf((unsigned)rrange(fs / 100, fs / 40), code, fs, amp, tw, noise); cov_hit(CV_DM_SHORT); }
    gen_dtmf((unsigned)tn, code, fs, amp, tw, noise);
    gen_noise((unsigned)gp, noise ? noise : 4);
    if (rpct(4)) { gen_noise((unsigned)rrange(fs / 50, fs / 5), amp); cov_hit(CV_DM_NOISE); }
  }
  if (withEnd) {
    gen_dtmf((unsigned)tone, 12, fs, amp, amp, noise);
    gen_noise((unsigned)gap, noise ? noise : 4);
  }
}
static void dm_run(const char *tag, int show, int fs, int mode, int frame, int pre_reset)
{
  unsigned pos = 0;
  int calls = 0, r;
  unsigned long long h = 1469598103934665603ULL;
  int rr[8] = { 0 };
  int preF = 0;
  unsigned preL = 0;
  if (pre_reset) {
    memset(&g_d, 0, sizeof g_d);
    g_d.fs = (unsigned short)fs;
    g_d.mode = (short)mode;
    reset_dtmf(&g_d);
    cov_hit(CV_DM_RESET);
  }
  if (mode == 1) cov_hit(CV_DM_MODE1); else cov_hit(CV_DM_MODE0);
  if (fs == 9600) cov_hit(CV_DM_9600);
  while (pos < g_sn) {
    unsigned n = (unsigned)(frame > 0 ? frame : rrange(8, 198));
    static short fr[400];
    unsigned i;
    if (pos + n > g_sn) break;
    for (i = 0; i < n; i++) fr[i] = g_sig[pos + i];
    { preF = g_d.fails; preL = g_d.lastDig; }
    r = dtmf_modem(fr, (unsigned short)n, &g_d);
    h ^= (unsigned)r; h *= 1099511628211ULL;
    emit(show && calls < 60, "%s call=%d n=%u -> %d ph=%d nd=%d", tag, calls, n, r, g_d.phase, g_d.nDig);
    if (calls < 40 || (r != 1) || (calls & 31) == 0) dump_dtmf(tag, 0, &g_d);
    dm_cov(r, preF, preL);
    if (r >= -1 && r <= 3 && r != 0) rr[r + 1]++;
    if (mode == 1 && g_d.nDig >= 0 && g_d.phase != 0 && g_d.phase != 1) cov_hit(CV_DM_START);
    pos += n;
    calls++;
  }
  dump_dtmf(tag, show, &g_d);
  emit(show, "%s fs=%d mode=%d frame=%d calls=%d h=%016llx r=%d,%d,%d,%d", tag, fs, mode, frame, calls, h, rr[0], rr[2], rr[3], rr[4]);
}
static void sc_dm(void)
{
  int t;
  dm_define(1);
  for (t = 0; t < 700; t++) {
    int fs = rpct(50) ? 8000 : 9600;
    int mode = (t % 5 == 0) ? (int)rmod(5) : 1 - (int)(t % 2) * (int)rmod(2) * 0 ;
    int frame = rpct(50) ? (fs == 8000 ? 160 : 192) : rpct(50) ? 0 : rrange(40, 198);
    int amp = rpct(10) ? rrange(500, 2000) : rrange(2500, 9000);
    int noise = rpct(25) ? amp / 6 : 0;
    char tag[16];
    int nd = rpct(10) ? rrange(15, 30) : rrange(3, 12);
    snprintf(tag, sizeof tag, "D%d", t);
    if (t % 3 == 0) mode = 1;
    g_sn = 0; g_ph1 = rnd(); g_ph2 = rnd();
    dtmf_message(fs, nd, mode == 1 || rpct(60), rpct(80), t % 4 == 3, amp, noise);
    if (rpct(15)) gen_noise((unsigned)(fs * rrange(1, 4)), rpct(50) ? 3 : 200); /* time-out tail */
    dm_run(tag, t < 6, fs, mode, frame, rpct(90));
  }
  /* time-outs: a long run of frames after some digits, and a start tone that never ends */
  for (t = 0; t < 40; t++) {
    int fs = t & 1 ? 8000 : 9600;
    char tag[16];
    snprintf(tag, sizeof tag, "O%d", t);
    g_sn = 0; g_ph1 = rnd(); g_ph2 = rnd();
    dtmf_message(fs, rrange(3, 6), 1, 0, 0, 6000, 0);
    gen_noise((unsigned)(fs * 5), 3);
    dm_run(tag, t < 2, fs, 1 + (int)rmod(2) * 0, t % 3 ? (fs == 8000 ? 160 : 192) : 0, 1);
  }
  dsplibs_debug_level = 2;
  for (t = 0; t < 30; t++) {
    int fs = t & 1 ? 8000 : 9600;
    char tag[16];
    snprintf(tag, sizeof tag, "G%d", t);
    g_sn = 0; g_ph1 = rnd(); g_ph2 = rnd();
    dtmf_message(fs, rrange(3, 9), 1, 1, 0, 7000, 0);
    dm_run(tag, t < 2, fs, 1, fs == 8000 ? 160 : 192, 1);
  }
  dsplibs_debug_level = 0;
}
/* every member of the object gets a random value, a FIXED number of PRNG draws (the object is bigger on x86-64: the stream must not depend on sizeof) */
static void dtmf_junk(void)
{
  int i;
  memset(&g_d, 0, sizeof g_d);
  g_d.state0 = rshort(); g_d.pad002 = rshort(); g_d.w004 = (int)rnd(); g_d.s008 = rshort();
  for (i = 0; i < 300; i++) g_d.blk[i] = rshort();
  for (i = 0; i < 100; i++) g_d.stash[i] = rshort();
  g_d.lastDig = (unsigned short)rnd(); g_d.cnt = rshort(); g_d.nDig = rshort(); g_d.phase = rshort(); g_d.halfFlag = rshort(); g_d.fails = rshort(); g_d.minEn = (unsigned short)rnd();
  g_d.blkp = NULL; g_d.fs = (unsigned short)rnd(); g_d.mode = rshort();
  for (i = 0; i < 20; i++) g_d.digits[i] = (char)rnd();
  g_d.s354 = rshort(); g_d.s356 = rshort(); g_d.iirBand[0] = rshort(); g_d.iirBand[1] = rshort(); g_d.s35c[0] = rshort(); g_d.s35c[1] = rshort();
  g_d.iirLow0[0] = rshort(); g_d.iirLow0[1] = rshort(); g_d.iirLow1[0] = rshort(); g_d.iirLow1[1] = rshort();
  for (i = 0; i < 16; i++) g_d.iirMtd[i / 2][i & 1] = rshort();
  g_d.samples = (int)rnd();
}
static void sc_dm_poke(void)
{
  int t;
  dm_define(0);
  cov_def(CV_RS_RUN, "reset: junk objects reset"); cov_def(CV_RS_DBG, "reset: debug line");
  /* reset_dtmf over junk objects */
  for (t = 0; t < 60; t++) {
    CidDtmf save;
    dtmf_junk();
    dsplibs_debug_level = (t & 1) ? 2 : 0;
    reset_dtmf(&g_d);
    if (t & 1) cov_hit(CV_RS_DBG);
    cov_hit(CV_RS_RUN);
    dump_dtmf("R", t < 4, &g_d);
    (void)save;
  }
  dsplibs_debug_level = 0;
  for (t = 0; t < 5000; t++) {
    int fs = rpct(50) ? 8000 : 9600, c;
    int mode = (int)rmod(5);
    char tag[16];
    snprintf(tag, sizeof tag, "P%d", t);
    memset(&g_d, 0, sizeof g_d);
    reset_dtmf(&g_d);
    g_d.fs = (unsigned short)fs;
    g_d.mode = (short)mode;
    g_d.phase = (short)(rpct(70) ? (int)rmod(4) : rrange(-1, 5));
    g_d.nDig = (short)(rpct(60) ? rrange(-1, 4) : rrange(-1, 22));
    if (g_d.nDig > 17) g_d.nDig = (short)(rpct(70) ? 19 : g_d.nDig);
    if (g_d.nDig >= 20 || g_d.nDig < -1) g_d.nDig = (short)(rpct(50) ? 19 : -1);
    g_d.cnt = (short)rrange(-2, 4);
    g_d.lastDig = (unsigned short)(rpct(70) ? dtmf_code[rmod(16)] : rpct(50) ? 0xffff : (int)rmod(30));
    g_d.fails = (short)rrange(0, 7);
    g_d.halfFlag = (short)rmod(2);
    g_d.samples = rpct(40) ? 0 : rpct(50) ? rrange(0, fs * 4) : fs * 3 + rrange(-200, 200);
    if (rpct(25)) g_d.samples = (int)(fs * 300u / 1000u) + rrange(-200, 200);
    g_d.minEn = (unsigned short)(rpct(40) ? 1 : rmod(200));
    for (c = 0; c < 96; c++) g_d.stash[c] = (short)(rshort() >> 4);
    for (c = 0; c < 8; c++) { g_d.iirMtd[c][0] = (short)(rshort() >> 4); g_d.iirMtd[c][1] = (short)(rshort() >> 4); }
    g_ph1 = rnd(); g_ph2 = rnd();
    for (c = 0; c < 8; c++) {
      static short fr[400];
      int n = rpct(70) ? (fs == 8000 ? 160 : 192) : rrange(2, 198);
      int code = rpct(70) ? dtmf_code[rmod(16)] : rrange(-2, 20);
      int amp = rpct(20) ? 700 : 6000, r;
      int pF;
      unsigned pL;
      unsigned i;
      g_sn = 0;
      if (rpct(80)) gen_dtmf((unsigned)n, code, fs, amp, amp, 0);
      else if (rpct(50)) gen_noise((unsigned)n, rpct(50) ? amp : 4);
      else { gen_dtmf((unsigned)(n / 2), code, fs, amp, amp, 0); gen_noise((unsigned)(n - n / 2), 4); }
      for (i = 0; i < (unsigned)n; i++) fr[i] = g_sig[i];
      pF = g_d.fails; pL = g_d.lastDig;
      r = dtmf_modem(fr, (unsigned short)n, &g_d);
      emit(t < 10, "%s c=%d n=%d code=%d -> %d", tag, c, n, code, r);
      dump_dtmf(tag, 0, &g_d);
      dm_cov(r, pF, pL);
      if (g_d.nDig > 17) cov_hit(CV_DM_20);
    }
    dump_dtmf(tag, t < 10, &g_d);
  }
}

/* ------------------------------------------------------------------------------------------------ 7: boundary sweeps and gap closers (R13) */
static struct { char pre[4096]; CidDtmf d; char post[4096]; } g_w;
/* only the port defines this symbol (the blob's table is file-local): the port must DROP a digit store outside the object, the blob writes it into the slack */
extern const short cid_MTD_COEF_1_8000[5] __attribute__((weak));
static void slack_check(void)
{
  unsigned i;
  if (&cid_MTD_COEF_1_8000[0] == NULL) return;
  for (i = 0; i < sizeof g_w.pre; i++) if (g_w.pre[i]) die3("port wrote behind the object start (digit_put guard)");
  for (i = 0; i < sizeof g_w.post; i++) if (g_w.post[i]) die3("port wrote behind the object end (digit_put guard)");
}
static void sc_gap(void)
{
  int t, c, a, k;
  static const int idxs[] = { -1, 0, 19, 20, 21, 25, 74, 75, 76, 77, 100, 255, 1000, 3000, -0x340, -0x341, -0x342, -800, -2000, -0x33f, -0x33e, 0x4c - 1 };
  cov_def(CV_GAP_OUT, "gap: digit stored outside the object"); cov_def(CV_GAP_IN, "gap: digit stored inside the object, far from digits[]"); cov_def(CV_GAP_N0, "gap: CID_FSD_demodulate n == 0");
  cov_def(CV_GAP_MTD, "gap: MTD sweep flips"); cov_def(CV_GAP_DT, "gap: DTMF sweep flips"); cov_def(CV_GAP_BP, "gap: band_pass sweep flips");
  /* guard of digit_put */
  for (t = 0; t < 22 * 12; t++) {
    int idx = idxs[t % 22], fs = (t & 1) ? 9600 : 8000, n = fs == 8000 ? 160 : 192, mode = (t / 19) % 5 < 3 ? 1 : 0;
    CidDtmf *pd = &g_w.d;
    char tag[24];
    snprintf(tag, sizeof tag, "GD%d", t);
    memset(&g_w, 0, sizeof g_w);
    reset_dtmf(pd);
    pd->fs = (unsigned short)fs; pd->mode = (short)mode;
    pd->phase = (short)(rpct(85) ? 2 : (int)rmod(4));
    pd->nDig = (short)idx;
    pd->lastDig = 0xffff;
    g_ph1 = rnd(); g_ph2 = rnd();
    for (c = 0; c < 8; c++) {
      static short fr[400];
      int code = (c % 4 == 3 && t % 3 == 0) ? 12 : (int)rmod(10), r, pre = pd->nDig;
      unsigned i;
      long off;
      g_sn = 0;
      if (c & 1) gen_noise((unsigned)n, 4); else gen_dtmf((unsigned)n, code, fs, 6000, 6000, 0);
      for (i = 0; i < (unsigned)n; i++) fr[i] = g_sig[i];
      r = dtmf_modem(fr, (unsigned short)n, pd);
      emit(t < 24, "%s c=%d idx=%d code=%d -> %d nd=%d", tag, c, idx, code, r, pd->nDig);
      dump_dtmf(tag, 0, pd);
      slack_check();
      off = (long)offsetof(CidDtmf, digits) + (long)pre;
      if (pd->nDig == pre + 1 && (off < 0 || off >= (long)sizeof(CidDtmf))) cov_hit(CV_GAP_OUT);
      if (pd->nDig == pre + 1 && off >= 20 + (long)offsetof(CidDtmf, digits) && off < (long)sizeof(CidDtmf)) cov_hit(CV_GAP_IN);
    }
    dump_dtmf(tag, t < 24, pd);
  }
  /* CID_FSD_demodulate with n == 0 (nothing consumed, nothing produced) between real blocks */
  for (t = 0; t < 120; t++) {
    int fs = g_fsk_fs[t % 3];
    unsigned short out[200];
    int r, i;
    char tag[24];
    snprintf(tag, sizeof tag, "N0.%d", t);
    fsk_init(fs, 0);
    if (t & 1) fsk_poke();
    g_f.fs = (unsigned short)fs;
    g_sn = 0; g_nbits = 0; g_ph1 = rnd();
    for (i = 0; i < 80; i++) bits_push(rpct(50));
    gen_fsk(fs, 1200, 2200, 8000, 0, 0, 1200);
    for (c = 0; c < 4; c++) {
      memset(out, 0xee, sizeof out);
      r = CID_FSD_demodulate(g_sig + c * 100, out, (short)(c == 1 || c == 3 ? 0 : 100), &g_f);
      emit(t < 6, "%s c=%d -> %d out0=%04x", tag, c, r, out[0]);
      if (c == 1 || c == 3) { cov_hit(CV_GAP_N0); if (r != 0 || out[0] != 0xeeee) die3("FSD n == 0 produced output"); }
      dump_fsk(tag, 0, &g_f);
    }
  }
  /* unit-step sweeps: the decisions flip at one amplitude; +-1 / +-2 count offsets of the sums move the flip point by a step */
  for (k = 0; k < 2; k++) {
    int fs = k ? 9600 : 8000, last = -1, flips = 0;
    static const int pairs[4][2] = { { 1200, 2200 }, { 2200, 1200 }, { 1300, 2100 }, { 2100, 1300 } };
    int pi;
    for (pi = 0; pi < 4; pi++) {
      static const int a1s[3] = { 700, 2000, 6000 };
      int ai;
      for (ai = 0; ai < 3; ai++) {
        int b, step = a1s[ai] * 3 / 1500 > 0 ? a1s[ai] * 3 / 1500 : 1;
        last = -1;
        for (b = 0; b <= a1s[ai] * 3; b += step) {
          int r;
          fsk_init(fs, 0);
          g_sn = 0; g_ph1 = 0; g_ph2 = 0;
          gen_tone2(160, pairs[pi][0], pairs[pi][1], a1s[ai], b, fs, 0, 0);
          r = CID_MTD_detect(g_sig, 160, &g_f);
          emit(0, "SM %d %d %d %d -> %d", fs, pi, a1s[ai], b, r);
          dump_fsk("SM", 0, &g_f);
          if (last >= 0 && r != last) { flips++; cov_hit(CV_GAP_MTD); }
          last = r;
        }
      }
    }
    for (a = 1; a < 500; a++) { /* single in-band tone through the absolute floor (150) */
      int r;
      fsk_init(fs, 0);
      g_sn = 0; g_ph1 = 0;
      gen_tone2(160, 1200, 0, a, 0, fs, 0, 0);
      r = CID_MTD_detect(g_sig, 160, &g_f);
      emit(0, "SF %d %d -> %d", fs, a, r);
    }
    emit(1, "mtd sweeps fs=%d flips=%d", fs, flips);
  }
  for (k = 0; k < 2; k++) {
    int fs = k ? 9600 : 8000, mode, di, flips = 0, last;
    for (mode = 0; mode < 5; mode++) for (di = 0; di < 4; di++) {
      static const int codes[4] = { 1, 5, 9, 15 };
      int w;
      for (w = 0; w < 2; w++) {
        for (a = 0; a <= 3000; a += (a < 1500 ? 1 : 5)) {
          int r;
          dtmf_clear(fs, mode, 0);
          g_sn = 0; g_ph1 = 0; g_ph2 = 0;
          gen_dtmf(160, codes[di], fs, w ? 1500 : a, w ? a : 1500, 0);
          r = DTMF_MTD_detect(g_sig, 160, &g_d);
          emit(0, "SD %d %d %d %d %d -> %d", fs, mode, di, w, a, r);
          dump_dtmf("SD", 0, &g_d);
          if (a && r != last) { flips++; cov_hit(CV_GAP_DT); }
          last = r;
        }
      }
    }
    emit(1, "dtmf sweeps fs=%d flips=%d", fs, flips);
  }
  for (k = 0; k < 2; k++) {
    int fs = k ? 9600 : 8000, mode, flips = 0, last = 0;
    for (mode = 0; mode < 5; mode++) {
      int f;
      for (f = 0; f < 3; f++) {
        static const int fq[3] = { 1209, 697, 941 };
        for (a = 5; a <= 4000; a += (a < 1500 ? 1 : 7)) {
          int r;
          unsigned i;
          g_sn = 0; g_ph1 = 0;
          gen_tone2(160, fq[f], 0, a, 0, fs, 0, 0);
          for (i = 0; i < 160; i++) g_bb[i] = g_sig[i];
          dtmf_clear(fs, mode, 0);
          r = band_pass(g_bb, 160, &g_d);
          emit(0, "SB %d %d %d %d -> %d %016llx", fs, mode, f, a, r, fnv_shorts(g_bb, 160));
          dump_dtmf("SB", 0, &g_d);
          if (a > 5 && r != last) { flips++; cov_hit(CV_GAP_BP); }
          last = r;
        }
      }
    }
    emit(1, "band_pass sweeps fs=%d flips=%d", fs, flips);
  }
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("R12_SEED");
  if (seed) g_lcg ^= (unsigned)strtoul(seed, NULL, 0) * 2654435761u;
  g_lcg ^= (unsigned)sc * 0x9e3779b9u;
  switch (sc) {
    case 0: sc_div(); break;
    case 1: sc_mtd(); break;
    case 2: sc_fsd(); break;
    case 3: sc_bp(); break;
    case 4: sc_dmtd(); break;
    case 5: sc_dm(); break;
    case 6: sc_dm_poke(); break;
    case 7: sc_gap(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  (void)g_what;
  cov_finish();
  return 0;
}
