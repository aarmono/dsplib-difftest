/*
 * Differential test harness for port chunk P04 (src/dsp/dsp_tone_detectors.cpp): GenericToneDetector (C1 C2 D1 D2 process(float)
 * process(float*, unsigned) reset) and ANSamToneDetector (C1 C2 D1 D2) of dsplibs.o, with the four 2100 Hz coefficient tables.
 *
 * ONE C source, no #ifdef, built by test/dsp/dsp_tone_detectors.sh against the blob (dsplibs.o), against the port (-O0 / -O2; the P02 port
 * object supplies the GenericIIR in the port builds) and for x86-64 (the 64-bit run is diffed against the 32-bit BLOB's output = the
 * golden, test/golden/dsp/dsp_tone_detectors.blob32.out).  The 11 functions are reached through __asm__ labels with the Itanium-mangled names (identical in
 * blob and port, C1 AND C2 / D1 AND D2).  The shared harness library (test/harness/) provides the PRNG, hashed canonical output, coverage
 * counters (exit 2), the arena allocator with red zones, the object pool and the field-wise dump; the field tables and mirror structs of
 * the two classes are GENERATED (tools/layout_gen.py from test/dsp/dsp_tone_detectors.layout -> test/dsp/dsp_tone_detectors_layout.h), the GenericIIR's from test/p02.layout.
 *
 * What is compared: every return value (the detection flag), every object (all fields: floats as bit patterns, the heap GenericIIR
 * object field-wise through the P02 table, its history blocks), the malloc / free sequence (the GenericIIR object is logged with its
 * i386 size 0x34: h_log_next_size), the store order of the constructor (malloc-time snapshot), the red zones.
 *
 * INDEPENDENT MODELS (written from the disassembly facts, not from the port): a delay-line model of the GenericIIR<float,double>
 * (extended accumulation in tap order, one extended division, doubles in the history), and a model of the detector's energy path (x*x +
 * sum and y*y + sum in extended precision, the extended reciprocal, V14 / V18 and the two decision procedures: the single-sample one
 * and the block one with its hysteresis path).  The detector's float state is compared bit for bit after every call.
 *
 * TWO-PASS TRIALS: the energy path (X, Y, YR, V14, V18 at every period end) does not depend on the threshold, the ratio or the run
 * lengths, so a trial first runs the model alone over the planned operations, then DERIVES thresh / ratio from that trace (the float
 * equal to a period's YR, its neighbours, twice / half of it, V18 / V14, specials: NaN, inf, +-0, ...), which puts the comparisons exactly
 * on their boundaries, and then runs the real object and the model in lock step.
 *
 * Scenarios (each its own process): 0 process(float) vs model, 1 process(float*, n) vs model, 2 both on the same object (the state is shared) with
 * generic filters, 3 ANSamToneDetector (rate 8000 / 9600 / other, real tables, tone / noise data) through the Generic interface, 4
 * exact-arithmetic trials (identity filter, small integers, power-of-two periods: equality boundaries), 5 constructor / reset / destructor
 * details (ceil of the run lengths, degenerate arguments, NULL IIR pointer, NaN payload words, malloc order), 6 non-finite / special data
 * and coefficients incl. signalling NaN payload words (blob vs port only: the model only checks what it can).
 *
 * Excluded because the BLOB itself is undefined there: period == 0 (#DE in the constructor), a filter with nNum == 0 or nDen == 0
 * (the first sample is stored one element past the history, see P02), sysdep_malloc returning NULL (the blob calls the GenericIIR
 * constructor with NULL).
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "../p02_layout.h"
#include "dsp_tone_detectors_layout.h"

#define L(m) __asm__(m)
extern void td_c1(void *, unsigned, unsigned, double *, double *, unsigned, unsigned, float, unsigned, float, unsigned, unsigned) L("_ZN19GenericToneDetectorC1EjjPdS0_jjfjfjj");
extern void td_c2(void *, unsigned, unsigned, double *, double *, unsigned, unsigned, float, unsigned, float, unsigned, unsigned) L("_ZN19GenericToneDetectorC2EjjPdS0_jjfjfjj");
extern void td_d1(void *) L("_ZN19GenericToneDetectorD1Ev");
extern void td_d2(void *) L("_ZN19GenericToneDetectorD2Ev");
extern unsigned td_p1(void *, float) L("_ZN19GenericToneDetector7processEf");
extern unsigned td_pn(void *, float *, unsigned) L("_ZN19GenericToneDetector7processEPfj");
extern void td_reset(void *) L("_ZN19GenericToneDetector5resetEv");
extern void an_c1(void *, unsigned, unsigned, float, unsigned, float, unsigned, unsigned, unsigned) L("_ZN17ANSamToneDetectorC1Ejjfjfjjj");
extern void an_c2(void *, unsigned, unsigned, float, unsigned, float, unsigned, unsigned, unsigned) L("_ZN17ANSamToneDetectorC2Ejjfjfjjj");
extern void an_d1(void *) L("_ZN17ANSamToneDetectorD1Ev");
extern void an_d2(void *) L("_ZN17ANSamToneDetectorD2Ev");

typedef long double LD;
typedef uint64_t u64_;

/* ------------------------------------------------------------------------------------------------ bit helpers */
static uint32_t bits_f(float f) { return h_bits_of(f); }
static float f_of(uint32_t b) { float f; memcpy(&f, &b, 4); return f; } /* only ever fed to the x87 / SSE by value in model code: no signalling NaNs are produced here */
static uint64_t bits_d(double d) { uint64_t b; memcpy(&b, &d, 8); return b; }
static int is_nan_w(uint32_t b) { return (b & 0x7f800000u) == 0x7f800000u && (b & 0x007fffffu) != 0; }
static int is_snan_w(uint32_t b) { return is_nan_w(b) && !(b & 0x00400000u); }
static int is_fin_w(uint32_t b) { return (b & 0x7f800000u) != 0x7f800000u; }
static uint32_t flt_bits_of_ld(LD v) { float f = (float)v; return bits_f(f); }
/* the float word whose value is v rounded to float32 (round to nearest even), as an integer */
static unsigned ceil_div64(unsigned a, unsigned p) { return (unsigned)(((u64_)a + (u64_)p - 1u) / (u64_)p); }

#define MAXN 12000
#define MAXOPS 3000
static uint32_t g_xb[MAXN];  /* planned input samples as float bit patterns */
static float g_xf[MAXN];     /* the same words as floats (for the block process pointer; never used by value) */
static unsigned g_nx;
static LD tYR[MAXN], tV14[MAXN], tV18[MAXN]; /* trace of the period ends of the pass-1 run */
static unsigned g_ntr;

/* value generators: integers only until the final power-of-two scale (exact on i386 and x86-64) */
static float pow2f(int e) { /* 2^e as a float, e in [-126, 127], from the exponent field */
  uint32_t b = (uint32_t)(e + 127) << 23;
  return f_of(b);
}
static uint32_t float_of_int_scaled(int v, int e) { /* v * 2^e, |v| < 2^24, result exact (no under- / overflow for the exponents used) */
  float f = (float)v * pow2f(e);
  return bits_f(f);
}
static double rdouble_unit(void) { u64_ r = ((u64_)h_rnd() << 32) | h_rnd(); return (double)((int64_t)(r >> 11) - (1LL << 52)) / 4503599627370496.0; }
static double pow2d(int e) { u64_ b = (u64_)(e + 1023) << 52; double d; memcpy(&d, &b, 8); return d; }

/* ------------------------------------------------------------------------------------------------ the four tables (independent copy) */
static const uint64_t T_B9600[11] = {0x3e23dc96064ffe9eull, 0x0000000000000000ull, 0xbe48d3bb87e3fe43ull, 0x0000000000000000ull, 0x3e58d3bb87e3fe43ull, 0x0000000000000000ull, 0xbe58d3bb87e3fe43ull, 0x0000000000000000ull, 0x3e48d3bb87e3fe43ull, 0x0000000000000000ull, 0xbe23dc96064ffe9eull};
static const uint64_t T_B8000[13] = {0x3f63ba735227ff4full, 0x3f5ff249bdcd9eadull, 0x3f87e321e08bdc24ull, 0x3f7f350d17cdf724ull, 0x3f9807bc8043dd6bull, 0x3f88ac27b2f1de69ull, 0x3f99a4e76833e335ull, 0x3f83ab4397ec8736ull, 0x3f8e9027b2c2a7beull, 0x3f6f8e50db1089cbull, 0x3f733e124fda1ae7ull, 0x3f44584ab91c97b0ull, 0x3f43f20703decdd8ull};
static const uint64_t T_A9600[11] = {0x3ff0000000000000ull, 0xbfff18d9b1ef5d43ull, 0x4019df7c21317446ull, 0xc020982b98a99851ull, 0x402ce43cae775973ull, 0xc0294c9136e33c96ull, 0x402cacd1f9a01876ull, 0xc02058bdbb9a2908ull, 0x40194bb482190355ull, 0xbffe2cdd8bcfdaa9ull, 0x3feed19a3b0b07e1ull};
static const uint64_t T_A8000[13] = {0x3ff0000000000000ull, 0x3feca73d2b8a9e16ull, 0x401811a0c557b7d1ull, 0x40114792a49b8339ull, 0x402da041f1d25b4bull, 0x4020a273c7282dc0ull, 0x40331f3a3604953bull, 0x401ff38215cb4ef9ull, 0x402b4fe16b1b931dull, 0x400e9bbc781cdc2eull, 0x401472ffbf79c5ccull, 0x3fe764d0cc9ab2a7ull, 0x3fe90a072a398914ull};
static double g_tabA[13], g_tabB[13]; /* the tables selected for a rate, as doubles (model) */
static unsigned g_tabN;
static void select_tables(unsigned rate) {
  unsigned i;
  g_tabN = (rate == 8000u) ? 13u : 11u;
  for (i = 0; i < g_tabN; i++) {
    u64_ a = (rate == 8000u) ? T_A8000[i] : T_A9600[i], b = (rate == 8000u) ? T_B8000[i] : T_B9600[i];
    memcpy(&g_tabA[i], &a, 8);
    memcpy(&g_tabB[i], &b, 8);
  }
}
/* which table a borrowed pointer points to, judged by its CONTENT (the address differs between the blob and the port): 0 none, 1 B9600 .. */
static int table_id(const void *p, int numerator) {
  if (!p) return 0;
  if (numerator) {
    if (!memcmp(p, T_B9600, sizeof T_B9600)) return 1;
    if (!memcmp(p, T_B8000, sizeof T_B8000)) return 2;
  } else {
    if (!memcmp(p, T_A9600, sizeof T_A9600)) return 3;
    if (!memcmp(p, T_A8000, sizeof T_A8000)) return 4;
  }
  return -1;
}

/* ================================================================================================ the models */
/* coverage ids */
enum {
  CT_TRIAL = 0, CT_C2, CT_D2, CT_P1, CT_PN, CT_PN0, CT_RESET, CT_ANS8000, CT_ANS9600, CT_ANSOTHER,
  CT_S_LOW, CT_S_HIT0, CT_S_HIT1, CT_S_RMISS, CT_B_HIT0, CT_B_HIT1, CT_B_RMISS, CT_B_HLOW, CT_B_HNORATIO, CT_B_HRATIO, CT_B_HHIT,
  CT_DET_SET, CT_DET_CLEAR, CT_MISSRESET, CT_NEED0, CT_MAXMISS0, CT_YR_EQ_THR, CT_YR_EQ_HALF, CT_RATIO_EQ, CT_THR_NAN, CT_THR_INF, CT_RATIO_NAN,
  CT_NONFIN, CT_PERIOD1, CT_CEIL_EXACT, CT_CEIL_ROUND, CT_NEEDGE2, CT_HITS_PROGRESS, CT_MULTI_PERIOD, CT_PERIOD_BIG, CT_DATA_TONE, CT_DATA_NOISE,
  CT_DATA_EXACT, CT_BLOCK_CROSS, CT_BLOCK_EXACT_PERIOD, CT_IDENTITY, CT_IIR, CT_FIR, CT_DTOR_NULL_IIR, CT_VERIFY, CT_RATIO_HI, CT_B_HYST_NAN, CT_S_UNORD,
  CT_DET_AFTER_RMISS_B, CT_RESET_MID, CT_SNAN_WORD, CT_BIGSUM
};

typedef struct { LD hx[16], hy[16]; } GiirModel; /* delay lines: hx[0] = newest input (as double), hy[0] = newest output */
typedef struct {
  /* parameters */
  unsigned nDen, nNum;
  double den[16], num[16];
  unsigned needHits, maxMiss, period, ratioTest;
  uint32_t thrW, ratioW;
  /* the GenericIIR model */
  double hx[16], hy[16];
  double lastY;  /* y_ after the last sample */
  int anyY;
  /* detector state */
  float s0, s1, v14, v18;
  unsigned cnt, hits, miss, det;
  int valid;     /* 0 after a non-finite value: state compared only where it is not NaN */
} TdModel;

static float mg_step(TdModel *m, float x) { /* GenericIIR<float,double>::process(float) */
  unsigned t;
  LD acc = 0;
  double d0, q;
  for (t = 15; t > 0; t--) m->hx[t] = m->hx[t - 1];
  m->hx[0] = (double)x;
  for (t = 0; t < m->nNum; t++) acc += (LD)m->num[t] * (LD)m->hx[t];
  for (t = 1; t < m->nDen; t++) acc -= (LD)m->den[t] * (LD)m->hy[t - 1];
  d0 = m->den[0];
  if (d0 < 0 || d0 > 0) q = (double)(acc / (LD)d0); else q = (double)acc;
  for (t = 15; t > 0; t--) m->hy[t] = m->hy[t - 1];
  m->hy[0] = q;
  m->lastY = q;
  m->anyY = 1;
  return (float)q;
}

static void tm_reset(TdModel *m) {
  memset(m->hx, 0, sizeof m->hx);
  memset(m->hy, 0, sizeof m->hy);
  m->s0 = m->s1 = m->v14 = m->v18 = 0.0f;
  m->cnt = m->hits = m->miss = m->det = 0;
  m->anyY = 0;
}

/* one period end: returns the event (for the coverage), updates hits / miss / det.  outcome 'H' hit, 'M' miss that joins the detection
 * test, 'N' plain miss */
typedef struct { int ev; int rmiss_join; int eqThr, eqHalf, eqRatio; } TdEv;
static TdEv tm_decide(TdModel *m, int block, LD YR, LD V14, LD V18) {
  TdEv e = { 0, 0, 0, 0, 0 };
  float thr = f_of(m->thrW), ratio = f_of(m->ratioW);
  char out;
  if (YR == (LD)thr) e.eqThr = 1;
  if (YR == (LD)thr * (LD)0.5L) e.eqHalf = 1;
  if (m->ratioTest && V18 == V14 * (LD)ratio) e.eqRatio = 1;
  if (YR >= (LD)thr) {
    if (!m->ratioTest) { out = 'H'; e.ev = block ? CT_B_HIT0 : CT_S_HIT0; }
    else if (V18 > V14 * (LD)ratio) { out = 'H'; e.ev = block ? CT_B_HIT1 : CT_S_HIT1; }
    else { out = 'M'; e.ev = block ? CT_B_RMISS : CT_S_RMISS; }
  } else if (!block) {
    out = 'N'; e.ev = CT_S_LOW;
  } else if ((LD)thr * (LD)0.5f > YR) { out = 'N'; e.ev = CT_B_HLOW; }
  else if (!m->ratioTest) { out = 'N'; e.ev = CT_B_HNORATIO; }
  else if (V14 * (LD)0.85 >= V18) { out = 'N'; e.ev = CT_B_HRATIO; }
  else { out = 'H'; e.ev = CT_B_HHIT; }
  if (out == 'H') {
    m->hits++;
    m->miss = 0;
    if (!(m->hits < m->needHits)) m->det = 1;
  } else {
    m->miss++;
    if (m->miss >= m->maxMiss) { m->hits = 0; m->det = 0; e.rmiss_join = -1; }
    if (out == 'M' && !(m->hits < m->needHits)) { m->det = 1; e.rmiss_join = 1; }
  }
  return e;
}

typedef struct { int ev; } TdStep;
static int g_last_ev, g_last_rj, g_last_eqThr, g_last_eqHalf, g_last_eqRatio, g_last_pe;
/* one sample through the detector model.  trace != 0: record YR / V14 / V18 of the period end */
static void tm_step(TdModel *m, int block, float x, int trace) {
  float y = mg_step(m, x);
  LD X = (LD)x * (LD)x + (LD)m->s0;
  LD Y = (LD)y * (LD)y + (LD)m->s1;
  g_last_pe = 0;
  m->cnt++;
  if (m->cnt != m->period) { m->s1 = (float)Y; m->s0 = (float)X; return; }
  {
    LD R = (LD)1.0f / (LD)m->cnt;
    LD XR = X * R, YR = Y * R;
    LD V14 = (LD)m->v14 * (LD)0.7f + XR * (LD)0.3f;
    LD V18 = YR * (LD)0.3f + (LD)0.7f * (LD)m->v18;
    TdEv e;
    m->v14 = (float)V14;
    m->v18 = (float)V18;
    if (trace) { if (g_ntr < MAXN) { tYR[g_ntr] = YR; tV14[g_ntr] = V14; tV18[g_ntr] = V18; g_ntr++; } }
    e = tm_decide(m, block, YR, V14, V18);
    g_last_pe = 1; g_last_ev = e.ev; g_last_rj = e.rmiss_join; g_last_eqThr = e.eqThr; g_last_eqHalf = e.eqHalf; g_last_eqRatio = e.eqRatio;
    m->cnt = 0;
    m->s0 = m->s1 = 0.0f;
  }
}

/* ================================================================================================ plans and data */
enum { OP_SINGLE = 0, OP_BLOCK, OP_RESET };
typedef struct { int kind; unsigned n; unsigned off; } Op;
static unsigned g_dump_every = 1; /* single-sample ops: dump the object every Nth op (the state is compared with the model after every op anyway) */
static int g_special_mode;  /* scenario 6: special data words in the segments */
static Op g_ops[MAXOPS];
static unsigned g_nops;

static const int SINE_Q14_8000[7] = { -2571, 23170, 31863, -29197, 0, -6393, 12540 };
static const int SINE_Q14_9600[7] = { 6393, 25997, 32138, -19948, 8481, 3212, 18205 };

enum { D_SILENT = 0, D_NOISE, D_SINE, D_CONST, D_IMPULSE, D_SQUARE, D_SMALLINT, D_BIG, D_NREG, D_SPECIAL = 100 };
/* special data words: inf, NaN with payloads (quiet and signalling), denormals, +-0, huge, small integers */
static uint32_t g_special_snan = 1;
static uint32_t special_data_word(void) {
  uint32_t sign = (h_rnd() & 1u) << 31, pay = h_rnd() & 0x007fffffu;
  switch (h_rmod(10)) {
    case 0: return sign | 0x7f800000u;
    case 1: return sign | 0x7fc00000u | pay;
    case 2: return g_special_snan ? (sign | 0x7f800000u | (pay | 1u)) : (sign | 0x7fc00000u | pay);
    case 3: return sign | (pay & 0x7fffffu);                  /* denormal / zero */
    case 4: return sign;                                       /* +-0 */
    case 5: return sign | ((uint32_t)(250 + h_rmod(5)) << 23) | pay; /* huge, may overflow in x*x */
    case 6: return sign | ((uint32_t)(1 + h_rmod(20)) << 23) | pay;  /* tiny: x*x underflows in the sums */
    default: return float_of_int_scaled((int)h_rmod(33) - 16, -(int)h_rmod(6));
  }
}
/* fill g_xb[base .. base+n) with a segment of the given regime and amplitude exponent */
static void gen_segment(unsigned base, unsigned n, int regime, int ampexp, unsigned rate) {
  unsigned i;
  int s1 = 0, s2 = 0, cq = (rate == 8000u) ? SINE_Q14_8000[h_rmod(7)] : SINE_Q14_9600[h_rmod(7)];
  int amp = 1000 + (int)h_rmod(28000), k = (int)h_rmod(5);
  int cv = (int)h_rmod(2001) - 1000;
  s1 = amp; s2 = 0;
  for (i = 0; i < n; i++) {
    int v = 0;
    switch (regime) {
      case D_SILENT: v = 0; break;
      case D_NOISE: v = (int)(h_rnd() >> 17) - 16384; break;
      case D_SINE: { int t = (int)(((int64_t)cq * s1) >> 14) - s2; s2 = s1; s1 = t; v = t; break; }
      case D_CONST: v = cv; break;
      case D_IMPULSE: v = (i % 37 == 5) ? amp : 0; break;
      case D_SQUARE: v = ((i / (unsigned)(2 + k)) & 1u) ? 12000 : -12000; break;
      case D_SMALLINT: v = (int)h_rmod(9) - 4; break;
      default: v = (int)(h_rnd() >> 17) - 16384; break;
    }
    if (regime == D_SPECIAL) { g_xb[base + i] = special_data_word(); continue; }
    if (regime == D_BIG) g_xb[base + i] = float_of_int_scaled(v, 52 + ampexp);
    else if (regime == D_SMALLINT) g_xb[base + i] = float_of_int_scaled(v, ampexp);
    else g_xb[base + i] = float_of_int_scaled(v, ampexp - 15);
    if (regime == D_SILENT && h_rpct(5)) g_xb[base + i] = 0x80000000u;
  }
}

/* choose the plan: mode 0 singles only, 1 blocks only, 2 mixed.  Fills g_ops / g_xb / g_xf.  `exact`: small integer data, power-of-two scale.
 * Returns the number of samples. */
static unsigned plan_make(int mode, unsigned period0, unsigned total, int exact, unsigned rate, int tone) {
  unsigned pos = 0, nseg = 0, segend = 0, period = period0 > 1000u ? 1000u : period0; /* huge periods: block sizes by a clamped value */
  g_nops = 0;
  g_nx = 0;
  while (pos < total && g_nops < MAXOPS - 4) {
    unsigned n, left = total - pos;
    int kind;
    if (pos >= segend) { /* new data segment */
      unsigned len = 20 + h_rmod(400);
      int regime, ampexp = (int)h_rmod(24) - 12;
      if (exact) { regime = D_SMALLINT; ampexp = (int)h_rmod(7) - 3; }
      else if (g_special_mode && h_rpct(65)) regime = D_SPECIAL;
      else if (tone && h_rpct(60)) regime = D_SINE;
      else { regime = (int)h_rmod(D_NREG); if (regime == D_BIG && !h_rpct(6)) regime = D_NOISE; }
      if (regime == D_SINE) h_cov_hit(CT_DATA_TONE);
      if (regime == D_NOISE) h_cov_hit(CT_DATA_NOISE);
      if (exact) h_cov_hit(CT_DATA_EXACT);
      if (regime == D_BIG) h_cov_hit(CT_BIGSUM);
      if (len > left) len = left;
      gen_segment(pos, len, regime, ampexp, rate);
      segend = pos + len;
      nseg++;
    }
    kind = (mode == 0) ? OP_SINGLE : (mode == 1) ? OP_BLOCK : (h_rpct(40) ? OP_SINGLE : OP_BLOCK);
    if (kind == OP_BLOCK) {
      unsigned r = h_rmod(100);
      if (r < 8) n = 0;
      else if (r < 20) n = 1;
      else if (r < 40) n = 1 + h_rmod(period + 2);
      else if (r < 55) n = period;
      else if (r < 65) n = period + (h_rpct(50) ? 1u : 0u) - (period > 1 && h_rpct(50) ? 1u : 0u);
      else if (r < 85) n = period * (1 + h_rmod(4));
      else n = 1 + h_rmod(period * 6 + 20);
      if (n > left) n = left;
      if (n > segend - pos && n > 0) { /* a block may span two segments: make sure the data exists */
        unsigned len2 = n - (segend - pos), base2 = segend;
        gen_segment(base2, len2, exact ? D_SMALLINT : g_special_mode ? D_SPECIAL : (tone ? D_SINE : D_NOISE), exact ? 0 : (int)h_rmod(10) - 8, rate);
        segend = pos + n;
      }
      if (n > period0) h_cov_hit(CT_BLOCK_CROSS);
      if (n != 0 && period0 != 0 && n % period0 == 0) h_cov_hit(CT_BLOCK_EXACT_PERIOD);
    } else {
      n = 1;
    }
    g_ops[g_nops].kind = kind;
    g_ops[g_nops].n = n;
    g_ops[g_nops].off = pos;
    g_nops++;
    pos += n;
    if (h_rpct(4) && pos < total) { g_ops[g_nops].kind = OP_RESET; g_ops[g_nops].n = 0; g_ops[g_nops].off = pos; g_nops++; }
  }
  /* make sure that the data of every planned sample exists */
  { unsigned i; for (i = pos; i < MAXN && i < pos + 4; i++) g_xb[i] = 0; }
  g_nx = pos;
  { unsigned i; for (i = 0; i < pos + 4 && i < MAXN; i++) memcpy(&g_xf[i], &g_xb[i], 4); }
  return pos;
}

/* ================================================================================================ trial machinery */
typedef struct {
  int ans;
  unsigned rate;
  unsigned nDen, nNum;
  double *den, *num;       /* arena arrays (generic) */
  unsigned minS, maxG, ratioTest, period, extra;
  uint32_t thrW, ratioW;
  int useC2, useD2;
  int mode;                /* plan mode 0 single, 1 block, 2 mixed */
  int exact;
  int nofail;
} Cfg;

static void tm_init(TdModel *m, const Cfg *c) {
  unsigned i;
  memset(m, 0, sizeof *m);
  m->nDen = c->ans ? g_tabN : c->nDen;
  m->nNum = c->ans ? g_tabN : c->nNum;
  for (i = 0; i < m->nDen; i++) m->den[i] = c->ans ? g_tabA[i] : c->den[i];
  for (i = 0; i < m->nNum; i++) m->num[i] = c->ans ? g_tabB[i] : c->num[i];
  m->period = c->period;
  m->needHits = ceil_div64(c->minS, c->period);
  m->maxMiss = ceil_div64(c->maxG, c->period);
  m->ratioTest = c->ratioTest;
  m->thrW = c->thrW;
  m->ratioW = c->ratioW;
  tm_reset(m);
}

static void model_pass(const Cfg *c, TdModel *m, int trace) {
  unsigned i, k;
  if (trace) g_ntr = 0;
  tm_init(m, c);
  for (i = 0; i < g_nops; i++) {
    const Op *op = &g_ops[i];
    if (op->kind == OP_RESET) tm_reset(m);
    else for (k = 0; k < op->n; k++) tm_step(m, op->kind == OP_BLOCK, g_xf[op->off + k], trace);
  }
}

/* neighbours of a float word, by integer arithmetic on the bit pattern (finite values only) */
static uint32_t word_up(uint32_t w) { return (w & 0x7fffffffu) == 0x7f800000u ? w : ((w >> 31) ? (((w & 0x7fffffffu) == 0) ? 1u : w - 1u) : w + 1u); }
static uint32_t word_down(uint32_t w) { return (w & 0x7fffffffu) == 0x7f800000u ? w : ((w >> 31) ? w + 1u : (w == 0 ? 0x80000001u : w - 1u)); }
static uint32_t word_scale2(uint32_t w, int de) { /* times 2^de for finite normal values, else unchanged */
  unsigned e = (w >> 23) & 0xffu;
  if (e == 0 || e == 0xff) return w;
  if ((int)e + de <= 0 || (int)e + de >= 255) return w;
  return (w & 0x807fffffu) | ((unsigned)((int)e + de) << 23);
}
static uint32_t special_word(int pct_nan) {
  static const uint32_t sp[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0x7f800000u, 0xff800000u, 0x00000001u, 0x007fffffu, 0x7e800000u, 0x01000000u, 0xbf800000u, 0xc0000000u, 0x7fc01234u, 0xffc00001u, 0x3f800000u };
  (void)pct_nan;
  return sp[h_rmod(sizeof sp / sizeof sp[0])];
}
static uint32_t choose_thr(void) {
  unsigned r = h_rmod(100);
  if (g_ntr == 0 || r < 12) return special_word(0);
  {
    unsigned j = h_rmod(g_ntr), v = h_rmod(8);
    uint32_t w = flt_bits_of_ld(tYR[j]);
    if (r < 62) {
      switch (v) {
        case 0: case 1: return w;
        case 2: return word_up(w);
        case 3: return word_down(w);
        case 4: return word_scale2(w, 1);              /* thr * 0.5 == YR exactly */
        case 5: return word_up(word_scale2(w, 1));
        case 6: return word_down(word_scale2(w, 1));
        default: return word_scale2(w, -1);
      }
    }
    return word_scale2(w, (int)h_rmod(13) - 6);
  }
}
static uint32_t choose_ratio(void) {
  unsigned r = h_rmod(100);
  if (g_ntr == 0 || r < 14) { static const uint32_t sp[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0x7f800000u, 0x3f800000u, 0x3f59999au, 0x3f000000u, 0x40000000u, 0xbf800000u, 0x2edbe6ffu, 0x7f7fffffu, 0x7fc00005u }; return sp[h_rmod(sizeof sp / sizeof sp[0])]; }
  {
    unsigned j = h_rmod(g_ntr), v = h_rmod(6);
    LD q = tV18[j] / tV14[j];
    uint32_t w = flt_bits_of_ld(q);
    switch (v) {
      case 0: case 1: return w;
      case 2: return word_up(w);
      case 3: return word_down(w);
      default: return word_scale2(w, (int)h_rmod(5) - 2);
    }
  }
}
static unsigned choose_run(unsigned period) { /* a minSamples / maxGapSamples value */
  unsigned r = h_rmod(100);
  if (r < 8) return 0;
  if (r < 14) return 1;
  if (r < 24) return period;
  if (r < 30) return period > 1 ? period - 1 : 1;
  if (r < 36) return period + 1;
  if (r < 52) return period * (1 + h_rmod(4));
  if (r < 62) return period * (1 + h_rmod(4)) + 1 + h_rmod(period);
  if (r < 66) return 0xffffffffu;
  if (r < 68) return 0x80000000u + h_rmod(3);
  return h_rmod(period * 12 + 5);
}
static unsigned choose_period(void) {
  static const unsigned p[] = { 1, 1, 2, 2, 3, 4, 4, 5, 7, 8, 8, 10, 16, 16, 20, 32, 50, 64, 100, 160, 200, 333 };
  return p[h_rmod(sizeof p / sizeof p[0])];
}
static unsigned pick_extra(void) {
  static const unsigned e[] = { 0, 1, 2, 3, 5, 8, 17, 40 };
  return e[h_rmod(sizeof e / sizeof e[0])];
}

/* the detector fields as the mirror sees them, compared with the model */
static void cmp_f(const char *what, float got, float want) {
  uint32_t g = bits_f(got), w = bits_f(want);
  if (is_nan_w(w)) { if (!is_nan_w(g)) h_die3("%s: object %08x, model NaN", what, g); return; }
  if (g != w) h_die3("%s: object %08x, model %08x", what, g, w);
}
static void cmp_state(const TdObj *o, const TdModel *m, const char *when, unsigned opno) {
  char b[96];
  const GiirObj *g = (const GiirObj *)o->iir_;
  sprintf(b, "%s op %u sumX2_", when, opno); cmp_f(b, o->sumX2_, m->s0);
  sprintf(b, "%s op %u sumY2_", when, opno); cmp_f(b, o->sumY2_, m->s1);
  sprintf(b, "%s op %u avgX2_", when, opno); cmp_f(b, o->avgX2_, m->v14);
  sprintf(b, "%s op %u avgY2_", when, opno); cmp_f(b, o->avgY2_, m->v18);
  if (o->count_ != m->cnt || o->hits_ != m->hits || o->misses_ != m->miss || o->detected_ != m->det)
    h_die3("%s op %u: counters obj cnt %u hits %u miss %u det %u, model cnt %u hits %u miss %u det %u", when, opno, o->count_, o->hits_, o->misses_, o->detected_, m->cnt, m->hits, m->miss, m->det);
  if (m->anyY) {
    uint64_t gb = bits_d(g->y_), wb = bits_d(m->lastY);
    int wnan = (wb & 0x7ff0000000000000ull) == 0x7ff0000000000000ull && (wb & 0xfffffffffffffull);
    if (wnan) { if (!((gb & 0x7ff0000000000000ull) == 0x7ff0000000000000ull && (gb & 0xfffffffffffffull))) h_die3("%s op %u: iir y_ %016llx, model NaN", when, opno, (unsigned long long)gb); }
    else if (gb != wb) h_die3("%s op %u: iir y_ %016llx, model %016llx", when, opno, (unsigned long long)gb, (unsigned long long)wb);
  }
}

/* dump of the detector and of the heap GenericIIR it owns (field-wise: the raw block holds pointers) */
static void dump_td(int show, const char *tag, const TdObj *o, int ans) {
  char t2[64];
  h_dump_obj(show, tag, o, f_TdObj, NF_TdObj);
  if (!o->iir_) { h_emit(show, "%s.iir NULL", tag); return; }
  sprintf(t2, "%s.iir", tag);
  if (ans) {
    GiirObj tmp;
    memcpy(&tmp, o->iir_, sizeof tmp);
    tmp.den_ = NULL;
    tmp.num_ = NULL;
    h_dump_obj(show, t2, &tmp, f_GiirObj, NF_GiirObj);
    h_emit(show, "%s.tables den=%d num=%d", tag, table_id(((const GiirObj *)o->iir_)->den_, 0), table_id(((const GiirObj *)o->iir_)->num_, 1));
  } else {
    h_dump_obj(show, t2, o->iir_, f_GiirObj, NF_GiirObj);
  }
}

#define WORD2F(var, w) float var; memcpy(&var, &(w), 4)

static TdObj *make_obj(const Cfg *c, int probe) {
  TdObj *o = (TdObj *)h_obj_new(sizeof(TdObj));
  WORD2F(thr, c->thrW);
  WORD2F(rat, c->ratioW);
  h_wild = 0;
  h_log_next_size = 0x34;  /* the GenericIIR object: sizeof differs on x86-64 */
  if (c->ans) {
    if (probe) { H_CTOR_PROBE(o, f_TdObj, NF_TdObj, sizeof(TdObj), (c->useC2 ? an_c2 : an_c1)(o, c->minS, c->maxG, thr, c->ratioTest, rat, c->rate, c->period, c->extra)); }
    else (c->useC2 ? an_c2 : an_c1)(o, c->minS, c->maxG, thr, c->ratioTest, rat, c->rate, c->period, c->extra);
  } else {
    if (probe) { H_CTOR_PROBE(o, f_TdObj, NF_TdObj, sizeof(TdObj), (c->useC2 ? td_c2 : td_c1)(o, c->nDen, c->nNum, c->den, c->num, c->minS, c->maxG, thr, c->ratioTest, rat, c->period, c->extra)); }
    else (c->useC2 ? td_c2 : td_c1)(o, c->nDen, c->nNum, c->den, c->num, c->minS, c->maxG, thr, c->ratioTest, rat, c->period, c->extra);
  }
  h_log_next_size = 0;
  if (o->iir_) { uint64_t v = 0xA5A5A5A5A5A5A5A5ull; memcpy(&((GiirObj *)o->iir_)->y_, &v, 8); } /* y_ is not written by the ctor / reset: canonical value */
  return o;
}

/* the constructor's visible result against the model's expectation */
static void check_ctor(const TdObj *o, const Cfg *c, const TdModel *m) {
  const GiirObj *g = (const GiirObj *)o->iir_;
  unsigned nDen = c->ans ? g_tabN : c->nDen, nNum = c->ans ? g_tabN : c->nNum;
  if (!g) h_die3("ctor left iir_ NULL");
  if (bits_f(o->thresh_) != c->thrW || bits_f(o->ratio_) != c->ratioW) h_die3("ctor: thresh_/ratio_ words %08x/%08x, expected %08x/%08x", bits_f(o->thresh_), bits_f(o->ratio_), c->thrW, c->ratioW);
  if (o->period_ != c->period || o->ratioTest_ != c->ratioTest || o->needHits_ != m->needHits || o->maxMiss_ != m->maxMiss)
    h_die3("ctor fields period %u ratioTest %u needHits %u maxMiss %u, expected %u %u %u %u", o->period_, o->ratioTest_, o->needHits_, o->maxMiss_, c->period, c->ratioTest, m->needHits, m->maxMiss);
  if (bits_f(o->sumX2_) != 0 || bits_f(o->sumY2_) != 0 || bits_f(o->avgX2_) != 0 || bits_f(o->avgY2_) != 0 || o->count_ || o->hits_ || o->misses_ || o->detected_) h_die3("ctor: state not zero");
  if (g->nDen_ != nDen || g->nNum_ != nNum || g->lenNum_ != nNum + c->extra || g->lenDen_ != nDen + c->extra) h_die3("ctor: GenericIIR fields");
  if (c->ans) {
    if (table_id(g->den_, 0) != (c->rate == 8000u ? 4 : 3) || table_id(g->num_, 1) != (c->rate == 8000u ? 2 : 1)) h_die3("ANSam ctor: wrong coefficient tables (den %d num %d)", table_id(g->den_, 0), table_id(g->num_, 1));
  } else if (g->den_ != c->den || g->num_ != c->num) h_die3("ctor: coefficient pointers");
  if (h_nmalloc < 3) h_die3("ctor must allocate");
}

/* run the planned ops on the object, the model in lock step.  check: compare with the model. */
static void run_plan(const Cfg *c, TdObj *o, TdModel *m, unsigned long *ctr) {
  unsigned i, k;
  for (i = 0; i < g_nops; i++) {
    const Op *op = &g_ops[i];
    unsigned ret = 0, wantret = 0, pdet;
    if (op->kind == OP_RESET) {
      td_reset(o);
      tm_reset(m);
      h_cov_hit(CT_RESET);
      if (i > 0 && i + 1 < g_nops) h_cov_hit(CT_RESET_MID);
      h_emit(0, "reset");
    } else if (op->kind == OP_SINGLE) {
      h_cov_hit(CT_P1);
      pdet = m->det;
      ret = td_p1(o, g_xf[op->off]);
      tm_step(m, 0, g_xf[op->off], 0);
      wantret = m->det;
      if (g_last_pe) { h_cov_hit(g_last_ev); if (g_last_eqThr) h_cov_hit(CT_YR_EQ_THR); if (g_last_eqHalf && g_last_ev != CT_S_HIT0) h_cov_hit(CT_YR_EQ_HALF); if (g_last_eqRatio) h_cov_hit(CT_RATIO_EQ); if (pdet == 0 && m->det == 1) h_cov_hit(CT_DET_SET); if (pdet == 1 && m->det == 0) h_cov_hit(CT_DET_CLEAR); if (g_last_rj == -1) h_cov_hit(CT_MISSRESET); }
      h_emit(0, "p1 %u", ret);
    } else {
      h_cov_hit(CT_PN);
      if (op->n == 0) h_cov_hit(CT_PN0);
      pdet = m->det;
      ret = td_pn(o, &g_xf[op->off], op->n);
      for (k = 0; k < op->n; k++) {
        unsigned d0 = m->det;
        tm_step(m, 1, g_xf[op->off + k], 0);
        if (g_last_pe) { h_cov_hit(g_last_ev); if (g_last_eqThr) h_cov_hit(CT_YR_EQ_THR); if (g_last_eqHalf) h_cov_hit(CT_YR_EQ_HALF); if (g_last_eqRatio) h_cov_hit(CT_RATIO_EQ); if (d0 == 0 && m->det == 1) h_cov_hit(CT_DET_SET); if (d0 == 1 && m->det == 0) h_cov_hit(CT_DET_CLEAR); if (g_last_rj == -1) h_cov_hit(CT_MISSRESET); if (g_last_rj == 1) h_cov_hit(CT_DET_AFTER_RMISS_B); }
      }
      wantret = m->det;
      h_emit(0, "pn %u %u", op->n, ret);
    }
    if (op->kind != OP_RESET && ret != wantret) h_die3("op %u (%s): returned %u, model %u", i, op->kind == OP_SINGLE ? "process(float)" : "process(float*,n)", ret, wantret);
    if (!is_fin_w(bits_f(m->s0)) || !is_fin_w(bits_f(m->s1)) || !is_fin_w(bits_f(m->v14)) || !is_fin_w(bits_f(m->v18))) h_cov_hit(CT_NONFIN);
    cmp_state(o, m, c->ans ? "ANSam" : "Generic", i);
    if (op->kind != OP_SINGLE || g_dump_every <= 1 || i % g_dump_every == 0 || i + 1 == g_nops) dump_td(h_sample(ctr, 6, 1999), c->ans ? "AN" : "TD", o, c->ans);
    h_check_not_wild("TD");
  }
}

/* release the heap GenericIIR by hand and clear iir_ (the dtor must then not call sysdep_free at all) */
static void null_iir(TdObj *o) {
  GiirObj *g = (GiirObj *)o->iir_;
  if (g->hnum_) sysdep_free(g->hnum_);
  if (g->hden_) sysdep_free(g->hden_);
  sysdep_free(g);
  o->iir_ = NULL;
  h_cov_hit(H_COV_DTOR_NULL);
  h_cov_hit(CT_DTOR_NULL_IIR);
}
static void destroy(const Cfg *c, TdObj *o) {
  unsigned long nf0 = h_nfree;
  int nulled = 0;
  if (h_rpct(6)) { null_iir(o); nulled = 1; }
  nf0 = h_nfree;
  if (c->ans) { if (c->useD2) { an_d2(o); h_cov_hit(CT_D2); } else an_d1(o); }
  else { if (c->useD2) { td_d2(o); h_cov_hit(CT_D2); } else td_d1(o); }
  if (h_nfree != nf0 + (nulled ? 0u : 3u)) h_die3("dtor must free the IIR object and its two histories (freed %lu)", h_nfree - nf0);
}

/* ================================================================================================ trials and scenarios */
static void define_cov(void) {
  h_cov_define(CT_TRIAL, "trials");
  h_cov_define(CT_C2, "C2 constructor");
  h_cov_define(CT_D2, "D2 destructor");
  h_cov_define(CT_P1, "process(float)");
  h_cov_define(CT_PN, "process(block)");
  h_cov_define(CT_PN0, "process(n == 0)");
  h_cov_define(CT_RESET, "reset");
  h_cov_define(CT_ANS8000, "ANSam rate 8000");
  h_cov_define(CT_ANS9600, "ANSam rate 9600");
  h_cov_define(CT_ANSOTHER, "ANSam other rate (-> 9600 tables)");
  h_cov_define(CT_S_LOW, "single: YR < thresh (or NaN)");
  h_cov_define(CT_S_HIT0, "single: hit without the ratio test");
  h_cov_define(CT_S_HIT1, "single: hit with the ratio test");
  h_cov_define(CT_S_RMISS, "single: ratio miss");
  h_cov_define(CT_B_HIT0, "block: hit without the ratio test");
  h_cov_define(CT_B_HIT1, "block: hit with the ratio test");
  h_cov_define(CT_B_RMISS, "block: ratio miss (joins the detection test)");
  h_cov_define(CT_B_HLOW, "block hysteresis: thresh / 2 > YR (miss)");
  h_cov_define(CT_B_HNORATIO, "block hysteresis: no ratio test (miss)");
  h_cov_define(CT_B_HRATIO, "block hysteresis: V14 * 0.85 >= V18 (miss)");
  h_cov_define(CT_B_HHIT, "block hysteresis: hit");
  h_cov_define(CT_DET_SET, "detected 0 -> 1");
  h_cov_define(CT_DET_CLEAR, "detected 1 -> 0");
  h_cov_define(CT_MISSRESET, "misses >= maxMiss (hits cleared)");
  h_cov_define(CT_NEED0, "needHits == 0");
  h_cov_define(CT_MAXMISS0, "maxMiss == 0");
  h_cov_define(CT_YR_EQ_THR, "YR == thresh exactly");
  h_cov_define(CT_YR_EQ_HALF, "YR == thresh / 2 exactly");
  h_cov_define(CT_RATIO_EQ, "V18 == V14 * ratio exactly");
  h_cov_define(CT_THR_NAN, "thresh NaN");
  h_cov_define(CT_THR_INF, "thresh infinite");
  h_cov_define(CT_RATIO_NAN, "ratio NaN");
  h_cov_define(CT_NONFIN, "non-finite detector state");
  h_cov_define(CT_PERIOD1, "period == 1");
  h_cov_define(CT_CEIL_EXACT, "run length a multiple of the period");
  h_cov_define(CT_CEIL_ROUND, "run length rounded up");
  h_cov_define(CT_NEEDGE2, "needHits >= 2");
  h_cov_define(CT_HITS_PROGRESS, "hits >= 2 reached");
  h_cov_define(CT_PERIOD_BIG, "period >= 100");
  h_cov_define(CT_DATA_TONE, "tone data");
  h_cov_define(CT_DATA_NOISE, "noise data");
  h_cov_define(CT_BLOCK_CROSS, "block spanning several periods");
  h_cov_define(CT_BLOCK_EXACT_PERIOD, "block of a multiple of the period");
  h_cov_define(CT_IDENTITY, "identity filter");
  h_cov_define(CT_IIR, "feedback filter");
  h_cov_define(CT_FIR, "feedforward filter");
  h_cov_define(CT_DTOR_NULL_IIR, "dtor with a NULL IIR pointer");
  h_cov_define(CT_RESET_MID, "reset in the middle of a run");
  h_cov_define(CT_DET_AFTER_RMISS_B, "block: detection set by a ratio miss");
  h_cov_define(CT_DATA_EXACT, "exact small-integer data");
  h_cov_define(CT_BIGSUM, "huge data (float sums overflow)");
  h_cov_define(CT_SNAN_WORD, "signalling NaN word as thresh / ratio");
}

static void fill_filter(Cfg *c, int kind) { /* kind 0 identity, 1 FIR, 2 IIR; arena arrays */
  unsigned i;
  if (kind == 0) { c->nDen = 1; c->nNum = 1; }
  else if (kind == 1 || kind == 3) { c->nDen = kind == 3 ? 1 + h_rmod(5) : 1; c->nNum = 1 + h_rmod(9); }
  else { c->nDen = 2 + h_rmod(7); c->nNum = 1 + h_rmod(8); }
  c->den = (double *)sysdep_malloc(c->nDen * 8u);
  c->num = (double *)sysdep_malloc(c->nNum * 8u);
  for (i = 0; i < c->nDen; i++) c->den[i] = rdouble_unit() * pow2d(-(int)(c->nDen) - 1);
  for (i = 0; i < c->nNum; i++) c->num[i] = rdouble_unit();
  if (kind == 3) { /* special coefficients: inf, NaN, +-0, denormal, huge */
    static const uint64_t sp[] = { 0x0000000000000000ull, 0x8000000000000000ull, 0x7ff0000000000000ull, 0xfff0000000000000ull, 0x7ff8000000000001ull, 0x0000000000000001ull,
                                   0x000fffffffffffffull, 0x7fe0000000000000ull, 0x3ff0000000000000ull, 0xbff0000000000000ull, 0x7ff8000000000000ull, 0x7ff0000000000001ull };
    for (i = 0; i < c->nDen; i++) if (h_rpct(25)) { uint64_t b = sp[h_rmod(sizeof sp / sizeof sp[0])]; memcpy(&c->den[i], &b, 8); }
    for (i = 0; i < c->nNum; i++) if (h_rpct(30)) { uint64_t b = sp[h_rmod(sizeof sp / sizeof sp[0])]; memcpy(&c->num[i], &b, 8); }
    { uint64_t b = sp[h_rmod(sizeof sp / sizeof sp[0])]; if (h_rpct(70)) memcpy(&c->den[0], &b, 8); else c->den[0] = 1.0; }
    return;
  }
  if (kind == 0) { c->den[0] = 1.0; c->num[0] = 1.0; }
  else {
    unsigned r = h_rmod(10);
    c->den[0] = r < 6 ? 1.0 : r == 6 ? 4.0 : r == 7 ? 0.5 : r == 8 ? 3.0 : -2.0;
    if (kind == 1) h_cov_hit(CT_FIR); else h_cov_hit(CT_IIR);
  }
  if (kind == 0) h_cov_hit(CT_IDENTITY);
}

/* one complete trial with a filled-in cfg (period, run lengths, filter, ratioTest, extra, mode); thr / ratio words are chosen here unless forced */
static void trial_run(Cfg *c, unsigned total, int force_words, unsigned long *ctr, int snan_data) {
  static TdModel m;
  TdObj *o;
  unsigned long nm0 = h_nmalloc;
  (void)snan_data;
  if (c->period == 1) h_cov_hit(CT_PERIOD1);
  if (c->period >= 100) h_cov_hit(CT_PERIOD_BIG);
  if (c->ans) select_tables(c->rate);
  if (c->useC2) h_cov_hit(CT_C2);
  if (c->minS % c->period == 0) h_cov_hit(CT_CEIL_EXACT); else h_cov_hit(CT_CEIL_ROUND);
  if (c->maxG % c->period == 0) h_cov_hit(CT_CEIL_EXACT); else h_cov_hit(CT_CEIL_ROUND);
  if (ceil_div64(c->minS, c->period) == 0) h_cov_hit(CT_NEED0);
  if (ceil_div64(c->maxG, c->period) == 0) h_cov_hit(CT_MAXMISS0);
  if (ceil_div64(c->minS, c->period) >= 2) h_cov_hit(CT_NEEDGE2);
  if (c->ans) { if (c->rate == 8000u) h_cov_hit(CT_ANS8000); else if (c->rate == 9600u) h_cov_hit(CT_ANS9600); else h_cov_hit(CT_ANSOTHER); }
  plan_make(c->mode, c->period, total, c->exact, c->ans ? c->rate : 8000u, c->ans);
  { uint32_t fw_t = c->thrW, fw_r = c->ratioW;
    c->thrW = 0x7fc00000u;
    c->ratioW = 0x3f800000u;
    model_pass(c, &m, 1);
    if (force_words) { c->thrW = fw_t; c->ratioW = fw_r; } else { c->thrW = choose_thr(); c->ratioW = choose_ratio(); } }
  if (is_nan_w(c->thrW)) h_cov_hit(CT_THR_NAN);
  if ((c->thrW & 0x7fffffffu) == 0x7f800000u) h_cov_hit(CT_THR_INF);
  if (is_nan_w(c->ratioW)) h_cov_hit(CT_RATIO_NAN);
  if (is_snan_w(c->thrW) || is_snan_w(c->ratioW)) h_cov_hit(CT_SNAN_WORD);
  tm_init(&m, c);
  o = make_obj(c, 1);
  h_cov_hit(CT_TRIAL);
  check_ctor(o, c, &m);
  if (h_nmalloc != nm0 + 3) h_die3("ctor: unexpected number of allocations");
  dump_td(h_sample(ctr, 6, 211), c->ans ? "AN ctor" : "TD ctor", o, c->ans);
  run_plan(c, o, &m, ctr);
  if (m.hits >= 2) h_cov_hit(CT_HITS_PROGRESS);
  destroy(c, o);
  if (!c->ans) { sysdep_free(c->num); sysdep_free(c->den); }
  h_trial_end("TD");
}

/* like trial_run, but the thresh / ratio words come from a list (signalling NaNs, zeros, infinities) 50% of the time */
static void trial_run_words(Cfg *c, unsigned total, const uint32_t *words, unsigned nwords, unsigned long *ctr) {
  int force = h_rpct(50);
  static TdModel mm;
  (void)mm;
  if (force) { c->thrW = words[h_rmod(nwords)]; c->ratioW = words[h_rmod(nwords)]; }
  trial_run(c, total, force, ctr, 0);
}

static void trial_generic(int mode, int exact, unsigned long *ctr) {
  Cfg c;
  unsigned total;
  memset(&c, 0, sizeof c);
  c.mode = mode;
  c.exact = exact;
  c.period = exact ? (1u << h_rmod(6)) : choose_period();
  total = c.period * (8 + h_rmod(40));
  if (total < 200) total = 200 + h_rmod(300);
  if (total > 6000) total = 3000 + h_rmod(3000);
  c.minS = choose_run(c.period);
  c.maxG = choose_run(c.period);
  c.ratioTest = h_rpct(25) ? 0u : h_rpct(70) ? 1u : 1u + h_rmod(100);
  c.extra = pick_extra();
  c.useC2 = h_rpct(30);
  c.useD2 = h_rpct(30);
  fill_filter(&c, exact ? 0 : (int)h_rmod(3));
  trial_run(&c, total, 0, ctr, 0);
}

static void trial_ansam(int mode, unsigned long *ctr) {
  Cfg c;
  unsigned total, r = h_rmod(100);
  static const unsigned others[] = { 0, 1, 7999, 8001, 9599, 9600, 16000, 11025, 0xffffffffu, 8000u * 2 };
  memset(&c, 0, sizeof c);
  c.ans = 1;
  c.mode = mode;
  c.rate = r < 40 ? 8000u : r < 70 ? 9600u : others[h_rmod(sizeof others / sizeof others[0])];
  c.period = choose_period();
  total = c.period * (8 + h_rmod(40));
  if (total < 400) total = 400 + h_rmod(600);
  if (total > 6000) total = 3000 + h_rmod(3000);
  c.minS = choose_run(c.period);
  c.maxG = choose_run(c.period);
  c.ratioTest = h_rpct(25) ? 0u : 1u + h_rmod(3);
  c.extra = pick_extra();
  c.useC2 = h_rpct(35);
  c.useD2 = h_rpct(35);
  trial_run(&c, total, 0, ctr, 0);
}

static void sc_generic(int mode, int exact, int ans, unsigned ntrials) {
  unsigned long ctr = 0;
  unsigned t;
  define_cov();
  h_scenario_begin(0);
  if (mode == 0) g_dump_every = 8;
  if (!ans) { h_covreq[CT_ANS8000] = h_covreq[CT_ANS9600] = h_covreq[CT_ANSOTHER] = 0; }
  if (ans) { h_covreq[CT_IDENTITY] = h_covreq[CT_IIR] = h_covreq[CT_FIR] = 0; }
  if (mode == 0) { h_covreq[CT_PN] = h_covreq[CT_PN0] = h_covreq[CT_B_HIT0] = h_covreq[CT_B_HIT1] = h_covreq[CT_B_RMISS] = h_covreq[CT_B_HLOW] = h_covreq[CT_B_HNORATIO] = h_covreq[CT_B_HRATIO] = h_covreq[CT_B_HHIT] = h_covreq[CT_BLOCK_CROSS] = h_covreq[CT_BLOCK_EXACT_PERIOD] = h_covreq[CT_DET_AFTER_RMISS_B] = 0; }
  if (mode == 1) { h_covreq[CT_P1] = h_covreq[CT_S_LOW] = h_covreq[CT_S_HIT0] = h_covreq[CT_S_HIT1] = h_covreq[CT_S_RMISS] = 0; }
  if (exact) { h_covreq[CT_SNAN_WORD] = 0; h_covreq[CT_B_HRATIO] = 0; h_covreq[CT_PERIOD_BIG] = 0; h_covreq[CT_DATA_TONE] = h_covreq[CT_DATA_NOISE] = h_covreq[CT_BIGSUM] = h_covreq[CT_FIR] = h_covreq[CT_IIR] = 0; }
  else h_covreq[CT_DATA_EXACT] = 0;
  /* events that need the right data / parameters: required only in the scenarios built for them */
  h_covreq[CT_NONFIN] = 0;
  if (!exact) { h_covreq[CT_YR_EQ_THR] = h_covreq[CT_YR_EQ_HALF] = h_covreq[CT_RATIO_EQ] = 0; }
  for (t = 0; t < ntrials; t++) {
    if (ans) trial_ansam(mode, &ctr); else trial_generic(mode, exact, &ctr);
  }
}

/* scenario 5: constructor / destructor / reset details: the ceil of the run lengths over boundary values (incl. 2^31, 2^32 - 1), degenerate filters,
 * signalling NaN words as thresh / ratio (the object must hold the words unchanged), D1 / D2, a NULL iir_, reset right after the constructor */
static void sc_ctor(void) {
  static const unsigned pers[] = { 1, 2, 3, 4, 5, 7, 16, 100, 0x7fffffffu, 0x80000000u, 0xfffffffeu, 0xffffffffu };
  static const uint32_t words[] = { 0x7f800001u, 0xff800001u, 0x7fa00000u, 0x7fbfffffu, 0xffa12345u, 0x7fc00000u, 0x00000000u, 0x80000000u, 0x3f800000u, 0x7f800000u, 0x00000001u, 0xbf000000u };
  unsigned long ctr = 0;
  unsigned t;
  define_cov();
  h_scenario_begin(0);
  h_covreq[CT_B_HHIT] = h_covreq[CT_B_HRATIO] = h_covreq[CT_B_HNORATIO] = h_covreq[CT_DATA_EXACT] = h_covreq[CT_YR_EQ_THR] = h_covreq[CT_YR_EQ_HALF] = h_covreq[CT_RATIO_EQ] = 0;
  h_covreq[CT_NONFIN] = h_covreq[CT_FIR] = h_covreq[CT_BIGSUM] = h_covreq[CT_BLOCK_CROSS] = h_covreq[CT_BLOCK_EXACT_PERIOD] = h_covreq[CT_PERIOD_BIG] = 0;
  h_covreq[CT_DET_AFTER_RMISS_B] = h_covreq[CT_HITS_PROGRESS] = h_covreq[CT_DET_CLEAR] = h_covreq[CT_MISSRESET] = h_covreq[CT_B_HLOW] = h_covreq[CT_B_HIT1] = h_covreq[CT_B_RMISS] = 0;
  h_covreq[CT_S_HIT1] = h_covreq[CT_S_RMISS] = h_covreq[CT_B_HIT0] = h_covreq[CT_DET_SET] = h_covreq[CT_IDENTITY] = h_covreq[CT_IIR] = h_covreq[CT_DATA_TONE] = h_covreq[CT_DATA_NOISE] = 0;
  h_covreq[CT_THR_INF] = h_covreq[CT_THR_NAN] = h_covreq[CT_RATIO_NAN] = h_covreq[CT_NEEDGE2] = h_covreq[CT_NEED0] = h_covreq[CT_MAXMISS0] = 0;
  h_covreq[CT_S_HIT0] = h_covreq[CT_S_LOW] = 0;
  for (t = 0; t < 900; t++) {
    Cfg c;
    unsigned total = 30 + h_rmod(120), r = h_rmod(100);
    memset(&c, 0, sizeof c);
    c.mode = (int)h_rmod(3);
    c.period = pers[h_rmod(sizeof pers / sizeof pers[0])];
    if (h_rpct(30)) c.period = 1 + h_rmod(12);
    { unsigned k = h_rmod(14); c.minS = k < 3 ? k : k == 3 ? c.period - 1 : k == 4 ? c.period : k == 5 ? c.period + 1 : k == 6 ? 2u * c.period - 1u : k == 7 ? 2u * c.period : k == 8 ? 2u * c.period + 1u :
                     k == 9 ? 0xffffffffu : k == 10 ? 0x80000000u : k == 11 ? 0x7fffffffu : k == 12 ? 0xfffffffeu : h_rnd(); }
    { unsigned k = h_rmod(14); c.maxG = k < 3 ? k : k == 3 ? c.period - 1 : k == 4 ? c.period : k == 5 ? c.period + 1 : k == 6 ? 2u * c.period - 1u : k == 7 ? 2u * c.period : k == 8 ? 2u * c.period + 1u :
                     k == 9 ? 0xffffffffu : k == 10 ? 0x80000000u : k == 11 ? 0x7fffffffu : k == 12 ? 0xfffffffeu : h_rnd(); }
    c.ratioTest = h_rmod(3);
    c.extra = h_rpct(20) ? h_rmod(2000) : pick_extra();
    c.useC2 = h_rpct(50);
    c.useD2 = h_rpct(50);
    if (r < 45) { /* ANSam */
      c.ans = 1;
      c.rate = h_rpct(40) ? 8000u : h_rpct(50) ? 9600u : h_rnd();
    } else {
      fill_filter(&c, (int)h_rmod(3));
      if (h_rpct(25)) { /* wider filters: 14 coefficients each */
        sysdep_free(c.num); sysdep_free(c.den);
        c.nDen = 1 + h_rmod(14); c.nNum = 1 + h_rmod(14);
        c.den = (double *)sysdep_malloc(c.nDen * 8u); c.num = (double *)sysdep_malloc(c.nNum * 8u);
        { unsigned i; for (i = 0; i < c.nDen; i++) c.den[i] = rdouble_unit() * pow2d(-(int)c.nDen - 1); for (i = 0; i < c.nNum; i++) c.num[i] = rdouble_unit(); c.den[0] = 1.0; }
      }
    }
    if (c.period == 1) h_cov_hit(CT_PERIOD1);
    trial_run_words(&c, total, words, (unsigned)(sizeof words / sizeof words[0]), &ctr);
  }
}

/* scenario 6: non-finite / special data and coefficients (inf, NaN payloads incl. signalling NaNs, denormals, signed zeros, huge / tiny values) */
static void sc_special(void) {
  unsigned long ctr = 0;
  unsigned t;
  define_cov();
  h_scenario_begin(0);
  h_covreq[CT_YR_EQ_THR] = h_covreq[CT_YR_EQ_HALF] = h_covreq[CT_RATIO_EQ] = h_covreq[CT_DATA_EXACT] = h_covreq[CT_DATA_TONE] = h_covreq[CT_DATA_NOISE] = h_covreq[CT_FIR] = 0;
  h_covreq[CT_IDENTITY] = h_covreq[CT_IIR] = h_covreq[CT_ANS8000] = h_covreq[CT_ANS9600] = h_covreq[CT_ANSOTHER] = 0;
  g_special_mode = 1;
  g_dump_every = 4;
  for (t = 0; t < 400; t++) {
    if (h_rpct(35)) trial_ansam((int)h_rmod(3), &ctr);
    else {
      Cfg c;
      unsigned total;
      memset(&c, 0, sizeof c);
      c.mode = (int)h_rmod(3);
      c.period = choose_period();
      if (c.period > 64) c.period = 1 + h_rmod(64);
      total = c.period * (8 + h_rmod(30));
      if (total < 200) total = 200 + h_rmod(200);
      c.minS = choose_run(c.period);
      c.maxG = choose_run(c.period);
      c.ratioTest = h_rmod(3);
      c.extra = pick_extra();
      c.useC2 = h_rpct(30);
      c.useD2 = h_rpct(30);
      fill_filter(&c, h_rpct(60) ? 3 : (int)h_rmod(3));
      trial_run(&c, total, 0, &ctr, 1);
    }
  }
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P04_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  p02_layout_check();
  p04_layout_check();
  h_x87_check();
  h_arena_init();
  switch (sc) {
    case 0: sc_generic(0, 0, 0, 500); break;
    case 1: sc_generic(1, 0, 0, 500); break;
    case 2: sc_generic(2, 0, 0, 500); break;
    case 3: sc_generic(2, 0, 1, 500); break;
    case 4: sc_generic(2, 1, 0, 600); break;
    case 5: sc_ctor(); break;
    case 6: sc_special(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu", h_nmalloc, h_nfree);
  h_cov_finish();
  return 0;
}
