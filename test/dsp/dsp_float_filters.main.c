/*
 * Differential test harness for port chunk P02 (src/dsp/dsp_float_filters.cpp): FloatFIR (8 functions), FloatIIR (7),
 * FloatARMA (7), GenericIIR<float, double> (5) of dsplibs.o.
 *
 * ONE C source, no #ifdef, built by test/dsp/dsp_float_filters.sh against the blob (dsplibs.o), against the port (-O0 / -O2) and for
 * x86-64 (the 64-bit run is diffed against the 32-bit BLOB's output = the golden, test/golden/dsp/dsp_float_filters.blob32.out).  The 27 functions
 * are reached through __asm__ labels carrying the Itanium-mangled names (identical in blob and port, C1 AND C2 / D1 AND D2).
 * The shared harness library (test/harness/) provides the PRNG, hashed canonical output, coverage counters (exit 2), the arena
 * allocator with red zones, the object pool and the field-wise dump; the field tables and the mirror structs of the four classes
 * are GENERATED (tools/layout_gen.py from test/p02.layout -> test/p02_layout.h) and checked against the port headers by the probe.
 *
 * What is compared: every output sample as a BIT PATTERN, every object (all fields; floats / doubles as bits, pointers normalised)
 * with the heap blocks it owns, the malloc / free sequence, the store order of the constructors (malloc-time snapshot), the red
 * zones.  INDEPENDENT MODELS (written from the disassembly facts, not from the port): a pure DELAY LINE of the inputs / outputs
 * (no wrap logic, no pointers) with the extended-precision accumulation order of the blob (two accumulators, even / odd terms).
 * Model results are compared bit-exactly (a NaN model result only requires a NaN); a mismatch exits 3 (model violation).
 * Scenarios (each its own process): 0 FloatFIR vs model, 1 FloatIIR vs model, 2 FloatARMA process(float) vs model, 3 FloatARMA block
 * (dead in the blob) vs model, 4 GenericIIR vs model, 5 special values without NaN payloads (inf, denormals,
 * signed zeros; exact on x86-64 SSE too), 7 the same with NaN payloads incl. signalling NaNs (x87-exact only), 6 setCoefficients / odd + degenerate configurations, 8 / 9 / 10 FloatFIR / FloatIIR / FloatARMA with poked
 * non-multiple-of-4 tap counts (remainder loops), 11 designed cancellation vectors for the accumulation order of every dot-product function.
 * (5, 6, 7 are blob-vs-port only.)
 *
 * Excluded because the BLOB itself is undefined there (documented in the port): a filter with 0 taps (taps_ == 0 / nAr == 0 /
 * nMa == 0 / nNum == 0 / nDen == 0) processes a sample (stores one element past its history block), FloatIIR with extra == 0 (stores
 * at hist[-1]), the wrap copy with taps - 1 == 0 (2^32 iterations), FloatARMA with nAr == 0 (reads ar[0]) .
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "../p02_layout.h"

#define L(m) __asm__(m)
/* ---- FloatFIR ---- */
extern void fir_c1(void *, unsigned, float *, unsigned) L("_ZN8FloatFIRC1EjPfj");
extern void fir_c2(void *, unsigned, float *, unsigned) L("_ZN8FloatFIRC2EjPfj");
extern void fir_d1(void *) L("_ZN8FloatFIRD1Ev");
extern void fir_d2(void *) L("_ZN8FloatFIRD2Ev");
extern void fir_reset(void *) L("_ZN8FloatFIR5resetEv");
extern int fir_setc(void *, float *, unsigned) L("_ZN8FloatFIR15setCoefficientsEPfj");
extern float fir_p1(void *, float) L("_ZN8FloatFIR7processEf");
extern void fir_pn(void *, const float *, float *, unsigned) L("_ZN8FloatFIR7processEPKfPfj");
/* ---- FloatIIR ---- */
extern void iir_c1(void *, unsigned, float *, unsigned) L("_ZN8FloatIIRC1EjPfj");
extern void iir_c2(void *, unsigned, float *, unsigned) L("_ZN8FloatIIRC2EjPfj");
extern void iir_d1(void *) L("_ZN8FloatIIRD1Ev");
extern void iir_d2(void *) L("_ZN8FloatIIRD2Ev");
extern void iir_reset(void *) L("_ZN8FloatIIR5resetEv");
extern int iir_setc(void *, float *, unsigned) L("_ZN8FloatIIR15setCoefficientsEPfj");
extern void iir_pn(void *, const float *, float *, unsigned) L("_ZN8FloatIIR7processEPKfPfj");
/* ---- FloatARMA ---- */
extern void arma_c1(void *, unsigned, unsigned, float *, float *, unsigned) L("_ZN9FloatARMAC1EjjPfS0_j");
extern void arma_c2(void *, unsigned, unsigned, float *, float *, unsigned) L("_ZN9FloatARMAC2EjjPfS0_j");
extern void arma_d1(void *) L("_ZN9FloatARMAD1Ev");
extern void arma_d2(void *) L("_ZN9FloatARMAD2Ev");
extern void arma_reset(void *) L("_ZN9FloatARMA5resetEv");
extern float arma_p1(void *, float) L("_ZN9FloatARMA7processEf");
extern void arma_pn(void *, const float *, float *, unsigned) L("_ZN9FloatARMA7processEPKfPfj");
/* ---- GenericIIR<float, double> ---- */
extern void giir_c1(void *, unsigned, unsigned, double *, double *, unsigned) L("_ZN10GenericIIRIfdEC1EjjPdS1_j");
extern void giir_d1(void *) L("_ZN10GenericIIRIfdED1Ev");
extern void giir_reset(void *) L("_ZN10GenericIIRIfdE5resetEv");
extern float giir_p1(void *, float) L("_ZN10GenericIIRIfdE7processEf");
extern void giir_pn(void *, const float *, float *, unsigned) L("_ZN10GenericIIRIfdE7processEPKfPfj");

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ value generators */
static uint32_t bits_f(float f) { return h_bits_of(f); }
static uint64_t bits_d(double d) { uint64_t b; memcpy(&b, &d, 8); return b; }
static int is_nan_f(uint32_t b) { return (b & 0x7f800000u) == 0x7f800000u && (b & 0x007fffffu) != 0; }
static int is_snan_f(uint32_t b) { return is_nan_f(b) && !(b & 0x00400000u); }
static int is_fin_f(uint32_t b) { return (b & 0x7f800000u) != 0x7f800000u; }

/* RULE of the special-value scenario: signalling-NaN data (regime 6) only with FINITE coefficients; whenever the coefficients may be inf / NaN the data
 * uses regime 7.  (An sNaN datum meeting a QNaN coefficient: the blob quiets the datum by `fld` and the larger payload wins, a port that
 * lets gcc take the datum as the memory operand of fmuls lets the QNaN win.)
 * regimes: 0 uniform [-1,1) full 24-bit mantissa, 1 random exponent -20..20, 2 small integers, 3 huge (2^100..2^127: overflow to inf),
 * 4 denormals / tiny, 5 zeros of both signs, 6 specials (inf, NaN with payload, signalling NaN), 7 the same WITHOUT signalling NaNs.
 * Why 7 exists: the blob's memory-operand forms (`fadd DWORD [in]` in FloatIIR, `fmul DWORD [coef]`) hand a signalling NaN to the FPU
 * unquieted, where the x87 prefers a QUIET NaN operand over it; the port's `fld` first quiets it and the larger payload wins.  Only
 * reachable with an sNaN operand meeting a NaN: out of scope (a modem never feeds signalling NaNs), documented in the port; regime 7 is
 * used wherever the data meets arithmetic with memory operands, regime 6 where the sNaN only has to be STORED (FIR / ARMA history). */
enum { R_UNIT = 0, R_EXP, R_INT, R_HUGE, R_TINY, R_ZERO, R_SPECIAL, R_NFIN = 6, R_SPECIAL_Q = 7, R_INF = 8 };
static uint32_t rbits(int regime) {
  uint32_t r = h_rnd(), sign = (r & 1u) << 31, m;
  switch (regime) {
    case R_UNIT: { float f = (float)((int32_t)(h_rnd() >> 7) - (1 << 24)) / 16777216.0f; uint32_t b; memcpy(&b, &f, 4); return b; }
    case R_EXP: m = h_rnd() & 0x7fffffu; return sign | ((uint32_t)(127 + (int)h_rmod(41) - 20) << 23) | m;
    case R_INT: { float f = (float)((int)h_rmod(17) - 8); uint32_t b; memcpy(&b, &f, 4); return b; }
    case R_HUGE: return sign | ((uint32_t)(127 + 100 + (int)h_rmod(28)) << 23) | (h_rnd() & 0x7fffffu);
    case R_TINY: return sign | ((h_rpct(50) ? 0u : (uint32_t)(1 + h_rmod(3))) << 23) | (h_rnd() & 0x7fffffu);
    case R_ZERO: return sign;
    case R_INF: return sign | 0x7f800000u;
    default: {
      uint32_t pay = h_rnd() & 0x007fffffu, k = h_rmod(6);
      if (regime == R_SPECIAL_Q && (k == 2 || k == 3 || k == 4)) k = 1;
      if (k == 0) return sign | 0x7f800000u;                 /* inf */
      if (k == 1) return sign | 0x7fc00000u | pay;          /* quiet NaN, payload */
      if (k == 2) return sign | 0x7f800000u | (pay | 1u);   /* signalling NaN (quiet bit may be clear) */
      if (k == 3) return sign | 0x7f800001u;                /* the smallest signalling NaN */
      if (k == 4) return sign | 0x7fa00000u | (pay & 0x1fffffu);
      return sign | 0x7fc00000u;
    }
  }
}
/* float values are produced by value only for FINITE regimes (a float returned by value goes through st0 on i386, which would quiet an sNaN):
 * everything that may hold specials is filled through rfill() / bits */
static float rfloat(int regime) { uint32_t b = rbits(regime); float f; memcpy(&f, &b, 4); return f; }
static void rfill(float *dst, int regime) { uint32_t b = rbits(regime); memcpy(dst, &b, 4); }
static double rdouble(int regime) {
  uint64_t r = ((uint64_t)h_rnd() << 32) | h_rnd(), sign = (r >> 63) << 63, mant = r & 0xfffffffffffffull;
  uint64_t b;
  switch (regime) {
    case R_UNIT: { double d = (double)((int64_t)(r >> 11) - (1LL << 52)) / 4503599627370496.0; return d; } /* [-1,1) with up to 53 significant bits */
    case R_EXP: b = sign | ((uint64_t)(1023 + (int)h_rmod(41) - 20) << 52) | mant; break;
    case R_INT: return (double)((int)h_rmod(17) - 8);
    case R_HUGE: b = sign | ((uint64_t)(1023 + 900 + (int)h_rmod(100)) << 52) | mant; break;
    case R_TINY: b = sign | ((uint64_t)(h_rpct(50) ? 0u : (uint32_t)(1 + h_rmod(3))) << 52) | mant; break;
    case R_ZERO: b = sign; break;
    case R_INF: b = sign | 0x7ff0000000000000ull; break;
    default: {
      uint32_t k = h_rmod(4);
      if (regime == R_SPECIAL_Q && k == 2) k = 1;
      if (k == 0) b = sign | 0x7ff0000000000000ull;
      else if (k == 1) b = sign | 0x7ff8000000000000ull | (mant & 0x0007ffffffffffffull);
      else if (k == 2) b = sign | 0x7ff0000000000000ull | (mant | 1u);
      else b = sign | 0x7ff8000000000000ull;
    }
  }
  { double d; memcpy(&d, &b, 8); return d; }
}
/* exact power-of-two scales (the harness data path must not depend on the build's float evaluation: on i386 a float expression keeps excess
 * precision between operations, on x86-64 SSE it does not; power-of-two products and a single rounded operation are identical on both) */
static float pow2_scale_f(unsigned n) { float sc = 1.0f; unsigned t = 1; while (t < n) { sc *= 0.5f; t <<= 1; } return sc; }
static double pow2_scale_d(unsigned n) { double sc = 1.0; unsigned t = 1; while (t < n) { sc *= 0.5; t <<= 1; } return sc; }
static unsigned pick_extra(void) {
  static const unsigned e[] = { 0, 1, 2, 3, 4, 5, 7, 8, 15, 16, 31, 33, 64, 100, 257 };
  if (h_rpct(2)) return 32760u + h_rmod(40); /* room / index values around and above 2^15: a 16-bit index variable is wrong there */
  return h_rpct(70) ? e[h_rmod(sizeof e / sizeof e[0])] : h_rmod(60);
}
static float *alloc_f(unsigned n) { return (float *)sysdep_malloc(n * 4u); }
static double *alloc_d(unsigned n) { return (double *)sysdep_malloc(n * 8u); }

/* ---------------------------------------------------------------------------------------------------- the models */
/* two extended accumulators, A: even terms, B: odd terms, result B + A (taps are multiples of 4: no remainder) */
typedef struct { LD a, b; } Acc2;
/* term k of an n-term dot product: terms of the 4x-unrolled part alternate A (even k) / B (odd k), the REMAINDER terms (k >= n & ~3, only
 * reachable by poking the tap count to a non-multiple of 4) all go to A */
static void acc_add_n(Acc2 *s, unsigned k, unsigned n, LD term) { if ((k & 1u) && k < (n & ~3u)) s->b += term; else s->a += term; }
static LD acc_sum(const Acc2 *s) { return s->b + s->a; }

#define MAXN 20000
static float g_x[MAXN];   /* inputs in time order (float bits) */
static float g_y[MAXN];   /* outputs in time order */
static double g_xd[MAXN]; /* GenericIIR: the history values as doubles */
static double g_yd[MAXN];
static unsigned g_n;      /* samples so far */
static float xs(int k) { int i = (int)g_n - 1 - k; return i < 0 ? 0.0f : g_x[i]; }          /* x[n-k], zero before the start */
static float ys(int k) { int i = (int)g_n - 1 - k; return i < 0 ? 0.0f : g_y[i]; }
static double xsd(int k) { int i = (int)g_n - 1 - k; return i < 0 ? 0.0 : g_xd[i]; }
static double ysd(int k) { int i = (int)g_n - 1 - k; return i < 0 ? 0.0 : g_yd[i]; }

/* ================================================================================================ common trial pieces */

static void chk_out(const char *what, uint32_t got, uint32_t want, int valid, unsigned idx) {
  if (!valid) return;
  if (is_nan_f(want)) { if (!is_nan_f(got)) h_die3("%s sample %u: model NaN, blob %08x", what, idx, got); return; }
  if (got != want) h_die3("%s sample %u: blob/port %08x, model %08x", what, idx, got, want);
}

/* dtor probe: release the block a pointer field owns by hand and clear the field: the destructor must then skip it (the blob tests every
 * pointer; the stub dies on sysdep_free(NULL)) */
static int null_owned(void *o, size_t off, int pct) {
  void *p, *z = NULL;
  if (!h_rpct(pct)) return 0;
  memcpy(&p, (uint8_t *)o + off, sizeof p);
  if (!p) return 0;
  sysdep_free(p);
  memcpy((uint8_t *)o + off, &z, sizeof z);
  h_cov_hit(H_COV_DTOR_NULL);
  return 1;
}

/* ============================================================================================ FloatFIR / FloatIIR (0, 1) */
enum { CF_TRIAL = 0, CF_C2, CF_D2, CF_EXTRA0, CF_EXTRA_SMALL, CF_EXTRA_LARGE, CF_WRAP, CF_MANYWRAP, CF_P1, CF_PN, CF_PN0, CF_PNBIG, CF_RESET, CF_MASK,
       CF_COEF_EXP, CF_NONFIN_OUT, CF_TINY, CF_ZERO_IN, CF_INT_IN, CF_TAPS4, CF_TAPS_BIG, CF_VERIFY, CF_MODEL_OUT, CF_INPLACE, CF_WRAP_SINGLE, CF_REMAINDER };

/* the history window after an op equals the delay line: FIR hist[room+1+k] = x[n-k] (k <= taps-2: the write index points one below the
 * newest sample), IIR hist[room+k] = y[n-k] (k <= taps-1: room is the index of the newest output) */
static void fi_verify(int iir, const FirObj *o, unsigned taps, unsigned len, const char *what) {
  unsigned k, lim = iir ? taps : taps - 1;
  const float *h = (const float *)o->hist_;
  int room = o->room_;
  if (room < 0 || (unsigned)room >= len) h_die3("%s: room_ %d out of range (len %u taps %u)", what, room, len, taps);
  for (k = 0; k < lim; k++) {
    unsigned idx = (unsigned)room + (iir ? 0u : 1u) + k;
    uint32_t want = bits_f(iir ? ys((int)k) : xs((int)k));
    if (idx >= len) h_die3("%s: window index %u beyond the history block", what, idx);
    if (bits_f(h[idx]) != want) h_die3("%s: hist[%u] = %08x, delay line says %08x (k %u, room %d)", what, idx, bits_f(h[idx]), want, k, room);
  }
  h_cov_hit(CF_VERIFY);
}

static void sc_fir_iir(int iir, int poke) {
  unsigned long ctr = 0;
  unsigned trial, ntrials = 700;
  const char *name = iir ? "IIR" : "FIR";
  h_cov_define(CF_TRIAL, "trials");
  h_cov_define(CF_C2, "C2 constructor");
  h_cov_define(CF_D2, "D2 destructor");
  h_cov_define(CF_EXTRA0, "extra == 0");
  h_cov_define(CF_EXTRA_SMALL, "extra 1..3");
  h_cov_define(CF_EXTRA_LARGE, "extra >= 64");
  h_cov_define(CF_WRAP, "history wrap");
  h_cov_define(CF_MANYWRAP, "trial with >= 8 wraps");
  if (!iir) { h_cov_define(CF_P1, "process(float)"); h_cov_define(CF_WRAP_SINGLE, "wrap inside process(float)"); }
  h_cov_define(CF_PN, "process(block)");
  h_cov_define(CF_PN0, "process(n == 0)");
  h_cov_define(CF_PNBIG, "block of >= 3*histLen samples");
  h_cov_define(CF_RESET, "reset");
  h_cov_define(CF_MASK, "ntaps with low bits (&~3)");
  h_cov_define(CF_COEF_EXP, "coefficients with random exponents");
  h_cov_define(CF_NONFIN_OUT, "non-finite output (overflow to inf / NaN)");
  h_cov_define(CF_TINY, "denormal / tiny data");
  h_cov_define(CF_ZERO_IN, "zero inputs");
  h_cov_define(CF_INT_IN, "small-integer data");
  h_cov_define(CF_TAPS4, "taps == 4");
  h_cov_define(CF_TAPS_BIG, "taps >= 48");
  h_cov_define(CF_VERIFY, "window verifications");
  h_cov_define(CF_MODEL_OUT, "model-checked outputs");
  h_cov_define(CF_INPLACE, "in-place block (in == out)");
  if (poke) h_cov_define(CF_REMAINDER, "taps_ poked to a non-multiple of 4 (remainder loop)");
  h_scenario_begin(1);
  if (iir) h_covreq[CF_EXTRA0] = 0; /* extra == 0 stores at hist[-1] in FloatIIR: excluded (UB in the blob) */
  for (trial = 0; trial < ntrials; trial++) {
    unsigned taps = 4u * (unsigned)(h_rpct(8) ? h_rrange(1, 4) : h_rpct(10) ? h_rrange(12, 20) : h_rrange(1, 8)), ntaps = taps + (h_rpct(30) ? h_rmod(4) : 0u);
    unsigned extra = pick_extra(), len, nops, op, i, nwrap = 0, ncoef;
    int creg = h_rpct(60) ? R_UNIT : h_rpct(50) ? R_EXP : R_INT, wide;
    int c2 = h_rpct(30), d2 = h_rpct(30), mvalid = 1, mroom, rtop;
    float *coef;
    FirObj *o = (FirObj *)h_obj_new(sizeof(FirObj));
    unsigned long nm0 = h_nmalloc, nf0;
    if (iir && extra == 0) extra = 1 + h_rmod(5);
    len = taps + extra;
    wide = (taps <= 8 && h_rpct(60)); /* short filters with wide exponent spread: the sum of the products is inexact in 64 bits, so the accumulator split (and the remainder grouping) shows */
    if (wide) creg = R_EXP;
    ncoef = poke ? ntaps + len + 4 : ntaps;
    coef = alloc_f(ncoef);
    for (i = 0; i < ncoef; i++) coef[i] = rfloat(creg);
    if (iir) { /* keep the feedback tame: scale the coefficients so that sum |c| stays below ~1 mostly */
      for (i = 0; i < ncoef; i++) if (creg != R_INT && h_rpct(85)) coef[i] = coef[i] * pow2_scale_f(taps);
      if (creg == R_INT) for (i = 0; i < ncoef; i++) coef[i] = (float)((int)h_rmod(3) - 1) * 0.25f * (h_rpct(50) ? 1.f : 0.f);
    }
    h_wild = 0;
    H_CTOR_PROBE(o, f_FirObj, NF_FirObj, sizeof(FirObj),
                 if (iir) { if (c2) iir_c2(o, ntaps, coef, extra); else iir_c1(o, ntaps, coef, extra); }
                 else { if (c2) fir_c2(o, ntaps, coef, extra); else fir_c1(o, ntaps, coef, extra); });
    h_cov_hit(CF_TRIAL);
    if (c2) h_cov_hit(CF_C2);
    if (extra == 0) h_cov_hit(CF_EXTRA0);
    if (extra >= 1 && extra <= 3) h_cov_hit(CF_EXTRA_SMALL);
    if (extra >= 64) h_cov_hit(CF_EXTRA_LARGE);
    if (ntaps != taps) h_cov_hit(CF_MASK);
    if (creg == R_EXP) h_cov_hit(CF_COEF_EXP);
    if (taps == 4) h_cov_hit(CF_TAPS4);
    if (taps >= 48) h_cov_hit(CF_TAPS_BIG);
    if (h_nmalloc != nm0 + 2 || h_al[h_last_alloc].size != len * 4u) h_die3("ctor must allocate (taps+extra)*4 = %u bytes once (after the coefficient array)", len * 4u);
    if (o->taps_ != taps || o->histLen_ != len || o->room_ != (int)extra || o->coef_ != coef || o->hist_ != h_al[h_last_alloc].p) h_die3("%s ctor fields", name);
    for (i = 0; i < len; i++) if (bits_f(((float *)o->hist_)[i]) != 0) h_die3("%s ctor did not zero hist[%u]", name, i);
    h_dump_obj(h_sample(&ctr, 12, 97), iir ? "IIR ctor" : "FIR ctor", o, f_FirObj, NF_FirObj);
    if (poke) { /* poke the tap count to a non-multiple of 4 (ctor and setCoefficients never produce one): the blob's remainder loop of the dot product */
      unsigned pt = 0, hi = iir ? len - 1 : len, tries;
      for (tries = 0; tries < 20 && !pt; tries++) { unsigned c = 2 + h_rmod(37); if (c <= hi && (c & 3u)) pt = c; }
      if (pt) { taps = pt; o->taps_ = pt; if (iir) iir_reset(o); else fir_reset(o); h_cov_hit(CF_REMAINDER); }
    }
    rtop = (int)(len - taps);
    g_n = 0;
    mroom = rtop;
    nops = (unsigned)h_rrange(6, 30);
    for (op = 0; op < nops; op++) {
      int r = (int)h_rmod(100), dreg = h_rpct(55) ? R_UNIT : (int)h_rmod(R_NFIN);
      if (dreg == R_HUGE && h_rpct(60)) dreg = R_UNIT;
      if (wide && h_rpct(70)) dreg = R_EXP;
      if (r < 6 && op > 0) { /* reset: both histories are zero again, the delay lines restart */
        if (iir) iir_reset(o); else fir_reset(o);
        h_cov_hit(CF_RESET);
        g_n = 0;
        mroom = rtop;
        for (i = 0; i < len; i++) if (bits_f(((float *)o->hist_)[i]) != 0) h_die3("reset left hist[%u] nonzero", i);
        h_emit(0, "reset");
        mvalid = 1;
      } else if (!iir && r < 45) { /* process(float) */
        float x = rfloat(dreg), y;
        Acc2 s = { 0, 0 };
        unsigned k;
        if (g_n >= MAXN) break;
        y = fir_p1(o, x);
        h_cov_hit(CF_P1);
        g_x[g_n++] = x;
        for (k = 0; k < taps; k++) acc_add_n(&s, k, taps, (LD)xs((int)k) * (LD)coef[k]);
        { float w = (float)acc_sum(&s); chk_out("FIR p1", bits_f(y), bits_f(w), mvalid, g_n); h_cov_hit(CF_MODEL_OUT); }
        mroom--;
        if (mroom < 0) { mroom = rtop; nwrap++; h_cov_hit(CF_WRAP_SINGLE); }
        if (dreg == R_ZERO) h_cov_hit(CF_ZERO_IN);
        if (dreg == R_INT) h_cov_hit(CF_INT_IN);
        if (dreg == R_TINY) h_cov_hit(CF_TINY);
        if (!is_fin_f(bits_f(y))) h_cov_hit(CF_NONFIN_OUT);
        h_emit(0, "p1 %08x", bits_f(y));
      } else { /* process(block) */
        unsigned n = h_rpct(8) ? 0u : h_rpct(15) ? 3u * len + h_rmod(7) : h_rmod(2 * len + 8); if (n > 4000) n = 4000;
        float *in = alloc_f(n + 4), *out = alloc_f(n + 4), *src;
        int inplace = h_rpct(15);
        uint32_t *xin = (uint32_t *)malloc(4u * (n + 1u));
        unsigned k;
        if (g_n + n >= MAXN) n = 0;
        for (i = 0; i < n + 4; i++) { rfill(&in[i], dreg); out[i] = h_float_from_bits(0xEEEEEEEEu); }
        if (inplace) for (i = 0; i < n; i++) out[i] = in[i]; /* in == out: the blob reads in[i] before it writes out[i] */
        src = inplace ? out : in;
        for (i = 0; i < n; i++) xin[i] = bits_f(src[i]);
        h_cov_hit(CF_PN);
        if (n == 0) h_cov_hit(CF_PN0);
        if (n >= 3 * len) h_cov_hit(CF_PNBIG);
        if (inplace && n) h_cov_hit(CF_INPLACE);
        if (iir) iir_pn(o, src, out, n); else fir_pn(o, src, out, n);
        for (k = 0; k < n; k++) {
          uint32_t got = bits_f(out[k]);
          float xf = h_float_from_bits(xin[k]);
          Acc2 s = { 0, 0 };
          unsigned t;
          if (!iir) {
            g_x[g_n++] = xf;
            for (t = 0; t < taps; t++) acc_add_n(&s, t, taps, (LD)xs((int)t) * (LD)coef[t]);
            { float w = (float)acc_sum(&s); chk_out("FIR pn", got, bits_f(w), mvalid, g_n); }
          } else {
            float w;
            for (t = 0; t < taps; t++) acc_add_n(&s, t, taps, (LD)ys((int)t) * (LD)coef[t]);
            w = (float)(acc_sum(&s) + (LD)xf);
            g_y[g_n++] = h_float_from_bits(got);
            chk_out("IIR pn", got, bits_f(w), mvalid, g_n);
            if (!is_fin_f(bits_f(w))) mvalid = 0; /* feedback with inf / NaN: no more model checks in this trial */
          }
          h_cov_hit(CF_MODEL_OUT);
          if (!is_fin_f(got)) h_cov_hit(CF_NONFIN_OUT);
          mroom--;
          if (mroom < 0) { mroom = rtop; nwrap++; }
          h_emit(0, "pn %08x", got);
        }
        if (dreg == R_ZERO) h_cov_hit(CF_ZERO_IN);
        if (dreg == R_INT) h_cov_hit(CF_INT_IN);
        if (dreg == R_TINY) h_cov_hit(CF_TINY);
        free(xin);
        sysdep_free(out);
        sysdep_free(in);
      }
      if (o->room_ != mroom) h_die3("%s: room_ is %d, the index model says %d", name, o->room_, mroom);
      if (mvalid) fi_verify(iir, o, taps, len, name);
      h_dump_obj(h_sample(&ctr, 12, 211), iir ? "IIR" : "FIR", o, f_FirObj, NF_FirObj);
      h_check_not_wild(name);
    }
    if (nwrap >= 1) h_cov_hit(CF_WRAP);
    if (nwrap >= 8) h_cov_hit(CF_MANYWRAP);
    { int nulled = null_owned(o, offsetof(FirObj, hist_), 10);
      nf0 = h_nfree;
      if (d2) { if (iir) iir_d2(o); else fir_d2(o); h_cov_hit(CF_D2); } else { if (iir) iir_d1(o); else fir_d1(o); }
      if (h_nfree != nf0 + (nulled ? 0u : 1u)) h_die3("dtor must free the history block exactly once (and not at all when it is NULL)"); }
    sysdep_free(coef);
    h_trial_end(name);
  }
}

/* ================================================================================================ scenario 2 / 3: FloatARMA */
enum { CA_TRIAL = 0, CA_C2, CA_D2, CA_A0_ONE, CA_A0_OTHER, CA_A0_NAN, CA_A0_ZERO, CA_A0_INF, CA_PAD_AR, CA_PAD_MA, CA_NO_PAD, CA_EXTRA0, CA_P1, CA_PN, CA_PN0, CA_RESET,
       CA_WRAPX, CA_WRAPY, CA_VERIFY, CA_MODEL, CA_NONFIN, CA_NMA1, CA_NAR1, CA_BIG, CA_REMAINDER };
static void arma_model_coefs(unsigned nAr, unsigned nMa, const float *ar, const float *ma, float *ar2, float *ma2, unsigned nb4, unsigned na4) {
  unsigned i;
  float a0 = ar[0];
  for (i = 0; i < nb4; i++) ar2[i] = i < nAr ? ar[i] : 0.0f;
  for (i = 0; i < na4; i++) ma2[i] = i < nMa ? ma[i] : 0.0f;
  if (a0 < 1.0f || a0 > 1.0f) {
    LD recip = (LD)1 / (LD)a0;
    for (i = 0; i < nMa; i++) ma2[i] = (float)(recip * (LD)ma[i]);
    for (i = 1; i < nAr; i++) ar2[i] = (float)(recip * (LD)ar[i]);
  }
  ar2[0] = 0.0f;
}
static void sc_arma(int block, int poke) {
  unsigned long ctr = 0;
  unsigned trial, ntrials = 500;
  h_cov_define(CA_TRIAL, "trials");
  h_cov_define(CA_C2, "C2 constructor");
  h_cov_define(CA_D2, "D2 destructor");
  h_cov_define(CA_A0_ONE, "ar[0] == 1.0");
  h_cov_define(CA_A0_OTHER, "ar[0] finite != 1, != 0");
  h_cov_define(CA_A0_NAN, "ar[0] NaN");
  h_cov_define(CA_A0_ZERO, "ar[0] == 0");
  h_cov_define(CA_A0_INF, "ar[0] inf");
  h_cov_define(CA_PAD_AR, "nAr not a multiple of 4");
  h_cov_define(CA_PAD_MA, "nMa not a multiple of 4");
  h_cov_define(CA_NO_PAD, "nAr and nMa multiples of 4");
  h_cov_define(CA_EXTRA0, "extra == 0");
  h_cov_define(CA_P1, "process(float)");
  if (block) { h_cov_define(CA_PN, "process(block)"); h_cov_define(CA_PN0, "process(n == 0)"); }
  h_cov_define(CA_RESET, "reset");
  h_cov_define(CA_WRAPX, "x history wrap");
  h_cov_define(CA_WRAPY, "y history wrap");
  h_cov_define(CA_VERIFY, "window verifications");
  h_cov_define(CA_MODEL, "model-checked outputs");
  h_cov_define(CA_NMA1, "nMa == 1");
  h_cov_define(CA_NAR1, "nAr == 1");
  h_cov_define(CA_BIG, "nMa or nAr > 16");
  if (poke) h_cov_define(CA_REMAINDER, "nAr4_ / nMa4_ poked to non-multiples of 4 (remainder loops)");
  h_scenario_begin(1);
  for (trial = 0; trial < ntrials; trial++) {
    unsigned nAr = h_rpct(10) ? 1u : h_rpct(30) ? 4u * (unsigned)h_rrange(1, 5) : (unsigned)h_rrange(1, 24);
    unsigned nMa = h_rpct(10) ? 1u : h_rpct(30) ? 4u * (unsigned)h_rrange(1, 5) : (unsigned)h_rrange(1, 24);
    unsigned extra = pick_extra(), nb4 = (nAr + 3u) & ~3u, na4 = (nMa + 3u) & ~3u, xlen = na4 + extra, ylen = nb4 + extra, nops, op, i;
    float *ar = alloc_f(nAr + 4), *ma = alloc_f(nMa + 4), *ar2 = (float *)malloc(4 * (nb4 + 4)), *ma2 = (float *)malloc(4 * (na4 + 4));
    int c2 = h_rpct(30), d2 = h_rpct(30), mvalid = 1, a0kind = (int)h_rmod(10), model_coefs = 1, mxr, myr;
    ArmaObj *o = (ArmaObj *)h_obj_new(sizeof(ArmaObj));
    unsigned long nm0 = h_nmalloc, nf0 = h_nfree;
    float a0;
    for (i = 0; i < nAr; i++) ar[i] = rfloat(R_UNIT) * pow2_scale_f(2 * nAr);
    for (i = 0; i < nMa; i++) ma[i] = h_rpct(60) ? rfloat(R_UNIT) : rfloat(R_EXP);
    if (a0kind < 3) a0 = 1.0f;
    else if (a0kind < 7) { a0 = rfloat(R_UNIT); if (a0 > -0.05f && a0 < 0.05f) a0 = 0.5f; a0 = a0 * 4.0f; }
    else if (a0kind == 7) a0 = 0.0f;
    else if (a0kind == 8) a0 = h_float_from_bits(h_rpct(50) ? 0x7fc01234u : 0xffc00001u);
    else a0 = h_float_from_bits(h_rpct(50) ? 0x7f800000u : 0xff800000u);
    ar[0] = a0;
    for (i = nAr; i < nAr + 4; i++) { uint32_t v = 0x5A5A5A5Au + i; memcpy(&ar[i], &v, 4); } /* poison beyond the source arrays: an over-copy would show */
    for (i = nMa; i < nMa + 4; i++) { uint32_t v = 0x6B6B6B6Bu + i; memcpy(&ma[i], &v, 4); }
    if (a0 == 1.0f) h_cov_hit(CA_A0_ONE); else if (a0 != a0) h_cov_hit(CA_A0_NAN); else if (a0 == 0.0f) h_cov_hit(CA_A0_ZERO); else if (a0 > 1e38f || a0 < -1e38f) h_cov_hit(CA_A0_INF); else h_cov_hit(CA_A0_OTHER);
    if (!(a0 == a0) || a0 == 0.0f || a0 > 1e38f || a0 < -1e38f) mvalid = 0; /* NaN / 0 / inf normalisation: blob-vs-port only */
    if (nAr & 3u) h_cov_hit(CA_PAD_AR);
    if (nMa & 3u) h_cov_hit(CA_PAD_MA);
    if (!(nAr & 3u) && !(nMa & 3u)) h_cov_hit(CA_NO_PAD);
    if (extra == 0) h_cov_hit(CA_EXTRA0);
    if (nMa == 1) h_cov_hit(CA_NMA1);
    if (nAr == 1) h_cov_hit(CA_NAR1);
    if (nMa > 16 || nAr > 16) h_cov_hit(CA_BIG);
    arma_model_coefs(nAr, nMa, ar, ma, ar2, ma2, nb4, na4);
    h_wild = 0;
    H_CTOR_PROBE(o, f_ArmaObj, NF_ArmaObj, sizeof(ArmaObj), if (c2) arma_c2(o, nAr, nMa, ar, ma, extra); else arma_c1(o, nAr, nMa, ar, ma, extra));
    h_cov_hit(CA_TRIAL);
    { /* the ctor never writes y_ / w_ (they are written before they are read): the harness' random fill differs between i386 and
       * x86-64 (different sizeof), so give both a canonical value before anything is dumped (the malloc-time probe above has
       * already recorded `y_=unset w_=unset` for the blob and the port) */
      uint32_t v = 0xA5A5A5A5u;
      memcpy(&o->y_, &v, 4);
      memcpy(&o->w_, &v, 4);
    }
    if (c2) h_cov_hit(CA_C2);
    if (h_nmalloc != nm0 + 4) h_die3("ARMA ctor must allocate four blocks (after the two coefficient arrays)");
    if (o->nAr4_ != nb4 || o->nMa4_ != na4 || o->xlen_ != xlen || o->ylen_ != ylen || o->xr_ != (int)extra || o->yr_ != (int)extra || o->idx_ != ylen)
      h_die3("ARMA ctor fields: nAr4 %u nMa4 %u xlen %u ylen %u xr %d yr %d idx %u", o->nAr4_, o->nMa4_, o->xlen_, o->ylen_, o->xr_, o->yr_, o->idx_);
    if (mvalid && model_coefs) {
      for (i = 0; i < nb4; i++) if (bits_f(((float *)o->ar_)[i]) != bits_f(ar2[i])) h_die3("ARMA ctor: ar_[%u] = %08x, model %08x", i, bits_f(((float *)o->ar_)[i]), bits_f(ar2[i]));
      for (i = 0; i < na4; i++) if (bits_f(((float *)o->ma_)[i]) != bits_f(ma2[i])) h_die3("ARMA ctor: ma_[%u] = %08x, model %08x", i, bits_f(((float *)o->ma_)[i]), bits_f(ma2[i]));
    }
    for (i = 0; i < xlen; i++) if (bits_f(((float *)o->xhist_)[i]) != 0) h_die3("ARMA ctor did not zero xhist[%u]", i);
    for (i = 0; i < ylen; i++) if (bits_f(((float *)o->yhist_)[i]) != 0) h_die3("ARMA ctor did not zero yhist[%u]", i);
    h_dump_obj(h_sample(&ctr, 12, 97), "ARMA ctor", o, f_ArmaObj, NF_ArmaObj);
    g_n = 0;
    if (poke && na4 >= 2 && nb4 >= 2) { /* poke the padded tap counts to non-multiples of 4: the blob's remainder loops */
      unsigned pm = 2 + h_rmod(na4 - 1), pa = 2 + h_rmod(nb4 - 1);
      if (!(pm & 3u)) pm = pm == na4 ? pm - 1 : pm + 1;
      if (!(pa & 3u)) pa = pa == nb4 ? pa - 1 : pa + 1;
      if (pm <= na4 && pa <= nb4 && pm >= 2 && pa >= 2) {
        o->nMa4_ = pm; o->nAr4_ = pa; arma_reset(o);
        na4 = pm; nb4 = pa;
        h_cov_hit(CA_REMAINDER);
      }
    }
    mxr = (int)(xlen - na4);
    myr = (int)(ylen - nb4);
    nops = (unsigned)h_rrange(6, 24);
    for (op = 0; op < nops; op++) {
      int r = (int)h_rmod(100), dreg = h_rpct(60) ? R_UNIT : (int)h_rmod(R_NFIN);
      if (dreg == R_HUGE) dreg = R_UNIT;
      if (r < 5 && op > 0) {
        mxr = (int)(xlen - na4);
        myr = (int)(ylen - nb4);
        arma_reset(o);
        h_cov_hit(CA_RESET);
        g_n = 0;
        mvalid = mvalid; /* reset zeroes both histories: the delay lines restart */
        if (o->idx_ != ylen || o->xr_ != mxr || o->yr_ != myr) h_die3("ARMA reset fields");
        for (i = 0; i < xlen; i++) if (bits_f(((float *)o->xhist_)[i]) != 0) h_die3("ARMA reset: xhist[%u]", i);
        for (i = 0; i < ylen; i++) if (bits_f(((float *)o->yhist_)[i]) != 0) h_die3("ARMA reset: yhist[%u]", i);
        h_emit(0, "reset");
      } else if (!block || r < 40) {
        float x = rfloat(dreg), y;
        if (g_n >= MAXN) break;
        y = arma_p1(o, x);
        h_cov_hit(CA_P1);
        g_x[g_n++] = x;
        if (mvalid) {
          Acc2 s = { 0, 0 }, t = { 0, 0 };
          unsigned k;
          float y1, w, out;
          for (k = 0; k < na4; k++) acc_add_n(&s, k, na4, (LD)xs((int)k) * (LD)ma2[k]);
          y1 = (float)acc_sum(&s);
          for (k = 1; k < nb4; k++) acc_add_n(&t, k, nb4, (LD)ar2[k] * (LD)ys((int)k)); /* y[n-k]: g_n already counts the current input, its output is not pushed yet */
          w = (float)acc_sum(&t);
          out = (float)((LD)y1 - (LD)w);
          chk_out("ARMA p1", bits_f(y), bits_f(out), 1, g_n);
          h_cov_hit(CA_MODEL);
          g_y[g_n - 1] = out;
          if (!is_fin_f(bits_f(out))) mvalid = 0;
          if (o->y_ != y && !(y != y)) h_die3("ARMA process(float) did not return y_");
        } else g_y[g_n - 1] = y;
        if (!is_fin_f(bits_f(y))) h_cov_hit(CA_NONFIN);
        if (--mxr < 0) { mxr = (int)(xlen - na4); h_cov_hit(CA_WRAPX); }
        if (--myr < 0) { myr = (int)(ylen - nb4); h_cov_hit(CA_WRAPY); }
        h_emit(0, "p1 %08x y_ %08x w_ %08x", bits_f(y), bits_f(o->y_), bits_f(o->w_));
      } else {
        unsigned n = h_rpct(8) ? 0u : h_rpct(15) ? 3u * (xlen > ylen ? xlen : ylen) + h_rmod(5) : h_rmod(2 * (xlen > ylen ? xlen : ylen) + 8); if (n > 4000) n = 4000;
        float *in = alloc_f(n + 4), *out = alloc_f(n + 4);
        unsigned k;
        if (g_n + n >= MAXN) n = 0;
        for (i = 0; i < n + 4; i++) { rfill(&in[i], dreg); out[i] = h_float_from_bits(0xEEEEEEEEu); }
        h_cov_hit(CA_PN);
        if (n == 0) h_cov_hit(CA_PN0);
        arma_pn(o, in, out, n);
        for (k = 0; k < n; k++) {
          uint32_t got = bits_f(out[k]);
          g_x[g_n++] = in[k];
          if (mvalid) {
            Acc2 s = { 0, 0 }, t = { 0, 0 };
            unsigned q;
            float y1, w, want;
            for (q = 0; q < na4; q++) acc_add_n(&s, q, na4, (LD)xs((int)q) * (LD)ma2[q]);
            y1 = (float)acc_sum(&s);
            for (q = 1; q < nb4; q++) acc_add_n(&t, q, nb4, (LD)ar2[q] * (LD)ys((int)q));
            w = (float)acc_sum(&t);
            want = (float)((LD)y1 - (LD)w);
            chk_out("ARMA pn", got, bits_f(want), 1, g_n);
            h_cov_hit(CA_MODEL);
            g_y[g_n - 1] = want;
            if (!is_fin_f(bits_f(want))) mvalid = 0;
          } else g_y[g_n - 1] = h_float_from_bits(got);
          if (!is_fin_f(got)) h_cov_hit(CA_NONFIN);
          if (--mxr < 0) { mxr = (int)(xlen - na4); h_cov_hit(CA_WRAPX); }
          if (--myr < 0) { myr = (int)(ylen - nb4); h_cov_hit(CA_WRAPY); }
          h_emit(0, "pn %08x", got);
        }
        h_emit(0, "end y_ %08x w_ %08x", bits_f(o->y_), bits_f(o->w_));
        sysdep_free(out);
        sysdep_free(in);
      }
      /* window checks: newest elements of both histories equal the delay lines (after the op, the write index points one below) */
      if (mvalid) {
        unsigned k;
        for (k = 0; k + 1 < na4; k++) {
          unsigned idx = (unsigned)o->xr_ + 1u + k;
          if (idx >= xlen || bits_f(((float *)o->xhist_)[idx]) != bits_f(xs((int)k))) h_die3("ARMA xhist window k=%u idx %u", k, idx);
        }
        for (k = 0; k + 1 < nb4; k++) {
          unsigned idx = (unsigned)o->yr_ + 1u + k;
          if (idx >= ylen || bits_f(((float *)o->yhist_)[idx]) != bits_f(ys((int)k))) h_die3("ARMA yhist window k=%u idx %u", k, idx);
        }
        h_cov_hit(CA_VERIFY);
      }
      if (o->xr_ != mxr || o->yr_ != myr) h_die3("ARMA: xr_/yr_ are %d/%d, the index model says %d/%d", o->xr_, o->yr_, mxr, myr);
      h_dump_obj(h_sample(&ctr, 12, 211), "ARMA", o, f_ArmaObj, NF_ArmaObj);
      h_check_not_wild("ARMA");
    }
    { unsigned nulled = (unsigned)null_owned(o, offsetof(ArmaObj, ar_), 6) + (unsigned)null_owned(o, offsetof(ArmaObj, ma_), 6) +
                        (unsigned)null_owned(o, offsetof(ArmaObj, xhist_), 6) + (unsigned)null_owned(o, offsetof(ArmaObj, yhist_), 6);
      nf0 = h_nfree;
      if (d2) { arma_d2(o); h_cov_hit(CA_D2); } else arma_d1(o);
      if (h_nfree != nf0 + 4 - nulled) h_die3("ARMA dtor must free the four blocks that are not NULL"); }
    sysdep_free(ma);
    sysdep_free(ar);
    free(ar2);
    free(ma2);
    h_trial_end("ARMA");
  }
}

/* ============================================================================================== scenario 4: GenericIIR */
enum { CG_TRIAL = 0, CG_D0_ONE, CG_D0_OTHER, CG_D0_ZERO, CG_D0_NAN, CG_P1, CG_PN, CG_PN0, CG_RESET, CG_WRAPN, CG_WRAPD, CG_VERIFY, CG_MODEL, CG_NDEN1, CG_NNUM1,
       CG_EXTRA0, CG_BIG, CG_NONFIN, CG_PN_RECIP_DIFF, CG_IDX };
static void sc_giir(void) {
  unsigned long ctr = 0;
  unsigned trial, ntrials = 600;
  h_cov_define(CG_TRIAL, "trials");
  h_cov_define(CG_D0_ONE, "den[0] == 1.0");
  h_cov_define(CG_D0_OTHER, "den[0] finite != 0, != 1");
  h_cov_define(CG_D0_ZERO, "den[0] == 0 (no division)");
  h_cov_define(CG_D0_NAN, "den[0] NaN (treated as equal to 0)");
  h_cov_define(CG_P1, "process(float)");
  h_cov_define(CG_PN, "process(block)");
  h_cov_define(CG_PN0, "process(n == 0)");
  h_cov_define(CG_RESET, "reset");
  h_cov_define(CG_WRAPN, "num history wrap");
  h_cov_define(CG_WRAPD, "den history wrap");
  h_cov_define(CG_VERIFY, "window verifications");
  h_cov_define(CG_MODEL, "model-checked outputs");
  h_cov_define(CG_NDEN1, "nDen == 1");
  h_cov_define(CG_NNUM1, "nNum == 1");
  h_cov_define(CG_EXTRA0, "extra == 0");
  h_cov_define(CG_BIG, "nDen or nNum > 8");
  h_cov_define(CG_NONFIN, "non-finite output");
  h_cov_define(CG_PN_RECIP_DIFF, "block: acc * (1/den0) differs from acc / den0");
  h_cov_define(CG_IDX, "idx_ final value checked");
  h_scenario_begin(1); /* 8-byte elements: the allocator keeps 16-byte block alignment, the 4-byte mask is enough */
  for (trial = 0; trial < ntrials; trial++) {
    unsigned nDen = h_rpct(12) ? 1u : (unsigned)h_rrange(1, 12), nNum = h_rpct(12) ? 1u : (unsigned)h_rrange(1, 12), extra = pick_extra(), i, nops, op;
    unsigned lenN = nNum + extra, lenD = nDen + extra;
    double *den = alloc_d(nDen), *num = alloc_d(nNum);
    GiirObj *o = (GiirObj *)h_obj_new(sizeof(GiirObj));
    unsigned long nm0 = h_nmalloc, nf0 = h_nfree;
    int kind = (int)h_rmod(20), mvalid = 1, mrn, mrd;
    double d0;
    for (i = 0; i < nDen; i++) den[i] = rdouble(R_UNIT) * pow2_scale_d(2 * nDen);
    for (i = 0; i < nNum; i++) num[i] = h_rpct(70) ? rdouble(R_UNIT) : rdouble(R_EXP);
    if (kind < 8) d0 = 1.0;
    else if (kind < 16) { d0 = rdouble(R_UNIT); if (d0 > -0.05 && d0 < 0.05) d0 = 0.5; d0 = d0 * 4.0; }
    else if (kind == 16) d0 = 0.0;
    else if (kind == 17) d0 = -0.0;
    else if (kind == 18) { uint64_t b = 0x7ff8000000001234ull; memcpy(&d0, &b, 8); }
    else d0 = 4.0; /* power of two: 1/d0 exact */
    den[0] = d0;
    if (d0 == 1.0) h_cov_hit(CG_D0_ONE); else if (d0 != d0) h_cov_hit(CG_D0_NAN); else if (d0 == 0.0) h_cov_hit(CG_D0_ZERO); else h_cov_hit(CG_D0_OTHER);
    if (nDen == 1) h_cov_hit(CG_NDEN1);
    if (nNum == 1) h_cov_hit(CG_NNUM1);
    if (extra == 0) h_cov_hit(CG_EXTRA0);
    if (nDen > 8 || nNum > 8) h_cov_hit(CG_BIG);
    h_wild = 0;
    H_CTOR_PROBE(o, f_GiirObj, NF_GiirObj, sizeof(GiirObj), giir_c1(o, nDen, nNum, den, num, extra));
    h_cov_hit(CG_TRIAL);
    { uint64_t v = 0xA5A5A5A5A5A5A5A5ull; memcpy(&o->y_, &v, 8); } /* y_ is not written by the ctor / reset: canonical value (see the ARMA trial) */
    if (h_nmalloc != nm0 + 2) h_die3("GenericIIR ctor must allocate two blocks (after the two coefficient arrays)");
    if (o->nDen_ != nDen || o->nNum_ != nNum || o->lenNum_ != lenN || o->lenDen_ != lenD || o->rNum_ != (int)extra || o->rDen_ != (int)extra || o->idx_ != lenD ||
        o->den_ != den || o->num_ != num) h_die3("GenericIIR ctor fields");
    for (i = 0; i < lenN; i++) if (bits_d(((double *)o->hnum_)[i]) != 0) h_die3("GenericIIR ctor: hnum[%u] not zero", i);
    for (i = 0; i < lenD; i++) if (bits_d(((double *)o->hden_)[i]) != 0) h_die3("GenericIIR ctor: hden[%u] not zero", i);
    h_dump_obj(h_sample(&ctr, 12, 97), "GIIR ctor", o, f_GiirObj, NF_GiirObj);
    g_n = 0;
    mrn = mrd = (int)extra;
    nops = (unsigned)h_rrange(6, 26);
    for (op = 0; op < nops; op++) {
      int r = (int)h_rmod(100), dreg = h_rpct(60) ? R_UNIT : (int)h_rmod(R_NFIN);
      if (dreg == R_HUGE && !h_rpct(30)) dreg = R_UNIT;
      if (r < 5 && op > 0) {
        giir_reset(o);
        mrn = mrd = (int)extra;
        h_cov_hit(CG_RESET);
        g_n = 0;
        if (o->idx_ != lenD || o->rNum_ != (int)extra || o->rDen_ != (int)extra) h_die3("GenericIIR reset fields");
        for (i = 0; i < lenN; i++) if (bits_d(((double *)o->hnum_)[i]) != 0) h_die3("GenericIIR reset: hnum[%u]", i);
        for (i = 0; i < lenD; i++) if (bits_d(((double *)o->hden_)[i]) != 0) h_die3("GenericIIR reset: hden[%u]", i);
        h_emit(0, "reset");
      } else {
        int single = r < 45;
        unsigned n = single ? 1u : h_rpct(8) ? 0u : h_rpct(15) ? 3u * (lenN > lenD ? lenN : lenD) + h_rmod(5) : h_rmod(2 * (lenN > lenD ? lenN : lenD) + 8), k;
        if (n > 4000) n = 4000;
        float *in = alloc_f(n + 4), *out = alloc_f(n + 4);
        float sres = 0;
        if (g_n + n >= MAXN) n = single ? 1u : 0u;
        for (i = 0; i < n + 4; i++) { rfill(&in[i], dreg); out[i] = h_float_from_bits(0xEEEEEEEEu); }
        if (single) { sres = giir_p1(o, in[0]); out[0] = sres; h_cov_hit(CG_P1); }
        else { h_cov_hit(CG_PN); if (n == 0) h_cov_hit(CG_PN0); giir_pn(o, in, out, n); }
        for (k = 0; k < n; k++) {
          uint32_t got = bits_f(out[k]);
          g_xd[g_n] = (double)in[k];
          g_n++;
          if (mvalid) {
            LD acc = 0, q;
            unsigned t;
            double yd;
            float yf;
            for (t = 0; t < nNum; t++) acc += (LD)num[t] * (LD)xsd((int)t);
            for (t = 1; t < nDen; t++) acc -= (LD)den[t] * (LD)ysd((int)t); /* y[n-t]: g_n already counts the current input, its output is not pushed yet */
            if (d0 < 0 || d0 > 0) {
              if (single) q = acc / (LD)d0; else { LD rc = (LD)1 / (LD)d0; q = acc * rc; if ((double)q != (double)(acc / (LD)d0)) h_cov_hit(CG_PN_RECIP_DIFF); }
            } else q = acc;
            yd = (double)q;
            yf = (float)yd;
            g_yd[g_n - 1] = yd;
            chk_out(single ? "GIIR p1" : "GIIR pn", got, bits_f(yf), 1, g_n);
            h_cov_hit(CG_MODEL);
            if (!is_fin_f(bits_f(yf)) || !(yd == yd)) { mvalid = 0; h_cov_hit(CG_NONFIN); }
          } else {
            g_yd[g_n - 1] = 0;
          }
          if (--mrn < 0) { mrn = (int)(lenN - nNum); h_cov_hit(CG_WRAPN); }
          if (--mrd < 0) { mrd = (int)(lenD - nDen); h_cov_hit(CG_WRAPD); }
          h_emit(0, "%s %08x", single ? "p1" : "pn", got);
        }
        if (n) {
          if (o->idx_ != (nDen > 1 ? nDen : 1u)) h_die3("GenericIIR process: idx_ %u, expected %u", o->idx_, nDen > 1 ? nDen : 1u);
          h_cov_hit(CG_IDX);
          h_emit(0, "y_ %016llx", (unsigned long long)bits_d(o->y_));
        }
        sysdep_free(out);
        sysdep_free(in);
      }
      if (mvalid) {
        unsigned k;
        for (k = 0; k + 1 < nNum; k++) {
          unsigned idx = (unsigned)o->rNum_ + 1u + k;
          if (idx >= lenN || bits_d(((double *)o->hnum_)[idx]) != bits_d(xsd((int)k))) h_die3("GenericIIR hnum window k=%u idx %u", k, idx);
        }
        for (k = 0; k + 1 < nDen; k++) {
          unsigned idx = (unsigned)o->rDen_ + 1u + k;
          if (idx >= lenD || bits_d(((double *)o->hden_)[idx]) != bits_d(ysd((int)k))) h_die3("GenericIIR hden window k=%u idx %u", k, idx);
        }
        h_cov_hit(CG_VERIFY);
      }
      if (o->rNum_ != mrn || o->rDen_ != mrd) h_die3("GenericIIR: rNum_/rDen_ are %d/%d, the index model says %d/%d", o->rNum_, o->rDen_, mrn, mrd);
      h_dump_obj(h_sample(&ctr, 12, 211), "GIIR", o, f_GiirObj, NF_GiirObj);
      h_check_not_wild("GenericIIR");
    }
    { unsigned nulled = (unsigned)null_owned(o, offsetof(GiirObj, hnum_), 8) + (unsigned)null_owned(o, offsetof(GiirObj, hden_), 8);
      nf0 = h_nfree;
      giir_d1(o);
      if (h_nfree != nf0 + 2 - nulled) h_die3("GenericIIR dtor must free the two blocks that are not NULL"); }
    sysdep_free(num);
    sysdep_free(den);
    h_trial_end("GenericIIR");
  }
}

/* ======================================================================================== scenario 5: special values */
/* blob-vs-port only: NaN payloads (also signalling), inf, denormals, signed zeros in the inputs and in the coefficients */
enum { CS_FIR_SNAN_HIST = 0, CS_FIR_PN_SNAN_HIST, CS_ARMA_SNAN_HIST, CS_ARMA_PN_SNAN_HIST, CS_NAN_OUT, CS_INF_OUT, CS_ZERO_OUT, CS_NEGZERO_OUT, CS_IIR_NAN,
       CS_GIIR_NAN, CS_DENORM_OUT, CS_COEF_SPECIAL, CS_SNAN_ARG };
static void sc_special(int nan) {
  const int RS = nan ? R_SPECIAL : R_INF, RQ = nan ? R_SPECIAL_Q : R_INF;
  unsigned long ctr = 0;
  unsigned trial;
  h_cov_define(CS_FIR_SNAN_HIST, "FIR process(float): sNaN argument stored bit-exactly");
  h_cov_define(CS_FIR_PN_SNAN_HIST, "FIR block: sNaN input stored bit-exactly");
  h_cov_define(CS_ARMA_SNAN_HIST, "ARMA process(float): sNaN argument stored bit-exactly");
  h_cov_define(CS_ARMA_PN_SNAN_HIST, "ARMA block: sNaN input stored bit-exactly");
  h_cov_define(CS_NAN_OUT, "NaN output");
  h_cov_define(CS_INF_OUT, "inf output");
  h_cov_define(CS_ZERO_OUT, "+0 output");
  h_cov_define(CS_NEGZERO_OUT, "-0 output");
  h_cov_define(CS_IIR_NAN, "IIR NaN input");
  h_cov_define(CS_GIIR_NAN, "GenericIIR NaN input");
  h_cov_define(CS_DENORM_OUT, "denormal output");
  h_cov_define(CS_COEF_SPECIAL, "special coefficient");
  h_cov_define(CS_SNAN_ARG, "sNaN passed by value");
  if (!nan) { h_covreq[CS_FIR_SNAN_HIST] = h_covreq[CS_FIR_PN_SNAN_HIST] = h_covreq[CS_ARMA_SNAN_HIST] = h_covreq[CS_ARMA_PN_SNAN_HIST] = 0; h_covreq[CS_IIR_NAN] = h_covreq[CS_GIIR_NAN] = h_covreq[CS_SNAN_ARG] = 0; }
  h_scenario_begin(1);
  for (trial = 0; trial < 400; trial++) {
    int kind = trial % 4; /* 0 FIR, 1 IIR, 2 ARMA, 3 GenericIIR */
    unsigned taps = 4u * (unsigned)h_rrange(1, 5), extra = pick_extra(), len, nops, op, i, nm0 = (unsigned)h_nmalloc;
    int cspecial = h_rpct(30);
    (void)nm0;
    h_wild = 0;
    if (kind == 0 || kind == 1) {
      float *coef;
      FirObj *o = (FirObj *)h_obj_new(sizeof(FirObj));
      if (kind == 1 && extra == 0) extra = 2;
      len = taps + extra;
      coef = alloc_f(taps);
      for (i = 0; i < taps; i++) { rfill(&coef[i], h_rpct(cspecial ? 40 : 0) ? RQ : h_rpct(50) ? R_UNIT : R_TINY); if (kind == 1) coef[i] *= 0.25f; if (cspecial && !is_fin_f(bits_f(coef[i]))) h_cov_hit(CS_COEF_SPECIAL); }
      if (kind == 0) fir_c1(o, taps, coef, extra); else iir_c1(o, taps, coef, extra);
      nops = (unsigned)h_rrange(8, 30);
      for (op = 0; op < nops; op++) {
        int dreg = h_rpct(35) ? (kind == 0 && !cspecial ? RS : RQ) : h_rpct(40) ? R_ZERO : h_rpct(50) ? R_TINY : R_UNIT;
        if (kind == 0 && h_rpct(45)) {
          uint32_t xb = rbits(dreg);
          float x, y;
          int room = o->room_;
          memcpy(&x, &xb, 4);
          y = fir_p1(o, x);
          if (is_snan_f(xb)) { h_cov_hit(CS_SNAN_ARG); if (bits_f(((float *)o->hist_)[room]) == xb) h_cov_hit(CS_FIR_SNAN_HIST); else h_die3("sNaN argument was altered on its way into the history (%08x -> %08x)", xb, bits_f(((float *)o->hist_)[room])); }
          { uint32_t yb = bits_f(y); if (is_nan_f(yb)) h_cov_hit(CS_NAN_OUT); else if ((yb & 0x7fffffffu) == 0x7f800000u) h_cov_hit(CS_INF_OUT); else if (yb == 0) h_cov_hit(CS_ZERO_OUT); else if (yb == 0x80000000u) h_cov_hit(CS_NEGZERO_OUT); else if ((yb & 0x7f800000u) == 0) h_cov_hit(CS_DENORM_OUT); }
          h_emit(0, "p1 %08x", bits_f(y));
        } else {
          unsigned n = h_rmod(len + 6); if (n > 3000) n = 3000;
          float *in = alloc_f(n + 4), *out = alloc_f(n + 4);
          for (i = 0; i < n + 4; i++) { rfill(&in[i], dreg); out[i] = h_float_from_bits(0xEEEEEEEEu); }
          if (kind == 0) {
            fir_pn(o, in, out, n);
            if (n >= 1 && is_snan_f(bits_f(in[n - 1]))) { /* the last sample's cell is at room_+1 unless it wrapped; if it did not wrap check it */
              int ro = o->room_;
              if (ro + 1 < (int)len && bits_f(((float *)o->hist_)[ro + 1]) == bits_f(in[n - 1])) h_cov_hit(CS_FIR_PN_SNAN_HIST);
            }
          } else { iir_pn(o, in, out, n); if (n && is_nan_f(bits_f(in[0]))) h_cov_hit(CS_IIR_NAN); }
          for (i = 0; i < n; i++) {
            uint32_t yb = bits_f(out[i]);
            if (is_nan_f(yb)) h_cov_hit(CS_NAN_OUT); else if ((yb & 0x7fffffffu) == 0x7f800000u) h_cov_hit(CS_INF_OUT); else if (yb == 0) h_cov_hit(CS_ZERO_OUT); else if (yb == 0x80000000u) h_cov_hit(CS_NEGZERO_OUT); else if ((yb & 0x7f800000u) == 0) h_cov_hit(CS_DENORM_OUT);
            h_emit(0, "pn %08x", yb);
          }
          sysdep_free(out);
          sysdep_free(in);
        }
        if (h_rpct(5)) { if (kind == 0) fir_reset(o); else iir_reset(o); }
        h_dump_obj(h_sample(&ctr, 10, 151), kind == 0 ? "FIR" : "IIR", o, f_FirObj, NF_FirObj);
      }
      null_owned(o, offsetof(FirObj, hist_), 10);
      if (kind == 0) fir_d1(o); else iir_d1(o);
      sysdep_free(coef);
    } else if (kind == 2) {
      unsigned nAr = (unsigned)h_rrange(1, 10), nMa = (unsigned)h_rrange(1, 10);
      float *ar = alloc_f(nAr), *ma = alloc_f(nMa);
      ArmaObj *o = (ArmaObj *)h_obj_new(sizeof(ArmaObj));
      for (i = 0; i < nAr; i++) { if (cspecial && h_rpct(30)) rfill(&ar[i], RQ); else ar[i] = rfloat(R_UNIT) * 0.125f; }
      for (i = 0; i < nMa; i++) { if (cspecial && h_rpct(30)) rfill(&ma[i], RQ); else ma[i] = rfloat(R_UNIT); }
      if (h_rpct(50)) ar[0] = h_rpct(50) ? 1.0f : rfloat(R_UNIT) + 2.0f;
      arma_c1(o, nAr, nMa, ar, ma, extra);
      { uint32_t v = 0xA5A5A5A5u; memcpy(&o->y_, &v, 4); memcpy(&o->w_, &v, 4); }
      len = o->xlen_;
      nops = (unsigned)h_rrange(8, 30);
      for (op = 0; op < nops; op++) {
        int dreg = h_rpct(35) ? (cspecial ? RQ : RS) : h_rpct(40) ? R_ZERO : h_rpct(50) ? R_TINY : R_UNIT;
        if (h_rpct(50)) {
          uint32_t xb = rbits(dreg);
          float x, y;
          int xr = o->xr_;
          memcpy(&x, &xb, 4);
          y = arma_p1(o, x);
          if (is_snan_f(xb)) { h_cov_hit(CS_SNAN_ARG); if (bits_f(((float *)o->xhist_)[xr]) == xb || o->xr_ > xr) h_cov_hit(CS_ARMA_SNAN_HIST); }
          { uint32_t yb = bits_f(y); if (is_nan_f(yb)) h_cov_hit(CS_NAN_OUT); else if ((yb & 0x7fffffffu) == 0x7f800000u) h_cov_hit(CS_INF_OUT); else if (yb == 0) h_cov_hit(CS_ZERO_OUT); else if (yb == 0x80000000u) h_cov_hit(CS_NEGZERO_OUT); }
          h_emit(0, "p1 %08x", bits_f(y));
        } else {
          unsigned n = h_rmod(len + 6); if (n > 3000) n = 3000;
          float *in = alloc_f(n + 4), *out = alloc_f(n + 4);
          for (i = 0; i < n + 4; i++) { rfill(&in[i], dreg); out[i] = h_float_from_bits(0xEEEEEEEEu); }
          arma_pn(o, in, out, n);
          if (n >= 1 && is_snan_f(bits_f(in[n - 1])) && (o->xr_ + 1 < (int)o->xlen_) && bits_f(((float *)o->xhist_)[o->xr_ + 1]) == bits_f(in[n - 1])) h_cov_hit(CS_ARMA_PN_SNAN_HIST);
          for (i = 0; i < n; i++) h_emit(0, "pn %08x", bits_f(out[i]));
          sysdep_free(out);
          sysdep_free(in);
        }
        h_dump_obj(h_sample(&ctr, 10, 151), "ARMA", o, f_ArmaObj, NF_ArmaObj);
      }
      null_owned(o, offsetof(ArmaObj, ar_), 8); null_owned(o, offsetof(ArmaObj, xhist_), 8);
      arma_d1(o);
      sysdep_free(ma);
      sysdep_free(ar);
    } else {
      unsigned nDen = (unsigned)h_rrange(1, 8), nNum = (unsigned)h_rrange(1, 8);
      double *den = alloc_d(nDen), *num = alloc_d(nNum);
      GiirObj *o = (GiirObj *)h_obj_new(sizeof(GiirObj));
      for (i = 0; i < nDen; i++) den[i] = (cspecial && h_rpct(30)) ? rdouble(RQ) : rdouble(R_UNIT) * 0.125;
      for (i = 0; i < nNum; i++) num[i] = (cspecial && h_rpct(30)) ? rdouble(RQ) : rdouble(R_UNIT);
      if (h_rpct(60)) den[0] = h_rpct(50) ? 1.0 : (h_rpct(50) ? 0.0 : rdouble(R_UNIT) + 2.0);
      giir_c1(o, nDen, nNum, den, num, extra);
      { uint64_t v = 0xA5A5A5A5A5A5A5A5ull; memcpy(&o->y_, &v, 8); }
      len = o->lenNum_;
      nops = (unsigned)h_rrange(8, 30);
      for (op = 0; op < nops; op++) {
        int dreg = h_rpct(35) ? RS : h_rpct(40) ? R_ZERO : h_rpct(50) ? R_TINY : R_UNIT;
        if (h_rpct(50)) {
          uint32_t xb = rbits(dreg);
          float x, y;
          memcpy(&x, &xb, 4);
          y = giir_p1(o, x);
          if (is_nan_f(xb)) h_cov_hit(CS_GIIR_NAN);
          if (is_snan_f(xb)) h_cov_hit(CS_SNAN_ARG);
          { uint32_t yb = bits_f(y); if (is_nan_f(yb)) h_cov_hit(CS_NAN_OUT); else if ((yb & 0x7fffffffu) == 0x7f800000u) h_cov_hit(CS_INF_OUT); else if (yb == 0) h_cov_hit(CS_ZERO_OUT); else if (yb == 0x80000000u) h_cov_hit(CS_NEGZERO_OUT); else if ((yb & 0x7f800000u) == 0) h_cov_hit(CS_DENORM_OUT); }
          h_emit(0, "p1 %08x", bits_f(y));
        } else {
          unsigned n = h_rmod(len + 6); if (n > 3000) n = 3000;
          float *in = alloc_f(n + 4), *out = alloc_f(n + 4);
          for (i = 0; i < n + 4; i++) { rfill(&in[i], dreg); out[i] = h_float_from_bits(0xEEEEEEEEu); }
          giir_pn(o, in, out, n);
          for (i = 0; i < n; i++) h_emit(0, "pn %08x", bits_f(out[i]));
          sysdep_free(out);
          sysdep_free(in);
        }
        h_dump_obj(h_sample(&ctr, 10, 151), "GIIR", o, f_GiirObj, NF_GiirObj);
      }
      null_owned(o, offsetof(GiirObj, hden_), 10);
      giir_d1(o);
      sysdep_free(num);
      sysdep_free(den);
    }
    h_check_not_wild("special");
    h_trial_end("special");
  }
}

/* ============================================================================== scenario 6: setCoefficients, odd configs */
enum { C6_DEGEN = 20, C6_DEGEN_ARMA_NMA0, C6_DEGEN_GIIR_NDEN0, C6_DEGEN_GIIR_NNUM0, C6_DEGEN_EXTRA0, C6_SETC_OK = 0, C6_SETC_ERR, C6_TAPS_CHANGE, C6_TAPS_SAME, C6_ROOM_CLAMP, C6_ROOM_KEEP, C6_SMALL_CTOR, C6_BIGCTOR, C6_PROC_AFTER, C6_IIR };
static void sc_degenerate(unsigned long *ctr);
static void sc_setc(void) {
  unsigned long ctr = 0;
  unsigned trial;
  h_cov_define(C6_SETC_OK, "setCoefficients returned 0");
  h_cov_define(C6_SETC_ERR, "setCoefficients returned -1");
  h_cov_define(C6_TAPS_CHANGE, "taps changed");
  h_cov_define(C6_TAPS_SAME, "taps unchanged");
  h_cov_define(C6_ROOM_CLAMP, "room_ clamped");
  h_cov_define(C6_ROOM_KEEP, "room_ kept");
  h_cov_define(C6_SMALL_CTOR, "ctor with ntaps < 4 (taps_ == 0)");
  h_cov_define(C6_BIGCTOR, "ctor with ntaps >= 64");
  h_cov_define(C6_PROC_AFTER, "process after setCoefficients");
  h_cov_define(C6_IIR, "FloatIIR trial");
  h_scenario_begin(1);
  for (trial = 0; trial < 600; trial++) {
    int iir = (int)(trial & 1);
    unsigned ntaps = h_rpct(10) ? h_rmod(4) : 4u * (unsigned)h_rrange(1, 16) + h_rmod(4), extra = pick_extra(), nops, op, i, len;
    float *c0, *c1, *c2;
    FirObj *o = (FirObj *)h_obj_new(sizeof(FirObj));
    if (iir && extra == 0) extra = 3;
    c0 = alloc_f(ntaps + 1);
    c1 = alloc_f(80);
    c2 = alloc_f(80);
    for (i = 0; i < ntaps + 1; i++) c0[i] = rfloat(R_UNIT) * (iir ? 0.05f : 1.0f);
    for (i = 0; i < 80; i++) { c1[i] = rfloat(R_UNIT) * (iir ? 0.05f : 1.0f); c2[i] = rfloat(R_EXP) * (iir ? 0.01f : 1.0f); }
    h_wild = 0;
    H_CTOR_PROBE(o, f_FirObj, NF_FirObj, sizeof(FirObj), if (iir) iir_c1(o, ntaps, c0, extra); else fir_c1(o, ntaps, c0, extra));
    if (iir) h_cov_hit(C6_IIR);
    if (ntaps < 4) h_cov_hit(C6_SMALL_CTOR);
    if (ntaps >= 64) h_cov_hit(C6_BIGCTOR);
    len = o->histLen_;
    h_dump_obj(h_sample(&ctr, 12, 61), iir ? "IIR ctor" : "FIR ctor", o, f_FirObj, NF_FirObj);
    nops = (unsigned)h_rrange(4, 16);
    for (op = 0; op < nops; op++) {
      int r = (int)h_rmod(100);
      if (r < 40) {
        unsigned n = h_rpct(25) ? len + h_rmod(8) : h_rmod(len + 4), t0 = o->taps_;
        if ((n & ~3u) < len && n > 76) n = h_rmod(77); /* the coefficient arrays hold 80 floats: a tap count above that would read past them */
        int room0 = o->room_, ret;
        float *nc = h_rpct(50) ? c1 : c2;
        ret = iir ? iir_setc(o, nc, n) : fir_setc(o, nc, n);
        if ((n & ~3u) >= len) { if (ret != -1) h_die3("setCoefficients(%u) with histLen %u returned %d", n, len, ret); h_cov_hit(C6_SETC_ERR); }
        else {
          if (ret != 0) h_die3("setCoefficients(%u) returned %d", n, ret);
          h_cov_hit(C6_SETC_OK);
          if (o->coef_ != nc) h_die3("setCoefficients did not store the pointer");
          if (t0 != (n & ~3u)) { h_cov_hit(C6_TAPS_CHANGE); if (o->room_ < room0) h_cov_hit(C6_ROOM_CLAMP); else h_cov_hit(C6_ROOM_KEEP); } else h_cov_hit(C6_TAPS_SAME);
        }
        h_emit(0, "setc %u -> %d", n, ret);
      } else if (r < 60) {
        if (iir) iir_reset(o); else fir_reset(o);
      } else if (o->taps_ >= 4 && !(iir && 0)) {
        /* process a few samples (only when the wrap copy is safe: taps >= 4) */
        unsigned n = h_rmod(2 * len + 3); if (n > 3000) n = 3000;
        float *in = alloc_f(n + 4), *out = alloc_f(n + 4);
        for (i = 0; i < n + 4; i++) { rfill(&in[i], R_UNIT); out[i] = h_float_from_bits(0xEEEEEEEEu); }
        if (iir) iir_pn(o, in, out, n); else if (h_rpct(30)) { for (i = 0; i < n; i++) out[i] = fir_p1(o, in[i]); } else fir_pn(o, in, out, n);
        for (i = 0; i < n; i++) h_emit(0, "o %08x", bits_f(out[i]));
        h_cov_hit(C6_PROC_AFTER);
        sysdep_free(out);
        sysdep_free(in);
      }
      h_dump_obj(h_sample(&ctr, 12, 83), iir ? "IIR" : "FIR", o, f_FirObj, NF_FirObj);
    }
    h_check_not_wild("setc");
    null_owned(o, offsetof(FirObj, hist_), 10);
    if (iir) iir_d1(o); else fir_d1(o);
    sysdep_free(c2);
    sysdep_free(c1);
    sysdep_free(c0);
    h_trial_end("setc");
  }
  sc_degenerate(&ctr);
}

/* fill the heap block a pointer field owns with garbage (the harness owns the memory; a reset must clean it) */
static void dirty_block(const void *p) {
  HAl *a = h_find_al(p, 0);
  if (a && a->p == (const uint8_t *)p && a->size) memset(a->p, 0xA7, a->size);
}

/* degenerate constructors: constructor + reset + destructor only (a filter with no taps is undefined once it PROCESSES: the first sample is stored one
 * element past the history block).  Reaches the `len != 0` guards of reset / the ctor zero fills and the idx_ final values. */
static void sc_degenerate(unsigned long *ctr) {
  unsigned trial;
  h_cov_define(C6_DEGEN, "degenerate constructor trials");
  h_cov_define(C6_DEGEN_ARMA_NMA0, "FloatARMA with nMa == 0");
  h_cov_define(C6_DEGEN_GIIR_NDEN0, "GenericIIR with nDen == 0");
  h_cov_define(C6_DEGEN_GIIR_NNUM0, "GenericIIR with nNum == 0");
  h_cov_define(C6_DEGEN_EXTRA0, "degenerate with extra == 0");
  for (trial = 0; trial < 240; trial++) {
    unsigned extra = (trial & 1u) ? 0u : (unsigned)h_rrange(1, 5), kind = trial % 3, i;
    unsigned long nm0, nf0;
    h_wild = 0;
    h_cov_hit(C6_DEGEN);
    if (extra == 0) h_cov_hit(C6_DEGEN_EXTRA0);
    if (kind == 0) { /* FloatFIR / FloatIIR with ntaps < 4 (taps_ == 0) */
      int iir = (trial / 3) & 1;
      unsigned ntaps = h_rmod(4);
      float *c = alloc_f(8);
      FirObj *o = (FirObj *)h_obj_new(sizeof(FirObj));
      for (i = 0; i < 8; i++) rfill(&c[i], R_UNIT);
      H_CTOR_PROBE(o, f_FirObj, NF_FirObj, sizeof(FirObj), if (iir) iir_c1(o, ntaps, c, extra); else fir_c1(o, ntaps, c, extra));
      h_dump_obj(h_sample(ctr, 8, 17), "DEG FIR ctor", o, f_FirObj, NF_FirObj);
      dirty_block(o->hist_);
      if (iir) iir_reset(o); else fir_reset(o);
      h_dump_obj(h_sample(ctr, 8, 17), "DEG FIR reset", o, f_FirObj, NF_FirObj);
      if (iir) iir_d1(o); else fir_d1(o);
      sysdep_free(c);
    } else if (kind == 1) { /* FloatARMA with nMa == 0 (nAr >= 1: ar[0] is read and ar_[0] written) */
      unsigned nAr = (unsigned)h_rrange(1, 6);
      float *ar = alloc_f(8), *ma = alloc_f(4);
      ArmaObj *o = (ArmaObj *)h_obj_new(sizeof(ArmaObj));
      for (i = 0; i < 8; i++) rfill(&ar[i], R_UNIT);
      for (i = 0; i < 4; i++) rfill(&ma[i], R_UNIT);
      if (h_rpct(30)) { ar[0] = 1.0f; }
      h_cov_hit(C6_DEGEN_ARMA_NMA0);
      H_CTOR_PROBE(o, f_ArmaObj, NF_ArmaObj, sizeof(ArmaObj), arma_c1(o, nAr, 0, ar, ma, extra));
      { uint32_t v = 0xA5A5A5A5u; memcpy(&o->y_, &v, 4); memcpy(&o->w_, &v, 4); }
      h_dump_obj(h_sample(ctr, 8, 17), "DEG ARMA ctor", o, f_ArmaObj, NF_ArmaObj);
      dirty_block(o->xhist_); dirty_block(o->yhist_);
      arma_reset(o);
      h_dump_obj(h_sample(ctr, 8, 17), "DEG ARMA reset", o, f_ArmaObj, NF_ArmaObj);
      arma_d1(o);
      sysdep_free(ma);
      sysdep_free(ar);
    } else { /* GenericIIR with nDen == 0 and / or nNum == 0 */
      unsigned nDen = h_rpct(60) ? 0u : (unsigned)h_rrange(1, 3), nNum = (nDen != 0 || h_rpct(30)) ? 0u : (unsigned)h_rrange(1, 3);
      double *den = alloc_d(4), *num = alloc_d(4);
      GiirObj *o = (GiirObj *)h_obj_new(sizeof(GiirObj));
      for (i = 0; i < 4; i++) { den[i] = rdouble(R_UNIT); num[i] = rdouble(R_UNIT); }
      if (nDen == 0) h_cov_hit(C6_DEGEN_GIIR_NDEN0);
      if (nNum == 0) h_cov_hit(C6_DEGEN_GIIR_NNUM0);
      nm0 = h_nmalloc;
      H_CTOR_PROBE(o, f_GiirObj, NF_GiirObj, sizeof(GiirObj), giir_c1(o, nDen, nNum, den, num, extra));
      { uint64_t v = 0xA5A5A5A5A5A5A5A5ull; memcpy(&o->y_, &v, 8); }
      if (h_nmalloc != nm0 + 2) h_die3("GenericIIR degenerate ctor must allocate two blocks");
      h_dump_obj(h_sample(ctr, 8, 17), "DEG GIIR ctor", o, f_GiirObj, NF_GiirObj);
      dirty_block(o->hnum_); dirty_block(o->hden_);
      giir_reset(o);
      h_dump_obj(h_sample(ctr, 8, 17), "DEG GIIR reset", o, f_GiirObj, NF_GiirObj);
      nf0 = h_nfree;
      giir_d1(o);
      if (h_nfree != nf0 + 2) h_die3("GenericIIR dtor must free both blocks (also when they are empty)");
      sysdep_free(num);
      sysdep_free(den);
    }
    h_check_not_wild("degenerate");
    h_trial_end("degenerate");
  }
}

/* ============================================================ scenario 11: designed vectors for the ACCUMULATION ORDER */
/* The result of a dot product is rounded to float32, so the grouping of the extended-precision partial sums (two accumulators, even / odd terms, result
 * B + A) is invisible for ordinary data: a different grouping changes the 64-bit sum only in the last bits, which the float rounding almost always hides.
 * Cancellation designs expose it: terms like {-2^-65, 2^-64, 1, -1} whose float result depends on which partial sums were formed.  The designs are
 * SEARCHED at start-up (deterministic, from the harness PRNG) against three alternative groupings, and every function that holds a dot-product loop is
 * driven with them through poked histories (coefficients all 1.0f): FloatFIR process(float) / block, FloatIIR block, FloatARMA process(float) / block
 * (y_ and w_ individually); tap counts 4, 8, 12 and the poked remainder counts 5, 6, 7, 9, 10, 11.  The expected value is the model grouping; blob and
 * port are compared with it AND with each other. */
static LD sum_split(const float *p, unsigned n) { Acc2 s = { 0, 0 }; unsigned k; for (k = 0; k < n; k++) acc_add_n(&s, k, n, (LD)p[k]); return acc_sum(&s); }
static LD sum_seq(const float *p, unsigned n) { LD a = 0; unsigned k; for (k = 0; k < n; k++) a += p[k]; return a; }
static LD sum_adjacent(const float *p, unsigned n) { LD t = 0; unsigned k; for (k = 0; k < n; k += 2) { LD pr = p[k]; if (k + 1 < n) pr += p[k + 1]; t += pr; } return t; }
static LD sum_firstblock(const float *p, unsigned n) { Acc2 s = { 0, 0 }; LD a; unsigned k; for (k = 0; k < 4 && k < n; k++) acc_add_n(&s, k, 4, (LD)p[k]); a = acc_sum(&s); for (; k < n; k++) a += p[k]; return a; }
static float mkf(int e, uint32_t m, int sg) { uint32_t b = (sg ? 0x80000000u : 0) | ((uint32_t)(127 + e) << 23) | m; return h_float_from_bits(b); }
#define NDES 24
static float g_des[16][NDES][12];
static void make_designs(void) {
  static const int es[] = { 0, -23, -24, -25, -30, -40, -55, -62, -63, -64, -65, -66, -70 };
  float pool[3 * 13];
  unsigned np = 0, n, d, t;
  for (t = 0; t < 13; t++) { pool[np++] = mkf(es[t], 0, 0); pool[np++] = mkf(es[t], 0, 1); pool[np++] = mkf(es[t], 1, 0); }
  for (n = 4; n <= 12; n++)
    for (d = 0; d < NDES; d++) {
      for (t = 0; t < 4000000; t++) {
        float p[12];
        unsigned k;
        for (k = 0; k < n; k++) p[k] = pool[h_rmod(np)];
        { float a = (float)sum_split(p, n);
          if (bits_f(a) != bits_f((float)sum_seq(p, n)) && bits_f(a) != bits_f((float)sum_adjacent(p, n)) && (n <= 4 || bits_f(a) != bits_f((float)sum_firstblock(p, n)))) {
            memcpy(g_des[n][d], p, sizeof p);
            break;
          } }
      }
      if (t == 4000000) h_die3("no accumulation-order design found for %u terms", n);
    }
}
enum { C11_FIR_P1 = 0, C11_FIR_PN, C11_IIR, C11_ARMA_P1, C11_ARMA_PN, C11_REMAINDER, C11_DESIGNS };
static void sc_adversarial(void) {
  unsigned long ctr = 0;
  unsigned n, d, fn;
  float ones[16], zero4[4] = { 0, 0, 0, 0 };
  unsigned i;
  h_cov_define(C11_FIR_P1, "FIR process(float) design");
  h_cov_define(C11_FIR_PN, "FIR block design");
  h_cov_define(C11_IIR, "IIR block design");
  h_cov_define(C11_ARMA_P1, "ARMA process(float) design (y_ and w_)");
  h_cov_define(C11_ARMA_PN, "ARMA block design (y_ and w_)");
  h_cov_define(C11_REMAINDER, "design with a remainder (poked tap count)");
  h_cov_define(C11_DESIGNS, "designs applied");
  (void)zero4;
  h_scenario_begin(1);
  make_designs();
  for (i = 0; i < 16; i++) ones[i] = 1.0f;
  for (n = 4; n <= 12; n++)
    for (d = 0; d < NDES; d++)
      for (fn = 0; fn < 5; fn++) {
        const float *p = g_des[n][d], *q = g_des[n][(d + 7) % NDES];
        unsigned extra = 1 + h_rmod(6), len = n + extra, k;
        float *c = alloc_f(16), res = 0;
        float want, got;
        h_wild = 0;
        for (k = 0; k < 16; k++) c[k] = 1.0f;
        h_cov_hit(C11_DESIGNS);
        if (n & 3u) h_cov_hit(C11_REMAINDER);
        if (fn <= 2) {
          FirObj *o = (FirObj *)h_obj_new(sizeof(FirObj));
          float *hist, *out = alloc_f(4);
          { unsigned nb4 = (n + 3u) & ~3u;
            if (fn == 2) iir_c1(o, nb4, c, extra); else fir_c1(o, nb4, c, extra);
            if (n != nb4) { o->taps_ = n; if (fn == 2) iir_reset(o); else fir_reset(o); } }
          len = o->histLen_;
          hist = (float *)o->hist_;
          if (n > len) h_die3("design larger than the history");
          if (fn == 2) { /* IIR: hist[room + k] = p[k], k < n; input 0 */
            float zero = 0.0f;
            for (k = 0; k < n; k++) hist[o->room_ + (int)k] = p[k];
            iir_pn(o, &zero, out, 1);
            res = out[0];
            want = (float)(sum_split(p, n) + (LD)0.0f);
            h_cov_hit(C11_IIR);
          } else { /* FIR: hist[room + k] = p[k] for k >= 1, p[0] is the argument */
            for (k = 1; k < n; k++) hist[o->room_ + (int)k] = p[k];
            if (fn == 0) { res = fir_p1(o, p[0]); h_cov_hit(C11_FIR_P1); }
            else { float in0 = p[0]; fir_pn(o, &in0, out, 1); res = out[0]; h_cov_hit(C11_FIR_PN); }
            want = (float)sum_split(p, n);
          }
          got = res;
          if (bits_f(got) != bits_f(want)) h_die3("design n=%u d=%u fn=%u: blob/port %08x, model %08x", n, d, fn, bits_f(got), bits_f(want));
          h_emit(h_sample(&ctr, 6, 17), "ADV fn%u n%u d%u %08x", fn, n, d, bits_f(got));
          h_dump_obj(0, "ADV", o, f_FirObj, NF_FirObj);
          null_owned(o, offsetof(FirObj, hist_), 10);
          if (fn == 2) iir_d1(o); else fir_d1(o);
          sysdep_free(out);
        } else {
          ArmaObj *o = (ArmaObj *)h_obj_new(sizeof(ArmaObj));
          float *out = alloc_f(4), *xh, *yh;
          unsigned nb = (n + 3u) & ~3u;
          arma_c1(o, nb, nb, ones, ones, extra);
          if (n != nb) { o->nAr4_ = n; o->nMa4_ = n; arma_reset(o); }
          xh = (float *)o->xhist_;
          yh = (float *)o->yhist_;
          for (k = 1; k < n; k++) { xh[o->xr_ + (int)k] = p[k]; yh[o->yr_ + (int)k] = q[k]; }
          yh[o->yr_] = 1.0f; /* the stale slot is multiplied by ar_[0] == 0 */
          if (fn == 3) { res = arma_p1(o, p[0]); h_cov_hit(C11_ARMA_P1); }
          else { float in0 = p[0]; arma_pn(o, &in0, out, 1); res = out[0]; h_cov_hit(C11_ARMA_PN); }
          { float qq[12], y1, w;
            for (k = 0; k < n; k++) qq[k] = k ? q[k] : 0.0f;
            y1 = (float)sum_split(p, n); w = (float)sum_split(qq, n);
            want = (float)((LD)y1 - (LD)w);
            if (bits_f(o->y_) != bits_f(want) || bits_f(res) != bits_f(want)) h_die3("ARMA design n=%u d=%u fn=%u: y_ %08x ret %08x model %08x (y1 %08x w %08x w_ %08x)", n, d, fn, bits_f(o->y_), bits_f(res), bits_f(want), bits_f(y1), bits_f(w), bits_f(o->w_));
            if (bits_f(o->w_) != bits_f(w)) h_die3("ARMA design n=%u d=%u: w_ %08x, model %08x", n, d, bits_f(o->w_), bits_f(w)); }
          h_emit(h_sample(&ctr, 6, 17), "ADV fn%u n%u d%u y_ %08x w_ %08x", fn, n, d, bits_f(o->y_), bits_f(o->w_));
          h_dump_obj(0, "ADV", o, f_ArmaObj, NF_ArmaObj);
          null_owned(o, offsetof(ArmaObj, yhist_), 10);
          arma_d1(o);
          sysdep_free(out);
        }
        sysdep_free(c);
        h_check_not_wild("adversarial");
        h_trial_end("adversarial");
      }
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P02_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  p02_layout_check();
  h_x87_check();
  h_arena_init();
  switch (sc) {
    case 0: sc_fir_iir(0, 0); break;
    case 1: sc_fir_iir(1, 0); break;
    case 2: sc_arma(0, 0); break;
    case 3: sc_arma(1, 0); break;
    case 4: sc_giir(); break;
    case 5: sc_special(0); break;
    case 6: sc_setc(); break;
    case 7: sc_special(1); break;
    case 8: sc_fir_iir(0, 1); break;
    case 9: sc_fir_iir(1, 1); break;
    case 10: sc_arma(1, 1); break;
    case 11: sc_adversarial(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu", h_nmalloc, h_nfree);
  h_cov_finish();
  return 0;
}
