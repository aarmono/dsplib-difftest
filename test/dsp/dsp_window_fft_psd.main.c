/*
 * Differential test harness for port chunk P03 (src/dsp/dsp_window_fft_psd.cpp): the window / statistics function templates
 * (boxcar, hanning, hamming, blackman, designWindow, sinc, sum, mean, sqrSum, Var, Std), four1 / realfft, LowPassFIR<float> (4 functions),
 * Psd (8), Agc<float> (4), SineWave<float, float> (3) of dsplibs.o: 32 functions.
 *
 * ONE C source, no #ifdef except the type of the extended-precision return values (below), built by test/dsp/dsp_window_fft_psd.sh
 * against the blob (dsplibs.o), against the port (-O0 / -O2) and for x86-64 (diffed against the 32-bit BLOB's output = the golden).
 * Shared library: test/harness/ (PRNG, hashed canonical output, coverage counters exit 2, arena allocator with red zones, object pool,
 * field-wise dump).  The mirror structs / field tables are GENERATED (tools/layout_gen.py from test/p03.layout -> test/p03_layout.h).
 *
 * What is compared: every output sample of every function as a BIT PATTERN (hashed blocks with a sample of lines printed), every
 * object (all fields; floats as bits; the heap blocks it owns), the malloc / free sequence (incl. the block the 4-argument
 * LowPassFIR::design leaks), the red zones.  INDEPENDENT MODELS (not copies of the port; exact math in __float128 with the blob's double
 * constants, or an explicit state machine in long double): windows, sinc, naive DFT for four1 / realfft / Psd, the windowed-sinc design,
 * Agc, SineWave; models are compared within a stated tolerance (1 float ulp for closed forms; the transform / design models with a
 * relative bound: float32 data through a 12-stage transform) because the blob's extended arithmetic is not exact math; the exact
 * comparison is blob vs port.  A model violation exits 3.
 *
 * Scenarios (each its own process; 0..2, 4..9 produce output that the x86-64 build reproduces, 3 = the extended-precision return values of
 * sum / mean / sqrSum / Var / Std, readable only from st(0) on i386 -> x87-only, not compared on x86-64):
 *   0 windows        1 sinc           2 statistics (float-rounded)   3 statistics (extended bits)   4 four1       5 realfft
 *   6 Psd            7 LowPassFIR     8 Agc                          9 SineWave                    10 transcendental sites: the instruction
 *     sequences of the blob on dense / special inputs (blob vs port; the 64-bit port reproduces them because it uses the instructions too)
 *
 * Excluded because the BLOB is undefined / crashes there (documented in the port): Psd with N == 0, N == 1 (writes buf_[2], one past the
 * block) or N == overlap (division by zero), n < overlap (huge segment count); realfft / four1 on blocks shorter than the one-based layout.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>
#include "../harness/harness.h"
#include "../p03_layout.h"

#define L(m) __asm__(m)
typedef long double LD;
typedef __float128 Q;

/* The blob's sum / mean / sqrSum / Var / Std return an UNROUNDED extended value in st(0): readable as a long double on i386.  On x86-64
 * the port returns a float (xmm0), the 64-bit run only has the float-rounded value. */
#if defined(__i386__)
typedef long double XRET;
#else
typedef float XRET;
#endif

extern void w_boxcar(float *, unsigned) L("_Z6boxcarIfEvPT_j");
extern void w_hanning(float *, unsigned) L("_Z7hanningIfEvPT_j");
extern void w_hamming(float *, unsigned) L("_Z7hammingIfEvPT_j");
extern void w_blackman(float *, unsigned) L("_Z8blackmanIfEvPT_j");
extern void w_design(int, float *, unsigned) L("_Z12designWindowIfEv10WindowTypePT_j");
extern float f_sinc(float) L("_Z4sincIfET_S0_");
extern XRET f_sum(float *, unsigned) L("_Z3sumIfET_PS0_j");
extern XRET f_mean(float *, unsigned) L("_Z4meanIfET_PS0_j");
extern XRET f_sqrsum(float *, unsigned) L("_Z6sqrSumIfET_PS0_j");
extern XRET f_var(float *, unsigned) L("_Z3VarIfET_PS0_j");
extern XRET f_std(float *, unsigned) L("_Z3StdIfET_PS0_j");
extern void f_four1(float *, unsigned long, int) L("_Z5four1Pfmi");
extern void f_realfft(float *, unsigned long, int) L("_Z7realfftPfmi");
extern void lpf_c1(void *, unsigned, float, int, float) L("_ZN10LowPassFIRIfEC1Ejf10WindowTypef");
extern void lpf_d1(void *) L("_ZN10LowPassFIRIfED1Ev");
extern int lpf_d4(void *, unsigned, float, int, float) L("_ZN10LowPassFIRIfE6designEjf10WindowTypef");
extern int lpf_d5(void *, unsigned, float, float, const float *, int) L("_ZN10LowPassFIRIfE6designEjffPKfi");
extern void psd_c1(void *, unsigned, int, unsigned) L("_ZN3PsdC1Ej10WindowTypej");
extern void psd_c2(void *, unsigned, int, unsigned) L("_ZN3PsdC2Ej10WindowTypej");
extern void psd_d1(void *) L("_ZN3PsdD1Ev");
extern void psd_d2(void *) L("_ZN3PsdD2Ev");
extern void psd_setov(void *, unsigned) L("_ZN3Psd16setOverlapLengthEj");
extern void psd_setwt(void *, int) L("_ZN3Psd13setWindowTypeE10WindowType");
extern void psd_proc(void *, float *, unsigned, float *, int) L("_ZN3Psd7processEPfjS0_NS_12OutputOptionE");
extern void psd_freq(const void *, float *, float) L("_ZNK3Psd14getFrequenciesEPff");
extern void agc_c1(void *) L("_ZN3AgcIfEC1Ev");
extern void agc_reset(void *) L("_ZN3AgcIfE5resetEv");
extern void agc_freeze(void *) L("_ZN3AgcIfE6freezeEv");
extern void agc_proc(void *, const float *, float *, unsigned) L("_ZN3AgcIfE7processEPKfPfj");
/* float arguments by value: on i386 a float is a plain 4-byte stack word, so the bits can be passed as uint32_t WITHOUT a trip through st(0) (which would quiet a
 * signalling NaN); on x86-64 floats travel in SSE registers, where a movss is exact too */
#if defined(__i386__)
typedef uint32_t farg_t;
#define FARG(bits) ((uint32_t)(bits))
#else
typedef float farg_t;
static float farg_from_bits(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }
#define FARG(bits) farg_from_bits(bits)
#endif
extern void sw_c1(void *, farg_t, farg_t, farg_t, farg_t) L("_ZN8SineWaveIffEC1Effff");
extern void sw_d1(void *) L("_ZN8SineWaveIffED1Ev");
extern void sw_gen(void *, float *, unsigned long) L("_ZN8SineWaveIffE8generateEPfm");

/* ------------------------------------------------------------------------------------------------ small helpers */
static int g_scenario;
static void scen_begin(void) {
  h_scenario_begin(1);
  if (g_scenario != 6 && g_scenario != 7 && g_scenario != 13) h_covreq[H_COV_DTOR_NULL] = 0; /* only Psd / LowPassFIR have a destructor that tests its block pointers */
}
static uint32_t bits_f(float f) { return h_bits_of(f); }
static int is_nan_f(uint32_t b) { return (b & 0x7f800000u) == 0x7f800000u && (b & 0x007fffffu) != 0; }
static float f_from(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }
static uint64_t fnv_blk(const void *p, size_t n) {
  const uint8_t *q = (const uint8_t *)p;
  uint64_t h = 0xcbf29ce484222325ull;
  size_t i;
  for (i = 0; i < n; i++) { h ^= q[i]; h *= 0x100000001b3ull; }
  return h;
}
static float *alloc_f(unsigned n) { return (float *)sysdep_malloc(n * 4u); }
static unsigned long g_ctr;
#define EMIT_BLK(tag, p, n, a, b) h_emit(h_sample(&g_ctr, 40, 61), "%s a=%u b=%d n=%u h=%016llx first=%08x last=%08x", tag, (unsigned)(a), (int)(b), (unsigned)(n), \
    (unsigned long long)fnv_blk((p), (size_t)(n) * 4), (n) ? bits_f((p)[0]) : 0u, (n) ? bits_f((p)[(n) - 1]) : 0u)

/* ordered integer key of a float (monotonic over the real line; NaN excluded by the caller) */
static int64_t fkey(float f) { int32_t b; memcpy(&b, &f, 4); return b < 0 ? -(int64_t)(b & 0x7fffffff) : (int64_t)b; }
static int64_t ulps(float a, float b) { int64_t d = fkey(a) - fkey(b); return d < 0 ? -d : d; }
static unsigned long g_model_checked, g_model_exact;
/* model check: got within `tol` float ulps of the exact value `want` (|want| below `floor`: absolute check 1e-30) */
static void chk_q(const char *what, unsigned idx, float got, Q want, int tol, double floor_) {
  float w = (float)want;
  Q aw = want < 0 ? -want : want;
  if (aw != aw) { /* model NaN */
    if (!is_nan_f(bits_f(got))) h_die3("%s[%u]: model NaN, got %08x", what, idx, bits_f(got));
    return;
  }
  if (is_nan_f(bits_f(got))) h_die3("%s[%u]: got NaN %08x, model %.9g", what, idx, bits_f(got), (double)want);
  g_model_checked++;
  if (bits_f(got) == bits_f(w)) { g_model_exact++; return; }
  if (aw < (Q)floor_) { double d = (double)got - (double)want; if (d < 0) d = -d; if (d <= floor_) return; }
  if (ulps(got, w) > tol) h_die3("%s[%u]: got %08x (%.9g), model %08x (%.9g), %lld ulps", what, idx, bits_f(got), (double)got, bits_f(w), (double)w, (long long)ulps(got, w));
}

/* float128 trigonometry (Taylor series after reduction mod 2 pi; ~1e-31 absolute error): the models' only transcendental */
static const Q QPI = 3.14159265358979323846264338327950288419716939937510582097494Q;
static Q q_reduce(Q x) {
  Q t = x / (2 * QPI);
  long long k = (long long)(t < 0 ? t - 0.5Q : t + 0.5Q);
  return x - (Q)k * 2 * QPI;
}
static Q q_sin(Q x) {
  Q r = q_reduce(x), t = r, s = r, r2 = r * r;
  int m;
  for (m = 1; m < 40; m++) { t = -t * r2 / (Q)((2 * m) * (2 * m + 1)); s += t; }
  return s;
}
static Q q_cos(Q x) {
  Q r = q_reduce(x), t = 1, s = 1, r2 = r * r;
  int m;
  for (m = 1; m < 40; m++) { t = -t * r2 / (Q)((2 * m - 1) * (2 * m)); s += t; }
  return s;
}

/* ------------------------------------------------------------------------------------------------ value generators */
enum { R_UNIT = 0, R_EXP, R_INT, R_HUGE, R_TINY, R_ZERO, R_SPECIAL, R_QNAN1, R_INFONLY };
static uint32_t rbits(int regime) {
  uint32_t r = h_rnd(), sign = (r & 1u) << 31;
  switch (regime) {
    case R_UNIT: { float f = (float)((int32_t)(h_rnd() >> 7) - (1 << 24)) / 16777216.0f; uint32_t b; memcpy(&b, &f, 4); return b; }
    case R_EXP: return sign | ((uint32_t)(127 + (int)h_rmod(41) - 20) << 23) | (h_rnd() & 0x7fffffu);
    case R_INT: { float f = (float)((int)h_rmod(17) - 8); uint32_t b; memcpy(&b, &f, 4); return b; }
    case R_HUGE: return sign | ((uint32_t)(127 + 100 + (int)h_rmod(28)) << 23) | (h_rnd() & 0x7fffffu);
    case R_TINY: return sign | ((h_rpct(50) ? 0u : (uint32_t)(1 + h_rmod(3))) << 23) | (h_rnd() & 0x7fffffu);
    case R_ZERO: return sign;
    case R_INFONLY: return h_rpct(8) ? (sign | 0x7f800000u) : rbits(R_UNIT); /* mostly unit data, some +-inf (the NaNs it makes are all the default NaN) */
    case R_QNAN1: return h_rpct(6) ? (0x7fc00000u | (h_rnd() & 0x3fffffu)) : rbits(R_UNIT); /* mostly unit data, a few positive quiet NaNs with payload (no inf: see NOTES) */
    default: { /* inf / quiet NaN with payload (no signalling NaNs: see NOTES, float rule 6) */
      uint32_t pay = h_rnd() & 0x003fffffu, k = h_rmod(3);
      if (k == 0) return sign | 0x7f800000u;
      if (k == 1) return sign | 0x7fc00000u | pay;
      return sign | 0x7fc00000u;
    }
  }
}
static void rfill(float *dst, int regime) { uint32_t b = rbits(regime); memcpy(dst, &b, 4); }
static float rfloat(int regime) { uint32_t b = rbits(regime); return f_from(b); } /* finite regimes only (a float returned by value goes through st0) */

/* the 12 sites need these constants exactly: checked in scenario 10 against the blob's own values */

/* ================================================================================================ scenario 0: windows */
enum { C0_N0 = 0, C0_N1, C0_N2, C0_SMALL, C0_BIG, C0_POW2, C0_BOX, C0_HANN, C0_HAMM, C0_BLACK, C0_DEFAULT, C0_DIRECT, C0_MODEL };
static Q win_model(int type, unsigned i, unsigned n) {
  const Q twopi = (Q)6.283185307179586, fourpi = (Q)12.566370614359172;
  switch (type) {
    case 1: return ((Q)1 - q_cos((Q)(i + 1) * twopi / (Q)(n + 1))) * (Q)0.5;
    case 2: return (Q)0.54 - (Q)0.46 * q_cos((Q)i * twopi / (Q)(n - 1));
    case 3: return (Q)0.08 * q_cos((Q)i * fourpi / (Q)(n - 1)) + ((Q)0.42 - (Q)0.5 * q_cos((Q)i * twopi / (Q)(n - 1)));
    default: return 1;
  }
}
static void win_check(int type, const float *w, unsigned n) {
  unsigned i, step = n > 200 ? n / 64 : 1; /* the exact comparison is blob vs port; the exact-math model is a sanity check */
  if (type == 0) { for (i = 0; i < n; i++) if (bits_f(w[i]) != 0x3f800000u) h_die3("boxcar[%u] = %08x", i, bits_f(w[i])); return; }
  if ((type == 2 || type == 3) && n == 1) { if (!is_nan_f(bits_f(w[0]))) h_die3("hamming/blackman n=1 must be NaN: %08x", bits_f(w[0])); return; }
  for (i = 0; i < n; i += step) chk_q(type == 1 ? "hanning" : type == 2 ? "hamming" : "blackman", i, w[i], win_model(type, i, n), 1, 1e-30);
  h_cov_hit(C0_MODEL);
}
static void sc_windows(void) {
  unsigned trial, n, nn = 0;
  static const unsigned fixed[] = { 255, 256, 257, 511, 512, 513, 1000, 1023, 1024, 1025, 2047, 2048, 4096, 8000, 65536 };
  h_cov_define(C0_N0, "n == 0"); h_cov_define(C0_N1, "n == 1"); h_cov_define(C0_N2, "n == 2"); h_cov_define(C0_SMALL, "n 3..130");
  h_cov_define(C0_BIG, "n >= 4096"); h_cov_define(C0_POW2, "power of two"); h_cov_define(C0_BOX, "boxcar"); h_cov_define(C0_HANN, "hanning");
  h_cov_define(C0_HAMM, "hamming"); h_cov_define(C0_BLACK, "blackman"); h_cov_define(C0_DEFAULT, "designWindow with an out-of-range type");
  h_cov_define(C0_DIRECT, "direct calls of the four families"); h_cov_define(C0_MODEL, "model-checked windows");
  scen_begin();
  for (trial = 0; trial < 131 + 200 + sizeof fixed / sizeof fixed[0]; trial++) {
    int t;
    if (trial <= 130) n = trial;
    else if (trial < 131 + 200) n = 131 + h_rmod(2000) + (h_rpct(10) ? h_rmod(2000) : 0u);
    else n = fixed[trial - 331];
    nn++;
    if (n == 0) h_cov_hit(C0_N0); else if (n == 1) h_cov_hit(C0_N1); else if (n == 2) h_cov_hit(C0_N2); else if (n <= 130) h_cov_hit(C0_SMALL);
    if (n >= 4096) h_cov_hit(C0_BIG);
    if (n && !(n & (n - 1))) h_cov_hit(C0_POW2);
    for (t = 0; t < 7; t++) {
      static const int types[7] = { 0, 1, 2, 3, 4, 77, -1 };
      int ty = types[t];
      float *w = alloc_f(n + 1);
      unsigned i;
      for (i = 0; i < n + 1; i++) w[i] = f_from(0x7fe5a5a5u); /* poison (also guards the element after the window) */
      w_design(ty, w, n);
      if (bits_f(w[n]) != 0x7fe5a5a5u) h_die3("designWindow wrote past n");
      { int fam = (ty >= 0 && ty <= 3) ? ty : 0; win_check(fam, w, n); h_cov_hit(fam == 0 ? C0_BOX : fam == 1 ? C0_HANN : fam == 2 ? C0_HAMM : C0_BLACK); if (t >= 4) h_cov_hit(C0_DEFAULT); }
      EMIT_BLK("design", w, n, n, ty);
      sysdep_free(w);
    }
    { /* the four families directly */
      float *w = alloc_f(n + 1);
      unsigned i, f;
      for (f = 0; f < 4; f++) {
        for (i = 0; i < n + 1; i++) w[i] = f_from(0x7fe5a5a5u);
        if (f == 0) w_boxcar(w, n); else if (f == 1) w_hanning(w, n); else if (f == 2) w_hamming(w, n); else w_blackman(w, n);
        if (bits_f(w[n]) != 0x7fe5a5a5u) h_die3("window family %u wrote past n", f);
        EMIT_BLK("direct", w, n, n, f);
      }
      h_cov_hit(C0_DIRECT);
      sysdep_free(w);
    }
    h_trial_end("windows");
  }
  h_emit(1, "windows: %u sizes", nn);
}

/* ================================================================================================ scenario 1: sinc */
enum { C1_ZERO = 0, C1_NEGZERO, C1_NAN, C1_INF, C1_DENORM, C1_INT, C1_HALFINT, C1_GRID, C1_RANDOM, C1_HUGE, C1_BEYOND63, C1_MODEL, C1_NEG };
static void sinc_one(float x, unsigned long *ctr, int model) {
  float r = f_sinc(x);
  uint32_t bx = bits_f(x), br = bits_f(r);
  h_emit(h_sample(ctr, 40, 997), "sinc %08x -> %08x", bx, br);
  if (is_nan_f(bx) && br != 0x3f800000u) h_die3("sinc(NaN) must be 1.0, got %08x", br);
  if ((bx & 0x7fffffffu) == 0 && br != 0x3f800000u) h_die3("sinc(+-0) must be 1.0, got %08x", br);
  if (model) {
    Q xq = (Q)x, y = xq * QPI;
    /* near an integer the true value is a tiny difference that the instruction computes with its 66-bit pi: not modelled */
    double fr = (double)x - (double)(long long)(x + (x < 0 ? -0.5f : 0.5f));
    if (fr < 0) fr = -fr;
    if (fr > 2e-3) { h_cov_hit(C1_MODEL); chk_q("sinc", 0, r, q_sin(y) / y, 1, 1e-30); }
  }
}
static void sc_sinc(void) {
  unsigned long ctr = 0;
  int k;
  unsigned i;
  h_cov_define(C1_ZERO, "+0"); h_cov_define(C1_NEGZERO, "-0"); h_cov_define(C1_NAN, "NaN"); h_cov_define(C1_INF, "inf"); h_cov_define(C1_DENORM, "denormal");
  h_cov_define(C1_INT, "integer arguments"); h_cov_define(C1_HALFINT, "half-integer arguments"); h_cov_define(C1_GRID, "grid k/16"); h_cov_define(C1_RANDOM, "random");
  h_cov_define(C1_HUGE, "huge |x| (> 1e9)"); h_cov_define(C1_BEYOND63, "pi x >= 2^63 (fsin leaves the operand unchanged)"); h_cov_define(C1_MODEL, "model-checked");
  h_cov_define(C1_NEG, "negative");
  scen_begin();
  sinc_one(0.0f, &ctr, 1); h_cov_hit(C1_ZERO);
  sinc_one(f_from(0x80000000u), &ctr, 1); h_cov_hit(C1_NEGZERO);
  sinc_one(f_from(0x7fc00000u), &ctr, 1); sinc_one(f_from(0xffc01234u), &ctr, 1); sinc_one(f_from(0x7fc12345u), &ctr, 1); h_cov_hit(C1_NAN);
  sinc_one(f_from(0x7f800000u), &ctr, 1); sinc_one(f_from(0xff800000u), &ctr, 1); h_cov_hit(C1_INF);
  for (i = 1; i < 40; i++) { sinc_one(f_from(i * 3 + 1u), &ctr, 1); sinc_one(f_from(0x80000000u | (i * 3 + 1u)), &ctr, 1); }
  sinc_one(f_from(0x00800000u), &ctr, 1); sinc_one(f_from(0x007fffffu), &ctr, 1); h_cov_hit(C1_DENORM);
  for (k = -2000; k <= 2000; k++) { sinc_one((float)k / 16.0f, &ctr, 1); if (k < 0) h_cov_hit(C1_NEG); }
  h_cov_hit(C1_GRID);
  for (k = -3000; k <= 3000; k++) { sinc_one((float)k, &ctr, 1); }
  h_cov_hit(C1_INT);
  for (k = -3000; k <= 3000; k++) { sinc_one((float)k + 0.5f, &ctr, 1); }
  h_cov_hit(C1_HALFINT);
  for (i = 0; i < 400000; i++) {
    float x;
    if (i % 5 == 0) x = rfloat(R_EXP); else if (i % 5 == 1) x = rfloat(R_UNIT) * 64.0f; else if (i % 5 == 2) x = rfloat(R_INT) * 0.25f; else if (i % 5 == 3) x = rfloat(R_UNIT);
    else x = (float)(int)h_rrange(-200000, 200000) / 1024.0f;
    sinc_one(x, &ctr, (i & 15) == 0);
  }
  h_cov_hit(C1_RANDOM);
  for (i = 0; i < 2000; i++) { sinc_one(f_from(0x4e000000u + h_rmod(0x1000000u)), &ctr, 0); sinc_one(f_from(0xce000000u + h_rmod(0x1000000u)), &ctr, 0); } /* 5e8 .. */
  h_cov_hit(C1_HUGE);
  { static const uint32_t big[] = { 0x5f000000u /* 2^63 */, 0x5f000001u, 0x5e800000u, 0x5ea7c5acu /* 2^63/pi-ish: 5.2e18 -> 2.9e18 */, 0x5ea7c5ad, 0x5ea7c5ab, 0x7f7fffffu, 0x7e000000u, 0x60000000u };
    unsigned j;
    for (j = 0; j < sizeof big / sizeof big[0]; j++) { sinc_one(f_from(big[j]), &ctr, 0); sinc_one(f_from(big[j] | 0x80000000u), &ctr, 0); }
    for (i = 0; i < 2000; i++) sinc_one(f_from(0x5ea00000u + h_rmod(0x80000u)), &ctr, 0);
    h_cov_hit(C1_BEYOND63); }
  h_emit(1, "sinc done");
}

/* ================================================================================================ scenarios 2 / 3: statistics */
enum { C2_TRIAL = 0, C2_N0, C2_N1, C2_REG_UNIT, C2_REG_EXP, C2_REG_INT, C2_REG_HUGE, C2_REG_TINY, C2_REG_ZERO, C2_REG_CONST, C2_NONFIN, C2_NEGVAR, C2_DIFFROUND, C2_BIG, C2_MODEL };
static XRET stat_call(int which, float *p, unsigned n) {
  switch (which) { case 0: return f_sum(p, n); case 1: return f_mean(p, n); case 2: return f_sqrsum(p, n); case 3: return f_var(p, n); default: return f_std(p, n); }
}
/* independent extended-precision model written from the disassembly facts */
static LD stat_model(int which, const float *p, unsigned n) {
  LD s = 0, s2 = 0, nn = (LD)n;
  unsigned i;
  float sq;
  for (i = 0; i < n; i++) s += (LD)p[i];
  for (i = 0; i < n; i++) s2 += (LD)p[i] * (LD)p[i];
  s2 = s2 / nn;
  if (which == 0) return s;
  s = s / nn;
  if (which == 1) return s;
  if (which == 2) return s2;
  sq = (float)s2;
  { LD v = (LD)sq - s * s; return which == 3 ? v : sqrtl(v); }
}
static void put_ld(char *out, LD v) { unsigned char b[16]; int i; memset(b, 0, sizeof b); memcpy(b, &v, 10); for (i = 0; i < 10; i++) sprintf(out + 2 * i, "%02x", b[9 - i]); }
static void sc_stats(int ext) {
  unsigned long ctr = 0;
  unsigned trial;
  h_cov_define(C2_TRIAL, "trials"); h_cov_define(C2_N0, "n == 0"); h_cov_define(C2_N1, "n == 1"); h_cov_define(C2_REG_UNIT, "unit data"); h_cov_define(C2_REG_EXP, "random exponents");
  h_cov_define(C2_REG_INT, "small integers"); h_cov_define(C2_REG_HUGE, "huge data (overflow of the square)"); h_cov_define(C2_REG_TINY, "denormal / tiny"); h_cov_define(C2_REG_ZERO, "zeros");
  h_cov_define(C2_REG_CONST, "constant data (variance from rounding only)"); h_cov_define(C2_NONFIN, "non-finite result"); h_cov_define(C2_NEGVAR, "negative variance (Std NaN)");
  h_cov_define(C2_DIFFROUND, "extended result differs from its float rounding"); h_cov_define(C2_BIG, "n >= 1000"); h_cov_define(C2_MODEL, "model-checked results");
  if (!ext) h_covreq[C2_DIFFROUND] = 0; else h_covreq[C2_NEGVAR] = 0;
  scen_begin();
  for (trial = 0; trial < 3000; trial++) {
    unsigned n = h_rpct(5) ? (unsigned)h_rmod(3) : h_rpct(10) ? 1000u + h_rmod(3000) : (unsigned)h_rrange(1, 200), i;
    int reg = (int)h_rmod(7), w;
    float *p = alloc_f(n + 1), c = 0;
    if (reg == R_SPECIAL) reg = h_rpct(30) ? R_SPECIAL : R_UNIT;
    if (reg == R_HUGE && h_rpct(60)) reg = R_UNIT;
    if (h_rpct(10)) { c = rfloat(R_UNIT); reg = 100; }
    for (i = 0; i < n + 1; i++) { if (reg == 100) p[i] = c; else rfill(&p[i], reg); }
    h_cov_hit(C2_TRIAL);
    if (n == 0) h_cov_hit(C2_N0);
    if (n == 1) h_cov_hit(C2_N1);
    if (n >= 1000) h_cov_hit(C2_BIG);
    h_cov_hit(reg == R_UNIT ? C2_REG_UNIT : reg == R_EXP ? C2_REG_EXP : reg == R_INT ? C2_REG_INT : reg == R_HUGE ? C2_REG_HUGE : reg == R_TINY ? C2_REG_TINY : reg == R_ZERO ? C2_REG_ZERO : reg == 100 ? C2_REG_CONST : C2_NONFIN);
    for (w = 0; w < 5; w++) {
      XRET r = stat_call(w, p, n);
      LD m = stat_model(w, p, n);
      float rf = (float)r;
      uint32_t b = bits_f(rf);
      if (ext) {
        char s[24], sm[24];
        put_ld(s, (LD)r);
        put_ld(sm, m);
        h_emit(h_sample(&ctr, 40, 211), "x%d n=%u %s", w, n, s);
        if (strcmp(s, sm) != 0 && !(is_nan_f(bits_f((float)m)) && (m != m) && (LD)r != (LD)r))
          h_die3("stat %d n=%u: extended result %s, model %s", w, n, s, sm);
        h_cov_hit(C2_MODEL);
        if ((LD)(float)(LD)r != (LD)r && (LD)r == (LD)r) h_cov_hit(C2_DIFFROUND);
      } else {
        h_emit(h_sample(&ctr, 40, 211), "s%d n=%u %08x", w, n, b);
        { uint32_t bm = bits_f((float)m);
          if (bm != b && !(is_nan_f(bm) && is_nan_f(b))) h_die3("stat %d n=%u: float-rounded result %08x, model %08x", w, n, b, bm);
          h_cov_hit(C2_MODEL); }
        if ((b & 0x7f800000u) == 0x7f800000u) h_cov_hit(C2_NONFIN);
        if (w == 3 && (LD)stat_model(3, p, n) < 0) h_cov_hit(C2_NEGVAR);
      }
    }
    sysdep_free(p);
    h_trial_end("stats");
  }
  h_emit(1, "stats done");
}

/* ================================================================================================ scenarios 4 / 5: FFT */
enum { C4_TRIAL = 0, C4_POW2, C4_NONPOW2, C4_FWD, C4_INV, C4_OTHER_SIGN, C4_N1, C4_N2, C4_BIG, C4_UNIT, C4_IMPULSE, C4_CONST, C4_INT, C4_SPECIAL, C4_EXP, C4_MODEL, C4_ROUNDTRIP, C4_WRAPCHECK };
static void dft_q(const float *x, unsigned n, int sign, Q *re, Q *im) {
  /* naive DFT X_k = sum_j x_j exp(sign * 2 pi i jk / n) of an n-point complex sequence x[2j], x[2j+1] */
  static Q cs[8192], sn[8192];
  unsigned j, k;
  for (j = 0; j < n; j++) { cs[j] = q_cos(2 * QPI * (Q)j / (Q)n); sn[j] = q_sin(2 * QPI * (Q)j / (Q)n); }
  for (k = 0; k < n; k++) {
    Q ar = 0, ai = 0;
    for (j = 0; j < n; j++) {
      unsigned t = (unsigned)(((unsigned long long)j * k) % n);
      Q c = cs[t], s = sign > 0 ? sn[t] : -sn[t];
      Q xr = (Q)x[2 * j], xi = (Q)x[2 * j + 1];
      ar += xr * c - xi * s;
      ai += xr * s + xi * c;
    }
    re[k] = ar; im[k] = ai;
  }
}
static void fill_fft(float *d, unsigned cnt, int kind, int *cls) { /* d[1 .. cnt] */
  unsigned i;
  *cls = kind;
  for (i = 1; i <= cnt; i++) {
    switch (kind) {
      case 0: rfill(&d[i], R_UNIT); break;
      case 1: d[i] = f_from(i == 1 ? 0x3f800000u : 0u); break;                       /* impulse */
      case 2: d[i] = f_from(0x3f000000u); break;                                      /* constant */
      case 3: rfill(&d[i], R_INT); break;                                             /* small integers: exact transforms of power-of-two sizes up to rounding of the twiddles */
      case 4: rfill(&d[i], R_QNAN1); break;   /* positive quiet NaNs with payload among unit data; inf data is excluded: the NaN that an inf - inf produces first (default NaN) depends on the grouping of the sums */
      default: rfill(&d[i], R_EXP); break;
    }
  }
}
static void sc_fft(int real, int nan) {
  unsigned trial;
  h_cov_define(C4_TRIAL, "trials"); h_cov_define(C4_POW2, "power-of-two size"); h_cov_define(C4_NONPOW2, "non-power-of-two size (the memory-safe ones: four1 nn = 2^k - 1, realfft n >> 1 = 2^k - 1)");
  h_cov_define(C4_FWD, "isign == 1"); h_cov_define(C4_INV, "isign == -1"); h_cov_define(C4_OTHER_SIGN, "isign not +-1"); h_cov_define(C4_N1, "n == 1 / 2");
  h_cov_define(C4_N2, "n == 4 / 8"); h_cov_define(C4_BIG, "n >= 1024"); h_cov_define(C4_UNIT, "unit data"); h_cov_define(C4_IMPULSE, "impulse"); h_cov_define(C4_CONST, "constant");
  h_cov_define(C4_INT, "small integers"); h_cov_define(C4_SPECIAL, "quiet-NaN data"); h_cov_define(C4_EXP, "random exponents"); h_cov_define(C4_MODEL, "model-checked transforms");
  h_cov_define(C4_ROUNDTRIP, "round trips checked"); h_cov_define(C4_WRAPCHECK, "bytes outside the used window untouched");
  scen_begin();
  if (nan) { h_covreq[C4_UNIT] = h_covreq[C4_IMPULSE] = h_covreq[C4_CONST] = h_covreq[C4_INT] = h_covreq[C4_EXP] = h_covreq[C4_MODEL] = h_covreq[C4_ROUNDTRIP] = 0; }
  else h_covreq[C4_SPECIAL] = 0;
  for (trial = 0; trial < (nan ? 250u : real ? 1100u : 900u); trial++) {
    static const unsigned pows[] = { 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192 };
    unsigned n, cnt, i;
    int isign, kind, cls, model_ok, nonpow;
    uint32_t pad;
    float *d, *d0;
    Q mxin;
    if (h_rpct(10)) n = pows[h_rmod(13)];
    else n = pows[h_rmod(9)];
    if (h_rpct(2)) n = 16384u << h_rmod(3);
    if (!nan && h_rpct(1)) n = real ? 262144u : 131072u;                       /* realfft: n >> 2 = 65536 does not fit a 16-bit loop index; four1 nn = 2^17 */                                   /* 16 bit index variables are wrong from 2^15 elements on (n up to 65536: 131072 floats) */
    if (h_rpct(10)) n = real ? (1u << h_rrange(3, 11)) - 1u - h_rmod(2) : (1u << h_rrange(2, 10)) - 1u;  /* measured memory-safe non-powers of two: four1 nn = 2^k - 1, realfft n >> 1 = 2^k - 1 */
    cnt = real ? n : 2 * n;                                               /* elements used: data[1 .. cnt] */
    isign = h_rpct(45) ? 1 : h_rpct(80) ? -1 : (int)h_rmod(5) - 2 + (h_rpct(10) ? 1000 : 0);
    if (isign == 0 && h_rpct(50)) isign = 1;
    kind = h_rpct(60) ? 0 : (int)h_rmod(6);
    if (nan) kind = 4; else if (kind == 4) kind = 0;   /* NaN data only in the x87-exact scenarios 11 / 12 */
    d = alloc_f(cnt + 3);
    d0 = alloc_f(cnt + 3);
    nonpow = (n & (n - 1)) != 0;
    pad = nonpow ? 0x3e800000u : 0x7fe5a5a5u;   /* sentinel: a quiet NaN (a read outside the window poisons the result); the non-power-of-two sizes read AND write the 1-2 floats after the window (measured with finite padding: with a NaN pad the NaN is written back unchanged and nothing looks modified): finite padding, no sentinel check, the whole block is compared blob vs port */
    for (i = 0; i < cnt + 3; i++) { d[i] = f_from(pad); }
    fill_fft(d, cnt, kind, &cls);
    for (i = 0; i < cnt + 3; i++) d0[i] = d[i];
    h_cov_hit(C4_TRIAL);
    if (n && !(n & (n - 1))) h_cov_hit(C4_POW2); else h_cov_hit(C4_NONPOW2);
    if (isign == 1) h_cov_hit(C4_FWD); else if (isign == -1) h_cov_hit(C4_INV); else h_cov_hit(C4_OTHER_SIGN);
    if (n <= 2) h_cov_hit(C4_N1);
    if (n == 4 || n == 8) h_cov_hit(C4_N2);
    if (n >= 1024) h_cov_hit(C4_BIG);
    h_cov_hit(kind == 0 ? C4_UNIT : kind == 1 ? C4_IMPULSE : kind == 2 ? C4_CONST : kind == 3 ? C4_INT : kind == 4 ? C4_SPECIAL : C4_EXP);
    if (real) f_realfft(d, n, isign); else f_four1(d, n, isign);
    EMIT_BLK(real ? "realfft" : "four1", d, cnt + 3, n, isign + 100000 * kind);
    if (!nonpow && (bits_f(d[0]) != pad || bits_f(d[cnt + 1]) != pad || bits_f(d[cnt + 2]) != pad)) h_die3("%s touched data[0] or beyond the used window (n %u isign %d: d[0] %08x d[cnt+1] %08x d[cnt+2] %08x, pad %08x)", real ? "realfft" : "four1", n, isign, bits_f(d[0]), bits_f(d[cnt + 1]), bits_f(d[cnt + 2]), pad);
    h_cov_hit(C4_WRAPCHECK);
    model_ok = (n >= 2 && n <= 256 && !(n & (n - 1)) && (isign == 1 || isign == -1) && kind != 4);
    mxin = 0;
    for (i = 1; i <= cnt; i++) { Q a = (Q)d0[i]; if (a < 0) a = -a; if (a > mxin) mxin = a; }
    if (model_ok && !real) {
      Q re[256], im[256];
      float in[2 * 256];
      Q mx = 0;
      for (i = 0; i < 2 * n; i++) in[i] = d0[i + 1];
      dft_q(in, n, isign, re, im);
      for (i = 0; i < n; i++) { Q a = re[i] < 0 ? -re[i] : re[i], b = im[i] < 0 ? -im[i] : im[i]; if (a > mx) mx = a; if (b > mx) mx = b; }
      for (i = 0; i < n; i++) {
        Q e1 = (Q)d[2 * i + 1] - re[i], e2 = (Q)d[2 * i + 2] - im[i];
        if (e1 < 0) e1 = -e1; if (e2 < 0) e2 = -e2;
        if (e1 > mx * (Q)2e-5 + (Q)1e-30 || e2 > mx * (Q)2e-5 + (Q)1e-30) h_die3("four1 n %u isign %d bin %u: (%.9g, %.9g), model (%.9g, %.9g)", n, isign, i, (double)d[2 * i + 1], (double)d[2 * i + 2], (double)re[i], (double)im[i]);
      }
      h_cov_hit(C4_MODEL);
      if (isign == 1) { /* round trip: four1(-1) restores n * x */
        float *e = alloc_f(cnt + 3);
        Q m2 = 0;
        for (i = 0; i < cnt + 3; i++) e[i] = d[i];
        f_four1(e, n, -1);
        for (i = 0; i < 2 * n; i++) { Q a = (Q)d0[i + 1] * (Q)n; Q df = (Q)e[i + 1] - a; if (df < 0) df = -df; if (a < 0) a = -a; if (a > m2) m2 = a; if (df > (Q)1e-4 * (Q)n * mxin + (Q)1e-30) h_die3("four1 round trip n %u elem %u: %.9g vs %.9g", n, i, (double)e[i + 1], (double)((Q)d0[i + 1] * (Q)n)); }
        h_cov_hit(C4_ROUNDTRIP);
        sysdep_free(e);
      }
    }
    if (model_ok && real) {
      Q re[256], im[256];
      float in[2 * 256];
      Q mx = 0;
      if (isign == 1) {
        for (i = 0; i < n; i++) { in[2 * i] = d0[i + 1]; in[2 * i + 1] = 0.0f; }
        dft_q(in, n, 1, re, im);
        for (i = 0; i < n; i++) { Q a = re[i] < 0 ? -re[i] : re[i]; if (a > mx) mx = a; }
        for (i = 0; i <= n / 2; i++) {
          Q e1, e2;
          if (i == 0) { e1 = (Q)d[1] - re[0]; e2 = 0; }
          else if (i == n / 2) { e1 = (Q)d[2] - re[n / 2]; e2 = 0; }
          else { e1 = (Q)d[2 * i + 1] - re[i]; e2 = (Q)d[2 * i + 2] - im[i]; }
          if (e1 < 0) e1 = -e1; if (e2 < 0) e2 = -e2;
          if (e1 > mx * (Q)2e-5 + (Q)1e-30 || e2 > mx * (Q)2e-5 + (Q)1e-30) h_die3("realfft n %u bin %u: model mismatch (%.9g, %.9g) vs (%.9g, %.9g)", n, i, (double)d[2 * i + 1], (double)d[2 * i + 2], (double)re[i], (double)im[i]);
        }
        h_cov_hit(C4_MODEL);
        { float *e = alloc_f(cnt + 3);   /* realfft(-1) after realfft(1) restores (n / 2) x */
          for (i = 0; i < cnt + 3; i++) e[i] = d[i];
          f_realfft(e, n, -1);
          for (i = 1; i <= n; i++) { Q a = (Q)d0[i] * (Q)n / 2, df = (Q)e[i] - a; if (df < 0) df = -df; if (df > (Q)1e-4 * (Q)n * mxin + (Q)1e-30) h_die3("realfft round trip n %u elem %u: %.9g vs %.9g", n, i, (double)e[i], (double)a); }
          h_cov_hit(C4_ROUNDTRIP);
          sysdep_free(e); }
      }
    }
    sysdep_free(d0);
    sysdep_free(d);
    h_trial_end("fft");
  }
  h_emit(1, "fft done");
}

/* ================================================================================================ scenario 6: Psd */
enum { C6_TRIAL = 0, C6_C2, C6_D2, C6_OPT0, C6_OPT1, C6_OPT2, C6_OPT_OTHER, C6_OVERLAP0, C6_OVERLAP, C6_NSEG0, C6_NSEG1, C6_NSEGMANY, C6_ODD, C6_BIG, C6_SETWT, C6_SETOV, C6_FREQ,
       C6_WT0, C6_WT1, C6_WT2, C6_WT3, C6_ALLZERO, C6_MODEL, C6_NONFIN, C6_DTOR_NULL, C6_DC };
static void psd_model(const PsdObj *o, const float *in, unsigned n, unsigned nseg, Q *acc, unsigned nb) {
  /* exact model of out[k] = sum over segments of |FFT(window * segment)|^2 (the blob's packing: bin 0 = DC + i * Nyquist) */
  unsigned N = o->size_, seg, k, j, stride = N - o->overlap_;
  const float *win = (const float *)o->window_;
  (void)n;
  for (k = 0; k < nb; k++) acc[k] = 0;
  for (seg = 0; seg < nseg; seg++) {
    float x[2 * 256];
    Q re[256], im[256];
    for (j = 0; j < N; j++) { x[2 * j] = (float)((LD)in[seg * stride + j] * (LD)win[j]); x[2 * j + 1] = 0.0f; }
    dft_q(x, N, 1, re, im);
    for (k = 0; k < nb; k++) {
      Q a, b;
      if (k == 0) { a = re[0]; b = re[N / 2]; }
      else { a = re[k]; b = im[k]; }
      acc[k] += a * a + b * b;
    }
  }
}
static void sc_psd(int nan) {
  unsigned long ctr = 0;
  unsigned trial;
  h_cov_define(C6_TRIAL, "trials"); h_cov_define(C6_C2, "C2 constructor"); h_cov_define(C6_D2, "D2 destructor"); h_cov_define(C6_OPT0, "option 0 (dB / nseg)");
  h_cov_define(C6_OPT1, "option 1 (dB / max)"); h_cov_define(C6_OPT2, "option 2 (linear / nseg)"); h_cov_define(C6_OPT_OTHER, "option other (raw sums)"); h_cov_define(C6_OVERLAP0, "overlap 0");
  h_cov_define(C6_OVERLAP, "overlap > 0"); h_cov_define(C6_NSEG0, "no complete segment"); h_cov_define(C6_NSEG1, "one segment"); h_cov_define(C6_NSEGMANY, ">= 8 segments"); h_cov_define(C6_ODD, "odd N");
  h_cov_define(C6_BIG, "N >= 512"); h_cov_define(C6_SETWT, "setWindowType"); h_cov_define(C6_SETOV, "setOverlapLength"); h_cov_define(C6_FREQ, "getFrequencies");
  h_cov_define(C6_WT0, "boxcar"); h_cov_define(C6_WT1, "hanning"); h_cov_define(C6_WT2, "hamming"); h_cov_define(C6_WT3, "blackman"); h_cov_define(C6_ALLZERO, "all-zero input (max == 0)");
  h_cov_define(C6_MODEL, "model-checked bins"); h_cov_define(C6_NONFIN, "non-finite input"); h_cov_define(C6_DTOR_NULL, "dtor with a NULL block pointer"); h_cov_define(C6_DC, "constant / alternating data with a boxcar window (exactly zero bins next to a non-zero maximum)");
  scen_begin();
  if (!nan) h_covreq[C6_NONFIN] = 0; else h_covreq[C6_ALLZERO] = h_covreq[C6_MODEL] = h_covreq[C6_DC] = 0;
  for (trial = 0; trial < (nan ? 120u : 400u); trial++) {
    unsigned N, ov, n, nseg, i, nb, nops, op;
    int wt, c2 = h_rpct(30), d2 = h_rpct(30), reg, dcmode;
    PsdObj *o = (PsdObj *)h_obj_new(sizeof(PsdObj));
    unsigned long nm0 = h_nmalloc;
    static const unsigned Ns[] = { 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048 };
    N = h_rpct(25) ? 1u << h_rrange(1, 5) : Ns[h_rmod(h_rpct(15) ? 11 : 8)];
    if (h_rpct(8)) N = (1u << h_rrange(1, 6)) + 1u; /* N / 2 must be a power of two: the NR four1 / realfft index outside buf_ otherwise (measured: excluded, UB in the blob) */
    ov = h_rpct(40) ? 0u : h_rpct(60) ? N / 2 : h_rmod(N);
    if (ov >= N) ov = N - 1;
    wt = (int)h_rmod(h_rpct(10) ? 6 : 4);
    reg = h_rpct(70) ? R_UNIT : (int)h_rmod(7);
    if (reg == R_SPECIAL || nan) reg = nan ? R_QNAN1 : R_UNIT;  /* NaN data only in scenario 13 (x87-exact: the NaN that wins an addition depends on the order of the operands) */
    nseg = h_rpct(10) ? 0u : h_rpct(25) ? 1u : h_rpct(25) ? 8u + h_rmod(6) : 1u + h_rmod(5);
    n = ov + (N - ov) * nseg + (h_rpct(40) ? h_rmod(N - ov) : 0u);
    if (nseg == 0) n = h_rmod(N - ov + ov); /* n < ov + stride: no complete segment; n >= ov (n - ov must not underflow) */
    if (n < ov) n = ov;
    nb = N >> 1;
    dcmode = (!nan && h_rpct(10)) ? 1 + (int)h_rmod(2) : 0;
    if (dcmode) { wt = 0; reg = R_UNIT; }
    H_CTOR_PROBE(o, f_PsdObj, NF_PsdObj, sizeof(PsdObj), (c2 ? (c2 = 1, h_cov_hit(C6_C2), psd_c2(o, N, wt, ov)) : psd_c1(o, N, wt, ov)));
    h_cov_hit(C6_TRIAL);
    if (h_nmalloc != nm0 + 2) h_die3("Psd ctor must allocate two blocks");
    if (o->size_ != N || o->overlap_ != ov) h_die3("Psd ctor fields");
    h_cov_hit(ov == 0 ? C6_OVERLAP0 : C6_OVERLAP);
    if (N & 1) h_cov_hit(C6_ODD);
    if (N >= 512) h_cov_hit(C6_BIG);
    h_cov_hit(wt == 0 ? C6_WT0 : wt == 1 ? C6_WT1 : wt == 2 ? C6_WT2 : wt == 3 ? C6_WT3 : C6_WT0);
    h_dump_obj(h_sample(&ctr, 12, 53), "Psd ctor", o, f_PsdObj, NF_PsdObj);
    nops = 2 + h_rmod(3);
    for (op = 0; op < nops; op++) {
      int act = (int)h_rmod(100);
      if (act < 12 && !dcmode) { int w2 = (int)h_rmod(5); psd_setwt(o, w2); h_cov_hit(C6_SETWT); h_emit(0, "setwt %d", w2); h_dump_obj(h_sample(&ctr, 12, 53), "Psd setwt", o, f_PsdObj, NF_PsdObj); }
      else if (act < 20) {
        unsigned nov = h_rmod(N);
        psd_setov(o, nov); h_cov_hit(C6_SETOV); ov = nov;
        if (n < ov) n = ov; /* keep n >= overlap: the blob divides (n - overlap) and the stride is unsigned */
        h_emit(0, "setov %u", nov);
      } else if (act < 32) {
        float fs = h_rpct(20) ? 0.0f : rfloat(h_rpct(50) ? R_UNIT : R_EXP) * (h_rpct(50) ? 8000.0f : 1.0f);
        float *fr = alloc_f(nb + 1);
        for (i = 0; i < nb + 1; i++) fr[i] = f_from(0x7fe5a5a5u);
        psd_freq(o, fr, fs);
        if (bits_f(fr[nb]) != 0x7fe5a5a5u) h_die3("getFrequencies wrote past N/2");
        EMIT_BLK("freq", fr, nb + 1, N, bits_f(fs));
        h_cov_hit(C6_FREQ);
        { Q fsq = (Q)fs; for (i = 0; i < nb; i++) if (fs == fs && fs < 3e38f && fs > -3e38f) chk_q("freq", i, fr[i], (Q)i * fsq / (Q)N, 1, 1e-30); }
        sysdep_free(fr);
      } else {
        int opt = (int)h_rmod(10);
        float *in, *out;
        unsigned lim = n + 2;
        opt = opt < 3 ? 0 : opt < 6 ? 1 : opt < 9 ? 2 : (h_rpct(50) ? 3 : -1 - (int)h_rmod(3));
        if (reg == R_ZERO && h_rpct(70)) opt = h_rpct(60) ? 0 : 1; /* all-zero data with the dB options: log10(0 * recip + 1e-25) = -250 dB (opt 0) / NaN from 0 * inf (opt 1) */
        if (opt == 0) h_cov_hit(C6_OPT0); else if (opt == 1) h_cov_hit(C6_OPT1); else if (opt == 2) h_cov_hit(C6_OPT2); else h_cov_hit(C6_OPT_OTHER);
        in = alloc_f(lim);
        out = alloc_f(nb + 2);
        for (i = 0; i < lim; i++) rfill(&in[i], reg);
        if (dcmode) { for (i = 0; i < lim; i++) in[i] = (dcmode == 1 || !(i & 1)) ? 0.5f : -0.5f; h_cov_hit(C6_DC); }
        if (reg == R_ZERO) h_cov_hit(C6_ALLZERO);
        if (reg == R_QNAN1) h_cov_hit(C6_NONFIN);
        for (i = 0; i < nb + 2; i++) out[i] = f_from(0xeeeeeeeeu);
        { unsigned segs = (n - o->overlap_) / (N - o->overlap_);
          psd_proc(o, in, n, out, opt);
          if (segs == 0) h_cov_hit(C6_NSEG0); else if (segs == 1) h_cov_hit(C6_NSEG1); else if (segs >= 8) h_cov_hit(C6_NSEGMANY);
          if (bits_f(out[nb]) != 0xeeeeeeeeu || bits_f(out[nb + 1]) != 0xeeeeeeeeu) h_die3("Psd::process wrote past N/2");
          EMIT_BLK("psd", out, nb, N, opt);
          h_emit(h_sample(&ctr, 40, 41), "buf %016llx", (unsigned long long)fnv_blk(o->buf_, (N + 1) * 4));
          if (opt >= 2 && reg == R_UNIT && N <= 256 && !(N & (N - 1)) && segs <= 3 && segs > 0) { /* model: linear average */
            Q acc[256];
            Q mx = 0;
            psd_model(o, in, n, segs, acc, nb);
            for (i = 0; i < nb; i++) { Q v = acc[i] / (Q)(opt == 2 ? segs : 1); if (v > mx) mx = v; }
            for (i = 0; i < nb; i++) {
              Q want = acc[i] / (Q)(opt == 2 ? segs : 1), df = (Q)out[i] - want;
              if (df < 0) df = -df;
              if (opt == 2 && df > mx * (Q)2e-5 + (Q)1e-30) h_die3("Psd linear bin %u: %.9g vs model %.9g", i, (double)out[i], (double)want);
            }
            h_cov_hit(C6_MODEL);
          }
          if ((opt == 0 || opt == 1) && reg == R_UNIT && N <= 256 && !(N & (N - 1)) && segs <= 3 && segs > 0) { /* dB options against log10l of the exact sums, loose (float32 sums) */
            Q acc[256];
            Q mx = 0;
            psd_model(o, in, n, segs, acc, nb);
            for (i = 0; i < nb; i++) if (acc[i] > mx) mx = acc[i];
            for (i = 0; i < nb; i++) {
              Q v = opt == 0 ? acc[i] / (Q)segs : acc[i] / mx;
              double want = 10.0 * (double)log10l((LD)(v + (Q)1e-25)), df = (double)out[i] - want;
              if (df < 0) df = -df;
              if (df > 2e-3 + 1e-6 * (want < 0 ? -want : want) && acc[i] > mx * (Q)1e-12) h_die3("Psd dB bin %u: %.9g, model %.9g (opt %d)", i, (double)out[i], want, opt);
            }
            h_cov_hit(C6_MODEL);
          }
        }
        sysdep_free(out);
        sysdep_free(in);
      }
      h_dump_obj(h_sample(&ctr, 12, 53), "Psd", o, f_PsdObj, NF_PsdObj);
      h_check_not_wild("Psd");
    }
    { unsigned long nf0 = h_nfree;
      unsigned nulled = 0;
      void *p;
      if (h_rpct(8)) { memcpy(&p, (uint8_t *)o + offsetof(PsdObj, window_), sizeof p); sysdep_free(p); p = NULL; memcpy((uint8_t *)o + offsetof(PsdObj, window_), &p, sizeof p); nulled++; h_cov_hit(C6_DTOR_NULL); h_cov_hit(H_COV_DTOR_NULL); }
      if (h_rpct(8)) { memcpy(&p, (uint8_t *)o + offsetof(PsdObj, buf_), sizeof p); sysdep_free(p); p = NULL; memcpy((uint8_t *)o + offsetof(PsdObj, buf_), &p, sizeof p); nulled++; h_cov_hit(C6_DTOR_NULL); h_cov_hit(H_COV_DTOR_NULL); }
      nf0 = h_nfree;
      if (d2) { h_cov_hit(C6_D2); psd_d2(o); } else psd_d1(o);
      if (h_nfree != nf0 + 2 - nulled) h_die3("Psd dtor must free the two blocks that are not NULL");
    }
    h_trial_end("Psd");
  }
  h_emit(1, "psd done");
}

/* ================================================================================================ scenario 7: LowPassFIR */
enum { C7_TRIAL = 0, C7_CTOR, C7_DESIGN4, C7_DESIGN5_NULL, C7_DESIGN5_COPY, C7_DESIGN5_ADOPT, C7_FC0, C7_FC1, C7_FC_HALF, C7_FC_BAD, C7_FC_NAN, C7_N_BAD, C7_REDESIGN, C7_GAIN_NEG, C7_GAIN0,
       C7_LEAK, C7_MODEL, C7_BIG, C7_INTEGER_X, C7_WT0, C7_WT1, C7_WT2, C7_WT3, C7_WT_OTHER, C7_DTOR_NULL, C7_NEG0, C7_MALLOC_FAIL };
static void lpf_check_model(const LpfObj *o, unsigned n, float fc, float gain, int wt, const float *win, int have_win) {
  /* exact math in float128: coefficients c_i = fc sinc(fc (i - (n-1)/2)) w_i scaled to sum == gain; loose tolerance: the blob rounds the sinc argument to float */
  Q c[2048], sum = 0, mx = 0;
  unsigned i;
  const float *g = (const float *)o->coef_;
  if (n > 2048) return;
  LD xrec = (-(LD)fc * (LD)(n - 1)) * (LD)0.5f;   /* the vendor's argument recurrence: x(i+1) = (float)x(i) + fc (error accumulates; part of the specification) */
  for (i = 0; i < n; i++) {
    float xf = (float)xrec;
    Q x = (Q)xf, s, w;
    xrec = (LD)xf + (LD)fc;
    s = x == 0 ? (Q)1 : q_sin(QPI * x) / (QPI * x);
    s = (Q)(float)s;       /* sinc returns a float */
    w = have_win ? (Q)win[i] : (wt >= 0 ? win_model(wt == 0 ? 0 : wt, i, n) : win_model(2, i, n));
    c[i] = (Q)(float)((Q)fc * s * w);   /* the blob stores the float coefficient before summing them */
    sum += c[i];
  }
  for (i = 0; i < n; i++) { c[i] = c[i] * (Q)gain / sum; Q a = c[i] < 0 ? -c[i] : c[i]; if (a > mx) mx = a; }
  for (i = 0; i < n; i++) { Q df = (Q)g[i] - c[i]; if (df < 0) df = -df; if (df > mx * (Q)2e-5 + (Q)1e-30) h_die3("LowPassFIR coef %u: %.9g, model %.9g (n %u fc %.9g gain %.9g wt %d have_win %d)", i, (double)g[i], (double)c[i], n, (double)fc, (double)gain, wt, have_win); }
  h_cov_hit(C7_MODEL);
}
static void sc_lpf(void) {
  unsigned long ctr = 0;
  unsigned trial;
  h_cov_define(C7_TRIAL, "trials"); h_cov_define(C7_CTOR, "constructor"); h_cov_define(C7_DESIGN4, "design(n, fc, wt, gain)"); h_cov_define(C7_DESIGN5_NULL, "design(..., NULL window): hamming");
  h_cov_define(C7_DESIGN5_COPY, "design(..., window, adopt 0)"); h_cov_define(C7_DESIGN5_ADOPT, "design(..., window, adopt 1)"); h_cov_define(C7_FC0, "fc == 0"); h_cov_define(C7_FC1, "fc == 1");
  h_cov_define(C7_FC_HALF, "fc == 0.5 (integer sinc arguments)"); h_cov_define(C7_FC_BAD, "fc < 0 or > 1: rejected"); h_cov_define(C7_FC_NAN, "fc NaN: rejected"); h_cov_define(C7_N_BAD, "n <= 1: rejected");
  h_cov_define(C7_REDESIGN, "design on an object that already owns a block"); h_cov_define(C7_GAIN_NEG, "negative gain"); h_cov_define(C7_GAIN0, "gain 0"); h_cov_define(C7_LEAK, "rejected design(n, fc, wt, gain) leaks its window block");
  h_cov_define(C7_MODEL, "model-checked designs"); h_cov_define(C7_BIG, "n >= 200"); h_cov_define(C7_INTEGER_X, "sinc arguments that are exact integers"); h_cov_define(C7_WT0, "boxcar");
  h_cov_define(C7_WT1, "hanning"); h_cov_define(C7_WT2, "hamming"); h_cov_define(C7_WT3, "blackman"); h_cov_define(C7_WT_OTHER, "other window type"); h_cov_define(C7_DTOR_NULL, "dtor with a NULL block pointer");
  h_cov_define(C7_NEG0, "fc == -0"); h_cov_define(C7_MALLOC_FAIL, "design(4): the window allocation fails (returns 1, nothing changed)");
  scen_begin();
  for (trial = 0; trial < 700; trial++) {
    LpfObj *o = (LpfObj *)h_obj_new(sizeof(LpfObj));
    unsigned nops = 1 + h_rmod(4), op;
    int first = 1;
    h_cov_hit(C7_TRIAL);
    for (op = 0; op < nops; op++) {
      unsigned n = h_rpct(8) ? h_rmod(3) : h_rpct(12) ? 150u + h_rmod(120) : (unsigned)h_rrange(2, 80);
      static const float fcs[] = { 0.0f, 1.0f, 0.5f, 0.25f, 0.125f, 0.75f, 0.0625f, 0.3f, 0.9f, 0.05f };
      float fc, gain;
      int wt = (int)h_rmod(h_rpct(12) ? 7 : 4), how = (int)h_rmod(10), rc, redesign = !first && !h_rpct(10), mfail = 0;
      float *win = NULL;
      unsigned long nm0 = h_nmalloc;
      uint32_t fb;
      if (h_rpct(50)) fc = fcs[h_rmod(10)]; else if (h_rpct(70)) fc = (float)(int)h_rmod(1000) / 1000.0f; else fc = rfloat(R_UNIT);
      if (h_rpct(4)) fc = f_from(0x80000000u);
      if (h_rpct(3)) fc = f_from(0x7fc00000u);
      if (h_rpct(4)) fc = (float)(int)h_rmod(40) / 16.0f - 0.5f;
      if (h_rpct(3)) fc = f_from(0xbf800000u);
      gain = h_rpct(50) ? 1.0f : h_rpct(60) ? rfloat(R_UNIT) * 4.0f : (h_rpct(30) ? 0.0f : 0.125f * (float)(int)h_rmod(40));
      fb = bits_f(fc);
      if (n >= 200) h_cov_hit(C7_BIG);
      if (first || how < 2) {
        if (first && !h_rpct(50)) {
          rc = 0; /* constructor path: rc is set below from the arguments */
          mfail = h_rpct(5);
          h_fail_next_malloc = mfail;
          H_CTOR_PROBE(o, f_LpfObj, NF_LpfObj, sizeof(LpfObj), lpf_c1(o, n, fc, wt, gain));
          h_fail_next_malloc = 0;
          h_cov_hit(C7_CTOR); h_cov_hit(C7_DESIGN4);
          { int ok = (fc == fc) && fc >= 0 && fc <= 1 && n > 1 && !mfail;
            rc = ok ? 0 : 1;
            if (mfail) h_cov_hit(C7_MALLOC_FAIL);
            if (!ok) { /* the blob leaves n_ uninitialised and the window block leaked: canonical value (object pool fill) and release the leaked block */
              o->n_ = 0; if (h_nmalloc == nm0 + 1) sysdep_free(h_al[h_last_alloc].p); h_cov_hit(C7_LEAK);
              if (n == 0) { /* malloc(0) */ } }
            else lpf_check_model(o, n, fc, gain, wt, NULL, 0); }
        } else {
          if (first) { memset((uint8_t *)o, 0, sizeof(LpfObj)); }
          mfail = h_rpct(6);
          h_fail_next_malloc = mfail;
          rc = lpf_d4(o, n, fc, wt, gain);
          h_fail_next_malloc = 0;
          h_cov_hit(C7_DESIGN4);
          if (mfail) h_cov_hit(C7_MALLOC_FAIL);
          if (rc != ((fc == fc) && fc >= 0 && fc <= 1 && n > 1 && !mfail ? 0 : 1)) h_die3("design4 return value %d (n %u fc %08x)", rc, n, fb);
          if (rc == 1 && h_nmalloc == nm0 + 1) { sysdep_free(h_al[h_last_alloc].p); h_cov_hit(C7_LEAK); }
          if (rc == 0 && n <= 2048) lpf_check_model(o, n, fc, gain, wt, NULL, 0);
        }
        h_cov_hit(wt == 0 ? C7_WT0 : wt == 1 ? C7_WT1 : wt == 2 ? C7_WT2 : wt == 3 ? C7_WT3 : C7_WT_OTHER);
      } else {
        int mode = how < 5 ? 0 : how < 8 ? 1 : 2; /* NULL window / copy / adopt */
        if (mode) { unsigned i; win = alloc_f(n + 1); for (i = 0; i < n + 1; i++) { if (h_rpct(70)) win[i] = rfloat(R_UNIT); else if (h_rpct(50)) win[i] = 1.0f; else win[i] = rfloat(R_EXP); } }
        { unsigned long nf0 = h_nfree;
          int adopt = mode == 2;
          nm0 = h_nmalloc;
          rc = lpf_d5(o, n, fc, gain, win, adopt);
          h_cov_hit(mode == 0 ? C7_DESIGN5_NULL : mode == 1 ? C7_DESIGN5_COPY : C7_DESIGN5_ADOPT);
          if (rc != ((fc == fc) && fc >= 0 && fc <= 1 && n > 1 ? 0 : 1)) h_die3("design5 return value %d (n %u fc %08x)", rc, n, fb);
          (void)nf0;
          if (rc == 0 && n <= 2048 && !adopt) lpf_check_model(o, n, fc, gain, -1, win, mode == 1);
          if (adopt && rc == 0) win = NULL; /* now owned by the object (freed by the dtor / the next design) */
          if (mode == 2 && rc != 0) { /* rejected: the caller still owns the window */ }
        }
      }
      if (redesign) h_cov_hit(C7_REDESIGN);
      first = 0;
      if (rc != 0) { if (fc != fc) h_cov_hit(C7_FC_NAN); else if (fc < 0 || fc > 1) h_cov_hit(C7_FC_BAD); if (n <= 1) h_cov_hit(C7_N_BAD); }
      else {
        if (fc == 0.0f) h_cov_hit(C7_FC0); if (fc == 1.0f) h_cov_hit(C7_FC1); if (fc == 0.5f) h_cov_hit(C7_FC_HALF);
        if (gain < 0) h_cov_hit(C7_GAIN_NEG); if (gain == 0) h_cov_hit(C7_GAIN0);
        { double x0 = -(double)fc * (double)(n - 1) * 0.5; if (x0 == (double)(long long)x0) h_cov_hit(C7_INTEGER_X); } /* exact in double on every host */
      }
      if (bits_f(fc) == 0x80000000u && rc == 0) h_cov_hit(C7_NEG0);
      h_emit(h_sample(&ctr, 30, 53), "design rc=%d n=%u fc=%08x gain=%08x", rc, n, fb, bits_f(gain));
      if (rc == 0) h_dump_obj(h_sample(&ctr, 30, 17), "LPF", o, f_LpfObj, NF_LpfObj);
      if (win) sysdep_free(win);
      h_check_not_wild("LowPassFIR");
    }
    { void *p;
      if (h_rpct(10)) { memcpy(&p, (uint8_t *)o + offsetof(LpfObj, coef_), sizeof p); if (p) { sysdep_free(p); p = NULL; memcpy((uint8_t *)o + offsetof(LpfObj, coef_), &p, sizeof p); h_cov_hit(C7_DTOR_NULL); h_cov_hit(H_COV_DTOR_NULL); } }
      lpf_d1(o); }
    h_trial_end("LowPassFIR");
  }
  h_emit(1, "lpf done");
}

/* ================================================================================================ scenario 8: Agc */
enum { C8_TRIAL = 0, C8_CTOR, C8_RESET, C8_FREEZE, C8_N0, C8_PERIOD_HIT, C8_UPDATE, C8_FROZEN_HIT, C8_LOWPOWER, C8_NANMIX, C8_MIX0, C8_MIX_HALF, C8_COUNT_POKE, C8_MANYPERIODS, C8_PERIOD1, C8_NONFIN,
       C8_TARGET, C8_INPLACE, C8_MODEL, C8_BOUNDARY, C8_BOUNDARY_EQ, C8_FROZEN_HUGE };
typedef struct { float mix, saved, gain, target, power, acc; unsigned period, count; } AgcM;
static void agc_model(AgcM *m, const float *in, float *out, unsigned n, unsigned *nper, unsigned *nupd, unsigned *nfrozen, unsigned *nlow) {
  unsigned i;
  for (i = 0; i < n; i++) {
    LD x = in[i];
    m->acc = (float)(x * x + (LD)m->acc);
    out[i] = (float)((LD)m->gain * x);
    if (--m->count == 0) {
      LD mp = ((LD)1 / (LD)m->period) * (LD)m->acc;
      m->count = m->period;
      m->acc = 0.0f;
      m->power = (float)mp;
      (*nper)++;
      if (m->mix < 1.0f || m->mix > 1.0f) {
        if (mp > (LD)1e-10f) {
          LD g0 = m->mix;
          m->gain = (float)((g0 * (LD)m->gain) + (((LD)1 - g0) * sqrtl(((LD)1 / mp) * (LD)m->target)));
          (*nupd)++;
        } else (*nlow)++;
      } else (*nfrozen)++;
    }
  }
}
static void sc_agc(int nan) {
  unsigned long ctr = 0;
  unsigned trial;
  h_cov_define(C8_TRIAL, "trials"); h_cov_define(C8_CTOR, "constructor"); h_cov_define(C8_RESET, "reset"); h_cov_define(C8_FREEZE, "freeze"); h_cov_define(C8_N0, "process(n == 0)");
  h_cov_define(C8_PERIOD_HIT, "period boundaries"); h_cov_define(C8_UPDATE, "gain updates"); h_cov_define(C8_FROZEN_HIT, "period end with the gain frozen (mix == 1)"); h_cov_define(C8_LOWPOWER, "period end with power <= 1e-10");
  h_cov_define(C8_NANMIX, "mix NaN"); h_cov_define(C8_MIX0, "mix 0"); h_cov_define(C8_MIX_HALF, "mix in (0,1)"); h_cov_define(C8_COUNT_POKE, "count_ poked"); h_cov_define(C8_MANYPERIODS, ">= 5 periods in one call");
  h_cov_define(C8_PERIOD1, "period 1"); h_cov_define(C8_NONFIN, "non-finite data"); h_cov_define(C8_TARGET, "target != 1"); h_cov_define(C8_INPLACE, "in-place (in == out)"); h_cov_define(C8_MODEL, "model-checked blocks"); h_cov_define(C8_BOUNDARY, "power-threshold boundary steps"); h_cov_define(C8_BOUNDARY_EQ, "... with power == threshold exactly"); h_cov_define(C8_FROZEN_HUGE, "frozen object with an overflowing target");
  scen_begin();
  if (nan) h_covreq[C8_MODEL] = h_covreq[C8_UPDATE] = h_covreq[C8_FROZEN_HIT] = h_covreq[C8_LOWPOWER] = 0;
  for (trial = 0; trial < (nan ? 300u : 800u); trial++) {
    AgcObj *o = (AgcObj *)h_obj_new(sizeof(AgcObj));
    AgcM m;
    unsigned nops = 3 + h_rmod(6), op;
    h_cov_hit(C8_TRIAL);
    agc_c1(o);
    h_cov_hit(C8_CTOR);
    if (o->period_ != 500 || o->count_ != 500 || bits_f(o->mix_) != 0x3f800000u || bits_f(o->gain_) != 0x3f800000u || bits_f(o->target_) != 0x3f800000u || bits_f(o->power_) != 0 || bits_f(o->acc_) != 0) h_die3("Agc ctor fields");
    m.mix = m.saved = m.gain = m.target = 1.0f; m.power = m.acc = 0.0f; m.period = m.count = 500;
    h_dump_obj(h_sample(&ctr, 12, 71), "Agc ctor", o, f_AgcObj, NF_AgcObj);
    for (op = 0; op < nops; op++) {
      int act = (int)h_rmod(100);
      if (act < 8) { agc_reset(o); h_cov_hit(C8_RESET); m.mix = m.saved = m.gain = 1.0f; m.power = m.acc = 0.0f; m.count = m.period; h_emit(0, "reset"); }
      else if (act < 14) { agc_freeze(o); h_cov_hit(C8_FREEZE); m.saved = m.mix; m.mix = 1.0f; h_emit(0, "freeze"); }
      else if (act < 40) { /* poke the configuration (these fields are plain members written by the owner) */
        int w = (int)h_rmod(10);
        if (w == 0) { float v = f_from(0x7fc00000u); o->mix_ = v; m.mix = v; h_cov_hit(C8_NANMIX); }
        else if (w == 1) { o->mix_ = 0.0f; m.mix = 0.0f; h_cov_hit(C8_MIX0); }
        else if (w == 2) { float v = 0.0625f * (float)(int)h_rrange(1, 15); o->mix_ = v; m.mix = v; h_cov_hit(C8_MIX_HALF); }
        else if (w == 3) { float v = rfloat(R_UNIT) * 0.5f + 0.5f; o->mix_ = v; m.mix = v; h_cov_hit(C8_MIX_HALF); }
        else if (w == 4) { float v = h_rpct(30) ? 4.0f : h_rpct(50) ? 0.25f : 1.0f + rfloat(R_UNIT); o->target_ = v; m.target = v; h_cov_hit(C8_TARGET); }
        else if (w == 5) { unsigned v = h_rpct(25) ? 1u : h_rpct(30) ? 2u : h_rpct(30) ? 500u : 3 + h_rmod(120); o->period_ = v; m.period = v; o->count_ = v; m.count = v; if (v == 1) h_cov_hit(C8_PERIOD1); }
        else if (w == 6) { unsigned v = h_rpct(30) ? 1u : h_rmod(40); if (v == 0) v = 1; o->count_ = v; m.count = v; h_cov_hit(C8_COUNT_POKE); }
        else if (w == 7) { float v = rfloat(R_UNIT); if (v < 0) v = -v; o->gain_ = v; m.gain = v; }
        else if (w == 8) {
          /* boundary of the gain update: period 1, acc preset to the threshold 1e-10f (0x2edbe6ff) and its neighbours, one zero sample: meanpow == threshold exactly (no update), threshold + 1 ulp (update) */
          static const uint32_t eb[3] = { 0x2edbe6feu, 0x2edbe6ffu, 0x2edbe700u };
          float zero = 0.0f, ov = 0.0f, mo2 = 0.0f;
          unsigned a = 0, b2 = 0, c = 0, d = 0;
          uint32_t pick = eb[h_rmod(3)];
          float mixv = h_rpct(50) ? 0.5f : 0.25f;
          o->period_ = 1; o->count_ = 1; o->acc_ = f_from(pick); o->mix_ = mixv; o->gain_ = 0.75f; o->target_ = 1.0f;
          m.period = 1; m.count = 1; m.acc = f_from(pick); m.mix = mixv; m.gain = 0.75f; m.target = 1.0f;
          agc_model(&m, &zero, &mo2, 1, &a, &b2, &c, &d);
          agc_proc(o, &zero, &ov, 1);
          if (bits_f(o->gain_) != bits_f(m.gain) || bits_f(o->power_) != bits_f(m.power) || bits_f(ov) != bits_f(mo2)) h_die3("Agc boundary pick %08x: gain %08x model %08x", pick, bits_f(o->gain_), bits_f(m.gain));
          h_cov_hit(C8_BOUNDARY);
          if (pick == 0x2edbe6ffu) h_cov_hit(C8_BOUNDARY_EQ);
        } else {
          /* frozen mix with a target that overflows sqrt(target / power) to inf: an update (which a frozen object must not do) would give 0 * inf = NaN */
          float one = 1.0f, ov = 0.0f, mo2 = 0.0f;
          unsigned a = 0, b2 = 0, c = 0, d = 0;
          float tv = h_rpct(50) ? f_from(0x7f800000u) : 3.0e38f;
          o->period_ = 1; o->count_ = 1; o->acc_ = 0.0f; o->mix_ = 1.0f; o->target_ = tv; o->gain_ = 2.0f;
          m.period = 1; m.count = 1; m.acc = 0.0f; m.mix = 1.0f; m.target = tv; m.gain = 2.0f;
          agc_model(&m, &one, &mo2, 1, &a, &b2, &c, &d);
          agc_proc(o, &one, &ov, 1);
          if (bits_f(o->gain_) != bits_f(m.gain) || bits_f(ov) != bits_f(mo2)) h_die3("Agc frozen/huge target: gain %08x model %08x", bits_f(o->gain_), bits_f(m.gain));
          h_cov_hit(C8_FROZEN_HUGE);
        }
        h_emit(0, "poke %d", w);
      } else {
        unsigned n = h_rpct(6) ? 0u : h_rpct(30) ? 1 + h_rmod(40) : h_rpct(30) ? m.period * (4 + h_rmod(6)) + h_rmod(5) : 1 + h_rmod(1200), i;
        int reg = h_rpct(60) ? R_UNIT : (int)h_rmod(7), inplace = h_rpct(15);
        if (nan) reg = h_rpct(30) ? R_UNIT : R_SPECIAL;                  /* scenario 14: infinities and NaNs with payload / sign (x87-exact only) */
        else if (reg == R_SPECIAL) reg = R_INFONLY;                      /* NaN ties (equal significand, different sign) are decided by the instruction form: x87-only */
        float *in = alloc_f(n + 2), *out = inplace ? in : alloc_f(n + 2), *mo = alloc_f(n + 2);
        unsigned nper = 0, nupd = 0, nfro = 0, nlow = 0;
        if (reg == R_HUGE && h_rpct(40)) reg = R_UNIT;
        for (i = 0; i < n + 2; i++) { rfill(&in[i], reg); if (h_rpct(30) && reg == R_UNIT) in[i] = f_from((bits_f(in[i]) & 0x807fffffu) | 0x3c000000u); if (!inplace) out[i] = f_from(0xeeeeeeeeu); }
        if (reg == R_SPECIAL || reg == R_INFONLY) h_cov_hit(C8_NONFIN);
        if (inplace) h_cov_hit(C8_INPLACE);
        { float *copy = alloc_f(n + 2);
          for (i = 0; i < n + 2; i++) copy[i] = in[i];
          agc_model(&m, copy, mo, n, &nper, &nupd, &nfro, &nlow);
          sysdep_free(copy); }
        agc_proc(o, in, out, n);
        if (n == 0) h_cov_hit(C8_N0);
        if (nper) h_cov_hit(C8_PERIOD_HIT); if (nupd) h_cov_hit(C8_UPDATE); if (nfro) h_cov_hit(C8_FROZEN_HIT); if (nlow) h_cov_hit(C8_LOWPOWER); if (nper >= 5) h_cov_hit(C8_MANYPERIODS);
        for (i = 0; i < n; i++) {
          if (bits_f(out[i]) != bits_f(mo[i]) && !(is_nan_f(bits_f(out[i])) && is_nan_f(bits_f(mo[i])))) h_die3("Agc out[%u] = %08x, model %08x (n %u)", i, bits_f(out[i]), bits_f(mo[i]), n);
        }
        if (n && !inplace && (bits_f(out[n]) != 0xeeeeeeeeu || bits_f(out[n + 1]) != 0xeeeeeeeeu)) h_die3("Agc wrote past n");
        if (bits_f(o->mixSaved_) != bits_f(m.saved) && !(is_nan_f(bits_f(o->mixSaved_)) && is_nan_f(bits_f(m.saved)))) h_die3("Agc mixSaved %08x, model %08x", bits_f(o->mixSaved_), bits_f(m.saved));
        if (bits_f(o->gain_) != bits_f(m.gain) && !(is_nan_f(bits_f(o->gain_)) && is_nan_f(bits_f(m.gain)))) h_die3("Agc gain %08x, model %08x", bits_f(o->gain_), bits_f(m.gain));
        if (bits_f(o->acc_) != bits_f(m.acc) && !(is_nan_f(bits_f(o->acc_)) && is_nan_f(bits_f(m.acc)))) h_die3("Agc acc %08x, model %08x", bits_f(o->acc_), bits_f(m.acc));
        if (bits_f(o->power_) != bits_f(m.power) && !(is_nan_f(bits_f(o->power_)) && is_nan_f(bits_f(m.power)))) h_die3("Agc power %08x, model %08x", bits_f(o->power_), bits_f(m.power));
        if (o->count_ != (n ? m.count : o->count_) ) h_die3("Agc count_ %u, model %u", o->count_, m.count);
        h_cov_hit(C8_MODEL);
        EMIT_BLK("agc", out, n, n, 0);
        if (reg == R_SPECIAL || reg == R_HUGE) { m.acc = o->acc_; m.gain = o->gain_; m.power = o->power_; } /* a non-finite model state: re-sync (payloads) */
        sysdep_free(mo);
        if (!inplace) sysdep_free(out);
        sysdep_free(in);
      }
      h_dump_obj(h_sample(&ctr, 20, 37), "Agc", o, f_AgcObj, NF_AgcObj);
    }
    h_trial_end("Agc");
  }
  h_emit(1, "agc done");
}

/* ================================================================================================ scenario 9: SineWave */
enum { C9_TRIAL = 0, C9_N0, C9_N1, C9_BIGBLOCK, C9_WRAP, C9_NEGPHASE, C9_BIGPHASE, C9_NANARG, C9_ZEROFS, C9_ZEROFREQ, C9_MODEL, C9_CHAIN, C9_HUGEPHASE, C9_INDEFINITE, C9_DTOR, C9_SNAN };
static void sc_sine(void) {
  unsigned long ctr = 0;
  unsigned trial;
  h_cov_define(C9_TRIAL, "trials"); h_cov_define(C9_N0, "generate(n == 0)"); h_cov_define(C9_N1, "generate(n == 1)"); h_cov_define(C9_BIGBLOCK, "block >= 1000"); h_cov_define(C9_WRAP, "phase wrap with |k| >= 1");
  h_cov_define(C9_NEGPHASE, "negative phase"); h_cov_define(C9_BIGPHASE, "|phase| >= 100"); h_cov_define(C9_NANARG, "NaN / inf parameter"); h_cov_define(C9_ZEROFS, "fs == 0"); h_cov_define(C9_ZEROFREQ, "freq == 0");
  h_cov_define(C9_MODEL, "model-checked samples"); h_cov_define(C9_CHAIN, "chained blocks"); h_cov_define(C9_HUGEPHASE, "phase / (2 pi) out of the int range (integer indefinite)");
  h_cov_define(C9_INDEFINITE, "the wrap used the integer indefinite"); h_cov_define(C9_DTOR, "destructor"); h_cov_define(C9_SNAN, "constructor with a signalling NaN argument (bit copy)");
  h_covreq[C9_HUGEPHASE] = 0;
  scen_begin();
  for (trial = 0; trial < 700; trial++) {
    SineObj *o = (SineObj *)h_obj_new(sizeof(SineObj));
    float amp, freq, phase, fs;
    unsigned nb = 1 + h_rmod(4), b;
    int snan_ctor;
    uint32_t pb[4];
    h_cov_hit(C9_TRIAL);
    amp = h_rpct(50) ? 1.0f : rfloat(R_UNIT) * 3.0f;
    fs = h_rpct(50) ? 8000.0f : h_rpct(50) ? 9600.0f : (h_rpct(5) ? 0.0f : 100.0f + (float)h_rmod(50000));
    freq = h_rpct(60) ? 1.0f + (float)h_rmod(4000) : h_rpct(50) ? rfloat(R_UNIT) * 4000.0f : h_rpct(10) ? 0.0f : rfloat(R_EXP);
    phase = h_rpct(40) ? 0.0f : h_rpct(50) ? rfloat(R_UNIT) * 7.0f : h_rpct(70) ? rfloat(R_UNIT) * 1000.0f : (h_rpct(30) ? f_from(0x7fc00000u) : rfloat(R_EXP));
    if (h_rpct(2)) freq = f_from(0x7f800000u);
    if (h_rpct(3)) phase = (h_rpct(50) ? 1.0e11f : -3.5e12f) * (1.0f + (float)h_rmod(8));      /* phase / (2 pi) outside the int32 range: the wrap stores the integer indefinite */
    snan_ctor = h_rpct(4) ? (int)(1 + h_rmod(4)) : 0;
    if (fs == 0.0f) h_cov_hit(C9_ZEROFS);
    if (freq == 0.0f) h_cov_hit(C9_ZEROFREQ);
    if (phase != phase || freq != freq || freq - freq != 0) h_cov_hit(C9_NANARG);
    if (phase < 0) h_cov_hit(C9_NEGPHASE);
    if (phase > 100 || phase < -100) h_cov_hit(C9_BIGPHASE);
    pb[0] = bits_f(amp); pb[1] = bits_f(freq); pb[2] = bits_f(phase); pb[3] = bits_f(fs);
    if (snan_ctor) pb[snan_ctor - 1] = 0x7fa00000u | (h_rnd() & 0x1fffffu) | 1u; /* a SIGNALLING NaN argument: the constructor moves the words, it must not quiet them */
    H_CTOR_PROBE(o, f_SineObj, NF_SineObj, sizeof(SineObj), sw_c1(o, FARG(pb[0]), FARG(pb[1]), FARG(pb[2]), FARG(pb[3])));
    if (bits_f(o->amp_) != pb[0] || bits_f(o->freq_) != pb[1] || bits_f(o->phase_) != pb[2] || bits_f(o->fs_) != pb[3]) h_die3("SineWave ctor fields");
    if (snan_ctor) { /* the generate() below would load the signalling NaN (x87 quiets it): from here on the object is used with quiet values only */
      float q = 0.0f;
      if (snan_ctor == 1) memcpy(&o->amp_, &q, 4); else if (snan_ctor == 2) memcpy(&o->freq_, &q, 4); else if (snan_ctor == 3) memcpy(&o->phase_, &q, 4); else { q = 8000.0f; memcpy(&o->fs_, &q, 4); }
      h_cov_hit(C9_SNAN);
    }
    h_dump_obj(h_sample(&ctr, 12, 71), "Sine ctor", o, f_SineObj, NF_SineObj);
    for (b = 0; b < nb; b++) {
      unsigned long n = h_rpct(8) ? (h_rpct(50) ? 0u : 1u) : h_rpct(15) ? 1000u + h_rmod(3000) : (unsigned long)h_rrange(2, 400);
      float *out = alloc_f((unsigned)n + 2);
      unsigned long i;
      LD ph, dphi, twopi = (LD)6.2831855f;
      float mphase = o->phase_;
      for (i = 0; i < n + 2; i++) out[i] = f_from(0xeeeeeeeeu);
      /* model: the same recurrence written as a state machine with the C library's sinl (hardware-independent up to 1 float ulp) */
      dphi = ((LD)o->freq_ * twopi) / (LD)o->fs_;
      ph = (LD)mphase;
      { float *mo = alloc_f((unsigned)n + 2);
        float amp_ = o->amp_;
        for (i = 0; i < n; i++) {
          if (i) mphase = (float)ph;
          mo[i] = (float)(sinl(ph) * (LD)amp_);
          ph = (LD)mphase + dphi;
        }
        { LD q = ph * (LD)0.15915494f; int k = (q > -2147483649.0L && q < 2147483648.0L) ? (int)q : (int)0x80000000u;
          if (k == (int)0x80000000u) h_cov_hit(C9_INDEFINITE);
          if (k >= 1 || k <= -1) h_cov_hit(C9_WRAP);
          mphase = (float)(ph - (LD)k * twopi); }
        sw_gen(o, out, n);
        if (n == 0) h_cov_hit(C9_N0); if (n == 1) h_cov_hit(C9_N1); if (n >= 1000) h_cov_hit(C9_BIGBLOCK); if (b) h_cov_hit(C9_CHAIN);
        if (bits_f(out[n]) != 0xeeeeeeeeu || bits_f(out[n + 1]) != 0xeeeeeeeeu) h_die3("SineWave::generate wrote past n");
        if (bits_f(o->phase_) != bits_f(mphase) && !(is_nan_f(bits_f(o->phase_)) && is_nan_f(bits_f(mphase)))) h_die3("SineWave phase %08x, model %08x (n %lu)", bits_f(o->phase_), bits_f(mphase), n);
        { int fin = (freq == freq) && (freq - freq == 0) && (fs == fs) && fs != 0.0f && (amp == amp) && fabsf(phase) < 1.0e5f && fabsf(freq) < 1.0e7f; /* beyond ~1e5 rad the instruction's 66-bit pi reduction differs from the library's: not modelled (blob vs port still compared) */
          for (i = 0; i < n; i++) {
            if (!fin) break;
            if (is_nan_f(bits_f(mo[i]))) { if (!is_nan_f(bits_f(out[i]))) h_die3("SineWave out[%lu]: model NaN", i); continue; }
            if (ulps(out[i], mo[i]) > 1 && ((double)(out[i] > mo[i] ? out[i] - mo[i] : mo[i] - out[i])) > 1e-30) h_die3("SineWave out[%lu] = %08x, model %08x", i, bits_f(out[i]), bits_f(mo[i]));
            g_model_checked++;
          }
          if (fin) h_cov_hit(C9_MODEL); }
        sysdep_free(mo); }
      EMIT_BLK("sine", out, (unsigned)n, n, b);
      h_dump_obj(h_sample(&ctr, 20, 37), "Sine", o, f_SineObj, NF_SineObj);
      sysdep_free(out);
      if (bits_f(o->phase_) == 0xffffffffu) h_cov_hit(C9_HUGEPHASE);
    }
    sw_d1(o);
    h_cov_hit(C9_DTOR);
    h_trial_end("SineWave");
  }
  h_emit(1, "sine done");
}

/* ================================================================================================ scenario 10: the transcendental sites */
/* The 12 sites exercised directly on dense and special inputs through the exported functions (blob vs port).  Constants of the blob are
 * re-derived by the model so that a wrong literal in the port shows up as a bit difference, not only as a model tolerance. */
static void sc_sites(void) {
  unsigned i, n;
  scen_begin();
  /* (a) windows at every size 1 .. 600 (hanning / hamming / blackman call fcos 1 / 1 / 2 times per sample) - scenario 0 covers more sizes, this one hashes whole families */
  for (n = 1; n <= 600; n++) {
    float *w = alloc_f(n + 1);
    int t;
    for (t = 1; t <= 3; t++) { w_design(t, w, n); EMIT_BLK("site-win", w, n, n, t); }
    sysdep_free(w);
  }
  h_trial_end("sites");
  /* (b) four1 / realfft: every theta of every power-of-two size, both signs: impulse and ones (the twiddles themselves appear in the output) */
  for (n = 1; n <= 4096; n <<= 1) {
    int s;
    for (s = -1; s <= 1; s += 2) {
      float *d = alloc_f(2 * n + 3), *r = alloc_f(n + 3);
      unsigned k;
      for (k = 0; k < 2 * n + 3; k++) d[k] = f_from(0u);
      for (k = 1; k <= 2 * n; k += 2) d[k] = f_from(0x3f800000u + (k << 3));
      f_four1(d, n, s);
      EMIT_BLK("site-four1", d, 2 * n + 3, n, s);
      for (k = 0; k < n + 3; k++) r[k] = f_from(0u);
      for (k = 1; k <= n; k++) r[k] = f_from(0x3f800000u + (k << 5));
      if (n >= 2) { f_realfft(r, n, s); EMIT_BLK("site-realfft", r, n + 3, n, s); }
      sysdep_free(r);
      sysdep_free(d);
    }
  }
  h_trial_end("sites");
  /* (c) Psd dB conversion over a dense sweep of power values: out[] pre-set through option 3 is not possible, so drive the dB loop through process() with controlled inputs:
   * a constant segment gives bin 0 = N^2-ish and zeros elsewhere; random data gives a spread of values (log10 of 1e-25 .. 1e9) */
  for (i = 0; i < 40; i++) {
    unsigned N = 8u << (i % 6);
    PsdObj *o = (PsdObj *)h_obj_new(sizeof(PsdObj));
    unsigned nseg = 1 + (i % 4), len = N * nseg, k;
    float *in = alloc_f(len + 1), *out = alloc_f(N / 2 + 1);
    int opt;
    psd_c1(o, N, 1 + (int)(i % 3), 0);
    for (opt = 0; opt <= 1; opt++) {
      for (k = 0; k < len; k++) { float v = rfloat(R_EXP); in[k] = (i & 1) ? v : v * 1.0e-3f; }
      psd_proc(o, in, len, out, opt);
      EMIT_BLK("site-psd", out, N / 2, N, opt);
    }
    psd_d1(o);
    sysdep_free(out);
    sysdep_free(in);
    h_trial_end("sites");
  }
  h_emit(1, "sites done");
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  g_scenario = sc;
  const char *seed = getenv("P03_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  p03_layout_check();
  h_x87_check();
  h_arena_init();
  switch (sc) {
    case 0: sc_windows(); break;
    case 1: sc_sinc(); break;
    case 2: sc_stats(0); break;
    case 3: sc_stats(1); break;
    case 4: sc_fft(0, 0); break;
    case 5: sc_fft(1, 0); break;
    case 6: sc_psd(0); break;
    case 7: sc_lpf(); break;
    case 8: sc_agc(0); break;
    case 9: sc_sine(); break;
    case 10: sc_sites(); break;
    case 11: sc_fft(0, 1); break;
    case 12: sc_fft(1, 1); break;
    case 13: sc_psd(1); break;
    case 14: sc_agc(1); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu", h_nmalloc, h_nfree);
  fprintf(stderr, "model checks: %lu (%lu exact)\n", g_model_checked, g_model_exact);
  h_cov_finish();
  return 0;
}
