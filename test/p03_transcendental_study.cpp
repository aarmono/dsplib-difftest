/*
 * Transcendental policy study of chunk P03 (NOTES.md "Transcendental policy").  Not part of the differential test: it measures, for the 12
 * transcendental sites of the chunk, how far three implementations are from the x87 INSTRUCTION that the blob executes:
 *
 *   X87   fsin / fcos / fyl2x (`fldlg2; fyl2x` for log10) on extended operands          = the blob, = src/core/dsp_math.h
 *   LDL   the C library's long double routines sinl / cosl / log10l (glibc: software, 64-bit mantissa)   = dsp_math.h with DSP_MATH_PORTABLE
 *   DBL   the C library's DOUBLE routines sin / cos / log10 (argument rounded to double, result back to extended)   = what `-O2` code that
 *         calls the plain libm functions on a non-x87 host would get (a binary128 `long double` host is not simulated)
 *
 * Part A: the raw instruction results at 80-bit precision over dense / structured input sweeps (ULP of the 64-bit mantissa).
 * Part B: the SITES: each site formula of the port (transcribed here, templated on the implementation; the X87 instantiation is checked bit
 * for bit against the port functions that are linked in) evaluated on the reachable inputs, compared at the STORED precision (float32 /
 * double): how many stored values differ from the X87 ones and by how many float ULPs at most.
 *
 * Build / run: sh test/p03_study.sh   (x86-64, -O2; the x87 instructions give the same bits as in the 32-bit process on the same CPU).
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dsp/dsp_window_fft_psd.h"
#include "core/dsp_math.h"

extern "C" {
void *sysdep_malloc(unsigned int n) { return malloc(n ? n : 1); }
void sysdep_free(void *p) { free(p); }
}

typedef long double LD;

struct X87 { static const char *name() { return "x87"; } static LD sin(LD x) { return dsp_fsin(x); } static LD cos(LD x) { return dsp_fcos(x); } static LD log10(LD x) { return dsp_log10(x); } };
struct LDL { static const char *name() { return "ldlibm"; } static LD sin(LD x) { return sinl(x); } static LD cos(LD x) { return cosl(x); } static LD log10(LD x) { return log10l(x); } };
struct DBL { static const char *name() { return "dbllibm"; } static LD sin(LD x) { return (LD)::sin((double)x); } static LD cos(LD x) { return (LD)::cos((double)x); } static LD log10(LD x) { return (LD)::log10((double)x); } };

/* ------------------------------------------------------------------------------------------------ ULP helpers */
static unsigned h_rng = 0x9e3779b9u;
static unsigned rnd() { h_rng ^= h_rng << 13; h_rng ^= h_rng >> 17; h_rng ^= h_rng << 5; return h_rng; }
static double urand() { return ((double)(((uint64_t)rnd() << 21) ^ rnd()) / 9007199254740992.0) ; }   /* [0,1) with ~53 bits (xorshift32 pairs) */
static __int128 key80(LD x) {
  struct { uint64_t m; uint16_t se; } u;
  memset(&u, 0, sizeof u);
  memcpy(&u, &x, 10);
  __int128 mag = ((__int128)(u.se & 0x7fff) << 64) | (__int128)u.m;
  return (u.se & 0x8000) ? -mag : mag;
}
static int64_t key32(float f) { int32_t b; memcpy(&b, &f, 4); return b < 0 ? -(int64_t)(b & 0x7fffffff) : (int64_t)b; }
static uint32_t bits32(float f) { uint32_t b; memcpy(&b, &f, 4); return b; }
static bool isnan32(float f) { return f != f; }

struct Stat {
  const char *what;
  unsigned long n = 0, diff = 0;
  double maxulp = 0, sumulp = 0;
  void add80(LD a, LD b) {
    n++;
    if (a != a || b != b) { if ((a != a) != (b != b)) { diff++; maxulp = 1e30; } return; }
    __int128 d = key80(a) - key80(b);
    if (d < 0) d = -d;
    if (d != 0) { diff++; double u = (double)d; sumulp += u; if (u > maxulp) maxulp = u; }
  }
  void add32(float a, float b) {
    n++;
    if (isnan32(a) || isnan32(b)) { if (isnan32(a) != isnan32(b)) { diff++; maxulp = 1e30; } return; }
    int64_t d = key32(a) - key32(b);
    if (d < 0) d = -d;
    if (d != 0) { diff++; sumulp += (double)d; if ((double)d > maxulp) maxulp = (double)d; }
  }
};
static void row(const char *site, const char *impl, const Stat &s, const char *unit) {
  printf("  %-34s %-8s n=%-10lu differs=%-9lu (%.3e)  max %-12.6g %s  mean-of-differing %.3g\n", site, impl, s.n, s.diff, s.n ? (double)s.diff / (double)s.n : 0.0, s.maxulp, unit, s.diff ? s.sumulp / (double)s.diff : 0.0);
}

/* ------------------------------------------------------------------------------------------------ Part A: the raw instructions */
template <class I> static void raw_sweep(const char *name, LD (*fx)(LD), LD (*fi)(LD), LD lo, LD hi, unsigned long count, bool logscale, Stat &st) {
  (void)name;
  for (unsigned long k = 0; k < count; k++) {
    LD t = (LD)urand();
    LD x = logscale ? lo * powl(hi / lo, t) : lo + (hi - lo) * t;
    st.add80(fx(x), fi(x));
  }
}
static LD x_sin(LD x) { return X87::sin(x); }
static LD x_cos(LD x) { return X87::cos(x); }
static LD x_log10(LD x) { return X87::log10(x); }
static LD l_sin(LD x) { return LDL::sin(x); }
static LD l_cos(LD x) { return LDL::cos(x); }
static LD l_log10(LD x) { return LDL::log10(x); }
static LD d_sin(LD x) { return DBL::sin(x); }
static LD d_cos(LD x) { return DBL::cos(x); }
static LD d_log10(LD x) { return DBL::log10(x); }

static void partA(unsigned long N) {
  printf("PART A: the raw instruction results, 80-bit ULPs (x87 instruction vs the C library), %lu samples per row\n", N);
  struct { const char *name; LD (*x)(LD); LD (*l)(LD); LD (*d)(LD); LD lo, hi; bool lg; } rows[] = {
    { "fsin   x in [0, 2pi]", x_sin, l_sin, d_sin, 0.0L, 6.283185307179586L, false },
    { "fsin   x in [-8, 8] (sinc: pi x)", x_sin, l_sin, d_sin, -8.0L, 8.0L, false },
    { "fsin   x in [1e-6, 1e-1] (log)", x_sin, l_sin, d_sin, 1e-6L, 1e-1L, true },
    { "fsin   x in [0, 3.2e3] (pi * x, |x| <= 1000)", x_sin, l_sin, d_sin, 0.0L, 3141.592653589793L, false },
    { "fcos   x in [0, 2pi]", x_cos, l_cos, d_cos, 0.0L, 6.283185307179586L, false },
    { "fcos   x in [1e-6, 1e-1] (log)", x_cos, l_cos, d_cos, 1e-6L, 1e-1L, true },
    { "fyl2x  log10 x in [1e-25, 1] (log)", x_log10, l_log10, d_log10, 1e-25L, 1.0L, true },
    { "fyl2x  log10 x in [0.9, 1.1]", x_log10, l_log10, d_log10, 0.9L, 1.1L, false },
    { "fyl2x  log10 x in [1 - 1e-6, 1 + 1e-6]", x_log10, l_log10, d_log10, 0.999999L, 1.000001L, false },
    { "fyl2x  log10 x in [1, 1e12] (log)", x_log10, l_log10, d_log10, 1.0L, 1e12L, true },
  };
  for (unsigned i = 0; i < sizeof rows / sizeof rows[0]; i++) {
    Stat a, b;
    h_rng = 0x1234567u + i;
    raw_sweep<X87>("", rows[i].x, rows[i].l, rows[i].lo, rows[i].hi, N, rows[i].lg, a);
    h_rng = 0x1234567u + i;
    raw_sweep<X87>("", rows[i].x, rows[i].d, rows[i].lo, rows[i].hi, N, rows[i].lg, b);
    row(rows[i].name, "ldlibm", a, "ulp80");
    row(rows[i].name, "dbllibm", b, "ulp80");
  }
  /* structured: multiples of pi / 2 (fsin / fcos use a 66-bit pi: the famous near-zero cases), as double and as extended arguments */
  { Stat a, b, c;
    for (int k = -4000; k <= 4000; k++) {
      LD xd = (LD)((double)k * 1.5707963267948966), xe = (LD)k * 1.57079632679489661923132169163975144L;
      a.add80(X87::sin(xd), LDL::sin(xd)); b.add80(X87::cos(xd), LDL::cos(xd)); c.add80(X87::sin(xe), LDL::sin(xe));
    }
    row("fsin   k * (pi/2 as double), |k| <= 4000", "ldlibm", a, "ulp80"); row("fcos   k * (pi/2 as double)", "ldlibm", b, "ulp80"); row("fsin   k * pi/2 (extended)", "ldlibm", c, "ulp80"); }
}

/* ------------------------------------------------------------------------------------------------ Part B: the sites */
static const LD TWOPI = (LD)6.283185307179586, FOURPI = (LD)12.566370614359172, PI = (LD)3.141592653589793;

template <class I> static float hann_w(unsigned k, unsigned n) { LD q = (LD)1 / (LD)(uint32_t)(n + 1u); LD x = ((LD)k * TWOPI) * q; return (float)(((LD)1 - I::cos(x)) * (LD)0.5f); }
template <class I> static float hamm_w(unsigned i, unsigned n) { LD q = (LD)1 / (LD)(uint32_t)(n - 1u); LD x = ((LD)i * TWOPI) * q; return (float)((LD)0.54 - I::cos(x) * (LD)0.46); }
template <class I> static float black_w(unsigned i, unsigned n) {
  LD q = (LD)1 / (LD)(uint32_t)(n - 1u);
  LD c2 = I::cos(((LD)i * TWOPI) * q), c4 = I::cos(((LD)i * FOURPI) * q);
  return (float)(c4 * (LD)0.08 + ((LD)0.42 - c2 * (LD)0.5f));
}
template <class I> static float sinc_t(float x) {
  LD xv = (LD)x;
  if (!(xv < 0 || xv > 0)) return 1.0f;
  LD y = xv * PI;
  return (float)(I::sin(y) / y);
}

/* the FFT pair with the implementation as a parameter (a transcription of src/dsp/dsp_window_fft_psd.cpp; the X87 instantiation is verified against the port) */
template <class I> static void four1_t(float *data, unsigned long nn, int isign) {
  uint32_t n = (uint32_t)nn * 2u, i, j = 1, m, mmax, istep;
  for (i = 1; i < n; i += 2) {
    if (j > i) { float t = data[j]; data[j] = data[i]; data[i] = t; t = data[j + 1]; data[j + 1] = data[i + 1]; data[i + 1] = t; }
    m = n;
    for (;;) { m >>= 1; if (!(m > 1 && j > m)) break; j -= m; }
    j += m;
  }
  if (n <= 2) return;
  mmax = 2;
  for (;;) {
    istep = mmax << 1;
    LD theta = ((LD)1 / (LD)(uint32_t)mmax) * (LD)6.28318530717959;
    theta = theta * (LD)(double)isign;
    LD wpi = I::sin(theta);
    LD wtemp = I::sin(theta * (LD)0.5);
    LD wpr = (wtemp * wtemp) * (LD)(-2.0);
    LD wr = 1, wi = 0;
    for (m = 1; m < mmax; m += 2) {
      for (i = m; i <= n; i += istep) {
        j = i + mmax;
        float tempr = (float)(((LD)data[j] * wr) - (wi * (LD)data[j + 1]));
        float tempi = (float)(((LD)data[j] * wi) + (wr * (LD)data[j + 1]));
        data[j] = (float)((LD)data[i] - (LD)tempr);
        data[j + 1] = (float)((LD)data[i + 1] - (LD)tempi);
        data[i] = (float)((LD)tempr + (LD)data[i]);
        data[i + 1] = (float)((LD)tempi + (LD)data[i + 1]);
      }
      LD wrn = wr + ((wr * wpr) - (wi * wpi));
      LD win = wi + ((wr * wpi) + (wi * wpr));
      wr = wrn; wi = win;
    }
    mmax = istep;
    if (!(n > mmax)) break;
  }
}
template <class I> static void realfft_t(float *data, unsigned long nl, int isign) {
  uint32_t n = (uint32_t)nl;
  LD theta = ((LD)1 / (LD)(uint32_t)(n >> 1)) * PI;
  float c1 = 0.5f, c2;
  if (isign == 1) { double ts = (double)theta; c2 = -0.5f; four1_t<I>(data, n >> 1, 1); theta = (LD)ts; }
  else { theta = -theta; c2 = 0.5f; }
  const LD sinHalf = I::sin((LD)c1 * theta), sinTheta = I::sin(theta);
  double wpi = (double)sinTheta, wi = (double)sinTheta;
  const LD wprExt = (sinHalf * sinHalf) * (LD)(-2.0f);
  double wpr = (double)wprExt;
  LD wr = (LD)1 + wprExt;
  const float negc2 = -c2;
  const uint32_t np3 = n + 3u;
  for (uint32_t i = 2; i <= (n >> 2); i++) {
    uint32_t i1 = i + i - 1u, i2 = i1 + 1u, i3 = np3 - i2, i4 = i3 + 1u;
    float a = data[i1], b = data[i2], c = data[i3], d = data[i4];
    LD h2r = ((LD)b + (LD)d) * (LD)negc2, h1r = ((LD)c + (LD)a) * (LD)c1, h2i = ((LD)a - (LD)c) * (LD)c2, h1i = ((LD)b - (LD)d) * (LD)c1;
    data[i1] = (float)((h1r + h2r * wr) - ((LD)wi * h2i));
    data[i2] = (float)(((h2i * wr) + h1i) + (h2r * (LD)wi));
    data[i3] = (float)(((LD)wi * h2i) + (h1r - h2r * wr));
    LD nh1i = -h1i;
    data[i4] = (float)((nh1i + (h2i * wr)) + (h2r * (LD)wi));
    LD wrn = wr + (((LD)wpr * wr) - ((LD)wi * (LD)wpi));
    double win = (double)(((wr * (LD)wpi) + ((LD)wi * (LD)wpr)) + (LD)wi);
    wr = wrn; wi = win;
  }
  float d1 = data[1], d2 = data[2];
  if (isign == 1) { data[1] = (float)((LD)d1 + (LD)d2); data[2] = (float)((LD)d1 - (LD)d2); }
  else { data[1] = (float)(((LD)d1 + (LD)d2) * (LD)c1); data[2] = (float)(((LD)d1 - (LD)d2) * (LD)c1); four1_t<I>(data, n >> 1, -1); }
}
template <class I> static float db_t(float v, LD recip) { LD x = (LD)v * recip + (LD)1e-25; float l = (float)I::log10(x); return (float)((LD)l * (LD)10.0f); }
/* SineWave::generate */
template <class I> static void sine_t(float amp, float freq, float &phase, float fs, float *out, unsigned n) {
  const LD twopi = (LD)6.2831855f;
  const LD dphi = ((LD)freq * twopi) / (LD)fs;
  LD ph = (LD)phase;
  for (unsigned i = 0; i < n; i++) {
    if (i != 0) phase = (float)ph;
    out[i] = (float)(I::sin(ph) * (LD)amp);
    ph = (LD)phase + dphi;
  }
  int k = dsp_trunc_i32(ph * (LD)0.15915494f);
  phase = (float)(ph - (LD)k * twopi);
}

static unsigned long g_selfcheck_fail = 0;

static void partB() {
  printf("\nPART B: the sites at the STORED precision (float32 ULPs of the stored value vs the x87 instruction), reachable inputs\n");
  /* --- windows: every n up to 4096 and selected larger ones, every sample --- */
  Stat h2[3], m2[3], b2[3];
  const char *impl[3] = { "x87", "ldlibm", "dbllibm" };
  for (unsigned n = 1; n <= 4096 + 40; n++) {
    unsigned nn = n <= 4096 ? n : (n - 4096) * 997u + 4096u;
    float *w = (float *)malloc((nn + 1) * 4);
    for (int t = 1; t <= 3; t++) {
      if (t == 1) { hanning<float>(w, nn); for (unsigned k = 1; k <= nn; k++) { float x = hann_w<X87>(k, nn); if (bits32(x) != bits32(w[k - 1]) && !isnan32(x)) g_selfcheck_fail++; h2[0].add32(x, x); h2[1].add32(x, hann_w<LDL>(k, nn)); h2[2].add32(x, hann_w<DBL>(k, nn)); } }
      if (t == 2 && nn > 1) { hamming<float>(w, nn); for (unsigned i = 0; i < nn; i++) { float x = hamm_w<X87>(i, nn); if (bits32(x) != bits32(w[i])) g_selfcheck_fail++; m2[0].add32(x, x); m2[1].add32(x, hamm_w<LDL>(i, nn)); m2[2].add32(x, hamm_w<DBL>(i, nn)); } }
      if (t == 3 && nn > 1) { blackman<float>(w, nn); for (unsigned i = 0; i < nn; i++) { float x = black_w<X87>(i, nn); if (bits32(x) != bits32(w[i])) g_selfcheck_fail++; b2[0].add32(x, x); b2[1].add32(x, black_w<LDL>(i, nn)); b2[2].add32(x, black_w<DBL>(i, nn)); } }
    }
    free(w);
  }
  for (int i = 1; i < 3; i++) row("hanning  (n 1..4096, +40 larger)", impl[i], h2[i], "float ulp");
  for (int i = 1; i < 3; i++) row("hamming  (same sizes)", impl[i], m2[i], "float ulp");
  for (int i = 1; i < 3; i++) row("blackman (same sizes)", impl[i], b2[i], "float ulp");
  { Stat a[3], b[3];  /* power-of-two and typical sizes up to 2^20 (every sample) */
    for (int e = 12; e <= 20; e++) { unsigned n = 1u << e; for (unsigned k = 1; k <= n; k += (e > 16 ? 7 : 1)) { float x = hann_w<X87>(k, n); a[1].add32(x, hann_w<LDL>(k, n)); a[2].add32(x, hann_w<DBL>(k, n)); float y = black_w<X87>(k % n, n); b[1].add32(y, black_w<LDL>(k % n, n)); b[2].add32(y, black_w<DBL>(k % n, n)); } }
    for (int i = 1; i < 3; i++) row("hanning  (n = 2^12 .. 2^20)", impl[i], a[i], "float ulp");
    for (int i = 1; i < 3; i++) row("blackman (n = 2^12 .. 2^20)", impl[i], b[i], "float ulp"); }

  /* --- sinc through the LowPassFIR argument sequence (x0 = -fc (n-1)/2, x(i+1) = (float)x(i) + fc) and as a whole design --- */
  { Stat s[3], intargs[3], coef[3]; unsigned long ncoefsum = 0; double worstrel[3] = { 0, 0, 0 }; unsigned long sumdiffs[3] = { 0, 0, 0 };
    for (unsigned n = 2; n <= 400; n++) {
      for (int kf = 0; kf <= 64; kf++) {
        float fc = (float)kf / 64.0f;
        LD x = (-(LD)fc * (LD)(uint32_t)(n - 1u)) * (LD)0.5f;
        float cx[3][512], win[512];
        for (unsigned i = 0; i < n; i++) win[i] = hamm_w<X87>(i, n);
        for (int im = 0; im < 3; im++) {
          LD xx = x; LD sum = 0; float cf[512];
          for (unsigned i = 0; i < n; i++) {
            float xf = (float)xx;
            float sc = im == 0 ? sinc_t<X87>(xf) : im == 1 ? sinc_t<LDL>(xf) : sinc_t<DBL>(xf);
            if (im != 0) { float sx = sinc_t<X87>(xf); s[im].add32(sx, sc); if (xf == (float)(int)xf && (xf != 0)) intargs[im].add32(sx, sc); }
            cf[i] = (float)(((LD)sc * (LD)fc) * (LD)win[i]);
            xx = (LD)xf + (LD)fc;
          }
          for (unsigned i = 0; i < n; i++) sum += (LD)cf[i];
          LD scale = (LD)1 / sum;
          for (unsigned i = 0; i < n; i++) cx[im][i] = (float)((LD)cf[i] * scale);
        }
        float peak = 0; for (unsigned i = 0; i < n; i++) if (fabsf(cx[0][i]) > peak) peak = fabsf(cx[0][i]);
        for (int im = 1; im < 3; im++) for (unsigned i = 0; i < n; i++) { coef[im].add32(cx[0][i], cx[im][i]); if (cx[0][i] != cx[im][i]) { sumdiffs[im]++; double r = fabs((double)cx[0][i] - (double)cx[im][i]) / (peak > 0 ? peak : 1); if (r > worstrel[im]) worstrel[im] = r; } }
        ncoefsum++;
      }
    }
    for (int i = 1; i < 3; i++) row("sinc  (LowPassFIR argument sequences)", impl[i], s[i], "float ulp");
    for (int i = 1; i < 3; i++) row("sinc  (exact integer arguments only)", impl[i], intargs[i], "float ulp");
    for (int i = 1; i < 3; i++) { row("LowPassFIR coefficients (n 2..400, fc k/64)", impl[i], coef[i], "float ulp"); printf("      -> worst |difference| / peak coefficient: %.3e over %lu designs\n", worstrel[i], ncoefsum); }
    (void)sumdiffs; }

  /* --- FFT: every power-of-two size 2 .. 2^16 (twiddles = every theta of the recurrence), both signs, random unit data; whole transform compared --- */
  { Stat f1[3], rf[3];
    for (unsigned e = 1; e <= 16; e++) {
      unsigned n = 1u << e;
      for (int sg = -1; sg <= 1; sg += 2) {
        unsigned reps = e <= 8 ? 40u : e <= 12 ? 6u : 1u;
        for (unsigned r = 0; r < reps; r++) {
          float *d0 = (float *)malloc((2 * n + 3) * 4), *d1 = (float *)malloc((2 * n + 3) * 4), *d2 = (float *)malloc((2 * n + 3) * 4), *d3 = (float *)malloc((2 * n + 3) * 4);
          for (unsigned i = 0; i < 2 * n + 3; i++) d0[i] = (float)((int32_t)(rnd() >> 7) - (1 << 24)) / 16777216.0f;
          memcpy(d1, d0, (2 * n + 3) * 4); memcpy(d2, d0, (2 * n + 3) * 4); memcpy(d3, d0, (2 * n + 3) * 4);
          four1_t<X87>(d1, n, sg); four1_t<LDL>(d2, n, sg); four1_t<DBL>(d3, n, sg);
          if (r == 0 && e <= 12) { float *p = (float *)malloc((2 * n + 3) * 4); memcpy(p, d0, (2 * n + 3) * 4); four1(p, n, sg); for (unsigned i = 1; i <= 2 * n; i++) if (bits32(p[i]) != bits32(d1[i])) g_selfcheck_fail++; free(p); }
          for (unsigned i = 1; i <= 2 * n; i++) { f1[1].add32(d1[i], d2[i]); f1[2].add32(d1[i], d3[i]); }
          memcpy(d1, d0, (2 * n + 3) * 4); memcpy(d2, d0, (2 * n + 3) * 4); memcpy(d3, d0, (2 * n + 3) * 4);
          realfft_t<X87>(d1, n, sg); realfft_t<LDL>(d2, n, sg); realfft_t<DBL>(d3, n, sg);
          if (r == 0 && e <= 12) { float *p = (float *)malloc((2 * n + 3) * 4); memcpy(p, d0, (2 * n + 3) * 4); realfft(p, n, sg); for (unsigned i = 1; i <= n; i++) if (bits32(p[i]) != bits32(d1[i])) g_selfcheck_fail++; free(p); }
          for (unsigned i = 1; i <= n; i++) { rf[1].add32(d1[i], d2[i]); rf[2].add32(d1[i], d3[i]); }
          free(d0); free(d1); free(d2); free(d3);
        }
      }
    }
    for (int i = 1; i < 3; i++) row("four1   (n 2..65536, both signs)", impl[i], f1[i], "float ulp");
    for (int i = 1; i < 3; i++) row("realfft (n 2..65536, both signs)", impl[i], rf[i], "float ulp"); }
  /* the twiddle inputs themselves: sin(theta), sin(theta/2) for theta = +-2pi / 2^k, k = 1 .. 31 and pi / 2^k (realfft): 80-bit ULPs */
  { Stat a, b, c, d;
    for (int k = 1; k <= 31; k++) for (int sg = -1; sg <= 1; sg += 2) {
      LD th = ((LD)1 / (LD)(uint32_t)(1u << k)) * (LD)6.28318530717959 * (LD)(double)sg, th2 = th * (LD)0.5, tp = ((LD)1 / (LD)(uint32_t)(1u << k)) * PI * (LD)sg;
      a.add80(X87::sin(th), LDL::sin(th)); a.add80(X87::sin(th2), LDL::sin(th2)); b.add80(X87::sin(th), DBL::sin(th)); b.add80(X87::sin(th2), DBL::sin(th2));
      c.add80(X87::sin(tp), LDL::sin(tp)); c.add80(X87::sin(tp * (LD)0.5f), LDL::sin(tp * (LD)0.5f)); d.add80(X87::sin(tp), DBL::sin(tp)); d.add80(X87::sin(tp * (LD)0.5f), DBL::sin(tp * (LD)0.5f));
    }
    row("four1 twiddle sin(theta), sin(theta/2)", "ldlibm", a, "ulp80"); row("four1 twiddle sin(theta), sin(theta/2)", "dbllibm", b, "ulp80");
    row("realfft twiddle sin(theta), sin(theta/2)", "ldlibm", c, "ulp80"); row("realfft twiddle sin(theta), sin(theta/2)", "dbllibm", d, "ulp80"); }

  /* --- Psd dB conversion: x = v / max + 1e-25 over the value range a spectrum has --- */
  { Stat a[3], rel[3], near1[3];
    h_rng = 777;
    for (unsigned long k = 0; k < 3000000; k++) {
      float v = (float)(pow(10.0, -25.0 + 25.0 * urand()));                       /* normalised bin power 1e-25 .. 1 */
      float x = db_t<X87>(v, (LD)1);
      a[1].add32(x, db_t<LDL>(v, (LD)1)); a[2].add32(x, db_t<DBL>(v, (LD)1));
      float w = (float)(1.0 - 1e-3 * urand());                                    /* the bins next to the maximum */
      float y = db_t<X87>(w, (LD)1);
      near1[1].add32(y, db_t<LDL>(w, (LD)1)); near1[2].add32(y, db_t<DBL>(w, (LD)1));
    }
    for (int i = 1; i < 3; i++) row("Psd dB (v/max in 1e-25 .. 1, log-uniform)", impl[i], a[i], "float ulp");
    for (int i = 1; i < 3; i++) row("Psd dB (v/max in [0.999, 1])", impl[i], near1[i], "float ulp");
    (void)rel; }

  /* --- SineWave::generate: the oscillator settings of the modem (tones around 1 kHz at 8 / 9.6 kHz) and random ones --- */
  { Stat a[3]; Stat ph[3];
    static const float fr[] = { 300, 980, 1000, 1200, 1800, 2100, 2400, 2700, 3000, 3600, 100, 4000 };
    static const float fsr[] = { 8000, 9600, 7200, 11025, 44100 };
    for (unsigned fi = 0; fi < 12; fi++) for (unsigned si = 0; si < 5; si++) for (unsigned pi = 0; pi < 6; pi++) {
      float p0[3] = { 0.1f * pi, 0.1f * pi, 0.1f * pi };
      for (int blk = 0; blk < 40; blk++) {
        float o[3][1000];
        sine_t<X87>(4780.0f, fr[fi], p0[0], fsr[si], o[0], 1000); sine_t<LDL>(4780.0f, fr[fi], p0[1], fsr[si], o[1], 1000); sine_t<DBL>(4780.0f, fr[fi], p0[2], fsr[si], o[2], 1000);
        for (int i = 0; i < 1000; i++) { a[1].add32(o[0][i], o[1][i]); a[2].add32(o[0][i], o[2][i]); }
        ph[1].add32(p0[0], p0[1]); ph[2].add32(p0[0], p0[2]);
        if (blk == 0 && fi == 3 && si == 0) { float pp = 0.1f * pi; float o2[1000]; SineWave<float, float> sw(4780.0f, fr[fi], pp, fsr[si]); sw.generate(o2, 1000); float q = 0.1f * pi; sine_t<X87>(4780.0f, fr[fi], q, fsr[si], o[0], 1000); for (int i = 0; i < 1000; i++) if (bits32(o2[i]) != bits32(o[0][i])) g_selfcheck_fail++; }
      }
    }
    for (int i = 1; i < 3; i++) { row("SineWave samples (12 freqs x 5 rates, 40 blocks)", impl[i], a[i], "float ulp"); row("SineWave phase field after a block", impl[i], ph[i], "float ulp"); } }
  printf("\nself-check (X87 instantiations of the transcribed formulas vs the port functions, bit for bit): %lu mismatches\n", g_selfcheck_fail);
}

int main(int argc, char **argv) {
  unsigned long N = argc > 1 ? strtoul(argv[1], 0, 0) : 2000000ul;
  partA(N);
  partB();
  return g_selfcheck_fail ? 1 : 0;
}
