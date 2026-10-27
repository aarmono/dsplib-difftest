/*
 * Differential test harness for port chunk P41 (src/vpcm/v90_equalizer_phases.cpp): V90Equalizer::enterPhase4() (2,050 B), convertEqualizerToMmx() (2,573 B), enterDataPhase() (1,688 B).
 * Derived from test/vpcm/v90_equalizer_process.main.c (same recorder / snapshot / independent-model design).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_equalizer_phases.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port
 * (-O0 / -O2: the port object alone, the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_equalizer_phases.blob32.out).  The functions are reached through __asm__
 * labels; the object is the generated C mirror of test/vpcm/v90_equalizer_process.layout (the WHOLE V90Equalizer, P40 / P41 / P42), the foreign objects it touches are the mirrors of P05 (V90Resampler: state_), P07
 * (V90Parameters: enableEqualizerMmx), P21 (V90SpectralVerifier: flag_), P37 (V90Phase4Demodulator, only its address is passed to resetRRNDetector): member-wise access only.
 *
 * Callees (all RECORDERS with scripted results): V90Equalizer::setLinearEquBeta / setDfeBeta (record the raw argument word, write the step size), V90Resampler::setBllState (writes state_),
 * ResamplerTimingOffset::getTimingOffsetPPM (the blob's: an UNROUNDED extended value in st(0): the recorder returns a long double) and getTimingOffsetPPM_LD (the port's: the same script),
 * V90Phase4Demodulator::resetRRNDetector, edprintf (the formatted text is the record).  convertEqualizerToMmx is called by enterDataPhase INSIDE the object under test (blob: the blob's own).
 *
 * What is compared: the whole equalizer object with the heap blocks it owns (the generated field table), the foreign words, the return value, every record (callee, arguments, and a SNAPSHOT hash
 * of the equalizer + all its arrays + the foreign words at the moment of the call / print: the store order before every call / print), the canary behind the object.  The INDEPENDENT MODEL
 * (written from the archaeology in a different shape, its own extended-precision helpers; it runs first on the LIVE world, which is saved before and restored afterwards, through the same callee
 * environment) must leave the same world, the same result and the same records (exit 3 otherwise); coverage sites in the model (exit 2 if a required site is not reached per scenario).
 *
 * Scenarios (each its own process): 0 enterPhase4 sequences, 1 convertEqualizerToMmx / enterDataPhase sequences, 2 random mixes of all three.
 * Excluded: signalling NaNs in every float (the harness feeds quiet NaNs only: the blob quiets them with fld / fabs / fst anyway, on i386 g++ -O0 as well), the 16-bit tables are only written inside
 * the allocated blocks (the functions write n + 8 shorts), linCoef_ / dfeCoef_ always hold at least one element (the blob reads element 0 even for a zero length).
 */

#include <limits.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../harness/harness.h"
#include "../p05_layout.h"
#include "pcm_parameters_layout.h"
#include "v90_spectral_shaper_layout.h"
#include "v90_phase4_demod_decisions_layout.h"
#include "v90_equalizer_process_layout.h"

#define L(m) __asm__(m)

/* the functions under test */
extern void f_enterP4(void *) L("_ZN12V90Equalizer11enterPhase4Ev");
extern void f_convert(void *) L("_ZN12V90Equalizer21convertEqualizerToMmxEv");
extern int f_enterDP(void *) L("_ZN12V90Equalizer14enterDataPhaseEv");

/* the interposed callees */
extern void c_setLin(void *, float) L("_ZN12V90Equalizer16setLinearEquBetaEf");
extern void c_setDfe(void *, float) L("_ZN12V90Equalizer10setDfeBetaEf");
extern void c_bll(void *, unsigned, unsigned) L("_ZN12V90Resampler11setBllStateE11V90BllStatej");
extern long double c_ppm(const void *) L("_ZNK21ResamplerTimingOffset18getTimingOffsetPPMEv");
extern long double c_ppmLD(const void *) L("_ZNK21ResamplerTimingOffset21getTimingOffsetPPM_LDEv");
extern void c_rrn(void *) L("_ZN20V90Phase4Demodulator16resetRRNDetectorEv");

typedef EquObj T;
#define SZ sizeof(T)
enum { CAN = 64, NSC = 64, LOGMAX = 400, TXTMAX = 320 };

static uint32_t fnv_add(uint32_t h, const void *p, size_t n) { const uint8_t *q = (const uint8_t *)p; while (n--) { h ^= *q++; h *= 16777619u; } return h; }
static uint32_t fbits(float f) { uint32_t b; memcpy(&b, &f, 4); return b; }
static float ffb(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }

/* ------------------------------------------------------------------------------------------------ the world */
typedef struct {
  T *e;
  Ph4DemObj *p4;
  SvObj *sv;
  V90ParObj *par;
  VrObj *rs;
} W;
static W g_w;
static uint8_t *g_can;
static HFld g_fe[128];       /* the equalizer's field table without the pointers to foreign objects */
static int g_nfe;

/* ------------------------------------------------------------------------------------------------ scripts, records */
enum { K_PPM, K_BETA, K_BLL, K_RRN, K_N };
static uint32_t g_sc[K_N][NSC];
static unsigned g_cur[K_N];
static uint32_t sw(int k) { return g_sc[k][g_cur[k]++ % NSC]; }
typedef struct { char t[TXTMAX]; } Rec;
typedef struct { Rec r[LOGMAX]; unsigned n; unsigned long total; } Log;
static Log g_il, g_ml;
static Log *g_log = &g_il;
static int g_show;
static unsigned long g_symctr;

static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}

/* the snapshot: a hash over every scalar of the equalizer, the arrays it owns and the foreign words the functions read / write */
static uint32_t snap_hash(const W *w) {
  uint32_t h = 2166136261u;
  const T *e = w->e;
  int i;
  for (i = 0; i < g_nfe; i++) {
    const HFld *f = &g_fe[i];
    const uint8_t *q = (const uint8_t *)e + f->off;
    switch (f->kind) {
      case 'b': h = fnv_add(h, q, 1); break;
      case 'h': h = fnv_add(h, q, 2); break;
      case 'u': case 'i': case 'f': h = fnv_add(h, q, 4); break;
      case 'o': { void *p; HAl *a; memcpy(&p, q, sizeof p); a = h_find_al(p, 0); if (a) h = fnv_add(h, a->p, a->size); break; }
      default: break;
    }
  }
  h = fnv_add(h, &w->rs->state_, 4); h = fnv_add(h, &w->rs->stateFlag_, 4);
  return h;
}
static void rec_add(Log *lg, const W *w, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2], snap[64];
  va_list ap;
  Rec *r = &lg->r[lg->n < LOGMAX ? lg->n : LOGMAX - 1];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  snprintf(snap, sizeof snap, "|S=%08x", snap_hash(w));
  r->t[0] = kind; r->t[1] = ' '; r->t[2] = 0;
  esc_cat(r->t, sizeof r->t, buf);
  if (strlen(r->t) + strlen(snap) + 1 >= sizeof r->t) h_die3("record text does not fit (%zu + %zu)", strlen(r->t), strlen(snap));
  strcat(r->t, snap);
  lg->n++; lg->total++;
  if (lg->n > LOGMAX) h_die3("more than %d records in one call", LOGMAX);
}
#define CHECK(p, what) do { if ((void *)(p) != (void *)(what)) h_die3("callee %s got an unexpected object %p (expected %p)", __func__, (void *)(p), (void *)(what)); } while (0)

/* ------------------------------------------------------------------------------------------------ the callee environment (ONE implementation for the impl's recorders and the model) */
static uint32_t xs(uint32_t *s) { uint32_t x = *s ? *s : 0x9e3779b9u; x ^= x << 13; x ^= x >> 17; x ^= x << 5; *s = x; return x; }
static uint32_t quiet_bits(uint32_t b) { return ((b & 0x7f800000u) == 0x7f800000u && (b & 0x007fffffu)) ? (b | 0x00400000u) : b; }
#define REC(kind, ...) rec_add(g_log, &g_w, (kind), __VA_ARGS__)
static void env_setLin(uint32_t b) { uint32_t r = sw(K_BETA); REC('c', "setLin(%08x)", b); b = quiet_bits(b); memcpy(&g_w.e->linBeta_, &b, 4); if (r & 1u) g_w.e->linMul_ = (int)(r >> 8) % 77; }
static void env_setDfe(uint32_t b) { uint32_t r = sw(K_BETA); REC('c', "setDfe(%08x)", b); b = quiet_bits(b); memcpy(&g_w.e->dfeBeta_, &b, 4); if (r & 2u) g_w.e->dfeMul_ = (int)(r >> 8) % 77; }
static void env_bll(unsigned st, unsigned fl) { uint32_t r = sw(K_BLL); REC('c', "setBllState(%u,%u)", st, fl); g_w.rs->state_ = st; if (r & 1u) g_w.rs->stateFlag_ = r >> 4; }
static void env_rrn(void) { uint32_t r = sw(K_RRN); REC('c', "resetRRNDetector"); if (r & 1u) g_w.p4->state_ = r >> 8; }
/* the scripted timing offset: an extended value that is NOT a float most of the time (the blob rounds only the first call's value) */
static long double ppm_val(uint32_t a, uint32_t b) {
  switch (a % 24u) {
    case 0: return 0.0L;
    case 1: return -0.0L;
    case 2: return __builtin_nanl("");
    case 3: return -__builtin_nanl("");
    case 4: return __builtin_infl();
    case 5: return -__builtin_infl();
    case 6: return (long double)(int)(b % 2001u) - 1000.0L;
    case 7: return (long double)(int32_t)b * 4294967296.0L;            /* beyond the int range */
    case 8: return (long double)(int32_t)b / 16.0L;                   /* many integers, large */
    case 9: return ((long double)(int32_t)b) / 2147483648.0L;         /* |x| < 1: sign and fraction only */
    case 10: return -(long double)(b % 1000u) / 1048576.0L;           /* tiny negative */
    default: return (long double)(int32_t)b / 1048576.0L;             /* 31 significant bits: rounds when stored as a float */
  }
}
static long double env_ppm(void) { uint32_t a = sw(K_PPM), b = sw(K_PPM); REC('c', "getTimingOffsetPPM"); return ppm_val(a, b); }
static void env_print(const char *fmt, va_list ap) {
  char buf[TXTMAX * 2];
  vsnprintf(buf, sizeof buf, fmt, ap);
  REC('e', "%s", buf);
}

void c_setLin(void *e, float b) { CHECK(e, g_w.e); env_setLin(fbits(b)); }
void c_setDfe(void *e, float b) { CHECK(e, g_w.e); env_setDfe(fbits(b)); }
void c_bll(void *p, unsigned st, unsigned fl) { CHECK(p, g_w.rs); env_bll(st, fl); }
long double c_ppm(const void *p) { CHECK(p, g_w.rs); return env_ppm(); }
long double c_ppmLD(const void *p) { CHECK(p, g_w.rs); return env_ppm(); }
void c_rrn(void *p) { CHECK(p, g_w.p4); env_rrn(); }
void edprintf(const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print(fmt, ap); va_end(ap); }
/* referenced by other blob functions (never called here): a recorder that would show up as a difference if it were */
extern unsigned int dsplibs_debug_level;
int dsplibs_debug_printf(const char *fmt, ...) { char b[TXTMAX * 2]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap); REC('d', "%s", b); return 0; }

/* ------------------------------------------------------------------------------------------------ the independent model */
typedef long double XL;
enum { G_ALL = 7, G_S0 = 1, G_S1 = 2, G_S2 = 4, G_S01 = 3, G_S02 = 5, G_S12 = 6 };   /* the scenarios that must reach a site */
typedef struct { unsigned long n; unsigned char hit, grp; const char *name; } CovSite;
#define CV(g, nm) do { static CovSite cs_ __attribute__((section("covsites"), used, aligned(sizeof(void *)))) = { 0, 0, (g), (nm) }; cs_.hit = 1; } while (0)
extern CovSite __start_covsites[], __stop_covsites[];
static void cov_flush(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) if (c->hit) { c->n++; c->hit = 0; } }
static void cov_clear(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) c->hit = 0; }

static int m_f32(XL v) { if (v != v || v >= 2147483648.0L || v <= -2147483649.0L) return INT_MIN; return (int)v; }
static int m_f16(XL v) { if (v != v || v >= 32768.0L || v <= -32769.0L) return -32768; return (int)v; }
static int m_abs(int v) { return v < 0 ? (int)(0u - (unsigned)v) : v; }
static int m_sgn(XL v) { return (v > 0.0L || v != v) ? '+' : '-'; }
static XL m_fabs(XL v) { __asm__("fabs" : "+t"(v)); return v; }
static XL m_log10(XL x) { XL r; __asm__("fldlg2; fxch %%st(1); fyl2x" : "=t"(r) : "0"(x) : "st(1)"); return r; }
static float m_rf(XL v) { volatile float t = (float)v; return t; }
static void m_pf(const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print(fmt, ap); va_end(ap); }
/* the print of a value: sign of sg, chopped |v|, |chop((v - chop(v)) * scale)| */
static void m_pfix(const char *fmt, XL v, XL sg, XL scale) {
  int ip = m_f32(v);
  int a1 = m_f32(m_fabs(v));
  int fr = m_abs(m_f32((v - (XL)ip) * scale));
  m_pf(fmt, m_sgn(sg), a1, fr);
}

typedef struct { float mx, mn, sum, asum; } Scan;
/* the scan of the report: extremes from element 0, sums from element 1 (floats, rounded after every term) */
static Scan m_scan(const float *c, unsigned n, float *mxSlot, float *mnSlot) {
  Scan s;
  unsigned k;
  float a0 = m_rf(m_fabs((XL)c[0]));
  float mx = a0, mn = a0, sum = 0.0f, asum = 0.0f;
  for (k = 1; k < n; k++) {
    float a = m_rf(m_fabs((XL)c[k]));
    int keepMax = (a <= mx) || a != a || mx != mx;        /* jbe: below-or-equal / unordered */
    int newMin = (a < mn) || a != a || mn != mn;          /* jae not taken: below / unordered */
    if (!keepMax) { CV(G_ALL, "scan: new max"); mx = a; }
    if (newMin) { CV(G_ALL, "scan: new min"); mn = a; }
    if (a != a) CV(G_ALL, "scan: NaN element");
    sum = m_rf((XL)sum + (XL)c[k]);
    asum = m_rf(m_fabs((XL)c[k]) + (XL)asum);
  }
  if (n <= 1) CV(G_ALL, "scan: single element");
  *mxSlot = mx; *mnSlot = mn;
  s.mx = mx; s.mn = mn; s.sum = sum; s.asum = asum;
  return s;
}
static void m_report(T *e) {
  Scan s = m_scan((const float *)e->linCoef_, e->linLen_, &e->linCoefMax_, &e->linCoefMin_);
  m_pf("=======================================================\r\n");
  m_pf("Linear Equalizer:\r\n");
  m_pfix("minLeCoefValue  = %c%d.%010d\r\n", (XL)e->linCoefMin_, (XL)e->linCoefMin_, 10000000000.0L);
  m_pfix("maxLeCoefValue  = %c%d.%06d\r\n", (XL)e->linCoefMax_, (XL)e->linCoefMax_, 1000000.0L);
  m_pfix("coefs sum  = %c%d.%06d\r\n", (XL)s.sum, (XL)s.sum, 1000000.0L);
  m_pfix("abs coefs sum  = %c%d.%06d\r\n", (XL)s.asum, (XL)s.asum, 1000000.0L);
  m_pf("=======================================================\r\n");
  s = m_scan((const float *)e->dfeCoef_, e->dfeLen_, &e->dfeCoefMax_, &e->dfeCoefMin_);
  m_pf("DFE:\r\n");
  m_pfix("minDfeCoefValue  = %c%d.%010d\r\n", (XL)e->dfeCoefMin_, (XL)e->dfeCoefMin_, 10000000000.0L);
  m_pfix("maxDfeCoefValue  = %c%d.%06d\r\n", (XL)e->dfeCoefMax_, (XL)e->dfeCoefMax_, 1000000.0L);
  m_pfix("coefs sum  = %c%d.%06d\r\n", (XL)s.sum, (XL)s.sum, 1000000.0L);
  m_pfix("abs coefs sum  = %c%d.%06d\r\n", (XL)s.asum, (XL)s.asum, 1000000.0L);
  m_pf("=======================================================\r\n");
}

/* one equalizer's worth of the conversion */
typedef struct {
  const char *gainFmt, *outFmt, *betaFmt, *fsumFmt, *fabsFmt, *hsumFmt, *habsFmt, *hminFmt, *hmaxFmt, *histMaxFmt, *histMinFmt;
  unsigned n, hn;
  const float *coef, *hist;
  short *hi, *lo, *hist16;
  float maxAbs, beta;
  XL k;
  float *gain;
  int *div, *mul, *shift;
} Side;
static void m_side_math(Side *s) {
  XL g = (1.0L / (XL)s->maxAbs) * 1073741824.0L;
  XL h = (XL)1.52587890625e-05f * g;
  unsigned i;
  *s->gain = m_rf(g);
  *s->div = m_f32(h);
  if ((XL)s->beta == 0.0L || (XL)s->beta != (XL)s->beta) { CV(G_S12, "convert: beta zero / NaN"); *s->mul = 0; *s->shift = 0; }
  else {
    XL q = (1.0L / ((XL)s->beta * s->k)) * (XL)s->maxAbs;
    XL e2 = m_log10(m_fabs(q)) / m_log10(2.0L);
    int sh = m_f32(e2);
    XL bg = (XL)s->beta * g;
    *s->shift = sh;
    if (sh < 0 || sh > 31) CV(G_S12, "convert: shift outside 0..31"); else CV(G_S12, "convert: shift inside 0..31");
    if (sh == 31) CV(G_S12, "convert: shift 31");
    *s->mul = m_f32(bg * (XL)(int)(1u << (sh & 31)));
  }
  for (i = 0; i < s->n; i++) {
    int v = m_f32((XL)s->coef[i] * g);
    s->lo[i] = (short)(v & 0xffff);
    s->hi[i] = (short)(v >> 16);
  }
}
static void m_side_report(Side *s) {
  unsigned i;
  float fs = 0.0f, fa = 0.0f;
  int isum = 0, iabs = 0, imax = 0, imin = 0x10000, hmax = 0, hmin = 0x8000;
  for (i = 0; i < s->n; i++) {
    int h = s->hi[i], a = m_abs(h);
    isum += h; iabs += a;
    if (a > imax) imax = a;
    if (a < imin) imin = a;
    fs = m_rf((XL)fs + (XL)s->coef[i]);
    fa = m_rf(m_fabs((XL)s->coef[i]) + (XL)fa);
  }
  m_pf("==============================================================\r\n");
  m_pfix(s->gainFmt, (XL)(double)1e-8 * (XL)*s->gain, (XL)*s->gain, 100000.0L);
  m_pf(s->outFmt, m_sgn((XL)*s->div), m_f32(m_fabs((XL)*s->div)), 0);
  m_pf(s->betaFmt, *s->mul, *s->shift);
  m_pf("==============================================================\r\n");
  m_pfix(s->fsumFmt, (XL)fs, (XL)fs, 10000.0L);
  m_pfix(s->fabsFmt, (XL)fa, (XL)fa, 10000.0L);
  m_pf(s->hsumFmt, isum);
  m_pf(s->habsFmt, iabs);
  m_pf(s->hminFmt, imin);
  m_pf(s->hmaxFmt, imax);
  for (i = 0; i < s->hn; i++) {
    int v = m_f16((XL)s->hist[i]);
    int a;
    s->hist16[i] = (short)v;
    a = (short)m_abs((short)v);
    if (v == -32768) CV(G_S12, "history: -32768");
    if (a > hmax) hmax = a;
    if (a < hmin) hmin = a;
  }
  m_pf(s->histMaxFmt, hmax);
  m_pf(s->histMinFmt, hmin);
}
static void m_convert(void) {
  T *e = g_w.e;
  Side a, b;
  if (e->mmx_ != 0) { CV(G_S12, "convert: already converted"); goto fl; }
  if (e->mmxCapable_ == 0) { CV(G_S12, "convert: not capable"); goto fl; }
  if (g_w.par->enableEqualizerMmx == 0) { CV(G_S12, "convert: disabled by the parameters"); goto fl; }
  memset(&a, 0, sizeof a); memset(&b, 0, sizeof b);
  a.gainFmt = "V90Equalizer: linearEquMmxConversionFactor = %c%d.%05de8\r\n";
  a.outFmt = "V90Equalizer: linearEquMmxOutputConversionFactor = %c%d.%03d\r\n";
  a.betaFmt = "V90Equalizer: linearEquBetaMmx = %d, beta exponent = %d\r\n";
  a.fsumFmt = "V90Equalizer: float LE coeffs sum = %c%d.%04d\r\n";
  a.fabsFmt = "V90Equalizer: float LE coeffs abs sum = %c%d.%04d\r\n";
  a.hsumFmt = "V90Equalizer: short high LE coeffs sum = %d\r\n";
  a.habsFmt = "V90Equalizer: short high LE coeffs abs sum = %d\r\n";
  a.hminFmt = "V90Equalizer: short high LE coeffs min value = %d\r\n";
  a.hmaxFmt = "V90Equalizer: short high LE coeffs max value = %d\r\n";
  a.histMaxFmt = "V90Equalizer: Max LE History = %d\r\n";
  a.histMinFmt = "V90Equalizer: Min LE History = %d\r\n";
  a.n = e->linLen_; a.hn = e->linHistLen_; a.coef = (const float *)e->linCoef_; a.hist = (const float *)e->linHist_;
  a.hi = (short *)e->linHi_; a.lo = (short *)e->linLo_; a.hist16 = (short *)e->hist16_;
  a.maxAbs = e->linCoefMax_; a.beta = e->linBeta_; a.k = 16777216.0L;
  a.gain = &e->linMmxGain_; a.div = &e->linDiv_; a.mul = &e->linMul_; a.shift = &e->linShift_;
  b.gainFmt = "V90Equalizer: dfeMmxConversionFactor = %c%d.%05de8\r\n";
  b.outFmt = "V90Equalizer: dfeMmxOutputConversionFactor = %c%d.%03d\r\n";
  b.betaFmt = "V90Equalizer: dfeBetaMmx = %d, beta exponent = %d\r\n";
  b.fsumFmt = "V90Equalizer: float dfe coeffs sum = %c%d.%04d\r\n";
  b.fabsFmt = "V90Equalizer: float dfe coeffs abs sum = %c%d.%04d\r\n";
  b.hsumFmt = "V90Equalizer: short high dfe coeffs sum = %d\r\n";
  b.habsFmt = "V90Equalizer: short high dfe coeffs abs sum = %d\r\n";
  b.hminFmt = "V90Equalizer: short high dfe coeffs min value = %d\r\n";
  b.hmaxFmt = "V90Equalizer: short high dfe coeffs max value = %d\r\n";
  b.histMaxFmt = "V90Equalizer: Max dfe History = %d\r\n";
  b.histMinFmt = "V90Equalizer: Min dfe History = %d\r\n";
  b.n = e->dfeLen_; b.hn = e->dfeLen_; b.coef = (const float *)e->dfeCoef_; b.hist = (const float *)e->dfeHist_;
  b.hi = (short *)e->dfeHi_; b.lo = (short *)e->dfeLo_; b.hist16 = (short *)e->dfeHist16_;
  b.maxAbs = e->dfeCoefMax_; b.beta = e->dfeBeta_; b.k = 1048576.0L;
  b.gain = &e->dfeMmxGain_; b.div = &e->dfeDiv_; b.mul = &e->dfeMul_; b.shift = &e->dfeShift_;
  m_side_math(&a);
  m_side_report(&a);
  e->histPos16_ = e->linPos_;
  m_side_math(&b);
  m_side_report(&b);
  CV(G_S12, "convert: success");
  e->mmx_ = 1;
  m_pf("V90Equalizer: LE & DFE running in MMX mode.\r\n");
  return;
fl:
  m_pf("V90Equalizer: LE & DFE running in FLOAT mode.\r\n");
  e->mmx_ = 0;
}
static void m_enterP4(void) {
  T *e = g_w.e;
  unsigned k;
  float t0;
  int ip, frac, ipa, sg;
  if (e->mode_ == 2) { CV(G_S02, "enterPhase4: already"); return; }
  m_pf("V90Equalizer: enter Phase 4\r\n");
  e->mode_ = 2;
  env_setLin(0); env_setDfe(0);
  e->bllSaved_ = g_w.rs->state_;
  env_bll(0, 1);
  t0 = m_rf(env_ppm());
  ip = m_f32(env_ppm());
  t0 = m_rf((XL)t0 - (XL)ip);
  frac = m_abs(m_f32(10000.0L * (XL)t0));
  { XL v = env_ppm(); ipa = m_f32(m_fabs(v)); }
  { XL v = env_ppm(); sg = m_sgn(v); }
  m_pf("V90Equalizer: timing offset on freeze (phase4) = %c%d.%04d\r\n", sg, ipa, frac);
  if (g_w.sv->flag_ == 2) {
    CV(G_S02, "enterPhase4: german pbx");
    for (k = 0; k < e->dfeLen_; k++) ((float *)e->dfeCoef_)[k] = ffb(0);
    if (e->mmx_) { CV(G_S02, "enterPhase4: 16-bit DFE zeroed"); for (k = 0; k < e->dfeLen_ + 8; k++) { ((short *)e->rawDfeHi_)[k] = 0; ((short *)e->rawDfeLo_)[k] = 0; } }
    m_pf("V90Equalizer: Dfe coefs zeroed!!\r\n");
  }
  e->mseFull_ = 0; e->mseIdx_ = 0;
  m_report(e);
}
static int m_enterDP(void) {
  T *e = g_w.e;
  if (e->mode_ == 3) { CV(G_S12, "enterDataPhase: already"); return 0; }
  m_pf("V90Equalizer: enter Data Phase\r\n");
  e->mode_ = 3;
  env_rrn();
  if (e->mmx_ != 0) { CV(G_S12, "enterDataPhase: already 16-bit"); return 0; }
  m_report(e);
  m_convert();
  return e->mmx_ != 0;
}

/* ------------------------------------------------------------------------------------------------ the world: saved before the model, restored for the impl, compared afterwards */
typedef struct { void *p; size_t n; } Reg;
static Reg g_reg[64];
static int g_nreg;
static uint8_t *g_sv0[64], *g_sv1[64];
static void add_reg(void *p, size_t n) { if (g_nreg >= 64) h_die3("too many regions"); g_reg[g_nreg].p = p; g_reg[g_nreg].n = n; g_nreg++; }
static void build_regions(void) {
  int i;
  W *w = &g_w;
  g_nreg = 0;
  add_reg(w->e, SZ);
  for (i = 0; i < g_nfe; i++) {
    if (g_fe[i].kind == 'o') { void *p; HAl *a; memcpy(&p, (uint8_t *)w->e + g_fe[i].off, sizeof p); a = h_find_al(p, 0); if (!a) h_die3("owned block not found"); add_reg(a->p, a->size); }
  }
  add_reg(w->p4, sizeof(Ph4DemObj)); add_reg(w->sv, sizeof(SvObj)); add_reg(w->par, sizeof(V90ParObj)); add_reg(w->rs, sizeof(VrObj));
}
static void save_to(uint8_t **dst) { int i; for (i = 0; i < g_nreg; i++) { dst[i] = (uint8_t *)realloc(dst[i], g_reg[i].n); memcpy(dst[i], g_reg[i].p, g_reg[i].n); } }
static void restore_from(uint8_t **src) { int i; for (i = 0; i < g_nreg; i++) memcpy(g_reg[i].p, src[i], g_reg[i].n); }
static void log_cmp(void) {
  unsigned i, n = g_ml.n < LOGMAX ? g_ml.n : LOGMAX;
  if (g_ml.n != g_il.n) {
    for (i = 0; i < n && i < g_il.n && i < LOGMAX; i++) fprintf(stderr, "m: %s\ni: %s\n", g_ml.r[i].t, g_il.r[i].t);
    h_die3("%u model records, %u impl records", g_ml.n, g_il.n);
  }
  for (i = 0; i < n; i++) if (strcmp(g_ml.r[i].t, g_il.r[i].t)) h_die3("record %u differs:\n model: %s\n impl:  %s", i, g_ml.r[i].t, g_il.r[i].t);
}
static void world_cmp(void) {
  int i;
  for (i = 0; i < g_nreg; i++) if (memcmp(g_sv1[i], g_reg[i].p, g_reg[i].n)) {
    size_t k;
    for (k = 0; k < g_reg[i].n; k++) if (g_sv1[i][k] != ((uint8_t *)g_reg[i].p)[k]) break;
    h_die3("region %d (%zu bytes) differs from the model at +0x%zx (model %02x, impl %02x)", i, g_reg[i].n, k, g_sv1[i][k], ((uint8_t *)g_reg[i].p)[k]);
  }
}

/* ------------------------------------------------------------------------------------------------ trial setup (floats from integers / 2^k or bit patterns only: arch independent) */
static int g_cls;     /* coefficient class: 0 normal (with specials), 1 clean small, 2 wild exponents, 3 identity / zeros, 4 huge / tiny, 5 negative small */
static float pick_special(uint32_t r) { static const uint32_t b[] = { 0, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x3f800000u, 0xbf800000u, 0x7f800000u, 0xff800000u, 0x00000001u, 0x7f7fffffu }; return ffb(b[r % 10u]); }
static float pick_coef(void) {
  uint32_t r = h_rnd();
  switch (g_cls) {
    case 1: return (float)((int)h_rmod(20001) - 10000) * (1.0f / 4096.0f);
    case 2: { uint32_t v = ((uint32_t)(0x61u + h_rmod(0x3eu)) << 23) | h_rmod(1u << 23); if (h_rpct(50)) v |= 0x80000000u; return ffb(v); }
    case 3: return h_rpct(85) ? 0.0f : (h_rpct(50) ? 1.0f : -2.0f);
    case 4: { uint32_t v = ((uint32_t)(h_rpct(50) ? 0x08u + h_rmod(0x10u) : 0xd0u + h_rmod(0x28u)) << 23) | h_rmod(1u << 23); if (h_rpct(50)) v |= 0x80000000u; return ffb(v); }
    case 5: return -(float)(1 + (int)h_rmod(3000)) * (1.0f / 8192.0f);
    default: if ((r & 0xff) < 6) return pick_special(r >> 8); return (float)((int)h_rmod(20001) - 10000) * (1.0f / 4096.0f);
  }
}
static float pick_hist(void) {
  uint32_t r = h_rnd();
  if ((r & 0xff) < 8) { static const uint32_t b[] = { 0, 0x80000000u, 0x7fc00000u, 0x477fff00u, 0x47800000u, 0xc7000000u, 0xc7000100u, 0x46fffe00u, 0x7f7fffffu, 0x4f000000u, 0xcf000000u }; return ffb(b[(r >> 8) % 11u]); }
  if ((r & 0xff) < 80) return (float)((int)h_rmod(262145u) - 131072) * (1.0f / 2.0f);   /* beyond the 16-bit range */
  return (float)((int)h_rmod(40001) - 20000) * (1.0f / 8.0f) + (float)h_rmod(100) * (1.0f / 128.0f);
}
static uint32_t pick_beta(void) {
  static const uint32_t b[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7f800000u, 0x3f800000u, 0xbf800000u, 0x00000001u, 0x7f7fffffu, 0x3a83126fu, 0x358637bdu, 0x38d1b717u, 0x3727c5acu };
  uint32_t v;
  if (h_rpct(30)) v = b[h_rmod(sizeof b / sizeof b[0])];
  else { v = ((uint32_t)(h_rpct(75) ? 0x42u + h_rmod(0x28u) : 0x60u + h_rmod(0x1cu)) << 23) | h_rmod(1u << 23); if (h_rpct(10)) v |= 0x80000000u; }
  return quiet_bits(v);
}
static float pick_f(void) { uint32_t v = h_rpct(25) ? pick_beta() : ((uint32_t)(0x31u + h_rmod(14)) << 23) | h_rmod(1u << 23); return ffb(quiet_bits(v)); }
static void *new_arr(unsigned bytes) { return sysdep_malloc(bytes ? bytes : 4u); }

static void new_world(int sc) {
  W *w = &g_w;
  T *e;
  unsigned N = h_rpct(60) ? 4u * (1u + h_rmod(4)) : h_rmod(18), M = h_rpct(60) ? 4u * (1u + h_rmod(3)) : h_rmod(14), Lh = N + 2u + 2u * h_rmod(6), k;
  uint8_t *raw = (uint8_t *)h_obj_new(SZ + CAN);
  uintptr_t a;
  (void)sc;
  memset(raw, 0, SZ);
  memset(raw + SZ, 0x5a, CAN);
  w->e = e = (T *)raw; g_can = raw + SZ;
  w->p4 = (Ph4DemObj *)h_obj_new(sizeof(Ph4DemObj));
  w->sv = (SvObj *)h_obj_new(sizeof(SvObj));
  w->par = (V90ParObj *)h_obj_new(sizeof(V90ParObj));
  w->rs = (VrObj *)h_obj_new(sizeof(VrObj));
  e->resampler_ = w->rs; e->p4_ = w->p4; e->sv_ = w->sv; e->params_ = w->par;
  e->p3_ = h_obj_new(64); e->dem_ = h_obj_new(64); e->ce_ = h_obj_new(64); e->pf_ = h_obj_new(64);
  e->linLen_ = N; e->dfeLen_ = M; e->linHistLen_ = Lh;
  e->linCoef_ = new_arr(N * 4); e->linHist_ = new_arr(Lh * 4); e->fadeWinA_ = new_arr(N * 4); e->fadeWinB_ = new_arr(N * 4);
  e->dfeCoef_ = new_arr(M * 4); e->dfeHist_ = new_arr(M * 4); e->mseHist_ = new_arr(0x4b0);
  e->mmxIn_ = new_arr(0x400); e->mmxOut_ = new_arr(0x200);
  e->rawLinHi_ = new_arr((N + 8) * 2); e->rawLinLo_ = new_arr((N + 8) * 2); e->rawHist16_ = new_arr((Lh + 8) * 2);
  e->rawDfeHi_ = new_arr((M + 8) * 2); e->rawDfeLo_ = new_arr((M + 8) * 2); e->rawDfeHist16_ = new_arr((M + 8) * 2);
#define ALIGNED(raw_, al_, off_) a = (uintptr_t)(raw_); a = ((a + 7u) & ~(uintptr_t)7u) - a; (off_) = (uint32_t)(a >> 1); (al_) = (void *)((uint8_t *)(raw_) + (a & ~(uintptr_t)1u))
  ALIGNED(e->rawLinHi_, e->linHi_, e->linHiOff_);
  ALIGNED(e->rawLinLo_, e->linLo_, e->linLoOff_);
  ALIGNED(e->rawHist16_, e->hist16_, e->hist16Off_);
  ALIGNED(e->rawDfeHi_, e->dfeHi_, e->dfeHiOff_);
  ALIGNED(e->rawDfeLo_, e->dfeLo_, e->dfeLoOff_);
  ALIGNED(e->rawDfeHist16_, e->dfeHist16_, e->dfeHist16Off_);
  g_cls = (int)h_rmod(6);
  for (k = 0; k < (N ? N : 1u); k++) ((float *)e->linCoef_)[k] = pick_coef();
  for (k = 0; k < (M ? M : 1u); k++) ((float *)e->dfeCoef_)[k] = pick_coef();
  for (k = 0; k < Lh; k++) ((float *)e->linHist_)[k] = pick_hist();
  for (k = 0; k < (M ? M : 1u); k++) ((float *)e->dfeHist_)[k] = pick_hist();
  for (k = 0; k < N; k++) { ((float *)e->fadeWinA_)[k] = pick_f(); ((float *)e->fadeWinB_)[k] = pick_f(); }
  for (k = 0; k < 300; k++) ((float *)e->mseHist_)[k] = pick_f();
  for (k = 0; k < 512; k++) ((short *)e->mmxIn_)[k] = (short)h_rnd();
  for (k = 0; k < 256; k++) ((short *)e->mmxOut_)[k] = (short)h_rnd();
  for (k = 0; k < N + 8; k++) { ((short *)e->rawLinHi_)[k] = (short)h_rnd(); ((short *)e->rawLinLo_)[k] = (short)h_rnd(); }
  for (k = 0; k < Lh + 8; k++) ((short *)e->rawHist16_)[k] = (short)h_rnd();
  for (k = 0; k < M + 8; k++) { ((short *)e->rawDfeHi_)[k] = (short)h_rnd(); ((short *)e->rawDfeLo_)[k] = (short)h_rnd(); ((short *)e->rawDfeHist16_)[k] = (short)h_rnd(); }
  e->bllSaved_ = h_rnd(); e->w8_ = (uint16_t)h_rnd();
  { uint32_t v = pick_beta(); memcpy(&e->linBeta_, &v, 4); v = pick_beta(); memcpy(&e->dfeBeta_, &v, 4); }
  e->linPos_ = 1u + 2u * h_rmod((Lh - N) / 2u + 1u);
  e->frameCnt_ = h_rnd();
  e->mode_ = h_rpct(30) ? h_rmod(8) : h_rpct(50) ? h_rnd() : 0u;
  e->event_ = h_rnd(); e->pend_ = h_rnd(); e->pendSample_ = ffb(quiet_bits(h_rnd()));
  e->errCnt_ = h_rnd(); e->errLimit_ = h_rnd(); e->errSum_ = h_rnd();
  e->rmsErr_ = pick_f(); e->meanErr_ = pick_f(); e->statMean_ = pick_f(); e->statMax_ = pick_f(); e->statMin_ = pick_f(); e->stat13c_ = pick_f(); e->errK_ = pick_f();
  e->badCnt_ = h_rnd(); e->mseIdx_ = h_rmod(300); e->mseFull_ = h_rnd(); e->mseEnable_ = h_rnd();
  e->mmxCapable_ = h_rpct(85) ? 1u : h_rpct(50) ? 0u : h_rnd();
  e->mmx_ = h_rpct(20) ? (h_rpct(70) ? 1u : h_rnd()) : 0u;
  e->linDiv_ = (int)h_rnd(); e->dfeDiv_ = (int)h_rnd(); e->linMul_ = (int)h_rnd(); e->dfeMul_ = (int)h_rnd(); e->linShift_ = (int)h_rnd(); e->dfeShift_ = (int)h_rnd();
  e->flag144_ = (uint16_t)h_rnd(); e->flag146_ = (uint16_t)h_rnd(); e->flag148_ = h_rnd(); e->fadeLenA_ = h_rnd(); e->fadeLenB_ = h_rnd(); e->ratio_ = pick_f();
  e->timingOffset_ = pick_f();
  e->linCoefMax_ = pick_f(); e->linCoefMin_ = pick_f(); e->linMmxGain_ = pick_f(); e->dfeCoefMax_ = pick_f(); e->dfeCoefMin_ = pick_f(); e->dfeMmxGain_ = pick_f();
  e->histPos16_ = 1u + 2u * h_rmod((Lh - N) / 2u + 1u);
  /* the foreign objects */
  w->p4->state_ = h_rnd();
  w->sv->flag_ = h_rpct(35) ? 2u : h_rpct(80) ? h_rmod(4) : h_rnd();
  w->par->enableEqualizerMmx = h_rpct(85) ? 1 : h_rpct(50) ? 0 : (int32_t)h_rnd();
  w->rs->state_ = h_rpct(70) ? h_rmod(10) : h_rnd(); w->rs->stateFlag_ = h_rnd();
}
static void free_world(void) {
  T *e = g_w.e;
  void *blocks[] = { e->linCoef_, e->linHist_, e->fadeWinA_, e->fadeWinB_, e->dfeCoef_, e->dfeHist_, e->mseHist_, e->mmxIn_, e->mmxOut_, e->rawLinHi_, e->rawLinLo_, e->rawHist16_, e->rawDfeHi_, e->rawDfeLo_,
                     e->rawDfeHist16_ };
  unsigned k;
  for (k = 0; k < sizeof blocks / sizeof blocks[0]; k++) sysdep_free(blocks[k]);
}
static void gen_script(void) {
  unsigned i;
  for (i = 0; i < NSC; i++) { g_sc[K_PPM][i] = h_rnd(); g_sc[K_BETA][i] = h_rnd(); g_sc[K_BLL][i] = h_rnd(); g_sc[K_RRN][i] = h_rnd(); }
  memset(g_cur, 0, sizeof g_cur);
}

/* ------------------------------------------------------------------------------------------------ one call */
static unsigned long g_ncalls;
static int g_showall;
static void dump_foreign(int show) {
  const W *w = &g_w;
  h_emit(show, "p4 st=%u | rs st=%u fl=%u | sv %u | par %d", w->p4->state_, w->rs->state_, w->rs->stateFlag_, w->sv->flag_, (int)w->par->enableEqualizerMmx);
}
static void do_call(int fn) {
  W *w = &g_w;
  T *e = w->e;
  int mret = 0, iret = 0;
  unsigned k;
  /* the function's "already done" early-out is taken in some calls only */
  if (fn == 0 && h_rpct(25)) e->mode_ = 2;
  if (fn == 2 && h_rpct(25)) e->mode_ = 3;
  if ((fn == 0 && e->mode_ == 2 && !h_rpct(50)) || (fn == 2 && e->mode_ == 3 && !h_rpct(50))) e->mode_ = h_rpct(50) ? 0u : 1u + h_rmod(5);
  if (h_rpct(10)) { uint32_t v = pick_beta(); memcpy(&e->linBeta_, &v, 4); }
  if (h_rpct(10)) { uint32_t v = pick_beta(); memcpy(&e->dfeBeta_, &v, 4); }
  if (h_rpct(5)) e->mmx_ = 0;
  gen_script();
  g_show = h_sample(&g_symctr, 12, 250) || g_showall;
  /* the model on the live world first (saved and restored), then the impl */
  build_regions();
  save_to(g_sv0);
  g_log = &g_ml; g_ml.n = 0; memset(g_cur, 0, sizeof g_cur); cov_clear();
  if (fn == 0) m_enterP4(); else if (fn == 1) m_convert(); else mret = m_enterDP();
  save_to(g_sv1);
  restore_from(g_sv0);
  g_log = &g_il; g_il.n = 0; memset(g_cur, 0, sizeof g_cur);
  h_emit(g_show, "call fn=%d mode=%u mmx=%u cap=%u par=%d sv=%u N=%u M=%u L=%u cls=%d", fn, e->mode_, e->mmx_, e->mmxCapable_, (int)w->par->enableEqualizerMmx, w->sv->flag_, e->linLen_, e->dfeLen_, e->linHistLen_, g_cls);
  if (fn == 0) f_enterP4(e); else if (fn == 1) f_convert(e); else iret = f_enterDP(e);
  __asm__ volatile("" ::: "memory");
  log_cmp(); world_cmp();
  if (fn == 2 && iret != mret) h_die3("the result of enterDataPhase: model %d, impl %d", mret, iret);
  cov_flush();
  for (k = 0; k < g_il.n && k < LOGMAX; k++) h_emit(g_show, "%s", g_il.r[k].t);
  h_emit(g_show, "ret %d", fn == 2 ? iret : 0);
  h_dump_obj(g_show, "E", e, g_fe, g_nfe);
  h_wild = 0;
  dump_foreign(g_show);
  { unsigned i; for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("canary byte %u behind the object was overwritten", i); }
  g_ncalls++;
}

static void trial(int sc) {
  unsigned ncalls = sc == 2 ? 2u + h_rmod(4) : 1u + h_rmod(3), k;
  new_world(sc);
  for (k = 0; k < ncalls; k++) {
    int fn = sc == 0 ? 0 : sc == 1 ? (h_rpct(50) ? 1 : 2) : (int)h_rmod(3);
    if (sc == 2 && k == 0 && h_rpct(20)) { g_w.e->mmx_ = 0; g_w.e->mode_ = 2; }
    do_call(fn);
  }
  free_world();
  h_trial_end("p41");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, ntr, i;
  const char *seed = getenv("P41_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P41_ALL")) g_showall = 1;
  p05_layout_check(); p07_layout_check(); p21_layout_check(); p37_layout_check(); p40_layout_check();
  for (i = 0; i < NF_EquObj; i++) {
    const char *nm = f_EquObj[i].name;
    if (!strcmp(nm, "resampler_") || !strcmp(nm, "p3_") || !strcmp(nm, "p4_") || !strcmp(nm, "dem_") || !strcmp(nm, "ce_") || !strcmp(nm, "sv_") || !strcmp(nm, "pf_") || !strcmp(nm, "params_")) continue;
    g_fe[g_nfe++] = f_EquObj[i];
  }
  h_x87_check();
  h_arena_init();
  h_rz_by_id = 1;
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(1);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P41_TRIALS") ? atoi(getenv("P41_TRIALS")) : 6000;
  if (sc < 0 || sc > 2) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  { CovSite *c; int bad = 0;
    for (c = __start_covsites; c < __stop_covsites; c++) {
      if (!(c->grp & (1u << sc))) continue;
      h_emit(1, "COV %s %lu", c->name, c->n);
      if (c->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", c->name); bad = 1; }
    }
    h_cov_finish();
    return (bad && !getenv("P41_NOCOV")) ? 2 : 0; }
}
