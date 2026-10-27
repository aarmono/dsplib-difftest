/*
 * Differential test harness for port chunk P42 (src/vpcm/v90_equalizer.cpp): the other 25 methods of V90Equalizer: C1 / C2, D1 / D2, reset, resetMeanErrorEnergyDiagnostics, setLinearEquBeta,
 * setDfeBeta, freeze, enterPhase3, getDfeBeta, zeroLinearEquCoefs, zeroDfeCoefs, setLinearEquCoeff, setDfeCoeff, loadCoefsFromFile, linearEquFadeEdges, printCoefsToFile,
 * restoreEqualizerToFloat, enterFPE, enterRRN, enterChannelVerification, printEquStuff, calcMeanErrorStatistics, setLinearEquEdgesFadingParams.  Derived from test/vpcm/v90_equalizer_phases.main.c.
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_equalizer.main.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port
 * (-O0 / -O2: the port object alone, the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_equalizer.main.blob32.out).  The functions are reached through __asm__
 * labels; the object is the generated C mirror of test/vpcm/v90_equalizer_process.layout (the WHOLE V90Equalizer), the foreign objects it touches are the mirrors of P05 (V90Resampler: state_ / stateFlag_), P07
 * (V90Parameters + the modem parameter block), P21 (V90SpectralVerifier: flag_), P37 (V90Phase4Demodulator, address only): member-wise access only.
 *
 * Callees (RECORDERS with scripted results): V90Resampler::setBllState (writes state_ / stateFlag_), V90PreFilter::isV90WithEia6, hamming<float> (writes a deterministic pattern into its window),
 * mean / Std / Var<float> (scripted floats), edprintf (the formatted text is the record).  setLinearEquBeta / setDfeBeta / reset / restoreEqualizerToFloat are the REAL functions of the blob / the
 * port (they are called by the other methods; their own effects, prints and records are part of every comparison); sysdep_malloc / sysdep_free are the harness allocator (malloc log).
 *
 * What is compared: the whole equalizer object with the heap blocks it owns (the generated field table), the foreign words, the return value, every record (callee, arguments, and a SNAPSHOT
 * hash of the equalizer + all its arrays + the foreign words at the moment of the call / print: the store order before every call / print), the canary behind the object, the malloc / free log
 * (hashed into the HASH line) and, for the constructor, which members were already stored at every malloc (H_CTOR_PROBE).  The INDEPENDENT MODEL (written from the archaeology in a different shape,
 * its own extended-precision helpers; it runs first on the LIVE world, which is saved before and restored afterwards, through the same callee environment) must leave the same world, the same result
 * and the same records (exit 3 otherwise); coverage sites in the model (exit 2 if a required site is not reached per scenario).  The constructor is checked against invariants (block sizes, order, the
 * result is a fixed point of the model's reset(linLen_ / 2), capability rule).
 *
 * Scenarios (each its own process): 0 constructor / destructor (C1 / C2 / D1 / D2), 1 state methods (betas, freeze, phase entries, zero / set coefficients, FPE / RRN, restore), 2 data methods
 * (fade edges, fade parameters, reset, mean error statistics), 3 random mixes of all methods, 4 lifecycle (constructor, random calls, destructor).
 * Excluded: signalling NaNs in every float EXCEPT the raw copies of setLinearEquCoeff / setDfeCoeff (the blob quiets them with fld / fst; g++ -O0 on i386 would too); the 16-bit tables are only
 * written inside the allocated blocks (the functions write n + 8 shorts); linear length 0 with the constructor (the identity tap would be written into a zero-length block); calcMeanErrorStatistics with
 * no history returns an uninitialised stack float (its return value is not compared); a history length shorter than the linear length (the blob wraps).
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
extern void f_c1(void *, unsigned, unsigned, void *, void *, void *, void *, void *, void *, void *, void *, unsigned)
    L("_ZN12V90EqualizerC1EjjP20V90Phase3DemodulatorP20V90Phase4DemodulatorP11V90DemapperP22V90ConnectionEvaluatorP19V90SpectralVerifierP13V90ParametersP12V90ResamplerP12V90PreFilter20V90ComputationalMode");
extern void f_c2(void *, unsigned, unsigned, void *, void *, void *, void *, void *, void *, void *, void *, unsigned)
    L("_ZN12V90EqualizerC2EjjP20V90Phase3DemodulatorP20V90Phase4DemodulatorP11V90DemapperP22V90ConnectionEvaluatorP19V90SpectralVerifierP13V90ParametersP12V90ResamplerP12V90PreFilter20V90ComputationalMode");
extern void f_d1(void *) L("_ZN12V90EqualizerD1Ev");
extern void f_d2(void *) L("_ZN12V90EqualizerD2Ev");
extern void f_reset(void *, unsigned) L("_ZN12V90Equalizer5resetEj");
extern void f_resetMee(void *) L("_ZN12V90Equalizer31resetMeanErrorEnergyDiagnosticsEv");
extern void f_setLin(void *, float) L("_ZN12V90Equalizer16setLinearEquBetaEf");
extern void f_setDfe(void *, float) L("_ZN12V90Equalizer10setDfeBetaEf");
extern void f_freeze(void *) L("_ZN12V90Equalizer6freezeEv");
extern void f_enter3(void *) L("_ZN12V90Equalizer11enterPhase3Ev");
extern float f_getDfe(void *) L("_ZN12V90Equalizer10getDfeBetaEv");
extern void f_zeroLin(void *) L("_ZN12V90Equalizer18zeroLinearEquCoefsEv");
extern void f_zeroDfe(void *) L("_ZN12V90Equalizer12zeroDfeCoefsEv");
extern void f_setLinC(void *, float *, unsigned) L("_ZN12V90Equalizer17setLinearEquCoeffEPfj");
extern void f_setDfeC(void *, float *, unsigned) L("_ZN12V90Equalizer11setDfeCoeffEPfj");
extern void f_load(void *) L("_ZN12V90Equalizer17loadCoefsFromFileEv");
extern void f_fade(void *) L("_ZN12V90Equalizer18linearEquFadeEdgesEv");
extern void f_print(const void *) L("_ZNK12V90Equalizer16printCoefsToFileEv");
extern void f_restore(void *) L("_ZN12V90Equalizer23restoreEqualizerToFloatEv");
extern int f_fpe(void *) L("_ZN12V90Equalizer8enterFPEEv");
extern int f_rrn(void *) L("_ZN12V90Equalizer8enterRRNEv");
extern void f_cv(void *) L("_ZN12V90Equalizer24enterChannelVerificationEv");
extern void f_printEqu(void *) L("_ZN12V90Equalizer13printEquStuffEv");
extern float f_calc(void *) L("_ZN12V90Equalizer23calcMeanErrorStatisticsEv");
extern void f_setFade(void *, float, float) L("_ZN12V90Equalizer29setLinearEquEdgesFadingParamsEff");

/* the interposed callees */
extern void c_bll(void *, unsigned, unsigned) L("_ZN12V90Resampler11setBllStateE11V90BllStatej");
extern int c_eia6(const void *) L("_ZNK12V90PreFilter13isV90WithEia6Ev");
extern void c_ham(float *, unsigned) L("_Z7hammingIfEvPT_j");
extern float c_mean(float *, unsigned) L("_Z4meanIfET_PS0_j");
extern float c_std(float *, unsigned) L("_Z3StdIfET_PS0_j");
extern float c_var(float *, unsigned) L("_Z3VarIfET_PS0_j");

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
  ModParObj *mp;
  VrObj *rs;
  void *p3, *dem, *ce, *pf;
} W;
static W g_w;
static uint8_t *g_can;
static HFld g_fe[128];       /* the equalizer's field table without the pointers to foreign objects */
static int g_nfe;
#define MP_MODE(mp) (*(int32_t *)((uint8_t *)(mp) + 0x5c))   /* _tagModemParameters +0x5c: not named in P07 */

/* ------------------------------------------------------------------------------------------------ scripts, records */
enum { K_BLL, K_EIA, K_STAT, K_N };
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
static uint32_t quiet_bits(uint32_t b) { return ((b & 0x7f800000u) == 0x7f800000u && (b & 0x007fffffu)) ? (b | 0x00400000u) : b; }
#define REC(kind, ...) rec_add(g_log, &g_w, (kind), __VA_ARGS__)
static void env_bll(unsigned st, unsigned fl) { uint32_t r = sw(K_BLL); REC('c', "setBllState(%u,%u)", st, fl); g_w.rs->state_ = st; if (r & 1u) g_w.rs->stateFlag_ = r >> 4; }
static int env_eia6(void) { uint32_t r = sw(K_EIA); REC('c', "isV90WithEia6"); return (int)(r & 1u); }
static void env_ham(float *w, unsigned n) {
  unsigned i;
  const char *nm = (void *)w == g_w.e->fadeWinA_ ? "A" : (void *)w == g_w.e->fadeWinB_ ? "B" : "?";
  REC('c', "hamming(%s,%u)", nm, n);
  for (i = 0; i < n; i++) w[i] = (float)(i + 1u) * 0.25f + (float)n;
}
static float stat_val(uint32_t r, uint32_t r2) {
  switch (r % 14u) {
    case 0: return 0.0f;
    case 1: return ffb(0x7fc00000u);
    case 2: return ffb(0x7f800000u);
    case 3: return ffb(0xff800000u);
    case 4: return -(float)(r2 % 5000u) * (1.0f / 64.0f);
    case 5: return ffb(0x00000001u);
    case 6: return ffb(0x80000000u);
    case 12: return ffb(0xcf000000u + r2 % 9u);          /* just below -2^31: the print's fraction is a large negative int */
    case 13: return ffb(0x4f000000u + r2 % 9u);
    default: return (float)(r2 % 100000u) * (1.0f / 1024.0f);
  }
}
static float env_stat(const char *nm, unsigned n) { uint32_t a = sw(K_STAT), b = sw(K_STAT); REC('c', "%s(%u)", nm, n); return stat_val(a, b); }
static void env_print(const char *fmt, va_list ap) {
  char buf[TXTMAX * 2];
  vsnprintf(buf, sizeof buf, fmt, ap);
  REC('e', "%s", buf);
}

void c_bll(void *p, unsigned st, unsigned fl) { CHECK(p, g_w.rs); env_bll(st, fl); }
int c_eia6(const void *p) { CHECK(p, g_w.pf); return env_eia6(); }
void c_ham(float *w, unsigned n) { env_ham(w, n); }
float c_mean(float *p, unsigned n) { CHECK(p, g_w.e->mseHist_); return env_stat("mean", n); }
float c_std(float *p, unsigned n) { CHECK(p, g_w.e->mseHist_); return env_stat("Std", n); }
float c_var(float *p, unsigned n) { CHECK(p, g_w.e->mseHist_); return env_stat("Var", n); }
void edprintf(const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print(fmt, ap); va_end(ap); }
/* referenced by other blob functions (never called here): a recorder that would show up as a difference if it were */
extern unsigned int dsplibs_debug_level;
int dsplibs_debug_printf(const char *fmt, ...) { char b[TXTMAX * 2]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap); REC('d', "%s", b); return 0; }

/* ------------------------------------------------------------------------------------------------ the independent model */
typedef long double XL;
enum { S0 = 1, S1 = 2, S2 = 4, S3 = 8, S4 = 16 };   /* the scenarios that must reach a site */
#define G_ALL 31
#define G_MIX (S1 | S3)       /* state methods: scenarios 1 and 3 */
#define G_MIX2 (S2 | S3)      /* data methods: scenarios 2 and 3 */
#define G_BETA (S1 | S3)      /* the beta setters are reached in 1 and 3 (and in reset, 2) */
#define G_RESET (S2 | S3)
#define G_CTOR (S0 | S4)
typedef struct { unsigned long n; unsigned char hit, grp; const char *name; } CovSite;
#define CV(g, nm) do { static CovSite cs_ __attribute__((section("covsites"), used, aligned(sizeof(void *)))) = { 0, 0, (g), (nm) }; cs_.hit = 1; } while (0)
extern CovSite __start_covsites[], __stop_covsites[];
static void cov_flush(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) if (c->hit) { c->n++; c->hit = 0; } }
static void cov_clear(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) c->hit = 0; }

static int m_f32(XL v) { if (v != v || v >= 2147483648.0L || v <= -2147483649.0L) return INT_MIN; return (int)v; }
static int m_abs(int v) { return v < 0 ? (int)(0u - (unsigned)v) : v; }
static int m_sgn(XL v) { return (v > 0.0L || v != v) ? '+' : '-'; }
static XL m_fabs(XL v) { __asm__("fabs" : "+t"(v)); return v; }
static XL m_log10(XL x) { XL r; __asm__("fldlg2; fxch %%st(1); fyl2x" : "=t"(r) : "0"(x) : "st(1)"); return r; }
static float m_rf(XL v) { volatile float t = (float)v; return t; }
static XL m_mul(XL a, XL b) { XL r = a * b; __asm__("" : "+t"(r)); return r; }   /* x87 product: no SSE narrowing (NaN operand rule) */
static uint32_t m_lo64(XL v) { if (!(v > -9223372036854775809.0L && v < 9223372036854775808.0L)) return 0; return (uint32_t)(uint64_t)(int64_t)v; }
static void m_pf(const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print(fmt, ap); va_end(ap); }
/* the print of a value: sign of sg, chopped |v|, |chop((v - chop(v)) * scale)| */
static void m_pfix(const char *fmt, XL v, XL sg, XL scale) {
  int ip = m_f32(v);
  int a1 = m_f32(m_fabs(v));
  int fr = m_abs(m_f32((v - (XL)ip) * scale));
  m_pf(fmt, m_sgn(sg), a1, fr);
}
static void m_pfl(const char *fmt, float v) { m_pfix(fmt, (XL)v, (XL)v, 1000000.0L); }

#define S_LEBETA "V90Equalizer: LE Beta = %c%d.%05de-10\r\n"
#define S_DFEBETA "V90Equalizer: DFE Beta = %c%d.%05de-7\r\n"

/* setLinearEquBeta (dfe = 0) / setDfeBeta (dfe = 1): the beta arrives as a raw word */
static void m_setBeta(int dfe, uint32_t bits) {
  T *e = g_w.e;
  float *bp = dfe ? &e->dfeBeta_ : &e->linBeta_;
  float nb = ffb(bits), ob = *bp;
  if (ob < nb || ob > nb) { CV(G_BETA, "beta: print"); m_pfix(dfe ? S_DFEBETA : S_LEBETA, (XL)(dfe ? 10000000.0f : 10000000000.0f) * (XL)nb, (XL)nb, 100000.0L); }
  else CV(G_BETA, "beta: equal / unordered: no print");
  memcpy(bp, &bits, 4);
  if (e->mmx_ != 0) {
    int *mul = dfe ? &e->dfeMul_ : &e->linMul_;
    int *sh = dfe ? &e->dfeShift_ : &e->linShift_;
    XL b = (XL)nb, mx = (XL)(dfe ? e->dfeCoefMax_ : e->linCoefMax_), k = dfe ? 1048576.0L : 16777216.0L;
    XL g = (XL)(dfe ? e->dfeMmxGain_ : e->linMmxGain_);
    if (b == 0.0L || b != b) { CV(G_BETA, "beta: mmx zero / NaN"); *mul = 0; *sh = 0; }
    else {
      XL q = mx / (b * k);
      XL e2 = m_log10(m_fabs(q)) / m_log10(2.0L);
      int s = m_f32(e2);
      CV(G_BETA, "beta: mmx multiplier");
      if (s < 0 || s > 31) CV(G_BETA, "beta: shift outside 0..31"); else CV(G_BETA, "beta: shift inside 0..31");
      if (s == 31) CV(G_BETA, "beta: shift 31");
      *sh = s;
      *mul = m_f32((b * g) * (XL)(int)(1u << (s & 31)));
    }
  } else CV(G_BETA, "beta: float mode");
}
static void m_zeroDfeBlocks(T *e, unsigned with16) {
  unsigned k;
  for (k = 0; k < e->dfeLen_; k++) ((uint32_t *)e->dfeCoef_)[k] = 0;
  if (with16) for (k = 0; k < e->dfeLen_ + 8u; k++) { ((int16_t *)e->rawDfeHi_)[k] = 0; ((int16_t *)e->rawDfeLo_)[k] = 0; }
}
static void m_zeroLin(void) {
  T *e = g_w.e;
  unsigned k;
  for (k = 0; k < e->linLen_; k++) ((float *)e->linCoef_)[k] = 0.0f;
  if (e->mmx_) { CV(G_MIX, "zeroLin: 16 bit"); for (k = 0; k < e->linLen_ + 8u; k++) { ((int16_t *)e->rawLinHi_)[k] = 0; ((int16_t *)e->rawLinLo_)[k] = 0; } }
  else CV(G_MIX, "zeroLin: float");
}
static void m_zeroDfe(void) {
  T *e = g_w.e;
  if (e->mmx_) CV(G_MIX, "zeroDfe: 16 bit"); else CV(G_MIX, "zeroDfe: float");
  m_zeroDfeBlocks(e, e->mmx_ != 0);
}
static float m_unpack(int16_t hi, int16_t lo, float gain) {
  int v = (int)hi * 65536 + (int)(uint16_t)lo;
  return m_rf((1.0L / (XL)gain) * (XL)v);
}
static void m_restore(void) {
  T *e = g_w.e;
  unsigned k;
  if (e->mmx_ == 0) { CV(G_MIX, "restore: already float"); return; }
  CV(G_MIX, "restore: 16 bit");
  for (k = 0; k < e->linLen_; k++) ((float *)e->linCoef_)[k] = m_unpack(((int16_t *)e->linHi_)[k], ((int16_t *)e->linLo_)[k], e->linMmxGain_);
  for (k = 0; k < e->linHistLen_; k++) ((float *)e->linHist_)[k] = (float)((int16_t *)e->hist16_)[k];
  e->linPos_ = e->histPos16_;
  for (k = 0; k < e->dfeLen_; k++) {
    ((float *)e->dfeCoef_)[k] = m_unpack(((int16_t *)e->dfeHi_)[k], ((int16_t *)e->dfeLo_)[k], e->dfeMmxGain_);
    ((float *)e->dfeHist_)[k] = (float)((int16_t *)e->dfeHist16_)[k];
  }
  m_pf("V90Equalizer: Equalizer restored to FLOAT mode.\r\n");
  e->mmx_ = 0;
}
static int m_state(unsigned mode, const char *txt) {
  T *e = g_w.e;
  int eia;
  if (e->mode_ == mode) { CV(G_MIX, "state: already"); return 0; }
  m_pf("%s", txt);
  e->mode_ = mode;
  if (g_w.sv->flag_ == 2) {
    uint32_t wd;
    CV(G_MIX, "state: german pbx");
    m_zeroDfeBlocks(e, e->mmx_ != 0);
    if (e->mmx_) CV(G_MIX, "state: german pbx 16 bit");
    m_pf("V90Equalizer: Dfe coefs zeroed (GERMAN_PBX) !!\r\n");
    memcpy(&wd, &g_w.par->germanPbxDfeTrn2dFastBeta, 4);
    m_setBeta(1, wd);
  }
  eia = env_eia6();
  if (eia) {
    CV(G_MIX, "state: eia6");
    m_setBeta(0, 0); m_setBeta(1, 0);
    m_pf("V90Equalizer: Dfe & Linear EQU coefs freezed (RRN : EIA6 )!!\r\n");
  }
  if (e->mmx_ == 0) { CV(G_MIX, "state: float, nothing to restore"); return 0; }
  CV(G_MIX, "state: restore");
  m_pf("V90Equalizer: restoring Equalizer to float...\r\n");
  m_restore();
  e->mmx_ = 0;
  return 1;
}
static void m_enter3(void) {
  T *e = g_w.e;
  if (e->mode_ == 1) { CV(G_MIX, "enter3: already"); return; }
  m_pf("V90Equalizer: enter Phase 3\r\n");
  m_setBeta(0, 0); m_setBeta(1, 0);
  e->mode_ = 1; e->event_ = 0;
}
static void m_cv(void) {
  T *e = g_w.e;
  if (e->mode_ == 6) { CV(G_MIX, "cv: already"); return; }
  m_pf("V90Equalizer: enterChannelVerification\r\n");
  m_setBeta(0, 0); m_setBeta(1, 0);
  e->mode_ = 6; e->event_ = 0;
  env_bll(11, 1);
}
static void m_fade(void) {
  T *e = g_w.e;
  unsigned i, n = e->linLen_;
  float *c = (float *)e->linCoef_;
  int m16 = e->mmx_ != 0;
  if (m16) {
    CV(G_MIX2, "fade: 16 bit");
    for (i = 0; i < n; i++) c[i] = m_unpack(((int16_t *)e->linHi_)[i], ((int16_t *)e->linLo_)[i], e->linMmxGain_);
    for (i = 0; i < e->dfeLen_; i++) ((float *)e->dfeCoef_)[i] = m_unpack(((int16_t *)e->dfeHi_)[i], ((int16_t *)e->dfeLo_)[i], e->dfeMmxGain_);
  } else CV(G_MIX2, "fade: float");
  for (i = 0; i < e->fadeLenA_; i++) c[i] = m_rf(m_mul((XL)((float *)e->fadeWinA_)[i], (XL)c[i]));
  for (i = 0; i < e->fadeLenB_; i++) c[n - 1u - i] = m_rf(m_mul((XL)((float *)e->fadeWinB_)[i], (XL)c[n - 1u - i]));
  if (e->fadeLenA_ || e->fadeLenB_) CV(G_MIX2, "fade: windows applied");
  if (m16) {
    for (i = 0; i < n; i++) {
      int v = m_f32((XL)c[i] * (XL)e->linMmxGain_);
      ((int16_t *)e->linLo_)[i] = (int16_t)(v & 0xffff);
      ((int16_t *)e->linHi_)[i] = (int16_t)(v >> 16);
    }
  }
}
static XL m_clamp(float x) {
  XL v = (XL)x;
  if (v != v || v < 0.0L) { CV(G_MIX2, "clamp: negative / NaN"); return 0.0L; }
  if (v > 0.5L) { CV(G_MIX2, "clamp: above 0.5"); return 0.5L; }
  CV(G_MIX2, "clamp: in range");
  return v;
}
static void m_setFade(float left, float right) {
  T *e = g_w.e;
  XL n = (XL)e->linLen_, l = m_clamp(left), r = m_clamp(right);
  e->fadeLenA_ = m_lo64(l * n);
  e->fadeLenB_ = m_lo64(r * n);
  env_ham((float *)e->fadeWinA_, 2u * e->fadeLenA_);
  env_ham((float *)e->fadeWinB_, 2u * e->fadeLenB_);
}
static void m_reset(unsigned arg) {
  T *e = g_w.e;
  unsigned k, n = e->linLen_, h = e->linHistLen_, m = e->dfeLen_;
  uint32_t w;
  m_pf("V90Equalizer: reset\r\n");
  e->w8_ = 0;
  w = 0x283424dcu; memcpy(&e->linBeta_, &w, 4); memcpy(&e->dfeBeta_, &w, 4);
  e->mmx_ = 0;
  w = 0; memcpy(&e->linCoefMax_, &w, 4); memcpy(&e->linCoefMin_, &w, 4); memcpy(&e->dfeCoefMax_, &w, 4); memcpy(&e->dfeCoefMin_, &w, 4);
  m_setBeta(0, 0); m_setBeta(1, 0);
  e->mode_ = 0; e->event_ = 0;
  e->linPos_ = h - n - 1u;
  for (k = 0; k < n; k++) { ((float *)e->linCoef_)[k] = 0.0f; ((float *)e->linHist_)[h - 1u - k] = 0.0f; }
  if (n - 1u < arg) { CV(G_RESET, "reset: identity tap clamped"); arg = n - 1u; } else CV(G_RESET, "reset: identity tap as asked");
  ((float *)e->linCoef_)[arg] = 1.0f;
  for (k = 0; k < m; k++) { ((float *)e->dfeHist_)[k] = 0.0f; ((float *)e->dfeCoef_)[k] = 0.0f; }
  if (e->mmxCapable_) {
    CV(G_RESET, "reset: 16 bit tables");
    for (k = 0; k < n + 8u; k++) { ((int16_t *)e->rawLinHi_)[k] = 0; ((int16_t *)e->rawLinLo_)[k] = 0; }
    for (k = 0; k < h + 8u; k++) ((int16_t *)e->rawHist16_)[k] = 0;
    for (k = 0; k < m + 8u; k++) { ((int16_t *)e->rawDfeHi_)[k] = 0; ((int16_t *)e->rawDfeLo_)[k] = 0; ((int16_t *)e->rawDfeHist16_)[k] = 0; }
  } else CV(G_RESET, "reset: not capable");
  w = 0;
  memcpy(&e->pendSample_, &w, 4); memcpy(&e->rmsErr_, &w, 4); memcpy(&e->meanErr_, &w, 4); memcpy(&e->statMean_, &w, 4); memcpy(&e->statMin_, &w, 4); memcpy(&e->statMax_, &w, 4);
  memcpy(&e->stat13c_, &w, 4); memcpy(&e->ratio_, &w, 4);
  memcpy(&e->errK_, &g_w.par->errorEnergyMeanK, 4);
  e->pend_ = 0; e->errCnt_ = 0; e->errLimit_ = (uint32_t)g_w.par->errorEnergyMeanBlockLen; e->mseIdx_ = 0; e->errSum_ = 0; e->mseFull_ = 0; e->mseEnable_ = 0; e->badCnt_ = 0;
  e->flag144_ = 1; e->flag146_ = 1; e->frameCnt_ = 0;
  m_setFade(g_w.par->linearEquFadeLeftEdgeRatio, g_w.par->linearEquFadeRightEdgeRatio);
}
static float m_calc(int *valid) {
  T *e = g_w.e;
  unsigned n, i;
  float mean, sd, vr, mx, mn;
  *valid = 1;
  if (e->mseIdx_ == 0 && e->mseFull_ == 0) { CV(G_MIX2, "calc: no history"); *valid = 0; return 0.0f; }
  n = e->mseFull_ ? 300u : e->mseIdx_;
  if (e->mseFull_) CV(G_MIX2, "calc: wrapped history"); else CV(G_MIX2, "calc: partial history");
  mean = env_stat("mean", n); e->statMean_ = mean;
  sd = env_stat("Std", n);
  vr = env_stat("Var", n);
  mx = mn = ((float *)e->mseHist_)[0];
  e->statMax_ = mx; e->statMin_ = mn;
  if (n <= 1) CV(G_MIX2, "calc: one entry");
  for (i = 1; i < n; i++) {
    float a = ((float *)e->mseHist_)[i];
    if (a > mx) { CV(G_MIX2, "calc: new max"); mx = a; e->statMax_ = a; }
    if (!(a >= mn)) { CV(G_MIX2, "calc: new min"); mn = a; e->statMin_ = a; }
    if (a != a) CV(G_MIX2, "calc: NaN entry");
  }
  m_pf("####################################################\r\n");
  m_pf("V90Equalizer: meanErrorEnergy Debug:\r\n");
  m_pf("V90Equalizer: calculated over %d mean errors\r\n", (int)n);
  m_pf("--------------------------------------------\r\n");
  m_pfl("V90Equalizer: meanErrorEnergy mean  = %c%d.%06d\r\n", e->statMean_);
  m_pfl("V90Equalizer: current meanErrorEnergy  = %c%d.%06d\r\n", e->meanErr_);
  m_pf("--------------------------------------------\r\n");
  m_pfl("V90Equalizer: meanErrorEnergy Std  = %c%d.%06d\r\n", sd);
  m_pfl("V90Equalizer: meanErrorEnergy Variance  = %c%d.%06d\r\n", vr);
  m_pf("--------------------------------------------\r\n");
  m_pfl("V90Equalizer: meanErrorEnergy min value  = %c%d.%06d\r\n", e->statMin_);
  m_pfl("V90Equalizer: meanErrorEnergy max value  = %c%d.%06d\r\n", e->statMax_);
  m_pf("####################################################\r\n");
  return sd;
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
    if (g_fe[i].kind == 'o') { void *p; HAl *a; memcpy(&p, (uint8_t *)w->e + g_fe[i].off, sizeof p); if (!p) continue; a = h_find_al(p, 1); if (!a) { if (w->e->mmxCapable_ == 0) continue; h_die3("owned block not found"); } add_reg(a->p, a->size); }
  }
  add_reg(w->sv, sizeof(SvObj)); add_reg(w->par, sizeof(V90ParObj)); add_reg(w->rs, sizeof(VrObj));
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
static float pick_nan(void) { uint32_t v = 0x7fc00000u | h_rmod(1u << 22); if (h_rpct(50)) v |= 0x80000000u; return ffb(v); }   /* a quiet NaN with a random payload (the x87 picks the larger significand of two NaNs) */
static float pick_special(uint32_t r) { if (r % 17u == 0) return pick_nan(); static const uint32_t b[] = { 0, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x3f800000u, 0xbf800000u, 0x7f800000u, 0xff800000u, 0x00000001u, 0x7f7fffffu }; return ffb(b[r % 10u]); }
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
  if ((r & 0xff) < 80) return (float)((int)h_rmod(262145u) - 131072) * (1.0f / 2.0f);
  return (float)((int)h_rmod(40001) - 20000) * (1.0f / 8.0f) + (float)h_rmod(100) * (1.0f / 128.0f);
}
static uint32_t pick_beta(void) {
  static const uint32_t b[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7f800000u, 0x3f800000u, 0xbf800000u, 0x00000001u, 0x7f7fffffu, 0x3a83126fu, 0x358637bdu, 0x38d1b717u, 0x3727c5acu };
  uint32_t v;
  if (h_rpct(30)) v = b[h_rmod(sizeof b / sizeof b[0])];
  else { v = ((uint32_t)(h_rpct(75) ? 0x42u + h_rmod(0x28u) : 0x60u + h_rmod(0x1cu)) << 23) | h_rmod(1u << 23); if (h_rpct(10)) v |= 0x80000000u; }
  return quiet_bits(v);
}
static float pick_f(void) { if (h_rpct(4)) return pick_nan(); uint32_t v = h_rpct(25) ? pick_beta() : ((uint32_t)(0x31u + h_rmod(14)) << 23) | h_rmod(1u << 23); return ffb(quiet_bits(v)); }
/* the fade ratios: around 0 .. 0.5 and beyond, negative, NaN, inf */
static float pick_ratio(void) {
  static const uint32_t b[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0x3f000000u, 0x3effffffu, 0x3f000001u, 0xbf000000u, 0x3f800000u, 0x7f800000u, 0xff800000u, 0x00000001u, 0x3e800000u, 0x3d000000u, 0x7f7fffffu };
  if (h_rpct(35)) return ffb(b[h_rmod(sizeof b / sizeof b[0])]);
  if (h_rpct(10)) return -(float)h_rmod(100) * (1.0f / 128.0f);
  return (float)h_rmod(70) * (1.0f / 128.0f);
}
static float pick_gain(void) {
  if (h_rpct(12)) return ffb(pick_beta());
  return (float)(1 + (int)h_rmod(1u << 20)) * (float)(1u << (h_rmod(8) + 10));
}
static void *new_arr(unsigned bytes) { return sysdep_malloc(bytes ? bytes : 4u); }

static void new_foreign(void) {
  W *w = &g_w;
  w->p4 = (Ph4DemObj *)h_obj_new(sizeof(Ph4DemObj));
  w->sv = (SvObj *)h_obj_new(sizeof(SvObj));
  w->par = (V90ParObj *)h_obj_new(sizeof(V90ParObj));
  w->mp = (ModParObj *)h_obj_new(sizeof(ModParObj));
  w->rs = (VrObj *)h_obj_new(sizeof(VrObj));
  w->p3 = h_obj_new(64); w->dem = h_obj_new(64); w->ce = h_obj_new(64); w->pf = h_obj_new(64);
  w->par->params = (void *)w->mp;
  w->p4->state_ = h_rnd();
  w->sv->flag_ = h_rpct(35) ? 2u : h_rpct(80) ? h_rmod(4) : h_rnd();
  w->par->enableEqualizerMmx = h_rpct(85) ? 1 : h_rpct(50) ? 0 : (int32_t)h_rnd();
  w->par->germanPbxDfeTrn2dFastBeta = ffb(pick_beta());
  w->par->errorEnergyMeanK = pick_f();
  w->par->errorEnergyMeanBlockLen = (int32_t)h_rnd();
  w->par->linearEquFadeLeftEdgeRatio = pick_ratio();
  w->par->linearEquFadeRightEdgeRatio = pick_ratio();
  MP_MODE(w->mp) = h_rpct(60) ? (int32_t)h_rmod(4) : (int32_t)h_rnd();
  w->rs->state_ = h_rpct(70) ? h_rmod(10) : h_rnd(); w->rs->stateFlag_ = h_rnd();
}
static uint8_t *new_obj_mem(void) {
  uint8_t *raw = (uint8_t *)h_obj_new(SZ + CAN);
  memset(raw, 0, SZ);
  memset(raw + SZ, 0x5a, CAN);
  g_w.e = (T *)raw; g_can = raw + SZ;
  return raw;
}
static void fill_scalars(T *e, unsigned N, unsigned M, unsigned Lh) {
  e->bllSaved_ = h_rnd(); e->w8_ = (uint16_t)h_rnd();
  { uint32_t v = pick_beta(); memcpy(&e->linBeta_, &v, 4); v = pick_beta(); memcpy(&e->dfeBeta_, &v, 4); }
  e->linPos_ = 1u + 2u * h_rmod((Lh - N) / 2u + 1u);
  e->frameCnt_ = h_rnd();
  e->mode_ = h_rpct(30) ? h_rmod(8) : h_rpct(50) ? h_rnd() : 0u;
  e->event_ = h_rnd(); e->pend_ = h_rnd(); e->pendSample_ = ffb(quiet_bits(h_rnd()));
  e->errCnt_ = h_rnd(); e->errLimit_ = h_rnd(); e->errSum_ = h_rnd();
  e->rmsErr_ = pick_f(); e->meanErr_ = h_rpct(8) ? ffb(0xcf000000u + h_rmod(9)) : pick_f(); e->statMean_ = pick_f(); e->statMax_ = pick_f(); e->statMin_ = pick_f(); e->stat13c_ = pick_f(); e->errK_ = pick_f();
  e->badCnt_ = h_rnd(); e->mseIdx_ = h_rpct(15) ? 0u : h_rpct(15) ? 1u : h_rmod(300); e->mseFull_ = h_rpct(40) ? 0u : h_rnd(); e->mseEnable_ = h_rnd();
  e->mmxCapable_ = h_rpct(85) ? 1u : h_rpct(50) ? 0u : h_rnd();
  e->mmx_ = h_rpct(45) ? (h_rpct(80) ? 1u : h_rnd()) : 0u;
  e->linDiv_ = (int)h_rnd(); e->dfeDiv_ = (int)h_rnd(); e->linMul_ = (int)h_rnd(); e->dfeMul_ = (int)h_rnd(); e->linShift_ = (int)h_rnd(); e->dfeShift_ = (int)h_rnd();
  e->flag144_ = (uint16_t)h_rnd(); e->flag146_ = (uint16_t)h_rnd(); e->flag148_ = h_rnd(); e->ratio_ = pick_f();
  e->fadeLenA_ = h_rmod(N + 1u); e->fadeLenB_ = h_rmod(N + 1u);
  e->timingOffset_ = pick_f();
  e->linCoefMax_ = pick_f(); e->linCoefMin_ = pick_f(); e->linMmxGain_ = pick_gain(); e->dfeCoefMax_ = pick_f(); e->dfeCoefMin_ = pick_f(); e->dfeMmxGain_ = pick_gain();
  e->histPos16_ = 1u + 2u * h_rmod((Lh - N) / 2u + 1u);
  (void)M;
}
/* random contents of every table the equalizer owns (the 16-bit tables only when they exist) */
static unsigned alloc_elems(const void *p) { HAl *a = h_find_al(p, 1); return a ? a->size / 4u : 0u; }
static void fill_data(T *e) {
  unsigned N = e->linLen_, M = e->dfeLen_, Lh = e->linHistLen_, k;
  unsigned nl = N ? N : (alloc_elems(e->linCoef_) ? 1u : 0u), nd = M ? M : (alloc_elems(e->dfeCoef_) ? 1u : 0u);
  g_cls = (int)h_rmod(6);
  for (k = 0; k < nl; k++) ((float *)e->linCoef_)[k] = pick_coef();
  for (k = 0; k < nd; k++) ((float *)e->dfeCoef_)[k] = pick_coef();
  for (k = 0; k < Lh; k++) ((float *)e->linHist_)[k] = pick_hist();
  for (k = 0; k < nd; k++) ((float *)e->dfeHist_)[k] = pick_hist();
  for (k = 0; k < N; k++) { ((float *)e->fadeWinA_)[k] = pick_f(); ((float *)e->fadeWinB_)[k] = pick_f(); }
  { int zc = h_rpct(6), bg = h_rpct(6);      /* classes: all +-0 (a maximum that is replaced by an equal value of the other sign), values just below -2^31 */
    for (k = 0; k < 300; k++)
      ((float *)e->mseHist_)[k] = zc ? ffb(h_rpct(50) ? 0u : 0x80000000u) : bg ? ffb(0xcf000000u + h_rmod(9)) : h_rpct(10) ? ffb(quiet_bits(pick_beta())) : pick_f();
  }
  if (e->mmxIn_) for (k = 0; k < 512; k++) ((short *)e->mmxIn_)[k] = (short)h_rnd();
  if (e->mmxOut_) for (k = 0; k < 256; k++) ((short *)e->mmxOut_)[k] = (short)h_rnd();
  if (e->rawLinHi_) for (k = 0; k < N + 8; k++) { ((short *)e->rawLinHi_)[k] = (short)h_rnd(); ((short *)e->rawLinLo_)[k] = (short)h_rnd(); }
  if (e->rawHist16_) for (k = 0; k < Lh + 8; k++) ((short *)e->rawHist16_)[k] = (short)h_rnd();
  if (e->rawDfeHi_) for (k = 0; k < M + 8; k++) { ((short *)e->rawDfeHi_)[k] = (short)h_rnd(); ((short *)e->rawDfeLo_)[k] = (short)h_rnd(); ((short *)e->rawDfeHist16_)[k] = (short)h_rnd(); }
}
/* the aligned 16-bit pointers of the hand-built world: raw + 0 .. 3 shorts (the functions only read / write n + 8 shorts from the raw block) */
static void set_aligned(int16_t **al, uint32_t *off, void *raw) { unsigned o = h_rpct(50) ? 0u : h_rmod(4); *off = o; *al = (int16_t *)raw + o; }
static void new_hand_world(void) {
  W *w = &g_w;
  T *e;
  unsigned N = h_rpct(60) ? 4u * (1u + h_rmod(4)) : h_rmod(18), M = h_rpct(60) ? 4u * (1u + h_rmod(3)) : h_rmod(14), Lh = N + 2u + 2u * h_rmod(6), k;
  new_obj_mem();
  e = w->e;
  new_foreign();
  e->resampler_ = w->rs; e->p4_ = w->p4; e->sv_ = w->sv; e->params_ = w->par;
  e->p3_ = w->p3; e->dem_ = w->dem; e->ce_ = w->ce; e->pf_ = w->pf;
  e->linLen_ = N; e->dfeLen_ = M; e->linHistLen_ = Lh;
  e->linCoef_ = new_arr(N * 4); e->linHist_ = new_arr(Lh * 4); e->fadeWinA_ = new_arr(N * 4); e->fadeWinB_ = new_arr(N * 4);
  e->dfeCoef_ = new_arr(M * 4); e->dfeHist_ = new_arr(M * 4); e->mseHist_ = new_arr(0x4b0);
  e->mmxIn_ = new_arr(0x400); e->mmxOut_ = new_arr(0x200);
  e->rawLinHi_ = new_arr((N + 8) * 2); e->rawLinLo_ = new_arr((N + 8) * 2); e->rawHist16_ = new_arr((Lh + 8) * 2);
  e->rawDfeHi_ = new_arr((M + 8) * 2); e->rawDfeLo_ = new_arr((M + 8) * 2); e->rawDfeHist16_ = new_arr((M + 8) * 2);
  set_aligned((int16_t **)&e->linHi_, &e->linHiOff_, e->rawLinHi_);
  set_aligned((int16_t **)&e->linLo_, &e->linLoOff_, e->rawLinLo_);
  set_aligned((int16_t **)&e->hist16_, &e->hist16Off_, e->rawHist16_);
  set_aligned((int16_t **)&e->dfeHi_, &e->dfeHiOff_, e->rawDfeHi_);
  set_aligned((int16_t **)&e->dfeLo_, &e->dfeLoOff_, e->rawDfeLo_);
  set_aligned((int16_t **)&e->dfeHist16_, &e->dfeHist16Off_, e->rawDfeHist16_);
  fill_data(e);
  fill_scalars(e, N, M, Lh);
}
static void free_world(void) {
  T *e = g_w.e;
  void *blocks[] = { e->linCoef_, e->linHist_, e->fadeWinA_, e->fadeWinB_, e->dfeCoef_, e->dfeHist_, e->mseHist_, e->mmxIn_, e->mmxOut_, e->rawLinHi_, e->rawLinLo_, e->rawHist16_, e->rawDfeHi_, e->rawDfeLo_,
                     e->rawDfeHist16_ };
  unsigned k;
  for (k = 0; k < sizeof blocks / sizeof blocks[0]; k++) sysdep_free(blocks[k]);
}
static void gen_script(void) {
  unsigned i, k;
  for (k = 0; k < K_N; k++) for (i = 0; i < NSC; i++) g_sc[k][i] = h_rnd();
  memset(g_cur, 0, sizeof g_cur);
}

/* ------------------------------------------------------------------------------------------------ one call */
enum { F_SETLIN, F_SETDFE, F_FREEZE, F_ENTER3, F_CV, F_GETDFE, F_RESETMEE, F_ZLIN, F_ZDFE, F_SETLINC, F_SETDFEC, F_LOAD, F_PRINT, F_PRINTEQU, F_FPE, F_RRN, F_RESTORE,
       F_FADE, F_SETFADE, F_RESET, F_CALC, F_N };
static const char *const fnames[F_N] = { "setLinearEquBeta", "setDfeBeta", "freeze", "enterPhase3", "enterChannelVerification", "getDfeBeta", "resetMeanErrorEnergyDiagnostics", "zeroLinearEquCoefs",
  "zeroDfeCoefs", "setLinearEquCoeff", "setDfeCoeff", "loadCoefsFromFile", "printCoefsToFile", "printEquStuff", "enterFPE", "enterRRN", "restoreEqualizerToFloat", "linearEquFadeEdges",
  "setLinearEquEdgesFadingParams", "reset", "calcMeanErrorStatistics" };
typedef struct { uint32_t a, b; unsigned n; float *src; unsigned srcN; } Args;
static unsigned long g_ncalls;
static int g_showall;
static unsigned long g_fcnt[F_N];
static void dump_foreign(int show) {
  const W *w = &g_w;
  h_emit(show, "p4 st=%u | rs st=%u fl=%u | sv %u | par %d", w->p4->state_, w->rs->state_, w->rs->stateFlag_, w->sv->flag_, (int)w->par->enableEqualizerMmx);
}

static int model_call(int fn, const Args *a, uint32_t *ret, int *rvalid) {
  T *e = g_w.e;
  unsigned i;
  *ret = 0; *rvalid = 0;
  switch (fn) {
    case F_SETLIN: m_setBeta(0, a->a); break;
    case F_SETDFE: m_setBeta(1, a->a); break;
    case F_FREEZE: m_setBeta(0, 0); m_setBeta(1, 0); break;
    case F_ENTER3: m_enter3(); break;
    case F_CV: m_cv(); break;
    case F_GETDFE: { memcpy(ret, &e->dfeBeta_, 4); *rvalid = 1; break; }
    case F_RESETMEE: e->mseIdx_ = 0; e->mseFull_ = 0; break;
    case F_ZLIN: m_zeroLin(); break;
    case F_ZDFE: m_zeroDfe(); break;
    case F_SETLINC: for (i = 0; i < a->n; i++) memcpy((uint8_t *)e->linCoef_ + 4u * i, a->src + i, 4); break;
    case F_SETDFEC: for (i = 0; i < a->n; i++) memcpy((uint8_t *)e->dfeCoef_ + 4u * i, a->src + i, 4); break;
    case F_LOAD: case F_PRINT: case F_PRINTEQU: break;
    case F_FPE: *ret = (uint32_t)m_state(5u, "V90Equalizer: enter FPE state\r\n"); *rvalid = 1; break;
    case F_RRN: *ret = (uint32_t)m_state(4u, "V90Equalizer: enter RRN state\r\n"); *rvalid = 1; break;
    case F_RESTORE: m_restore(); break;
    case F_FADE: m_fade(); break;
    case F_SETFADE: m_setFade(ffb(a->a), ffb(a->b)); break;
    case F_RESET: m_reset(a->a); break;
    case F_CALC: { int v; float r = m_calc(&v); *ret = fbits(r); *rvalid = v; break; }
    default: h_die3("unknown function %d", fn);
  }
  return 0;
}
static void impl_call(int fn, const Args *a, uint32_t *ret, int *rvalid) {
  T *e = g_w.e;
  *ret = 0; *rvalid = 0;
  switch (fn) {
    case F_SETLIN: f_setLin(e, ffb(a->a)); break;
    case F_SETDFE: f_setDfe(e, ffb(a->a)); break;
    case F_FREEZE: f_freeze(e); break;
    case F_ENTER3: f_enter3(e); break;
    case F_CV: f_cv(e); break;
    case F_GETDFE: *ret = fbits(f_getDfe(e)); *rvalid = 1; break;
    case F_RESETMEE: f_resetMee(e); break;
    case F_ZLIN: f_zeroLin(e); break;
    case F_ZDFE: f_zeroDfe(e); break;
    case F_SETLINC: f_setLinC(e, a->src, a->n); break;
    case F_SETDFEC: f_setDfeC(e, a->src, a->n); break;
    case F_LOAD: f_load(e); break;
    case F_PRINT: f_print(e); break;
    case F_PRINTEQU: f_printEqu(e); break;
    case F_FPE: *ret = (uint32_t)f_fpe(e); *rvalid = 1; break;
    case F_RRN: *ret = (uint32_t)f_rrn(e); *rvalid = 1; break;
    case F_RESTORE: f_restore(e); break;
    case F_FADE: f_fade(e); break;
    case F_SETFADE: f_setFade(e, ffb(a->a), ffb(a->b)); break;
    case F_RESET: f_reset(e, a->a); break;
    case F_CALC: { float r = f_calc(e); *ret = fbits(r); *rvalid = 1; break; }
    default: h_die3("unknown function %d", fn);
  }
}
static void pick_args(int fn, Args *a) {
  T *e = g_w.e;
  unsigned i;
  memset(a, 0, sizeof *a);
  switch (fn) {
    case F_SETLIN: case F_SETDFE: {
      a->a = pick_beta();
      if (h_rpct(10)) a->a = fbits(fn == F_SETLIN ? e->linBeta_ : e->dfeBeta_);
      else if (h_rpct(8)) {                  /* the print's value K * beta just beyond the int range: chop(v) = INT_MIN, the fraction (v - INT_MIN) * 1e5 is a large negative int */
        XL v = 2147483648.0L + (XL)h_rmod(30000) + (XL)h_rmod(1000) / 1000.0L;
        if (h_rpct(70)) v = -v;
        a->a = fbits((float)(v / (fn == F_SETLIN ? 10000000000.0L : 10000000.0L)));
      }
      else if (e->mmx_ && h_rpct(40)) {      /* a beta that gives a shift of -3 .. 36 for the current maximum (31 is the boundary) */
        float mx = fn == F_SETLIN ? e->linCoefMax_ : e->dfeCoefMax_;
        XL k = fn == F_SETLIN ? 16777216.0L : 1048576.0L, pw = 1.0L, fr = 1.0L + (XL)h_rmod(100) / 128.0L;
        int sh = (int)h_rmod(40) - 3, j;
        if (mx == mx && mx != 0.0f && mx - mx == 0.0f) {
          for (j = 0; j < (sh < 0 ? -sh : sh); j++) pw *= 2.0L;
          if (sh < 0) pw = 1.0L / pw;
          a->a = fbits((float)((XL)(mx < 0 ? -mx : mx) / (k * pw * fr)));
          if (h_rpct(10)) a->a ^= 0x80000000u;
        }
      }
      break;
    }
    case F_SETLINC: case F_SETDFEC: {
      unsigned lim = fn == F_SETLINC ? e->linLen_ : e->dfeLen_;
      a->n = h_rmod(lim + 1u);
      a->srcN = a->n ? a->n : 1u;
      a->src = (float *)h_obj_new(4u * a->srcN);
      for (i = 0; i < a->srcN; i++) { uint32_t w = h_rnd(); if (h_rpct(10)) w |= 0x7f800000u; memcpy(&a->src[i], &w, 4); }
      break;
    }
    case F_SETFADE: a->a = fbits(pick_ratio()); a->b = fbits(pick_ratio()); break;
    case F_RESET: a->a = h_rpct(50) ? h_rmod(e->linLen_ + 1u) : h_rpct(50) ? e->linLen_ / 2u : h_rpct(50) ? e->linLen_ : h_rnd(); if (e->linLen_ == 0) a->a = 0; else if (a->a >= e->linLen_ && h_rpct(60)) a->a = h_rmod(e->linLen_ + 3u); break;
    default: break;
  }
}
static void do_call(int fn) {
  W *w = &g_w;
  T *e = w->e;
  Args a;
  uint32_t mret, iret;
  int mv, iv;
  unsigned k;
  /* the functions' "already done" early-outs are taken in some calls only */
  if ((fn == F_ENTER3 || fn == F_CV || fn == F_FPE || fn == F_RRN) && h_rpct(25)) e->mode_ = fn == F_ENTER3 ? 1u : fn == F_CV ? 6u : fn == F_FPE ? 5u : 4u;
  if (h_rpct(10)) { uint32_t v = pick_beta(); memcpy(&e->linBeta_, &v, 4); }
  if (h_rpct(10)) { uint32_t v = pick_beta(); memcpy(&e->dfeBeta_, &v, 4); }
  if (h_rpct(8) && e->mmxCapable_ != 0 && e->rawLinHi_ != 0) e->mmx_ = h_rpct(50);
  if (fn == F_RESET && e->linHistLen_ < e->linLen_ + 1u) h_die3("harness: history shorter than the linear length");
  pick_args(fn, &a);
  if (getenv("P42_DBG") && fn == F_FADE) { unsigned q; for (q = 0; q < e->linLen_; q++) fprintf(stderr, "c[%u]=%08x wA=%08x wB=%08x\n", q, fbits(((float *)e->linCoef_)[q]), fbits(((float *)e->fadeWinA_)[q]), fbits(((float *)e->fadeWinB_)[q])); fprintf(stderr, "lenA=%u lenB=%u\n", e->fadeLenA_, e->fadeLenB_); }
  gen_script();
  g_show = h_sample(&g_symctr, 12, 250) || g_showall;
  build_regions();
  save_to(g_sv0);
  g_log = &g_ml; g_ml.n = 0; memset(g_cur, 0, sizeof g_cur); cov_clear();
  model_call(fn, &a, &mret, &mv);
  save_to(g_sv1);
  restore_from(g_sv0);
  g_log = &g_il; g_il.n = 0; memset(g_cur, 0, sizeof g_cur);
  h_emit(g_show, "call %s mode=%u mmx=%u cap=%u par=%d sv=%u N=%u M=%u L=%u a=%08x b=%08x n=%u", fnames[fn], e->mode_, e->mmx_, e->mmxCapable_, (int)w->par->enableEqualizerMmx, w->sv->flag_, e->linLen_,
         e->dfeLen_, e->linHistLen_, a.a, a.b, a.n);
  impl_call(fn, &a, &iret, &iv);
  __asm__ volatile("" ::: "memory");
  log_cmp(); world_cmp();
  if (mv && iv && mret != iret) h_die3("%s: result: model %08x, impl %08x", fnames[fn], mret, iret);
  cov_flush();
  for (k = 0; k < g_il.n && k < LOGMAX; k++) h_emit(g_show, "%s", g_il.r[k].t);
  if (mv) h_emit(g_show, "ret %08x", iret); else h_emit(g_show, "ret (undefined)");
  if (mv) h_emit(0, "retv %08x", iret);
  h_dump_obj(g_show, "E", e, g_fe, g_nfe);
  dump_foreign(g_show);
  { unsigned i; for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("canary byte %u behind the object was overwritten", i); }
  if (fn == F_SETLINC || fn == F_SETDFEC) {     /* the raw copy kept the signalling NaNs (compared above); arithmetic on them is excluded: quiet them for the following calls */
    uint32_t *c = (uint32_t *)(fn == F_SETLINC ? e->linCoef_ : e->dfeCoef_);
    for (k = 0; k < a.n; k++) c[k] = quiet_bits(c[k]);
  }
  g_ncalls++; g_fcnt[fn]++;
}

/* ------------------------------------------------------------------------------------------------ the constructor / destructor */
static int g_c2;
static void ctor_inputs(unsigned *lin, unsigned *dfe, unsigned *mode) {
  *lin = h_rpct(60) ? 4u * (1u + h_rmod(5)) : 4u + h_rmod(20);          /* 4 .. 23: > 0 (a zero length would write the identity tap into a zero-length block) */
  if (h_rpct(10)) *lin = 4u + h_rmod(4);
  *dfe = h_rpct(60) ? 4u * h_rmod(5) : h_rmod(20);                         /* 0 .. 19: lengths below 4 give 0 */
  *mode = h_rpct(40) ? 1u : h_rpct(40) ? 2u : h_rpct(50) ? 0u : h_rnd();
}
static void expect_block(const T *e, const void *p, unsigned size, unsigned *lastId, const char *nm) {
  HAl *a = h_find_al(p, 1);
  if (!a || a->p != (const uint8_t *)p) h_die3("ctor: %s is not a heap block", nm);
  if (a->size != size) h_die3("ctor: %s has size %u, expected %u", nm, a->size, size);
  if (a->id <= *lastId) h_die3("ctor: %s allocated out of order (id %u after %u)", nm, a->id, *lastId);
  *lastId = a->id;
  (void)e;
}
static void ctor_check(unsigned lin, unsigned dfe, unsigned mode, int hl, int sc0) {
  T *e = g_w.e;
  unsigned N = (lin >> 2) * 4u, M = (dfe >> 2) * 4u, Lh = (unsigned)(hl / 2) * 2u, id = 0;
  int cap = 0;
  if (g_w.par->enableEqualizerMmx != 0) cap = mode == 1u ? MP_MODE(g_w.mp) != 2 : MP_MODE(g_w.mp) == 1;
  if (e->linLen_ != N || e->dfeLen_ != M || e->linHistLen_ != Lh) h_die3("ctor: lengths %u %u %u, expected %u %u %u", e->linLen_, e->dfeLen_, e->linHistLen_, N, M, Lh);
  if ((e->mmxCapable_ != 0) != cap) h_die3("ctor: capability %u, expected %d", e->mmxCapable_, cap);
  if (e->mmxCapable_ > 1) h_die3("ctor: capability flag %u", e->mmxCapable_);
  expect_block(e, e->linCoef_, N * 4u, &id, "linCoef_");
  expect_block(e, e->linHist_, Lh * 4u, &id, "linHist_");
  expect_block(e, e->fadeWinA_, N * 4u, &id, "fadeWinA_");
  expect_block(e, e->fadeWinB_, N * 4u, &id, "fadeWinB_");
  expect_block(e, e->dfeCoef_, M * 4u, &id, "dfeCoef_");
  expect_block(e, e->dfeHist_, M * 4u, &id, "dfeHist_");
  if (cap) {
    CV(G_CTOR, "ctor: 16-bit tables");
    expect_block(e, e->mmxOut_, 0x200u, &id, "mmxOut_");
    expect_block(e, e->mmxIn_, 0x400u, &id, "mmxIn_");
    expect_block(e, e->rawLinHi_, (N + 8u) * 2u, &id, "rawLinHi_");
    expect_block(e, e->rawLinLo_, (N + 8u) * 2u, &id, "rawLinLo_");
    expect_block(e, e->rawHist16_, (Lh + 8u) * 2u, &id, "rawHist16_");
    expect_block(e, e->rawDfeHi_, (M + 8u) * 2u, &id, "rawDfeHi_");
    expect_block(e, e->rawDfeLo_, (M + 8u) * 2u, &id, "rawDfeLo_");
    expect_block(e, e->rawDfeHist16_, (M + 8u) * 2u, &id, "rawDfeHist16_");
    {
      struct { void *raw, *al; uint32_t off; const char *nm; } t[6] = { { e->rawLinHi_, e->linHi_, e->linHiOff_, "linHi_" }, { e->rawLinLo_, e->linLo_, e->linLoOff_, "linLo_" },
        { e->rawHist16_, e->hist16_, e->hist16Off_, "hist16_" }, { e->rawDfeHi_, e->dfeHi_, e->dfeHiOff_, "dfeHi_" }, { e->rawDfeLo_, e->dfeLo_, e->dfeLoOff_, "dfeLo_" },
        { e->rawDfeHist16_, e->dfeHist16_, e->dfeHist16Off_, "dfeHist16_" } };
      int i;
      for (i = 0; i < 6; i++) {
        uintptr_t r = (uintptr_t)t[i].raw, al = (uintptr_t)t[i].al;
        if (al != r + 2u * t[i].off || (al & 7u) != 0 || al < r || al - r > 7u) h_die3("ctor: %s is not the 8-byte aligned pointer into its raw block", t[i].nm);
        if (t[i].off != 0) CV(G_CTOR, "ctor: a raw block not 8-byte aligned (nonzero offset)"); else CV(G_CTOR, "ctor: a raw block already aligned");
      }
    }
  } else CV(G_CTOR, "ctor: no 16-bit tables");
  expect_block(e, e->mseHist_, 0x4b0u, &id, "mseHist_");
  if (!cap && sc0 && (e->mmxIn_ != (void *)(uintptr_t)0x1230 || e->rawDfeHist16_ != (void *)(uintptr_t)0x12a0)) h_die3("ctor: a 16-bit pointer was changed without the capability");
  if (!cap && !sc0 && e->mmxIn_ != 0) h_die3("ctor: 16-bit pointer stored without the capability");
}
static void world_fixpoint(void) {
  /* the constructor ends in reset(linLen_ / 2): a second reset(linLen_ / 2) by the MODEL must leave the world unchanged */
  T *e = g_w.e;
  static Log keep;
  build_regions();
  save_to(g_sv0);
  keep = g_il;
  g_log = &g_ml; g_ml.n = 0; memset(g_cur, 0, sizeof g_cur); gen_script();
  m_reset(e->linLen_ / 2u);
  save_to(g_sv1);
  restore_from(g_sv0);
  g_log = &g_il; g_il = keep;
  world_cmp();
  CV(G_CTOR, "ctor: result is a fixed point of the model's reset");
}
static void ctor_trial(int sc) {
  W *w = &g_w;
  unsigned lin, dfe, mode, Hh, k, ncalls;
  int hl, c2 = (int)h_rpct(30), d2 = (int)h_rpct(30);
  new_obj_mem();
  new_foreign();
  ctor_inputs(&lin, &dfe, &mode);
  hl = (int)(((lin >> 2) * 4u) + h_rmod(30));      /* the history length parameter: >= the linear length (the blob wraps otherwise) */
  if (h_rpct(10)) hl = (int)((lin >> 2) * 4u) + 1;
  w->par->linearEquHistoryLength = hl;
  Hh = (unsigned)(hl / 2) * 2u;
  if (Hh < ((lin >> 2) * 4u) + 0u) h_die3("harness: history shorter than the linear length");
  /* untouched members: their (deterministic) start values must survive the constructor */
  w->e->bllSaved_ = 0x5a5a1234u; w->e->flag148_ = 0xa5a5u; w->e->pad0a_[0] = 0xa5; w->e->pad0a_[1] = 0x5a; w->e->pad138_[3] = 0x77; w->e->timingOffset_ = ffb(0x3f9e0000u);
  w->e->mmxCapable_ = 0x5a5a5a5au;      /* the constructor always stores the flag (0 or 1) */
  if (sc == 0) {                         /* garbage (non-heap) pointers in the 16-bit table members: a destructor of a non-capable object must not free them */
    w->e->mmxIn_ = (void *)(uintptr_t)0x1230; w->e->mmxOut_ = (void *)(uintptr_t)0x1240; w->e->rawLinHi_ = (void *)(uintptr_t)0x1250; w->e->rawLinLo_ = (void *)(uintptr_t)0x1260;
    w->e->rawHist16_ = (void *)(uintptr_t)0x1270; w->e->rawDfeHi_ = (void *)(uintptr_t)0x1280; w->e->rawDfeLo_ = (void *)(uintptr_t)0x1290; w->e->rawDfeHist16_ = (void *)(uintptr_t)0x12a0;
  }
  gen_script();
  g_log = &g_il; g_il.n = 0; memset(g_cur, 0, sizeof g_cur);
  g_show = h_sample(&g_symctr, 12, 250) || g_showall;
  h_emit(g_show, "ctor %s lin=%u dfe=%u mode=%u hl=%d par=%d mp5c=%d", c2 ? "C2" : "C1", lin, dfe, mode, hl, (int)w->par->enableEqualizerMmx, (int)MP_MODE(w->mp));
  cov_clear();
  if (c2) { H_CTOR_PROBE(w->e, g_fe, g_nfe, SZ, f_c2(w->e, lin, dfe, w->p3, w->p4, w->dem, w->ce, w->sv, w->par, w->rs, w->pf, mode)); }
  else { H_CTOR_PROBE(w->e, g_fe, g_nfe, SZ, f_c1(w->e, lin, dfe, w->p3, w->p4, w->dem, w->ce, w->sv, w->par, w->rs, w->pf, mode)); }
  __asm__ volatile("" ::: "memory");
  if (w->e->resampler_ != w->rs || w->e->p3_ != w->p3 || w->e->p4_ != w->p4 || w->e->dem_ != w->dem || w->e->ce_ != w->ce || w->e->sv_ != w->sv || w->e->params_ != w->par || w->e->pf_ != w->pf)
    h_die3("ctor: a foreign pointer was not stored");
  if (w->e->bllSaved_ != 0x5a5a1234u || w->e->flag148_ != 0xa5a5u || w->e->pad0a_[0] != 0xa5 || w->e->pad0a_[1] != 0x5a || w->e->pad138_[3] != 0x77 || fbits(w->e->timingOffset_) != 0x3f9e0000u)
    h_die3("ctor: a member the constructor does not set was changed");
  ctor_check(lin, dfe, mode, hl, sc == 0);
  world_fixpoint();
  if (w->e->mmxCapable_ != 0) CV(G_CTOR, "ctor: capable"); else CV(G_CTOR, "ctor: not capable");
  if (mode == 1u) CV(G_CTOR, "ctor: mode 1"); else CV(G_CTOR, "ctor: other mode");
  if (hl & 1) CV(G_CTOR, "ctor: odd history length"); else CV(G_CTOR, "ctor: even history length");
  if (c2) CV(G_CTOR, "ctor: C2"); else CV(G_CTOR, "ctor: C1");
  if (w->par->enableEqualizerMmx == 0) CV(G_CTOR, "ctor: disabled by the parameters");
  if (w->e->dfeLen_ == 0) CV(G_CTOR, "ctor: no DFE");
  if (w->par->enableEqualizerMmx != 0 && mode == 1u && MP_MODE(w->mp) == 2) CV(G_CTOR, "ctor: mode 1 with cpu mode 2");
  if (w->par->enableEqualizerMmx != 0 && mode != 1u && MP_MODE(w->mp) == 1) CV(G_CTOR, "ctor: other mode with cpu mode 1");
  if (w->par->enableEqualizerMmx != 0 && mode != 1u && MP_MODE(w->mp) != 1) CV(G_CTOR, "ctor: other mode without cpu mode 1");
  {
    unsigned i;
    for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  }
  h_dump_obj(g_show, "E", w->e, g_fe, g_nfe);
  h_wild = 0;
  dump_foreign(g_show);
  cov_flush();
  ncalls = sc == 4 ? 1u + h_rmod(4) : 0u;
  if (sc == 4) {                      /* random contents and state on top of the constructed object (capability and the pointers stay) */
    unsigned cap = w->e->mmxCapable_;
    fill_data(w->e);
    fill_scalars(w->e, w->e->linLen_, w->e->dfeLen_, w->e->linHistLen_);
    w->e->mmxCapable_ = cap;
    if (!cap) w->e->mmx_ = 0;
  }
  for (k = 0; k < ncalls; k++) {
    int fn = (int)h_rmod(F_N);
    if (fn == F_RESET && w->e->linHistLen_ < w->e->linLen_ + 1u) fn = F_FADE;
    do_call(fn);
  }
  /* the destructor: frees in the blob's order, clears nothing; the object is printed afterwards (-flifetime-dse) */
  g_il.n = 0;
  if (h_rpct(35)) {        /* release one owned block by hand and clear its pointer: the destructor must not call sysdep_free for a NULL pointer */
    int idx[32], n = 0, q;
    for (q = 0; q < g_nfe; q++) if (g_fe[q].kind == 'o') { void *pp; memcpy(&pp, (uint8_t *)w->e + g_fe[q].off, sizeof pp); if (pp && h_find_al(pp, 0)) idx[n++] = q; }
    if (n > 0) {
      void *pp, *z = 0;
      q = idx[h_rmod((unsigned)n)];
      memcpy(&pp, (uint8_t *)w->e + g_fe[q].off, sizeof pp);
      sysdep_free(pp);
      memcpy((uint8_t *)w->e + g_fe[q].off, &z, sizeof z);
      CV(G_CTOR, "dtor: a NULL block pointer is skipped");
    }
  }
  if (d2) f_d2(w->e); else f_d1(w->e);
  __asm__ volatile("" ::: "memory");
  if (d2) CV(G_CTOR, "dtor: D2"); else CV(G_CTOR, "dtor: D1");
  cov_flush();
  h_emit(g_show, "dtor %s", d2 ? "D2" : "D1");
  h_dump_obj(g_show, "E", w->e, g_fe, g_nfe);
  { unsigned i; for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("canary byte %u behind the object was overwritten", i); }
  g_ncalls++;
}

/* ------------------------------------------------------------------------------------------------ trials */
static int pick_fn(int sc) {
  if (sc == 1) { static const int s[] = { F_SETLIN, F_SETDFE, F_FREEZE, F_ENTER3, F_CV, F_GETDFE, F_RESETMEE, F_ZLIN, F_ZDFE, F_SETLINC, F_SETDFEC, F_LOAD, F_PRINT, F_PRINTEQU, F_FPE, F_RRN, F_RESTORE };
    static const int wgt[] = { 14, 14, 3, 4, 4, 2, 2, 3, 3, 3, 3, 1, 1, 1, 12, 12, 8 };
    unsigned t = h_rmod(90), i = 0;
    for (i = 0; i < sizeof s / sizeof s[0]; i++) { if (t < (unsigned)wgt[i]) return s[i]; t -= (unsigned)wgt[i]; }
    return F_SETLIN; }
  if (sc == 2) { static const int s[] = { F_FADE, F_SETFADE, F_RESET, F_CALC }; return s[h_rmod(4)]; }
  return (int)h_rmod(F_N);
}
static void trial(int sc) {
  if (sc == 0 || sc == 4) { ctor_trial(sc); h_trial_end("p42"); return; }
  {
    unsigned ncalls = 2u + h_rmod(5), k;
    new_hand_world();
    for (k = 0; k < ncalls; k++) {
      int fn = pick_fn(sc);
      if (fn == F_RESET && g_w.e->linHistLen_ < g_w.e->linLen_ + 1u) fn = F_FADE;
      do_call(fn);
    }
    free_world();
  }
  h_trial_end("p42");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, ntr, i;
  const char *seed = getenv("P42_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P42_ALL")) g_showall = 1;
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
  h_scenario_begin(sc == 0 || sc == 4 ? 2 : 1);   /* 2: blocks only 2-byte aligned (the 16-bit tables' alignment arithmetic) */
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P42_TRIALS") ? atoi(getenv("P42_TRIALS")) : 4000;
  if (sc < 0 || sc > 4) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  { CovSite *c; int bad = 0;
    for (c = __start_covsites; c < __stop_covsites; c++) {
      if (!(c->grp & (1u << sc))) continue;
      h_emit(1, "COV %s %lu", c->name, c->n);
      if (c->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", c->name); bad = 1; }
    }
    if (sc >= 1 && sc <= 3) for (i = 0; i < F_N; i++) if (!g_fcnt[i] && !(sc == 1 && i >= F_FADE) && !(sc == 2 && i < F_FADE)) { fprintf(stderr, "COVERAGE FAIL: %s never called\n", fnames[i]); bad = 1; }
    h_cov_finish();
    return (bad && !getenv("P42_NOCOV")) ? 2 : 0; }
}
