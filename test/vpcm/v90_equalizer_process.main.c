/*
 * Differential test harness for port chunk P40 (src/vpcm/v90_equalizer_process.cpp): V90Equalizer::process(float *in, unsigned n, short *dec, float *out, unsigned &outCnt), 9,364 B, 322 x87 instructions,
 * three jump tables.  Derived from test/vpcm/v90_phase4_demod_decisions.main.c (same recorder / snapshot design).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_equalizer_process.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2:
 * the port object alone, the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_equalizer_process.blob32.out).  The function is reached through an __asm__ label; the
 * object is the generated C mirror (test/vpcm/v90_equalizer_process.layout), the objects it points to are the mirrors of P05 (V90Resampler), P07 (V90Parameters), P17 (V90ConnectionEvaluator), P21 (V90SpectralVerifier),
 * P28 (V90Demapper), P34 (V90Phase3Demodulator), P37 (V90Phase4Demodulator): member-wise access only (their layouts differ on x86-64).  The V90PreFilter is an opaque block (only its address matters).
 *
 * Every callee is a RECORDER with scripted results: V90Equalizer::setLinearEquBeta / setDfeBeta (also WRITE the step sizes and the 16-bit-mode multiplier / shift), enterPhase4 (writes the mode),
 * enterDataPhase (converts: writes the 16-bit tables, sets the MMX flag), enterFPE / enterRRN (convert back, clear the MMX flag), linearEquFadeEdges, calcMeanErrorStatistics (returns a float in st0,
 * writes the mean error), V90Phase3Demodulator::getDecision / V90Phase4Demodulator::getDecision (WRITE the state / counter / event words the function reads afterwards), detectRRN / detectFPE,
 * V90Demapper::hardDecision / linearMappingStudy, ResamplerTiming::SdHalfBaudDft / resetSdHalfBaudDft, ResamplerTimingOffset::setTimingOffset, V90Resampler::setBllState (writes state_),
 * V90ConnectionEvaluator::updateAvePdsnr, V90PreFilter::isV90WithEia6, edprintf and dsplibs_debug_printf.  The results carry garbage in the bits the function must not look at (the 16-bit
 * decisions: the upper 16 bits).
 *
 * What is compared: the outputs (the decision shorts, the equalized floats, the symbol count), the whole equalizer object with the heap blocks it owns (the generated field table), the foreign words the
 * function reads / writes, every record (callee, arguments, and a SNAPSHOT hash of the equalizer + the arrays + the foreign words + the output buffers at the moment of the call: the store order before
 * each call / print), the canary behind the object.
 *
 * Excluded: a mode above 6 (the blob stores a decision slot it never wrote), the float-only decision modes 0 / 1 / 2 / 6 at the START of a call in the 16-bit mode (stale float slots), enterRRN / enterFPE
 * results in the float mode and enterDataPhase results in the 16-bit mode (the harness's recorders return 0 there), dfeLen_ / linLen_ == 0 (the blob loops 2^32 times), a zero or -1 divisor of the 16-bit
 * mode, signalling NaNs in every float the blob passes by value (the harness feeds quiet NaNs only).
 *
 * Scenarios (each its own process): 0 float mode (all decision modes, the conversions to the 16-bit mode), 1 16-bit mode start (modes 3 / 4 / 5, enterRRN / enterFPE), 2 mixed multi-call sequences with
 * odd sample counts, the pending sample, the error blocks and the frame counter.
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

#include <stdarg.h>
#include "../harness/harness.h"
#include "../p05_layout.h"
#include "pcm_parameters_layout.h"
#include "v90_connection_evaluator_layout.h"
#include "v90_spectral_shaper_layout.h"
#include "v90_mapper_layout.h"
#include "v90_phase3_demod_v92_decision_layout.h"
#include "v90_phase4_demod_decisions_layout.h"
#include "v90_equalizer_process_layout.h"

#define L(m) __asm__(m)

extern unsigned int dsplibs_debug_level;

/* the function under test */
extern void f_process(void *, float *, unsigned, short *, float *, unsigned *) L("_ZN12V90Equalizer7processEPfjPsS0_Rj");

/* the interposed callees */
extern void c_setLin(void *, float) L("_ZN12V90Equalizer16setLinearEquBetaEf");
#ifdef P42_INTEG
extern void f_ctor(void *, unsigned, unsigned, void *, void *, void *, void *, void *, void *, void *, void *, unsigned)
    L("_ZN12V90EqualizerC1EjjP20V90Phase3DemodulatorP20V90Phase4DemodulatorP11V90DemapperP22V90ConnectionEvaluatorP19V90SpectralVerifierP13V90ParametersP12V90ResamplerP12V90PreFilter20V90ComputationalMode");
extern void f_dtor(void *) L("_ZN12V90EqualizerD1Ev");
#endif
extern void c_setDfe(void *, float) L("_ZN12V90Equalizer10setDfeBetaEf");
extern void c_enterP4(void *) L("_ZN12V90Equalizer11enterPhase4Ev");
extern int c_enterDP(void *) L("_ZN12V90Equalizer14enterDataPhaseEv");
extern int c_enterFPE(void *) L("_ZN12V90Equalizer8enterFPEEv");
extern int c_enterRRN(void *) L("_ZN12V90Equalizer8enterRRNEv");
extern void c_fade(void *) L("_ZN12V90Equalizer18linearEquFadeEdgesEv");
extern float c_calc(void *) L("_ZN12V90Equalizer23calcMeanErrorStatisticsEv");
extern int c_p3dec(void *, float) L("_ZN20V90Phase3Demodulator11getDecisionEf");
extern int c_p4dec(void *, short) L("_ZN20V90Phase4Demodulator11getDecisionEs");
extern int c_detRRN(void *, short) L("_ZN20V90Phase4Demodulator9detectRRNEs");
extern int c_detFPE(void *, short) L("_ZN20V90Phase4Demodulator9detectFPEEs");
extern int c_hd(void *, short) L("_ZN11V90Demapper12hardDecisionEs");
extern void c_lms(void *, short, short) L("_ZN11V90Demapper18linearMappingStudyEss");
extern void c_sdft(void *, float) L("_ZN15ResamplerTiming13SdHalfBaudDftEf");
extern void c_sdreset(void *) L("_ZN15ResamplerTiming18resetSdHalfBaudDftEv");
extern void c_settoff(void *, float) L("_ZN21ResamplerTimingOffset15setTimingOffsetEf");
extern void c_bll(void *, unsigned, unsigned) L("_ZN12V90Resampler11setBllStateE11V90BllStatej");
extern void c_avepd(void *, float, unsigned) L("_ZN22V90ConnectionEvaluator14updateAvePdsnrEfj");
extern int c_eia6(void *) L("_ZNK12V90PreFilter13isV90WithEia6Ev");

typedef EquObj T;
#define SZ sizeof(T)
enum { CAN = 64, NSC = 160, LOGMAX = 1500, TXTMAX = 640, MAXN = 64 };

static uint32_t fnv_add(uint32_t h, const void *p, size_t n) { const uint8_t *q = (const uint8_t *)p; while (n--) { h ^= *q++; h *= 16777619u; } return h; }
static uint32_t fbits(float f) { uint32_t b; memcpy(&b, &f, 4); return b; }
static float ffb(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }

/* ------------------------------------------------------------------------------------------------ the world */
typedef struct {
  T *e;
  Ph3DemObj *p3;
  Ph4DemObj *p4;
  DemObj *dem, *dem2;       /* E->dem_ and p4->dem_ */
  EvObj *ce;
  SvObj *sv;
  V90ParObj *par;
  VrObj *rs;
  void *pf;
  float *in;                /* the input buffer (n + 4 floats), the output buffers */
  short *dec;
  float *out;
  unsigned n, nin;
} W;
static W g_w;
static uint8_t *g_can;
static HFld g_fe[128];       /* the equalizer's field table without the pointers to foreign objects */
static int g_nfe;

/* ------------------------------------------------------------------------------------------------ scripts, cursors, records */
enum { K_P3A, K_P3B, K_P3C, K_P4A, K_P4B, K_P4C, K_HD, K_RRN, K_FPE, K_EIA6, K_EDP, K_ERRN, K_EFPE, K_EP4, K_CALC, K_BETA, K_MISC, K_PRN, K_N };
static uint32_t g_sc[K_N][NSC];
static unsigned g_cur[K_N];
static uint32_t sw(int k) { return g_sc[k][g_cur[k]++ % NSC]; }
typedef struct { char t[TXTMAX]; } Rec;
typedef struct { Rec r[LOGMAX]; unsigned n; unsigned long total; } Log;
static Log g_il;
static int g_show;
static int g_rz, g_nomodel;
static int g_fdef;   /* the float slots (yf / Lf / Df) of the function are defined in this call: it started in the float mode, or enterRRN / enterFPE converted back */
static unsigned long g_symctr;
static unsigned long g_nrec[8];

static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}

/* the snapshot: a hash over every scalar of the equalizer, the arrays it owns, the foreign words the function reads / writes and the caller's buffers */
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
  h = fnv_add(h, &w->p3->state_, 4); h = fnv_add(h, &w->p3->cnt_, 4); h = fnv_add(h, &w->p3->event_, 4); h = fnv_add(h, &w->p3->status_, 4); h = fnv_add(h, &w->p3->b424_, 1);
  h = fnv_add(h, &w->p4->state_, 4); h = fnv_add(h, &w->p4->cnt_, 4); h = fnv_add(h, &w->p4->event_, 4); h = fnv_add(h, &w->p4->w38_, 4); h = fnv_add(h, &w->p4->studyEnd_, 4);
  h = fnv_add(h, &w->dem->z1eb4_, 2); h = fnv_add(h, &w->dem2->z1ea4_, 2); h = fnv_add(h, &w->dem2->z1ea6_, 2);
  h = fnv_add(h, &w->rs->state_, 4); h = fnv_add(h, &w->rs->stateFlag_, 4);
  h = fnv_add(h, w->dec, w->nin * 2u); h = fnv_add(h, w->out, w->nin * 4u);
  return h;
}
static void rec_add(Log *lg, const W *w, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2], snap[64];
  va_list ap;
  Rec *r = &lg->r[lg->n < LOGMAX ? lg->n : LOGMAX - 1];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (g_rz) { fprintf(stderr, "rz check before record: %s\n", buf); h_check_redzones(); }
  snprintf(snap, sizeof snap, "|S=%08x", snap_hash(w));
  r->t[0] = kind; r->t[1] = ' '; r->t[2] = 0;
  esc_cat(r->t, sizeof r->t, buf);
  if (strlen(r->t) + strlen(snap) + 1 >= sizeof r->t) h_die3("record text does not fit (%zu + %zu)", strlen(r->t), strlen(snap));
  strcat(r->t, snap);
  lg->n++; lg->total++;
  if (lg->n > LOGMAX) h_die3("more than %d records in one call", LOGMAX);
}
#define CHECK(p, what) do { if ((void *)(p) != (void *)(what)) h_die3("callee %s got an unexpected object %p (expected %p)", __func__, (void *)(p), (void *)(what)); } while (0)

/* ------------------------------------------------------------------------------------------------ the scripted behaviour of the callees (shared by the impl's recorders and the model) */
static uint32_t xs(uint32_t *s) { uint32_t x = *s ? *s : 0x9e3779b9u; x ^= x << 13; x ^= x >> 17; x ^= x << 5; *s = x; return x; }
static float pick_coef_f(uint32_t *s) {
  uint32_t r = xs(s);
  if ((r & 0xff) < 6) { static const uint32_t b[] = { 0, 0x80000000u, 0x7fc00000u, 0x3f800000u, 0xbf800000u, 0x7f800000u }; return ffb(b[(r >> 8) % 6]); }
  return (float)((int)(xs(s) % 20001u) - 10000) * (1.0f / 4096.0f);
}
static float pick_hist_f(uint32_t *s) {
  uint32_t r = xs(s);
  if ((r & 0xff) < 4) { static const uint32_t b[] = { 0, 0x80000000u, 0x7fc00000u, 0x7f7fffffu, 0x38000000u }; return ffb(b[(r >> 8) % 5]); }
  { int a = (int)(xs(s) % 40001u) - 20000; unsigned b = xs(s) % 100u; return (float)a * (1.0f / 8.0f) + (float)b * (1.0f / 128.0f); }   /* exactly representable: arch independent */
}
static int16_t pick_s16(uint32_t *s) { uint32_t r = xs(s); return (r & 0x3f) < 3 ? (int16_t)(r >> 8) : (int16_t)((int)(xs(s) % 4001u) - 2000); }
static int pick_div(uint32_t *s) { static const int d[] = { 1, 2, 3, 7, 8, 100, 4096, -2, -7, 30000, 5 }; return d[xs(s) % (sizeof d / sizeof d[0])]; }

/* enterDataPhase / enterFPE / enterRRN: the data the conversion leaves */
static void conv_to_mmx(W *w, uint32_t seed, int mid) {
  T *e = w->e;
  uint32_t s = seed | 1u;
  unsigned k, N = e->linLen_, M = e->dfeLen_, Lh = e->linHistLen_;
  e->mmx_ = 1;
  if (mid && (xs(&s) & 1) == 0) { static const uint32_t m[] = { 1, 2, 6, 0, 3, 4, 5, 10, 11, 12, 13, 14, 15, 16, 7 }; e->mode_ = m[xs(&s) % 15u]; }   /* the float-only modes continue with the stale float slots (defined: the float iteration ran) */
  e->linDiv_ = pick_div(&s); e->dfeDiv_ = pick_div(&s);
  e->linMul_ = (int)(xs(&s) % 41u) - 20; e->dfeMul_ = (int)(xs(&s) % 41u) - 20;
  e->linShift_ = (int)(xs(&s) % 70u); e->dfeShift_ = (int)(xs(&s) % 70u);
  e->histPos16_ = mid ? e->linPos_ : 1 + 2 * (xs(&s) % ((Lh - N) / 2));   /* the blob: histPos16_ = linPos_ (even, mid-iteration: the update steps it to odd) */
  for (k = 0; k < Lh + 4; k++) ((short *)e->hist16_)[k] = pick_s16(&s);
  for (k = 0; k < N; k++) { ((short *)e->linHi_)[k] = pick_s16(&s); ((short *)e->linLo_)[k] = pick_s16(&s); }
  for (k = 0; k < M; k++) { ((short *)e->dfeHi_)[k] = pick_s16(&s); ((short *)e->dfeLo_)[k] = pick_s16(&s); ((short *)e->dfeHist16_)[k] = pick_s16(&s); }
}
static void conv_to_float(W *w, uint32_t seed) {
  T *e = w->e;
  uint32_t s = seed | 1u;
  unsigned k, N = e->linLen_, M = e->dfeLen_, Lh = e->linHistLen_;
  e->mmx_ = 0;
  e->linPos_ = e->histPos16_;   /* the blob: linPos_ = histPos16_ (even, mid-iteration) */
  for (k = 0; k < Lh; k++) ((float *)e->linHist_)[k] = pick_hist_f(&s);
  for (k = 0; k < N; k++) ((float *)e->linCoef_)[k] = pick_coef_f(&s);
  for (k = 0; k < M; k++) { ((float *)e->dfeCoef_)[k] = pick_coef_f(&s); ((float *)e->dfeHist_)[k] = pick_hist_f(&s); }
}

/* a mode change from inside a handler (the decision of the current symbol is defined, so the next symbol's mode > 6 sees it) */
static void mode_poke(W *w, uint32_t r) {
  static const uint32_t hi[] = { 10, 11, 12, 13, 14, 15, 16, 13, 14, 15, 16, 7 }, lo[] = { 3, 4, 5, 0x80000000u };
  if ((r >> 20) % 10 != 0) return;
  { uint32_t k = (r >> 24) % 20u;
    if (k < 14) w->e->mode_ = hi[k % 12u];
    else if (k < 17 || w->e->mmx_ != 0) w->e->mode_ = lo[k % 4u];
    else w->e->mode_ = k - 17u; }   /* 0 .. 2 in the float mode only: the float-only modes keep working with the stale float slots */
}
/* a callee that changes the phase-3 state (the function re-reads p3->state_ after every call) */
static void misc_poke(W *w) {
  uint32_t q = sw(K_MISC);
  if (q % 12u == 0) w->p3->state_ = (q >> 4) % 20u;
}
static uint32_t quiet_bits(uint32_t b) { return ((b & 0x7f800000u) == 0x7f800000u && (b & 0x007fffffu)) ? (b | 0x00400000u) : b; }   /* the recorded ARGUMENT keeps its bits; the value written into the object is made quiet (the model's and the blob's NaN arithmetic are only compared on quiet NaNs) */
static void cb_setLin(W *w, uint32_t b) { uint32_t r = sw(K_BETA); mode_poke(w, r); misc_poke(w); b = quiet_bits(b); memcpy(&w->e->linBeta_, &b, 4); w->e->linMul_ = (int)(r & 0xff) - 100; w->e->linShift_ = (int)((r >> 8) & 0x3f); }
static void cb_setDfe(W *w, uint32_t b) { uint32_t r = sw(K_BETA); misc_poke(w); b = quiet_bits(b); memcpy(&w->e->dfeBeta_, &b, 4); w->e->dfeMul_ = (int)((r >> 16) & 0xff) - 100; w->e->dfeShift_ = (int)((r >> 24) & 0x3f); }
/* a decision near the symbol x: the script word r picks a delta (the errors around the threshold of 300 matter) or a random value; the garbage stays in the upper 16 bits */
static uint32_t dec_near(uint32_t r, int x) {
  static const int dt[16] = { 0, 0, 1, -1, 2, -5, 299, 300, 301, -299, -300, -301, 600, -1000, 12345, -32768 };
  unsigned sel = (r >> 16) & 7u;
  int d = (sel < 5u) ? (int)(int16_t)(uint16_t)(x + dt[r & 15u]) : (int)(r & 0xffffu);
  return (r & 0xffff0000u) | (uint32_t)(uint16_t)d;
}
static int f2i16(float x) { if (!(x > -32769.0f && x < 32768.0f)) return -32768; return (int)x; }
static uint32_t hi_ff(uint32_t v) { return v == 0xffu ? 0xffffffffu : v; }   /* 0xff in a scripted state / event stands for 0xffffffff (negative as a signed value) */
static void cb_p3(W *w, float x, uint32_t *ret) {
  uint32_t a = dec_near(sw(K_P3A), f2i16(x)), b = sw(K_P3B), c = sw(K_P3C);
  w->p3->event_ = hi_ff(b & 0xff); w->p3->state_ = hi_ff((b >> 8) & 0xff); w->p3->cnt_ = (b >> 16) & 0xff; w->p3->b424_ = (uint8_t)(b >> 24);
  w->p3->status_ = c;
  *ret = a;
}
static void cb_p4(W *w, int x, uint32_t *ret) {
  uint32_t a = dec_near(sw(K_P4A), x), b = sw(K_P4B), c = sw(K_P4C);
  w->p4->event_ = hi_ff(b & 0xff); w->p4->state_ = hi_ff((b >> 8) & 0xff); w->p4->cnt_ = (b >> 16) & 0xff; w->p4->w38_ = (b >> 24) & 3u ? 1u : 0u;
  w->dem2->z1ea4_ = (uint16_t)((b >> 26) & 1u ? 1u : 0u); w->dem2->z1ea6_ = (uint16_t)((b >> 27) & 1u ? 0x100u : 0u);
  w->p4->studyEnd_ = c & 0xff;
  *ret = a;
}
static uint32_t cb_edp(W *w) { uint32_t r = sw(K_EDP); if (w->e->mmx_ != 0 || (r & 3) == 0) return 0; conv_to_mmx(w, r >> 2, 1); return r | 1u; }
static uint32_t cb_efpe(W *w, int kind) { uint32_t r = sw(kind ? K_ERRN : K_EFPE); if (w->e->mmx_ == 0 || (r & 3) == 0) return 0; conv_to_float(w, r >> 2); g_fdef = 1; return r | 1u; }
static void cb_ep4(W *w) {
  static const uint32_t big[] = { 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, 0x20, 0x80000000u, 0xffffffffu, 100 };
  uint32_t r = sw(K_EP4);
  if ((r >> 8) % 5 == 0) w->e->mode_ = big[(r >> 12) % (sizeof big / sizeof big[0])];   /* modes above 6: the blob keeps the decision of the previous symbol */
  else if (w->e->mmx_ != 0) w->e->mode_ = 3 + r % 3; else w->e->mode_ = r % 7;
}
static uint32_t cb_calc(W *w) { uint32_t r = sw(K_CALC), q = sw(K_BETA); w->e->statMean_ = ffb(r); if ((q >> 5) % 4 == 0) w->p4->state_ = (q >> 8) & 7; return r; }

/* the callee environment: ONE implementation of every callee's scripted behaviour (the record text, the world it writes, the result), used by the impl's recorders (c_*) and by the model (m_*) */
static Log *g_log = &g_il;
#define REC(kind, ...) rec_add(g_log, &g_w, (kind), __VA_ARGS__)
static void env_setLin(uint32_t b) { REC('c', "setLin(%08x)", b); cb_setLin(&g_w, b); }
static void env_setDfe(uint32_t b) { REC('c', "setDfe(%08x)", b); cb_setDfe(&g_w, b); }
static void env_enterP4(void) { REC('c', "enterPhase4"); cb_ep4(&g_w); }
static uint32_t env_enterDP(void) { uint32_t r = cb_edp(&g_w); REC('c', "enterDataPhase -> %08x", r); return r; }
static uint32_t env_enterFPE(void) { uint32_t r = cb_efpe(&g_w, 0); REC('c', "enterFPE -> %08x", r); return r; }
static uint32_t env_enterRRN(void) { uint32_t r = cb_efpe(&g_w, 1); REC('c', "enterRRN -> %08x", r); return r; }
static void env_fade(void) { REC('c', "linearEquFadeEdges"); }
static uint32_t env_calc(void) { uint32_t r = cb_calc(&g_w); REC('c', "calcMeanErrorStatistics -> %08x", r); return r; }
static uint32_t env_p3dec(float x) { uint32_t r; cb_p3(&g_w, x, &r); REC('c', "p3.getDecision(%08x) -> %d ev=%u st=%u cnt=%u", fbits(x), (int)(int16_t)r, g_w.p3->event_, g_w.p3->state_, g_w.p3->cnt_); return r; }
static uint32_t env_p4dec(int x) { uint32_t r; cb_p4(&g_w, x, &r); REC('c', "p4.getDecision(%d) -> %d ev=%u st=%u cnt=%u", x, (int)(int16_t)r, g_w.p4->event_, g_w.p4->state_, g_w.p4->cnt_); return r; }
static uint32_t env_detRRN(int x) { uint32_t r = sw(K_RRN); REC('c', "p4.detectRRN(%d) -> %08x", x, r); return r; }
static uint32_t env_detFPE(int x) { uint32_t r = sw(K_FPE); REC('c', "p4.detectFPE(%d) -> %08x", x, r); return r; }
static uint32_t env_hd(int x) { uint32_t r = dec_near(sw(K_HD), x); REC('c', "hardDecision(%d) -> %d", x, (int)(int16_t)r); return r; }
static void env_lms(int a, int b) { REC('c', "linearMappingStudy(%d,%d)", a, b); }
static void env_sdft(uint32_t b) { REC('c', "SdHalfBaudDft(%08x)", b); }
static void env_sdreset(void) { REC('c', "resetSdHalfBaudDft"); }
static void env_settoff(uint32_t b) { REC('c', "setTimingOffset(%08x)", b); }
static void env_bll(unsigned st, unsigned fl) { REC('c', "setBllState(%u,%u)", st, fl); g_w.rs->state_ = st; }
static void env_avepd(uint32_t b, unsigned c) { REC('c', "updateAvePdsnr(%08x,%u)", b, c); }
static int env_eia6(void) { uint32_t r = sw(K_EIA6); REC('c', "isV90WithEia6 -> %u", r & 1u); return (int)(r & 1u); }
static void env_print(char kind, const char *fmt, va_list ap) {
  char buf[TXTMAX * 2];
  uint32_t r = sw(K_PRN);
  vsnprintf(buf, sizeof buf, fmt, ap);
  REC(kind, "%s", buf);
  if (r % 9u == 0 && g_w.e->mmx_ != 0 && g_fdef) {   /* a print that leaves the 16-bit mode (the function reloads its cache of mmx_ after some prints); the float history position becomes a legal odd one */
    T *e = g_w.e;
    e->mmx_ = 0;
    /* the position parity the float code expects: the prints of the error handling run AFTER the push of the pair (even: the float update still steps it) except the two update-phase prints of the 16-bit mode (the
     * update path is already chosen: the next float pair starts at an odd position) */
    e->linPos_ = (strstr(buf, "momentary") || strstr(buf, "consecutive") ? 1u : 0u) + 2u * ((r >> 8) % ((e->linHistLen_ - e->linLen_) / 2u));
  }
}

/* the impl's recorders */
#ifndef P42_INTEG   /* P42 integration build (test/vpcm/v90_equalizer.integ.sh): the six P42 callees are the REAL functions of the blob / of the P42 port, not recorders */
void c_setLin(void *e, float b) { CHECK(e, g_w.e); env_setLin(fbits(b)); }
void c_setDfe(void *e, float b) { CHECK(e, g_w.e); env_setDfe(fbits(b)); }
#endif
void c_enterP4(void *e) { CHECK(e, g_w.e); env_enterP4(); }
int c_enterDP(void *e) { CHECK(e, g_w.e); return (int)env_enterDP(); }
#ifndef P42_INTEG
int c_enterFPE(void *e) { CHECK(e, g_w.e); return (int)env_enterFPE(); }
int c_enterRRN(void *e) { CHECK(e, g_w.e); return (int)env_enterRRN(); }
void c_fade(void *e) { CHECK(e, g_w.e); env_fade(); }
float c_calc(void *e) { CHECK(e, g_w.e); return ffb(env_calc()); }
#endif
int c_p3dec(void *p, float x) { CHECK(p, g_w.p3); return (int)env_p3dec(x); }
int c_p4dec(void *p, short x) { CHECK(p, g_w.p4); return (int)env_p4dec((int)x); }
int c_detRRN(void *p, short x) { CHECK(p, g_w.p4); return (int)env_detRRN((int)x); }
int c_detFPE(void *p, short x) { CHECK(p, g_w.p4); return (int)env_detFPE((int)x); }
int c_hd(void *p, short x) { CHECK(p, g_w.dem); return (int)env_hd((int)x); }
void c_lms(void *p, short a, short b) { CHECK(p, g_w.dem); env_lms((int)a, (int)b); }
void c_sdft(void *p, float x) { CHECK(p, g_w.rs); env_sdft(fbits(x)); }
void c_sdreset(void *p) { CHECK(p, g_w.rs); env_sdreset(); }
void c_settoff(void *p, float x) { CHECK(p, g_w.rs); env_settoff(fbits(x)); }
void c_bll(void *p, unsigned st, unsigned fl) { CHECK(p, g_w.rs); env_bll(st, fl); }
void c_avepd(void *p, float x, unsigned c) { CHECK(p, g_w.ce); env_avepd(fbits(x), c); }
int c_eia6(void *p) { CHECK(p, g_w.pf); return env_eia6(); }
void edprintf(const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print('e', fmt, ap); va_end(ap); }
int dsplibs_debug_printf(const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print('d', fmt, ap); va_end(ap); return 0; }


/* ------------------------------------------------------------------------------------------------ the independent model */
/* written from porting/decompiled/dp/p40_equalizer_process.txt in a different shape than the port (per-symbol functions, no gotos); its own extended-precision helpers.  It runs on the LIVE world first (the world
 * is saved before and restored afterwards), through the same callee environment, and must leave the same world, the same outputs and the same records (with the same snapshot hashes) as the impl. */
typedef long double XL;
typedef struct {
  T *e;
  float *in;
  unsigned n, i, cnt, adapt, mode, mmx;
  short *p74, *decOut;
  float *outF;
  int y16, dec, q1, q2, e1, e2;
  volatile float yf, Lf, Df, decF;
  XL err;
} M;
enum { G_ALL = 7, G_S0 = 1, G_S1 = 2, G_S2 = 4, G_S01 = 3, G_S02 = 5, G_S12 = 6 };   /* the scenarios that must reach a site */
typedef struct { unsigned long n; unsigned char hit, grp; const char *name; } CovSite;
#define CV(g, nm) do { static CovSite cs_ __attribute__((section("covsites"), used, aligned(sizeof(void *)))) = { 0, 0, (g), (nm) }; cs_.hit = 1; } while (0)
extern CovSite __start_covsites[], __stop_covsites[];
static void cov_flush(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) if (c->hit) { c->n++; c->hit = 0; } }
static void cov_clear(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) c->hit = 0; }

static int m_f16(XL v) { if (v != v || v >= 32768.0L || v <= -32769.0L) return -32768; return (int)v; }
static int m_f32(XL v) { if (v != v || v >= 2147483648.0L || v <= -2147483649.0L) return INT_MIN; return (int)v; }
static uint32_t m_f64lo(XL v) { if (v != v || v >= 9223372036854775808.0L || v <= -9223372036854775809.0L) return 0; return (uint32_t)(uint64_t)(int64_t)v; }
static float m_rf(XL v) { volatile float t = (float)v; return t; }
static int m_abs(int v) { return v < 0 ? (int)(0u - (unsigned)v) : v; }
static int m_sgn(XL v) { return (v > 0.0L || v != v) ? '+' : '-'; }
static XL m_dot(const float *a, const float *b, unsigned n) {
  XL A = 0.0L, B = 0.0L;
  unsigned k, blk = n & ~3u;
  for (k = 0; k < n; k++) { XL p = (XL)a[k] * (XL)b[k]; if (k < blk && (k & 1u)) B += p; else A += p; }
  return B + A;
}
static void m_pf(char kind, const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print(kind, fmt, ap); va_end(ap); }
#define PF4 "V90Equalizer: "
static uint32_t m_wd(float f) { return fbits(f); }
static void m_lin(M *m, float b) { env_setLin(m_wd(b)); (void)m; }
static void m_dfe(M *m, float b) { env_setDfe(m_wd(b)); (void)m; }
static float P_(const void *p) { float f; memcpy(&f, p, 4); return f; }   /* a parameter word as float */
#define PAR(m_) P_(&g_w.par->m_)
static void m_exit(M *m) { m->mmx = m->e->mmx_; m->mode = m->e->mode_; }

/* conversion of the working data: float -> 16 bit (kind 0: Df 32 bit, 1: Df 16 bit), 16 bit -> float */
static void m_to16(M *m, int dfBits) {
  T *e = m->e;
  unsigned k, c = m->n - 2u * m->i;
  short *mi = (short *)e->mmxIn_;
  m->q2 = m_f16((XL)m->Lf);
  m->y16 = m_f16((XL)m->yf);
  m->q1 = dfBits == 32 ? m_f32((XL)m->Df) : m_f16((XL)m->Df);
  for (k = 0; k < c; k++) mi[k] = (short)m_f16((XL)m->in[k]);
  m->p74 = mi;
  for (k = 0; k < m->i; k++) ((short *)e->mmxOut_)[k] = (short)m_f16((XL)m->outF[k]);
}
static void m_tofloat(M *m) {
  T *e = m->e;
  unsigned k;
  const short *mi = (const short *)e->mmxIn_;
  m->Lf = (float)m->q2;
  m->in += 2u * m->i + (mi[0] != 0 ? 1u : 2u);
  m->yf = (float)(int16_t)m->y16;
  m->Df = (float)m->q1;
  for (k = 0; k < m->i; k++) m->outF[k] = (float)((short *)e->mmxOut_)[k];
}

static void m_phase3_tail(M *m) {
  T *e = m->e;
  W *w = &g_w;
  unsigned cn = w->p3->cnt_, st = w->p3->state_;
  if (st == 0 && w->p3->b424_ != 0) {
    CV(G_ALL, "p3 tail: half-baud DFT");
    if (e->mmx_) { env_sdft(fbits((float)((short *)e->hist16_)[e->histPos16_ + 1])); env_sdft(fbits((float)((short *)e->hist16_)[e->histPos16_])); }
    else { env_sdft(((uint32_t *)e->linHist_)[e->linPos_ + 1]); env_sdft(((uint32_t *)e->linHist_)[e->linPos_]); }
  }
  if (w->p3->state_ == 3 && (unsigned)w->par->linearEquTrn1dFreezeDuration == cn && e->flag148_ == 0) { CV(G_ALL, "p3 tail: lin trn1d"); m_lin(m, PAR(linearEquTrn1dBeta)); }
  if (w->p3->state_ == 3 && (unsigned)w->par->dfeTrn1dFreezeDuration == cn && e->flag148_ == 0) { CV(G_ALL, "p3 tail: dfe trn1d"); m_dfe(m, PAR(dfeTrn1dBeta)); }
  if (e->flag148_ != 0 && w->p3->state_ == 4) {
    if ((unsigned)w->par->linearEquQcTrn1dFreezeDuration == cn) { CV(G_ALL, "p3 tail: lin qc"); m_lin(m, PAR(linearEquDataBeta)); }
    if ((unsigned)w->par->dfeQcTrn1dFreezeDuration == cn) { CV(G_ALL, "p3 tail: dfe qc"); m_dfe(m, PAR(dfeDataBeta)); }
  }
  st = w->p3->state_;
  if (st == 5 || (st >= 10 && st <= 16)) { CV(G_ALL, "p3 tail: adapt = status"); m->adapt = w->p3->status_; }
  if (w->p3->state_ == 0x11) m->adapt = w->p3->status_;
  else if (w->p3->state_ == 4 && (unsigned)w->par->nofDdSymbolsBeforeMeanErrorDiagPhase3 == cn) { CV(G_ALL, "p3 tail: mse enable"); e->mseEnable_ = 1; }
  st = w->p3->state_;
  if (st == 10 || st == 13) { CV(G_ALL, "p3 tail: mse disable"); e->mseEnable_ = 0; }
  m_exit(m);
}
static void m_mode1(M *m) {
  T *e = m->e;
  W *w = &g_w;
  unsigned ev;
  m->dec = (int16_t)env_p3dec(m->yf);
  ev = w->p3->event_;
  if (ev != 0) {
    e->event_ = ev;
    switch (ev) {
      case 3:
        if (e->flag148_ != 0) { CV(G_ALL, "ev3: timing offset"); env_settoff(fbits(e->timingOffset_)); w->rs->stateFlag_ = 1; }
        else {
          unsigned k;
          for (k = 0; k < e->linLen_; k++) ((float *)e->linCoef_)[k] = 0.0f;
          CV(G_ALL, "ev3: coefficients cleared");
          if (e->mmx_) { CV(G_S2, "ev3: 16-bit tables cleared"); for (k = 0; k < e->linLen_ + 8u; k++) { ((short *)e->rawLinHi_)[k] = 0; ((short *)e->rawLinLo_)[k] = 0; } }
          env_bll(2, 1);
        }
        break;
      case 8: CV(G_ALL, "p3 ev 8"); m_lin(m, PAR(linearEquDilBeta)); m_dfe(m, PAR(dfeDilBeta)); break;
      case 9: CV(G_ALL, "p3 ev 9"); env_sdreset(); break;
      case 10: {
        unsigned short d;
        m_lin(m, PAR(linearEquDilHighUcodeBeta));
        if (dsplibs_debug_level > 1) m_pf('d', PF4 "DfeProtectionOnDil = %d \r\n", (int)(int16_t)e->w8_);
        d = e->w8_;
        CV(G_ALL, "p3 ev 10");
        if (d == 0) { CV(G_ALL, "ev10: w8 == 0"); m_dfe(m, PAR(dfeDilHighUcodeBeta)); }
        else m_dfe(m, m_rf((XL)PAR(dfeDilHighUcodeBeta) / (XL)(int16_t)d));
        if (w->rs->state_ != 0) { CV(G_ALL, "ev10: freeze timing"); e->bllSaved_ = w->rs->state_; env_bll(0, 1); m_pf('e', PF4 "DemodDilInHighUcodesStage -> freeze timing\r\n"); }
        m_pf('e', PF4 "DemodDilInHighUcodesStage\r\n");
        break; }
      case 11: CV(G_ALL, "p3 ev 11"); m_lin(m, PAR(linearEquDilMedUcodeBeta)); m_dfe(m, PAR(dfeDilMedUcodeBeta)); m_pf('e', PF4 "DemodDilHighUcodesStageTerminated setting medium ucode beta\r\n"); break;
      case 12: CV(G_ALL, "p3 ev 12"); m_lin(m, PAR(linearEquDilMedUcodeBeta)); m_dfe(m, PAR(dfeDilMedUcodeBeta)); m_pf('e', PF4 "DemodDilInMedUcodesStage\n"); break;
      case 13: CV(G_ALL, "p3 ev 13"); m_lin(m, PAR(linearEquDilBeta)); m_dfe(m, PAR(dfeDilBeta)); m_pf('e', PF4 "DemodDilInMedUcodesStageTerminated setting normal dil beta\r\n"); break;
      case 14: CV(G_ALL, "p3 ev 14"); m_lin(m, 0.0f); m_dfe(m, 0.0f); m_pf('e', PF4 "DemodDilInitialErrorRelaxation => freeze LE & DFE\r\n"); break;
      case 15: CV(G_ALL, "p3 ev 15"); m_lin(m, PAR(linearEquDilErrorRelaxBeta)); m_dfe(m, PAR(dfeDilErrorRelaxBeta)); env_bll(7, 1); m_pf('e', PF4 "DemodDilInitialErrorRelaxTerminated => unfreezing LE & DFE and Timing.\r\n"); break;
      case 16:
        CV(G_ALL, "p3 ev 16");
        m_lin(m, 0.0f); m_dfe(m, 0.0f);
        if (w->rs->state_ != 0) { CV(G_ALL, "ev16: freeze timing"); e->bllSaved_ = w->rs->state_; env_bll(0, 1); }
        m_pf('e', PF4 "DemodDilInFreeze => freeze LE & DFE & timing.\r\n");
        break;
      case 20: CV(G_ALL, "p3 ev 20"); env_enterP4(); break;
      default: break;
    }
  }
  m_phase3_tail(m);
}
static void m_p4tailA(M *m) {
  T *e = m->e;
  W *w = &g_w;
  unsigned st = w->p4->state_, cn = w->p4->cnt_;
  if ((st == 3 || st == 2) && (unsigned)w->par->linearEquTrn2dInitialDuration == cn) {
    CV(G_ALL, "p4 tail A: trn2d initial");
    m_lin(m, PAR(linearEquTrn2dBeta));
    if (e->flag148_ == 0) { CV(G_ALL, "p4 tail A: bll 9"); env_bll(9, 1); }
  }
  if (w->p4->state_ == 3) {
    if (w->p4->studyEnd_ == cn) { CV(G_ALL, "p4 tail A: study end"); e->mseEnable_ = 1; }
    if (w->dem2->z1ea4_ != 0 && e->flag144_ != 0) {
      CV(G_ALL, "p4 tail A: first study");
      e->flag144_ = 0; (void)env_calc(); e->mseFull_ = 0; memcpy(&e->stat13c_, &e->statMean_, 4); e->mseIdx_ = 0;
    }
    if (w->p4->state_ == 3 && w->dem2->z1ea6_ != 0 && e->flag146_ != 0) {
      XL ratio;
      uint32_t sb;
      CV(G_ALL, "p4 tail A: second study");
      e->flag146_ = 0;
      (void)env_calc();
      ratio = (e->statMean_ == 0.0f || e->statMean_ != e->statMean_) ? 0.0L : (XL)e->stat13c_ / (XL)e->statMean_;
      e->ratio_ = m_rf(ratio);
      if (ratio == 0.0L) CV(G_ALL, "ratio zero");
      if (dsplibs_debug_level > 1) {
        CV(G_ALL, "ratio print");
        int ip = m_f32(ratio);
        (void)sb;
        m_pf('d', PF4 "ph4MeanErrorEnergyBeforeToAfterUpdateRatio = %c%d.%03d\r\n", m_sgn((XL)e->ratio_), m_f32(ratio < 0.0L ? -ratio : ratio), m_abs(m_f32((ratio - (XL)ip) * 1000.0L)));
      }
    }
  }
  st = w->p4->state_;
  if (st == 4 || st == 5) e->mseEnable_ = 0;
  m_exit(m);
}
static void m_p4tailB(M *m) {
  T *e = m->e;
  W *w = &g_w;
  unsigned st;
  if (w->p4->state_ == 3 && w->p4->studyEnd_ == w->p4->cnt_) { CV(G_ALL, "p4 tail B: study end"); e->mseEnable_ = 1; e->mseIdx_ = 0; e->mseFull_ = 0; }
  st = w->p4->state_;
  if (st == 4 || st == 5) e->mseEnable_ = 0;
  m_exit(m);
}
static void m_mode2(M *m) {
  T *e = m->e;
  W *w = &g_w;
  unsigned ev;
  int a;
  if (m->yf > 32767.0f) m->yf = 32767.0f;
  else if (!(m->yf >= -32767.0f)) m->yf = -32767.0f;
  a = (int)w->p4->state_ > 1 ? m_f16((XL)m->yf) : m_f16((XL)m->Lf);
  m->dec = (int16_t)env_p4dec((int16_t)a);
  ev = w->p4->event_;
  if (ev != 0) {
    e->event_ = ev;
    if (ev == 0x17) {
      CV(G_ALL, "p4 ev 0x17");
      m_lin(m, PAR(linearEquTrn2dInitialBeta));
      if (w->sv->flag_ == 2) { CV(G_ALL, "ev17: german pbx"); m_dfe(m, PAR(germanPbxDfeTrn2dFastBeta)); }
      else if (env_eia6() == 0) m_dfe(m, PAR(dfeTrn2dBeta));
      else { CV(G_ALL, "ev17: eia6"); m_pf('e', PF4 "RiNot on EIA6 DFE fast\r\n"); m_dfe(m, PAR(eia6DfeTrn2dFastBeta)); }
      env_bll(e->flag148_ != 0 ? 9 : 8, 1);
    } else if (ev == 0x18) {
      CV(G_ALL, "p4 ev 0x18");
      if (w->sv->flag_ == 2) { CV(G_ALL, "ev18: german pbx"); m_pf('e', PF4 "middle of TRN2d\r\n"); m_dfe(m, PAR(germanPbxDfeTrn2dSlowBeta)); }
      if (env_eia6() != 0) { CV(G_ALL, "ev18: eia6"); m_pf('e', PF4 "middle of TRN2d (DFE EIA6 CONDITION)\r\n"); m_dfe(m, PAR(eia6DfeTrn2dSlowBeta)); }
    } else if (ev == 0x1c) {
      CV(G_ALL, "mode2 ev 0x1c");
      m_lin(m, PAR(linearEquDataBeta)); m_dfe(m, PAR(dfeDataBeta));
    } else if (ev == 0x1d) {
      if (env_enterDP() != 0) { CV(G_ALL, "mode2: to 16 bit"); m_to16(m, 16); }
    }
  }
  m_p4tailA(m);
}
static void m_mode3(M *m) {
  T *e = m->e;
  W *w = &g_w;
  m->dec = (int16_t)env_hd((int16_t)m->y16);
  if (w->dem->z1eb4_ != 0) { CV(G_ALL, "mode3: study"); env_lms((int16_t)m->y16, (int16_t)m->dec); }
  if (env_detRRN((int16_t)m->dec) != 0) {
    CV(G_ALL, "mode3: RRN");
    e->event_ = 0x23; w->dem->z1eb4_ = 0;
    if (dsplibs_debug_level > 1) m_pf('d', PF4 "disable linear mapping study.\n");
    if (env_enterRRN() != 0) { CV(G_ALL, "mode3: RRN to float"); m_tofloat(m); }
  }
  if (env_detFPE((int16_t)m->dec) != 0) {
    CV(G_ALL, "mode3: FPE");
    e->event_ = 0x25; w->dem->z1eb4_ = 0;
    if (dsplibs_debug_level > 1) m_pf('d', PF4 "disable linear mapping study.\n");
    if (env_enterFPE() != 0) { CV(G_ALL, "mode3: FPE to float"); m_tofloat(m); }
  }
  m_exit(m);
}
static void m_mode4(M *m) {
  T *e = m->e;
  W *w = &g_w;
  unsigned ev;
  m->dec = (int16_t)env_p4dec((int16_t)m->y16);
  ev = w->p4->event_;
  if (ev != 0) {
    e->event_ = ev;
    switch (ev) {
      case 0x1c:
        if (w->ce->rrnSilence_ != 0 && w->p4->w38_ != 0) { CV(G_ALL, "mode4 ev 0x1c: freeze"); m_pf('e', PF4 "Freezing equ & dfe on silence between Ed and Rt\r\n"); m_lin(m, 0.0f); m_dfe(m, 0.0f); }
        else { CV(G_ALL, "mode4 ev 0x1c: data betas"); m_lin(m, PAR(linearEquDataBeta)); m_dfe(m, PAR(dfeDataBeta)); }
        break;
      case 0x1d: if (env_enterDP() != 0) { CV(G_ALL, "mode4: to 16 bit"); m_to16(m, 32); } break;
      case 0x28:
        CV(G_ALL, "mode4 ev 0x28");
        m_lin(m, PAR(linearEquTrn2dBeta));
        if (w->sv->flag_ == 2) { CV(G_ALL, "mode4 0x28: german pbx"); m_dfe(m, PAR(germanPbxDfeTrn2dSlowBeta)); }
        else if (env_eia6() == 0) m_dfe(m, PAR(dfeTrn2dBeta));
        else { CV(G_ALL, "mode4 0x28: eia6"); m_pf('e', PF4 "Rtnot on  EIA6 DFE SLOW\r\n"); m_dfe(m, PAR(eia6DfeTrn2dSlowBeta)); }
        break;
      case 0x2c:
        if (env_eia6() != 0) { CV(G_ALL, "mode4 ev 0x2c"); m_pf('e', PF4 "after rrn (EIA6): set fast...\r\n"); m_lin(m, PAR(linearEquDataBeta)); m_dfe(m, PAR(eia6DfeTrn2dRrnBeta)); }
        break;
      case 0x35: CV(G_ALL, "mode4 ev 0x35"); m_pf('e', PF4 "Freezing equ & dfe on silence between Ed and Rt\r\n"); m_lin(m, 0.0f); m_dfe(m, 0.0f); break;
      default: break;
    }
  }
  m_p4tailB(m);
}
static void m_mode5(M *m) {
  T *e = m->e;
  W *w = &g_w;
  unsigned ev;
  m->dec = (int16_t)env_p4dec((int16_t)m->y16);
  ev = w->p4->event_;
  if (ev != 0) {
    e->event_ = ev;
    if (ev == 0x1d && env_enterDP() != 0) { CV(G_ALL, "mode5: to 16 bit"); m_to16(m, 32); }
  }
  m_exit(m);
}
static void m_mode6(M *m) {
  T *e = m->e;
  W *w = &g_w;
  unsigned ev;
  m->dec = (int16_t)env_p3dec(m->yf);
  ev = w->p3->event_;
  if (ev != 0) {
    e->event_ = ev;
    if (ev == 0x39) { CV(G_ALL, "mode6: ev 0x39"); env_bll(0, 1); }
  }
  m_exit(m);
}

/* the sums of one pair */
static void m_pair16(M *m) {
  T *e = m->e;
  short *h = (short *)e->hist16_;
  unsigned k;
  int s1 = 0, s2 = 0;
  h[e->histPos16_] = m->p74[0];
  e->histPos16_ -= 1;
  h[e->histPos16_] = m->p74[1];
  m->p74 += 2;
  for (k = 0; (int)k < (int)e->dfeLen_; k++) s1 = (int)((unsigned)s1 + (unsigned)((int)((short *)e->dfeHi_)[k] * (int)((short *)e->dfeHist16_)[k]));
  m->q1 = s1 / e->dfeDiv_;
  for (k = 0; (int)k < (int)e->linLen_; k++) s2 = (int)((unsigned)s2 + (unsigned)((int)((short *)e->linHi_)[k] * (int)h[e->histPos16_ + k]));
  m->q2 = s2 / e->linDiv_;
  m->y16 = (int16_t)(m->q2 - m->q1);
}
static void m_pairf(M *m) {
  T *e = m->e;
  float *h = (float *)e->linHist_;
  uint32_t *hw = (uint32_t *)e->linHist_;
  if (e->pend_) { e->pend_ = 0; memcpy(&hw[e->linPos_], &e->pendSample_, 4); e->linPos_ -= 1; memcpy(&hw[e->linPos_], m->in, 4); m->in += 1; }
  else { memcpy(&hw[e->linPos_], m->in, 4); e->linPos_ -= 1; memcpy(&hw[e->linPos_], m->in + 1, 4); m->in += 2; }
  m->Df = m_rf(m_dot((float *)e->dfeHist_, (float *)e->dfeCoef_, e->dfeLen_));
  m->Lf = m_rf(m_dot(&h[e->linPos_], (float *)e->linCoef_, e->linLen_));
  { XL y = (XL)m->Lf - (XL)m->Df; m->y16 = m_f16(y); m->yf = m_rf(y); }
}

/* the adaptation of one pair */
static int m_quiet_mode(unsigned mode) { return !(mode >= 10 && mode <= 16); }
static void m_update16(M *m) {
  T *e = m->e;
  unsigned k;
  short *dh = (short *)e->dfeHist16_;
  ((int16_t *)m->decOut)[m->i] = (int16_t)m->dec;
  m->e1 = (int16_t)(m->y16 - m->dec);
  if (m_abs(m->e1) > 300 && (int)m->mode > 1) {
    CV(G_ALL, "mmx: large error");
    if (e->badCnt_ <= 1) { m_pf('e', PF4 "High momentary error, symbol#%d, error %d, soft Decision %d\r\n", (int)m->i, m->e1, m->y16); m->mmx = e->mmx_; }
    e->badCnt_ += 1;
    m->adapt = 0;
  } else if (m_quiet_mode(m->mode) && e->badCnt_ > 2) {
    m->adapt += 1;
    if (m->adapt == 4) { CV(G_ALL, "mmx: consecutive errors"); m_pf('e', PF4 "nof consecutive errors = %d\r\n", (int)e->badCnt_); e->badCnt_ = 0; m->mmx = e->mmx_; }
  }
  m->e2 = (int16_t)(m->q2 - m->dec);
  if (m->adapt != 0) {
    CV(G_ALL, "mmx: adapt");
    int ml = (int)((0u - (unsigned)e->linMul_) * (unsigned)m->e2), md = (int)((unsigned)m->e1 * (unsigned)e->dfeMul_);
    for (k = 0; (int)k < (int)e->linLen_; k++) {
      int c = (int)(((unsigned)(int)((short *)e->linHi_)[k] << 16) | (uint16_t)((short *)e->linLo_)[k]);
      c = (int)((unsigned)c + (unsigned)((int)((unsigned)(int)((short *)e->hist16_)[e->histPos16_ + k] * (unsigned)ml) >> (e->linShift_ & 31)));
      ((short *)e->linLo_)[k] = (short)c; ((short *)e->linHi_)[k] = (short)(c >> 16);
    }
    for (k = 0; (int)k < (int)e->dfeLen_; k++) {
      int c = (int)(((unsigned)(int)((short *)e->dfeHi_)[k] << 16) | (uint16_t)((short *)e->dfeLo_)[k]);
      c = (int)((unsigned)c + (unsigned)((int)((unsigned)(int)dh[k] * (unsigned)md) >> (e->dfeShift_ & 31)));
      ((short *)e->dfeLo_)[k] = (short)c; ((short *)e->dfeHi_)[k] = (short)(c >> 16);
    }
  }
  for (k = e->dfeLen_ - 1; k > 0; k--) dh[k] = dh[k - 1];
  dh[0] = (short)m->e2;
  if ((int)(e->histPos16_ - 1) < 0) {
    CV(G_ALL, "mmx: history wrap");
    unsigned N = e->linLen_, Lh = e->linHistLen_;
    short *h = (short *)e->hist16_;
    e->histPos16_ = Lh - N - 1;
    for (k = 0; k < N; k++) h[Lh - 1 - k] = h[N - 1 - k];
  } else e->histPos16_ -= 1;
  e->errSum_ += (unsigned)m->e1 * (unsigned)m->e1;
  ((short *)e->mmxOut_)[m->i] = (short)m->y16;
}
static void m_updatef(M *m) {
  T *e = m->e;
  unsigned k;
  float *dh = (float *)e->dfeHist_, *h = (float *)e->linHist_;
  ((int16_t *)m->decOut)[m->i] = (int16_t)m->dec;
  m->decF = (float)(int16_t)m->dec;
  m->err = (XL)m->yf - (XL)(int16_t)m->dec;
  if (!(300.0f >= (m->err < 0.0L ? -m->err : m->err)) && (int)m->mode > 1) {
    CV(G_ALL, "float: large error");
    if (e->badCnt_ <= 1) {
      XL ae = m->err < 0.0L ? -m->err : m->err, fy = (XL)m->yf;
      int iy = m_f32(fy), ie = m_f32(m->err);
      m_pf('e', PF4 "High momentary error, symbol#%d, error = %c%d.%03d,   soft Decision = %c%d.%03d\r\n", (int)m->i, m_sgn(m->err), m_f32(ae), m_abs(m_f32(1000.0L * (m->err - (XL)ie))),
           m_sgn(fy), m_f32(fy < 0.0L ? -fy : fy), m_abs(m_f32((fy - (XL)iy) * 1000.0L)));
      m->err = (XL)m_rf(m->err);
      m->mmx = e->mmx_;
    }
    e->badCnt_ += 1;
    m->adapt = 0;
  } else if (m_quiet_mode(m->mode) && e->badCnt_ > 2) {
    m->adapt += 1;
    if (m->adapt == 4) { CV(G_ALL, "float: consecutive errors"); m->err = (XL)m_rf(m->err); m_pf('e', PF4 "nof consecutive errors = %d\r\n", (int)e->badCnt_); e->badCnt_ = 0; m->mmx = e->mmx_; }
  }
  {
    XL lerr = (XL)m->Lf - (XL)m->decF;
    if (m->adapt != 0) {
      XL cd = (XL)e->dfeBeta_ * m->err, cl = -(XL)e->linBeta_ * lerr;
      for (k = 0; k < e->dfeLen_; k++) ((float *)e->dfeCoef_)[k] = m_rf((XL)dh[k] * cd + (XL)((float *)e->dfeCoef_)[k]);
      for (k = 0; k < e->linLen_; k++) ((float *)e->linCoef_)[k] = m_rf((XL)h[e->linPos_ + k] * cl + (XL)((float *)e->linCoef_)[k]);
    }
    for (k = e->dfeLen_ - 1; k > 0; k--) memcpy(&dh[k], &dh[k - 1], 4);
    dh[0] = m_rf(lerr);
  }
  if ((int)(e->linPos_ - 1) < 0) {
    CV(G_ALL, "float: history wrap");
    unsigned N = e->linLen_, Lh = e->linHistLen_;
    e->linPos_ = Lh - N - 1;
    for (k = 0; k < N; k++) memcpy(&h[Lh - 1 - k], &h[N - 1 - k], 4);
  } else e->linPos_ -= 1;
  e->errSum_ += m_f64lo(m->err * m->err);
  m->outF[m->i] = m->yf;
}

static void m_process(float *in, unsigned n, short *decOut, float *outF, unsigned *outCnt) {
  M m;
  T *e = g_w.e;
  unsigned k;
  memset(&m, 0, sizeof m);
  m.e = e; m.in = in; m.n = n; m.decOut = decOut; m.outF = outF; m.adapt = 1;
  e->event_ = 0;
  {
    short *mi = (short *)e->mmxIn_;
    m.p74 = mi + 1;
    if (e->mmx_) {
      for (k = 0; k < n; k++) mi[1 + k] = (short)m_f16((XL)in[k]);
      if (e->pend_) { mi[0] = (short)m_f16((XL)e->pendSample_); m.p74 = mi; e->pend_ = 0; m.n++; }
      else mi[0] = 0;
    } else if (e->pend_) m.n++;
  }
  m.cnt = m.n >> 1;
  *outCnt = m.cnt;
  m.mmx = e->mmx_;
  for (m.i = 0; m.i < m.cnt; m.i++) {
    if (m.mmx) m_pair16(&m); else m_pairf(&m);
    m.mode = e->mode_;
    if (m.mode > 6) CV(G_ALL, "mode above 6");
    if (m.mode <= 6) {
      switch (m.mode) {
        case 0: CV(G_ALL, "mode 0"); m.dec = m_f16((XL)m.yf); break;
        case 1: m_mode1(&m); break;
        case 2: m_mode2(&m); break;
        case 3: m_mode3(&m); break;
        case 4: m_mode4(&m); break;
        case 5: m_mode5(&m); break;
        default: m_mode6(&m); break;
      }
    }
    if (m.mmx) m_update16(&m); else m_updatef(&m);
  }
  if (m.cnt == 0) m.mmx = e->mmx_;
  if (m.mmx) for (k = 0; k < m.cnt; k++) outF[k] = (float)((short *)e->mmxOut_)[k];
  e->errCnt_ += m.cnt;
  if (e->errCnt_ >= e->errLimit_) {
    CV(G_ALL, "error block");
    XL rms = (XL)(uint64_t)e->errSum_ / (XL)(uint64_t)e->errCnt_, kk = (XL)e->errK_;
    __asm__("fsqrt" : "+t"(rms));
    e->rmsErr_ = m_rf(rms);
    e->meanErr_ = m_rf((1.0L - kk) * rms + kk * (XL)e->meanErr_);
    env_avepd(fbits(e->meanErr_), e->errCnt_);
    e->errCnt_ = 0; e->errSum_ = 0;
    if (e->mseEnable_) {
      CV(G_ALL, "error block: mse history");
      memcpy(&((uint32_t *)e->mseHist_)[e->mseIdx_], &e->rmsErr_, 4);
      if (e->mseIdx_ + 1 == 300) { CV(G_ALL, "error block: mse wrap"); e->mseIdx_ = 0; e->mseFull_ = 1; } else e->mseIdx_ += 1;
    }
  }
  if (m.n & 1u) {
    CV(G_ALL, "odd sample kept");
    e->pend_ = 1;
    if (e->mmx_) e->pendSample_ = (float)*m.p74; else memcpy(&e->pendSample_, m.in, 4);
  }
  e->frameCnt_ += 1;
  if (e->frameCnt_ == (unsigned)g_w.par->linearEquFadeEdgesCycle) { CV(G_ALL, "fade edges"); e->frameCnt_ = 0; env_fade(); }
}

/* ------------------------------------------------------------------------------------------------ the world: saved before the model, restored for the impl, compared afterwards */
typedef struct { void *p; size_t n; } Reg;
static Reg g_reg[64];
static int g_nreg;
static uint8_t *g_sv0[64], *g_sv1[64];
static Log g_ml;
static void add_reg(void *p, size_t n) { if (g_nreg >= 64) h_die3("too many regions"); g_reg[g_nreg].p = p; g_reg[g_nreg].n = n; g_nreg++; }
static void build_regions(void) {
  int i;
  W *w = &g_w;
  g_nreg = 0;
  add_reg(w->e, SZ);
  for (i = 0; i < g_nfe; i++) {
    if (g_fe[i].kind == 'o') { void *p; HAl *a; memcpy(&p, (uint8_t *)w->e + g_fe[i].off, sizeof p); a = h_find_al(p, 0); if (!a) h_die3("owned block not found"); add_reg(a->p, a->size); }
  }
  add_reg(w->p3, sizeof(Ph3DemObj)); add_reg(w->p4, sizeof(Ph4DemObj)); add_reg(w->dem, sizeof(DemObj)); add_reg(w->dem2, sizeof(DemObj)); add_reg(w->ce, sizeof(EvObj));
  add_reg(w->sv, sizeof(SvObj)); add_reg(w->par, sizeof(V90ParObj)); add_reg(w->rs, sizeof(VrObj));
  add_reg(w->dec, w->nin * 2u); add_reg(w->out, w->nin * 4u);
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

/* ------------------------------------------------------------------------------------------------ trial setup */
static unsigned rnd_range(unsigned lo, unsigned hi) { return lo + h_rmod(hi - lo + 1); }
static int g_cls;     /* trial class: 0..4 normal, 5 wild dynamic range, 6 reset-like identity equalizer, 7 identity with boundary inputs */
static float wild_f(void);
static int g_snan;
/* the BITS of a parameter float: finite most of the time, quiet NaN / inf / 0 / negative sometimes; P40_SNAN=1 adds signalling NaNs (only ever moved as words: a float return would quiet them on i386) */
static int g_snan_ok;   /* set while a word that the function only MOVES (a parameter / timing offset passed on as an argument) is generated */
static uint32_t pick_fb(void) {
  static const uint32_t b[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7f800000u, 0xff800000u, 0x3f800000u, 0xbf800000u, 0x00000001u, 0x7f7fffffu, 0x3a83126fu, 0x358637bdu, 0x38d1b717u, 0x3f000000u };
  uint32_t v;
  if (g_snan && g_snan_ok && h_rpct(20)) return 0x7f800001u + h_rmod(1u << 22);
  if (h_rpct(25)) v = b[h_rmod(sizeof b / sizeof b[0])];
  else { v = ((uint32_t)(0x31u + h_rmod(14)) << 23) | h_rmod(1u << 23); if (h_rpct(15)) v |= 0x80000000u; }   /* built from bits: no float arithmetic */
  if ((v & 0x7f800000u) == 0x7f800000u && (v & 0x007fffffu)) v |= 0x00400000u;
  return v;
}
static float pick_f(void) { uint32_t v = pick_fb(); return ffb(v); }
static uint32_t pick_small(void) { return h_rpct(80) ? h_rmod(4) : h_rpct(50) ? h_rnd() : (h_rpct(50) ? 0xffffffffu : 0x80000000u); }
static unsigned pick_flag(void) { return h_rpct(45) ? 0u : h_rpct(70) ? 1u : h_rnd(); }
static void *new_arr(unsigned bytes) { return sysdep_malloc(bytes); }
static void fill_f(float *a, unsigned n, uint32_t seed, int coef) { uint32_t s = seed | 1u; unsigned k; for (k = 0; k < n; k++) a[k] = coef ? pick_coef_f(&s) : pick_hist_f(&s); }
static void fill_s(short *a, unsigned n, uint32_t seed) { uint32_t s = seed | 1u; unsigned k; for (k = 0; k < n; k++) a[k] = pick_s16(&s); }

static void new_world(int sc) {
  W *w = &g_w;
  T *e;
  unsigned N = h_rpct(60) ? 4u * (1u + h_rmod(4)) : 2u + h_rmod(16), M = h_rpct(60) ? 4u * (1u + h_rmod(3)) : 2u + h_rmod(13), Lh = N + 2u + 2u * h_rmod(6);
  uint8_t *raw = (uint8_t *)h_obj_new(SZ + CAN);
  uintptr_t a;
  memset(raw, 0, SZ);
  memset(raw + SZ, 0x5a, CAN);
  w->e = e = (T *)raw; g_can = raw + SZ;
  w->p3 = (Ph3DemObj *)h_obj_new(sizeof(Ph3DemObj));
  w->p4 = (Ph4DemObj *)h_obj_new(sizeof(Ph4DemObj));
  w->dem = (DemObj *)h_obj_new(sizeof(DemObj));
  w->dem2 = (DemObj *)h_obj_new(sizeof(DemObj));
  w->ce = (EvObj *)h_obj_new(sizeof(EvObj));
  w->sv = (SvObj *)h_obj_new(sizeof(SvObj));
  w->par = (V90ParObj *)h_obj_new(sizeof(V90ParObj));
  w->rs = (VrObj *)h_obj_new(sizeof(VrObj));
  w->pf = h_obj_new(64);
  e->resampler_ = w->rs; e->p3_ = w->p3; e->p4_ = w->p4; e->dem_ = w->dem; e->ce_ = w->ce; e->sv_ = w->sv; e->pf_ = w->pf; e->params_ = w->par;
  w->p4->dem_ = w->dem2;
#ifdef P42_INTEG    /* the equalizer is built by the REAL constructor (C1: reset, the tables, the aligned pointers); the data is filled on top */
  N = 4u * (1u + h_rmod(5)); M = 4u * (1u + h_rmod(4)); Lh = N + 2u + 2u * h_rmod(6);
  { ModParObj *mp = (ModParObj *)h_obj_new(sizeof(ModParObj));
    memset(mp, 0, sizeof *mp);
    *(int32_t *)((uint8_t *)mp + 0x5c) = 1;
    w->par->params = (void *)mp; w->par->enableEqualizerMmx = 1; w->par->linearEquHistoryLength = (int32_t)Lh;
    w->par->errorEnergyMeanBlockLen = 4; w->par->errorEnergyMeanK = 0.5f; w->par->linearEquFadeLeftEdgeRatio = 0.25f; w->par->linearEquFadeRightEdgeRatio = 0.125f;
    f_ctor(e, N, M, w->p3, w->p4, w->dem, w->ce, w->sv, w->par, w->rs, w->pf, 2u);
  }
  if (e->mmxCapable_ != 1 || e->linLen_ != N || e->dfeLen_ != M || e->linHistLen_ != Lh) h_die3("the real constructor built an unexpected equalizer");
#else
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
#endif
  g_cls = (int)h_rmod(8);
  fill_f((float *)e->linCoef_, N, h_rnd(), 1); fill_f((float *)e->linHist_, Lh, h_rnd(), 0);
  fill_f((float *)e->fadeWinA_, N, h_rnd(), 1); fill_f((float *)e->fadeWinB_, N, h_rnd(), 1);
  fill_f((float *)e->dfeCoef_, M, h_rnd(), 1); fill_f((float *)e->dfeHist_, M, h_rnd(), 0);
  fill_f((float *)e->mseHist_, 300, h_rnd(), 1);
  if (g_cls == 5) { unsigned k; for (k = 0; k < N; k++) ((float *)e->linCoef_)[k] = wild_f(); for (k = 0; k < Lh; k++) ((float *)e->linHist_)[k] = wild_f();
                    for (k = 0; k < M; k++) { ((float *)e->dfeCoef_)[k] = wild_f(); ((float *)e->dfeHist_)[k] = wild_f(); } }
  if (g_cls >= 6) { unsigned k, d = h_rmod(N); for (k = 0; k < N; k++) ((float *)e->linCoef_)[k] = k == d ? 1.0f : 0.0f; for (k = 0; k < M; k++) { ((float *)e->dfeCoef_)[k] = 0.0f; ((float *)e->dfeHist_)[k] = 0.0f; } }
  fill_s((short *)e->mmxIn_, 512, h_rnd()); fill_s((short *)e->mmxOut_, 256, h_rnd());
  fill_s((short *)e->rawLinHi_, N + 8, h_rnd()); fill_s((short *)e->rawLinLo_, N + 8, h_rnd()); fill_s((short *)e->rawHist16_, Lh + 8, h_rnd());
  fill_s((short *)e->rawDfeHi_, M + 8, h_rnd()); fill_s((short *)e->rawDfeLo_, M + 8, h_rnd()); fill_s((short *)e->rawDfeHist16_, M + 8, h_rnd());
  /* scalars */
  e->bllSaved_ = h_rnd(); e->w8_ = (uint16_t)(h_rpct(35) ? 0u : h_rpct(60) ? h_rmod(4) : h_rpct(50) ? 0xffffu : h_rnd());
  e->linBeta_ = pick_f(); e->dfeBeta_ = pick_f();
  e->linPos_ = 1u + 2u * h_rmod((Lh - N) / 2u);
  e->frameCnt_ = h_rpct(60) ? h_rmod(6) : h_rnd();
  e->mode_ = 0; e->event_ = h_rnd();
  e->pend_ = h_rpct(35) ? 1u : h_rpct(70) ? 0u : h_rnd();
  e->pendSample_ = h_float_from_bits_quiet(h_rpct(60) ? fbits(pick_hist_f(&(uint32_t){ h_rnd() })) : h_rnd());
  e->errCnt_ = h_rpct(70) ? h_rmod(6) : h_rpct(50) ? h_rnd() : 0xfffffff0u + h_rmod(16);
  e->errLimit_ = h_rpct(75) ? 1u + h_rmod(8) : h_rpct(50) ? h_rnd() : 0u;
  e->errSum_ = h_rpct(60) ? h_rmod(1000) : h_rnd();
  e->rmsErr_ = pick_f(); e->meanErr_ = pick_f(); e->statMean_ = pick_f(); e->statMax_ = pick_f(); e->statMin_ = pick_f(); e->stat13c_ = pick_f();
  e->errK_ = h_rpct(30) ? pick_f() : h_float_from_bits_quiet(fbits(0.5f) + h_rmod(1u << 22));
  e->badCnt_ = h_rpct(70) ? h_rmod(7) : h_rnd();
  e->mseIdx_ = h_rpct(40) ? 299u : h_rpct(50) ? 298u : h_rmod(300); e->mseFull_ = h_rnd(); e->mseEnable_ = h_rpct(50) ? 1u : 0u;
  e->mmxCapable_ = 1; e->mmx_ = 0;
  e->linDiv_ = 1; e->dfeDiv_ = 1;
  e->flag144_ = (uint16_t)(h_rpct(40) ? 0u : h_rpct(60) ? 1u : h_rnd()); e->flag146_ = (uint16_t)(h_rpct(40) ? 0u : h_rpct(60) ? 1u : h_rnd());
  e->flag148_ = h_rpct(60) ? 0u : h_rpct(70) ? 1u : h_rnd();
  { uint32_t v_; g_snan_ok = 1; v_ = pick_fb(); g_snan_ok = 0; memcpy(&e->timingOffset_, &v_, 4); }
  e->linCoefMax_ = pick_f(); e->linCoefMin_ = pick_f(); e->linMmxGain_ = pick_f(); e->dfeCoefMax_ = pick_f(); e->dfeCoefMin_ = pick_f(); e->dfeMmxGain_ = pick_f();
  e->histPos16_ = 1u + 2u * h_rmod((Lh - N) / 2u);
  e->fadeLenA_ = h_rnd(); e->fadeLenB_ = h_rnd(); e->ratio_ = pick_f(); e->errCnt_ = e->errCnt_;
#ifdef P42_INTEG
  e->fadeLenA_ = h_rmod(N + 1u); e->fadeLenB_ = h_rmod(N + 1u);   /* the real linearEquFadeEdges windows that many taps */
#endif
  /* the foreign objects */
  w->p3->state_ = h_rmod(0x13); w->p3->cnt_ = pick_small(); w->p3->event_ = h_rnd(); w->p3->status_ = pick_flag(); w->p3->b424_ = (uint8_t)(h_rpct(50) ? 1u : 0u);
  w->p4->state_ = h_rmod(8); w->p4->cnt_ = pick_small(); w->p4->event_ = h_rnd(); w->p4->w38_ = pick_flag(); w->p4->studyEnd_ = pick_small();
  w->dem->z1eb4_ = (uint16_t)(h_rpct(50) ? 0u : h_rpct(70) ? 1u : h_rnd()); w->dem2->z1ea4_ = (uint16_t)(h_rpct(40) ? 0u : h_rpct(70) ? 1u : h_rnd()); w->dem2->z1ea6_ = (uint16_t)(h_rpct(40) ? 0u : h_rpct(70) ? 1u : h_rnd());
  w->ce->rrnSilence_ = (int32_t)pick_flag(); w->sv->flag_ = h_rpct(80) ? h_rmod(4) : h_rnd();
  w->rs->state_ = h_rpct(70) ? h_rmod(10) : h_rnd(); w->rs->stateFlag_ = h_rnd();
  { V90ParObj *p = w->par;
    float f;
    p->linearEquFadeEdgesCycle = (int32_t)(h_rpct(80) ? 1u + h_rmod(6) : h_rpct(50) ? 0u : h_rnd());
    p->linearEquTrn1dFreezeDuration = (int32_t)pick_small(); p->linearEquTrn2dInitialDuration = (int32_t)pick_small(); p->dfeTrn1dFreezeDuration = (int32_t)pick_small();
    p->linearEquQcTrn1dFreezeDuration = (int32_t)pick_small(); p->dfeQcTrn1dFreezeDuration = (int32_t)pick_small(); p->nofDdSymbolsBeforeMeanErrorDiagPhase3 = (int32_t)pick_small();
#define PF(m) do { uint32_t v_; g_snan_ok = 1; v_ = pick_fb(); g_snan_ok = 0; memcpy(&p->m, &v_, 4); (void)f; } while (0)
    PF(linearEquTrn1dBeta); PF(linearEquDilBeta); PF(linearEquDilMedUcodeBeta); PF(linearEquDilHighUcodeBeta); PF(linearEquDilErrorRelaxBeta); PF(linearEquTrn2dInitialBeta); PF(linearEquTrn2dBeta);
    PF(linearEquDataBeta); PF(dfeTrn1dBeta); PF(dfeDilBeta); PF(dfeTrn2dBeta); PF(dfeDataBeta); PF(dfeDilMedUcodeBeta); PF(dfeDilHighUcodeBeta); PF(dfeDilErrorRelaxBeta);
    PF(germanPbxDfeTrn2dFastBeta); PF(germanPbxDfeTrn2dSlowBeta); PF(eia6DfeTrn2dFastBeta); PF(eia6DfeTrn2dSlowBeta); PF(eia6DfeTrn2dRrnBeta);
  }
  if (sc == 1) { e->mode_ = 3 + h_rmod(3); conv_to_mmx(w, h_rnd(), 0); }
}
static void free_world(void) {
  T *e = g_w.e;
#ifdef P42_INTEG
  f_dtor(e);
  return;
#endif
  void *blocks[] = { e->linCoef_, e->linHist_, e->fadeWinA_, e->fadeWinB_, e->dfeCoef_, e->dfeHist_, e->mseHist_, e->mmxIn_, e->mmxOut_, e->rawLinHi_, e->rawLinLo_, e->rawHist16_, e->rawDfeHi_, e->rawDfeLo_,
                     e->rawDfeHist16_ };
  unsigned k;
  for (k = 0; k < sizeof blocks / sizeof blocks[0]; k++) sysdep_free(blocks[k]);
}

/* the scripted results: the bits the function must not look at are garbage; the events are drawn from the ones the function acts on */
static uint32_t pick_from(const uint32_t *t, unsigned n) { return t[h_rmod(n)]; }
static void gen_script(void) {
  static const uint32_t ev3[] = { 3, 8, 9, 10, 11, 12, 13, 14, 15, 16, 20, 4, 5, 6, 7, 17, 18, 19, 2, 21, 0x39, 1, 0xff, 3, 10, 15, 16, 20, 14, 13 };
  static const uint32_t st3[] = { 0, 0, 3, 3, 4, 5, 10, 11, 12, 13, 14, 15, 16, 17, 1, 2, 6, 7, 8, 9, 18, 0xff, 4, 3, 0, 4, 4, 4, 3, 17, 5 };
  static const uint32_t ev4[] = { 0x17, 0x18, 0x1c, 0x1d, 0x28, 0x2c, 0x35, 0x19, 0x1a, 0x1b, 0x1e, 0x23, 0x25, 0x2d, 0x36, 0xff, 1, 0x34, 0x20, 0x1c, 0x1d, 0x28, 0x2c, 0x35, 0x17, 0x18 };
  static const uint32_t st4[] = { 0, 1, 2, 3, 3, 3, 4, 5, 6, 7, 0xff, 2, 3 };
  static const int16_t syms[] = { 0, 1, -1, 100, -100, 32767, -32768, 4000, -4000, 0x7f, 0x1234, 300, -300, 301, -301 };
  unsigned i;
  for (i = 0; i < NSC; i++) {
    uint32_t ev, st, cnt, b424, b;
    g_sc[K_P3A][i] = h_rnd();
    ev = h_rpct(35) ? 0u : pick_from(ev3, sizeof ev3 / sizeof ev3[0]);
    st = pick_from(st3, sizeof st3 / sizeof st3[0]);
    cnt = h_rmod(4);
    b424 = h_rpct(50) ? 1u : h_rpct(50) ? 0u : (h_rnd() & 0xffu);
    g_sc[K_P3B][i] = ev | (st << 8) | (cnt << 16) | (b424 << 24);
    g_sc[K_P3C][i] = h_rpct(80) ? h_rmod(4) : h_rnd();
    g_sc[K_P4A][i] = h_rnd();
    ev = h_rpct(35) ? 0u : pick_from(ev4, sizeof ev4 / sizeof ev4[0]);
    st = pick_from(st4, sizeof st4 / sizeof st4[0]);
    cnt = h_rmod(4);
    b = (uint32_t)(h_rpct(50) ? 1u : 0u) | ((uint32_t)h_rpct(50) << 2) | ((uint32_t)h_rpct(50) << 3);
    g_sc[K_P4B][i] = ev | (st << 8) | (cnt << 16) | (b << 24);
    g_sc[K_P4C][i] = h_rmod(4);
    { uint32_t lo = h_rpct(60) ? (uint16_t)syms[h_rmod(sizeof syms / sizeof syms[0])] : (h_rnd() & 0xffffu); g_sc[K_HD][i] = (h_rnd() & 0xffff0000u) | lo; }
    g_sc[K_RRN][i] = h_rpct(75) ? 0u : (h_rpct(50) ? 1u : h_rnd() | 1u);
    g_sc[K_FPE][i] = h_rpct(75) ? 0u : (h_rpct(50) ? 1u : h_rnd() | 1u);
    g_sc[K_EIA6][i] = h_rnd();
    g_sc[K_EDP][i] = h_rnd(); g_sc[K_ERRN][i] = h_rnd(); g_sc[K_EFPE][i] = h_rnd(); g_sc[K_EP4][i] = h_rnd();
    g_sc[K_CALC][i] = fbits(pick_f());
    g_sc[K_BETA][i] = h_rnd();
    g_sc[K_MISC][i] = h_rnd();
    g_sc[K_PRN][i] = h_rnd();
  }
  memset(g_cur, 0, sizeof g_cur);
}

static int g_small;   /* this call's inputs are small (|x| < 128): the errors stay near the threshold of 300 */
static float wild_f(void) {   /* exponent spread 2^-14 .. 2^15, random mantissa: sums lose bits, so the accumulation ORDER is observable */
  uint32_t v = ((uint32_t)(0x71u + h_rmod(0x1eu)) << 23) | h_rmod(1u << 23);
  if (h_rpct(50)) v |= 0x80000000u;
  return h_float_from_bits(v);
}
static float pick_in(void) {
  uint32_t r = h_rnd();
  if (g_cls == 5) return wild_f();
  if (g_cls == 7 && (r & 3) != 0) {
    static const uint32_t b[] = { 0x46fffe00u, 0x46ffff00u, 0x46ffffffu, 0x47000000u, 0x47000100u, 0xc7000000u, 0xc7000100u, 0xc70000ffu, 0xc7000200u, 0xc7000080u, 0x4effffffu, 0x4f000000u, 0xcf000000u, 0xcf000001u,
                                  0x4f400000u, 0x5f000000u, 0xdf000000u, 0x3f000000u, 0xbf000000u, 0x43958000u, 0x43960000u, 0x43964000u, 0x43968000u, 0xc3960000u, 0xc3960100u, 0x00000000u, 0x80000000u, 0x7fc00000u };
    return h_float_from_bits_quiet(b[h_rmod(sizeof b / sizeof b[0])]);
  }
  if (g_small && (r & 0x3f) >= 2) return (float)((int)h_rmod(257) - 128) * (1.0f / 2.0f);
  if ((r & 0x3f) < 2) { static const uint32_t b[] = { 0, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7f800000u, 0xff800000u, 0x47000000u, 0xc7000100u, 0x46fffe00u, 0xc6fffe00u, 0x7e800000u, 0x00000001u, 0x43960000u, 0x43960001u, 0x4f000000u }; return h_float_from_bits_quiet(b[h_rmod(sizeof b / sizeof b[0])]); }
  if ((r & 0x3f) < 12) return (float)((int)h_rmod(40001) - 20000);                       /* integer valued */
  if ((r & 0x3f) < 30) return (float)((int)h_rmod(2001) - 1000) + (float)h_rmod(1024) * (1.0f / 1024.0f);   /* exact */
  return (float)((int)h_rmod(200001) - 100000) * (1.0f / 16.0f);
}
static unsigned pick_n(void) { return h_rpct(4) ? 0u : h_rpct(70) ? 1u + h_rmod(10) : h_rpct(85) ? 1u + h_rmod(24) : 1u + h_rmod(60); }

/* ------------------------------------------------------------------------------------------------ one call */
static unsigned long g_ncalls;
static int g_showall;
static void dump_foreign(int show) {
  const W *w = &g_w;
  h_emit(show, "p3 st=%u cnt=%u ev=%u status=%u b424=%u | p4 st=%u cnt=%u ev=%u w38=%u se=%u | dem z1eb4=%u dem2 z1ea4=%u z1ea6=%u | rs st=%u fl=%u | ce %d | sv %u",
         w->p3->state_, w->p3->cnt_, w->p3->event_, w->p3->status_, (unsigned)w->p3->b424_, w->p4->state_, w->p4->cnt_, w->p4->event_, w->p4->w38_, w->p4->studyEnd_,
         (unsigned)w->dem->z1eb4_, (unsigned)w->dem2->z1ea4_, (unsigned)w->dem2->z1ea6_, w->rs->state_, w->rs->stateFlag_, (int)w->ce->rrnSilence_, w->sv->flag_);
}
static void do_call(int sc) {
  W *w = &g_w;
  T *e = w->e;
  unsigned n = pick_n(), k, ncap, cnt = 0, mc2 = 0;
  uint32_t seed = h_rnd();
  unsigned long hashline = 0;
  (void)sc; (void)hashline;
  /* the mode: the float-only modes need the float mode at the start of the call */
  if (e->mmx_ != 0) e->mode_ = 3 + h_rmod(3);
  else e->mode_ = h_rpct(8) ? h_rmod(7) : 1u + h_rmod(6);
  if (h_rpct(15) && e->mmx_ == 0) e->mode_ = 3 + h_rmod(3);
  ncap = n / 2 + 4;
  w->n = n; w->nin = ncap;
  w->in = (float *)sysdep_malloc((n + 4) * 4);
  w->dec = (short *)sysdep_malloc(ncap * 2);
  w->out = (float *)sysdep_malloc(ncap * 4);
  g_small = h_rpct(30);
  for (k = 0; k < n + 4; k++) w->in[k] = pick_in();
  for (k = 0; k < ncap; k++) { w->dec[k] = (short)0x5a5a; ((uint32_t *)w->out)[k] = 0xa5a5a5a5u; }
  e->event_ = h_rnd();
  g_fdef = e->mmx_ == 0;
  gen_script();
  g_show = h_sample(&g_symctr, 12, 250) || g_showall;
  /* the model on the live world first (saved and restored), then the impl */
  build_regions();
  save_to(g_sv0);
  g_fdef = e->mmx_ == 0;
  g_log = &g_ml; g_ml.n = 0; memset(g_cur, 0, sizeof g_cur); cov_clear();
  { unsigned mcnt = 0; m_process(w->in, n, w->dec, w->out, &mcnt); mc2 = mcnt; }
  save_to(g_sv1);
  restore_from(g_sv0);
  g_fdef = e->mmx_ == 0;
  g_log = &g_il; g_il.n = 0; memset(g_cur, 0, sizeof g_cur);
  h_emit(g_show, "call sc=%d mode=%u mmx=%u n=%u pend=%u lvl=%u N=%u M=%u L=%u pos=%u/%u", sc, e->mode_, e->mmx_, n, e->pend_, dsplibs_debug_level, e->linLen_, e->dfeLen_, e->linHistLen_, e->linPos_, e->histPos16_);
  f_process(e, w->in, n, w->dec, w->out, &cnt);
  __asm__ volatile("" ::: "memory");
  if (!g_nomodel) { log_cmp(); world_cmp(); }
  if (!g_nomodel && cnt != mc2) h_die3("the symbol count: model %u, impl %u", mc2, cnt);
  cov_flush();
  for (k = 0; k < g_il.n && k < LOGMAX; k++) h_emit(g_show, "%s", g_il.r[k].t);
  h_emit(g_show, "ret cnt=%u", cnt);
  { char b[1024]; size_t o = 0; unsigned q; for (q = 0; q < ncap && o + 24 < sizeof b; q++) o += (size_t)snprintf(b + o, sizeof b - o, "%04x/%08x ", (unsigned)(uint16_t)w->dec[q], fbits(w->out[q])); h_emit(g_show, "bufs %s", b); }
  h_dump_obj(g_show, "E", e, g_fe, g_nfe);
  h_wild = 0;
  dump_foreign(g_show);
  { unsigned i; for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("canary byte %u behind the object was overwritten", i); }
  sysdep_free(w->in); sysdep_free(w->dec); sysdep_free(w->out);
  (void)seed;
  g_ncalls++;
}

static void trial(int sc) {
  unsigned ncalls = sc == 2 ? rnd_range(3, 8) : rnd_range(1, 3), k;
  new_world(sc);
  dsplibs_debug_level = h_rpct(50) ? 2u + h_rmod(2) : (h_rpct(60) ? 0u : h_rnd());
  for (k = 0; k < ncalls; k++) {
    if (k > 0 && h_rpct(15)) dsplibs_debug_level = h_rpct(50) ? 2u + h_rmod(2) : (h_rpct(60) ? 0u : 1u);
    do_call(sc);
  }
  free_world();
  h_trial_end("p40");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, ntr, i;
  const char *seed = getenv("P40_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P40_ALL")) g_showall = 1;
  if (getenv("P40_RZ")) g_rz = 1;
  if (getenv("P40_NOMODEL")) g_nomodel = 1;
#ifdef P42_INTEG
  g_nomodel = 1;   /* the independent model scripts the six callees: with the real P42 functions only blob vs port is compared */
#endif   /* debugging only: do not compare against the model */
  if (getenv("P40_SNAN")) g_snan = 1;
  p05_layout_check(); p07_layout_check(); p17_layout_check(); p21_layout_check(); p28_layout_check(); p34_layout_check(); p37_layout_check(); p40_layout_check();
  for (i = 0; i < NF_EquObj; i++) {
    const char *nm = f_EquObj[i].name;
    if (!strcmp(nm, "resampler_") || !strcmp(nm, "p3_") || !strcmp(nm, "p4_") || !strcmp(nm, "dem_") || !strcmp(nm, "ce_") || !strcmp(nm, "sv_") || !strcmp(nm, "pf_") || !strcmp(nm, "params_")) continue;
    g_fe[g_nfe++] = f_EquObj[i];
  }
  h_x87_check();
  h_arena_init();
  h_rz_by_id = 1;   /* red-zone bytes depend on the block id: a copy of red-zone bytes into a block's red zone (hist[-1] = hist[-2]) is detected */
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(1);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P40_TRIALS") ? atoi(getenv("P40_TRIALS")) : 8000;
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
    return (bad && !getenv("P40_NOCOV")) ? 2 : 0; }
}
