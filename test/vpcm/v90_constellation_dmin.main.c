/*
 * Differential test harness for port chunk P25 (src/vpcm/v90_constellation_dmin.cpp): the rest of class V90ConstellationDesigner - determineDminForRrn, process, constellationDesign,
 * spectralDesign, C1 / C2 / D1 / D2, reset, setMinMaxRates and the dead helpers pow6, maxK, calcK, constelBuild, calcMtoMatchKtarget, realK, findConstelMaxValueIndex, findMinValueIndex,
 * findNextUcodeToAdd, reconstructInitialConditions (20 functions).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_constellation_dmin.sh against the blob (dsplibs.o with edprintf and the four callees of process / constellationDesign
 * - setConstellationToNoise, ..._forceRate, adjustConstellationsPower, adjustConstellationsToNewK (P23 / P24) - WEAK so that the recorders below win), against the port (-O0 / -O2; the P06
 * port object supplies linear2alaw / linear2ulaw / alaw2linear / ulaw2linear) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_constellation_dmin.blob32.out).  The functions are reached
 * through __asm__ labels; the designer object (0x54 B, five pointers), the V90Parameters block (P07 mirror), the mapping blocks (P11 mirror, 0x650 B) and the detector (P18 mirror, 0xa9b0 B,
 * one pointer) are the generated C mirrors; the pointer-holding objects are STATIC / member-wise filled (their sizes differ on x86-64).
 *
 * Scenarios (each its own process):
 *   0 determineDminForRrn (the REAL function against the independent model m_ddfr; blob = port): counts, K around the real K, dMin_, tables, levels, hooks
 *   1 determineDminForRrn, edge regimes (all phases suspected, tiny counts, wild dMin_, large unk360, counts up to 255)
 *   2 process (callees = recorders with a scripted effect on the mapping block; the model m_proc)
 *   3 constellationDesign (the same recorders)
 *   4 ctor / dtor / reset / setMinMaxRates / spectralDesign
 *   5 the small helpers: findConstelMaxValueIndex, findMinValueIndex, reconstructInitialConditions, constelBuild, findNextUcodeToAdd, maxK, calcMtoMatchKtarget
 *   6 the float-returning helpers pow6 / calcK / realK, float-rounded (the golden)
 *   7 (i386 only, NOT in the golden) the same three with the EXTENDED return value read from st(0)
 *
 * What is compared: the whole designer object (pointers excluded), the V90Parameters block, the mapping blocks, the inputs after every call, every edprintf / dsplibs_debug_printf call (kind,
 * text with the integer arguments formatted, a snapshot of the object / block / the parameter words process writes at the moment of the call), every recorded callee call (arguments
 * normalised to table offsets) and the unsigned `level > 1` tests (levels 0, 1, 2, 3, >= 2^31; a hook lowers the level inside the k-th print; two more hooks swap T->params_ / T->mapParams_
 * inside the k-th print: every re-read of the blob is a re-read here).
 *
 * INDEPENDENT MODELS (specification style, long double arithmetic with the same x87 instruction for log10): m_ddfr (the whole determineDminForRrn: normalisation, M search, K checks, the two
 * scans, the nudges, the prints), m_proc (process: rate limits, loop of at most two designs, D / rate / flag8_, forcing), small models of the helpers.  A model that says the blob would not
 * terminate (an exponent loop of 2^32 turns) EXCLUDES the input (counted as `skipped`; a watchdog alarm catches a wrong guess).
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
#include "pcm_parameters_layout.h"
#include "v92_cp_layout.h"
#include "v90_adid_layout.h"
#include "v90_constellation_noise_layout.h"

#define L(m) __asm__(m)

/* the functions under test */
extern void cd_c1(void *, void *, void *, void *) L("_ZN24V90ConstellationDesignerC1EP13V90ParametersP12V90PreFilterP21V90ConstellationPower");
extern void cd_c2(void *, void *, void *, void *) L("_ZN24V90ConstellationDesignerC2EP13V90ParametersP12V90PreFilterP21V90ConstellationPower");
extern void cd_d1(void *) L("_ZN24V90ConstellationDesignerD1Ev");
extern void cd_d2(void *) L("_ZN24V90ConstellationDesignerD2Ev");
extern void cd_reset(void *) L("_ZN24V90ConstellationDesigner5resetEv");
extern void cd_setmm(void *, unsigned, unsigned) L("_ZN24V90ConstellationDesigner14setMinMaxRatesEjj");
extern void cd_ddfr(void *, unsigned) L("_ZN24V90ConstellationDesigner19determineDminForRrnEj");
extern int cd_process(void *, unsigned, void *, float, int, void *, void *, void *, void *, void *, void *, unsigned char, unsigned, unsigned, unsigned)
    L("_ZN24V90ConstellationDesigner7processEjP25V90AutoDigitalImpDetectorfiP16V90MappingParamsPA128_sS5_PsPhS7_h23__tHardwareCodecTypes__j28V90SpecialSpectralConditions");
extern void cd_design(void *, float, void *, void *, void *, void *, void *, void *) L("_ZN24V90ConstellationDesigner19constellationDesignEfPA128_sS1_PsPhS3_PA128_h");
extern void cd_spectral(void *, unsigned, unsigned) L("_ZN24V90ConstellationDesigner14spectralDesignEj28V90SpecialSpectralConditions");
extern int cd_findmax(void *, void *) L("_ZN24V90ConstellationDesigner24findConstelMaxValueIndexEP16V90MappingParams");
extern int cd_findmin(void *, void *) L("_ZN24V90ConstellationDesigner17findMinValueIndexEP16V90MappingParams");
extern void cd_recon(void *, void *, void *) L("_ZN24V90ConstellationDesigner28reconstructInitialConditionsEP16V90MappingParamsPh");
extern unsigned char cd_build(void *, short, short) L("_ZN24V90ConstellationDesigner12constelBuildEss");
extern int cd_nextu(void *, void *, unsigned char, void *, void *, void *, void *) L("_ZN24V90ConstellationDesigner18findNextUcodeToAddEPhhPA128_sS2_PsPA128_h");
extern unsigned cd_maxk(void *, void *) L("_ZN24V90ConstellationDesigner4maxKEP16V90MappingParams");
extern unsigned cd_calcm(void *, float, float) L("_ZN24V90ConstellationDesigner19calcMtoMatchKtargetEff");
/* the float-returning helpers: the blob / port leave the UNROUNDED extended value in st(0) on i386; XRET reads it there (x86-64: the port returns a float) */
#if defined(__i386__)
typedef long double XRET;
#else
typedef float XRET;
#endif
extern float cd_pow6_f(void *, short) L("_ZN24V90ConstellationDesigner4pow6Es");
extern float cd_calck_f(void *, unsigned, float *) L("_ZN24V90ConstellationDesigner5calcKEjPf");
extern float cd_realk_f(void *, void *) L("_ZN24V90ConstellationDesigner5realKEP16V90MappingParams");
extern XRET cd_pow6_x(void *, short) L("_ZN24V90ConstellationDesigner4pow6Es");
extern XRET cd_calck_x(void *, unsigned, float *) L("_ZN24V90ConstellationDesigner5calcKEjPf");
extern XRET cd_realk_x(void *, void *) L("_ZN24V90ConstellationDesigner5realKEP16V90MappingParams");
extern int linear2alaw(int);
extern int linear2ulaw(int);
extern int alaw2linear(int);
extern int ulaw2linear(int);
extern unsigned int dsplibs_debug_level;
/* the interposed callees of process / constellationDesign (P23 / P24): defined below */
extern void h_scn(void *, float, void *, void *, void *, void *, void *) L("_ZN24V90ConstellationDesigner23setConstellationToNoiseEfPA128_sS1_PsPhPA128_h");
extern void h_scnfr(void *, float, void *, void *, void *, void *, void *, void *) L("_ZN24V90ConstellationDesigner33setConstellationToNoise_forceRateEfPA128_sS1_PsPhS3_PA128_h");
extern void h_pow(void *) L("_ZN24V90ConstellationDesigner25adjustConstellationsPowerEv");
extern void h_newk(void *, void *, void *, void *, void *) L("_ZN24V90ConstellationDesigner26adjustConstellationsToNewKEPA128_sS1_PsPA128_h");

typedef long double LD;
typedef V90ParObj ParObj;

static const HFld f_mp[] = {
  HFLD(MapPar90, w0_, 'u'), HFLDN(MapPar90, listA_, 'x', 768), HFLDN(MapPar90, listB_, 'x', 768), HFLDN(MapPar90, cnt_, 'u', 6), HFLD(MapPar90, flag_, 'u'), HFLD(MapPar90, w620_, 'u'),
  HFLD(MapPar90, w624_, 'u'), HFLD(MapPar90, f628_, 'f'), HFLD(MapPar90, f62c_, 'f'), HFLD(MapPar90, f630_, 'f'), HFLD(MapPar90, f634_, 'f'), HFLDN(MapPar90, idx_, 'u', 6),
};
#define NF_mp ((int)(sizeof f_mp / sizeof f_mp[0]))

/* ------------------------------------------------------------------------------------------------ recorders */
#define LOGMAX 400
#define TXTMAX 520
typedef struct { char kind; char text[TXTMAX]; } LogEnt;
static LogEnt g_log[LOGMAX];
static unsigned g_nlog, g_nlog_total_e, g_nlog_total_d, g_nlog_total_c;
static int g_show;
static CdObj *g_snap_t;       /* object / block / parameters snapshotted at every print and callee call */
static MapPar90 *g_snap_mp;
static ParObj *g_snap_par;
enum { HK_NONE = 0, HK_LEVEL, HK_PAR, HK_MP };
static int g_hook_kind;
static unsigned g_hook_k, g_hook_level;   /* HK_LEVEL: level after the k-th dsplibs_debug_printf; HK_PAR / HK_MP: swap after the k-th print of any kind */
static unsigned g_nprints_in_call, g_nd_in_call;
static ParObj *g_alt_par;
static MapPar90 *g_alt_mp;

static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else if (*s == '\t') { out[n++] = '\\'; out[n++] = 't'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
static void snapshot(void) {
  if (g_snap_t) {
    CdObj tmp;
    memcpy(&tmp, g_snap_t, sizeof tmp);
    tmp.params_ = NULL; tmp.mapParams_ = NULL; tmp.adid_ = NULL; tmp.power_ = NULL; tmp.preFilter_ = NULL;
    h_dump_obj(0, "snap", &tmp, f_CdObj, NF_CdObj);
  }
  if (g_snap_mp) h_dump_obj(0, "snapm", g_snap_mp, f_mp, NF_mp);
  if (g_snap_par)
    h_emit(0, "snapp fr=%d pr=%d red=%d rf=%d up=%d dn=%d", g_snap_par->forceRateEnable, g_snap_par->enableDigitalPowerReduction, g_snap_par->enableRedundancyOptimization, g_snap_par->rateForce,
           g_snap_par->enableRrnUp, g_snap_par->enableRrnDown);
}
static void record(char kind, const char *fmt, va_list ap) {
  char buf[TXTMAX * 2];
  vsnprintf(buf, sizeof buf, fmt, ap);
  if (g_nlog < LOGMAX) { g_log[g_nlog].kind = kind; g_log[g_nlog].text[0] = 0; esc_cat(g_log[g_nlog].text, sizeof g_log[g_nlog].text, buf); }
  h_emit(g_show, "%cprintf %s", kind, g_nlog < LOGMAX ? g_log[g_nlog].text : "(many)");
  g_nlog++;
  snapshot();
}
static void do_hook(char kind) {
  g_nprints_in_call++;
  if (kind == 'd') g_nd_in_call++;
  if (!g_hook_k) return;
  if (g_hook_kind == HK_LEVEL && kind == 'd' && g_nd_in_call == g_hook_k) dsplibs_debug_level = g_hook_level;
  if (g_hook_kind == HK_PAR && g_nprints_in_call == g_hook_k && g_snap_t) g_snap_t->params_ = g_alt_par;
  if (g_hook_kind == HK_MP && g_nprints_in_call == g_hook_k && g_snap_t) g_snap_t->mapParams_ = g_alt_mp;
}
void edprintf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  record('e', fmt, ap);
  va_end(ap);
  g_nlog_total_e++;
  do_hook('e');
}
int dsplibs_debug_printf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  record('d', fmt, ap);
  va_end(ap);
  g_nlog_total_d++;
  do_hook('d');
  return 0;
}
static void log_begin(void) { g_nlog = 0; g_nd_in_call = 0; g_nprints_in_call = 0; }

/* ------------------------------------------------------------------------------------------------ small helpers */
#define EXPECT_EQ(what, got, want)                                                                                                   \
  do {                                                                                                                              \
    unsigned long long g_ = (unsigned long long)(got), w_ = (unsigned long long)(want);                                              \
    if (g_ != w_) h_die3("%s: got 0x%llx expected 0x%llx (line %d)", (what), g_, w_, __LINE__);                                     \
  } while (0)
static void relax_reserved(void) { h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0; }
static unsigned pick_level(void) {
  static const unsigned lv[] = { 0, 1, 2, 3, 7, 0x80000000u, 0xffffffffu, 0x80000001u };
  return h_rpct(60) ? 2u : lv[h_rmod(sizeof lv / sizeof lv[0])];
}
static float F(uint32_t bits) { float f; memcpy(&f, &bits, 4); return f; }
/* a float STORE: the blob's fstp to a float slot rounds; a plain (float) cast may stay in an x87 register in this compiler's default excess-precision mode */
static float RF(LD v) { volatile float f = (float)v; return f; }
static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }

/* ------------------------------------------------------------------------------------------------ models: arithmetic */
static LD m_log10(LD x) {   /* fldlg2; fxch; fyl2x */
  LD r, lg2 = 0.30102999566398119521373889472449302677L;
  __asm__("fyl2x" : "=t"(r) : "0"(x), "u"(lg2) : "st(1)");
  return r;
}
static LD m_prod(const MapPar90 *mp) {
  LD p = (LD)mp->cnt_[0];
  int k;
  for (k = 1; k < 6; k++) p = p * (LD)mp->cnt_[k];
  return p;
}
static float m_lg2f(void) { return RF(m_log10(2.0L)); }
static unsigned m_trunc64lo(LD v) {
  if (!(v > -9223372036854775809.0L && v < 9223372036854775808.0L)) return 0;
  return (unsigned)(long long)v;
}
static int m_i16(LD v) {   /* fistp WORD with truncation: 0x8000 for NaN / out of range */
  if (!(v > -32769.0L && v < 32768.0L)) return -32768;
  return (int)(long long)v;
}
static unsigned m_kint(const MapPar90 *mp) {
  LD p = m_prod(mp), lg;
  float den;
  if (p == 0.0L) return 0;
  lg = m_log10(p);
  den = m_lg2f();
  return m_trunc64lo(lg / (LD)den + (LD)F(0x358637bdu));
}
static unsigned char *m_la(MapPar90 *mp, unsigned p, unsigned i) { return (unsigned char *)mp + 4u + p * 0x80u + i; }
static unsigned char *m_lb(MapPar90 *mp, unsigned p, unsigned i) { return (unsigned char *)mp + 0x304u + p * 0x80u + i; }

/* ------------------------------------------------------------------------------------------------ model state */
typedef struct { uint32_t cnt[6]; uint32_t w620; int swap_par, swap_mp; } Eff;
#define NEFF 16
typedef struct MS_ {
  CdObj t;                       /* non-pointer members */
  ParObj *par;                   /* T->params_ (switched by a PAR hook / a callee effect) */
  ParObj parm, altparm;          /* model copies of the real parameter blocks */
  MapPar90 *mp;                  /* T->mapParams_ (a model copy; switched by a MP hook) */
  MapPar90 mpm, altm;            /* model copies of the real blocks */
  const AdidObj *ad;             /* T->adid_ (read-only for ddfr; the model reads pcmType_ / codecIsAlaw_ for process) */
  unsigned level;
  int hook_kind;
  unsigned hook_k, hook_level, nprint, nd;
  LogEnt log[LOGMAX];
  unsigned nlog;
  int hang;                      /* the blob would not terminate / read garbage on this input */
  unsigned char cov[128];
  /* process / constellationDesign */
  Eff eff[NEFF];
  unsigned ncall;
  int retv;
  const unsigned char *A, *B, *dual, *fixedU, *cur, *V;   /* identity of the table pointers (offsets are logged) */
} MS;

static void m_print(MS *s, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (s->nlog < LOGMAX) { s->log[s->nlog].kind = kind; s->log[s->nlog].text[0] = 0; esc_cat(s->log[s->nlog].text, sizeof s->log[s->nlog].text, buf); }
  s->nlog++;
  if (kind == 'c') return;    /* a recorded callee call is not a print: it does not count for the hooks */
  s->nprint++;
  if (kind == 'd') s->nd++;
  if (s->hook_k) {
    if (s->hook_kind == HK_LEVEL && kind == 'd' && s->nd == s->hook_k) s->level = s->hook_level;
    if (s->hook_kind == HK_PAR && s->nprint == s->hook_k) s->par = &s->altparm;
    if (s->hook_kind == HK_MP && s->nprint == s->hook_k) s->mp = &s->altm;
  }
}
#define M_E(s, ...) m_print(s, 'e', __VA_ARGS__)
#define M_D(s, ...) m_print(s, 'd', __VA_ARGS__)
#define COV(s, id) ((s)->cov[id] = 1)

/* ------------------------------------------------------------------------------------------------ model: determineDminForRrn */
enum { CD_PRINT_M16, CD_PH_NONE, CD_DOWN_OK, CD_DOWN_LOW, CD_DOWN_LOW_NUDGE, CD_DOWN_LOW_KEEP, CD_DOWN_ARB, CD_DOWN_SCAN, CD_DOWN_SCAN_MULTI, CD_DOWN_PREV_EQ, CD_DOWN_K1, CD_DOWN_K2, CD_DOWN_K3,
       CD_DOWN_SKIP, CD_DOWN_ITER_CAP, CD_UP_OK, CD_UP_HIGH, CD_UP_HIGH_NUDGE, CD_UP_FAIL, CD_UP_ARB, CD_UP_SCAN, CD_UP_SCAN_MULTI, CD_UP_OK2, CD_UP_TWO, CD_UP_ITER_CAP, CD_SCAN_IN, CD_SCAN_EMPTY,
       CD_HOOK_MP, CD_HOOK_PAR, CD_HOOK_LEVEL, CD_PH_LAST, CD_LAST };
static const char *cdcov_names[CD_LAST] = {
  [CD_PRINT_M16] = "ddfr: m[1..6] print", [CD_PH_NONE] = "ddfr: no eligible phase (maxM 0)", [CD_DOWN_OK] = "ddfr: down kq OK", [CD_DOWN_LOW] = "ddfr: down 'too low'", [CD_DOWN_LOW_NUDGE] = "ddfr: down nudge M+1",
  [CD_DOWN_LOW_KEEP] = "ddfr: down 'too low', no nudge", [CD_DOWN_ARB] = "ddfr: down arbitrary", [CD_DOWN_SCAN] = "ddfr: down scan", [CD_DOWN_SCAN_MULTI] = "ddfr: down scan, several rounds",
  [CD_DOWN_PREV_EQ] = "ddfr: down prevCount == M", [CD_DOWN_K1] = "ddfr: down result tempDmin (kq(count))", [CD_DOWN_K2] = "ddfr: down result tempDmin (kq(prev))", [CD_DOWN_K3] = "ddfr: down result prevDmin",
  [CD_DOWN_ITER_CAP] = "ddfr: down scan hit 100 rounds", [CD_UP_OK] = "ddfr: up kq OK", [CD_UP_HIGH] = "ddfr: up 'too high'", [CD_UP_HIGH_NUDGE] = "ddfr: up nudge M+1",
  [CD_UP_FAIL] = "ddfr: up failed (dMin)", [CD_UP_ARB] = "ddfr: up arbitrary", [CD_UP_SCAN] = "ddfr: up scan", [CD_UP_SCAN_MULTI] = "ddfr: up scan, several rounds", [CD_UP_OK2] = "ddfr: up result tempDmin",
  [CD_UP_TWO] = "ddfr: up 'might cause 2 rates up'", [CD_UP_ITER_CAP] = "ddfr: up scan hit 100 rounds", [CD_SCAN_IN] = "ddfr: a ucode accepted", [CD_SCAN_EMPTY] = "ddfr: scan range empty (unk360 > first ucode)",
};

/* the number of ucodes u in [lo, first ucode of the phase] with distance above a running half distance that the detector allows: `cnt` is a byte */
static unsigned m_scan(MS *s, MapPar90 *mp, int ph, unsigned lo, int tmp) {
  unsigned char cnt = 0;
  int half = tmp / 2 - ((tmp % 2 != 0 && tmp < 0) ? 1 : 0);   /* arithmetic shift right by one */
  unsigned first = *m_la(mp, (unsigned)ph, 0), u;
  const short *lin = (const short *)s->ad;
  const unsigned char *ok = (const unsigned char *)s->ad + 0xd00;
  if (lo > first) { COV(s, CD_SCAN_EMPTY); return 0; }
  for (u = lo; u <= first; u++) {
    short e = lin[(unsigned)ph * 128u + u];
    if (e > (short)half && ok[(unsigned)ph * 128u + u] != 0) {
      cnt++;
      half = (short)(tmp + e);
      COV(s, CD_SCAN_IN);
    }
  }
  return cnt;
}
static LD m_kq(unsigned m, const float *nrm) {
  LD acc = 1.0L;
  int i;
  float lgf, den;
  for (i = 0; i < 6; i++) acc = acc * ((LD)m * (LD)nrm[i]);
  lgf = RF(m_log10(acc));
  den = m_lg2f();
  return (LD)lgf * ((LD)1.0L / (LD)den);
}
/* M = trunc(2^n * 1.0069555^q) for the exponent t; *hang when a loop count would be huge (negative / NaN-free t below zero) */
static unsigned m_mfind(LD t, int *hang) {
  unsigned n = m_trunc64lo(t), p2 = 1, q, c;
  LD pw = 1.0L;
  if (n > 5000u) { *hang = 1; return 0; }
  for (c = 0; c < n; c++) p2 += p2;
  q = m_trunc64lo((t - (LD)n) * (LD)100.0f);
  if (q > 5000u) { *hang = 1; return 0; }
  for (c = 0; c < q; c++) pw = pw * (LD)F(0x3f80e3ebu);
  return m_trunc64lo(pw * (LD)p2);
}

static void m_ddfr(MS *s, unsigned K) {
  MapPar90 *mp;
  unsigned maxM = 0, i, ph = 0, mDown, mUp;
  float nrm[6], lgf, den;
  LD recip, pn = 1.0L, t;
  volatile double pnD;
  /* the 'unrestricted' tests of the blob read the level of the moment: every print is followed by a re-read, so the current level IS the cached one */
  if (s->level > 1) {
    COV(s, CD_PRINT_M16);
    M_D(s, "V90ConstellationDesigner::determineDminForRrn : pParams->m[1..6] = %d %d %d %d %d %d\r\n", (int)s->mp->cnt_[0], (int)s->mp->cnt_[1], (int)s->mp->cnt_[2], (int)s->mp->cnt_[3], (int)s->mp->cnt_[4], (int)s->mp->cnt_[5]);
  }
  mp = s->mp;
  for (i = 0; i < 6; i++)
    if (mp->cnt_[i] > maxM && ((const unsigned char *)s->ad)[0x280c + i] == 0) { maxM = mp->cnt_[i]; ph = i; }
  if (maxM == 0) COV(s, CD_PH_NONE);
  recip = (LD)1.0L / (LD)maxM;
  for (i = 0; i < 6; i++) {
    LD r = (LD)mp->cnt_[i] * recip;
    nrm[i] = RF(r);
    pn = pn * r;
  }
  pnD = (double)pn;
  lgf = RF(m_log10((LD)pnD));
  den = m_lg2f();
  /* ---- down ---- */
  t = (((LD)K - (LD)F(0x3f400000u)) - (LD)lgf * ((LD)1.0L / (LD)den)) * (LD)F(0x3e2aaaabu);
  mDown = m_mfind(t, &s->hang);
  if (s->hang) return;
  if ((LD)K - (LD)F(0x3f4ccccdu) > m_kq(mDown, nrm)) {            /* K - 0.8 > kq: rate down maxM too low */
    unsigned m1;
    COV(s, CD_DOWN_LOW);
    if (s->level > 1) M_D(s, "V90ConstellationDesigner:: rrn down maxM too low - probably will cause more then one rate down\r\n");
    m1 = mDown + 1;
    if ((LD)K - (LD)F(0x3e99999au) > m_kq(m1, nrm)) {              /* K - 0.3 > kq(M + 1): nudge */
      COV(s, CD_DOWN_LOW_NUDGE);
      mDown = m1;
      if (s->level > 1) M_D(s, "V90ConstellationDesigner:: increment maxM for one rate down\r\n");
    } else COV(s, CD_DOWN_LOW_KEEP);
    mp = s->mp;
  } else COV(s, CD_DOWN_OK);
  if (s->level > 1) {
    M_D(s, "V90ConstellationDesigner:: current maxM = %d  rrn down maxM = %d\r\n", (int)mp->cnt_[ph], (int)mDown);
    mp = s->mp;
  }
  if (mp->cnt_[ph] <= mDown) {
    COV(s, CD_DOWN_ARB);
    s->t.rrnDownDmin_ = (uint16_t)m_i16((LD)(short)s->t.dMin_ * (LD)F(0x3fa00000u) + (LD)F(0x3f000000u));
    if (s->level > 1) {
      M_D(s, "V90ConstellationDesigner:: maxM>pParams->m[phase] seting rrnDownDmin arbitrary !!!\r\n");
      if (s->level > 1) M_D(s, "V90ConstellationDesigner:: rrnDownDmin = %d\r\n", (int)(short)s->t.rrnDownDmin_);
    }
  } else {
    unsigned char u = (unsigned char)mp->cnt_[ph], prevCount = 0;
    int tmp = (short)s->t.dMin_, prevDmin = 0;
    unsigned iter = 0, lo;
    const ParObj *P = s->par;
    COV(s, CD_DOWN_SCAN);
    if (u < mDown) { s->hang = 1; return; }   /* the blob reads uninitialised stack slots */
    do {
      prevCount = u;
      prevDmin = tmp;
      tmp = m_i16((LD)(short)prevDmin * (LD)F(0x3f828f5cu) + (LD)1.0f);
      lo = (unsigned)P->unk360;
      u = (unsigned char)m_scan(s, mp, (int)ph, lo, tmp);
      iter++;
    } while (u >= mDown && iter <= 99);
    if (iter > 1) COV(s, CD_DOWN_SCAN_MULTI);
    if (iter == 100) COV(s, CD_DOWN_ITER_CAP);
    if (s->level > 1) {
      M_D(s, "V90ConstellationDesigner:: rrn down - nofUcodes = %d  prevNofUcodes = %d\r\n", (int)u, (int)prevCount);
      if (s->level > 1) M_D(s, "V90ConstellationDesigner:: rrn down - tempDmin  = %d  prevDmin      = %d\r\n", tmp, prevDmin);
    }
    if ((unsigned)prevCount == mDown) { COV(s, CD_DOWN_PREV_EQ); s->t.rrnDownDmin_ = (uint16_t)prevDmin; }
    else if (!((LD)K - (LD)F(0x3f4ccccdu) > m_kq(u, nrm))) { COV(s, CD_DOWN_K1); s->t.rrnDownDmin_ = (uint16_t)tmp; }
    else if (!((LD)K - (LD)F(0x3e99999au) > m_kq(prevCount, nrm))) { COV(s, CD_DOWN_K2); s->t.rrnDownDmin_ = (uint16_t)tmp; }
    else { COV(s, CD_DOWN_K3); s->t.rrnDownDmin_ = (uint16_t)prevDmin; }
    if (s->level > 1) M_D(s, "V90ConstellationDesigner:: rrnDownDmin = %d\r\n", (int)(short)s->t.rrnDownDmin_);
  }
  /* ---- up ---- */
  lgf = RF(m_log10((LD)pnD));
  den = m_lg2f();
  t = (((LD)K + (LD)F(0x3f866666u)) - (LD)lgf / (LD)den) * (LD)F(0x3e2aaaabu);
  mUp = m_mfind(t, &s->hang);
  if (s->hang) return;
  {
    int failed = 0;
    if ((LD)K + (LD)1.0L > m_kq(mUp, nrm)) {
      unsigned m1 = mUp + 1;
      COV(s, CD_UP_HIGH);
      if (s->level > 1) M_D(s, "V90ConstellationDesigner:: maxM too high - probably will not cause rate up\r\n");
      if ((LD)K + (LD)F(0x3fd9999au) > m_kq(m1, nrm)) {
        COV(s, CD_UP_HIGH_NUDGE);
        mUp = m1;
        if (s->level > 1) M_D(s, "V90ConstellationDesigner:: increment maxM for one rate up\r\n");
      } else failed = 1;
    } else COV(s, CD_UP_OK);
    if (s->level > 1) M_D(s, "V90ConstellationDesigner:: current maxM = %d  rrn up maxM = %d\r\n", (int)s->mp->cnt_[ph], (int)mUp);
    if (failed) {
      COV(s, CD_UP_FAIL);
      s->t.rrnUpDmin_ = s->t.dMin_;
      if (s->level > 1) {
        M_D(s, "V90ConstellationDesigner:: failed to find rrnUpDmin => seting rrnUpDmin = dMin !!!\r\n");
        if (s->level > 1) M_D(s, "V90ConstellationDesigner:: rrnUpDmin = %d\r\n", (int)(short)s->t.rrnUpDmin_);
      }
      return;
    }
  }
  mp = s->mp;
  if (mp->cnt_[ph] >= mUp) {
    COV(s, CD_UP_ARB);
    s->t.rrnUpDmin_ = (uint16_t)m_i16((LD)(short)s->t.dMin_ * (LD)F(0x3f666666u) + (LD)F(0x3f000000u));
    if (s->level > 1) {
      M_D(s, "V90ConstellationDesigner:: maxM<pParams->m[phase] seting rrnUpDmin arbitrary !!!\r\n");
      if (s->level > 1) M_D(s, "V90ConstellationDesigner:: rrnUpDmin = %d\r\n", (int)(short)s->t.rrnUpDmin_);
    }
    return;
  }
  {
    unsigned char u = (unsigned char)mp->cnt_[ph];
    int tmp = (short)s->t.dMin_;
    unsigned iter = 0, lo;
    const ParObj *P = s->par;
    COV(s, CD_UP_SCAN);
    if (u < mUp) do {
      tmp = m_i16((LD)(short)tmp * (LD)F(0x3f733333u));
      lo = (unsigned)P->unk360;
      u = (unsigned char)m_scan(s, mp, (int)ph, lo, tmp);
      iter++;
    } while (u < mUp && iter <= 99);
    if (iter > 1) COV(s, CD_UP_SCAN_MULTI);
    if (iter == 100) COV(s, CD_UP_ITER_CAP);
    if (s->level > 1) {
      M_D(s, "V90ConstellationDesigner:: rrn up nofUcodes = %d \r\n", (int)u);
      if (s->level > 1) M_D(s, "V90ConstellationDesigner:: rrn up tempDmin  = %d \r\n", tmp);
    }
    if ((LD)K + (LD)F(0x3fd9999au) >= m_kq(u, nrm)) { COV(s, CD_UP_OK2); s->t.rrnUpDmin_ = (uint16_t)tmp; }
    else {
      COV(s, CD_UP_TWO);
      if (s->level > 1) { s->t.rrnUpDmin_ = (uint16_t)tmp; M_D(s, "V90ConstellationDesigner:: tempDmin might cause 2 rates up => rrnUpDmin = dMin\r\n"); }
      s->t.rrnUpDmin_ = s->t.dMin_;
    }
  }
  if (s->level > 1) M_D(s, "V90ConstellationDesigner:: rrnUpDmin = %d\r\n", (int)(short)s->t.rrnUpDmin_);
}

/* ------------------------------------------------------------------------------------------------ the objects of a trial */
typedef struct { MapPar90 m; uint8_t canary[32]; } MpBox;
static CdObj g_t;
static ParObj g_par, g_altpar;
static AdidObj g_ad;
static MpBox g_mpb, g_altmpb;
static unsigned char g_Abuf[(768 + 256) * 2 + 64], g_Bbuf[(768 + 256) * 2 + 64], g_dualbuf[16 * 2], g_curbuf[16], g_fixbuf[16], g_Vbuf[768 + 256 + 64];
static short *const g_A = (short *)(void *)g_Abuf;
static short *const g_B = (short *)(void *)g_Bbuf;
static short *const g_dual = (short *)(void *)g_dualbuf;
static unsigned long g_dumpctr;
static unsigned g_nskipped, g_ntrials;
enum { C_TRIALS = 90, C_SKIPPED, C_L0, C_L1, C_L2, C_LHI, C_CHAIN, C_ALT_HOOK_MP, C_ALT_HOOK_PAR, C_HOOK_LEVEL };
static void cov_define_trials(void) {
  h_cov_define(C_TRIALS, "trials"); h_cov_define(C_SKIPPED, "skipped (excluded input)"); h_cov_define(C_L0, "level 0"); h_cov_define(C_L1, "level 1");
  h_cov_define(C_L2, "level 2 (prints)"); h_cov_define(C_LHI, "level >= 2^31"); h_cov_define(C_CHAIN, "second call on the same object");
  h_cov_define(C_ALT_HOOK_MP, "mapParams_ hook"); h_cov_define(C_ALT_HOOK_PAR, "params_ hook"); h_cov_define(C_HOOK_LEVEL, "level hook");
}
static int cov_id(int i) { return i < H_COV_STRADDLE ? i : i + 2; }   /* ids 60 / 61 are reserved by the harness library */
static void cov_define_names(const char *const *names, int from, int to) {
  int i;
  for (i = from; i < to; i++) if (names[i]) h_cov_define(cov_id(i), names[i]);
}
static void cov_hit_model(const MS *s, int n) {
  int i;
  for (i = 0; i < n; i++) if (s->cov[i]) h_cov_hit(cov_id(i));
}

static int ucode_value(int alaw, int u) {
  int v = alaw ? alaw2linear((u ^ 0xd5) & 0xff) : ulaw2linear((~u) & 0xff);
  return v < 0 ? -v : v;
}
/* a distance table: the linear value of every ucode (increasing), jagged by `jag` percent, + a random slack of 256 entries behind (indices above 127 run on) */
static void fill_table(short *tab, int alaw, int jag, int scale) {
  int ph, u;
  for (ph = 0; ph < 6; ph++)
    for (u = 0; u < 128; u++) {
      int v = ucode_value(alaw, u) / (scale ? scale : 1);
      if (h_rpct(jag)) v += (int)h_rrange(-40, 40);
      if (h_rpct(jag / 4)) v = (int)h_rrange(-300, 3000);
      if (h_rpct(1)) v = (int)h_rrange(-32768, 32767);
      tab[ph * 128 + u] = (short)v;
    }
  for (u = 0; u < 256; u++) tab[768 + u] = (short)(h_rnd() & 0x7fff);
}
static void t_prefill(CdObj *t) {
  memset(t, 0, sizeof *t);
  t->flag8_ = (uint8_t)h_rnd(); t->unk09_[0] = 0x5a; t->dMin_ = (uint16_t)h_rnd(); t->rrnDownDmin_ = (uint16_t)h_rnd(); t->rrnUpDmin_ = (uint16_t)h_rnd(); t->dMinAlt_ = (uint16_t)h_rnd();
  t->unk12_[0] = 0xa5; t->unk12_[1] = 0x3c;
  t->threshUp_ = F(h_rnd() & 0x3fffffffu); t->threshDown_ = F(h_rnd() & 0x3fffffffu); t->threshRetrain_ = F(h_rnd() & 0x3fffffffu);
  t->w24_ = h_rnd(); t->w28_ = h_rnd(); t->useAlaw_ = h_rnd();
  t->unk34_[0] = 1; t->unk34_[1] = 2; t->unk34_[2] = 3; t->unk34_[3] = 4;
  t->w38_ = (uint8_t)h_rnd(); t->unk39_[0] = 9; t->unk39_[1] = 8; t->unk39_[2] = 7;
  t->w3c_ = h_rnd(); t->w40_ = h_rnd(); t->state_ = h_rnd(); t->maxRate_ = h_rnd(); t->minRate_ = h_rnd();
}
static void mp_canary(MpBox *b) { memset(b->canary, 0xc7, sizeof b->canary); }
static void mp_check(const MpBox *b, const char *what) {
  unsigned i;
  for (i = 0; i < sizeof b->canary; i++) if (b->canary[i] != 0xc7) h_die3("%s: canary byte %u of a mapping block overwritten (0x%02x)", what, i, b->canary[i]);
}
static void fill_words(void *p, size_t n) {   /* member-independent random fill of n bytes (n % 4 == 0) */
  size_t k;
  for (k = 0; k + 4 <= n; k += 4) { uint32_t w = h_rnd(); memcpy((uint8_t *)p + k, &w, 4); }
}
/* the detector: random below the pointer member (identical on every arch), the tables structured */
static void gen_adid(int alaw, int jag, int suspect_pct) {
  int ph, q;
  memset(&g_ad, 0, sizeof g_ad);
  fill_words(&g_ad, 0x2814);
  fill_table((short *)(void *)g_ad.linearMapping_, alaw, jag, h_rpct(30) ? 1 + (int)h_rmod(3) : 1);
  for (ph = 0; ph < 6; ph++)
    for (q = 0; q < 128; q++) g_ad.ucodeAllowed_[ph * 128 + q] = (uint8_t)(h_rpct(88) ? 1 : h_rpct(50) ? 0 : (int)h_rnd());
  for (q = 0; q < 6; q++) g_ad.suspectedPhase_[q] = (uint8_t)(h_rpct(suspect_pct) ? (h_rpct(50) ? 1 : (int)h_rnd() | 1) : 0);
  g_ad.pcmType_ = h_rpct(60) ? (int)h_rmod(2) : (int)h_rnd();
  g_ad.codecIsAlaw_ = h_rpct(60) ? (int)h_rmod(2) : (int)h_rnd();
}
/* a mapping block: random garbage everywhere, then the structure.  regime: 0 nominal (counts 12 .. 127), 1 small counts, 2 equal counts, 3 mixed (a few tiny counts), 4 up to 255 (edge) */
static void gen_block(MapPar90 *mp, int regime) {
  unsigned ph, i, c0 = 8 + h_rmod(110);
  int high = h_rpct(8);
  fill_words(mp, sizeof(MapPar90));
  mp->w620_ = h_rpct(70) ? h_rmod(12) : h_rmod(34);
  for (ph = 0; ph < 6; ph++) {
    unsigned cnt, top;
    unsigned char *la = m_la(mp, ph, 0), *lb = m_lb(mp, ph, 0);
    switch (regime) {
      case 1: cnt = 1 + h_rmod(24); break;
      case 2: cnt = c0; break;
      case 3: cnt = h_rpct(25) ? 1 + h_rmod(4) : 10 + h_rmod(110); break;
      case 4: cnt = h_rpct(50) ? 200 + h_rmod(56) : 1 + h_rmod(255); break;
      default: cnt = 12 + h_rmod(116); break;
    }
    mp->cnt_[ph] = cnt;
    top = h_rpct(60) ? 100 + h_rmod(28) : h_rpct(20) ? h_rmod(256) : (cnt > 127 ? 127 : cnt - 1) + h_rmod(129 - (cnt > 127 ? 127 : cnt));
    if (high) top = h_rpct(50) ? 0x7f + h_rmod(3) : 0xfd + h_rmod(3);
    if (top > 130 && regime != 4 && h_rpct(90)) top = 100 + h_rmod(28);
    for (i = 0; i < 128; i++) la[i] = (unsigned char)(i < cnt ? top : h_rnd());
    for (i = 0; i < cnt && i < 128; i++) { la[i] = (unsigned char)top; if (top > 2 && h_rpct(85)) top -= 1 + h_rmod(3); }
    for (i = 0; i < 128; i++) lb[i] = (unsigned char)h_rnd();
  }
}
static unsigned pick_k(const MapPar90 *mp) {
  LD p = m_prod(mp);
  unsigned k0 = p == 0.0L ? 0u : m_trunc64lo(m_log10(p) / m_log10(2.0L));
  unsigned r = h_rmod(100);
  if (r < 40) return k0;
  if (r < 60) return k0 + 1;
  if (r < 75) return k0 - 1;
  if (r < 85) return k0 + 2;
  if (r < 92) return k0 - 2;
  if (r < 96) return k0 + 3 + h_rmod(4);
  return h_rpct(50) ? h_rmod(60) : h_rnd();
}
static void gen_par(ParObj *p, int unk360) {
  memset(p, 0, sizeof *p);
  p->unk360 = unk360;
  p->unk39c = (int)h_rnd();
}

/* ------------------------------------------------------------------------------------------------ comparison helpers */
static void model_init(MS *s, unsigned level, int hook_kind, unsigned hook_k, unsigned hook_level) {
  memset(s, 0, sizeof *s);
  memcpy(&s->t, &g_t, sizeof s->t);
  memcpy(&s->parm, &g_par, sizeof s->parm); memcpy(&s->altparm, &g_altpar, sizeof s->altparm);
  memcpy(&s->mpm, &g_mpb.m, sizeof s->mpm); memcpy(&s->altm, &g_altmpb.m, sizeof s->altm);
  s->par = &s->parm; s->mp = &s->mpm; s->ad = &g_ad;
  s->level = level; s->hook_kind = hook_kind; s->hook_k = hook_k; s->hook_level = hook_level;
}
static void compare_run(const char *what, MS *s, int lvl, int hk, unsigned hkk) {
  CdObj tm, tr;
  unsigned i;
  memcpy(&tm, &s->t, sizeof tm); memcpy(&tr, &g_t, sizeof tr);
  tm.params_ = tm.mapParams_ = tm.adid_ = tm.power_ = tm.preFilter_ = NULL;
  tr.params_ = tr.mapParams_ = tr.adid_ = tr.power_ = tr.preFilter_ = NULL;
  for (i = 0; i < sizeof tm; i++) if (((uint8_t *)&tm)[i] != ((uint8_t *)&tr)[i]) h_die3("%s: object byte +0x%x: real 0x%02x model 0x%02x (level %d hook %d/%u)", what, i, ((uint8_t *)&tr)[i], ((uint8_t *)&tm)[i], lvl, hk, hkk);
  if ((g_t.params_ == (void *)&g_altpar) != (s->par == &s->altparm)) h_die3("%s: params_ pointer after the call", what);
  if ((g_t.mapParams_ == (void *)&g_altmpb.m) != (s->mp == &s->altm)) h_die3("%s: mapParams_ pointer after the call", what);
  if (memcmp(&g_par, &s->parm, sizeof g_par)) { for (i = 0; i < sizeof g_par; i++) if (((uint8_t *)&g_par)[i] != ((uint8_t *)&s->parm)[i]) h_die3("%s: params byte +0x%x: real 0x%02x model 0x%02x", what, i, ((uint8_t *)&g_par)[i], ((uint8_t *)&s->parm)[i]); }
  if (memcmp(&g_altpar, &s->altparm, sizeof g_altpar)) h_die3("%s: the alternate params block differs", what);
  for (i = 0; i < sizeof(MapPar90); i++) {
    if (((uint8_t *)&g_mpb.m)[i] != ((uint8_t *)&s->mpm)[i]) h_die3("%s: mapping block byte +0x%x: real 0x%02x model 0x%02x", what, i, ((uint8_t *)&g_mpb.m)[i], ((uint8_t *)&s->mpm)[i]);
    if (((uint8_t *)&g_altmpb.m)[i] != ((uint8_t *)&s->altm)[i]) h_die3("%s: alt mapping block byte +0x%x: real 0x%02x model 0x%02x", what, i, ((uint8_t *)&g_altmpb.m)[i], ((uint8_t *)&s->altm)[i]);
  }
  mp_check(&g_mpb, what); mp_check(&g_altmpb, what);
  if (g_nlog != s->nlog) {
    for (i = 0; i < g_nlog && i < 14; i++) fprintf(stderr, "real %u %c %s\n", i, g_log[i].kind, g_log[i].text);
    for (i = 0; i < s->nlog && i < 14; i++) fprintf(stderr, "model %u %c %s\n", i, s->log[i].kind, s->log[i].text);
    h_die3("%s: print count: real %u model %u (level %d hook %d/%u)", what, g_nlog, s->nlog, lvl, hk, hkk);
  }
  for (i = 0; i < g_nlog && i < LOGMAX; i++)
    if (g_log[i].kind != s->log[i].kind || strcmp(g_log[i].text, s->log[i].text))
      h_die3("%s: print %u differs (level %d hook %d/%u):\n  real  %c %s\n  model %c %s", what, i, lvl, hk, hkk, g_log[i].kind, g_log[i].text, s->log[i].kind, s->log[i].text);
}
static void dump_all(const char *tag, int with_alt) {
  CdObj tmp;
  int show = h_sample(&g_dumpctr, 40, 97);
  memcpy(&tmp, &g_t, sizeof tmp);
  tmp.params_ = NULL; tmp.mapParams_ = NULL; tmp.adid_ = NULL; tmp.power_ = NULL; tmp.preFilter_ = NULL;
  h_dump_obj(show, tag, &tmp, f_CdObj, NF_CdObj);
  h_dump_obj(show, "mp", &g_mpb.m, f_mp, NF_mp);
  if (with_alt) h_dump_obj(show, "alt", &g_altmpb.m, f_mp, NF_mp);
  h_emit(show, "par fr=%d pr=%d red=%d rf=%d up=%d dn=%d", g_par.forceRateEnable, g_par.enableDigitalPowerReduction, g_par.enableRedundancyOptimization, g_par.rateForce, g_par.enableRrnUp, g_par.enableRrnDown);
}

/* ------------------------------------------------------------------------------------------------ scenarios 0 / 1: determineDminForRrn */
typedef struct { unsigned K, level; int hook_kind; unsigned hook_k, hook_level; } DCall;
static int do_dd(const DCall *c0, int emit, int chain) {
  MS ms;
  DCall c = *c0;
  unsigned i;
  /* steering: a hook position is chosen from a model run without the hook (a print index), then the model runs again WITH the hook */
  {
    unsigned np = 0, nd = 0;
    model_init(&ms, c.level, HK_NONE, 0, 0);
    m_ddfr(&ms, c.K);
    if (ms.hang) { g_nskipped++; h_cov_hit(C_SKIPPED); return 0; }
    if (c.hook_kind == HK_NONE && c.level > 1 && h_rpct(35)) {   /* rare prints (the failed / arbitrary / nudge branches): a hook right after them */
      static const char *const rare[] = { "V90ConstellationDesigner:: failed to find", "V90ConstellationDesigner:: tempDmin might cause", "V90ConstellationDesigner:: maxM>pParams",
        "V90ConstellationDesigner:: maxM<pParams", "V90ConstellationDesigner:: rrn down maxM too low", "V90ConstellationDesigner:: increment maxM", "V90ConstellationDesigner:: maxM too high" };
      unsigned cand[LOGMAX], nc = 0, pi = 0, di = 0, n, q;
      int hk = (int)(1 + h_rmod(3));
      for (n = 0; n < ms.nlog && n < LOGMAX; n++) {
        if (ms.log[n].kind == 'c') continue;
        pi++;
        if (ms.log[n].kind == 'd') di++;
        for (q = 0; q < sizeof rare / sizeof rare[0]; q++)
          if (!strncmp(ms.log[n].text, rare[q], strlen(rare[q]))) { cand[nc++] = hk == HK_LEVEL ? di : pi; break; }
      }
      if (nc) { c.hook_kind = hk; c.hook_k = cand[h_rmod(nc)]; c.hook_level = h_rpct(60) ? 1u : (unsigned)h_rmod(3); }
    }
  }
  if (c.hook_kind != HK_NONE && !c.hook_k) {
    unsigned np = 0, nd = 0;
    np = ms.nprint; nd = ms.nd;
    c.hook_k = c.hook_kind == HK_LEVEL ? (nd ? 1 + h_rmod(nd) : 0) : (np ? 1 + h_rmod(np) : 0);
    if (h_rpct(60)) {   /* steering: put the hook on one of the prints after which the blob re-reads the level / mapParams_ (the reload points of the up and down halves) */
      static const char *const named[] = { "V90ConstellationDesigner:: rrn down maxM too low", "V90ConstellationDesigner:: increment maxM for one rate down", "V90ConstellationDesigner:: current maxM",
        "V90ConstellationDesigner:: maxM too high", "V90ConstellationDesigner:: increment maxM for one rate up", "V90ConstellationDesigner:: failed to find rrnUpDmin", "V90ConstellationDesigner:: maxM>pParams",
        "V90ConstellationDesigner:: maxM<pParams", "V90ConstellationDesigner:: rrn down - nofUcodes", "V90ConstellationDesigner:: rrn up nofUcodes", "V90ConstellationDesigner:: tempDmin might cause",
        "V90ConstellationDesigner:: rrnDownDmin", "V90ConstellationDesigner:: rrnUpDmin", "V90ConstellationDesigner:: rrn down - tempDmin" };
      unsigned cand[LOGMAX], nc = 0, pi = 0, di = 0, n, q;
      for (n = 0; n < ms.nlog && n < LOGMAX; n++) {
        if (ms.log[n].kind == 'c') continue;
        pi++;
        if (ms.log[n].kind == 'd') di++;
        for (q = 0; q < sizeof named / sizeof named[0]; q++)
          if (!strncmp(ms.log[n].text, named[q], strlen(named[q]))) { if (c.hook_kind != HK_LEVEL || ms.log[n].kind == 'd') cand[nc++] = c.hook_kind == HK_LEVEL ? di : pi; break; }
      }
      if (nc) c.hook_k = cand[h_rmod(nc)];
    }
    if (!c.hook_k) c.hook_kind = HK_NONE;
  }
  model_init(&ms, c.level, c.hook_kind, c.hook_k, c.hook_level);
  m_ddfr(&ms, c.K);
  if (ms.hang) { g_nskipped++; h_cov_hit(C_SKIPPED); return 0; }
  g_t.params_ = &g_par; g_t.mapParams_ = &g_mpb.m; g_t.adid_ = &g_ad; g_t.power_ = NULL; g_t.preFilter_ = NULL;
  g_alt_par = &g_altpar; g_alt_mp = &g_altmpb.m;
  g_snap_t = &g_t; g_snap_mp = &g_mpb.m; g_snap_par = &g_par;
  g_hook_kind = c.hook_kind; g_hook_k = c.hook_k; g_hook_level = c.hook_level;
  dsplibs_debug_level = c.level;
  log_begin();
  cd_ddfr(&g_t, c.K);
  g_hook_k = 0; g_snap_t = NULL; g_snap_mp = NULL; g_snap_par = NULL;
  compare_run("ddfr", &ms, (int)c.level, c.hook_kind, c.hook_k);
  cov_hit_model(&ms, CD_LAST);
  h_cov_hit(C_TRIALS);
  if (c.level == 0) h_cov_hit(C_L0); else if (c.level == 1) h_cov_hit(C_L1); else if (c.level >= 0x80000000u) h_cov_hit(C_LHI); else h_cov_hit(C_L2);
  if (chain) h_cov_hit(C_CHAIN);
  if (c.hook_kind == HK_MP && c.hook_k) { h_cov_hit(C_ALT_HOOK_MP); }
  if (c.hook_kind == HK_PAR && c.hook_k) h_cov_hit(C_ALT_HOOK_PAR);
  if (c.hook_kind == HK_LEVEL && c.hook_k) h_cov_hit(C_HOOK_LEVEL);
  if (emit) {
    h_emit(0, "call ddfr K=%u prints %u", c.K, g_nlog);
    dump_all("T", c.hook_kind == HK_MP);
  }
  (void)i;
  return 1;
}
static void gen_alt(const MapPar90 *main) {
  int k;
  memcpy(&g_altmpb.m, main, sizeof(MapPar90));
  if (h_rpct(50)) g_altmpb.m.cnt_[h_rmod(6)] = 1 + h_rmod(130);
  if (h_rpct(30)) for (k = 0; k < 6; k++) g_altmpb.m.cnt_[k] += h_rmod(3);
  if (h_rpct(30)) g_altmpb.m.cnt_[h_rmod(6)] = h_rnd();
  if (h_rpct(55)) for (k = 0; k < 6; k++) g_altmpb.m.cnt_[k] = main->cnt_[k] + 1 + h_rmod(6);   /* every count differs: a stale read of mapParams_ is visible whatever the phase */
}
static void sc_ddfr(int edge, int ntr) {
  int tr;
  cov_define_trials();
  cov_define_names(cdcov_names, 0, CD_LAST);
  h_scenario_begin(0);
  relax_reserved();
  if (!edge) h_covreq[cov_id(CD_PH_NONE)] = 0;
  if (edge) { h_covreq[cov_id(CD_DOWN_ITER_CAP)] = 0; h_covreq[cov_id(CD_UP_ITER_CAP)] = 0; }
  mp_canary(&g_mpb); mp_canary(&g_altmpb);
  for (tr = 0; tr < ntr; tr++) {
    int alaw = h_rpct(50), regime;
    DCall c;
    t_prefill(&g_t);
    regime = edge ? (h_rpct(30) ? 4 : h_rpct(50) ? 1 : 3) : (h_rpct(50) ? 0 : h_rpct(40) ? 2 : h_rpct(50) ? 3 : 1);
    gen_block(&g_mpb.m, regime);
    gen_alt(&g_mpb.m);
    gen_adid(alaw, h_rpct(40) ? 0 : (int)h_rmod(60), edge && h_rpct(30) ? 60 : h_rpct(35) ? 25 : 0);
    gen_par(&g_par, h_rpct(70) ? (int)h_rmod(4) : h_rpct(60) ? (int)h_rmod(130) : (int)h_rnd());
    gen_par(&g_altpar, (int)h_rmod(5));
    g_t.dMin_ = (uint16_t)(h_rpct(60) ? (int)h_rrange(20, 400) : h_rpct(60) ? (int)h_rrange(-40, 3000) : edge ? (int)h_rnd() : (int)h_rrange(1, 90));
    if (edge && h_rpct(15)) g_t.dMin_ = (uint16_t)(h_rpct(50) ? 32700 + h_rmod(68) : 0x8000 + h_rmod(5));
    c.K = pick_k(&g_mpb.m);
    c.level = pick_level();
    c.hook_kind = h_rpct(25) ? (int)(1 + h_rmod(3)) : HK_NONE;
    c.hook_k = 0; c.hook_level = h_rpct(60) ? 1u : (unsigned)h_rmod(3);
    if (do_dd(&c, 1, 0) && h_rpct(30)) {   /* a second call on the same object (the object keeps rrn*Dmin_ of the first) */
      c.K = pick_k(&g_mpb.m); c.level = pick_level(); c.hook_kind = HK_NONE;
      do_dd(&c, 1, 1);
    }
  }
}


/* ------------------------------------------------------------------------------------------------ the recorded callees of process / constellationDesign */
static Eff g_eff[NEFF];
static unsigned g_ncall_real;
static const char *norm_ptr(char *buf, const void *p) {
  const unsigned char *q = (const unsigned char *)p;
  if (!q) { strcpy(buf, "NULL"); return buf; }
  if (q >= g_Abuf && q < g_Abuf + sizeof g_Abuf) sprintf(buf, "A+%d", (int)(q - g_Abuf));
  else if (q >= g_Bbuf && q < g_Bbuf + sizeof g_Bbuf) sprintf(buf, "B+%d", (int)(q - g_Bbuf));
  else if (q >= g_dualbuf && q < g_dualbuf + sizeof g_dualbuf) sprintf(buf, "D+%d", (int)(q - g_dualbuf));
  else if (q >= g_curbuf && q < g_curbuf + sizeof g_curbuf) sprintf(buf, "U+%d", (int)(q - g_curbuf));
  else if (q >= g_fixbuf && q < g_fixbuf + sizeof g_fixbuf) sprintf(buf, "X+%d", (int)(q - g_fixbuf));
  else if (q >= g_Vbuf && q < g_Vbuf + sizeof g_Vbuf) sprintf(buf, "V+%d", (int)(q - g_Vbuf));
  else if (q >= (const unsigned char *)g_ad.ucodeAllowed_ && q < (const unsigned char *)g_ad.ucodeAllowed_ + sizeof g_ad.ucodeAllowed_) sprintf(buf, "AV+%d", (int)(q - (const unsigned char *)g_ad.ucodeAllowed_));
  else strcpy(buf, "?");
  return buf;
}
static void callee_text(char *out, size_t cap, const char *name, uint32_t noisebits, int hasnoise, const void *a, const void *b, const void *dual, const void *fx, const void *cur, const void *v) {
  char x[6][40];
  snprintf(out, cap, "%s", name);
  if (hasnoise) snprintf(out + strlen(out), cap - strlen(out), " noise=%08x", noisebits);
  if (a) snprintf(out + strlen(out), cap - strlen(out), " A=%s B=%s dual=%s", norm_ptr(x[0], a), norm_ptr(x[1], b), norm_ptr(x[2], dual));
  if (fx) snprintf(out + strlen(out), cap - strlen(out), " fixed=%s", norm_ptr(x[3], fx));
  if (cur) snprintf(out + strlen(out), cap - strlen(out), " cur=%s", norm_ptr(x[4], cur));
  if (v) snprintf(out + strlen(out), cap - strlen(out), " V=%s", norm_ptr(x[5], v));
}
static void effect_real(CdObj *t) {
  Eff *e = &g_eff[g_ncall_real < NEFF ? g_ncall_real : NEFF - 1];
  MapPar90 *mp = (MapPar90 *)t->mapParams_;
  g_ncall_real++;
  if (e->w620 != 0xffffffffu) {
    int k;
    for (k = 0; k < 6; k++) mp->cnt_[k] = e->cnt[k];
    mp->w620_ = e->w620;
  }
  if (e->swap_par) t->params_ = g_alt_par;
  if (e->swap_mp) t->mapParams_ = g_alt_mp;
}
static void rec_callee(CdObj *t, const char *txt) {
  if (t != g_snap_t) h_die3("a callee was called with this %p, expected %p", (void *)t, (void *)g_snap_t);
  if (g_nlog < LOGMAX) { g_log[g_nlog].kind = 'c'; g_log[g_nlog].text[0] = 0; esc_cat(g_log[g_nlog].text, sizeof g_log[g_nlog].text, txt); }
  h_emit(g_show, "cprintf %s", txt);
  g_nlog++;
  g_nlog_total_c++;
  snapshot();
  effect_real(t);
}
void h_scn(void *t, float noise, void *A, void *B, void *dual, void *cur, void *V) {
  char buf[200];
  callee_text(buf, sizeof buf, "scn", h_bits_of(noise), 1, A, B, dual, NULL, cur, V);
  rec_callee((CdObj *)t, buf);
}
void h_scnfr(void *t, float noise, void *A, void *B, void *dual, void *fx, void *cur, void *V) {
  char buf[200];
  callee_text(buf, sizeof buf, "scnfr", h_bits_of(noise), 1, A, B, dual, fx, cur, V);
  rec_callee((CdObj *)t, buf);
}
void h_pow(void *t) { rec_callee((CdObj *)t, "pow"); }
void h_newk(void *t, void *A, void *B, void *dual, void *V) {
  char buf[200];
  callee_text(buf, sizeof buf, "newk", 0, 0, A, B, dual, NULL, NULL, V);
  rec_callee((CdObj *)t, buf);
}
static void m_callee(MS *s, const char *txt) {
  Eff *e = &s->eff[s->ncall < NEFF ? s->ncall : NEFF - 1];
  s->ncall++;
  m_print(s, 'c', "%s", txt);
  if (e->w620 != 0xffffffffu) {
    int k;
    for (k = 0; k < 6; k++) s->mp->cnt_[k] = e->cnt[k];
    s->mp->w620_ = e->w620;
  }
  if (e->swap_par) s->par = &s->altparm;
  if (e->swap_mp) s->mp = &s->altm;
}

/* ------------------------------------------------------------------------------------------------ model: process / constellationDesign */
typedef struct {
  unsigned S;
  float noise;
  int mask;
  short *A, *B, *dual;
  unsigned char *fixedU, *cur;
  unsigned char plim;
  unsigned codec, arg13, special;
  void *V;     /* constellationDesign only (process takes the detector's table) */
} PArgs;
enum { PR_MASKED, PR_NOTMASKED, PR_D_LE_14, PR_D_GT_2A, PR_D_NORM, PR_RATE_HI, PR_RATE_LO, PR_RATE_OK, PR_LOOP2, PR_LOOP2_FR, PR_K_ZERO, PR_K_LIMIT, PR_FLAG8_INC, PR_FLAG8_NOINC, PR_SPEC_PBX, PR_SPEC_NORMAL,
       PR_POW_CALL, PR_NEWK_CALL, PR_SCN_CALL, PR_SCNFR_CALL, PR_MIN_CLAMP_LO, PR_MIN_CLAMP_HI, PR_MAX_CLAMP_LO, PR_MAX_CLAMP_HI, PR_MIN_GT_MAX, PR_RRNUP_SET, PR_RRNDN_SET, PR_RRNUP_CLR, PR_RRNDN_CLR,
       PR_W24_SET, PR_W24_KEEP, PR_SWAP_MP, PR_SWAP_PAR, PR_RETV1, PR_LOOP3_EXIT, PR_SHIFT_WRAP, PR_ID_MIN_LOOK, PR_ID_MIN_ID, PR_LAST };
static const char *prcov_names[PR_LAST] = {
  [PR_MASKED] = "process: rate masked by the provider", [PR_NOTMASKED] = "process: rate allowed by the mask", [PR_D_LE_14] = "process: D <= 0x14 (design ERROR)", [PR_D_GT_2A] = "process: D > 0x2a (capped)",
  [PR_D_NORM] = "process: D in range", [PR_RATE_HI] = "process: rate > maxRate (forced)", [PR_RATE_LO] = "process: rate < minRate (forced)", [PR_RATE_OK] = "process: rate within limits",
  [PR_LOOP2] = "process: second design round", [PR_LOOP2_FR] = "process: second round with forceRate", [PR_K_ZERO] = "process: count product 0", [PR_K_LIMIT] = "process: K limited",
  [PR_FLAG8_INC] = "process: flag8_ incremented", [PR_FLAG8_NOINC] = "process: flag8_ not incremented", [PR_SPEC_PBX] = "process: German PBX shaper", [PR_SPEC_NORMAL] = "process: normal shaper",
  [PR_POW_CALL] = "process: adjustConstellationsPower called", [PR_NEWK_CALL] = "process: adjustConstellationsToNewK called", [PR_SCN_CALL] = "process: setConstellationToNoise called",
  [PR_SCNFR_CALL] = "process: forceRate variant called", [PR_MIN_CLAMP_LO] = "process: minRate < 28000", [PR_MIN_CLAMP_HI] = "process: minRate > 56000", [PR_MAX_CLAMP_LO] = "process: maxRate < 28000",
  [PR_MAX_CLAMP_HI] = "process: maxRate > 56000", [PR_MIN_GT_MAX] = "process: minRate > maxRate", [PR_RRNUP_SET] = "process: enableRrnUp = 1", [PR_RRNDN_SET] = "process: enableRrnDown = 1",
  [PR_RRNUP_CLR] = "process: enableRrnUp = 0", [PR_RRNDN_CLR] = "process: enableRrnDown = 0", [PR_W24_SET] = "process: w24_ = rate", [PR_W24_KEEP] = "process: w24_ kept",
  [PR_SWAP_MP] = "process: mapParams_ swapped by a callee", [PR_SWAP_PAR] = "process: params_ swapped by a callee", [PR_RETV1] = "process: returns 1", [PR_LOOP3_EXIT] = "process: loop ended by the round limit",
  [PR_SHIFT_WRAP] = "process: shift count wraps (D - 0x15 negative)",
};
static unsigned m_u(LD v) { return m_trunc64lo(v); }
static void m_proc(MS *s, const PArgs *a) {
  MapPar90 *argmp = &s->mpm;
  unsigned rate = 0, kk = 0, mn, mx;
  int cnt = 1, done = 0, rounds = 0;
  char buf[200];
  s->retv = 0;
  s->t.w28_ = (uint32_t)s->ad->pcmType_;
  s->t.w40_ = a->arg13;
  s->t.w3c_ = a->codec;
  s->t.useAlaw_ = (uint32_t)s->ad->codecIsAlaw_;
  s->t.w38_ = a->plim;
  s->mp = argmp;
  argmp->flag_ = 1;
  M_E(s, "V90ConstellationDesigner: digital rate mask %d\r\n", a->mask);
  {
    const ParObj *P = s->par;
    MapPar90 *m = s->mp;
    uint32_t id, w;
    if (a->special == 2) {
      COV(s, PR_SPEC_PBX);
      id = (uint32_t)P->germanPbxSpectralShaperId;
      m->w624_ = id < a->S ? id : a->S;
      memcpy(&w, &P->germanPbxSpectralShaperA1, 4); memcpy(&m->f628_, &w, 4);
      m->w620_ = (uint32_t)P->germanPbxSpectralShaperSr;
      memcpy(&w, &P->germanPbxSpectralShaperA2, 4); memcpy(&m->f62c_, &w, 4);
      memcpy(&w, &P->germanPbxSpectralShaperB1, 4); memcpy(&m->f630_, &w, 4);
      memcpy(&w, &P->germanPbxSpectralShaperB2, 4); memcpy(&m->f634_, &w, 4);
    } else {
      COV(s, PR_SPEC_NORMAL);
      id = (uint32_t)P->spectralShaperId;
      m->w624_ = id < a->S ? id : a->S;
      memcpy(&w, &P->spectralShaperA1, 4); memcpy(&m->f628_, &w, 4);
      m->w620_ = (uint32_t)P->spectralShaperSr;
      memcpy(&w, &P->spectralShaperA2, 4); memcpy(&m->f62c_, &w, 4);
      memcpy(&w, &P->spectralShaperB1, 4); memcpy(&m->f630_, &w, 4);
      memcpy(&w, &P->spectralShaperB2, 4); memcpy(&m->f634_, &w, 4);
    }
  }
  if (s->t.minRate_ < 28000u) { COV(s, PR_MIN_CLAMP_LO); s->t.minRate_ = 28000u; } else if (s->t.minRate_ > 56000u) { COV(s, PR_MIN_CLAMP_HI); s->t.minRate_ = 56000u; }
  if (s->t.maxRate_ < 28000u) { COV(s, PR_MAX_CLAMP_LO); s->t.maxRate_ = 28000u; } else if (s->t.maxRate_ > 56000u) { COV(s, PR_MAX_CLAMP_HI); s->t.maxRate_ = 56000u; }
  if (s->t.minRate_ > s->t.maxRate_) { COV(s, PR_MIN_GT_MAX); s->t.minRate_ = s->t.maxRate_; }
  while (!done) {
    ParObj *P = s->par;
    MapPar90 *m2;
    uint32_t D, K;
    int masked;
    char na[40];
    rounds++;
    cnt = (short)(cnt + 1);
    if (rounds == 2) COV(s, PR_LOOP2);
    {
      uint32_t nb;
      memcpy(&nb, &a->noise, 4);
      if (P->forceRateEnable == 0) {
        COV(s, PR_SCN_CALL);
        callee_text(buf, sizeof buf, "scn", nb, 1, a->A, a->B, a->dual, NULL, a->cur, g_ad.ucodeAllowed_);
      } else {
        COV(s, PR_SCNFR_CALL);
        if (rounds == 2) COV(s, PR_LOOP2_FR);
        callee_text(buf, sizeof buf, "scnfr", nb, 1, a->A, a->B, a->dual, a->fixedU, a->cur, g_ad.ucodeAllowed_);
      }
      m_callee(s, buf);
    }
    P = s->par;
    if (P->enableDigitalPowerReduction != 0) { COV(s, PR_POW_CALL); m_callee(s, "pow"); P = s->par; }
    if (P->enableRedundancyOptimization != 0) {
      COV(s, PR_NEWK_CALL);
      callee_text(buf, sizeof buf, "newk", 0, 0, a->A, a->B, a->dual, NULL, NULL, g_ad.ucodeAllowed_);
      m_callee(s, buf);
    }
    m2 = s->mp;
    if (m2 != argmp) COV(s, PR_SWAP_MP);
    if (s->par != &s->parm) COV(s, PR_SWAP_PAR);
    {
      LD prod = m_prod(argmp);
      unsigned limit = (unsigned char)(0x2au - (unsigned char)(6u - (unsigned char)m2->w620_));
      K = prod == 0.0L ? 0u : m_u(m_log10(prod) / (LD)m_lg2f() + (LD)F(0x358637bdu));
      if (prod == 0.0L) COV(s, PR_K_ZERO);
      if (K > limit) { COV(s, PR_K_LIMIT); K = limit; }
    }
    D = K - m2->w620_ + 6u;
    if (D > 0x2au) { COV(s, PR_D_GT_2A); m2->w0_ = 0x2au; K = m2->w620_ + 0x24u; } else m2->w0_ = D;
    if (((m2->w0_ - 0x15u) & 0x1f) != ((m2->w0_ - 0x15u))) COV(s, PR_SHIFT_WRAP);
    masked = ((a->mask >> ((m2->w0_ - 0x15u) & 0x1f)) & 1) == 0;
    if (masked) {
      int q;
      COV(s, PR_MASKED);
      for (q = 0; q < 3; q++) M_E(s, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
      M_E(s, "!!!!!!!!!!!!!!!!!!    V90ConstellationDesigner: Rate Used Masked By Provider    !!!!!!!!!!!!!!!!!!\r\n");
      for (q = 0; q < 3; q++) M_E(s, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
      m2 = s->mp;
    } else COV(s, PR_NOTMASKED);
    rate = (m2->w0_ * 8000u) / 6u;
    {
      unsigned char f = (unsigned char)(0x2au - (unsigned char)m2->w0_);
      if (f > 4) { f++; COV(s, PR_FLAG8_INC); } else COV(s, PR_FLAG8_NOINC);
      s->t.flag8_ = f;
    }
    if (m2->w0_ <= 0x14u) {
      COV(s, PR_D_LE_14);
      M_E(s, "V90ConstellationDesigner: Connection design ERROR, D choosen is smaller than minimum.\r\n");
      s->retv = 1; COV(s, PR_RETV1);
    } else {
      if (m2->w0_ <= 0x2au) COV(s, PR_D_NORM);
      M_E(s, "V90ConstellationDesigner: S = %d\r\n", (int)(6u - m2->w620_));
      M_E(s, "V90ConstellationDesigner: K = %d\r\n", (int)K);
      M_E(s, "V90ConstellationDesigner: final rate chosen - D = %d ( %d )\r\n", (int)s->mp->w0_, (int)rate);
    }
    (void)na; kk = K;
    mx = s->t.maxRate_; mn = s->t.minRate_;
    if (rate > mx) {
      COV(s, PR_RATE_HI);
      M_E(s, "V90ConstellationDesigner: currentRate = %d is larger then maxRate = %d => seting currentRate = %d\r\n", (int)rate, (int)mx, (int)mx);
      s->par->forceRateEnable = 1; s->par->enableDigitalPowerReduction = 0; s->par->enableRedundancyOptimization = 0; s->par->rateForce = (int)s->t.maxRate_;
    } else if (rate < mn) {
      COV(s, PR_RATE_LO);
      M_E(s, "V90ConstellationDesigner: currentRate = %d is smaller then minRate = %d => seting currentRate = %d\r\n", (int)rate, (int)mn, (int)mn);
      s->par->forceRateEnable = 1; s->par->enableDigitalPowerReduction = 0; s->par->enableRedundancyOptimization = 0; s->par->rateForce = (int)s->t.minRate_;
    } else { COV(s, PR_RATE_OK); done = 1; }
    if (!done && cnt > 2) { COV(s, PR_LOOP3_EXIT); done = 1; }
  }
  (void)kk;
  if (s->t.maxRate_ != 56000u) { s->par->enableRrnUp = 1; COV(s, PR_RRNUP_SET); }
  if (s->t.minRate_ != 28000u) { s->par->enableRrnDown = 1; COV(s, PR_RRNDN_SET); }
  if (s->t.maxRate_ <= rate) { s->par->enableRrnUp = 0; COV(s, PR_RRNUP_CLR); }
  if (rate <= s->t.minRate_) { s->par->enableRrnDown = 0; COV(s, PR_RRNDN_CLR); }
  if (s->t.w24_ == 0) { s->t.w24_ = rate; COV(s, PR_W24_SET); } else COV(s, PR_W24_KEEP);
}
static void m_design(MS *s, const PArgs *a) {
  char buf[200];
  uint32_t nb;
  memcpy(&nb, &a->noise, 4);
  if (s->par->forceRateEnable == 0) callee_text(buf, sizeof buf, "scn", nb, 1, a->A, a->B, a->dual, NULL, a->cur, a->V);
  else callee_text(buf, sizeof buf, "scnfr", nb, 1, a->A, a->B, a->dual, a->fixedU, a->cur, a->V);
  COV(s, s->par->forceRateEnable == 0 ? PR_SCN_CALL : PR_SCNFR_CALL);
  m_callee(s, buf);
  if (s->par->enableDigitalPowerReduction != 0) { COV(s, PR_POW_CALL); m_callee(s, "pow"); }
  if (s->par->enableRedundancyOptimization != 0) {
    COV(s, PR_NEWK_CALL);
    callee_text(buf, sizeof buf, "newk", 0, 0, a->A, a->B, a->dual, NULL, NULL, a->V);
    m_callee(s, buf);
  }
}

/* ------------------------------------------------------------------------------------------------ scenarios 2 / 3: process / constellationDesign */
static float quiet_noise(void) {
  uint32_t b = h_rpct(8) ? (h_rpct(50) ? 0x7fc00000u | (h_rnd() & 0x3fffffu) : 0x7f800000u | ((h_rnd() & 1u) << 31)) : h_rnd();
  if ((b & 0x7f800000u) == 0x7f800000u && (b & 0x7fffffu) != 0) b |= 0x400000u;   /* no signalling NaN (the by-value float goes through the x87 on i386) */
  return F(b);
}
static void gen_eff_counts(uint32_t *c) {
  unsigned kt = h_rpct(8) ? h_rmod(8) : 12 + h_rmod(40), base = 1u << (kt / 6 > 7 ? 7 : kt / 6), r = kt % 6, i, dbl = 0;
  for (i = 0; i < 6; i++) {
    unsigned v = base * (70 + h_rmod(61)) / 100;
    if (!v) v = 1;
    c[i] = v;
  }
  while (r--) { i = h_rmod(6); if (!(dbl & (1u << i))) { c[i] *= 2; dbl |= 1u << i; } else r++; if (!r) break; }
  for (i = 0; i < 6; i++) if (c[i] > 255) c[i] = 255;
  if (h_rpct(22)) { for (i = 0; i < 6; i++) c[i] = 1u << h_rmod(8); }   /* a power-of-two product */
  if (h_rpct(2)) c[h_rmod(6)] = 0;
}
static void gen_effs(void) {
  unsigned k, i;
  for (k = 0; k < NEFF; k++) {
    Eff *e = &g_eff[k];
    memset(e, 0, sizeof *e);
    if (h_rpct(20)) { e->w620 = 0xffffffffu; }
    else { e->w620 = h_rpct(70) ? h_rmod(9) : h_rmod(40); gen_eff_counts(e->cnt); }
    e->swap_par = h_rpct(4); e->swap_mp = h_rpct(4);
    (void)i;
  }
}
static unsigned pick_rate(void) {
  static const unsigned sp[] = { 0, 27999, 28000, 28001, 55999, 56000, 56001, 100000, 0xffffffffu };
  if (h_rpct(25)) return sp[h_rmod(sizeof sp / sizeof sp[0])];
  return 28000u + h_rmod(28001);
}
static void gen_pargs(PArgs *a, int design) {
  a->S = h_rpct(60) ? h_rmod(3) : h_rpct(70) ? h_rmod(40) : h_rnd();
  a->noise = quiet_noise();
  a->mask = h_rpct(40) ? -1 : h_rpct(15) ? 0 : (int)h_rnd();
  a->A = g_A; a->B = g_B; a->dual = g_dual; a->fixedU = g_fixbuf; a->cur = g_curbuf;
  a->plim = (unsigned char)h_rnd();
  a->codec = h_rpct(60) ? h_rmod(8) : h_rnd();
  a->arg13 = h_rnd();
  a->special = h_rpct(30) ? 2u : h_rpct(70) ? h_rmod(4) : h_rnd();
  a->V = design ? (void *)g_Vbuf : (void *)g_ad.ucodeAllowed_;
}
static void gen_params_flags(ParObj *p, int fr_pct) {
  p->forceRateEnable = h_rpct(fr_pct) ? (h_rpct(50) ? 1 : (int)h_rnd() | 1) : 0;
  p->enableDigitalPowerReduction = h_rpct(50) ? (int)h_rnd() | 1 : 0;
  p->enableRedundancyOptimization = h_rpct(50) ? (int)h_rnd() | 1 : 0;
  p->rateForce = (int)h_rnd();
  p->enableRrnUp = (int)h_rnd(); p->enableRrnDown = (int)h_rnd();
  p->spectralShaperA1 = F(h_rnd()); p->spectralShaperA2 = F(h_rnd()); p->spectralShaperB1 = F(h_rnd()); p->spectralShaperB2 = F(h_rnd());
  p->spectralShaperSr = (int)h_rnd(); p->spectralShaperId = h_rpct(70) ? (int)h_rmod(6) : (int)h_rnd();
  p->germanPbxSpectralShaperA1 = F(h_rnd()); p->germanPbxSpectralShaperA2 = F(h_rnd()); p->germanPbxSpectralShaperB1 = F(h_rnd()); p->germanPbxSpectralShaperB2 = F(h_rnd());
  p->germanPbxSpectralShaperSr = (int)h_rnd(); p->germanPbxSpectralShaperId = h_rpct(70) ? (int)h_rmod(6) : (int)h_rnd();
}
static int do_proc(int design, const PArgs *pa, int hook_kind, int emit, int chain) {
  MS ms;
  int hk = hook_kind;
  unsigned hkk = 0, i, nprint;
  int rv = 0;
  /* a print hook (swap params_ / mapParams_ inside the k-th print): k from a model run without it */
  model_init(&ms, 2, HK_NONE, 0, 0);
  memcpy(ms.eff, g_eff, sizeof ms.eff);
  if (design) m_design(&ms, pa); else m_proc(&ms, pa);
  nprint = ms.nprint;
  if (hk != HK_NONE) {
    if (!nprint) hk = HK_NONE; else hkk = 1 + h_rmod(nprint);
    model_init(&ms, 2, hk, hkk, 0);
    memcpy(ms.eff, g_eff, sizeof ms.eff);
    if (design) m_design(&ms, pa); else m_proc(&ms, pa);
  }
  g_t.params_ = &g_par; g_t.mapParams_ = &g_mpb.m; g_t.adid_ = design ? (void *)&g_ad : g_t.adid_; g_t.power_ = NULL; g_t.preFilter_ = NULL;
  if (!design) { g_t.adid_ = NULL; if (h_rpct(50)) g_t.mapParams_ = h_rpct(80) ? (void *)&g_altmpb.m : NULL; }   /* process stores mapParams_ itself: the initial value must not matter */
  g_alt_par = &g_altpar; g_alt_mp = &g_altmpb.m;
  g_snap_t = &g_t; g_snap_mp = &g_mpb.m; g_snap_par = &g_par;
  g_hook_kind = hk; g_hook_k = hkk; g_hook_level = 0;
  g_ncall_real = 0;
  dsplibs_debug_level = 2;
  log_begin();
  if (design) cd_design(&g_t, pa->noise, pa->A, pa->B, pa->dual, pa->fixedU, pa->cur, pa->V);
  else rv = cd_process(&g_t, pa->S, &g_ad, pa->noise, pa->mask, &g_mpb.m, pa->A, pa->B, pa->dual, pa->fixedU, pa->cur, pa->plim, pa->codec, pa->arg13, pa->special);
  g_hook_k = 0; g_snap_t = NULL; g_snap_mp = NULL; g_snap_par = NULL;
  if (!design) {
    if (g_t.adid_ != (void *)&g_ad) h_die3("process did not store adid_");
    if (rv != ms.retv) h_die3("process returned %d, model %d", rv, ms.retv);
    if ((g_t.mapParams_ == (void *)&g_altmpb.m) != (ms.mp == &ms.altm)) h_die3("mapParams_ after process");
    g_t.adid_ = NULL;
  }
  compare_run(design ? "design" : "process", &ms, 2, hk, hkk);
  if (g_ncall_real != ms.ncall) h_die3("callee call count: real %u model %u", g_ncall_real, ms.ncall);
  cov_hit_model(&ms, PR_LAST);
  h_cov_hit(C_TRIALS);
  if (chain) h_cov_hit(C_CHAIN);
  if (hk == HK_MP) h_cov_hit(C_ALT_HOOK_MP);
  if (hk == HK_PAR) h_cov_hit(C_ALT_HOOK_PAR);
  if (emit) {
    h_emit(0, "call %s prints %u ret %d", design ? "design" : "process", g_nlog, rv);
    dump_all("T", hk == HK_MP || ms.mp == &ms.altm);
  }
  (void)i;
  return 1;
}
static void sc_proc(int design, int ntr) {
  int tr;
  cov_define_trials();
  cov_define_names(prcov_names, 0, PR_LAST);
  h_scenario_begin(0);
  relax_reserved();
  h_covreq[C_L0] = h_covreq[C_L1] = h_covreq[C_L2] = h_covreq[C_LHI] = h_covreq[C_HOOK_LEVEL] = h_covreq[C_SKIPPED] = 0;
  if (design) {
    int i;
    for (i = 0; i < PR_LAST; i++) if (prcov_names[i]) h_covreq[cov_id(i)] = 0;
    h_covreq[cov_id(PR_SCN_CALL)] = h_covreq[cov_id(PR_SCNFR_CALL)] = h_covreq[cov_id(PR_POW_CALL)] = h_covreq[cov_id(PR_NEWK_CALL)] = 1;
    h_covreq[C_ALT_HOOK_MP] = h_covreq[C_ALT_HOOK_PAR] = 0;
  } else {
    h_covreq[cov_id(PR_LOOP3_EXIT)] = 1;
  }
  mp_canary(&g_mpb); mp_canary(&g_altmpb);
  for (tr = 0; tr < ntr; tr++) {
    PArgs pa;
    t_prefill(&g_t);
    gen_block(&g_mpb.m, 0);
    gen_block(&g_altmpb.m, h_rpct(50) ? 2 : 0);
    gen_adid(h_rpct(50), 0, 10);
    gen_par(&g_par, 0); gen_par(&g_altpar, 0);
    gen_params_flags(&g_par, design ? 30 : 15);
    gen_params_flags(&g_altpar, 30);
    g_t.minRate_ = pick_rate(); g_t.maxRate_ = pick_rate();
    if (h_rpct(10)) g_t.maxRate_ = g_t.minRate_;
    g_t.w24_ = h_rpct(50) ? 0u : h_rnd();
    gen_effs();
    gen_pargs(&pa, design);
    do_proc(design, &pa, h_rpct(18) ? (int)(2 + h_rmod(2)) : HK_NONE, 1, 0);
    if (h_rpct(30)) {   /* a second call on the same object: the first call left forceRate / rate limits behind */
      gen_effs();
      gen_pargs(&pa, design);
      do_proc(design, &pa, HK_NONE, 1, 1);
    }
  }
}

/* ------------------------------------------------------------------------------------------------ scenario 4: ctor / dtor / reset / setMinMaxRates / spectralDesign */
typedef struct { CdObj t; uint8_t canary[48]; } TBox;
static TBox g_tb;
static void tbox_prefill(void) { t_prefill(&g_tb.t); memset(g_tb.canary, 0xe9, sizeof g_tb.canary); }
static void tbox_check(const char *w) {
  unsigned i;
  for (i = 0; i < sizeof g_tb.canary; i++) if (g_tb.canary[i] != 0xe9) h_die3("%s: object canary byte %u overwritten", w, i);
}
static void ptr_expect(const char *what, const void *got, const void *want) { if (got != want) h_die3("%s: pointer %p expected %p", what, got, want); }
static void dump_t(const char *tag, const CdObj *t) {
  CdObj tmp;
  int show = h_sample(&g_dumpctr, 40, 97);
  memcpy(&tmp, t, sizeof tmp);
  tmp.params_ = NULL; tmp.mapParams_ = NULL; tmp.adid_ = NULL; tmp.power_ = NULL; tmp.preFilter_ = NULL;
  h_dump_obj(show, tag, &tmp, f_CdObj, NF_CdObj);
}
static void sc_small_obj(int ntr) {
  int tr;
  cov_define_trials();
  h_covreq[C_L0] = h_covreq[C_L1] = h_covreq[C_L2] = h_covreq[C_LHI] = h_covreq[C_SKIPPED] = h_covreq[C_CHAIN] = h_covreq[C_ALT_HOOK_MP] = h_covreq[C_ALT_HOOK_PAR] = h_covreq[C_HOOK_LEVEL] = 0;
  h_scenario_begin(0);
  relax_reserved();
  for (tr = 0; tr < ntr; tr++) {
    int what = (int)h_rmod(6);
    CdObj before, exp;
    mp_canary(&g_mpb); mp_canary(&g_altmpb);
    tbox_prefill();
    memcpy(&before, &g_tb.t, sizeof before);
    memcpy(&exp, &g_tb.t, sizeof exp);
    if (what == 0 || what == 1) {   /* C1 / C2 then D1 / D2 */
      void *p = &g_par, *pf = (void *)&g_ad, *pw = (void *)&g_altpar;
      if (what == 0) cd_c1(&g_tb.t, p, pf, pw); else cd_c2(&g_tb.t, p, pf, pw);
      ptr_expect("params_", g_tb.t.params_, p); ptr_expect("preFilter_", g_tb.t.preFilter_, pf); ptr_expect("power_", g_tb.t.power_, pw);
      exp.flag8_ = 0; exp.state_ = 0; exp.w38_ = 0x16; exp.minRate_ = 28000u; exp.maxRate_ = 56000u;
      exp.params_ = g_tb.t.params_; exp.power_ = g_tb.t.power_; exp.preFilter_ = g_tb.t.preFilter_;
      if (memcmp(&exp, &g_tb.t, sizeof exp)) h_die3("constructor result differs from the model");
      dump_t("ctor", &g_tb.t);
      if (h_rpct(50)) cd_d1(&g_tb.t); else cd_d2(&g_tb.t);
      if (memcmp(&exp, &g_tb.t, sizeof exp)) h_die3("a destructor modified the object");
      dump_t("dtor", &g_tb.t);
    } else if (what == 2) {   /* reset */
      gen_par(&g_par, 0);
      g_tb.t.params_ = &g_par;
      memcpy(&exp, &g_tb.t, sizeof exp);
      cd_reset(&g_tb.t);
      exp.state_ = 0; exp.dMin_ = 0; exp.w24_ = (uint32_t)g_par.unk39c; exp.rrnDownDmin_ = 0; exp.rrnUpDmin_ = 0; exp.dMinAlt_ = 0;
      if (memcmp(&exp, &g_tb.t, sizeof exp)) h_die3("reset result differs from the model");
      dump_t("reset", &g_tb.t);
    } else if (what == 3 || what == 4) {   /* setMinMaxRates (levels, a hook between the two prints) */
      unsigned mn = pick_rate(), mx = pick_rate(), lv = pick_level();
      int hook = h_rpct(30);
      g_snap_t = &g_tb.t; g_snap_mp = NULL; g_snap_par = NULL;
      g_hook_kind = hook ? HK_LEVEL : HK_NONE; g_hook_k = hook ? 1 : 0; g_hook_level = h_rpct(60) ? 1u : 0u;
      dsplibs_debug_level = lv;
      log_begin();
      cd_setmm(&g_tb.t, mn, mx);
      g_hook_k = 0; g_snap_t = NULL;
      exp.minRate_ = mn; exp.maxRate_ = mx;
      if (memcmp(&exp, &g_tb.t, sizeof exp)) h_die3("setMinMaxRates result differs from the model");
      {
        unsigned want = lv > 1 ? (hook && g_hook_level <= 1 ? 1u : 2u) : 0u;
        if (g_nlog != want) h_die3("setMinMaxRates: %u prints, model %u (level %u hook %d)", g_nlog, want, lv, hook);
        if (want >= 1 && strcmp(g_log[0].text, "V90ConstellationDesigner: set min rate to ") != 0) {
          char b[100]; snprintf(b, sizeof b, "V90ConstellationDesigner: set min rate to %d\\r\\n", (int)mn);
          if (strcmp(g_log[0].text, b)) h_die3("first print: '%s'", g_log[0].text);
        }
        if (want == 2) {
          char b[100]; snprintf(b, sizeof b, "V90ConstellationDesigner: set max rate to %d\\r\\n", (int)mx);
          if (strcmp(g_log[1].text, b)) h_die3("second print: '%s'", g_log[1].text);
        }
      }
      dump_t("setmm", &g_tb.t);
      h_emit(0, "setmm prints %u", g_nlog);
    } else {   /* spectralDesign */
      MapPar90 want;
      unsigned S = h_rpct(60) ? h_rmod(4) : h_rnd(), special = h_rpct(40) ? 2u : h_rpct(70) ? h_rmod(4) : h_rnd();
      ParObj *P = &g_par;
      uint32_t w, id;
      memset(P, 0, sizeof *P);
      gen_params_flags(P, 0);
      gen_block(&g_mpb.m, 0);
      memcpy(&want, &g_mpb.m, sizeof want);
      g_tb.t.params_ = P; g_tb.t.mapParams_ = &g_mpb.m;
      cd_spectral(&g_tb.t, S, special);
      if (special == 2) {
        id = (uint32_t)P->germanPbxSpectralShaperId; want.w624_ = id < S ? id : S; want.w620_ = (uint32_t)P->germanPbxSpectralShaperSr;
        memcpy(&w, &P->germanPbxSpectralShaperA1, 4); memcpy(&want.f628_, &w, 4); memcpy(&w, &P->germanPbxSpectralShaperA2, 4); memcpy(&want.f62c_, &w, 4);
        memcpy(&w, &P->germanPbxSpectralShaperB1, 4); memcpy(&want.f630_, &w, 4); memcpy(&w, &P->germanPbxSpectralShaperB2, 4); memcpy(&want.f634_, &w, 4);
      } else {
        id = (uint32_t)P->spectralShaperId; want.w624_ = id < S ? id : S; want.w620_ = (uint32_t)P->spectralShaperSr;
        memcpy(&w, &P->spectralShaperA1, 4); memcpy(&want.f628_, &w, 4); memcpy(&w, &P->spectralShaperA2, 4); memcpy(&want.f62c_, &w, 4);
        memcpy(&w, &P->spectralShaperB1, 4); memcpy(&want.f630_, &w, 4); memcpy(&w, &P->spectralShaperB2, 4); memcpy(&want.f634_, &w, 4);
      }
      if (memcmp(&want, &g_mpb.m, sizeof want)) h_die3("spectralDesign: the block differs from the model (special %u)", special);
      mp_check(&g_mpb, "spectralDesign");
      h_dump_obj(h_sample(&g_dumpctr, 40, 97), "spec", &g_mpb.m, f_mp, NF_mp);
    }
    tbox_check("small object");
    h_cov_hit(C_TRIALS);
  }
}

/* ------------------------------------------------------------------------------------------------ scenario 5: the small helpers */
static void gen_tie_block(MapPar90 *mp) {   /* first ucodes / counts from tiny sets: every tie rule of the pick helpers is reached */
  unsigned ph, i;
  fill_words(mp, sizeof(MapPar90));
  for (ph = 0; ph < 6; ph++) {
    unsigned top = h_rpct(30) ? h_rmod(3) : h_rpct(70) ? 100 + h_rmod(3) : h_rmod(256);
    mp->cnt_[ph] = h_rpct(50) ? 1 + h_rmod(3) : h_rpct(80) ? 1 + h_rmod(127) : h_rmod(300);
    for (i = 0; i < 128; i++) *m_la(mp, ph, i) = (unsigned char)(i == 0 ? top : top - (i < 100 ? i : 0));
    for (i = 0; i < 128; i++) *m_lb(mp, ph, i) = (unsigned char)h_rnd();
  }
}
/* the models of the two pick helpers: the blob keeps (value, count) in two registers and, an equal value only takes over with a larger count */
static int m_pick2(const MapPar90 *mp, int highest) {
  unsigned best = 0, v = mp->listA_[0], c = mp->cnt_[0], ph;
  for (ph = 1; ph < 6; ph++) {
    unsigned nv = mp->listA_[ph * 128], nc = mp->cnt_[ph];
    if (highest ? nv > v : nv < v) { v = nv; c = nc; best = ph; }
    else if (nv == v && nc > c) { c = nc; best = ph; }
  }
  return (int)best;
}
static int g_nextu_flags;
static void sc_helpers(int ntr) {
  int tr;
  cov_define_trials();
  h_covreq[C_L0] = h_covreq[C_L1] = h_covreq[C_L2] = h_covreq[C_LHI] = h_covreq[C_SKIPPED] = h_covreq[C_CHAIN] = h_covreq[C_ALT_HOOK_MP] = h_covreq[C_ALT_HOOK_PAR] = h_covreq[C_HOOK_LEVEL] = 0;
  h_scenario_begin(0);
  relax_reserved();
  mp_canary(&g_mpb);
  for (tr = 0; tr < ntr; tr++) {
    int what = (int)h_rmod(7);
    MapPar90 *mp = &g_mpb.m, want;
    tbox_prefill();
    g_tb.t.params_ = &g_par; g_tb.t.mapParams_ = mp; g_tb.t.adid_ = &g_ad;
    if (what == 0 || what == 1) {   /* findConstelMaxValueIndex / findMinValueIndex */
      int r, m;
      gen_tie_block(mp);
      r = what == 0 ? cd_findmax(&g_tb.t, mp) : cd_findmin(&g_tb.t, mp);
      m = m_pick2(mp, what == 0);
      if (r != m) h_die3("find%s index: real %d model %d", what ? "Min" : "Max", r, m);
      h_emit(0, "find%d -> %d", what, r);
    } else if (what == 2) {   /* reconstructInitialConditions */
      unsigned char uc[6];
      unsigned ph, i, d;
      gen_block(mp, h_rpct(30) ? 1 : 0);
      for (ph = 0; ph < 6; ph++) {
        unsigned cnt, ti;
        if (h_rpct(25)) mp->cnt_[ph] = h_rmod(4);   /* tiny / zero counts: the target may lie BEYOND the count */
        cnt = mp->cnt_[ph] > 128 ? 128 : mp->cnt_[ph];
        for (i = 0; i < 128; i++) *m_la(mp, ph, i) = (unsigned char)(250 - i * 2);
        ti = h_rpct(30) ? 0 : h_rpct(70) ? h_rmod(cnt ? cnt : 1) : h_rmod(cnt + 2);   /* d <= cnt + 1: a second pass over an emptied list would never end (the blob too) */
        uc[ph] = *m_la(mp, ph, ti);
      }
      memcpy(&want, mp, sizeof want);
      for (ph = 0; ph < 6; ph++) {
        for (d = 0; *m_la(&want, ph, d) != uc[ph]; d++) ;
        while (d--) {
          unsigned c = want.cnt_[ph];
          for (i = 0; i < c; i++) { *m_la(&want, ph, i) = *m_la(&want, ph, i + 1); *m_lb(&want, ph, i) = *m_lb(&want, ph, i + 1); }
          want.cnt_[ph]--;
        }
      }
      cd_recon(&g_tb.t, mp, uc);
      if (memcmp(&want, mp, sizeof want)) h_die3("reconstructInitialConditions: the block differs from the model");
      h_dump_obj(h_sample(&g_dumpctr, 40, 97), "recon", mp, f_mp, NF_mp);
    } else if (what == 3) {   /* constelBuild(dmin, phase) */
      int alaw = h_rpct(50), ph = (int)h_rmod(6);
      short dm = (short)(h_rpct(70) ? (int)h_rrange(0, 400) : h_rpct(60) ? (int)h_rrange(-3000, 3000) : (int)h_rnd());
      unsigned lo;
      unsigned char r, m = 0;
      int half;
      unsigned u, first;
      gen_adid(alaw, h_rpct(40) ? 0 : (int)h_rmod(60), 0);
      gen_par(&g_par, h_rpct(70) ? (int)h_rmod(4) : h_rpct(60) ? (int)h_rmod(130) : (int)h_rnd());
      gen_block(mp, h_rpct(20) ? 4 : 0);
      lo = (unsigned)g_par.unk360;
      r = cd_build(&g_tb.t, dm, (short)ph);
      first = *m_la(mp, (unsigned)ph, 0);
      half = (int)dm >> 1;
      if (lo <= first)
        for (u = lo; u <= first; u++) {
          short e = ((const short *)g_ad.linearMapping_)[ph * 128 + u];
          if (e > (short)half && ((const unsigned char *)g_ad.ucodeAllowed_)[ph * 128 + u] != 0) { m++; half = (short)(dm + e); }
        }
      if (r != m) h_die3("constelBuild: real %u model %u", r, m);
      h_emit(0, "build %d -> %u", ph, r);
    } else if (what == 4) {   /* findNextUcodeToAdd */
      int ph = (int)h_rmod(6), alaw = h_rpct(50), isdual = h_rpct(50);
      unsigned char out[2] = { 0x11, 0x22 }, mo[2];
      unsigned u0, u, base;
      int ret, mret, thr;
      short *dual = g_dual;
      gen_block(mp, 0);
      *m_la(mp, (unsigned)ph, 0) = (unsigned char)(h_rpct(8) ? 0xfc + h_rmod(4) : h_rpct(12) ? 0x80 + h_rmod(0x80) : h_rpct(40) ? h_rmod(0x75) : h_rpct(50) ? 0x70 + h_rmod(0x14) : 0x7c + h_rmod(0x8));
      fill_table(g_A, alaw, h_rpct(50) ? 0 : (int)h_rmod(60), 1);
      fill_table(g_B, alaw, h_rpct(50) ? 0 : (int)h_rmod(60), 1);
      memset(g_dualbuf, 0, sizeof g_dualbuf);
      dual[ph] = (short)(isdual ? (h_rpct(50) ? 1 : (int)h_rnd() | 1) : 0);
      g_tb.t.useAlaw_ = (uint32_t)alaw; g_tb.t.dMin_ = (uint16_t)(h_rpct(70) ? (int)h_rrange(0, 300) : (int)h_rnd()); g_tb.t.dMinAlt_ = (uint16_t)(h_rpct(70) ? (int)h_rrange(0, 300) : (int)h_rnd());
      if (h_rpct(35)) {   /* equality steering: the threshold equals a table entry further up (A[u1] == A[u0] + d, A[u2] == B[u0] + d) */
        unsigned b0 = (unsigned)ph * 128u, uu0 = *m_la(mp, (unsigned)ph, 0), u1 = uu0 + 1 + h_rmod(6), u2 = uu0 + h_rmod(6);
        int d = (int)(short)g_A[b0 + (u1 & 0xff)] - (int)(short)g_A[b0 + uu0];
        if (isdual) { g_tb.t.dMinAlt_ = (uint16_t)d; g_B[b0 + uu0] = (short)((int)(short)g_A[b0 + (u2 & 0xff)] - d); }
        else g_tb.t.dMin_ = (uint16_t)d;
      }
      ret = cd_nextu(&g_tb.t, out, (unsigned char)ph, g_A, g_B, dual, g_Vbuf);
      u0 = *m_la(mp, (unsigned)ph, 0); u = u0; base = (unsigned)ph * 128u;
      if (dual[ph]) {
        if (u0 <= 0x71) {
          thr = g_A[base + u0] + (short)g_tb.t.dMinAlt_;
          for (;;) {
            int e = g_A[base + u];
            if (!(e < thr) && e >= g_B[base + u0] + (short)g_tb.t.dMinAlt_) break;
            if (++u > 0x71) break;
          }
        }
      } else if (!((signed char)u0 < 0)) {
        thr = g_A[base + u0] + (short)g_tb.t.dMin_;
        for (;;) {
          if (g_A[base + u] >= thr) break;
          u++;
          if ((signed char)u < 0) break;
        }
      }
      mo[0] = (unsigned char)u;
      { int v = g_A[base + (unsigned char)u]; v = v < 0 ? -v : v; mo[1] = alaw ? (unsigned char)(linear2alaw(v) ^ 0xd5) : (unsigned char)~linear2ulaw(v); }
      mret = (signed char)(unsigned char)u >= 0;
      if (ret != mret || out[0] != mo[0] || out[1] != mo[1]) h_die3("findNextUcodeToAdd: real ret %d out %02x %02x, model ret %d out %02x %02x (phase %d dual %d u0 %u)", ret, out[0], out[1], mret, mo[0], mo[1], ph, isdual, u0);
      h_emit(0, "nextu %d -> %d %02x %02x", ph, ret, out[0], out[1]);
    } else if (what == 5) {   /* maxK */
      unsigned r;
      gen_block(mp, h_rpct(50) ? 0 : 3);
      if (h_rpct(40)) { unsigned q; for (q = 0; q < 6; q++) mp->cnt_[q] = 1u << h_rmod(h_rpct(80) ? 6 : 12); }   /* a power-of-two product: log10(prod) / (float)log10(2) sits at an integer (the 1e-6f matters) */
      if (h_rpct(8)) mp->cnt_[h_rmod(6)] = 0;
      if (h_rpct(8)) mp->cnt_[h_rmod(6)] = h_rnd();
      r = cd_maxk(&g_tb.t, mp);
      if (r != m_kint(mp)) h_die3("maxK: real %u model %u", r, m_kint(mp));
      h_emit(0, "maxK -> %u", r);
    } else {   /* calcMtoMatchKtarget(a, b) */
      float a = (float)((int)h_rrange(-80, 600)) / 8.0f, b = (float)((int)h_rrange(1, 1000000)) / (h_rpct(50) ? 1.0f : 1024.0f);
      LD t;
      float lgf, den;
      unsigned r, m;
      int hang = 0;
      if (h_rpct(6)) { static const uint32_t sp[] = { 0x7fc00000u, 0x7f800000u, 0xff800000u, 0, 0x80000000u, 0x3f800000u, 0x00000001u, 0x7f7fffffu }; b = F(sp[h_rmod(8)]); }
      if (h_rpct(4)) { static const uint32_t sp[] = { 0x7fc00000u, 0x7f800000u, 0xff800000u, 0, 0x80000000u }; a = F(sp[h_rmod(5)]); }
      lgf = RF(m_log10((LD)b)); den = m_lg2f();
      t = ((LD)a - (LD)lgf / (LD)den) * (LD)F(0x3e2aaaabu);
      m = m_mfind(t, &hang);
      if (hang) { g_nskipped++; continue; }
      r = cd_calcm(&g_tb.t, a, b);
      if (r != m) h_die3("calcMtoMatchKtarget(%08x, %08x): real %u model %u", h_bits_of(a), h_bits_of(b), r, m);
      h_emit(0, "calcM %08x %08x -> %u", h_bits_of(a), h_bits_of(b), r);
    }
    (void)g_nextu_flags;
    mp_check(&g_mpb, "helpers");
    tbox_check("helpers");
    h_cov_hit(C_TRIALS);
  }
}

/* ------------------------------------------------------------------------------------------------ scenarios 6 / 7: pow6 / calcK / realK */
static void hexld(char *out, LD v) {
  unsigned char b[16];
  int i;
  memset(b, 0, sizeof b);
  memcpy(b, &v, 10);
  for (i = 9; i >= 0; i--) sprintf(out + 2 * (9 - i), "%02x", b[i]);
}
static void sc_floats(int ext, int ntr) {
  int tr;
  cov_define_trials();
  h_covreq[C_L0] = h_covreq[C_L1] = h_covreq[C_L2] = h_covreq[C_LHI] = h_covreq[C_SKIPPED] = h_covreq[C_CHAIN] = h_covreq[C_ALT_HOOK_MP] = h_covreq[C_ALT_HOOK_PAR] = h_covreq[C_HOOK_LEVEL] = 0;
  h_scenario_begin(0);
  relax_reserved();
  mp_canary(&g_mpb);
  for (tr = 0; tr < ntr; tr++) {
    int what = (int)h_rmod(3);
    MapPar90 *mp = &g_mpb.m;
    char hx[40];
    tbox_prefill();
    if (what == 0) {
      short x = (short)(h_rpct(60) ? (int)h_rrange(-40, 40) : h_rpct(50) ? (int)h_rrange(-32768, 32767) : (int)h_rrange(-3, 3));
      LD m = (LD)x, acc = m;
      int i;
      for (i = 2; i <= 6; i++) acc = acc * m;
      if (ext) {
        XRET r = cd_pow6_x(&g_tb.t, x);
        hexld(hx, (LD)r);
#if defined(__i386__)
        { char mh[40]; hexld(mh, acc); if (strcmp(hx, mh)) h_die3("pow6(%d) extended: real %s model %s", x, hx, mh); }
#endif
        h_emit(0, "pow6x %d %s", x, hx);
      } else {
#if defined(__i386__)
        { XRET rx = cd_pow6_x(&g_tb.t, x); char h1[40], h2[40]; hexld(h1, (LD)rx); hexld(h2, acc); if (strcmp(h1, h2)) h_die3("pow6(%d) extended: real %s model %s", x, h1, h2); }
#endif
        float r = cd_pow6_f(&g_tb.t, x);
        if (h_bits_of(r) != h_bits_of(RF(acc))) h_die3("pow6(%d): real %08x model %08x", x, h_bits_of(r), h_bits_of(RF(acc)));
        h_emit(0, "pow6 %d %08x", x, h_bits_of(r));
      }
    } else if (what == 1) {
      float arr[6];
      unsigned m = h_rpct(60) ? 1 + h_rmod(130) : h_rpct(50) ? h_rmod(5) : h_rnd();
      LD acc = 1.0L, ml = (LD)m, kq;
      int i;
      for (i = 0; i < 6; i++) arr[i] = h_rpct(8) ? F(h_rpct(50) ? 0u : 0x7f800000u) : F((h_rnd() & 0x007fffffu) | ((uint32_t)(0x76 + h_rmod(0xe)) << 23));
      for (i = 0; i < 6; i++) acc = acc * (ml * (LD)arr[i]);
      kq = (LD)RF(m_log10(acc)) * ((LD)1.0L / (LD)m_lg2f());
      if (ext) {
        XRET r = cd_calck_x(&g_tb.t, m, arr);
        hexld(hx, (LD)r);
#if defined(__i386__)
        { char mh[40]; hexld(mh, kq); if (strcmp(hx, mh)) h_die3("calcK extended: real %s model %s", hx, mh); }
#endif
        h_emit(0, "calcKx %u %s", m, hx);
      } else {
#if defined(__i386__)
        { XRET rx = cd_calck_x(&g_tb.t, m, arr); char h1[40], h2[40]; hexld(h1, (LD)rx); hexld(h2, kq); if (strcmp(h1, h2)) h_die3("calcK extended: real %s model %s", h1, h2); }
#endif
        float r = cd_calck_f(&g_tb.t, m, arr);
        if (h_bits_of(r) != h_bits_of(RF(kq))) h_die3("calcK(%u): real %08x model %08x", m, h_bits_of(r), h_bits_of(RF(kq)));
        h_emit(0, "calcK %u %08x", m, h_bits_of(r));
      }
    } else {
      LD p, rk;
      gen_block(mp, h_rpct(50) ? 0 : 3);
      if (h_rpct(40)) { unsigned q; for (q = 0; q < 6; q++) mp->cnt_[q] = 1u << h_rmod(h_rpct(80) ? 6 : 12); }
      if (h_rpct(8)) mp->cnt_[h_rmod(6)] = 0;
      p = m_prod(mp);
      if (p == 0.0L) rk = 0.0L;
      else rk = (LD)RF(m_log10(p)) / (LD)m_lg2f() + (LD)F(0x3089705fu);
      if (ext) {
        XRET r = cd_realk_x(&g_tb.t, mp);
        hexld(hx, (LD)r);
#if defined(__i386__)
        { char mh[40]; hexld(mh, rk); if (strcmp(hx, mh)) h_die3("realK extended: real %s model %s", hx, mh); }
#endif
        h_emit(0, "realKx %s", hx);
      } else {
#if defined(__i386__)
        { XRET rx = cd_realk_x(&g_tb.t, mp); char h1[40], h2[40]; hexld(h1, (LD)rx); hexld(h2, rk); if (strcmp(h1, h2)) h_die3("realK extended: real %s model %s", h1, h2); }
#endif
        float r = cd_realk_f(&g_tb.t, mp);
        if (h_bits_of(r) != h_bits_of(RF(rk))) h_die3("realK: real %08x model %08x", h_bits_of(r), h_bits_of(RF(rk)));
        h_emit(0, "realK %08x", h_bits_of(r));
      }
    }
    mp_check(&g_mpb, "floats");
    h_cov_hit(C_TRIALS);
  }
}
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P25_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P25_DBG")) g_show = 1;
  p07_layout_check();
  p11_layout_check();
  p18_layout_check();
  p24_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(240);
  switch (sc) {
    case 0: sc_ddfr(0, 4000); break;
    case 1: sc_ddfr(1, 3000); break;
    case 2: sc_proc(0, 3000); break;
    case 3: sc_proc(1, 1500); break;
    case 4: sc_small_obj(3000); break;
    case 5: sc_helpers(6000); break;
    case 6: sc_floats(0, 3000); break;
    case 7: sc_floats(1, 3000); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu edprintf %u dbgprintf %u callee %u skipped %u", h_nmalloc, h_nfree, g_nlog_total_e, g_nlog_total_d, g_nlog_total_c, g_nskipped);
  h_cov_finish();
  return 0;
}
