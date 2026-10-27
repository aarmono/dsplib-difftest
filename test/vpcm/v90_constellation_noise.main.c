/*
 * Differential test harness for port chunk P24 (src/vpcm/v90_constellation_noise.cpp): V90ConstellationDesigner::setConstellationToNoise and
 * V90ConstellationDesigner::setConstellationToNoise_forceRate (2 functions).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_constellation_noise.sh against the blob (dsplibs.o with `edprintf` WEAK so that the recorders below win), against the port
 * (-O0 / -O2; linear2alaw / linear2ulaw come from the P06 port object there) and for x86-64 (diffed against the 32-bit BLOB's output = the golden test/golden/vpcm/v90_constellation_noise.blob32.out; -mfpmath=387 in
 * test/vpcm/v90_constellation_noise.golden64.sh).  The functions are reached through __asm__ labels; the designer object (0x54 B, four pointers), the V90Parameters block (P07 mirror: only unk360 /
 * useRestricedDmin / forcedDmin / rateForce are read) and the V90MappingParams block (P11 mirror, 0x650 B, the output) are the generated C mirrors; the pointer-holding designer object and
 * the parameter block are STATIC / member-wise filled (their sizes differ on x86-64), everything else comes from the arena pool.
 *
 * What is compared: the whole designer object, the whole mapping block, the caller's `cur[]` array (forceRate writes it) and the inputs (read-only) after every call, every edprintf /
 * dsplibs_debug_printf call (kind, text with the integer arguments formatted, a SNAPSHOT of the designer object and the mapping block at the moment of the call: the store order between
 * prints) and the unsigned `level > 1` test at every dsplibs_debug_printf site (levels 0, 1, 2, 3, >= 2^31; a hook lowers the level inside the k-th print; two more hooks swap T->params_
 * / T->mapParams_ inside the k-th print: every re-read of the blob is a re-read here).
 *
 * INDEPENDENT MODELS (not copies of the port; written from the disassembly notes in a specification style, long double arithmetic with long long conversions): model_scn is the whole
 * setConstellationToNoise (dMin / thresholds / state machine / ucode scan / G.711 codes / the report prints with their level re-reads), model_fr the whole forceRate (2^n, the product search,
 * the per-phase counts, the reciprocal / product phase distances, the iteration with its step / counter, the shrink and grow loops, every print).  The G.711 encoders are the linked
 * linear2alaw / linear2ulaw (blob in the blob run, P06 port in the port runs: P06 is verified separately).  The model also decides whether the BLOB would terminate: inputs on which the
 * forceRate growth loop finds no candidate (the blob loops forever) or the 2^n loop would run 2^32 times are skipped and counted.
 */

#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "pcm_parameters_layout.h"
#include "v92_cp_layout.h"
#include "v90_constellation_noise_layout.h"

#define L(m) __asm__(m)

extern void cd_scn(void *, float, short (*)[128], short (*)[128], short *, unsigned char *, unsigned char (*)[128]) L("_ZN24V90ConstellationDesigner23setConstellationToNoiseEfPA128_sS1_PsPhPA128_h");
extern void cd_fr(void *, float, short (*)[128], short (*)[128], short *, unsigned char *, unsigned char *, unsigned char (*)[128])
    L("_ZN24V90ConstellationDesigner33setConstellationToNoise_forceRateEfPA128_sS1_PsPhS3_PA128_h");
extern int linear2alaw(int);
extern int linear2ulaw(int);
extern int alaw2linear(int);
extern int ulaw2linear(int);
extern unsigned int dsplibs_debug_level;

typedef long double LD;

/* the mapping block's dump table: the two 768-byte lists are printed as hashes ('x' above 64 bytes) */
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
static unsigned g_nlog, g_nlog_total_e, g_nlog_total_d;
static int g_show;
static CdObj *g_snap_t;       /* object / mapping block snapshotted at every print */
static MapPar90 *g_snap_mp;
enum { HK_NONE = 0, HK_LEVEL, HK_PAR, HK_MP };
static int g_hook_kind;
static unsigned g_hook_k, g_hook_level;   /* HK_LEVEL: level after the k-th dsplibs_debug_printf; HK_PAR / HK_MP: swap after the k-th print of any kind */
static unsigned g_nprints_in_call, g_nd_in_call;
static V90ParObj *g_alt_par;
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
  return h_rpct(55) ? 2u : lv[h_rmod(sizeof lv / sizeof lv[0])];
}
static float f_of(uint32_t b) { return h_float_from_bits_quiet(b); }
static uint32_t rf_bits(void) {   /* quiet float patterns only */
  uint32_t r = h_rmod(100), sign = (h_rnd() & 1u) << 31;
  float f;
  uint32_t b;
  if (r < 45) { f = (float)((int)h_rrange(-4000, 4000)) / 64.0f; memcpy(&b, &f, 4); return b; }
  if (r < 60) { f = (float)((int)h_rrange(-100, 100)); memcpy(&b, &f, 4); return b; }
  if (r < 72) { f = (float)((int)h_rrange(-200000, 200000)) / 4096.0f; memcpy(&b, &f, 4); return b; }
  if (r < 80) { f = (float)((int)h_rrange(-9000, 9000)) * 16.0f; memcpy(&b, &f, 4); return b; }
  if (r < 85) return sign | 0x7f800000u;
  if (r < 90) return sign | 0x7fc00000u | (h_rnd() & 0x3fffffu);
  if (r < 94) return sign;
  return sign | ((uint32_t)(100 + h_rmod(60)) << 23) | (h_rnd() & 0x7fffffu);
}

/* ------------------------------------------------------------------------------------------------ models: arithmetic */
static int m_i32(LD v) {   /* fistp DWORD with truncation: the integer indefinite for NaN / out of range */
  if (!(v > -2147483649.0L && v < 2147483648.0L)) return (int)0x80000000u;
  return (int)(long long)v;
}
static int m_i16(LD v) {   /* fistp WORD: 0x8000 indefinite, result sign-extended */
  if (!(v > -32769.0L && v < 32768.0L)) return -32768;
  return (int)(long long)v;
}
static void m_split(LD v, LD scale, int *sign, int *ip, int *fp) {
  int t = m_i32(v), f;
  long long fl;
  LD av = v < 0 ? -v : v;
  LD sc = (v - (LD)t) * scale;
  if (v != v) av = v;
  *sign = (v > 0.0L || v != v) ? '+' : '-';
  *ip = m_i32(av);
  f = m_i32(sc);
  fl = f;
  if (fl < 0) fl = -fl;
  *fp = (int)fl;
  if (f == (int)0x80000000u) *fp = (int)0x80000000u;
}
/* a float STORE: the blob's fstp to a float slot rounds; a plain (float) cast may stay in an x87 register in this compiler's default excess-precision mode */
static float RF(LD v) { volatile float f = (float)v; return f; }
/* the float constants of the blob */
static float F(uint32_t bits) { float f; memcpy(&f, &bits, 4); return f; }
#define C428 F(0x40d8d6b6u)
#define C42C F(0x411e6666u)
#define C434 F(0x405550b1u)
#define C43C F(0x41036adbu)
#define C44C F(0x3ee66666u)
#define C450 F(0x3fb4cccdu)
#define C470 F(0x3dcccccdu)
#define C478 F(0x3d4ccccdu)
#define C47C F(0x3f666666u)
static double rate_scale(void) { uint64_t w = 0x3f489375237ec81bull; double d; memcpy(&d, &w, 8); return d; }

/* ------------------------------------------------------------------------------------------------ model state */
typedef struct {
  /* inputs (shared, read-only) */
  float a;
  const short *A, *B;            /* flat [6][128] (+ slack) */
  const short *dual;
  const unsigned char *lim;      /* maxU (scn) / fixed (fr) */
  const unsigned char *valid;    /* flat [6][128] */
  unsigned char cur[8];          /* fr: the caller's array, written by the call */
  /* the object, as the model sees it */
  CdObj t;                       /* non-pointer members */
  const V90ParObj *par;          /* T->params_ (switched by a PAR hook) */
  MapPar90 *mp;                  /* T->mapParams_ (a model copy; switched by a MP hook) */
  MapPar90 mpm, altm;            /* model copies of the real blocks */
  const V90ParObj *altpar;
  /* printing */
  unsigned level;
  int hook_kind;
  unsigned hook_k, hook_level, nprint, nd;
  LogEnt log[LOGMAX];
  unsigned nlog;
  int hang;                      /* the blob would not terminate on this input */
  /* coverage notes (hit by the model while it runs; counted only if the model result is used) */
  unsigned char cov[128];
} MS;

static void m_print(MS *s, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (s->nlog < LOGMAX) { s->log[s->nlog].kind = kind; s->log[s->nlog].text[0] = 0; esc_cat(s->log[s->nlog].text, sizeof s->log[s->nlog].text, buf); }
  s->nlog++;
  s->nprint++;
  if (kind == 'd') s->nd++;
  if (s->hook_k) {
    if (s->hook_kind == HK_LEVEL && kind == 'd' && s->nd == s->hook_k) s->level = s->hook_level;
    if (s->hook_kind == HK_PAR && s->nprint == s->hook_k) s->par = s->altpar;
    if (s->hook_kind == HK_MP && s->nprint == s->hook_k) s->mp = &s->altm;
  }
}
#define M_E(s, ...) m_print(s, 'e', __VA_ARGS__)
#define M_D(s, ...) m_print(s, 'd', __VA_ARGS__)

static unsigned char m_code(const MS *s, int v) {   /* the G.711 code of a distance */
  int a = v < 0 ? -v : v;
  if (v == INT_MIN) a = v;
  if (s->t.useAlaw_ == 0) return (unsigned char)(~(unsigned char)linear2ulaw(a));
  return (unsigned char)((unsigned char)linear2alaw(a) ^ 0xd5);
}

/* the report both functions end with */
static void m_report(MS *s) {
  unsigned maxcnt = (unsigned short)s->mp->cnt_[0], k, i;
  for (k = 1; k < 6; k++) if (s->mp->cnt_[k] > maxcnt) maxcnt = (unsigned short)s->mp->cnt_[k];
  if (s->level > 1) {
    M_D(s, "\n-----------------------------------------------------------------------------------------------------------------\r\n");
    if (s->level > 1) {
      M_D(s, "V90 Constellation Designer report:\r\n");
      if (s->level > 1) {
        M_D(s, "constelation size phase[0..5]  :  %d  %d  %d  %d  %d  %d\r\n", s->mp->cnt_[0], s->mp->cnt_[1], s->mp->cnt_[2], s->mp->cnt_[3], s->mp->cnt_[4], s->mp->cnt_[5]);
        if (s->level > 1) M_D(s, "----------- constel phase 0,1,2,3,4,5\t codec constel phase 0,1,2,3,4,5\tlinearMapping of constel --------------\r\n");
      }
    }
  }
  for (i = 0; i < maxcnt; i++) {
    if (s->level > 1) {
      const unsigned char *l1 = s->mp->listA_, *l2 = s->mp->listB_;
      M_D(s, "ucode[%d]  :  %d  %d  %d  %d  %d  %d  :  %d  %d  %d  %d  %d  %d  :  %d  %d  %d  %d  %d  %d\r\n", i, l1[i], l1[0x80 + i], l1[0x100 + i], l1[0x180 + i], l1[0x200 + i], l1[0x280 + i],
          l2[i], l2[0x80 + i], l2[0x100 + i], l2[0x180 + i], l2[0x200 + i], l2[0x280 + i], (int)s->A[l1[i]], (int)s->A[0x80 + l1[0x80 + i]], (int)s->A[0x100 + l1[0x100 + i]],
          (int)s->A[0x180 + l1[0x180 + i]], (int)s->A[0x200 + l1[0x200 + i]], (int)s->A[0x280 + l1[0x280 + i]]);
    }
  }
  if (s->level > 1) M_D(s, "---------------------------------------------------------------------------------------------------------------------\r\n");
}

/* ------------------------------------------------------------------------------------------------ model: setConstellationToNoise */
enum { CV_ST1, CV_ST0, CV_STNEG, CV_ST2, CV_ST3, CV_ST4, CV_RB_GE, CV_RB_LT, CV_UNRESTR, CV_NOISE_NAN, CV_FORCED, CV_ADJ53, CV_ADJ53_LO, CV_ADJ53_HI, CV_UP0, CV_UP_LE, CV_UP_GT,
       CV_DN0, CV_DN_GE, CV_DN_LT, CV_DUAL, CV_SINGLE, CV_ALAW, CV_ULAW, CV_CODE_GE, CV_CODE_LT, CV_CODE_NOCMP, CV_ACCEPT, CV_VALID_SKIP, CV_A_SKIP, CV_B_SKIP, CV_LO_GT_HI,
       CV_HALF_NEG, CV_LEVEL_ALL, CV_ADJ53_SKIP_W24, CV_MAXCNT_OVER, CV_HOOK_PAR, CV_HOOK_MP, CV_FR0 /* first id of the forceRate group */ };
enum { CF_N0 = CV_FR0, CF_N1, CF_N2, CF_ADJ, CF_NOADJ, CF_S_FOUND, CF_S128, CF_DUAL_PH, CF_FIXED_PH, CF_PLAIN_PH, CF_NU1, CF_NU_GT, CF_NU_LT, CF_NU_EQ, CF_ITER_LIM, CF_STEP_DEC,
       CF_SHRINK, CF_GROW, CF_GROW_MAX, CF_GROW_LATER, CF_BRK_A, CF_BRK_B, CF_ACC_SINGLE, CF_ACC_DUAL, CF_PHD_NAN, CF_SV9_NEG, CF_SV9_POS, CF_NSKIP_HANG, CF_BL0, CF_L_NEG,
       CF_HOOK_MP_MAX, CF_HOOK_PAR_MAX, CF_FIRST_ACCEPT, CF_PHD_BIG, CF_LAST };

static void m_scn(MS *s) {
  LD a = (LD)s->a;
  int old = (short)s->t.dMin_;
  int restricted = s->par->useRestricedDmin != 0;
  LD hiR, loR;       /* the values left on the x87 stack for the debug print */
  float a4;
  int ph, st;
  unsigned base;
  if (!restricted) {
    s->t.dMin_ = (uint16_t)m_i16(a * (LD)C428 + (LD)C42C);
    hiR = loR = __builtin_nanl("");
    a4 = 4.0f;
    s->cov[CV_UNRESTR] = 1;
  } else {
    LD A1 = a * (LD)C434 + 80.0L, B1 = a * (LD)C43C + 12.0L;
    LD m = (B1 >= A1) ? B1 : A1;
    s->t.dMin_ = (uint16_t)m_i16(m);
    hiR = A1; loR = B1;
    a4 = 2.0f;
    s->cov[B1 >= A1 ? CV_RB_GE : CV_RB_LT] = 1;
  }
  if (a != a) s->cov[CV_NOISE_NAN] = 1;
  if (s->level > 1) {
    int sg, ip, fp;
    float fh = RF(hiR), fl = RF(loR);
    m_split(a, 100.0L, &sg, &ip, &fp);
    M_D(s, "V90ConstellationDesigner: noiseEnergy = %c%d.%02d\r\n", sg, ip, fp);
    if (s->level > 1) {
      M_D(s, "V90ConstellationDesigner: dMinHighRates = %d  dMinLowRates = %d\r\n", m_i16((LD)fh), m_i16((LD)fl));
      if (s->level > 1) M_D(s, "V90ConstellationDesigner: current dMin = %d\r\n", (int)(short)s->t.dMin_);
    }
  }
  st = (int)s->t.state_;
  if (st == 1) {
    s->cov[CV_ST1] = 1;
    s->t.dMin_ = (uint16_t)(short)old;
    if (s->level > 1) M_D(s, "V90ConstellationDesigner: KeepRate => keep dMin and pdsnr thresh\r\n");
  } else if (st < 0) {
    s->cov[CV_STNEG] = 1;
  } else if (st == 0) {
    s->cov[CV_ST0] = 1;
    if (s->level > 1) M_D(s, "V90ConstellationDesigner: dMin calc NoRestriction\r\n");
    s->t.threshUp_ = RF(a * (LD)C44C); s->t.threshDown_ = RF(a * (LD)C450); s->t.threshRetrain_ = RF((LD)a4 * a);
    if (s->t.w24_ == 0) {
      int d = (short)s->t.dMin_;
      if (d > 0x3e && d <= 0x43) {
        s->cov[CV_ADJ53] = 1;
        if (d == 0x3f) s->cov[CV_ADJ53_LO] = 1;
        if (d == 0x43) s->cov[CV_ADJ53_HI] = 1;
        if (s->level > 1) M_D(s, "V90ConstellationDesigner: adjusting dMin for rate>=53k. Orig dMin=%d Modified dMin=%d\r\n", d, 0x3e);
        s->t.dMin_ = 0x3e;
      }
    } else s->cov[CV_ADJ53_SKIP_W24] = 1;
  } else if (st == 2) {
    int r = (short)s->t.rrnUpDmin_, d = (short)s->t.dMin_;
    s->cov[CV_ST2] = 1;
    if (r == 0) { s->cov[CV_UP0] = 1; }
    else if (r <= d) { s->cov[CV_UP_LE] = 1; d = r; }
    else s->cov[CV_UP_GT] = 1;
    s->t.dMin_ = (uint16_t)d;
    if (s->level > 1) {
      M_D(s, "V90ConstellationDesigner: dMin calc OneRateUp\r\n");
      if (s->level > 1) M_D(s, "V90ConstellationDesigner: rrnUpDmin = %d\r\n", (int)(short)s->t.rrnUpDmin_);
    }
    s->t.threshUp_ = RF(a * (LD)C44C); s->t.threshDown_ = RF(a * (LD)C450); s->t.threshRetrain_ = RF((LD)a4 * a);
  } else if (st == 3) {
    int r = (short)s->t.rrnDownDmin_, d = (short)s->t.dMin_;
    s->cov[CV_ST3] = 1;
    if (r == 0) { s->cov[CV_DN0] = 1; }
    else if (r >= d) { s->cov[CV_DN_GE] = 1; d = r; }
    else s->cov[CV_DN_LT] = 1;
    s->t.dMin_ = (uint16_t)d;
    if (s->level > 1) {
      M_D(s, "V90ConstellationDesigner: dMin calc OneRateDown\r\n");
      if (s->level > 1) M_D(s, "V90ConstellationDesigner: rrnDownDmin = %d\r\n", (int)(short)s->t.rrnDownDmin_);
    }
    s->t.threshUp_ = RF(a * (LD)C44C); s->t.threshDown_ = RF(a * (LD)C450); s->t.threshRetrain_ = RF((LD)a4 * a);
  } else {
    s->cov[CV_ST4] = 1;
  }
  s->t.state_ = 1;
  s->t.dMinAlt_ = (uint16_t)m_i16((LD)(short)s->t.dMin_ * (LD)1.25f);
  if (s->par->forcedDmin >= 0) {
    s->cov[CV_FORCED] = 1;
    s->t.dMin_ = (uint16_t)(short)s->par->forcedDmin;
    M_E(s, "V90ConstellationDesigner: dMin Forced to: %d\r\n", (int)(short)s->par->forcedDmin);
  }
  M_E(s, "V90ConstellationDesigner: final dMin = %d\r\n", (int)(short)s->t.dMin_);
  M_E(s, "V90ConstellationDesigner: USE_RESTRICED_DMIN = %d\r\n", (int)s->par->useRestricedDmin);
  { int sg, ip, fp;
    m_split((LD)s->t.threshUp_, 100.0L, &sg, &ip, &fp);   M_E(s, "V90ConstellationDesigner: pdSnrThreshForRateUp = %c%d.%02d\r\n", sg, ip, fp);
    m_split((LD)s->t.threshDown_, 100.0L, &sg, &ip, &fp); M_E(s, "V90ConstellationDesigner: pdSnrThreshForRateDown = %c%d.%02d\r\n", sg, ip, fp);
    m_split((LD)s->t.threshRetrain_, 100.0L, &sg, &ip, &fp); M_E(s, "V90ConstellationDesigner: pdSnrThreshForRetrain = %c%d.%02d\r\n", sg, ip, fp); }
  for (ph = 0; ph < 6; ph++) {
    unsigned char got[300];
    int cnt = 0, dualp = s->dual[ph] != 0, dm, half, j;
    unsigned lo = (unsigned)s->par->unk360, hi = s->lim[ph], u;
    s->cov[dualp ? CV_DUAL : CV_SINGLE] = 1;
    dm = dualp ? (short)s->t.dMinAlt_ : (short)s->t.dMin_;
    half = dm / 2;
    base = (unsigned)ph * 128u;
    if (lo > hi) s->cov[CV_LO_GT_HI] = 1;
    for (u = lo; u <= hi; u++) {
      int av = s->A[base + u], bv = s->B[base + u], mx;
      if (av <= (short)half) { s->cov[CV_A_SKIP] = 1; continue; }
      if (dualp && bv <= (short)half) { s->cov[CV_B_SKIP] = 1; continue; }
      if (s->valid[base + u] == 0) { s->cov[CV_VALID_SKIP] = 1; continue; }
      got[cnt++] = (unsigned char)u;
      s->cov[CV_ACCEPT] = 1;
      mx = dualp ? (bv >= av ? bv : av) : av;
      half = (short)(mx + dm);
      if (half < 0) s->cov[CV_HALF_NEG] = 1;
    }
    s->mp->cnt_[ph] = (uint32_t)cnt;
    for (j = 0; j < cnt; j++) {
      unsigned char uc = got[cnt - 1 - j];
      s->mp->listA_[base + j] = uc;
      if (s->t.useAlaw_ == s->t.w28_) {
        unsigned char code = m_code(s, s->A[base + uc]);
        s->cov[code >= uc ? CV_CODE_GE : CV_CODE_LT] = 1;
        s->mp->listB_[base + j] = code >= uc ? uc : m_code(s, s->A[base + uc]);
      } else {
        s->cov[CV_CODE_NOCMP] = 1;
        s->mp->listB_[base + j] = m_code(s, s->A[base + uc]);
      }
      s->cov[s->t.useAlaw_ ? CV_ALAW : CV_ULAW] = 1;
    }
  }
  m_report(s);
}

/* ------------------------------------------------------------------------------------------------ model: setConstellationToNoise_forceRate */
#define M_BUDGET 3000000L
static unsigned g_why[12];
#define WHY(k) (g_why[k]++)
static void m_fr(MS *s) {
  LD a = (LD)s->a, X, prod, Q, W, sv;
  int e16, n, i, ph;
  int s16 = 128;
  unsigned t;
  int nu[6];
  float phd[6];
  long budget = M_BUDGET;
  s->hang = 0;
  for (i = 0; i < 6; i++) if (s->cur[i] > 0x7e) { s->hang = 1; WHY(0); return; }   /* excluded: the blob's lists would run out of their rows */
  e16 = m_i16((LD)rate_scale() * (LD)(int)s->par->rateForce + (LD)0.5f);
  n = (short)((unsigned)(unsigned short)e16 + s->mp->w620_ - 6u);
  if (n < 0) { WHY(1); s->hang = 1; s->cov[CF_NSKIP_HANG] = 1; if (getenv("P24_HANG")) fprintf(stderr, "hang: n=%d\n", n); return; }
  { LD r = 1.0L, b2 = 2.0L; int k; for (k = n; k; k >>= 1) { if (k & 1) r *= b2; b2 *= b2; } X = r; }
  s->cov[n == 0 ? CF_N0 : n == 1 ? CF_N1 : CF_N2] = 1;
  prod = 1.0L;
  for (i = 0; i < 6; i++) { if (s->dual[i]) prod *= 4.0L; else if (s->lim[i]) prod *= 2.0L; }
  Q = prod * X;
  for (t = 1; t <= 127; t++) {
    LD w = (LD)t * (LD)t * (LD)t * (LD)t * (LD)t * (LD)t;
    if (w > Q) { s16 = (int)t - 1; s->cov[CF_S_FOUND] = 1; break; }
  }
  if (s16 == 128) s->cov[CF_S128] = 1;
  sv = (LD)s16;
  W = sv * sv * sv * sv * sv * sv;
  for (i = 0; i < 6; i++) {
    if (s->dual[i]) { nu[i] = (unsigned short)m_i16(sv * (LD)0.25f); W *= (LD)0.25f; s->cov[CF_DUAL_PH] = 1; }
    else if (s->lim[i]) { nu[i] = (unsigned short)m_i16(sv * (LD)0.5f); W *= (LD)0.5f; s->cov[CF_FIXED_PH] = 1; }
    else { nu[i] = (unsigned short)s16; s->cov[CF_PLAIN_PH] = 1; }
  }
  if (W < X || W != W) {
    LD acc;
    s->cov[CF_ADJ] = 1;
    do {
      int best = 32000, idx = 0;
      for (i = 0; i < 6; i++) if (!s->lim[i] && nu[i] < best) { best = nu[i]; idx = i; }
      nu[idx] = (nu[idx] + 1) & 0xffff;
      acc = 1.0L;
      for (i = 0; i < 6; i++) acc *= (LD)nu[i];
      if (--budget < 0) { WHY(2); s->hang = 1; s->cov[CF_NSKIP_HANG] = 1; if (getenv("P24_HANG")) fprintf(stderr, "hang: adjust loop n=%d\n", n); return; }
    } while (acc < X || acc != acc);
  } else s->cov[CF_NOADJ] = 1;
  for (i = 0; i < 6; i++) if (nu[i] > 127) { WHY(3); s->hang = 1; return; }   /* excluded: more than 127 list entries would leave the rows */
  for (i = 0; i < 6; i++) {
    LD r = 1.0L / ((LD)nu[i] - (LD)0.5f);
    phd[i] = RF((LD)m_i16((LD)s->A[i * 128 + s->cur[i]] * r));
    if (phd[i] != phd[i]) s->cov[CF_PHD_NAN] = 1;
  }
  { LD t10 = a * (LD)10.0f;
    s->t.threshUp_ = RF(a * (LD)C470);
    s->t.threshDown_ = RF(t10);
    s->t.threshRetrain_ = RF(t10);
    M_E(s, "V90ConstellationDesigner: pdSnrThreshForRateUp = %d pdSnrThreshForRateDown = %d pdSnrThreshForRetrain = %d\r\n", m_i32((LD)s->t.threshUp_), m_i32((LD)s->t.threshDown_), m_i32(t10)); }
  M_E(s, "V90ConstellationDesigner: requested rate force is %d\n", (int)s->par->rateForce);
  M_E(s, "V90ConstellationDesigner: initial nofUcodeInPhase[1..6] = %d %d %d %d %d %d\n", nu[0], nu[1], nu[2], nu[3], nu[4], nu[5]);
  M_E(s, "V90ConstellationDesigner: initial phaseDmin[1..6] = %d %d %d %d %d %d\n", m_i16((LD)phd[0]), m_i16((LD)phd[1]), m_i16((LD)phd[2]), m_i16((LD)phd[3]), m_i16((LD)phd[4]), m_i16((LD)phd[5]));
  { MapPar90 *mp = s->mp;   /* the register copy of T->mapParams_ (reloaded after the "reached max" print) */
    for (ph = 0; ph < 6; ph++) {
      unsigned base = (unsigned)ph * 128u, cnt = 1;
      float step = C478;
      int sv9 = 0, iter = 0, dualp = s->dual[ph] != 0;
      unsigned nuv = (unsigned)nu[ph];
      if (nuv != 1) {
        int more;
        do {
          int bl = s->cur[ph], c = m_i16((LD)phd[ph]), Lim;
          int ia = (int)(base + (unsigned)bl);
          unsigned u;
          if (dualp) { int x = s->B[ia] - c, y = s->A[ia] - c; Lim = (short)(x < y ? x : y); }
          else Lim = (short)(s->A[ia] - c);
          mp->listA_[base] = (unsigned char)bl;
          mp->listB_[base] = m_code(s, s->A[ia]);
          cnt = 1;
          s->cov[CF_FIRST_ACCEPT] = 1;
          u = (unsigned)bl - 1u;
          if (bl == 0) { WHY(4); s->hang = 1; return; }   /* excluded: the scan would start at index -1 and run down through memory */
          if (!((unsigned)s->par->unk360 > u)) {
            for (;;) {
              LD half = (LD)phd[ph] * (LD)0.5f;
              int ix = (int)(base + u);
              int av = s->A[ix], bv = dualp ? s->B[ix] : 0, skip;
              if (!((LD)av >= half)) { s->cov[CF_BRK_A] = 1; break; }
              if (dualp && !((LD)bv >= half)) { s->cov[CF_BRK_B] = 1; break; }
              skip = dualp ? (av > Lim || bv > Lim) : (av > Lim || s->valid[ix] == 0);
              if (!skip) {
                int c2 = m_i16((LD)phd[ph]);
                mp->listA_[base + cnt] = (unsigned char)u;
                mp->listB_[base + cnt] = m_code(s, s->A[base + (unsigned)mp->listA_[base + cnt]]);
                cnt++;
                s->cov[dualp ? CF_ACC_DUAL : CF_ACC_SINGLE] = 1;
                if (dualp) { int x = s->B[ix] - c2, y = s->A[ix] - c2; Lim = (short)(x < y ? x : y); }
                else Lim = (short)(s->A[ix] - c2);
                if (Lim < 0) s->cov[CF_L_NEG] = 1;
              }
              u--;
              if (u == 0xffffffffu) { WHY(5); s->hang = 1; return; }   /* excluded: unk360 == 0 lets the descending scan wrap */
              if ((unsigned)s->par->unk360 > u) break;
            }
          }
          if (nuv > cnt) {
            phd[ph] = RF(((LD)1.0f - (LD)step) * (LD)phd[ph]);
            if (sv9 <= 0) sv9++;
            s->cov[CF_NU_GT] = 1;
          } else if (nuv < cnt) {
            phd[ph] = RF(((LD)step + (LD)1.0f) * (LD)phd[ph]);
            if (sv9 >= 0) sv9--;
            s->cov[CF_NU_LT] = 1;
          } else s->cov[CF_NU_EQ] = 1;
          if (sv9 == 0) { step = RF((LD)step * (LD)C47C); s->cov[CF_STEP_DEC] = 1; }
          if (sv9 < 0) s->cov[CF_SV9_NEG] = 1;
          if (sv9 > 0) s->cov[CF_SV9_POS] = 1;
          iter++;
          if (iter > 199) s->cov[CF_ITER_LIM] = 1;
          if (phd[ph] > 1e8f || phd[ph] < -1e8f) s->cov[CF_PHD_BIG] = 1;
          mp = s->mp;
          more = (nuv != cnt) && (iter <= 199);
        } while (more);
      } else s->cov[CF_NU1] = 1;
      if (nuv < cnt) {
        s->cov[CF_SHRINK] = 1;
        while (nuv < cnt) {
          unsigned j;
          cnt--;
          for (j = 0; j < cnt; j++) { mp->listA_[base + j] = mp->listA_[base + j + 1]; mp->listB_[base + j] = mp->listB_[base + j + 1]; }
        }
      }
      while (nuv > cnt) {
        unsigned u0 = mp->listA_[base], cand = 0, found = 0, k;
        s->cov[CF_GROW] = 1;
        if (u0 > 0x73) {
          M_E(s, "V90ConstellationDesigner: reached max posible ucode -> Not able to reach requested rate !!!");
          mp = s->mp;
          s->cov[CF_GROW_MAX] = 1;
          if (s->hook_kind == HK_MP && s->hook_k == s->nprint) s->cov[CF_HOOK_MP_MAX] = 1;
          if (s->hook_kind == HK_PAR && s->hook_k == s->nprint) s->cov[CF_HOOK_PAR_MAX] = 1;
          break;
        }
        for (cand = u0 + 1; cand <= 0x74; cand++) {
          LD sum = (LD)s->A[base + u0] + (LD)phd[ph];
          if (!(sum >= (LD)s->A[base + cand])) { found = 1; break; }
        }
        if (!found) { WHY(6); s->hang = 1; s->cov[CF_NSKIP_HANG] = 1; if (getenv("P24_HANG")) fprintf(stderr, "hang: grow ph=%d u0=%u nu=%u cnt=%u phd=%g\n", ph, u0, nuv, cnt, (double)phd[ph]); return; }
        if (cand > u0 + 1) s->cov[CF_GROW_LATER] = 1;
        for (k = cnt; k != 0; k--) { mp->listA_[base + k] = mp->listA_[base + k - 1]; mp->listB_[base + k] = mp->listB_[base + k - 1]; }
        mp->listA_[base] = 0;
        mp->listB_[base] = m_code(s, s->A[base]);
        s->cur[ph] = (unsigned char)(s->cur[ph] + 1);
        mp = s->mp;
        cnt++;
        if (--budget < 0) { s->hang = 1; return; }
      }
      mp->cnt_[ph] = cnt;
      if (getenv("P24_MDBG")) fprintf(stderr, "  model ph %d nu %u cnt %u iter %d sv9 %d phd %g cur %d\n", ph, nuv, cnt, iter, sv9, (double)phd[ph], s->cur[ph]);
    }
  }
  M_E(s, "V90ConstellationDesigner: final nofUcodeInPhase[1..6] = %d %d %d %d %d %d\n", s->mp->cnt_[0], s->mp->cnt_[1], s->mp->cnt_[2], s->mp->cnt_[3], s->mp->cnt_[4], s->mp->cnt_[5]);
  M_E(s, "V90ConstellationDesigner: final phaseDmin[1..6] = %d %d %d %d %d %d\n", m_i16((LD)phd[0]), m_i16((LD)phd[1]), m_i16((LD)phd[2]), m_i16((LD)phd[3]), m_i16((LD)phd[4]), m_i16((LD)phd[5]));
  m_report(s);
}

/* ------------------------------------------------------------------------------------------------ coverage ids */
static const char *const cov_names[CF_LAST] = {
  [CV_ST1] = "state 1 KeepRate", [CV_ST0] = "state 0 NoRestriction", [CV_STNEG] = "state < 0", [CV_ST2] = "state 2 OneRateUp", [CV_ST3] = "state 3 OneRateDown", [CV_ST4] = "state >= 4",
  [CV_RB_GE] = "restricted B >= A", [CV_RB_LT] = "restricted B < A", [CV_UNRESTR] = "unrestricted", [CV_NOISE_NAN] = "noise NaN", [CV_FORCED] = "forced dMin", [CV_ADJ53] = "53k dMin adjust",
  [CV_ADJ53_LO] = "adjust at 0x3f", [CV_ADJ53_HI] = "adjust at 0x43", [CV_UP0] = "up: rrnUp == 0", [CV_UP_LE] = "up: rrnUp <= dMin", [CV_UP_GT] = "up: rrnUp > dMin", [CV_DN0] = "down: rrnDown == 0",
  [CV_DN_GE] = "down: rrnDown >= dMin", [CV_DN_LT] = "down: rrnDown < dMin", [CV_DUAL] = "dual phase", [CV_SINGLE] = "single phase", [CV_ALAW] = "A-law", [CV_ULAW] = "u-law",
  [CV_CODE_GE] = "code >= ucode (kept)", [CV_CODE_LT] = "code < ucode (recomputed)", [CV_CODE_NOCMP] = "useAlaw != w28", [CV_ACCEPT] = "ucode accepted", [CV_VALID_SKIP] = "valid == 0 skip",
  [CV_A_SKIP] = "A <= half skip", [CV_B_SKIP] = "B <= half skip", [CV_LO_GT_HI] = "lo > hi", [CV_HALF_NEG] = "half < 0", 
  [CV_ADJ53_SKIP_W24] = "w24 != 0 skips the 53k adjust", [CV_HOOK_PAR] = "params_ hook", [CV_HOOK_MP] = "mapParams_ hook",
  [CF_N0] = "fr n == 0", [CF_N1] = "fr n == 1", [CF_N2] = "fr n >= 2", [CF_ADJ] = "fr product adjust loop", [CF_NOADJ] = "fr W >= X", [CF_S_FOUND] = "fr s found", [CF_S128] = "fr s = 128",
  [CF_DUAL_PH] = "fr dual phase", [CF_FIXED_PH] = "fr fixed phase", [CF_PLAIN_PH] = "fr plain phase", [CF_NU1] = "fr nu == 1", [CF_NU_GT] = "fr nu > cnt step", [CF_NU_LT] = "fr nu < cnt step",
  [CF_NU_EQ] = "fr nu == cnt", [CF_ITER_LIM] = "fr 199 iterations", [CF_STEP_DEC] = "fr step * 0.9", [CF_SHRINK] = "fr shrink", [CF_GROW] = "fr grow", [CF_GROW_MAX] = "fr grow: max ucode message",
  [CF_GROW_LATER] = "fr grow: later candidate", [CF_BRK_A] = "fr scan break on A", [CF_BRK_B] = "fr scan break on B", [CF_ACC_SINGLE] = "fr accept (single)", [CF_ACC_DUAL] = "fr accept (dual)",
  [CF_SV9_NEG] = "fr sV9 < 0", [CF_SV9_POS] = "fr sV9 > 0", [CF_NSKIP_HANG] = "fr input the blob would not finish (skipped)", [CF_BL0] = "fr cur == 0",
  [CF_L_NEG] = "fr L < 0", [CF_HOOK_MP_MAX] = "fr hook swapped mp at the max-ucode print", [CF_HOOK_PAR_MAX] = "fr hook swapped params at the max-ucode print", [CF_FIRST_ACCEPT] = "fr iteration body", [CF_PHD_BIG] = "fr |phd| huge",
};
static void cov_define_range(int from, int to) {
  int i;
  for (i = from; i < to; i++) if (cov_names[i]) h_cov_define(i < H_COV_STRADDLE ? i : i + 2, cov_names[i]);
}
static int cov_id(int i) { return i < H_COV_STRADDLE ? i : i + 2; }   /* ids 60 / 61 are reserved by the harness library */
static void cov_hit_model(const MS *s) {
  int i;
  for (i = 0; i < CF_LAST; i++) if (s->cov[i]) h_cov_hit(cov_id(i));
}
enum { C_TRIALS = 90, C_SKIPPED, C_L0, C_L1, C_L2, C_LHI, C_CHAIN, C_ALT_HOOK_MP, C_ALT_HOOK_PAR, C_HOOK_LEVEL };
static void cov_define_trials(void) {
  h_cov_define(C_TRIALS, "trials"); h_cov_define(C_SKIPPED, "skipped (blob would not terminate)"); h_cov_define(C_L0, "level 0"); h_cov_define(C_L1, "level 1");
  h_cov_define(C_L2, "level 2 (prints)"); h_cov_define(C_LHI, "level >= 2^31"); h_cov_define(C_CHAIN, "second call on the same object");
  h_cov_define(C_ALT_HOOK_MP, "mapParams_ hook"); h_cov_define(C_ALT_HOOK_PAR, "params_ hook"); h_cov_define(C_HOOK_LEVEL, "level hook");
}

/* ------------------------------------------------------------------------------------------------ the objects of a trial */
#define PADS 64
typedef struct {
  short *Abuf, *Bbuf;            /* PADS shorts of slack in front, 768 + 256 behind */
  unsigned char *Vbuf;
  short *A, *B;
  unsigned char *V;
  short *dual;                   /* 8 shorts */
  unsigned char *lim;            /* 8 bytes: maxU / fixed */
  unsigned char *cur;            /* 8 bytes */
  MapPar90 *mp, *altmp;
} Env;
#define ARR_SHORTS (PADS + 768 + 256)
static CdObj g_t;
static V90ParObj g_par, g_altpar;
static MS g_ms;
static unsigned g_nskipped;

static void env_new(Env *e) {
  e->Abuf = (short *)h_obj_new(ARR_SHORTS * 2);
  e->Bbuf = (short *)h_obj_new(ARR_SHORTS * 2);
  e->Vbuf = (unsigned char *)h_obj_new(ARR_SHORTS);
  e->A = e->Abuf + PADS; e->B = e->Bbuf + PADS; e->V = e->Vbuf + PADS;
  e->dual = (short *)h_obj_new(16);
  e->lim = (unsigned char *)h_obj_new(16);
  e->cur = (unsigned char *)h_obj_new(16);
  e->mp = (MapPar90 *)h_obj_new(sizeof(MapPar90));
  e->altmp = (MapPar90 *)h_obj_new(sizeof(MapPar90));
}

/* a distance table: G.711 decoded values of the ucode index (code c of a u-law / A-law byte) with perturbations, increasing in u unless `jag` */
static int ucode_value(int alaw, int u) {
  int v = alaw ? alaw2linear((u ^ 0xd5) & 0xff) : ulaw2linear((~u) & 0xff);
  return v < 0 ? -v : v;
}
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
  for (u = 0; u < 256 - 0; u++) tab[768 + u] = (short)(h_rnd() & 0x7fff);
}
/* the object: member-wise (its size and the offsets behind the pointers differ on x86-64), unknown regions set to fixed words */
static void t_prefill(CdObj *t) {
  memset(t, 0, sizeof *t);
  t->flag8_ = (uint8_t)h_rnd(); t->unk09_[0] = 0x5a; t->dMin_ = (uint16_t)h_rnd(); t->rrnDownDmin_ = (uint16_t)h_rnd(); t->rrnUpDmin_ = (uint16_t)h_rnd(); t->dMinAlt_ = (uint16_t)h_rnd();
  t->unk12_[0] = 0xa5; t->unk12_[1] = 0x3c;
  t->threshUp_ = f_of(rf_bits()); t->threshDown_ = f_of(rf_bits()); t->threshRetrain_ = f_of(rf_bits());
  t->w24_ = h_rnd(); t->w28_ = h_rnd(); t->useAlaw_ = h_rnd();
  t->unk34_[0] = 1; t->unk34_[1] = 2; t->unk34_[2] = 3; t->unk34_[3] = 4;
  t->w38_ = (uint8_t)h_rnd(); t->unk39_[0] = 9; t->unk39_[1] = 8; t->unk39_[2] = 7;
  t->w3c_ = h_rnd(); t->w40_ = h_rnd(); t->state_ = h_rnd(); t->maxRate_ = h_rnd(); t->minRate_ = h_rnd();
}
static void par_fill(V90ParObj *p) {
  memset(p, 0, sizeof *p);
  p->unk360 = 0; p->useRestricedDmin = 0; p->forcedDmin = -1; p->rateForce = 0;
}
static unsigned long g_dumpctr;
static void dump_all(const char *tag, const CdObj *t, const MapPar90 *mp, const MapPar90 *amp, const Env *e) {
  CdObj tmp;
  int show = h_sample(&g_dumpctr, 40, 97);
  memcpy(&tmp, t, sizeof tmp);
  tmp.params_ = NULL; tmp.mapParams_ = NULL; tmp.adid_ = NULL; tmp.power_ = NULL; tmp.preFilter_ = NULL;
  h_dump_obj(show, tag, &tmp, f_CdObj, NF_CdObj);
  h_dump_obj(show, "mp", mp, f_mp, NF_mp);
  if (amp) h_dump_obj(show, "alt", amp, f_mp, NF_mp);
  h_emit(show, "cur %02x %02x %02x %02x %02x %02x", e->cur[0], e->cur[1], e->cur[2], e->cur[3], e->cur[4], e->cur[5]);
}

typedef struct {
  int fr;                      /* 0 = setConstellationToNoise, 1 = forceRate */
  float a;
  unsigned level;
  int hook_kind;
  unsigned hook_k, hook_level;
  const char *hook_text;       /* steering: place the hook after the first print whose text starts with this (found by a model run without the hook) */
} Call;

static unsigned char g_sA[ARR_SHORTS * 2], g_sB[ARR_SHORTS * 2], g_sV[ARR_SHORTS];
static void run_model(const Call *c, Env *e) {
  memset(&g_ms, 0, sizeof g_ms);
  g_ms.a = c->a; g_ms.A = e->A; g_ms.B = e->B; g_ms.dual = e->dual; g_ms.lim = e->lim; g_ms.valid = e->V;
  memcpy(g_ms.cur, e->cur, 8);
  memcpy(&g_ms.t, &g_t, sizeof g_ms.t);
  g_ms.par = &g_par; g_ms.altpar = &g_altpar;
  memcpy(&g_ms.mpm, e->mp, sizeof g_ms.mpm); memcpy(&g_ms.altm, e->altmp, sizeof g_ms.altm);
  g_ms.mp = &g_ms.mpm;
  g_ms.level = c->level; g_ms.hook_kind = c->hook_kind; g_ms.hook_k = c->hook_k; g_ms.hook_level = c->hook_level;
  if (c->fr) m_fr(&g_ms); else m_scn(&g_ms);
  if (getenv("P24_MDBG")) fprintf(stderr, "  model done hang=%d nlog=%u\n", g_ms.hang, g_ms.nlog);
}

/* one call of the blob / port function and of the model on the same state; returns 0 when the model says the blob would not terminate (nothing is run then) */
static int do_call(const Call *c0, Env *e, int emit, int chain) {
  unsigned i;
  CdObj tm, tr;
  Call cl = *c0;
  const Call *c = &cl;
  /* model on copies */
  memcpy(g_sA, e->Abuf, sizeof g_sA); memcpy(g_sB, e->Bbuf, sizeof g_sB); memcpy(g_sV, e->Vbuf, sizeof g_sV);
  if (cl.hook_text) {
    Call nohook = cl;
    nohook.hook_kind = HK_NONE; nohook.hook_k = 0;
    run_model(&nohook, e);
    cl.hook_kind = c0->hook_kind;
    cl.hook_k = 0;
    if (!g_ms.hang) {
      unsigned n;
      for (n = 0; n < g_ms.nlog && n < LOGMAX; n++) if (!strncmp(g_ms.log[n].text, cl.hook_text, strlen(cl.hook_text))) { cl.hook_k = n + 1; break; }
    }
    if (!cl.hook_k) cl.hook_kind = HK_NONE;
  }
  run_model(c, e);
  if (g_ms.hang) { g_nskipped++; h_cov_hit(C_SKIPPED); return 0; }
  if (getenv("P24_TRACE")) { fprintf(stderr, "call fr=%d a=%08x lvl=%u dual=%d,%d,%d,%d,%d,%d lim=%d,%d,%d,%d,%d,%d cur=%d,%d,%d,%d,%d,%d unk360=%d rate=%d w620=%u hook=%d/%u\n", c->fr, h_bits_of(c->a), c->level, e->dual[0], e->dual[1], e->dual[2], e->dual[3], e->dual[4], e->dual[5], e->lim[0], e->lim[1], e->lim[2], e->lim[3], e->lim[4], e->lim[5], e->cur[0], e->cur[1], e->cur[2], e->cur[3], e->cur[4], e->cur[5], g_par.unk360, g_par.rateForce, e->mp->w620_, cl.hook_kind, cl.hook_k); }
  /* the real call */
  g_t.params_ = &g_par; g_t.mapParams_ = e->mp;
  g_alt_par = &g_altpar; g_alt_mp = e->altmp;
  g_snap_t = &g_t; g_snap_mp = e->mp;
  g_hook_kind = c->hook_kind; g_hook_k = c->hook_k; g_hook_level = c->hook_level;
  dsplibs_debug_level = c->level;
  log_begin();
  if (c->fr) cd_fr(&g_t, c->a, (short (*)[128])e->A, (short (*)[128])e->B, e->dual, e->lim, e->cur, (unsigned char (*)[128])e->V);
  else cd_scn(&g_t, c->a, (short (*)[128])e->A, (short (*)[128])e->B, e->dual, e->lim, (unsigned char (*)[128])e->V);
  g_hook_k = 0;
  g_snap_t = NULL; g_snap_mp = NULL;
  /* compare with the model */
  if (g_nlog != g_ms.nlog) {
    for (i = 0; i < g_nlog && i < 12; i++) fprintf(stderr, "real %u %c %s\n", i, g_log[i].kind, g_log[i].text);
    for (i = 0; i < g_ms.nlog && i < 12; i++) fprintf(stderr, "model %u %c %s\n", i, g_ms.log[i].kind, g_ms.log[i].text);
    h_die3("print count: real %u model %u (call %d level %u hook %d/%u)", g_nlog, g_ms.nlog, c->fr, c->level, c->hook_kind, c->hook_k);
  }
  for (i = 0; i < g_nlog && i < LOGMAX; i++)
    if (g_log[i].kind != g_ms.log[i].kind || strcmp(g_log[i].text, g_ms.log[i].text))
      h_die3("print %u differs (call %d level %u hook %d/%u):\n  real  %c %s\n  model %c %s", i, c->fr, c->level, c->hook_kind, c->hook_k, g_log[i].kind, g_log[i].text, g_ms.log[i].kind, g_ms.log[i].text);
  memcpy(&tm, &g_ms.t, sizeof tm); memcpy(&tr, &g_t, sizeof tr);
  tm.params_ = tm.mapParams_ = tr.params_ = tr.mapParams_ = NULL;
  if (memcmp(&tm, &tr, sizeof tm)) {
    for (i = 0; i < sizeof tm; i++) if (((uint8_t *)&tm)[i] != ((uint8_t *)&tr)[i]) h_die3("object byte +0x%x: real 0x%02x model 0x%02x (call %d level %u)", i, ((uint8_t *)&tr)[i], ((uint8_t *)&tm)[i], c->fr, c->level);
  }
  if ((g_t.params_ == (void *)&g_altpar) != (g_ms.par == &g_altpar)) h_die3("params_ pointer after the call");
  if ((g_t.mapParams_ == (void *)e->altmp) != (g_ms.mp == &g_ms.altm)) h_die3("mapParams_ pointer after the call");
  for (i = 0; i < sizeof(MapPar90); i++) {
    if (((uint8_t *)e->mp)[i] != ((uint8_t *)&g_ms.mpm)[i]) h_die3("mapping block byte +0x%x: real 0x%02x model 0x%02x (call %d)", i, ((uint8_t *)e->mp)[i], ((uint8_t *)&g_ms.mpm)[i], c->fr);
    if (((uint8_t *)e->altmp)[i] != ((uint8_t *)&g_ms.altm)[i]) h_die3("alt mapping block byte +0x%x: real 0x%02x model 0x%02x (call %d)", i, ((uint8_t *)e->altmp)[i], ((uint8_t *)&g_ms.altm)[i], c->fr);
  }
  if (c->fr) for (i = 0; i < 8; i++) if (e->cur[i] != g_ms.cur[i]) h_die3("cur[%u]: real %u model %u", i, e->cur[i], g_ms.cur[i]);
  if (memcmp(g_sA, e->Abuf, sizeof g_sA) || memcmp(g_sB, e->Bbuf, sizeof g_sB) || memcmp(g_sV, e->Vbuf, sizeof g_sV)) h_die3("an input table was modified");
  cov_hit_model(&g_ms);
  h_cov_hit(C_TRIALS);
  if (c->level == 0) h_cov_hit(C_L0); else if (c->level == 1) h_cov_hit(C_L1); else if (c->level >= 0x80000000u) h_cov_hit(C_LHI); else h_cov_hit(C_L2);
  if (chain) h_cov_hit(C_CHAIN);
  if (c->hook_kind == HK_MP && g_ms.nprint >= c->hook_k) h_cov_hit(C_ALT_HOOK_MP);
  if (c->hook_kind == HK_PAR && g_ms.nprint >= c->hook_k) h_cov_hit(C_ALT_HOOK_PAR);
  if (c->hook_kind == HK_LEVEL && g_ms.nd >= c->hook_k) h_cov_hit(C_HOOK_LEVEL);
  if (c->hook_kind == HK_PAR && g_ms.nprint >= c->hook_k) h_cov_hit(CV_HOOK_PAR);
  if (c->hook_kind == HK_MP && g_ms.nprint >= c->hook_k) h_cov_hit(CV_HOOK_MP);
  if (emit) {
    h_emit(0, "call fr %d prints %u", c->fr, g_nlog);
    dump_all("T", &g_t, e->mp, c->hook_kind == HK_MP ? e->altmp : NULL, e);
  }
  return 1;
}

/* ------------------------------------------------------------------------------------------------ generators */
static uint32_t noise_bits(int edge) {
  uint32_t r = h_rmod(100);
  float f;
  uint32_t b;
  if (edge && r < 12) { static const uint32_t sp[] = { 0x00000000u, 0x80000000u, 0x7f800000u, 0xff800000u, 0x7fc00000u, 0x3f7fffffu, 0x42c7ffffu, 0x461c3fffu, 0x4f000000u, 0xcf000000u, 0x00000001u }; return sp[h_rmod(11)]; }
  if (r < 50) { f = (float)((int)h_rrange(0, 4000)) / 64.0f; memcpy(&b, &f, 4); return b; }       /* 0 .. 62: the noise energies of the receiver */
  if (r < 62) { f = (float)((int)h_rrange(-400, 4000)) / 8.0f; memcpy(&b, &f, 4); return b; }
  if (r < 75) { f = (float)((int)h_rrange(1, 60)); memcpy(&b, &f, 4); return b; }
  if (r < 85) { f = (float)((int)h_rrange(0, 100000)) / 16.0f; memcpy(&b, &f, 4); return b; }
  return rf_bits();
}
static void gen_alt_objects(Env *e) {
  uint32_t k;
  memset(&g_altpar, 0, sizeof g_altpar);
  g_altpar.unk360 = (int32_t)(h_rpct(50) ? h_rmod(8) : h_rmod(60));
  g_altpar.useRestricedDmin = (int32_t)(h_rpct(50) ? 1 : 0);
  g_altpar.forcedDmin = (int32_t)(h_rpct(50) ? -1 : (int)h_rrange(0, 300));
  g_altpar.rateForce = (int32_t)(28000 + h_rmod(28000));
  for (k = 0; k < sizeof(MapPar90); k += 4) { uint32_t w = h_rnd(); memcpy((uint8_t *)e->altmp + k, &w, 4); }
  e->altmp->w620_ = h_rmod(30);
  for (k = 0; k < 6; k++) e->altmp->cnt_[k] = h_rmod(60);
}
static void pick_hook(Call *c, unsigned maxk) {
  c->hook_kind = HK_NONE; c->hook_k = 0; c->hook_level = 0; c->hook_text = NULL;
  if (h_rpct(14)) {
    unsigned r = h_rmod(3);
    c->hook_kind = r == 0 ? HK_LEVEL : r == 1 ? HK_PAR : HK_MP;
    c->hook_k = 1 + h_rmod(maxk);
    c->hook_level = h_rpct(60) ? 0u : 1u;
  }
}
static void fill_dual_lim(Env *e, int pdual, int plim, int maxu_mode) {
  int ph;
  for (ph = 0; ph < 8; ph++) {
    e->dual[ph] = (short)(h_rpct(pdual) ? (h_rpct(60) ? 1 : (int)(short)h_rnd() | 1) : (h_rpct(10) ? 0 : 0));
    if (maxu_mode == 0) e->lim[ph] = (unsigned char)(h_rpct(plim) ? (h_rpct(70) ? 1 + h_rmod(255) : 1) : 0);
    else e->lim[ph] = (unsigned char)(h_rpct(8) ? h_rmod(256) : h_rpct(30) ? 127 : h_rrange(0, 127));
  }
  if (maxu_mode != 0) { /* maxU never above 127 (the stack buffer of the blob holds 140 bytes) */ for (ph = 0; ph < 8; ph++) if (e->lim[ph] > 127) e->lim[ph] = (unsigned char)(e->lim[ph] & 0x7f); }
}

/* setConstellationToNoise: one random case */
static void gen_scn(Env *e, Call *c, int scan_heavy) {
  int alaw = h_rpct(50), jag = h_rpct(40) ? 0 : (int)h_rmod(60);
  uint32_t k;
  c->fr = 0;
  c->a = f_of(noise_bits(!scan_heavy));
  c->level = pick_level();
  t_prefill(&g_t);
  par_fill(&g_par);
  par_fill(&g_par);
  gen_alt_objects(e);
  g_par.useRestricedDmin = h_rpct(60) ? 1 : (h_rpct(70) ? 0 : (int32_t)h_rnd());
  if (h_rpct(20)) {   /* noise that puts the computed dMin next to the 53k window 0x3f .. 0x43 (integer arithmetic: the data path has no float arithmetic) */
    int d = 0x3d + (int)h_rmod(9), ai;
    if (g_par.useRestricedDmin == 0) ai = (d * 1024 * 1000 - 9900 * 1024) / 6776; else ai = ((d - 80) * 1024 * 1000) / 3333;
    c->a = (float)(ai + (int)h_rrange(-2, 2)) / 1024.0f;
  }
  g_par.forcedDmin = h_rpct(78) ? -1 : h_rpct(50) ? (int32_t)h_rrange(0, 600) : (int32_t)(h_rpct(50) ? h_rnd() : h_rpct(50) ? 0x7fffffff : -2);
  g_par.unk360 = (int32_t)(h_rpct(55) ? h_rmod(4) : h_rpct(70) ? h_rmod(130) : h_rpct(50) ? (int)h_rnd() : -1);
  { static const int32_t sts[] = { 0, 1, 2, 3, 4, 5, 100, -1, (int32_t)0x80000000, 0x7fffffff, (int32_t)0x80000001 };
    g_t.state_ = (uint32_t)(h_rpct(88) ? (int32_t)h_rmod(4) : sts[h_rmod(11)]); }
  g_t.dMin_ = (uint16_t)(h_rpct(25) ? (int)h_rrange(0x3c, 0x46) : h_rpct(60) ? (int)h_rrange(-40, 400) : (int)h_rnd());
  g_t.rrnUpDmin_ = (uint16_t)(h_rpct(25) ? 0 : h_rpct(70) ? (int)((short)g_t.dMin_ + (int)h_rrange(-3, 3)) : (int)h_rnd());
  g_t.rrnDownDmin_ = (uint16_t)(h_rpct(25) ? 0 : h_rpct(70) ? (int)((short)g_t.dMin_ + (int)h_rrange(-3, 3)) : (int)h_rnd());
  g_t.w24_ = h_rpct(60) ? 0u : h_rnd();
  g_t.useAlaw_ = (uint32_t)(alaw ? (h_rpct(85) ? 1 : (int)h_rnd() | 1) : 0);
  g_t.w28_ = h_rpct(60) ? g_t.useAlaw_ : (h_rpct(50) ? (uint32_t)(g_t.useAlaw_ == 0) : h_rnd());
  fill_table(e->A, alaw, jag, 1);
  fill_table(e->B, h_rpct(80) ? alaw : !alaw, jag, h_rpct(80) ? 1 : 2);
  if (h_rpct(20)) for (k = 0; k < 768; k++) e->A[k] = (short)(e->A[k] > 0 ? e->A[k] : -e->A[k]);
  for (k = 0; k < 768 + 256; k++) e->V[k] = (unsigned char)(h_rpct(88) ? 1 : h_rpct(70) ? 0 : (int)h_rnd());
  fill_dual_lim(e, scan_heavy ? 45 : 30, 0, 1);
  for (k = 0; k < sizeof(MapPar90); k += 4) { uint32_t w = h_rnd(); memcpy((uint8_t *)e->mp + k, &w, 4); }
  pick_hook(c, scan_heavy ? 3 : 12);
}

/* forceRate: one random case; edge selects the aggressive generators */
static void gen_fr(Env *e, Call *c, int edge) {
  int alaw = h_rpct(50), n, rate, e16, ph;
  uint32_t k;
  c->fr = 1;
  c->a = f_of(noise_bits(edge));
  c->level = pick_level();
  t_prefill(&g_t);
  par_fill(&g_par);
  gen_alt_objects(e);
  g_t.useAlaw_ = (uint32_t)(alaw ? 1 : 0);
  g_par.unk360 = (int32_t)(h_rpct(60) ? h_rmod(3) : h_rpct(70) ? h_rmod(60) : h_rpct(30) ? (int)h_rnd() : 200);
  /* the rate -> n relation: n = (short)(trunc(0.00075 * rate + 0.5) + w620 - 6) */
  if (h_rpct(edge ? 30 : 8)) {   /* around an integer boundary of the product */
    int j = 18 + (int)h_rmod(30);
    rate = (int)((((double)j - 0.5) / rate_scale()) + 0.5) + (int)h_rrange(-2, 2);
  } else rate = (int)(28000 + h_rmod(28001));
  if (edge && h_rpct(6)) rate = h_rpct(50) ? 43000000 + (int)h_rmod(1000) : (int)h_rnd();
  g_par.rateForce = rate;
  e16 = m_i16((LD)rate_scale() * (LD)rate + (LD)0.5f);
  n = edge ? (h_rpct(30) ? (int)h_rmod(5) : (int)h_rmod(40)) : (int)(14 + h_rmod(24));
  if (edge && h_rpct(3)) n = h_rpct(50) ? -1 - (int)h_rmod(3) : 40 + (int)h_rmod(30);
  fill_table(e->A, alaw, edge ? (int)h_rmod(50) : 0, 1);
  fill_table(e->B, alaw, edge ? (int)h_rmod(50) : 0, 1);
  /* make the distances larger and strictly increasing for most trials: scale by 3 + noise */
  for (ph = 0; ph < 6; ph++) for (k = 0; k < 128; k++) {
    int v = (int)e->A[ph * 128 + k];
    if (!edge || h_rpct(60)) { v = v * (2 + ph) / 3 + (int)k; if (v > 32000) v = 32000; }
    e->A[ph * 128 + k] = (short)v;
    v = (int)e->B[ph * 128 + k];
    if (!edge || h_rpct(60)) { v = v * (2 + ph) / 3 + (int)k + (int)h_rrange(-4, 30); if (v > 32000) v = 32000; }
    e->B[ph * 128 + k] = (short)v;
  }
  for (k = 0; k < 768 + 256; k++) e->V[k] = (unsigned char)(h_rpct(edge ? 80 : 94) ? 1 : h_rpct(70) ? 0 : (int)h_rnd());
  fill_dual_lim(e, edge ? 40 : 25, edge ? 22 : 20, 0);
  for (ph = 0; ph < 8; ph++) {
    e->cur[ph] = (unsigned char)(h_rpct(edge ? 75 : 92) ? h_rrange(40, 112) : h_rpct(15) ? 0 : h_rpct(60) ? h_rrange(0x70, 0x7e) : h_rpct(70) ? (int)h_rmod(0x80) : (int)h_rmod(256));
    if (!edge && e->cur[ph] > 112) e->cur[ph] = 100;
    if (e->cur[ph] > 0x90) e->cur[ph] = (unsigned char)(e->cur[ph] & 0x7f);
  }
  for (k = 0; k < sizeof(MapPar90); k += 4) { uint32_t w = h_rnd(); memcpy((uint8_t *)e->mp + k, &w, 4); }
  e->mp->w620_ = (uint32_t)(n + 6 - e16);
  pick_hook(c, edge ? 12 : 8);
  if (edge && h_rpct(35)) { c->hook_text = "V90ConstellationDesigner: reached max"; c->hook_kind = h_rpct(60) ? HK_MP : HK_PAR; c->hook_k = 1; }
  if (edge && h_rpct(10)) {   /* every phase dual or fixed and a large n: s = 128 and nu <= 127 */
    for (ph = 0; ph < 6; ph++) { int d = h_rpct(50); e->dual[ph] = (short)d; e->lim[ph] = (unsigned char)!d; }
    n = 36 + (int)h_rmod(12);
    e->mp->w620_ = (uint32_t)(n + 6 - e16);
  }
}

/* ------------------------------------------------------------------------------------------------ scenarios */
static void sc_scn(int scan_heavy, unsigned ntrials) {
  unsigned trial;
  h_scenario_begin(1);
  relax_reserved();
  cov_define_trials();
  cov_define_range(0, CV_FR0);
  h_covreq[cov_id(CV_HOOK_PAR)] = h_covreq[cov_id(CV_HOOK_MP)] = 1;
  h_covreq[C_SKIPPED] = 0; h_covreq[C_CHAIN] = 0;
  for (trial = 0; trial < ntrials; trial++) {
    Env e;
    Call c;
    env_new(&e);
    gen_scn(&e, &c, scan_heavy);
    if (do_call(&c, &e, 1, 0)) { /* second call on the same object (state is now 1: KeepRate, or the first-call path again) */ }
    h_trial_end("scn");
  }
}
static void sc_scn_chain(unsigned nchains) {
  unsigned trial;
  h_scenario_begin(1);
  relax_reserved();
  cov_define_trials();
  cov_define_range(0, CV_FR0);
  h_covreq[C_SKIPPED] = 0;
  h_covreq[CV_ADJ53_LO] = h_covreq[CV_ADJ53_HI] = 0;   /* the window edges are steered in scenarios 0 / 1 */
  for (trial = 0; trial < nchains; trial++) {
    Env e;
    Call c;
    unsigned ncalls = 2 + h_rmod(5), k;
    env_new(&e);
    gen_scn(&e, &c, 0);
    for (k = 0; k < ncalls; k++) {
      int alaw;
      if (!do_call(&c, &e, 1, k > 0)) break;
      /* next call: new noise, the object keeps its state; sometimes a new state / mode */
      c.a = f_of(noise_bits(0));
      c.level = pick_level();
      pick_hook(&c, 12);
      if (h_rpct(35)) g_t.state_ = (uint32_t)(h_rmod(5));
      if (h_rpct(25)) g_par.useRestricedDmin = (int32_t)h_rpct(50);
      if (h_rpct(25)) g_par.forcedDmin = h_rpct(50) ? -1 : (int32_t)h_rrange(0, 300);
      if (h_rpct(30)) g_t.rrnUpDmin_ = (uint16_t)h_rrange(-3, 300);
      if (h_rpct(30)) g_t.rrnDownDmin_ = (uint16_t)h_rrange(-3, 300);
      if (h_rpct(25)) g_t.w24_ = h_rnd();
      alaw = h_rpct(50);
      if (h_rpct(30)) { g_t.useAlaw_ = (uint32_t)alaw; g_t.w28_ = h_rpct(60) ? g_t.useAlaw_ : (uint32_t)!alaw; }
      if (h_rpct(40)) { fill_table(e.A, alaw, 20, 1); fill_table(e.B, alaw, 20, 1); }
      if (h_rpct(30)) fill_dual_lim(&e, 40, 0, 1);
    }
    h_trial_end("scn_chain");
  }
}
static void sc_fr(int edge, unsigned ntrials) {
  unsigned trial, done = 0;
  h_scenario_begin(1);
  relax_reserved();
  cov_define_trials();
  cov_define_range(CV_FR0, CF_LAST);
  h_covreq[C_SKIPPED] = 0; h_covreq[C_CHAIN] = 0;
  h_covreq[C_ALT_HOOK_PAR] = 1; h_covreq[C_ALT_HOOK_MP] = 1; h_covreq[C_HOOK_LEVEL] = 1;
  if (!edge) { h_covreq[cov_id(CF_N0)] = h_covreq[cov_id(CF_N1)] = 0; h_covreq[cov_id(CF_GROW_MAX)] = h_covreq[cov_id(CF_HOOK_MP_MAX)] = h_covreq[cov_id(CF_HOOK_PAR_MAX)] = 0; h_covreq[cov_id(CF_BL0)] = 0; h_covreq[cov_id(CF_PHD_NAN)] = 0;
              h_covreq[cov_id(CF_NSKIP_HANG)] = 0; h_covreq[cov_id(CF_S128)] = 0; h_covreq[cov_id(CF_PHD_BIG)] = 0; }
  else { h_covreq[cov_id(CF_NSKIP_HANG)] = 0; h_covreq[cov_id(CF_BL0)] = 0; }
  for (trial = 0; trial < ntrials; trial++) {
    Env e;
    Call c;
    env_new(&e);
    gen_fr(&e, &c, edge);
    done += (unsigned)do_call(&c, &e, 1, 0);
    h_trial_end("fr");
  }
  if (getenv("P24_HANG")) fprintf(stderr, "why: cur>7e %u n<0 %u adjust %u nu>120 %u bl0 %u wrap %u nogrow %u\n", g_why[0], g_why[1], g_why[2], g_why[3], g_why[4], g_why[5], g_why[6]);
  if (done < ntrials * 2 / 5) h_die3("only %u of %u forceRate trials terminate in the model", done, ntrials);
}
static void sc_fr_chain(unsigned nchains) {
  unsigned trial, done = 0;
  h_scenario_begin(1);
  relax_reserved();
  cov_define_trials();
  cov_define_range(CV_FR0, CF_LAST);
  h_covreq[C_SKIPPED] = 0;
  h_covreq[cov_id(CF_BL0)] = 0; h_covreq[cov_id(CF_NSKIP_HANG)] = 0; h_covreq[cov_id(CF_N0)] = h_covreq[cov_id(CF_N1)] = 0; h_covreq[cov_id(CF_S128)] = 0; h_covreq[cov_id(CF_PHD_NAN)] = 0; h_covreq[cov_id(CF_BL0)] = 0;
  h_covreq[cov_id(CF_GROW_MAX)] = 0; h_covreq[cov_id(CF_HOOK_MP_MAX)] = h_covreq[cov_id(CF_HOOK_PAR_MAX)] = 0;
  for (trial = 0; trial < nchains; trial++) {
    Env e;
    Call c;
    unsigned ncalls = 2 + h_rmod(3), k;
    env_new(&e);
    gen_fr(&e, &c, h_rpct(30));
    for (k = 0; k < ncalls; k++) {
      if (!do_call(&c, &e, 1, k > 0)) break;
      done++;
      /* the next call sees the caller's cur[] as the first call left it; new noise, a new rate (n) and level */
      c.a = f_of(noise_bits(0));
      c.level = pick_level();
      pick_hook(&c, 8);
      if (h_rpct(60)) {
        int rate = (int)(28000 + h_rmod(28001)), e16 = m_i16((LD)rate_scale() * (LD)rate + (LD)0.5f);
        g_par.rateForce = rate;
        e.mp->w620_ = (uint32_t)((int)(14 + h_rmod(30)) + 6 - e16);
      }
      if (h_rpct(30)) fill_dual_lim(&e, 30, 20, 0);
    }
    h_trial_end("fr_chain");
  }
  if (done == 0) h_die3("no forceRate chain call terminates in the model");
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P24_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P24_DBG")) g_show = 1;
  p07_layout_check();
  p11_layout_check();
  p24_layout_check();
  h_x87_check();
  h_arena_init();
  switch (sc) {
    case 0: sc_scn(0, 3000); break;
    case 1: sc_scn(1, 4000); break;
    case 2: sc_scn_chain(400); break;
    case 3: sc_fr(0, 2500); break;
    case 4: sc_fr(1, 4000); break;
    case 5: sc_fr_chain(500); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  if (getenv("P24_HANG")) fprintf(stderr, "why: cur>7e %u n<0 %u adjust %u nu>120 %u bl0 %u wrap %u nogrow %u\n", g_why[0], g_why[1], g_why[2], g_why[3], g_why[4], g_why[5], g_why[6]);
  h_emit(1, "calls: malloc %lu free %lu edprintf %u dbgprintf %u skipped %u", h_nmalloc, h_nfree, g_nlog_total_e, g_nlog_total_d, g_nskipped);
  h_cov_finish();
  return 0;
}
