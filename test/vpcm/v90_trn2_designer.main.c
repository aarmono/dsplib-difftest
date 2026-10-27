/*
 * Differential test harness for port chunk P26 (src/vpcm/v90_trn2_designer.cpp): class V90TRN2Designer - V90TRN2Design (3,767 B), setNofUcodesInTrn2, the dead maxK and setTrn2DummyConstel,
 * C1 / C2, D1 / D2 (8 functions).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_trn2_designer.sh against the blob (dsplibs.o with edprintf and the two callees of V90TRN2Design -
 * V90ConstellationPower::getPower / getPowerIndexForPower (P06) - WEAK so that the recorders below win), against the port (-O0 / -O2; the P06 port object supplies the G.711 coders and, WEAKENED,
 * the two callees) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_trn2_designer.blob32.out).  The functions are reached through __asm__ labels; the designer object (8 B,
 * two pointers), the V90Parameters block (P07 mirror) and the mapping blocks (P11 mirror, 0x650 B) are the generated C mirrors; the pointer-holding objects are static.
 *
 * Scenarios (each its own process):
 *   0 V90TRN2Design nominal (the REAL function against the independent model m_design; blob = port): realistic tables, both codecs, flags 0 / 1, N = 1 .. 40, levels, hooks
 *   1 V90TRN2Design edge regimes (wild tables incl. random shorts, N <= 0 / 128 .. 140, cur 0 .. 255, garbage argument bits, odd special / pcm values, unk360 around the ucodes)
 *   2 V90TRN2Design forced design / failures (flags mostly nonzero, thin tables, the dummy constellation and the failure print)
 *   3 ctor / dtor / setNofUcodesInTrn2 / maxK / setTrn2DummyConstel
 *
 * What is compared: the whole mapping block, the parameter blocks, the return value, the object (pointers), the canaries, every edprintf / dsplibs_debug_printf call (kind, text with the integer
 * arguments formatted, a snapshot of the mapping block at the moment of the call), every recorded callee call (getPower: object, block, point, pcm; getPowerIndexForPower: object, the float
 * bits) and the unsigned `level > 1` tests (levels 0, 1, 2, 3, >= 2^31; a hook lowers the level inside the k-th print; another hook swaps T->params_ inside the k-th print: every re-read of the
 * blob is a re-read here).
 *
 * INDEPENDENT MODEL m_design (specification style, long double arithmetic, the pops / reloads of the x87 sequences written from the disassembly notes, not from the port): the whole function.  A model
 * that says the blob would not terminate (the cur lowering / the ucode search wrapping around the byte range) EXCLUDES the input (counted as `skipped`; a watchdog alarm catches a wrong guess).
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
#include "v90_trn2_designer_layout.h"

#define L(m) __asm__(m)

/* the functions under test */
extern void tr_c1(void *, void *, void *) L("_ZN15V90TRN2DesignerC1EP13V90ParametersP21V90ConstellationPower");
extern void tr_c2(void *, void *, void *) L("_ZN15V90TRN2DesignerC2EP13V90ParametersP21V90ConstellationPower");
extern void tr_d1(void *) L("_ZN15V90TRN2DesignerD1Ev");
extern void tr_d2(void *) L("_ZN15V90TRN2DesignerD2Ev");
extern void tr_setn(void *, int) L("_ZN15V90TRN2Designer18setNofUcodesInTrn2Es");
extern unsigned tr_maxk(void *, void *) L("_ZN15V90TRN2Designer4maxKEP16V90MappingParams");
extern void tr_dummy(void *, void *) L("_ZN15V90TRN2Designer19setTrn2DummyConstelEP16V90MappingParams");
extern int tr_design(void *, void *, void *, void *, void *, void *, int, int, int, void *, unsigned, unsigned, int)
    L("_ZN15V90TRN2Designer13V90TRN2DesignEP16V90MappingParamsPA128_sS3_PA128_hPs7PcmTypeS7_sPhjh28V90SpecialSpectralConditions");
extern int linear2alaw(int);
extern int linear2ulaw(int);
extern int alaw2linear(int);
extern int ulaw2linear(int);
extern unsigned int dsplibs_debug_level;
/* the interposed callees of V90TRN2Design (P06): defined below */
extern float h_getpower(void *, void *, int, int) L("_ZN21V90ConstellationPower8getPowerEP16V90MappingParams26V90TxPowerMeasurementPoint7PcmType");
extern unsigned h_getidx(void *, float) L("_ZN21V90ConstellationPower21getPowerIndexForPowerEf");
extern unsigned cp_limits[35] L("_ZN21V90ConstellationPower18averagePowerLimitsE");

typedef long double LD;
typedef V90ParObj ParObj;

static const HFld f_mp[] = {
  HFLD(MapPar90, w0_, 'u'), HFLDN(MapPar90, listA_, 'x', 768), HFLDN(MapPar90, listB_, 'x', 768), HFLDN(MapPar90, cnt_, 'u', 6), HFLD(MapPar90, flag_, 'u'), HFLD(MapPar90, w620_, 'u'),
  HFLD(MapPar90, w624_, 'u'), HFLD(MapPar90, f628_, 'f'), HFLD(MapPar90, f62c_, 'f'), HFLD(MapPar90, f630_, 'f'), HFLD(MapPar90, f634_, 'f'), HFLDN(MapPar90, idx_, 'u', 6),
};
#define NF_mp ((int)(sizeof f_mp / sizeof f_mp[0]))

/* ------------------------------------------------------------------------------------------------ recorders */
#define LOGMAX 420
#define TXTMAX 520
typedef struct { char kind; char text[TXTMAX]; } LogEnt;
static LogEnt g_log[LOGMAX];
static unsigned g_nlog, g_nlog_total_e, g_nlog_total_d, g_nlog_total_c;
static int g_show;
static TrnObj *g_snap_t;
static MapPar90 *g_snap_mp;
enum { HK_NONE = 0, HK_LEVEL, HK_PAR };
static int g_hook_kind;
static unsigned g_hook_k, g_hook_level;   /* HK_LEVEL: level after the k-th dsplibs_debug_printf; HK_PAR: swap after the k-th print of any kind */
static unsigned g_nprints_in_call, g_nd_in_call;
static ParObj *g_alt_par;
static unsigned char g_pow[64];            /* the power object (never dereferenced) */
static float g_script_p;                   /* getPower returns this */
static unsigned g_script_idx;              /* getPowerIndexForPower returns this */
static MapPar90 *g_expect_mp;
static int g_expect_pcm;

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
static void snapshot(int rowprint) {
  if (g_snap_t) {
    TrnObj tmp;
    memcpy(&tmp, g_snap_t, sizeof tmp);
    tmp.params_ = NULL; tmp.power_ = NULL;
    h_dump_obj(0, "snap", &tmp, f_TrnObj, NF_TrnObj);
  }
  if (g_snap_mp && !rowprint) h_dump_obj(0, "snapm", g_snap_mp, f_mp, NF_mp);
}
static void record(char kind, const char *fmt, va_list ap) {
  char buf[TXTMAX * 2];
  vsnprintf(buf, sizeof buf, fmt, ap);
  if (g_nlog < LOGMAX) { g_log[g_nlog].kind = kind; g_log[g_nlog].text[0] = 0; esc_cat(g_log[g_nlog].text, sizeof g_log[g_nlog].text, buf); }
  h_emit(g_show, "%cprintf %s", kind, g_nlog < LOGMAX ? g_log[g_nlog].text : "(many)");
  g_nlog++;
  snapshot(!strncmp(buf, "trn2 constel[", 13));
}
static void do_hook(char kind) {
  g_nprints_in_call++;
  if (kind == 'd') g_nd_in_call++;
  if (!g_hook_k) return;
  if (g_hook_kind == HK_LEVEL && kind == 'd' && g_nd_in_call == g_hook_k) dsplibs_debug_level = g_hook_level;
  if (g_hook_kind == HK_PAR && g_nprints_in_call == g_hook_k && g_snap_t) g_snap_t->params_ = g_alt_par;
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
static void rec_c(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  record('c', fmt, ap);
  va_end(ap);
  g_nlog_total_c++;
}
static void log_begin(void) { g_nlog = 0; g_nd_in_call = 0; g_nprints_in_call = 0; }
/* the callees: the power of the block and its index in the table, scripted per trial */
float h_getpower(void *pw, void *mp, int point, int pcm) {
  uint32_t b;
  if (pw != (void *)g_pow) h_die3("getPower: object %p", pw);
  if (mp != (void *)g_expect_mp) h_die3("getPower: block %p", mp);
  rec_c("getPower point=%d pcm=%d", point, pcm);
  if (point != 1) h_die3("getPower: point %d", point);
  if (pcm != g_expect_pcm) h_die3("getPower: pcm %d expected %d", pcm, g_expect_pcm);
  memcpy(&b, &g_script_p, 4);
  (void)b;
  return g_script_p;
}
unsigned h_getidx(void *pw, float p) {
  uint32_t b;
  if (pw != (void *)g_pow) h_die3("getPowerIndexForPower: object %p", pw);
  memcpy(&b, &p, 4);
  rec_c("getPowerIndexForPower bits=%08x", b);
  return g_script_idx;
}

/* ------------------------------------------------------------------------------------------------ small helpers */
#define EXPECT_EQ(what, got, want)                                                                                                   \
  do {                                                                                                                              \
    unsigned long long g_ = (unsigned long long)(got), w_ = (unsigned long long)(want);                                              \
    if (g_ != w_) h_die3("%s: got 0x%llx expected 0x%llx (line %d)", (what), g_, w_, __LINE__);                                     \
  } while (0)
static void relax_reserved(void) { h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0; }
static unsigned pick_level(void) {
  static const unsigned lv[] = { 0, 1, 2, 3, 7, 0x80000000u, 0xffffffffu, 0x80000001u };
  return h_rpct(70) ? 2u : lv[h_rmod(sizeof lv / sizeof lv[0])];
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
static LD m_sqrt(LD x) { LD r = x; __asm__("fsqrt" : "+t"(r)); return r; }
static LD m_prod(const MapPar90 *mp) {
  LD p = (LD)mp->cnt_[0];
  int k;
  for (k = 1; k < 6; k++) p = p * (LD)mp->cnt_[k];
  return p;
}
static unsigned m_trunc64lo(LD v) {
  if (!(v > -9223372036854775809.0L && v < 9223372036854775808.0L)) return 0;
  return (unsigned)(long long)v;
}
static int m_i16(LD v) {   /* fistp WORD with truncation: 0x8000 for NaN / out of range */
  if (!(v > -32769.0L && v < 32768.0L)) return -32768;
  return (int)(long long)v;
}
static int m_i32(LD v) {   /* fistp DWORD with truncation: 0x80000000 for NaN / out of range */
  if (!(v > -2147483649.0L && v < 2147483648.0L)) return INT_MIN;
  return (int)(long long)v;
}
static unsigned m_kint(const MapPar90 *mp) {
  LD p = m_prod(mp), lg;
  float den;
  if (p == 0.0L) return 0;
  lg = m_log10(p);
  den = RF(m_log10(2.0L));
  return m_trunc64lo(lg / (LD)den + (LD)F(0x358637bdu));
}
static unsigned char *m_la(MapPar90 *mp, unsigned p, unsigned i) { return (unsigned char *)mp + 4u + p * 0x80u + i; }
static unsigned char *m_lb(MapPar90 *mp, unsigned p, unsigned i) { return (unsigned char *)mp + 0x304u + p * 0x80u + i; }
static int m_absi(int v) { return v < 0 ? (int)(0u - (unsigned)v) : v; }
static int m_g711(int alaw, int v) {   /* the G.711 code of |v|, as the blob forms it */
  int a = m_absi(v);
  return alaw ? ((linear2alaw(a) ^ 0xd5) & 0xff) : ((~linear2ulaw(a)) & 0xff);
}

/* ------------------------------------------------------------------------------------------------ the objects of a trial */
typedef struct { MapPar90 m; uint8_t canary[32]; } MpBox;
static TrnObj g_t;
static ParObj g_par, g_altpar;
static MpBox g_mpb;
#define TPAD 160
#define TLEN (768 + 256)
static short g_Apad[TPAD + TLEN + TPAD], g_Bpad[TPAD + TLEN + TPAD];
static unsigned char g_Vpad[TPAD + TLEN + TPAD];
static short *const g_A = g_Apad + TPAD;
static short *const g_B = g_Bpad + TPAD;
static unsigned char *const g_V = g_Vpad + TPAD;
static short g_flags[8];
static unsigned char g_cur[8];
static unsigned long g_dumpctr;
static unsigned g_nskipped, g_ntrials;

/* the model state */
#define MLOGMAX LOGMAX
typedef struct {
  MapPar90 mpm;                   /* the model's mapping block */
  ParObj parm, altparm;           /* model copies of the real parameter blocks */
  const ParObj *par;              /* T->params_ (switched by a PAR hook) */
  unsigned level;
  int hook_kind;
  unsigned hook_k, hook_level, nprint, nd;
  LogEnt log[MLOGMAX];
  unsigned nlog;
  int hang;
  int ret;
  unsigned char ab[6];            /* the lowered start ucodes (for the steering of params->unk360) */
  unsigned char cov[128];
} MS;
typedef struct {
  int pcmA, pcm, special;
  unsigned limit;
  unsigned pidx;
  uint32_t pbits;                 /* getPower's value */
  unsigned pidxret;               /* getPowerIndexForPower's value */
  unsigned char cur[6];
  short flags[6];
} DA;
static void m_print(MS *s, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (s->nlog < MLOGMAX) { s->log[s->nlog].kind = kind; s->log[s->nlog].text[0] = 0; esc_cat(s->log[s->nlog].text, sizeof s->log[s->nlog].text, buf); }
  s->nlog++;
  if (kind == 'c') return;    /* a recorded callee call is not a print: it does not count for the hooks */
  s->nprint++;
  if (kind == 'd') s->nd++;
  if (s->hook_k) {
    if (s->hook_kind == HK_LEVEL && kind == 'd' && s->nd == s->hook_k) s->level = s->hook_level;
    if (s->hook_kind == HK_PAR && s->nprint == s->hook_k) s->par = &s->altparm;
  }
}
#define M_E(s, ...) m_print(s, 'e', __VA_ARGS__)
#define M_D(s, ...) m_print(s, 'd', __VA_ARGS__)
#define M_C(s, ...) m_print(s, 'c', __VA_ARGS__)
#define COV(s, id) ((s)->cov[id] = 1)

enum { MC_PBX, MC_STD, MC_K_ZERO, MC_ALAW, MC_ULAW, MC_CUR_LOWERED, MC_MAXU_RAISED, MC_DMIN_PH, MC_DMIN_SEEK, MC_DMIN_FAIL, MC_DMIN_CUR_END, MC_DMIN_N0, MC_FORCED_PH, MC_FORCED_ACCEPT,
       MC_FORCED_MULTI, MC_FORCED_TCAP, MC_FORCED_FAIL, MC_FORCED_N0, MC_FORCED_SKIP, MC_FORCED_ZERO, MC_FORCED_LIM, MC_FORCED_VALID, MC_FORCED_HALF_A, MC_FORCED_HALF_B, MC_FORCED_FULL,
       MC_FAIL_PATH, MC_FAIL_PRINT, MC_SAME_CODE, MC_SAME_COPY, MC_DIFF_CODEC, MC_P_NEG, MC_P_NAN, MC_P_ZERO, MC_P_POS, MC_REPORT, MC_CHAIN_CUT, MC_ROWS_CUT, MC_ROWS_PRINT, MC_PAR_SWAP,
       MC_OK, MC_N_NEG, MC_EDGE_ACCEPT, MC_EDGE_NOACCEPT, MC_LAST };
static const char *mcov_names[MC_LAST] = {
  [MC_PBX] = "design: special == 2 (PBX shaper set)", [MC_STD] = "design: other special (standard shaper set)", [MC_K_ZERO] = "design: a zero count (K = 0)", [MC_ALAW] = "design: A-law",
  [MC_ULAW] = "design: u-law", [MC_CUR_LOWERED] = "design: a start ucode lowered", [MC_MAXU_RAISED] = "design: max ucode raised to params->unk74", [MC_DMIN_PH] = "design: a dMin phase",
  [MC_DMIN_SEEK] = "dMin: the seek for A >= half ran", [MC_DMIN_FAIL] = "dMin: failure (ran out of ucodes)", [MC_DMIN_CUR_END] = "dMin: last slot filled at the start ucode",
  [MC_DMIN_N0] = "dMin: no slot (N <= 0)", [MC_FORCED_PH] = "design: a forced phase", [MC_FORCED_ACCEPT] = "forced: a ucode accepted", [MC_FORCED_MULTI] = "forced: several rounds",
  [MC_FORCED_TCAP] = "forced: stopped by the 200-round cap", [MC_FORCED_FAIL] = "forced: failure (count != N)", [MC_FORCED_N0] = "forced: N <= 0", [MC_FORCED_SKIP] = "forced: scan skipped (unk360 > cur - 1)",
  [MC_FORCED_ZERO] = "forced: a zero table word ends the scan", [MC_FORCED_LIM] = "forced: a ucode above the limit", [MC_FORCED_VALID] = "forced: a ucode not valid", [MC_FORCED_HALF_A] = "forced: A below half ends the scan",
  [MC_FORCED_HALF_B] = "forced: B below half ends the scan", [MC_FORCED_FULL] = "forced: N ucodes found", [MC_FAIL_PATH] = "design: the dummy constellation", [MC_FAIL_PRINT] = "design: failure print",
  [MC_SAME_CODE] = "second list: same codec, code below the ucode", [MC_SAME_COPY] = "second list: same codec, ucode kept", [MC_DIFF_CODEC] = "second list: different codecs",
  [MC_P_NEG] = "power: sign '-'", [MC_P_NAN] = "power: NaN", [MC_P_ZERO] = "power: zero", [MC_P_POS] = "power: positive", [MC_REPORT] = "report: header chain complete",
  [MC_CHAIN_CUT] = "report: header chain cut by the level", [MC_ROWS_CUT] = "report: rows stop (level lowered)", [MC_ROWS_PRINT] = "report: rows printed", [MC_PAR_SWAP] = "params_ swapped by a hook",
  [MC_OK] = "design: success", [MC_N_NEG] = "design: N < 0", [MC_EDGE_ACCEPT] = "forced: the scan at idx == unk360 accepts", [MC_EDGE_NOACCEPT] = "forced: the scan at idx == unk360 does not accept",
};

/* split of a value into the pieces of "%c%d.%0nd" (the model's own derivation) */
static void m_split(LD v, LD scale, LD sv, int *sign, int *ip, int *fp) {
  int t = m_i32(v);
  LD a = v < 0 ? -v : v;
  if (v != v) a = v;    /* fabs of a NaN stays a NaN: the conversion gives the indefinite */
  *ip = m_i32(a);
  *fp = m_absi(m_i32((v - (LD)t) * scale));
  *sign = sv <= 0.0L ? '-' : '+';
}

/* ------------------------------------------------------------------------------------------------ model: V90TRN2Design */
static void m_design(MS *s, const DA *a, const short *A, const short *B, const unsigned char *V) {
  MapPar90 *mp = &s->mpm;
  const ParObj *P = s->par;
  const int alaw = a->pcm != 0;
  unsigned char ab[6], maxU = 0;
  unsigned ph, k;
  uint32_t sr, id;
  int fail = 0, ph_fail = 0;
  unsigned long guard = 0;
  unsigned wrap;

  mp->flag_ = 1;
  if (P->unk78 < 0) COV(s, MC_N_NEG);
  for (k = 0; k < 6; k++) mp->cnt_[k] = (uint32_t)P->unk78;
  if (a->special == 2) {
    COV(s, MC_PBX);
    id = (uint32_t)P->germanPbxSpectralShaperId; sr = (uint32_t)P->germanPbxSpectralShaperSr;
    memcpy(&mp->f628_, &P->germanPbxSpectralShaperA1, 4); memcpy(&mp->f62c_, &P->germanPbxSpectralShaperA2, 4);
    memcpy(&mp->f630_, &P->germanPbxSpectralShaperB1, 4); memcpy(&mp->f634_, &P->germanPbxSpectralShaperB2, 4);
  } else {
    COV(s, MC_STD);
    id = (uint32_t)P->spectralShaperId; sr = (uint32_t)P->spectralShaperSr;
    memcpy(&mp->f628_, &P->spectralShaperA1, 4); memcpy(&mp->f62c_, &P->spectralShaperA2, 4);
    memcpy(&mp->f630_, &P->spectralShaperB1, 4); memcpy(&mp->f634_, &P->spectralShaperB2, 4);
  }
  mp->w620_ = sr;
  mp->w624_ = id > a->limit ? a->limit : id;
  if (m_prod(mp) == 0.0L) COV(s, MC_K_ZERO);
  mp->w0_ = m_kint(mp) - sr + 6u;
  for (k = 0; k < 6; k++) mp->idx_[k] = 0;
  COV(s, alaw ? MC_ALAW : MC_ULAW);
  {
    unsigned lim = cp_limits[a->pidx + 4];
    unsigned lv = m_trunc64lo(m_sqrt((LD)lim));
    int level = m_i32((LD)lv * (LD)F(0x3fd043fau));
    maxU = (unsigned char)(m_g711(alaw, level) - 2);
    M_E(s, "V90TRN2Design: max TX index = %d, givin power limit = %d, implying max level =%d\r\n", (int)a->pidx, (int)lv, level);
  }
  P = s->par;
  M_E(s, "V90TRN2Design: ...hence max ucode (after factor) = %d, by params maxUcode = %d\r\n", (int)maxU, P->unk74);
  P = s->par;
  if (maxU < (unsigned char)P->unk74) { maxU = (unsigned char)P->unk74; COV(s, MC_MAXU_RAISED); }
  for (ph = 0; ph < 6; ph++) {
    unsigned char c = a->cur[ph];
    wrap = 0;
    while (m_g711(alaw, A[ph * 128 + c]) > (int)maxU) {
      c--;
      COV(s, MC_CUR_LOWERED);
      if (++wrap > 600) { s->hang = 1; return; }
    }
    ab[ph] = c;
    s->ab[ph] = c;
  }
  P = s->par;
  for (ph = 0; ph < 6 && !fail; ph++) {
    const short *At = A + ph * 128, *Bt = B + ph * 128;
    unsigned char cur = ab[ph];
    if (a->flags[ph] == 0) {
      int dmin, half, j;
      unsigned char cl;
      COV(s, MC_DMIN_PH);
      dmin = m_i16(((LD)At[cur] * (LD)F(0x3f59999au)) / ((LD)P->unk78 - (LD)F(0x3f000000u)));
      M_E(s, "Trn2 Design: phase %d, dMin = %d...\r\n", (int)ph, dmin);
      P = s->par;
      half = dmin / 2;
      cl = (unsigned char)P->unk360;
      j = (short)(P->unk78 - 1);
      if (j < 0) { COV(s, MC_DMIN_N0); continue; }
      for (;;) {
        if (At[cl] < (short)half) {
          COV(s, MC_DMIN_SEEK);
          wrap = 0;
          while (cur >= cl) {
            cl++;
            if (!(At[cl] < (short)half)) break;
            if (++wrap > 600) { s->hang = 1; return; }
          }
        }
        *m_la(mp, ph, (unsigned)j) = cl;
        half = (short)(dmin + (unsigned short)At[cl]);
        j = (short)(j - 1);
        if (cur <= cl) {
          P = s->par;
          if (j >= 0) { COV(s, MC_DMIN_FAIL); fail = 1; ph_fail = 1; } else COV(s, MC_DMIN_CUR_END);
          break;
        }
        if (j < 0) { P = s->par; break; }
        if (++guard > 3000000ul) { s->hang = 1; return; }
      }
    } else {
      int rr, cnt = 0, rounds = 0, N = P->unk78;
      COV(s, MC_FORCED_PH);
      rr = m_i16((LD)At[cur] * ((LD)1.0L / ((LD)N - (LD)F(0x3f000000u))));
      if (N <= 0) COV(s, MC_FORCED_N0);
      if (N > 0) {
        for (;;) {
          int lim = (short)(((int)Bt[cur] - rr) <= ((int)At[cur] - rr) ? ((int)Bt[cur] - rr) : ((int)At[cur] - rr));
          int idx, ha, hb, mm, more;
          short a1, b1;
          *m_la(mp, ph, 0) = cur;
          cnt = 1;
          P = s->par;
          idx = (int)cur - 1;
          if (P->unk360 > idx) {
            COV(s, MC_FORCED_SKIP);
            a1 = At[idx]; b1 = Bt[idx];
          } else {
            short half2 = (short)(rr / 2);
            int edge = (P->unk360 == idx);
            for (;;) {
              a1 = At[idx];
              b1 = Bt[idx];
              if (edge && idx == (int)cur - 1 && idx == P->unk360) COV(s, MC_EDGE_NOACCEPT);
              if (a1 < half2) { COV(s, MC_FORCED_HALF_A); break; }
              if (b1 < half2) { COV(s, MC_FORCED_HALF_B); break; }
              if (a1 == 0 || b1 == 0) { COV(s, MC_FORCED_ZERO); break; }
              if (a1 > lim || b1 > lim) COV(s, MC_FORCED_LIM);
              else if (V[(int)ph * 128 + idx] == 0) COV(s, MC_FORCED_VALID);
              else {
                int l2, h2;
                COV(s, MC_FORCED_ACCEPT);
                if (edge && idx == (int)cur - 1) COV(s, MC_EDGE_ACCEPT);
                *m_la(mp, ph, (unsigned)cnt) = (unsigned char)idx;
                cnt = (short)(cnt + 1);
                l2 = (int)b1 - rr; h2 = (int)a1 - rr;
                lim = (short)(l2 <= h2 ? l2 : h2);
                P = s->par;
                if (cnt == P->unk78) { COV(s, MC_FORCED_FULL); break; }
              }
              idx--;
              if (!(P->unk360 <= idx)) { a1 = At[idx]; b1 = Bt[idx]; break; }
              if (++guard > 3000000ul) { s->hang = 1; return; }
            }
          }
          rounds++;
          mm = b1 <= a1 ? b1 : a1;
          ha = P->unk78;
          more = cnt < ha && (short)rounds <= 199;
          rr = m_i16((LD)rr - (((LD)rr * (LD)F(0x3f000000u) - (LD)mm) * ((LD)1.0L / ((LD)ha + (LD)F(0x3f000000u)))));
          if (!more) { if ((short)rounds > 199) COV(s, MC_FORCED_TCAP); break; }
          if (++guard > 3000000ul) { s->hang = 1; return; }
        }
        if (rounds > 1) COV(s, MC_FORCED_MULTI);
      }
      if (cnt != P->unk78 && (short)cnt >= 0) { COV(s, MC_FORCED_FAIL); fail = 1; ph_fail = 1; }
    }
  }
  if (fail) {
    unsigned i, o;
    int n;
    COV(s, MC_FAIL_PATH);
    (void)ph_fail;
    for (k = 0; k < 6; k++) {
      unsigned char c = 0x4e;
      n = 0;
      o = 0;
      while (n < P->unk78) {
        *m_la(mp, k, o) = c;
        *m_lb(mp, k, o) = c;
        n = (short)(n + 1); c--; P = s->par; o++;
      }
    }
    (void)i;
    if (s->level > 1) { COV(s, MC_FAIL_PRINT); M_D(s, "V90TRN2Designer::V90TRN2Design()  constelation design failed !!!\r\n"); }
    s->ret = 0;
    return;
  }
  /* second list */
  for (k = 0; k < 6; k++) {
    int n = P->unk78, i = 0;
    if (n <= 0) continue;
    while (i < n) {
      unsigned u = *m_la(mp, k, (unsigned)i);
      int code = m_g711(alaw, A[k * 128 + u]);
      if (a->pcmA == a->pcm) {
        if (code < (int)u) { COV(s, MC_SAME_CODE); *m_lb(mp, k, (unsigned)i) = (unsigned char)code; }
        else { COV(s, MC_SAME_COPY); *m_lb(mp, k, (unsigned)i) = (unsigned char)u; }
      } else { COV(s, MC_DIFF_CODEC); *m_lb(mp, k, (unsigned)i) = (unsigned char)code; }
      i++;
      P = s->par;
      n = P->unk78;
    }
  }
  /* power */
  {
    float Pw = F(a->pbits);
    int sg, ip, fp;
    LD sq;
    M_C(s, "getPower point=1 pcm=%d", a->pcm);
    M_C(s, "getPowerIndexForPower bits=%08x", (unsigned)a->pbits);
    M_E(s, "--------------------------------------\r\n");
    M_E(s, "V90TRN2Design:\r\n");
    sq = m_sqrt((LD)Pw);
    m_split(sq, 1000.0L, (LD)Pw, &sg, &ip, &fp);
    M_E(s, "V90TRN2Design: sqrt(power)  = %c%d.%03d\r\n", sg, ip, fp);
    {
      LD v = (LD)(unsigned)(a->pidxret + 1u) * (LD)(-0.5f);
      m_split(v, 10.0L, v, &sg, &ip, &fp);
      M_E(s, "V90TRN2Design: index %d, power in dBm0  = %c%d.%01d\r\n", (int)a->pidxret, sg, ip, fp);
    }
    if (Pw != Pw) COV(s, MC_P_NAN); else if (Pw == 0.0f) COV(s, MC_P_ZERO); else if (Pw < 0) COV(s, MC_P_NEG); else COV(s, MC_P_POS);
  }
  /* report */
  {
    unsigned lv = s->level;
    int n, i;
    if (lv > 1) {
      M_D(s, "\n--------------------------------------\r\n"); lv = s->level;
      if (lv > 1) {
        M_D(s, "V90TRN2Designer : ADI design report : \r\n"); lv = s->level;
        if (lv > 1) {
          M_D(s, "trn2 size  :  %d\r\n", s->par->unk78); lv = s->level;
          if (lv > 1) { M_D(s, "\n------------------------------------------------------------------------------------------------------\r\n"); lv = s->level; COV(s, MC_REPORT); }
          else COV(s, MC_CHAIN_CUT);
        } else COV(s, MC_CHAIN_CUT);
      } else COV(s, MC_CHAIN_CUT);
    }
    P = s->par;
    n = P->unk78;
    i = 0;
    while (i < n) {
      if (lv > 1) {
        unsigned q;
        int v[6];
        COV(s, MC_ROWS_PRINT);
        for (q = 0; q < 6; q++) v[q] = A[q * 128 + *m_la(mp, q, (unsigned)i)];
        M_D(s, "trn2 constel[%d]  :  %d %d %d %d %d %d  :  %d %d %d %d %d %d  :  %d %d %d %d %d %d\r\n", i, (int)*m_la(mp, 0, i), (int)*m_la(mp, 1, i), (int)*m_la(mp, 2, i), (int)*m_la(mp, 3, i),
            (int)*m_la(mp, 4, i), (int)*m_la(mp, 5, i), (int)*m_lb(mp, 0, i), (int)*m_lb(mp, 1, i), (int)*m_lb(mp, 2, i), (int)*m_lb(mp, 3, i), (int)*m_lb(mp, 4, i), (int)*m_lb(mp, 5, i), v[0], v[1],
            v[2], v[3], v[4], v[5]);
        P = s->par; lv = s->level;
        if (lv <= 1) COV(s, MC_ROWS_CUT);
      }
      i++;
      n = P->unk78;
      if (lv <= 1 && i >= n) break;
    }
    if (lv > 1) M_D(s, "\n------------------------------------------------------------------------------------------------------\r\n");
  }
  if (s->par != &s->parm) COV(s, MC_PAR_SWAP);
  COV(s, MC_OK);
  s->ret = 1;
}

/* ------------------------------------------------------------------------------------------------ comparison helpers */
static void model_init(MS *s, unsigned level, int hook_kind, unsigned hook_k, unsigned hook_level) {
  memset(s, 0, sizeof *s);
  memcpy(&s->parm, &g_par, sizeof s->parm); memcpy(&s->altparm, &g_altpar, sizeof s->altparm);
  memcpy(&s->mpm, &g_mpb.m, sizeof s->mpm);
  s->par = &s->parm;
  s->level = level; s->hook_kind = hook_kind; s->hook_k = hook_k; s->hook_level = hook_level;
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

/* ------------------------------------------------------------------------------------------------ generators */
static int ucode_value(int alaw, int u) {
  int v = alaw ? alaw2linear((u ^ 0xd5) & 0xff) : ulaw2linear((~u) & 0xff);
  return v < 0 ? -v : v;
}
/* a distance table: the linear value of every ucode (increasing), jagged by `jag` percent, scaled down by `scale`, + a random slack behind (indices above 127 run on) */
static int g_tiny;
static void fill_table(short *tab, int alaw, int jag, int scale, int wild) {
  int ph, u;
  for (ph = 0; ph < 6; ph++) {
    int sc = scale + (h_rpct(25) ? (int)h_rmod(3) : 0);
    if (g_tiny) sc = 60 + (int)h_rmod(300);
    for (u = 0; u < 128; u++) {
      int v = ucode_value(alaw, u) / (sc ? sc : 1);
      if (h_rpct(jag)) v += (int)h_rrange(-40, 40);
      if (jag >= 8 && h_rpct(jag / 8)) v = (int)h_rrange(-300, 3000);
      if (wild && h_rpct(wild)) v = (int)h_rrange(-32768, 32767);
      if (g_tiny && h_rpct(6)) v = (int)h_rrange(-3, 3);
      tab[ph * 128 + u] = (short)v;
    }
  }
  for (u = 0; u < 256; u++) tab[768 + u] = (short)(h_rnd() & 0x7fff);
}
/* regime: 0 nominal, 1 edge (wild), 2 forced / thin */
static void gen_tables(int alaw, int regime) {
  int ph, u, jag = regime == 1 ? 20 : 3;
  for (u = 0; u < TPAD; u++) { g_Apad[u] = (short)h_rnd(); g_Bpad[u] = (short)h_rnd(); g_Vpad[u] = (unsigned char)h_rnd(); }
  for (u = TPAD + TLEN; u < TPAD + TLEN + TPAD; u++) { g_Apad[u] = (short)h_rnd(); g_Bpad[u] = (short)h_rnd(); g_Vpad[u] = (unsigned char)h_rnd(); }
  g_tiny = regime != 0 && h_rpct(8);
  fill_table(g_A, alaw, jag, 1, regime == 1 ? 4 : 0);
  /* B is A with a jitter (the second distance table of the ADI): mostly equal or a little above / below */
  for (ph = 0; ph < 6; ph++)
    for (u = 0; u < 128; u++) {
      int v = g_A[ph * 128 + u];
      if (h_rpct(60)) v += (int)h_rrange(-8, 40);
      if (regime == 1 ? h_rpct(3) : (h_rpct(1) && h_rpct(10))) v = (int)h_rrange(-300, 32000);
      g_B[ph * 128 + u] = (short)v;
    }
  for (u = 768; u < TLEN; u++) g_B[u] = (short)(h_rnd() & 0x7fff);
  for (u = 0; u < TLEN; u++) g_V[u] = (unsigned char)(h_rpct(regime == 2 ? 80 : 92) ? 1 : h_rpct(50) ? 0 : (int)h_rnd());
}
static void gen_par(ParObj *p, int regime, int N) {
  uint32_t w;
  memset(p, 0, sizeof *p);
  p->unk78 = N;
  p->unk80 = h_rpct(30) ? (int)h_rrange(-5, 60) : (int)h_rnd();
  p->unk74 = h_rpct(75) ? (int)h_rmod(60) : h_rpct(70) ? (int)h_rrange(60, 127) : (int)h_rnd();
  if (regime == 1 && h_rpct(30)) p->unk74 = (int)h_rrange(120, 255);
  p->unk360 = h_rpct(70) ? (int)h_rmod(24) : h_rpct(60) ? (int)h_rmod(130) : h_rpct(50) ? (int)h_rrange(-4, 0) : (int)h_rnd();
  if (regime == 0 && h_rpct(80)) p->unk360 = (int)h_rmod(12);
  if (regime == 2 && h_rpct(50)) p->unk360 = (int)h_rrange(40, 140);
  w = h_rnd(); memcpy(&p->spectralShaperA1, &w, 4); w = h_rnd(); memcpy(&p->spectralShaperA2, &w, 4);
  w = h_rnd(); memcpy(&p->spectralShaperB1, &w, 4); w = h_rnd(); memcpy(&p->spectralShaperB2, &w, 4);
  p->spectralShaperSr = h_rpct(70) ? (int)h_rmod(12) : (int)h_rnd();
  p->spectralShaperId = h_rpct(70) ? (int)h_rmod(8) : (int)h_rnd();
  w = h_rnd(); memcpy(&p->germanPbxSpectralShaperA1, &w, 4); w = h_rnd(); memcpy(&p->germanPbxSpectralShaperA2, &w, 4);
  w = h_rnd(); memcpy(&p->germanPbxSpectralShaperB1, &w, 4); w = h_rnd(); memcpy(&p->germanPbxSpectralShaperB2, &w, 4);
  p->germanPbxSpectralShaperSr = h_rpct(70) ? (int)h_rmod(12) : (int)h_rnd();
  p->germanPbxSpectralShaperId = h_rpct(70) ? (int)h_rmod(8) : (int)h_rnd();
}
static int pick_n(int regime) {
  int r = (int)h_rmod(100);
  if (regime == 1) {
    if (r < 10) return 0;
    if (r < 20) return -1 - (int)h_rmod(3);
    if (r < 40) return 126 + (int)h_rmod(15);
    return 1 + (int)h_rmod(128);
  }
  if (regime == 2) return r < 15 ? 1 + (int)h_rmod(3) : 2 + (int)h_rmod(24);
  if (r < 50) return 2 + (int)h_rmod(12);
  if (r < 85) return 2 + (int)h_rmod(32);
  return 1 + (int)h_rmod(40);
}
static uint32_t pick_pbits(void) {
  static const uint32_t sp[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7fc12345u, 0x7f800000u, 0xff800000u, 0x00000001u, 0x80000001u, 0x7f7fffffu, 0x3f800000u, 0xbf800000u, 0x4b189680u, 0x3f000000u };
  int r = (int)h_rmod(100);
  if (r < 10) return sp[h_rmod(sizeof sp / sizeof sp[0])];
  if (r < 25) return 0xbf000000u | (h_rnd() & 0x80ffffffu);   /* negative */
  if (r < 85) {   /* normal range 1e2 .. 1e9 */
    float f = (float)(100.0 + (double)(h_rnd() % 1000000u) * (double)(1 + (h_rnd() % 1000u)));
    uint32_t b;
    memcpy(&b, &f, 4);
    return b;
  }
  return h_rnd() & 0x7f7fffffu;
}
static unsigned pick_pidxret(void) {
  int r = (int)h_rmod(100);
  if (r < 60) return h_rmod(36);
  if (r < 70) return 0xffffffffu;
  if (r < 78) return 0x7fffffffu;
  if (r < 85) return 0x80000000u;
  return h_rnd();
}

typedef struct { unsigned level; int hook_kind; unsigned hook_k, hook_level; } Call;
static void gen_da(DA *d, int regime) {
  int p, fmode = (int)h_rmod(100) < 55 ? 0 : (int)h_rmod(100) < 40 ? 1 : 2;
  memset(d, 0, sizeof *d);
  d->pcm = h_rpct(50) ? 0 : 1;
  d->pcmA = h_rpct(60) ? d->pcm : !d->pcm;
  d->special = h_rpct(25) ? 2 : (int)h_rmod(4);
  d->limit = h_rpct(60) ? 1 + h_rmod(8) : h_rpct(50) ? h_rmod(200) : h_rnd();
  d->pidx = h_rmod(31);
  if (regime == 1) {
    if (h_rpct(10)) d->pcm = (int)(2 + h_rmod(3)); else if (h_rpct(5)) d->pcm = (int)h_rnd();
    if (h_rpct(10)) d->pcmA = (int)h_rnd();
    if (h_rpct(10)) d->special = (int)h_rnd();
  }
  d->pbits = pick_pbits();
  d->pidxret = pick_pidxret();
  for (p = 0; p < 6; p++) {
    d->cur[p] = (unsigned char)(h_rpct(75) ? 60 + h_rmod(68) : h_rpct(70) ? h_rmod(128) : h_rnd());
    if (regime == 1 && h_rpct(8)) d->cur[p] = (unsigned char)(h_rpct(50) ? 255 : 0);
    if (regime == 2) d->cur[p] = (unsigned char)(h_rpct(80) ? 70 + h_rmod(58) : h_rmod(128));
    d->flags[p] = (short)(regime == 0 && fmode == 0 ? 0 : regime == 0 && fmode == 1 ? 1 : regime == 2 ? (h_rpct(85) ? (h_rpct(70) ? 1 : (int)h_rnd() | 1) : 0) : (h_rpct(50) ? 0 : h_rpct(70) ? 1 : (int)h_rnd() | 0x100));
  }
}
static void dump_all(const char *tag) {
  TrnObj tmp;
  int show = h_sample(&g_dumpctr, 40, 97);
  memcpy(&tmp, &g_t, sizeof tmp);
  tmp.params_ = NULL; tmp.power_ = NULL;
  h_dump_obj(show, tag, &tmp, f_TrnObj, NF_TrnObj);
  h_dump_obj(show, "mp", &g_mpb.m, f_mp, NF_mp);
  h_emit(show, "par n=%d n80=%d", g_par.unk78, g_par.unk80);
}

enum { C_TRIALS = 0, C_SKIPPED, C_L0, C_L1, C_L2, C_LHI, C_HOOK_LEVEL, C_HOOK_PAR, C_RET0, C_RET1, C_NMODEL };
static void cov_define_trials(void) {
  h_cov_define(C_TRIALS, "trials"); h_cov_define(C_SKIPPED, "skipped (excluded input)"); h_cov_define(C_L0, "level 0"); h_cov_define(C_L1, "level 1");
  h_cov_define(C_L2, "level 2 (prints)"); h_cov_define(C_LHI, "level >= 2^31"); h_cov_define(C_HOOK_LEVEL, "level hook"); h_cov_define(C_HOOK_PAR, "params_ hook");
  h_cov_define(C_RET0, "returned 0"); h_cov_define(C_RET1, "returned 1");
}
static int cov_id(int i) { return i + 10; }
static void cov_define_names(void) {
  int i;
  for (i = 0; i < MC_LAST; i++) if (mcov_names[i]) h_cov_define(cov_id(i), mcov_names[i]);
}
static void cov_hit_model(const MS *s) {
  int i;
  for (i = 0; i < MC_LAST; i++) if (s->cov[i]) h_cov_hit(cov_id(i));
}

static void compare_run(const char *what, MS *s, int lvl, int hk, unsigned hkk, int ret) {
  unsigned i;
  if (ret != s->ret) h_die3("%s: return %d model %d (level %d hook %d/%u)", what, ret, s->ret, lvl, hk, hkk);
  if ((g_t.params_ == (void *)&g_altpar) != (s->par == &s->altparm)) h_die3("%s: params_ pointer after the call", what);
  if (memcmp(&g_par, &s->parm, sizeof g_par)) h_die3("%s: the parameter block differs", what);
  if (memcmp(&g_altpar, &s->altparm, sizeof g_altpar)) h_die3("%s: the alternate parameter block differs", what);
  for (i = 0; i < sizeof(MapPar90); i++)
    if (((uint8_t *)&g_mpb.m)[i] != ((uint8_t *)&s->mpm)[i]) h_die3("%s: mapping block byte +0x%x: real 0x%02x model 0x%02x (level %d hook %d/%u)", what, i, ((uint8_t *)&g_mpb.m)[i], ((uint8_t *)&s->mpm)[i], lvl, hk, hkk);
  mp_check(&g_mpb, what);
  if (g_nlog != s->nlog) {
    for (i = 0; i < g_nlog && i < 14; i++) fprintf(stderr, "real %u %c %s\n", i, g_log[i].kind, g_log[i].text);
    for (i = 0; i < s->nlog && i < 14; i++) fprintf(stderr, "model %u %c %s\n", i, s->log[i].kind, s->log[i].text);
    h_die3("%s: print count: real %u model %u (level %d hook %d/%u)", what, g_nlog, s->nlog, lvl, hk, hkk);
  }
  for (i = 0; i < g_nlog && i < LOGMAX; i++)
    if (g_log[i].kind != s->log[i].kind || strcmp(g_log[i].text, s->log[i].text))
      h_die3("%s: print %u differs (level %d hook %d/%u):\n  real  %c %s\n  model %c %s", what, i, lvl, hk, hkk, g_log[i].kind, g_log[i].text, s->log[i].kind, s->log[i].text);
}

/* one call: the model decides whether the blob would terminate, picks hooks from its own print list, then the real function runs */
static int do_design(const DA *da, Call c, int emit) {
  MS ms;
  int ret;
  unsigned char gp;
  /* steering: a hook position is chosen from a model run without the hook (a print index), then the model runs again WITH the hook */
  model_init(&ms, c.level, HK_NONE, 0, 0);
  m_design(&ms, da, g_A, g_B, g_V);
  if (ms.hang) { g_nskipped++; h_cov_hit(C_SKIPPED); return 0; }
  if (h_rpct(12)) {   /* steering: params->unk360 at the first ucode below a (lowered) start ucode: the forced scan's first test (unk360 > cur - 1) and the dMin start sit on the edge */
    unsigned q = h_rmod(6);
    g_par.unk360 = (int)ms.ab[q] - 1 + (int)h_rmod(3) - (h_rpct(30) ? 1 : 0);
    if (h_rpct(50)) g_altpar.unk360 = g_par.unk360;
    if (h_rpct(45)) { g_par.unk78 = (int)(1 + h_rmod(3)); if (h_rpct(60)) g_altpar.unk78 = g_par.unk78; }   /* two ucodes: the first one below the start ucode decides success */
    if (h_rpct(35) && ms.ab[q] >= 2) {   /* a forced phase whose ucode cur - 1 is exactly acceptable in the first round (A = B = 3000 at cur, 1000 below: rr = 2000, half = lim = 1000, N = 2) */
      unsigned qq = q, cc = ms.ab[q];
      g_A[qq * 128 + cc] = 3000; g_B[qq * 128 + cc] = 3000 + (short)h_rmod(3);
      g_A[qq * 128 + cc - 1] = 1000 + (short)(h_rpct(30) ? (int)h_rmod(3) - 1 : 0); g_B[qq * 128 + cc - 1] = 1000 + (short)(h_rpct(30) ? (int)h_rmod(3) - 1 : 0);
      g_V[qq * 128 + cc - 1] = (unsigned char)(h_rpct(90) ? 1 : 0);
      g_flags[qq] = 1; ((DA *)da)->flags[qq] = 1;
      g_par.unk78 = 2; if (h_rpct(60)) g_altpar.unk78 = 2;
      g_par.unk360 = (int)cc - 1 - (h_rpct(70) ? 0 : (int)h_rmod(3)); if (h_rpct(50)) g_altpar.unk360 = g_par.unk360;
    }
    model_init(&ms, c.level, HK_NONE, 0, 0);
    m_design(&ms, da, g_A, g_B, g_V);
    if (ms.hang) { g_nskipped++; h_cov_hit(C_SKIPPED); return 0; }
  }
  if (c.hook_kind == HK_NONE && c.level > 1 && h_rpct(15)) {   /* steering: a level hook inside the table rows of a successful design */
    unsigned n, di = 0, cand[LOGMAX], nc = 0;
    for (n = 0; n < ms.nlog && n < LOGMAX; n++) {
      if (ms.log[n].kind == 'd') di++;
      if (ms.log[n].kind == 'd' && !strncmp(ms.log[n].text, "trn2 constel[", 13)) cand[nc++] = di;
    }
    if (nc) { c.hook_kind = HK_LEVEL; c.hook_k = cand[h_rmod(nc)]; c.hook_level = h_rpct(70) ? 1u : 0u; }
  }
  if (c.hook_kind != HK_NONE && !c.hook_k) {
    static const char *const named[] = { "V90TRN2Design: max TX index", "V90TRN2Design: ...hence", "Trn2 Design: phase", "V90TRN2Design: sqrt", "V90TRN2Design: index", "\\n----", "V90TRN2Designer : ADI",
      "trn2 size", "trn2 constel[0]", "trn2 constel[1]", "V90TRN2Designer::V90TRN2Design()" };
    unsigned np = ms.nprint, nd = ms.nd, pi = 0, di = 0, n, q, cand[LOGMAX], nc = 0;
    c.hook_k = c.hook_kind == HK_LEVEL ? (nd ? 1 + h_rmod(nd) : 0) : (np ? 1 + h_rmod(np) : 0);
    if (h_rpct(60)) {
      for (n = 0; n < ms.nlog && n < LOGMAX; n++) {
        if (ms.log[n].kind == 'c') continue;
        pi++;
        if (ms.log[n].kind == 'd') di++;
        for (q = 0; q < sizeof named / sizeof named[0]; q++)
          if (!strncmp(ms.log[n].text, named[q], strlen(named[q]))) { if (c.hook_kind != HK_LEVEL || ms.log[n].kind == 'd') cand[nc++] = c.hook_kind == HK_LEVEL ? di : pi; break; }
      }
      if (nc) c.hook_k = cand[h_rmod(nc)];
    }
    if (h_rpct(45)) {   /* steering: a hook inside the table rows (the level lowered / params_ swapped between two rows) */
      pi = di = nc = 0;
      for (n = 0; n < ms.nlog && n < LOGMAX; n++) {
        if (ms.log[n].kind == 'c') continue;
        pi++;
        if (ms.log[n].kind == 'd') di++;
        if (!strncmp(ms.log[n].text, "trn2 constel[", 13)) cand[nc++] = c.hook_kind == HK_LEVEL ? di : pi;
      }
      if (nc) c.hook_k = cand[h_rmod(nc)];
    }
    if (!c.hook_k) c.hook_kind = HK_NONE;
    c.hook_level = h_rpct(60) ? 1u : (unsigned)h_rmod(3);
  }
  model_init(&ms, c.level, c.hook_kind, c.hook_k, c.hook_level);
  m_design(&ms, da, g_A, g_B, g_V);
  if (ms.hang) { g_nskipped++; h_cov_hit(C_SKIPPED); return 0; }
  g_t.params_ = &g_par; g_t.power_ = g_pow;
  g_alt_par = &g_altpar;
  g_snap_t = &g_t; g_snap_mp = &g_mpb.m;
  g_hook_kind = c.hook_kind; g_hook_k = c.hook_k; g_hook_level = c.hook_level;
  g_expect_mp = &g_mpb.m; g_expect_pcm = da->pcm;
  memcpy(&g_script_p, &da->pbits, 4);
  g_script_idx = da->pidxret;
  dsplibs_debug_level = c.level;
  log_begin();
  gp = da->pidx > 255 ? 0 : (unsigned char)da->pidx;
  ret = tr_design(&g_t, &g_mpb.m, g_A, g_B, g_V, g_flags, da->pcmA, da->pcm, (int)h_rnd(), g_cur, da->limit, h_garbage_hi(gp), da->special);
  g_hook_k = 0; g_snap_t = NULL; g_snap_mp = NULL;
  compare_run("design", &ms, (int)c.level, c.hook_kind, c.hook_k, ret);
  cov_hit_model(&ms);
  h_cov_hit(C_TRIALS);
  h_cov_hit(ret ? C_RET1 : C_RET0);
  if (c.level == 0) h_cov_hit(C_L0); else if (c.level == 1) h_cov_hit(C_L1); else if (c.level >= 0x80000000u) h_cov_hit(C_LHI); else h_cov_hit(C_L2);
  if (c.hook_kind == HK_PAR && c.hook_k) h_cov_hit(C_HOOK_PAR);
  if (c.hook_kind == HK_LEVEL && c.hook_k) h_cov_hit(C_HOOK_LEVEL);
  if (emit) {
    h_emit(0, "call design ret=%d prints=%u", ret, g_nlog);
    dump_all("T");
  }
  return 1;
}

/* ------------------------------------------------------------------------------------------------ scenarios 0 / 1 / 2: V90TRN2Design */
static void sc_design(int regime, int ntr) {
  int tr, p;
  cov_define_trials();
  cov_define_names();
  h_scenario_begin(0);
  relax_reserved();
  {
    int i;
    for (i = 0; i < MC_LAST; i++) h_covreq[cov_id(i)] = 1;
    h_covreq[C_SKIPPED] = 0;
    if (regime == 0) {   /* the nominal regime does not reach every failure / edge branch */
      h_covreq[cov_id(MC_N_NEG)] = h_covreq[cov_id(MC_DMIN_N0)] = h_covreq[cov_id(MC_FORCED_N0)] = h_covreq[cov_id(MC_K_ZERO)] = h_covreq[cov_id(MC_FORCED_TCAP)] = 0;
    }
    if (regime == 1) h_covreq[cov_id(MC_N_NEG)] = 0;
    if (regime == 2) h_covreq[cov_id(MC_N_NEG)] = h_covreq[cov_id(MC_DMIN_N0)] = h_covreq[cov_id(MC_FORCED_N0)] = h_covreq[cov_id(MC_K_ZERO)] = 0;
  }
  mp_canary(&g_mpb);
  for (tr = 0; tr < ntr; tr++) {
    DA da;
    Call c;
    int N = pick_n(regime);
    gen_tables(h_rpct(50), regime);
    fill_words(&g_mpb.m, sizeof g_mpb.m);
    gen_par(&g_par, regime, N);
    gen_par(&g_altpar, regime, h_rpct(60) ? pick_n(regime) : N);
    if (h_rpct(30)) g_altpar.unk360 = g_par.unk360;
    memset(&g_t, 0, sizeof g_t);
    gen_da(&da, regime);
    for (p = 0; p < 6; p++) { g_flags[p] = da.flags[p]; g_cur[p] = da.cur[p]; }
    g_flags[6] = g_flags[7] = 0x5a5a; g_cur[6] = g_cur[7] = 0x99;
    c.level = pick_level();
    c.hook_kind = h_rpct(28) ? (int)(1 + h_rmod(2)) : HK_NONE;
    c.hook_k = 0; c.hook_level = 0;
    do_design(&da, c, 1);
    EXPECT_EQ("flags canary", (unsigned)(unsigned short)g_flags[6], 0x5a5au);
    EXPECT_EQ("cur canary", g_cur[6], 0x99u);
    if (h_rpct(25)) {   /* a second call with the first call's block as the initial state (every field is rewritten) */
      Call c2 = c;
      c2.hook_kind = HK_NONE; c2.level = pick_level();
      do_design(&da, c2, 1);
    }
  }
}

/* ------------------------------------------------------------------------------------------------ scenario 3: ctor / dtor / setNofUcodesInTrn2 / maxK / setTrn2DummyConstel */
typedef struct { TrnObj t; uint8_t canary[48]; } TBox;
static TBox g_tb;
static void tbox_prefill(void) { memset(&g_tb.t, 0, sizeof g_tb.t); g_tb.t.params_ = (void *)(uintptr_t)h_rnd(); g_tb.t.power_ = (void *)(uintptr_t)h_rnd(); memset(g_tb.canary, 0xe9, sizeof g_tb.canary); }
static void tbox_check(const char *w) {
  unsigned i;
  for (i = 0; i < sizeof g_tb.canary; i++) if (g_tb.canary[i] != 0xe9) h_die3("%s: object canary byte %u overwritten", w, i);
}
static void sc_small(int ntr) {
  int tr;
  cov_define_trials();
  h_covreq[C_L0] = h_covreq[C_L1] = h_covreq[C_L2] = h_covreq[C_LHI] = h_covreq[C_SKIPPED] = h_covreq[C_HOOK_LEVEL] = h_covreq[C_HOOK_PAR] = h_covreq[C_RET0] = h_covreq[C_RET1] = 0;
  h_scenario_begin(0);
  relax_reserved();
  mp_canary(&g_mpb);
  for (tr = 0; tr < ntr; tr++) {
    int what = (int)h_rmod(5);
    tbox_prefill();
    if (what == 0 || what == 1) {   /* C1 / C2 then D1 / D2 */
      void *p = &g_par, *pw = g_pow;
      TrnObj before;
      memset(&before, 0, sizeof before);
      if (what == 0) tr_c1(&g_tb.t, p, pw); else tr_c2(&g_tb.t, p, pw);
      if (g_tb.t.params_ != p || g_tb.t.power_ != pw) h_die3("constructor: pointers %p %p", g_tb.t.params_, g_tb.t.power_);
      dump_all("ctor");
      if (h_rpct(50)) tr_d1(&g_tb.t); else tr_d2(&g_tb.t);
      if (g_tb.t.params_ != p || g_tb.t.power_ != pw) h_die3("destructor changed the object");
    } else if (what == 2) {   /* setNofUcodesInTrn2 */
      int n = h_rpct(30) ? 0 : h_rpct(50) ? (int)h_rrange(-3, 3) : (int)h_rnd();
      int n78, n80;
      gen_par(&g_par, 0, (int)h_rrange(-5, 40));
      n78 = g_par.unk78; n80 = g_par.unk80;
      g_tb.t.params_ = &g_par;
      {
        /* the argument is a `short`: the low 16 bits decide, the upper 16 are garbage */
        int arg = (int)(((unsigned)n & 0xffffu) | (h_rnd() & 0xffff0000u));
        tr_setn(&g_tb.t, arg);
        if ((short)n != 0) EXPECT_EQ("setNofUcodesInTrn2", (unsigned)g_par.unk78, (unsigned)n80);
        else EXPECT_EQ("setNofUcodesInTrn2 (zero)", (unsigned)g_par.unk78, (unsigned)n78);
        EXPECT_EQ("unk80 kept", (unsigned)g_par.unk80, (unsigned)n80);
      }
      h_emit(0, "setn arg=%d n78=%d n80=%d -> %d", (short)n, n78, n80, g_par.unk78);
      dump_all("setn");
    } else if (what == 3) {   /* maxK */
      uint32_t c0 = h_rpct(20) ? 0u : h_rpct(50) ? 1u + h_rmod(130) : h_rnd();
      unsigned got, want;
      fill_words(&g_mpb.m, sizeof g_mpb.m);
      {
        int k;
        for (k = 0; k < 6; k++) g_mpb.m.cnt_[k] = h_rpct(60) ? c0 : h_rpct(50) ? 1u + h_rmod(130) : h_rnd();
        if (h_rpct(8)) g_mpb.m.cnt_[h_rmod(6)] = 0;
        if (h_rpct(15)) for (k = 0; k < 6; k++) g_mpb.m.cnt_[k] = 1u << h_rmod(32);
      }
      got = tr_maxk(&g_tb.t, &g_mpb.m);
      want = m_kint(&g_mpb.m);
      if (got != want) h_die3("maxK: got %u model %u", got, want);
      h_emit(0, "maxk %u", got);
      mp_check(&g_mpb, "maxK");
    } else {   /* setTrn2DummyConstel */
      MapPar90 want;
      int N = h_rpct(15) ? (int)h_rrange(-3, 1) : h_rpct(15) ? 120 + (int)h_rmod(20) : 1 + (int)h_rmod(40), k, i;
      gen_par(&g_par, 0, N);
      fill_words(&g_mpb.m, sizeof g_mpb.m);
      memcpy(&want, &g_mpb.m, sizeof want);
      g_tb.t.params_ = &g_par;
      for (k = 0; k < 6; k++) {
        unsigned char cc = 0x4e;
        for (i = 0; i < N; i++, cc--) { *m_la(&want, (unsigned)k, (unsigned)i) = cc; *m_lb(&want, (unsigned)k, (unsigned)i) = cc; }
      }
      tr_dummy(&g_tb.t, &g_mpb.m);
      if (memcmp(&want, &g_mpb.m, sizeof want)) h_die3("setTrn2DummyConstel: the block differs from the model (N %d)", N);
      mp_check(&g_mpb, "setTrn2DummyConstel");
      dump_all("dummy");
    }
    tbox_check("small object");
    h_cov_hit(C_TRIALS);
  }
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P26_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P26_DBG")) g_show = 1;
  p07_layout_check();
  p11_layout_check();
  p26_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(240);
  switch (sc) {
    case 0: sc_design(0, 3000); break;
    case 1: sc_design(1, 3000); break;
    case 2: sc_design(2, 3000); break;
    case 3: sc_small(3000); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu edprintf %u dbgprintf %u callee %u skipped %u", h_nmalloc, h_nfree, g_nlog_total_e, g_nlog_total_d, g_nlog_total_c, g_nskipped);
  h_cov_finish();
  return 0;
}
