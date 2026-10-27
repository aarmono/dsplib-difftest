/*
 * Differential test harness for port chunk P23 (src/vpcm/v90_constellation_power.cpp): V90ConstellationDesigner::adjustConstellationsPower and
 * V90ConstellationDesigner::adjustConstellationsToNewK (2 functions).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_constellation_power.sh against the blob (dsplibs.o with `edprintf` and determineDminForRrn WEAK so that the recorders below
 * win), against the port (-O0 / -O2; getPower / getPowerIndexForPower / linear2alaw / linear2ulaw come from the P06 port object there) and for x86-64 (diffed against the 32-bit BLOB's output =
 * the golden test/golden/vpcm/v90_constellation_power.blob32.out; -mfpmath=387 in test/vpcm/v90_constellation_power.golden64.sh).  The functions are reached through __asm__ labels; the designer object (0x54 B, five pointers), the
 * V90Parameters block (P07 mirror: only upRoundK is read), the V90ConstellationPower object (P06 mirror, one pointer) and the V90MappingParams blocks (P11 mirror, 0x650 B) are the
 * generated C mirrors; the pointer-holding objects are STATIC / member-wise filled (their sizes differ on x86-64), everything else comes from the arena pool.
 *
 * determineDminForRrn (row P25; scenarios 0-6) is INTERPOSED by a recorder in all builds (it is a leaf that writes only rrnDownDmin_ / rrnUpDmin_ and prints through
 * dsplibs_debug_printf; the x86-64 run has no blob to link): it logs (this, K) at its place in the print sequence (with a snapshot of the object and the block) and writes values that depend
 * on K and on the call index.
 *
 * What is compared: the whole designer object, the whole mapping block (and the alternate block after a swap), the power object, the inputs (read-only) after every call, every edprintf /
 * dsplibs_debug_printf / determineDminForRrn call (kind, text with the integer arguments formatted, a SNAPSHOT of the designer object and the mapping block at the moment of the call: the
 * store order between prints) and the unsigned `level > 1` test at every dsplibs_debug_printf site (levels 0, 1, 2, 3, >= 2^31; a hook lowers the level inside the k-th print; two more hooks
 * swap T->params_ / T->mapParams_ inside the k-th print: every re-read of the blob is a re-read here).
 *
 * INDEPENDENT MODELS (written from the disassembly notes in a specification style, long double arithmetic with the same x87 instruction for log10): m_power (the whole
 * adjustConstellationsPower: K, power / limit loop with 1 / 10 / 20 removals per round, BUG stop, K <= 20 stop, restore, prints) and m_newk (the whole adjustConstellationsToNewK: K, real K,
 * the add loop with its two scans, G.711 codes, overflow undo, the remove loop with restore, prints).  getPower / getPowerIndexForPower are the linked functions (blob in the blob run, P06
 * port in the port runs: P06 is verified separately) on the model's OWN power object and block copies; the G.711 encoders likewise.
 */

#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "pcm_leaf_classes_layout.h"
#include "pcm_parameters_layout.h"
#include "v92_cp_layout.h"
#include "v90_constellation_noise_layout.h"

#define L(m) __asm__(m)

extern void cd_power(void *) L("_ZN24V90ConstellationDesigner25adjustConstellationsPowerEv");
extern void cd_newk(void *, short (*)[128], short (*)[128], short *, unsigned char (*)[128]) L("_ZN24V90ConstellationDesigner26adjustConstellationsToNewKEPA128_sS1_PsPA128_h");
extern void cp_c1(void *) L("_ZN21V90ConstellationPowerC1Ev");
extern float cp_power(void *, void *, int, int) L("_ZN21V90ConstellationPower8getPowerEP16V90MappingParams26V90TxPowerMeasurementPoint7PcmType");
extern unsigned cp_pidx(void *, float) L("_ZN21V90ConstellationPower21getPowerIndexForPowerEf");
extern unsigned cp_limits[35] L("_ZN21V90ConstellationPower18averagePowerLimitsE");
extern int linear2alaw(int);
extern int linear2ulaw(int);
extern int alaw2linear(int);
extern int ulaw2linear(int);
extern unsigned int dsplibs_debug_level;
/* scenario 7 (i386 only): the REAL blob function behind the hook (the build script links it as real_ddfr; absent in the x86-64 build) */
extern void real_ddfr(void *, unsigned) __attribute__((weak));
/* the P24 callees of process, referenced by the P25 port object that supplies real_ddfr in the port builds: never called here (the blob provides them in the blob run) */
__attribute__((weak)) void h_unused_scn(void) L("_ZN24V90ConstellationDesigner23setConstellationToNoiseEfPA128_sS1_PsPhPA128_h");
__attribute__((weak)) void h_unused_scnfr(void) L("_ZN24V90ConstellationDesigner33setConstellationToNoise_forceRateEfPA128_sS1_PsPhS3_PA128_h");
void h_unused_scn(void) { abort(); }
void h_unused_scnfr(void) { abort(); }
/* the interposed callee (row P25): defined below */
extern void h_ddfr(void *, unsigned) L("_ZN24V90ConstellationDesigner19determineDminForRrnEj");

typedef long double LD;

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
static unsigned g_nlog, g_nlog_total_e, g_nlog_total_d, g_nlog_total_r;
static int g_show;
static CdObj *g_snap_t;       /* object / mapping block snapshotted at every print */
static MapPar90 *g_snap_mp;
enum { HK_NONE = 0, HK_LEVEL, HK_PAR, HK_MP };
static int g_hook_kind;
static unsigned g_hook_k, g_hook_level;   /* HK_LEVEL: level after the k-th dsplibs_debug_printf; HK_PAR / HK_MP: swap after the k-th print of any kind */
static unsigned g_nprints_in_call, g_nd_in_call, g_dd_seq;
static int g_real_ddfr;       /* scenario 7: determineDminForRrn forwards to the blob's own function */
struct MS_;
static struct MS_ *g_model_ms;  /* set while the MODEL runs the real function: its debug prints go to the model's log */
static void model_dbg(const char *fmt, va_list ap);
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
  if (g_model_ms) { model_dbg(fmt, ap); va_end(ap); return 0; }
  record('d', fmt, ap);
  va_end(ap);
  g_nlog_total_d++;
  do_hook('d');
  return 0;
}
static void log_begin(void) { g_nlog = 0; g_nd_in_call = 0; g_nprints_in_call = 0; g_dd_seq = 0; }

/* the recorder behind determineDminForRrn: values depend on K and the call index; (this, K) are logged in the print sequence */
static unsigned short dd_down(unsigned k, unsigned seq) { return (unsigned short)(k * 7u + 3u * seq + 1u); }
static unsigned short dd_up(unsigned k, unsigned seq) { return (unsigned short)(k * 5u + 11u * seq + 2u); }
void h_ddfr(void *t, unsigned k) {
  CdObj *o = (CdObj *)t;
  if (t != (void *)g_snap_t) h_die3("determineDminForRrn called with this %p, expected %p", t, (void *)g_snap_t);
  {
    char buf[64];
    snprintf(buf, sizeof buf, "ddfr K=%u", k);
    if (g_nlog < LOGMAX) { g_log[g_nlog].kind = 'r'; g_log[g_nlog].text[0] = 0; esc_cat(g_log[g_nlog].text, sizeof g_log[g_nlog].text, buf); }
    h_emit(g_show, "rprintf %s", buf);
    g_nlog++;
    snapshot();
  }
  if (g_real_ddfr) {
    if (!real_ddfr) h_die3("scenario 7 needs the blob's determineDminForRrn (i386 builds only)");
    real_ddfr(t, k);
    g_dd_seq++;
    g_nlog_total_r++;
    return;
  }
  o->rrnDownDmin_ = dd_down(k, g_dd_seq);
  o->rrnUpDmin_ = dd_up(k, g_dd_seq);
  g_dd_seq++;
  g_nlog_total_r++;
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
  return h_rpct(60) ? 2u : lv[h_rmod(sizeof lv / sizeof lv[0])];
}
static float F(uint32_t bits) { float f; memcpy(&f, &bits, 4); return f; }
/* a float STORE: the blob's fstp to a float slot rounds; a plain (float) cast may stay in an x87 register in this compiler's default excess-precision mode */
static float RF(LD v) { volatile float f = (float)v; return f; }

/* ------------------------------------------------------------------------------------------------ models: arithmetic */
static int m_i32(LD v) {   /* fistp DWORD with truncation: the integer indefinite for NaN / out of range */
  if (!(v > -2147483649.0L && v < 2147483648.0L)) return (int)0x80000000u;
  return (int)(long long)v;
}
static void m_split(LD v, LD scale, LD sv, int *sign, int *ip, int *fp) {
  int t = m_i32(v), f;
  long long fl;
  LD av = v < 0 ? -v : v;
  LD sc = (v - (LD)t) * scale;
  if (v != v) av = v;
  *sign = (sv > 0.0L || sv != sv) ? '+' : '-';
  *ip = m_i32(av);
  f = m_i32(sc);
  fl = f;
  if (fl < 0) fl = -fl;
  *fp = (int)fl;
  if (f == (int)0x80000000u) *fp = (int)0x80000000u;
}
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
static float m_lg2f(void) { return RF(m_log10(2.0L)); }
static unsigned m_trunc64lo(LD v) {
  if (!(v > -9223372036854775809.0L && v < 9223372036854775808.0L)) return 0;
  return (unsigned)(long long)v;
}
static unsigned m_kint(const MapPar90 *mp) {
  LD p = m_prod(mp), lg;
  float den;
  if (p == 0.0L) return 0;
  lg = m_log10(p);
  den = m_lg2f();
  return m_trunc64lo(lg / (LD)den + (LD)F(0x358637bdu));
}
static float m_kreal_before(const MapPar90 *mp) {
  LD p = m_prod(mp);
  float lgf, den;
  if (p == 0.0L) return 0.0f;
  lgf = RF(m_log10(p));
  den = m_lg2f();
  return RF((LD)lgf / (LD)den + (LD)F(0x3089705fu));
}
static LD m_kreal_after(const MapPar90 *mp) {
  LD p = m_prod(mp);
  float lgf, den;
  if (p == 0.0L) return 0.0L;
  lgf = RF(m_log10(p));
  den = m_lg2f();
  return (LD)lgf / (LD)den + (LD)F(0x3089705fu);
}

/* ------------------------------------------------------------------------------------------------ model state */
typedef struct MS_ {
  const short *A, *B;            /* flat [6][128] (+ slack) */
  const short *dual;
  CdObj t;                       /* non-pointer members */
  const V90ParObj *par;          /* T->params_ (switched by a PAR hook) */
  MapPar90 *mp;                  /* T->mapParams_ (a model copy; switched by a MP hook) */
  MapPar90 mpm, altm;            /* model copies of the real blocks */
  const V90ParObj *altpar;
  CpObj pow;                     /* the model's own power object */
  unsigned level;
  int hook_kind;
  unsigned hook_k, hook_level, nprint, nd, ddseq;
  LogEnt log[LOGMAX];
  unsigned nlog;
  int hang;                      /* the blob would not terminate / read garbage on this input */
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
  if (kind == 'r') return;
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

static unsigned char *m_la(MapPar90 *mp, unsigned p, unsigned i) { return (unsigned char *)mp + 4u + p * 0x80u + i; }
static unsigned char *m_lb(MapPar90 *mp, unsigned p, unsigned i) { return (unsigned char *)mp + 0x304u + p * 0x80u + i; }
static unsigned char m_code(const MS *s, int v) {   /* the G.711 code of a distance */
  int a = v < 0 ? -v : v;
  if (v == INT_MIN) a = v;
  if (s->t.useAlaw_ == 0) return (unsigned char)(~(unsigned char)linear2ulaw(a));
  return (unsigned char)((unsigned char)linear2alaw(a) ^ 0xd5);
}
static float m_pw(MS *s, MapPar90 *mp) {
  int k;
  for (k = 0; k < 6; k++) if (mp->cnt_[k] == 0) { /* the blob divides by the count: #DE */ if (getenv("P23_DBG")) fprintf(stderr, "model: getPower on a block with a zero count (phase %d)\n", k); s->hang = 1; return 0.0f; }
  { volatile float f = cp_power(&s->pow, mp, 1, (int)s->t.useAlaw_); return f; }
}
static unsigned m_pidx(MS *s, float p) { return cp_pidx(&s->pow, p); }
static void *g_cur_adid;
static void model_dbg(const char *fmt, va_list ap) {
  MS *s = (MS *)g_model_ms;
  char buf[TXTMAX * 2];
  vsnprintf(buf, sizeof buf, fmt, ap);
  m_print(s, 'd', "%s", buf);
  dsplibs_debug_level = s->level;            /* the real function re-reads the level after every print */
  s->t.params_ = (void *)s->par;             /* ... and the two pointers of its object */
  s->t.mapParams_ = (void *)s->mp;
}
static void m_ddfr(MS *s, unsigned k) {
  m_print(s, 'r', "ddfr K=%u", k);
  if (g_real_ddfr) {
    /* the real function loops 2^32 times when its distance estimate is negative: it is only run on blocks whose K matches their counts and whose largest eligible count is >= 3 (it is never
     * called otherwise: K comes from the same counts) */
    unsigned q, maxc = 0;
    LD lp = m_log10(m_prod(s->mp)) / m_log10(2.0L);
    for (q = 0; q < 6; q++) if (((unsigned char *)g_cur_adid)[0x280c + q] == 0 && s->mp->cnt_[q] > maxc) maxc = s->mp->cnt_[q];
    if (maxc < 3 || !((LD)k >= lp - 2.0L && (LD)k <= lp + 3.0L)) { s->hang = 1; return; }
    s->t.params_ = (void *)s->par; s->t.mapParams_ = (void *)s->mp; s->t.adid_ = g_cur_adid;
    dsplibs_debug_level = s->level;
    g_model_ms = s;
    real_ddfr(&s->t, k);
    g_model_ms = NULL;
    s->ddseq++;
    return;
  }
  s->t.rrnDownDmin_ = dd_down(k, s->ddseq);
  s->t.rrnUpDmin_ = dd_up(k, s->ddseq);
  s->ddseq++;
}
/* the highest / lowest first ucode, ties: the larger count (only strictly larger takes), then the lower phase */
static unsigned m_pick(MapPar90 *mp, int highest) {
  unsigned best = 0, ph;
  for (ph = 1; ph < 6; ph++) {
    unsigned v = *m_la(mp, ph, 0), bv = *m_la(mp, best, 0);
    int better = highest ? v > bv : v < bv;
    if (better || (v == bv && mp->cnt_[ph] > mp->cnt_[best])) {
      /* the blob tracks (value, count) of the current best: a strictly better value replaces both, an equal value only a larger count */
      best = ph;
    }
  }
  return best;
}

/* ------------------------------------------------------------------------------------------------ coverage ids */
enum { CP_ENTRY_SKIP, CP_ROUND1, CP_ROUND10, CP_ROUND20, CP_FLAG8, CP_BUG, CP_K_LE20_REMOVED, CP_K_LE20_NONE, CP_RESTORE, CP_CONT_LOOP, CP_POW_NAN_STOP, CP_LIM22, CP_LIM_LOW,
       CP_TIE, CP_HOOK_MP_P, CP_HOOK_PAR_P,
       CN_ADD, CN_NOADD, CN_ADD_NOLOOP, CN_ADD_SINGLE, CN_ADD_DUAL, CN_ADD_DUAL_SKIP, CN_ADD_SINGLE_SKIP, CN_ADD_OVER_U, CN_ADD_OVER_CNT, CN_ADD_DONE, CN_REMOVE, CN_REMOVE_LOOP,
       CN_REMOVE_BUG, CN_RESTORE, CN_NORESTORE_1, CN_K1_ZERO, CN_ALAW, CN_ULAW, CN_TIE_MIN, CN_TIE_MAX, CN_ADD_THEN_REMOVE, CN_UNDO_SHIFT, CN_KREAL_ZERO, CN_UPR_EQ, CN_UPR_NAN,
       CN_HOOK_MP, CN_HOOK_PAR, CN_ADD_MULTI, CN_REMOVE_MULTI, CN_DUAL_BREAK_A, CN_DUAL_BREAK_B, CN_SINGLE_BREAK, CN_DUAL_RUNOUT, CN_SINGLE_RUNOUT, CN_PAR_AFTER, CF_LAST };

/* ------------------------------------------------------------------------------------------------ model: adjustConstellationsPower */
static void m_power_print_sqrt(MS *s, const char *fmt, int pre, int haspre, float P) {
  int sg, ip, fp;
  m_split(m_sqrt((LD)P), 1000.0L, (LD)P, &sg, &ip, &fp);
  if (haspre) M_E(s, fmt, pre, sg, ip, fp); else M_E(s, fmt, sg, ip, fp);
}
static void m_power_print_idx(MS *s, const char *fmt, unsigned idx) {
  int sg, ip, fp;
  LD v = (LD)(unsigned)(idx + 1u) * (LD)F(0xbf000000u);
  m_split(v, 10.0L, v, &sg, &ip, &fp);
  M_E(s, fmt, (int)idx, sg, ip, fp);
}

static void m_power(MS *s) {
  MapPar90 *mp = s->mp;
  int cont = 1, removed = 0;
  unsigned char svA = 0, svB = 0;
  unsigned lastP = 0, lim, diff, idx;
  float P, Plim;
  mp->w0_ = m_kint(mp) - mp->w620_ + 6u;
  P = m_pw(s, s->mp);
  idx = m_pidx(s, P);
  M_E(s, "--------------------------------------\r\n");
  M_E(s, "V90ConstellationDesigner:\r\n");
  m_power_print_sqrt(s, "V90ConstellationDesigner: initial sqrt(power)  = %c%d.%03d\r\n", 0, 0, P);
  m_power_print_idx(s, "V90ConstellationDesigner: initial index %d, power in dBm0  = %c%d.%01d\r\n", idx);
  lim = s->t.w38_ > 0x15 ? 0x16u : s->t.w38_;
  if (lim == 0x16) s->cov[CP_LIM22] = 1;
  if (lim < 5) s->cov[CP_LIM_LOW] = 1;
  diff = lim - idx;
  Plim = RF((LD)cp_limits[lim]);
  if (!((LD)m_pw(s, s->mp) > (LD)Plim)) { s->cov[CP_ENTRY_SKIP] = 1; goto fin_a; }
  for (;;) {
    unsigned n = diff > 7 ? 20u : (diff < 4 ? 1u : 10u);
    if (n == 20) s->cov[CP_ROUND20] = 1; else if (n == 1) s->cov[CP_ROUND1] = 1; else s->cov[CP_ROUND10] = 1;
    if (s->t.flag8_ > 13) { n = 1; s->cov[CP_FLAG8] = 1; }
    mp = s->mp;
    for (; n != 0; n--) {
      unsigned p = m_pick(mp, 1), q;
      unsigned char first = *m_la(mp, 0, 0);
      (void)first;
      for (q = 0; q < 6; q++) if (q != p && *m_la(mp, q, 0) == *m_la(mp, p, 0)) s->cov[CP_TIE] = 1;
      lastP = p;
      if (mp->cnt_[p] == 0) { s->hang = 1; return; }   /* the blob wraps the count to 2^32 - 1 and runs wild */
      if (--mp->cnt_[p] == 0) {
        s->cov[CP_BUG] = 1;
        if (s->level > 1) {
          M_D(s, "V90ConnectionDesigner: adjustConstellationPower - BUG !!! reached constellation length 0!!!\n");
          if (s->hook_kind == HK_MP && s->hook_k == s->nprint) s->cov[CP_HOOK_MP_P] = 1;
          mp = s->mp;
        }
        cont = 0;
        mp->cnt_[p]++;
        goto recompute;
      }
      svB = *m_lb(mp, p, 0);
      svA = *m_la(mp, p, 0);
      { unsigned i; for (i = 0; ; i++) { *m_lb(mp, p, i) = *m_lb(mp, p, i + 1); *m_la(mp, p, i) = *m_la(mp, p, i + 1); if (!(mp->cnt_[p] > i + 1)) break; } }
      removed++;
    }
  recompute : {
    unsigned K = m_kint(mp) - mp->w620_ + 6u;
    mp->w0_ = K;
    if (K > 0x14) {
      float P2 = m_pw(s, mp);
      diff = lim - m_pidx(s, P2);
      if ((LD)P2 > (LD)Plim && cont) { s->cov[CP_CONT_LOOP] = 1; continue; }
      if (!((LD)P2 > (LD)Plim)) s->cov[CP_POW_NAN_STOP] = 1;
      goto fin_a;
    }
    if (s->level > 1) M_D(s, "V90ConstellationDesigner::adjustConstellationsPower (pParams->d=%d)<V90_MIN_D_IN_DATA\n", (int)K);
    if (removed == 0) { s->cov[CP_K_LE20_NONE] = 1; goto fin_a; }
    s->cov[CP_K_LE20_REMOVED] = 1;
    if (s->level > 1) M_D(s, "V90ConstellationDesigner::adjustConstellationsPower restoring last removed point\n");
    mp = s->mp;
    {
      unsigned c = mp->cnt_[lastP] + 1u, k;
      mp->cnt_[lastP] = c;
      for (k = c; k != 0; k--) { *m_lb(mp, lastP, k) = *m_lb(mp, lastP, k - 1); *m_la(mp, lastP, k) = *m_la(mp, lastP, k - 1); }
      mp = s->mp;
      *m_lb(mp, lastP, 0) = svB;
      *m_la(mp, lastP, 0) = svA;
    }
    s->cov[CP_RESTORE] = 1;
    removed--;
    mp->w0_ = m_kint(mp) - mp->w620_ + 6u;
    goto fin_b;
  }
  }
fin_a:
  mp = s->mp;
fin_b : {
  float P2 = m_pw(s, mp);
  unsigned idx2 = m_pidx(s, P2);
  m_power_print_sqrt(s, "V90ConstellationDesigner: nof points removed %d, new sqrt(power)  = %c%d.%03d\r\n", removed, 1, P2);
  m_power_print_idx(s, "V90ConstellationDesigner: final index %d, final power in dBm0  = %c%d.%01d\r\n", idx2);
  M_E(s, "--------------------------------------\r\n");
}
}

/* ------------------------------------------------------------------------------------------------ model: adjustConstellationsToNewK */
static void m_newk(MS *s) {
  MapPar90 *mp = s->mp;
  int removedCnt = 0, b7 = 1;
  unsigned char addc = 0, saved[6], svA2 = 0, svB2 = 0;
  unsigned lastP = 0, K0, curK, ph;
  int havesv = 0;   /* the blob's save slots of the remove loop are written by its first non-BUG iteration only (stale stack otherwise) */
  float Kreal, P;
  int sg, ip, fp;
  K0 = m_kint(mp);
  curK = m_kint(mp);
  mp->w0_ = K0 - mp->w620_ + 6u;
  Kreal = m_kreal_before(mp);
  if (Kreal == 0.0f) s->cov[CN_KREAL_ZERO] = 1;
  P = m_pw(s, mp);
  (void)m_pidx(s, P);
  M_E(s, "---------------------------------------------------------------\r\n");
  M_E(s, "V90ConnectionDesigner: redundant points optimization :\r\n");
  m_split((LD)Kreal, 100000.0L, (LD)Kreal, &sg, &ip, &fp);
  M_E(s, "V90ConnectionDesigner: real K before optimization  = %c%d.%05d\r\n", sg, ip, fp);
  {
    LD d = (LD)Kreal - (LD)K0;
    LD th = (LD)s->par->upRoundK;
    if (d == th) s->cov[CN_UPR_EQ] = 1;
    if (th != th) s->cov[CN_UPR_NAN] = 1;
    if (!(d > th)) { s->cov[CN_NOADD] = 1; goto after_add; }
  }
  s->cov[CN_ADD] = 1;
  {
    unsigned target = K0 + 1u;
    m_ddfr(s, target);
    b7 = 0;
    mp = s->mp;
    for (ph = 0; ph < 6; ph++) saved[ph] = *m_la(mp, ph, 0);
    if (target <= curK) { s->cov[CN_ADD_NOLOOP] = 1; }
    else {
      unsigned iters = 0;
      do {
        unsigned p = m_pick(mp, 0), u0, idx0, u, q;
        unsigned char code;
        for (q = 0; q < 6; q++) if (q != p && *m_la(mp, q, 0) == *m_la(mp, p, 0)) s->cov[CN_TIE_MIN] = 1;
        mp->cnt_[p]++;
        u0 = *m_la(mp, p, 0);
        idx0 = p * 128u + u0;
        u = u0;
        if (s->dual[p] == 0) {
          s->cov[CN_ADD_SINGLE] = 1;
          if ((signed char)u < 0) s->cov[CN_ADD_SINGLE_SKIP] = 1;
          else {
            int thr = (int)s->A[idx0] + (int)(short)s->t.dMin_, k = 0;
            for (;;) {
              int cv = s->A[idx0 + k];
              k++;
              if (cv >= thr) { s->cov[CN_SINGLE_BREAK] = 1; break; }
              u = (u + 1) & 0xff;
              if ((signed char)u < 0) { s->cov[CN_SINGLE_RUNOUT] = 1; break; }
            }
          }
        } else {
          s->cov[CN_ADD_DUAL] = 1;
          if (u > 0x71) s->cov[CN_ADD_DUAL_SKIP] = 1;
          else {
            int dA = (int)(short)s->t.dMinAlt_, thr1 = (int)s->A[idx0] + dA, thr2 = (int)s->B[idx0] + dA, k = 0;
            for (;;) {
              int cv = s->A[idx0 + k];
              if (cv >= thr1 && cv >= thr2) { s->cov[thr1 >= thr2 ? CN_DUAL_BREAK_A : CN_DUAL_BREAK_B] = 1; break; }
              u = (u + 1) & 0xff;
              k++;
              if (u > 0x71) { s->cov[CN_DUAL_RUNOUT] = 1; break; }
            }
          }
        }
        code = m_code(s, s->A[p * 128u + u]);
        s->cov[s->t.useAlaw_ ? CN_ALAW : CN_ULAW] = 1;
        if ((signed char)u < 0 || s->mp->cnt_[p] > 0x80u) {
          if ((signed char)u < 0) s->cov[CN_ADD_OVER_U] = 1; else s->cov[CN_ADD_OVER_CNT] = 1;
          if (s->level > 1) {
            M_D(s, "V90ConnectionDesigner::adjustConstellationsToNewK !!! reached Max constellation length!!!\r\n");
            if (s->hook_kind == HK_MP && s->hook_k == s->nprint) s->cov[CN_HOOK_MP] = 1;
          }
          b7 = 1;
          mp = s->mp;
          mp->cnt_[p]--;
          for (ph = 0; ph < 6; ph++) {
            unsigned char dl = 0;
            unsigned steps = 0;
            while (*m_la(mp, ph, dl) != saved[ph]) { dl++; if (++steps > 256) { s->hang = 1; return; } }   /* never found (a swapped block at this print): the blob loops forever */
            while (dl != 0) {
              unsigned c = mp->cnt_[ph], bl = 0;
              if (c > 255) { s->hang = 1; return; }   /* the byte counter never reaches the count: endless in the blob */
              s->cov[CN_UNDO_SHIFT] = 1;
              while (bl < c) { *m_la(mp, ph, bl) = *m_la(mp, ph, bl + 1); *m_lb(mp, ph, bl) = *m_lb(mp, ph, bl + 1); bl = (bl + 1) & 0xff; }
              mp->cnt_[ph]--;
              dl--;
            }
          }
          goto after_add;
        }
        mp = s->mp;
        { unsigned c = mp->cnt_[p], k; for (k = c; k != 0; k--) { *m_lb(mp, p, k) = *m_lb(mp, p, k - 1); *m_la(mp, p, k) = *m_la(mp, p, k - 1); } }
        *m_lb(mp, p, 0) = code;
        *m_la(mp, p, 0) = (unsigned char)u;
        { unsigned K2 = m_kint(mp); addc++; curK = K2; mp->w0_ = K2 - mp->w620_ + 6u; }
        iters++;
      } while (target > curK);
      if (iters > 1) s->cov[CN_ADD_MULTI] = 1;
      s->cov[CN_ADD_DONE] = 1;
    }
    M_E(s, "V90ConnectionDesigner: Optimizing new K: nof points Add %d \r\n", (int)addc);
  }
after_add:
  mp = s->mp;
  if (b7) {
    unsigned K1 = m_kint(mp);
    if (K1 == 0) s->cov[CN_K1_ZERO] = 1;
    m_ddfr(s, K1);
    if (curK < K1) { s->hang = 1; return; }   /* the blob reads an uninitialised stack slot: excluded */
    s->cov[CN_REMOVE] = 1;
    mp = s->mp;
    {
      unsigned nrem = 0;
      for (;;) {
        unsigned p = m_pick(mp, 1), c, q;
        unsigned K3;
        for (q = 0; q < 6; q++) if (q != p && *m_la(mp, q, 0) == *m_la(mp, p, 0)) s->cov[CN_TIE_MAX] = 1;
        lastP = p;
        if (mp->cnt_[p] == 0) { s->hang = 1; return; }
        c = --mp->cnt_[p];
        if (c == 0) {
          s->cov[CN_REMOVE_BUG] = 1;
          if (s->level > 1) {
            M_D(s, "V90ConnectionDesigner::adjustConstellationsToNewK - BUG !!! reached constellation length 0!!!\r\n");
            if (s->hook_kind == HK_MP && s->hook_k == s->nprint) s->cov[CN_HOOK_MP] = 1;
            mp = s->mp;
          }
          mp->cnt_[p]++;
          break;
        }
        svB2 = *m_lb(mp, p, 0);
        svA2 = *m_la(mp, p, 0);
        havesv = 1;
        { unsigned i; for (i = 0; ; i++) { *m_lb(mp, p, i) = *m_lb(mp, p, i + 1); *m_la(mp, p, i) = *m_la(mp, p, i + 1); mp = s->mp; if (!(mp->cnt_[p] > i + 1)) break; } }
        K3 = m_kint(mp);
        removedCnt++;
        nrem++;
        mp->w0_ = K3 - mp->w620_ + 6u;
        if (!(K3 >= K1)) break;
        s->cov[CN_REMOVE_LOOP] = 1;
      }
      if (nrem > 1) s->cov[CN_REMOVE_MULTI] = 1;
    }
    {
      unsigned c = mp->cnt_[lastP];
      if (c != 1) {
        unsigned k;
        if (!havesv) { s->hang = 1; return; }   /* restores stale stack bytes: excluded */
        for (k = c; k != 0; k--) { *m_lb(mp, lastP, k) = *m_lb(mp, lastP, k - 1); *m_la(mp, lastP, k) = *m_la(mp, lastP, k - 1); mp = s->mp; }
        mp = s->mp;
        mp->cnt_[lastP]++;
        *m_lb(mp, lastP, 0) = svB2;
        mp = s->mp;
        *m_la(mp, lastP, 0) = svA2;
        removedCnt--;
        s->cov[CN_RESTORE] = 1;
      } else s->cov[CN_NORESTORE_1] = 1;
      M_E(s, "V90ConnectionDesigner: Optimizing new K: nof points removed %d \r\n", removedCnt);
      mp = s->mp;
    }
  }
  {
    LD R = m_kreal_after(mp);
    m_split(R, 100000.0L, R, &sg, &ip, &fp);
    M_E(s, "V90ConnectionDesigner: real K after optimization  = %c%d.%05d\r\n", sg, ip, fp);
  }
  mp = s->mp;
  mp->w0_ = m_kint(mp) - mp->w620_ + 6u;
  {
    float P3 = m_pw(s, s->mp);
    unsigned idx3 = m_pidx(s, P3);
    m_power_print_sqrt(s, "V90ConnectionDesigner: Optimizing new K: new sqrt(power)  = %c%d.%03d\r\n", 0, 0, P3);
    m_power_print_idx(s, "V90ConstellationDesigner: final index %d, final power in dBm0  = %c%d.%01d\r\n", idx3);
    M_E(s, "---------------------------------------------------------------\r\n");
  }
}

/* ------------------------------------------------------------------------------------------------ coverage names */
static const char *const cov_names[CF_LAST] = {
  [CP_ENTRY_SKIP] = "power: not above the limit (skip)", [CP_ROUND1] = "power: round of 1", [CP_ROUND10] = "power: round of 10", [CP_ROUND20] = "power: round of 20", [CP_FLAG8] = "power: flag8 > 13",
  [CP_BUG] = "power: BUG (empty list)", [CP_K_LE20_REMOVED] = "power: K <= 20 after removals", [CP_K_LE20_NONE] = "power: K <= 20 without removals", [CP_RESTORE] = "power: restore",
  [CP_CONT_LOOP] = "power: another round", [CP_POW_NAN_STOP] = "power: stop (power not above limit)", [CP_LIM22] = "power: limit index 22", [CP_LIM_LOW] = "power: limit index < 5",
  [CP_TIE] = "power: tie of first ucodes", [CP_HOOK_MP_P] = "power: mapParams_ swapped at the BUG print", [CP_HOOK_PAR_P] = "(unused)",
  [CN_ADD] = "newk: add branch", [CN_NOADD] = "newk: no add", [CN_ADD_NOLOOP] = "newk: add, target <= curK", [CN_ADD_SINGLE] = "newk: single-phase scan", [CN_ADD_DUAL] = "newk: dual-phase scan",
  [CN_ADD_DUAL_SKIP] = "newk: dual u0 > 0x71", [CN_ADD_SINGLE_SKIP] = "newk: single u0 >= 0x80", [CN_ADD_OVER_U] = "newk: overflow (u < 0)", [CN_ADD_OVER_CNT] = "newk: overflow (cnt > 0x80)",
  [CN_ADD_DONE] = "newk: additions reached the target", [CN_REMOVE] = "newk: remove loop entered", [CN_REMOVE_LOOP] = "newk: remove loop iterates", [CN_REMOVE_BUG] = "newk: remove BUG (empty list)",
  [CN_RESTORE] = "newk: restore after removal", [CN_NORESTORE_1] = "newk: no restore (count 1)", [CN_K1_ZERO] = "newk: K1 == 0", [CN_ALAW] = "newk: A-law", [CN_ULAW] = "newk: u-law",
  [CN_TIE_MIN] = "newk: tie (add loop)", [CN_TIE_MAX] = "newk: tie (remove loop)", [CN_ADD_THEN_REMOVE] = "(unused)", [CN_UNDO_SHIFT] = "newk: overflow undo shifts", [CN_KREAL_ZERO] = "newk: real K == 0",
  [CN_UPR_EQ] = "newk: Kreal - K0 == upRoundK", [CN_UPR_NAN] = "newk: upRoundK NaN", [CN_HOOK_MP] = "newk: mapParams_ swapped at a BUG / overflow print", [CN_HOOK_PAR] = "(unused)",
  [CN_ADD_MULTI] = "newk: several additions", [CN_REMOVE_MULTI] = "newk: several removals", [CN_DUAL_BREAK_A] = "newk: dual break (A and B)", [CN_DUAL_BREAK_B] = "newk: dual break",
  [CN_DUAL_RUNOUT] = "newk: dual scan ran out", [CN_SINGLE_BREAK] = "newk: single break", [CN_SINGLE_RUNOUT] = "newk: single scan ran out", [CN_PAR_AFTER] = "(unused)",
};
static int cov_id(int i) { return i < H_COV_STRADDLE ? i : i + 2; }   /* ids 60 / 61 are reserved by the harness library */
static void cov_define_range(int from, int to) {
  int i;
  for (i = from; i < to; i++) if (cov_names[i]) h_cov_define(cov_id(i), cov_names[i]);
}
static void cov_hit_model(const MS *s) {
  int i;
  for (i = 0; i < CF_LAST; i++) if (s->cov[i]) h_cov_hit(cov_id(i));
}
enum { C_TRIALS = 90, C_SKIPPED, C_L0, C_L1, C_L2, C_LHI, C_CHAIN, C_ALT_HOOK_MP, C_ALT_HOOK_PAR, C_HOOK_LEVEL, C_POWER, C_NEWK };
static void cov_define_trials(void) {
  h_cov_define(C_TRIALS, "trials"); h_cov_define(C_SKIPPED, "skipped (excluded input)"); h_cov_define(C_L0, "level 0"); h_cov_define(C_L1, "level 1");
  h_cov_define(C_L2, "level 2 (prints)"); h_cov_define(C_LHI, "level >= 2^31"); h_cov_define(C_CHAIN, "second call on the same object");
  h_cov_define(C_ALT_HOOK_MP, "mapParams_ hook"); h_cov_define(C_ALT_HOOK_PAR, "params_ hook"); h_cov_define(C_HOOK_LEVEL, "level hook");
  h_cov_define(C_POWER, "adjustConstellationsPower calls"); h_cov_define(C_NEWK, "adjustConstellationsToNewK calls");
}

/* ------------------------------------------------------------------------------------------------ the objects of a trial */
#define PADS 64
typedef struct {
  short *Abuf, *Bbuf;            /* PADS shorts of slack in front, 768 + 256 behind */
  unsigned char *Vbuf;
  short *A, *B;
  unsigned char *V;
  short *dual;                   /* 8 shorts */
  MapPar90 *mp, *altmp;
  unsigned char *adid;           /* scenario 7: the 0x3000-byte block the real determineDminForRrn reads through adid_ */
} Env;
#define ARR_SHORTS (PADS + 768 + 256)
static CdObj g_t;
static CpObj g_pow;
static V90ParObj g_par, g_altpar;
static MS g_ms;
static unsigned g_nskipped;

static void env_new(Env *e) {
  e->Abuf = (short *)h_obj_new(ARR_SHORTS * 2);
  e->Bbuf = (short *)h_obj_new(ARR_SHORTS * 2);
  e->Vbuf = (unsigned char *)h_obj_new(ARR_SHORTS);
  e->A = e->Abuf + PADS; e->B = e->Bbuf + PADS; e->V = e->Vbuf + PADS;
  e->dual = (short *)h_obj_new(16);
  e->mp = (MapPar90 *)h_obj_new(sizeof(MapPar90) + 16);
  e->altmp = (MapPar90 *)h_obj_new(sizeof(MapPar90) + 16);
  e->adid = g_real_ddfr ? (unsigned char *)h_obj_new(0x3000) : NULL;
}

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
static unsigned long g_dumpctr;
static void dump_all(const char *tag, const CdObj *t, const MapPar90 *mp, const MapPar90 *amp) {
  CdObj tmp;
  CpObj ptmp;
  int show = h_sample(&g_dumpctr, 40, 97);
  memcpy(&tmp, t, sizeof tmp);
  tmp.params_ = NULL; tmp.mapParams_ = NULL; tmp.adid_ = NULL; tmp.power_ = NULL; tmp.preFilter_ = NULL;
  h_dump_obj(show, tag, &tmp, f_CdObj, NF_CdObj);
  h_dump_obj(show, "mp", mp, f_mp, NF_mp);
  if (amp) h_dump_obj(show, "alt", amp, f_mp, NF_mp);
  memcpy(&ptmp, &g_pow, sizeof ptmp);
  ptmp.tab_ = NULL;
  h_dump_obj(show, "pow", &ptmp, f_CpObj, NF_CpObj);
}

typedef struct {
  int fn;                      /* 0 = adjustConstellationsPower, 1 = adjustConstellationsToNewK */
  unsigned level;
  int hook_kind;
  unsigned hook_k, hook_level;
  const char *hook_text;       /* steering: place the hook after the first print whose text starts with this (found by a model run without the hook) */
} Call;

static unsigned char g_sA[ARR_SHORTS * 2], g_sB[ARR_SHORTS * 2], g_sV[ARR_SHORTS];
static void run_model(const Call *c, Env *e) {
  memset(&g_ms, 0, sizeof g_ms);
  g_ms.A = e->A; g_ms.B = e->B; g_ms.dual = e->dual;
  g_cur_adid = e->adid;
  memcpy(&g_ms.t, &g_t, sizeof g_ms.t);
  g_ms.par = &g_par; g_ms.altpar = &g_altpar;
  memcpy(&g_ms.mpm, e->mp, sizeof g_ms.mpm); memcpy(&g_ms.altm, e->altmp, sizeof g_ms.altm);
  g_ms.mp = &g_ms.mpm;
  memcpy(&g_ms.pow, &g_pow, sizeof g_ms.pow);
  g_ms.level = c->level; g_ms.hook_kind = c->hook_kind; g_ms.hook_k = c->hook_k; g_ms.hook_level = c->hook_level;
  if (c->fn) m_newk(&g_ms); else m_power(&g_ms);
}

static int do_call(const Call *c0, Env *e, int emit, int chain) {
  unsigned i;
  CdObj tm, tr;
  CpObj pm, pr;
  Call cl = *c0;
  const Call *c = &cl;
  memcpy(g_sA, e->Abuf, sizeof g_sA); memcpy(g_sB, e->Bbuf, sizeof g_sB); memcpy(g_sV, e->Vbuf, sizeof g_sV);
  if (cl.hook_text) {
    Call nohook = cl;
    nohook.hook_kind = HK_NONE; nohook.hook_k = 0;
    run_model(&nohook, e);
    cl.hook_kind = c0->hook_kind;
    cl.hook_k = 0;
    if (!g_ms.hang) {
      unsigned n, np = 0, nd = 0;
      for (n = 0; n < g_ms.nlog && n < LOGMAX; n++) {
        if (g_ms.log[n].kind != 'r') np++;
        if (g_ms.log[n].kind == 'd') nd++;
        if (g_ms.log[n].kind != 'r' && !strncmp(g_ms.log[n].text, cl.hook_text, strlen(cl.hook_text))) {
          if (cl.hook_kind == HK_LEVEL && g_ms.log[n].kind != 'd') continue;   /* the level hook counts debug prints only */
          cl.hook_k = cl.hook_kind == HK_LEVEL ? nd : np;
          break;
        }
      }
    }
    if (!cl.hook_k) cl.hook_kind = HK_NONE;
  }
  run_model(c, e);
  if (g_ms.hang) { g_nskipped++; h_cov_hit(C_SKIPPED); return 0; }
  if (getenv("P23_TRACE")) fprintf(stderr, "call fn=%d lvl=%u hook=%d/%u useAlaw=%u w38=%u flag8=%u w620=%u cnt=%u,%u,%u,%u,%u,%u\n", c->fn, c->level, cl.hook_kind, cl.hook_k, g_t.useAlaw_, g_t.w38_, g_t.flag8_, e->mp->w620_,
                                  e->mp->cnt_[0], e->mp->cnt_[1], e->mp->cnt_[2], e->mp->cnt_[3], e->mp->cnt_[4], e->mp->cnt_[5]);
  g_t.params_ = &g_par; g_t.mapParams_ = e->mp; g_t.power_ = &g_pow; g_t.adid_ = e->adid; g_cur_adid = e->adid;
  g_alt_par = &g_altpar; g_alt_mp = e->altmp;
  g_snap_t = &g_t; g_snap_mp = e->mp;
  g_hook_kind = c->hook_kind; g_hook_k = c->hook_k; g_hook_level = c->hook_level;
  dsplibs_debug_level = c->level;
  log_begin();
  if (c->fn) cd_newk(&g_t, (short (*)[128])e->A, (short (*)[128])e->B, e->dual, (unsigned char (*)[128])e->V);
  else cd_power(&g_t);
  g_hook_k = 0;
  g_snap_t = NULL; g_snap_mp = NULL;
  memcpy(&tm, &g_ms.t, sizeof tm); memcpy(&tr, &g_t, sizeof tr);
  tm.params_ = tm.mapParams_ = tr.params_ = tr.mapParams_ = NULL;
  tm.power_ = tr.power_ = NULL;
  if (memcmp(&tm, &tr, sizeof tm)) {
    for (i = 0; i < sizeof tm; i++) if (((uint8_t *)&tm)[i] != ((uint8_t *)&tr)[i]) h_die3("object byte +0x%x: real 0x%02x model 0x%02x (fn %d level %u)", i, ((uint8_t *)&tr)[i], ((uint8_t *)&tm)[i], c->fn, c->level);
  }
  if ((g_t.params_ == (void *)&g_altpar) != (g_ms.par == &g_altpar)) h_die3("params_ pointer after the call");
  if ((g_t.mapParams_ == (void *)e->altmp) != (g_ms.mp == &g_ms.altm)) h_die3("mapParams_ pointer after the call");
  for (i = 0; i < sizeof(MapPar90); i++) {
    if (((uint8_t *)e->mp)[i] != ((uint8_t *)&g_ms.mpm)[i]) h_die3("mapping block byte +0x%x: real 0x%02x model 0x%02x (fn %d)", i, ((uint8_t *)e->mp)[i], ((uint8_t *)&g_ms.mpm)[i], c->fn);
    if (((uint8_t *)e->altmp)[i] != ((uint8_t *)&g_ms.altm)[i]) h_die3("alt mapping block byte +0x%x: real 0x%02x model 0x%02x (fn %d level %u hook %d/%u text %s; real mp %s alt)", i, ((uint8_t *)e->altmp)[i], ((uint8_t *)&g_ms.altm)[i], c->fn, c->level, cl.hook_kind, cl.hook_k, cl.hook_text ? cl.hook_text : "-", g_t.mapParams_ == (void *)e->altmp ? "is" : "is not");
  }
  if (g_nlog != g_ms.nlog) {
    for (i = 0; i < g_nlog && i < 14; i++) fprintf(stderr, "real %u %c %s\n", i, g_log[i].kind, g_log[i].text);
    for (i = 0; i < g_ms.nlog && i < 14; i++) fprintf(stderr, "model %u %c %s\n", i, g_ms.log[i].kind, g_ms.log[i].text);
    h_die3("print count: real %u model %u (fn %d level %u hook %d/%u)", g_nlog, g_ms.nlog, c->fn, c->level, cl.hook_kind, cl.hook_k);
  }
  for (i = 0; i < g_nlog && i < LOGMAX; i++)
    if (g_log[i].kind != g_ms.log[i].kind || strcmp(g_log[i].text, g_ms.log[i].text))
      h_die3("print %u differs (fn %d level %u hook %d/%u):\n  real  %c %s\n  model %c %s", i, c->fn, c->level, cl.hook_kind, cl.hook_k, g_log[i].kind, g_log[i].text, g_ms.log[i].kind, g_ms.log[i].text);
  memcpy(&pm, &g_ms.pow, sizeof pm); memcpy(&pr, &g_pow, sizeof pr);
  pm.tab_ = pr.tab_ = NULL;
  if (memcmp(&pm, &pr, sizeof pm)) h_die3("the power object differs after the call (fn %d): the last getPower call had another block / argument", c->fn);
  if (memcmp(g_sA, e->Abuf, sizeof g_sA) || memcmp(g_sB, e->Bbuf, sizeof g_sB) || memcmp(g_sV, e->Vbuf, sizeof g_sV)) h_die3("an input table was modified");
  cov_hit_model(&g_ms);
  h_cov_hit(C_TRIALS);
  h_cov_hit(c->fn ? C_NEWK : C_POWER);
  if (c->level == 0) h_cov_hit(C_L0); else if (c->level == 1) h_cov_hit(C_L1); else if (c->level >= 0x80000000u) h_cov_hit(C_LHI); else h_cov_hit(C_L2);
  if (chain) h_cov_hit(C_CHAIN);
  if (cl.hook_kind == HK_MP && g_ms.nprint >= cl.hook_k) h_cov_hit(C_ALT_HOOK_MP);
  if (cl.hook_kind == HK_PAR && g_ms.nprint >= cl.hook_k) h_cov_hit(C_ALT_HOOK_PAR);
  if (cl.hook_kind == HK_LEVEL && g_ms.nd >= cl.hook_k) h_cov_hit(C_HOOK_LEVEL);
  if (emit) {
    h_emit(0, "call fn %d prints %u", c->fn, g_nlog);
    dump_all("T", &g_t, e->mp, cl.hook_kind == HK_MP ? e->altmp : NULL);
  }
  return 1;
}

/* ------------------------------------------------------------------------------------------------ generators */
/* a mapping block: random garbage everywhere, then the structure.  regime: 0 nominal, 1 small counts, 2 large counts (up to 128), 3 counts 1 / 2, 4 tiny K (counts <= 3) */
static void gen_block(MapPar90 *mp, int regime) {
  unsigned k, ph, i;
  unsigned cap = 3 << h_rmod(6);
  int high = h_rpct(10), edge = h_rpct(6);
  unsigned etop = (unsigned)(h_rpct(50) ? 0x7f + h_rmod(3) : 0xfd + h_rmod(3));   /* edge block: every phase starts at the same ucode next to the signed-byte limit */
  for (k = 0; k < sizeof(MapPar90); k += 4) { uint32_t w = h_rnd(); memcpy((uint8_t *)mp + k, &w, 4); }
  mp->w620_ = h_rpct(70) ? h_rmod(12) : h_rmod(34);
  for (ph = 0; ph < 6; ph++) {
    unsigned cnt, top, lo = h_rmod(60), hi;
    unsigned char *la = (unsigned char *)mp + 4 + ph * 0x80, *lb = (unsigned char *)mp + 0x304 + ph * 0x80;
    switch (regime) {
      case 1: cnt = 1 + h_rmod(4); break;
      case 2: cnt = h_rpct(60) ? 90 + h_rmod(39) : 1 + h_rmod(cap); break;
      case 3: cnt = 1 + h_rmod(2); break;
      case 4: cnt = 1 + h_rmod(3); break;
      default: cnt = 1 + h_rmod(cap > 127 ? 127 : cap); break;
    }
    if (cnt > 128) cnt = 128;
    mp->cnt_[ph] = cnt;
    top = h_rpct(35) ? 100 + h_rmod(4) : h_rpct(15) ? h_rmod(256) : (cnt - 1) + h_rmod(129 - cnt);
    if (edge) top = etop;
    else if (high) top = h_rpct(35) ? (unsigned)(h_rpct(50) ? 0x7f + h_rmod(3) : 0xfd + h_rmod(3)) : 0x6c + h_rmod(0x94);
    for (i = 0; i < cnt; i++) {
      la[i] = (unsigned char)top;
      if (top > 2 && h_rpct(85)) top -= 1 + h_rmod(3);
    }
    hi = lo + 1 + h_rmod(128 - lo);
    for (i = 0; i < 128; i++) lb[i] = (unsigned char)(h_rpct(90) ? lo + h_rmod(hi - lo) : h_rnd());
  }
}
static void gen_par(V90ParObj *p, float up) {
  memset(p, 0, sizeof *p);
  p->unk360 = 0; p->useRestricedDmin = 0; p->forcedDmin = -1; p->rateForce = 0;
  p->upRoundK = up;
}
static float pick_up(const MapPar90 *mp) {
  /* around the fractional part of the real K (the add branch decision), plus special values */
  unsigned r = h_rmod(100);
  float kb = m_kreal_before(mp);
  LD fr = (LD)kb - (LD)m_kint(mp);
  float f = (float)fr;
  uint32_t b;
  if (r < 12) { static const uint32_t sp[] = { 0x00000000u, 0x80000000u, 0x7f800000u, 0xff800000u, 0x7fc00000u, 0x3f800000u, 0xbf800000u, 0x7f7fffffu }; return F(sp[h_rmod(8)]); }
  if (r < 40) { memcpy(&b, &f, 4); b += (uint32_t)h_rrange(-2, 2); return F(b); }
  if (r < 58) return F(0x3dcccccdu) * (float)h_rmod(11);
  if (r < 75) return (float)((int)h_rrange(-8, 8)) / 8.0f;
  return h_rpct(50) ? 0.0f : 0.5f;
}
static void gen_obj(Env *e, int regime) {
  int alaw = h_rpct(50), jag = h_rpct(40) ? 0 : (int)h_rmod(60), ph;
  unsigned k;
  t_prefill(&g_t);
  cp_c1(&g_pow);
  g_t.useAlaw_ = (uint32_t)(alaw ? (h_rpct(90) ? 1 : (int)h_rnd() | 1) : 0);
  g_t.flag8_ = (uint8_t)(h_rpct(80) ? h_rmod(14) : 14 + h_rmod(242));
  g_t.w38_ = (uint8_t)(h_rpct(70) ? h_rmod(23) : h_rpct(60) ? 22 + h_rmod(3) : (int)h_rnd());
  g_t.dMin_ = (uint16_t)(h_rpct(60) ? (int)h_rrange(0, 120) : h_rpct(60) ? (int)h_rrange(-40, 400) : (int)h_rnd());
  g_t.dMinAlt_ = (uint16_t)(h_rpct(60) ? (int)h_rrange(0, 150) : h_rpct(60) ? (int)h_rrange(-40, 500) : (int)h_rnd());
  gen_block(e->mp, regime);
  if (h_rpct(4)) {   /* a block whose u-law codes are all 0 (decoded 0xff = 0): power exactly 0 (the sign of a zero in the prints) */
    unsigned q;
    g_t.useAlaw_ = 0;
    for (q = 0; q < 6; q++) memset((unsigned char *)e->mp + 0x304 + q * 0x80, 0x00, 0x80);
  }
  gen_block(e->altmp, h_rpct(40) ? 3 : h_rpct(65) ? 1 : 0);
  if (h_rpct(25)) {   /* the alternate block starts as a copy of the main one (a swap in the middle of an add / remove sequence then finds consistent saved ucodes) */
    memcpy(e->altmp, e->mp, sizeof(MapPar90));
    if (h_rpct(40)) e->altmp->w620_ = h_rmod(30);
    if (h_rpct(30)) e->altmp->cnt_[h_rmod(6)] += 1 + h_rmod(3);
  }
  gen_par(&g_par, pick_up(e->mp));
  gen_par(&g_altpar, pick_up(e->altmp));
  fill_table(e->A, alaw, jag, 1);
  fill_table(e->B, h_rpct(80) ? alaw : !alaw, jag, h_rpct(80) ? 1 : 2);
  if (h_rpct(25)) for (k = 0; k < 768; k++) e->A[k] = (short)(e->A[k] > 0 ? e->A[k] : -e->A[k]);
  if (h_rpct(12)) {   /* flat distance tables: the scans never reach their threshold and run out */
    short lv = (short)h_rrange(-50, 300);
    for (k = 0; k < 768; k++) e->A[k] = (short)(lv + (h_rpct(10) ? (int)h_rrange(-2, 2) : 0));
    g_t.dMin_ = (uint16_t)(1 + h_rmod(60)); g_t.dMinAlt_ = (uint16_t)(1 + h_rmod(60));
  }
  for (k = 0; k < 768 + 256; k++) e->V[k] = (unsigned char)h_rnd();
  for (ph = 0; ph < 8; ph++) e->dual[ph] = (short)(h_rpct(40) ? (h_rpct(60) ? 1 : (int)(short)h_rnd() | 1) : 0);
}
static const char *g_hook_bias;   /* a scenario's favourite hook text */
static const char *g_hook_bias_list[10];   /* ... or several (one of them at random) */
static unsigned g_hook_nbias;
static void pick_hook(Call *c, unsigned maxk) {
  c->hook_kind = HK_NONE; c->hook_k = 0; c->hook_level = 0; c->hook_text = NULL;
  if (h_rpct(14)) {
    unsigned r = h_rmod(3);
    c->hook_kind = r == 0 ? HK_LEVEL : r == 1 ? HK_PAR : HK_MP;
    c->hook_k = 1 + h_rmod(maxk);
    c->hook_level = h_rpct(60) ? 0u : 1u;
  }
  if (h_rpct(30)) {   /* steering: just after a print the code reloads / re-reads after */
    static const char *const tx[] = { "V90ConnectionDesigner: adjustConstellationPower", "V90ConstellationDesigner::adjustConstellationsPower restoring", "V90ConstellationDesigner::adjustConstellationsPower (pParams",
      "V90ConnectionDesigner: real K before", "V90ConnectionDesigner::adjustConstellationsToNewK !!!", "V90ConnectionDesigner::adjustConstellationsToNewK - BUG", "V90ConnectionDesigner: Optimizing new K: nof points Add",
      "V90ConnectionDesigner: Optimizing new K: nof points removed", "V90ConnectionDesigner: real K after", "V90ConstellationDesigner: initial sqrt", "V90ConnectionDesigner: redundant", "V90ConstellationDesigner:\\r" };
    c->hook_text = tx[h_rmod(sizeof tx / sizeof tx[0])];
    c->hook_kind = h_rpct(60) ? HK_MP : h_rpct(50) ? HK_PAR : HK_LEVEL;
    c->hook_level = h_rpct(60) ? 0u : 1u;
  }
  if ((g_hook_bias || g_hook_nbias) && h_rpct(45)) {
    c->hook_text = g_hook_nbias ? g_hook_bias_list[h_rmod(g_hook_nbias)] : g_hook_bias;
    c->hook_kind = h_rpct(60) ? HK_MP : h_rpct(50) ? HK_PAR : HK_LEVEL;
    c->hook_level = h_rpct(60) ? 0u : 1u;
  }
}

/* the remove loop reaching the BUG stop on its SECOND iteration: phase X (large count, highest first ucode) loses a point and keeps K, then phase Y (count 1, next highest first ucode) is empty */
static void shape_remove_bug(Env *e) {
  unsigned x = h_rmod(6), y = (x + 1 + h_rmod(5)) % 6, ph, i;
  MapPar90 *mp = e->mp;
  unsigned char *lx = (unsigned char *)mp + 4 + x * 0x80, *ly = (unsigned char *)mp + 4 + y * 0x80;
  mp->cnt_[x] = 6 + h_rmod(60);
  mp->cnt_[y] = 1;
  for (ph = 0; ph < 6; ph++) if (ph != x && ph != y) {
    unsigned char *l = (unsigned char *)mp + 4 + ph * 0x80;
    unsigned top = h_rmod(100);
    for (i = 0; i < mp->cnt_[ph]; i++) { l[i] = (unsigned char)top; if (top > 2) top -= 1 + h_rmod(2); }
  }
  { unsigned top = 126; for (i = 0; i < mp->cnt_[x]; i++) { lx[i] = (unsigned char)top; if (top > 2) top -= 1 + h_rmod(2); } }
  ly[0] = 125;
  g_par.upRoundK = 3.0f;   /* no add branch */
}

/* ------------------------------------------------------------------------------------------------ scenarios */
static void sc_common(void) {
  h_scenario_begin(1);
  relax_reserved();
  cov_define_trials();
  h_covreq[C_SKIPPED] = 0; h_covreq[C_CHAIN] = 0;
}
static void sc_power(int regime, unsigned ntrials) {
  unsigned trial;
  sc_common();
  g_hook_bias = regime == 1 ? "V90ConnectionDesigner: adjustConstellationPower" : NULL;
  cov_define_range(0, CN_ADD);
  h_covreq[C_NEWK] = 0;
  h_covreq[cov_id(CP_HOOK_PAR_P)] = 0;
  for (trial = 0; trial < ntrials; trial++) {
    Env e;
    Call c;
    env_new(&e);
    gen_obj(&e, regime);
    c.fn = 0;
    c.level = pick_level();
    pick_hook(&c, 20);
    do_call(&c, &e, 1, 0);
    h_trial_end("power");
  }
}
static void sc_newk(int regime, unsigned ntrials) {
  unsigned trial;
  sc_common();
  cov_define_range(CN_ADD, CF_LAST);
  h_covreq[C_POWER] = 0;
  g_hook_bias = regime == 3 ? "V90ConnectionDesigner::adjustConstellationsToNewK - BUG" : regime == 2 ? "V90ConnectionDesigner::adjustConstellationsToNewK !!!" : NULL;
  if (regime == 0 || regime == 2) {
    g_hook_bias_list[0] = "V90ConnectionDesigner: real K before"; g_hook_bias_list[1] = "V90ConnectionDesigner: redundant"; g_hook_bias_list[2] = "V90ConnectionDesigner: Optimizing new K: nof points Add";
    g_hook_bias_list[3] = "V90ConnectionDesigner: Optimizing new K: nof points removed"; g_hook_bias_list[4] = "V90ConnectionDesigner: real K after"; g_hook_bias_list[5] = "V90ConnectionDesigner::adjustConstellationsToNewK !!!";
    g_hook_bias_list[6] = "V90ConnectionDesigner::adjustConstellationsToNewK - BUG"; g_hook_bias_list[7] = "V90ConnectionDesigner: Optimizing new K: new sqrt"; g_hook_nbias = 8; g_hook_bias = NULL;
  }
  if (regime == 3) { h_covreq[cov_id(CN_HOOK_MP)] = 0; h_covreq[cov_id(CN_RESTORE)] = 0; h_covreq[cov_id(CN_ADD_OVER_CNT)] = 0; h_covreq[cov_id(CN_REMOVE_MULTI)] = 0; h_covreq[cov_id(CN_UNDO_SHIFT)] = 0; h_covreq[cov_id(CN_REMOVE_LOOP)] = 0; }
  h_covreq[cov_id(CN_ADD_THEN_REMOVE)] = h_covreq[cov_id(CN_HOOK_PAR)] = h_covreq[cov_id(CN_PAR_AFTER)] = 0;
  h_covreq[cov_id(CN_ADD_NOLOOP)] = 0;   /* target = K0 + 1 <= curK = K0 is structurally impossible */
  h_covreq[cov_id(CN_K1_ZERO)] = h_covreq[cov_id(CN_KREAL_ZERO)] = 0;   /* a zero count makes getPower divide by zero (#DE) in the blob before K is looked at */
  h_covreq[cov_id(CN_HOOK_MP)] = 0; h_covreq[C_HOOK_LEVEL] = 0;   /* the swap at a BUG print is covered by scenario 6 (a BUG on the first iteration + a swapped block restores stale stack bytes: excluded) */
  if (regime == 0) h_covreq[cov_id(CN_SINGLE_RUNOUT)] = h_covreq[cov_id(CN_DUAL_RUNOUT)] = 0;   /* the table scans ran out only in the large-count regime */
  if (regime != 0) { h_covreq[cov_id(CN_ADD_MULTI)] = 0; }
  for (trial = 0; trial < ntrials; trial++) {
    Env e;
    Call c;
    env_new(&e);
    gen_obj(&e, regime);
    c.fn = 1;
    c.level = pick_level();
    pick_hook(&c, 24);
    do_call(&c, &e, 1, 0);
    h_trial_end("newk");
  }
}
static void sc_newk_bug2(unsigned ntrials) {
  unsigned trial;
  sc_common();
  cov_define_range(CN_ADD, CF_LAST);
  h_covreq[C_POWER] = 0;
  g_hook_bias = "V90ConnectionDesigner::adjustConstellationsToNewK - BUG";
  g_hook_bias_list[0] = g_hook_bias; g_hook_bias_list[1] = "V90ConnectionDesigner: Optimizing new K: nof points removed"; g_hook_bias_list[2] = "V90ConnectionDesigner: real K after"; g_hook_bias_list[3] = "V90ConnectionDesigner: Optimizing new K: new sqrt";
  g_hook_nbias = 4;
  for (trial = 0; trial < (unsigned)CF_LAST; trial++) h_covreq[cov_id((int)trial)] = 0;
  h_covreq[cov_id(CN_HOOK_MP)] = h_covreq[cov_id(CN_REMOVE_BUG)] = h_covreq[cov_id(CN_REMOVE_LOOP)] = h_covreq[cov_id(CN_RESTORE)] = 1;
  h_covreq[C_HOOK_LEVEL] = 0;
  for (trial = 0; trial < ntrials; trial++) {
    Env e;
    Call c;
    env_new(&e);
    gen_obj(&e, 0);
    shape_remove_bug(&e);
    c.fn = 1;
    c.level = h_rpct(70) ? 2u : pick_level();
    pick_hook(&c, 24);
    do_call(&c, &e, 1, 0);
    h_trial_end("newk_bug2");
  }
}
/* scenario 7 (i386): adjustConstellationsToNewK with the blob's REAL determineDminForRrn behind the hook (the model calls the same function on its own object copy, its debug prints go to the model's log) */
static void sc_real(unsigned ntrials) {
  unsigned trial;
  g_real_ddfr = 1;
  if (!real_ddfr) { h_emit(1, "scenario 7 skipped: the blob's determineDminForRrn is not linked (x86-64)"); return; }
  sc_common();
  cov_define_range(CN_ADD, CF_LAST);
  h_covreq[C_POWER] = 0;
  g_hook_bias = NULL;
  for (trial = 0; trial < (unsigned)CF_LAST; trial++) h_covreq[cov_id((int)trial)] = 0;
  h_covreq[cov_id(CN_ADD)] = h_covreq[cov_id(CN_NOADD)] = h_covreq[cov_id(CN_REMOVE)] = 1;
  h_covreq[C_HOOK_LEVEL] = 0; h_covreq[C_ALT_HOOK_MP] = h_covreq[C_ALT_HOOK_PAR] = 0;
  for (trial = 0; trial < ntrials; trial++) {
    Env e;
    Call c;
    unsigned k;
    env_new(&e);
    gen_obj(&e, h_rpct(60) ? 0 : 2);
    for (k = 0; k < 0x3000; k += 4) { uint32_t w = h_rnd(); if (h_rpct(60)) w &= 0x00ff00ffu; memcpy(e.adid + k, &w, 4); }
    for (k = 0; k < 6; k++) e.adid[0x280c + k] = (unsigned char)(h_rpct(85) ? 0 : h_rnd());   /* the phases the function may take its maximum count from */
    g_par.unk360 = (int32_t)(h_rpct(60) ? h_rmod(8) : h_rmod(60));
    g_altpar.unk360 = (int32_t)h_rmod(60);
    c.fn = 1;
    c.level = pick_level();
    pick_hook(&c, 24);
    if (c.hook_kind == HK_MP) c.hook_kind = HK_LEVEL;   /* a swapped block could break the function's sanity conditions */
    do_call(&c, &e, 1, 0);
    h_trial_end("real_ddfr");
  }
}
/* a chain as constellationDesign makes it: adjustConstellationsPower, then adjustConstellationsToNewK, on the same object and block; sometimes again */
static void sc_chain(unsigned nchains) {
  unsigned trial;
  sc_common();
  cov_define_range(0, CF_LAST);
  for (trial = 0; trial < (unsigned)CF_LAST; trial++) h_covreq[cov_id((int)trial)] = 0;
  h_covreq[C_POWER] = h_covreq[C_NEWK] = 1;
  for (trial = 0; trial < nchains; trial++) {
    Env e;
    Call c;
    unsigned ncalls = 2 + h_rmod(4), k;
    env_new(&e);
    gen_obj(&e, h_rpct(70) ? 0 : 1 + (int)h_rmod(4));
    c.fn = h_rpct(50);
    for (k = 0; k < ncalls; k++) {
      c.level = pick_level();
      pick_hook(&c, 20);
      if (!do_call(&c, &e, 1, k > 0)) break;
      c.fn = !c.fn;
      if (h_rpct(30)) { g_t.w38_ = (uint8_t)h_rmod(23); g_t.flag8_ = (uint8_t)h_rmod(20); }
      if (h_rpct(25)) { g_par.upRoundK = pick_up(e.mp); }
      if (h_rpct(30)) g_t.dMin_ = (uint16_t)h_rrange(0, 120);
      if (h_rpct(25)) { int alaw = h_rpct(50); g_t.useAlaw_ = (uint32_t)alaw; fill_table(e.A, alaw, 20, 1); }
    }
    h_trial_end("chain");
  }
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P23_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P23_DBG")) g_show = 1;
  p06_layout_check();
  p07_layout_check();
  p11_layout_check();
  p24_layout_check();
  h_x87_check();
  h_arena_init();
  switch (sc) {
    case 0: sc_power(0, 3000); break;
    case 1: sc_power(1, 3000); break;
    case 2: sc_newk(0, 3000); break;
    case 3: sc_newk(2, 3000); break;
    case 4: sc_newk(3, 3000); break;
    case 5: sc_chain(1200); break;
    case 6: sc_newk_bug2(1500); break;
    case 7: sc_real(1500); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu edprintf %u dbgprintf %u ddfr %u skipped %u", h_nmalloc, h_nfree, g_nlog_total_e, g_nlog_total_d, g_nlog_total_r, g_nskipped);
  h_cov_finish();
  return 0;
}
