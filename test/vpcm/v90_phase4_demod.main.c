/*
 * Differential test harness for port chunk P38 (src/vpcm/v90_phase4_demod.cpp): the 15 other methods of V90Phase4Demodulator (C1 / C2, D1 / D2, reset, setSessionFlag, enterWaitForCP / MP / Ed,
 * resetRRNDetector, detectRRN, resetBeforRRN, detectFPE, getDecision, trn2dKnownDemod; 1,556 B).  Derived from test/vpcm/v90_phase4_demod_decisions.main.c (the same recorder / model / snapshot design).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_phase4_demod.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2, the
 * port object alone: the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_phase4_demod.blob32.out).  The functions are reached through __asm__ labels; the object is
 * the generated C mirror (test/vpcm/v90_phase4_demod_decisions.layout), the objects it points to are the mirrors of P07 (V90Parameters), P09 (V90MP), P10 (V90CP), P11 (V90MappingParams), P18 (V90AutoDigitalImpDetector): member-wise
 * access only (their layouts differ on x86-64).
 *
 * Every callee is a RECORDER with scripted results and side effects: V90Phase4Modulator C1 / D1 / setSessionFlag / generateSymbol (garbage in the upper 16 bits) / reset / setMappingParams, V90RDetector C1 /
 * D1 / reset / detectR / detectRf (also WRITE the detector's dir_), V90CP::reset / V90MP::reset (also WRITE the object's trn2dD_ and the demodulator's w34fc_: the stores the function does AFTER the call
 * must win), V90Demapper::reset, getV90Decision / getV92Decision (garbage results; may flip session_), linear2alaw / linear2ulaw (garbage in the upper 24 bits), edprintf, dsplibs_debug_printf.
 * What is compared: the result, a hash of the whole demodulator (every field: the pointers as object ids) at every record, every record (callee, arguments, results), the whole object and every foreign object
 * after the call, the canary behind the object.  The INDEPENDENT MODEL (m_*: written from porting/decompiled/dp/p38_phase4_demod_rest.txt in the blob's control-flow shape, not from the port source) predicts the
 * result, the final world and every record; a violation exits 3.
 *
 * Scenarios (each its own process): 0 constructors C1 / C2, 1 destructors D1 / D2, 2 the small methods (setSessionFlag, enterWaitFor*, resetRRNDetector, detectRRN, resetBeforRRN, detectFPE, getDecision,
 * trn2dKnownDemod), 3 reset.
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
#include "v90_mp_layout.h"
#include "v90_cp_layout.h"
#include "v92_cp_layout.h"
#include "v90_adid_layout.h"
#include "v90_phase4_demod_decisions_layout.h"

#define L(m) __asm__(m)

extern unsigned int dsplibs_debug_level;

/* the functions under test */
#define CT_ARGS void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, unsigned
extern void f_c1(CT_ARGS) L("_ZN20V90Phase4DemodulatorC1EP16V90MappingParamsS1_P11V90DemapperP5V90CPP5V90MPP11DescramblerIhiEP22V90ConnectionEvaluatorP13V90ParametersP20V90Phase3DemodulatorP25V90AutoDigitalImpDetectorj");
extern void f_c2(CT_ARGS) L("_ZN20V90Phase4DemodulatorC2EP16V90MappingParamsS1_P11V90DemapperP5V90CPP5V90MPP11DescramblerIhiEP22V90ConnectionEvaluatorP13V90ParametersP20V90Phase3DemodulatorP25V90AutoDigitalImpDetectorj");
extern void f_d1(void *) L("_ZN20V90Phase4DemodulatorD1Ev");
extern void f_d2(void *) L("_ZN20V90Phase4DemodulatorD2Ev");
extern void f_ssf(void *, unsigned) L("_ZN20V90Phase4Demodulator14setSessionFlagEj");
extern void f_ewcp(void *) L("_ZN20V90Phase4Demodulator14enterWaitForCPEv");
extern void f_ewmp(void *) L("_ZN20V90Phase4Demodulator14enterWaitForMPEv");
extern void f_ewed(void *) L("_ZN20V90Phase4Demodulator14enterWaitForEdEv");
extern void f_rrd(void *) L("_ZN20V90Phase4Demodulator16resetRRNDetectorEv");
extern int f_drrn(void *, unsigned) L("_ZN20V90Phase4Demodulator9detectRRNEs");
extern void f_rbr(void *) L("_ZN20V90Phase4Demodulator13resetBeforRRNEv");
extern int f_dfpe(void *, unsigned) L("_ZN20V90Phase4Demodulator9detectFPEEs");
extern short f_t2d(void *, unsigned) L("_ZN20V90Phase4Demodulator15trn2dKnownDemodEs");
extern short f_getd(void *, unsigned) L("_ZN20V90Phase4Demodulator11getDecisionEs");
extern void f_reset(void *, unsigned, unsigned, unsigned, unsigned) L("_ZN20V90Phase4Demodulator5resetEh22Phase4DemodulatorStatejj");

/* the interposed callees */
extern void c_modctor(void *, void *, unsigned, void *, void *, void *, void *, void *, unsigned) L("_ZN18V90Phase4ModulatorC1EP13V90ParametersjP15V90BitsToSymbolP5V90MPP16V90MappingParamsS7_P5V90CPj");
extern void c_moddtor(void *) L("_ZN18V90Phase4ModulatorD1Ev");
extern void c_modssf(void *, unsigned) L("_ZN18V90Phase4Modulator14setSessionFlagEj");
extern int c_modgen(void *) L("_ZN18V90Phase4Modulator14generateSymbolEv");
extern void c_modreset(void *, int, unsigned char, unsigned, unsigned, unsigned) L("_ZN18V90Phase4Modulator5resetE7PcmTypeh20Phase4ModulatorStatejj");
extern void c_modsmp(void *, void *) L("_ZN18V90Phase4Modulator16setMappingParamsEP16V90MappingParams");
extern void c_rdctor(void *, void *) L("_ZN12V90RDetectorC1EP13V90Parameters");
extern void c_rddtor(void *) L("_ZN12V90RDetectorD1Ev");
extern void c_rdreset(void *, unsigned, unsigned) L("_ZN12V90RDetector5resetEjj");
extern int c_dR(void *, short) L("_ZN12V90RDetector7detectREs");
extern int c_dRf(void *, short) L("_ZN12V90RDetector8detectRfEs");
extern void c_cpreset(void *) L("_ZN5V90CP5resetEv");
extern void c_mpreset(void *) L("_ZN5V90MP5resetEv");
extern void c_demreset(void *, void *) L("_ZN11V90Demapper5resetEP16V90MappingParams");
extern short c_get90(void *, short) L("_ZN20V90Phase4Demodulator14getV90DecisionEs");
extern short c_get92(void *, short) L("_ZN20V90Phase4Demodulator14getV92DecisionEs");
extern int c_a2l(int) L("linear2alaw");
extern int c_u2l(int) L("linear2ulaw");

typedef Ph4DemObj T;
#define SZ sizeof(T)
enum { CAN = 64, NSC = 24, LOGMAX = 512, TXTMAX = 640 };

/* ------------------------------------------------------------------------------------------------ the world */
enum { ID_DEM, ID_MP, ID_CP, ID_PAR, ID_ADID, ID_MP1, ID_MP2, ID_DESCR, ID_CE, ID_P3, ID_MOD, ID_R, ID_RF, NID, ID_NULL = -1, ID_WILD = 99 };
static const char *const idname[NID] = { "dem", "mp", "cp", "par", "adid", "mp1", "mp2", "descr", "ce", "p3", "mod", "r", "rf" };
typedef struct { T *t; MpObj *mp; CpObj *cp; V90ParObj *par; AdidObj *adid; MapPar90 *mp1, *mp2; } W;
static T g_mt;               /* the model's copies */
static MpObj g_mmp;
static CpObj g_mcp;
static V90ParObj g_mpar;
static AdidObj g_madid;
static MapPar90 g_mmp1, g_mmp2;
static W g_iw, g_mw;         /* impl world / model world */
static uint8_t *g_can;
static void *g_obj[NID];     /* the impl objects by id (the pointers every object holds point to THESE, also in the model's copies) */
static void *g_fake[4];      /* dem, descr, ce, p3: opaque heap objects */

static int pid(const void *p) {
  int k;
  if (!p) return ID_NULL;
  for (k = 0; k < NID; k++) if (g_obj[k] == p) return k;
  return ID_WILD;
}
static const char *idn(int id) { static char b[4][16]; static int n; char *o = b[n++ & 3]; if (id == ID_NULL) return "null"; if (id == ID_WILD) return "wild"; if (id < 0 || id >= NID) { snprintf(o, 16, "bad%d", id); return o; } return idname[id]; }

/* ------------------------------------------------------------------------------------------------ scripts, cursors, records */
typedef struct {
  uint32_t sym[NSC];           /* generateSymbol results: the low 16 bits are the symbol, the upper 16 garbage */
  uint32_t dR[NSC], dRf[NSC];  /* the detectors' results (the whole eax is tested) */
  uint32_t ddir[NSC];          /* the polarity the detector leaves in dir_ */
  uint32_t a2[NSC], u2[NSC];   /* linear2alaw / linear2ulaw results (garbage above the low byte) */
  uint32_t dec[NSC];           /* getV9xDecision results (garbage above the low 16 bits) */
  uint32_t flip[NSC];          /* getV9xDecision: 0 leave session_, 1 write flipv[] to it */
  uint32_t flipv[NSC];
  uint32_t s1[NSC], s2[NSC];   /* the words the CP / MP reset write into trn2dD_ and into w34fc_ */
} Script;
typedef struct { unsigned sym, dR, dRf, a2, u2, dec, rs; } Cur;
typedef struct { char t[TXTMAX]; } Rec;
typedef struct { Rec r[LOGMAX]; unsigned n; unsigned long total; } Log;
static Script g_sc;
static Cur g_ic, g_mc;
static Log g_il, g_ml;
static int g_show;
static unsigned long g_symctr;

static uint32_t fnv_upd(uint32_t h, const void *p, size_t n) { const uint8_t *q = (const uint8_t *)p; while (n--) { h ^= *q++; h *= 16777619u; } return h; }
static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
/* a hash of the whole demodulator: every scalar / byte field, the pointer fields as object ids (arch independent) */
static uint32_t obj_hash(const T *t) {
  uint32_t h = 2166136261u;
  int i;
  for (i = 0; i < NF_Ph4DemObj; i++) {
    const HFld *f = &f_Ph4DemObj[i];
    unsigned n = f->count ? f->count : 1, es = 4;
    const uint8_t *q = (const uint8_t *)t + f->off;
    switch (f->kind) {
      case 'p': case 'o': { void *v; int id; memcpy(&v, q, sizeof v); id = pid(v); h = fnv_upd(h, &id, sizeof id); continue; }
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'd': es = 8; break;
      default: es = 4; break;
    }
    h = fnv_upd(h, q, (size_t)n * es);
  }
  return h;
}
static void snapshot(char *out, size_t cap, const W *w) {
  const T *t = w->t;
  snprintf(out, cap, "|h=%08x sess=%u st=%u cnt=%u ev=%u lim=%u w34=%u w34fc=%08x mpd=%08x cpd=%08x", obj_hash(t), (unsigned)t->session_, (unsigned)t->state_, (unsigned)t->cnt_, (unsigned)t->event_,
           (unsigned)t->limit_, (unsigned)t->w34_, (unsigned)t->w34fc_, (unsigned)w->mp->trn2dD_, (unsigned)w->cp->trn2dD_);
}
static void rec_add(Log *lg, const W *w, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2], snap[TXTMAX * 2];
  va_list ap;
  Rec *r = &lg->r[lg->n < LOGMAX ? lg->n : LOGMAX - 1];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  snapshot(snap, sizeof snap, w);
  r->t[0] = kind; r->t[1] = ' '; r->t[2] = 0;
  esc_cat(r->t, sizeof r->t, buf);
  if (strlen(r->t) + strlen(snap) + 1 >= sizeof r->t) h_die3("record text does not fit (%zu + %zu)", strlen(r->t), strlen(snap));
  strcat(r->t, snap);
  lg->n++; lg->total++;
  if (lg->n > LOGMAX) h_die3("more than %d records in one call", LOGMAX);
}
#define REC(w, lg, kind, ...) rec_add((lg), (w), (kind), __VA_ARGS__)

/* ------------------------------------------------------------------------------------------------ coverage: per-site counters in their own section */
typedef struct { unsigned long n; unsigned char hit, grp; const char *name; } CovSite;
#define CV(g, nm) do { static CovSite cs_ __attribute__((section("covsites"), used, aligned(sizeof(void *)))) = { 0, 0, (g), (nm) }; cs_.hit = 1; } while (0)
extern CovSite __start_covsites[], __stop_covsites[];
static void cov_flush(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) if (c->hit) { c->n++; c->hit = 0; } }
static void cov_clear(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) c->hit = 0; }
enum { G_CTOR = 0, G_DTOR = 1, G_SMALL = 2, G_RESET = 3 };

/* ------------------------------------------------------------------------------------------------ the environment (callee semantics: shared by the impl's recorders and the model) */
#define CHECK(p, what) do { if ((p) != (what)) h_die3("callee %s got an unexpected object %p (expected %p)", __func__, (void *)(p), (void *)(what)); } while (0)

static void cb_modctor(W *w, Log *l, unsigned flag, int params, int b2s, int mp, int dataMp, int trn2dMp, int cp, unsigned a8) {
  REC(w, l, 'c', "mod.ctor(params=%s flag=%u b2s=%s mp=%s dataMp=%s trn2dMp=%s cp=%s a8=%u)", idn(params), flag, idn(b2s), idn(mp), idn(dataMp), idn(trn2dMp), idn(cp), a8);
  w->t->mod_flag_ = 0xC0DE0000u | (flag & 0xffffu);   /* the callee's own stores: the caller must not undo them */
  w->t->mod_state_ = 0xC0DE0001u;
}
static void cb_moddtor(W *w, Log *l) { REC(w, l, 'c', "mod.dtor"); w->t->mod_flag_ = 0xDEAD0000u; }
static void cb_modssf(W *w, Log *l, unsigned f) { REC(w, l, 'c', "mod.setSessionFlag(%u)", f); w->t->mod_flag_ = f; }
static int cb_modgen(W *w, Cur *c, Log *l) { int r = (int)g_sc.sym[c->sym++ % NSC]; REC(w, l, 'c', "mod.generateSymbol -> %d", (int)(int16_t)r); return r; }
static void cb_modreset(W *w, Log *l, int pcm, unsigned code, unsigned st, unsigned n, unsigned limit) { REC(w, l, 'c', "mod.reset(pcm=%d code=%u st=%u n=%u limit=%u)", pcm, code & 0xffu, st, n, limit); w->t->mod_state_ = st; }
static void cb_modsmp(W *w, Log *l, int mp) { REC(w, l, 'c', "mod.setMappingParams(%s)", idn(mp)); }
static void cb_rdctor(W *w, Log *l, int which, int params) { REC(w, l, 'c', "%s.ctor(%s)", idn(which), idn(params)); if (which == ID_R) w->t->r_cnt_ = 0xA1A10001u; else w->t->rf_cnt_ = 0xA1A10002u; }
static void cb_rddtor(W *w, Log *l, int which) { REC(w, l, 'c', "%s.dtor", idn(which)); if (which == ID_R) w->t->r_cnt_ = 0xDEAD0001u; else w->t->rf_cnt_ = 0xDEAD0002u; }
static void cb_rdreset(W *w, Log *l, int which, unsigned a, unsigned b) { REC(w, l, 'c', "%s.reset(%u,%u)", idn(which), a, b); }
/* which: ID_R detectR, ID_RF detectRf: the detector leaves its polarity in dir_ */
static int cb_det(W *w, Cur *c, Log *l, int which, int16_t x) {
  unsigned *cur = which == ID_R ? &c->dR : &c->dRf;
  const uint32_t *tab = which == ID_R ? g_sc.dR : g_sc.dRf;
  unsigned k = (*cur)++ % NSC;
  int32_t r = (int32_t)tab[k];
  if (which == ID_R) w->t->r_dir_ = (int32_t)g_sc.ddir[k]; else w->t->rf_dir_ = (int32_t)g_sc.ddir[k];
  REC(w, l, 'c', "%s.detect(%d) -> %d dir=%d", idn(which), (int)x, (int)r, (int)g_sc.ddir[k]);
  return r;
}
static void cb_cpreset(W *w, Cur *c, Log *l) { unsigned k = c->rs++ % NSC; REC(w, l, 'c', "cp.reset"); w->cp->trn2dD_ = g_sc.s1[k]; w->t->w34fc_ = g_sc.s2[k]; }
static void cb_mpreset(W *w, Cur *c, Log *l) { unsigned k = c->rs++ % NSC; REC(w, l, 'c', "mp.reset"); w->mp->trn2dD_ = g_sc.s1[k]; w->t->w34fc_ = g_sc.s2[k]; }
static void cb_demreset(W *w, Log *l, int mp) { REC(w, l, 'c', "dem.reset(%s)", idn(mp)); }
static int cb_get(W *w, Cur *c, Log *l, int v92, int16_t x) {
  unsigned k = c->dec++ % NSC;
  uint32_t r = g_sc.dec[k];
  REC(w, l, 'c', "getV%dDecision(%d) -> %d", v92 ? 92 : 90, (int)x, (int)(int16_t)r);
  if (g_sc.flip[k]) w->t->session_ = g_sc.flipv[k];
  return (int)r;
}
static int cb_a2l(W *w, Cur *c, Log *l, int v) { int r = (int)g_sc.a2[c->a2++ % NSC]; REC(w, l, 'c', "linear2alaw(%d) -> %d", v, r); return r; }
static int cb_u2l(W *w, Cur *c, Log *l, int v) { int r = (int)g_sc.u2[c->u2++ % NSC]; REC(w, l, 'c', "linear2ulaw(%d) -> %d", v, r); return r; }
static void cb_ed(W *w, Log *l, char kind, const char *s) { REC(w, l, kind, "%s", s); }

/* the impl's recorders */
#define IW (&g_iw)
void c_modctor(void *d, void *p, unsigned flag, void *b2s, void *mp, void *dataMp, void *trn2dMp, void *cp, unsigned a8) {
  CHECK(d, g_obj[ID_MOD]);
  cb_modctor(IW, &g_il, flag, pid(p), pid(b2s), pid(mp), pid(dataMp), pid(trn2dMp), pid(cp), a8);
}
void c_moddtor(void *d) { CHECK(d, g_obj[ID_MOD]); cb_moddtor(IW, &g_il); }
void c_modssf(void *d, unsigned f) { CHECK(d, g_obj[ID_MOD]); cb_modssf(IW, &g_il, f); }
int c_modgen(void *d) { CHECK(d, g_obj[ID_MOD]); return cb_modgen(IW, &g_ic, &g_il); }
void c_modreset(void *d, int pcm, unsigned char code, unsigned st, unsigned n, unsigned limit) { CHECK(d, g_obj[ID_MOD]); cb_modreset(IW, &g_il, pcm, code, st, n, limit); }
void c_modsmp(void *d, void *mp) { CHECK(d, g_obj[ID_MOD]); cb_modsmp(IW, &g_il, pid(mp)); }
void c_rdctor(void *d, void *p) {
  int which = d == g_obj[ID_R] ? ID_R : ID_RF;
  CHECK(d, g_obj[which]);
  cb_rdctor(IW, &g_il, which, pid(p));
}
void c_rddtor(void *d) {
  int which = d == g_obj[ID_R] ? ID_R : ID_RF;
  CHECK(d, g_obj[which]);
  cb_rddtor(IW, &g_il, which);
}
void c_rdreset(void *d, unsigned a, unsigned b) {
  int which = d == g_obj[ID_R] ? ID_R : ID_RF;
  CHECK(d, g_obj[which]);
  cb_rdreset(IW, &g_il, which, a, b);
}
int c_dR(void *d, short x) { CHECK(d, g_obj[ID_R]); return cb_det(IW, &g_ic, &g_il, ID_R, x); }
int c_dRf(void *d, short x) { CHECK(d, g_obj[ID_RF]); return cb_det(IW, &g_ic, &g_il, ID_RF, x); }
void c_cpreset(void *d) { CHECK(d, g_obj[ID_CP]); cb_cpreset(IW, &g_ic, &g_il); }
void c_mpreset(void *d) { CHECK(d, g_obj[ID_MP]); cb_mpreset(IW, &g_ic, &g_il); }
void c_demreset(void *d, void *mp) { CHECK(d, g_obj[ID_DEM]); cb_demreset(IW, &g_il, pid(mp)); }
short c_get90(void *d, short x) { CHECK(d, (void *)g_iw.t); return (short)cb_get(IW, &g_ic, &g_il, 0, x); }
short c_get92(void *d, short x) { CHECK(d, (void *)g_iw.t); return (short)cb_get(IW, &g_ic, &g_il, 1, x); }
int c_a2l(int v) { return cb_a2l(IW, &g_ic, &g_il, v); }
int c_u2l(int v) { return cb_u2l(IW, &g_ic, &g_il, v); }
void edprintf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  cb_ed(IW, &g_il, 'e', buf);
}
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  cb_ed(IW, &g_il, 'd', buf);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ the independent model */
#define MW (&g_mw)
#define MC (&g_mc)
#define ML (&g_ml)
#define PFX "V90Phase4Demodulator: "
static void m_pf(W *w, char kind, const char *fmt, ...) {
  char b[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  cb_ed(w, ML, kind, b);
}

/* the constructor: the modulator, r_, rf_ first, then the eleven members.  a[0..9] = mp1 mp2 dem cp mp descr ce params p3 adid, session */
static void m_ctor(W *w, void *const *a, unsigned session) {
  T *t = w->t;
  int pm1 = pid(a[0]), pm2 = pid(a[1]), pdem = pid(a[2]), pcp = pid(a[3]), pmp = pid(a[4]), pde = pid(a[5]), pce = pid(a[6]), ppar = pid(a[7]), pp3 = pid(a[8]), pad = pid(a[9]);
  (void)pdem; (void)pcp; (void)pmp; (void)pde; (void)pce; (void)pp3; (void)pad;
  CV(G_CTOR, "ctor");
  cb_modctor(w, ML, session, ppar, ID_NULL, ID_NULL, pm2, pm1, ID_NULL, 0xc);
  cb_rdctor(w, ML, ID_R, ppar);
  cb_rdctor(w, ML, ID_RF, ppar);
  t->mp1_ = a[0]; t->params_ = a[7]; t->session_ = session; t->dem_ = a[2]; t->mp2_ = a[1]; t->cp_ = a[3]; t->mp_ = a[4]; t->descr_ = a[5]; t->ce_ = a[6]; t->p3_ = a[8]; t->adid_ = a[9];
  if (session) CV(G_CTOR, "ctor: session != 0"); else CV(G_CTOR, "ctor: session == 0");
}
static void m_dtor(W *w) {
  CV(G_DTOR, "dtor");
  cb_rddtor(w, ML, ID_RF);
  cb_rddtor(w, ML, ID_R);
  cb_moddtor(w, ML);
}

static void m_ssf(W *w, unsigned f) { CV(G_SMALL, "setSessionFlag"); w->t->session_ = f; cb_modssf(w, ML, f); }
static void m_ew(W *w, int which) {
  T *t = w->t;
  static const char *const s[3] = { PFX "enter WaitForV90CP state @ %d\r\n", PFX "enter WaitForMP state @ %d\r\n", PFX "enter WaitForEd state @ %d\r\n" };
  if (which == 0) CV(G_SMALL, "enterWaitForCP"); else if (which == 1) CV(G_SMALL, "enterWaitForMP"); else CV(G_SMALL, "enterWaitForEd");
  m_pf(w, 'e', s[which], (int)t->cnt_);
  t->state_ = 4u + (unsigned)which; t->cnt_ = 0;
}
static void m_rrd(W *w) {
  CV(G_SMALL, "resetRRNDetector");
  cb_rdreset(w, ML, ID_R, (uint32_t)w->par->rrnRDetectionLength, 0x18);
  cb_rdreset(w, ML, ID_RF, 0xb4, 0xc);
}
static int m_drrn(W *w, int16_t x) {
  T *t = w->t;
  if (cb_det(w, MC, ML, ID_R, x) == 0) { CV(G_SMALL, "detectRRN: no"); return 0; }
  m_pf(w, 'e', PFX "Rd detected, polarity = %d\r\n", (int)t->r_dir_);
  t->state_ = 9; t->cnt_ = 0; t->event_ = 0;
  if (t->session_ == 0) { CV(G_SMALL, "detectRRN: yes, V.90"); t->w38_ = 1; t->w3c_ = 1; }
  else CV(G_SMALL, "detectRRN: yes, V.92");
  return 1;
}
static void m_rbr(W *w) { T *t = w->t; CV(G_SMALL, "resetBeforRRN"); t->w38_ = 1; t->w3c_ = 1; t->w44_ = 0; t->w48_ = 0; t->cpSeen_ = 0; }
static int m_dfpe(W *w, int16_t x) {
  T *t = w->t;
  if (cb_det(w, MC, ML, ID_RF, x) == 0) { CV(G_SMALL, "detectFPE: no"); return 0; }
  CV(G_SMALL, "detectFPE: yes");
  m_pf(w, 'e', PFX "Rf detected, polarity = %d\r\n", (int)t->r_dir_);   /* the blob prints r_'s polarity (this + 0x3020) */
  m_pf(w, 'e', PFX "enter FPE !");
  t->state_ = 0x10; t->cnt_ = 0; t->event_ = 0;
  return 1;
}
static int16_t m_getd(W *w, int16_t x) {
  if (w->t->session_ != 0) { CV(G_SMALL, "getDecision: V.92"); return (int16_t)cb_get(w, MC, ML, 1, x); }
  CV(G_SMALL, "getDecision: V.90");
  return (int16_t)cb_get(w, MC, ML, 0, x);
}
static int m_iabs(int v) { return v < 0 ? (int)(0u - (unsigned)v) : v; }
static int16_t m_t2d(W *w) {
  T *t = w->t;
  int16_t s = (int16_t)cb_modgen(w, MC, ML);
  int sgn = s < 0 ? -1 : 1;
  int mag = (int16_t)m_iabs((int)s);            /* the magnitude cut to 16 bits */
  unsigned ph = (t->cnt_ - 1u) % 6u;
  unsigned lvl;
  mag = m_iabs(mag);
  if ((uint16_t)w->adid->pcmType_ != 0) { CV(G_SMALL, "trn2dKnownDemod: A-law"); lvl = (unsigned)cb_a2l(w, MC, ML, mag) ^ 0xd5u; lvl &= 0xffu; }
  else { CV(G_SMALL, "trn2dKnownDemod: mu-law"); lvl = 0xffu - ((unsigned)cb_u2l(w, MC, ML, mag) & 0xffu); }
  if (s < 0) CV(G_SMALL, "trn2dKnownDemod: negative"); else CV(G_SMALL, "trn2dKnownDemod: positive");
  if (s == -32768) CV(G_SMALL, "trn2dKnownDemod: -32768");
  { unsigned v = w->adid->linearMapping_[ph * 128u + lvl]; return (int16_t)(sgn * (int)v); }
}
static void m_reset(W *w, unsigned code, unsigned st, unsigned arg3, unsigned arg4) {
  T *t = w->t;
  unsigned lvl = dsplibs_debug_level, n;
  int prev;
  CV(G_RESET, "reset");
  t->event_ = 0; t->keepRate_ = 0; t->code_ = (uint8_t)code; t->cnt_ = 0; t->w38_ = 1; t->state_ = st; t->w3c_ = 0; t->w40_ = 0; t->w44_ = 0; t->w48_ = 0; t->cpSeen_ = 0;
  cb_rdreset(w, ML, ID_R, (uint32_t)w->par->phase4RDetectionLength, 0x18);
  cb_rdreset(w, ML, ID_RF, (uint32_t)w->par->phase4RDetectionLength, 0x18);
  t->w34_ = arg4;
  if (t->session_ != 0) {
    CV(G_RESET, "reset: V.92 (CP)");
    cb_cpreset(w, MC, ML);
    t->w34fc_ = w->mp1->w0_;
    w->cp->trn2dD_ = w->mp1->w0_;
  } else {
    CV(G_RESET, "reset: V.90 (MP)");
    cb_mpreset(w, MC, ML);
    w->mp->trn2dD_ = w->mp1->w0_;
  }
  cb_demreset(w, ML, ID_MP1);
  t->studyEnd_ = 0;
  cb_modreset(w, ML, w->adid->pcmType_, t->code_, 3, 0, 0);
  cb_modsmp(w, ML, ID_MP1);
  if (lvl > 1) { CV(G_RESET, "reset: print"); m_pf(w, 'd', PFX "reset called, quickConnect indication is %d\r\n", (int)t->w34_); }
  if (t->w34_ != 0) { CV(G_RESET, "reset: quick connect"); t->limit_ = (uint32_t)w->par->trn2dQcDdLength; }
  else { CV(G_RESET, "reset: no quick connect"); t->limit_ = (uint32_t)w->par->trn2dDdLength; }
  m_pf(w, 'e', PFX "trn2dDDLength = %d symbols\r\n", (int)t->limit_);
  if (arg3 == 0) CV(G_RESET, "reset: no decisions");
  n = arg3;
  prev = -1;
  while (n != 0) {
    if (prev >= 0 && prev != (t->session_ != 0)) CV(G_RESET, "reset: session_ changed inside the loop");
    prev = t->session_ != 0;
    if (t->session_ != 0) { CV(G_RESET, "reset: decision V.92"); cb_get(w, MC, ML, 1, 0); }
    else { CV(G_RESET, "reset: decision V.90"); cb_get(w, MC, ML, 0, 0); }
    n--;
    if (n != 0 && arg3 > 1) CV(G_RESET, "reset: several decisions");
  }
}

/* ------------------------------------------------------------------------------------------------ trial setup */
static unsigned rnd_range(unsigned lo, unsigned hi) { return lo + h_rmod(hi - lo + 1); }
static unsigned pick_level(void) {
  static const unsigned lv[] = { 0, 1, 2, 3, 4, 0x80000000u, 0xffffffffu, 0x80000001u };
  return h_rpct(40) ? 2u + h_rmod(2) : lv[h_rmod(sizeof lv / sizeof lv[0])];
}
static uint32_t pick_res(void) {   /* a detector result: the whole eax is tested */
  static const uint32_t v[] = { 1, 0x100, 0x10000, 0x1000000, 0x80000000u, 0xffffffffu, 2, 0x7fffffffu };
  return h_rpct(45) ? 0u : h_rpct(50) ? 1u : h_rpct(50) ? v[h_rmod(sizeof v / sizeof v[0])] : (h_rnd() | 1u);
}
static uint32_t pick_word(void) { return h_rpct(50) ? h_rnd() : h_rpct(50) ? 0u : h_rpct(50) ? 0xffffffffu : 0x80000000u; }
static uint32_t pick_flag(void) { return h_rpct(45) ? 0u : h_rpct(70) ? 1u : h_rnd(); }
static void gen_script(void) {
  unsigned i;
  static const int16_t syms[] = { 0, 1, -1, 100, -100, 32767, -32768, -32767, 4000, -4000, 0x7f, 0x1234, 255, -255, 256, 8031, -8031 };
  for (i = 0; i < NSC; i++) {
    uint32_t lo = h_rpct(60) ? (uint16_t)syms[h_rmod(sizeof syms / sizeof syms[0])] : (h_rnd() & 0xffffu);
    g_sc.sym[i] = (h_rnd() & 0xffff0000u) | lo;
    g_sc.dR[i] = pick_res(); g_sc.dRf[i] = pick_res();
    g_sc.ddir[i] = h_rpct(50) ? (h_rpct(50) ? 1u : 0xffffffffu) : h_rnd();
    /* the level inside the 128-entry row: (code ^ 0xd5) and 0xff - code stay below 128 (the table has 768 entries; beyond them the mirror's next members differ per architecture); garbage above the low byte */
    g_sc.a2[i] = (h_rnd() & 0xffffff00u) | (((h_rnd() & 0x7fu)) ^ 0xd5u);
    g_sc.u2[i] = (h_rnd() & 0xffffff00u) | (0x80u | (h_rnd() & 0x7fu));
    g_sc.dec[i] = h_rnd();
    g_sc.flip[i] = h_rpct(25) ? 1u : 0u;
    g_sc.flipv[i] = pick_flag();
    g_sc.s1[i] = pick_word(); g_sc.s2[i] = pick_word();
  }
}
/* random garbage in every non-pointer field (the pointer fields: garbage too when `ptrs` is set) */
static void fill_obj(T *t, int ptrs) {
  int i;
  for (i = 0; i < NF_Ph4DemObj; i++) {
    const HFld *f = &f_Ph4DemObj[i];
    unsigned n = f->count ? f->count : 1, es, tot, j;
    uint8_t *q = (uint8_t *)t + f->off;
    switch (f->kind) {
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'u': case 'i': case 'f': es = 4; break;
      case 'd': es = 8; break;
      case 'p': case 'o': {
        void *v = NULL;
        if (ptrs) { uintptr_t r = h_rnd(); r = (r << 4) ^ (uintptr_t)h_rnd(); v = (void *)(r | 1u); }
        memcpy(q, &v, sizeof v);
        continue;
      }
      default: continue;
    }
    tot = n * es;
    for (j = 0; j < tot; j += 4) { uint32_t r = h_rnd(); memcpy(q + j, &r, tot - j < 4 ? tot - j : 4); }
  }
}
static void new_world(int ptrs_known) {
  int k;
  uint8_t *raw = (uint8_t *)h_obj_new(SZ + CAN);
  T *t;
  g_iw.t = t = (T *)raw; g_can = raw + SZ;
  g_iw.mp = (MpObj *)h_obj_new(sizeof(MpObj));
  g_iw.cp = (CpObj *)h_obj_new(sizeof(CpObj));
  g_iw.par = (V90ParObj *)h_obj_new(sizeof(V90ParObj));
  g_iw.adid = (AdidObj *)h_obj_new(sizeof(AdidObj));
  g_iw.mp1 = (MapPar90 *)h_obj_new(sizeof(MapPar90));
  g_iw.mp2 = (MapPar90 *)h_obj_new(sizeof(MapPar90));
  for (k = 0; k < 4; k++) { uint32_t *q = (uint32_t *)(g_fake[k] = sysdep_malloc(0x40)); unsigned i; for (i = 0; i < 16; i++) q[i] = h_rnd(); }
  memset(raw, 0, SZ);
  memset(raw + SZ, 0x5a, CAN);
  fill_obj(t, !ptrs_known);
  g_obj[ID_DEM] = g_fake[0]; g_obj[ID_MP] = g_iw.mp; g_obj[ID_CP] = g_iw.cp; g_obj[ID_PAR] = g_iw.par; g_obj[ID_ADID] = g_iw.adid; g_obj[ID_MP1] = g_iw.mp1; g_obj[ID_MP2] = g_iw.mp2;
  g_obj[ID_DESCR] = g_fake[1]; g_obj[ID_CE] = g_fake[2]; g_obj[ID_P3] = g_fake[3];
  g_obj[ID_MOD] = &t->mod_flag_; g_obj[ID_R] = &t->r_cnt_; g_obj[ID_RF] = &t->rf_cnt_;
  if (ptrs_known) {
    t->params_ = g_iw.par; t->mp1_ = g_iw.mp1; t->mp2_ = g_iw.mp2; t->cp_ = g_iw.cp; t->mp_ = g_iw.mp; t->p3_ = g_fake[3]; t->dem_ = g_fake[0]; t->descr_ = g_fake[1]; t->ce_ = g_fake[2];
    t->adid_ = g_iw.adid;
  }
  { unsigned i; for (i = 0; i < 768; i++) g_iw.adid->linearMapping_[i] = (uint16_t)(h_rpct(5) ? 0u : h_rpct(5) ? 0xffffu : h_rnd()); }   /* explicit (the pool's fill depends on sizeof) */
  g_iw.mp->trn2dD_ = h_rnd();
  g_iw.cp->trn2dD_ = h_rnd();
  g_iw.mp1->w0_ = h_rnd(); g_iw.mp2->w0_ = h_rnd();
  g_iw.adid->pcmType_ = (int32_t)(h_rpct(30) ? 0u : h_rpct(30) ? 1u : h_rpct(50) ? (h_rpct(50) ? 0x10000u : 0x10001u) : h_rpct(30) ? 0x8000u : h_rpct(30) ? 0xffff0000u : h_rnd());
  g_iw.par->phase4RDetectionLength = (int32_t)(h_rpct(60) ? rnd_range(0, 200) : h_rnd());
  g_iw.par->rrnRDetectionLength = (int32_t)(h_rpct(60) ? rnd_range(0, 200) : h_rnd());
  g_iw.par->trn2dDdLength = (int32_t)(h_rpct(60) ? rnd_range(0, 20000) : h_rnd());
  g_iw.par->trn2dQcDdLength = (int32_t)(h_rpct(60) ? rnd_range(0, 20000) : h_rnd());
}
static void free_world(void) { int k; for (k = 0; k < 4; k++) sysdep_free(g_fake[k]); }

static unsigned long g_ncalls;
static int g_showall;
static void copy_world_to_model(void) {
  memcpy(&g_mt, g_iw.t, SZ);
  memcpy(&g_mmp, g_iw.mp, sizeof g_mmp);
  memcpy(&g_mcp, g_iw.cp, sizeof g_mcp);
  memcpy(&g_mpar, g_iw.par, sizeof g_mpar);
  memcpy(&g_madid, g_iw.adid, sizeof g_madid);
  memcpy(&g_mmp1, g_iw.mp1, sizeof g_mmp1);
  memcpy(&g_mmp2, g_iw.mp2, sizeof g_mmp2);
  g_mw.t = &g_mt; g_mw.mp = &g_mmp; g_mw.cp = &g_mcp; g_mw.par = &g_mpar; g_mw.adid = &g_madid; g_mw.mp1 = &g_mmp1; g_mw.mp2 = &g_mmp2;
}
static const char *g_what = "setup";
static void cmp_blob(const char *what, const char *nm, const void *m, const void *i, size_t n) {
  if (memcmp(m, i, n)) {
    const uint8_t *x = (const uint8_t *)m, *y = (const uint8_t *)i;
    size_t k;
    for (k = 0; k < n; k++) if (x[k] != y[k]) break;
    h_die3("%s: the %s differs from the model at mirror offset 0x%zx (model %02x, impl %02x)", what, nm, k, x[k], y[k]);
  }
}
static void cmp_world(const char *what) {
  cmp_blob(what, "object", &g_mt, g_iw.t, SZ);
  cmp_blob(what, "MP", &g_mmp, g_iw.mp, sizeof g_mmp);
  cmp_blob(what, "CP", &g_mcp, g_iw.cp, sizeof g_mcp);
  cmp_blob(what, "parameter block", &g_mpar, g_iw.par, sizeof g_mpar);
  cmp_blob(what, "adid", &g_madid, g_iw.adid, sizeof g_madid);
  cmp_blob(what, "mp1", &g_mmp1, g_iw.mp1, sizeof g_mmp1);
  cmp_blob(what, "mp2", &g_mmp2, g_iw.mp2, sizeof g_mmp2);
}
static void log_cmp(void) {
  unsigned i, n = g_ml.n < LOGMAX ? g_ml.n : LOGMAX;
  if (g_ml.n != g_il.n) {
    for (i = 0; i < n && i < g_il.n && i < LOGMAX; i++) fprintf(stderr, "m: %s\ni: %s\n", g_ml.r[i].t, g_il.r[i].t);
    h_die3("%s: %u model records, %u impl records", g_what, g_ml.n, g_il.n);
  }
  for (i = 0; i < n; i++) if (strcmp(g_ml.r[i].t, g_il.r[i].t)) h_die3("%s: record %u differs:\n model: %s\n impl:  %s", g_what, i, g_ml.r[i].t, g_il.r[i].t);
  if (memcmp(&g_mc, &g_ic, sizeof g_mc)) h_die3("%s: the callee cursors differ", g_what);
}
static void emit_state(const char *tag) {
  const T *t = g_iw.t;
  h_emit(g_show, "%s h=%08x sess=%u st=%u cnt=%u ev=%u lim=%u cs=%u w=%u/%u/%u/%u/%u/%u code=%u se=%u w34fc=%08x mpd=%08x cpd=%08x dirs=%d/%d", tag, obj_hash(t), (unsigned)t->session_, (unsigned)t->state_,
         (unsigned)t->cnt_, (unsigned)t->event_, (unsigned)t->limit_, (unsigned)t->cpSeen_, (unsigned)t->w34_, (unsigned)t->w38_, (unsigned)t->w3c_, (unsigned)t->w40_, (unsigned)t->w44_,
         (unsigned)t->w48_, (unsigned)t->code_, (unsigned)t->studyEnd_, (unsigned)t->w34fc_, (unsigned)g_iw.mp->trn2dD_, (unsigned)g_iw.cp->trn2dD_, (int)t->r_dir_, (int)t->rf_dir_);
}
static unsigned g16(int16_t x) { return (h_rnd() & 0xffff0000u) | (uint16_t)x; }   /* a 16-bit argument with garbage above (the callee reads the low word only) */

/* one call: model on its copy first, then the impl, then compare everything.  op: 0 C1 1 C2 2 D1 3 D2 4 setSessionFlag 5 enterWaitForCP 6 MP 7 Ed 8 resetRRNDetector 9 detectRRN 10 resetBeforRRN 11 detectFPE
 * 12 trn2dKnownDemod 13 getDecision 14 reset */
typedef struct { int op; void *a[10]; unsigned u[4]; int16_t x; } Call;
static const char *const opname[] = { "C1", "C2", "D1", "D2", "setSessionFlag", "enterWaitForCP", "enterWaitForMP", "enterWaitForEd", "resetRRNDetector", "detectRRN", "resetBeforRRN", "detectFPE",
                                      "trn2dKnownDemod", "getDecision", "reset" };
static void do_call(const Call *c) {
  int mr = 0, ir = 0, cmpres = 0;
  T *t = g_iw.t;
  copy_world_to_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  g_what = opname[c->op];
  switch (c->op) {
    case 0: case 1: m_ctor(MW, c->a, c->u[0]); break;
    case 2: case 3: m_dtor(MW); break;
    case 4: m_ssf(MW, c->u[0]); break;
    case 5: case 6: case 7: m_ew(MW, c->op - 5); break;
    case 8: m_rrd(MW); break;
    case 9: mr = m_drrn(MW, c->x); cmpres = 1; break;
    case 10: m_rbr(MW); break;
    case 11: mr = m_dfpe(MW, c->x); cmpres = 1; break;
    case 12: mr = m_t2d(MW); cmpres = 2; break;
    case 13: mr = m_getd(MW, c->x); cmpres = 2; break;
    case 14: m_reset(MW, c->u[0] & 0xffu, c->u[1], c->u[2], c->u[3]); break;
  }
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  h_emit(g_show, "call %s u=%u/%u/%u/%u x=%d lvl=%u", opname[c->op], c->u[0], c->u[1], c->u[2], c->u[3], (int)c->x, dsplibs_debug_level);
  switch (c->op) {
    case 0: f_c1(t, c->a[0], c->a[1], c->a[2], c->a[3], c->a[4], c->a[5], c->a[6], c->a[7], c->a[8], c->a[9], c->u[0]); break;
    case 1: f_c2(t, c->a[0], c->a[1], c->a[2], c->a[3], c->a[4], c->a[5], c->a[6], c->a[7], c->a[8], c->a[9], c->u[0]); break;
    case 2: f_d1(t); break;
    case 3: f_d2(t); break;
    case 4: f_ssf(t, c->u[0]); break;
    case 5: f_ewcp(t); break;
    case 6: f_ewmp(t); break;
    case 7: f_ewed(t); break;
    case 8: f_rrd(t); break;
    case 9: ir = f_drrn(t, g16(c->x)); break;
    case 10: f_rbr(t); break;
    case 11: ir = f_dfpe(t, g16(c->x)); break;
    case 12: ir = (int)f_t2d(t, g16(c->x)); break;
    case 13: ir = (int)f_getd(t, g16(c->x)); break;
    case 14: f_reset(t, h_garbage_hi(c->u[0] & 0xffu), c->u[1], c->u[2], c->u[3]); break;
  }
  __asm__ volatile("" ::: "memory");
  { unsigned i; for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t); }
  if (cmpres == 1 && mr != ir) h_die3("%s: result: model %d, impl %d", g_what, mr, ir);
  if (cmpres == 2 && (int16_t)mr != (int16_t)ir) h_die3("%s: result: model %d, impl %d", g_what, (int)(int16_t)mr, (int)(int16_t)ir);
  if (cmpres) h_emit(g_show, "ret %d", cmpres == 2 ? (int)(int16_t)ir : ir);
  log_cmp();
  cmp_world(g_what);
  { unsigned i; for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", g_what, i); }
  emit_state("end");
  cov_flush();
  g_ncalls++;
}

/* ------------------------------------------------------------------------------------------------ the trials */
static void pick_ctor_args(Call *c, int sc_garbage) {
  /* mp1 mp2 dem cp mp descr ce params p3 adid: distinct known objects mostly; NULL / wild / duplicates sometimes */
  void *known[10] = { g_obj[ID_MP1], g_obj[ID_MP2], g_obj[ID_DEM], g_obj[ID_CP], g_obj[ID_MP], g_obj[ID_DESCR], g_obj[ID_CE], g_obj[ID_PAR], g_obj[ID_P3], g_obj[ID_ADID] };
  int k;
  (void)sc_garbage;
  for (k = 0; k < 10; k++) {
    c->a[k] = known[k];
    if (h_rpct(6)) c->a[k] = NULL;
    else if (h_rpct(5)) c->a[k] = known[h_rmod(10)];
    else if (h_rpct(4)) { uintptr_t r = h_rnd(); c->a[k] = (void *)((r << 4) | 1u); }
  }
}
static void trial(int sc) {
  Call c;
  unsigned ncalls = 1, k;
  int ptrs_known = sc != 0;
  new_world(ptrs_known);
  gen_script();
  dsplibs_debug_level = pick_level();
  if (sc == 2 && h_rpct(30)) ncalls = rnd_range(2, 4);
  for (k = 0; k < ncalls; k++) {
    T *t = g_iw.t;
    memset(&c, 0, sizeof c);
    c.x = (int16_t)(h_rpct(40) ? (int16_t)h_rnd() : h_rpct(50) ? (int16_t)((int)h_rmod(401) - 200) : (h_rpct(50) ? -32768 : 32767));
    if (k > 0 && h_rpct(50)) gen_script();
    switch (sc) {
      case 0: c.op = (int)h_rmod(2); pick_ctor_args(&c, 1); c.u[0] = pick_flag(); break;
      case 1: c.op = 2 + (int)h_rmod(2); break;
      case 2: {
        static const int ops[] = { 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 9, 11, 12, 13 };
        c.op = ops[h_rmod(sizeof ops / sizeof ops[0])];
        c.u[0] = pick_flag();
        t->session_ = pick_flag();
        t->cnt_ = h_rpct(50) ? h_rmod(20) : h_rpct(30) ? 0u : h_rnd();
        t->r_dir_ = (int32_t)h_rnd(); t->rf_dir_ = (int32_t)h_rnd();
        if (h_rpct(60)) { unsigned i; for (i = 0; i < NSC; i++) if (h_rpct(35)) g_sc.dR[i] = g_sc.dRf[i] = h_rpct(50) ? 0u : 1u; }
        break;
      }
      default:
        c.op = 14;
        c.u[0] = h_rnd();
        c.u[1] = h_rpct(40) ? h_rmod(18) : h_rnd();
        c.u[2] = h_rpct(30) ? 0u : h_rpct(80) ? rnd_range(1, 8) : rnd_range(9, 20);
        c.u[3] = pick_flag();
        t->session_ = pick_flag();
        break;
    }
    if (h_rpct(10)) dsplibs_debug_level = pick_level();
    do_call(&c);
  }
  free_world();
  h_trial_end("p38");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("P38_SEED");
  CovSite *c;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P38_ALL")) g_showall = 1;
  p07_layout_check(); p09_layout_check(); p10_layout_check(); p11_layout_check(); p18_layout_check(); p37_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P38_TRIALS") ? atoi(getenv("P38_TRIALS")) : (sc == 2 ? 40000 : 20000);
  if (sc < 0 || sc > 3) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (c = __start_covsites; c < __stop_covsites; c++) {
    if (c->grp != sc) continue;
    h_emit(1, "COV %s %lu", c->name, c->n);
    if (c->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", c->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P38_NOCOV")) ? 2 : 0;
}
