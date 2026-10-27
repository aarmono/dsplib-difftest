/*
 * Differential test harness for port chunk P35 (src/vpcm/v90_phase3_demod_v90_decision.cpp): V90Phase3Demodulator::getV90Decision(float), the V.90 phase-3 RX state machine (8,379 B).
 * Derived from test/vpcm/v90_phase3_demod_v92_decision.main.c (getV92Decision): the same recorder / model / snapshot design; what is new is the V90Jd frame (V90Jd::unPackData / unPackReset), the b424_ latch of state 0
 * (reads the first word of the V90SdDetector object: scripted by the harness), event 4 at the end of state 4, state 6 = wait for the V90Jd frame, and the debug-level print of state 30.
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_phase3_demod_v90_decision.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2:
 * the port object alone, the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_phase3_demod_v90_decision.blob32.out).  The function is reached through an __asm__ label; the
 * object is the generated C mirror (test/vpcm/v90_phase3_demod_v92_decision.layout), the V90Parameters block / the AutoDigitalImpDetector are the mirrors of P07 / P18 (member-wise access: their layouts differ on x86-64).
 *
 * Every callee is a RECORDER with scripted results: V90Phase3Modulator::generateSymbol (also writes the four modulator fields the demodulator reads: event_ / segCnt_ / isRef_ / code_) and ::reset,
 * SerialDifferentialDecoder<int>::process, Descrambler<int,int>::process, V90SdDetector::process, GenericToneDetector::process / reset, V90Jd::unPackData / unPackReset, the twelve
 * V90AutoDigitalImpDetector methods the function calls (updateLinMappMeanAndVar / calculateLinearMeanAndVar also WRITE the linear mapping table: the order of table reads and callee calls is
 * observable), linear2alaw / linear2ulaw (scripted: the blob calls them twice where it recomputes the code), edprintf and dsplibs_debug_printf.  The results of the callees carry garbage in the bits
 * the function must not look at (isAltRbs / isThereAnyAltRbsPhase: the upper 16 bits; V90Jd::unPackData: the upper 24 bits; the G.711 coders: the upper 24 bits).
 *
 * What is compared: the returned symbol, the whole object, the adid and the parameter block after every call, every record (callee, arguments, results, and a SNAPSHOT of the demodulator object and the
 * three parameter words the function writes at the moment of the call: the store order before each call / print), the canary behind the object.  The INDEPENDENT MODEL (m_*: written from
 * porting/decompiled/dp/p35_phase3_demod_getv90decision.txt in the shape of the blob's own control flow, not from the port source) predicts the result, the final world and every record; a model violation exits 3.
 * Excluded: states 7, 8, 18 and > 0x21 (the blob returns the CALLER's edi: the result is not compared there, everything else is), signalling NaN inputs (the blob's fld / fstp quiets them), phase_ > 7
 * (table reads outside the detector object).
 *
 * Scenarios (each its own process): 0 the non-DIL states, 1 the DIL states 10 .. 17, 2 every state (illegal ones too), extreme inputs, call sequences.
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
#include "v90_adid_layout.h"
#include "v90_phase3_demod_v92_decision_layout.h"

#define L(m) __asm__(m)

extern unsigned int dsplibs_debug_level;

/* the function under test */
extern short f_get(void *, float) L("_ZN20V90Phase3Demodulator14getV90DecisionEf");

/* the interposed callees */
extern short c_gen(void *) L("_ZN18V90Phase3Modulator14generateSymbolEv");
extern void c_mreset(void *, unsigned, unsigned char, unsigned, unsigned, void *, void *, const void *, unsigned)
    L("_ZN18V90Phase3Modulator5resetE7PcmTypeh20Phase3ModulatorStatejP5V90JdP5V92JdPK19tagV90DILdescriptorj");
extern int c_sdd(void *, int) L("_ZN25SerialDifferentialDecoderIiE7processEi");
extern int c_descr(void *, int) L("_ZN11DescramblerIiiE7processEi");
extern int c_sd(void *, float) L("_ZN13V90SdDetector7processEf");
extern unsigned c_ans(void *, float) L("_ZN19GenericToneDetector7processEf");
extern void c_ansreset(void *) L("_ZN19GenericToneDetector5resetEv");
extern void c_jdreset(void *) L("_ZN5V90Jd11unPackResetEv");
extern unsigned c_jddata(void *, int) L("_ZN5V90Jd10unPackDataEi");
extern int c_isalt(void *, short, short, float) L("_ZN25V90AutoDigitalImpDetector8isAltRbsEssf");
extern int c_anyalt(void *) L("_ZN25V90AutoDigitalImpDetector21isThereAnyAltRbsPhaseEv");
extern void c_calc(void *, short, short, unsigned) L("_ZN25V90AutoDigitalImpDetector25calculateLinearMeanAndVarEssj");
extern void c_upd(void *, short, short) L("_ZN25V90AutoDigitalImpDetector23updateLinMappMeanAndVarEss");
extern void c_add(void *, short, unsigned char, float) L("_ZN25V90AutoDigitalImpDetector26addReceivedSampleToStorageEshf");
extern void c_unite(void *, unsigned char) L("_ZN25V90AutoDigitalImpDetector35uniteLinMappInfoOfUnsuspectedPhasesEh");
extern void c_resetstudy(void *, unsigned) L("_ZN25V90AutoDigitalImpDetector21resetStudyUrefHandlerEj");
extern int c_study(void *, float, unsigned) L("_ZN25V90AutoDigitalImpDetector16studyUrefHandlerEfj");
extern void c_first(void *) L("_ZN25V90AutoDigitalImpDetector17porcessFirstStudyEv");
extern void c_second(void *) L("_ZN25V90AutoDigitalImpDetector18porcessSecondStudyEv");
extern void c_qcmap(void *) L("_ZN25V90AutoDigitalImpDetector18setQcLinearMappingEv");
extern void c_adjust(void *, short) L("_ZN25V90AutoDigitalImpDetector24adjustUinfoToPhaseOffsetEs");
extern int c_lin_a(int) L("linear2alaw");
extern int c_lin_u(int) L("linear2ulaw");

typedef Ph3DemObj T;
#define SZ sizeof(T)
enum { CAN = 64, NSC = 24, LOGMAX = 24, TXTMAX = 640, FAKEN = 6 };

static uint32_t fnv(const void *p, size_t n) { const uint8_t *q = (const uint8_t *)p; uint32_t h = 2166136261u; while (n--) { h ^= *q++; h *= 16777619u; } return h; }

/* ------------------------------------------------------------------------------------------------ the world: the demodulator, the detector, the parameter block, the fake objects */
typedef struct { T *t; AdidObj *a; V90ParObj *p; } W;
static T g_mt;               /* the model's copy of the object */
static AdidObj g_ma;         /* ... of the detector */
static V90ParObj g_mp;       /* ... of the parameter block */
static W g_iw, g_mw;         /* impl world / model world */
static uint8_t *g_can;
static void *g_fake[FAKEN]; /* the heap objects the object points to: sd, ans, jd, jd92, dil, modem params */
#define F_SD 0
#define F_ANS 1
#define F_JD 2
#define F_JD92 3
#define F_DIL 4
#define F_MP 5
static int fid(const void *p) {
  int k;
  if (!p) return -1;
  for (k = 0; k < FAKEN; k++) if (p == g_fake[k]) return k;
  return 9;
}

/* ------------------------------------------------------------------------------------------------ scripts, cursors, records */
typedef struct {
  int16_t sym[NSC];
  uint8_t mev[NSC], mref[NSC], mseg[NSC], mcode[NSC];
  uint32_t mrefw[NSC], msegw[NSC];
  int32_t sdd[NSC], descr[NSC], sd[NSC];
  uint32_t ans[NSC], jdd[NSC];
  int32_t alt[NSC], anyalt[NSC], study[NSC];
  uint32_t lina[NSC], linu[NSC];
  uint32_t updv[NSC], calcv[NSC], calcu[NSC];
} Script;
typedef struct { unsigned gen, sdd, descr, sd, ans, jdd, alt, anyalt, study, lina, linu, upd, calc; } Cur;
typedef struct { char t[TXTMAX]; } Rec;
typedef struct { Rec r[LOGMAX]; unsigned n; unsigned long total; } Log;
static Script g_sc;
static Cur g_ic, g_mc;
static Log g_il, g_ml;
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
static uint32_t fbits(float f) { uint32_t b; memcpy(&b, &f, 4); return b; }
static uint32_t pword(const float *f) { uint32_t b; memcpy(&b, f, 4); return b; }
/* the object fields the function writes / reads between its stores (the snapshot at every record) */
static void snapshot(char *out, size_t cap, const W *w) {
  const T *t = w->t;
  snprintf(out, cap, "|st=%u cnt=%u ev=%u ph=%u stat=%u jn=%u tl=%u w3f4=%u c3fc=%u v=%u me=%u ms=%u mr=%u mc=%u p438=%08x p31c=%u p4cc=%d",
           (unsigned)t->state_, (unsigned)t->cnt_, (unsigned)t->event_, (unsigned)t->phase_, (unsigned)t->status_, (unsigned)t->jdNotCnt_, (unsigned)t->trn1Len_, (unsigned)t->w3f4_, (unsigned)t->cnt3fc_,
           (unsigned)t->verify_, (unsigned)t->mod_event_, (unsigned)t->mod_segCnt_, (unsigned)t->mod_isRef_, (unsigned)t->mod_code_, pword(&w->p->phase4MeanErrorBefToAftUpdateRatioThresh),
           (unsigned)w->p->unk31c, (int)w->p->anspcmDemodulationLength);
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
enum { G_NODIL = 0, G_DIL = 1, G_ANY = 2, G_ILL = 3 };

/* ------------------------------------------------------------------------------------------------ the environment (callee semantics: shared by the impl's recorders and the model) */
#define CHECK(p, what) do { if ((p) != (what)) h_die3("callee %s got an unexpected object %p (expected %p)", __func__, (void *)(p), (void *)(what)); } while (0)

static short cb_gen(W *w, Cur *c, Log *l) {
  unsigned k = c->gen++ % NSC;
  short y = g_sc.sym[k];
  T *t = w->t;
  t->mod_event_ = g_sc.mev[k];
  t->mod_isRef_ = (uint16_t)g_sc.mrefw[k];
  t->mod_segCnt_ = g_sc.msegw[k];
  t->mod_code_ = g_sc.mcode[k];
  REC(w, l, 'c', "gen -> %d", (int)y);
  return y;
}
static void cb_mreset(W *w, Log *l, unsigned pcm, unsigned code, unsigned st, unsigned n, const void *jd, const void *jd92, const void *dil, unsigned tb) {
  REC(w, l, 'c', "mod.reset pcm=%u code=%u st=%u n=%u jd=%d jd92=%d dil=%d tb=%u", pcm, code, st, n, fid(jd), fid(jd92), fid(dil), tb);
}
static int cb_sdd(W *w, Cur *c, Log *l, int v) { int r = g_sc.sdd[c->sdd++ % NSC]; REC(w, l, 'c', "sdd(%d) -> %d", v, r); return r; }
static int cb_descr(W *w, Cur *c, Log *l, int v) { int r = g_sc.descr[c->descr++ % NSC]; REC(w, l, 'c', "descr(%d) -> %d", v, r); return r; }
static int cb_sd(W *w, Cur *c, Log *l, float x) { int r = g_sc.sd[c->sd++ % NSC]; REC(w, l, 'c', "sd(%08x) -> %d", fbits(x), r); return r; }
static unsigned cb_ans(W *w, Cur *c, Log *l, float x) { unsigned r = g_sc.ans[c->ans++ % NSC]; REC(w, l, 'c', "ans(%08x) -> %u", fbits(x), r); return r; }
static void cb_ansreset(W *w, Log *l) { REC(w, l, 'c', "ans.reset"); }
static void cb_jdreset(W *w, Log *l) { REC(w, l, 'c', "jd.reset"); }
static unsigned cb_jddata(W *w, Cur *c, Log *l, int v) { unsigned r = g_sc.jdd[c->jdd++ % NSC]; REC(w, l, 'c', "jd.data(%d) -> %u", v, r); return r; }
static int cb_isalt(W *w, Cur *c, Log *l, short ph, short uc, float x) { int r = g_sc.alt[c->alt++ % NSC]; REC(w, l, 'c', "isAltRbs(%d,%d,%08x) -> %d", (int)ph, (int)uc, fbits(x), r); return r; }
static int cb_anyalt(W *w, Cur *c, Log *l) { int r = g_sc.anyalt[c->anyalt++ % NSC]; REC(w, l, 'c', "isThereAnyAltRbsPhase -> %d", r); return r; }
/* the real detector writes its linear mapping: the harness writes one scripted word into the base table at (phase, a scripted ucode) */
static void put_base(W *w, unsigned phase, unsigned uc, unsigned val) {
  uint8_t *b = (uint8_t *)w->a + offsetof(AdidObj, linearMapping_);
  uint16_t v = (uint16_t)val;
  memcpy(b + 2u * (((phase & 0xffffu) << 7) + (uc & 0xffu)), &v, 2);
}
static void cb_calc(W *w, Cur *c, Log *l, short s, short m, unsigned ph) {
  unsigned k = c->calc++ % NSC;
  REC(w, l, 'c', "calc(%d,%d,%u)", (int)s, (int)m, ph);
  put_base(w, ph, g_sc.calcu[k], g_sc.calcv[k]);
}
static void cb_upd(W *w, Cur *c, Log *l, short ph, short uc) {
  unsigned k = c->upd++ % NSC;
  REC(w, l, 'c', "upd(%d,%d)", (int)ph, (int)uc);
  put_base(w, (unsigned)ph, (unsigned)uc, g_sc.updv[k]);
}
static void cb_add(W *w, Log *l, short ph, unsigned char uc, float x) { REC(w, l, 'c', "add(%d,%u,%08x)", (int)ph, (unsigned)uc, fbits(x)); }
static void cb_unite(W *w, Log *l, unsigned char uc) { REC(w, l, 'c', "unite(%u)", (unsigned)uc); }
static void cb_resetstudy(W *w, Log *l, unsigned v) { REC(w, l, 'c', "resetStudy(%u)", v); }
static int cb_study(W *w, Cur *c, Log *l, float x, unsigned ph) { int r = g_sc.study[c->study++ % NSC]; REC(w, l, 'c', "study(%08x,%u) -> %d", fbits(x), ph, r); return r; }
static void cb_first(W *w, Log *l) { REC(w, l, 'c', "porcessFirstStudy"); }
static void cb_second(W *w, Log *l) { REC(w, l, 'c', "porcessSecondStudy"); }
static void cb_qcmap(W *w, Log *l) { REC(w, l, 'c', "setQcLinearMapping"); }
static void cb_adjust(W *w, Log *l, short v) { REC(w, l, 'c', "adjustUinfo(%d)", (int)v); }
static int cb_lin(W *w, Cur *c, Log *l, int alaw, int v) {
  unsigned r = alaw ? g_sc.lina[c->lina++ % NSC] : g_sc.linu[c->linu++ % NSC];
  REC(w, l, 'c', alaw ? "linear2alaw(%d) -> %u" : "linear2ulaw(%d) -> %u", v, r);
  return (int)r;
}
static void cb_ed(W *w, Log *l, char kind, const char *s) { REC(w, l, kind, "%s", s); }

/* the impl's recorders */
#define IW (&g_iw)
short c_gen(void *t) { CHECK(t, (void *)&g_iw.t->mod_flag_); return cb_gen(IW, &g_ic, &g_il); }
void c_mreset(void *t, unsigned pcm, unsigned char code, unsigned st, unsigned n, void *jd, void *jd92, const void *dil, unsigned tb) {
  CHECK(t, (void *)&g_iw.t->mod_flag_);
  cb_mreset(IW, &g_il, pcm, code, st, n, jd, jd92, dil, tb);
}
int c_sdd(void *t, int v) { CHECK(t, (void *)&g_iw.t->sdd_state_); return cb_sdd(IW, &g_ic, &g_il, v); }
int c_descr(void *t, int v) { CHECK(t, (void *)&g_iw.t->descr_buf_); return cb_descr(IW, &g_ic, &g_il, v); }
int c_sd(void *t, float x) { CHECK(t, g_iw.t->sd_); return cb_sd(IW, &g_ic, &g_il, x); }
unsigned c_ans(void *t, float x) { CHECK(t, g_iw.t->ans_); return cb_ans(IW, &g_ic, &g_il, x); }
void c_ansreset(void *t) { CHECK(t, g_iw.t->ans_); cb_ansreset(IW, &g_il); }
void c_jdreset(void *t) { CHECK(t, g_iw.t->jd_); cb_jdreset(IW, &g_il); }
unsigned c_jddata(void *t, int v) { CHECK(t, g_iw.t->jd_); return cb_jddata(IW, &g_ic, &g_il, v); }
int c_isalt(void *t, short ph, short uc, float x) { CHECK(t, (void *)g_iw.a); return cb_isalt(IW, &g_ic, &g_il, ph, uc, x); }
int c_anyalt(void *t) { CHECK(t, (void *)g_iw.a); return cb_anyalt(IW, &g_ic, &g_il); }
void c_calc(void *t, short s, short m, unsigned ph) { CHECK(t, (void *)g_iw.a); cb_calc(IW, &g_ic, &g_il, s, m, ph); }
void c_upd(void *t, short ph, short uc) { CHECK(t, (void *)g_iw.a); cb_upd(IW, &g_ic, &g_il, ph, uc); }
void c_add(void *t, short ph, unsigned char uc, float x) { CHECK(t, (void *)g_iw.a); cb_add(IW, &g_il, ph, uc, x); }
void c_unite(void *t, unsigned char uc) { CHECK(t, (void *)g_iw.a); cb_unite(IW, &g_il, uc); }
void c_resetstudy(void *t, unsigned v) { CHECK(t, (void *)g_iw.a); cb_resetstudy(IW, &g_il, v); }
int c_study(void *t, float x, unsigned ph) { CHECK(t, (void *)g_iw.a); return cb_study(IW, &g_ic, &g_il, x, ph); }
void c_first(void *t) { CHECK(t, (void *)g_iw.a); cb_first(IW, &g_il); }
void c_second(void *t) { CHECK(t, (void *)g_iw.a); cb_second(IW, &g_il); }
void c_qcmap(void *t) { CHECK(t, (void *)g_iw.a); cb_qcmap(IW, &g_il); }
void c_adjust(void *t, short v) { CHECK(t, (void *)g_iw.a); cb_adjust(IW, &g_il, v); }
int c_lin_a(int v) { return cb_lin(IW, &g_ic, &g_il, 1, v); }
int c_lin_u(int v) { return cb_lin(IW, &g_ic, &g_il, 0, v); }
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
static int16_t m_neg(int16_t v) { return (int16_t)(-(int)v); }
static int m_abs(int16_t m) { int v = m; return v < 0 ? -v : v; }
/* the G.711 code of |m|: A-law: low byte of the coder ^ 0xd5, mu-law: complement of the low byte */
static unsigned m_code(unsigned pcm, int16_t m) {
  if (pcm) { int r = cb_lin(MW, MC, ML, 1, m_abs(m)); return ((unsigned)r & 0xffu) ^ 0xd5u; }
  { int r = cb_lin(MW, MC, ML, 0, m_abs(m)); return 0xffu - ((unsigned)r & 0xffu); }
}
static int16_t m_tab(const W *w, int which, unsigned phase16, unsigned uc) {   /* which: 0 base, 1 alt, 2 prev(ph ignored; zero-extended) */
  const uint8_t *b = (const uint8_t *)w->a;
  unsigned off = which == 0 ? offsetof(AdidObj, linearMapping_) : which == 1 ? offsetof(AdidObj, linearMappingAlt_) : offsetof(AdidObj, prevSessionLinearMapping_);
  unsigned idx = which == 2 ? uc : ((phase16 & 0xffffu) << 7) + uc;
  uint16_t v;
  memcpy(&v, b + off + 2u * idx, 2);
  return (int16_t)v;
}
static int16_t m_altph(const W *w, unsigned ph) { uint16_t v; memcpy(&v, (const uint8_t *)w->a + offsetof(AdidObj, altRbsPhase_) + 2u * ph, 2); return (int16_t)v; }
static unsigned m_susp(const W *w, unsigned ph) { return ((const uint8_t *)w->a)[offsetof(AdidObj, suspectedPhase_) + ph]; }
static int16_t m_sgn(int16_t s) { return s < 0 ? -1 : 1; }
/* sign(s) * table value as a 16-bit result; the prev table is read zero-extended */
static int16_t m_scaled(const W *w, int which, int16_t s, unsigned ph, unsigned uc) {
  int v = which == 2 ? (int)(uint16_t)m_tab(w, which, ph, uc) : (int)m_tab(w, which, ph, uc);
  return (int16_t)(v * (int)m_sgn(s));
}
static void m_ed(W *w, Log *l, const char *fmt, unsigned a) { char b[TXTMAX]; snprintf(b, sizeof b, fmt, (int)a); cb_ed(w, l, 'e', b); }
static void m_ed0(W *w, Log *l, const char *s) { cb_ed(w, l, 'e', s); }
static void m_dbg(W *w, Log *l, unsigned lvl, const char *fmt, int havearg, unsigned a) {
  if (lvl > 1) { char b[TXTMAX]; if (havearg) snprintf(b, sizeof b, fmt, (int)a); else snprintf(b, sizeof b, "%s", fmt); cb_ed(w, l, 'd', b); }
}
static void m_adv(T *t) { t->phase_ = (t->phase_ + 1 == 6) ? 0 : t->phase_ + 1; }
static int m_eq64(unsigned cnt, uint64_t v) { return (uint64_t)cnt == v; }
#define PFX "V90Phase3Demodulator: "

/* the bit decision of the states 4 5 7 8 9 */
static int16_t m_bit(W *w, float x, int *b2) {
  T *t = w->t;
  int alt = 0, bit;
  int16_t v, r;
  if ((int16_t)w->a->studyDone_[0] != 0) {
    int ia = cb_isalt(w, MC, ML, (int16_t)t->phase_, (int16_t)t->code_, x);
    alt = (int16_t)ia != 0;
    if (alt) CV(G_ANY, "bit: studyDone, isAltRbs true -> alt table"); else CV(G_ANY, "bit: studyDone, isAltRbs false -> base table");
  } else CV(G_ANY, "bit: study not done -> base table");
  v = m_tab(w, alt, t->phase_, t->code_);
  if (x > 0.0f) { bit = 1; r = v; CV(G_ANY, "bit: x > 0"); }
  else { bit = 0; r = m_neg(v); CV(G_ANY, "bit: x <= 0 (or NaN)"); }
  *b2 = cb_descr(w, MC, ML, cb_sdd(w, MC, ML, bit));
  return r;
}
static void m_trn1d_tmo(W *w) {
  T *t = w->t;
  if (m_eq64(t->cnt_, 36000)) { CV(G_NODIL, "TRN1d time-out"); t->state_ = 0x16; t->cnt_ = 0; t->event_ = 0x15; m_ed0(w, ML, PFX "TRN1dDemod TimeOut\r\n"); }
}
static void m_term(W *w) {
  T *t = w->t;
  if (t->mod_event_ == 6) { CV(G_DIL, "DIL: modulator terminated"); m_ed(w, ML, PFX "Phase3 Terminated @ %d\r\n", t->cnt_); t->state_ = 0x13; t->cnt_ = 0; t->event_ = 0x14; }
}
static void m_dil_tmo(W *w, const char *msg) {
  T *t = w->t;
  if (m_eq64(t->cnt_, 40000)) { CV(G_DIL, "DIL: 40000 time-out"); t->state_ = 0x18; t->cnt_ = 0; t->event_ = 0x15; m_ed0(w, ML, msg); }
}
static void m_copy438(W *w) { memcpy(&w->p->phase4MeanErrorBefToAftUpdateRatioThresh, &w->p->unk440, 4); }

static int16_t model(W *w, float x, unsigned lvl) {
  T *t = w->t;
  V90ParObj *p = w->p;
  int16_t s, m, r;
  int b2;
  { long double lx = x; s = (!(lx > -32769.0L && lx < 32768.0L)) ? (int16_t)0x8000 : (int16_t)(int)x; }
  t->cnt_++;
  t->event_ = 0;
  switch (t->state_) {
    case 0: {
      int res;
      uint32_t cnt0;
      memcpy(&cnt0, g_fake[F_SD], 4);   /* V90SdDetector::count_ (the recorder never writes it) */
      if (cnt0 == 10) { CV(G_NODIL, "st0: sd count 10 -> latch"); t->b424_ = 1; }
      if (t->b424_ != 0 && cnt0 == 0) { CV(G_NODIL, "st0: latch and count 0 -> event 9"); t->b424_ = 0; t->event_ = 9; }
      else if (t->b424_ != 0) CV(G_NODIL, "st0: latch set, count != 0");
      res = cb_sd(w, MC, ML, x);
      if (res > 0) { CV(G_NODIL, "st0: Sd detected"); m_ed(w, ML, PFX "Sd detected @ %d\r\n", t->cnt_); t->state_ = 1; t->cnt_ = 0; t->event_ = 1; }
      else if (m_eq64(t->cnt_, (uint64_t)t->timeBase_ + 12000u)) { CV(G_NODIL, "st0: WaitForSd time-out"); t->state_ = 0x14; t->cnt_ = 0; t->event_ = 0x15; m_ed0(w, ML, PFX "WaitForSd TimeOut\r\n"); }
      else CV(G_NODIL, "st0: nothing");
      return s;
    }
    case 1: {
      int res = cb_sd(w, MC, ML, x);
      if (res < 0) { CV(G_NODIL, "st1: SdNot detected"); m_ed(w, ML, PFX "SdNot detected @ %d\r\n", t->cnt_); t->state_ = 2; t->cnt_ = 0; t->event_ = 2; }
      else if (t->cnt_ == 0x180) { CV(G_NODIL, "st1: time-out"); t->state_ = 0x15; t->cnt_ = 0; t->event_ = 0x15; m_ed0(w, ML, PFX "SdDemod TimeOut\r\n"); }
      else CV(G_NODIL, "st1: nothing");
      return s;
    }
    case 2:
      if (t->cnt_ == 0x30) {
        if (t->lineMode_ == 0) { CV(G_NODIL, "st2: known data"); t->state_ = 3; m_ed0(w, ML, PFX "enter TRN1d Known Data state\r\n"); }
        else {
          CV(G_NODIL, "st2: DD");
          t->state_ = 4; t->trn1Len_ = (unsigned)p->trn1QcDdLength; t->w3f4_ = (unsigned)p->unk4a4;
          cb_jdreset(w, ML);
          m_ed0(w, ML, PFX "enter TRN1d DD state\r\n");
        }
        cb_resetstudy(w, ML, t->lineMode_);
        t->cnt_ = 0; t->event_ = 3;
      } else CV(G_NODIL, "st2: nothing");
      return s;
    case 3:
      m = cb_gen(w, MC, ML);
      if (t->cnt_ == 0x7f8) {
        if (t->jd_ == NULL) {
          CV(G_NODIL, "st3: null jd");
          t->state_ = 0x19; t->event_ = 0x15; m_dbg(w, ML, lvl, PFX "ERROR: Null JdDetector\r\n", 0, 0); t->cnt_ = 0;
        } else {
          t->state_ = 4;
          if (t->lineMode_ == 0) { CV(G_NODIL, "st3: lineMode 0"); t->trn1Len_ = (unsigned)p->trn1dDdLength; t->w3f4_ = (unsigned)p->unk344; }
          else { CV(G_NODIL, "st3: lineMode != 0"); m_dbg(w, ML, lvl, PFX "setting params for short TRN1\n", 0, 0); t->trn1Len_ = (unsigned)p->trn1QcDdLength; t->w3f4_ = (unsigned)p->unk4a4; }
          cb_resetstudy(w, ML, t->lineMode_);
          cb_jdreset(w, ML);
          m_ed0(w, ML, PFX "enter TRN1d DD state\r\n");
          t->cnt_ = 0;
        }
      } else CV(G_NODIL, "st3: nothing");
      return m;
    case 4:
      r = m_bit(w, x, &b2);
      if (t->cnt_ == t->trn1Len_) { CV(G_NODIL, "st4: length reached"); m_ed(w, ML, PFX "enter study reference Ucode state @ %d\r\n", t->cnt_); t->state_ = 5; t->cnt_ = 0; t->event_ = 4; }
      else { CV(G_NODIL, "st4: other"); m_trn1d_tmo(w); }
      return r;
    case 5:
      r = m_bit(w, x, &b2);
      t->status_ = (unsigned)cb_study(w, MC, ML, x, t->phase_);
      m_adv(t);
      if (t->status_ == 2) {
        if (t->cnt_ % 6 == 0) {
          CV(G_NODIL, "st5: study done at a frame boundary");
          if ((int16_t)cb_anyalt(w, MC, ML) != 0) { CV(G_NODIL, "st5: altRbs -> copy"); m_copy438(w); }
          m_ed(w, ML, PFX "enter Wait For Jd state @ %d\r\n", t->cnt_);
          t->state_ = 6; t->cnt_ = 0; t->event_ = 5;
        } else CV(G_NODIL, "st5: study 2, not at a boundary");
        t->status_ = 1;
      } else { CV(G_NODIL, "st5: study != 2"); m_trn1d_tmo(w); }
      return r;
    case 6:
      r = m_bit(w, x, &b2);
      m_adv(t);
      if ((uint8_t)cb_jddata(w, MC, ML, b2) != 0) {
        if (t->cnt_ % 6 != 0 && t->lineMode_ == 0) { CV(G_NODIL, "st6: frame, not at boundary -> reset"); cb_jdreset(w, ML); }
        else {
          CV(G_NODIL, "st6: frame detected");
          m_dbg(w, ML, lvl, PFX "waitForJd framePosition = %d\n", 1, t->phase_);
          if (t->phase_ != 0) { CV(G_NODIL, "st6: phase adjust"); m_dbg(w, ML, lvl, PFX "adjustUinfoToPhaseOffset\n", 0, 0); cb_adjust(w, ML, (int16_t)t->phase_); t->phase_ = 0; }
          m_ed(w, ML, PFX "Jd detected @ %d\r\n", t->cnt_);
          t->event_ = 6; t->state_ = 9; t->cnt_ = 0; t->jdNotCnt_ = 0;
        }
      } else CV(G_NODIL, "st6: no frame");
      m_trn1d_tmo(w);
      return r;
    case 9:
      r = m_bit(w, x, &b2);
      m_adv(t);
      if (b2 != 0) t->jdNotCnt_ = 0; else t->jdNotCnt_++;
      if (t->jdNotCnt_ > 11 && t->cnt_ % 72 == 12) {
        m_ed(w, ML, PFX "JdNot detected @ %d\r\n", t->cnt_);
        t->event_ = 8;
        if (t->lineMode_ == 0) { t->state_ = p->probingMode == 0 ? 0xa : 0x11; if (p->probingMode == 0) CV(G_NODIL, "st9: JdNot, mode 0 -> 0xa"); else CV(G_NODIL, "st9: JdNot, mode != 0 -> 0x11"); }
        else { CV(G_NODIL, "st9: JdNot, QC"); t->state_ = 0xd; m_dbg(w, ML, lvl, PFX "changing state to DILDemodQCfirstStudy\n", 0, 0); }
        t->cnt_ = 0;
        cb_mreset(w, ML, t->pcm_, t->code_, 9, 0, NULL, NULL, t->dil_, 0);
      } else {
        if (b2 == 0 && t->jdNotCnt_ > 11) CV(G_NODIL, "st9: run > 11 but not at the position");
        if (m_eq64(t->cnt_, (uint64_t)t->timeBase_ + 38760u)) { CV(G_NODIL, "st9: time-out"); t->state_ = 0x17; t->cnt_ = 0; t->event_ = 0x15; m_ed0(w, ML, PFX "JdDemod TimeOut\r\n"); }
      }
      return r;
    case 10: case 13: {
      int st = (int)t->state_;
      m = cb_gen(w, MC, ML);
      r = s;
      if (t->mod_isRef_ != 0) {
        int alt = (int16_t)cb_isalt(w, MC, ML, (int16_t)t->phase_, (int16_t)t->code_, x) != 0;
        if (alt) CV(G_DIL, "st10/13: ref, alt table"); else CV(G_DIL, "st10/13: ref, base table");
        r = m_scaled(w, alt, s, t->phase_, m_code(t->pcm_, m));
      } else CV(G_DIL, "st10/13: not a reference symbol");
      t->status_ = 1;
      cb_calc(w, MC, ML, s, m, t->phase_);
      m_adv(t);
      if (t->cnt_ == (unsigned)p->unk30c) {
        CV(G_DIL, "st10/13: first study done");
        cb_first(w, ML);
        if ((int16_t)cb_anyalt(w, MC, ML) != 0) m_copy438(w);
        t->cnt_ = 0; t->state_ = st == 10 ? 0xb : 0xe;
      } else m_dil_tmo(w, st == 10 ? PFX "DILDemodFirstStudy TimeOut\r\n" : PFX "DILDemodQCfirstStudy TimeOut\r\n");
      m_term(w);
      return r;
    }
    case 11: {
      m = cb_gen(w, MC, ML);
      cb_calc(w, MC, ML, s, m, t->phase_);
      if (m_altph(w, t->phase_) == 0) {
        if (m_abs(m) != (int)(int16_t)t->level_) { CV(G_DIL, "st11: alt 0, |m| != level: update"); cb_upd(w, MC, ML, (int16_t)t->phase_, (int16_t)m_code(t->pcm_, m)); }
        else CV(G_DIL, "st11: alt 0, |m| == level");
        r = m_scaled(w, 0, s, t->phase_, m_code(t->pcm_, m));
      } else if (m_abs(m) == (int)(int16_t)t->level_) {
        int alt = (int16_t)cb_isalt(w, MC, ML, (int16_t)t->phase_, (int16_t)t->code_, x) != 0;
        if (alt) CV(G_DIL, "st11: alt phase, |m| == level, alt table"); else CV(G_DIL, "st11: alt phase, |m| == level, base table");
        r = m_scaled(w, alt, s, t->phase_, m_code(t->pcm_, m));
      } else {
        CV(G_DIL, "st11: alt phase, |m| != level: store");
        cb_add(w, ML, (int16_t)t->phase_, (uint8_t)m_code(t->pcm_, m), x);
        r = s;
      }
      t->status_ = 1;
      if (t->mod_segCnt_ == 0 && t->mod_code_ != t->code_) { CV(G_DIL, "st11: unite"); cb_unite(w, ML, t->mod_code_); }
      if (t->cnt_ == (unsigned)p->unk314) { CV(G_DIL, "st11: event 0xc"); t->event_ = 0xc; }
      if (t->cnt_ == (unsigned)p->unk300) {
        if ((int16_t)t->flag400_ != 0) { CV(G_DIL, "st11: event 0x10"); t->event_ = 0x10; p->unk31c = p->unk320; } else { CV(G_DIL, "st11: event 0xa"); t->event_ = 0xa; }
      }
      m_adv(t);
      if (t->cnt_ == (unsigned)p->unk308) {
        if (p->unk310 == 0) { CV(G_DIL, "st11: end -> 0x10 (second study)"); cb_second(w, ML); t->event_ = 0x11; t->state_ = 0x10; t->cnt_ = 0; }
        else { CV(G_DIL, "st11: end -> 0xc"); t->event_ = 0x10; t->state_ = 0xc; t->cnt_ = 0; }
      } else m_dil_tmo(w, PFX "DILDemodSecondStudy TimeOut\r\n");
      m_term(w);
      return r;
    }
    case 12: {
      m = cb_gen(w, MC, ML);
      if (m_altph(w, t->phase_) == 0) {
        if (t->mod_isRef_ == 0) { CV(G_DIL, "st12: alt 0, not ref: calc + update"); cb_calc(w, MC, ML, s, m, t->phase_); cb_upd(w, MC, ML, (int16_t)t->phase_, (int16_t)m_code(t->pcm_, m)); }
        else CV(G_DIL, "st12: alt 0, ref");
        r = m_scaled(w, 0, s, t->phase_, m_code(t->pcm_, m));
      } else {
        r = s;
        if (t->mod_isRef_ == 0) { CV(G_DIL, "st12: alt phase, not ref: store"); cb_add(w, ML, (int16_t)t->phase_, (uint8_t)m_code(t->pcm_, m), x); }
        else CV(G_DIL, "st12: alt phase, ref");
      }
      m_adv(t);
      t->status_ = 0;
      if (t->mod_segCnt_ == 0 && t->mod_code_ != t->code_) { CV(G_DIL, "st12: unite"); cb_unite(w, ML, t->mod_code_); }
      if (t->cnt_ == (unsigned)p->unk310) { CV(G_DIL, "st12: end"); cb_second(w, ML); t->cnt_ = 0; t->event_ = 0x11; t->state_ = 0x10; }
      else m_dil_tmo(w, PFX "DILDemodThirdStudyStage TimeOut\r\n");
      m_term(w);
      return r;
    }
    case 14: {
      m = cb_gen(w, MC, ML);
      cb_calc(w, MC, ML, s, m, t->phase_);
      if (m_altph(w, t->phase_) == 0) {
        if (m_abs(m) == (int)(int16_t)t->level_) { CV(G_DIL, "st14: alt 0, |m| == level"); r = m_scaled(w, 0, s, t->phase_, m_code(t->pcm_, m)); }
        else if (m_susp(w, t->phase_) == 0) { CV(G_DIL, "st14: alt 0, |m| != level, not suspected: prev table"); r = m_scaled(w, 2, s, 0, m_code(t->pcm_, m)); }
        else { CV(G_DIL, "st14: alt 0, |m| != level, suspected"); r = s; }
      } else if (m_abs(m) == (int)(int16_t)t->level_) {
        int alt = (int16_t)cb_isalt(w, MC, ML, (int16_t)t->phase_, (int16_t)t->code_, x) != 0;
        if (alt) CV(G_DIL, "st14: alt phase, |m| == level, alt table"); else CV(G_DIL, "st14: alt phase, |m| == level, base table");
        r = m_scaled(w, alt, s, t->phase_, m_code(t->pcm_, m));
      } else {
        CV(G_DIL, "st14: alt phase, |m| != level: store");
        cb_add(w, ML, (int16_t)t->phase_, (uint8_t)m_code(t->pcm_, m), x);
        r = s;
      }
      t->status_ = 1;
      if (t->cnt_ == (unsigned)p->unk314) t->event_ = 0xc;
      if (t->cnt_ == (unsigned)p->unk300) {
        if ((int16_t)t->flag400_ != 0) { t->event_ = 0x10; p->unk31c = p->unk320; } else t->event_ = 0xa;
      }
      m_adv(t);
      if (t->cnt_ == (unsigned)p->unk308) { CV(G_DIL, "st14: end"); t->cnt_ = 0; t->event_ = 0x10; t->state_ = 0xf; }
      else m_dil_tmo(w, PFX "DILDemodQCsecondStudy TimeOut\r\n");
      m_term(w);
      return r;
    }
    case 15: {
      m = cb_gen(w, MC, ML);
      if (t->mod_isRef_ == 0) { CV(G_DIL, "st15: not ref: calc"); cb_calc(w, MC, ML, s, m, t->phase_); } else CV(G_DIL, "st15: ref");
      if (m_altph(w, t->phase_) == 0) {
        r = s;
        if (m_susp(w, t->phase_) == 0) { CV(G_DIL, "st15: alt 0, not suspected: prev table"); r = m_scaled(w, 2, s, 0, m_code(t->pcm_, m)); } else CV(G_DIL, "st15: alt 0, suspected");
      } else {
        r = s;
        if (t->mod_isRef_ == 0) { CV(G_DIL, "st15: alt phase, not ref: store"); cb_add(w, ML, (int16_t)t->phase_, (uint8_t)m_code(t->pcm_, m), x); }
      }
      m_adv(t);
      t->status_ = 0;
      if (t->cnt_ == (unsigned)p->unk310) { CV(G_DIL, "st15: end"); cb_qcmap(w, ML); t->cnt_ = 0; t->event_ = 0x11; t->state_ = 0x10; }
      else m_dil_tmo(w, PFX "DILDemodQCthirdStudy TimeOut\r\n");
      m_term(w);
      return r;
    }
    case 16: {
      m = cb_gen(w, MC, ML);
      r = s;
      if (m_altph(w, t->phase_) == 0) { CV(G_DIL, "st16: alt 0"); r = m_scaled(w, 0, s, t->phase_, m_code(t->pcm_, m)); }
      else if (t->mod_isRef_ != 0) {
        int alt = (int16_t)cb_isalt(w, MC, ML, (int16_t)t->phase_, (int16_t)t->code_, x) != 0;
        if (alt) CV(G_DIL, "st16: alt phase, ref, alt table"); else CV(G_DIL, "st16: alt phase, ref, base table");
        r = m_scaled(w, alt, s, t->phase_, m_code(t->pcm_, m));
      } else CV(G_DIL, "st16: alt phase, not ref");
      m_adv(t);
      if ((uint8_t)t->flag3f9_ != 0) {
        t->cnt3fc_++;
        if (t->cnt3fc_ == (unsigned)p->unk31c) { CV(G_DIL, "st16: counter reached"); t->event_ = 0xf; t->status_ = 1; }
      } else CV(G_DIL, "st16: counter off");
      if (t->cnt_ == (unsigned)p->unk318) { CV(G_DIL, "st16: end"); t->event_ = 0x12; }
      else m_dil_tmo(w, PFX "DILDemodErrorRelaxation TimeOut\r\n");
      m_term(w);
      return r;
    }
    case 17:
      m = cb_gen(w, MC, ML);
      t->status_ = (m_abs(m) == 0xffc && t->mod_segCnt_ > (unsigned)p->dfeLength) ? 1u : 0u;
      if (t->status_) CV(G_DIL, "st17: status 1"); else CV(G_DIL, "st17: status 0");
      if (t->cnt_ == t->dilLen_) { CV(G_DIL, "st17: probing ended"); m_ed0(w, ML, PFX "Probing DIL ended\r\n"); t->event_ = 0x13; }
      else if (t->cnt_ == 40000) { CV(G_DIL, "st17: time-out"); t->state_ = 0x18; t->cnt_ = 0; t->event_ = 0x15; m_ed0(w, ML, PFX "ProbingDILDemod TimeOut\r\n"); }
      return s;
    case 19: case 20: case 21: case 22: case 23: case 24: case 25:
      CV(G_NODIL, "st19..25: no output");
      return 0;
    case 26: {
      int res;
      cb_ans(w, MC, ML, x);
      res = cb_sd(w, MC, ML, x);
      if (res > 0) { CV(G_NODIL, "st26: QTS"); m_ed(w, ML, PFX "QTS detected @ %d\r\n", t->cnt_); t->state_ = 0x1b; t->cnt_ = 0; t->event_ = 0x37; }
      else if (t->cnt_ == (unsigned)p->anspcmDemodulationLength) { CV(G_NODIL, "st26: time-out"); t->state_ = 0x20; t->cnt_ = 0; t->verify_ = 0; t->event_ = 0x3a; m_ed0(w, ML, PFX "WaitFor Qts TimeOut, decision is set to not same line\r\n"); }
      else CV(G_NODIL, "st26: nothing");
      return s;
    }
    case 27: {
      int res;
      cb_ans(w, MC, ML, x);
      res = cb_sd(w, MC, ML, x);
      if (res < 0) { CV(G_NODIL, "st27: QTSNot"); m_ed(w, ML, PFX "QTSNot detected @ %d\r\n", t->cnt_); t->state_ = 0x1c; t->cnt_ = 0; t->event_ = 0x38; }
      else if (t->cnt_ == 0x300) {
        CV(G_NODIL, "st27: time-out");
        t->state_ = 0x21; t->cnt_ = 0; t->verify_ = 0; m_ed0(w, ML, PFX "WaitFor QtsNot TimeOut, decision is set to not same line\r\n");
        if ((((const uint8_t *)g_fake[F_MP])[0] & 1) != 0) { CV(G_NODIL, "st27: flag -> 0x320"); p->anspcmDemodulationLength = 0x320; }
      } else CV(G_NODIL, "st27: nothing");
      return s;
    }
    case 28:
      if (t->cnt_ == 0x30) {
        CV(G_NODIL, "st28: enter ANSpcm");
        t->state_ = 0x1d; t->cnt_ = 0;
        if ((((const uint8_t *)g_fake[F_MP])[0] & 1) != 0) { CV(G_NODIL, "st28: flag -> 0x320"); p->anspcmDemodulationLength = 0x320; }
        t->event_ = 0x39; t->verify_ = 1;
        cb_ansreset(w, ML);
        m_ed0(w, ML, PFX "enter ANSpcm demod state\r\n");
      } else CV(G_NODIL, "st28: nothing");
      return s;
    case 29:
      cb_ans(w, MC, ML, x);
      if (t->cnt_ == (unsigned)p->anspcmDemodulationLength) { CV(G_NODIL, "st29: event 0x3a"); t->event_ = 0x3a; } else CV(G_NODIL, "st29: nothing");
      return s;
    case 30:
      if (cb_ans(w, MC, ML, x) == 0) { CV(G_NODIL, "st30: drop"); m_dbg(w, ML, lvl, PFX "ANSpcm energy drop detected...\r\n", 0, 0); t->event_ = 0x3b; t->state_ = 0x1f; }
      else CV(G_NODIL, "st30: still present");
      return s;
    case 31:
      CV(G_NODIL, "st31: nothing");
      return s;
    case 32:
      if (cb_ans(w, MC, ML, x) != 0) { CV(G_NODIL, "st32: ANSpcm detected"); m_ed0(w, ML, PFX "ANSpcm detected on recovery, move to wait for drop...\r\n"); t->event_ = 0x3a; }
      if (t->cnt_ > 0x7cf) { CV(G_NODIL, "st32: faked drop"); m_ed0(w, ML, PFX "faking ANSpcm energy drop detected (QTs timeout)...\r\n"); t->event_ = 0x3b; t->state_ = 0x1f; }
      return s;
    case 33:
      if (cb_ans(w, MC, ML, x) != 0) {
        if (t->cnt_ >= (unsigned)p->anspcmDemodulationLength) { CV(G_NODIL, "st33: detected after the length"); m_ed0(w, ML, PFX "ANSpcm detected on recovery, move to wait for drop...\r\n"); t->event_ = 0x3a; }
        else CV(G_NODIL, "st33: detected before the length");
      } else CV(G_NODIL, "st33: not detected");
      return s;
    default:
      CV(G_ILL, "state 7 / 8 / 18 / above 0x21: nothing");
      return 0;
  }
}

/* ------------------------------------------------------------------------------------------------ trial setup */
static unsigned rnd_range(unsigned lo, unsigned hi) { return lo + h_rmod(hi - lo + 1); }
static unsigned pick_pat32(void) {
  static const unsigned v[] = { 0, 1, 2, 0x7fffffffu, 0x80000000u, 0xffffffffu, 0xfffffffeu, 0x30, 0x180, 0x300, 0x7f8, 0x9c40, 0x7cf, 0x7d0 };
  return h_rpct(60) ? h_rnd() : v[h_rmod(sizeof v / sizeof v[0])];
}
static unsigned pick_level(void) {
  static const unsigned lv[] = { 0, 1, 2, 3, 7, 0x80000000u, 0xffffffffu, 0x80000001u };
  return h_rpct(50) ? 2u : lv[h_rmod(sizeof lv / sizeof lv[0])];
}
static unsigned garb(unsigned bits, unsigned lowmask) { return (h_rnd() & ~lowmask & bits) | 0u; }
/* the scripted results: the bits the function must not look at are garbage */
static void gen_script(void) {
  unsigned i;
  static const int16_t syms[] = { 0, 1, -1, 0xffc, -0xffc, 100, -100, 32767, -32768, 4000, -4000, 0x7f, 0x1234 };
  static const unsigned codes[] = { 0, 1, 0x55, 0x7f, 0x80, 0xd5, 0xff, 0x2a };
  for (i = 0; i < NSC; i++) {
    g_sc.sym[i] = h_rpct(75) ? syms[h_rmod(sizeof syms / sizeof syms[0])] : (int16_t)h_rnd();
    g_sc.mev[i] = (uint8_t)(h_rpct(30) ? 6u : h_rpct(50) ? 0u : h_rmod(10));
    g_sc.mrefw[i] = h_rpct(45) ? 0u : h_rpct(70) ? 1u : (h_rnd() & 0xffffu) | 1u;
    g_sc.msegw[i] = h_rpct(40) ? 0u : h_rpct(50) ? rnd_range(1, 12) : h_rnd();
    g_sc.mcode[i] = (uint8_t)(h_rpct(50) ? codes[h_rmod(8)] : h_rnd());
    g_sc.sdd[i] = h_rpct(60) ? (int)h_rmod(2) : (int)h_rnd();
    g_sc.descr[i] = h_rpct(40) ? 0 : h_rpct(70) ? 1 : (int)h_rnd();
    g_sc.sd[i] = h_rpct(30) ? 0 : h_rpct(30) ? 1 : h_rpct(30) ? -1 : h_rpct(50) ? (int)h_rnd() : (int)(h_rpct(50) ? 0x80000000u : 0x7fffffffu);
    g_sc.ans[i] = h_rpct(40) ? 0u : h_rpct(50) ? 1u : h_rnd();
    g_sc.jdd[i] = h_rpct(40) ? (garb(0xffffff00u, 0xffu)) : (garb(0xffffff00u, 0xffu) | (h_rpct(60) ? 1u : (h_rnd() & 0xffu) | 1u));
    g_sc.alt[i] = (int)(garb(0xffff0000u, 0xffffu) | (h_rpct(50) ? 0u : (h_rpct(50) ? 1u : (h_rnd() & 0xffffu) | 1u)));
    g_sc.anyalt[i] = (int)(garb(0xffff0000u, 0xffffu) | (h_rpct(50) ? 0u : (h_rpct(50) ? 1u : (h_rnd() & 0xffffu) | 1u)));
    g_sc.study[i] = h_rpct(35) ? 2 : h_rpct(50) ? (int)h_rmod(4) : (int)h_rnd();
    g_sc.lina[i] = (h_rnd() & 0xffffff00u) | ((codes[h_rmod(8)] ^ 0xd5u) & 0xffu);
    g_sc.linu[i] = (h_rnd() & 0xffffff00u) | ((0xffu - codes[h_rmod(8)]) & 0xffu);
    g_sc.updv[i] = h_rnd();
    g_sc.calcv[i] = h_rnd();
    g_sc.calcu[i] = codes[h_rmod(8)];
  }
}
static void fill_random(void *p, size_t n) { uint8_t *q = (uint8_t *)p; size_t i; for (i = 0; i < n; i++) q[i] = (uint8_t)h_rnd(); }
static void put_i(int32_t *d, unsigned v) { *d = (int32_t)v; }
static void put_f(float *d, uint32_t v) { memcpy(d, &v, 4); }

static void new_world(void) {
  int k;
  uint8_t *raw = (uint8_t *)h_obj_new(SZ + CAN);
  T *t;
  AdidObj *a;
  V90ParObj *p;
  unsigned i;
  g_iw.t = t = (T *)raw; g_can = raw + SZ;
  g_iw.a = a = (AdidObj *)h_obj_new(sizeof(AdidObj));
  g_iw.p = p = (V90ParObj *)h_obj_new(sizeof(V90ParObj));
  fill_random(a, 0x1000);   /* the tables and what lies behind them (the pool fill depends on the object sizes, so it differs on x86-64) */
  for (k = 0; k < FAKEN; k++) { g_fake[k] = sysdep_malloc(0x40); fill_random(g_fake[k], 0x40); }
  memset(raw, 0, SZ);
  memset(raw + SZ, 0x5a, CAN);
  /* the object: pointer members, then member-wise random content */
  t->adid_ = a; t->params_ = p; t->sd_ = g_fake[F_SD]; t->ans_ = g_fake[F_ANS]; t->jd_ = h_rpct(8) ? NULL : g_fake[F_JD]; t->jd92_ = g_fake[F_JD92]; t->dil_ = g_fake[F_DIL];
  t->phase_ = h_rpct(92) ? h_rmod(6) : h_rmod(8);
  t->session_ = h_rnd();
  t->pcm_ = h_rpct(45) ? 0u : h_rpct(70) ? 1u : h_rnd();
  t->timeBase_ = h_rpct(20) ? 0xfffffff0u - h_rmod(30000) : h_rpct(40) ? h_rmod(5000) : h_rnd();
  t->code_ = (uint8_t)h_rnd();
  t->pad19_[0] = (uint8_t)h_rnd();
  t->level_ = (uint16_t)h_rnd();
  t->state_ = 0;
  t->cnt_ = h_rnd();
  t->event_ = h_rnd();
  t->mod_flag_ = h_rnd(); t->mod_event_ = h_rnd(); t->mod_segCnt_ = h_rnd(); t->mod_isRef_ = (uint16_t)h_rnd(); t->mod_code_ = (uint8_t)h_rnd();
  t->sdd_state_ = (int32_t)h_rnd();
  t->w3f4_ = h_rnd(); t->impairInfo_ = (uint8_t)h_rnd();
  t->flag3f9_ = (uint8_t)(h_rpct(50) ? 0u : h_rpct(70) ? 1u : h_rnd());
  t->cnt3fc_ = h_rpct(60) ? h_rmod(20) : h_rnd();
  t->flag400_ = (uint16_t)(h_rpct(45) ? 0u : h_rpct(60) ? 1u : (h_rnd() & 0xffffu));
  t->jdNotCnt_ = h_rpct(60) ? h_rmod(16) : h_rnd();
  t->status_ = h_rnd();
  t->dilLen_ = h_rpct(50) ? h_rmod(60000) : h_rnd();
  t->lineMode_ = h_rpct(50) ? 0u : h_rpct(60) ? 1u : h_rnd();
  t->arg9_ = (uint16_t)h_rnd();
  put_f(&t->arg10_, h_rnd());
  t->verify_ = h_rnd(); t->trn1Len_ = h_rnd(); t->b424_ = (uint8_t)h_rnd();
  for (i = 0; i < 2; i++) { t->pad3fa_[i] = (uint8_t)h_rnd(); t->pad402_[i] = (uint8_t)h_rnd(); t->pad416_[i] = (uint8_t)h_rnd(); }
  for (i = 0; i < 3; i++) t->pad425_[i] = (uint8_t)h_rnd();
  /* the detector: the tables the function reads, member-wise random */
  for (i = 0; i < 768; i++) { a->linearMapping_[i] = (uint16_t)(h_rpct(30) ? h_rmod(3) : h_rnd()); a->linearMappingAlt_[i] = (uint16_t)(h_rpct(30) ? h_rmod(3) : h_rnd()); }
  for (i = 0; i < 128; i++) a->prevSessionLinearMapping_[i] = (uint16_t)(h_rpct(30) ? h_rmod(3) : h_rnd());
  for (i = 0; i < 6; i++) { a->altRbsPhase_[i] = (uint16_t)(h_rpct(50) ? 0u : h_rpct(50) ? 1u : h_rnd()); a->suspectedPhase_[i] = (uint8_t)(h_rpct(50) ? 0u : h_rpct(50) ? 1u : h_rnd()); }
  a->unk2812_[0] = (uint8_t)h_rnd(); a->unk2812_[1] = (uint8_t)h_rnd();
  a->studyDone_[0] = (uint16_t)(h_rpct(40) ? 0u : h_rpct(60) ? 1u : (h_rnd() & 0xffffu));
  a->params_ = p;
  /* the parameter block: the members the function reads / writes */
  p->params = g_fake[F_MP];
  ((uint8_t *)g_fake[F_MP])[0] = (uint8_t)(h_rpct(50) ? 1u : h_rnd());
  p->probingMode = (int32_t)(h_rpct(50) ? 0u : h_rpct(50) ? 1u : h_rnd());
  p->dfeLength = (int32_t)(h_rpct(50) ? rnd_range(0, 12) : h_rnd());
  p->trn1dDdLength = (int32_t)(h_rpct(60) ? rnd_range(1, 60) : h_rnd());
  p->unk300 = (int32_t)rnd_range(1, 60); p->unk308 = (int32_t)rnd_range(1, 60); p->unk30c = (int32_t)rnd_range(1, 60); p->unk310 = (int32_t)(h_rpct(30) ? 0u : rnd_range(1, 60));
  p->unk314 = (int32_t)rnd_range(1, 60); p->unk318 = (int32_t)rnd_range(1, 60); p->unk31c = (int32_t)rnd_range(1, 20); p->unk320 = (int32_t)rnd_range(1, 20);
  p->unk344 = (int32_t)h_rnd();
  put_f(&p->phase4MeanErrorBefToAftUpdateRatioThresh, h_rnd());
  put_f(&p->unk440, h_rpct(10) ? 0x7fa00001u : h_rnd());   /* includes a signalling NaN: the word copy must preserve it */
  p->trn1QcDdLength = (int32_t)(h_rpct(60) ? rnd_range(1, 60) : h_rnd());
  p->unk4a4 = (int32_t)h_rnd();
  p->anspcmDemodulationLength = (int32_t)(h_rpct(70) ? rnd_range(1, 60) : h_rpct(50) ? 0x320 : h_rnd());
}

/* the input sample */
static float pick_x(void) {
  static const uint32_t b[] = {
    0x00000000u, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7f800000u, 0xff800000u, 0x3f800000u, 0xbf800000u, 0x00000001u, 0x80000001u, 0x47000000u /* 32768 */, 0xc7000000u /* -32768 */,
    0x46fffe00u /* 32767 */, 0x46ffffffu, 0xc6fffe00u /* -32767 */, 0xc7000100u /* -32769 */, 0xc7000080u /* -32768.5 */, 0x47000080u, 0x7e800000u, 0xfe800000u, 0x3f000000u, 0xbf000000u,
    0x437f0000u, 0x40400000u, 0x4e000000u
  };
  uint32_t v;
  float f;
  if (h_rpct(35)) v = b[h_rmod(sizeof b / sizeof b[0])];
  else if (h_rpct(50)) { float g = (float)(int)(h_rnd() % 70000u) - 35000.0f; g += (float)(h_rnd() & 0xff) / 256.0f; memcpy(&v, &g, 4); }
  else { v = h_rnd(); if (((v >> 23) & 0xffu) == 0xffu && (v & 0x7fffffu) != 0) v |= 0x400000u; }   /* NaN payloads: quiet only (the blob's fld / fstp would quiet a signalling NaN) */
  if (((v >> 23) & 0xffu) == 0xffu && (v & 0x7fffffu) != 0) v |= 0x400000u;
  memcpy(&f, &v, 4);
  return f;
}

/* aim the counter at one of the state's trigger values (the table is the harness' own steering, not the model) */
static void aim_cnt(T *t, const V90ParObj *p) {
  unsigned c[12], n = 0, st = t->state_;
  switch (st) {
    case 0: c[n++] = t->timeBase_ + 12000u; break;
    case 1: c[n++] = 0x180; break;
    case 2: c[n++] = 0x30; break;
    case 3: c[n++] = 0x7f8; break;
    case 4: c[n++] = t->trn1Len_; c[n++] = 36000; break;
    case 5: c[n++] = 6u * (1u + h_rmod(5)); c[n++] = 36000; break;
    case 6: c[n++] = 6u * (1u + h_rmod(5)); c[n++] = 36000; c[n++] = 6u * (1u + h_rmod(5)) + 1u; break;
    case 9: c[n++] = 72u * h_rmod(5) + 12u; c[n++] = t->timeBase_ + 38760u; c[n++] = 72u * h_rmod(5) + 11u; break;
    case 10: case 13: c[n++] = (unsigned)p->unk30c; c[n++] = 40000; break;
    case 11: c[n++] = (unsigned)p->unk314; c[n++] = (unsigned)p->unk300; c[n++] = (unsigned)p->unk308; c[n++] = 40000; break;
    case 12: c[n++] = (unsigned)p->unk310; c[n++] = 40000; break;
    case 14: c[n++] = (unsigned)p->unk314; c[n++] = (unsigned)p->unk300; c[n++] = (unsigned)p->unk308; c[n++] = 40000; break;
    case 15: c[n++] = (unsigned)p->unk310; c[n++] = 40000; break;
    case 16: c[n++] = (unsigned)p->unk318; c[n++] = 40000; break;
    case 17: c[n++] = t->dilLen_; c[n++] = 40000; break;
    case 26: c[n++] = (unsigned)p->anspcmDemodulationLength; break;
    case 27: c[n++] = 0x300; break;
    case 28: c[n++] = 0x30; break;
    case 29: case 33: c[n++] = (unsigned)p->anspcmDemodulationLength; c[n++] = (unsigned)p->anspcmDemodulationLength + 1u; break;
    case 32: c[n++] = 0x7cf; c[n++] = 0x7d0; break;
    default: break;
  }
  if (n == 0 || !h_rpct(70)) { t->cnt_ = h_rpct(50) ? h_rmod(80) : pick_pat32(); return; }
  t->cnt_ = c[h_rmod(n)] - 1u - (h_rpct(12) ? (h_rmod(3) + 0u) : 0u) + (h_rpct(8) ? 1u : 0u);
}
/* the model-side hooks: aim the other members at the state's decision logic */
static void aim_members(T *t, V90ParObj *p, AdidObj *a) {
  unsigned st = t->state_;
  if (st == 9) {
    unsigned k;
    t->jdNotCnt_ = h_rpct(60) ? 10u + h_rmod(3) : h_rmod(20);
    if (h_rpct(12)) t->jdNotCnt_ = 0x80000000u + h_rmod(14);    /* the run counter's compare with 11 is unsigned */
    if (h_rpct(60)) for (k = 0; k < NSC; k++) g_sc.descr[k] = h_rpct(85) ? 0 : 1;   /* a run of zero bits */
  }
  if (st == 11 || st == 14) {
    unsigned k = h_rpct(70) ? 0u : h_rmod(NSC);   /* the first symbol of the call is the one the function sees */
    if (h_rpct(60)) t->level_ = (uint16_t)(h_rpct(50) ? (unsigned)(g_sc.sym[k] < 0 ? -(int)g_sc.sym[k] : g_sc.sym[k]) : (unsigned)(-(int)g_sc.sym[k]));
    if (h_rpct(50)) { unsigned ph = t->phase_ > 5 ? 0u : t->phase_; a->altRbsPhase_[ph] = (uint16_t)(h_rpct(60) ? 1u : h_rnd() | 1u); }
  }
  if (st == 16 && h_rpct(60)) { t->flag3f9_ = 1; t->cnt3fc_ = (unsigned)p->unk31c - 1u - (h_rpct(20) ? 1u : 0u); }
  if (st == 17 && h_rpct(40)) { unsigned k; for (k = 0; k < NSC; k++) if (h_rpct(30)) g_sc.sym[k] = h_rpct(50) ? 0xffc : -0xffc; if (h_rpct(60)) t->mod_segCnt_ = (unsigned)p->dfeLength + h_rmod(3); }
  if (st == 3 && h_rpct(30)) t->jd_ = NULL;
  if (st == 0) {   /* the Sd detector's hit counter and the latch */
    uint32_t c = h_rpct(35) ? 10u : h_rpct(50) ? 0u : h_rpct(50) ? h_rmod(12) : h_rnd();
    memcpy(g_fake[F_SD], &c, 4);
    if (h_rpct(60)) t->b424_ = (uint8_t)(h_rpct(50) ? 1u : h_rpct(50) ? 0u : h_rnd());
  }
  /* states 4 5 6 9 read the frame position as a 16-bit value for the table index (movzwl): garbage above bit 15 must not matter there */
  if ((st == 4 || st == 5 || st == 6 || st == 9) && h_rpct(10)) t->phase_ = ((1u + h_rmod(0xfffe)) << 16) | h_rmod(6);
}
static unsigned pick_state(int sc) {
  static const unsigned nodil[] = { 0, 1, 2, 3, 4, 5, 6, 9, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33 };
  if (sc == 0) return nodil[h_rmod(sizeof nodil / sizeof nodil[0])];
  if (sc == 1) return 10u + h_rmod(8);
  if (h_rpct(10)) { static const unsigned ill[] = { 7, 8, 18, 34, 35, 0x40, 0xff, 0x10000, 0x80000000u, 0xffffffffu, 0x100000a }; return ill[h_rmod(sizeof ill / sizeof ill[0])]; }
  if (h_rpct(10)) return pick_state(0);
  return h_rmod(34);
}

/* one call: model on its copy first, then the impl, then compare everything */
static unsigned long g_ncalls, g_nskipped;
static int g_showall;
static void copy_world_to_model(void) {
  memcpy(&g_mt, g_iw.t, SZ);
  memcpy(&g_ma, g_iw.a, sizeof g_ma);
  memcpy(&g_mp, g_iw.p, sizeof g_mp);
  g_mw.t = &g_mt; g_mw.a = &g_ma; g_mw.p = &g_mp;
}
static const char *g_what = "setup";
static void cmp_world(const char *what) {
  if (memcmp(&g_mt, g_iw.t, SZ)) {
    const uint8_t *x = (const uint8_t *)&g_mt, *y = (const uint8_t *)g_iw.t;
    unsigned i;
    for (i = 0; i < SZ; i++) if (x[i] != y[i]) break;
    h_die3("%s: the object differs from the model at mirror offset 0x%x (model %02x, impl %02x)", what, i, x[i], y[i]);
  }
  if (memcmp(&g_ma, g_iw.a, sizeof g_ma)) {
    const uint8_t *x = (const uint8_t *)&g_ma, *y = (const uint8_t *)g_iw.a;
    unsigned i;
    for (i = 0; i < sizeof g_ma; i++) if (x[i] != y[i]) break;
    h_die3("%s: the detector differs from the model at mirror offset 0x%x", what, i);
  }
  if (memcmp(&g_mp, g_iw.p, sizeof g_mp)) h_die3("%s: the parameter block differs from the model", what);
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
  h_emit(g_show, "%s st=%u cnt=%u ev=%u ph=%u stat=%u jn=%u tl=%u w3f4=%u c3fc=%u v=%u l400=%u", tag, (unsigned)t->state_, (unsigned)t->cnt_, (unsigned)t->event_, (unsigned)t->phase_, (unsigned)t->status_,
         (unsigned)t->jdNotCnt_, (unsigned)t->trn1Len_, (unsigned)t->w3f4_, (unsigned)t->cnt3fc_, (unsigned)t->verify_, (unsigned)t->flag400_);
}
static void do_call(float x) {
  short mr, ir;
  unsigned lvl = dsplibs_debug_level, st = g_iw.t->state_;
  int ignore = (st == 7 || st == 8 || st == 18 || st > 0x21);
  copy_world_to_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  mr = model(&g_mw, x, lvl);
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  h_emit(g_show, "call st=%u cnt=%u x=%08x ph=%u pcm=%u code=%u lvl=%u", st, (unsigned)g_iw.t->cnt_, fbits(x), (unsigned)g_iw.t->phase_, (unsigned)g_iw.t->pcm_, (unsigned)g_iw.t->code_, lvl);
  ir = f_get(g_iw.t, x);
  __asm__ volatile("" ::: "memory");
  { unsigned i; for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t); }
  if (!ignore && mr != ir) h_die3("%s: result: model %d, impl %d", g_what, (int)mr, (int)ir);
  if (!ignore) h_emit(g_show, "ret %d", (int)ir);
  else h_emit(g_show, "ret (not compared: the blob returns the caller's edi)");
  log_cmp();
  cmp_world(g_what);
  { unsigned i; for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", g_what, i); }
  emit_state("end");
  h_emit(g_show, "adid base=%08x p438=%08x p31c=%d p4cc=%d", fnv(g_iw.a->linearMapping_, sizeof g_iw.a->linearMapping_), pword(&g_iw.p->phase4MeanErrorBefToAftUpdateRatioThresh), (int)g_iw.p->unk31c, (int)g_iw.p->anspcmDemodulationLength);
  cov_flush();
  g_ncalls++;
}

static void free_world(void) {
  int k;
  for (k = 0; k < FAKEN; k++) sysdep_free(g_fake[k]);
}
static void trial(int sc) {
  unsigned ncalls = (sc == 2 && h_rpct(60)) ? rnd_range(2, 6) : 1, k;
  new_world();
  gen_script();
  dsplibs_debug_level = pick_level();
  for (k = 0; k < ncalls; k++) {
    T *t = g_iw.t;
    if (k == 0 || h_rpct(40)) {
      t->state_ = pick_state(sc);
      aim_cnt(t, g_iw.p);
      aim_members(t, g_iw.p, g_iw.a);
    } else if (h_rpct(50)) aim_cnt(t, g_iw.p);
    if ((t->phase_ & 0xffffu) > 7 || ((t->phase_ >> 16) != 0 && !(t->state_ == 4 || t->state_ == 5 || t->state_ == 6 || t->state_ == 9))) t->phase_ = h_rmod(8);   /* the table reads must stay inside the part of the detector the harness fills */
    if (h_rpct(15)) gen_script();
    if (h_rpct(10)) dsplibs_debug_level = pick_level();
    g_what = "getV90Decision";
    do_call(pick_x());
  }
  free_world();
  h_trial_end("p35");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("P35_SEED");
  CovSite *c;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P35_ALL")) g_showall = 1;
  p07_layout_check(); p18_layout_check(); p34_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P35_TRIALS") ? atoi(getenv("P35_TRIALS")) : (sc == 0 ? 30000 : sc == 1 ? 30000 : 20000);
  if (sc < 0 || sc > 2) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (c = __start_covsites; c < __stop_covsites; c++) {
    int req = sc == 0 ? (c->grp == G_ANY || c->grp == G_NODIL) : sc == 1 ? (c->grp == G_DIL) : (c->grp == G_ILL);
    if (!req) continue;
    h_emit(1, "COV %s %lu", c->name, c->n);
    if (c->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", c->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P35_NOCOV")) ? 2 : 0;
}
