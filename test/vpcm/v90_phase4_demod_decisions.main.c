/*
 * Differential test harness for port chunk P37 (src/vpcm/v90_phase4_demod_decisions.cpp): V90Phase4Demodulator::getV90Decision(short) and ::getV92Decision(short), the V.90 / V.92 phase-4 RX state machines
 * (3,095 B + 3,252 B, 126 x87 instructions each).  Derived from test/vpcm/v90_phase3_demod_v90_decision.main.c: the same recorder / model / snapshot design.
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_phase4_demod_decisions.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2:
 * the port object alone, the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_phase4_demod_decisions.blob32.out).  The functions are reached through __asm__ labels; the
 * object is the generated C mirror (test/vpcm/v90_phase4_demod_decisions.layout), the objects it points to are the mirrors of P07 (V90Parameters), P09 (V90MP), P10 (V90CP), P17 (V90ConnectionEvaluator), P28 (V90Demapper):
 * member-wise access only (their layouts differ on x86-64).
 *
 * Every callee is a RECORDER with scripted results: V90Demapper::hardDecision / process (also WRITES the byte buffer and the byte count of the demodulator: the order of the buffer reads and the
 * callee calls is observable) / incrementRBSFramePosition / resetLinearMappStudy / resetNoSpectral / linearMappingStudy, V90RDetector::detectR / detectRNot (the embedded r_) / detectRf / detectRfNot (rf_)
 * (also WRITE the detector's polarity member dir_ that the function prints) / reset, V90MP::reset / bitsToInfo / printNofRecievedMpMpNot, V90CP::reset / bitsToInfo,
 * Descrambler<unsigned char, int>::process, edprintf and dsplibs_debug_printf.  The results carry garbage in the bits the function must not look at (the 16-bit hardDecision result: the upper 16 bits;
 * the V.92 Descrambler byte: the upper 24 bits of the result before the movzx).
 *
 * What is compared: the returned symbol, the whole object, the demapper / MP / CP / evaluator / parameter objects after every call, every record (callee, arguments, results, and a SNAPSHOT of the
 * demodulator's fields and the foreign words the function writes at the moment of the call: the store order before each call / print), the canary behind the object.  The INDEPENDENT MODEL (m_*:
 * written from porting/decompiled/dp/p37_phase4_demod_decisions.txt in the shape of the blob's own control flow, not from the port source; the x87 prints with its own long double code and inline fyl2x / fsqrt)
 * predicts the result, the final world and every record; a model violation exits 3.
 * Excluded: V.90 states 4, 16 and V.92 states 5, 6 and every state above 0x11 (the blob returns the CALLER's edi: the result is not compared there, everything else is), signalling NaNs in the float
 * accumulators (the blob's fld / fadd would quiet them: the harness feeds quiet NaNs only).
 *
 * Scenarios (each its own process): 0 getV90Decision, every state (illegal ones too), 1 getV92Decision, 2 both functions on one object: call sequences, extreme inputs, the echo-energy states.
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
#include "v90_connection_evaluator_layout.h"
#include "v90_mapper_layout.h"
#include "v90_phase4_demod_decisions_layout.h"

#define L(m) __asm__(m)

extern unsigned int dsplibs_debug_level;

/* the functions under test */
extern short f_get90(void *, short) L("_ZN20V90Phase4Demodulator14getV90DecisionEs");
extern short f_get92(void *, short) L("_ZN20V90Phase4Demodulator14getV92DecisionEs");

/* the interposed callees */
extern int c_hd(void *, short) L("_ZN11V90Demapper12hardDecisionEs");
extern unsigned c_proc(void *, unsigned char *, unsigned *) L("_ZN11V90Demapper7processEPhRj");
extern void c_inc(void *) L("_ZN11V90Demapper25incrementRBSFramePositionEv");
extern void c_rls(void *, unsigned) L("_ZN11V90Demapper20resetLinearMappStudyEj");
extern void c_rns(void *, void *) L("_ZN11V90Demapper15resetNoSpectralEP16V90MappingParams");
extern void c_lms(void *, short, short) L("_ZN11V90Demapper18linearMappingStudyEss");
extern int c_dR(void *, short) L("_ZN12V90RDetector7detectREs");
extern int c_dRn(void *, short) L("_ZN12V90RDetector10detectRNotEs");
extern int c_dRf(void *, short) L("_ZN12V90RDetector8detectRfEs");
extern int c_dRfn(void *, short) L("_ZN12V90RDetector11detectRfNotEs");
extern void c_rdreset(void *, unsigned, unsigned) L("_ZN12V90RDetector5resetEjj");
extern void c_mpreset(void *) L("_ZN5V90MP5resetEv");
extern int c_mpbits(void *, int) L("_ZN5V90MP10bitsToInfoEi");
extern void c_mpprint(void *) L("_ZN5V90MP23printNofRecievedMpMpNotEv");
extern void c_cpreset(void *) L("_ZN5V90CP5resetEv");
extern int c_cpbits(void *, unsigned) L("_ZN5V90CP10bitsToInfoEh");
extern int c_descr(void *, unsigned) L("_ZN11DescramblerIhiE7processEh");

typedef Ph4DemObj T;
#define SZ sizeof(T)
enum { CAN = 64, NSC = 24, LOGMAX = 256, TXTMAX = 640 };

static uint32_t fnv(const void *p, size_t n) { const uint8_t *q = (const uint8_t *)p; uint32_t h = 2166136261u; while (n--) { h ^= *q++; h *= 16777619u; } return h; }

/* ------------------------------------------------------------------------------------------------ the world: the demodulator and the objects it points to */
typedef struct { T *t; DemObj *dem; MpObj *mp; CpObj *cp; EvObj *ce; V90ParObj *p; } W;
static T g_mt;               /* the model's copies */
static DemObj g_mdem;
static MpObj g_mmp;
static CpObj g_mcp;
static EvObj g_mce;
static V90ParObj g_mpar;
static W g_iw, g_mw;         /* impl world / model world */
static uint8_t *g_can;
static void *g_fake[3];      /* mp1 / mp2 (V90MappingParams) and the descrambler: opaque heap objects */
#define F_MP1 0
#define F_MP2 1
#define F_DESCR 2
static int fid(const void *p) {
  int k;
  if (!p) return -1;
  for (k = 0; k < 3; k++) if (p == g_fake[k]) return k;
  return 9;
}

/* ------------------------------------------------------------------------------------------------ scripts, cursors, records */
typedef struct {
  uint32_t hd[NSC];            /* hardDecision results: the low 16 bits are the decision, the upper 16 garbage */
  uint32_t pr[NSC], pn[NSC];   /* V90Demapper::process: the result and the byte count */
  uint8_t pbytes[256];
  uint32_t dR[NSC], dRn[NSC], dRf[NSC], dRfn[NSC]; /* the detectors' results (the whole eax is tested) */
  uint32_t ddir[NSC];          /* the polarity the detector leaves in dir_ */
  uint32_t mpb[NSC], cpb[NSC]; /* bitsToInfo results (whole eax) */
  uint32_t descr[NSC];         /* Descrambler results (whole eax in V.90, the low byte in V.92) */
} Script;
typedef struct { unsigned hd, proc, dR, dRn, dRf, dRfn, mpb, cpb, descr; } Cur;
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
/* the fields the functions write / read between their stores (the snapshot at every record) */
static void snapshot(char *out, size_t cap, const W *w) {
  const T *t = w->t;
  snprintf(out, cap, "|st=%u cnt=%u ev=%u lim=%u cs=%u w38=%u w3c=%u w44=%u w48=%u w4c=%u eb=%08x ea=%08x kr=%u se=%u b1=%u/%u nb=%u z=%u pd=%u mpd=%u cf=%u cpd=%u",
           (unsigned)t->state_, (unsigned)t->cnt_, (unsigned)t->event_, (unsigned)t->limit_, (unsigned)t->cpSeen_, (unsigned)t->w38_, (unsigned)t->w3c_, (unsigned)t->w44_, (unsigned)t->w48_,
           (unsigned)t->w4c_, fbits(t->echoBefore_), fbits(t->echoAfter_), (unsigned)t->keepRate_, (unsigned)t->studyEnd_, (unsigned)t->b1dZeros_, (unsigned)t->b1dBits_, (unsigned)t->nBytes_,
           (unsigned)w->dem->z1eb4_, (unsigned)w->dem->phase_, (unsigned)w->mp->trn2dD_, (unsigned)w->cp->f13_, (unsigned)w->cp->trn2dD_);
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
enum { G_COM = 0, G_V90 = 1, G_V92 = 2, G_ILL = 3 };

/* ------------------------------------------------------------------------------------------------ the environment (callee semantics: shared by the impl's recorders and the model) */
#define CHECK(p, what) do { if ((p) != (what)) h_die3("callee %s got an unexpected object %p (expected %p)", __func__, (void *)(p), (void *)(what)); } while (0)

static int cb_hd(W *w, Cur *c, Log *l, int16_t x) {   /* the raw result: the decision in the low 16 bits, garbage above */
  uint32_t r = g_sc.hd[c->hd++ % NSC];
  REC(w, l, 'c', "hd(%d) -> %d", (int)x, (int)(int16_t)r);
  return (int)r;
}
/* the demapper writes the decided bytes and their count into the demodulator's buffer; with result 0 the count is garbage (the functions must not read it) */
static unsigned cb_proc(W *w, Cur *c, Log *l, uint8_t *out, uint32_t *cnt) {
  unsigned k = c->proc++ % NSC, i, n = g_sc.pn[k], r = g_sc.pr[k];
  if (r != 0) { for (i = 0; i < n; i++) out[i] = g_sc.pbytes[(i + 13u * k) & 255u]; }
  *cnt = n;
  REC(w, l, 'c', "proc -> %u n=%u", r, n);
  return r;
}
static void cb_inc(W *w, Log *l) { REC(w, l, 'c', "incRBS"); }
static void cb_rls(W *w, Log *l, unsigned v) { REC(w, l, 'c', "resetStudy(%u)", v); }
static void cb_rns(W *w, Log *l, const void *mp) { REC(w, l, 'c', "resetNoSpectral(mp%d)", fid(mp)); }
static void cb_lms(W *w, Log *l, int16_t a, int16_t b) { REC(w, l, 'c', "study(%d,%d)", (int)a, (int)b); }
/* kind: 0 r.detectR 1 r.detectRNot 2 rf.detectRf 3 rf.detectRfNot; the detector leaves its polarity in dir_ */
static int cb_det(W *w, Cur *c, Log *l, int kind, int16_t x) {
  static const char *nm[4] = { "r.detectR", "r.detectRNot", "rf.detectRf", "rf.detectRfNot" };
  unsigned *cur = kind == 0 ? &c->dR : kind == 1 ? &c->dRn : kind == 2 ? &c->dRf : &c->dRfn;
  const uint32_t *tab = kind == 0 ? g_sc.dR : kind == 1 ? g_sc.dRn : kind == 2 ? g_sc.dRf : g_sc.dRfn;
  unsigned k = (*cur)++ % NSC;
  int32_t r = (int32_t)tab[k];
  if (kind < 2) w->t->r_dir_ = (int32_t)g_sc.ddir[k]; else w->t->rf_dir_ = (int32_t)g_sc.ddir[k];
  REC(w, l, 'c', "%s(%d) -> %d dir=%d", nm[kind], (int)x, (int)r, (int)g_sc.ddir[k]);
  return r;
}
static void cb_rdreset(W *w, Log *l, int which, unsigned a, unsigned b) { REC(w, l, 'c', "%s.reset(%u,%u)", which ? "rf" : "r", a, b); }
static void cb_mpreset(W *w, Log *l) { REC(w, l, 'c', "mp.reset"); }
static int cb_mpbits(W *w, Cur *c, Log *l, int b) { int r = (int)g_sc.mpb[c->mpb++ % NSC]; REC(w, l, 'c', "mp.bits(%d) -> %d", b, r); return r; }
static void cb_mpprint(W *w, Log *l) { REC(w, l, 'c', "mp.printNofRecieved"); }
static void cb_cpreset(W *w, Log *l) { REC(w, l, 'c', "cp.reset"); }
static int cb_cpbits(W *w, Cur *c, Log *l, unsigned b) { int r = (int)g_sc.cpb[c->cpb++ % NSC]; REC(w, l, 'c', "cp.bits(%u) -> %d", b & 0xffu, r); return r; }
static int cb_descr(W *w, Cur *c, Log *l, unsigned b) { int r = (int)g_sc.descr[c->descr++ % NSC]; REC(w, l, 'c', "descr(%u) -> %d", b & 0xffu, r); return r; }
static void cb_ed(W *w, Log *l, char kind, const char *s) { REC(w, l, kind, "%s", s); }

/* the impl's recorders */
#define IW (&g_iw)
int c_hd(void *d, short x) { CHECK(d, (void *)g_iw.dem); return cb_hd(IW, &g_ic, &g_il, x); }
unsigned c_proc(void *d, unsigned char *out, unsigned *cnt) {
  CHECK(d, (void *)g_iw.dem); CHECK((void *)out, (void *)g_iw.t->buf_); CHECK((void *)cnt, (void *)&g_iw.t->nBytes_);
  return cb_proc(IW, &g_ic, &g_il, out, cnt);
}
void c_inc(void *d) { CHECK(d, (void *)g_iw.dem); cb_inc(IW, &g_il); }
void c_rls(void *d, unsigned v) { CHECK(d, (void *)g_iw.dem); cb_rls(IW, &g_il, v); }
void c_rns(void *d, void *mp) { CHECK(d, (void *)g_iw.dem); cb_rns(IW, &g_il, mp); }
void c_lms(void *d, short a, short b) { CHECK(d, (void *)g_iw.dem); cb_lms(IW, &g_il, a, b); }
int c_dR(void *d, short x) { CHECK(d, (void *)&g_iw.t->r_cnt_); return cb_det(IW, &g_ic, &g_il, 0, x); }
int c_dRn(void *d, short x) { CHECK(d, (void *)&g_iw.t->r_cnt_); return cb_det(IW, &g_ic, &g_il, 1, x); }
int c_dRf(void *d, short x) { CHECK(d, (void *)&g_iw.t->rf_cnt_); return cb_det(IW, &g_ic, &g_il, 2, x); }
int c_dRfn(void *d, short x) { CHECK(d, (void *)&g_iw.t->rf_cnt_); return cb_det(IW, &g_ic, &g_il, 3, x); }
void c_rdreset(void *d, unsigned a, unsigned b) {
  if (d == (void *)&g_iw.t->r_cnt_) cb_rdreset(IW, &g_il, 0, a, b);
  else { CHECK(d, (void *)&g_iw.t->rf_cnt_); cb_rdreset(IW, &g_il, 1, a, b); }
}
void c_mpreset(void *d) { CHECK(d, (void *)g_iw.mp); cb_mpreset(IW, &g_il); }
int c_mpbits(void *d, int b) { CHECK(d, (void *)g_iw.mp); return cb_mpbits(IW, &g_ic, &g_il, b); }
void c_mpprint(void *d) { CHECK(d, (void *)g_iw.mp); cb_mpprint(IW, &g_il); }
void c_cpreset(void *d) { CHECK(d, (void *)g_iw.cp); cb_cpreset(IW, &g_il); }
int c_cpbits(void *d, unsigned b) { CHECK(d, (void *)g_iw.cp); return cb_cpbits(IW, &g_ic, &g_il, b); }
int c_descr(void *d, unsigned b) { CHECK(d, g_fake[F_DESCR]); return cb_descr(IW, &g_ic, &g_il, b); }
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
static int16_t m_hd(W *w, int16_t x) { return (int16_t)cb_hd(w, MC, ML, x); }
static uint32_t m_w0(int which) { uint32_t v; memcpy(&v, g_fake[which], 4); return v; }
/* x87 helpers of the model: fist / fistp with the chop control word (NaN and out of range give the "integer indefinite"), abs with the cdq idiom, fsqrt and fyl2x as instructions */
static int m_fist(long double v) {
  if (v != v) return INT_MIN;
  if (v >= 2147483648.0L || v <= -2147483649.0L) return INT_MIN;
  return (int)v;
}
static int m_iabs(int v) { return (int)(v < 0 ? 0u - (unsigned)v : (unsigned)v); }
static int m_sign(long double v) { return (v > 0.0L || v != v) ? '+' : '-'; }
static long double m_fsqrt(long double v) { long double r = v; __asm__("fsqrt" : "+t"(r)); return r; }
static long double m_log10(long double v) { long double r; __asm__("fldlg2\n\tfxch %%st(1)\n\tfyl2x" : "=t"(r) : "0"(v) : "st(1)"); return r; }
static long double m_abs(long double v) { return v < 0 ? -v : v; }
/* "... = %c%d.%04d" of sqrt(e) (e a float): sign of e, trunc |sqrt|, the four digits of the fraction */
static void m_energy(W *w, const char *fmt, float e) {
  long double s = m_fsqrt((long double)e);
  int ip = m_fist(s);
  int fr = m_iabs(m_fist((s - (long double)ip) * 10000.0L));
  m_pf(w, 'e', fmt, m_sign((long double)e), m_fist(m_abs(s)), fr);
}

/* state 0 / 1 */
static int16_t m_ri(W *w, int16_t x) {
  T *t = w->t;
  if (cb_det(w, MC, ML, 0, x) != 0) { CV(G_COM, "st0: Ri detected"); m_pf(w, 'e', PFX "Ri detected @ %d, polarity = %d\r\n", (int)t->cnt_, (int)t->r_dir_); t->state_ = 1; t->cnt_ = 0; t->event_ = 0x16; }
  else CV(G_COM, "st0: nothing");
  return x;
}
static int16_t m_rinot(W *w, int16_t x) {
  T *t = w->t;
  if (cb_det(w, MC, ML, 1, x) != 0) {
    m_pf(w, 'e', PFX "RiNot detected @ %d, enter TRN2dKnownData state\r\n", (int)t->cnt_);
    t->state_ = 2; t->cnt_ = 0; t->event_ = 0x17;
    if (t->w34_ == 0) { CV(G_COM, "st1: RiNot, w34 == 0"); cb_rls(w, ML, 0x1c20); t->studyEnd_ = 0x7d0; }
    else { CV(G_COM, "st1: RiNot, w34 != 0"); cb_rls(w, ML, 0x960); t->studyEnd_ = 0x258; }
  } else CV(G_COM, "st1: nothing");
  return x;
}
/* states 2 / 3 */
static int16_t m_known(W *w, int16_t x, unsigned lvl) {
  T *t = w->t;
  int16_t hd = m_hd(w, x);
  if (t->cnt_ == t->studyEnd_) {
    CV(G_COM, "st3: study start");
    w->dem->z1eb4_ = 1;
    if (lvl > 1) { CV(G_COM, "st3: study start print"); m_pf(w, 'd', "V90Phase4Demodulator reset & enable linear mapping study in TRN2.\n"); }
  }
  if (w->dem->z1eb4_ != 0) { CV(G_COM, "st3: study runs"); cb_lms(w, ML, x, hd); }
  cb_proc(w, MC, ML, t->buf_, &t->nBytes_);
  if (t->cnt_ == (t->limit_ >> 1)) { CV(G_COM, "st3: half"); t->event_ = 0x18; }
  if (t->cnt_ == t->limit_) {
    CV(G_COM, "st3: limit");
    t->event_ = 0x19; w->dem->z1eb4_ = 0;
    if (lvl > 1) { CV(G_COM, "st3: limit print"); m_pf(w, 'd', PFX "disable linear mapping study\n"); }
  }
  return hd;
}
/* state 7: B1d */
static int16_t m_b1d(W *w, int16_t x) {
  T *t = w->t;
  int16_t hd = m_hd(w, x);
  unsigned r = cb_proc(w, MC, ML, t->buf_, &t->nBytes_);
  uint32_t w0 = m_w0(F_MP2);
  if (r != 0 && t->nBytes_ != 0) {
    unsigned i = 0;
    CV(G_COM, "st7: bytes");
    do {
      int b = cb_descr(w, MC, ML, t->buf_[i]);
      t->b1dBits_++;
      if (b == 0 && t->b1dBits_ > 3u * w0 + 0x17u) { CV(G_COM, "st7: zero after the delay"); t->b1dZeros_++; }
      i++;
    } while (t->nBytes_ > i);
  }
  if (t->cnt_ != 0x120) { CV(G_COM, "st7: not at 0x120"); return hd; }
  CV(G_COM, "st7: terminated");
  m_pf(w, 'e', PFX "Phase4 Terminated @ %d,  nof B1d bits = %d,  B1d Zeros (after delay) = %d\r\n", 0x120, (int)t->b1dBits_, (int)t->b1dZeros_);
  {
    long double num = (long double)(uint32_t)t->b1dZeros_;
    long double den = (long double)(uint32_t)(t->b1dBits_ - 3u * w0 - 0x17u);
    long double q = num / den;
    int ip = m_fist(q);
    int fr = m_iabs(m_fist((q - (long double)ip) * (long double)100000000.0f));
    long double rr = (1.0L / den) * num;
    m_pf(w, 'e', PFX "B1d BER = %c%d.%08d\r\n", m_sign(rr), m_fist(m_abs(q)), fr);
  }
  t->event_ = 0x1d; t->state_ = 8; t->cnt_ = 0; t->w3c_ = 0;
  return hd;
}
/* states 10 .. 13 */
static int16_t m_silence(W *w, int16_t x) {
  T *t = w->t;
  cb_inc(w, ML);
  if (t->cnt_ < (unsigned)w->p->rrnSilenceWaitBeforeEchoCalc) { CV(G_COM, "st10: before the wait"); return x; }
  if (t->cnt_ % 6 != 0) { CV(G_COM, "st10: not a multiple of 6"); return x; }
  CV(G_COM, "st10: -> 11");
  m_pf(w, 'e', PFX "entering CalcErrorEnergyBeforeEchoCancellation state @ %d\r\n", (int)t->cnt_);
  t->state_ = 0xb; t->cnt_ = 0;
  return x;
}
static int16_t m_echo_before(W *w, int16_t x) {
  T *t = w->t;
  long double acc;
  cb_inc(w, ML);
  acc = (long double)((int)x * (int)x) + (long double)t->echoBefore_;
  if (t->cnt_ >= (unsigned)w->p->rrnSilenceEchoCalcPeriod && t->cnt_ % 6 == 0) {
    CV(G_COM, "st11: period end");
    t->echoBefore_ = (float)(acc / (long double)(uint32_t)t->cnt_);
    m_pf(w, 'e', PFX "entering WaitForEchoCancellation state @ %d\r\n", (int)t->cnt_);
    m_energy(w, PFX "error energy before echo cancellation  = %c%d.%04d\r\n", t->echoBefore_);
    t->state_ = 0xc; t->cnt_ = 0;
    return x;
  }
  CV(G_COM, "st11: accumulate");
  t->echoBefore_ = (float)acc;
  return x;
}
static int16_t m_echo_wait(W *w, int16_t x) {
  T *t = w->t;
  unsigned thr;
  cb_inc(w, ML);
  thr = (unsigned)w->p->rrnSilenceScrLength - 3u * (unsigned)w->p->rrnSilenceEchoCalcPeriod;
  if (t->cnt_ < thr) { CV(G_COM, "st12: before the threshold"); return x; }
  if (t->cnt_ % 6 != 0) { CV(G_COM, "st12: not a multiple of 6"); return x; }
  CV(G_COM, "st12: -> 13");
  m_pf(w, 'e', PFX "entering CalcErrorEnergyAfterEchoCancellation state @ %d\r\n", (int)t->cnt_);
  t->state_ = 0xd; t->cnt_ = 0; t->echoAfter_ = 0.0f;
  return x;
}
static int16_t m_echo_after(W *w, int16_t x) {
  T *t = w->t;
  long double acc;
  cb_inc(w, ML);
  acc = (long double)((int)x * (int)x) + (long double)t->echoAfter_;
  if (t->cnt_ >= (unsigned)w->p->rrnSilenceEchoCalcPeriod && t->cnt_ % 6 == 0) {
    long double ratio, db;
    volatile float d;
    int ip;
    CV(G_COM, "st13: period end");
    t->echoAfter_ = (float)(acc / (long double)(uint32_t)t->cnt_);
    m_pf(w, 'e', PFX "entering WaitForRt state @ %d\r\n", (int)t->cnt_);
    m_energy(w, PFX "error energy after echo cancellation  = %c%d.%04d\r\n", t->echoAfter_);
    ratio = (1.0L / (long double)t->echoAfter_) * (long double)t->echoBefore_;
    db = m_log10(ratio) * (long double)10.0f;
    d = (float)db;
    ip = m_fist((long double)d);
    m_pf(w, 'e', PFX "silence SCR echo energy [dB]  = %c%d.%04d\r\n", m_sign((long double)d), m_fist(m_abs((long double)d)), m_iabs(m_fist(((long double)d - (long double)ip) * 10000.0L)));
    t->event_ = 0x2a; t->state_ = 0xe;
    t->keepRate_ = ((long double)w->p->rrnSilenceMinEchoEnergyForKeepRate >= (long double)d) ? 0u : 1u;
    t->cnt_ = 0;
    return x;
  }
  CV(G_COM, "st13: accumulate");
  t->echoAfter_ = (float)acc;
  return x;
}

static int16_t model(W *w, int v92, int16_t x, unsigned lvl) {
  T *t = w->t;
  unsigned st = t->state_;
  int16_t hd;
  t->cnt_++;
  t->event_ = 0;
  if (st > 0x11) { CV(G_ILL, "state above 0x11: nothing"); return 0; }
  switch (st) {
    case 0: return m_ri(w, x);
    case 1: return m_rinot(w, x);
    case 2: t->state_ = 3; /* fall through */
    case 3: return m_known(w, x, lvl);
    case 4:
      if (!v92) { CV(G_ILL, "V.90 state 4: nothing"); return 0; }
      {
        unsigned r, i;
        hd = m_hd(w, x);
        r = cb_proc(w, MC, ML, t->buf_, &t->nBytes_);
        if (r == 0) { CV(G_V92, "st4: process 0"); return hd; }
        if (t->nBytes_ == 0) { CV(G_V92, "st4: no bytes"); return hd; }
        i = 0;
        do {
          int b = cb_descr(w, MC, ML, t->buf_[i]);
          unsigned k = (unsigned)cb_cpbits(w, MC, ML, (unsigned)b & 0xffu);
          if (k == 1) { CV(G_V92, "st4: CP"); t->event_ = 0x2d; m_pf(w, 'e', PFX "CP detected @ %d\r\n", (int)t->cnt_); }
          else if (k == 2) { CV(G_V92, "st4: CPnot"); t->event_ = 0x2e; m_pf(w, 'e', PFX "CPnot detected @ %d\r\n", (int)t->cnt_); }
          else if (k == 3) {
            t->cpSeen_ = 1;
            if (t->w3c_ == 0) { CV(G_V92, "st4/3: no RRN pending"); t->event_ = 0x2f; }
            else {
              t->w4c_ = w->cp->wCa0_;
              if (t->w40_ == 0 && w->cp->wCa0_ == 0) { CV(G_V92, "st4/3: pending, nothing"); t->event_ = 0x2f; }
              else if (t->w48_ != 0) { CV(G_V92, "st4/3: answered"); t->event_ = 0x32; }
              else { CV(G_V92, "st4/3: first"); t->w44_ = 1; t->event_ = 0x31; }
            }
          } else if (k == 4) {
            if (t->w3c_ == 0) { CV(G_V92, "st4/4: no RRN pending"); t->event_ = 0x30; }
            else {
              t->cpSeen_ = 1;
              t->w4c_ = w->cp->wCa0_;
              if (t->w40_ == 0 && w->cp->wCa0_ == 0) { CV(G_V92, "st4/4: pending, nothing"); t->event_ = 0x30; }
              else if (t->w48_ != 0) { CV(G_V92, "st4/4: answered"); t->event_ = 0x34; }
              else { CV(G_V92, "st4/4: first"); t->w44_ = 1; t->event_ = 0x33; }
            }
          } else if (k == 5) {
            if (t->cpSeen_ == 0) CV(G_V92, "st4/5: no CP seen");
            else if (t->w3c_ == 0 || t->w44_ == 0 || t->w48_ != 0) {
              CV(G_V92, "st4/5: Ed -> B1d");
              t->event_ = 0x1c;
              m_pf(w, 'e', PFX "Ed detected @ %d, enter B1d state\r\n", (int)t->cnt_);
              t->state_ = 7; t->cnt_ = 0; cb_rns(w, ML, g_fake[F_MP2]); t->b1dZeros_ = 0; t->b1dBits_ = 0;
            } else {
              CV(G_V92, "st4/5: first Ed at RRN");
              t->event_ = 0x35;
              m_pf(w, 'e', PFX "First Ed at RRN detected @ %d, Silence state\r\n", (int)t->cnt_);
              t->cnt_ = 0; t->echoBefore_ = 0.0f;
              cb_rdreset(w, ML, 0, (unsigned)w->p->rrnRDetectionLength, 0x18);
              cb_rdreset(w, ML, 1, 0xb4, 0xc);
              t->w48_ = 1;
              { unsigned c4 = t->w4c_; t->cpSeen_ = 0; t->state_ = c4 == 0 ? 0xa : 0xe; if (c4 == 0) CV(G_V92, "st4/5: -> 10"); else CV(G_V92, "st4/5: -> 14"); }
            }
          } else CV(G_V92, "st4: other kind");
          i++;
        } while (t->nBytes_ > i);
        return hd;
      }
    case 5: case 6:
      if (v92) { CV(G_ILL, "V.92 state 5 / 6: nothing"); return 0; }
      {
        unsigned r, i;
        hd = m_hd(w, x);
        r = cb_proc(w, MC, ML, t->buf_, &t->nBytes_);
        if (r == 0) { CV(G_V90, "st5/6: process 0"); return hd; }
        if (t->nBytes_ == 0) { CV(G_V90, "st5/6: no bytes"); return hd; }
        i = 0;
        do {
          int b = cb_descr(w, MC, ML, t->buf_[i]);
          int k = cb_mpbits(w, MC, ML, b);
          if (st == 5) {
            if (k == 1) { CV(G_V90, "st5: MP"); t->event_ = 0x1a; m_pf(w, 'e', PFX "MP detected @ %d\r\n", (int)t->cnt_); }
            else if (k == 2) { CV(G_V90, "st5: MPnot"); t->event_ = 0x1b; m_pf(w, 'e', PFX "MPnot detected @ %d\r\n", (int)t->cnt_); }
            else CV(G_V90, "st5: other");
          } else {
            if (k == 2) {
              CV(G_V90, "st6: MPnot"); t->event_ = 0x1b;
              if (lvl > 2) { CV(G_V90, "st6: MPnot print"); m_pf(w, 'd', PFX "MPnot detected on WaitForEd @ %d\r\n", (int)t->cnt_); }
            } else if (k == 3) {
              if (t->state_ != 6) CV(G_V90, "st6: Ed, state already changed");
              else {
                t->event_ = 0x1c;
                cb_mpprint(w, ML);
                if (w->ce->rrnSilence_ != 0 && t->w3c_ != 0 && t->w38_ != 0) {
                  CV(G_V90, "st6: Ed -> wait for Rt");
                  m_pf(w, 'e', PFX "Ed detected @ %d, enter wait for Rt state\r\n", (int)t->cnt_);
                  t->cnt_ = 0; t->echoBefore_ = 0.0f; t->state_ = 0xa;
                  cb_rdreset(w, ML, 0, (unsigned)w->p->rrnRDetectionLength, 0x18);
                  cb_rdreset(w, ML, 1, 0xb4, 0xc);
                } else {
                  CV(G_V90, "st6: Ed -> B1d");
                  m_pf(w, 'e', PFX "Ed detected @ %d, enter B1d state\r\n", (int)t->cnt_);
                  t->state_ = 7; t->cnt_ = 0; cb_rns(w, ML, g_fake[F_MP2]); t->b1dZeros_ = 0; t->b1dBits_ = 0;
                }
              }
            } else CV(G_V90, "st6: other");
          }
          i++;
        } while (t->nBytes_ > i);
        return hd;
      }
    case 7: return m_b1d(w, x);
    case 8: case 17: CV(G_COM, "st8 / 17: return 0"); return 0;
    case 9: {
      hd = m_hd(w, x);
      cb_proc(w, MC, ML, t->buf_, &t->nBytes_);
      if (cb_det(w, MC, ML, 1, hd) == 0) { CV(G_COM, "st9: no RdNot"); return hd; }
      m_pf(w, 'e', PFX "RdNot detected @ %d, enter TRN2d DD state\r\n", (int)t->cnt_);
      cb_rls(w, ML, 0x1c20);
      t->cnt_ = 0; t->studyEnd_ = 0x7d0; t->state_ = 3; t->event_ = 0x2c; t->limit_ = (unsigned)w->p->rrnTrn2dDdLength;
      if (!v92) { CV(G_V90, "st9: RdNot (V.90)"); cb_mpreset(w, ML); w->mp->trn2dD_ = m_w0(F_MP1); }
      else { CV(G_V92, "st9: RdNot (V.92)"); cb_cpreset(w, ML); w->cp->f13_ = 0; w->cp->trn2dD_ = m_w0(F_MP1); t->cpSeen_ = 0; }
      cb_rns(w, ML, g_fake[F_MP1]);
      return hd;
    }
    case 10: return m_silence(w, x);
    case 11: return m_echo_before(w, x);
    case 12: return m_echo_wait(w, x);
    case 13: return m_echo_after(w, x);
    case 14:
      cb_inc(w, ML);
      if (cb_det(w, MC, ML, 0, x) == 0) { CV(G_COM, "st14: nothing"); return x; }
      m_pf(w, 'e', PFX "Rt detected @ %d, polarity = %d\r\n", (int)t->cnt_, (int)t->r_dir_);
      if (v92) { CV(G_V92, "st14: Rt (V.92)"); t->event_ = 0x27; } else CV(G_V90, "st14: Rt (V.90)");
      t->state_ = 0xf; t->cnt_ = 0; t->w38_ = 0;
      return x;
    case 15:
      cb_inc(w, ML);
      if (cb_det(w, MC, ML, 1, x) == 0) { CV(G_COM, "st15: nothing"); return x; }
      m_pf(w, 'e', PFX "RtNot detected @ %d\r\n", (int)t->cnt_);
      if (!v92) {
        CV(G_V90, "st15: RtNot (V.90)");
        m_pf(w, 'e', PFX "enter WaitForMP state @ %d\r\n", (int)t->cnt_);
        t->state_ = 5; t->cnt_ = 0; t->event_ = 0x28; cb_mpreset(w, ML); w->mp->trn2dD_ = m_w0(F_MP1);
      } else {
        CV(G_V92, "st15: RtNot (V.92)");
        t->state_ = 4; t->cnt_ = 0; t->event_ = 0x28; cb_cpreset(w, ML); w->cp->trn2dD_ = m_w0(F_MP1);
      }
      m_pf(w, 'e', PFX "No reset to demapper, current Phase - %d\r\n", (int)w->dem->phase_);
      return x;
    case 16:
      if (!v92) { CV(G_ILL, "V.90 state 16: nothing"); return 0; }
      hd = m_hd(w, x);
      cb_proc(w, MC, ML, t->buf_, &t->nBytes_);
      if (cb_det(w, MC, ML, 3, hd) == 0) { CV(G_V92, "st16: no RfNot"); return hd; }
      CV(G_V92, "st16: RfNot");
      m_pf(w, 'e', PFX "RfNot detected @ %d, enter WaitForCPu state\r\n", (int)t->cnt_);
      t->state_ = 4; t->cnt_ = 0; cb_cpreset(w, ML); w->cp->f13_ = 0; w->cp->trn2dD_ = m_w0(F_MP2); t->cpSeen_ = 0;
      return hd;
  }
  return 0;
}

/* ------------------------------------------------------------------------------------------------ trial setup */
static unsigned rnd_range(unsigned lo, unsigned hi) { return lo + h_rmod(hi - lo + 1); }
static unsigned pick_level(void) {
  static const unsigned lv[] = { 0, 1, 2, 3, 4, 0x80000000u, 0xffffffffu, 0x80000001u };
  return h_rpct(40) ? 2u + h_rmod(2) : lv[h_rmod(sizeof lv / sizeof lv[0])];
}
static uint32_t pick_res(void) {   /* a detector / process result: the whole eax is tested */
  static const uint32_t v[] = { 1, 0x100, 0x10000, 0x1000000, 0x80000000u, 0xffffffffu, 2, 0x7fffffffu };
  return h_rpct(45) ? 0u : h_rpct(50) ? 1u : h_rpct(50) ? v[h_rmod(sizeof v / sizeof v[0])] : (h_rnd() | 1u);
}
/* the scripted results: the bits the function must not look at are garbage */
static void gen_script(void) {
  unsigned i;
  static const int16_t syms[] = { 0, 1, -1, 0xffc, -0xffc, 100, -100, 32767, -32768, 4000, -4000, 0x7f, 0x1234 };
  for (i = 0; i < NSC; i++) {
    uint32_t lo = h_rpct(60) ? (uint16_t)syms[h_rmod(sizeof syms / sizeof syms[0])] : (h_rnd() & 0xffffu);
    g_sc.hd[i] = (h_rnd() & 0xffff0000u) | lo;
    g_sc.pr[i] = pick_res();
    g_sc.pn[i] = h_rpct(8) ? 0u : h_rpct(80) ? rnd_range(1, 6) : rnd_range(7, 40);
    if (g_sc.pr[i] == 0 && h_rpct(50)) g_sc.pn[i] = h_rnd();   /* garbage count: never read after a zero result (the buffer is not written then) */
    g_sc.dR[i] = pick_res(); g_sc.dRn[i] = pick_res(); g_sc.dRf[i] = pick_res(); g_sc.dRfn[i] = pick_res();
    g_sc.ddir[i] = h_rpct(50) ? (h_rpct(50) ? 1u : 0xffffffffu) : h_rnd();
    g_sc.mpb[i] = h_rpct(88) ? h_rmod(4) : h_rpct(40) ? ((h_rmod(4)) | ((h_rnd() | 0x10000u) & 0xffff0000u)) : h_rpct(50) ? h_rnd() : 4u + h_rmod(3);   /* kinds with garbage above bit 15: the whole eax is compared */
    g_sc.cpb[i] = h_rpct(90) ? h_rmod(6) : h_rpct(50) ? h_rnd() : 6u + h_rmod(3);
    g_sc.descr[i] = h_rpct(45) ? 0u : h_rpct(50) ? 1u : h_rpct(35) ? (h_rpct(50) ? ((h_rnd() | 0x10000u) & 0xffff0000u) : ((h_rnd() | 0x100u) & 0xffffff00u)) : h_rnd();   /* zero low 16 / 8 bits, nonzero above: V.90 tests the whole eax, V.92 the low byte */
  }
  for (i = 0; i < 256; i++) g_sc.pbytes[i] = (uint8_t)h_rnd();
}
static void fill_fields(void *obj, const HFld *f, int nf) {
  int i;
  for (i = 0; i < nf; i++) {
    unsigned n = f[i].count ? f[i].count : 1, es, tot, j;
    uint8_t *q = (uint8_t *)obj + f[i].off;
    switch (f[i].kind) {
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'u': case 'i': case 'f': es = 4; break;
      case 'd': es = 8; break;
      default: continue;
    }
    tot = n * es;
    if (tot > 64) tot = 64;
    for (j = 0; j < tot; j += 4) { uint32_t r = h_rnd(); memcpy(q + j, &r, tot - j < 4 ? tot - j : 4); }
  }
}
static float pick_energy(void) {
  static const uint32_t b[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7f800000u, 0xff800000u, 0x3f800000u, 0xbf800000u, 0x00000001u, 0x7f7fffffu, 0x4b000000u, 0x4cbebc20u, 0x358637bdu,
                                0x461c4000u, 0x3a83126fu, 0x7e800000u };
  uint32_t v;
  float f;
  if (h_rpct(30)) v = b[h_rmod(sizeof b / sizeof b[0])];
  else if (h_rpct(60)) { float g = (float)(h_rnd() % 4000000u) / (float)(1u + h_rmod(1000)); memcpy(&v, &g, 4); }
  else { v = h_rnd(); if (((v >> 23) & 0xffu) == 0xffu && (v & 0x7fffffu) != 0) v |= 0x400000u; }
  memcpy(&f, &v, 4);
  return f;
}
static uint32_t pick_w0(void) { return h_rpct(60) ? rnd_range(0, 40) : h_rpct(50) ? h_rnd() : (h_rpct(50) ? 0x55555555u : 0xffffffffu); }
static unsigned pick_small(void) { return h_rpct(70) ? rnd_range(0, 30) : h_rpct(50) ? h_rnd() : (h_rpct(50) ? 0x80000000u : 0xfffffff0u + h_rmod(16)); }
static unsigned pick_flag(void) { return h_rpct(45) ? 0u : h_rpct(70) ? 1u : h_rnd(); }

static void new_world(void) {
  int k;
  uint8_t *raw = (uint8_t *)h_obj_new(SZ + CAN);
  T *t;
  g_iw.t = t = (T *)raw; g_can = raw + SZ;
  g_iw.dem = (DemObj *)h_obj_new(sizeof(DemObj));
  g_iw.mp = (MpObj *)h_obj_new(sizeof(MpObj));
  g_iw.cp = (CpObj *)h_obj_new(sizeof(CpObj));
  g_iw.ce = (EvObj *)h_obj_new(sizeof(EvObj));
  g_iw.p = (V90ParObj *)h_obj_new(sizeof(V90ParObj));
  for (k = 0; k < 3; k++) { uint32_t *q = (uint32_t *)(g_fake[k] = sysdep_malloc(0x40)); unsigned i; for (i = 0; i < 16; i++) q[i] = h_rnd(); }
  { uint32_t v = pick_w0(); memcpy(g_fake[F_MP1], &v, 4); v = pick_w0(); memcpy(g_fake[F_MP2], &v, 4); }
  memset(raw, 0, SZ);
  memset(raw + SZ, 0x5a, CAN);
  fill_fields(t, f_Ph4DemObj, NF_Ph4DemObj);
  /* the pointer members */
  t->params_ = g_iw.p; t->mp1_ = g_fake[F_MP1]; t->mp2_ = g_fake[F_MP2]; t->cp_ = g_iw.cp; t->mp_ = g_iw.mp; t->p3_ = (void *)g_iw.ce; t->dem_ = g_iw.dem; t->descr_ = g_fake[F_DESCR]; t->ce_ = g_iw.ce;
  t->adid_ = (void *)g_iw.mp;
  /* the scalar members the functions read or write */
  t->session_ = h_rnd();
  t->state_ = 0;
  t->cnt_ = h_rpct(50) ? h_rmod(100) : h_rnd();
  t->event_ = h_rnd();
  t->limit_ = h_rpct(70) ? rnd_range(0, 40) : h_rnd();
  t->cpSeen_ = (uint8_t)(h_rpct(50) ? 1u : h_rpct(50) ? 0u : h_rnd());
  t->w34_ = pick_flag(); t->w38_ = pick_flag(); t->w3c_ = pick_flag(); t->w40_ = pick_flag(); t->w44_ = pick_flag(); t->w48_ = pick_flag(); t->w4c_ = pick_flag();
  t->nBytes_ = h_rnd();
  t->b1dZeros_ = h_rpct(60) ? h_rmod(30) : h_rnd();
  t->b1dBits_ = h_rpct(60) ? h_rmod(200) : h_rnd();
  t->echoBefore_ = pick_energy(); t->echoAfter_ = pick_energy();
  t->keepRate_ = h_rnd();
  t->studyEnd_ = h_rpct(70) ? rnd_range(0, 40) : h_rnd();
  t->r_dir_ = (int32_t)h_rnd(); t->rf_dir_ = (int32_t)h_rnd();
  /* the objects behind the pointers */
  g_iw.dem->phase_ = h_rpct(70) ? h_rmod(6) : h_rnd();
  g_iw.dem->z1eb4_ = (uint16_t)(h_rpct(40) ? 0u : h_rpct(60) ? 1u : (h_rnd() & 0xffffu));
  g_iw.mp->trn2dD_ = h_rnd();
  g_iw.cp->f13_ = (uint8_t)h_rnd(); g_iw.cp->trn2dD_ = h_rnd();
  g_iw.cp->wCa0_ = h_rpct(40) ? 0u : h_rpct(60) ? 1u : h_rnd();
  g_iw.ce->rrnSilence_ = (int32_t)(h_rpct(40) ? 0u : h_rpct(60) ? 1u : h_rnd());
  { V90ParObj *p = g_iw.p;
    p->rrnRDetectionLength = (int32_t)h_rnd();
    p->rrnTrn2dDdLength = (int32_t)(h_rpct(70) ? rnd_range(0, 40) : h_rnd());
    p->rrnSilenceScrLength = (int32_t)(h_rpct(60) ? rnd_range(0, 80) : h_rnd());
    p->rrnSilenceWaitBeforeEchoCalc = (int32_t)(h_rpct(70) ? rnd_range(0, 30) : h_rpct(50) ? h_rnd() : 0x80000000u + h_rmod(8));
    p->rrnSilenceEchoCalcPeriod = (int32_t)(h_rpct(70) ? rnd_range(0, 30) : h_rpct(50) ? h_rnd() : 0x80000000u + h_rmod(8));
    { float e = pick_energy(); memcpy(&p->rrnSilenceMinEchoEnergyForKeepRate, &e, 4); }
  }
}

/* the input symbol */
static int16_t pick_x(void) {
  static const int16_t b[] = { 0, 1, -1, 2, -2, 100, -100, 32767, -32768, -32767, 181, -181, 1000, -1000, 4095, -4096, 255, 256, -255, -256 };
  return h_rpct(40) ? b[h_rmod(sizeof b / sizeof b[0])] : h_rpct(60) ? (int16_t)((int)h_rmod(401) - 200) : (int16_t)h_rnd();
}

/* the legal state numbers of a function: the state_ the harness sets */
static unsigned pick_state(int sc, int v92) {
  static const unsigned ill[] = { 0x12, 0x13, 0x20, 0x40, 0xff, 0x10000, 0x80000000u, 0xffffffffu, 0x100000a };
  if (sc != 2 && h_rpct(8)) return ill[h_rmod(sizeof ill / sizeof ill[0])];
  if (sc == 2 && h_rpct(10)) return h_rpct(50) ? ill[h_rmod(sizeof ill / sizeof ill[0])] : (v92 ? 5u + h_rmod(2) : (h_rpct(50) ? 4u : 16u));
  if (h_rpct(30)) return 10u + h_rmod(4);   /* the echo-energy states are the x87 ones */
  return h_rmod(18);
}

/* aim the counter at one of the state's trigger values (the table is the harness' own steering, not the model) */
static unsigned ceil6(unsigned v) { return (v / 6u + (v % 6u ? 1u : 0u)) * 6u; }
static void aim_cnt(T *t, const V90ParObj *p) {
  unsigned st = t->state_, tgt = 0;
  int have = 1;
  switch (st) {
    case 2: case 3: { unsigned c[3]; c[0] = t->studyEnd_; c[1] = t->limit_ >> 1; c[2] = t->limit_; tgt = c[h_rmod(3)]; break; }
    case 7: tgt = 0x120; break;
    case 10: tgt = ceil6((unsigned)p->rrnSilenceWaitBeforeEchoCalc) + 6u * h_rmod(2); break;
    case 11: case 13: tgt = ceil6((unsigned)p->rrnSilenceEchoCalcPeriod) + 6u * h_rmod(2); break;
    case 12: tgt = ceil6((unsigned)p->rrnSilenceScrLength - 3u * (unsigned)p->rrnSilenceEchoCalcPeriod) + 6u * h_rmod(2); break;
    default: have = 0; break;
  }
  if (!have || !h_rpct(75)) { t->cnt_ = h_rpct(50) ? h_rmod(80) : h_rnd(); return; }
  t->cnt_ = tgt - 1u - (h_rpct(12) ? (h_rmod(3) + 0u) : 0u) + (h_rpct(8) ? 1u : 0u);
}
/* the other members and scripts the state's decision logic reads */
static void aim_members(T *t, V90ParObj *p, int v92) {
  unsigned st = t->state_, k;
  (void)p;
  if (st == 7) {
    uint32_t w0 = m_w0(F_MP2);
    if (h_rpct(60)) for (k = 0; k < NSC; k++) g_sc.descr[k] = h_rpct(70) ? 0u : h_rnd();
    if (h_rpct(70)) t->b1dBits_ = 3u * w0 + 0x17u - rnd_range(0, 3) + (h_rpct(30) ? 0u : h_rmod(4));
    if (h_rpct(30)) t->b1dBits_ = 3u * w0 + 0x17u + h_rmod(2);   /* the denominator 0 / 1 */
    if (h_rpct(40)) t->b1dZeros_ = h_rpct(30) ? 0u : rnd_range(0, 5);
  }
  if (st == 6 && !v92) {
    if (h_rpct(60)) for (k = 0; k < NSC; k++) if (h_rpct(35)) g_sc.mpb[k] = 3;
    if (h_rpct(30)) for (k = 0; k < NSC; k++) if (h_rpct(35)) g_sc.mpb[k] = 2;
  }
  if (st == 5 && !v92 && h_rpct(60)) for (k = 0; k < NSC; k++) if (h_rpct(35)) g_sc.mpb[k] = 1 + h_rmod(2);
  if (st == 4 && v92 && h_rpct(60)) for (k = 0; k < NSC; k++) if (h_rpct(40)) g_sc.cpb[k] = 1 + h_rmod(5);
  if (st == 11 || st == 13) {   /* the accumulators: keep them finite most of the time so the dB path runs */
    if (h_rpct(50)) { float e = (float)(h_rnd() % 100000u) + (h_rpct(30) ? 0.0f : 0.5f); if (st == 11) t->echoBefore_ = e; else t->echoAfter_ = e; if (st == 13 && h_rpct(60)) t->echoBefore_ = (float)(1 + h_rmod(1000000)); }
    if (st == 13 && h_rpct(30)) t->echoAfter_ = 0.0f;
  }
}

/* one call: model on its copy first, then the impl, then compare everything */
static unsigned long g_ncalls, g_nskipped;
static int g_showall;
static void copy_world_to_model(void) {
  memcpy(&g_mt, g_iw.t, SZ);
  memcpy(&g_mdem, g_iw.dem, sizeof g_mdem);
  memcpy(&g_mmp, g_iw.mp, sizeof g_mmp);
  memcpy(&g_mcp, g_iw.cp, sizeof g_mcp);
  memcpy(&g_mce, g_iw.ce, sizeof g_mce);
  memcpy(&g_mpar, g_iw.p, sizeof g_mpar);
  g_mw.t = &g_mt; g_mw.dem = &g_mdem; g_mw.mp = &g_mmp; g_mw.cp = &g_mcp; g_mw.ce = &g_mce; g_mw.p = &g_mpar;
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
  cmp_blob(what, "demapper", &g_mdem, g_iw.dem, sizeof g_mdem);
  cmp_blob(what, "MP", &g_mmp, g_iw.mp, sizeof g_mmp);
  cmp_blob(what, "CP", &g_mcp, g_iw.cp, sizeof g_mcp);
  cmp_blob(what, "evaluator", &g_mce, g_iw.ce, sizeof g_mce);
  cmp_blob(what, "parameter block", &g_mpar, g_iw.p, sizeof g_mpar);
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
  h_emit(g_show, "%s st=%u cnt=%u ev=%u lim=%u cs=%u w=%u/%u/%u/%u/%u eb=%08x ea=%08x kr=%u se=%u b1=%u/%u nb=%u buf=%08x z=%u mpd=%u cf=%u cpd=%u dirs=%d/%d", tag, (unsigned)t->state_, (unsigned)t->cnt_, (unsigned)t->event_,
         (unsigned)t->limit_, (unsigned)t->cpSeen_, (unsigned)t->w38_, (unsigned)t->w3c_, (unsigned)t->w44_, (unsigned)t->w48_, (unsigned)t->w4c_, fbits(t->echoBefore_), fbits(t->echoAfter_),
         (unsigned)t->keepRate_, (unsigned)t->studyEnd_, (unsigned)t->b1dZeros_, (unsigned)t->b1dBits_, (unsigned)t->nBytes_, fnv(t->buf_, sizeof t->buf_), (unsigned)g_iw.dem->z1eb4_,
         (unsigned)g_iw.mp->trn2dD_, (unsigned)g_iw.cp->f13_, (unsigned)g_iw.cp->trn2dD_, (int)t->r_dir_, (int)t->rf_dir_);
}
static void do_call(int v92, int16_t x) {
  short mr, ir;
  unsigned lvl = dsplibs_debug_level, st = g_iw.t->state_;
  int ignore = st > 0x11 || (!v92 && (st == 4 || st == 16)) || (v92 && (st == 5 || st == 6));
  copy_world_to_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  mr = model(&g_mw, v92, x, lvl);
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  h_emit(g_show, "call v%d st=%u cnt=%u x=%d lvl=%u", v92 ? 92 : 90, st, (unsigned)g_iw.t->cnt_, (int)x, lvl);
  ir = v92 ? f_get92(g_iw.t, x) : f_get90(g_iw.t, x);
  __asm__ volatile("" ::: "memory");
  { unsigned i; for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t); }
  if (!ignore && mr != ir) h_die3("%s: result: model %d, impl %d", g_what, (int)mr, (int)ir);
  if (!ignore) h_emit(g_show, "ret %d", (int)ir);
  else h_emit(g_show, "ret (not compared: the blob returns the caller's edi)");
  log_cmp();
  cmp_world(g_what);
  { unsigned i; for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", g_what, i); }
  emit_state("end");
  cov_flush();
  g_ncalls++;
}

static void free_world(void) {
  int k;
  for (k = 0; k < 3; k++) sysdep_free(g_fake[k]);
}
static void trial(int sc) {
  unsigned ncalls = (sc == 2 && h_rpct(60)) ? rnd_range(2, 6) : 1, k;
  new_world();
  gen_script();
  dsplibs_debug_level = pick_level();
  for (k = 0; k < ncalls; k++) {
    T *t = g_iw.t;
    int v92 = sc == 0 ? 0 : sc == 1 ? 1 : (int)h_rmod(2);
    if (k == 0 || h_rpct(40)) {
      t->state_ = pick_state(sc, v92);
      aim_cnt(t, g_iw.p);
      aim_members(t, g_iw.p, v92);
    } else if (h_rpct(50)) aim_cnt(t, g_iw.p);
    if (h_rpct(15)) gen_script();
    if (h_rpct(10)) dsplibs_debug_level = pick_level();
    g_what = v92 ? "getV92Decision" : "getV90Decision";
    do_call(v92, pick_x());
  }
  free_world();
  h_trial_end("p37");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("P37_SEED");
  CovSite *c;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P37_ALL")) g_showall = 1;
  p07_layout_check(); p09_layout_check(); p10_layout_check(); p17_layout_check(); p28_layout_check(); p37_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P37_TRIALS") ? atoi(getenv("P37_TRIALS")) : (sc == 2 ? 30000 : 40000);
  if (sc < 0 || sc > 2) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (c = __start_covsites; c < __stop_covsites; c++) {
    int req = sc == 0 ? (c->grp == G_COM || c->grp == G_V90) : sc == 1 ? (c->grp == G_COM || c->grp == G_V92) : (c->grp == G_ILL);
    if (!req) continue;
    h_emit(1, "COV %s %lu", c->name, c->n);
    if (c->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", c->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P37_NOCOV")) ? 2 : 0;
}
