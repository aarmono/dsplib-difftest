/*
 * Differential test harness for port chunk P36 (src/vpcm/v90_phase3_demod.cpp): the other 17 methods of V90Phase3Demodulator (19 symbols with the C2 / D2 twins; getV92Decision is P34, getV90Decision P35):
 * constructors C1 / C2 (the V90Phase3Modulator / SerialDifferentialDecoder<int> / Descrambler<int,int> members and the heap V90SdDetector (0x1c B) and ANSamToneDetector (0x3c B)), destructors D1 / D2, reset,
 * clearVerificationStatus, setSessionFlag, exitDIL, setDigitalImairmentsInfo, enterWaitForANSpcmDrop, getMaxUcode, getDecision and the dead resetJdNotDetector / incrementFramePosition / setAltRbsParams /
 * JdNotDetector / twoLevelDemod.
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_phase3_demod.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2: the
 * port object alone, the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_phase3_demod.blob32.out).  The methods are reached through __asm__ labels; the object is the
 * generated C mirror (test/vpcm/v90_phase3_demod_v92_decision.layout), the V90Parameters block / the AutoDigitalImpDetector are the mirrors of P07 / P18 (member-wise access: their layouts differ on x86-64).  By-value FLOAT arguments are
 * passed as raw words on i386 (typedef FW: the x87 would quiet a signalling NaN in the harness' own call; the stack slot is identical) and as floats on x86-64 (SSE keeps the bits).
 *
 * Every callee is a RECORDER with scripted results: V90Phase3Modulator C1 / D1 / reset / setSessionFlag / exitDIL (writes its event_), Descrambler<int,int> C1 / D1 / reset / process,
 * SerialDifferentialDecoder<int>::process, V90SdDetector C1 / D1 / reset, ANSamToneDetector C1 / D1, V90Jd::unPackReset, V92Jd::unPackJdReset / unPackJdPhaseReset, six V90AutoDigitalImpDetector methods
 * (reset, resetLinearMapping, isAltRbs, determineMaxUcode, findPadGain, applyPadGainToLinMapp), calculateDilLength, alaw2linear / ulaw2linear, getV92Decision / getV90Decision, edprintf,
 * dsplibs_debug_printf; sysdep_malloc / sysdep_free are the harness allocator (malloc log, red zones, poisoned frees).  The results carry garbage in the bits the function must not look at.  What is compared:
 * every returned value, the whole object / the detector / the parameter block after every call, every record (callee, arguments, results, and a SNAPSHOT of the demodulator object at the moment of the call: the
 * store order), the canary behind the object, the allocation / free counts.  The INDEPENDENT MODEL (m_*: written from porting/decompiled/dp/p36_phase3_demod_rest.txt, not from the port source) predicts the result, the
 * new world and every record; a model violation exits 3.
 *
 * Scenarios (each its own process): 0 the small methods (events, dead methods, getDecision, twoLevelDemod), 1 reset (every initial state, laws, NULL Jd objects), 2 constructor C1 / C2 + destructor D1 / D2.
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

#if defined(__i386__)
typedef uint32_t FW;
static inline uint32_t fw_bits(FW v) { return v; }
static inline FW fw_make(uint32_t b) { return b; }
#else
typedef float FW;
static inline uint32_t fw_bits(FW v) { uint32_t b; memcpy(&b, &v, 4); return b; }
static inline FW fw_make(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }
#endif

#define DM(n) "_ZN20V90Phase3Demodulator" n
/* the methods under test */
extern void f_c1(void *, void *, void *, unsigned, void *) L(DM("C1EP13V90ParametersP19V90SpectralVerifierjP25V90AutoDigitalImpDetector"));
extern void f_c2(void *, void *, void *, unsigned, void *) L(DM("C2EP13V90ParametersP19V90SpectralVerifierjP25V90AutoDigitalImpDetector"));
extern void f_d1(void *) L(DM("D1Ev"));
extern void f_d2(void *) L(DM("D2Ev"));
extern void f_reset(void *, unsigned, unsigned char, unsigned, unsigned, void *, void *, void *, short, short, FW, unsigned) L(DM("5resetE7PcmTypeh22Phase3DemodulatorStatejP5V90JdP5V92JdP19tagV90DILdescriptorssfj"));
extern void f_resetjd(void *) L(DM("18resetJdNotDetectorEv"));
extern void f_clearv(void *) L(DM("23clearVerificationStatusEv"));
extern void f_setsess(void *, unsigned) L(DM("14setSessionFlagEj"));
extern void f_exitdil(void *) L(DM("7exitDILEv"));
extern void f_setdig(void *) L(DM("24setDigitalImairmentsInfoEv"));
extern void f_incfp(void *) L(DM("22incrementFramePositionEv"));
extern void f_setalt(void *) L(DM("15setAltRbsParamsEv"));
extern unsigned char *f_getmax(void *) L(DM("11getMaxUcodeEv"));
extern int f_jdnot(void *, int) L(DM("13JdNotDetectorEi"));
extern void f_enterans(void *) L(DM("22enterWaitForANSpcmDropEv"));
extern int f_twolevel(void *, float, int *) L(DM("13twoLevelDemodEfRi"));
extern short f_getdec(void *, FW) L(DM("11getDecisionEf"));

/* the interposed callees */
extern void c_modctor(void *, void *, unsigned) L("_ZN18V90Phase3ModulatorC1EP13V90Parametersj");
extern void c_moddtor(void *) L("_ZN18V90Phase3ModulatorD1Ev");
extern void c_modreset(void *, unsigned, unsigned char, unsigned, unsigned, void *, void *, const void *, unsigned)
    L("_ZN18V90Phase3Modulator5resetE7PcmTypeh20Phase3ModulatorStatejP5V90JdP5V92JdPK19tagV90DILdescriptorj");
extern void c_modsess(void *, unsigned) L("_ZN18V90Phase3Modulator14setSessionFlagEj");
extern void c_modexit(void *) L("_ZN18V90Phase3Modulator7exitDILEv");
extern void c_descrctor(void *, unsigned, unsigned, unsigned) L("_ZN11DescramblerIiiEC1Ejjj");
extern void c_descrdtor(void *) L("_ZN11DescramblerIiiED1Ev");
extern void c_descrreset(void *, int) L("_ZN11DescramblerIiiE5resetEi");
extern int c_descr(void *, int) L("_ZN11DescramblerIiiE7processEi");
extern int c_sdd(void *, int) L("_ZN25SerialDifferentialDecoderIiE7processEi");
extern void c_sdctor(void *, FW, FW, FW, unsigned) L("_ZN13V90SdDetectorC1Efffj");
extern void c_sddtor(void *) L("_ZN13V90SdDetectorD1Ev");
extern void c_sdreset(void *) L("_ZN13V90SdDetector5resetEv");
extern void c_ansctor(void *, unsigned, unsigned, FW, unsigned, FW, unsigned, unsigned, unsigned) L("_ZN17ANSamToneDetectorC1Ejjfjfjjj");
extern void c_ansdtor(void *) L("_ZN17ANSamToneDetectorD1Ev");
extern void c_jdreset(void *) L("_ZN5V90Jd11unPackResetEv");
extern void c_j92reset(void *) L("_ZN5V92Jd13unPackJdResetEv");
extern void c_j92preset(void *) L("_ZN5V92Jd18unPackJdPhaseResetEv");
extern void c_adreset(void *, unsigned char, unsigned, short) L("_ZN25V90AutoDigitalImpDetector5resetEh7PcmTypes");
extern void c_adlin(void *) L("_ZN25V90AutoDigitalImpDetector18resetLinearMappingEv");
extern int c_isalt(void *, short, short, float) L("_ZN25V90AutoDigitalImpDetector8isAltRbsEssf");
extern void c_detmax(void *, short) L("_ZN25V90AutoDigitalImpDetector17determineMaxUcodeEs");
extern void c_findpad(void *) L("_ZN25V90AutoDigitalImpDetector11findPadGainEv");
extern void c_applypad(void *) L("_ZN25V90AutoDigitalImpDetector21applyPadGainToLinMappEv");
extern int c_dill(void *, unsigned) L("_Z18calculateDilLengthP19tagV90DILdescriptor7PcmType");
extern int c_lin_a(int) L("alaw2linear");
extern int c_lin_u(int) L("ulaw2linear");
extern short c_dec92(void *, FW) L("_ZN20V90Phase3Demodulator14getV92DecisionEf");
extern short c_dec90(void *, FW) L("_ZN20V90Phase3Demodulator14getV90DecisionEf");

typedef Ph3DemObj T;
#define SZ sizeof(T)
enum { CAN = 64, NSC = 16, LOGMAX = 40, TXTMAX = 640, FAKEN = 6 };

static uint32_t fnv(const void *p, size_t n) { const uint8_t *q = (const uint8_t *)p; uint32_t h = 2166136261u; while (n--) { h ^= *q++; h *= 16777619u; } return h; }

/* ------------------------------------------------------------------------------------------------ the world */
typedef struct { T *t; AdidObj *a; V90ParObj *p; } W;
static T g_mt;               /* the model's copy of the object */
static AdidObj g_ma;         /* ... of the detector */
static V90ParObj g_mp;       /* ... of the parameter block */
static W g_iw, g_mw;         /* impl world / model world */
static uint8_t *g_can;
static void *g_fake[FAKEN];  /* heap blocks the object may point to: sd, ans, jd, jd92, dil, spare */
static int g_fakefreed[FAKEN];
#define F_SD 0
#define F_ANS 1
#define F_JD 2
#define F_JD92 3
#define F_DIL 4
#define F_SPARE 5
static HFld g_ft[128];
static int g_nft;
static unsigned long g_free_base;

/* block id of a heap pointer; the pool objects get fixed ids */
static int pid(const void *p) {
  HAl *a;
  if (!p) return -1;
  if (p == (const void *)g_iw.a || p == (const void *)g_mw.a) return 100;
  if (p == (const void *)g_iw.p || p == (const void *)g_mw.p) return 101;
  a = h_find_al(p, 0);
  if (a) return (int)a->id;
  return 9;
}

/* ------------------------------------------------------------------------------------------------ scripts, cursors, records */
typedef struct {
  uint32_t lina[NSC], linu[NSC], dil[NSC], dec92[NSC], dec90[NSC], mev[NSC];
  int32_t sdd[NSC], descr[NSC], alt[NSC];
} Script;
typedef struct { unsigned lina, linu, dil, sdd, descr, alt, dec92, dec90, mev; } Cur;
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
/* the object fields the methods write between their stores (the snapshot at every record) */
static void snapshot(char *out, size_t cap, const W *w) {
  const T *t = w->t;
  snprintf(out, cap, "|st=%u cnt=%u ev=%u ph=%u ses=%u pcm=%u tb=%u code=%u lv=%u stat=%u jn=%u dl=%u lm=%u v=%u b424=%u f3f9=%u c3fc=%u f400=%u a9=%u a10=%08x sdd=%d mf=%u me=%u ptr=%d,%d,%d,%d,%d,%d,%d",
           (unsigned)t->state_, (unsigned)t->cnt_, (unsigned)t->event_, (unsigned)t->phase_, (unsigned)t->session_, (unsigned)t->pcm_, (unsigned)t->timeBase_, (unsigned)t->code_, (unsigned)t->level_,
           (unsigned)t->status_, (unsigned)t->jdNotCnt_, (unsigned)t->dilLen_, (unsigned)t->lineMode_, (unsigned)t->verify_, (unsigned)t->b424_, (unsigned)t->flag3f9_, (unsigned)t->cnt3fc_,
           (unsigned)t->flag400_, (unsigned)t->arg9_, pword(&t->arg10_), (int)t->sdd_state_, (unsigned)t->mod_flag_, (unsigned)t->mod_event_, pid(t->adid_), pid(t->params_), pid(t->dil_), pid(t->jd_),
           pid(t->jd92_), pid(t->sd_), pid(t->ans_));
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
enum { G_SMALL = 0, G_RESET = 1, G_LIFE = 2 };

/* ------------------------------------------------------------------------------------------------ the environment (callee semantics: shared by the impl's recorders and the model) */
#define CHECK(p, what) do { if ((void *)(p) != (void *)(what)) h_die3("callee %s got an unexpected object %p (expected %p)", __func__, (void *)(p), (void *)(what)); } while (0)

static void cb_modctor(W *w, Log *l, const void *p, unsigned flag) {
  REC(w, l, 'c', "mod.ctor p=%d flag=%u", pid(p), flag);
  w->t->mod_flag_ = flag;   /* the real constructor stores the flag (and params_, never read): visible in the later snapshots */
}
static void cb_moddtor(W *w, Log *l) { REC(w, l, 'c', "mod.dtor freed=%lu", h_nfree - g_free_base); }
static void cb_modreset(W *w, Log *l, unsigned pcm, unsigned code, unsigned st, unsigned n, const void *jd, const void *jd92, const void *dil, unsigned tb) {
  REC(w, l, 'c', "mod.reset pcm=%u code=%u st=%u n=%u jd=%d jd92=%d dil=%d tb=%u", pcm, code, st, n, pid(jd), pid(jd92), pid(dil), tb);
}
static void cb_modsess(W *w, Log *l, unsigned f) { REC(w, l, 'c', "mod.setSessionFlag %u", f); w->t->mod_flag_ = f; }
static void cb_modexit(W *w, Cur *c, Log *l) {
  unsigned k = c->mev++ % NSC;
  REC(w, l, 'c', "mod.exitDIL");
  w->t->mod_event_ = g_sc.mev[k];   /* the real one may end the pattern: event_ = 6 */
}
static void cb_descrctor(W *w, Log *l, unsigned a, unsigned b, unsigned c) { REC(w, l, 'c', "descr.ctor %u %u %u", a, b, c); }
static void cb_descrdtor(W *w, Log *l) { REC(w, l, 'c', "descr.dtor freed=%lu", h_nfree - g_free_base); }
static void cb_descrreset(W *w, Log *l, int v) { REC(w, l, 'c', "descr.reset(%d)", v); }
static int cb_descr(W *w, Cur *c, Log *l, int v) { int r = g_sc.descr[c->descr++ % NSC]; REC(w, l, 'c', "descr(%d) -> %d", v, r); return r; }
static int cb_sdd(W *w, Cur *c, Log *l, int v) { int r = g_sc.sdd[c->sdd++ % NSC]; REC(w, l, 'c', "sdd(%d) -> %d", v, r); return r; }
static void cb_sdctor(W *w, Log *l, const void *th, uint32_t a, uint32_t b, uint32_t c, unsigned d) { REC(w, l, 'c', "sd.ctor %08x %08x %08x %u blk=%d", a, b, c, d, pid(th)); }
static void cb_sddtor(W *w, Log *l, const void *th) { REC(w, l, 'c', "sd.dtor blk=%d freed=%lu", pid(th), h_nfree - g_free_base); }
static void cb_sdreset(W *w, Log *l, const void *th) { REC(w, l, 'c', "sd.reset blk=%d", pid(th)); }
static void cb_ansctor(W *w, Log *l, const void *th, unsigned a, unsigned b, uint32_t c, unsigned d, uint32_t e, unsigned f, unsigned g, unsigned h) {
  REC(w, l, 'c', "ans.ctor %u %u %08x %u %08x %u %u %u blk=%d", a, b, c, d, e, f, g, h, pid(th));
}
static void cb_ansdtor(W *w, Log *l, const void *th) { REC(w, l, 'c', "ans.dtor blk=%d freed=%lu", pid(th), h_nfree - g_free_base); }
static void cb_jdreset(W *w, Log *l, const void *th) { REC(w, l, 'c', "jd.reset blk=%d", pid(th)); }
static void cb_j92reset(W *w, Log *l, const void *th) { REC(w, l, 'c', "jd92.reset blk=%d", pid(th)); }
static void cb_j92preset(W *w, Log *l, const void *th) { REC(w, l, 'c', "jd92.phaseReset blk=%d", pid(th)); }
static void cb_adreset(W *w, Log *l, unsigned code, unsigned pcm, short a3) { REC(w, l, 'c', "adid.reset(%u,%u,%d)", code, pcm, (int)a3); }
static void cb_adlin(W *w, Log *l) { REC(w, l, 'c', "adid.resetLinearMapping"); }
static int cb_isalt(W *w, Cur *c, Log *l, short ph, short uc, uint32_t xb) { int r = g_sc.alt[c->alt++ % NSC]; REC(w, l, 'c', "isAltRbs(%d,%d,%08x) -> %d", (int)ph, (int)uc, xb, r); return r; }
static void cb_detmax(W *w, Log *l, short v) { REC(w, l, 'c', "determineMaxUcode(%d)", (int)v); }
static void cb_findpad(W *w, Log *l) { REC(w, l, 'c', "findPadGain"); }
static void cb_applypad(W *w, Log *l) { REC(w, l, 'c', "applyPadGainToLinMapp"); }
static int cb_dill(W *w, Cur *c, Log *l, const void *dil, unsigned pcm) { unsigned r = g_sc.dil[c->dil++ % NSC]; REC(w, l, 'c', "calculateDilLength(dil=%d, %u) -> %u", pid(dil), pcm, r); return (int)r; }
static int cb_lin(W *w, Cur *c, Log *l, int alaw, int v) {
  unsigned r = alaw ? g_sc.lina[c->lina++ % NSC] : g_sc.linu[c->linu++ % NSC];
  REC(w, l, 'c', alaw ? "alaw2linear(%d) -> %u" : "ulaw2linear(%d) -> %u", v, r);
  return (int)r;
}
static short cb_dec(W *w, Cur *c, Log *l, int v92, uint32_t xb) {
  unsigned r = v92 ? g_sc.dec92[c->dec92++ % NSC] : g_sc.dec90[c->dec90++ % NSC];
  REC(w, l, 'c', v92 ? "getV92Decision(%08x) -> %u" : "getV90Decision(%08x) -> %u", xb, r);
  return (short)r;
}
static void cb_ed(W *w, Log *l, char kind, const char *s) { REC(w, l, kind, "%s", s); }

/* the impl's recorders */
#define IW (&g_iw)
void c_modctor(void *t, void *p, unsigned flag) { CHECK(t, &g_iw.t->mod_flag_); cb_modctor(IW, &g_il, p, flag); }
void c_moddtor(void *t) { CHECK(t, &g_iw.t->mod_flag_); cb_moddtor(IW, &g_il); }
void c_modreset(void *t, unsigned pcm, unsigned char code, unsigned st, unsigned n, void *jd, void *jd92, const void *dil, unsigned tb) {
  CHECK(t, &g_iw.t->mod_flag_);
  cb_modreset(IW, &g_il, pcm, code, st, n, jd, jd92, dil, tb);
}
void c_modsess(void *t, unsigned f) { CHECK(t, &g_iw.t->mod_flag_); cb_modsess(IW, &g_il, f); }
void c_modexit(void *t) { CHECK(t, &g_iw.t->mod_flag_); cb_modexit(IW, &g_ic, &g_il); }
void c_descrctor(void *t, unsigned a, unsigned b, unsigned c) {
  CHECK(t, &g_iw.t->descr_buf_);
  cb_descrctor(IW, &g_il, a, b, c);
  h_log_next_size = 0x1c;   /* the next sysdep_malloc is the V90SdDetector (0x1c B on i386; the port allocates sizeof) */
}
void c_descrdtor(void *t) { CHECK(t, &g_iw.t->descr_buf_); cb_descrdtor(IW, &g_il); }
void c_descrreset(void *t, int v) { CHECK(t, &g_iw.t->descr_buf_); cb_descrreset(IW, &g_il, v); }
int c_descr(void *t, int v) { CHECK(t, &g_iw.t->descr_buf_); return cb_descr(IW, &g_ic, &g_il, v); }
int c_sdd(void *t, int v) { CHECK(t, &g_iw.t->sdd_state_); return cb_sdd(IW, &g_ic, &g_il, v); }
static void check_block(const void *t, unsigned minsz, const char *who) {
  HAl *a = h_find_al(t, 0);
  if (!a || a->p != (const uint8_t *)t || a->freed || a->size < minsz) h_die3("%s: the object is not a live heap block of at least %u bytes", who, minsz);
}
void c_sdctor(void *t, FW a, FW b, FW c, unsigned d) {
  check_block(t, 0x1c, "sd.ctor");
  cb_sdctor(IW, &g_il, t, fw_bits(a), fw_bits(b), fw_bits(c), d);
  h_log_next_size = 0x3c;   /* the next sysdep_malloc is the ANSamToneDetector */
}
void c_sddtor(void *t) { CHECK(t, g_iw.t->sd_); cb_sddtor(IW, &g_il, t); }
void c_sdreset(void *t) { CHECK(t, g_iw.t->sd_); cb_sdreset(IW, &g_il, t); }
void c_ansctor(void *t, unsigned a, unsigned b, FW c, unsigned d, FW e, unsigned f, unsigned g, unsigned h) {
  check_block(t, 0x3c, "ans.ctor");
  cb_ansctor(IW, &g_il, t, a, b, fw_bits(c), d, fw_bits(e), f, g, h);
}
void c_ansdtor(void *t) { CHECK(t, g_iw.t->ans_); cb_ansdtor(IW, &g_il, t); }
void c_jdreset(void *t) { CHECK(t, g_iw.t->jd_); cb_jdreset(IW, &g_il, t); }
void c_j92reset(void *t) { CHECK(t, g_iw.t->jd92_); cb_j92reset(IW, &g_il, t); }
void c_j92preset(void *t) { CHECK(t, g_iw.t->jd92_); cb_j92preset(IW, &g_il, t); }
void c_adreset(void *t, unsigned char code, unsigned pcm, short a3) { CHECK(t, g_iw.a); cb_adreset(IW, &g_il, code, pcm, a3); }
void c_adlin(void *t) { CHECK(t, g_iw.a); cb_adlin(IW, &g_il); }
int c_isalt(void *t, short ph, short uc, float x) { CHECK(t, g_iw.a); return cb_isalt(IW, &g_ic, &g_il, ph, uc, fbits(x)); }
void c_detmax(void *t, short v) { CHECK(t, g_iw.a); cb_detmax(IW, &g_il, v); }
void c_findpad(void *t) { CHECK(t, g_iw.a); cb_findpad(IW, &g_il); }
void c_applypad(void *t) { CHECK(t, g_iw.a); cb_applypad(IW, &g_il); }
int c_dill(void *d, unsigned pcm) { return cb_dill(IW, &g_ic, &g_il, d, pcm); }
int c_lin_a(int v) { return cb_lin(IW, &g_ic, &g_il, 1, v); }
int c_lin_u(int v) { return cb_lin(IW, &g_ic, &g_il, 0, v); }
short c_dec92(void *t, FW x) { CHECK(t, g_iw.t); return cb_dec(IW, &g_ic, &g_il, 1, fw_bits(x)); }
short c_dec90(void *t, FW x) { CHECK(t, g_iw.t); return cb_dec(IW, &g_ic, &g_il, 0, fw_bits(x)); }
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
#define PFX "V90Phase3Demodulator: "
static void m_ed(W *w, Log *l, const char *fmt, unsigned a) { char b[TXTMAX]; snprintf(b, sizeof b, fmt, (int)a); cb_ed(w, l, 'e', b); }
static void m_ed0(W *w, Log *l, const char *s) { cb_ed(w, l, 'e', s); }
static void m_dbg(W *w, Log *l, unsigned lvl, const char *fmt, int havearg, unsigned a) {
  if (lvl > 1) { char b[TXTMAX]; if (havearg) snprintf(b, sizeof b, fmt, (int)a); else snprintf(b, sizeof b, "%s", fmt); cb_ed(w, l, 'd', b); }
}
static int16_t m_neg(int16_t v) { return (int16_t)(-(int)v); }

/* reset: the argument block */
typedef struct {
  unsigned pcm, code, st, n, tb;
  void *jd, *jd92, *dil;
  int16_t a8, a9;
  uint32_t a10w;
} RA;
static void m_reset(W *w, const RA *a) {
  T *t = w->t;
  unsigned lvl = dsplibs_debug_level;
  int alaw = a->pcm != 0;
  unsigned x = a->code & 0x7fu, nmod;
  m_dbg(w, ML, lvl, PFX "Reset called, sessionFlag = %d !\r\n", 1, t->session_);
  t->b424_ = 0; t->arg9_ = (uint16_t)a->a9; t->event_ = 0; t->pcm_ = a->pcm; t->timeBase_ = a->tb; memcpy(&t->arg10_, &a->a10w, 4); t->cnt_ = a->n; t->state_ = a->st;
  if (alaw) { CV(G_RESET, "reset: A-law level"); t->level_ = (uint16_t)cb_lin(w, MC, ML, 1, (int)(x ^ 0xd5u)); }
  else { CV(G_RESET, "reset: mu-law level"); t->level_ = (uint16_t)cb_lin(w, MC, ML, 0, (int)(x ^ 0xffu)); }
  t->sdd_state_ = 0; t->code_ = (uint8_t)a->code; t->dil_ = a->dil;
  cb_descrreset(w, ML, 0);
  t->jd_ = a->jd;
  if (a->jd) { CV(G_RESET, "reset: V90Jd reset"); cb_jdreset(w, ML, a->jd); } else CV(G_RESET, "reset: no V90Jd");
  t->jd92_ = a->jd92;
  if (a->jd92) { CV(G_RESET, "reset: V92Jd reset"); cb_j92reset(w, ML, a->jd92); cb_j92preset(w, ML, a->jd92); } else CV(G_RESET, "reset: no V92Jd");
  t->jdNotCnt_ = 0;
  cb_sdreset(w, ML, t->sd_);
  cb_adreset(w, ML, a->code, t->pcm_, a->a8);
  t->flag3f9_ = 0; t->phase_ = 0; t->cnt3fc_ = 0; t->flag400_ = 0;
  nmod = 0;
  if (a->st == 3) { CV(G_RESET, "reset: state 3 TRN1dKnownData"); m_dbg(w, ML, lvl, PFX "initial state set to TRN1dKnownData\r\n", 0, 0); cb_adlin(w, ML); nmod = a->n; }
  else if (a->st == 0x1a) { CV(G_RESET, "reset: state 0x1a WaitForQTS (no mapping reset)"); m_dbg(w, ML, lvl, PFX "initial state set to WaitForQTS\r\n", 0, 0); }
  else if (a->st == 0) { CV(G_RESET, "reset: state 0 WaitForSd"); m_dbg(w, ML, lvl, PFX "initial state set to WaitForSd\r\n", 0, 0); cb_adlin(w, ML); }
  else { CV(G_RESET, "reset: other state IRREGULAR"); m_dbg(w, ML, lvl, PFX "initial state set to IRREGULAR !!!!! (%d)\r\n", 1, a->st); cb_adlin(w, ML); }
  cb_modreset(w, ML, a->pcm, a->code, 2, nmod, NULL, NULL, NULL, 0);
  t->status_ = 0;
  t->dilLen_ = (unsigned)cb_dill(w, MC, ML, t->dil_, t->pcm_);
  t->lineMode_ = 0;
}

/* the constructor: dil / the block pointers of the model come from the impl's object (the allocation is the impl's) */
typedef struct { void *sd, *ans; } CA;
static void m_ctor(W *w, void *p, unsigned session, void *adid, const CA *c, const RA *ra) {
  T *t = w->t;
  uint32_t th[4];
  cb_modctor(w, ML, p, session);
  t->sdd_state_ = 0;
  cb_descrctor(w, ML, 0x12, 0x17, 0x63);
  t->params_ = p; t->session_ = session; t->adid_ = adid;
  memcpy(&th[0], &w->p->sdDetectorEnergyThreshold, 4);
  memcpy(&th[1], &w->p->sdDetectorPositiveCorrThreshold, 4);
  memcpy(&th[2], &w->p->sdDetectorNegativeCorrThreshold, 4);
  th[3] = (uint32_t)w->p->sdDetectorDetectionCounterThreshold;
  /* the V90SdDetector is allocated and constructed, THEN stored */
  cb_sdctor(w, ML, c->sd, th[0], th[1], th[2], th[3]);
  t->sd_ = c->sd;
  cb_ansctor(w, ML, c->ans, 0x190, 100, 0x48960000u, 0, 0x3f000000u, 8000, 0x32, 0x63);
  t->ans_ = c->ans;
  m_reset(w, ra);
}
/* the destructor */
static void m_dtor(W *w) {
  T *t = w->t;
  unsigned fr = 0;
  void *sd = t->sd_, *ans = t->ans_;
  if (sd) { CV(G_LIFE, "dtor: V90SdDetector freed"); cb_sddtor(w, ML, sd); fr++; } else CV(G_LIFE, "dtor: no V90SdDetector");
  if (ans) {
    CV(G_LIFE, "dtor: ANSamToneDetector freed");
    /* the free count at the time of the call: the free of sd_ has happened */
    { char b[TXTMAX]; snprintf(b, sizeof b, "ans.dtor blk=%d freed=%u", pid(ans), fr); REC(w, ML, 'c', "%s", b); }
    fr++;
  } else CV(G_LIFE, "dtor: no ANSamToneDetector");
  { char b[TXTMAX]; snprintf(b, sizeof b, "descr.dtor freed=%u", fr); REC(w, ML, 'c', "%s", b); }
  { char b[TXTMAX]; snprintf(b, sizeof b, "mod.dtor freed=%u", fr); REC(w, ML, 'c', "%s", b); }
}

/* twoLevelDemod / getDecision / small methods */
static int m_twolevel(W *w, float x, int *out) {
  T *t = w->t;
  int alt = 0, bit;
  int16_t v, r;
  unsigned idx;
  uint16_t e;
  if ((int16_t)w->a->studyDone_[0] != 0) {
    int ia = cb_isalt(w, MC, ML, (int16_t)t->phase_, (int16_t)t->code_, fbits(x));
    alt = (int16_t)ia != 0;
    if (alt) CV(G_SMALL, "twoLevel: studyDone, isAltRbs true -> alt table"); else CV(G_SMALL, "twoLevel: studyDone, isAltRbs false -> base table");
  } else CV(G_SMALL, "twoLevel: study not done -> base table");
  idx = (((unsigned)(uint16_t)t->phase_) << 7) + t->code_;
  memcpy(&e, (const uint8_t *)w->a + (alt ? offsetof(AdidObj, linearMappingAlt_) : offsetof(AdidObj, linearMapping_)) + 2u * idx, 2);
  v = (int16_t)e;
  if (x > 0.0f) { CV(G_SMALL, "twoLevel: x > 0"); bit = 1; r = v; }
  else { CV(G_SMALL, "twoLevel: x <= 0 (or NaN)"); bit = 0; r = m_neg(v); }
  bit = cb_sdd(w, MC, ML, bit);
  bit = cb_descr(w, MC, ML, bit);
  *out = bit;
  return (int)r;
}
static void m_exitdil(W *w) {
  T *t = w->t;
  unsigned s = t->state_;
  int dil = (s >= 0xa && s <= 0x10);
  if (!dil) { CV(G_SMALL, "exitDIL: not a DIL state: nothing"); return; }
  m_ed0(w, ML, PFX "exit DIL\r\n");
  cb_modexit(w, MC, ML);
  if (t->mod_event_ == 6) { CV(G_SMALL, "exitDIL: modulator event 6 -> Phase3 Terminated"); m_ed(w, ML, PFX "Phase3 Terminated @ %d\r\n", t->cnt_); t->state_ = 0x13; t->cnt_ = 0; t->event_ = 0x14; }
  else CV(G_SMALL, "exitDIL: modulator event != 6");
}

/* ------------------------------------------------------------------------------------------------ trial setup */
static unsigned rnd_range(unsigned lo, unsigned hi) { return lo + h_rmod(hi - lo + 1); }
static unsigned pick_pat32(void) {
  static const unsigned v[] = { 0, 1, 2, 3, 0x7fffffffu, 0x80000000u, 0xffffffffu, 0xfffffffeu, 0x30, 0x180, 0x1a, 0x21, 0x22, 0x40, 0x9c40 };
  return h_rpct(60) ? h_rnd() : v[h_rmod(sizeof v / sizeof v[0])];
}
static unsigned pick_level(void) {
  static const unsigned lv[] = { 0, 1, 2, 3, 7, 0x80000000u, 0xffffffffu, 0x80000001u };
  return h_rpct(50) ? 2u : lv[h_rmod(sizeof lv / sizeof lv[0])];
}
static void gen_script(void) {
  unsigned i;
  for (i = 0; i < NSC; i++) {
    g_sc.lina[i] = h_rnd();
    g_sc.linu[i] = h_rnd();
    g_sc.dil[i] = h_rpct(50) ? h_rmod(60000) : h_rnd();
    g_sc.dec92[i] = h_rnd();   /* the low 16 bits are the result, the rest is garbage the caller must drop */
    g_sc.dec90[i] = h_rnd();
    g_sc.mev[i] = h_rpct(45) ? 6u : h_rpct(50) ? 0u : h_rnd();
    g_sc.sdd[i] = h_rpct(60) ? (int)h_rmod(2) : (int)h_rnd();
    g_sc.descr[i] = h_rpct(60) ? (int)h_rmod(2) : (int)h_rnd();
    g_sc.alt[i] = (int)((h_rnd() & 0xffff0000u) | (h_rpct(50) ? 0u : h_rpct(50) ? 1u : ((h_rnd() & 0xffffu) | 1u)));
  }
}
static unsigned kind_size(const HFld *f) {
  unsigned one = 1;
  switch (f->kind) {
    case 'b': one = 1; break;
    case 'h': one = 2; break;
    case 'u': case 'i': case 'f': one = 4; break;
    case 'd': one = 8; break;
    case 'p': case 'o': case 'e': case 'v': one = (unsigned)sizeof(void *); break;
    case 'x': return f->count;
    default: h_die3("kind %c", f->kind);
  }
  return one * (f->count > 1 ? f->count : 1u);
}
static int is_ptr(char k) { return k == 'p' || k == 'o' || k == 'e' || k == 'v'; }
/* member-wise random fill (the content of a member does not depend on the architecture's offsets) */
static void fill_obj(T *t) {
  int i;
  unsigned j, n;
  for (i = 0; i < g_nft; i++) {
    uint8_t *q = (uint8_t *)t + g_ft[i].off;
    n = kind_size(&g_ft[i]);
    if (is_ptr(g_ft[i].kind)) { uintptr_t v = (uintptr_t)h_rnd(); memcpy(q, &v, sizeof v); }
    else for (j = 0; j < n; j++) q[j] = (uint8_t)h_rnd();
  }
}
static void put_f(float *d, uint32_t v) { memcpy(d, &v, 4); }
static uint32_t pick_fword_raw(void);
static uint32_t pick_fword_q(void) {
  uint32_t v = pick_fword_raw();
  if (((v >> 23) & 0xffu) == 0xffu && (v & 0x7fffffu) != 0) v |= 0x400000u;
  return v;
}
static uint32_t pick_fword_raw(void) {
  static const uint32_t b[] = { 0, 0x80000000u, 0x3f800000u, 0xbf800000u, 0x7f800000u, 0x7fc00000u, 0x7fa00001u /* signalling NaN */, 0xffa00001u, 0x00000001u, 0x41200000u };
  return h_rpct(50) ? b[h_rmod(sizeof b / sizeof b[0])] : h_rnd();
}

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
  memset(raw, 0, SZ);
  memset(raw + SZ, 0x5a, CAN);
  for (k = 0; k < FAKEN; k++) { g_fake[k] = sysdep_malloc(0x40); g_fakefreed[k] = 0; for (i = 0; i < 0x40; i++) ((uint8_t *)g_fake[k])[i] = (uint8_t)h_rnd(); }
  { uint8_t *q = (uint8_t *)a; for (i = 0; i < 0x1000; i++) q[i] = (uint8_t)h_rnd(); }
  fill_obj(t);
  /* the pointer members, then member-wise content in the ranges the methods look at */
  t->adid_ = a; t->params_ = p; t->sd_ = g_fake[F_SD]; t->ans_ = g_fake[F_ANS]; t->jd_ = g_fake[F_JD]; t->jd92_ = h_rpct(8) ? NULL : g_fake[F_JD92]; t->dil_ = g_fake[F_DIL];
  t->session_ = h_rpct(40) ? 0u : h_rpct(50) ? 1u : h_rnd();
  t->state_ = h_rpct(60) ? h_rmod(0x22) : h_rpct(50) ? pick_pat32() : h_rnd();
  t->cnt_ = h_rpct(40) ? h_rmod(200) : h_rnd();
  t->phase_ = h_rpct(92) ? h_rmod(6) : h_rmod(8);
  t->mod_event_ = h_rpct(30) ? 6u : h_rnd();
  t->jdNotCnt_ = h_rpct(60) ? h_rmod(16) : h_rnd();
  for (i = 0; i < 768; i++) { a->linearMapping_[i] = (uint16_t)(h_rpct(30) ? h_rmod(3) : h_rnd()); a->linearMappingAlt_[i] = (uint16_t)(h_rpct(30) ? h_rmod(3) : h_rnd()); }
  a->studyDone_[0] = (uint16_t)(h_rpct(40) ? 0u : h_rpct(60) ? 1u : (h_rnd() & 0xffffu));
  for (i = 0; i < 6; i++) a->maxUcodeArray_[i] = (uint8_t)h_rnd();
  p->params = g_fake[F_SPARE];
  put_f(&p->sdDetectorEnergyThreshold, pick_fword_q());
  put_f(&p->sdDetectorPositiveCorrThreshold, pick_fword_q());
  put_f(&p->sdDetectorNegativeCorrThreshold, pick_fword_q());
  p->sdDetectorDetectionCounterThreshold = (int32_t)(h_rpct(50) ? rnd_range(1, 20) : h_rnd());
  put_f(&p->phase4MeanErrorBefToAftUpdateRatioThresh, pick_fword_raw());
  put_f(&p->unk440, pick_fword_raw());
}

static uint32_t pick_xw(int snan_ok) {
  static const uint32_t b[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7f800000u, 0xff800000u, 0x3f800000u, 0xbf800000u, 0x00000001u, 0x80000001u, 0x47000000u, 0xc7000000u, 0x3f000000u };
  uint32_t v;
  (void)snan_ok;   /* quiet NaNs only: g++ -O0 on i386 forwards a float argument with flds / fstps (documented in the port's header comment) */
  if (h_rpct(40)) v = b[h_rmod(sizeof b / sizeof b[0])];
  else if (h_rpct(50)) { float g = (float)(int)(h_rnd() % 70000u) - 35000.0f; g += (float)(h_rnd() & 0xff) / 256.0f; memcpy(&v, &g, 4); }
  else v = h_rnd();
  if (((v >> 23) & 0xffu) == 0xffu && (v & 0x7fffffu) != 0) v |= 0x400000u;   /* quiet NaNs only: the x87 loads of twoLevelDemod would quiet a signalling one */
  return v;
}

/* ------------------------------------------------------------------------------------------------ one op */
enum { O_RESETJD, O_INCFP, O_SETALT, O_GETMAX, O_JDNOT, O_SETSESS, O_CLEARV, O_EXITDIL, O_SETDIG, O_ENTERANS, O_GETDEC, O_TWOLEVEL, O_RESET, NSMALL = O_RESET };
static const char *g_opname[] = { "resetJdNotDetector", "incrementFramePosition", "setAltRbsParams", "getMaxUcode", "JdNotDetector", "setSessionFlag", "clearVerificationStatus", "exitDIL",
                                  "setDigitalImairmentsInfo", "enterWaitForANSpcmDrop", "getDecision", "twoLevelDemod", "reset" };
static unsigned long g_ncalls;
static int g_showall;
static const char *g_what = "setup";

static void copy_world_to_model(void) {
  memcpy(&g_mt, g_iw.t, SZ);
  memcpy(&g_ma, g_iw.a, sizeof g_ma);
  memcpy(&g_mp, g_iw.p, sizeof g_mp);
  g_mw.t = &g_mt; g_mw.a = &g_ma; g_mw.p = &g_mp;
}
static void cmp_world(const char *what) {
  if (memcmp(&g_mt, g_iw.t, SZ)) {
    const uint8_t *x = (const uint8_t *)&g_mt, *y = (const uint8_t *)g_iw.t;
    unsigned i;
    for (i = 0; i < SZ; i++) if (x[i] != y[i]) break;
    h_die3("%s: the object differs from the model at mirror offset 0x%x (model %02x, impl %02x)", what, i, x[i], y[i]);
  }
  if (memcmp(&g_ma, g_iw.a, sizeof g_ma)) h_die3("%s: the detector differs from the model", what);
  if (memcmp(&g_mp, g_iw.p, sizeof g_mp)) h_die3("%s: the parameter block differs from the model", what);
  { unsigned i; for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", what, i); }
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
/* the canonical object line: every member by kind, pointers as block ids, arrays hashed */
static void emit_obj(const char *tag) {
  char line[3000];
  int i, n = 0;
  const T *t = g_iw.t;
  n += snprintf(line + n, sizeof line - (size_t)n, "%s", tag);
  for (i = 0; i < g_nft && n < (int)sizeof line - 80; i++) {
    const uint8_t *q = (const uint8_t *)t + g_ft[i].off;
    char k = g_ft[i].kind;
    if (is_ptr(k)) { void *v; memcpy(&v, q, sizeof v); n += snprintf(line + n, sizeof line - (size_t)n, " %d", pid(v)); }
    else if (k == 'b' && g_ft[i].count <= 1) n += snprintf(line + n, sizeof line - (size_t)n, " %02x", q[0]);
    else if (k == 'h' && g_ft[i].count <= 1) { uint16_t v; memcpy(&v, q, 2); n += snprintf(line + n, sizeof line - (size_t)n, " %04x", v); }
    else if ((k == 'u' || k == 'i' || k == 'f') && g_ft[i].count <= 1) { uint32_t v; memcpy(&v, q, 4); n += snprintf(line + n, sizeof line - (size_t)n, " %08x", v); }
    else n += snprintf(line + n, sizeof line - (size_t)n, " #%08x", fnv(q, kind_size(&g_ft[i])));
  }
  h_emit(g_show, "%s", line);
}

typedef struct { int op; RA ra; unsigned arg; uint32_t xw; int bit; } Args;
static float wf(uint32_t w) { float f; memcpy(&f, &w, 4); return f; }

static void run_impl(const Args *a, long *ret, int *outv) {
  T *t = g_iw.t;
  switch (a->op) {
    case O_RESETJD: f_resetjd(t); break;
    case O_INCFP: f_incfp(t); break;
    case O_SETALT: f_setalt(t); break;
    case O_GETMAX: *ret = ((uint8_t *)f_getmax(t) == (uint8_t *)g_iw.a + offsetof(AdidObj, maxUcodeArray_)) ? 1 : -1; break;   /* the pointer into the detector (the offset differs per architecture) */
    case O_JDNOT: *ret = f_jdnot(t, a->bit); break;
    case O_SETSESS: f_setsess(t, a->arg); break;
    case O_CLEARV: f_clearv(t); break;
    case O_EXITDIL: f_exitdil(t); break;
    case O_SETDIG: f_setdig(t); break;
    case O_ENTERANS: f_enterans(t); break;
    case O_GETDEC: *ret = (short)f_getdec(t, fw_make(a->xw)); break;
    case O_TWOLEVEL: *ret = f_twolevel(t, wf(a->xw), outv); break;
    case O_RESET: f_reset(t, a->ra.pcm, (unsigned char)a->ra.code, a->ra.st, a->ra.n, a->ra.jd, a->ra.jd92, a->ra.dil, a->ra.a8, a->ra.a9, fw_make(a->ra.a10w), a->ra.tb); break;
    default: h_die3("op %d", a->op);
  }
}
static void run_model(const Args *a, long *ret, int *outv) {
  W *w = MW;
  T *t = w->t;
  unsigned lvl = dsplibs_debug_level;
  switch (a->op) {
    case O_RESETJD: CV(G_SMALL, "resetJdNotDetector"); t->jdNotCnt_ = 0; break;
    case O_INCFP: CV(G_SMALL, "incrementFramePosition"); t->phase_ = (t->phase_ + 1 == 6) ? 0 : t->phase_ + 1; break;
    case O_SETALT: CV(G_SMALL, "setAltRbsParams"); memcpy(&w->p->phase4MeanErrorBefToAftUpdateRatioThresh, &w->p->unk440, 4); break;
    case O_GETMAX: CV(G_SMALL, "getMaxUcode"); *ret = 1; break;
    case O_JDNOT:
      if (a->bit) { CV(G_SMALL, "JdNotDetector: bit set"); t->jdNotCnt_ = 0; } else { CV(G_SMALL, "JdNotDetector: bit clear"); t->jdNotCnt_++; }
      if (t->jdNotCnt_ > 11u) { if (t->cnt_ % 72u == 12u) { CV(G_SMALL, "JdNotDetector: detected"); *ret = 1; } else { CV(G_SMALL, "JdNotDetector: run long, cnt phase wrong"); *ret = 0; } }
      else { CV(G_SMALL, "JdNotDetector: run short"); *ret = 0; }
      break;
    case O_SETSESS: CV(G_SMALL, "setSessionFlag"); t->session_ = a->arg; cb_modsess(w, ML, a->arg); break;
    case O_CLEARV: CV(G_SMALL, "clearVerificationStatus"); m_dbg(w, ML, lvl, PFX "clearVerificationStatus called\r\n", 0, 0); t->verify_ = 0; break;
    case O_EXITDIL: m_exitdil(w); break;
    case O_SETDIG: CV(G_SMALL, "setDigitalImairmentsInfo"); cb_detmax(w, ML, (int16_t)t->impairInfo_); cb_findpad(w, ML); cb_applypad(w, ML); break;
    case O_ENTERANS:
      if (t->state_ != 0x1e) { CV(G_SMALL, "enterWaitForANSpcmDrop: enters"); m_ed0(w, ML, PFX "enter WaitForANSpcmDrop\r\n"); t->state_ = 0x1e; t->cnt_ = 0; }
      else CV(G_SMALL, "enterWaitForANSpcmDrop: already there");
      break;
    case O_GETDEC:
      if (t->session_ != 0) { CV(G_SMALL, "getDecision: V.92"); *ret = cb_dec(w, MC, ML, 1, a->xw); } else { CV(G_SMALL, "getDecision: V.90"); *ret = cb_dec(w, MC, ML, 0, a->xw); }
      break;
    case O_TWOLEVEL: *ret = m_twolevel(w, wf(a->xw), outv); break;
    case O_RESET: m_reset(w, &a->ra); break;
    default: h_die3("op %d", a->op);
  }
}
static void do_op(const Args *a) {
  long mr = 0, ir = 0;
  int mo = 0x5a5a5a5a, io = 0x5a5a5a5a;
  g_what = g_opname[a->op];
  copy_world_to_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  g_free_base = h_nfree;
  cov_clear();
  run_model(a, &mr, &mo);
  g_show = h_sample(&g_symctr, 12, 250) || g_showall;
  h_emit(g_show, "op %s lvl=%u arg=%u x=%08x bit=%d st=%u cnt=%u", g_what, dsplibs_debug_level, a->arg, a->xw, a->bit, (unsigned)g_iw.t->state_, (unsigned)g_iw.t->cnt_);
  run_impl(a, &ir, &io);
  __asm__ volatile("" ::: "memory");
  { unsigned i; for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t); }
  if (mr != ir) h_die3("%s: result: model %ld, impl %ld", g_what, mr, ir);
  if (mo != io) h_die3("%s: out value: model %d, impl %d", g_what, mo, io);
  h_emit(g_show, "ret %ld out %d", ir, io);
  log_cmp();
  cmp_world(g_what);
  emit_obj("obj");
  h_emit(g_show, "params p438=%08x", pword(&g_iw.p->phase4MeanErrorBefToAftUpdateRatioThresh));
  cov_flush();
  g_ncalls++;
}

static void free_world(void) {
  int k;
  for (k = 0; k < FAKEN; k++) if (!g_fakefreed[k]) sysdep_free(g_fake[k]);
}

/* ------------------------------------------------------------------------------------------------ scenarios */
static void pick_ra(RA *r) {
  static const unsigned sts[] = { 0, 3, 0x1a, 1, 2, 4, 5, 9, 0x21, 0x22, 0x1b, 0x80000000u, 0xffffffffu, 0x19 };
  r->pcm = h_rpct(45) ? 0u : h_rpct(70) ? 1u : h_rnd();
  r->code = h_rpct(30) ? (unsigned)(h_rnd() & 0xffu) : (h_rpct(50) ? 0x40u : (unsigned)(h_rnd() & 0xffu));
  r->st = h_rpct(75) ? sts[h_rmod(sizeof sts / sizeof sts[0])] : h_rmod(0x30);
  r->n = h_rpct(50) ? h_rmod(80) : pick_pat32();
  r->tb = h_rpct(50) ? 0u : pick_pat32();
  r->jd = h_rpct(25) ? NULL : g_fake[F_JD];
  r->jd92 = h_rpct(25) ? NULL : g_fake[F_JD92];
  r->dil = h_rpct(25) ? NULL : g_fake[F_DIL];
  r->a8 = (int16_t)(h_rpct(50) ? (int)h_rmod(8) - 2 : (int)(int16_t)h_rnd());
  r->a9 = (int16_t)(h_rpct(50) ? (int)h_rmod(3) : (int)(int16_t)h_rnd());
  r->a10w = pick_fword_raw();
}

static void trial_small(void) {
  unsigned ncalls = rnd_range(1, 6), k;
  new_world();
  gen_script();
  dsplibs_debug_level = pick_level();
  for (k = 0; k < ncalls; k++) {
    Args a;
    T *t = g_iw.t;
    memset(&a, 0, sizeof a);
    a.op = (int)h_rmod(NSMALL);
    a.arg = h_rpct(50) ? h_rmod(2) : h_rnd();
    a.xw = pick_xw(a.op == O_GETDEC);
    a.bit = h_rpct(50) ? 0 : (h_rpct(70) ? 1 : (int)h_rnd());
    if (a.op == O_JDNOT) {
      if (h_rpct(70)) { t->jdNotCnt_ = h_rpct(60) ? 10u + h_rmod(3) : h_rmod(20); if (h_rpct(10)) t->jdNotCnt_ = 0x80000000u + h_rmod(14); if (h_rpct(60)) a.bit = 0; }
      if (h_rpct(60)) t->cnt_ = 72u * h_rmod(5) + (h_rpct(75) ? 12u : 11u + h_rmod(3));
    }
    if (a.op == O_EXITDIL && h_rpct(75)) t->state_ = 0xa + h_rmod(7) + (h_rpct(15) ? (h_rpct(50) ? 7u : 0xfffffff0u) : 0u);
    if (a.op == O_ENTERANS && h_rpct(40)) t->state_ = 0x1e;
    if (a.op == O_TWOLEVEL) {
      t->phase_ = (h_rpct(25) ? (h_rnd() & 0xffff0000u) : 0u) | h_rmod(8);
      t->code_ = (uint8_t)h_rnd();
    }
    if (a.op == O_INCFP) t->phase_ = h_rpct(50) ? h_rmod(8) : h_rnd();
    if (h_rpct(15)) gen_script();
    if (h_rpct(10)) dsplibs_debug_level = pick_level();
    do_op(&a);
  }
  free_world();
  h_trial_end("p36");
}
static void trial_reset(void) {
  Args a;
  new_world();
  gen_script();
  dsplibs_debug_level = pick_level();
  memset(&a, 0, sizeof a);
  a.op = O_RESET;
  pick_ra(&a.ra);
  do_op(&a);
  if (h_rpct(30)) { pick_ra(&a.ra); if (h_rpct(30)) dsplibs_debug_level = pick_level(); do_op(&a); }
  free_world();
  h_trial_end("p36");
}

/* constructor (C1 / C2) then destructor (D1 / D2); and the destructor alone with NULL / fake block pointers */
static void trial_life(void) {
  int c2 = (int)h_rmod(2), d2 = (int)h_rmod(2);
  T *t;
  CA ca;
  RA ra;
  long dummy;
  uint32_t hv;
  new_world();
  gen_script();
  dsplibs_debug_level = pick_level();
  t = g_iw.t;
  /* ---- constructor: the object holds random bytes member-wise (new_world filled it); the pointers the constructor must define are garbage */
  { void *z = NULL; (void)z; }
  fill_obj(t);
  copy_world_to_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  g_free_base = h_nfree;
  g_what = c2 ? "ctor C2" : "ctor C1";
  { unsigned session = h_rpct(40) ? 0u : h_rpct(50) ? 1u : h_rnd(); unsigned long nm0 = h_nmalloc; cov_clear();
    g_show = h_sample(&g_symctr, 12, 120) || g_showall;
    h_emit(g_show, "op %s lvl=%u session=%u", g_what, dsplibs_debug_level, session);
    if (c2) f_c2(t, g_iw.p, NULL, session, g_iw.a); else f_c1(t, g_iw.p, NULL, session, g_iw.a);
    __asm__ volatile("" ::: "memory");
    if (h_nmalloc - nm0 != 2) h_die3("ctor: %lu allocations", h_nmalloc - nm0);
    ca.sd = t->sd_; ca.ans = t->ans_;
    { HAl *s = h_find_al(ca.sd, 0), *n = h_find_al(ca.ans, 0);
      if (!s || !n || s->p != (uint8_t *)ca.sd || n->p != (uint8_t *)ca.ans || s->id + 1 != n->id) h_die3("ctor: sd_ / ans_ are not the two consecutive heap blocks"); }
    memset(&ra, 0, sizeof ra);
    ra.pcm = 0; ra.code = 0x40; ra.st = 0; ra.n = 0; ra.tb = 0; ra.jd = NULL; ra.jd92 = NULL; ra.dil = NULL; ra.a8 = 0; ra.a9 = 1; ra.a10w = 0;
    if (c2) CV(G_LIFE, "ctor C2"); else CV(G_LIFE, "ctor C1");
    m_ctor(MW, g_iw.p, session, g_iw.a, &ca, &ra);
    { unsigned i; for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t); }
    log_cmp();
    cmp_world(g_what);
    emit_obj("obj");
    cov_flush();
    g_ncalls++;
  }
  (void)dummy; (void)hv;
  /* ---- destructor of the object the constructor built (both heap blocks are live: the destructor frees them) */
  copy_world_to_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  g_free_base = h_nfree;
  g_what = d2 ? "dtor D2" : "dtor D1";
  cov_clear();
  if (d2) CV(G_LIFE, "dtor D2 after ctor"); else CV(G_LIFE, "dtor D1 after ctor");
  m_dtor(MW);
  h_emit(g_show, "op %s", g_what);
  if (d2) f_d2(t); else f_d1(t);
  __asm__ volatile("" ::: "memory");
  { unsigned i; for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t); }
  log_cmp();
  cmp_world(g_what);   /* nothing is cleared */
  emit_obj("obj");
  h_emit(g_show, "free %lu", h_nfree - g_free_base);
  if (h_nfree - g_free_base != 2) h_die3("dtor: %lu frees", h_nfree - g_free_base);
  cov_flush();
  g_ncalls++;
  /* ---- destructor alone: sd_ / ans_ NULL or fake blocks */
  d2 = (int)h_rmod(2);
  fill_obj(t);
  t->adid_ = g_iw.a; t->params_ = g_iw.p;
  t->sd_ = h_rpct(35) ? NULL : g_fake[F_SD];
  t->ans_ = h_rpct(35) ? NULL : g_fake[F_ANS];
  copy_world_to_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  g_free_base = h_nfree;
  g_what = d2 ? "dtor D2 (alone)" : "dtor D1 (alone)";
  cov_clear();
  m_dtor(MW);
  g_show = h_sample(&g_symctr, 12, 120) || g_showall;
  h_emit(g_show, "op %s sd=%d ans=%d", g_what, pid(t->sd_), pid(t->ans_));
  { void *sd = t->sd_, *ans = t->ans_;
    if (d2) f_d2(t); else f_d1(t);
    __asm__ volatile("" ::: "memory");
    if (sd) g_fakefreed[F_SD] = 1;
    if (ans) g_fakefreed[F_ANS] = 1; }
  { unsigned i; for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t); }
  log_cmp();
  cmp_world(g_what);
  emit_obj("obj");
  h_emit(g_show, "free %lu", h_nfree - g_free_base);
  cov_flush();
  g_ncalls++;
  free_world();
  h_trial_end("p36");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr, i;
  const char *seed = getenv("P36_SEED");
  CovSite *c;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P36_ALL")) g_showall = 1;
  p07_layout_check(); p18_layout_check(); p34_layout_check();
  h_x87_check();
  h_arena_init();
  /* the field table of the object: the sub-object heap pointers of sd_ / ans_ are plain 'p' (their blocks have a per-architecture size) */
  g_nft = NF_Ph3DemObj;
  if (g_nft > (int)(sizeof g_ft / sizeof g_ft[0])) h_die3("field table too long");
  memcpy(g_ft, f_Ph3DemObj, sizeof f_Ph3DemObj);
  for (i = 0; i < g_nft; i++) if (g_ft[i].kind == 'o') g_ft[i].kind = 'p';
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P36_TRIALS") ? atoi(getenv("P36_TRIALS")) : (sc == 0 ? 30000 : sc == 1 ? 30000 : 12000);
  if (sc < 0 || sc > 2) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  for (tr = 0; tr < ntr; tr++) { if (sc == 0) trial_small(); else if (sc == 1) trial_reset(); else trial_life(); }
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (c = __start_covsites; c < __stop_covsites; c++) {
    int req = sc == 0 ? (c->grp == G_SMALL) : sc == 1 ? (c->grp == G_RESET) : (c->grp == G_LIFE);
    if (!req) continue;
    h_emit(1, "COV %s %lu", c->name, c->n);
    if (c->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", c->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P36_NOCOV")) ? 2 : 0;
}
