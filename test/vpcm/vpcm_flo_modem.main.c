/*
 * Differential test harness for port chunk P47 (src/vpcm/vpcm_flo_modem.cpp): VPcmFloModem B: the constructor (C1 / C2), the destructor (D1 / D2) and the other 24 methods (setters, getV90JaBits / getV90CpBits,
 * setPcmSessionType, externalReset, setPhaseIIinfo, getUinfoValue, vPcmResetPhase3Modem, enterPhase3, getConstellation / getLinearEqualizer / getDFE, qcLineVerification).
 * Derived from test/vpcm/vpcm_flo_modem_run.main.c (recorders + independent model + per-record snapshot + world blocks with model copies).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/vpcm_flo_modem.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2: the
 * port object alone) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/vpcm_flo_modem.blob32.out).  The functions are reached through __asm__ labels; the object under test is the generated C mirror
 * (test/vpcm/vpcm_flo_modem.layout: PcmFlo47Obj); the foreign blocks are the mirrors of the earlier chunks (V90Demodulator DmdObj, V90Equalizer EquObj, V90Phase3Demodulator Ph3DemObj, V90ConnectionEvaluator EvObj,
 * V90Parameters V90ParObj) and the P47 phase 2 info views (P47V90InfoObj / P47V92InfoObj).
 *
 * Every callee is a RECORDER: V90Modem::progress / setSessionFlag / reset, V92Modem::reset, V92EchoCanceller::reset, V90Parameters::init / initSession, V92Parameters::init, V90Demodulator::reInit / enterPhase3,
 * V90Phase3Demodulator::enterWaitForANSpcmDrop, SineWave::generate, DILdescriptorPacker, the three V.34 entry points, edprintf / dsplibs_debug_printf, and (constructor / destructor scenario) the six sub-object
 * constructors and destructors.  The record carries the arguments as tokens and a SNAPSHOT of the object + the foreign blocks; a recorder then applies its scripted EFFECT and may POKE the world by script (debug
 * level, pointer members, state words): the time at which the blob / port reloads a member is observable.  The INDEPENDENT MODEL (m_*: written from porting/decompiled/dp/p47_vpcmflo_b.txt in a different shape than the
 * port) runs AFTER the implementation on copies of the world, through the same environment functions, and must reproduce every record, the returned value, the object and every foreign block (exit 3 otherwise).
 *
 * Scenarios (own process each): 0 the setters + getV90JaBits / getV90CpBits, 1 setPcmSessionType / externalReset / setPhaseIIinfo / vPcmResetPhase3Modem / enterPhase3 / qcLineVerification, 2 getUinfoValue +
 * the three visual diagnostics (x87), 3 lifecycle: 1 - 4 calls of any function on the same object, 4 constructors C1 / C2 + destructors D1 / D2 (recorders fill the sub-objects).
 */

#include "p47_common.h"
#include "pcm_parameters_layout.h"
#include "v90_connection_evaluator_layout.h"
#include "v90_phase3_demod_v92_decision_layout.h"
#include "v90_equalizer_process_layout.h"
#include "v90_demodulator_progress_layout.h"
#include "vpcm_flo_modem_layout.h"

/* ---- the functions under test ---- */
extern void f_ja(void *, unsigned) L("_ZN12VPcmFloModem18setTerminateJaFlagEh");
extern void f_cp(void *, unsigned) L("_ZN12VPcmFloModem18setTerminateCpFlagEh");
extern void f_cpnot(void *, unsigned) L("_ZN12VPcmFloModem21setTerminateCpNotFlagEh");
extern void f_minseq(void *, unsigned) L("_ZN12VPcmFloModem26setMinNofTransmitSequencesEt");
extern void f_nofbits(void *, unsigned) L("_ZN12VPcmFloModem16setNofBitsPhase4Ej");
extern void f_resetbp(void *) L("_ZN12VPcmFloModem15resetBitPointerEv");
extern int f_getja(void *, short *) L("_ZN12VPcmFloModem12getV90JaBitsEPs");
extern int f_getcp(void *, short *) L("_ZN12VPcmFloModem12getV90CpBitsEPs");
extern void f_baud90(void *) L("_ZN12VPcmFloModem16setV34BaudForV90Ev");
extern void f_baud34(void *) L("_ZN12VPcmFloModem16setV34BaudForV34Ev");
extern void f_ireset(void *) L("_ZN12VPcmFloModem13internalResetEv");
extern void f_copymp(void *) L("_ZN12VPcmFloModem22copyMpInfoForInterfaceEv");
extern void f_sess(void *, int) L("_ZN12VPcmFloModem17setPcmSessionTypeEi");
extern void f_ereset(void *) L("_ZN12VPcmFloModem13externalResetEv");
extern void f_ph2(void *, int *, int) L("_ZN12VPcmFloModem14setPhaseIIinfoEPii");
extern int f_uinfo(void *, short) L("_ZN12VPcmFloModem13getUinfoValueEs");
extern void f_reset3(void *) L("_ZN12VPcmFloModem20vPcmResetPhase3ModemEv");
extern void f_enter3(void *) L("_ZN12VPcmFloModem11enterPhase3Ev");
extern int f_const(void *, void *, unsigned long) L("_ZN12VPcmFloModem16getConstellationEP11int_complexm");
extern int f_lineq(void *, void *, unsigned long) L("_ZN12VPcmFloModem18getLinearEqualizerEP11int_complexm");
extern int f_dfe(void *, void *, unsigned long) L("_ZN12VPcmFloModem6getDFEEP11int_complexm");
extern int f_qc(void *, float *, float *, unsigned, int *, int *, int *, int *) L("_ZN12VPcmFloModem18qcLineVerificationEPfS0_jPiS1_S1_S1_");
extern void f_c1(void *, void *, unsigned, void *, unsigned, unsigned, unsigned) L("_ZN12VPcmFloModemC1EPv12V90ModemSideP19_tagModemParametersj20V90ComputationalMode20V92ComputationalMode");
extern void f_c2(void *, void *, unsigned, void *, unsigned, unsigned, unsigned) L("_ZN12VPcmFloModemC2EPv12V90ModemSideP19_tagModemParametersj20V90ComputationalMode20V92ComputationalMode");
extern void f_d1(void *) L("_ZN12VPcmFloModemD1Ev");
extern void f_d2(void *) L("_ZN12VPcmFloModemD2Ev");

/* sysdep_malloc / sysdep_free of the blob's other code (the shared harness library is compiled with -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free); nothing in P47 allocates */
void *h_sysdep_malloc(unsigned size);
void h_sysdep_free(void *q);
void *sysdep_malloc(unsigned size) { return h_sysdep_malloc(size); }
void sysdep_free(void *q) { h_sysdep_free(q); }

/* ------------------------------------------------------------------------------------------------ the world */
enum { B_OBJ, B_DEM, B_DEM2, B_PAR, B_PAR2, B_MPAR, B_EQ, B_EQ2, B_P3, B_P3B, B_CE, B_CE2, B_I90, B_I90B, B_I92, B_I92B, B_V92P, B_LIN, B_DFE, B_EQBUF, B_PROBE, B_INFO0, B_CTX, B_CTX2,
       B_IN, B_OUT, B_BITS, B_NB, B_BITS2, B_NB2, B_COUT, B_SHORT, NBLK };
enum { X_V90 = NBLK, X_V92, X_EC, X_UA, X_UB, X_L2, X_UD, X_DIL, X_CPNF, X_CPNL, X_SINE, X_ANS, X_IIR, NID };
static const char *const bname[NID] = { "obj", "dem", "dem2", "par", "par2", "mpar", "eq", "eq2", "p3", "p3b", "ce", "ce2", "i90", "i90b", "i92", "i92b", "v92p", "lin", "dfe", "eqbuf", "probe", "info0", "ctx", "ctx2",
                                        "in", "out", "bits", "nb", "bits2", "nb2", "cout", "short",
                                        "obj.v90_", "obj.v92_", "obj.ec_", "obj.uinfoA_", "obj.uinfoB_", "obj.l2_", "obj.uinfoD_", "obj.dil_", "obj.cpnotFrame_", "obj.cpnotLen_", "obj.sine_", "obj.ansam_", "obj.iir_" };
typedef struct { uint8_t *b[NBLK]; } W;
static W g_iw, g_mw;
static int g_blkidx[NBLK];
static uint8_t *g_can;
#define OBJ(w) ((PcmFlo47Obj *)(w)->b[B_OBJ])
typedef uint16_t __attribute__((may_alias)) u16a;   /* the frame buffers are byte arrays in the mirror (hashed by the dump); the blob copies them as `short` */
#define CPNF(o) ((u16a *)(o)->cpnotFrame_)
#define CPF(o) ((u16a *)(o)->cpFrame_)
#define DBG() (dsplibs_debug_level > 1)
enum { BUF = 256 };   /* the sample / complex / coefficient buffers have 256 elements */

enum { G_SET = 0, G_SES = 1, G_UIN = 2, G_LIFE = 3, G_CTOR = 4 };

/* ------------------------------------------------------------------------------------------------ script, pokes, effects */
typedef struct { uint32_t kind[NSC], val[NSC], res[NSC]; } Script;
typedef struct { unsigned pk, rv; } Cur;
static Script g_sc;
static Cur g_ic, g_mc;
static Log g_il, g_ml;
static int g_show, g_showall;
static unsigned long g_symctr, g_ncalls;
static unsigned g_pokemask;
static const char *g_what = "setup";
#define IW (&g_iw)
#define IL (&g_il)
#define IC (&g_ic)
#define MW (&g_mw)
#define ML (&g_ml)
#define MC (&g_mc)

/* a pointer the object holds (an impl-world address) as seen from world w: the model reads its own copies of the blocks */
static void *P(W *w, const void *p) {
  int k;
  if (w != &g_mw || !p) return (void *)p;
  for (k = 0; k < NBLK; k++) {
    const uint8_t *b = g_iw.b[k];
    if ((const uint8_t *)p >= b && (const uint8_t *)p < b + g_blk[g_blkidx[k]].n) return w->b[k] + ((const uint8_t *)p - b);
  }
  h_die3("model: pointer %p is not in a world block", p);
}
static uint32_t sres(Cur *c) { return g_sc.res[c->rv++ % NSC]; }

static void gen_script(void) {
  unsigned i;
  for (i = 0; i < NSC; i++) {
    g_sc.kind[i] = h_rpct(72) ? 0u : 1u + h_rmod(14);
    g_sc.val[i] = h_rnd();
    g_sc.res[i] = h_rnd();
  }
}
static uint32_t pick_w(uint32_t v) { switch (v & 7u) { case 0: return 0; case 1: return 1; case 2: return 0xffffffffu; case 3: return 0x80000000u; case 4: return 0x10000u; default: return v >> 3; } }
static uint32_t pick_qccode(uint32_t v) { switch (v % 6u) { case 0: case 1: return 0x3au; case 2: case 3: return 0x3bu; case 4: return v >> 8; default: return (v >> 8) % 0x40u; } }
static void *cand(unsigned v, int blk) { return v % 4 == 0 ? g_iw.b[blk + 1] : g_iw.b[blk]; }
/* the pokes the recorders apply AFTER recording (the same function runs in the impl's and the model's world) */
static void poke(W *w, Cur *c) {
  unsigned k = c->pk++ % NSC, kind = g_sc.kind[k], v = g_sc.val[k];
  PcmFlo47Obj *o = OBJ(w);
  if (!kind || !((g_pokemask >> kind) & 1u)) return;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->v90Dem_ = cand(v >> 3, B_DEM); break;
    case 3: { DmdObj *d = (DmdObj *)P(w, o->v90Dem_); if (v & 8) d->state_ = pick_qccode(v >> 4); else d->phase_ = pick_w(v >> 4); break; }
    case 4: { DmdObj *d = (DmdObj *)P(w, o->v90Dem_);
      if (v & 8) d->p3_ = (v & 16) ? g_iw.b[B_P3B] : g_iw.b[B_P3];
      else ((Ph3DemObj *)P(w, d->p3_))->verify_ = (v & 16) ? (uint32_t)(v >> 8) : pick_w(v >> 5);
      break; }
    case 5: switch ((v >> 3) % 3u) { case 0: o->qcState_ = (v & 0x80) ? (uint32_t)(v >> 12 & 3u) : pick_w(v >> 8); break; case 1: o->qcFlag_ = pick_w(v >> 8); break; default: o->qcCount_ = (v & 0x80) ? (int)(v >> 12 & 0x1ffu) + 0x1d8 : (int)pick_w(v >> 8); break; }
      break;
    case 6: if (v & 8) o->v90Info_ = (v & 16) ? g_iw.b[B_I90B] : g_iw.b[B_I90]; else o->v92Info_ = (v & 16) ? g_iw.b[B_I92B] : g_iw.b[B_I92]; break;
    case 7: if (v & 8) o->flag6120_ = pick_w(v >> 4); else o->v92Session_ = pick_w(v >> 4); break;
    case 8: if (v & 8) o->v90Params_ = (v & 16) ? g_iw.b[B_PAR2] : g_iw.b[B_PAR];
      else { V90ParObj *p = (V90ParObj *)P(w, o->v90Params_); p->phase2InfoUinfo = (int32_t)(v & 16 ? (v >> 5 & 1 ? 0 : (int32_t)(v >> 8)) : (int32_t)pick_w(v >> 5)); }
      break;
    case 9: switch ((v >> 3) % 5u) { case 0: o->dropToV34_ = (uint8_t)(v >> 12); break; case 1: o->jaFlag_ = (uint8_t)(v >> 12); break; case 2: o->cpFlag_ = (uint8_t)(v >> 12); break;
        case 3: o->cpnotFlag_ = (uint8_t)(v >> 12); break; default: o->cpnotCopied_ = (uint8_t)(v >> 12); break; }
      break;
    case 10: w->b[B_MPAR][3] = (uint8_t)(v >> 7); break;
    case 11: o->ctx_ = (v & 8) ? g_iw.b[B_CTX2] : g_iw.b[B_CTX]; break;
    case 12: { DmdObj *d = (DmdObj *)P(w, o->v90Dem_); d->eq_ = (v & 8) ? g_iw.b[B_EQ2] : g_iw.b[B_EQ]; break; }
    case 13: o->bitPtr_ = (uint16_t)((v >> 8) % 1500u); o->cpLen_ = (uint16_t)(v >> 6); break;
    case 14: ((EvObj *)P(w, ((DmdObj *)P(w, o->v90Dem_))->ce_))->delayedRetrainRequest_ = (int32_t)pick_w(v >> 4); break;
    default: break;
  }
}

/* ------------------------------------------------------------------------------------------------ the snapshot of the world at the moment of a record */
static uint32_t hash_tbl(uint32_t h, const uint8_t *t, const HFld *tbl, int nf) {
  int i;
  for (i = 0; i < nf; i++) {
    const HFld *fl = &tbl[i];
    unsigned n = fl->count ? fl->count : 1, es, j;
    const uint8_t *q = t + fl->off;
    switch (fl->kind) {
      case 'p': case 'o':
        for (j = 0; j < n; j++) { void *v; int id; memcpy(&v, q + j * sizeof v, sizeof v); id = tok_of(v); h = fnv_upd(h, &id, sizeof id); }
        continue;
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'd': es = 8; break;
      default: es = 4; break;
    }
    h = fnv_upd(h, q, (size_t)n * es);
  }
  return h;
}
static void obj_snapshot(char *out, size_t cap, int model) {
  const W *w = model ? &g_mw : &g_iw;
  uint32_t h = hash_tbl(2166136261u, w->b[B_OBJ], f_PcmFlo47Obj, NF_PcmFlo47Obj), f = 2166136261u;
  int k;
  f = hash_tbl(f, w->b[B_I90], f_P47V90InfoObj, NF_P47V90InfoObj);
  f = hash_tbl(f, w->b[B_I90B], f_P47V90InfoObj, NF_P47V90InfoObj);
  f = hash_tbl(f, w->b[B_I92], f_P47V92InfoObj, NF_P47V92InfoObj);
  f = hash_tbl(f, w->b[B_I92B], f_P47V92InfoObj, NF_P47V92InfoObj);
  for (k = 0; k < 2; k++) {
    const DmdObj *d = (const DmdObj *)w->b[B_DEM + k];
    const Ph3DemObj *p3 = (const Ph3DemObj *)w->b[B_P3 + k];
    const EvObj *ce = (const EvObj *)w->b[B_CE + k];
    f = fnv_upd(f, &d->state_, 4); f = fnv_upd(f, &d->phase_, 4); f = fnv_upd(f, &p3->verify_, 4);
    f = fnv_upd(f, &ce->delayedRetrainRequest_, 4); f = fnv_upd(f, &ce->delayedRetrainEnable_, 4);
  }
  f = fnv_upd(f, w->b[B_MPAR], 8);
  snprintf(out, cap, "|h=%08x f=%08x lvl=%u", h, f, dsplibs_debug_level);
}

/* ------------------------------------------------------------------------------------------------ the callee environment: record, effect, poke (the impl's recorders and the model call the same functions) */
static const char *nm47(int id) {
  static char b[40][24];
  static int n;
  char *o = b[n++ % 40];
  if (id >= 0 && id < g_nobj) return g_idname[id];
  if (id == ID_NULL) return "null";
  if (id == ID_WILD) return "wild";
  snprintf(o, 24, "A%d+%d", (id - 100) / 4096, (id - 100) % 4096);
  return o;
}
#define T_(p) nm47(tok_of(p))
#define ENVREC(w, c, l, ...) do { rec_add((l), (w) == &g_mw, 'c', __VA_ARGS__); } while (0)
static unsigned rd32(W *w, const void *p) { uint32_t v; if (!p) return 0xdeadbeefu; memcpy(&v, P(w, p), 4); return v; }
static uint32_t fill_word(uint32_t r) {   /* sample bit patterns: normal, denormal, zero, inf, NaN (quiet and signalling), huge */
  switch (r & 15u) {
    case 0: return 0;
    case 1: return 0x80000000u;
    case 2: return 0x7f800000u | (r & 0x80000000u);
    case 3: return 0x7fc00000u | (r >> 8 & 0x3fffffu);
    case 4: return 0x7f800001u | (r >> 8 & 0x3fffffu);      /* signalling NaN */
    case 5: return (r >> 8 & 0x7fffffu) | (r & 0x80000000u); /* denormal */
    case 6: return 0x7f000000u | (r >> 8 & 0xfffffu);
    default: return r;
  }
}

static void env_ed(W *w, Cur *c, Log *l, char kind, const char *s) { rec_add(l, w == &g_mw, kind, "%s", s); poke(w, c); }
static void env_prog90(W *w, Cur *c, Log *l, void *self, int *bits, unsigned *nb, float *in, unsigned n) {
  uint32_t r = sres(c), q = sres(c);
  DmdObj *d;
  ENVREC(w, c, l, "v90.progress(this=%s bits=%s nb=%s in=%s n=%u nbv=%u)", T_(self), T_(bits), T_(nb), T_(in), n, rd32(w, nb));
  d = (DmdObj *)P(w, OBJ(w)->v90Dem_);
  d->state_ = pick_qccode(r);
  if (q % 4u == 0) d->phase_ = (q & 8u) ? 4u : q >> 4;
  if (nb) { uint32_t v = q >> 3; memcpy(P(w, nb), &v, 4); }
  poke(w, c);
}
static void env_obj1(W *w, Cur *c, Log *l, const char *nm, void *self) { ENVREC(w, c, l, "%s(this=%s)", nm, T_(self)); poke(w, c); }
static void env_obj1u(W *w, Cur *c, Log *l, const char *nm, void *self, unsigned a) { ENVREC(w, c, l, "%s(this=%s %u)", nm, T_(self), a); poke(w, c); }
static void env_enterp3(W *w, Cur *c, Log *l, void *self) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "dem.enterPhase3(this=%s)", T_(self));
  if (r % 3u == 0) ((DmdObj *)P(w, OBJ(w)->v90Dem_))->state_ = pick_qccode(r >> 4);
  poke(w, c);
}
static void env_sinegen(W *w, Cur *c, Log *l, void *self, float *out, unsigned long n) {
  uint32_t q = sres(c);
  unsigned i;
  ENVREC(w, c, l, "sine.generate(this=%s out=%s n=%lu)", T_(self), T_(out), n);
  for (i = 0; i < n && i < BUF; i++) { uint32_t b = fill_word(q * 2246822519u + i * 3266489917u); memcpy((float *)P(w, out) + i, &b, 4); }
  poke(w, c);
}
static void env_dil(W *w, Cur *c, Log *l, void *d, short *bits, short *len) {
  uint32_t r = sres(c);
  short v = (short)((r & 3u) ? (r >> 8) % 1801u : (r >> 8));
  ENVREC(w, c, l, "DILdescriptorPacker(d=%s bits=%s len=%s)", T_(d), T_(bits), T_(len));
  memcpy(P(w, len), &v, 2);
  poke(w, c);
}
static double *env_probe(W *w, Cur *c, Log *l, void *ctx) { ENVREC(w, c, l, "V34XF.GetProbeResultsPtr(ctx=%s)", T_(ctx)); poke(w, c); return (double *)g_iw.b[B_PROBE]; }
static int env_rtd(W *w, Cur *c, Log *l, void *ctx) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "V34XF.GetRTD(ctx=%s)", T_(ctx));
  poke(w, c);
  return (int)((r & 3u) ? (uint32_t)(int32_t)(int16_t)(r >> 8) | (r & 0x10000u ? 0x12340000u : 0u) : r);   /* a short in ax, garbage above it */
}
static int *env_info0(W *w, Cur *c, Log *l, void *ctx) { ENVREC(w, c, l, "V34XF.GetInfo0BitsPtr(ctx=%s)", T_(ctx)); poke(w, c); return (int *)g_iw.b[B_INFO0]; }

/* ---- the constructor / destructor scenario: the sub-object constructors fill their region of the object (by member names: the same random stream on i386 and x86-64) */
static void fill_range(W *w, const char *first, const char *last, uint32_t seed) {
  PcmFlo47Obj *o = OBJ(w);
  unsigned a = 0, b = 0, i, j;
  uint32_t s = seed * 2654435761u + 12345u;
  for (i = 0; i < (unsigned)NF_PcmFlo47Obj; i++) { if (!strcmp(f_PcmFlo47Obj[i].name, first)) a = i; if (!strcmp(f_PcmFlo47Obj[i].name, last)) b = i; }
  for (i = a; i <= b; i++) {
    const HFld *f = &f_PcmFlo47Obj[i];
    unsigned n = f->count ? f->count : 1, es = (f->kind == 'b' || f->kind == 'x') ? 1 : f->kind == 'h' ? 2 : f->kind == 'd' ? 8 : 4;
    uint8_t *q = (uint8_t *)o + f->off;
    if (f->kind == 'p' || f->kind == 'o') {
      for (j = 0; j < n; j++) { void *v; s = s * 1664525u + 1013904223u; v = (void *)(uintptr_t)((s & 0x3ffffff0u) | 1u); memcpy(q + j * sizeof v, &v, sizeof v); }
      continue;
    }
    for (j = 0; j < n * es; j += 4) { s = s * 1664525u + 1013904223u; memcpy(q + j, &s, n * es - j < 4 ? n * es - j : 4); }
  }
}
static uint32_t fbits(float f) { uint32_t b; memcpy(&b, &f, 4); return b; }
static void env_c_v90(W *w, Cur *c, Log *l, void *self, unsigned side, void *par, void *dil, unsigned blk, unsigned mode, unsigned sess) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "V90Modem(this=%s side=%u params=%s dil=%s blk=%u mode=%u session=%u)", T_(self), side, T_(par), T_(dil), blk, mode, sess);
  fill_range(w, "v90Mod_", "v90Side_", r);
  poke(w, c);
}
static void env_c_v92(W *w, Cur *c, Log *l, void *self, unsigned side, void *par, unsigned blk, void *dil, unsigned mode) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "V92Modem(this=%s side=%u params=%s blk=%u dil=%s mode=%u)", T_(self), side, T_(par), blk, T_(dil), mode);
  fill_range(w, "v92Mod_", "v92Side_", r);
  OBJ(w)->v92Params_ = g_iw.b[B_V92P];
  poke(w, c);
}
static void env_c_ec(W *w, Cur *c, Log *l, void *self, void *par, unsigned a, unsigned b) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "V92EchoCanceller(this=%s params=%s %u %u)", T_(self), T_(par), a, b);
  fill_range(w, "ecParams_", "ecDelay_", r);
  poke(w, c);
}
static void env_c_ans(W *w, Cur *c, Log *l, void *self, unsigned a1, unsigned a2, float f3, unsigned a4, float f5, unsigned a6, unsigned a7, unsigned a8) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "ANSamToneDetector(this=%s %u %u %08x %u %08x %u %u %u)", T_(self), a1, a2, fbits(f3), a4, fbits(f5), a6, a7, a8);
  fill_range(w, "ansIir_", "ansDetected_", r);
  poke(w, c);
}
static void env_c_sine(W *w, Cur *c, Log *l, void *self, float a, float b, float d, float e) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "SineWave(this=%s %08x %08x %08x %08x)", T_(self), fbits(a), fbits(b), fbits(d), fbits(e));
  fill_range(w, "sineAmp_", "sineFs_", r);
  poke(w, c);
}
static void env_c_iir(W *w, Cur *c, Log *l, void *self, unsigned nd, unsigned nn, const double *den, const double *num, unsigned extra) {
  uint32_t r = sres(c);
  char t[300];
  int i;
  uint64_t x;
  size_t n = 0;
  for (i = 0; i < 5; i++) { memcpy(&x, den + i, 8); n += (size_t)snprintf(t + n, sizeof t - n, "%016llx ", (unsigned long long)x); }
  n += (size_t)snprintf(t + n, sizeof t - n, "/ ");
  for (i = 0; i < 5; i++) { memcpy(&x, num + i, 8); n += (size_t)snprintf(t + n, sizeof t - n, "%016llx ", (unsigned long long)x); }
  ENVREC(w, c, l, "GenericIIR(this=%s %u %u den,num=%s extra=%u)", T_(self), nd, nn, t, extra);
  fill_range(w, "iirDen_", "iirY_", r);
  poke(w, c);
}

/* ------------------------------------------------------------------------------------------------ the recorders (the blob's callees, weakened in the copy of dsplibs.o) */
extern void rc_prog90(void *, int *, unsigned *, float *, unsigned) L("_ZN8V90Modem8progressEPiRjPfj");
extern void rc_sessf(void *, unsigned) L("_ZN8V90Modem14setSessionFlagEj");
extern void rc_v90reset(void *, unsigned) L("_ZN8V90Modem5resetEj");
extern void rc_v92reset(void *) L("_ZN8V92Modem5resetEv");
extern void rc_ecreset(void *) L("_ZN16V92EchoCanceller5resetEv");
extern void rc_parinit(void *) L("_ZN13V90Parameters4initEv");
extern void rc_parinits(void *) L("_ZN13V90Parameters11initSessionEv");
extern void rc_par92(void *) L("_ZN13V92Parameters4initEv");
extern void rc_reinit(void *) L("_ZN14V90Demodulator6reInitEv");
extern void rc_enterp3(void *) L("_ZN14V90Demodulator11enterPhase3Ev");
extern void rc_waitans(void *) L("_ZN20V90Phase3Demodulator22enterWaitForANSpcmDropEv");
extern void rc_sinegen(void *, float *, unsigned long) L("_ZN8SineWaveIffE8generateEPfm");
extern void rc_dil(void *, short *, short *) L("DILdescriptorPacker");
extern double *rc_probe(void *) L("V34XF_GetProbeResultsPtr");
extern int rc_rtd(void *) L("V34XF_GetRTD");
extern int *rc_info0(void *) L("V34XF_GetInfo0BitsPtr");
extern void rc_c_v90(void *, unsigned, void *, void *, unsigned, unsigned, unsigned) L("_ZN8V90ModemC1E12V90ModemSideP19_tagModemParametersP19tagV90DILdescriptorj20V90ComputationalModej");
extern void rc_c_v92(void *, unsigned, void *, unsigned, void *, unsigned) L("_ZN8V92ModemC1E12V92ModemSideP19_tagModemParametersjP19tagV90DILdescriptor20V92ComputationalMode");
extern void rc_c_ec(void *, void *, unsigned, unsigned) L("_ZN16V92EchoCancellerC1EP13V92Parametersjj");
extern void rc_c_ans(void *, unsigned, unsigned, float, unsigned, float, unsigned, unsigned, unsigned) L("_ZN17ANSamToneDetectorC1Ejjfjfjjj");
extern void rc_c_sine(void *, float, float, float, float) L("_ZN8SineWaveIffEC1Effff");
extern void rc_c_iir(void *, unsigned, unsigned, double *, double *, unsigned) L("_ZN10GenericIIRIfdEC1EjjPdS1_j");
extern void rc_d_v90(void *) L("_ZN8V90ModemD1Ev");
extern void rc_d_v92(void *) L("_ZN8V92ModemD1Ev");
extern void rc_d_ec(void *) L("_ZN16V92EchoCancellerD1Ev");
extern void rc_d_ans(void *) L("_ZN17ANSamToneDetectorD1Ev");
extern void rc_d_sine(void *) L("_ZN8SineWaveIffED1Ev");
extern void rc_d_iir(void *) L("_ZN10GenericIIRIfdED1Ev");

void rc_prog90(void *t, int *b, unsigned *nb, float *in, unsigned n) { env_prog90(IW, IC, IL, t, b, nb, in, n); }
void rc_sessf(void *t, unsigned f) { env_obj1u(IW, IC, IL, "v90.setSessionFlag", t, f); }
void rc_v90reset(void *t, unsigned a) { env_obj1u(IW, IC, IL, "v90.reset", t, a); }
void rc_v92reset(void *t) { env_obj1(IW, IC, IL, "v92.reset", t); }
void rc_ecreset(void *t) { env_obj1(IW, IC, IL, "ec.reset", t); }
void rc_parinit(void *t) { env_obj1(IW, IC, IL, "V90Parameters.init", t); }
void rc_parinits(void *t) { env_obj1(IW, IC, IL, "V90Parameters.initSession", t); }
void rc_par92(void *t) { env_obj1(IW, IC, IL, "V92Parameters.init", t); }
void rc_reinit(void *t) { env_obj1(IW, IC, IL, "dem.reInit", t); }
void rc_enterp3(void *t) { env_enterp3(IW, IC, IL, t); }
void rc_waitans(void *t) { env_obj1(IW, IC, IL, "p3.enterWaitForANSpcmDrop", t); }
void rc_sinegen(void *t, float *out, unsigned long n) { env_sinegen(IW, IC, IL, t, out, n); }
void rc_dil(void *d, short *b, short *l) { env_dil(IW, IC, IL, d, b, l); }
double *rc_probe(void *c) { return env_probe(IW, IC, IL, c); }
int rc_rtd(void *c) { return (int)(int16_t)env_rtd(IW, IC, IL, c); } /* the library definition returns an already sign-extended 16 bit value */
int *rc_info0(void *c) { return env_info0(IW, IC, IL, c); }
void rc_c_v90(void *t, unsigned s, void *p, void *d, unsigned b, unsigned m, unsigned se) { env_c_v90(IW, IC, IL, t, s, p, d, b, m, se); }
void rc_c_v92(void *t, unsigned s, void *p, unsigned b, void *d, unsigned m) { env_c_v92(IW, IC, IL, t, s, p, b, d, m); }
void rc_c_ec(void *t, void *p, unsigned a, unsigned b) { env_c_ec(IW, IC, IL, t, p, a, b); }
void rc_c_ans(void *t, unsigned a1, unsigned a2, float f3, unsigned a4, float f5, unsigned a6, unsigned a7, unsigned a8) { env_c_ans(IW, IC, IL, t, a1, a2, f3, a4, f5, a6, a7, a8); }
void rc_c_sine(void *t, float a, float b, float d, float e) { env_c_sine(IW, IC, IL, t, a, b, d, e); }
void rc_c_iir(void *t, unsigned nd, unsigned nn, double *den, double *num, unsigned extra) { env_c_iir(IW, IC, IL, t, nd, nn, den, num, extra); }
void rc_d_v90(void *t) { env_obj1(IW, IC, IL, "~V90Modem", t); }
void rc_d_v92(void *t) { env_obj1(IW, IC, IL, "~V92Modem", t); }
void rc_d_ec(void *t) { env_obj1(IW, IC, IL, "~V92EchoCanceller", t); }
void rc_d_ans(void *t) { env_obj1(IW, IC, IL, "~ANSamToneDetector", t); }
void rc_d_sine(void *t) { env_obj1(IW, IC, IL, "~SineWave", t); }
void rc_d_iir(void *t) { env_obj1(IW, IC, IL, "~GenericIIR", t); }
void edprintf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  env_ed(IW, IC, IL, 'e', buf);
}
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  env_ed(IW, IC, IL, 'd', buf);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ the independent model (from porting/decompiled/dp/p47_vpcmflo_b.txt) */
static void mpf(char kind, const char *fmt, ...) {
  char b[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  env_ed(MW, MC, ML, kind, b);
}
#define MDBG(...) do { if (DBG()) mpf('d', __VA_ARGS__); } while (0)
static PcmFlo47Obj *mo(void) { return OBJ(MW); }
static DmdObj *mdem(void) { return (DmdObj *)P(MW, mo()->v90Dem_); }
static V90ParObj *mpar(void) { return (V90ParObj *)P(MW, mo()->v90Params_); }
static P47V90InfoObj *mi90(void) { return (P47V90InfoObj *)P(MW, mo()->v90Info_); }
static P47V92InfoObj *mi92(void) { return (P47V92InfoObj *)P(MW, mo()->v92Info_); }
static void *ip(int x) { return g_obj[x]; }
static const uint8_t k_f0[6] = { 1, 0, 1, 1, 1, 0 }, k_f1[6] = { 1, 0, 1, 1, 1, 1 }, k_fc[6] = { 1, 1, 1, 1, 1, 1 };
static int m_chop32(long double v) { if (v != v || v >= 2147483648.0L || v <= -2147483649.0L) return INT_MIN; return (int)v; }

static void m_idle(void) {   /* internalReset / enterPhase3 / externalReset (after the print) */
  PcmFlo47Obj *o = mo();
  o->cpnotLen_ = 0; o->cpLen_ = 0; o->bitPtr_ = 0;
  o->trainConstel_ = 0; o->rrnConstel_ = 0; o->localRrn_ = 0; o->dropToV34_ = 0; o->clearDown_ = 0;
  memcpy(o->v34Flags_, k_f0, 6);
  o->jaFlag_ = 0; o->cpFlag_ = 0; o->cpnotFlag_ = 0; o->cpnotCopied_ = 0;
  o->nofBitsPhase4_ = 2; o->cpSeqCnt_ = 0; o->minTxSeq_ = 1;
}
static void m_flagset(int which, unsigned v) {
  PcmFlo47Obj *o = mo();
  if (which == 0) o->jaFlag_ = (uint8_t)v; else if (which == 1) o->cpFlag_ = (uint8_t)v; else o->cpnotFlag_ = (uint8_t)v;
  if (DBG()) CV(G_SET, "flag setter: debug print"); else CV(G_SET, "flag setter: no print");
  if (which == 0) CV(G_SET, "setTerminateJaFlag"); else if (which == 1) CV(G_SET, "setTerminateCpFlag"); else CV(G_SET, "setTerminateCpNotFlag");
  if (which == 0) { MDBG("Ja Flag set to %d\r\n", (int)(uint8_t)v); }
  else if (which == 1) { MDBG("CP Flag set to %d\r\n", (int)(uint8_t)v); }
  else { MDBG("CPnot Flag set to %d\r\n", (int)(uint8_t)v); }
}
static int m_getja(short *out) {
  PcmFlo47Obj *o = mo();
  uint16_t *po = (uint16_t *)P(MW, out);
  int ret = 0;
  if (o->cpnotLen_ == 0) CV(G_SET, "getJa: no CPnot frame");
  if (o->cpnotLen_ != 0) {
    CV(G_SET, "getJa: two words");
    uint16_t first = CPNF(o)[o->bitPtr_];
    o->bitPtr_ = (uint16_t)(o->bitPtr_ + 1u);
    *po = first;
    int16_t second = (int16_t)CPNF(o)[o->bitPtr_];
    o->bitPtr_ = (uint16_t)(o->bitPtr_ + 1u);
    *po = (uint16_t)(((uint32_t)(int32_t)second * 2u) | (uint32_t)*po);
    if ((int)o->bitPtr_ == (int)(int16_t)o->cpnotLen_) { CV(G_SET, "getJa: frame end (pointer wraps)"); o->bitPtr_ = 0; }
  }
  if (o->jaFlag_ != 0) { CV(G_SET, "getJa: Ja flag set"); ret = 1; o->bitPtr_ = 0; } else CV(G_SET, "getJa: Ja flag clear");
  return ret;
}
static int m_getcp(short *out) {
  PcmFlo47Obj *o = mo();
  uint16_t *po = (uint16_t *)P(MW, out);
  int ret = 0;
  unsigned c;
  if (o->cpLen_ == 0) { CV(G_SET, "getCp: no CP"); return 0; }
  *po = 0;
  if (o->nofBitsPhase4_ == 0) CV(G_SET, "getCp: nof bits 0"); else if (o->nofBitsPhase4_ > 32) CV(G_SET, "getCp: shift count wraps (nof bits > 32)"); else CV(G_SET, "getCp: bits gathered");
  for (c = 0; c < o->nofBitsPhase4_; c++) {
    int16_t v = (int16_t)CPF(o)[o->bitPtr_];
    o->bitPtr_ = (uint16_t)(o->bitPtr_ + 1u);
    *po = (uint16_t)(((uint32_t)(int32_t)v << (c % 32u)) | (uint32_t)*po);
  }
  if ((int)o->bitPtr_ != (int)(int16_t)o->cpLen_) { CV(G_SET, "getCp: CP not complete"); return 0; }
  CV(G_SET, "getCp: CP complete");
  {
    EvObj *ce = (EvObj *)P(MW, mdem()->ce_);
    int32_t req = ce->delayedRetrainRequest_;
    o->cpSeqCnt_ = (uint16_t)(o->cpSeqCnt_ + 1u);
    ce->delayedRetrainEnable_ = req;
  }
  MDBG("End of CP #%d tx.... (terminateCp=%d, terminateCpNot=%d)\n", (int)o->cpSeqCnt_, (int)o->cpFlag_, (int)o->cpnotFlag_);
  if (o->cpnotFlag_ != 0) {
    ret = 1;
    CV(G_SET, "getCp: CPnot flag (terminate)");
    MDBG("GetV90CpBits: Indicating CP termination !!!!\r\n");
  } else if (o->cpFlag_ != 0 && o->cpnotCopied_ == 0 && o->cpSeqCnt_ >= o->minTxSeq_) {
    int i, n;
    CV(G_SET, "getCp: move to CPnot (copy)");
    MDBG("OK Time to move to CPNot TX...\r\n");
    n = (int)(int16_t)o->cpnotLen_;
    for (i = 0; i < n; i++) CPF(o)[i] = CPNF(o)[i];
    o->cpLen_ = o->cpnotLen_;
    o->cpnotCopied_ = 1;
    o->cpSeqCnt_ = 0;
  }
  else if (o->cpFlag_ == 0) CV(G_SET, "getCp: no CP flag");
  else if (o->cpnotCopied_ != 0) CV(G_SET, "getCp: CPnot already copied");
  else CV(G_SET, "getCp: too few sequences");
  o->bitPtr_ = 0;
  return ret;
}
static void m_sess(int s) {
  PcmFlo47Obj *o = mo();
  if (s != 0) CV(G_SES, "sess: V.92"); else CV(G_SES, "sess: V.90");
  mpf('e', "VPcmFloModem: setting PCM session to V.%d\n", s != 0 ? 92 : 90);
  o->v92Session_ = s != 0;
  mi92()->v92CapLocal_ = (uint8_t)s;
  env_obj1u(MW, MC, ML, "v90.setSessionFlag", ip(X_V90), (unsigned)s);
}
static void m_ereset(void) {
  PcmFlo47Obj *o = mo();
  memcpy(o->v34Flags_, k_f0, 6);
  env_obj1(MW, MC, ML, "V90Parameters.initSession", o->v90Params_);
  env_obj1(MW, MC, ML, "V90Parameters.init", o->v90Params_);
  env_obj1(MW, MC, ML, "V92Parameters.init", o->v92Params_);
  MDBG("V90_V34_Main: reinitializing parameters.\r\n");
  m_idle();
  if (o->flag6120_ != 0) { CV(G_SES, "ereset: V.92 side (reInit)"); env_obj1(MW, MC, ML, "dem.reInit", o->v90Dem_); } else CV(G_SES, "ereset: V.90 only");
  o->state_ = 0; o->flag6119_ = 0;
  o->qcState_ = 0; o->qcFlag_ = 0; o->qcCount_ = 0; o->qcStatus_ = 0;
  MDBG("V90_V34_Main: external reset called.\r\n");
}
static void m_ph2(int *info0, int rtd) {
  PcmFlo47Obj *o = mo();
  const int *pw = (const int *)P(MW, info0);
  int64_t sum = 0;
  int i, go;
  for (i = 0; i < 5; i++) sum += (int64_t)pw[33 + i] * (int64_t)(1 << i);
  mi90()->maxTx_ = (uint8_t)sum;
  if (o->flag6120_ != 0) { CV(G_SES, "ph2: flag6120 set"); mi90()->txPoint_ = pw[38] != 0; mi90()->pcmType_ = pw[39] != 0; } else CV(G_SES, "ph2: flag6120 clear");
  mi92()->maxTx_ = mi90()->maxTx_;
  mi92()->txPoint_ = mi90()->txPoint_;
  mi92()->pcmType_ = mi90()->pcmType_;
  if (o->flag6120_ != 0) { mi92()->shortPhase2Remote_ = (uint8_t)pw[26]; mi92()->v92CapRemote_ = (uint8_t)pw[27]; }
  else { mi92()->shortPhase2Remote_ = (uint8_t)pw[27]; mi92()->v92CapRemote_ = (uint8_t)pw[26]; }
  go = DBG();
  if (!go) CV(G_SES, "ph2: no debug");
  if (go) { CV(G_SES, "ph2: debug"); mpf('d', "P2 REPORT : A_LAW or MU_LAW %d ( 0 = Mu, 1 = A)\r\n", mi90()->pcmType_ == 1 ? 1 : 0); go = DBG(); }
  if (go) { mpf('d', "P2 REPORT : Tx power measurment point = %d ( 0 = digital modem terminal, 1 = codec output)\r\n", pw[38]); go = DBG(); }
  if (go) { mpf('d', "P2 REPORT : Max tx power = %d\r\n", (int)mi90()->maxTx_); go = DBG(); }
  if (go) { mpf('d', "P2 REPORT : short phase2: local=%d , remote=%d\r\n", (int)mi92()->shortPhase2Local_, (int)mi92()->shortPhase2Remote_); go = DBG(); }
  if (go) CV(G_SES, "ph2: all five prints");
  if (go) mpf('d', "P2 REPORT : V92 capabilities: local=%d , remote=%d , selected=%d\r\n", (int)mi92()->v92CapLocal_, (int)mi92()->v92CapRemote_, o->v92Session_ ? 92 : 90);
  mi90()->arrA_ = ip(X_UA); mi90()->arrB_ = ip(X_UB); mi90()->l2_ = ip(X_L2); mi90()->arrD_ = ip(X_UD);
  mi90()->rtd_ = (uint32_t)rtd; mi92()->rtd_ = (uint32_t)rtd;
  mi92()->arrA_ = mi90()->arrA_; mi92()->arrB_ = mi90()->arrB_; mi92()->l2_ = mi90()->l2_; mi92()->arrD_ = mi90()->arrD_;
  if (mi90()->pcmType_ == 1) CV(G_SES, "ph2: A-law");
  m_sess((int)o->v92Session_);
}
static long double m_log10(long double x) {
  static const long double lg2 = 0.30102999566398119521373889472449302677L;
  long double r;
  __asm__("fyl2x" : "=t"(r) : "0"(x), "u"(lg2) : "st(1)");
  return r;
}
static int m_uinfo(short x) {
  PcmFlo47Obj *o = mo();
  const double *pr = (const double *)P(MW, env_probe(MW, MC, ML, o->ctx_));
  int i, k, rtd, *info0;
  memset(o->uinfoA_, 0, sizeof o->uinfoA_); memset(o->uinfoB_, 0, sizeof o->uinfoB_); memset(o->l2_, 0, sizeof o->l2_); memset(o->uinfoD_, 0, sizeof o->uinfoD_);
  if (x == 0) {
    CV(G_UIN, "uinfo: probing levels");
    k = 0;
    for (i = 0; i < 25; i++) {
      volatile float a, b;
      long double lv;
      a = (float)((long double)pr[i] * 0x1p-14L);
      b = (float)m_log10((long double)a);
      lv = (long double)b * 10.0L + 60.0L;
      if (i != 5 && i != 7 && i != 11 && i != 15) { volatile float *d = &o->l2_[k++]; *d = (float)lv; }
    }
    MDBG("Final L2 Probing signal:\r\n");
    for (i = 15; i <= 20; i++) {
      if (!DBG()) continue;
      {
        long double v = (long double)o->l2_[14] - (long double)o->l2_[i];
        long double av = v < 0 ? -v : v;
        int ipart = m_chop32(av), whole = m_chop32(v), fr = m_chop32((v - (long double)whole) * 100000.0L);
        char sg = (v > 0.0L || v != v) ? '+' : '-';
        if (v != v) CV(G_UIN, "uinfo: print NaN difference"); else if (v > 0.0L) CV(G_UIN, "uinfo: print positive difference"); else if (v < 0.0L) CV(G_UIN, "uinfo: print negative difference"); else CV(G_UIN, "uinfo: print zero difference");
        if (fr < 0) CV(G_UIN, "uinfo: print negative fraction");
        if (ipart == INT_MIN) CV(G_UIN, "uinfo: print integer part indefinite");
        uint32_t ufr = fr < 0 ? 0u - (uint32_t)fr : (uint32_t)fr;
        mpf('d', "L2[%d]    = %c%d.%05d\r\n", i, sg, ipart, (int)ufr);
      }
    }
  }
  if (x != 0) CV(G_UIN, "uinfo: x != 0 (levels skipped)");
  rtd = (int)(int16_t)env_rtd(MW, MC, ML, o->ctx_);
  info0 = env_info0(MW, MC, ML, o->ctx_);
  m_ph2(info0, rtd);
  if (o->dropToV34_ == 0) {
    int u = (int16_t)mpar()->phase2InfoUinfo;
    if (u != 0) { CV(G_UIN, "uinfo: returns the info value"); return u; }
    CV(G_UIN, "uinfo: info value 0 (flags)");
  } else CV(G_UIN, "uinfo: drop to V34 (flags)");
  memcpy(o->v34Flags_, k_f1, 6);
  return 0;
}
static void m_reset3(void) {
  PcmFlo47Obj *o = mo();
  CV(G_SES, "reset3");
  o->constelCtr_ = 0;
  env_obj1(MW, MC, ML, "V90Parameters.init", o->v90Params_);
  env_obj1u(MW, MC, ML, "v90.setSessionFlag", ip(X_V90), o->v92Session_);
  env_obj1u(MW, MC, ML, "v90.reset", ip(X_V90), 0);
  env_obj1(MW, MC, ML, "v92.reset", ip(X_V92));
  env_obj1(MW, MC, ML, "ec.reset", ip(X_EC));
  o->echoCnt_ = 0; o->echoMode_ = 0;
  { uint8_t *b = (uint8_t *)P(MW, mpar()->params) + 3; *b &= 0xfb; }
  memset(&o->sinePhase_, 0, 4);
  o->state_ = 1;
}
static void m_enter3(void) {
  PcmFlo47Obj *o = mo();
  m_idle();
  if (DBG()) CV(G_SES, "enter3: debug print"); else CV(G_SES, "enter3: no print");
  MDBG("VPcmFloModem: enterPhase3 called.\r\n");
  env_enterp3(MW, MC, ML, o->v90Dem_);
  env_dil(MW, MC, ML, ip(X_DIL), (short *)ip(X_CPNF), (short *)ip(X_CPNL));
  mpf('e', "VPcmFloModem: enterPhase3: Ja length = %d\r\n", (int)(int16_t)o->cpnotLen_);
}
static int m_const(void *out, unsigned long max) {
  PcmFlo47Obj *o = mo();
  DmdObj *d;
  int32_t *po;
  unsigned n, fp, i;
  const float *buf;
  if (o->v92Session_ != 0 && o->flag6120_ == 0) { CV(G_UIN, "const: gated off"); return 0; }
  d = mdem();
  po = (int32_t *)P(MW, out);
  n = d->eqCnt_; fp = d->framePos_; buf = (const float *)P(MW, d->eqBuf_);
  if (n > max) { CV(G_UIN, "const: clamped by max"); n = (unsigned)max; }
  if (d->phase_ == 1) CV(G_UIN, "const: phase 1"); else CV(G_UIN, "const: other phase");
  if (o->constelCtr_ < 0) CV(G_UIN, "const: negative counter");
  if (n == 0) CV(G_UIN, "const: empty");
  for (i = 0; i < n; i++) {
    int64_t c = (int32_t)o->constelCtr_, q, r;
    long double prod = 1.7 * (long double)buf[i];
    int re;
    if (d->phase_ == 1) { q = c / 5; r = q % 750; re = (int)(35 * r - 14000); }
    else { int64_t t = (int64_t)((fp + i) % 6u); q = c / 15; r = q % 100 + 140 * t; re = (int)(35 * r - 14000); }
    o->constelCtr_ = (int32_t)(uint32_t)((uint32_t)c + 1u);
    po[2 * i] = re;
    if (m_chop32(prod) == INT_MIN) CV(G_UIN, "const: imaginary part indefinite");
    po[2 * i + 1] = m_chop32(prod);
  }
  return (int)n;
}
static int m_coefs(void *out, unsigned long max, int dfe) {
  PcmFlo47Obj *o = mo();
  EquObj *e;
  int32_t *po;
  unsigned n, i;
  const float *cf;
  if (o->v92Session_ != 0 && o->flag6120_ == 0) { CV(G_UIN, "coefs: gated off"); return 0; }
  if (dfe) CV(G_UIN, "coefs: DFE"); else CV(G_UIN, "coefs: linear");
  e = (EquObj *)P(MW, mdem()->eq_);
  po = (int32_t *)P(MW, out);
  n = dfe ? e->dfeLen_ : e->linLen_;
  if (n > max) { CV(G_UIN, "coefs: clamped by max"); n = (unsigned)max; }
  cf = (const float *)P(MW, dfe ? e->dfeCoef_ : e->linCoef_);
  for (i = 0; i < n; i++) { po[2 * i] = 0; po[2 * i + 1] = m_chop32((long double)cf[i] * 10000.0L); if (po[2 * i + 1] == INT_MIN) CV(G_UIN, "coefs: product indefinite"); }
  return (int)n;
}
static int m_qc(float *in, float *out, unsigned n, int *bits, int *nb, int *bits2, int *nb2) {
  PcmFlo47Obj *o = mo();
  int ret = 0, tone = 0, zero = 0, count;
  unsigned st;
  uint32_t code;
  (void)bits2;
  env_prog90(MW, MC, ML, ip(X_V90), bits, (unsigned *)nb, in, n);
  code = mdem()->state_;
  o->qcCount_ = (int)((uint32_t)o->qcCount_ + n);
  count = o->qcCount_;
  st = 0;
  if (code == 0x3a) {
    Ph3DemObj *p3;
    CV(G_SES, "qc: ANSpcm demod over");
    MDBG("VPcmFloModem (QC LineVerify): ANSpcm demod over (after %d samples), start TONEq...\r\n", count);
    p3 = (Ph3DemObj *)P(MW, mdem()->p3_);
    o->qcStatus_ = p3->verify_ & 0xffffu;
    MDBG("VPcmFloModem (QC LineVerify): Channel verification status is: %d\r\n", (int)o->qcStatus_);
    o->qcState_ = 1; o->qcFlag_ = 0; o->qcCount_ = 0;
    env_obj1(MW, MC, ML, "p3.enterWaitForANSpcmDrop", mdem()->p3_);
    st = o->qcState_;
  } else if (code == 0x3b) {
    st = o->qcState_;
    if (st != 1) {
      Ph3DemObj *p3;
      CV(G_SES, "qc: ANSpcm drop, not tone state");
      MDBG("VPcmFloModem (QC LineVerify): got ANSpcm drop detection, with no verification completion status !!!\r\n");
      p3 = (Ph3DemObj *)P(MW, mdem()->p3_);
      o->qcStatus_ = p3->verify_ & 0xffffu;
      MDBG("VPcmFloModem (QC LineVerify): demodulator verification status is %d\r\n", (int)o->qcStatus_);
      o->qcCount_ = 0; o->qcState_ = 1; o->qcFlag_ = 1;
      tone = 1;
    } else if (count > 0x1df) {
      CV(G_SES, "qc: drop, tone state, over 0x1df samples");
      MDBG("VPcmFloModem (QC LineVerify): TONEq mod over (after %d samples), tx silence...\r\n", count);
      o->qcCount_ = -0x180; o->qcState_ = 2; st = 2;
    } else {
      CV(G_SES, "qc: drop, tone state, still below 50 ms");
      if (DBG()) { mpf('d', "VPcmFloModem (QC LineVerify): TONEq termination requested, still bellow 50mS (nof samples = %d)...\r\n", count); st = o->qcState_; }
      o->qcFlag_ = 1;
    }
  } else { CV(G_SES, "qc: other code"); st = o->qcState_; }
  if (tone || st == 1) {
    CV(G_SES, "qc: tone");
    env_sinegen(MW, MC, ML, ip(X_SINE), out, n);
    if (o->qcFlag_ != 0 && o->qcCount_ > 0x1df) {
      CV(G_SES, "qc: tone over after generation");
      MDBG("VPcmFloModem (QC LineVerify): TONEq mod over (after %d samples), tx silence...\r\n", o->qcCount_);
      o->qcCount_ = -0x180; o->qcState_ = 2;
    }
  } else {
    unsigned i;
    zero = 1;
    CV(G_SES, "qc: silence");
    for (i = 0; i < n; i++) { uint32_t z = 0; memcpy((float *)P(MW, out) + i, &z, 4); }
    if (st == 2 && o->qcCount_ < 0) CV(G_SES, "qc: silence run-in");
    if (st == 2 && o->qcCount_ >= 0) { CV(G_SES, "qc: silence over, return 1"); MDBG("VPcmFloModem (QC LineVerify): Silence after TONEq over, move to phase2...\r\n"); ret = 1; }
  }
  (void)zero;
  { int32_t z = 0; memcpy(P(MW, nb), &z, 4); memcpy(P(MW, nb2), &z, 4); }
  return ret;
}

/* the constructor / destructor models */
static const double k_iirDen[5] = { 1.0, -3.924251435170327, 5.775633188131097, -3.77844829349304, 0.9270672393213297 };
static const double k_iirNum[5] = { 0.962843309849182, -3.851350039011478, 5.777013458394471, -3.851350039011477, 0.9628433098491815 };
static void m_ctor(void *ctx, unsigned side, void *params, unsigned blk, unsigned m90, unsigned m92) {
  PcmFlo47Obj *o = mo();
  uint8_t *pb = (uint8_t *)P(MW, params);
  o->ctx_ = ctx;
  if (side == 1u) CV(G_CTOR, "ctor: analog side"); else CV(G_CTOR, "ctor: other side");
  if (pb[3] & 4) CV(G_CTOR, "ctor: SAS bit was set"); else CV(G_CTOR, "ctor: SAS bit was clear");
  env_c_v90(MW, MC, ML, ip(X_V90), side, params, ip(X_DIL), blk, m90, 1u);
  env_c_v92(MW, MC, ML, ip(X_V92), side == 1u, params, blk, ip(X_DIL), m92);
  env_c_ec(MW, MC, ML, ip(X_EC), o->v92Params_, blk, 199u);
  env_c_ans(MW, MC, ML, ip(X_ANS), 0x1770u, 0x1c2u, 250000.0f, 1u, 0.58f, 9600u, 0x32u, 0x63u);
  env_c_sine(MW, MC, ML, ip(X_SINE), 4800.0f, 980.0f, 0.0f, 9600.0f);
  env_c_iir(MW, MC, ML, ip(X_IIR), 5u, 5u, k_iirDen, k_iirNum, 99u);
  memset(o->cpnotFrame_, 0, sizeof o->cpnotFrame_);
  o->cpnotLen_ = 0;
  memcpy(o->v34Flags_, k_fc, 6);
  o->constelCtr_ = 0;
  memset(o->cpFrame_, 0, sizeof o->cpFrame_);
  o->jaFlag_ = o->cpFlag_ = o->cpnotFlag_ = o->cpnotCopied_ = 0;
  o->nofBitsPhase4_ = 0;
  o->flag6119_ = 0;
  o->minTxSeq_ = 0; o->cpLen_ = 0; o->cpSeqCnt_ = 0;
  o->qcState_ = 0; o->qcFlag_ = 0; o->qcCount_ = 0; o->qcStatus_ = 0;
  pb[3] &= 0xfb;
  memset(o->ecOut_, 0, sizeof o->ecOut_);
  o->flag6120_ = side == 1u;
  o->v92Session_ = 0; o->state_ = 0; o->b7f5c_ = 0; o->echoMode_ = 0; o->echoCnt_ = 0;
}
static void m_dtor(void) {
  CV(G_CTOR, "dtor");
  env_obj1(MW, MC, ML, "~GenericIIR", ip(X_IIR));
  env_obj1(MW, MC, ML, "~SineWave", ip(X_SINE));
  env_obj1(MW, MC, ML, "~ANSamToneDetector", ip(X_ANS));
  env_obj1(MW, MC, ML, "~V92EchoCanceller", ip(X_EC));
  env_obj1(MW, MC, ML, "~V92Modem", ip(X_V92));
  env_obj1(MW, MC, ML, "~V90Modem", ip(X_V90));
}

/* ------------------------------------------------------------------------------------------------ world setup */
static void reg(int id, void *p, size_t n) { g_iw.b[id] = p; g_blkidx[id] = blk_add(bname[id], p, n); g_obj[id] = p; g_idname[id] = bname[id]; }
static void *newblk(int id, size_t n) { void *p = h_obj_new(n); reg(id, p, n); return p; }
static void fill_obj(uint8_t *t) {
  int i;
  for (i = 0; i < NF_PcmFlo47Obj; i++) {
    const HFld *f = &f_PcmFlo47Obj[i];
    unsigned n = f->count ? f->count : 1, es, tot, j;
    uint8_t *q = t + f->off;
    switch (f->kind) {
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'd': es = 8; break;
      case 'u': case 'i': case 'f': es = 4; break;
      case 'p': case 'o': {
        unsigned e;
        for (e = 0; e < n; e++) { void *v; uintptr_t r = h_rnd(); r = (r << 4) ^ (uintptr_t)h_rnd(); v = (void *)((r & 0x3ffffff0u) | 1u); memcpy(q + e * sizeof v, &v, sizeof v); }
        continue;
      }
      default: continue;
    }
    tot = n * es;
    for (j = 0; j < tot; j += 4) { uint32_t r = h_rnd(); memcpy(q + j, &r, tot - j < 4 ? tot - j : 4); }
  }
}
/* sizeof differs on x86-64: one h_rnd per block, the words come from a local generator (the random stream must not depend on sizeof) */
static void fill_blk(void *p, size_t n) { size_t j; uint32_t s = h_rnd(); for (j = 0; j + 4 <= n; j += 4) { s = s * 1664525u + 1013904223u; memcpy((uint8_t *)p + j, &s, 4); } }
static uint32_t wsel(void) { switch (h_rmod(8)) { case 0: return 0; case 1: return 1; case 2: return h_rnd(); case 3: return 0x100; case 4: return 0x80000000u; case 5: return h_rmod(16); default: return h_rnd(); } }
static unsigned cnt256(void) { switch (h_rmod(6)) { case 0: return 0; case 1: return 1; case 2: return BUF; case 3: return BUF - 1u - h_rmod(3); default: return h_rmod(BUF + 1u); } }
static uint32_t fw(void) { return fill_word(h_rnd()); }
/* a coefficient / sample float: bit pattern families (zero, tiny, normal near 1, big, huge, inf, NaN, signalling NaN) */
static uint32_t pick_float(void) {
  uint32_t r = h_rnd();
  switch (r % 12u) {
    case 0: return fill_word(h_rnd());
    case 1: return (uint32_t)0x3f800000u | (r >> 8 & 0x7fffffu);                       /* [1, 2) */
    case 2: return (uint32_t)0xbf800000u | (r >> 8 & 0x7fffffu);
    case 3: return (uint32_t)0x38000000u | (r >> 8 & 0x7fffffu) | (r & 0x80000000u);    /* ~ 2^-15 */
    case 4: return (uint32_t)0x4f000000u | (r >> 8 & 0x7fffffu) | (r & 0x80000000u);    /* ~ 2^31: the chop limit */
    case 5: return (uint32_t)0x46000000u | (r >> 8 & 0xfffffu) | (r & 0x80000000u);
    case 6: return (uint32_t)0x3e800000u | (r >> 8 & 0x7fffffu) | (r & 0x80000000u);    /* ~ 0.25 */
    default: return (uint32_t)(0x3f000000u + ((r >> 8) % 0x01800000u)) | (r & 0x80000000u);
  }
}
static void fill_floats(void *p, unsigned n) { unsigned i; for (i = 0; i < n; i++) { uint32_t b = pick_float(); memcpy((float *)p + i, &b, 4); } }
static double pick_double(void) {
  uint32_t r = h_rnd(), q = h_rnd();
  uint64_t b;
  switch (r % 14u) {
    case 0: return 0.0;
    case 1: return -0.0;
    case 2: b = 0x7ff8000000000000ull | q; break;                                 /* NaN */
    case 3: b = 0x7ff0000000000001ull | ((uint64_t)q << 20); break;               /* signalling NaN */
    case 4: b = 0x7ff0000000000000ull | ((uint64_t)(q & 1u) << 63); break;          /* +-inf */
    case 5: b = (uint64_t)q | ((uint64_t)(r & 0x800u) << 52); break;                /* denormal */
    case 6: return -(double)(q % 100000000u + 1u);
    case 7: return (double)(q % 1000u);                                             /* small integer: x 2^-14 < 1 and log10 < 0 */
    default: return (double)(q % 1000000000u + 1u) * (r & 0x1000u ? 1.0 : 1.0e-3);   /* the useful range */
  }
  { double d; memcpy(&d, &b, 8); return d; }
}
static void init_dem(DmdObj *d, int eq, int p3, int ce) {
  fill_blk(d, sizeof *d);
  d->state_ = pick_qccode(h_rnd());
  d->phase_ = h_rpct(40) ? 1u : h_rpct(70) ? h_rmod(6) : h_rnd();
  d->eq_ = g_iw.b[eq]; d->p3_ = g_iw.b[p3]; d->ce_ = g_iw.b[ce];
  d->eqBuf_ = g_iw.b[B_EQBUF];
  d->eqCnt_ = cnt256();
  d->framePos_ = h_rpct(50) ? h_rmod(12) : h_rnd();
}
static void init_par(V90ParObj *p) {
  fill_blk(p, sizeof *p);
  p->params = g_iw.b[B_MPAR];
  switch (h_rmod(8)) { case 0: case 1: case 2: p->phase2InfoUinfo = 0; break; case 3: p->phase2InfoUinfo = (int32_t)h_rmod(40); break; case 4: p->phase2InfoUinfo = (int32_t)0x10000 * (int32_t)(1 + h_rmod(3)); break;
    case 5: p->phase2InfoUinfo = (int32_t)0x8000; break; case 6: p->phase2InfoUinfo = -(int32_t)h_rmod(40); break; default: p->phase2InfoUinfo = (int32_t)h_rnd(); }
}
static uint8_t byte_pick(void) { return (uint8_t)(h_rpct(45) ? 0 : h_rpct(55) ? 1 : h_rnd()); }
static void new_world(int ctor) {
  uint8_t *raw;
  int k;
  size_t osz = sizeof(PcmFlo47Obj);
  PcmFlo47Obj *o;
  memset(&g_iw, 0, sizeof g_iw);
  blk_reset();
  g_nobj = NID;
  for (k = 0; k < NID; k++) { g_obj[k] = NULL; g_idname[k] = bname[k]; }
  raw = (uint8_t *)h_obj_new(osz + CAN);
  memset(raw + osz, 0x5a, CAN);
  g_can = raw + osz;
  fill_obj(raw);
  reg(B_OBJ, raw, osz);
  newblk(B_MPAR, 0x100);
  newblk(B_EQ, sizeof(EquObj)); newblk(B_EQ2, sizeof(EquObj));
  newblk(B_P3, sizeof(Ph3DemObj)); newblk(B_P3B, sizeof(Ph3DemObj));
  newblk(B_CE, sizeof(EvObj)); newblk(B_CE2, sizeof(EvObj));
  newblk(B_I90, sizeof(P47V90InfoObj)); newblk(B_I90B, sizeof(P47V90InfoObj));
  newblk(B_I92, sizeof(P47V92InfoObj)); newblk(B_I92B, sizeof(P47V92InfoObj));
  newblk(B_V92P, 0x40);
  newblk(B_LIN, BUF * 4); newblk(B_DFE, BUF * 4); newblk(B_EQBUF, BUF * 4);
  newblk(B_PROBE, 25 * 8); newblk(B_INFO0, 0x100);
  newblk(B_PAR, sizeof(V90ParObj)); newblk(B_PAR2, sizeof(V90ParObj));
  newblk(B_DEM, sizeof(DmdObj)); newblk(B_DEM2, sizeof(DmdObj));
  newblk(B_CTX, 0x40); newblk(B_CTX2, 0x40);
  newblk(B_IN, BUF * 4); newblk(B_OUT, BUF * 4); newblk(B_BITS, 64 * 4); newblk(B_NB, 16); newblk(B_BITS2, 64 * 4); newblk(B_NB2, 16);
  newblk(B_COUT, BUF * 8); newblk(B_SHORT, 16);
  fill_blk(g_iw.b[B_MPAR], 0x100);
  for (k = B_EQ; k <= B_EQ2; k++) {
    EquObj *e = (EquObj *)g_iw.b[k];
    fill_blk(e, sizeof *e);
    e->linLen_ = cnt256(); e->dfeLen_ = cnt256();
    e->linCoef_ = g_iw.b[k == B_EQ ? B_LIN : B_DFE]; e->dfeCoef_ = g_iw.b[k == B_EQ ? B_DFE : B_LIN];
  }
  for (k = B_P3; k <= B_P3B; k++) { Ph3DemObj *e = (Ph3DemObj *)g_iw.b[k]; fill_blk(e, sizeof *e); e->verify_ = h_rpct(50) ? h_rmod(8) : h_rpct(50) ? (h_rnd() & 0xffffu) | (h_rpct(50) ? 0xabcd0000u : 0u) : h_rnd(); }
  for (k = B_CE; k <= B_CE2; k++) { EvObj *e = (EvObj *)g_iw.b[k]; fill_blk(e, sizeof *e); e->delayedRetrainRequest_ = (int32_t)wsel(); e->delayedRetrainEnable_ = (int32_t)wsel(); }
  for (k = B_I90; k <= B_I90B; k++) fill_blk(g_iw.b[k], sizeof(P47V90InfoObj));
  for (k = B_I92; k <= B_I92B; k++) fill_blk(g_iw.b[k], sizeof(P47V92InfoObj));
  fill_blk(g_iw.b[B_V92P], 0x40);
  fill_floats(g_iw.b[B_LIN], BUF); fill_floats(g_iw.b[B_DFE], BUF); fill_floats(g_iw.b[B_EQBUF], BUF);
  { unsigned i; for (i = 0; i < 25; i++) { double d = pick_double(); memcpy((double *)g_iw.b[B_PROBE] + i, &d, 8); }
    if (h_rpct(25)) { double *pr = (double *)g_iw.b[B_PROBE]; unsigned j = h_rmod(25); for (i = 0; i < 25; i++) if (h_rpct(40)) pr[i] = pr[j]; }       /* equal levels: a zero difference */
    if (h_rpct(25)) { double *pr = (double *)g_iw.b[B_PROBE]; for (i = 0; i < 25; i++) pr[i] = (double)(h_rmod(900) + 100) * (i < 12 ? 1.0 : 2.0); } }
  { unsigned i; int *w = (int *)g_iw.b[B_INFO0]; for (i = 0; i < 0x40; i++) w[i] = h_rpct(70) ? (int)h_rmod(4) : (int)h_rnd(); w[38] = h_rpct(30) ? 0 : h_rpct(50) ? 1 : (int)h_rnd(); w[39] = h_rpct(30) ? 0 : h_rpct(50) ? 1 : (int)h_rnd(); }
  init_par((V90ParObj *)g_iw.b[B_PAR]); init_par((V90ParObj *)g_iw.b[B_PAR2]);
  init_dem((DmdObj *)g_iw.b[B_DEM], B_EQ, B_P3, B_CE); init_dem((DmdObj *)g_iw.b[B_DEM2], B_EQ2, B_P3B, B_CE2);
  { unsigned i; for (i = 0; i < BUF; i++) { uint32_t a = fw(), b = fw(); memcpy((uint32_t *)g_iw.b[B_IN] + i, &a, 4); memcpy((uint32_t *)g_iw.b[B_OUT] + i, &b, 4); } }
  { uint32_t v = h_rnd(); memcpy(g_iw.b[B_NB], &v, 4); v = h_rnd(); memcpy(g_iw.b[B_NB2], &v, 4); }
  fill_blk(g_iw.b[B_SHORT], 16); fill_blk(g_iw.b[B_COUT], BUF * 8); fill_blk(g_iw.b[B_BITS], 64 * 4); fill_blk(g_iw.b[B_BITS2], 64 * 4); fill_blk(g_iw.b[B_CTX], 0x40); fill_blk(g_iw.b[B_CTX2], 0x40);
  o = (PcmFlo47Obj *)raw;
  if (!ctor) {
    unsigned nof;
    o->ctx_ = g_iw.b[B_CTX];
    o->v90Dem_ = g_iw.b[B_DEM]; o->v90Params_ = g_iw.b[B_PAR]; o->v90Info_ = g_iw.b[B_I90]; o->v92Info_ = g_iw.b[B_I92]; o->v92Params_ = g_iw.b[B_V92P];
    o->state_ = (uint8_t)(h_rpct(88) ? h_rmod(5) : h_rnd());
    o->flag6119_ = byte_pick();
    o->flag6120_ = h_rpct(40) ? 0u : h_rpct(70) ? 1u : h_rnd();
    o->v92Session_ = h_rpct(40) ? 0u : h_rpct(70) ? 1u : h_rnd();
    o->dropToV34_ = byte_pick(); o->jaFlag_ = byte_pick(); o->cpFlag_ = byte_pick(); o->cpnotFlag_ = byte_pick(); o->cpnotCopied_ = byte_pick();
    o->constelCtr_ = (int32_t)(h_rpct(40) ? (int)h_rmod(5000) : h_rpct(30) ? (int)h_rnd() : (int)h_rmod(200000) - 100000);
    nof = h_rpct(30) ? 0 : h_rpct(60) ? 1 + h_rmod(6) : h_rpct(70) ? h_rmod(41) : h_rmod(256);
    o->nofBitsPhase4_ = (uint8_t)nof;
    o->bitPtr_ = (uint16_t)(h_rpct(40) ? 0 : h_rmod(1800 - nof - 1));
    if (h_rpct(50)) o->cpLen_ = (uint16_t)(o->bitPtr_ + nof); else o->cpLen_ = (uint16_t)(h_rpct(30) ? 0 : h_rpct(30) ? h_rmod(1801) : h_rpct(50) ? 0x8000u + h_rmod(40) : h_rnd());
    o->cpnotLen_ = (uint16_t)(h_rpct(40) ? o->bitPtr_ + 2u : h_rpct(25) ? 0 : h_rpct(60) ? h_rmod(1801) : 0x8000u + (h_rnd() & 0x7fffu));
    if (h_rpct(25)) o->cpnotLen_ = 0;
    if (h_rpct(50)) o->cpSeqCnt_ = (uint16_t)(h_rpct(60) ? h_rmod(6) : h_rpct(50) ? 0xffffu : h_rnd());
    o->minTxSeq_ = (uint16_t)(h_rpct(60) ? h_rmod(6) : h_rpct(50) ? 0xffffu : h_rnd());
    o->qcState_ = h_rpct(30) ? 0u : h_rpct(70) ? 1u + h_rmod(2) : h_rnd();
    o->qcFlag_ = h_rpct(40) ? 0u : h_rpct(70) ? 1u : h_rnd();
    o->qcCount_ = h_rpct(50) ? (int)(0x1d0 + h_rmod(24)) - (int)(h_rpct(30) ? 0x180 + 0x1d0 : 0) : h_rpct(30) ? -(int)h_rmod(0x200) : (int)wsel();
    if (h_rpct(10)) o->bitPtr_ = (uint16_t)(1800 - nof - 2 + h_rmod(2));   /* the last words of the frame */
    /* the frame buffers: random words (fill_obj) */
  }
  g_obj[X_V90] = raw + offsetof(PcmFlo47Obj, v90Mod_); g_obj[X_V92] = raw + offsetof(PcmFlo47Obj, v92Mod_); g_obj[X_EC] = raw + offsetof(PcmFlo47Obj, ecParams_);
  g_obj[X_UA] = raw + offsetof(PcmFlo47Obj, uinfoA_); g_obj[X_UB] = raw + offsetof(PcmFlo47Obj, uinfoB_); g_obj[X_L2] = raw + offsetof(PcmFlo47Obj, l2_); g_obj[X_UD] = raw + offsetof(PcmFlo47Obj, uinfoD_);
  g_obj[X_DIL] = raw + offsetof(PcmFlo47Obj, dil_); g_obj[X_CPNF] = raw + offsetof(PcmFlo47Obj, cpnotFrame_); g_obj[X_CPNL] = raw + offsetof(PcmFlo47Obj, cpnotLen_);
  g_obj[X_SINE] = raw + offsetof(PcmFlo47Obj, sineAmp_); g_obj[X_ANS] = raw + offsetof(PcmFlo47Obj, ansIir_); g_obj[X_IIR] = raw + offsetof(PcmFlo47Obj, iirDen_);
  blk_sync_model();
}
static void world_model(void) { int k; for (k = 0; k < NBLK; k++) g_mw.b[k] = g_blk[g_blkidx[k]].m; }

/* ------------------------------------------------------------------------------------------------ one call */
enum { OP_JA, OP_CP, OP_CPNOT, OP_MINSEQ, OP_NOFBITS, OP_RESETBP, OP_GETJA, OP_GETCP, OP_BAUD90, OP_BAUD34, OP_IRESET, OP_COPYMP,
       OP_SESS, OP_ERESET, OP_PH2, OP_RESET3, OP_ENTER3, OP_QC, OP_UINFO, OP_CONST, OP_LINEQ, OP_DFE, NOPS };
static const char *const opname[NOPS] = { "setTerminateJaFlag", "setTerminateCpFlag", "setTerminateCpNotFlag", "setMinNofTransmitSequences", "setNofBitsPhase4", "resetBitPointer", "getV90JaBits", "getV90CpBits",
  "setV34BaudForV90", "setV34BaudForV34", "internalReset", "copyMpInfoForInterface", "setPcmSessionType", "externalReset", "setPhaseIIinfo", "vPcmResetPhase3Modem", "enterPhase3", "qcLineVerification",
  "getUinfoValue", "getConstellation", "getLinearEqualizer", "getDFE" };
typedef struct { unsigned u; int i; unsigned long ul; } Args;
static Args gen_args(int op) {
  Args a;
  uint32_t r = h_rnd();
  memset(&a, 0, sizeof a);
  switch (op) {
    case OP_JA: case OP_CP: case OP_CPNOT: a.u = (r & 0xffu) | (h_rpct(60) ? 0u : h_garbage_hi(0)); if (h_rpct(40)) a.u = r & 1u; break;
    case OP_MINSEQ: a.u = (r & 0xffffu) | (h_rpct(50) ? 0u : (h_rnd() & 0xffff0000u)); if (h_rpct(40)) a.u = r & 3u; break;
    case OP_NOFBITS: a.u = h_rpct(40) ? 0u : h_rpct(50) ? 1u + (r & 7u) : h_rnd(); break;
    case OP_SESS: a.i = h_rpct(30) ? 0 : h_rpct(30) ? 1 : h_rpct(30) ? -1 : h_rpct(50) ? (int)(r & 0xffu) : (int)h_rnd(); break;
    case OP_PH2: a.i = h_rpct(50) ? (int)h_rmod(100) : (int)h_rnd(); break;
    case OP_UINFO: a.i = (int)((h_rpct(55) ? 0u : (r & 0xffffu)) | (h_rpct(50) ? 0u : (h_rnd() & 0xffff0000u))); break;
    case OP_CONST: case OP_LINEQ: case OP_DFE: a.ul = h_rpct(20) ? 0ul : h_rpct(15) ? 1ul : h_rpct(30) ? (unsigned long)BUF : h_rpct(25) ? 0xfffffffful : (unsigned long)h_rmod(BUF + 1u); break;
    case OP_QC: {
      static const int tg[] = { 0x1de, 0x1df, 0x1e0, 0x1e1, 0x1e2, -3, -2, -1, 0, 1 };   /* land the running count ON the 0x1df / sign boundaries (mutation steering) */
      a.u = h_rpct(25) ? h_rmod(3) : h_rpct(30) ? (unsigned)BUF : h_rpct(30) ? 0x100u - h_rmod(2) : h_rmod(BUF + 1u);
      if (h_rpct(35)) { int want = tg[h_rmod(sizeof tg / sizeof tg[0])] - ((PcmFlo47Obj *)g_iw.b[B_OBJ])->qcCount_; if (want >= 0 && want <= BUF) a.u = (unsigned)want; }
      break;
    }
    default: break;
  }
  return a;
}
static int run_impl(int op, uint8_t *t, Args *a) {
  switch (op) {
    case OP_JA: f_ja(t, a->u); return 0;
    case OP_CP: f_cp(t, a->u); return 0;
    case OP_CPNOT: f_cpnot(t, a->u); return 0;
    case OP_MINSEQ: f_minseq(t, a->u); return 0;
    case OP_NOFBITS: f_nofbits(t, a->u); return 0;
    case OP_RESETBP: f_resetbp(t); return 0;
    case OP_GETJA: return f_getja(t, (short *)g_iw.b[B_SHORT]);
    case OP_GETCP: return f_getcp(t, (short *)g_iw.b[B_SHORT]);
    case OP_BAUD90: f_baud90(t); return 0;
    case OP_BAUD34: f_baud34(t); return 0;
    case OP_IRESET: f_ireset(t); return 0;
    case OP_COPYMP: f_copymp(t); return 0;
    case OP_SESS: f_sess(t, a->i); return 0;
    case OP_ERESET: f_ereset(t); return 0;
    case OP_PH2: f_ph2(t, (int *)g_iw.b[B_INFO0], a->i); return 0;
    case OP_RESET3: f_reset3(t); return 0;
    case OP_ENTER3: f_enter3(t); return 0;
    case OP_QC: return f_qc(t, (float *)g_iw.b[B_IN], (float *)g_iw.b[B_OUT], a->u, (int *)g_iw.b[B_BITS], (int *)g_iw.b[B_NB], (int *)g_iw.b[B_BITS2], (int *)g_iw.b[B_NB2]);
    case OP_UINFO: return f_uinfo(t, (short)a->i);
    case OP_CONST: return f_const(t, g_iw.b[B_COUT], a->ul);
    case OP_LINEQ: return f_lineq(t, g_iw.b[B_COUT], a->ul);
    default: return f_dfe(t, g_iw.b[B_COUT], a->ul);
  }
}
static int run_model(int op, Args *a) {
  PcmFlo47Obj *o = mo();
  switch (op) {
    case OP_JA: m_flagset(0, a->u); return 0;
    case OP_CP: m_flagset(1, a->u); return 0;
    case OP_CPNOT: m_flagset(2, a->u); return 0;
    case OP_MINSEQ: o->cpSeqCnt_ = 0; o->minTxSeq_ = (uint16_t)a->u; return 0;
    case OP_NOFBITS: o->nofBitsPhase4_ = a->u ? 4 : 2; MDBG("SetNofBitsPhase4 - nof bits each call = %d\r\n", (int)o->nofBitsPhase4_); return 0;
    case OP_RESETBP: o->bitPtr_ = 0; o->jaFlag_ = o->cpFlag_ = o->cpnotFlag_ = o->cpnotCopied_ = 0; o->cpSeqCnt_ = 0; return 0;
    case OP_GETJA: return m_getja((short *)g_iw.b[B_SHORT]);
    case OP_GETCP: return m_getcp((short *)g_iw.b[B_SHORT]);
    case OP_BAUD90: memcpy(o->v34Flags_, k_f0, 6); return 0;
    case OP_BAUD34: memcpy(o->v34Flags_, k_f1, 6); return 0;
    case OP_IRESET: m_idle(); return 0;
    case OP_COPYMP:
      memcpy(o->mpB_, o->mpType_, 6);
      o->mpRate2_ = (uint16_t)((int)(int16_t)o->mpRate_ * 2);
      memcpy(o->mpH_, o->mpH1_, 12);
      return 0;
    case OP_SESS: m_sess(a->i); return 0;
    case OP_ERESET: m_ereset(); return 0;
    case OP_PH2: m_ph2((int *)g_iw.b[B_INFO0], a->i); return 0;
    case OP_RESET3: m_reset3(); return 0;
    case OP_ENTER3: m_enter3(); return 0;
    case OP_QC: return m_qc((float *)g_iw.b[B_IN], (float *)g_iw.b[B_OUT], a->u, (int *)g_iw.b[B_BITS], (int *)g_iw.b[B_NB], (int *)g_iw.b[B_BITS2], (int *)g_iw.b[B_NB2]);
    case OP_UINFO: return m_uinfo((short)a->i);
    case OP_CONST: return m_const(g_iw.b[B_COUT], a->ul);
    case OP_LINEQ: return m_coefs(g_iw.b[B_COUT], a->ul, 0);
    default: return m_coefs(g_iw.b[B_COUT], a->ul, 1);
  }
}
static void check_common(const char *what, uint8_t *t) {
  unsigned i;
  log_cmp(&g_ml, &g_il, what);
  if (memcmp(&g_mc, &g_ic, sizeof g_mc)) h_die3("%s: the script cursors differ", what);
  blk_compare(what);
  for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", what, i);
  (void)t;
}
static void do_call(int op) {
  uint8_t *t = g_iw.b[B_OBJ];
  unsigned i, lv0;
  int ri, rm;
  char snap[TXTMAX * 2];
  Args a = gen_args(op);
  blk_sync_model();
  world_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  g_what = opname[op];
  g_snapfn = obj_snapshot;
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  lv0 = dsplibs_debug_level;
  h_emit(g_show, "call %s u=%u i=%d ul=%lu lvl=%u", g_what, a.u, a.i, a.ul, dsplibs_debug_level);
  ri = run_impl(op, t, &a);
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  dsplibs_debug_level = lv0;
  rm = run_model(op, &a);
  check_common(g_what, t);
  if (ri != rm) h_die3("%s: the model returns %d, the impl %d", g_what, rm, ri);
  h_emit(g_show, "ret %d", ri);
  obj_snapshot(snap, sizeof snap, 0);
  h_emit(g_show, "world %s", snap);
  h_dump_obj(g_show, "D", t, f_PcmFlo47Obj, NF_PcmFlo47Obj);
  h_dump_obj(g_show, "I90", g_iw.b[B_I90], f_P47V90InfoObj, NF_P47V90InfoObj);
  h_dump_obj(g_show, "I92", g_iw.b[B_I92], f_P47V92InfoObj, NF_P47V92InfoObj);
  h_emit(g_show, "short %08x cout %08x", (unsigned)fnv_upd(2166136261u, g_iw.b[B_SHORT], 16), (unsigned)fnv_upd(2166136261u, g_iw.b[B_COUT], BUF * 8));
  cov_flush();
  g_ncalls++;
}

/* ---- constructors C1 / C2 + destructors D1 / D2 */
static void do_life(int use_c2, int use_d2) {
  uint8_t *t = g_iw.b[B_OBJ];
  unsigned i, lv0;
  char snap[TXTMAX * 2];
  unsigned side = h_rpct(35) ? 1u : h_rpct(50) ? 0u : h_rpct(50) ? 2u : h_rnd();
  unsigned blk = h_rpct(50) ? h_rmod(512) : h_rnd(), m90 = h_rpct(50) ? h_rmod(3) : h_rnd(), m92 = h_rpct(50) ? h_rmod(3) : h_rnd();
  void *ctx = h_rpct(30) ? NULL : g_iw.b[B_CTX];
  blk_sync_model();
  world_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  g_what = use_c2 ? "constructor C2" : "constructor C1";
  g_snapfn = obj_snapshot;
  g_show = h_sample(&g_symctr, 12, 100) || g_showall;
  lv0 = dsplibs_debug_level;
  h_emit(g_show, "call %s side=%u blk=%u m90=%u m92=%u ctx=%s lvl=%u", g_what, side, blk, m90, m92, ctx ? "ctx" : "null", dsplibs_debug_level);
  if (use_c2) f_c2(t, ctx, side, g_iw.b[B_MPAR], blk, m90, m92); else f_c1(t, ctx, side, g_iw.b[B_MPAR], blk, m90, m92);
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  dsplibs_debug_level = lv0;
  m_ctor(ctx, side, g_iw.b[B_MPAR], blk, m90, m92);
  check_common(g_what, t);
  obj_snapshot(snap, sizeof snap, 0);
  h_emit(g_show, "world %s", snap);
  h_dump_obj(g_show, "D", t, f_PcmFlo47Obj, NF_PcmFlo47Obj);
  h_emit(g_show, "mpar %02x%02x%02x%02x", g_iw.b[B_MPAR][0], g_iw.b[B_MPAR][1], g_iw.b[B_MPAR][2], g_iw.b[B_MPAR][3]);
  cov_flush();
  /* the destructor on the constructed object */
  blk_sync_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  g_what = use_d2 ? "destructor D2" : "destructor D1";
  lv0 = dsplibs_debug_level;
  h_emit(g_show, "call %s lvl=%u", g_what, dsplibs_debug_level);
  if (use_d2) f_d2(t); else f_d1(t);
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  dsplibs_debug_level = lv0;
  m_dtor();
  check_common(g_what, t);
  obj_snapshot(snap, sizeof snap, 0);
  h_emit(g_show, "world %s", snap);
  h_dump_obj(g_show, "D", t, f_PcmFlo47Obj, NF_PcmFlo47Obj);
  cov_flush();
  g_ncalls += 2;
}

/* ------------------------------------------------------------------------------------------------ the trials */
static unsigned pick_lv(void) { return h_rpct(65) ? 2u + h_rmod(2) : pick_level(); }
static const int g_ops0[] = { OP_JA, OP_CP, OP_CPNOT, OP_MINSEQ, OP_NOFBITS, OP_RESETBP, OP_GETJA, OP_GETCP, OP_GETCP, OP_GETJA, OP_BAUD90, OP_BAUD34, OP_IRESET, OP_COPYMP };
static const int g_ops1[] = { OP_SESS, OP_ERESET, OP_PH2, OP_RESET3, OP_ENTER3, OP_QC, OP_QC, OP_QC };
static const int g_ops2[] = { OP_UINFO, OP_UINFO, OP_CONST, OP_LINEQ, OP_DFE };
static void trial(int sc) {
  int k, nm;
  new_world(sc == G_CTOR);
  gen_script();
  dsplibs_debug_level = pick_lv();
  g_pokemask = sc == G_CTOR ? ((1u << 1) | (1u << 5) | (1u << 6) | (1u << 7) | (1u << 9) | (1u << 10) | (1u << 11)) : 0xfffffffeu;
  if (sc == G_SET) do_call(g_ops0[h_rmod(sizeof g_ops0 / sizeof g_ops0[0])]);
  else if (sc == G_SES) do_call(g_ops1[h_rmod(sizeof g_ops1 / sizeof g_ops1[0])]);
  else if (sc == G_UIN) do_call(g_ops2[h_rmod(sizeof g_ops2 / sizeof g_ops2[0])]);
  else if (sc == G_LIFE) {
    nm = (int)rnd_range(1, 4);
    for (k = 0; k < nm; k++) {
      if (k) { gen_script(); if (h_rpct(10)) dsplibs_debug_level = pick_lv(); }
      do_call((int)h_rmod(NOPS));
    }
  } else do_life(h_rpct(50), h_rpct(50));
  h_trial_end("p47");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("P47_SEED");
  CovSite *cv;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P47_ALL")) g_showall = 1;
  if (sc < 0 || sc > 4) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  p07_layout_check(); p17_layout_check(); p34_layout_check(); p40_layout_check(); p43_layout_check(); p47_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P47_TRIALS") ? atoi(getenv("P47_TRIALS")) : (sc == 3 ? 6000 : sc == 4 ? 3000 : 12000);
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (cv = __start_covsites; cv < __stop_covsites; cv++) {
    if (cv->grp != sc) continue;
    h_emit(1, "COV %s %lu", cv->name, cv->n);
    if (cv->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", cv->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P47_NOCOV")) ? 2 : 0;
}
