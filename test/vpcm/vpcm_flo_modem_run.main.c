/*
 * Differential test harness for port chunk P46 (src/vpcm/vpcm_flo_modem_run.cpp): VPcmFloModem::v90RunDemodulator and VPcmFloModem::runPcmModem.
 * Derived from test/vpcm/pcm_modem.main.c (recorders + independent model + per-record snapshot + world blocks with model copies).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/vpcm_flo_modem_run.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2: the
 * port object alone) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/vpcm_flo_modem_run.blob32.out).  The functions are reached through __asm__ labels; the object under test is the generated C mirror
 * (test/vpcm/vpcm_flo_modem_run.layout: PcmFloObj); the foreign blocks the functions read and write are the mirrors of the earlier chunks (V90Demodulator DmdObj, V90Parameters V90ParObj, V90ConnectionEvaluator EvObj,
 * V90Phase4Demodulator Ph4DemObj, V90CP CpObj, V92Modulator ModV92Obj, V92Phase4Modulator V92Ph4Obj) and plain opaque blocks.
 *
 * Every callee is a RECORDER: the V90Modem / V92Modem progress, the echo canceller (process, updateEchoHistory, setState), the resampler's timing offset (the blob's unrounded x87 getTimingOffsetPPM and the port's
 * _LD twin: the same script), the V.34 module entry points, V90CPPacker (scripted result), the Jd getConstelationSize / getJdPhase, the V92Modulator / V92Phase4Modulator event methods, setV92CPpckFromParamsInfo /
 * V92setParamsInfoFromCPUnPck, V90Demodulator::enterPhase3 / indicateRemoteRateReneg, edprintf and dsplibs_debug_printf.  The record carries the arguments as tokens and a SNAPSHOT of the object (+ the written
 * scalar fields of the foreign blocks + the debug level).  After recording, a recorder applies its scripted EFFECT (V90Modem::progress sets the demodulator's output code, V92Modem::progress the modulator's event
 * and fills the sample block, getConstelationSize writes its two bytes, ...) and then may POKE the world by script (state bytes, flags, the demodulator / parameter / modulator pointers swapped to a second block,
 * the demodulator's code, the parameter words, the modulator's state ...): the time at which the blob / port reloads a member is observable.  The INDEPENDENT MODEL (m_v90 / m_run: written from
 * porting/decompiled/dp/p46_vpcmflo_a.txt in a different shape than the port) runs AFTER the implementation on copies of the world, through the same environment functions, and must reproduce every record, the object
 * and every foreign block (exit 3 otherwise).
 *
 * Scenarios (own process each): 0 v90RunDemodulator (one call), 1 runPcmModem (one call), 2 lifecycle: 1 - 4 calls of either function on the same object.
 */

#include "p46_common.h"
#include "pcm_parameters_layout.h"
#include "v90_cp_layout.h"
#include "v90_connection_evaluator_layout.h"
#include "v92_phase4_modulator_symbol_layout.h"
#include "v90_phase4_demod_decisions_layout.h"
#include "v90_modulator_layout.h"
#include "v90_demodulator_progress_layout.h"
#include "vpcm_flo_modem_run_layout.h"

/* ---- the functions under test ---- */
extern int f_v90(void *, float *, unsigned, int *, int *) L("_ZN12VPcmFloModem17v90RunDemodulatorEPfjPiS1_");
extern int f_run(void *, float *, float *, unsigned, int *, int *, int *, int *) L("_ZN12VPcmFloModem11runPcmModemEPfS0_jPiS1_S1_S1_");

/* sysdep_malloc / sysdep_free of the blob's other code (the shared harness library is compiled with -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free); nothing in P46 allocates */
void *h_sysdep_malloc(unsigned size);
void h_sysdep_free(void *q);
void *sysdep_malloc(unsigned size) { return h_sysdep_malloc(size); }
void sysdep_free(void *q) { h_sysdep_free(q); }

/* ------------------------------------------------------------------------------------------------ the world */
typedef struct { uint32_t pcmType_, rtd_; } InfoObj;   /* V90Phase2Info prefix (P06): rtd_ is the only word read */
enum { B_OBJ, B_DEM, B_DEM2, B_PAR, B_PAR2, B_MPAR, B_CE, B_CE2, B_P4D, B_P4D2, B_CP, B_CP2, B_INFO, B_INFO2, B_MOD, B_MOD2, B_P4M, B_P4M2, B_JD, B_JD92, B_MAP, B_V92CP, B_CTX, B_CTX2, B_IN, B_OUT, B_BITS,
       B_NB, B_BITS2, B_NB2, NBLK };
enum { X_V90 = NBLK, X_V92, X_EC, X_MP1, X_MP2, X_ADDCP, X_CPV, X_CPFRAME, X_CPNOT, X_TRAIN, X_RRN, X_ECOUT, X_RS, X_RS2, NID };
static const char *const bname[NID] = { "obj", "dem", "dem2", "par", "par2", "mpar", "ce", "ce2", "p4d", "p4d2", "cp", "cp2", "info", "info2", "mod", "mod2", "p4m", "p4m2", "jd", "jd92", "map", "v92cp",
                                        "ctx", "ctx2", "in", "out", "bits", "nb", "bits2", "nb2",
                                        "obj.v90_", "obj.v92_", "obj.ec_", "obj.mp1_", "obj.mp2_", "obj.addCp_", "obj.cp_", "obj.cpFrame_", "obj.cpnotFrame_", "obj.trainConstel_", "obj.rrnConstel_", "obj.ecOut_",
                                        "dem.rs", "dem2.rs" };
typedef struct { uint8_t *b[NBLK]; } W;
static W g_iw, g_mw;
static int g_blkidx[NBLK];
static uint8_t *g_can;
#define OBJ(w) ((PcmFloObj *)(w)->b[B_OBJ])
#define DBG() (dsplibs_debug_level > 1)
enum { MAXF = 212, BUF = 256 };   /* ecOut_ holds 212 floats; the sample buffers have 256 */

enum { G_V90 = 0, G_RUN = 1 };

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
    g_sc.kind[i] = h_rpct(75) ? 0u : 1u + h_rmod(18);
    g_sc.val[i] = h_rnd();
    g_sc.res[i] = h_rnd();
  }
}
static uint32_t pick_code(uint32_t v, unsigned hi) {   /* the demodulator's output code: mostly 0 .. hi, sometimes anything */
  if (v % 16u == 0) return v >> 4;
  return (v >> 4) % (hi + 1u);
}
static uint32_t pick_event(uint32_t v) {
  static const uint32_t e[] = { 0, 1, 2, 3, 4, 5, 9, 0xa, 0xb, 3, 2, 0xa, 0x80000000u, 0xffffffffu, 0x7fffffffu, 3, 2, 0x10002u, 0x10003u, 0x1000au, 0xffff0003u, 0x20000002u };
  return v % 11u == 0 ? v : e[(v >> 3) % (sizeof e / sizeof e[0])];
}
static uint32_t pick_w(uint32_t v) { switch (v & 7u) { case 0: return 0; case 1: return 1; case 2: return 0xffffffffu; case 3: return 0x80000000u; case 4: return 0x10000u; default: return v >> 3; } }
static void *cand(W *w, unsigned v, int blk) { (void)w; return v % 5 == 0 ? g_iw.b[blk + 1] : g_iw.b[blk]; }
/* the pokes the recorders apply AFTER recording (the same function runs in the impl's and the model's world) */
static void poke(W *w, Cur *c) {
  unsigned k = c->pk++ % NSC, kind = g_sc.kind[k], v = g_sc.val[k];
  PcmFloObj *o = OBJ(w);
  if (!kind || !((g_pokemask >> kind) & 1u)) return;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->state_ = (uint8_t)(v % 11u < 8 ? (v >> 4) % 6u : v >> 4); break;
    case 3: o->flag6119_ = (uint8_t)(v >> 4); if (v & 1u) o->flag6119_ = 0; break;
    case 4: o->flag6120_ = pick_w(v >> 4); break;
    case 5: o->echoMode_ = (v & 3u) ? 0x21u : pick_w(v >> 4); break;
    case 6: o->echoCnt_ = (v & 1u) ? 0xb0u + (v >> 4) % 3u : pick_w(v >> 4); break;
    case 7: o->v90Dem_ = cand(w, v >> 3, B_DEM); break;
    case 8: o->v90Params_ = (v & 8) ? g_iw.b[B_PAR2] : g_iw.b[B_PAR]; break;
    case 9: o->v92Mod_ = (v & 8) ? g_iw.b[B_MOD2] : g_iw.b[B_MOD]; break;
    case 10: switch ((v >> 3) % 6u) {
        case 0: o->v90Info_ = (v & 0x100) ? g_iw.b[B_INFO2] : g_iw.b[B_INFO]; break;
        case 1: o->ctx_ = (v & 0x100) ? g_iw.b[B_CTX2] : g_iw.b[B_CTX]; break;
        case 2: o->trainConstel_ = (uint8_t)(v >> 12); break;
        case 3: o->rrnConstel_ = (uint8_t)(v >> 12); break;
        case 4: o->clearDown_ = (uint8_t)(v >> 12 & 1u ? 0 : v >> 13); break;
        default: o->localRrn_ = (uint8_t)(v >> 12 & 1u ? 0 : v >> 13); break;
      }
      break;
    case 11: switch ((v >> 3) % 5u) {
        case 0: o->cpFlag_ = (uint8_t)((v >> 12) & 1u ? 0 : v >> 13); break;
        case 1: o->cpnotFlag_ = (uint8_t)((v >> 12) & 1u ? 0 : v >> 13); break;
        case 2: o->b7dd1_ = (uint8_t)((v >> 12) & 1u ? 0 : v >> 13); break;
        case 3: o->cpF12_ = (uint8_t)((v >> 12) & 1u ? 0 : v >> 13); break;
        default: o->flag6119_ = (uint8_t)((v >> 12) & 1u ? 0 : v >> 13); break;
      }
      break;
    case 12: { DmdObj *d = (DmdObj *)P(w, o->v90Dem_); if (v & 8) d->phase_ = (v & 0x10) ? 4u : pick_w(v >> 5); else d->state_ = pick_code(v >> 5, 0x37); break; }
    case 13: { DmdObj *d = (DmdObj *)P(w, o->v90Dem_);
      switch ((v >> 3) % 3u) { case 0: d->ce_ = (v & 0x80) ? g_iw.b[B_CE2] : g_iw.b[B_CE]; break; case 1: d->p4_ = (v & 0x80) ? g_iw.b[B_P4D2] : g_iw.b[B_P4D]; break; default: d->cp_ = (v & 0x80) ? g_iw.b[B_CP2] : g_iw.b[B_CP]; }
      break; }
    case 14: { V90ParObj *p = (V90ParObj *)P(w, o->v90Params_);
      switch ((v >> 3) % 6u) {
        case 0: p->enableErrorCorrectionRrn = (int32_t)pick_w(v >> 7); break;
        case 1: p->silenceScr = (int32_t)(v & 0x100 ? 0x100 * (int)(v >> 12) : (int32_t)pick_w(v >> 7)); break;
        case 2: p->minimumRtdForNonSilenceScr = (v & 0x400) ? (int32_t)((InfoObj *)w->b[(v & 0x800) ? B_INFO2 : B_INFO])->rtd_ : (int32_t)pick_w(v >> 7); break;
        case 3: p->sensitiveIspDetected = (int32_t)pick_w(v >> 7); break;
        case 4: w->b[B_MPAR][3] = (uint8_t)(v >> 7); break;
        default: w->b[B_MPAR][3] = (uint8_t)(v >> 7) ^ 4u; break;
      }
      break; }
    case 15: switch ((v >> 3) % 6u) {
        case 0: ((EvObj *)w->b[B_CE])->rrnSilence_ = (int32_t)pick_w(v >> 7); break;
        case 1: ((EvObj *)w->b[B_CE2])->rrnSilence_ = (int32_t)pick_w(v >> 7); break;
        case 2: ((Ph4DemObj *)w->b[B_P4D])->w38_ = pick_w(v >> 7); ((Ph4DemObj *)w->b[B_P4D])->w3c_ = pick_w(v >> 9); break;
        case 3: ((InfoObj *)w->b[B_INFO])->rtd_ = pick_w(v >> 7); break;
        case 4: ((CpObj *)w->b[B_CP])->wCa0_ = pick_w(v >> 7); break;
        default: ((Ph4DemObj *)w->b[B_P4D2])->w3c_ = pick_w(v >> 7); break;
      }
      break;
    case 16: { ModV92Obj *m = (ModV92Obj *)P(w, o->v92Mod_);
      switch ((v >> 3) % 3u) { case 0: m->state_ = (v & 0x80) ? 3u : pick_w(v >> 8); break; case 1: m->event_ = pick_event(v >> 8); break; default: m->p4_ = (v & 0x80) ? g_iw.b[B_P4M2] : g_iw.b[B_P4M]; }
      break; }
    case 17: ((uint8_t *)&o->mpType_)[(v >> 3) % 6u] = (uint8_t)(v >> 12); o->mpRate_ = (uint16_t)(v >> 3); o->mpH1_[(v >> 5) % 6u] = (uint16_t)(v >> 16); break;
    case 18: o->v92Map_ = (v & 8) ? g_iw.b[B_V92CP] : g_iw.b[B_MAP]; o->v90Jd_ = (v & 16) ? g_iw.b[B_JD92] : g_iw.b[B_JD]; break;
    default: break;
  }
}

/* ------------------------------------------------------------------------------------------------ the snapshot of the world at the moment of a record */
static void obj_snapshot(char *out, size_t cap, int model) {
  const W *w = model ? &g_mw : &g_iw;
  const uint8_t *t = w->b[B_OBJ];
  uint32_t h = 2166136261u, f = 2166136261u;
  int i, k;
  for (i = 0; i < NF_PcmFloObj; i++) {
    const HFld *fl = &f_PcmFloObj[i];
    unsigned n = fl->count ? fl->count : 1, es, j;
    const uint8_t *q = t + fl->off;
    switch (fl->kind) {
      case 'p': case 'o':
        for (j = 0; j < n; j++) { void *v; int id; memcpy(&v, q + j * sizeof v, sizeof v); id = tok_of(v); h = fnv_upd(h, &id, sizeof id); }
        continue;
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      default: es = 4; break;
    }
    h = fnv_upd(h, q, (size_t)n * es);
  }
  for (k = 0; k < 2; k++) {   /* the foreign scalars the functions write or that the recorders write */
    const ModV92Obj *m = (const ModV92Obj *)w->b[B_MOD + k];
    const V92Ph4Obj *p4 = (const V92Ph4Obj *)w->b[B_P4M + k];
    const DmdObj *d = (const DmdObj *)w->b[B_DEM + k];
    f = fnv_upd(f, &m->codeA_, 1); f = fnv_upd(f, &m->codeB_, 1); f = fnv_upd(f, &m->skew_, 4); f = fnv_upd(f, &m->ppm_, 4); f = fnv_upd(f, &m->state_, 4); f = fnv_upd(f, &m->event_, 4);
    f = fnv_upd(f, &p4->w2c_, 4); f = fnv_upd(f, &p4->w30_, 4); f = fnv_upd(f, &p4->w38_, 4); f = fnv_upd(f, &p4->w1bc_, 4);
    f = fnv_upd(f, &d->state_, 4); f = fnv_upd(f, &d->phase_, 4);
  }
  f = fnv_upd(f, w->b[B_MPAR], 8);
  snprintf(out, cap, "|h=%08x f=%08x lvl=%u", h, f, dsplibs_debug_level);
}

/* ------------------------------------------------------------------------------------------------ the callee environment: record, effect, poke (the impl's recorders and the model call the same functions) */
static const char *nm46(int id) {
  static char b[40][24];
  static int n;
  char *o = b[n++ % 40];
  if (id >= 0 && id < g_nobj) return g_idname[id];
  if (id == ID_NULL) return "null";
  if (id == ID_WILD) return "wild";
  snprintf(o, 24, "A%d+%d", (id - 100) / 4096, (id - 100) % 4096);
  return o;
}
#define T_(p) nm46(tok_of(p))
#define ENVREC(w, c, l, ...) do { rec_add((l), (w) == &g_mw, 'c', __VA_ARGS__); } while (0)
static unsigned rd32(W *w, const void *p) { uint32_t v; if (!p) return 0xdeadbeefu; memcpy(&v, P(w, p), 4); return v; }

/* the scripted timing offset: an extended value that is NOT a float most of the time (the blob's getTimingOffsetPPM leaves it unrounded in st(0)) */
static long double ppm_val(uint32_t a, uint32_t b) {
  switch (a % 26u) {
    case 0: return 0.0L;
    case 1: return -0.0L;
    case 2: return __builtin_nanl("");
    case 3: return -__builtin_nanl("");
    case 4: return __builtin_infl();
    case 5: return -__builtin_infl();
    case 6: return (long double)(int)(b % 2001u) - 1000.0L;
    case 7: return (long double)(int32_t)b * 4294967296.0L;
    case 8: return (long double)(int32_t)b / 16.0L;
    case 9: return ((long double)(int32_t)b) / 2147483648.0L;
    case 10: return -(long double)(b % 1000u) / 1048576.0L;
    case 11: return ((long double)(int32_t)b) / 3.0L;
    case 12: return ((long double)(int32_t)b) / 1000.0L;
    case 13: return (long double)(b % 100000u) / 7.0L;
    case 14: return -(long double)(b % 100000u) / 7.0L;
    case 15: return 0.9999999L;
    case 16: return -0.9999999L;
    case 17: return 3276.7L + (long double)(b % 4u) / 10.0L;               /* around the 16-bit limit after the x10 */
    case 18: return -3276.8L - (long double)(b % 4u) / 10.0L;
    case 19: return (long double)(int32_t)b / 1048576.0L;
    case 20: return ((long double)(int32_t)b) * 1.0e-6L;
    case 21: return 3276.75L;                                              /* 32767.5 */
    case 22: return -3276.85L;
    case 23: return 3276.8L;
    case 24: return (long double)(int)(b % 65537u) / 10.0L - 3276.8L;      /* spans the 16-bit range */
    default: return ((long double)(int32_t)b) / 8.0L;
  }
}
static float ffb(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }
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
  d->state_ = pick_code(r, 0x37);
  if (q % 4u == 0) d->phase_ = (q & 8u) ? 4u : q >> 4;
  if (nb) { uint32_t v = q >> 3; memcpy(P(w, nb), &v, 4); }
  poke(w, c);
}
static void env_prog92(W *w, Cur *c, Log *l, void *self, int *bits, unsigned *nb, float *out, unsigned n) {
  uint32_t r = sres(c), q = sres(c);
  ModV92Obj *m;
  unsigned i;
  ENVREC(w, c, l, "v92.progress(this=%s bits=%s nb=%s out=%s n=%u nbv=%u)", T_(self), T_(bits), T_(nb), T_(out), n, rd32(w, nb));
  m = (ModV92Obj *)P(w, OBJ(w)->v92Mod_);
  m->event_ = pick_event(r);
  if (out) for (i = 0; i < n && i < BUF; i++) { uint32_t b = fill_word(q * 2654435761u + i * 40503u); memcpy((float *)P(w, out) + i, &b, 4); }
  if (nb) { uint32_t v = q >> 3; memcpy(P(w, nb), &v, 4); }
  poke(w, c);
}
static void env_ecproc(W *w, Cur *c, Log *l, void *self, float *in, float *out, unsigned n) {
  uint32_t q = sres(c);
  unsigned i;
  ENVREC(w, c, l, "ec.process(this=%s in=%s out=%s n=%u)", T_(self), T_(in), T_(out), n);
  for (i = 0; i < n && i < MAXF; i++) { uint32_t b = fill_word(q * 2246822519u + i * 3266489917u); memcpy((float *)P(w, out) + i, &b, 4); }
  poke(w, c);
}
static void env_echist(W *w, Cur *c, Log *l, void *self, float *x, unsigned n) { ENVREC(w, c, l, "ec.updateEchoHistory(this=%s x=%s n=%u)", T_(self), T_(x), n); poke(w, c); }
static void env_ecstate(W *w, Cur *c, Log *l, void *self, unsigned st) { ENVREC(w, c, l, "ec.setState(this=%s st=%u)", T_(self), st); poke(w, c); }
static long double env_ppm(W *w, Cur *c, Log *l, const void *self) {
  uint32_t a = sres(c), b = sres(c);
  ENVREC(w, c, l, "rs.getTimingOffsetPPM(this=%s)", T_(self));
  poke(w, c);
  return ppm_val(a, b);
}
static int env_pack(W *w, Cur *c, Log *l, void *mp, void *in, void *out, int clr) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "V90CPPacker(mp=%s in=%s out=%s clr=%d)", T_(mp), T_(in), T_(out), clr);
  poke(w, c);
  return (int)((r % 4u == 0) ? (r & 0xffff0000u) | (r >> 24 & 1u) : r);
}
static void env_gcs(W *w, Cur *c, Log *l, const char *nm, void *self, unsigned char *a, unsigned char *b) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "%s.getConstelationSize(this=%s a=%s b=%s)", nm, T_(self), T_(a), T_(b));
  *(uint8_t *)P(w, a) = (uint8_t)((r & 4u) ? r >> 8 : (r >> 8) & 1u);
  *(uint8_t *)P(w, b) = (uint8_t)((r & 8u) ? r >> 16 : (r >> 16) & 1u);
  poke(w, c);
}
static float env_phase(W *w, Cur *c, Log *l, void *self) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "jd92.getJdPhase(this=%s)", T_(self));
  poke(w, c);
  return h_float_from_bits_quiet(((r & 3u) == 0) ? 0x7fc00000u | (r >> 8 & 0x3fffffu) : (r & 0xff800000u) ? r : 0x3f800000u + (r >> 8));
}
static void env_dem1(W *w, Cur *c, Log *l, const char *nm, void *self) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "dem.%s(this=%s)", nm, T_(self));
  if (nm[0] == 'e') {   /* enterPhase3: the output code the caller tests next */
    DmdObj *d = (DmdObj *)P(w, OBJ(w)->v90Dem_);
    if ((r & 3u) != 0) d->state_ = 0x20; else if (r & 4u) d->state_ = r >> 8;
  }
  poke(w, c);
}
static void env_obj1(W *w, Cur *c, Log *l, const char *nm, void *self) { ENVREC(w, c, l, "%s(this=%s)", nm, T_(self)); poke(w, c); }
static int env_obj1i(W *w, Cur *c, Log *l, const char *nm, void *self) { ENVREC(w, c, l, "%s(this=%s)", nm, T_(self)); poke(w, c); return (int)(sres(c)); }
static void env_cppck(W *w, Cur *c, Log *l, const char *nm, void *a, void *b, void *d) { ENVREC(w, c, l, "%s(%s,%s,%s)", nm, T_(a), T_(b), T_(d)); poke(w, c); }
static void env_cppck2(W *w, Cur *c, Log *l, const char *nm, void *a, void *b) { ENVREC(w, c, l, "%s(%s,%s)", nm, T_(a), T_(b)); poke(w, c); }
static void env_v34(W *w, Cur *c, Log *l, const char *nm, void *ctx, int a, int b, int nargs) {
  if (nargs == 0) ENVREC(w, c, l, "%s(ctx=%s)", nm, T_(ctx));
  else if (nargs == 1) ENVREC(w, c, l, "%s(ctx=%s %d)", nm, T_(ctx), a);
  else ENVREC(w, c, l, "%s(ctx=%s %d %d)", nm, T_(ctx), a, b);
  poke(w, c);
}

/* ------------------------------------------------------------------------------------------------ the recorders (the blob's callees, weakened in the copy of dsplibs.o) */
extern void rc_prog90(void *, int *, unsigned *, float *, unsigned) L("_ZN8V90Modem8progressEPiRjPfj");
extern void rc_prog92(void *, int *, unsigned *, float *, unsigned) L("_ZN8V92Modem8progressEPiRjPfj");
extern void rc_ecproc(void *, float *, float *, unsigned) L("_ZN16V92EchoCanceller7processEPfS0_j");
extern void rc_echist(void *, float *, unsigned) L("_ZN16V92EchoCanceller17updateEchoHistoryEPfj");
extern void rc_ecstate(void *, unsigned) L("_ZN16V92EchoCanceller8setStateE21V92EchoCancellerState");
extern long double rc_ppm(const void *) L("_ZNK21ResamplerTimingOffset18getTimingOffsetPPMEv");
extern long double rc_ppmLD(const void *) L("_ZNK21ResamplerTimingOffset21getTimingOffsetPPM_LDEv");
extern int rc_pack(void *, void *, void *, int) L("_Z11V90CPPackerP16V90MappingParamsP22tagV90AdditionalCPinfoPsi");
extern void rc_gcs90(void *, unsigned char *, unsigned char *) L("_ZN5V90Jd19getConstelationSizeEPhS0_");
extern void rc_gcs92(void *, unsigned char *, unsigned char *) L("_ZN5V92Jd19getConstelationSizeEPhS0_");
extern float rc_phase(void *) L("_ZN5V92Jd10getJdPhaseEv");
extern void rc_enterp3(void *) L("_ZN14V90Demodulator11enterPhase3Ev");
extern void rc_indrrn(void *) L("_ZNK14V90Demodulator23indicateRemoteRateRenegEv");
extern void rc_setv92cp(void *, void *, void *) L("setV92CPpckFromParamsInfo");
extern void rc_v92set(void *, void *) L("V92setParamsInfoFromCPUnPck");
extern void rc_v34rr(void *, int, int) L("VPcmV34SetV90RateReneg");
extern void rc_v34lrrn(void *) L("VPcmV34IndicateLocalRRN");
extern void rc_v34rrrn(void *) L("VPcmV34IndicateRemoteRRN");
extern void rc_v34retr(void *) L("VPcmV34SetIndicationOfRemoteRetrain");
extern void rc_v34log(void *, int) L("VPcmV34LogTimingOffset");
extern void rc_v34jd(void *, int, int) L("V34XF_IndicateJdReceived");
extern void rc_v34dil(void *, int) L("V34XF_IndicateDilReceived");
extern void rc_v34trn(void *) L("V34XF_IndicateTrn2dReceived");
extern void rc_m_exitja(void *) L("_ZN12V92Modulator6exitJaEv");
extern void rc_m_exitsil(void *) L("_ZN12V92Modulator11exitSilenceEv");
extern void rc_m_exitsu(void *) L("_ZN12V92Modulator12exitSuSecondEv");
extern void rc_m_exittrn1(void *) L("_ZN12V92Modulator15exitTRN1uSecondEv");
extern void rc_m_exitcpt(void *) L("_ZN12V92Modulator7exitCPtEv");
extern int rc_m_initrrn(void *) L("_ZN12V92Modulator11initiateRRNEv");
extern int rc_m_initfpe(void *) L("_ZN12V92Modulator11initiateFPEEv");
extern void rc_p_exittrn2(void *) L("_ZN18V92Phase4Modulator9exitTRN2uEv");
extern void rc_p_ed(void *) L("_ZN18V92Phase4Modulator9recivedEdEv");
extern void rc_p_p1(void *) L("_ZN18V92Phase4Modulator27recivedPartOneSilenceRrnSUVEv");
extern void rc_p_p1t(void *) L("_ZN18V92Phase4Modulator30recivedPartOneSilenceRrnSUVtagEv");
extern void rc_p_p2(void *) L("_ZN18V92Phase4Modulator27recivedPartTwoSilenceRrnSUVEv");
extern void rc_p_p2t(void *) L("_ZN18V92Phase4Modulator30recivedPartTwoSilenceRrnSUVtagEv");
extern void rc_p_frst(void *) L("_ZN18V92Phase4Modulator17recivedFirstRrnEdEv");
extern void rc_p_cp(void *) L("_ZN18V92Phase4Modulator9recivedCPEv");
extern void rc_p_cpt(void *) L("_ZN18V92Phase4Modulator12recivedCPtagEv");
extern void rc_p_suv(void *) L("_ZN18V92Phase4Modulator10recivedSUVEv");
extern void rc_p_suvt(void *) L("_ZN18V92Phase4Modulator13recivedSUVtagEv");
extern void rc_p_rt(void *) L("_ZN18V92Phase4Modulator9recivedRtEv");

void rc_prog90(void *t, int *b, unsigned *nb, float *in, unsigned n) { env_prog90(IW, IC, IL, t, b, nb, in, n); }
void rc_prog92(void *t, int *b, unsigned *nb, float *out, unsigned n) { env_prog92(IW, IC, IL, t, b, nb, out, n); }
void rc_ecproc(void *t, float *in, float *out, unsigned n) { env_ecproc(IW, IC, IL, t, in, out, n); }
void rc_echist(void *t, float *x, unsigned n) { env_echist(IW, IC, IL, t, x, n); }
void rc_ecstate(void *t, unsigned s) { env_ecstate(IW, IC, IL, t, s); }
long double rc_ppm(const void *t) { return env_ppm(IW, IC, IL, t); }
long double rc_ppmLD(const void *t) { return env_ppm(IW, IC, IL, t); }
int rc_pack(void *mp, void *in, void *out, int clr) { return env_pack(IW, IC, IL, mp, in, out, clr); }
void rc_gcs90(void *t, unsigned char *a, unsigned char *b) { env_gcs(IW, IC, IL, "jd", t, a, b); }
void rc_gcs92(void *t, unsigned char *a, unsigned char *b) { env_gcs(IW, IC, IL, "jd92", t, a, b); }
float rc_phase(void *t) { return env_phase(IW, IC, IL, t); }
void rc_enterp3(void *t) { env_dem1(IW, IC, IL, "enterPhase3", t); }
void rc_indrrn(void *t) { env_dem1(IW, IC, IL, "indicateRemoteRateReneg", t); }
void rc_setv92cp(void *a, void *b, void *c) { env_cppck(IW, IC, IL, "setV92CPpckFromParamsInfo", a, b, c); }
void rc_v92set(void *a, void *b) { env_cppck2(IW, IC, IL, "V92setParamsInfoFromCPUnPck", a, b); }
void rc_v34rr(void *c, int a, int b) { env_v34(IW, IC, IL, "V34.SetV90RateReneg", c, a, b, 2); }
void rc_v34lrrn(void *c) { env_v34(IW, IC, IL, "V34.IndicateLocalRRN", c, 0, 0, 0); }
void rc_v34rrrn(void *c) { env_v34(IW, IC, IL, "V34.IndicateRemoteRRN", c, 0, 0, 0); }
void rc_v34retr(void *c) { env_v34(IW, IC, IL, "V34.SetIndicationOfRemoteRetrain", c, 0, 0, 0); }
void rc_v34log(void *c, int v) { env_v34(IW, IC, IL, "V34.LogTimingOffset", c, v, 0, 1); }
void rc_v34jd(void *c, int a, int b) { env_v34(IW, IC, IL, "V34XF.IndicateJdReceived", c, a, b, 2); }
void rc_v34dil(void *c, int a) { env_v34(IW, IC, IL, "V34XF.IndicateDilReceived", c, a, 0, 1); }
void rc_v34trn(void *c) { env_v34(IW, IC, IL, "V34XF.IndicateTrn2dReceived", c, 0, 0, 0); }
void rc_m_exitja(void *t) { env_obj1(IW, IC, IL, "mod.exitJa", t); }
void rc_m_exitsil(void *t) { env_obj1(IW, IC, IL, "mod.exitSilence", t); }
void rc_m_exitsu(void *t) { env_obj1(IW, IC, IL, "mod.exitSuSecond", t); }
void rc_m_exittrn1(void *t) { env_obj1(IW, IC, IL, "mod.exitTRN1uSecond", t); }
void rc_m_exitcpt(void *t) { env_obj1(IW, IC, IL, "mod.exitCPt", t); }
int rc_m_initrrn(void *t) { return env_obj1i(IW, IC, IL, "mod.initiateRRN", t); }
int rc_m_initfpe(void *t) { return env_obj1i(IW, IC, IL, "mod.initiateFPE", t); }
void rc_p_exittrn2(void *t) { env_obj1(IW, IC, IL, "p4.exitTRN2u", t); }
void rc_p_ed(void *t) { env_obj1(IW, IC, IL, "p4.recivedEd", t); }
void rc_p_p1(void *t) { env_obj1(IW, IC, IL, "p4.recivedPartOneSilenceRrnSUV", t); }
void rc_p_p1t(void *t) { env_obj1(IW, IC, IL, "p4.recivedPartOneSilenceRrnSUVtag", t); }
void rc_p_p2(void *t) { env_obj1(IW, IC, IL, "p4.recivedPartTwoSilenceRrnSUV", t); }
void rc_p_p2t(void *t) { env_obj1(IW, IC, IL, "p4.recivedPartTwoSilenceRrnSUVtag", t); }
void rc_p_frst(void *t) { env_obj1(IW, IC, IL, "p4.recivedFirstRrnEd", t); }
void rc_p_cp(void *t) { env_obj1(IW, IC, IL, "p4.recivedCP", t); }
void rc_p_cpt(void *t) { env_obj1(IW, IC, IL, "p4.recivedCPtag", t); }
void rc_p_suv(void *t) { env_obj1(IW, IC, IL, "p4.recivedSUV", t); }
void rc_p_suvt(void *t) { env_obj1(IW, IC, IL, "p4.recivedSUVtag", t); }
void rc_p_rt(void *t) { env_obj1(IW, IC, IL, "p4.recivedRt", t); }
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

/* ------------------------------------------------------------------------------------------------ the independent model (from porting/decompiled/dp/p46_vpcmflo_a.txt) */
static void mpf(char kind, const char *fmt, ...) {
  char b[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  env_ed(MW, MC, ML, kind, b);
}
#define MDBG(...) do { if (DBG()) mpf('d', __VA_ARGS__); } while (0)
static PcmFloObj *mo(void) { return OBJ(MW); }
static DmdObj *mdem(void) { return (DmdObj *)P(MW, mo()->v90Dem_); }
static V90ParObj *mpar(void) { return (V90ParObj *)P(MW, mo()->v90Params_); }
static ModV92Obj *mmod(void) { return (ModV92Obj *)P(MW, mo()->v92Mod_); }
static V92Ph4Obj *mp4(void) { return (V92Ph4Obj *)P(MW, mmod()->p4_); }
static void *ip(int x) { return g_obj[x]; }
static void *rsof(const DmdObj *d_model) {   /* the impl-world address of the resampler of the demodulator a model-world pointer belongs to */
  return d_model == (const DmdObj *)MW->b[B_DEM2] ? ip(X_RS2) : ip(X_RS);
}
static uint8_t *msas(void) { return (uint8_t *)P(MW, mpar()->params) + 3; }   /* the parameter block's flag byte */
static int m_chop16(long double ppm) {
  long double v = ppm * 10.0L;
  if (v != v || v >= 32768.0L || v <= -32769.0L) return -32768;
  return (int)__builtin_truncl(v);
}
static void m_logtail(void) {
  DmdObj *d = mdem();
  long double ppm = env_ppm(MW, MC, ML, rsof(d));
  void *c;
  int v = m_chop16(ppm);
  c = mo()->ctx_;
  env_v34(MW, MC, ML, "V34.LogTimingOffset", c, v, 0, 1);
}
static void m_logtail_dem(DmdObj *d) {   /* the demodulator pointer was read earlier (runPcmModem's code load) */
  long double ppm = env_ppm(MW, MC, ML, rsof(d));
  int v = m_chop16(ppm);
  env_v34(MW, MC, ML, "V34.LogTimingOffset", mo()->ctx_, v, 0, 1);
}
static void m_sas(void) { *msas() |= 4; }
static void m_flagsclear(void) {
  PcmFloObj *o = mo();
  o->jaFlag_ = 0; o->w1738_ = 0; o->cpFlag_ = 0; o->cpnotFlag_ = 0; o->b7dd1_ = 0; o->w7dd4_ = 0;
}
static void m_dropflags(void) {
  static const uint8_t f[6] = { 1, 0, 1, 1, 1, 1 };
  memcpy(mo()->v34Flags_, f, 6);
}
static unsigned char m_nof(unsigned char c) { return c ? 4 : 2; }
static void m_mpcopy(void) {
  PcmFloObj *o = mo();
  memcpy(o->mpB_, &o->mpType_, 6);
  o->mpRate2_ = (uint16_t)(((int)(int16_t)o->mpRate_) * 2);
  memcpy(o->mpH_, o->mpH1_, 12);
}
static int m_pack(void *mp, void *out, int clr) { return env_pack(MW, MC, ML, mp, ip(X_ADDCP), out, clr); }
static void m_rr(unsigned char rrnc) {   /* the V90 rate renegotiation indication */
  int16_t rs = (int16_t)((EvObj *)P(MW, mdem()->ce_))->rrnSilence_;
  env_v34(MW, MC, ML, "V34.SetV90RateReneg", mo()->ctx_, (int)rs, (int)rrnc, 2);
}
static void m_nofbits(void) {
  PcmFloObj *o = mo();
  o->nofBitsPhase4_ = m_nof(o->rrnConstel_);
  MDBG("SetNofBitsPhase4 - nof bits each call = %d\r\n", (int)o->nofBitsPhase4_);
}

static int m_v90(float *in, unsigned n, int *bits, int *nb) {
  PcmFloObj *o = mo();
  int ret;
  unsigned code;
  if (o->state_ == 3) {
    CV(G_V90, "v90: state 3");
    if (o->flag6120_ == 0) { CV(G_V90, "v90: state 3, flag6120 clear"); o->state_ = 4; ret = 3; }
    else if (mdem()->phase_ == 4 && mpar()->enableErrorCorrectionRrn != 0) { CV(G_V90, "v90: state 3, phase 4 and error correction"); o->state_ = 4; ret = 3; }
    else { CV(G_V90, "v90: state 3 stays"); ret = 2; }
  } else if (o->state_ == 2) { CV(G_V90, "v90: state 2"); ret = 1; }
  else if (o->state_ == 4) { CV(G_V90, "v90: state 4"); ret = 3; }
  else { CV(G_V90, "v90: state 0 / 1 / > 4"); ret = 0; }
  env_prog90(MW, MC, ML, ip(X_V90), bits, (unsigned *)nb, in, n);
  code = mdem()->state_;
  if (code > 0x2b) { CV(G_V90, "v90: code > 0x2b"); return ret; }
  switch (code) {
    case 1: CV(G_V90, "v90 code 1"); o->state_ = 1; mpf('e', "VPcmFloModem (V90): DemodSdDetected\r\n"); o->jaFlag_ = 1; MDBG("Ja Flag set to %d\r\n", 1); return 0;
    case 2: CV(G_V90, "v90 code 2"); mpf('e', "VPcmFloModem (V90): DemodSdNotDetected\r\n"); return ret;
    case 3: CV(G_V90, "v90 code 3");
      if (o->flag6119_ == 0) { CV(G_V90, "v90 code 3: flag6119 clear"); return ret; }
      mpf('e', "VPcmFloModem (V90): ON Start TRN1d restoring SAS detector\n"); m_sas(); return ret;
    case 6: {
      V90ParObj *p;
      uint8_t scr;
      CV(G_V90, "v90 code 6");
      env_gcs(MW, MC, ML, "jd", mo()->v90Jd_, ip(X_TRAIN), ip(X_RRN));
      MDBG("VPcmFloModem (V90): Train constellation : %d  RRN constellation : %d (0 = 4 points / 1 = 16 points)\r\n", (int)o->trainConstel_, (int)o->rrnConstel_);
      p = mpar();
      scr = (uint8_t)(p->silenceScr & 0xff);
      if (scr) {
        unsigned rtd = ((InfoObj *)P(MW, o->v90Info_))->rtd_;
        CV(G_V90, "v90 code 6: silence SCR");
        if (rtd > (unsigned)p->minimumRtdForNonSilenceScr) { CV(G_V90, "v90 code 6: RTD above the minimum"); mpf('e', "VPcmFloModem (V90): phase3 SCR set to NOT silence, due to RTD %d\r\n", (int)rtd); scr = 0; }
      }
      o->nofBitsPhase4_ = m_nof(o->trainConstel_);
      MDBG("SetNofBitsPhase4 - nof bits each call = %d\r\n", (int)o->nofBitsPhase4_);
      env_v34(MW, MC, ML, "V34XF.IndicateJdReceived", o->ctx_, (int)o->trainConstel_, (int)scr, 2);
      m_logtail();
      return ret;
    }
    case 8: CV(G_V90, "v90 code 8");
      if (*msas() & 2) { CV(G_V90, "v90 code 8: bit 1 set"); return ret; }
      *msas() &= 0xfb; return ret;
    case 18: {
      int len;
      CV(G_V90, "v90 code 18");
      env_v34(MW, MC, ML, "V34XF.IndicateDilReceived", o->ctx_, (int)o->trainConstel_, 0, 1);
      len = (int16_t)m_pack(ip(X_MP1), ip(X_CPFRAME), 0);
      o->cpLen_ = (uint16_t)len;
      MDBG("VPcmFloModem (V90): Building CPt, CPt length = %d\r\n", len);
      m_flagsclear();
      m_logtail();
      return ret;
    }
    case 21: CV(G_V90, "v90 code 21"); MDBG("VPcmFloModem (V90): DemodPhase3Error !!! \r\n"); return 5;
    case 33: CV(G_V90, "v90 code 33"); MDBG("VPcmFloModem (V90): retrain requested !!\r\n"); return 5;
    case 22: CV(G_V90, "v90 code 22"); o->state_ = 2; return 1;
    case 23: CV(G_V90, "v90 code 23"); o->cpnotFlag_ = 1; MDBG("CPnot Flag set to %d\r\n", 1);
      if (o->flag6119_ == 0) { CV(G_V90, "v90 code 23: flag6119 clear"); return ret; }
      m_sas(); return ret;
    case 25: {
      int len;
      CV(G_V90, "v90 code 25");
      o->addCp0_ = 0;
      len = (int16_t)m_pack(ip(X_MP2), ip(X_CPFRAME), (int)o->clearDown_);
      o->cpLen_ = (uint16_t)len;
      MDBG("VPcmFloModem (V90): Building CP, CP length = %d (clr=%d)\r\n", (int)(int16_t)o->cpLen_, (int)o->clearDown_);
      m_flagsclear(); o->w7dd6_ = 1;
      env_v34(MW, MC, ML, "V34XF.IndicateTrn2dReceived", o->ctx_, 0, 0, 0);
      m_logtail();
      return ret;
    }
    case 26: {
      int len;
      CV(G_V90, "v90 code 26");
      m_mpcopy();
      if (o->cpFlag_) { CV(G_V90, "v90 code 26: CP flag already set"); return ret; }
      o->addCp0_ = 1;
      len = (int16_t)m_pack(ip(X_MP2), ip(X_CPNOT), (int)o->clearDown_);
      o->cpnotLen_ = (uint16_t)len;
      MDBG("VPcmFloModem (V90): Building CPnot on MP receive, CPnot length = %d\r\n", len);
      o->cpFlag_ = 1; MDBG("CP Flag set to %d\r\n", 1);
      return ret;
    }
    case 27: {
      int len, term = 0;
      CV(G_V90, "v90 code 27");
      m_mpcopy();
      if (o->cpFlag_ == 0) {
        CV(G_V90, "v90 code 27: first MPnot");
        o->addCp0_ = 1;
        len = (int16_t)m_pack(ip(X_MP2), ip(X_CPNOT), 0);
        o->cpnotLen_ = (uint16_t)len;
        MDBG("VPcmFloModem (V90): Building CPnot on MPnot receive, CPnot length = %d\r\n", len);
        o->cpFlag_ = 1; MDBG("CP Flag set to %d\r\n", 1);
        if (mpar()->sensitiveIspDetected == 0) { CV(G_V90, "v90 code 27: no sensitive ISP"); return ret; }
        CV(G_V90, "v90 code 27: sensitive ISP");
        mpf('e', "VPcmFloModem (V90): on sensitive ISP, after one CPnot supposed to move to E...\r\n");
        term = 2;
      } else {
        if (o->b7dd1_ == 0) { CV(G_V90, "v90 code 27: b7dd1 clear"); return ret; }
        if (o->cpnotFlag_ != 0) { CV(G_V90, "v90 code 27: CPnot flag set"); return ret; }
        if (((EvObj *)P(MW, mdem()->ce_))->rrnSilence_ != 0) {
          Ph4DemObj *p4 = (Ph4DemObj *)P(MW, mdem()->p4_);
          CV(G_V90, "v90 code 27: silence RRN");
          if (p4->w3c_ != 0) {
            CV(G_V90, "v90 code 27: w3c set");
            if (p4->w38_ != 0) { CV(G_V90, "v90 code 27: w38 set"); return ret; }
          }
        }
        term = 1;
      }
      if (term == 1) MDBG("VPcmFloModem (V90): Going to terminate CPnot...\r\n");
      o->cpnotFlag_ = 1; MDBG("CPnot Flag set to %d\r\n", 1);
      o->w7dd4_ = 0; o->w7dd6_ = 1;
      return ret;
    }
    case 28: CV(G_V90, "v90 code 28");
      MDBG("VPcmFloModem (V90): Ed Received !!!\r\n");
      o->cpnotFlag_ = 1; MDBG("CPnot Flag set to %d\r\n", 1);
      o->w7dd4_ = 0; o->w7dd6_ = 1;
      if (o->clearDown_ != 0) { CV(G_V90, "v90 code 28: cleardown"); MDBG("VPcmFloModem (V90): Indicating Cleardown !\r\n"); return 8; }
      return ret;
    case 29: CV(G_V90, "v90 code 29"); MDBG("VPcmFloModem (V90): Phase4 terminated.\r\n"); return ret;
    case 30: CV(G_V90, "v90 code 30");
      MDBG("VPcmFloModem (V90): Enter Data Phase\r\n");
      o->state_ = 3; o->flag6119_ = 1; m_sas(); m_logtail(); return 2;
    case 31: CV(G_V90, "v90 code 31");
      MDBG("VPcmFloModem (V90): drop to V34 requested !! \r\n");
      o->dropToV34_ = 1; m_dropflags(); return 7;
    case 34: CV(G_V90, "v90 code 34");
      MDBG("VPcmFloModem (V90): rate renegotiation requested !!\r\n");
      m_rr(o->rrnConstel_); m_nofbits();
      o->localRrn_ = 1;
      env_v34(MW, MC, ML, "V34.IndicateLocalRRN", o->ctx_, 0, 0, 0);
      o->state_ = 2; return 1;
    case 35:
      if (o->localRrn_ != 0) { CV(G_V90, "v90 code 35: local flag"); o->localRrn_ = 0; return ret; }
      CV(G_V90, "v90 code 35: remote");
      MDBG("VPcmFloModem (V90): rate renegotiation detected !!\r\n");
      m_rr(o->rrnConstel_); m_nofbits();
      env_v34(MW, MC, ML, "V34.IndicateRemoteRRN", o->ctx_, 0, 0, 0);
      env_dem1(MW, MC, ML, "indicateRemoteRateReneg", o->v90Dem_);
      o->state_ = 2; return 1;
    case 38: CV(G_V90, "v90 code 38");
      MDBG("VPcmFloModem (V90): energy drop detected !!\r\n");
      env_v34(MW, MC, ML, "V34.SetIndicationOfRemoteRetrain", o->ctx_, 0, 0, 0);
      return 5;
    case 40: CV(G_V90, "v90 code 40"); mpf('e', "VPcmFloModem (V90): restoring SAS detector (RtNot)\n"); m_sas(); return ret;
    case 42: {
      int len;
      unsigned char clr;
      CV(G_V90, "v90 code 42");
      clr = o->clearDown_;
      o->addCp0_ = 0; o->addCp10_ = 0;
      len = (int16_t)m_pack(ip(X_MP2), ip(X_CPFRAME), (int)clr);
      o->cpLen_ = (uint16_t)len;
      MDBG("VPcmFloModem (V90): Rebuilding CP after silence rrn, CP length = %d\r\n", len);
      m_flagsclear();
      env_v34(MW, MC, ML, "V34XF.IndicateTrn2dReceived", o->ctx_, 0, 0, 0);
      return ret;
    }
    case 43: CV(G_V90, "v90 code 43");
      MDBG("VPcmFloModem (V90): terminate session requested !!\r\n");
      m_logtail(); return 8;
    default: CV(G_V90, "v90: a code without a handler"); return ret;
  }
}

static void m_pcheck(void) {}
static void m_p4call(const char *nm) { env_obj1(MW, MC, ML, nm, mmod()->p4_); }
static int m_run(float *in, float *out, unsigned n, int *bits, int *nb, int *bits2, int *nb2) {
  PcmFloObj *o = mo();
  int ret;
  ModV92Obj *mod0;
  long double ppm;
  DmdObj *dem;
  unsigned code;
  unsigned i;
  int ev;
  m_pcheck();
  if (o->state_ == 3) {
    CV(G_RUN, "run: state 3");
    if (o->flag6120_ != 0 && mdem()->phase_ == 4 && mpar()->enableErrorCorrectionRrn != 0) { CV(G_RUN, "run: state 3 -> 4"); o->state_ = 4; ret = 3; }
    else { CV(G_RUN, "run: state 3 stays"); ret = 2; }
  } else if (o->state_ == 2) { CV(G_RUN, "run: state 2"); ret = 1; }
  else if (o->state_ == 4) { CV(G_RUN, "run: state 4"); ret = 3; }
  else { CV(G_RUN, "run: state 0 / 1 / > 4"); ret = 0; }
  if (o->flag6120_ == 0) { CV(G_RUN, "run: flag6120 clear"); return ret; }
  if (o->echoMode_ == 0x21) {
    CV(G_RUN, "run: echo silence counter");
    if (o->echoCnt_ != 0xb1) { CV(G_RUN, "run: counter counts"); o->echoCnt_ += 1; } else CV(G_RUN, "run: counter saturated");
    o->ecOut_[0] = (float)(int32_t)o->echoCnt_;
  } else { CV(G_RUN, "run: no echo silence"); memset(&o->ecOut_[0], 0, 4); }
  env_ecproc(MW, MC, ML, ip(X_EC), in, ip(X_ECOUT), n);
  env_prog90(MW, MC, ML, ip(X_V90), bits, (unsigned *)nb, ip(X_ECOUT), n);
  mod0 = mmod();
  ppm = env_ppm(MW, MC, ML, rsof(mdem()));
  dem = mdem();
  code = dem->state_;
  mod0->ppm_ = (float)(-ppm);
  if (code > 0x35) CV(G_RUN, "run: code > 0x35");
  else switch (code) {
    case 1: CV(G_RUN, "run code 1");
      o->echoMode_ = 0x21; o->echoCnt_ = 0xaa;
      MDBG("\n *** Echo Silence ON *** \n");
      o->state_ = 1; ret = 0;
      env_obj1(MW, MC, ML, "mod.exitJa", o->v92Mod_);
      break;
    case 6: {
      CV(G_RUN, "run code 6");
      MDBG("\n *** Echo Silence Off *** \n");
      { void *m = o->v92Mod_; o->echoMode_ = 1; o->echoCnt_ = 0xaa; env_obj1(MW, MC, ML, "mod.exitSilence", m); }
      m_logtail();
      break;
    }
    case 7: {
      void *m1, *m3;
      float ph;
      CV(G_RUN, "run code 7");
      env_gcs(MW, MC, ML, "jd92", o->v90Jd92_, ip(X_TRAIN), ip(X_RRN));
      MDBG("VPcmFloModem (V92): Jd Detected: trainConstel = %d, rrnConstel =%d\r\n", (int)o->trainConstel_, (int)o->rrnConstel_);
      { ModV92Obj *m = mmod(); m->codeB_ = o->rrnConstel_; m->codeA_ = o->trainConstel_; }
      m1 = o->v92Mod_;
      ph = env_phase(MW, MC, ML, o->v90Jd92_);
      m3 = o->v92Mod_;
      ((ModV92Obj *)P(MW, m1))->skew_ = ph;
      env_obj1(MW, MC, ML, "mod.exitSuSecond", m3);
      if ((*msas() & 2) == 0) { CV(G_RUN, "run code 7: terminate bit clear"); *msas() &= 0xfb; } else CV(G_RUN, "run code 7: terminate bit set");
      break;
    }
    case 20: CV(G_RUN, "run code 20");
      env_cppck(MW, MC, ML, "setV92CPpckFromParamsInfo", ip(X_MP1), ip(X_ADDCP), o->v92Cp_);
      env_obj1(MW, MC, ML, "mod.exitTRN1uSecond", o->v92Mod_);
      m_logtail(); break;
    case 21: case 33: CV(G_RUN, "run code 21 / 33"); ret = 5; break;
    case 22: CV(G_RUN, "run code 22");
      o->state_ = 2; ret = 1;
      env_obj1(MW, MC, ML, "mod.exitCPt", o->v92Mod_);
      if (o->flag6119_ != 0) { CV(G_RUN, "run code 22: flag6119"); m_sas(); }
      break;
    case 25: CV(G_RUN, "run code 25");
      env_cppck(MW, MC, ML, "setV92CPpckFromParamsInfo", ip(X_MP2), ip(X_ADDCP), o->v92Cp_);
      m_p4call("p4.exitTRN2u");
      m_logtail(); break;
    case 28: CV(G_RUN, "run code 28"); m_p4call("p4.recivedEd"); break;
    case 30: CV(G_RUN, "run code 30"); o->state_ = 3; ret = 2; m_sas(); m_logtail(); break;
    case 31: CV(G_RUN, "run code 31"); o->dropToV34_ = 1; ret = 7; m_dropflags(); break;
    case 32: CV(G_RUN, "run code 32"); ret = 6; break;
    case 34: case 35: {
      if (code == 34) CV(G_RUN, "run code 34"); else CV(G_RUN, "run code 35");
      o->state_ = 2; ret = 1;
      if (mmod()->state_ == 3) {
        CV(G_RUN, "run code 34 / 35: modulator in the data phase");
        env_obj1i(MW, MC, ML, "mod.initiateRRN", o->v92Mod_);
        { DmdObj *d = mdem(); ModV92Obj *m = mmod(); V92Ph4Obj *p4 = (V92Ph4Obj *)P(MW, m->p4_); p4->w2c_ = (uint32_t)((EvObj *)P(MW, d->ce_))->rrnSilence_; }
      }
      env_v34(MW, MC, ML, code == 34 ? "V34.IndicateLocalRRN" : "V34.IndicateRemoteRRN", o->ctx_, 0, 0, 0);
      break;
    }
    case 36: case 37: CV(G_RUN, "run code 36 / 37");
      o->state_ = 2; ret = 1;
      if (mmod()->state_ == 3) { CV(G_RUN, "run code 36 / 37: modulator in the data phase"); env_obj1i(MW, MC, ML, "mod.initiateFPE", o->v92Mod_); }
      break;
    case 38: CV(G_RUN, "run code 38"); ret = 5; env_v34(MW, MC, ML, "V34.SetIndicationOfRemoteRetrain", o->ctx_, 0, 0, 0); break;
    case 39: CV(G_RUN, "run code 39"); m_p4call("p4.recivedRt"); break;
    case 43: CV(G_RUN, "run code 43"); ret = 8; m_logtail_dem(dem); break;
    case 45: CV(G_RUN, "run code 45");
      env_cppck2(MW, MC, ML, "V92setParamsInfoFromCPUnPck", o->v92Map_, ip(X_CPV));
      if (o->cpF12_ != 0) { CV(G_RUN, "run code 45: f12 set"); mp4()->w1bc_ = 1; }
      m_p4call("p4.recivedCP"); break;
    case 46: CV(G_RUN, "run code 46");
      env_cppck2(MW, MC, ML, "V92setParamsInfoFromCPUnPck", o->v92Map_, ip(X_CPV));
      m_p4call("p4.recivedCPtag"); break;
    case 47: CV(G_RUN, "run code 47"); m_p4call("p4.recivedSUV"); break;
    case 48: CV(G_RUN, "run code 48"); m_p4call("p4.recivedSUVtag"); break;
    case 49: case 51: {
      V92Ph4Obj *p4;
      if (code == 49) CV(G_RUN, "run code 49"); else CV(G_RUN, "run code 51");
      p4 = mp4();
      p4->w30_ = 1;
      p4->w38_ = ((CpObj *)P(MW, dem->cp_))->wCa0_;
      m_p4call(code == 49 ? "p4.recivedPartOneSilenceRrnSUV" : "p4.recivedPartOneSilenceRrnSUVtag");
      break;
    }
    case 50: CV(G_RUN, "run code 50"); m_p4call("p4.recivedPartTwoSilenceRrnSUV"); break;
    case 52: CV(G_RUN, "run code 52"); m_p4call("p4.recivedPartTwoSilenceRrnSUVtag"); break;
    case 53: CV(G_RUN, "run code 53"); m_p4call("p4.recivedFirstRrnEd"); break;
    default: CV(G_RUN, "run: a code without a handler"); break;
  }
  env_prog92(MW, MC, ML, ip(X_V92), bits2, (unsigned *)nb2, out, n);
  { float *po = (float *)P(MW, out);
    for (i = 0; i < n; i++) { float a = po[i]; po[i] = (float)((long double)a * (long double)0.4f); } }
  env_echist(MW, MC, ML, ip(X_EC), out, n);
  ev = (int)mmod()->event_;
  if (ev == 3) {
    CV(G_RUN, "run: event 3");
    MDBG("VPcmFloModem (V92): Freezing echo canceller...\r\n");
    env_ecstate(MW, MC, ML, ip(X_EC), 0);
    env_dem1(MW, MC, ML, "enterPhase3", o->v90Dem_);
    if (mdem()->state_ == 0x20) { CV(G_RUN, "run: fallback request"); mpf('e', "VPcmFloModem (V92): got V90 Fallback request...\r\n"); ret = 6; }
  } else if (ev > 3) {
    if (ev == 0xa) { CV(G_RUN, "run: event 10"); o->state_ = 3; } else CV(G_RUN, "run: event > 3, not 10");
  } else if (ev == 2) {
    CV(G_RUN, "run: event 2");
    o->state_ = 1;
    MDBG("VPcmFloModem (V92): Starting echo canceller training...\r\n");
    env_ecstate(MW, MC, ML, ip(X_EC), 1);
  } else CV(G_RUN, "run: event < 2");
  return ret;
}

/* ------------------------------------------------------------------------------------------------ world setup */
static void reg(int id, void *p, size_t n) { g_iw.b[id] = p; g_blkidx[id] = blk_add(bname[id], p, n); g_obj[id] = p; g_idname[id] = bname[id]; }
static void *newblk(int id, size_t n) { void *p = h_obj_new(n); reg(id, p, n); return p; }
static void fill_obj(uint8_t *t) {
  int i;
  for (i = 0; i < NF_PcmFloObj; i++) {
    const HFld *f = &f_PcmFloObj[i];
    unsigned n = f->count ? f->count : 1, es, tot, j;
    uint8_t *q = t + f->off;
    switch (f->kind) {
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
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
static void init_dem(DmdObj *d, int ce, int p4, int cp) {
  fill_blk(d, sizeof *d);
  d->state_ = pick_code(h_rnd(), 0x37);
  d->phase_ = h_rpct(40) ? 4u : h_rpct(70) ? h_rmod(6) : h_rnd();
  d->ce_ = g_iw.b[ce]; d->p4_ = g_iw.b[p4]; d->cp_ = g_iw.b[cp];
}
static void init_par(V90ParObj *p) {
  fill_blk(p, sizeof *p);
  p->params = g_iw.b[B_MPAR];
  p->silenceScr = (int32_t)(h_rpct(30) ? 0 : h_rpct(30) ? 0x100 * (int)h_rmod(5) : h_rpct(40) ? 1 : (int32_t)h_rnd());
  p->minimumRtdForNonSilenceScr = (int32_t)(h_rpct(30) ? (int32_t)h_rmod(40) : (int32_t)h_rnd());
  p->enableErrorCorrectionRrn = (int32_t)(h_rpct(40) ? 0 : h_rpct(50) ? 1 : (int32_t)h_rnd());
  p->sensitiveIspDetected = (int32_t)(h_rpct(40) ? 0 : h_rpct(30) ? 0x100 : h_rpct(50) ? 1 : (int32_t)h_rnd());
}
static void init_mod(ModV92Obj *m, int p4) {
  fill_blk(m, sizeof *m);
  m->state_ = h_rpct(45) ? 3u : h_rpct(70) ? h_rmod(5) : h_rnd();
  m->event_ = pick_event(h_rnd());
  m->p4_ = g_iw.b[p4];
  m->codeA_ = (uint8_t)h_rnd(); m->codeB_ = (uint8_t)h_rnd();
  { uint32_t a = h_rnd(), b = h_rnd(); memcpy(&m->skew_, &a, 4); memcpy(&m->ppm_, &b, 4); }
}
static void new_world(void) {
  uint8_t *raw;
  int k;
  size_t osz = sizeof(PcmFloObj);
  PcmFloObj *o;
  memset(&g_iw, 0, sizeof g_iw);
  blk_reset();
  g_nobj = NID;
  for (k = 0; k < NID; k++) { g_obj[k] = NULL; g_idname[k] = bname[k]; }
  raw = (uint8_t *)h_obj_new(osz + CAN);
  memset(raw + osz, 0x5a, CAN);
  g_can = raw + osz;
  fill_obj(raw);
  reg(B_OBJ, raw, osz);
  newblk(B_MPAR, 0x100); { uint32_t v = h_rnd(); memcpy(g_iw.b[B_MPAR] + 0, &v, 4); }
  newblk(B_CE, sizeof(EvObj)); newblk(B_CE2, sizeof(EvObj));
  newblk(B_P4D, sizeof(Ph4DemObj)); newblk(B_P4D2, sizeof(Ph4DemObj));
  newblk(B_CP, sizeof(CpObj)); newblk(B_CP2, sizeof(CpObj));
  newblk(B_INFO, sizeof(InfoObj)); newblk(B_INFO2, sizeof(InfoObj));
  newblk(B_P4M, sizeof(V92Ph4Obj)); newblk(B_P4M2, sizeof(V92Ph4Obj));
  newblk(B_MOD, sizeof(ModV92Obj)); newblk(B_MOD2, sizeof(ModV92Obj));
  newblk(B_PAR, sizeof(V90ParObj)); newblk(B_PAR2, sizeof(V90ParObj));
  newblk(B_DEM, sizeof(DmdObj)); newblk(B_DEM2, sizeof(DmdObj));
  newblk(B_JD, 0x40); newblk(B_JD92, 0x40); newblk(B_MAP, 0x40); newblk(B_V92CP, 0x40); newblk(B_CTX, 0x40); newblk(B_CTX2, 0x40);
  newblk(B_IN, BUF * 4); newblk(B_OUT, BUF * 4); newblk(B_BITS, 64 * 4); newblk(B_NB, 16); newblk(B_BITS2, 64 * 4); newblk(B_NB2, 16);
  fill_blk(g_iw.b[B_MPAR], 0x100);
  for (k = B_CE; k <= B_CE2; k++) { EvObj *e = (EvObj *)g_iw.b[k]; static const int32_t rs[] = { 0, 1, 0x10000, (int32_t)0xffff8000, 0x8000, 0x7fff, -1 }; fill_blk(e, sizeof *e); e->rrnSilence_ = h_rpct(60) ? rs[h_rmod(7)] : (int32_t)h_rnd(); }
  for (k = B_P4D; k <= B_P4D2; k++) { Ph4DemObj *e = (Ph4DemObj *)g_iw.b[k]; fill_blk(e, sizeof *e); e->w38_ = wsel(); e->w3c_ = wsel(); }
  for (k = B_CP; k <= B_CP2; k++) { CpObj *e = (CpObj *)g_iw.b[k]; fill_blk(e, sizeof *e); e->wCa0_ = h_rnd(); }
  for (k = B_INFO; k <= B_INFO2; k++) { InfoObj *e = (InfoObj *)g_iw.b[k]; e->pcmType_ = h_rnd(); e->rtd_ = h_rpct(60) ? h_rmod(40) : h_rnd(); }
  for (k = B_P4M; k <= B_P4M2; k++) { V92Ph4Obj *e = (V92Ph4Obj *)g_iw.b[k]; fill_blk(e, sizeof *e); e->w2c_ = h_rnd(); e->w30_ = h_rnd(); e->w38_ = h_rnd(); e->w1bc_ = h_rnd(); }
  init_mod((ModV92Obj *)g_iw.b[B_MOD], B_P4M); init_mod((ModV92Obj *)g_iw.b[B_MOD2], B_P4M2);
  init_par((V90ParObj *)g_iw.b[B_PAR]); init_par((V90ParObj *)g_iw.b[B_PAR2]);
  /* the RTD test of the Jd state is `rtd > minimum`: make the two equal often */
  if (h_rpct(35)) ((V90ParObj *)g_iw.b[B_PAR])->minimumRtdForNonSilenceScr = (int32_t)((InfoObj *)g_iw.b[B_INFO])->rtd_;
  if (h_rpct(35)) ((V90ParObj *)g_iw.b[B_PAR2])->minimumRtdForNonSilenceScr = (int32_t)((InfoObj *)g_iw.b[B_INFO])->rtd_;
  init_dem((DmdObj *)g_iw.b[B_DEM], B_CE, B_P4D, B_CP); init_dem((DmdObj *)g_iw.b[B_DEM2], B_CE2, B_P4D2, B_CP2);
  fill_blk(g_iw.b[B_JD], 0x40);
  { unsigned i; for (i = 0; i < BUF; i++) { uint32_t a = fill_word(h_rnd()), b = fill_word(h_rnd()); memcpy((uint32_t *)g_iw.b[B_IN] + i, &a, 4); memcpy((uint32_t *)g_iw.b[B_OUT] + i, &b, 4); } }
  { uint32_t v = h_rnd(); memcpy(g_iw.b[B_NB], &v, 4); v = h_rnd(); memcpy(g_iw.b[B_NB2], &v, 4); }
  o = (PcmFloObj *)raw;
  o->ctx_ = g_iw.b[B_CTX];
  o->v90Dem_ = g_iw.b[B_DEM]; o->v90Params_ = g_iw.b[B_PAR]; o->v90Info_ = g_iw.b[B_INFO]; o->v90Jd_ = g_iw.b[B_JD]; o->v90Jd92_ = g_iw.b[B_JD92];
  o->v92Mod_ = g_iw.b[B_MOD]; o->v92Map_ = g_iw.b[B_MAP]; o->v92Cp_ = g_iw.b[B_V92CP];
  o->state_ = (uint8_t)(h_rpct(88) ? h_rmod(5) : h_rnd());
  o->flag6119_ = (uint8_t)(h_rpct(50) ? 0 : h_rpct(70) ? 1 : h_rnd());
  o->flag6120_ = h_rpct(25) ? 0u : h_rpct(70) ? 1u : h_rnd();
  o->echoMode_ = h_rpct(40) ? 0x21u : h_rpct(50) ? h_rmod(3) : h_rnd();
  o->echoCnt_ = h_rpct(30) ? 0xb0u + h_rmod(3) : h_rpct(50) ? 0xaau + h_rmod(8) : h_rnd();
  o->cpF12_ = (uint8_t)(h_rpct(50) ? 0 : h_rnd());
  o->cpFlag_ = (uint8_t)(h_rpct(50) ? 0 : h_rnd());
  o->cpnotFlag_ = (uint8_t)(h_rpct(60) ? 0 : h_rnd());
  o->b7dd1_ = (uint8_t)(h_rpct(40) ? 0 : h_rnd());
  o->localRrn_ = (uint8_t)(h_rpct(60) ? 0 : h_rnd());
  o->clearDown_ = (uint8_t)(h_rpct(50) ? 0 : h_rpct(50) ? 1 : h_rnd());
  o->trainConstel_ = (uint8_t)(h_rpct(50) ? 0 : h_rnd());
  o->rrnConstel_ = (uint8_t)(h_rpct(50) ? 0 : h_rnd());
  g_obj[X_V90] = raw + offsetof(PcmFloObj, v90Mod_); g_obj[X_V92] = raw + offsetof(PcmFloObj, v92Mod_); g_obj[X_EC] = raw + offsetof(PcmFloObj, ecParams_);
  g_obj[X_MP1] = raw + offsetof(PcmFloObj, v90Mp1_); g_obj[X_MP2] = raw + offsetof(PcmFloObj, v90Mp2_); g_obj[X_ADDCP] = raw + offsetof(PcmFloObj, addCp0_);
  g_obj[X_CPV] = raw + offsetof(PcmFloObj, cpA_); g_obj[X_CPFRAME] = raw + offsetof(PcmFloObj, cpFrame_); g_obj[X_CPNOT] = raw + offsetof(PcmFloObj, cpnotFrame_);
  g_obj[X_TRAIN] = raw + offsetof(PcmFloObj, trainConstel_); g_obj[X_RRN] = raw + offsetof(PcmFloObj, rrnConstel_); g_obj[X_ECOUT] = raw + offsetof(PcmFloObj, ecOut_);
  g_obj[X_RS] = g_iw.b[B_DEM] + offsetof(DmdObj, rs_vptr_); g_obj[X_RS2] = g_iw.b[B_DEM2] + offsetof(DmdObj, rs_vptr_);
  blk_sync_model();
}
static void world_model(void) { int k; for (k = 0; k < NBLK; k++) g_mw.b[k] = g_blk[g_blkidx[k]].m; }

/* ------------------------------------------------------------------------------------------------ one call */
enum { OP_V90, OP_RUN };
static void do_call(int op, unsigned n) {
  uint8_t *t = g_iw.b[B_OBJ];
  unsigned i, lv0;
  int ri, rm;
  char snap[TXTMAX * 2];
  blk_sync_model();
  world_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  g_what = op == OP_V90 ? "v90RunDemodulator" : "runPcmModem";
  g_snapfn = obj_snapshot;
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  lv0 = dsplibs_debug_level;
  h_emit(g_show, "call %s n=%u lvl=%u state=%u", g_what, n, dsplibs_debug_level, (unsigned)((PcmFloObj *)t)->state_);
  /* ---- the impl */
  if (op == OP_V90) ri = f_v90(t, (float *)g_iw.b[B_IN], n, (int *)g_iw.b[B_BITS], (int *)g_iw.b[B_NB]);
  else ri = f_run(t, (float *)g_iw.b[B_IN], (float *)g_iw.b[B_OUT], n, (int *)g_iw.b[B_BITS], (int *)g_iw.b[B_NB], (int *)g_iw.b[B_BITS2], (int *)g_iw.b[B_NB2]);
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  /* ---- the model (the debug level it starts with is the impl's) */
  dsplibs_debug_level = lv0;
  if (op == OP_V90) rm = m_v90((float *)g_iw.b[B_IN], n, (int *)g_iw.b[B_BITS], (int *)g_iw.b[B_NB]);
  else rm = m_run((float *)g_iw.b[B_IN], (float *)g_iw.b[B_OUT], n, (int *)g_iw.b[B_BITS], (int *)g_iw.b[B_NB], (int *)g_iw.b[B_BITS2], (int *)g_iw.b[B_NB2]);
  log_cmp(&g_ml, &g_il, g_what);
  if (memcmp(&g_mc, &g_ic, sizeof g_mc)) h_die3("%s: the script cursors differ", g_what);
  if (ri != rm) h_die3("%s: the model returns %d, the impl %d", g_what, rm, ri);
  blk_compare(g_what);
  for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", g_what, i);
  h_emit(g_show, "ret %d", ri);
  obj_snapshot(snap, sizeof snap, 0);
  h_emit(g_show, "world %s", snap);
  h_dump_obj(g_show, "D", t, f_PcmFloObj, NF_PcmFloObj);
  cov_flush();
  g_ncalls++;
}

/* ------------------------------------------------------------------------------------------------ the trials */
static unsigned pick_n(int op) {
  if (op == OP_V90) return h_rpct(30) ? h_rmod(8) : h_rnd();
  switch (h_rmod(6)) { case 0: return 0; case 1: return 1; case 2: return MAXF; case 3: return MAXF - 1u - h_rmod(3); default: return h_rmod(MAXF + 1u); }
}
static unsigned pick_lv(void) { return h_rpct(65) ? 2u + h_rmod(2) : pick_level(); }
static void trial(int sc) {
  int k, nm;
  new_world();
  gen_script();
  dsplibs_debug_level = pick_lv();
  g_pokemask = 0xfffffffeu;
  if (sc == 0 || sc == 1) {
    do_call(sc == 0 ? OP_V90 : OP_RUN, pick_n(sc == 0 ? OP_V90 : OP_RUN));
  } else {
    nm = (int)rnd_range(1, 4);
    for (k = 0; k < nm; k++) {
      int op = h_rpct(50) ? OP_V90 : OP_RUN;
      if (k) { gen_script(); if (h_rpct(10)) dsplibs_debug_level = pick_lv(); }
      do_call(op, pick_n(op));
    }
  }
  h_trial_end("p46");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("P46_SEED");
  CovSite *cv;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P46_ALL")) g_showall = 1;
  if (sc < 0 || sc > 2) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  p07_layout_check(); p10_layout_check(); p17_layout_check(); p32_layout_check(); p37_layout_check(); p39_layout_check(); p43_layout_check(); p46_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P46_TRIALS") ? atoi(getenv("P46_TRIALS")) : (sc == 2 ? 6000 : 12000);
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (cv = __start_covsites; cv < __stop_covsites; cv++) {
    if (cv->grp != sc) continue;
    h_emit(1, "COV %s %lu", cv->name, cv->n);
    if (cv->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", cv->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P46_NOCOV")) ? 2 : 0;
}
