/*
 * Differential test harness for port chunk V03 (src/vpcm/vpcm_session_wrappers.cpp): V34GiveINFO1dBits, indicateJaTransmission, VPcmV34GetCurrentRxBitRate, VPcmV34InitMOH, VPcmV34SetMinMaxBitRates,
 * VPcmV34SetDelays, VPcmV34GetVisualDiagnostics and VPcmV34GetDiagnostics: eight small functions of the V.34 module (plain C symbols, two of them with C++ names; the port is extern "C" C++ because they call
 * methods of the V.90 / V.92 / K56Flex classes).  Derived from test/vpcm/vpcm_info1a_retrain.main.c (recorders + independent model + per-record world snapshot + pokes); shares test/v34/v01_common.h and the V.34 object mirror of
 * test/v34/v01.layout (V34Obj, 0xac4c B on i386; V03 named 35 more of its members), plus test/vpcm/vpcm_session_wrappers.layout (DiagV03Obj, the diagnostics record).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/vpcm_session_wrappers.sh against the blob (dsplibs.o with the callees WEAK), against the port (-O0 / -O2) and for x86-64 (diffed against the
 * 32-bit BLOB's output = test/golden/vpcm/vpcm_session_wrappers.blob32.out).
 *
 * Every callee is a RECORDER: VPcmV34InitiateRetrain, v34handshakinit, VPcmFloModem::enterPhase3 / getConstellation / getLinearEqualizer / getDFE, K56FlexFloModem::enterPhase3FullDuplex / setMinMaxRates /
 * getConstellation / getLinearEqualizer / getDFE / getDecisionErrors / getResamplerPhase / getResamplerOffset, V90ConstellationDesigner::setMinMaxRates, V92EchoCanceller::setEchoDelay,
 * GenericToneDetector::reset, V90Demodulator::getBitRate / getAT_UD, dsplibs_debug_printf.  The record carries the arguments as tokens (out pointers as block + offset) and a SNAPSHOT of the world (hashed);
 * a recorder then applies its scripted EFFECT (the getters fill the output array, getAT_UD scribbles over the diagnostics record, the retrain recorder rewrites a byte of the object) and RESULT (garbage in the upper
 * half where the blob sign-extends) and may POKE the world by script (debug level, the pointer slots, the mode words, the ctx flags, the V.92 modulator chain ...): the instant at which the blob reloads a member is
 * observable.  The INDEPENDENT MODEL (m_*: written from porting/decompiled/dp/v03_v34_small_wrappers.txt, a different shape than the port) runs AFTER the implementation on copies of the world, through the same environment
 * functions, and must reproduce every record, the returned value and every block (exit 3 otherwise).
 *
 * Scenarios (own process each): 0 V34GiveINFO1dBits, 1 indicateJaTransmission + VPcmV34GetCurrentRxBitRate + VPcmV34InitMOH, 2 VPcmV34SetMinMaxBitRates + VPcmV34SetDelays, 3 VPcmV34GetVisualDiagnostics, 4 VPcmV34GetDiagnostics
 * (all no pokes), 5 every function with every poke kind, 6 sequences of calls on one object with level pokes.
 */

#include "../v34/v01_common.h"
#include "pcm_bits_to_symbol_layout.h"
#include "v90_modulator_layout.h"
#include "v90_demodulator_progress_layout.h"
#include "vpcm_flo_modem_layout.h"
#include "../v34/v01_layout.h"
#include "vpcm_session_wrappers_layout.h"

/* ---- the functions under test ---- */
extern int f_give(void *, uint16_t *) L("V34GiveINFO1dBits");
extern void f_ind(void *) L("indicateJaTransmission");
extern int f_rxbr(void *) L("VPcmV34GetCurrentRxBitRate");
extern void f_moh(void *, unsigned, unsigned, unsigned) L("VPcmV34InitMOH");
extern void f_minmax(void *) L("_Z24VPcmV34SetMinMaxBitRatesP12tagV34Object");
extern void f_delays(void *) L("_Z16VPcmV34SetDelaysP12tagV34Object");
extern unsigned f_vis(void *, unsigned, void *, unsigned) L("VPcmV34GetVisualDiagnostics");
extern int f_diag(void *, void *) L("VPcmV34GetDiagnostics");

void *h_sysdep_malloc(unsigned size);
void h_sysdep_free(void *q);
void *sysdep_malloc(unsigned size) { return h_sysdep_malloc(size); }
void sysdep_free(void *q) { h_sysdep_free(q); }

/* ------------------------------------------------------------------------------------------------ the world */
enum { B_OBJ, B_P, B_P2, B_K, B_K2, B_INFO, B_CTX, B_CTX2, B_DEM, B_DEM2, B_CD, B_CD2, B_MOD, B_MOD2, B_B2S, B_B2S2, B_TX, B_TX2, B_IV, B_IV2, B_COEF, B_COEF2, B_ECHO, B_ECHO2, B_PCM, B_PCM2, B_OUT,
       B_DIAG, NBLK };
enum { X_EC = NBLK, X_EC2, X_ANS, X_ANS2, NID };
static const char *const bname[NID] = { "obj", "P", "P2", "k56", "k56b", "info", "ctx", "ctx2", "dem", "dem2", "cd", "cd2", "mod", "mod2", "b2s", "b2s2", "tx", "tx2", "iv", "iv2", "coef", "coef2", "echo", "echo2",
                                        "pcm", "pcm2", "out", "diag", "ctx.ec_", "ctx2.ec_", "ctx.ansam_", "ctx2.ansam_" };
typedef struct { uint8_t *b[NBLK]; } W;
static W g_iw, g_mw;
static int g_blkidx[NBLK];
static uint8_t *g_can;
#define OBJ(w) ((V34Obj *)(w)->b[B_OBJ])
#define DBG() (dsplibs_debug_level > 1u)
enum { INFOW = 32, GUARD = 16 };             /* the info record: 32 words, only words 0 .. 9 are used; GUARD random words before and after it */
#define INFOBLK ((INFOW + 2 * GUARD) * 2)
#define INFOP(w) ((uint16_t *)(w)->b[B_INFO] + GUARD)
enum { OUTCAP = 96, OGUARD = 4, COEFCAP = 64, SAMPCAP = 96, DGUARD = 32 };   /* output array: OUTCAP int_complex between OGUARD guard entries; coefficient / sample buffers; the diagnostics record has DGUARD bytes before / after */
#define OUTBLK ((OUTCAP + 2 * OGUARD) * 8)
#define OUTP(w) ((uint8_t *)(w)->b[B_OUT] + OGUARD * 8)
#define DIAGSZ 0x22c
#define DIAGBLK (DIAGSZ + 2 * DGUARD)
#define DIAGP(w) ((uint8_t *)(w)->b[B_DIAG] + DGUARD)

enum { G_GIVE = 0, G_MISC = 1, G_RATE = 2, G_VIS = 3, G_DIAG = 4, G_POKE = 5, G_SEQ = 6 };
enum { OP_GIVE, OP_IND, OP_RXBR, OP_MOH, OP_MINMAX, OP_DELAYS, OP_VIS, OP_DIAG, NOPS };
static const char *const opname[NOPS] = { "V34GiveINFO1dBits", "indicateJaTransmission", "VPcmV34GetCurrentRxBitRate", "VPcmV34InitMOH", "VPcmV34SetMinMaxBitRates", "VPcmV34SetDelays",
                                          "VPcmV34GetVisualDiagnostics", "VPcmV34GetDiagnostics" };

/* ------------------------------------------------------------------------------------------------ script, pokes */
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
    g_sc.kind[i] = h_rpct(70) ? 0u : 1u + h_rmod(14);
    g_sc.val[i] = h_rnd();
    g_sc.res[i] = h_rnd();
  }
}
static uint32_t pick_w(uint32_t v) { switch (v & 7u) { case 0: return 0; case 1: return 1; case 2: return 0xffffffffu; case 3: return 0x80000000u; case 4: return 0x10000u; default: return v >> 3; } }
static uint16_t pick_h(uint32_t v) { switch (v & 7u) { case 0: return 0; case 1: return 1; case 2: return 0xffffu; case 3: return 0x65; case 4: return 0x66; default: return (uint16_t)(v >> 3); } }
/* the pokes the recorders apply AFTER recording (the same function runs in the impl's and the model's world).  Only values that keep every later access inside the world's blocks are poked. */
static void poke(W *w, Cur *c) {
  unsigned k = c->pk++ % NSC, kind = g_sc.kind[k], v = g_sc.val[k];
  V34Obj *o = OBJ(w);
  if (!kind || !((g_pokemask >> kind) & 1u)) return;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->ctx_ = (v & 8) ? g_iw.b[B_CTX2] : g_iw.b[B_CTX]; break;
    case 3: o->k56_ = (v & 8) ? g_iw.b[B_K2] : g_iw.b[B_K]; break;
    case 4: o->params_ = (v & 8) ? g_iw.b[B_P2] : g_iw.b[B_P]; break;
    case 5: switch ((v >> 3) % 4u) { case 0: o->pcmMode_ = (int32_t)(v >> 6 & 3u) - (int32_t)(v >> 8 & 1u); break; case 1: o->k56Mode_ = (int32_t)(v >> 6 & 3u); break; case 2: o->mode_ = (int32_t)(v >> 6 & 3u); break;
        default: o->modeChar_ = pick_h(v >> 6); } break;
    case 6: switch ((v >> 3) % 6u) { case 0: o->wabc8_ = (int16_t)pick_h(v >> 6); break; case 1: o->waa98_ = (int16_t)pick_h(v >> 6); break; case 2: o->w359a_ = pick_h(v >> 6); break;
        case 3: o->minRateIdx_ = (int32_t)pick_w(v >> 6); break; case 4: o->w25dc_ = pick_h(v >> 6); break; default: o->constCnt_ = (int16_t)(v >> 6 & 0x3f); } break;
    case 7: { V34ModParObj *p = (V34ModParObj *)P(w, g_iw.b[(v & 8) ? B_P2 : B_P]);
      switch ((v >> 4) % 6u) { case 0: p->delay_ = (int32_t)pick_w(v >> 8); break; case 1: p->extDelay_ = (int32_t)pick_w(v >> 8); break; case 2: p->flags2_ = (uint8_t)(v >> 8); break;
        case 3: p->flags3_ = (uint8_t)(v >> 8); break; case 4: p->minRate_ = pick_w(v >> 8); break; default: p->maxRate_ = pick_w(v >> 8); } break; }
    case 8: { PcmFlo47Obj *cx = (PcmFlo47Obj *)P(w, g_iw.b[(v & 8) ? B_CTX2 : B_CTX]);
      switch ((v >> 4) % 3u) { case 0: cx->flag6120_ = pick_w(v >> 8); break; case 1: cx->v92Session_ = pick_w(v >> 8); break; default: cx->ecNCoef_ = (v >> 8) & 0x3fu; } break; }
    case 9: { K56HandleObj *kk = (K56HandleObj *)P(w, g_iw.b[(v & 8) ? B_K2 : B_K]); if (v & 16) kk->word0_ = pick_w(v >> 8); else kk->flag8_ = (uint8_t)(v >> 8); break; }
    case 10: ((uint16_t *)P(w, g_iw.b[B_INFO]) + GUARD)[(v >> 3) % 10u] = (uint16_t)(v >> 8); break;
    case 11: { PcmFlo47Obj *cx = (PcmFlo47Obj *)P(w, g_iw.b[(v & 8) ? B_CTX2 : B_CTX]);
      switch ((v >> 4) % 4u) { case 0: cx->v92Mod_ = g_iw.b[(v & 64) ? B_MOD2 : B_MOD]; break; case 1: cx->v92Info_ = g_iw.b[(v & 64) ? B_IV2 : B_IV]; break;
        case 2: cx->v90Dem_ = g_iw.b[(v & 64) ? B_DEM2 : B_DEM]; break; default: cx->ecCoef_ = g_iw.b[(v & 64) ? B_COEF2 : B_COEF]; } break; }
    case 12: switch ((v >> 3) % 3u) { case 0: ((ModV92Obj *)P(w, g_iw.b[(v & 64) ? B_MOD2 : B_MOD]))->state_ = (v & 128) ? 3u : (v >> 8 & 3u); break;
        case 1: ((ModV92Obj *)P(w, g_iw.b[(v & 64) ? B_MOD2 : B_MOD]))->b2s_ = g_iw.b[(v & 128) ? B_B2S2 : B_B2S]; break;
        default: ((B2S92 *)P(w, g_iw.b[(v & 64) ? B_B2S2 : B_B2S]))->tx_ = g_iw.b[(v & 128) ? B_TX2 : B_TX]; } break;
    case 13: switch ((v >> 3) % 3u) { case 0: ((P47V92InfoObj *)P(w, g_iw.b[(v & 64) ? B_IV2 : B_IV]))->v92CapLocal_ = (uint8_t)(v >> 8 & 1u); break;
        case 1: ((TxObj *)P(w, g_iw.b[(v & 64) ? B_TX2 : B_TX]))->w0_ = pick_w(v >> 8); break;
        default: ((DmdObj *)P(w, g_iw.b[(v & 64) ? B_DEM2 : B_DEM]))->cdes_ = g_iw.b[(v & 128) ? B_CD2 : B_CD]; } break;
    case 14: switch ((v >> 3) % 4u) { case 0: o->echoBuf_ = (int16_t *)g_iw.b[(v & 64) ? B_ECHO2 : B_ECHO]; break; case 1: o->pcmBuf_ = (int16_t *)g_iw.b[(v & 64) ? B_PCM2 : B_PCM]; break;
        case 2: o->waa88_ = (int16_t)pick_h(v >> 6); break; default: o->dgC0c_ = (int16_t)pick_h(v >> 6); } break;
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
static uint32_t hash_tok(uint32_t h, const void *p) { int id = tok_of(p); return fnv_upd(h, &id, sizeof id); }
static void obj_snapshot(char *out, size_t cap, int model) {
  const W *w = model ? &g_mw : &g_iw;
  uint32_t h = hash_tbl(2166136261u, w->b[B_OBJ], f_V34Obj, NF_V34Obj), f = 2166136261u;
  int k;
  for (k = 0; k < 2; k++) {
    const PcmFlo47Obj *cx = (const PcmFlo47Obj *)w->b[B_CTX + k];
    f = hash_tbl(f, w->b[B_P + k], f_V34ModParObj, NF_V34ModParObj);
    f = hash_tbl(f, w->b[B_K + k], f_K56HandleObj, NF_K56HandleObj);
    f = fnv_upd(f, &cx->flag6120_, 4); f = fnv_upd(f, &cx->v92Session_, 4); f = fnv_upd(f, &cx->ecNCoef_, 4);
    f = hash_tok(f, cx->v92Mod_); f = hash_tok(f, cx->v92Info_); f = hash_tok(f, cx->v90Dem_); f = hash_tok(f, cx->ecCoef_);
    f = fnv_upd(f, &((const ModV92Obj *)w->b[B_MOD + k])->state_, 4); f = hash_tok(f, ((const ModV92Obj *)w->b[B_MOD + k])->b2s_);
    f = hash_tok(f, ((const B2S92 *)w->b[B_B2S + k])->tx_); f = fnv_upd(f, &((const TxObj *)w->b[B_TX + k])->w0_, 4);
    f = fnv_upd(f, &((const P47V92InfoObj *)w->b[B_IV + k])->v92CapLocal_, 1); f = hash_tok(f, ((const DmdObj *)w->b[B_DEM + k])->cdes_);
    f = fnv_upd(f, w->b[B_COEF + k], COEFCAP * 4);
  }
  f = fnv_upd(f, w->b[B_INFO], INFOBLK);
  f = fnv_upd(f, w->b[B_OUT], OUTBLK);
  f = fnv_upd(f, w->b[B_DIAG], DIAGBLK);
  snprintf(out, cap, "|h=%08x f=%08x lvl=%u", h, f, dsplibs_debug_level);
}

/* ------------------------------------------------------------------------------------------------ the callee environment: record, effect, poke (the impl's recorders and the model call the same functions) */
static const char *nm03(int id) {
  static char b[40][24];
  static int n;
  char *o = b[n++ % 40];
  if (id >= 0 && id < g_nobj) return g_idname[id];
  if (id == ID_NULL) return "null";
  if (id == ID_WILD) return "wild";
  snprintf(o, 24, "A%d+%d", (id - 100) / 4096, (id - 100) % 4096);
  return o;
}
#define T_(p) nm03(tok_of(p))
#define ENVREC(w, c, l, ...) do { rec_add((l), (w) == &g_mw, 'c', __VA_ARGS__); } while (0)
/* a pointer into a world block (impl address) as "name+offset": the impl and the model world differ by their block addresses */
static const char *bo(const void *p) {
  static char b[16][40];
  static int n;
  char *o = b[n++ & 15];
  int k;
  if (!p) return "null";
  for (k = 0; k < NBLK; k++) {
    const uint8_t *q = g_iw.b[k];
    if ((const uint8_t *)p >= q && (const uint8_t *)p < q + g_blk[g_blkidx[k]].n) { snprintf(o, 40, "%s+%u", bname[k], (unsigned)((const uint8_t *)p - q)); return o; }
  }
  return "wild";
}

static void env_dbg(W *w, Cur *c, Log *l, const char *s) { rec_add(l, w == &g_mw, 'd', "%s", s); poke(w, c); }
static int env_hs(W *w, Cur *c, Log *l, void *obj, unsigned mode) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "v34handshakinit(obj=%s %u)", T_(obj), mode);
  poke(w, c);
  return (int)r;
}
/* the retrain recorder: the real one rewrites the object; here a byte that V34GiveINFO1dBits stores afterwards (b_ac00_) */
static void env_retrain(W *w, Cur *c, Log *l, void *obj, unsigned mode) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "VPcmV34InitiateRetrain(obj=%s %u)", T_(obj), mode & 0xffu);
  ((V34Obj *)P(w, obj))->b_ac00_ = (uint8_t)(r >> 8);
  poke(w, c);
}
static void env_this(W *w, Cur *c, Log *l, const char *nm, void *self) { ENVREC(w, c, l, "%s(this=%s)", nm, T_(self)); poke(w, c); }
static void env_k56mm(W *w, Cur *c, Log *l, void *self, int a, int b) { ENVREC(w, c, l, "K56.setMinMaxRates(this=%s %d %d)", T_(self), a, b); poke(w, c); }
static void env_cdmm(W *w, Cur *c, Log *l, void *self, unsigned a, unsigned b) { ENVREC(w, c, l, "cd.setMinMaxRates(this=%s %u %u)", T_(self), a, b); poke(w, c); }
static void env_echodelay(W *w, Cur *c, Log *l, void *self, unsigned d) { ENVREC(w, c, l, "setEchoDelay(this=%s %u)", T_(self), d); poke(w, c); }
static int env_bitrate(W *w, Cur *c, Log *l, void *self) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "getBitRate(this=%s)", T_(self));
  poke(w, c);
  return (int)r;
}
/* getAT_UD: scribbles over the first 0x224 bytes of the record (P44's TAG_DiagnosticResults) */
static void env_atud(W *w, Cur *c, Log *l, void *self, void *out) {
  uint32_t r = sres(c), i;
  uint8_t *q = (uint8_t *)P(w, out);
  ENVREC(w, c, l, "getAT_UD(this=%s out=%s)", T_(self), bo(out));
  for (i = 0; i < 0x224; i += 4) { uint32_t v = r ^ (i * 0x9e3779b1u); memcpy(q + i, &v, 4); }
  poke(w, c);
}
/* the visual-diagnostics getters: write min(r % 6, n) entries, return a scripted count (garbage above bit 15 half of the time: the blob returns eax as is) */
static int env_vis(W *w, Cur *c, Log *l, const char *nm, void *self, void *out, unsigned long n) {
  uint32_t r = sres(c), r2 = sres(c), i, cnt = r % 6u;
  uint8_t *q = (uint8_t *)P(w, out);
  ENVREC(w, c, l, "%s(this=%s out=%s n=%llu)", nm, T_(self), bo(out), (unsigned long long)n);
  if (cnt > n) cnt = (uint32_t)n;
  for (i = 0; i < cnt; i++) { uint32_t v[2]; v[0] = r2 + i; v[1] = ~r + i * 7u; memcpy(q + i * 8, v, 8); }
  poke(w, c);
  return (int)((r2 & 1u) ? r2 : (r2 & 0xffffu));
}

/* ------------------------------------------------------------------------------------------------ the recorders (the blob's callees, weakened in the copy of dsplibs.o) */
#define VIS_RC(NAME, LABEL, STR) \
  extern int rc_##NAME(void *, void *, unsigned long) L(LABEL); \
  int rc_##NAME(void *t, void *o, unsigned long n) { return env_vis(IW, IC, IL, STR, t, o, n); }
extern int rc_hs(void *, unsigned) L("v34handshakinit");
extern void rc_retrain(void *, unsigned) L("VPcmV34InitiateRetrain");
extern void rc_ep3(void *) L("_ZN12VPcmFloModem11enterPhase3Ev");
extern void rc_ep3fd(void *) L("_ZN15K56FlexFloModem21enterPhase3FullDuplexEv");
extern void rc_k56mm(void *, int, int) L("_ZN15K56FlexFloModem14setMinMaxRatesEii");
extern void rc_cdmm(void *, unsigned, unsigned) L("_ZN24V90ConstellationDesigner14setMinMaxRatesEjj");
extern void rc_echodelay(void *, unsigned) L("_ZN16V92EchoCanceller12setEchoDelayEj");
extern void rc_tdreset(void *) L("_ZN19GenericToneDetector5resetEv");
extern int rc_bitrate(void *) L("_ZNK14V90Demodulator10getBitRateEv");
extern void rc_atud(void *, void *) L("_ZNK14V90Demodulator8getAT_UDEP21TAG_DiagnosticResults");
VIS_RC(pc, "_ZN12VPcmFloModem16getConstellationEP11int_complexm", "VPcm.getConstellation")
VIS_RC(pl, "_ZN12VPcmFloModem18getLinearEqualizerEP11int_complexm", "VPcm.getLinearEqualizer")
VIS_RC(pd, "_ZN12VPcmFloModem6getDFEEP11int_complexm", "VPcm.getDFE")
VIS_RC(kc, "_ZN15K56FlexFloModem16getConstellationEP11int_complexm", "K56.getConstellation")
VIS_RC(kl, "_ZN15K56FlexFloModem18getLinearEqualizerEP11int_complexm", "K56.getLinearEqualizer")
VIS_RC(kd, "_ZN15K56FlexFloModem6getDFEEP11int_complexm", "K56.getDFE")
VIS_RC(ke, "_ZN15K56FlexFloModem17getDecisionErrorsEP11int_complexm", "K56.getDecisionErrors")
VIS_RC(kp, "_ZN15K56FlexFloModem17getResamplerPhaseEP11int_complexm", "K56.getResamplerPhase")
VIS_RC(ko, "_ZN15K56FlexFloModem18getResamplerOffsetEP11int_complexm", "K56.getResamplerOffset")
int rc_hs(void *o, unsigned m) { return env_hs(IW, IC, IL, o, m); }
void rc_retrain(void *o, unsigned m) { env_retrain(IW, IC, IL, o, m); }
void rc_ep3(void *t) { env_this(IW, IC, IL, "VPcm.enterPhase3", t); }
void rc_ep3fd(void *t) { env_this(IW, IC, IL, "K56.enterPhase3FullDuplex", t); }
void rc_k56mm(void *t, int a, int b) { env_k56mm(IW, IC, IL, t, a, b); }
void rc_cdmm(void *t, unsigned a, unsigned b) { env_cdmm(IW, IC, IL, t, a, b); }
void rc_echodelay(void *t, unsigned d) { env_echodelay(IW, IC, IL, t, d); }
void rc_tdreset(void *t) { env_this(IW, IC, IL, "GenericToneDetector.reset", t); }
int rc_bitrate(void *t) { return env_bitrate(IW, IC, IL, t); }
void rc_atud(void *t, void *o) { env_atud(IW, IC, IL, t, o); }
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  env_dbg(IW, IC, IL, buf);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ the independent model (from porting/decompiled/dp/v03_v34_small_wrappers.txt) */
static void mpf(const char *fmt, ...) {
  char b[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  env_dbg(MW, MC, ML, b);
}
#define S16(x) ((int)(int16_t)(x))
#define S16C(o, i) (((const int16_t *)(o)->constPts_)[i])
static V34Obj *mo(void) { return OBJ(MW); }
static void *mp(const void *p) { return P(MW, p); }
static PcmFlo47Obj *mcx(const void *ctxI) { return (PcmFlo47Obj *)mp(ctxI); }
static void *oobj(void) { return g_iw.b[B_OBJ]; }

/* V34GiveINFO1dBits: 1 = it started the retrain to V.34 upstream */
static int m_give(uint16_t *inf) {
  V34Obj *o = mo();
  void *ctxI = o->ctx_;
  PcmFlo47Obj *cx = mcx(ctxI);
  int pm = o->pcmMode_, cond;
  P47V92InfoObj *iv;
  cx->v92Session_ = 0;
  if (pm == 0) { CV(G_GIVE, "give: no PCM mode"); return 0; }
  if (DBG()) {
    CV(G_GIVE, "give: first print");
    mpf("VPcmV34Main: giveINFO1dBits\n");
    if (DBG()) { CV(G_GIVE, "give: info dump print"); mpf("VPcmV34Main: rxinfo1d = 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", inf[0], inf[1], inf[2], inf[3], inf[4], inf[5], inf[6], inf[7], inf[8], inf[9]); }
    else CV(G_POKE, "give: level dropped by the print");
  }
  iv = (P47V92InfoObj *)mp(cx->v92Info_);
  cond = 0;
  if (iv->v92CapLocal_ == 0) CV(G_GIVE, "give: local cap clear");
  else if (S16(o->wabc8_) == 0) CV(G_GIVE, "give: remote cap word clear");
  else if (!(inf[7] & 0x20u)) CV(G_GIVE, "give: info1 bit 5 clear");
  else { CV(G_GIVE, "give: PCM upstream requested"); cond = 1; }
  cx->v92Session_ = (uint32_t)cond;
  if (DBG()) { CV(G_GIVE, "give: selection print"); mpf("VPcmV34Main: upstream selection: local cap - %d, remote cap - %d, requested in info1 - %d, isPCM - %d\r\n", (int)iv->v92CapLocal_, (int)S16(o->wabc8_), (int)(inf[7] & 0x20u), (int)cx->v92Session_); }
  if (cx->v92Session_ == 0) { CV(G_GIVE, "give: session cleared"); return 0; }
  {
    V34ModParObj *p = (V34ModParObj *)mp(o->params_);
    if ((int8_t)p->flags2_ < 0) { CV(G_GIVE, "give: parameter bit 7 forbids the retrain"); return 0; }
  }
  CV(G_GIVE, "give: retrain to V.34 upstream");
  if (DBG()) { CV(G_GIVE, "give: retrain print"); mpf("VPcmV34Main: we got PCM upstream under V.92Lite (after Info1d), retraining to V.34 upstream...\r\n"); }
  env_retrain(MW, MC, ML, oobj(), 0x5a);
  mo()->b_ac00_ = 1;
  return 1;
}

static void m_ind(void) {
  V34Obj *o = mo();
  void *ctxI = o->ctx_, *kI = o->k56_;
  if (o->pcmMode_ > 1) { CV(G_MISC, "indicate: PCM modem phase 3"); env_this(MW, MC, ML, "VPcm.enterPhase3", ctxI); }
  else if (o->k56Mode_ > 1) { CV(G_MISC, "indicate: K56Flex phase 3"); env_this(MW, MC, ML, "K56.enterPhase3FullDuplex", kI); }
  else CV(G_MISC, "indicate: nothing");
}

static int m_rxbr(void) {
  V34Obj *o = mo();
  void *ctxI = o->ctx_, *kI = o->k56_;
  int m = o->mode_;
  if (o->modeChar_ == 0x66) {
    if (m == 1 || m == 2) { CV(G_MISC, "rxrate: V.90 demodulator rate"); return env_bitrate(MW, MC, ML, mcx(ctxI)->v90Dem_); }
    CV(G_MISC, "rxrate: 'f' family, other mode: table");
  } else if (m == 3) {
    CV(G_MISC, "rxrate: K56Flex handle word");
    return (int)((K56HandleObj *)mp(kI))->word0_;
  } else CV(G_MISC, "rxrate: not 'f', mode != 3: table");
  return (int)S16(o->waa98_) * 2400;
}

static void m_moh(unsigned a1, unsigned a2, unsigned a3) {
  V34Obj *o = mo();
  void *ctxI = o->ctx_;
  V34ModParObj *p = (V34ModParObj *)mp(o->params_);
  uint8_t b2 = (uint8_t)a2, b3 = (uint8_t)a3;
  p->flags3_ = (uint8_t)(p->flags3_ & 0xfbu);
  o->mohEC_ = a1;
  if (a1 == 1) {
    CV(G_MISC, "moh: argument 1 is 1 (bypass)");
    if (DBG()) { CV(G_MISC, "moh: bypass print"); mpf("VPcmV34Main: Special bypass - sending MOHreq instead of MOHFRR...\r\n"); }
    o->mohF0_ = 0;
    b3 = 0;
  } else { CV(G_MISC, "moh: argument 1 other"); o->mohF0_ = a1; }
  o->mohF9_ = b3;
  o->mohF4_ = 5;
  o->mohFA_ = b2 != 0;
  if (b2 != 0) CV(G_MISC, "moh: argument 2 nonzero"); else CV(G_MISC, "moh: argument 2 zero");
  o->mohE9_ = b2;
  o->mohE0_ = o->mohE2_ = o->mohE4_ = o->mohE6_ = 0;
  o->mohD8_ = 0;
  env_hs(MW, MC, ML, oobj(), 4);
  o->state_ = 7;
  o->w4bc_ = 0; o->w4be_ = 0; o->w4c0_ = 0;
  o->w2218_ = 2;
  o->mode_ = 0;
  env_this(MW, MC, ML, "GenericToneDetector.reset", (uint8_t *)ctxI + offsetof(PcmFlo47Obj, ansIir_));
  {
    int16_t mc = (int16_t)o->modeChar_;
    o->rd0_ = 0; o->rd4_ = 0; o->rd2_ = 0; o->rd6_ = 0; o->rd8_ = 0; o->rd14_ = 0; o->rd18_ = 0; o->rd1c_ = 0;
    if (mc == 0x65) { CV(G_MISC, "moh: detector reset for 'e'"); o->rdc_ = 0; o->rde_ = 0; o->rd10_ = 0x39c3; }
    else if (mc == 0x66) { CV(G_MISC, "moh: detector reset for 'f'"); o->rdc_ = 0x5a82; o->rde_ = 0x55fc; o->rd10_ = 0x39c3; }
    else CV(G_MISC, "moh: detector reset for other");
  }
}

static void m_minmax(void) {
  V34Obj *o = mo();
  int pm = o->pcmMode_;
  PcmFlo47Obj *cx = mcx(o->ctx_);
  void *kI = o->k56_;
  K56HandleObj *kk = (K56HandleObj *)mp(kI);
  V34ModParObj *p;
  uint32_t lo, hi;
  if (pm != 0 && cx->flag6120_ != 0) {
    DmdObj *dm;
    CV(G_RATE, "minmax: V.90 designer");
    p = (V34ModParObj *)mp(o->params_);
    dm = (DmdObj *)mp(cx->v90Dem_);
    env_cdmm(MW, MC, ML, dm->cdes_, p->minRate_, p->maxRate_);
    return;
  }
  if (o->k56Mode_ != 0 && kk->flag8_ != 0) {
    CV(G_RATE, "minmax: K56Flex");
    p = (V34ModParObj *)mp(o->params_);
    env_k56mm(MW, MC, ML, kI, (int)p->minRate_, (int)p->maxRate_);
    return;
  }
  p = (V34ModParObj *)mp(o->params_);
  lo = p->minRate_ / 2400u;
  hi = p->maxRate_ / 2400u;
  o->minRateIdx_ = (int32_t)lo;
  o->maxRateIdx_ = (int32_t)hi;
  if ((int32_t)lo > 14) { CV(G_RATE, "minmax: min clamped to 14"); lo = 14; o->minRateIdx_ = 14; }
  if ((int32_t)hi < (int32_t)lo) { CV(G_RATE, "minmax: max raised to min"); hi = lo; o->maxRateIdx_ = (int32_t)lo; }
  if ((int32_t)hi > 14) { CV(G_RATE, "minmax: max clamped to 14"); o->maxRateIdx_ = 14; return; }
  if (hi == 1) { CV(G_RATE, "minmax: max index 1: w359a"); o->w359a_ = 1; } else CV(G_RATE, "minmax: max index not 1");
}

static void m_delays(void) {
  V34Obj *o = mo();
  unsigned lvl = dsplibs_debug_level;
  V34ModParObj *p = (V34ModParObj *)mp(o->params_);
  void *ctxI = o->ctx_;
  int d = p->delay_, x;
  int32_t d2 = (int32_t)((uint32_t)d + 2u);                         /* the 32-bit wrap of lea 2(%ebx) */
  uint32_t fd = (uint32_t)(int32_t)((int64_t)d2 >> 2) + 0x22u;      /* floor(d2 / 4) + 0x22 */
  o->filtDelay_ = (uint16_t)fd;
  if (lvl > 1u) {
    CV(G_RATE, "delays: filtdelay print");
    mpf("V34 filtdelay set to %d (params initial delay = %d)\n", (int)(int16_t)o->filtDelay_, d);
    p = (V34ModParObj *)mp(o->params_);
    lvl = dsplibs_debug_level;
  } else CV(G_RATE, "delays: no filtdelay print");
  x = p->extDelay_;
  o->dmaDelay_ = (uint16_t)(0x610 - x);
  if (lvl > 1u) {
    CV(G_RATE, "delays: dmadelay print");
    mpf("V34FEC, V34dmadelay set to %d, (ext delay=%d)\n", (int)(int16_t)o->dmaDelay_, x);
    p = (V34ModParObj *)mp(o->params_);
  } else CV(G_RATE, "delays: no dmadelay print");
  env_echodelay(MW, MC, ML, (uint8_t *)ctxI + offsetof(PcmFlo47Obj, ecParams_), (unsigned)(p->extDelay_ + 0x68));
}

/* VPcmV34GetVisualDiagnostics: the peer object that answers (none / the PCM modem / the K56Flex handle) from the mode and the family char */
enum { PEER_NONE, PEER_PCM, PEER_K56 };
static int vis_peer(V34Obj *o) {
  int m = o->mode_;
  if (m == 2 || (m == 1 && o->modeChar_ == 0x66)) return PEER_PCM;
  if (m == 3 && o->modeChar_ == 0x65) return PEER_K56;
  return PEER_NONE;
}
static unsigned m_vis(unsigned sel, void *outI, unsigned n) {
  V34Obj *o = mo();
  void *ctxI = o->ctx_, *kI = o->k56_;
  int peer, m = o->mode_;
  uint8_t *out = (uint8_t *)mp(outI);
  unsigned i, cnt;
  int32_t *q = (int32_t *)out;
  if (sel > 8) { CV(G_VIS, "vis: selector above 8"); return 0; }
  peer = vis_peer(o);
  switch (sel) {
    case 0: case 1: {
      static const char *const pn[2] = { "VPcm.getConstellation", "VPcm.getLinearEqualizer" };
      static const char *const kn[2] = { "K56.getConstellation", "K56.getLinearEqualizer" };
      if (peer == PEER_PCM) { CV(G_VIS, "vis 0/1: PCM modem getter"); return (unsigned)env_vis(MW, MC, ML, pn[sel], ctxI, outI, n); }
      if (peer == PEER_K56) { CV(G_VIS, "vis 0/1: K56Flex getter"); return (unsigned)env_vis(MW, MC, ML, kn[sel], kI, outI, n); }
      if (sel == 0) {
        CV(G_VIS, "vis 0: own constellation points");
        cnt = (unsigned)(int)S16(o->constCnt_);
        if (cnt > n) { CV(G_VIS, "vis 0: count above the limit"); cnt = n; }
        o->constCnt_ = 0;
        for (i = 0; i < cnt; i++) { q[2 * i] = (int32_t)S16C(o, 2 * i) * 2; q[2 * i + 1] = (int32_t)S16C(o, 2 * i + 1) * 2; }
        return cnt;
      }
      CV(G_VIS, "vis 1: own equalizer taps");
      cnt = n;
      if (cnt > 0x50u) { CV(G_VIS, "vis 1: limited to 80 taps"); cnt = 0x50; }
      for (i = 0; i < cnt; i++) { q[2 * i] = (int32_t)(int16_t)o->eqRe_[i] * 2; q[2 * i + 1] = (int32_t)(int16_t)o->eqIm_[i] * 2; }
      return cnt;
    }
    case 2:
      if (peer == PEER_PCM) { CV(G_VIS, "vis 2: PCM modem getDFE"); return (unsigned)env_vis(MW, MC, ML, "VPcm.getDFE", ctxI, outI, n); }
      if (peer == PEER_K56) { CV(G_VIS, "vis 2: K56Flex getDFE"); return (unsigned)env_vis(MW, MC, ML, "K56.getDFE", kI, outI, n); }
      CV(G_VIS, "vis 2: no DFE");
      return 0;
    case 3: case 4:
      if (m == 1 || m == 2) CV(G_VIS, "vis 3/4: V.90 / V.92 mode: only the zero imaginary part");
      else if (peer == PEER_K56) { CV(G_VIS, "vis 3/4: K56Flex getter"); env_vis(MW, MC, ML, sel == 3 ? "K56.getResamplerPhase" : "K56.getResamplerOffset", kI, outI, n); }
      else if (sel == 3) { CV(G_VIS, "vis 3: own resampler phase"); q[0] = o->resPhase_; }
      else { CV(G_VIS, "vis 4: own resampler offset"); q[0] = (int32_t)S16(o->resOffs_); }
      q[1] = 0;
      return 1;
    case 5:
      if (m == 2) {
        PcmFlo47Obj *cx = mcx(ctxI);
        const float *src = (const float *)mp(cx->ecCoef_);
        cnt = cx->ecNCoef_;
        if (cnt > n) { CV(G_VIS, "vis 5: echo taps above the limit"); cnt = n; }
        CV(G_VIS, "vis 5: echo canceller taps x 10000");
        for (i = 0; i < cnt; i++) {
          double v = (double)src[i] * 10000.0;
          int32_t r;
          if (!(v > -2147483649.0 && v < 2147483648.0)) { CV(G_VIS, "vis 5: tap out of range / NaN: indefinite"); r = (int32_t)0x80000000u; }
          else r = (int32_t)v;
          q[2 * i] = 0; q[2 * i + 1] = r;
        }
        return cnt;
      }
      CV(G_VIS, "vis 5: own echo samples");
      cnt = o->echoCnt_;
      if (cnt > n) cnt = n;
      { const int16_t *s = (const int16_t *)mp(o->echoBuf_); for (i = 0; i < cnt; i++) { q[2 * i] = 0; q[2 * i + 1] = s[i]; } }
      return cnt;
    case 6:
      if (m != 0) { CV(G_VIS, "vis 6: mode != 0"); return 0; }
      CV(G_VIS, "vis 6: PCM samples");
      cnt = o->pcmCnt_;
      if (cnt > n) cnt = n;
      { const int16_t *s = (const int16_t *)mp(o->pcmBuf_); for (i = 0; i < cnt; i++) { q[2 * i] = 0; q[2 * i + 1] = s[i]; } }
      return cnt;
    case 7:
      CV(G_VIS, "vis 7: nothing");
      return 0;
    default: /* 8 */
      if (m == 1 || m == 2) { CV(G_VIS, "vis 8: V.90 / V.92: nothing"); return 0; }
      if (peer == PEER_K56) { CV(G_VIS, "vis 8: K56Flex decision errors"); return (unsigned)env_vis(MW, MC, ML, "K56.getDecisionErrors", kI, outI, n); }
      CV(G_VIS, "vis 8: no peer");
      return 0;
  }
}

/* VPcmV34GetDiagnostics: first the mode part (it may call getAT_UD, which scribbles over the record), then the common words of the +0xac04 block */
static int m_diag(void *outI) {
  V34Obj *o = mo();
  void *ctxI = o->ctx_;
  PcmFlo47Obj *cx = mcx(ctxI);
  DiagV03Obj *d = (DiagV03Obj *)mp(outI);
  unsigned lvl = dsplibs_debug_level;
  int mode;
  if (lvl > 1u) { CV(G_DIAG, "diag: called print"); mpf("VPcmV34GetDiagnostics called...\r\n"); lvl = dsplibs_debug_level; }
  mode = o->mode_;
  if (mode == 1 || mode == 2) {
    int v92 = (mode == 2);
    if (cx->flag6120_ == 0) {
      if (lvl > 1u) {
        if (v92) { CV(G_DIAG, "diag: V.92 Digital print"); mpf("...It is diagnostics of V.92 Digital...\r\n"); }
        else { CV(G_DIAG, "diag: V.90 Digital print"); mpf("...It is diagnostics of V.90 Digital...\r\n"); }
      } else CV(G_DIAG, "diag: Digital, no print");
    } else {
      if (lvl > 1u) {
        if (v92) { CV(G_DIAG, "diag: V.92 Analog print"); mpf("...It is diagnostics of V.92 Analog...\r\n"); }
        else { CV(G_DIAG, "diag: V.90 Analog print"); mpf("...It is diagnostics of V.90 Analog...\r\n"); }
      } else CV(G_DIAG, "diag: Analog, no print");
      env_atud(MW, MC, ML, mcx(ctxI)->v90Dem_, outI);
      if (!v92) {
        CV(G_DIAG, "diag: V.90 Analog words");
        d->baud_ = (int32_t)(int16_t)o->baud_;
        d->snrDb_ = (float)(-12.0 - (double)(int16_t)o->w25dc_);
        d->txRate_ = (int32_t)S16(o->waa88_) * 2400;
        d->carrier_ = (int32_t)(int16_t)o->carrier_;
      } else {
        ModV92Obj *mod = (ModV92Obj *)mp(cx->v92Mod_);
        int32_t r = 0;
        CV(G_DIAG, "diag: V.92 Analog words");
        d->baud_ = 0x1f40;
        d->snrDb_ = -12.0f;
        if (mod->state_ == 3) {
          B2S92 *b = (B2S92 *)mp(mod->b2s_);
          TxObj *t = (TxObj *)mp(b->tx_);
          uint32_t u = t->w0_ * 8000u;
          CV(G_DIAG, "diag: V.92 data phase: rate from the transmitter");
          r = (int32_t)(uint32_t)(((uint64_t)u * 11184811ull + (1ull << 26)) >> 27);   /* floor(0.5 + u * float(1/12)), exactly */
        } else CV(G_DIAG, "diag: V.92 not in the data phase");
        d->txRate_ = r;
        d->carrier_ = 0;
      }
    }
  } else {
    int32_t ecx, ebp = 0, ebx = 0;
    int16_t dv;
    if (lvl > 1u) { CV(G_DIAG, "diag: V.34 print"); mpf("...It is diagnostics of V.34 (or P2)...\r\n"); } else CV(G_DIAG, "diag: V.34 no print");
    d->baud_ = (int32_t)(int16_t)o->baud_;
    d->txRate_ = (int32_t)S16(o->waa88_) * 2400;
    d->rxRate_ = (int32_t)S16(o->waa98_) * 2400;
    d->wbc_ = (int32_t)S16(o->waaa8_);
    d->carrier_ = (int32_t)(int16_t)o->carrier_;
    d->wb4_ = (int32_t)S16(o->waa96_);
    d->snrDb_ = (float)(-12.0 - (double)(int16_t)o->w25dc_);
    dv = o->w47e_;
    if (dv > 0) {
      ecx = o->w4ac_ / dv;
      if (ecx > 0) {
        CV(G_DIAG, "diag: estimate, 6 dB steps");
        do {
          ebp = ecx;
          ecx = (int32_t)((uint32_t)ecx * 0x1013u) >> 14;
          if (ecx > 0) ebx += 6;
        } while (ecx > 0);
      } else CV(G_DIAG, "diag: estimate, quotient not positive");
    } else CV(G_DIAG, "diag: estimate, divisor not positive");
    if (ebp > 0) {
      int32_t e = ebp;
      CV(G_DIAG, "diag: estimate, 1 dB steps");
      for (;;) {
        e = (int32_t)((uint32_t)e * 0x32d6u) >> 14;
        if (e <= 0) break;
        ebx++;
      }
    }
    d->dly84_ = d->dly80_ = (int32_t)S16(o->waa7e_);
    d->qcA_ = d->qcB_ = (float)ebx;
  }
  /* the common tail */
  d->wc0_ = (int32_t)o->dgC04_;
  d->wc4_ = (int32_t)o->dgC08_;
  d->wec_ = (int32_t)S16(o->dgC0e_) + (int32_t)S16(o->dgC10_);
  d->wf4_ = (int32_t)S16(o->dgC14_);
  d->wf0_ = d->w100_ = d->we8_ = (int32_t)S16(o->dgC12_);
  if (S16(o->dgC0c_) == 0) { CV(G_DIAG, "diag: dgC0c zero: -1"); d->w228_ = -1; }
  else { CV(G_DIAG, "diag: dgC0c nonzero"); d->w228_ = (int32_t)S16(o->dgC0c_); }
  return d->w228_;
}

/* ------------------------------------------------------------------------------------------------ world setup */
static void reg(int id, void *p, size_t n) { g_iw.b[id] = (uint8_t *)p; g_blkidx[id] = blk_add(bname[id], p, n); g_obj[id] = p; g_idname[id] = bname[id]; }
static void *newblk(int id, size_t n) { void *p = h_obj_new(n); reg(id, p, n); return p; }
/* random-fill an object member by member (the random stream must not depend on sizeof: x86-64 pointers are 8 bytes) */
static void fill_tbl(uint8_t *t, const HFld *tbl, int nf) {
  int i;
  for (i = 0; i < nf; i++) {
    const HFld *f = &tbl[i];
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
static void fill_blk(void *p, size_t n) { size_t j; uint32_t s = h_rnd(); for (j = 0; j + 4 <= n; j += 4) { s = s * 1664525u + 1013904223u; memcpy((uint8_t *)p + j, &s, 4); } }
static uint32_t wsel(void) { switch (h_rmod(8)) { case 0: return 0; case 1: return 1; case 2: return h_rnd(); case 3: return 0x100; case 4: return 0x80000000u; case 5: return h_rmod(16); default: return h_rnd(); } }
static int32_t pick_mode3(void) { switch (h_rmod(10)) { case 0: case 1: case 2: return 0; case 3: case 4: return 1; case 5: case 6: return 2; case 7: return -1; case 8: return 3; default: return (int32_t)h_rnd(); } }
static uint16_t pick_modechar(void) { return (uint16_t)(h_rpct(38) ? 0x65 : h_rpct(55) ? 0x66 : h_rpct(30) ? (0x65 ^ (1u << h_rmod(16))) : h_rnd()); }
static uint16_t pick_h16(void) { return (uint16_t)(h_rpct(25) ? 0 : h_rpct(30) ? 1 : h_rpct(25) ? 0xffffu : h_rnd()); }
static uint32_t pick_rate(void) {
  uint32_t k = h_rmod(22);
  switch (h_rmod(10)) {
    case 0: case 1: case 2: return k * 2400u;
    case 3: return k * 2400u + 2399u;
    case 4: return k * 2400u - 1u;
    case 5: return k * 2400u + 1u;
    case 6: return 0;
    case 7: return h_rpct(50) ? 0xffffffffu : 0x80000000u + h_rmod(4);
    default: return h_rnd() >> h_rmod(32);
  }
}
static int32_t pick_delay(void) {
  switch (h_rmod(8)) { case 0: return (int32_t)0x7fffffff - (int32_t)h_rmod(3); case 1: return (int32_t)0x80000000u + (int32_t)h_rmod(3); case 2: return -(int32_t)h_rmod(40); case 3: return (int32_t)h_rnd(); case 4: return (int32_t)h_rmod(0x700); default: return (int32_t)h_rmod(200); }
}
/* float bit patterns for the echo taps: 0, small, near the 1/10000 integer boundaries, huge, inf, NaN (quiet / signalling), denormal */
static uint32_t pick_fbits(void) {
  switch (h_rmod(14)) {
    case 0: return 0; case 1: return 0x80000000u;
    case 2: return h_bits_of((float)((int)h_rmod(200) - 100) / 10000.0f);
    case 3: return h_bits_of((float)((int)h_rmod(60) - 30));
    case 4: return h_bits_of(214748.36f) + (h_rmod(5) - 2u);     /* 2147483647 / 10000 = 214748.3647: the limit of the integer range */
    case 5: return h_bits_of(-214748.36f) + (h_rmod(5) - 2u);
    case 6: return 0x7f800000u | (h_rpct(50) ? 0x80000000u : 0u);
    case 7: return 0x7fc00000u | h_rmod(4);
    case 8: return 0x7f800001u | (h_rnd() & 0x3fffffu);          /* signalling NaN */
    case 9: return h_rnd() & 0x007fffffu;                          /* denormal */
    case 10: return h_bits_of(1.0f / 10000.0f) + (h_rmod(3) - 1u);
    case 11: return 0x7f7fffffu;
    default: return h_rnd();
  }
}
static unsigned pick_n(void) {
  static const unsigned v[] = { 0, 1, 2, 3, 5, 0x4f, 0x50, 0x51, 40, 95, 96, 7, 64 };
  switch (h_rmod(8)) { case 0: return 0x80000000u; case 1: return 0xffffffffu; case 2: return 0x100; default: return v[h_rmod(sizeof v / sizeof v[0])]; }
}
static unsigned g_n;   /* the max-count argument of the visual-diagnostics call of this trial (the counts in the object stay inside the buffers when it is above the buffer size) */
static unsigned pick_count(void) {
  if (g_n > (unsigned)OUTCAP) return h_rmod(OUTCAP + 1);
  return h_rpct(60) ? h_rmod(OUTCAP + 1) : h_rpct(30) ? h_rnd() : h_rmod(8);
}
static void new_world(void) {
  uint8_t *raw;
  V34Obj *o;
  int k, i;
  size_t osz = sizeof(V34Obj);
  memset(&g_iw, 0, sizeof g_iw);
  blk_reset();
  g_nobj = NID;
  for (k = 0; k < NID; k++) { g_obj[k] = NULL; g_idname[k] = bname[k]; }
  raw = (uint8_t *)h_obj_new(osz + CAN);
  memset(raw + osz, 0x5a, CAN);
  g_can = raw + osz;
  fill_tbl(raw, f_V34Obj, NF_V34Obj);
  reg(B_OBJ, raw, osz);
  newblk(B_P, sizeof(V34ModParObj)); newblk(B_P2, sizeof(V34ModParObj));
  newblk(B_K, sizeof(K56HandleObj)); newblk(B_K2, sizeof(K56HandleObj));
  newblk(B_INFO, INFOBLK);
  newblk(B_CTX, sizeof(PcmFlo47Obj)); newblk(B_CTX2, sizeof(PcmFlo47Obj));
  newblk(B_DEM, sizeof(DmdObj)); newblk(B_DEM2, sizeof(DmdObj));
  newblk(B_CD, 0x54); newblk(B_CD2, 0x54);
  newblk(B_MOD, sizeof(ModV92Obj)); newblk(B_MOD2, sizeof(ModV92Obj));
  newblk(B_B2S, sizeof(B2S92)); newblk(B_B2S2, sizeof(B2S92));
  newblk(B_TX, sizeof(TxObj)); newblk(B_TX2, sizeof(TxObj));
  newblk(B_IV, sizeof(P47V92InfoObj)); newblk(B_IV2, sizeof(P47V92InfoObj));
  newblk(B_COEF, COEFCAP * 4); newblk(B_COEF2, COEFCAP * 4);
  newblk(B_ECHO, SAMPCAP * 2); newblk(B_ECHO2, SAMPCAP * 2);
  newblk(B_PCM, SAMPCAP * 2); newblk(B_PCM2, SAMPCAP * 2);
  newblk(B_OUT, OUTBLK);
  newblk(B_DIAG, DIAGBLK);
  for (k = B_P; k <= B_P2; k++) {
    V34ModParObj *p = (V34ModParObj *)g_iw.b[k];
    fill_tbl((uint8_t *)p, f_V34ModParObj, NF_V34ModParObj);
    p->minRate_ = pick_rate(); p->maxRate_ = pick_rate();
    if (h_rpct(35)) { p->minRate_ = (p->maxRate_ / 2400u) * 2400u; if (h_rpct(50)) p->maxRate_ = p->minRate_; }
    p->delay_ = pick_delay(); p->extDelay_ = pick_delay();
    p->flags2_ = (uint8_t)(h_rpct(50) ? (h_rnd() & 0x7fu) : (h_rnd() | 0x80u));
    p->flags3_ = (uint8_t)h_rnd();
  }
  for (k = B_K; k <= B_K2; k++) {
    K56HandleObj *kk = (K56HandleObj *)g_iw.b[k];
    fill_tbl((uint8_t *)kk, f_K56HandleObj, NF_K56HandleObj);
    kk->flag8_ = (uint8_t)(h_rpct(40) ? 0 : h_rpct(60) ? 1 : h_rnd());
  }
  fill_blk(g_iw.b[B_INFO], INFOBLK);
  { uint16_t *inf = INFOP(&g_iw); if (h_rpct(50)) inf[7] = (uint16_t)(inf[7] | 0x20u); else if (h_rpct(60)) inf[7] = (uint16_t)(inf[7] & ~0x20u); }
  for (k = 0; k < 2; k++) {
    PcmFlo47Obj *c = (PcmFlo47Obj *)g_iw.b[B_CTX + k];
    ModV92Obj *m = (ModV92Obj *)g_iw.b[B_MOD + k];
    B2S92 *b = (B2S92 *)g_iw.b[B_B2S + k];
    TxObj *t = (TxObj *)g_iw.b[B_TX + k];
    P47V92InfoObj *iv = (P47V92InfoObj *)g_iw.b[B_IV + k];
    float *cf = (float *)g_iw.b[B_COEF + k];
    fill_tbl((uint8_t *)c, f_PcmFlo47Obj, NF_PcmFlo47Obj);
    c->flag6120_ = h_rpct(45) ? 0u : h_rpct(70) ? 1u : h_rnd();
    c->v92Session_ = h_rpct(45) ? 0u : h_rpct(70) ? 1u : h_rnd();
    c->v92Mod_ = g_iw.b[B_MOD + k]; c->v92Info_ = g_iw.b[B_IV + k]; c->v90Dem_ = g_iw.b[B_DEM + k]; c->ecCoef_ = g_iw.b[B_COEF + k];
    c->ecNCoef_ = h_rpct(30) ? h_rmod(4) : h_rpct(30) ? COEFCAP : h_rmod(COEFCAP + 1);
    fill_blk(m, sizeof *m); fill_blk(b, sizeof *b); fill_blk(t, sizeof *t); fill_blk(iv, sizeof *iv); fill_blk(g_iw.b[B_DEM + k], sizeof(DmdObj)); fill_blk(g_iw.b[B_CD + k], 0x54);
    m->state_ = h_rpct(50) ? 3u : h_rpct(70) ? h_rmod(3) : h_rnd();
    m->b2s_ = g_iw.b[B_B2S + k];
    b->tx_ = g_iw.b[B_TX + k];
    t->w0_ = h_rpct(25) ? h_rmod(300) : h_rpct(20) ? 0x80000000u + h_rmod(3) : h_rpct(30) ? 0xffffffffu - h_rmod(3) : h_rnd() >> h_rmod(32);
    iv->v92CapLocal_ = (uint8_t)(h_rpct(50) ? 1u : h_rpct(50) ? 0u : h_rnd());
    ((DmdObj *)g_iw.b[B_DEM + k])->cdes_ = g_iw.b[B_CD + k];
    for (i = 0; i < COEFCAP; i++) { uint32_t fb = pick_fbits(); memcpy(cf + i, &fb, 4); }
    fill_blk(g_iw.b[B_ECHO + k], SAMPCAP * 2);
    fill_blk(g_iw.b[B_PCM + k], SAMPCAP * 2);
  }
  fill_blk(g_iw.b[B_OUT], OUTBLK);
  fill_blk(g_iw.b[B_DIAG], DIAGBLK);
  o = (V34Obj *)raw;
  o->ctx_ = g_iw.b[B_CTX]; o->k56_ = g_iw.b[B_K]; o->params_ = g_iw.b[B_P];
  o->echoBuf_ = (int16_t *)g_iw.b[B_ECHO]; o->pcmBuf_ = (int16_t *)g_iw.b[B_PCM];
  g_n = pick_n();
  o->echoCnt_ = pick_count(); o->pcmCnt_ = pick_count();
  o->constCnt_ = (int16_t)(g_n > (unsigned)OUTCAP ? (int)h_rmod(OUTCAP + 1) : (h_rpct(70) ? (int)h_rmod(OUTCAP + 1) : (int)(int16_t)h_rnd()));
  o->mode_ = pick_mode3(); o->state_ = (int32_t)wsel();
  o->pcmMode_ = pick_mode3(); o->k56Mode_ = pick_mode3();
  o->modeChar_ = pick_modechar();
  o->wabc8_ = (int16_t)pick_h16(); o->waa98_ = (int16_t)(h_rpct(30) ? (int)h_rmod(24) - 3 : (int)(int16_t)pick_h16()); o->waa88_ = (int16_t)(h_rpct(30) ? (int)h_rmod(24) - 3 : (int)(int16_t)pick_h16());
  o->w25dc_ = pick_h16(); o->b_ac00_ = (uint8_t)h_rnd();
  o->w47e_ = (int16_t)(h_rpct(55) ? 1 + (int)h_rmod(120) : h_rpct(40) ? (int)(int16_t)pick_h16() : 0x7fff);
  o->w4ac_ = (int32_t)(h_rpct(25) ? (int32_t)h_rmod(5000) : h_rpct(25) ? (int32_t)h_rnd() : h_rpct(20) ? (int32_t)(h_rnd() >> h_rmod(32)) : h_rpct(30) ? (int32_t)0x7fffffff - (int32_t)h_rmod(3) : (int32_t)(0x80000000u + h_rmod(3)));
  o->baud_ = pick_h16(); o->dgC0c_ = (int16_t)(h_rpct(30) ? 0u : h_rnd());
  o->minRateIdx_ = (int32_t)wsel(); o->maxRateIdx_ = (int32_t)wsel(); o->w359a_ = pick_h16();
  g_obj[X_EC] = (uint8_t *)g_iw.b[B_CTX] + offsetof(PcmFlo47Obj, ecParams_);
  g_obj[X_EC2] = (uint8_t *)g_iw.b[B_CTX2] + offsetof(PcmFlo47Obj, ecParams_);
  g_obj[X_ANS] = (uint8_t *)g_iw.b[B_CTX] + offsetof(PcmFlo47Obj, ansIir_);
  g_obj[X_ANS2] = (uint8_t *)g_iw.b[B_CTX2] + offsetof(PcmFlo47Obj, ansIir_);
  blk_sync_model();
}
static void world_model(void) { int k; for (k = 0; k < NBLK; k++) g_mw.b[k] = g_blk[g_blkidx[k]].m; }

/* ------------------------------------------------------------------------------------------------ one call */
static void check_common(const char *what) {
  unsigned i;
  log_cmp(&g_ml, &g_il, what);
  if (memcmp(&g_mc, &g_ic, sizeof g_mc)) h_die3("%s: the script cursors differ", what);
  blk_compare(what);
  for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", what, i);
}
static void dump_world(int show) {
  char snap[TXTMAX * 2];
  obj_snapshot(snap, sizeof snap, 0);
  h_emit(show, "world %s", snap);
  dump_sliced(show, "D", g_iw.b[B_OBJ], f_V34Obj, NF_V34Obj);
  h_dump_obj(show, "P", g_iw.b[B_P], f_V34ModParObj, NF_V34ModParObj);
  h_dump_obj(show, "K", g_iw.b[B_K], f_K56HandleObj, NF_K56HandleObj);
  h_dump_obj(show, "G", DIAGP(&g_iw), f_DiagV03Obj, NF_DiagV03Obj);
  h_emit(show, "info %08x out %08x diagblk %08x", (unsigned)fnv_upd(2166136261u, g_iw.b[B_INFO], INFOBLK), (unsigned)fnv_upd(2166136261u, g_iw.b[B_OUT], OUTBLK), (unsigned)fnv_upd(2166136261u, g_iw.b[B_DIAG], DIAGBLK));
}
static void do_call(int op) {
  uint8_t *t = g_iw.b[B_OBJ];
  unsigned i, lv0, a1 = 0, a2 = 0, a3 = 0, sel = 0, n = 0;
  int ri = 0, rm = 0;
  blk_sync_model();
  world_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  g_what = opname[op];
  g_snapfn = obj_snapshot;
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  if (op == OP_MOH) {
    static const unsigned v1[] = { 0, 1, 1, 1, 2, 3, 0xffffffffu, 0x100, 4 };
    a1 = h_rpct(85) ? v1[h_rmod(sizeof v1 / sizeof v1[0])] : h_rnd();
    a2 = (h_rpct(50) ? 0u : h_garbage_hi(0)) | (h_rpct(40) ? 0u : h_rpct(50) ? 1u : h_rmod(256));
    a3 = (h_rpct(50) ? 0u : h_garbage_hi(0)) | h_rmod(256);
  }
  if (op == OP_VIS) {
    static const unsigned sv[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 5, 9, 10, 0x80000000u, 0xffffffffu, 8u };
    sel = h_rpct(95) ? h_rmod(9) : sv[h_rmod(sizeof sv / sizeof sv[0])];
    n = h_rpct(25) ? g_n : pick_n();
    if (n > (unsigned)OUTCAP) {   /* the counts of the object must stay inside the buffers whatever the selector */
      V34Obj *ob = OBJ(&g_iw);
      if (ob->echoCnt_ > (unsigned)SAMPCAP) ob->echoCnt_ = h_rmod(SAMPCAP + 1);
      if (ob->pcmCnt_ > (unsigned)SAMPCAP) ob->pcmCnt_ = h_rmod(SAMPCAP + 1);
      if ((int16_t)ob->constCnt_ < 0 || (int16_t)ob->constCnt_ > OUTCAP) ob->constCnt_ = (int16_t)h_rmod(OUTCAP + 1);
      blk_sync_model();
    }
  }
  lv0 = dsplibs_debug_level;
  h_emit(g_show, "call %s a=%u,%u,%u sel=%u n=%u lvl=%u", g_what, a1, a2, a3, sel, n, dsplibs_debug_level);
  switch (op) {
    case OP_GIVE: ri = f_give(t, INFOP(&g_iw)); break;
    case OP_IND: f_ind(t); break;
    case OP_RXBR: ri = f_rxbr(t); break;
    case OP_MOH: f_moh(t, a1, a2, a3); break;
    case OP_MINMAX: f_minmax(t); break;
    case OP_DELAYS: f_delays(t); break;
    case OP_VIS: ri = (int)f_vis(t, sel, OUTP(&g_iw), n); break;
    default: ri = f_diag(t, DIAGP(&g_iw)); break;
  }
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  dsplibs_debug_level = lv0;
  switch (op) {
    case OP_GIVE: rm = m_give(INFOP(MW)); break;
    case OP_IND: m_ind(); break;
    case OP_RXBR: rm = m_rxbr(); break;
    case OP_MOH: m_moh(a1, a2, a3); break;
    case OP_MINMAX: m_minmax(); break;
    case OP_DELAYS: m_delays(); break;
    case OP_VIS: rm = (int)m_vis(sel, OUTP(&g_iw), n); break;
    default: rm = m_diag(DIAGP(&g_iw)); break;
  }
  check_common(g_what);
  if (ri != rm) h_die3("%s: the model returns %d, the impl %d", g_what, rm, ri);
  if (op == OP_GIVE || op == OP_RXBR || op == OP_VIS || op == OP_DIAG) h_emit(g_show, "ret %d", ri);
  dump_world(g_show);
  cov_flush();
  g_ncalls++;
}

/* ------------------------------------------------------------------------------------------------ the trials */
static unsigned pick_lv(void) { return h_rpct(65) ? 2u + h_rmod(2) : pick_level(); }
static int pick_op(int sc) {
  switch (sc) {
    case G_GIVE: return OP_GIVE;
    case G_MISC: return OP_IND + (int)h_rmod(3);
    case G_RATE: return OP_MINMAX + (int)h_rmod(2);
    case G_VIS: return OP_VIS;
    case G_DIAG: return OP_DIAG;
    default: return (int)h_rmod(NOPS);
  }
}
static void trial(int sc) {
  int k, nm;
  new_world();
  gen_script();
  dsplibs_debug_level = pick_lv();
  g_pokemask = sc == G_POKE ? 0xfffffffeu : sc == G_SEQ ? (1u << 1) : 0;
  if (sc != G_SEQ) do_call(pick_op(sc));
  else {
    nm = (int)rnd_range(2, 6);
    for (k = 0; k < nm; k++) {
      if (k) { gen_script(); if (h_rpct(15)) dsplibs_debug_level = pick_lv(); }
      do_call(pick_op(sc));
    }
  }
  h_trial_end("v03");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("V03_SEED");
  CovSite *cv;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("V03_ALL")) g_showall = 1;
  if (sc < 0 || sc > 6) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  p29_layout_check(); p39_layout_check(); p43_layout_check(); p47_layout_check(); v01_layout_check(); v03_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("V03_TRIALS") ? atoi(getenv("V03_TRIALS")) : (sc == G_VIS ? 20000 : sc == G_SEQ ? 4000 : 12000);
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (cv = __start_covsites; cv < __stop_covsites; cv++) {
    if (cv->grp != sc) continue;
    h_emit(1, "COV %s %lu", cv->name, cv->n);
    if (cv->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", cv->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("V03_NOCOV")) ? 2 : 0;
}
