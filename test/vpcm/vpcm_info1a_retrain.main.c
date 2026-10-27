/*
 * Differential test harness for port chunk V01 (src/vpcm/vpcm_info1a_retrain.cpp): V34SetINFO1aBits and VPcmV34InitiateRetrain, the first two blocked V.34 functions (plain C symbols; the port is
 * extern "C" C++ because they call methods of the V.90 classes).  Derived from test/vpcm/vpcm_flo_modem.main.c (recorders + independent model + per-record world snapshot + pokes).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/vpcm_info1a_retrain.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2: the
 * port object alone) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/vpcm_info1a_retrain.blob32.out).  The functions are reached through __asm__ labels; the V.34 object is the generated C mirror
 * (test/v34/v01.layout: V34Obj, 0xac4c B on i386); the foreign blocks are the dp parameter block (V34ModParObj), the K56Flex handle (K56HandleObj), the VPcmFloModem ctx (P47's PcmFlo47Obj), its V90Demodulator (P43's
 * DmdObj) and V90Parameters (P07's V90ParObj).
 *
 * Every callee is a RECORDER: bitreverse, VPcmFloModem::getUinfoValue / setPcmSessionType, V92EchoCanceller::setEchoDelay, K56FlexFloModem::setMinMaxRates, V90ConstellationDesigner::setMinMaxRates,
 * v34handshakinit, edprintf, dsplibs_debug_printf.  The record carries the arguments as tokens and a SNAPSHOT of the whole world (hashed); a recorder then applies its scripted RESULT (random 32 bits with garbage
 * in the upper half where the blob reads a 16-bit result: bitreverse / getUinfoValue) and may POKE the world by script (debug level, the three pointer slots of the object, the words the function reloads): the time
 * at which the blob / port reloads a member is observable.  The INDEPENDENT MODEL (m_*: written from porting/decompiled/dp/v01_v34_info1a_retrain.txt in a different shape than the port) runs AFTER the implementation on
 * copies of the world, through the same environment functions, and must reproduce every record, the returned value and every block (exit 3 otherwise).
 *
 * Scenarios (own process each): 0 V34SetINFO1aBits (no pokes), 1 VPcmV34InitiateRetrain (no pokes), 2 both with every poke kind enabled, 3 sequences of calls on one object (the state one call leaves is the
 * next one's input) with level pokes only.
 */

#include "../v34/v01_common.h"
#include "pcm_parameters_layout.h"
#include "v90_demodulator_progress_layout.h"
#include "vpcm_flo_modem_layout.h"
#include "../v34/v01_layout.h"

/* ---- the functions under test ---- */
extern int f_info1a(void *, uint16_t *) L("V34SetINFO1aBits");
extern void f_retrain(void *, unsigned) L("VPcmV34InitiateRetrain");

void *h_sysdep_malloc(unsigned size);
void h_sysdep_free(void *q);
void *sysdep_malloc(unsigned size) { return h_sysdep_malloc(size); }
void sysdep_free(void *q) { h_sysdep_free(q); }

/* ------------------------------------------------------------------------------------------------ the world */
enum { B_OBJ, B_P, B_P2, B_K, B_K2, B_INFO, B_CTX, B_CTX2, B_DEM, B_DEM2, B_PAR, B_PAR2, B_CD, B_CD2, NBLK };
enum { X_EC = NBLK, X_EC2, NID };
static const char *const bname[NID] = { "obj", "P", "P2", "k56", "k56b", "info", "ctx", "ctx2", "dem", "dem2", "par", "par2", "cd", "cd2", "ctx.ec_", "ctx2.ec_" };
typedef struct { uint8_t *b[NBLK]; } W;
static W g_iw, g_mw;
static int g_blkidx[NBLK];
static uint8_t *g_can;
#define OBJ(w) ((V34Obj *)(w)->b[B_OBJ])
#define DBG() (dsplibs_debug_level > 1u)
enum { INFOW = 32, GUARD = 16 };   /* the info record: 32 words, only words 0 .. 9 are used (the rest must stay untouched); GUARD random words before and after it (a store / load outside the record is a model violation) */
#define INFOBLK ((INFOW + 2 * GUARD) * 2)
#define INFOP(w) ((uint16_t *)(w)->b[B_INFO] + GUARD)

enum { G_INFO = 0, G_RET = 1, G_POKE = 2, G_SEQ = 3 };

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
    g_sc.kind[i] = h_rpct(70) ? 0u : 1u + h_rmod(13);
    g_sc.val[i] = h_rnd();
    g_sc.res[i] = h_rnd();
  }
}
static uint32_t pick_w(uint32_t v) { switch (v & 7u) { case 0: return 0; case 1: return 1; case 2: return 0xffffffffu; case 3: return 0x80000000u; case 4: return 0x10000u; default: return v >> 3; } }
static uint16_t pick_h(uint32_t v) { switch (v & 7u) { case 0: return 0; case 1: return 1; case 2: return 0xffffu; case 3: return 0x65; case 4: return 0x66; default: return (uint16_t)(v >> 3); } }
/* the pokes the recorders apply AFTER recording (the same function runs in the impl's and the model's world) */
static void poke(W *w, Cur *c) {
  unsigned k = c->pk++ % NSC, kind = g_sc.kind[k], v = g_sc.val[k];
  V34Obj *o = OBJ(w);
  if (!kind || !((g_pokemask >> kind) & 1u)) return;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->ctx_ = (v & 8) ? g_iw.b[B_CTX2] : g_iw.b[B_CTX]; break;
    case 3: o->k56_ = (v & 8) ? g_iw.b[B_K2] : g_iw.b[B_K]; break;
    case 4: o->params_ = (v & 8) ? g_iw.b[B_P2] : g_iw.b[B_P]; break;
    case 5: switch ((v >> 3) % 3u) { case 0: o->pcmMode_ = (int32_t)(v >> 6 & 3u) - (int32_t)(v >> 8 & 1u); break; case 1: o->k56Mode_ = (int32_t)(v >> 6 & 3u); break; default: o->mode_ = (int32_t)(v >> 6 & 3u); } break;
    case 6: switch ((v >> 3) % 7u) { case 0: o->shortPhase2_ = pick_h(v >> 6); break; case 1: o->w35a4_ = pick_h(v >> 6); break; case 2: o->pair0_ = pick_h(v >> 6); break;
        case 3: o->pair1_ = pick_h(v >> 6); break; case 4: o->pair2_ = pick_h(v >> 6); break; case 5: o->modeChar_ = pick_h(v >> 6); break; default: o->pcmMode_ = (int32_t)pick_w(v >> 6); } break;
    case 7: { V34ModParObj *p = (V34ModParObj *)P(w, g_iw.b[(v & 8) ? B_P2 : B_P]);
      switch ((v >> 4) % 7u) { case 0: p->level_ = (int32_t)pick_w(v >> 8); break; case 1: p->delay_ = (int32_t)pick_w(v >> 8); break; case 2: p->extDelay_ = (int32_t)pick_w(v >> 8); break;
        case 3: p->flags0_ = (uint8_t)(v >> 8); break; case 4: p->flags50_ = (uint8_t)(v >> 8); break; case 5: p->minRate_ = pick_w(v >> 8); break; default: p->maxRate_ = pick_w(v >> 8); } break; }
    case 8: { PcmFlo47Obj *cx = (PcmFlo47Obj *)P(w, g_iw.b[(v & 8) ? B_CTX2 : B_CTX]);
      switch ((v >> 4) % 3u) { case 0: cx->flag6120_ = pick_w(v >> 8); break; case 1: cx->v92Session_ = pick_w(v >> 8); break; default: cx->qcStatus_ = pick_w(v >> 8); } break; }
    case 9: { K56HandleObj *kk = (K56HandleObj *)P(w, g_iw.b[(v & 8) ? B_K2 : B_K]); if (v & 16) kk->word_c_ = (int32_t)pick_w(v >> 8); else kk->flag8_ = (uint8_t)(v >> 8); break; }
    case 10: ((uint16_t *)P(w, g_iw.b[B_INFO]) + GUARD)[(v >> 3) % 10u] = (uint16_t)(v >> 8); break;
    case 11: ((V90ParObj *)P(w, g_iw.b[(v & 8) ? B_PAR2 : B_PAR]))->sensitiveIspDetected = (int32_t)pick_w(v >> 4); break;
    case 12: { PcmFlo47Obj *cx = (PcmFlo47Obj *)P(w, g_iw.b[(v & 8) ? B_CTX2 : B_CTX]);
      switch ((v >> 4) % 3u) { case 0: cx->v90Params_ = (v & 64) ? g_iw.b[B_PAR2] : g_iw.b[B_PAR]; break; case 1: cx->v90Dem_ = (v & 64) ? g_iw.b[B_DEM2] : g_iw.b[B_DEM]; break;
        default: ((DmdObj *)P(w, g_iw.b[(v & 64) ? B_DEM2 : B_DEM]))->cdes_ = (v & 128) ? g_iw.b[B_CD2] : g_iw.b[B_CD]; } break; }
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
  uint32_t h = hash_tbl(2166136261u, w->b[B_OBJ], f_V34Obj, NF_V34Obj), f = 2166136261u;
  int k;
  for (k = 0; k < 2; k++) {
    const DmdObj *d = (const DmdObj *)w->b[B_DEM + k];
    int id = tok_of(d->cdes_);
    f = hash_tbl(f, w->b[B_P + k], f_V34ModParObj, NF_V34ModParObj);
    f = hash_tbl(f, w->b[B_K + k], f_K56HandleObj, NF_K56HandleObj);
    f = hash_tbl(f, w->b[B_CTX + k], f_PcmFlo47Obj, NF_PcmFlo47Obj);
    f = fnv_upd(f, &id, sizeof id);
    f = fnv_upd(f, &((const V90ParObj *)w->b[B_PAR + k])->sensitiveIspDetected, 4);
  }
  f = fnv_upd(f, w->b[B_INFO], INFOBLK);
  snprintf(out, cap, "|h=%08x f=%08x lvl=%u", h, f, dsplibs_debug_level);
}

/* ------------------------------------------------------------------------------------------------ the callee environment: record, effect, poke (the impl's recorders and the model call the same functions) */
static const char *nm01(int id) {
  static char b[40][24];
  static int n;
  char *o = b[n++ % 40];
  if (id >= 0 && id < g_nobj) return g_idname[id];
  if (id == ID_NULL) return "null";
  if (id == ID_WILD) return "wild";
  snprintf(o, 24, "A%d+%d", (id - 100) / 4096, (id - 100) % 4096);
  return o;
}
#define T_(p) nm01(tok_of(p))
#define ENVREC(w, c, l, ...) do { rec_add((l), (w) == &g_mw, 'c', __VA_ARGS__); } while (0)

static void env_ed(W *w, Cur *c, Log *l, char kind, const char *s) { rec_add(l, w == &g_mw, kind, "%s", s); poke(w, c); }
/* bitreverse(x, n): the blob passes zero-extended 16-bit words (movzwl) and small constants; the result is read as a SIGNED 16-bit value (cwtl): garbage above bit 15 */
static unsigned env_bitrev(W *w, Cur *c, Log *l, unsigned x, unsigned n) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "bitreverse(x=%u n=%u)", x & 0xffffu, n & 0xffffu);
  poke(w, c);
  return (r & 3u) ? r : (r & 0xffffu);
}
static int env_uinfo(W *w, Cur *c, Log *l, void *self, int x) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "getUinfoValue(this=%s x=%d)", T_(self), (int)(int16_t)x);
  poke(w, c);
  return (int)((r & 3u) == 0 ? 0u : (r & 4u) ? (r & 0xffffu) : r);   /* 0, a 16-bit value, or garbage in the upper half (the blob does cwtl) */
}
static void env_sess(W *w, Cur *c, Log *l, void *self, int s) { ENVREC(w, c, l, "setPcmSessionType(this=%s %d)", T_(self), s); poke(w, c); }
static void env_echodelay(W *w, Cur *c, Log *l, void *self, unsigned d) { ENVREC(w, c, l, "setEchoDelay(this=%s %u)", T_(self), d); poke(w, c); }
static void env_k56mm(W *w, Cur *c, Log *l, void *self, int a, int b) { ENVREC(w, c, l, "K56.setMinMaxRates(this=%s %d %d)", T_(self), a, b); poke(w, c); }
static void env_cdmm(W *w, Cur *c, Log *l, void *self, unsigned a, unsigned b) { ENVREC(w, c, l, "cd.setMinMaxRates(this=%s %u %u)", T_(self), a, b); poke(w, c); }
static int env_hs(W *w, Cur *c, Log *l, void *obj, unsigned mode) {
  uint32_t r = sres(c);
  ENVREC(w, c, l, "v34handshakinit(obj=%s %u)", T_(obj), mode);
  poke(w, c);
  return (int)r;
}

/* ------------------------------------------------------------------------------------------------ the recorders (the blob's callees, weakened in the copy of dsplibs.o) */
extern unsigned rc_bitrev(unsigned, unsigned) L("bitreverse");
extern int rc_uinfo(void *, int) L("_ZN12VPcmFloModem13getUinfoValueEs");
extern void rc_sess(void *, int) L("_ZN12VPcmFloModem17setPcmSessionTypeEi");
extern void rc_echodelay(void *, unsigned) L("_ZN16V92EchoCanceller12setEchoDelayEj");
extern void rc_k56mm(void *, int, int) L("_ZN15K56FlexFloModem14setMinMaxRatesEii");
extern void rc_cdmm(void *, unsigned, unsigned) L("_ZN24V90ConstellationDesigner14setMinMaxRatesEjj");
extern int rc_hs(void *, unsigned) L("v34handshakinit");
unsigned rc_bitrev(unsigned x, unsigned n) { return env_bitrev(IW, IC, IL, x, n); }
int rc_uinfo(void *t, int x) { return env_uinfo(IW, IC, IL, t, x); }
void rc_sess(void *t, int s) { env_sess(IW, IC, IL, t, s); }
void rc_echodelay(void *t, unsigned d) { env_echodelay(IW, IC, IL, t, d); }
void rc_k56mm(void *t, int a, int b) { env_k56mm(IW, IC, IL, t, a, b); }
void rc_cdmm(void *t, unsigned a, unsigned b) { env_cdmm(IW, IC, IL, t, a, b); }
int rc_hs(void *o, unsigned m) { return env_hs(IW, IC, IL, o, m); }
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

/* ------------------------------------------------------------------------------------------------ the independent model (from porting/decompiled/dp/v01_v34_info1a_retrain.txt) */
static void mpf(char kind, const char *fmt, ...) {
  char b[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  env_ed(MW, MC, ML, kind, b);
}
#define MDBG(...) do { if (DBG()) mpf('d', __VA_ARGS__); } while (0)
static V34Obj *mo(void) { return OBJ(MW); }
static void *mp(const void *p) { return P(MW, p); }
static int sx(unsigned v) { return (int)(int16_t)(uint16_t)v; }       /* cwtl */
static unsigned rev(unsigned x, unsigned n) { return env_bitrev(MW, MC, ML, x, n); }
static uint16_t bits_or(uint16_t w, unsigned add) { return (uint16_t)(w | add); }

/* INFO1a bit packers: the two shapes the function uses, `top` = bits 6:5 into word 0 and bits 4:0 into word 1, `mid` = bits 6:4 / 3:0 */
static void put_top(uint16_t *inf, int r, int store_first_word1) {
  if (store_first_word1) { inf[1] = (uint16_t)(((unsigned)r << 3) & 0xf8u); inf[0] = (uint16_t)((r >> 5) & 3); }
  else { inf[0] = bits_or(inf[0], (unsigned)((r >> 5) & 3)); inf[1] = bits_or(inf[1], ((unsigned)r << 3) & 0xf8u); }
}
static void put_mid(uint16_t *inf, int r) { inf[1] = bits_or(inf[1], (unsigned)((r >> 4) & 7)); inf[2] = (uint16_t)(((unsigned)r << 4) & 0xf0u); }
static void put_low(uint16_t *inf, int r) { inf[2] = bits_or(inf[2], (unsigned)((r >> 1) & 3)); inf[3] = (uint16_t)(((unsigned)r << 7) & 0x80u); }

static int m_info1a(uint16_t *inf) {
  V34Obj *o = mo();
  int pcm = o->pcmMode_, k56mode = o->k56Mode_;
  K56HandleObj *k56 = (K56HandleObj *)mp(o->k56_);
  void *ctxI = o->ctx_;
  PcmFlo47Obj *ctx = (PcmFlo47Obj *)mp(ctxI);
  /* (1) neither mode word set: the plain V.34 INFO1a request value goes into the first two words */
  if (pcm == 0 && k56mode == 0) {
    CV(G_INFO, "info1a: plain V.34 request (both mode words 0)");
    put_top(inf, sx(rev(o->w35a4_, 7)), 0);
    k56mode = o->k56Mode_;
  } else if (pcm != 0) CV(G_INFO, "info1a: pcmMode != 0 at entry"); else CV(G_INFO, "info1a: k56Mode != 0 at entry");
  /* (2) K56Flex request */
  if (k56mode != 0) {
    if (o->modeChar_ == 0x65) {
      CV(G_INFO, "info1a: K56Flex caller-analog (INFO1c)");
      MDBG("V34SetINFO1aBits: Setting INFO1c for K56Flex (Caller-Analog)...\r\n");
      put_top(inf, sx(rev(o->w35a4_, 7)), 0);
      { unsigned t = inf[8] & 0xfff1u; inf[8] = (uint16_t)(t | (k56->word_c_ == 1 ? 0x13u : 0x11u)); if (k56->word_c_ == 1) CV(G_INFO, "info1a: INFO1c k56 word == 1"); else CV(G_INFO, "info1a: INFO1c k56 word != 1"); }
      inf[9] = (uint16_t)(inf[9] & 0x3ffu);
    } else {
      CV(G_INFO, "info1a: K56Flex answer-digital (INFO1a)");
      MDBG("V34SetINFO1aBits: Setting INFO1a for K56Flex (Answer-Digital)...\r\n");
      { unsigned t = inf[3] & 0xfff9u; if (k56->word_c_ != 0) { CV(G_INFO, "info1a: INFO1a k56 word != 0"); inf[3] = (uint16_t)(t | 9u); } else { CV(G_INFO, "info1a: INFO1a k56 word == 0"); inf[3] = (uint16_t)(t | 0xbu); } }
    }
  }
  /* (3) the V.90 / V.92 PCM part */
  if (o->pcmMode_ == 0) { CV(G_INFO, "info1a: pcm mode off: return"); return 0; }
  if (ctx->flag6120_ == 0) {
    CV(G_INFO, "info1a: INFO1d (role flag 0)");
    MDBG("VPcmV34Main, setINFO1dBits\n");
    if (ctx->v92Session_ == 0) { CV(G_INFO, "info1a: INFO1d session 0"); inf[7] = (uint16_t)(inf[7] & 0xdfu); } else { CV(G_INFO, "info1a: INFO1d session != 0"); inf[7] = (uint16_t)(inf[7] | 0x20u); }
  } else {
    unsigned s;
    int u, bp = 4;
    CV(G_INFO, "info1a: INFO1a (role flag != 0)");
    MDBG("VPcmV34Main, setINFO1aBits\n");
    s = o->shortPhase2_;
    if (s != 0) {
      CV(G_INFO, "info1a: short phase 2");
      if (DBG()) { mpf('d', "VPcmV34Main: Short phase2 (Hard Coded) !!!\r\n"); s = o->shortPhase2_; CV(G_INFO, "info1a: short phase 2 print"); }
      ctx->v92Session_ = 0;
      o->baud_ = 0xc80; o->carrier_ = 0x725; o->preemp_ = 0; o->w25dc_ = 0;
    }
    u = sx((unsigned)env_uinfo(MW, MC, ML, ctxI, (short)s));
    if (u == 0) { CV(G_INFO, "info1a: no uinfo value: pcm mode cleared"); o->pcmMode_ = 0; return 0; }
    if (ctx->v92Session_ != 0) {
      uint16_t e = 0;
      CV(G_INFO, "info1a: PCM upstream (V.92 session)");
      MDBG("VPcmV34Main: PCM Upstream is selected (info1)...\r\n");
      inf[0] = 0;
      e |= (o->pair0_ & 1u) ? 0x80u : 0u; e |= (o->pair0_ & 2u) ? 0x40u : 0u;
      e |= (o->pair1_ & 1u) ? 0x20u : 0u; e |= (o->pair1_ & 2u) ? 0x10u : 0u;
      e |= (o->pair2_ & 1u) ? 0x08u : 0u; e |= (o->pair2_ & 2u) ? 0x04u : 0u;
      inf[0] = e;
      inf[1] = 0;
      put_mid(inf, sx(rev((uint16_t)u, 7)));
      put_low(inf, sx(rev(6, 3)));
      { int r = sx(rev(6, 3)); inf[4] = 0xfc; inf[3] = (uint16_t)(inf[3] | (((unsigned)r << 4) & 0x70u) | 0xfu); }
    } else {
      CV(G_INFO, "info1a: V.34 upstream (session 0)");
      MDBG("VPcmV34Main: V.34 Upstream is selected (info1)...\r\n");
      if (o->shortPhase2_ == 0) { CV(G_INFO, "info1a: V.34 upstream: long phase 2 (bit pair re-read)"); bp = sx(rev((((unsigned)inf[2] & 3u) << 1) | (((unsigned)inf[3] & 0x80u) >> 7), 3)); }
      put_top(inf, sx(rev(o->w35a4_, 7)), 1);
      put_mid(inf, sx(rev((uint16_t)u, 7)));
      put_low(inf, sx(rev((uint16_t)bp, 3)));
      { int r = sx(rev(6, 3)); inf[4] = 0; inf[3] = (uint16_t)(inf[3] | (((unsigned)r << 4) & 0x70u)); }
      if (o->shortPhase2_ != 0) { CV(G_INFO, "info1a: V.34 upstream: short phase 2 flag in word 4"); inf[4] = 1; }
      MDBG("VPcmV34Main: It is ===== V90 receiver ==== from Now on\n");
    }
  }
  o->pcmMode_ = 2;
  return 0;
}

static const int k_thresh[8] = { 71, 80, 90, 101, 113, 127, 142, 160 };   /* V34DisconnectThreshTable (the model's own copy) */
static int m_clamp14(int v) { return v > 14 ? 14 : v; }
static void m_retrain(unsigned arg) {
  V34Obj *o = mo();
  unsigned mode = arg & 0xffu;
  void *ctx1I = o->ctx_, *ctx3I, *k56I;
  PcmFlo47Obj *ctx1 = (PcmFlo47Obj *)mp(ctx1I), *ctx2, *ctx3;
  K56HandleObj *k56;
  unsigned lvl;
  int pcm;
  V34ModParObj *p;
  MDBG("VPcmV34Main: Initiating retrain, requested DP is %d\r\n", (int)mode);
  pcm = o->pcmMode_;
  ctx2 = (PcmFlo47Obj *)mp(o->ctx_);
  k56I = o->k56_;
  k56 = (K56HandleObj *)mp(k56I);
  /* the rate limits go to the constellation designer / the K56Flex modem, or are turned into the two rate indices */
  p = (V34ModParObj *)mp(o->params_);
  if (pcm != 0 && ctx2->flag6120_ != 0) {
    DmdObj *dem = (DmdObj *)mp(ctx2->v90Dem_);
    CV(G_RET, "retrain: V.90 designer rates");
    env_cdmm(MW, MC, ML, dem->cdes_, p->minRate_, p->maxRate_);
  } else if (o->k56Mode_ != 0 && k56->flag8_ != 0) {
    CV(G_RET, "retrain: K56Flex modem rates");
    env_k56mm(MW, MC, ML, k56I, (int)p->minRate_, (int)p->maxRate_);
  } else {
    int lo, hi;
    CV(G_RET, "retrain: V.34 rate indices");
    if (pcm != 0) CV(G_RET, "retrain: V.34 indices with pcm mode and role flag 0"); else CV(G_RET, "retrain: V.34 indices with pcm mode 0");
    if (o->k56Mode_ != 0) CV(G_RET, "retrain: V.34 indices with k56 mode but flag byte 0");
    lo = (int)(p->minRate_ / 2400u); hi = (int)(p->maxRate_ / 2400u);
    o->minRateIdx_ = lo; o->maxRateIdx_ = hi;
    if (lo > 14) { CV(G_RET, "retrain: min index clamped"); o->minRateIdx_ = 14; }
    if (o->maxRateIdx_ < o->minRateIdx_) { CV(G_RET, "retrain: max index raised to min"); o->maxRateIdx_ = o->minRateIdx_; }
    if (o->maxRateIdx_ > 14) { CV(G_RET, "retrain: max index clamped"); o->maxRateIdx_ = m_clamp14(o->maxRateIdx_); }
    else if (o->maxRateIdx_ == 1) { CV(G_RET, "retrain: max index 1"); o->w359a_ = 1; }
  }
  /* the minimum signal level, the filter delay and the dma delay */
  p = (V34ModParObj *)mp(o->params_);
  { unsigned idx = (unsigned)p->level_ + 0x30u; if (idx > 7u) { CV(G_RET, "retrain: level index out of range"); idx = 3; } else CV(G_RET, "retrain: level index in range"); lvl = dsplibs_debug_level; o->minSigLevel_ = k_thresh[idx];
    if (lvl > 1u) { CV(G_RET, "retrain: minLevel print"); mpf('d', "VPcmV34Main: minLevel given is %d , minSigLevel set to %d\n", p->level_, k_thresh[idx]); } }
  p = (V34ModParObj *)mp(o->params_);
  o->w234_ = 0;
  lvl = dsplibs_debug_level;
  ctx3I = o->ctx_;
  ctx3 = (PcmFlo47Obj *)mp(ctx3I);
  o->filtDelay_ = (uint16_t)((((int32_t)(p->delay_ + 2) >> 2) + 0x22));
  if (lvl > 1u) { CV(G_RET, "retrain: filtdelay print"); mpf('d', "V34 filtdelay set to %d (params initial delay = %d)\n", (int)(int16_t)o->filtDelay_, p->delay_); lvl = dsplibs_debug_level; }
  p = (V34ModParObj *)mp(o->params_);
  o->dmaDelay_ = (uint16_t)(0x610 - p->extDelay_);
  if (lvl > 1u) { CV(G_RET, "retrain: dma delay print"); mpf('d', "V34FEC, V34dmadelay set to %d, (ext delay=%d)\n", (int)(int16_t)o->dmaDelay_, p->extDelay_); }
  env_echodelay(MW, MC, ML, (uint8_t *)ctx3I + offsetof(PcmFlo47Obj, ecParams_), (unsigned)(((V34ModParObj *)mp(o->params_))->extDelay_ + 0x68));
  /* the sensitive ISP notification */
  if (((V90ParObj *)mp(ctx1->v90Params_))->sensitiveIspDetected != 0) {
    CV(G_RET, "retrain: sensitive ISP");
    mpf('e', "VPcmV34Main: Notifying Sensitive ISP detected...\r\n");
    ((V34ModParObj *)mp(o->params_))->flags50_ |= 8;
  }
  pcm = o->pcmMode_;
  o->shortPhase2_ = 0; o->wabca_ = 0;
  ctx1->qcStatus_ = 0;
  /* the validation of the requested modulation (only with the pcm mode word clear) */
  if (pcm == 0 && mode != 0) {
    int ok = 1;
    if (mode == 0x22) CV(G_RET, "retrain: requested mod 0x22");
    else if (mode == 0x38) {
      if (((V34ModParObj *)mp(o->params_))->flags0_ & 0x10) CV(G_RET, "retrain: K56Flex allowed"); else { ok = 0; CV(G_RET, "retrain: K56Flex not allowed"); MDBG("VPcmV34InitiateRetrain: requested mod K56Flex, but params.flex is OFF, keeping same modulation!!!\r\n"); }
    } else if (mode == 0x5a || mode == 0x5c) {
      if (((V34ModParObj *)mp(o->params_))->flags0_ & 8) CV(G_RET, "retrain: V.90 / V.92 allowed"); else { ok = 0; CV(G_RET, "retrain: V.90 / V.92 not allowed"); MDBG("VPcmV34InitiateRetrain: requested mod V.%d, but params.v90 is OFF, keeping same modulation!!!\r\n", (int)mode); }
    } else { ok = 0; CV(G_RET, "retrain: unknown requested mod"); MDBG("VPcmV34InitiateRetrain: unknown requested mod (%d), keeping same modulation!!!\r\n", (int)mode); }
    if (!ok) mode = 0;
  }
  /* the mode words and the session type */
  switch (mode) {
    case 0x38: CV(G_RET, "retrain: switch to K56Flex"); o->k56Mode_ = 1; o->pcmMode_ = 0; break;
    case 0: CV(G_RET, "retrain: keep modulation"); if (o->pcmMode_ > 0) { CV(G_RET, "retrain: keep modulation, pcm mode kept as 1"); o->pcmMode_ = 1; } break;
    case 0x5a: { int m; CV(G_RET, "retrain: switch to V.90"); env_sess(MW, MC, ML, ctx1I, 0); m = o->mode_; o->pcmMode_ = 1; if (m == 2) { CV(G_RET, "retrain: V.90 with mode 2"); o->b_ac00_ = 1; } break; }
    case 0x5c: CV(G_RET, "retrain: switch to V.92"); env_sess(MW, MC, ML, ctx1I, 1); o->pcmMode_ = 1; break;
    default: if (mode == 0x22) { CV(G_RET, "retrain: switch to 0x22 (echo flag)"); o->echoOn_ = 1; } else CV(G_RET, "retrain: switch to a plain V.34 mode"); o->pcmMode_ = 0; o->k56Mode_ = 0; break;
  }
  env_hs(MW, MC, ML, g_iw.b[B_OBJ], 1);
  o->state_ = 7; o->mode_ = 0; o->w2218_ = 2;
  o->w4bc_ = o->w4be_ = o->w4c0_ = 0;
  o->w256_ = 0; o->w258_ = 0;
  o->w254_ = (uint16_t)(10000 + 336 * sx(o->w35a4_));
  o->rd0_ = o->rd2_ = o->rd4_ = o->rd6_ = 0; o->rd8_ = 0; o->rd14_ = o->rd18_ = o->rd1c_ = 0;
  { int m = sx(o->modeChar_);
    if (m == 0x65) { CV(G_RET, "retrain: detector reset, mode 0x65"); o->rdc_ = 0; o->rde_ = 0; o->rd10_ = 0x39c3; }
    else if (m == 0x66) { CV(G_RET, "retrain: detector reset, mode 0x66"); o->rdc_ = 0x5a82; o->rde_ = 0x55fc; o->rd10_ = 0x39c3; }
    else CV(G_RET, "retrain: detector reset, other mode"); }
}

/* ------------------------------------------------------------------------------------------------ world setup */
static void reg(int id, void *p, size_t n) { g_iw.b[id] = p; g_blkidx[id] = blk_add(bname[id], p, n); g_obj[id] = p; g_idname[id] = bname[id]; }
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
static int32_t pick_siglevel(void) {
  switch (h_rmod(8)) { case 0: return -0x31; case 1: return -0x28; case 2: return 0; case 3: return (int32_t)h_rnd(); case 4: return -0x30 + 8 + (int32_t)h_rmod(3); default: return -0x30 + (int32_t)h_rmod(8); }
}
static int32_t pick_delay(void) {
  switch (h_rmod(8)) { case 0: return (int32_t)0x7fffffff - (int32_t)h_rmod(3); case 1: return (int32_t)0x80000000u + (int32_t)h_rmod(3); case 2: return -(int32_t)h_rmod(40); case 3: return (int32_t)h_rnd(); case 4: return (int32_t)h_rmod(0x700); default: return (int32_t)h_rmod(200); }
}
static void new_world(void) {
  uint8_t *raw;
  V34Obj *o;
  int k;
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
  newblk(B_PAR, sizeof(V90ParObj)); newblk(B_PAR2, sizeof(V90ParObj));
  newblk(B_CD, 0x54); newblk(B_CD2, 0x54);
  for (k = B_P; k <= B_P2; k++) {
    V34ModParObj *p = (V34ModParObj *)g_iw.b[k];
    fill_tbl((uint8_t *)p, f_V34ModParObj, NF_V34ModParObj);
    p->minRate_ = pick_rate(); p->maxRate_ = pick_rate();
    if (h_rpct(35)) { p->minRate_ = (p->maxRate_ / 2400u) * 2400u; if (h_rpct(50)) p->maxRate_ = p->minRate_; }
    p->level_ = pick_siglevel(); p->delay_ = pick_delay(); p->extDelay_ = pick_delay();
    p->flags0_ = (uint8_t)(h_rpct(30) ? 0x18 : h_rpct(25) ? 0 : h_rpct(30) ? 0x10 : h_rpct(40) ? 8 : h_rnd());
    p->flags50_ = (uint8_t)h_rnd();
  }
  for (k = B_K; k <= B_K2; k++) {
    K56HandleObj *kk = (K56HandleObj *)g_iw.b[k];
    fill_tbl((uint8_t *)kk, f_K56HandleObj, NF_K56HandleObj);
    kk->flag8_ = (uint8_t)(h_rpct(40) ? 0 : h_rpct(60) ? 1 : h_rnd());
    kk->word_c_ = (int32_t)(h_rpct(30) ? 0 : h_rpct(45) ? 1 : h_rpct(30) ? 2 : (int32_t)h_rnd());
  }
  fill_blk(g_iw.b[B_INFO], INFOBLK);
  if (h_rpct(25)) memset(INFOP(&g_iw), 0, INFOW * 2);
  for (k = B_CTX; k <= B_CTX2; k++) {
    PcmFlo47Obj *c = (PcmFlo47Obj *)g_iw.b[k];
    fill_tbl((uint8_t *)c, f_PcmFlo47Obj, NF_PcmFlo47Obj);
    c->flag6120_ = h_rpct(45) ? 0u : h_rpct(70) ? 1u : h_rnd();
    c->v92Session_ = h_rpct(45) ? 0u : h_rpct(70) ? 1u : h_rnd();
    c->qcStatus_ = wsel();
    c->v90Dem_ = g_iw.b[k == B_CTX ? B_DEM : B_DEM2];
    c->v90Params_ = g_iw.b[k == B_CTX ? B_PAR : B_PAR2];
  }
  for (k = B_DEM; k <= B_DEM2; k++) {
    DmdObj *d = (DmdObj *)g_iw.b[k];
    fill_blk(d, sizeof *d);
    d->cdes_ = g_iw.b[k == B_DEM ? B_CD : B_CD2];
  }
  for (k = B_PAR; k <= B_PAR2; k++) {
    V90ParObj *pp = (V90ParObj *)g_iw.b[k];
    fill_blk(pp, sizeof *pp);
    pp->sensitiveIspDetected = (int32_t)(h_rpct(50) ? 0u : h_rpct(70) ? 1u : h_rnd());
  }
  fill_blk(g_iw.b[B_CD], 0x54); fill_blk(g_iw.b[B_CD2], 0x54);
  o = (V34Obj *)raw;
  o->ctx_ = g_iw.b[B_CTX]; o->k56_ = g_iw.b[B_K]; o->params_ = g_iw.b[B_P];
  o->mode_ = pick_mode3(); o->state_ = (int32_t)wsel();
  o->pcmMode_ = pick_mode3(); o->k56Mode_ = pick_mode3();
  o->modeChar_ = pick_modechar();
  o->w35a4_ = pick_h16();
  o->shortPhase2_ = (uint16_t)(h_rpct(45) ? 0 : h_rpct(50) ? 1 : h_rnd());
  o->pair0_ = pick_h16(); o->pair1_ = pick_h16(); o->pair2_ = pick_h16();
  if (h_rpct(30)) { o->pair0_ = (uint16_t)h_rmod(4); o->pair1_ = (uint16_t)h_rmod(4); o->pair2_ = (uint16_t)h_rmod(4); }
  o->minRateIdx_ = (int32_t)wsel(); o->maxRateIdx_ = (int32_t)wsel();
  o->wabca_ = pick_h16(); o->b_ac00_ = (uint8_t)h_rnd();
  g_obj[X_EC] = (uint8_t *)g_iw.b[B_CTX] + offsetof(PcmFlo47Obj, ecParams_);
  g_obj[X_EC2] = (uint8_t *)g_iw.b[B_CTX2] + offsetof(PcmFlo47Obj, ecParams_);
  blk_sync_model();
}
static void world_model(void) { int k; for (k = 0; k < NBLK; k++) g_mw.b[k] = g_blk[g_blkidx[k]].m; }

/* ------------------------------------------------------------------------------------------------ one call */
enum { OP_INFO, OP_RET, NOPS };
static const char *const opname[NOPS] = { "V34SetINFO1aBits", "VPcmV34InitiateRetrain" };
static unsigned pick_retmode(void) {
  static const unsigned m[] = { 0, 0x22, 0x38, 0x5a, 0x5c, 0, 0x22, 0x38, 0x5a, 0x5c, 0x21, 0x37, 0x39, 0x5b, 0x5d, 1 };
  return h_rpct(80) ? m[h_rmod(sizeof m / sizeof m[0])] : (unsigned)(h_rnd() & 0xffu);
}
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
  h_emit(show, "info %08x ctx %08x", (unsigned)fnv_upd(2166136261u, g_iw.b[B_INFO], INFOBLK), (unsigned)hash_tbl(2166136261u, g_iw.b[B_CTX], f_PcmFlo47Obj, NF_PcmFlo47Obj));
}
static void do_call(int op) {
  uint8_t *t = g_iw.b[B_OBJ];
  unsigned i, lv0, arg = 0;
  int ri = 0, rm = 0;
  blk_sync_model();
  world_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  g_what = opname[op];
  g_snapfn = obj_snapshot;
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  if (op == OP_RET) arg = (h_rpct(60) ? 0u : h_garbage_hi(0)) | pick_retmode();
  lv0 = dsplibs_debug_level;
  h_emit(g_show, "call %s arg=%u lvl=%u", g_what, arg, dsplibs_debug_level);
  if (op == OP_INFO) ri = f_info1a(t, INFOP(&g_iw)); else f_retrain(t, arg);
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  dsplibs_debug_level = lv0;
  if (op == OP_INFO) rm = m_info1a(INFOP(MW)); else m_retrain(arg);
  check_common(g_what);
  if (ri != rm) h_die3("%s: the model returns %d, the impl %d", g_what, rm, ri);
  if (op == OP_INFO) h_emit(g_show, "ret %d", ri);
  dump_world(g_show);
  cov_flush();
  g_ncalls++;
}

/* ------------------------------------------------------------------------------------------------ the trials */
static unsigned pick_lv(void) { return h_rpct(65) ? 2u + h_rmod(2) : pick_level(); }
static void trial(int sc) {
  int k, nm;
  new_world();
  gen_script();
  dsplibs_debug_level = pick_lv();
  g_pokemask = sc == G_POKE ? 0xfffffffeu : sc == G_SEQ ? (1u << 1) : 0;
  if (sc == G_INFO) do_call(OP_INFO);
  else if (sc == G_RET) do_call(OP_RET);
  else if (sc == G_POKE) do_call(h_rpct(50) ? OP_INFO : OP_RET);
  else {
    nm = (int)rnd_range(2, 5);
    for (k = 0; k < nm; k++) {
      if (k) { gen_script(); if (h_rpct(15)) dsplibs_debug_level = pick_lv(); }
      do_call((int)h_rmod(NOPS));
    }
  }
  h_trial_end("v01");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("V01_SEED");
  CovSite *cv;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("V01_ALL")) g_showall = 1;
  if (sc < 0 || sc > 3) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  p07_layout_check(); p43_layout_check(); p47_layout_check(); v01_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("V01_TRIALS") ? atoi(getenv("V01_TRIALS")) : (sc == G_INFO || sc == G_RET ? 12000 : 6000);
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (cv = __start_covsites; cv < __stop_covsites; cv++) {
    if (cv->grp != sc) continue;
    h_emit(1, "COV %s %lu", cv->name, cv->n);
    if (cv->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", cv->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("V01_NOCOV")) ? 2 : 0;
}
