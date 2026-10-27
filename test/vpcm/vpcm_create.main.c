/*
 * Differential test harness for port chunk V06 (src/vpcm/vpcm_create.cpp): VPcmV34Create (blob .text 0xaa70, 2,376 B), the constructor of the V.34 module object.  Derived from test/vpcm/vpcm_info1a_retrain.main.c
 * (recorders + independent model + per-record world snapshot + pokes); ONE C source, no #ifdef around the code under test, built by test/vpcm/vpcm_create.sh against the blob (dsplibs.o with the callees WEAK
 * so that the recorders below win), against the port (-O0 / -O2: the port object alone) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/vpcm_create.blob32.out).
 *
 * The world: the V.34 object (V34Obj, test/v34/v01.layout: 0xac4c B on i386, canary tail), the dp parameter block (V34ModParObj; the ARGUMENT, and a second one the object's params_ slot can be poked to), the K56Flex
 * handle (K56HandleObj, uninitialised in the blob), the VPcmFloModem ctx (P47's PcmFlo47Obj; a second one), its V90Demodulator (P43's DmdObj) with the V90ConstellationDesigner it points to (opaque), and the V92Phase2Info
 * block (P47V92InfoObj).  Every callee is a RECORDER: sysdep_memset (records, then clears), V34InitializeImplementationSpecific, v34handshakinit, VPcmFloModem::externalReset, K56FlexFloModem::externalReset /
 * setMinMaxRates, V90ConstellationDesigner::setMinMaxRates, V90Demodulator::enterChannelVerification, V92EchoCanceller::setEchoDelay, GenericIIR<float,double>::reset, edprintf, dsplibs_debug_printf.  The record
 * carries the arguments as tokens and a SNAPSHOT of the world (hashed: the object, the K56 handle, the ctx, the info block); a recorder may then POKE the world by script (the debug level, the three pointer slots of
 * the object, the words the function re-reads after the call, the foreign blocks): the time at which the blob / port reloads a member is observable.  The INDEPENDENT MODEL (m_create: a goto transliteration of the
 * control-flow graph of the disassembly, written from porting/decompiled/dp/v06_vpcmv34create.txt) runs AFTER the implementation on copies of the world, through the same environment functions, and must reproduce every
 * record, the returned value and every block (exit 3 otherwise).
 *
 * Scenarios (own process each): 0 every session type / quick-connect sense / parameter shape (no pokes), 1 the V.90 / V.92 modes (1, 2) with the V92 info block and the constellation designer, 2 every poke kind
 * enabled, 3 sequences of calls on one object (the state one call leaves is the next one's input) with level and pointer pokes.
 */

#include "../v34/v01_common.h"
#include "v90_demodulator_progress_layout.h"
#include "vpcm_flo_modem_layout.h"
#include "../v34/v01_layout.h"

/* ---- the function under test ---- */
extern int f_create(void *, int, int, void *, int) L("VPcmV34Create");

void *h_sysdep_malloc(unsigned size);
void h_sysdep_free(void *q);
void *sysdep_malloc(unsigned size) { return h_sysdep_malloc(size); }
void sysdep_free(void *q) { h_sysdep_free(q); }

/* ------------------------------------------------------------------------------------------------ the world */
enum { B_OBJ, B_P, B_P2, B_K, B_K2, B_CTX, B_CTX2, B_DEM, B_DEM2, B_CD, B_CD2, B_INFO, B_INFO2, NBLK };
enum { X_EC = NBLK, X_EC2, X_IIR, X_IIR2, X_RX, X_B8, NID };
static const char *const bname[NID] = { "obj", "P", "P2", "k56", "k56b", "ctx", "ctx2", "dem", "dem2", "cd", "cd2", "info", "info2", "ctx.ec_", "ctx2.ec_", "ctx.iir_", "ctx2.iir_", "rx", "obj+35b8" };
typedef struct { uint8_t *b[NBLK]; } W;
static W g_iw, g_mw;
static int g_blkidx[NBLK];
static uint8_t *g_can;
#define OBJ(w) ((V34Obj *)(w)->b[B_OBJ])
#define DBG() (dsplibs_debug_level > 1u)
#define IMPL_OBJ ((uint8_t *)g_iw.b[B_OBJ])
#define IMPL_RX ((uint8_t *)&((V34Obj *)g_iw.b[B_OBJ])->rxW0_)                  /* obj + 0x264 on i386 */
#define RX_SIZE ((unsigned)((uint8_t *)&((V34Obj *)g_iw.b[B_OBJ])->unk08b0_[0x150] - IMPL_RX))   /* 0x79c on i386 */
enum { G_ANY = 0, G_V9X = 1, G_POKE = 2, G_SEQ = 3 };

/* ------------------------------------------------------------------------------------------------ script, pokes */
typedef struct { uint32_t kind[NSC], val[NSC], res[NSC]; } Script;
typedef struct { unsigned pk, rv; } Cur;
static Script g_sc;
static Cur g_ic, g_mc;
static Log g_il, g_ml;
static int g_show, g_showall;
static unsigned long g_symctr, g_ncalls;
static unsigned g_pokemask;
static int g_scribble = 1;   /* V06_NOSCRIBBLE=1: off (debugging) */
static const char *g_what = "setup";
#define IW (&g_iw)
#define IL (&g_il)
#define IC (&g_ic)
#define MW (&g_mw)
#define ML (&g_ml)
#define MC (&g_mc)

/* a pointer the world holds (an impl-world address) as seen from world w: the model reads its own copies of the blocks */
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
    g_sc.kind[i] = h_rpct(60) ? 0u : 1u + h_rmod(12);
    g_sc.val[i] = h_rnd();
    g_sc.res[i] = h_rnd();
  }
}
static uint32_t pick_w(uint32_t v) { switch (v & 7u) { case 0: return 0; case 1: return 1; case 2: return 0xffffffffu; case 3: return 0x80000000u; case 4: return 0xe; default: return v >> 3; } }
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
    case 5: if (v & 8) o->pcmMode_ = (int32_t)(v >> 4 & 3u) - (int32_t)(v >> 6 & 1u); else o->k56Mode_ = (int32_t)(v >> 4 & 3u) - (int32_t)(v >> 6 & 1u); break;
    case 6: if (v & 8) o->modeChar_ = pick_h(v >> 4); else o->w35a4_ = pick_h(v >> 4); break;
    case 7: { V34ModParObj *p = (V34ModParObj *)P(w, g_iw.b[(v & 8) ? B_P2 : B_P]);
      switch ((v >> 4) % 8u) { case 0: p->level_ = (int32_t)pick_w(v >> 8); break; case 1: p->delay_ = (int32_t)pick_w(v >> 8); break; case 2: p->extDelay_ = (int32_t)pick_w(v >> 8); break;
        case 3: p->minRate_ = pick_w(v >> 8); break; case 4: p->maxRate_ = pick_w(v >> 8); break; case 5: p->entrFilt_ = (int32_t)pick_w(v >> 8); break;
        case 6: p->w54_ = (int32_t)pick_w(v >> 8); break; default: p->w14_ = pick_w(v >> 8); } break; }
    case 8: { PcmFlo47Obj *cx = (PcmFlo47Obj *)P(w, g_iw.b[(v & 8) ? B_CTX2 : B_CTX]);
      switch ((v >> 4) % 3u) { case 0: cx->flag6120_ = pick_w(v >> 8); break; case 1: cx->v92Session_ = pick_w(v >> 8); break; default: cx->b7f5c_ = (uint8_t)(v >> 8); } break; }
    case 9: { K56HandleObj *kk = (K56HandleObj *)P(w, g_iw.b[(v & 8) ? B_K2 : B_K]); if (v & 16) kk->word_c_ = (int32_t)pick_w(v >> 8); else kk->flag8_ = (uint8_t)(v >> 8); break; }
    case 10: { PcmFlo47Obj *cx = (PcmFlo47Obj *)P(w, g_iw.b[(v & 8) ? B_CTX2 : B_CTX]);
      if (v & 16) cx->v92Info_ = (v & 32) ? g_iw.b[B_INFO2] : g_iw.b[B_INFO];
      else { ((P47V92InfoObj *)P(w, g_iw.b[(v & 32) ? B_INFO2 : B_INFO]))->unk14_[(v >> 6) & 3u] = (uint8_t)(v >> 8); }
      break; }
    case 11: { PcmFlo47Obj *cx = (PcmFlo47Obj *)P(w, g_iw.b[(v & 8) ? B_CTX2 : B_CTX]);
      if (v & 16) cx->v90Dem_ = (v & 32) ? g_iw.b[B_DEM2] : g_iw.b[B_DEM];
      else { ((DmdObj *)P(w, g_iw.b[(v & 32) ? B_DEM2 : B_DEM]))->cdes_ = (v & 64) ? g_iw.b[B_CD2] : g_iw.b[B_CD]; }
      break; }
    case 12: o->w35a4_ = (uint16_t)(v >> 3); break;
    default: break;
  }
}

/* ------------------------------------------------------------------------------------------------ the snapshot of the world at the moment of a record */
static uint32_t hash_tbl(uint32_t h, const uint8_t *t, const HFld *tbl, int nf, const uint8_t *pbase) {
  int i;
  for (i = 0; i < nf; i++) {
    const HFld *fl = &tbl[i];
    unsigned n = fl->count ? fl->count : 1, es, j;
    const uint8_t *q = t + fl->off;
    switch (fl->kind) {
      case 'p': case 'o':
        for (j = 0; j < n; j++) { void *v; int id; memcpy(&v, q + j * sizeof v, sizeof v); id = tok_of(v); h = fnv_upd(h, &id, sizeof id); }
        continue;
      case 'e':   /* a pointer into the object itself (the model's copy holds the impl-world address): its offset from the impl base */
        for (j = 0; j < n; j++) { const uint8_t *v; int32_t d; memcpy(&v, q + j * sizeof v, sizeof v); d = !v ? -1 : (v >= pbase && (size_t)(v - pbase) < 0x100000u) ? (int32_t)(v - pbase) : -2; h = fnv_upd(h, &d, sizeof d); }
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
  uint32_t h = hash_tbl(2166136261u, w->b[B_OBJ], f_V34Obj, NF_V34Obj, IMPL_OBJ), c = 2166136261u, i = 2166136261u, k = 2166136261u;
  c = hash_tbl(c, w->b[B_CTX], f_PcmFlo47Obj, NF_PcmFlo47Obj, IMPL_OBJ);
  c = hash_tbl(c, w->b[B_CTX2], f_PcmFlo47Obj, NF_PcmFlo47Obj, IMPL_OBJ);
  i = hash_tbl(i, w->b[B_INFO], f_P47V92InfoObj, NF_P47V92InfoObj, IMPL_OBJ);
  i = hash_tbl(i, w->b[B_INFO2], f_P47V92InfoObj, NF_P47V92InfoObj, IMPL_OBJ);
  k = hash_tbl(k, w->b[B_K], f_K56HandleObj, NF_K56HandleObj, IMPL_OBJ);
  k = hash_tbl(k, w->b[B_K2], f_K56HandleObj, NF_K56HandleObj, IMPL_OBJ);
  snprintf(out, cap, "|h=%08x c=%08x i=%08x k=%08x lvl=%u", h, c, i, k, dsplibs_debug_level);
}

/* ------------------------------------------------------------------------------------------------ the callee environment: record, effect, poke (the impl's recorders and the model call the same functions) */
static const char *nm06(int id) {
  static char b[40][24];
  static int n;
  char *o = b[n++ % 40];
  if (id >= 0 && id < g_nobj) return g_idname[id];
  if (id == ID_NULL) return "null";
  if (id == ID_WILD) return "wild";
  snprintf(o, 24, "A%d+%d", (id - 100) / 4096, (id - 100) % 4096);
  return o;
}
#define T_(p) nm06(tok_of(p))
#define ENVREC(w, l, ...) do { rec_add((l), (w) == &g_mw, 'c', __VA_ARGS__); } while (0)

static void env_ed(W *w, Cur *c, Log *l, char kind, const char *s) { rec_add(l, w == &g_mw, kind, "%s", s); poke(w, c); }
static void *env_memset(W *w, Cur *c, Log *l, void *dst, int ch, unsigned n) {
  uint8_t *impl = (uint8_t *)dst;
  const char *nm = impl == IMPL_OBJ ? "obj" : impl == IMPL_RX ? "rx" : "other";
  const char *sz = (impl == IMPL_OBJ && n == sizeof(V34Obj)) ? "sizeof" : (impl == IMPL_RX && n == RX_SIZE) ? "rxsize" : NULL;   /* the sizes differ on x86-64 */
  if (sz) ENVREC(w, l, "sysdep_memset(dst=%s c=%d n=%s)", nm, ch, sz);
  else ENVREC(w, l, "sysdep_memset(dst=%s c=%d n=%u)", nm, ch, n);
  if (!strcmp(nm, "other")) h_die3("sysdep_memset: unexpected destination");
  memset(P(w, dst), ch, n);
  poke(w, c);
  return dst;
}
/* the real V34InitializeImplementationSpecific / v34handshakinit write into the object: the recorders scribble over what the function overwrites afterwards (a store of 0 right behind the memset would otherwise be
 * invisible).  A private xorshift stream keeps the scribble identical in the impl's and the model's world and independent of sizeof (member-wise); pointer members are left alone. */
static uint32_t xs32(uint32_t *s) { *s ^= *s << 13; *s ^= *s >> 17; *s ^= *s << 5; return *s; }
static void scribble_all(V34Obj *o, uint32_t seed) {
  uint32_t s = seed | 1u;
  int i;
  uint16_t waa80 = o->waa80_;
  for (i = 0; i < NF_V34Obj; i++) {
    const HFld *fl = &f_V34Obj[i];
    unsigned n = fl->count ? fl->count : 1, es, j;
    uint8_t *q = (uint8_t *)o + fl->off;
    switch (fl->kind) {
      case 'p': case 'o': case 'e': case 'v': continue;
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'd': es = 8; break;
      default: es = 4; break;
    }
    for (j = 0; j < n * es; j++) q[j] = (uint8_t)(xs32(&s) >> 11);
  }
  o->waa80_ = waa80;
}
static void scribble_post(V34Obj *o, uint32_t seed) {   /* the words VPcmV34Create stores after v34handshakinit and that nothing reads in between */
  uint32_t s = seed | 1u;
#define SCR(m) do { uint32_t r_ = xs32(&s) >> 7; memcpy(&o->m, &r_, sizeof o->m); } while (0)
  SCR(w2aa6_); SCR(rd0_); SCR(rd2_); SCR(rd4_); SCR(rd6_); SCR(rd8_); SCR(rdc_); SCR(rde_); SCR(rd10_); SCR(rd14_); SCR(rd18_); SCR(rd1c_);
  SCR(w254_); SCR(w256_); SCR(w258_); SCR(b_ac16_); SCR(b_ac17_); SCR(w262_); SCR(dgC0c_); SCR(dgC04_); SCR(dgC08_); SCR(dgC0e_); SCR(dgC10_); SCR(dgC12_); SCR(dgC14_);
  SCR(w3554_); SCR(w3558_); SCR(w355c_); SCR(wa248_); SCR(w234_);   /* w234_ is cleared before the call AND after the minSigLevel print */
#undef SCR
}
static void env_init(W *w, Cur *c, Log *l, void *obj) {
  uint32_t r = sres(c);
  ENVREC(w, l, "V34InitializeImplementationSpecific(obj=%s)", T_(obj));
  if (g_scribble) scribble_all(OBJ(w), r);
  poke(w, c);
}
static int env_hs(W *w, Cur *c, Log *l, void *obj, unsigned mode) {
  uint32_t r = sres(c);
  ENVREC(w, l, "v34handshakinit(obj=%s %u)", T_(obj), mode);
  if (g_scribble) scribble_post(OBJ(w), r);
  poke(w, c);
  return (int)r;
}
static void env_pcmreset(W *w, Cur *c, Log *l, void *self) { ENVREC(w, l, "VPcmFloModem::externalReset(this=%s)", T_(self)); poke(w, c); }
static void env_k56reset(W *w, Cur *c, Log *l, void *self) { ENVREC(w, l, "K56.externalReset(this=%s)", T_(self)); poke(w, c); }
static void env_k56mm(W *w, Cur *c, Log *l, void *self, int a, int b) { ENVREC(w, l, "K56.setMinMaxRates(this=%s %d %d)", T_(self), a, b); poke(w, c); }
static void env_cdmm(W *w, Cur *c, Log *l, void *self, unsigned a, unsigned b) { ENVREC(w, l, "cd.setMinMaxRates(this=%s %u %u)", T_(self), a, b); poke(w, c); }
static void env_ecv(W *w, Cur *c, Log *l, void *self, int a, int b) { ENVREC(w, l, "dem.enterChannelVerification(this=%s %d %d)", T_(self), a, b); poke(w, c); }
static void env_echodelay(W *w, Cur *c, Log *l, void *self, unsigned d) { ENVREC(w, l, "setEchoDelay(this=%s %u)", T_(self), d); poke(w, c); }
static void env_iirreset(W *w, Cur *c, Log *l, void *self) { ENVREC(w, l, "iir.reset(this=%s)", T_(self)); poke(w, c); }

/* ------------------------------------------------------------------------------------------------ the recorders (the blob's callees, weakened in the copy of dsplibs.o) */
extern void *rc_memset(void *, int, unsigned) L("sysdep_memset");
extern void rc_init(void *) L("V34InitializeImplementationSpecific");
extern int rc_hs(void *, unsigned) L("v34handshakinit");
extern void rc_pcmreset(void *) L("_ZN12VPcmFloModem13externalResetEv");
extern void rc_k56reset(void *) L("_ZN15K56FlexFloModem13externalResetEv");
extern void rc_k56mm(void *, int, int) L("_ZN15K56FlexFloModem14setMinMaxRatesEii");
extern void rc_cdmm(void *, unsigned, unsigned) L("_ZN24V90ConstellationDesigner14setMinMaxRatesEjj");
extern void rc_ecv(void *, short, short) L("_ZN14V90Demodulator24enterChannelVerificationEss");
extern void rc_echodelay(void *, unsigned) L("_ZN16V92EchoCanceller12setEchoDelayEj");
extern void rc_iirreset(void *) L("_ZN10GenericIIRIfdE5resetEv");
void *rc_memset(void *d, int c, unsigned n) { return env_memset(IW, IC, IL, d, c, n); }
void rc_init(void *o) { env_init(IW, IC, IL, o); }
int rc_hs(void *o, unsigned m) { return env_hs(IW, IC, IL, o, m); }
void rc_pcmreset(void *t) { env_pcmreset(IW, IC, IL, t); }
void rc_k56reset(void *t) { env_k56reset(IW, IC, IL, t); }
void rc_k56mm(void *t, int a, int b) { env_k56mm(IW, IC, IL, t, a, b); }
void rc_cdmm(void *t, unsigned a, unsigned b) { env_cdmm(IW, IC, IL, t, a, b); }
void rc_ecv(void *t, short a, short b) { env_ecv(IW, IC, IL, t, a, b); }
void rc_echodelay(void *t, unsigned d) { env_echodelay(IW, IC, IL, t, d); }
void rc_iirreset(void *t) { env_iirreset(IW, IC, IL, t); }
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
  if (!strcmp(fmt, "RX at %lX\n")) {   /* a raw address: only its identity is observable */
    unsigned long a = va_arg(ap, unsigned long);
    snprintf(buf, sizeof buf, "RX at %s\n", a == (unsigned long)IMPL_RX ? "rx" : "wild");
  } else {
    vsnprintf(buf, sizeof buf, fmt, ap);
  }
  va_end(ap);
  env_ed(IW, IC, IL, 'd', buf);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ the independent model (from porting/decompiled/dp/v06_vpcmv34create.txt): the CFG of the disassembly as gotos */
static void mpf(char kind, const char *fmt, ...) {
  char b[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  env_ed(MW, MC, ML, kind, b);
}
static V34Obj *mo(void) { return OBJ(MW); }
static void *mp(const void *p) { return P(MW, p); }
static int sx(unsigned v) { return (int)(int16_t)(uint16_t)v; }       /* movswl / cwtl */

static const int k_thresh[8] = { 71, 80, 90, 101, 113, 127, 142, 160 };   /* V34DisconnectThreshTable (the model's own copy) */

static int m_create(int a1, int a2_, void *pmI, int mode) {
  V34Obj *o = mo();
  void *ctx0I = o->ctx_, *k56I = o->k56_;                          /* esi / edi at entry */
  V34ModParObj *p0 = (V34ModParObj *)mp(pmI);
  K56HandleObj *k56 = (K56HandleObj *)mp(k56I);
  PcmFlo47Obj *ctx0 = (PcmFlo47Obj *)mp(ctx0I);
  uint8_t *objI = IMPL_OBJ;
  unsigned qc = ((unsigned)p0->flags2_ >> 4) & 1u;                 /* 0x20(esp) */
  int s18 = p0->uqtsIdx_, s1c = p0->ansLevel_;                     /* 0x18 / 0x1c(esp) */
  int a2slot = a1;                                                 /* 0x44(esp): the first stack argument after obj */
  unsigned t17;                                                    /* 0x17(esp) */
  unsigned ax;                                                     /* 0x65 / 0x66 */
  unsigned lvl;
  V34ModParObj *p;
  PcmFlo47Obj *ctx;
  K56HandleObj *kk;
  P47V92InfoObj *info;
  int eax, edx, ecx;
  unsigned ueax;
  (void)a2_;
  if (!DBG()) { CV(G_ANY, "create: no entry prints"); goto aac7; }
  mpf('d', "VPcmV34Create: quick connect indication from phase1 = %d\r\n", (int)qc);
  if (!DBG()) { CV(G_POKE, "create: entry prints cut after 1"); goto aac7; }
  mpf('d', "VPcmV34Create: Uqts index is %d\r\n", s18);
  if (!DBG()) { CV(G_POKE, "create: entry prints cut after 2"); goto aac7; }
  mpf('d', "VPcmV34Create: ANSpcm level index is %d\r\n", s1c);
  if (!DBG()) { CV(G_POKE, "create: entry prints cut after 3"); goto aac7; }
  mpf('d', "VPcmV34Create, initial Session Type = %d\n", mode);
  if (!DBG()) { CV(G_POKE, "create: entry prints cut after 4"); goto aac7; }
  CV(G_ANY, "create: all five entry prints");
  mpf('d', "RX at rx\n");
aac7:
  env_memset(MW, MC, ML, objI, 0, (unsigned)sizeof(V34Obj));
  env_memset(MW, MC, ML, IMPL_RX, 0, RX_SIZE);
  o->k56_ = k56I; o->waa80_ = 1; o->params_ = pmI; o->ctx_ = ctx0I;
  env_init(MW, MC, ML, objI);
  o->echoOn_ = 1;
  o->w35ac_ = 0; o->w35a8_ = 0;
  o->w35b0_ = ((V34Obj *)objI)->unk35b8_;
  o->w35b4_ = 0x2580;
  o->mohD8_ = 0; o->wabdc_ = 0; o->mohE0_ = 0; o->mohE2_ = 0; o->mohE4_ = 0; o->mohE6_ = 0; o->w35a4_ = 0;
  if (DBG()) { CV(G_ANY, "create: TX MD print"); mpf('d', "On Create: Setting desired TX MD (%d mSec)!\n", 0); }
  /* abc.. */
  ueax = p0->w14_ * 24u;
  o->wabfc_ = (uint16_t)(ueax / 10u);
  info = (P47V92InfoObj *)mp(((PcmFlo47Obj *)mp(ctx0I))->v92Info_); info->shortPhase2Local_ = 0;
  info = (P47V92InfoObj *)mp(((PcmFlo47Obj *)mp(ctx0I))->v92Info_); info->v92CapLocal_ = 0;
  o->wabc4_ = 0; o->wabc6_ = 0; o->wabc8_ = 0; o->wabca_ = 0; o->shortPhase2_ = 0; o->wabd4_ = 0; o->pair2_ = 0; o->pair1_ = 0; o->pair0_ = 0;
  ((PcmFlo47Obj *)mp(ctx0I))->v92Session_ = 0;
  o->mode_ = 0; o->state_ = 0; o->k56Mode_ = 0;
  if (mode == 2) goto m_eq2;
  if (mode < 2) goto m_lt2;
  if (mode == 3) goto m_3;
  if (mode == 4) goto m_4;
  goto m_default;
m_eq2:
  CV(G_V9X, "create: mode 2");
  k56->word_c_ = 0;
  o->pcmMode_ = 1;
  env_pcmreset(MW, MC, ML, ctx0I);
  o->wabc6_ = 1;
  o->wabca_ = (uint16_t)qc;
  info = (P47V92InfoObj *)mp(((PcmFlo47Obj *)mp(ctx0I))->v92Info_);
  o->wabd4_ = info->unk14_[3];
  o->pair2_ = info->unk14_[2];
  o->pair1_ = info->unk14_[1];
  o->pair0_ = info->unk14_[0];
  info->v92CapLocal_ = 1;
  info = (P47V92InfoObj *)mp(((PcmFlo47Obj *)mp(ctx0I))->v92Info_);
  info->shortPhase2Local_ = (uint8_t)qc;
  t17 = (a2slot == 0);
  if (!(qc != 0 && t17)) { CV(G_V9X, "create: mode 2: no channel verification"); goto b069; }
  CV(G_V9X, "create: mode 2: channel verification");
  o->mode_ = 4;
  env_ecv(MW, MC, ML, ((PcmFlo47Obj *)mp(ctx0I))->v90Dem_, sx((unsigned)s18), sx((unsigned)s1c));
  o->state_ = 10;
b069:
  if (t17) { CV(G_V9X, "create: mode 2: modeChar 0x66"); ax = 0x66; goto ac87; }
  CV(G_V9X, "create: mode 2: session word 1, modeChar 0x65");
  ((PcmFlo47Obj *)mp(ctx0I))->v92Session_ = 1;
  ax = 0x65;
  goto ac87;
m_lt2:
  k56->word_c_ = 0;
  if (mode != 1) { CV(G_ANY, "create: mode <= 0"); o->pcmMode_ = 0; goto ac76; }
  CV(G_V9X, "create: mode 1");
  a2slot = (a2slot == 0);
  o->pcmMode_ = 1;
  env_pcmreset(MW, MC, ML, ctx0I);
  goto ac76;
m_3:
  CV(G_ANY, "create: mode 3");
  k56->word_c_ = 0;
  o->pcmMode_ = 0;
  goto b32f;
m_4:
  CV(G_ANY, "create: mode 4");
  k56->word_c_ = 1;
  o->pcmMode_ = 0;
b32f:
  mpf('e', "VPcmV34Main: it is K56Flex session type, (AorMu = %d)\r\n", k56->word_c_);
  o->k56Mode_ = 1;
  env_k56reset(MW, MC, ML, k56I);
  goto ac76;
m_default:
  CV(G_ANY, "create: mode > 4");
  k56->word_c_ = 0;
  o->pcmMode_ = 0;
ac76:
  ax = a2slot == 0 ? 0x65 : 0x66;
ac87:
  o->modeChar_ = (uint16_t)ax;
  o->w8_ = a2_;
  o->w114_ = 0; o->w218_ = 0;
  o->maxRateIdx_ = 0xe;
  o->w238_ = 0; o->w21c_ = 0; o->minRateIdx_ = 0; o->w228_ = 0; o->w22c_ = -1; o->minSigLevel_ = 0; o->w234_ = 0;
  o->w248_ = (int32_t)0xfffe8900u;
  o->w359a_ = 0;
  o->rxGainEst_ = 0x600; o->w49c_ = 0;
  o->w0c_ = 0; o->w10_ = 0;
  env_hs(MW, MC, ML, objI, 0);
  eax = sx(o->w35a4_);
  o->w2aa6_ = 0;
  o->rd2_ = 0; o->rd4_ = 0; o->rd6_ = 0; o->rd8_ = 0; o->rd14_ = 0; o->rd18_ = 0; o->rd1c_ = 0;
  o->w256_ = 0; o->w258_ = 0;
  ecx = eax * 5; eax = eax + ecx * 4; eax <<= 4; eax += 0x2710;
  o->w254_ = (uint16_t)eax;
  o->rd0_ = 0;
  edx = sx(o->modeChar_);
  if ((int16_t)edx == 0x65) { CV(G_ANY, "create: detector reset, mode 0x65"); o->rdc_ = 0; o->rde_ = 0; o->rd10_ = 0x39c3; }
  else if ((int16_t)edx == 0x66) { CV(G_ANY, "create: detector reset, mode 0x66"); o->rdc_ = 0x5a82; o->rde_ = 0x55fc; o->rd10_ = 0x39c3; }
  else CV(G_POKE, "create: detector reset, other mode word");
  o->b_ac16_ = 0; o->dgC0c_ = 0; o->b_ac17_ = 0; o->w262_ = 1;
  o->dgC04_ = 0; o->dgC08_ = 0; o->dgC0e_ = 0; o->dgC10_ = 0; o->dgC12_ = 0; o->dgC14_ = 0;
  o->w3554_ = 0x7d0; o->wa248_ = 0; o->w3558_ = 0x7fdf; o->w355c_ = 2;
  edx = o->pcmMode_;
  ctx = (PcmFlo47Obj *)mp(o->ctx_);
  kk = (K56HandleObj *)mp(o->k56_);
  if (edx == 0) goto ae68;
  if (((PcmFlo47Obj *)mp(o->ctx_))->flag6120_ == 0) goto ae68;
  CV(G_V9X, "create: V.90 constellation designer limits");
  {
    void *demI = ctx->v90Dem_;
    DmdObj *dem = (DmdObj *)mp(demI);
    p = (V34ModParObj *)mp(o->params_);
    env_cdmm(MW, MC, ML, dem->cdes_, p->minRate_, p->maxRate_);
  }
  goto aee1;
ae68:
  if (o->k56Mode_ == 0) goto ae7c;
  if (kk->flag8_ == 0) goto ae7c;
  CV(G_ANY, "create: K56Flex limits");
  p = (V34ModParObj *)mp(o->params_);
  env_k56mm(MW, MC, ML, o->k56_, (int)p->minRate_, (int)p->maxRate_);
  goto aee1;
ae7c:
  {
    uint64_t m;
    uint32_t hi;
    p = (V34ModParObj *)mp(o->params_);
    m = (uint64_t)p->minRate_ * 0x1b4e81b5u; hi = (uint32_t)(m >> 32) >> 8;
    o->minRateIdx_ = (int32_t)hi;
    m = (uint64_t)p->maxRate_ * 0x1b4e81b5u;
    o->maxRateIdx_ = (int32_t)((uint32_t)(m >> 32) >> 8);
    if ((int)hi > 14) { CV(G_ANY, "create: min index clamped"); o->minRateIdx_ = 14; }
    edx = o->maxRateIdx_; eax = o->minRateIdx_;
    if (edx < eax) { CV(G_ANY, "create: max raised to min"); o->maxRateIdx_ = eax; edx = eax; }
    if (edx > 14) { CV(G_ANY, "create: max clamped"); o->maxRateIdx_ = 14; }
    else { edx--; if (edx == 0) { CV(G_ANY, "create: max index 1: w359a"); o->w359a_ = 1; } }
  }
aee1:
  p = (V34ModParObj *)mp(o->params_);
  ueax = (uint32_t)p->level_ + 0x30u;
  if (ueax > 7u) ueax = 3;
  eax = k_thresh[ueax];
  o->minSigLevel_ = eax;
  if (DBG()) { CV(G_ANY, "create: minSigLevel print"); mpf('d', "VPcmV34Main: minLevel given is %d , minSigLevel set to %d\n", p->level_, eax); p = (V34ModParObj *)mp(o->params_); }
  o->w234_ = 0;
  lvl = dsplibs_debug_level;
  { int esi = p->delay_;
    void *ctx3I = o->ctx_;
    eax = ((int)((unsigned)esi + 2u) >> 2) + 0x22;
    o->filtDelay_ = (uint16_t)eax;
    if (lvl > 1u) { CV(G_ANY, "create: filtdelay print"); mpf('d', "V34 filtdelay set to %d (params initial delay = %d)\n", sx((unsigned)eax), esi); lvl = dsplibs_debug_level; p = (V34ModParObj *)mp(o->params_); }
    esi = p->extDelay_;
    o->dmaDelay_ = (uint16_t)(0x610 - esi);
    if (lvl > 1u) { CV(G_ANY, "create: dmadelay print"); mpf('d', "V34FEC, V34dmadelay set to %d, (ext delay=%d)\n", sx((unsigned)(0x610 - esi)), esi); p = (V34ModParObj *)mp(o->params_); }
    p = (V34ModParObj *)mp(o->params_);
    env_echodelay(MW, MC, ML, (uint8_t *)ctx3I + offsetof(PcmFlo47Obj, ecParams_), (unsigned)(p->extDelay_ + 0x68));
  }
  p = (V34ModParObj *)mp(o->params_);
  eax = p->entrFilt_;
  if (eax == -1) goto b242;
  eax--;
  if (eax == 0) goto b290;
  if (DBG()) { CV(G_ANY, "create: entrance filter forced disabled print"); mpf('d', "VPcmFlo: From Stream - Entrance Filter forced disabled...\r\n"); }
  ((PcmFlo47Obj *)mp(ctx0I))->b7f5c_ = 0;
  goto af9e;
b242:
  if (DBG()) { CV(G_ANY, "create: entrance filter according to HW print"); mpf('d', "VPcmFlo: From Stream - Entrance Filter according to HW...\r\n"); }
  p = (V34ModParObj *)mp(o->params_);
  ((PcmFlo47Obj *)mp(ctx0I))->b7f5c_ = (p->w54_ == 0xe) ? 1 : 0;
  if (p->w54_ == 0xe) CV(G_ANY, "create: entrance filter HW: on"); else CV(G_ANY, "create: entrance filter HW: off");
  goto af9e;
b290:
  if (DBG()) { CV(G_ANY, "create: entrance filter forced enabled print"); mpf('d', "VPcmFlo: From Stream - Entrance Filter forced enabled...\r\n"); }
  ((PcmFlo47Obj *)mp(ctx0I))->b7f5c_ = 1;
af9e:
  env_iirreset(MW, MC, ML, (uint8_t *)ctx0I + offsetof(PcmFlo47Obj, iirDen_));
  if (DBG()) { CV(G_ANY, "create: entrance filter applied print"); mpf('d', "VPcmFlo: YES! entrance filter applied = %d\r\n", (int)((PcmFlo47Obj *)mp(ctx0I))->b7f5c_); }
  return 0;
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
  switch (h_rmod(8)) { case 0: return (int32_t)0x7fffffff - (int32_t)h_rmod(3); case 1: return (int32_t)0x80000000u + (int32_t)h_rmod(3); case 2: return -(int32_t)h_rmod(40); case 3: return (int32_t)h_rnd(); case 4: return (int32_t)h_rmod(0x700); default: return (int32_t)h_rmod(400); }
}
static uint32_t pick_md(void) { switch (h_rmod(8)) { case 0: return 0; case 1: return 0x0aaaaaabu; case 2: return 0x0aaaaaaau; case 3: return 0xffffffffu; case 4: return 0x1999999au; case 5: return h_rnd(); case 6: return h_rmod(2000); default: return 0x80000000u + h_rmod(3); } }
static int32_t pick_efilt(void) { switch (h_rmod(8)) { case 0: case 1: return -1; case 2: case 3: return 1; case 4: return 0; case 5: return 2; case 6: return (int32_t)0x80000000u; default: return (int32_t)h_rnd(); } }
static int32_t pick_w54(void) { return h_rpct(40) ? 0xe : h_rpct(30) ? 0xf : h_rpct(30) ? (int32_t)0x8000000e : (int32_t)h_rnd(); }
static int pick_mode(int sc) {
  static const int all[] = { -1, 0, 1, 2, 3, 4, 5, 0x7fffffff, (int)0x80000000u, 1, 2, 2, 3, 4, 0 };
  if (sc == G_V9X) return h_rpct(85) ? 1 + (int)h_rmod(2) : all[h_rmod(sizeof all / sizeof all[0])];
  return h_rpct(90) ? all[h_rmod(sizeof all / sizeof all[0])] : (int)h_rnd();
}
static int pick_quick(void) { return h_rpct(45) ? 0 : h_rpct(50) ? 1 : h_rpct(30) ? 0x100 : h_rpct(40) ? (int)0x80000000u : (int)h_rnd(); }
static void fill_params(V34ModParObj *p) {
  fill_tbl((uint8_t *)p, f_V34ModParObj, NF_V34ModParObj);
  p->minRate_ = pick_rate(); p->maxRate_ = pick_rate();
  if (h_rpct(35)) { p->minRate_ = (p->maxRate_ / 2400u) * 2400u; if (h_rpct(50)) p->maxRate_ = p->minRate_; }
  if (h_rpct(8)) { p->minRate_ = 2400u + h_rmod(2); p->maxRate_ = 2400u + h_rmod(2); }
  p->level_ = pick_siglevel(); p->delay_ = pick_delay(); p->extDelay_ = pick_delay();
  p->flags2_ = (uint8_t)(h_rpct(40) ? 0x10 : h_rpct(30) ? 0 : h_rnd());
  p->ansLevel_ = h_rpct(40) ? (int32_t)h_rmod(8) : (int32_t)h_rnd();
  p->uqtsIdx_ = h_rpct(40) ? (int32_t)h_rmod(8) : h_rpct(40) ? (int32_t)(0x10000 + h_rmod(8)) : (int32_t)h_rnd();
  p->w14_ = pick_md();
  p->w54_ = pick_w54();
  p->entrFilt_ = pick_efilt();
}
static void fill_world(void) {
  int k;
  V34ModParObj *pp;
  for (k = B_P; k <= B_P2; k++) fill_params((V34ModParObj *)g_iw.b[k]);
  for (k = B_K; k <= B_K2; k++) {
    K56HandleObj *kk = (K56HandleObj *)g_iw.b[k];
    fill_tbl((uint8_t *)kk, f_K56HandleObj, NF_K56HandleObj);
    kk->flag8_ = (uint8_t)(h_rpct(40) ? 0 : h_rpct(60) ? 1 : h_rnd());
    kk->word_c_ = (int32_t)(h_rpct(30) ? 0 : h_rpct(45) ? 1 : h_rpct(30) ? 2 : (int32_t)h_rnd());
  }
  for (k = B_CTX; k <= B_CTX2; k++) {
    PcmFlo47Obj *c = (PcmFlo47Obj *)g_iw.b[k];
    fill_tbl((uint8_t *)c, f_PcmFlo47Obj, NF_PcmFlo47Obj);
    c->flag6120_ = h_rpct(45) ? 0u : h_rpct(70) ? 1u : h_rnd();
    c->v92Session_ = h_rpct(45) ? 0u : h_rpct(70) ? 1u : h_rnd();
    c->b7f5c_ = (uint8_t)h_rnd();
    c->v90Dem_ = g_iw.b[k == B_CTX ? B_DEM : B_DEM2];
    c->v92Info_ = g_iw.b[k == B_CTX ? B_INFO : B_INFO2];
  }
  for (k = B_DEM; k <= B_DEM2; k++) {
    DmdObj *d = (DmdObj *)g_iw.b[k];
    fill_blk(d, sizeof *d);
    d->cdes_ = g_iw.b[k == B_DEM ? B_CD : B_CD2];
  }
  for (k = B_INFO; k <= B_INFO2; k++) {
    P47V92InfoObj *in = (P47V92InfoObj *)g_iw.b[k];
    fill_tbl((uint8_t *)in, f_P47V92InfoObj, NF_P47V92InfoObj);
    if (h_rpct(30)) memset(in->unk14_, 0, 4);
    if (h_rpct(25)) memset(in->unk14_, 0xff, 4);
  }
  fill_blk(g_iw.b[B_CD], 0x54); fill_blk(g_iw.b[B_CD2], 0x54);
  (void)pp;
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
  newblk(B_CTX, sizeof(PcmFlo47Obj)); newblk(B_CTX2, sizeof(PcmFlo47Obj));
  newblk(B_DEM, sizeof(DmdObj)); newblk(B_DEM2, sizeof(DmdObj));
  newblk(B_CD, 0x54); newblk(B_CD2, 0x54);
  newblk(B_INFO, sizeof(P47V92InfoObj)); newblk(B_INFO2, sizeof(P47V92InfoObj));
  fill_world();
  o = (V34Obj *)raw;
  o->w35b0_ = h_rpct(45) ? NULL : h_rpct(50) ? (void *)o->unk35b8_ : (void *)(raw + 4u * h_rmod(2000) + 1u);   /* the pointer into the object: NULL, the right one, or a wrong one (tokens: obj+35b8 / wild).  X3: +1 = never EXACTLY the address of a registered member / sub-object (those are word aligned): whether raw + 4 * r coincides with one depends on the member offsets, which differ between i386 and x86-64 (pad64), and a token that is arch dependent made the x86-64 output differ from the blob's */
  o->ctx_ = g_iw.b[h_rpct(70) ? B_CTX : B_CTX2]; o->k56_ = g_iw.b[h_rpct(70) ? B_K : B_K2]; o->params_ = g_iw.b[B_P2];
  g_obj[X_EC] = (uint8_t *)g_iw.b[B_CTX] + offsetof(PcmFlo47Obj, ecParams_);
  g_obj[X_EC2] = (uint8_t *)g_iw.b[B_CTX2] + offsetof(PcmFlo47Obj, ecParams_);
  g_obj[X_IIR] = (uint8_t *)g_iw.b[B_CTX] + offsetof(PcmFlo47Obj, iirDen_);
  g_obj[X_IIR2] = (uint8_t *)g_iw.b[B_CTX2] + offsetof(PcmFlo47Obj, iirDen_);
  g_obj[X_RX] = (uint8_t *)&o->rxW0_;
  g_obj[X_B8] = o->unk35b8_;
  blk_sync_model();
}
static void world_model(void) { int k; for (k = 0; k < NBLK; k++) g_mw.b[k] = g_blk[g_blkidx[k]].m; }

/* ------------------------------------------------------------------------------------------------ one call */
static void dump_world(int show) {
  char snap[TXTMAX * 2];
  obj_snapshot(snap, sizeof snap, 0);
  h_emit(show, "world %s", snap);
  dump_sliced(show, "D", g_iw.b[B_OBJ], f_V34Obj, NF_V34Obj);
  h_dump_obj(show, "K", g_iw.b[B_K], f_K56HandleObj, NF_K56HandleObj);
  h_dump_obj(show, "K2", g_iw.b[B_K2], f_K56HandleObj, NF_K56HandleObj);
  h_dump_obj(show, "I", g_iw.b[B_INFO], f_P47V92InfoObj, NF_P47V92InfoObj);
  h_dump_obj(show, "I2", g_iw.b[B_INFO2], f_P47V92InfoObj, NF_P47V92InfoObj);
  h_emit(show, "ctx %08x ctx2 %08x", (unsigned)hash_tbl(2166136261u, g_iw.b[B_CTX], f_PcmFlo47Obj, NF_PcmFlo47Obj, IMPL_OBJ), (unsigned)hash_tbl(2166136261u, g_iw.b[B_CTX2], f_PcmFlo47Obj, NF_PcmFlo47Obj, IMPL_OBJ));
}
static void do_call(int sc) {
  uint8_t *t = g_iw.b[B_OBJ];
  unsigned i, lv0;
  int ri, rm, quick = pick_quick(), w8 = (int)wsel(), mode = pick_mode(sc);
  void *pm = g_iw.b[h_rpct(85) ? B_P : B_P2];
  blk_sync_model();
  world_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  g_what = "VPcmV34Create";
  g_snapfn = obj_snapshot;
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  lv0 = dsplibs_debug_level;
  h_emit(g_show, "call %s quick=%d w8=%d mode=%d P=%s lvl=%u", g_what, quick, w8, mode, T_(pm), dsplibs_debug_level);
  ri = f_create(t, quick, w8, pm, mode);
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  dsplibs_debug_level = lv0;
  rm = m_create(quick, w8, pm, mode);
  log_cmp(&g_ml, &g_il, g_what);
  if (memcmp(&g_mc, &g_ic, sizeof g_mc)) h_die3("%s: the script cursors differ", g_what);
  blk_compare(g_what);
  for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", g_what, i);
  if (ri != rm || ri != 0) h_die3("%s: the model returns %d, the impl %d", g_what, rm, ri);
  h_emit(g_show, "ret %d", ri);
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
  g_pokemask = sc == G_POKE ? 0xfffffffeu : sc == G_SEQ ? ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 4)) : 0;
  if (sc != G_SEQ) do_call(sc);
  else {
    nm = (int)rnd_range(2, 5);
    for (k = 0; k < nm; k++) {
      if (k) {
        V34Obj *o = (V34Obj *)g_iw.b[B_OBJ];
        gen_script();
        if (h_rpct(15)) dsplibs_debug_level = pick_lv();
        if (h_rpct(50)) { fill_params((V34ModParObj *)g_iw.b[B_P]); fill_params((V34ModParObj *)g_iw.b[B_P2]); }
        if (h_rpct(40)) o->ctx_ = g_iw.b[h_rpct(50) ? B_CTX : B_CTX2];
        if (h_rpct(40)) o->k56_ = g_iw.b[h_rpct(50) ? B_K : B_K2];
      }
      do_call(sc);
    }
  }
  h_trial_end("v06");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("V06_SEED");
  CovSite *cv;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("V06_ALL")) g_showall = 1;
  if (getenv("V06_NOSCRIBBLE")) g_scribble = 0;
  if (sc < 0 || sc > 3) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  p43_layout_check(); p47_layout_check(); v01_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("V06_TRIALS") ? atoi(getenv("V06_TRIALS")) : (sc == G_ANY || sc == G_V9X ? 12000 : 6000);
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (cv = __start_covsites; cv < __stop_covsites; cv++) {
    if (cv->grp != sc) continue;
    h_emit(1, "COV %s %lu", cv->name, cv->n);
    if (cv->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", cv->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("V06_NOCOV")) ? 2 : 0;
}
