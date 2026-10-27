/*
 * Differential test harness for port chunk V07 (src/vpcm/vpcm_progress.cpp): VPcmV34Progress (blob .text 0xb3c0, 7,278 B), the per-block driver of the V.34 / V.90 / V.92 / K56Flex session.  Derived from
 * test/vpcm/vpcm_create.main.c (recorders + independent model + per-record world snapshot + pokes); ONE C source, no #ifdef around the code under test, built by test/vpcm/vpcm_progress.sh against the blob (dsplibs.o with the
 * callees WEAK so that the recorders below win), against the port (-O0 / -O2: the port object alone) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/vpcm_progress.blob32.out).
 *
 * The world: the V.34 object (V34Obj, test/v34/v01.layout: 0xac4c B on i386, canary tail), the dp parameter block (V34ModParObj), the K56Flex handle (K56HandleObj), the VPcmFloModem ctx (P47's PcmFlo47Obj), its
 * V90Demodulator (opaque), V90Parameters (opaque), the V92Phase2Info block, the V92Modulator / V92BitsToSymbol / V92Transmitter chain read by the bit-rate computation, and the argument buffers (in / out samples, the
 * four bit arrays / counts).  Every callee is a RECORDER: sysdep_memset (records, then clears), adaptecho, modem_serrint, modulatevector, v34handshak, datapumpv34, v90RateReneg(Silence), V34EchoHistoryBackwardClean,
 * VPcmV34InitiateRetrain, GenericIIR<float,double>::process, GenericToneDetector::process, K56FlexFloModem::k56FlexEnterPhase3 / k56FlexRunDemodulator, V90Demodulator::getBitRate, V90Parameters::init,
 * VPcmFloModem::qcLineVerification / runPcmModem / v90RunDemodulator / vPcmResetPhase3Modem and edprintf / dsplibs_debug_printf.  The record carries the arguments as tokens and a SNAPSHOT of the world; a recorder
 * then applies its EFFECT (a function of the script's result word: the tx index advances, the echo sample is produced, the retrain scribbles the state, the demodulator results ...) and may POKE the world by script
 * (the debug level, the state / mode words, flags, counters ...): the time at which the blob / port reloads a member is observable.  The INDEPENDENT MODEL (m_progress: a structured re-derivation from
 * porting/decompiled/dp/v07_vpcmv34progress.txt, written without the port's gotos) runs AFTER the implementation on copies of the world, through the same environment functions, and must reproduce every record, the returned
 * value and every block (exit 3 otherwise).
 *
 * Scenarios (own process each): 0 every mode / state / parameter shape (no pokes), 1 the same with pokes enabled, 2 sequences of calls on one world (the state one call leaves is the next one's input) with pokes.
 */

#define V01_LOGMAX 3072
#include "../v34/v01_common.h"
#include "pcm_bits_to_symbol_layout.h"
#include "v90_modulator_layout.h"
#include "v90_demodulator_progress_layout.h"
#include "vpcm_flo_modem_layout.h"
#include "../v34/v01_layout.h"

/* ---- the function under test ---- */
extern int f_progress(void *, float *, float *, unsigned, int *, int *, int *, int *) L("VPcmV34Progress");

void *h_sysdep_malloc(unsigned size);
void h_sysdep_free(void *q);
void *sysdep_malloc(unsigned size) { return h_sysdep_malloc(size); }
void sysdep_free(void *q) { h_sysdep_free(q); }

/* ------------------------------------------------------------------------------------------------ the world */
enum { B_OBJ, B_P, B_K, B_CTX, B_DEM, B_V90P, B_INFO, B_MOD, B_B2S, B_TX, B_IN, B_OUT, B_A4, B_A5, B_A6, B_A7, NBLK };
enum { X_ANSAM = NBLK, X_IIR, NID };
static const char *const bname[NID] = { "obj", "P", "k56", "ctx", "dem", "v90p", "info", "mod", "b2s", "tx", "in", "out", "a4", "a5", "a6", "a7", "ctx.ansam_", "ctx.iir_" };
typedef struct { uint8_t *b[NBLK]; } W;
static W g_iw, g_mw;
static int g_blkidx[NBLK];
static uint8_t *g_can;
#define OBJ(w) ((V34Obj *)(w)->b[B_OBJ])
#define CTX(w) ((PcmFlo47Obj *)(w)->b[B_CTX])
#define PAR(w) ((V34ModParObj *)(w)->b[B_P])
#define DBG() (dsplibs_debug_level > 1u)
#define IMPL_OBJ ((uint8_t *)g_iw.b[B_OBJ])
enum { NSAMP = 256, GUARD = 16, NBITS = 1100, CANF = 64 };
enum { G_ANY = 0, G_POKE = 1, G_SEQ = 2 };

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
    g_sc.kind[i] = h_rpct(55) ? 0u : 1u + h_rmod(12);
    g_sc.val[i] = h_rnd();
    g_sc.res[i] = h_rnd();
  }
}
static uint16_t pick_h(uint32_t v) { switch (v & 7u) { case 0: return 0; case 1: return 1; case 2: return 0xffffu; case 3: return 0x65; case 4: return 0x66; default: return (uint16_t)(v >> 3); } }
/* the pokes the recorders apply AFTER recording (the same function runs in the impl's and the model's world) */
static void poke(W *w, Cur *c) {
  unsigned k = c->pk++ % NSC, kind = g_sc.kind[k], v = g_sc.val[k];
  V34Obj *o = OBJ(w);
  PcmFlo47Obj *cx = CTX(w);
  V34ModParObj *p = PAR(w);
  if (!kind || !((g_pokemask >> kind) & 1u)) return;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->state_ = (int32_t)(v % 19u); break;
    case 3: o->mode_ = (int32_t)(v % 12u); break;
    case 4: { static const int32_t pm[] = { -1, 0, 1, 2, 3, 0xb, 0xe, 0xf }; if (v & 8) o->pcmMode_ = pm[(v >> 4) & 7u]; else o->k56Mode_ = pm[(v >> 4) & 7u] % 4; break; }
    case 5: if (v & 8) p->flags2_ = (uint8_t)(v >> 4); else p->flags3_ = (uint8_t)(v >> 4); break;
    case 6: switch ((v >> 3) % 3u) { case 0: cx->v92Session_ = (v >> 8) & 1u; break; case 1: cx->qcStatus_ = (uint32_t)(int32_t)(int16_t)pick_h(v >> 8); break; default: cx->b7f5c_ = (uint8_t)(v >> 8); } break;
    case 7: switch ((v >> 3) % 6u) { case 0: o->wabc4_ = pick_h(v >> 8); break; case 1: o->wabc6_ = pick_h(v >> 8); break; case 2: o->wabc8_ = pick_h(v >> 8); break;
              case 3: o->wa248_ = pick_h(v >> 8); break; case 4: o->shortPhase2_ = pick_h(v >> 8); break; default: o->wabca_ = pick_h(v >> 8); } break;
    case 8: switch ((v >> 3) % 3u) { case 0: o->w10_ = (v >> 8) % 80u; break; case 1: o->w114_ = (v >> 8) % 65u; break; default: o->w218_ = (v >> 8) % 61u; o->w21c_ = (v >> 16) % 61u; } break;
    case 9: o->rxW0_ = (uint16_t)(int)((v >> 16) % 12u - 3u); break;   /* (no txIdx_ poke: the script cycles, so a periodic reset would keep the caller's loops from ever ending) */
    case 10: switch ((v >> 3) % 3u) { case 0: o->w240_ = (int32_t)(v >> 8) % 0x300; break; case 1: o->w238_ = (int32_t)((v >> 8) * 0x1000u); break; default: o->w244_ = (v >> 8) * 0x1000u; } break;
    case 11: switch ((v >> 3) % 6u) { case 0: o->waa74_ = (int32_t)(v >> 8) % 0x10000; break; case 1: o->wabdc_ = (v >> 8) % 0x10000u; break; case 2: o->mohD8_ = (v & 0x800u) ? 0xffffffffu : (v >> 12); break;
              case 3: o->mohE2_ = pick_h(v >> 8); break; case 4: o->mohE9_ = (uint8_t)(v >> 8); break; default: o->wAbff_ = (uint8_t)(v >> 8); } break;
    case 12: switch ((v >> 3) % 4u) { case 0: o->b_ac16_ = (uint8_t)(v >> 8); break; case 1: o->b_abfe_ = (uint8_t)(v >> 8); break; case 2: o->mohE0_ = (uint16_t)((v >> 8) % 16u); break;
              default: if (v & 0x200) o->w260_ = (uint16_t)(v >> 12); else o->modeChar_ = (v & 0x100) ? 0x65 : 0x66; } break;
    default: break;
  }
}

/* ------------------------------------------------------------------------------------------------ the snapshot of the world at the moment of a record (light: no big unknown regions except the ring) */
static uint32_t hash_tbl(uint32_t h, const uint8_t *t, const HFld *tbl, int nf, const uint8_t *pbase, int skipx, size_t ringoff) {
  int i;
  for (i = 0; i < nf; i++) {
    const HFld *fl = &tbl[i];
    unsigned n = fl->count ? fl->count : 1, es, j;
    const uint8_t *q = t + fl->off;
    switch (fl->kind) {
      case 'p': case 'o':
        for (j = 0; j < n; j++) { void *v; int id; memcpy(&v, q + j * sizeof v, sizeof v); id = tok_of(v); h = fnv_upd(h, &id, sizeof id); }
        continue;
      case 'e':
        for (j = 0; j < n; j++) { const uint8_t *v; int32_t d; memcpy(&v, q + j * sizeof v, sizeof v); d = !v ? -1 : (v >= pbase && (size_t)(v - pbase) < 0x100000u) ? (int32_t)(v - pbase) : -2; h = fnv_upd(h, &d, sizeof d); }
        continue;
      case 'b': case 'x': es = 1; if (fl->kind == 'x' && skipx && fl->off != ringoff) continue; break;
      case 'h': es = 2; break;
      case 'd': es = 8; break;
      default: es = 4; break;
    }
    h = fnv_upd(h, q, (size_t)n * es);
  }
  return h;
}
static size_t ring_field_off(void) { return offsetof(V34Obj, constPts_); }
static void obj_snapshot(char *out, size_t cap, int model) {
  const W *w = model ? &g_mw : &g_iw;
  uint32_t h = hash_tbl(2166136261u, w->b[B_OBJ], f_V34Obj, NF_V34Obj, IMPL_OBJ, 1, ring_field_off());
  uint32_t c = hash_tbl(2166136261u, w->b[B_CTX], f_PcmFlo47Obj, NF_PcmFlo47Obj, IMPL_OBJ, 1, (size_t)-1);
  uint32_t bp = hash_tbl(2166136261u, w->b[B_P], f_V34ModParObj, NF_V34ModParObj, IMPL_OBJ, 0, (size_t)-1);
  uint32_t bk = hash_tbl(2166136261u, w->b[B_K], f_K56HandleObj, NF_K56HandleObj, IMPL_OBJ, 0, (size_t)-1);
  uint32_t bi = hash_tbl(2166136261u, w->b[B_INFO], f_P47V92InfoObj, NF_P47V92InfoObj, IMPL_OBJ, 0, (size_t)-1);
  uint32_t bin = fnv_upd(2166136261u, w->b[B_IN], (NSAMP + 2 * GUARD) * 4);
  uint32_t bout = fnv_upd(2166136261u, w->b[B_OUT], (NSAMP + 2 * GUARD) * 4);
  uint32_t b57 = fnv_upd(fnv_upd(2166136261u, w->b[B_A5], 32), w->b[B_A7], 32);
  uint32_t b = (bp * 31u + bk) * 31u + bi;
  b = ((b * 31u + bin) * 31u + bout) * 31u + b57;
  snprintf(out, cap, "|h=%08x c=%08x b=%08x lvl=%u", h, c, b, dsplibs_debug_level);
  if (getenv("V07_DBGSNAP")) { char x[200]; snprintf(x, sizeof x, " [p=%08x k=%08x i=%08x in=%08x out=%08x a57=%08x]", bp, bk, bi, bin, bout, b57); strncat(out, x, cap - strlen(out) - 1); }
}

/* ------------------------------------------------------------------------------------------------ the callee environment: record, effect, poke (the impl's recorders and the model call the same functions) */
static const char *nm07(int id) {
  static char b[40][24];
  static int n;
  char *o = b[n++ % 40];
  if (id >= 0 && id < g_nobj) return g_idname[id];
  if (id == ID_NULL) return "null";
  if (id == ID_WILD) return "wild";
  snprintf(o, 24, "A%d+%d", (id - 100) / 4096, (id - 100) % 4096);
  return o;
}
/* a pointer argument: a registered object (the block start / an embedded sub-object), or buffer + element index, else wild */
static const char *pn(const void *p) {
  static char b[40][32];
  static int n;
  char *o = b[n++ % 40];
  int k;
  if (!p) return "null";
  for (k = 0; k < g_nobj; k++) if (g_obj[k] == p) return g_idname[k];
  for (k = B_IN; k <= B_A7; k++) {
    const uint8_t *bb = g_iw.b[k];
    if ((const uint8_t *)p >= bb && (const uint8_t *)p < bb + g_blk[g_blkidx[k]].n) { snprintf(o, 32, "%s+%d", bname[k], (int)(((const uint8_t *)p - bb) / 4)); return o; }
  }
  return "wild";
}
#define T_(p) pn(p)
#define ENVREC(w, l, ...) do { rec_add((l), (w) == &g_mw, 'c', __VA_ARGS__); } while (0)

static void env_ed(W *w, Cur *c, Log *l, char kind, const char *s) { rec_add(l, w == &g_mw, kind, "%s", s); poke(w, c); }
static void *env_memset(W *w, Cur *c, Log *l, void *dst, int ch, unsigned n) {
  uint8_t *impl = (uint8_t *)dst;
  const uint8_t *bo = g_iw.b[B_OUT];
  if (impl < bo || impl + n > bo + g_blk[g_blkidx[B_OUT]].n) h_die3("sysdep_memset: unexpected destination %p n=%u", dst, n);
  ENVREC(w, l, "sysdep_memset(dst=%s c=%d n=%u)", T_(dst), ch, n);
  memset(P(w, dst), ch, n);
  poke(w, c);
  return dst;
}
static uint32_t xs32(uint32_t *s) { *s ^= *s << 13; *s ^= *s >> 17; *s ^= *s << 5; return *s; }
static void env_retrain(W *w, Cur *c, Log *l, void *obj, unsigned mode) {
  uint32_t r = sres(c);
  V34Obj *o = OBJ(w);
  ENVREC(w, l, "VPcmV34InitiateRetrain(obj=%s mode=0x%x)", T_(obj), mode);
  o->state_ = (int32_t)(r % 19u);
  if (r & 0x100) o->mode_ = (int32_t)((r >> 12) % 12u);
  poke(w, c);
}
static void env_iir(W *w, Cur *c, Log *l, void *self, const float *in, float *out, unsigned n) {
  unsigned i;
  const float *pin = (const float *)P(w, in);
  float *pout = (float *)P(w, out);
  ENVREC(w, l, "iir.process(this=%s in=%s out=%s n=%u)", T_(self), T_(in), T_(out), n);
  for (i = 0; i < n; i++) { uint32_t u; float f = pin[i]; memcpy(&u, &f, 4); if (((u >> 23) & 0xffu) == 0xffu) { pout[i] = f; continue; } f = f * 0.75f; pout[i] = f; }   /* infinities and NaNs pass through */
  poke(w, c);
}
static unsigned env_tone(W *w, Cur *c, Log *l, void *self, float *in, unsigned n) {
  uint32_t r = sres(c);
  ENVREC(w, l, "tone.process(this=%s in=%s n=%u)", T_(self), T_(in), n);
  poke(w, c);
  return (r >> 4) & 3u;
}
static void env_k56enter(W *w, Cur *c, Log *l, void *self) { ENVREC(w, l, "K56.enterPhase3(this=%s)", T_(self)); poke(w, c); }
/* a demodulator result: usually a small code, sometimes a value outside every table (negative / huge: the blob's jump-table guards compare UNSIGNED) */
static int weird_ret(uint32_t r, int code) {
  static const int w[4] = { -1, (int)0x80000000u, 0x7fffffff, 9 };
  return ((r >> 24) & 15u) == 0 ? w[(r >> 16) & 3u] : code;
}
static int env_k56run(W *w, Cur *c, Log *l, void *self, float *in, unsigned n, int *a, int *b) {
  uint32_t r = sres(c);
  ENVREC(w, l, "K56.run(this=%s in=%s n=%u a4=%s a5=%s)", T_(self), T_(in), n, T_(a), T_(b));
  poke(w, c);
  return weird_ret(r, (int)((r >> 5) % 9u));
}
static int env_getbitrate(W *w, Cur *c, Log *l, void *self) {
  uint32_t r = sres(c);
  ENVREC(w, l, "dem.getBitRate(this=%s)", T_(self));
  poke(w, c);
  return (int)(r >> 3);
}
static void env_v90init(W *w, Cur *c, Log *l, void *self) { ENVREC(w, l, "V90Parameters.init(this=%s)", T_(self)); poke(w, c); }
static int env_qc(W *w, Cur *c, Log *l, void *self, float *in, float *out, unsigned n, int *a, int *b, int *cc, int *d) {
  uint32_t r = sres(c);
  unsigned i;
  float *pout = (float *)P(w, out);
  ENVREC(w, l, "ctx.qcLineVerification(this=%s in=%s out=%s n=%u a4=%s a5=%s a6=%s a7=%s)", T_(self), T_(in), T_(out), n, T_(a), T_(b), T_(cc), T_(d));
  for (i = 0; i < n; i++) pout[i] = (float)(int)(i + (r & 7u));
  poke(w, c);
  return (int)((r >> 6) & 1u);
}
static int env_runpcm(W *w, Cur *c, Log *l, void *self, float *in, float *out, unsigned n, int *a, int *b, int *cc, int *d) {
  uint32_t r = sres(c);
  unsigned i;
  float *pout = (float *)P(w, out);
  ENVREC(w, l, "ctx.runPcmModem(this=%s in=%s out=%s n=%u a4=%s a5=%s a6=%s a7=%s)", T_(self), T_(in), T_(out), n, T_(a), T_(b), T_(cc), T_(d));
  for (i = 0; i < n; i++) pout[i] = (float)(int)(i * 3u + (r & 7u));
  poke(w, c);
  return weird_ret(r, (int)((r >> 5) % 11u));
}
static int env_v90run(W *w, Cur *c, Log *l, void *self, float *in, unsigned n, int *a, int *b) {
  uint32_t r = sres(c);
  ENVREC(w, l, "ctx.v90RunDemodulator(this=%s in=%s n=%u a4=%s a5=%s)", T_(self), T_(in), n, T_(a), T_(b));
  poke(w, c);
  return weird_ret(r, (int)((r >> 5) % 11u));
}
static void env_resetph3(W *w, Cur *c, Log *l, void *self) { ENVREC(w, l, "ctx.vPcmResetPhase3Modem(this=%s)", T_(self)); poke(w, c); }
static void env_echo(W *w, Cur *c, Log *l, const char *nm, void *obj) {
  uint32_t r = sres(c);
  V34Obj *o = OBJ(w);
  ENVREC(w, l, "%s(obj=%s w260=%u)", nm, T_(obj), (unsigned)o->w260_);
  o->w25e_ = (uint16_t)(r >> 3);
  poke(w, c);
}
/* the handshake / modulator / rate-renegotiation callees advance the transmit index (so that the caller's loops end) and sometimes move the state word */
static void env_txstep(W *w, Cur *c, Log *l, const char *nm, void *obj) {
  static const int32_t interesting[9] = { 1, 2, 4, 5, 0xd, 0xf, 0x10, 0, 3 };
  uint32_t r = sres(c);
  V34Obj *o = OBJ(w);
  ENVREC(w, l, "%s(obj=%s txIdx=%d)", nm, T_(obj), (int)(int16_t)o->txIdx_);
  o->txIdx_ = (uint16_t)(o->txIdx_ + 1u + (r & 3u));
  if (((r >> 8) & 63u) == 0) o->state_ = interesting[(r >> 14) % 9u];
  if (((r >> 8) & 63u) == 1) o->state_ = (int32_t)(((uint32_t)o->state_ + 1u + (r >> 14) % 3u) % 19u);
  if (((r >> 8) & 63u) == 5 && o->state_ != 1) o->state_ = 1;
  if (((r >> 8) & 7u) == 3 && o->state_ == 0) o->state_ = 1;
  if (((r >> 8) & 31u) == 7) o->rxW0_ = (uint16_t)(int)((r >> 14) % 10u - 2u);
  poke(w, c);
}
static void env_clean(W *w, Cur *c, Log *l, void *obj, unsigned n) { ENVREC(w, l, "V34EchoHistoryBackwardClean(obj=%s n=%u)", T_(obj), n); poke(w, c); }

/* ------------------------------------------------------------------------------------------------ the recorders (the blob's callees, weakened in the copy of dsplibs.o) */
extern void *rc_memset(void *, int, unsigned) L("sysdep_memset");
extern void rc_retrain(void *, unsigned char) L("VPcmV34InitiateRetrain");
extern void rc_iir(void *, const float *, float *, unsigned) L("_ZN10GenericIIRIfdE7processEPKfPfj");
extern unsigned rc_tone(void *, float *, unsigned) L("_ZN19GenericToneDetector7processEPfj");
extern void rc_k56enter(void *) L("_ZN15K56FlexFloModem18k56FlexEnterPhase3Ev");
extern int rc_k56run(void *, float *, unsigned, int *, int *) L("_ZN15K56FlexFloModem21k56FlexRunDemodulatorEPfjPiS1_");
extern int rc_getbitrate(void *) L("_ZNK14V90Demodulator10getBitRateEv");
extern void rc_v90init(void *) L("_ZN13V90Parameters4initEv");
extern int rc_qc(void *, float *, float *, unsigned, int *, int *, int *, int *) L("_ZN12VPcmFloModem18qcLineVerificationEPfS0_jPiS1_S1_S1_");
extern int rc_runpcm(void *, float *, float *, unsigned, int *, int *, int *, int *) L("_ZN12VPcmFloModem11runPcmModemEPfS0_jPiS1_S1_S1_");
extern int rc_v90run(void *, float *, unsigned, int *, int *) L("_ZN12VPcmFloModem17v90RunDemodulatorEPfjPiS1_");
extern void rc_resetph3(void *) L("_ZN12VPcmFloModem20vPcmResetPhase3ModemEv");
extern int rc_adapt(void *) L("adaptecho");
extern int rc_serr(void *) L("modem_serrint");
extern void rc_modvec(void *) L("modulatevector");
extern void rc_hs(void *) L("v34handshak");
extern void rc_dpump(void *) L("datapumpv34");
extern int rc_rr(void *) L("v90RateReneg");
extern int rc_rrs(void *) L("v90RateRenegSilence");
extern void rc_clean(void *, unsigned) L("V34EchoHistoryBackwardClean");
void *rc_memset(void *d, int c, unsigned n) { return env_memset(IW, IC, IL, d, c, n); }
void rc_retrain(void *o, unsigned char m) { env_retrain(IW, IC, IL, o, m); }
void rc_iir(void *t, const float *i, float *o, unsigned n) { env_iir(IW, IC, IL, t, i, o, n); }
unsigned rc_tone(void *t, float *i, unsigned n) { return env_tone(IW, IC, IL, t, i, n); }
void rc_k56enter(void *t) { env_k56enter(IW, IC, IL, t); }
int rc_k56run(void *t, float *i, unsigned n, int *a, int *b) { return env_k56run(IW, IC, IL, t, i, n, a, b); }
int rc_getbitrate(void *t) { return env_getbitrate(IW, IC, IL, t); }
void rc_v90init(void *t) { env_v90init(IW, IC, IL, t); }
int rc_qc(void *t, float *i, float *o, unsigned n, int *a, int *b, int *c, int *d) { return env_qc(IW, IC, IL, t, i, o, n, a, b, c, d); }
int rc_runpcm(void *t, float *i, float *o, unsigned n, int *a, int *b, int *c, int *d) { return env_runpcm(IW, IC, IL, t, i, o, n, a, b, c, d); }
int rc_v90run(void *t, float *i, unsigned n, int *a, int *b) { return env_v90run(IW, IC, IL, t, i, n, a, b); }
void rc_resetph3(void *t) { env_resetph3(IW, IC, IL, t); }
int rc_adapt(void *o) { env_echo(IW, IC, IL, "adaptecho", o); return 0; }
int rc_serr(void *o) { env_echo(IW, IC, IL, "modem_serrint", o); return 0; }
void rc_modvec(void *o) { env_txstep(IW, IC, IL, "modulatevector", o); }
void rc_hs(void *o) { env_txstep(IW, IC, IL, "v34handshak", o); }
void rc_dpump(void *o) { env_txstep(IW, IC, IL, "datapumpv34", o); }
int rc_rr(void *o) { env_txstep(IW, IC, IL, "v90RateReneg", o); return 0; }
int rc_rrs(void *o) { env_txstep(IW, IC, IL, "v90RateRenegSilence", o); return 0; }
void rc_clean(void *o, unsigned n) { env_clean(IW, IC, IL, o, n); }
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

/* ------------------------------------------------------------------------------------------------ the independent model (from porting/decompiled/dp/v07_vpcmv34progress.txt): structured, one function per mode */
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
static int16_t m_t16(float f) { double d = f; if (!(d > -32769.0 && d < 32768.0)) return (int16_t)0x8000; return (int16_t)(int)d; }   /* fistp WORD, truncating */
static int16_t *m_ring(int idx) { return (int16_t *)((uint8_t *)mo()->constPts_ + 0x4b0) + idx; }
static void m_ringpush(int16_t s) {
  V34Obj *o = mo();
  int idx = sx(o->w2aa6_);
  unsigned nx;
  *m_ring(idx) = s;
  nx = (uint16_t)(idx + 1);
  o->w2aa6_ = (uint16_t)(nx > 0x257u ? 0 : nx);
}

typedef struct { float *in, *out; unsigned n; int *a4, *a5, *a6, *a7; void *ctxI, *k56I; } MA;   /* impl-world pointers: in / out advance with the blob's stack slots */
static MA ma;
#define MCTX() ((PcmFlo47Obj *)mp(ma.ctxI))
#define OBJI ((void *)IMPL_OBJ)
static void *ctx_ansam(void) { return g_obj[X_ANSAM]; }
static void *ctx_iir(void) { return g_obj[X_IIR]; }
static void m_ringfill(const float *srcI) {   /* srcI: impl-world pointer */
  const float *s = (const float *)mp(srcI);
  unsigned i;
  for (i = 0; (int)i < (int)ma.n; i++) m_ringpush(m_t16(s[i]));
}
static void m_zero(void) { uint32_t *p = (uint32_t *)mp(ma.out); unsigned i; for (i = 0; i < ma.n; i++) p[i] = 0; }
static void m_compact(void) {
  V34Obj *o = mo();
  unsigned head = o->w21c_, tail = o->w218_, k = 0;
  while (head < tail) { o->txWords_[k++] = o->txWords_[head++]; }
  o->w21c_ = 0; o->w218_ = k;
}
static int m_wanted(void) {
  V34Obj *o = mo();
  int d;
  if (o->w10_ == 0) return 0;
  d = (int)o->w10_ - (int)o->w218_;
  return d > 0 ? d << 4 : 0;
}
/* the end of mode 3 / mode 1 / the demodulator result blocks: compaction, *a7 */
static int m_finish_queue(void) {
  V34Obj *o = mo();
  int st;
  m_compact();
  st = o->state_;
  *(int *)mp(ma.a7) = st > 3 ? m_wanted() : 0;
  return st;
}
static int m_tail(int st);

/* the echo canceller loop (modes 1 and 3) */
static void m_echo(void) {
  V34Obj *o = mo();
  unsigned i;
  for (i = 0; i < ma.n; i++) {
    float *ip = (float *)mp(ma.in);
    float *op = (float *)mp(ma.out);
    unsigned cnt, smp;
    int s;
    o->w260_ = (uint16_t)m_t16(*ip);
    env_echo(MW, MC, ML, "adaptecho", OBJI);
    cnt = o->w254_;
    smp = o->w260_;
    if (cnt != 0) {
      uint32_t acc;
      CV(G_ANY, "echo: countdown running");
      acc = o->w258_ + (uint32_t)(int32_t)(int16_t)smp;
      cnt = (cnt - 1u) & 0xffffu;
      o->w254_ = (uint16_t)cnt;
      if ((cnt & 0x7fu) == 0) {
        CV(G_ANY, "echo: 128-sample block end");
        o->w256_ = (uint16_t)((sx(o->w256_) >> 1) + ((int32_t)acc >> 8));
        if (DBG()) { CV(G_ANY, "echo: DC print"); o->w258_ = acc; mpf('d', "Estimated DC = %d  (acc = %d)\n", sx(o->w256_), (int)acc); }
        o->w258_ = 0;
      } else {
        o->w258_ = acc;
      }
      smp = o->w260_;
    }
    s = (int16_t)((int)smp - sx(o->w256_));
    m_ringpush((int16_t)s);
    *(float *)mp(ma.in) = (float)s;
    ma.in++;
    *(float *)mp(ma.out) = (float)sx(o->w25e_);
    ma.out++;
  }
}

/* the retrain detector + the return of the function (blob b53f) */
static int m_tail(int st) {
  V34Obj *o = mo();
  int N, i, flag = 0;
  int c1, c2, c3;
  if ((unsigned)(st - 3) > 3u) { CV(G_ANY, "tail: state outside 3..6"); return st; }
  CV(G_ANY, "tail: detector runs");
  N = sx(o->w2aa6_);
  c1 = sx(o->rdc_); c2 = sx(o->rde_); c3 = sx(o->rd10_);
  for (i = 0; i < N; i++) {
    int x = *m_ring(i);
    int t3 = (int16_t)((sx(o->rd2_) * c3 + 0x2000) >> 14);
    int t2 = (int16_t)((sx(o->rd0_) * c2 + 0x2000) >> 14);
    int w4 = sx(o->rd4_);
    int t1 = (int16_t)((w4 * c1 + 0x2000) >> 14);
    int y;
    uint32_t s1, s2, cnt;
    y = x + t2 - t3 - t1 + (int)o->rd6_;
    y = (int16_t)y;
    o->rd2_ = o->rd0_;
    o->rd4_ = (uint16_t)x;
    o->rd6_ = (uint16_t)w4;
    o->rd0_ = (uint16_t)y;
    s1 = o->rd14_ + (uint32_t)((x * x + 0x20) >> 6);
    s2 = o->rd18_ + (uint32_t)((y * y + 0x20) >> 6);
    cnt = o->rd1c_ + 1u;
    if (cnt != 0x40u) {
      CV(G_ANY, "tail: accumulate");
      o->rd1c_ = cnt; o->rd18_ = s2; o->rd14_ = s1;
    } else {
      int e = (int)s1 >> 2;
      CV(G_ANY, "tail: 64-sample block end");
      if (e > (int)s2 && (int)s1 > 0x249f0 && (int)s2 <= 0x22550f) {
        uint32_t k = o->rd8_ + 1u;
        CV(G_ANY, "tail: notch block detected");
        o->rd14_ = s1; o->rd18_ = s2; o->rd1c_ = 0x40;
        o->rd8_ = k;
        if (DBG()) { CV(G_ANY, "tail: detector print"); mpf('d', "********** retrainDetector() notchDetectSigCnt = %d energyInp>>NOTCH_IN_OUT_RATIO_SHIFT = %d energyOut = %d\r\n", (int)k, e, (int)s2); }
      } else {
        if (e > (int)s2) CV(G_ANY, "tail: block not detected (energy)"); else CV(G_ANY, "tail: block not detected (ratio)");
        o->rd8_ = 0;
      }
      o->rd14_ = 0; o->rd18_ = 0; o->rd1c_ = 0;
    }
    if ((int)o->rd8_ > 5) { flag = 1; break; }
  }
  if (!flag) return o->state_;
  CV(G_ANY, "tail: retrain detected by the tone detector");
  if (DBG()) mpf('d', "VPcmV34Main: Retrain Detected by Tone detector !\r\n");
  env_retrain(MW, MC, ML, OBJI, 0);
  return o->state_;
}

/* modes 6 / 7 / 8 / 9 / 10 and the silence states */
static int m_mode67(int mode) {
  V34Obj *o = mo();
  int eax, edx;
  void *ctxI = ma.ctxI;
  if (mode == 6) {
    eax = (int)o->wabdc_; edx = o->waa74_;
    if (eax < edx) { CV(G_ANY, "mode6: wabdc < waa74"); o->state_ = 0xd; goto m7; }
    if ((unsigned)(eax - ma.n) < (unsigned)edx) {
      CV(G_ANY, "mode6: window not reached");
      if (DBG()) { CV(G_ANY, "mode6: 3-way print"); mpf('d', "VPcmV34Main: ANSam not detected on out-going, assuming 3-way call supported !\r\n"); mode = o->mode_; }
    }
    o->state_ = 0xe;
  }
m7:
  m_zero();
  *(int *)mp(ma.a5) = 0; *(int *)mp(ma.a7) = 0;
  if (mode == 7) { CV(G_ANY, "mode7: entry"); o->state_ = 0xd; }
  {
    int ne = (int)(ma.n + o->wabdc_);
    int hold = (int)o->mohD8_;
    o->wabdc_ = (uint32_t)ne;
    if (hold != -1 && ne >= hold) {
      CV(G_ANY, "hold: timeout expired");
      if (DBG()) mpf('d', "VPcmV34Main: Modem On Hold Timeout expired , ending session !!!\r\n");
      o->state_ = 0x10; o->mode_ = 0xa;
    }
  }
  if (env_tone(MW, MC, ML, ctx_ansam(), ma.in, ma.n) != 0) {
    int moved = 1;
    if (o->mode_ == 7) {
      if (o->mohE9_ != 0 && (int)o->wabdc_ <= 0xbb7f) {
        CV(G_ANY, "hold: ANSam later case");
        if (DBG()) mpf('d', "VPcmV34Main: ANSam detected on out going call (later case, after %d smp) ! assuming no 3-way call...\r\n", (int)o->wabdc_);
        o->mohE2_ = 2;
      } else {
        CV(G_ANY, "hold: ANSam on hold");
        if (DBG()) mpf('d', "VPcmV34Main: ANSam detected on hold ! requesting Reconnect...\r\n");
      }
    } else {
      CV(G_ANY, "mode6: ANSam on out-going");
      if (DBG()) mpf('d', "VPcmV34Main: ANSam detected on out going call ! assuming no 3-way call...\r\n");
      o->mohE2_ = 2;
    }
    if (moved) { o->state_ = 0xf; o->mode_ = 9; }
  } else CV(G_ANY, "hold: no ANSam");
  (void)ctxI;
  m_ringfill(ma.in);
  return m_tail(o->state_);
}
static int m_mode5(void) {
  V34Obj *o = mo();
  int big;
  m_zero();
  big = (uint32_t)o->w238_ > 0x464ffu;
  *(int *)mp(ma.a5) = 0; o->state_ = 0xc; *(int *)mp(ma.a7) = 0;
  if (big) {
    CV(G_ANY, "mode5: 30 s elapsed");
    if (DBG()) mpf('d', "VPcmV34Main: waiting for user response for 30sec, initiating retrain !\r\n");
    env_retrain(MW, MC, ML, OBJI, 0);
  }
  m_ringfill(ma.in);
  return m_tail(o->state_);
}
static int m_mode4(void) {
  V34Obj *o = mo();
  PcmFlo47Obj *ctx = MCTX();
  int r = env_qc(MW, MC, ML, ma.ctxI, ma.in, ma.out, ma.n, ma.a4, ma.a5, ma.a6, ma.a7);
  if (r == 0) { CV(G_ANY, "mode4: verification running"); o->state_ = 0xa; m_ringfill(ma.in); return m_tail(o->state_); }
  {
    unsigned lvl = dsplibs_debug_level;
    V34ModParObj *pp;
    if (lvl > 1u) { mpf('d', "VPcmV34Main: Line verification period completed !!!\r\n"); lvl = dsplibs_debug_level; }
    pp = (V34ModParObj *)mp(o->params_);
    if (!(pp->flags2_ & 0x20u)) {
      CV(G_ANY, "mode4: DP manager forces full phase 2");
      o->shortPhase2_ = 0;
      if (lvl > 1u) mpf('d', "VPcmV34Main: Moving to full phase2 upon DP Manager setting...\r\n");
    } else if (sx(o->wabca_) == (int)ctx->qcStatus_) {
      CV(G_ANY, "mode4: same line verification");
      if (lvl > 1u) mpf('d', "VPcmV34Main: Moving to short phase2 due to same line verification...\r\n");
      o->shortPhase2_ = 1;
    } else {
      CV(G_ANY, "mode4: false same line verification");
      o->shortPhase2_ = 0;
      if (lvl > 1u) mpf('d', "VPcmV34Main: Moving to full phase2 due to false same line verification...\r\n");
    }
    if (o->shortPhase2_ == 0) {
      P47V92InfoObj *info = (P47V92InfoObj *)mp(CTX(MW)->v92Info_);
      CV(G_ANY, "mode4: full phase 2: V90Parameters::init");
      info->shortPhase2Local_ = 0;
      o->wabca_ = 0;
      env_v90init(MW, MC, ML, CTX(MW)->v90Params_);
    }
    o->mode_ = 0; o->state_ = 0;
    m_ringfill(ma.in);
    return m_tail(o->state_);
  }
}
/* one result block that stores the sample ring from the echo-cancelled block (the V.92 results) */
static int m_pcm_result(int r) {
  V34Obj *o = mo();
  PcmFlo47Obj *ctx = MCTX();
  unsigned i;
  switch (r) {
    case 1: o->state_ = 3; break;
    case 2:
      CV(G_ANY, "pcm: result 2 (data)");
      if (o->b_ac16_ == 0) {
        int bits = env_getbitrate(MW, MC, ML, ctx->v90Dem_);
        P47V92InfoObj *unused = 0;
        void *modI = ctx->v92Mod_;
        ModV92Obj *mod = (ModV92Obj *)mp(modI);
        uint32_t val = 0;
        (void)unused;
        o->dgC08_ = (uint32_t)bits;
        if (mod->state_ == 3) {
          TxObj *tx = (TxObj *)mp(((B2S92 *)mp(mod->b2s_))->tx_);
          uint32_t e = tx->w0_ * 8000u;
          long double v = 0.5L + (long double)e * (long double)0.083333335816860198974609375f;
          CV(G_ANY, "pcm: V.92 modulator in data phase");
          val = (uint32_t)(int64_t)v;
        }
        o->dgC04_ = val;
        o->b_ac16_ = 1;
      }
      o->wabc4_ = 1; o->state_ = 4; o->wa248_ = 0;
      break;
    case 3: o->wabc4_ = 1; o->state_ = 5; o->wa248_ = 0; break;
    case 4: o->state_ = 0xb; o->mode_ = 5; o->w238_ = 0; break;
    case 5: env_retrain(MW, MC, ML, OBJI, 0x5c); break;
    case 6: env_retrain(MW, MC, ML, OBJI, 0x5a); break;
    case 7: env_retrain(MW, MC, ML, OBJI, 0x22); break;
    case 8: o->mode_ = 0xa; o->state_ = 0x10; break;
    default: o->state_ = 2; break;   /* 0 and everything above 8 */
  }
  for (i = 0; (int)i < (int)ma.n; i++) m_ringpush(m_t16(((float *)ctx->ecOut_)[i]));
  return m_tail(o->state_);
}
static int m_mode2(void) {
  int r = env_runpcm(MW, MC, ML, ma.ctxI, ma.in, ma.out, ma.n, ma.a4, ma.a5, ma.a6, ma.a7);
  CV(G_ANY, "mode2: runPcmModem");
  return m_pcm_result(r < 0 || r > 8 ? 99 : r);
}
/* mode 3: K56Flex */
static int m_mode3(void) {
  V34Obj *o = mo();
  int r;
  o->blkLen_ = (uint16_t)ma.n;
  while (sx(o->txIdx_) < sx(o->blkLen_)) {
    if (o->txFlags_ & 0x10u) env_txstep(MW, MC, ML, "modulatevector", OBJI); else env_txstep(MW, MC, ML, "v34handshak", OBJI);
  }
  if (ma.n) m_echo();
  ma.in -= ma.n;
  r = env_k56run(MW, MC, ML, ma.k56I, ma.in, ma.n, ma.a4, ma.a5);
  switch (r) {
    case 1: o->state_ = 3; break;
    case 2: case 3: {
      V34ModParObj *pp;
      CV(G_ANY, "mode3: result 2/3");
      if (r == 3) o->state_ = 5;
      o->wE4c_ = 1;
      if (o->b_ac16_ == 0) { o->b_ac16_ = 1; o->dgC08_ = ((K56HandleObj *)mp(ma.k56I))->word0_; }
      pp = (V34ModParObj *)mp(o->params_);
      pp->flags3_ = (uint8_t)(pp->flags3_ | 4u);
      break;
    }
    case 4: env_retrain(MW, MC, ML, OBJI, 0x38); break;
    case 5: env_retrain(MW, MC, ML, OBJI, 0x22); break;
    case 6: o->mode_ = 0xa; o->state_ = 0x10; break;
    default: o->state_ = 2; break;
  }
  CV(G_ANY, "mode3: finish");
  return m_tail(m_finish_queue());
}
/* mode 1: V.90 */
static int m_mode1(void) {
  V34Obj *o = mo();
  PcmFlo47Obj *ctx = MCTX();
  int eax = o->pcmMode_, st = o->state_, r;
  if (eax <= 1 || st <= 1) {
    CV(G_ANY, "mode1: wrong state print");
    if (DBG()) { mpf('d', "VPcmV34Main: BUG BUG, Wrong state & statuses... (v90receiver = %d, cond = %d ; DPstatus = %d, cond = %d)\r\n", eax, 2, st, 2); st = o->state_; }
    return m_tail(st);
  }
  o->blkLen_ = (uint16_t)ma.n;
  if (sx(o->txIdx_) < (int)(int16_t)(uint16_t)ma.n) {
    for (;;) {
      if (eax > 0xe) { CV(G_ANY, "mode1: rate reneg silence"); env_txstep(MW, MC, ML, "v90RateRenegSilence", OBJI); }
      else if (eax > 0xa) { CV(G_ANY, "mode1: rate reneg"); env_txstep(MW, MC, ML, "v90RateReneg", OBJI); }
      else if (o->txFlags_ & 0x10u) env_txstep(MW, MC, ML, "modulatevector", OBJI);
      else env_txstep(MW, MC, ML, "v34handshak", OBJI);
      if (sx(o->txIdx_) >= sx(o->blkLen_)) break;
      eax = o->pcmMode_;
    }
  }
  if (ma.n) m_echo();
  ma.in -= ma.n;
  r = env_v90run(MW, MC, ML, ma.ctxI, ma.in, ma.n, ma.a4, ma.a5);
  switch (r) {
    case 1: o->state_ = 3; break;
    case 2:
      CV(G_ANY, "mode1: result 2");
      o->wE4c_ = 1;
      if (o->b_ac16_ == 0) o->dgC08_ = (uint32_t)env_getbitrate(MW, MC, ML, ctx->v90Dem_);
      if (o->wabc4_ == 0) o->wabc4_ = (o->wabc6_ != 0 && o->wabc8_ != 0) ? 1 : 0;
      o->wa248_ = 0;
      break;
    case 3: o->wE4c_ = 1; o->state_ = 5; o->wa248_ = 0; break;
    case 4: o->state_ = 0xb; o->mode_ = 5; o->w238_ = 0; break;
    case 5: env_retrain(MW, MC, ML, OBJI, 0x5a); break;
    case 7: env_retrain(MW, MC, ML, OBJI, 0x22); break;
    case 8: o->mode_ = 0xa; o->state_ = 0x10; break;
    default: o->state_ = 2; break;
  }
  if (o->b_abfe_ != 0 && ma.n > 0x30u) {
    unsigned two = 2u * ma.n;
    CV(G_ANY, "mode1: request output sample clear");
    if (DBG()) mpf('d', "VPcmV34Main: requestOutputSampleClear (asking %d samples clear) !!!\r\n", (int)two);
    o->b_abfe_ = 0;
    ma.out -= ma.n;
    env_memset(MW, MC, ML, ma.out, 0, ma.n * 4u);
    o->wac44_ = two; o->wac40_ = 1; o->wac48_ = 0;
    {
      V34ModParObj *pp = (V34ModParObj *)mp(o->params_);
      pp->b51_ = (uint8_t)(pp->b51_ | 1u);
    }
    env_clean(MW, MC, ML, OBJI, two + ma.n);
  }
  return m_tail(m_finish_queue());
}
static int m_mode8(void) {
  V34Obj *o = mo();
  unsigned ne;
  int lt;
  m_zero();
  *(int *)mp(ma.a5) = 0; *(int *)mp(ma.a7) = 0;
  ne = ma.n + (unsigned)o->waa74_;
  lt = (int)ne < (int)o->wabdc_;
  o->waa74_ = (int32_t)ne;
  if (lt) { CV(G_ANY, "mode8: waiting"); o->state_ = 0; return m_tail(0); }
  CV(G_ANY, "mode8: delay ended");
  o->mode_ = 9;
  if (DBG()) { o->state_ = 0; mpf('d', "VPcmV34Main: Delay ended, indicating reconnect request...\r\n"); }
  o->state_ = 0xf;
  return m_tail(0xf);
}

/* mode 0: the V.34 data pump and its state transitions */
static int m_mode0(void) {
  V34Obj *o = mo();
  PcmFlo47Obj *ctx = MCTX();
  int st = o->state_, st0 = st;
  unsigned i, w;
  V34ModParObj *pp;
  if (ma.n) {
    unsigned left;
    for (left = ma.n; left != 0; left--) {
      float x = *(float *)mp(ma.in);
      ma.in++;
      o->w260_ = (uint16_t)m_t16(x);
      env_echo(MW, MC, ML, "modem_serrint", OBJI);
      *(float *)mp(ma.out) = (float)sx(o->w25e_);
      ma.out++;
      if (sx(o->txIdx_) < sx(o->blkLen_) || sx(o->rxW0_) > 5) {
        CV(G_ANY, "mode0: handshake / data pump call");
        if (o->txFlags_ & 0x10u) env_txstep(MW, MC, ML, "datapumpv34", OBJI); else env_txstep(MW, MC, ML, "v34handshak", OBJI);
      }
    }
    st = o->state_;
  }
  pp = (V34ModParObj *)mp(o->params_);
  if (st != st0 && (unsigned)(st - 4) <= 1u) { CV(G_ANY, "mode0: entered state 4/5"); pp->flags3_ = (uint8_t)(pp->flags3_ | 4u); st = o->state_; }
  if (st == 0) {
    unsigned dd = (uint32_t)o->w238_ - o->w244_;
    if (dd > 0x12bfu && dd < ma.n + 0x12c0u) {
      CV(G_ANY, "mode0: CAS mask window");
      if (DBG()) mpf('d', "VPcmV34Main: Masking CAS detection after %d in train...\r\n", (int)dd);
      pp = (V34ModParObj *)mp(o->params_);
      pp->flags3_ = (uint8_t)(pp->flags3_ & 0xfbu);
      st = o->state_;
    }
  }
  m_compact();
  *(int *)mp(ma.a7) = st > 3 ? m_wanted() : 0;
  {
    int *bp = (int *)mp(ma.a4);
    w = 0;
    for (i = 0; ; i++) {
      unsigned k;
      uint32_t v;
      w = o->w114_;
      if (w <= i) break;
      v = o->rxWords_[i];
      for (k = 0; k < 16; k++) *bp++ = (int)((v >> k) & 1u);
    }
    o->w114_ = 0;
    *(int *)mp(ma.a5) = (int)(w << 4);
  }
  switch ((unsigned)st > 0x10u ? 99 : st) {
    case 0x10:
      CV(G_ANY, "mode0: state 0x10");
      if (DBG()) { mpf('d', "VPcmV34Main: End Transmission indicated by V.34 / Handshake...\r\n"); st = o->state_; }
      o->mode_ = 0xa;
      break;
    case 0xf: {
      int eax = (o->mohE2_ == 0 ? 0xbb80 : 0) + 0x2580;
      CV(G_ANY, "mode0: state 0xf");
      o->mode_ = 8; o->waa74_ = 0; o->wabdc_ = (uint32_t)eax; o->state_ = 0;
      st = 0;
      if (DBG()) { mpf('d', "VPcmV34Main: Reconnect request indicated from phase2 (waiting min time = %d before reconenct request)...\r\n", eax); st = o->state_; }
      break;
    }
    case 0xd: {
      static const int hold[14] = { 0, 10, 20, 30, 40, 60, 120, 180, 240, 360, 480, 720, 960, -1 };
      unsigned e = o->mohE0_;
      int v = e > 13u ? 0 : hold[e];
      CV(G_ANY, "mode0: state 0xd");
      o->mohD8_ = (uint32_t)v; o->wabdc_ = 0;
      if (DBG()) { mpf('d', "VPcmV34Main: Modem On Hold approved by phase2 (ISP timeout is %d seconds) !!\r\n", v); v = (int)o->mohD8_; st = o->state_; }
      if (v > 0) o->mohD8_ = (uint32_t)(v * 0x2580);
      o->mode_ = 7;
      break;
    }
    case 2:
      CV(G_ANY, "mode0: state 2");
      if (o->pcmMode_ > 1) {
        if (ctx->v92Session_ == 0) { if (DBG()) mpf('d', "VPcmV34Main: Moving to Phase3 Modem V90..\r\n"); o->mode_ = 1; }
        else { if (DBG()) mpf('d', "VPcmV34Main: Moving to Phase3 Modem V92..\r\n"); o->mode_ = 2; }
      }
      if (o->k56Mode_ > 1) { if (DBG()) mpf('d', "VPcmV34Main: Moving to Phase3 Modem K56Flex..\r\n"); o->mode_ = 3; o->w254_ = 0; }
      st = o->state_;
      break;
    case 1: {
      int eax;
      if (st0 == 0) {
        CV(G_ANY, "mode0: state 1 from idle");
        o->wAbff_ = 0; o->w240_ = 0;
        pp = (V34ModParObj *)mp(o->params_);
        if ((pp->flags3_ & 2u) && o->wabc4_ != 0 && o->wa248_ == 0) {
          o->wa248_ = 1;
          o->echoMid_ = sx(o->baud_) * 5 + sx(o->period_) - 0x24c;
        } else {
          o->echoMid_ = ((sx(o->baud_) * 15) >> 3) + sx(o->period_) - 0x24c;
        }
        mpf('e', "VPcmV34Main: phase3halfDuplexLength = %d symbols (baud %d)\r\n", o->echoMid_, sx(o->baud_));
        if (o->pcmMode_ > 1) env_resetph3(MW, MC, ML, ma.ctxI);
        else if (o->k56Mode_ > 1) env_k56enter(MW, MC, ML, ma.k56I);
      }
      eax = (int)(ma.n + (unsigned)o->w240_);
      o->w240_ = eax;
      if (o->wAbff_ == 0) {
        if (eax <= 0x5f) { CV(G_ANY, "mode0: wait before P2 complete"); if (DBG()) mpf('d', "VPcmV34Main: Wait (before P2 COMPLETE)...\r\n"); return 0; }
        CV(G_ANY, "mode0: first P2 complete");
        o->wAbff_ = 1;
        if (DBG()) mpf('d', "VPcmV34Main: Indicating First P2 COMPLETE... (after %d)\r\n", (int)o->w240_);
        return 1;
      }
      if (o->wAbff_ == 1) {
        V34ModParObj *p2 = (V34ModParObj *)mp(o->params_);
        CV(G_ANY, "mode0: P2 complete silence");
        eax += (int)p2->w6c_;
        o->w240_ = eax;
        if (DBG()) mpf('d', "On PHASE2_COMPLETE: added Silence = %d, p2DelayCntr = %d\r\n", (int)p2->w6c_, eax);
      }
      if (o->w240_ <= 0x240) {
        CV(G_ANY, "mode0: wait after P2 complete");
        if (DBG()) mpf('d', "VPcmV34Main: Wait (after P2 COMPLETE)...\r\n");
        o->wAbff_ = (uint8_t)(o->wAbff_ + 1u);
        return 1;
      }
      if (st0 != 1) { CV(G_ANY, "mode0: state 1 tail, not from state 1"); return m_tail(o->state_); }
      CV(G_ANY, "mode0: state 1 tail, from state 1");
      if (o->pcmMode_ > 1 && ctx->v92Session_ != 0) { if (DBG()) mpf('d', "VPcmV34Main: Moving to Phase3 Modem V92..\r\n"); o->mode_ = 2; }
      if (o->k56Mode_ > 1) { if (DBG()) mpf('d', "VPcmV34Main: Moving to Phase3 Modem K56Flex..\r\n"); o->mode_ = 3; o->w254_ = 0; }
      st = o->state_;
      break;
    }
    default: break;
  }
  return m_tail(st);
}

static int m_progress(float *inI, float *outI, unsigned nIn, int *a4I, int *a5I, int *a6I, int *a7I) {
  V34Obj *o = mo();
  int mode;
  ma.in = inI; ma.out = outI; ma.a4 = a4I; ma.a5 = a5I; ma.a6 = a6I; ma.a7 = a7I;
  ma.ctxI = o->ctx_; ma.k56I = o->k56_;
  if (o->w262_ == 0) { CV(G_ANY, "entry: module not created"); return o->state_; }
  ma.n = nIn & ~3u;
  o->w238_ = (int32_t)(((uint32_t)o->w238_ + ma.n) & 0x7fffffffu);
  mode = o->mode_;
  if (mode == 0 || (mode == 1 && o->modeChar_ == 0x66) || (mode == 3 && o->modeChar_ == 0x65)) {
    int cnt = *(int *)mp(ma.a7) >> 4, k, bit;
    int *bits = (int *)mp(ma.a6);
    unsigned pos = 0;
    CV(G_ANY, "pack: bit packer entry");
    for (k = 0; k < cnt; k++) {
      uint32_t wd = 0;
      for (bit = 0; bit < 16; bit++) wd += ((unsigned)bits[pos++] & 1u) << bit;
      o->txWords_[o->w218_] = wd;
      o->w218_++;
      cnt = *(int *)mp(ma.a7) >> 4;
    }
  }
  if (MCTX()->b7f5c_ != 0) {
    CV(G_ANY, "entrance filter on");
    env_iir(MW, MC, ML, ctx_iir(), ma.in, ma.in, ma.n);
    mode = o->mode_;
  }
  switch ((unsigned)mode > 10u ? 99 : mode) {
    case 0: return m_mode0();
    case 1: return m_mode1();
    case 2: return m_mode2();
    case 3: return m_mode3();
    case 4: return m_mode4();
    case 5: return m_mode5();
    case 6: case 7: return m_mode67(mode);
    case 8: return m_mode8();
    case 9: CV(G_ANY, "mode9"); m_zero(); *(int *)mp(ma.a5) = 0; *(int *)mp(ma.a7) = 0; o->state_ = 0xf; return m_tail(0xf);
    case 10: CV(G_ANY, "mode10"); m_zero(); *(int *)mp(ma.a5) = 0; *(int *)mp(ma.a7) = 0; o->state_ = 0x10; return m_tail(0x10);
    default:
      CV(G_ANY, "default: illegal modem state");
      if (DBG()) mpf('d', "VPcmV34Main: Illegal Modem State !!!!\r\n");
      return m_tail(o->state_);
  }
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

static void new_world(void) {
  uint8_t *raw;
  V34Obj *o;
  PcmFlo47Obj *cx;
  int k;
  size_t osz = sizeof(V34Obj);
  memset(&g_iw, 0, sizeof g_iw);
  blk_reset();
  g_nobj = NID;
  for (k = 0; k < NID; k++) { g_obj[k] = NULL; g_idname[k] = bname[k]; }
  raw = (uint8_t *)h_obj_new(osz + CANF);
  memset(raw + osz, 0x5a, CANF);
  g_can = raw + osz;
  fill_tbl(raw, f_V34Obj, NF_V34Obj);
  reg(B_OBJ, raw, osz);
  fill_tbl(newblk(B_P, sizeof(V34ModParObj)), f_V34ModParObj, NF_V34ModParObj);
  fill_tbl(newblk(B_K, sizeof(K56HandleObj)), f_K56HandleObj, NF_K56HandleObj);
  cx = (PcmFlo47Obj *)newblk(B_CTX, sizeof(PcmFlo47Obj));
  fill_tbl((uint8_t *)cx, f_PcmFlo47Obj, NF_PcmFlo47Obj);
  fill_blk(newblk(B_DEM, sizeof(DmdObj)), sizeof(DmdObj));
  fill_blk(newblk(B_V90P, 0x600), 0x600);
  fill_tbl(newblk(B_INFO, sizeof(P47V92InfoObj)), f_P47V92InfoObj, NF_P47V92InfoObj);
  fill_tbl(newblk(B_MOD, sizeof(ModV92Obj)), f_ModV92Obj, NF_ModV92Obj);
  fill_tbl(newblk(B_B2S, sizeof(B2S92)), f_B2S92, NF_B2S92);
  fill_tbl(newblk(B_TX, sizeof(TxObj)), f_TxObj, NF_TxObj);
  fill_blk(newblk(B_IN, (NSAMP + 2 * GUARD) * 4), (NSAMP + 2 * GUARD) * 4);     /* (the guards stay as filled here: h_obj_new's own fill depends on the sizes of the blocks before it) */
  fill_blk(newblk(B_OUT, (NSAMP + 2 * GUARD) * 4), (NSAMP + 2 * GUARD) * 4);
  fill_blk(newblk(B_A4, NBITS * 4), NBITS * 4);
  fill_blk(newblk(B_A5, 32), 32);
  fill_blk(newblk(B_A6, NBITS * 4), NBITS * 4);
  fill_blk(newblk(B_A7, 32), 32);
  cx->v90Dem_ = g_iw.b[B_DEM]; cx->v90Params_ = g_iw.b[B_V90P]; cx->v92Mod_ = g_iw.b[B_MOD]; cx->v92Info_ = g_iw.b[B_INFO];
  ((ModV92Obj *)g_iw.b[B_MOD])->b2s_ = g_iw.b[B_B2S];
  ((B2S92 *)g_iw.b[B_B2S])->tx_ = g_iw.b[B_TX];
  o = (V34Obj *)raw;
  o->ctx_ = g_iw.b[B_CTX]; o->k56_ = g_iw.b[B_K]; o->params_ = g_iw.b[B_P];
  g_obj[X_ANSAM] = (uint8_t *)cx + offsetof(PcmFlo47Obj, ansIir_);
  g_obj[X_IIR] = (uint8_t *)cx + offsetof(PcmFlo47Obj, iirDen_);
  blk_sync_model();
}
static void world_model(void) { int k; for (k = 0; k < NBLK; k++) g_mw.b[k] = g_blk[g_blkidx[k]].m; }

/* ------------------------------------------------------------------------------------------------ one call: steering of the world and the arguments */
static int pick_mode(void) {
  unsigned r = h_rmod(100);
  if (r < 88) return (int)h_rmod(11);
  if (r < 93) return 11 + (int)h_rmod(2);
  if (r < 96) return -1 - (int)h_rmod(2);
  return (int)h_rnd();
}
static int pick_state(int mode) {
  unsigned r = h_rmod(100);
  if (mode == 3 && r < 20) return -1 - (int)h_rmod(5);   /* the K56Flex result blocks keep a negative state_ for the signed `st <= 3` of the queue tail */
  if (mode == 0 && r < 70) { static const int s[] = { 0, 1, 2, 4, 5, 0xd, 0xf, 0x10, 3, 6, 0xb, 0xc }; return s[h_rmod(12)]; }
  if (r < 80) return (int)h_rmod(19);
  if (r < 86) return (int)h_rmod(8);
  if (r < 91) return -1 - (int)h_rmod(5);
  if (r < 94) return 0x7fffffff - (int)h_rmod(3);
  return (int)h_rnd() >> h_rmod(32);
}
static uint16_t pick_small_h(void) { return h_rpct(40) ? 0 : h_rpct(60) ? 1 : h_rpct(50) ? (uint16_t)h_rmod(6) : (uint16_t)h_rnd(); }
static uint32_t pick_cnt(void) { switch (h_rmod(6)) { case 0: return 0; case 1: return 1; case 2: return 0x5f; case 3: return 0x60; default: return h_rmod(0x300); } }
static float pick_sample(void) {
  switch (h_rmod(14)) {
    case 0: return 0.0f;
    case 1: return -0.0f;
    case 2: return (float)((int)h_rmod(65536) - 32768);
    case 3: { static const uint32_t bits[] = { 0x46fffe00u, 0x46ffff00u, 0x47000000u, 0xc7000000u, 0xc7000100u, 0x477fff00u, 0xc6ffffffu, 0x46fffa00u, 0xc6fffe00u }; return h_float_from_bits(bits[h_rmod(9)]); }
    case 4: return 1.0e10f;
    case 5: return h_float_from_bits(0x7fc00000u);
    case 6: return h_rpct(50) ? h_float_from_bits(0x7f800000u) : h_float_from_bits(0xff800000u);
    case 7: return (float)(int)(h_rmod(2000)) + 0.75f;
    case 8: return (h_rpct(50) ? 0.5f : -0.5f) + (float)(int)h_rmod(3);
    case 9: return -(float)(int)h_rmod(900) - 0.25f;
    default: return (float)((int)h_rmod(6001) - 3000) + (h_rpct(50) ? 0.0f : 0.375f);
  }
}
/* integer sinusoid: s[k+1] = ((c * s[k]) >> 14) - s[k-1] (c = 2 cos(theta) * 2^14): a clean notch input for the retrain detector */
static void gen_sine(int16_t *dst, int count, int c, int *a_io, int *b_io) {
  int a = *a_io, b = *b_io, k;
  for (k = 0; k < count; k++) { int t = ((c * b) >> 14) - a; if (t > 32767) t = 32767; if (t < -32768) t = -32768; dst[k] = (int16_t)b; a = b; b = t; }
  *a_io = a; *b_io = b;
}
static const int k_cos[] = { 30000, 20000, 10000, 0, -10000, -20000, -30000, 32000 };
static void steer(int sc, int mode, unsigned *nIn) {
  V34Obj *o = OBJ(IW);
  PcmFlo47Obj *cx = CTX(IW);
  V34ModParObj *p = PAR(IW);
  K56HandleObj *k = (K56HandleObj *)g_iw.b[B_K];
  ModV92Obj *md = (ModV92Obj *)g_iw.b[B_MOD];
  TxObj *tx = (TxObj *)g_iw.b[B_TX];
  P47V92InfoObj *info = (P47V92InfoObj *)g_iw.b[B_INFO];
  float *in = (float *)g_iw.b[B_IN] + GUARD, *out = (float *)g_iw.b[B_OUT] + GUARD;
  int *a4 = (int *)g_iw.b[B_A4], *a6 = (int *)g_iw.b[B_A6], *a5 = (int *)g_iw.b[B_A5], *a7 = (int *)g_iw.b[B_A7];
  unsigned i, n;
  int notch = h_rpct(35), cidx = (int)h_rmod(8);
  (void)sc; (void)k; (void)info;
  /* the block length passed in */
  n = 4u * h_rmod(41);
  if (h_rpct(25)) n += h_rmod(4);
  if (h_rpct(4)) n = h_rmod(4);
  if (h_rpct(8)) n = 4u * (40u + h_rmod(10));
  *nIn = n;
  n &= ~3u;
  /* the object */
  o->w262_ = h_rpct(95) ? 1 : (h_rpct(50) ? 0 : (uint16_t)h_rnd());
  o->mode_ = mode;
  o->modeChar_ = h_rpct(40) ? 0x65 : h_rpct(66) ? 0x66 : (uint16_t)h_rnd();
  o->state_ = pick_state(mode);
  o->txIdx_ = (uint16_t)(int)(h_rmod(n + 12u) - 6u);
  o->blkLen_ = (uint16_t)h_rnd();
  o->rxW0_ = (uint16_t)(int)(h_rmod(12) - 3);
  o->txFlags_ = (uint16_t)((h_rnd() & ~0x10u) | (h_rpct(50) ? 0x10u : 0u));
  { static const int pm[] = { -3, 0, 1, 2, 5, 9, 0xa, 0xb, 0xc, 0xe, 0xf, 0x20 }; o->pcmMode_ = h_rpct(75) ? pm[h_rmod(12)] : (int32_t)h_rnd(); }
  { static const int km[] = { -1, 0, 1, 2, 3 }; o->k56Mode_ = h_rpct(85) ? km[h_rmod(5)] : (int32_t)h_rnd(); }
  o->wabc4_ = pick_small_h(); o->wabc6_ = pick_small_h(); o->wabc8_ = pick_small_h(); o->wa248_ = pick_small_h(); o->shortPhase2_ = pick_small_h();
  o->wabca_ = h_rpct(60) ? (uint16_t)h_rmod(4) : (uint16_t)h_rnd();
  cx->qcStatus_ = h_rpct(50) ? (uint32_t)(int32_t)(int16_t)o->wabca_ : h_rpct(50) ? (uint32_t)h_rmod(4) : h_rnd();
  o->baud_ = (uint16_t)(h_rpct(60) ? 2400 + 200 * h_rmod(9) : h_rnd()); o->period_ = (uint16_t)(h_rpct(70) ? h_rmod(60) : h_rnd());
  o->echoMid_ = (int32_t)h_rnd();
  o->b_ac16_ = h_rpct(50) ? 0 : (uint8_t)(1 + h_rmod(255)); o->b_abfe_ = h_rpct(50) ? 0 : (uint8_t)(1 + h_rmod(255));
  o->wAbff_ = h_rpct(30) ? 0 : h_rpct(40) ? 1 : (uint8_t)h_rnd();
  o->mohE9_ = h_rpct(50) ? 0 : (uint8_t)(1 + h_rmod(255));
  o->mohE2_ = h_rpct(50) ? 0 : (uint16_t)h_rmod(5);
  o->mohE0_ = h_rpct(85) ? (uint16_t)h_rmod(15) : (uint16_t)h_rnd();
  o->w254_ = h_rpct(35) ? 0 : h_rpct(30) ? (uint16_t)(1 + h_rmod(130)) : h_rpct(30) ? 0x80 : (uint16_t)h_rnd();
  o->w256_ = (uint16_t)(int)(h_rmod(400) - 200); o->w258_ = h_rpct(50) ? h_rmod(0x8000) : h_rnd();
  /* the word queues */
  o->w218_ = h_rpct(30) ? 0 : h_rmod(57);
  o->w21c_ = h_rpct(30) ? 0 : h_rmod(60);
  o->w10_ = h_rpct(30) ? 0 : h_rpct(70) ? h_rmod(80) : h_rpct(30) ? (h_rpct(50) ? 0xffffffffu : 0xfffffffeu) : h_rpct(30) ? o->w218_ : h_rpct(30) ? o->w218_ + 1 : h_rnd();
  o->w114_ = h_rpct(35) ? 0 : h_rmod(65);
  /* counters: the compare windows of mode 5, 0, 6/7/8 and the tail */
  { uint32_t base = h_rpct(60) ? h_rnd() & 0x7fffffffu : (h_rpct(50) ? 0x7fffffffu - h_rmod(8) : h_rmod(0x20000));
    static const unsigned xs[] = { 0, 0x12bf, 0x12c0, 0x12c1 };
    o->w238_ = (int32_t)base;
    if (h_rpct(40)) o->w238_ = (int32_t)(0x464ffu - n + h_rmod(8));
    if (h_rpct(60)) { uint32_t nw = ((uint32_t)o->w238_ + n) & 0x7fffffffu; unsigned x = h_rpct(50) ? xs[h_rmod(4)] : (h_rpct(50) ? n + 0x12bfu + h_rmod(4) : h_rnd()); o->w244_ = nw - x; }
    else o->w244_ = h_rnd();
  }
  if (mode == 0 && o->state_ == 0 && h_rpct(70)) {   /* the CAS-mask window of the state-0 post-processing: dd = (w238_ + n) - w244_ at its edges */
    static const unsigned edge[] = { 0x12bfu, 0x12c0u, 0x12c1u, 0x12c0u - 1u, 0u };
    unsigned nw = ((uint32_t)o->w238_ + n) & 0x7fffffffu, x = h_rpct(50) ? edge[h_rmod(5)] : n + 0x12c0u + h_rmod(5) - 3u;
    if (x == 0u) x = n + 0x12bfu + h_rmod(3);
    o->w244_ = nw - x;
  }
  if (mode == 0 && o->state_ != 0 && h_rpct(60)) {   /* the state-1 tail: the P2 COMPLETE counters */
    static const uint8_t ab[] = { 0, 1, 2, 5, 0x80 };
    o->wAbff_ = ab[h_rmod(5)];
    if (h_rpct(70)) o->w240_ = (int32_t)(h_rpct(50) ? 0x240 - (int)n + (int)h_rmod(7) - 3 : 0x5f - (int)n + (int)h_rmod(5) - 2);
  }
  else o->w240_ = h_rpct(50) ? (int32_t)(0x5f - n + h_rmod(3) - 1) : h_rpct(50) ? (int32_t)(0x240 - n + h_rmod(5) - 2) : (int32_t)h_rnd() >> h_rmod(28);
  p->w6c_ = h_rpct(40) ? 0 : h_rpct(60) ? h_rmod(0x300) : h_rnd();
  o->wabdc_ = h_rpct(50) ? h_rmod(0xbd00) : h_rpct(50) ? 0xbb7fu + h_rmod(3) - 1 : h_rnd();
  o->waa74_ = h_rpct(40) ? (int32_t)o->wabdc_ - (int32_t)n + (int32_t)h_rmod(5) - 2 : h_rpct(50) ? (int32_t)(h_rmod(0xc000)) : (int32_t)h_rnd();
  o->mohD8_ = h_rpct(25) ? 0xffffffffu : h_rpct(50) ? (uint32_t)((int32_t)o->wabdc_ + (int32_t)h_rmod(5) - 2 + (int32_t)n) : h_rpct(50) ? h_rmod(0x20000) : h_rnd();
  p->flags2_ = (uint8_t)((h_rnd() & ~0x20u) | (h_rpct(55) ? 0x20u : 0u));
  p->flags3_ = (uint8_t)h_rnd();
  p->b51_ = (uint8_t)h_rnd();
  cx->b7f5c_ = h_rpct(30) ? (uint8_t)(1 + h_rmod(255)) : 0;
  cx->v92Session_ = h_rpct(50) ? 0 : h_rpct(70) ? 1 : h_rnd();
  md->state_ = h_rpct(60) ? 3 : h_rnd();
  tx->w0_ = h_rpct(70) ? h_rmod(60) : h_rnd();
  /* the retrain detector record and the ring */
  { int idx; unsigned r = h_rmod(100);
    if (r < 55) idx = (int)h_rmod(0x258);
    else if (r < 80) idx = 0x257 - (int)h_rmod(4);
    else if (r < 92) idx = (int)h_rmod(0x2ed);
    else idx = -(int)(1 + h_rmod(0x258));
    o->w2aa6_ = (uint16_t)idx; }
  if (notch) {
    int s0 = 0, s1 = 1 + (int)h_rmod(3000), c = k_cos[cidx];
    int cnt = 64 + (int)h_rmod(180);
    gen_sine((int16_t *)((uint8_t *)o->constPts_ + 0x4b0), cnt, c, &s0, &s1);
    o->w2aa6_ = (uint16_t)cnt;
    o->rdc_ = (uint16_t)c; o->rde_ = (uint16_t)(c * 8 / 10 + (int)h_rmod(3)); o->rd10_ = (uint16_t)(10650 + h_rmod(40) - 20);
    o->rd0_ = 0; o->rd2_ = 0; o->rd4_ = 0; o->rd6_ = 0; o->rd14_ = 0; o->rd18_ = 0;
    o->rd1c_ = h_rpct(60) ? 0 : h_rmod(64);
    o->rd8_ = h_rpct(70) ? 5 : h_rmod(8);
    /* the input block continues the sinusoid */
    {
      int16_t tmp[NSAMP];
      gen_sine(tmp, NSAMP, c, &s0, &s1);
      for (i = 0; i < NSAMP; i++) in[i] = (float)(int)tmp[i];
    }
  } else {
    o->rdc_ = (uint16_t)h_rnd(); o->rde_ = (uint16_t)h_rnd(); o->rd10_ = (uint16_t)h_rnd();
    o->rd0_ = (uint16_t)h_rnd(); o->rd2_ = (uint16_t)h_rnd(); o->rd4_ = (uint16_t)h_rnd(); o->rd6_ = (uint16_t)h_rnd();
    o->rd8_ = h_rpct(60) ? h_rmod(8) : h_rnd();
    o->rd14_ = h_rpct(50) ? h_rmod(1000) : h_rnd();
    o->rd18_ = h_rpct(50) ? h_rmod(1000) : h_rnd();
    o->rd1c_ = h_rpct(50) ? 0x3f : h_rpct(70) ? h_rmod(64) : h_rnd();
    for (i = 0; i < NSAMP; i++) in[i] = pick_sample();
  }
  for (i = 0; i < NSAMP; i++) out[i] = pick_sample();
  for (i = 0; i < NBITS; i++) { a6[i] = h_rpct(50) ? (int)(h_rnd() & 1u) : (int)h_rnd(); a4[i] = (int)h_rnd(); }
  a5[0] = (int)h_rnd(); a7[0] = h_rpct(30) ? 0 : (int)(16u * h_rmod(4) + (h_rpct(50) ? h_rmod(16) : 0u));
  if (h_rpct(3)) a7[0] = -16 * (int)h_rmod(3);
  /* the detector's thresholds: one ring sample ends a 64-sample block and the sums land ON the edges (mode 0, state 3 .. 6, nothing else touches the ring) */
  if (h_rpct(6)) {
    int16_t *ring = (int16_t *)((uint8_t *)o->constPts_ + 0x4b0);
    int x = (int)h_rmod(2001) - 1000, c1 = (int)(int16_t)h_rnd(), c2 = (int)(int16_t)h_rnd(), c3 = (int)(int16_t)h_rnd();
    int r0 = (int)(int16_t)h_rnd(), r2 = (int)(int16_t)h_rnd(), r4 = (int)(int16_t)h_rnd();
    unsigned r6 = h_rnd() & 0xffffu, kind = h_rmod(4);
    int t3 = (int16_t)((r2 * c3 + 0x2000) >> 14), t2 = (int16_t)((r0 * c2 + 0x2000) >> 14), t1 = (int16_t)((r4 * c1 + 0x2000) >> 14);
    int y = (int16_t)(x + t2 - t3 - t1 + (int)r6);
    uint32_t q1 = (uint32_t)((x * x + 0x20) >> 6), q2 = (uint32_t)((y * y + 0x20) >> 6), s1, s2;
    switch (kind) {
      case 0: s1 = 0x249f0u + h_rmod(5) - 2u; s2 = h_rmod(30000); break;
      case 1: s2 = 0x22550fu + h_rmod(4) - 2u; s1 = 9200000u + h_rmod(1000000); break;
      case 2: s1 = 150001u + h_rmod(100000); s2 = ((uint32_t)((int32_t)s1 >> 2)) + h_rmod(3) - 1u; break;
      default: s1 = h_rnd() >> h_rmod(16); s2 = h_rnd() >> h_rmod(16); break;
    }
    o->mode_ = 0; o->state_ = 3 + (int32_t)h_rmod(4); *nIn = h_rmod(4);
    ring[0] = (int16_t)x; o->w2aa6_ = 1;
    o->rdc_ = (uint16_t)c1; o->rde_ = (uint16_t)c2; o->rd10_ = (uint16_t)c3;
    o->rd0_ = (uint16_t)r0; o->rd2_ = (uint16_t)r2; o->rd4_ = (uint16_t)r4; o->rd6_ = (uint16_t)r6;
    o->rd14_ = s1 - q1; o->rd18_ = s2 - q2; o->rd1c_ = 0x3f;
    o->rd8_ = h_rpct(50) ? 5 : h_rmod(8);
    o->w262_ = 1;
  }
  /* keep the packer inside the queue: tail + words <= 64 */
  { int words = a7[0] >> 4; if (words > 0 && o->w218_ + (unsigned)words > 60u) o->w218_ = 60u - (unsigned)words; }
  (void)a5;
}

/* ------------------------------------------------------------------------------------------------ one call */
static uint32_t hash_words(const void *p, size_t n) { return fnv_upd(2166136261u, p, n); }
static void dump_world(int show) {
  char snap[TXTMAX * 2];
  obj_snapshot(snap, sizeof snap, 0);
  h_emit(show, "world %s", snap);
  dump_sliced(show, "D", g_iw.b[B_OBJ], f_V34Obj, NF_V34Obj);
  h_emit(show, "ctx %08x P %08x k %08x info %08x",
         (unsigned)hash_tbl(2166136261u, g_iw.b[B_CTX], f_PcmFlo47Obj, NF_PcmFlo47Obj, IMPL_OBJ, 0, (size_t)-1),
         (unsigned)hash_tbl(2166136261u, g_iw.b[B_P], f_V34ModParObj, NF_V34ModParObj, IMPL_OBJ, 0, (size_t)-1),
         (unsigned)hash_tbl(2166136261u, g_iw.b[B_K], f_K56HandleObj, NF_K56HandleObj, IMPL_OBJ, 0, (size_t)-1),
         (unsigned)hash_tbl(2166136261u, g_iw.b[B_INFO], f_P47V92InfoObj, NF_P47V92InfoObj, IMPL_OBJ, 0, (size_t)-1));
  h_emit(show, "bufs in %08x out %08x a4 %08x a5 %08x a6 %08x a7 %08x",
         (unsigned)hash_words(g_iw.b[B_IN], (NSAMP + 2 * GUARD) * 4), (unsigned)hash_words(g_iw.b[B_OUT], (NSAMP + 2 * GUARD) * 4), (unsigned)hash_words(g_iw.b[B_A4], NBITS * 4),
         (unsigned)hash_words(g_iw.b[B_A5], 32), (unsigned)hash_words(g_iw.b[B_A6], NBITS * 4), (unsigned)hash_words(g_iw.b[B_A7], 32));
  h_emit(show, "a5 %d a7 %d out0 %08x out1 %08x", ((int *)g_iw.b[B_A5])[0], ((int *)g_iw.b[B_A7])[0], ((uint32_t *)g_iw.b[B_OUT])[GUARD], ((uint32_t *)g_iw.b[B_OUT])[GUARD + 1]);
}
static void do_call(int sc) {
  uint8_t *t = g_iw.b[B_OBJ];
  unsigned i, lv0, nIn;
  int ri, rm, mode = pick_mode();
  float *in, *out;
  int *a4 = (int *)g_iw.b[B_A4], *a5 = (int *)g_iw.b[B_A5], *a6 = (int *)g_iw.b[B_A6], *a7 = (int *)g_iw.b[B_A7];
  steer(sc, mode, &nIn);
  in = (float *)g_iw.b[B_IN] + GUARD; out = (float *)g_iw.b[B_OUT] + GUARD;
  blk_sync_model();
  world_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  g_what = "VPcmV34Progress";
  g_snapfn = obj_snapshot;
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  lv0 = dsplibs_debug_level;
  h_emit(g_show, "call %s mode=%d nIn=%u lvl=%u", g_what, mode, nIn, dsplibs_debug_level);
  ri = f_progress(t, in, out, nIn, a4, a5, a6, a7);
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  dsplibs_debug_level = lv0;
  rm = m_progress(in, out, nIn, a4, a5, a6, a7);
  log_cmp(&g_ml, &g_il, g_what);
  if (memcmp(&g_mc, &g_ic, sizeof g_mc)) h_die3("%s: the script cursors differ", g_what);
  blk_compare(g_what);
  for (i = 0; i < CANF; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", g_what, i);
  if (ri != rm) h_die3("%s: the model returns %d, the impl %d", g_what, rm, ri);
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
  g_pokemask = sc == G_POKE ? 0xfffffffeu : sc == G_SEQ ? 0xfffffffeu : 0;
  if (sc != G_SEQ) do_call(sc);
  else {
    nm = (int)rnd_range(2, 5);
    for (k = 0; k < nm; k++) {
      if (k) {
        gen_script();
        if (h_rpct(15)) dsplibs_debug_level = pick_lv();
      }
      do_call(sc);
    }
  }
  h_trial_end("v07");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("V07_SEED");
  CovSite *cv;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("V07_ALL")) g_showall = 1;
  if (sc < 0 || sc > 2) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  p29_layout_check(); p39_layout_check(); p43_layout_check(); p47_layout_check(); v01_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("V07_TRIALS") ? atoi(getenv("V07_TRIALS")) : (sc == G_ANY ? 12000 : 6000);
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (cv = __start_covsites; cv < __stop_covsites; cv++) {
    if (cv->grp != sc) continue;
    h_emit(1, "COV %s %lu", cv->name, cv->n);
    if (cv->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", cv->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("V07_NOCOV")) ? 2 : 0;
}
