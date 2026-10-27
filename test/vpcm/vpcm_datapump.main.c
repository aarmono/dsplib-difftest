/*
 * Differential test harness for port chunk V05 (src/vpcm/vpcm_datapump.cpp): datapumpv34 (blob .text 0x71960, 1,028 B), the per-block V.34 data pump.  Derived from test/vpcm/vpcm_info1a_retrain.main.c (recorders +
 * independent model + per-record world snapshot + pokes); ONE C source, no #ifdef around the code under test, built by test/vpcm/vpcm_datapump.sh against the blob (dsplibs.o with the callees WEAK so
 * that the recorders below win), against the port (-O0 / -O2: the port object alone) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/vpcm_datapump.blob32.out).
 *
 * The only world block is the V.34 object (V34Obj, test/v34/v01.layout: 0xac4c B on i386, canary tail).  Every callee is a RECORDER: v34handshak, modulatevector, receiver (the three loop drivers: each
 * applies a scripted effect to the object - advance the transmit index, drop the receive gate rxW0_, change the front-end words and the two clocks w238_ / w248_ - which is what makes the loops
 * terminate; after SETTLE recorder calls in one top-level call the drivers force the exit condition), v34handshakinit (records the mode), VPcmV34IndicateLocalRRN / RemoteRRN, dsplibs_debug_printf.  The
 * record carries the arguments and a SNAPSHOT of the whole object (hashed), a recorder then may POKE the object by script (the words datapumpv34 re-reads after a call: observable reload timing).
 * The INDEPENDENT MODEL (m_datapump: a goto transliteration of the control-flow graph of the disassembly, a different shape than the port's structured code) runs AFTER the implementation on a copy of
 * the object, through the same environment functions, and must reproduce every record and the final object (exit 3 otherwise).
 *
 * Scenarios (own process each): 0 w2218_ <= 1 (modulatevector + receive + rate renegotiation decision), 1 w2218_ > 1 (v34handshak loop), 2 either with every poke kind enabled, 3 sequences of calls on
 * one object (the state one call leaves is the next one's input).
 */

#include "../v34/v01_common.h"
#include "../v34/v01_layout.h"

extern void f_dp(void *) L("datapumpv34");

void *h_sysdep_malloc(unsigned size);
void h_sysdep_free(void *q);
void *sysdep_malloc(unsigned size) { return h_sysdep_malloc(size); }
void sysdep_free(void *q) { h_sysdep_free(q); }

enum { B_OBJ, NBLK };
enum { NID = 1 };
static const char *const bname[NID] = { "obj" };
typedef struct { uint8_t *b[NBLK]; } W;
static W g_iw, g_mw;
static int g_blkidx[NBLK];
static uint8_t *g_can;
#define OBJ(w) ((V34Obj *)(w)->b[B_OBJ])
#define DBG() (dsplibs_debug_level > 1u)
enum { G_RUN = 0, G_HS = 1, G_POKE = 2, G_SEQ = 3 };
enum { SETTLE = 20 };

typedef struct { uint32_t kind[NSC], val[NSC], val2[NSC], res[NSC]; } Script;
typedef struct { unsigned pk, rv, ncall; } Cur;
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

static int32_t add32(int32_t a, int32_t b) { return (int32_t)((uint32_t)a + (uint32_t)b); }
static uint32_t sres(Cur *c) { return g_sc.res[c->rv++ % NSC]; }

static int16_t pick_s(uint32_t v) {   /* small signed word with boundary values */
  switch (v & 15u) {
    case 0: return 0; case 1: return 1; case 2: return -1; case 3: return 2; case 4: return 3; case 5: return 5; case 6: return 6; case 7: return 0x7fff;
    case 8: return (int16_t)0x8000; case 9: return -2; case 10: return 4; default: return (int16_t)((v >> 4) % 11u) - 2;
  }
}
static uint16_t pick_cnt(uint32_t v) {   /* counters / thresholds: small values, the wrap boundary, random */
  switch (v & 15u) {
    case 0: return 0; case 1: return 1; case 2: return 0x7fff; case 3: return 0x8000; case 4: return 0xffff; case 5: return 0x7ffe; case 6: return 8; case 7: return 9;
    case 8: return 17; case 9: return 16; case 10: return (uint16_t)(v >> 4); default: return (uint16_t)((v >> 4) % 20u);
  }
}
static uint16_t pick_gain(uint32_t v) {
  switch (v & 7u) { case 0: return 0; case 1: return 1; case 2: return 2; case 3: return 3; case 4: return 4; case 5: return 0xffff; case 6: return 0x8000; default: return (uint16_t)((v >> 3) % 7u); }
}
static int32_t pick_step(uint32_t v) {
  switch (v & 15u) {
    case 0: case 1: case 2: return 0; case 3: return 1; case 4: return 0x1000; case 5: return 0x10000; case 6: return 0x23281; case 7: return 0x23280; case 8: return 0x119401; case 9: return 0x119400;
    case 10: return -1; case 11: return 0x46501; case 12: return 0x1fffff; default: return (int32_t)(v >> 4);
  }
}
static int32_t pick_diff(uint32_t v) {
  static const int32_t d[] = { 0, 1, 0x23280, 0x23281, 0x23280 - 1, 0x119400, 0x119401, 0x119400 - 1, 0x46500, 0x46501, 0x46500 - 1, -1, 0x7fffffff, (int32_t)0x80000000u, 0x1000, 0x30000 };
  return v & 1u ? d[(v >> 1) % (sizeof d / sizeof d[0])] : (int32_t)(v >> 1);
}

static void gen_script(void) {
  unsigned i;
  for (i = 0; i < NSC; i++) {
    g_sc.kind[i] = h_rpct(60) ? 0u : 1u + h_rmod(15);
    g_sc.val[i] = h_rnd();
    g_sc.val2[i] = h_rnd();
    g_sc.res[i] = h_rnd();
  }
}

/* the generic pokes (after a recorder): the words datapumpv34 reads again later */
static void poke(W *w, Cur *c) {
  unsigned k = c->pk++ % NSC, kind = g_sc.kind[k], v = g_sc.val[k];
  V34Obj *o = OBJ(w);
  if (!kind || !((g_pokemask >> kind) & 1u)) return;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->txIdx_ = (uint16_t)pick_s(v); break;
    case 3: o->blkLen_ = (uint16_t)pick_s(v); break;
    case 4: o->rxW0_ = pick_s(v); break;
    case 5: o->w47e_ = pick_cnt(v); break;
    case 6: o->rxFlags_ = (uint16_t)(v & 0xffffu); break;
    case 7: o->waa96_ = pick_gain(v); break;
    case 8: o->waa98_ = (uint16_t)v; break;
    case 9: o->w4b6_ = pick_cnt(v); o->w4b8_ = pick_cnt(v >> 8); o->w4ba_ = pick_cnt(v >> 16); break;
    case 10: o->w4bc_ = pick_cnt(v); o->w4be_ = pick_cnt(v >> 8); o->w4c0_ = pick_cnt(v >> 16); break;
    case 11: o->w2218_ = (v & 1) ? (int32_t)(v >> 1) : (int32_t)(v >> 1 & 3u); break;
    case 12: o->w388_ = pick_cnt(v); break;
    case 13: o->state_ = (int32_t)v; break;
    case 14: o->w238_ = add32(o->w238_, pick_step(v)); break;
    case 15: o->w248_ = add32(o->w248_, pick_step(v)); break;
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
  uint32_t h = hash_tbl(2166136261u, w->b[B_OBJ], f_V34Obj, NF_V34Obj);
  snprintf(out, cap, "|h=%08x lvl=%u", h, dsplibs_debug_level);
}

/* ------------------------------------------------------------------------------------------------ the callee environment: record, effect, poke (the impl's recorders and the model call the same functions) */
#define ENVREC(w, l, ...) do { rec_add((l), (w) == &g_mw, 'c', __VA_ARGS__); } while (0)
static void env_ed(W *w, Cur *c, Log *l, char kind, const char *s) { rec_add(l, w == &g_mw, kind, "%s", s); poke(w, c); }

/* a loop driver: after SETTLE calls the loops are forced to end; `who` selects the default effect, derived from the script's second word */
enum { WHO_MOD, WHO_HS, WHO_REC };
static void drive(W *w, Cur *c, int who) {
  V34Obj *o = OBJ(w);
  uint32_t v = g_sc.val2[c->pk % NSC];
  if (c->ncall > SETTLE) { o->txIdx_ = o->blkLen_; o->rxW0_ = 0; return; }
  switch (who) {
    case WHO_MOD: o->txIdx_ = (uint16_t)(o->txIdx_ + ((v & 3u) == 0 ? 0u : (v & 3u) == 3u ? 50u : (v & 3u))); break;
    case WHO_HS:
      o->txIdx_ = (uint16_t)(o->txIdx_ + (v & 3u));
      if (((v >> 2) & 3u) != 0) o->rxW0_ = (uint16_t)(o->rxW0_ - 1u - ((v >> 4) & 7u));
      break;
    default:
      if ((v & 3u) != 0) o->rxW0_ = (uint16_t)(o->rxW0_ - ((v >> 2) & 7u));
      if (((v >> 5) & 3u) != 0) o->w47e_ = pick_cnt(v >> 8);
      o->w238_ = add32(o->w238_, pick_step(v >> 16));
      if (((v >> 7) & 7u) == 0) o->w248_ = add32(o->w248_, pick_step(v >> 20));
      break;
  }
}
static void env_drv(W *w, Cur *c, Log *l, const char *name, int who) {
  c->ncall++;
  ENVREC(w, l, "%s(obj=obj)", name);
  drive(w, c, who);
  poke(w, c);
}
static int env_hs(W *w, Cur *c, Log *l, unsigned mode) {
  uint32_t r = sres(c);
  if (getenv("V05_DBG")) { V34Obj *o = OBJ(w); fprintf(stderr, "%s hinit%u: st=%d 238=%x 248=%x rxW0=%d fl=%x 388=%x 47e=%x b6=%x b8=%x ba=%x bc=%x be=%x c0=%x c2=%x c4=%x 96=%x 98=%x 2218=%x tx=%x blk=%x\n", w == &g_mw ? "M" : "I", mode, o->state_, o->w238_, o->w248_, o->rxW0_, o->rxFlags_, o->w388_, o->w47e_, o->w4b6_, o->w4b8_, o->w4ba_, o->w4bc_, o->w4be_, o->w4c0_, o->w4c2_, o->w4c4_, o->waa96_, o->waa98_, o->w2218_, o->txIdx_, o->blkLen_); }
  ENVREC(w, l, "v34handshakinit(obj=obj %u)", mode);
  poke(w, c);
  return (int)r;
}
static void env_rrn(W *w, Cur *c, Log *l, const char *name) { ENVREC(w, l, "%s(obj=obj)", name); poke(w, c); }

/* ------------------------------------------------------------------------------------------------ the recorders (the blob's callees, weakened in the copy of dsplibs.o) */
extern void rc_hand(void *) L("v34handshak");
extern void rc_mod(void *) L("modulatevector");
extern void rc_rec(void *) L("receiver");
extern int rc_hinit(void *, unsigned) L("v34handshakinit");
extern void rc_lrrn(void *) L("VPcmV34IndicateLocalRRN");
extern void rc_rrrn(void *) L("VPcmV34IndicateRemoteRRN");
void rc_hand(void *o) { (void)o; env_drv(IW, IC, IL, "v34handshak", WHO_HS); }
void rc_mod(void *o) { (void)o; env_drv(IW, IC, IL, "modulatevector", WHO_MOD); }
void rc_rec(void *o) { (void)o; env_drv(IW, IC, IL, "receiver", WHO_REC); }
int rc_hinit(void *o, unsigned m) { (void)o; return env_hs(IW, IC, IL, m); }
void rc_lrrn(void *o) { (void)o; env_rrn(IW, IC, IL, "VPcmV34IndicateLocalRRN"); }
void rc_rrrn(void *o) { (void)o; env_rrn(IW, IC, IL, "VPcmV34IndicateRemoteRRN"); }
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  env_ed(IW, IC, IL, 'd', buf);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ the independent model: the control-flow graph of the disassembly as gotos */
static void mpf(const char *s) { env_ed(MW, MC, ML, 'd', s); }
static int sx(unsigned v) { return (int)(int16_t)(uint16_t)v; }       /* movswl / cwtl */

static void m_datapump(void) {
  V34Obj *o = (V34Obj *)MW->b[B_OBJ];
  uint32_t eax, edx, ecx;
  int di;
  unsigned fl;
  eax = (uint32_t)o->w238_; edx = (uint32_t)o->w248_; eax -= edx;
  if (eax > 0x46500u) { CV(G_RUN, "dp: clock jump: state 5"); o->state_ = 5; } else CV(G_RUN, "dp: no clock jump");
  if ((uint32_t)o->w2218_ <= 1u) goto plain;
  CV(G_HS, "dp: w2218 > 1: handshake loop");
  if (sx(o->txIdx_) < sx(o->blkLen_)) { CV(G_HS, "dp: hs loop entered by the transmit index"); goto hs_loop; }
  if (sx(o->rxW0_) <= 5) { CV(G_HS, "dp: hs loop skipped"); return; }
  CV(G_HS, "dp: hs loop entered by the receive gate");
hs_loop:
  env_drv(MW, MC, ML, "v34handshak", WHO_HS);
  if (sx(o->txIdx_) < sx(o->blkLen_)) goto hs_loop;
  if (sx(o->rxW0_) > 5) goto hs_loop;
  return;

plain:
  if (sx(o->txIdx_) >= sx(o->blkLen_)) goto rx_head;
tx_loop:
  env_drv(MW, MC, ML, "modulatevector", WHO_MOD);
  if (sx(o->txIdx_) < sx(o->blkLen_)) goto tx_loop;
rx_head:
  if (sx(o->rxW0_) <= 5) { CV(G_RUN, "dp: no receive"); goto decide; }
rx_body:
  if (sx(o->w388_) <= 0x752f) { CV(G_RUN, "dp: idle counter bumped"); o->w388_ = (uint16_t)(o->w388_ + 1u); } else CV(G_RUN, "dp: idle counter saturated");
  env_drv(MW, MC, ML, "receiver", WHO_REC);
  di = sx(o->w47e_);
  eax = (di > sx(o->w4b6_)) ? (uint32_t)o->w4bc_ + 1u : 0u;
  if (di > sx(o->w4b6_)) CV(G_RUN, "dp: w4bc kept"); else CV(G_RUN, "dp: w4bc restarted");
  o->w4bc_ = (uint16_t)eax;
  edx = (uint32_t)o->w238_; ecx = (uint32_t)o->w248_;
  eax = edx - ecx;
  if (eax > 0x23280u) {
    CV(G_RUN, "dp: clock above 0x23280");
    eax = (di > sx(o->w4b8_)) ? (uint32_t)o->w4be_ + 1u : 0u;
    if (di > sx(o->w4b8_)) CV(G_RUN, "dp: w4be kept"); else CV(G_RUN, "dp: w4be restarted");
    o->w4be_ = (uint16_t)eax;
  }
  edx -= ecx;
  if (edx > 0x119400u) {
    CV(G_RUN, "dp: clock above 0x119400");
    if (di >= sx(o->w4ba_)) { CV(G_RUN, "dp: w4c0 cleared"); o->w4c0_ = 0; } else { CV(G_RUN, "dp: w4c0 bumped"); o->w4c0_ = (uint16_t)(o->w4c0_ + 1u); }
  }
  if (sx(o->rxW0_) > 5) goto rx_body;

decide:
  fl = o->rxFlags_;
  if (fl & 0x40u) { CV(G_RUN, "dp: retrain flag 0x40"); goto retrain; }
  ecx = (uint32_t)(sx(o->waa96_) >> 1);
  if (sx(o->w4bc_) > (int)(int16_t)ecx) { CV(G_RUN, "dp: w4bc above half the gain"); goto retrain; }
  goto flag20;
retrain: {
    int half;
    env_hs(MW, MC, ML, 1);
    half = sx(o->waa96_) >> 1;
    o->w2218_ = (int32_t)((sx(o->w4bc_) <= (int)(int16_t)half ? 1u : 0u) + 2u);
    if (sx(o->w4bc_) <= (int)(int16_t)half) CV(G_RUN, "dp: retrain: w2218 3"); else CV(G_RUN, "dp: retrain: w2218 2");
    o->w4bc_ = 0; o->w4be_ = 0; o->w4c0_ = 0;
    if (DBG()) { CV(G_RUN, "dp: retrain print"); mpf("V34TIMING,Retrain started\n"); }
    fl = o->rxFlags_;
    if (!(fl & 0x20u)) goto rrn_big;
  }
  goto remote;
flag20:
  if (!(fl & 0x20u)) goto rrn_big;
remote:
  CV(G_RUN, "dp: remote RRN");
  env_hs(MW, MC, ML, 3);
  o->w4c0_ = 0; o->w2218_ = 4; o->w4bc_ = 0; o->w4be_ = 0; o->w4c2_ = 1; o->w4c4_ = o->waa98_;
  if (DBG()) { CV(G_RUN, "dp: remote RRN print"); mpf("V34RNEG, rate renegotiation detected \n"); }
  env_rrn(MW, MC, ML, "VPcmV34IndicateRemoteRRN");
  ecx = o->waa96_;
  goto big_test;
rrn_big:
  ecx = o->waa96_;
big_test:
  if (sx(o->w4be_) <= 2 * sx(ecx)) { CV(G_RUN, "dp: no large error"); goto small_test; }
  CV(G_RUN, "dp: large error: RRN down");
  env_hs(MW, MC, ML, 2);
  o->w2218_ = 5; o->w4bc_ = 0; o->w4be_ = 0; o->w4c0_ = 0; o->w4c2_ = 2; o->w4c4_ = o->waa98_;
  if (DBG()) { CV(G_RUN, "dp: RRN down print"); mpf("V34RNEG, rate renegotiation DOWN initiated due to large error \n"); }
  env_rrn(MW, MC, ML, "VPcmV34IndicateLocalRRN");
  ecx = o->waa96_;
small_test:
  if (sx(o->w4c0_) <= (sx(ecx) << 3)) { CV(G_RUN, "dp: no small error"); return; }
  CV(G_RUN, "dp: small error: RRN up");
  env_hs(MW, MC, ML, 2);
  o->w4c0_ = 0; o->w2218_ = 5; o->w4bc_ = 0; o->w4be_ = 0; o->w4c2_ = 3; o->w4c4_ = o->waa98_;
  if (DBG()) { CV(G_RUN, "dp: RRN up print"); mpf("V34RNEG, rate renegotiation UP initiated due to small error \n"); }
  env_rrn(MW, MC, ML, "VPcmV34IndicateLocalRRN");
}

/* ------------------------------------------------------------------------------------------------ world setup */
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
static void set_world(V34Obj *o, int sc) {
  o->w248_ = (int32_t)h_rnd();
  o->w238_ = add32(o->w248_, pick_diff(h_rnd()));
  if (h_rpct(25)) { o->w238_ = (int32_t)h_rnd(); }
  o->state_ = (int32_t)(h_rpct(50) ? h_rmod(8) : h_rnd());
  o->rxW0_ = h_rpct(40) ? (int16_t)(6 + h_rmod(4)) : pick_s(h_rnd());
  o->rxFlags_ = (uint16_t)(h_rpct(35) ? (h_rnd() & 0x60u) | (h_rpct(50) ? 0 : h_rnd() & 0xff9fu) : h_rnd());
  o->w388_ = h_rpct(70) ? pick_cnt(h_rnd()) : (uint16_t)(0x752c + h_rmod(8));
  o->w47e_ = pick_cnt(h_rnd());
  o->w4b6_ = pick_cnt(h_rnd()); o->w4b8_ = pick_cnt(h_rnd()); o->w4ba_ = pick_cnt(h_rnd());
  if (h_rpct(40)) { o->w4b6_ = (uint16_t)(o->w47e_ + h_rmod(3) - 1u); o->w4b8_ = (uint16_t)(o->w47e_ + h_rmod(3) - 1u); o->w4ba_ = (uint16_t)(o->w47e_ + h_rmod(3) - 1u); }
  o->w4bc_ = pick_cnt(h_rnd()); o->w4be_ = pick_cnt(h_rnd()); o->w4c0_ = pick_cnt(h_rnd());
  o->w4c2_ = (uint16_t)h_rnd(); o->w4c4_ = (uint16_t)h_rnd();
  o->waa96_ = pick_gain(h_rnd()); o->waa98_ = (uint16_t)h_rnd();
  if (h_rpct(40)) { o->w4bc_ = (uint16_t)((o->waa96_ >> 1) + h_rmod(3) - 1u); }
  if (h_rpct(35)) { o->w4be_ = (uint16_t)(2u * o->waa96_ + h_rmod(3) - 1u); }
  if (h_rpct(35)) { o->w4c0_ = (uint16_t)(8u * o->waa96_ + h_rmod(3) - 1u); }
  o->blkLen_ = (uint16_t)pick_s(h_rnd());
  o->txIdx_ = h_rpct(60) ? (uint16_t)(o->blkLen_ - h_rmod(4) + 1u) : (uint16_t)pick_s(h_rnd());
  if (sc == G_RUN) o->w2218_ = (int32_t)h_rmod(2);
  else if (sc == G_HS) o->w2218_ = h_rpct(60) ? (int32_t)(2 + h_rmod(5)) : h_rpct(50) ? (int32_t)0x80000000u : (int32_t)(h_rnd() | 2u);
  else o->w2218_ = h_rpct(50) ? (int32_t)h_rmod(2) : h_rpct(40) ? (int32_t)(2 + h_rmod(5)) : (int32_t)h_rnd();
}
static void new_world(int sc) {
  uint8_t *raw;
  size_t osz = sizeof(V34Obj);
  memset(&g_iw, 0, sizeof g_iw);
  blk_reset();
  g_nobj = NID;
  g_obj[0] = NULL; g_idname[0] = bname[0];
  raw = (uint8_t *)h_obj_new(osz + CAN);
  memset(raw + osz, 0x5a, CAN);
  g_can = raw + osz;
  fill_tbl(raw, f_V34Obj, NF_V34Obj);
  g_iw.b[B_OBJ] = raw;
  g_blkidx[B_OBJ] = blk_add(bname[B_OBJ], raw, osz);
  g_obj[B_OBJ] = raw;
  set_world((V34Obj *)raw, sc);
  blk_sync_model();
}
static void world_model(void) { g_mw.b[B_OBJ] = g_blk[g_blkidx[B_OBJ]].m; }

/* ------------------------------------------------------------------------------------------------ one call */
static void dump_world(int show) {
  char snap[TXTMAX * 2];
  obj_snapshot(snap, sizeof snap, 0);
  h_emit(show, "world %s", snap);
  dump_sliced(show, "D", g_iw.b[B_OBJ], f_V34Obj, NF_V34Obj);
}
static void do_call(void) {
  unsigned i, lv0;
  unsigned long n0;
  blk_sync_model();
  world_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  g_what = "datapumpv34";
  g_snapfn = obj_snapshot;
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  lv0 = dsplibs_debug_level;
  h_emit(g_show, "call %s lvl=%u", g_what, dsplibs_debug_level);
  n0 = g_ncalls;
  f_dp(g_iw.b[B_OBJ]);
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  dsplibs_debug_level = lv0;
  m_datapump();
  log_cmp(&g_ml, &g_il, g_what);
  if (memcmp(&g_mc, &g_ic, sizeof g_mc)) h_die3("%s: the script cursors differ", g_what);
  blk_compare(g_what);
  for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", g_what, i);
  dump_world(g_show);
  cov_flush();
  g_ncalls = n0 + 1;
}

/* ------------------------------------------------------------------------------------------------ the trials */
static unsigned pick_lv(void) { return h_rpct(65) ? 2u + h_rmod(2) : pick_level(); }
static void trial(int sc) {
  int k, nm;
  new_world(sc);
  gen_script();
  dsplibs_debug_level = pick_lv();
  g_pokemask = sc == G_POKE || sc == G_SEQ ? 0xfffffffeu : 0;
  if (sc != G_SEQ) do_call();
  else {
    nm = (int)rnd_range(2, 5);
    for (k = 0; k < nm; k++) {
      if (k) {
        gen_script();
        if (h_rpct(15)) dsplibs_debug_level = pick_lv();
        if (h_rpct(60)) set_world((V34Obj *)g_iw.b[B_OBJ], h_rpct(50) ? G_RUN : G_POKE);   /* a fresh pump state, same object */
      }
      do_call();
    }
  }
  h_trial_end("v05");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("V05_SEED");
  CovSite *cv;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("V05_ALL")) g_showall = 1;
  if (sc < 0 || sc > 3) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  v01_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("V05_TRIALS") ? atoi(getenv("V05_TRIALS")) : (sc == G_RUN || sc == G_HS ? 12000 : 6000);
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (cv = __start_covsites; cv < __stop_covsites; cv++) {
    if (cv->grp != sc) continue;
    h_emit(1, "COV %s %lu", cv->name, cv->n);
    if (cv->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", cv->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("V05_NOCOV")) ? 2 : 0;
}
