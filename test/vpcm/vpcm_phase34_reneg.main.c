/*
 * Differential test harness for port chunk V02 (src/vpcm/vpcm_phase34_reneg.cpp): v90Phase34, k56FlexPhase34, v90RateReneg and v90RateRenegSilence, the phase 3 / 4 transmit state machines of the V.34
 * module (plain C symbols; the port is extern "C" C++ because the functions call methods of VPcmFloModem / K56FlexFloModem).  Derived from test/vpcm/vpcm_info1a_retrain.main.c (recorders + independent model + per-record
 * world snapshot + pokes); shares test/v34/v01_common.h and the V.34 object mirror of test/v34/v01.layout (V34Obj, 0xac4c B on i386; V02 named more of its members).
 *
 * ONE C source, no #ifdef around the code under test (the blob build only adds the run-time redirect of the blob's LOCAL getMPrecvdBits, see patch_gmp), built by test/vpcm/vpcm_phase34_reneg.sh against the blob
 * (dsplibs.o with the callees WEAK), against the port (-O0 / -O2) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/vpcm_phase34_reneg.blob32.out).
 *
 * Every callee is a RECORDER: txmit, txmitdibit, txmitquadbit, V34scrambler, initdigital, VPcmFloModem::getV90CpBits / getV90JaBits, K56FlexFloModem::getK56FlexJaBits / getK56FlexMpBits, getMPrecvdBits
 * (LOCAL regparm(1) in the blob: its first bytes are overwritten at run time by `mov ecx,<recorder> ; jmp ecx`), dsplibs_debug_printf.  The record carries the arguments as tokens and a SNAPSHOT of the whole world
 * (hashed); a recorder then applies its scripted RESULT (the bit getters also write the bits through their out pointer; results of 16-bit-valued callees carry garbage in the upper half: the blob does cwde /
 * movsx) and may POKE the world by script (debug level, the three pointer slots, the state words, the signature / flag words, the counters, the parameter bytes): the instant at which the blob reloads a member
 * is observable.  The INDEPENDENT MODEL (m_*: a state-table interpreter written from porting/decompiled/dp/v02_v34_phase34_reneg.txt, a different shape than the port) runs AFTER the implementation on copies of the world,
 * through the same environment functions, and must reproduce every record, the returned value and every block (exit 3 otherwise).
 *
 * Scenarios (own process each): 0 v90Phase34, 1 k56FlexPhase34, 2 v90RateReneg + v90RateRenegSilence (all no pokes), 3 all four with every poke kind, 4 sequences of calls on one object (the state one call
 * leaves is the next one's input; includes whole walks through each state machine) with level pokes only.
 */

#include <sys/mman.h>

#include "../v34/v01_common.h"
#include "../v34/v01_layout.h"

/* ---- the functions under test ---- */
extern int f_ph34(void *) L("v90Phase34");
extern int f_k56(void *) L("k56FlexPhase34");
extern int f_rr(void *) L("v90RateReneg");
extern int f_rrs(void *) L("v90RateRenegSilence");
extern const int vect4[4];
extern const int vect16[16];

void *h_sysdep_malloc(unsigned size);
void h_sysdep_free(void *q);
void *sysdep_malloc(unsigned size) { return h_sysdep_malloc(size); }
void sysdep_free(void *q) { h_sysdep_free(q); }

/* ------------------------------------------------------------------------------------------------ the world */
enum { B_OBJ, B_P, B_P2, B_K, B_K2, B_C, B_C2, NBLK };
enum { NID = NBLK };
static const char *const bname[NID] = { "obj", "P", "P2", "k56", "k56b", "ctx", "ctx2" };
typedef struct { uint8_t *b[NBLK]; } W;
static W g_iw, g_mw;
static int g_blkidx[NBLK];
static uint8_t *g_can;
#define OBJ(w) ((V34Obj *)(w)->b[B_OBJ])
#define DBG() (dsplibs_debug_level > 1u)
enum { CTXSZ = 64 };

enum { G_PH = 0, G_K56 = 1, G_RR = 2, G_POKE = 3, G_SEQ = 4 };
enum { OP_PH, OP_K56, OP_RR, OP_RRS, NOPS };
static const char *const opname[NOPS] = { "v90Phase34", "k56FlexPhase34", "v90RateReneg", "v90RateRenegSilence" };

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
    g_sc.kind[i] = h_rpct(70) ? 0u : 1u + h_rmod(9);
    g_sc.val[i] = h_rnd();
    g_sc.res[i] = h_rnd();
  }
}
static uint16_t pick_h(uint32_t v) { switch (v & 7u) { case 0: return 0; case 1: return 1; case 2: return 0xffffu; case 3: return 0x7f; case 4: return 0x80; default: return (uint16_t)(v >> 3); } }
static uint16_t pick_sig(uint32_t v) { switch (v & 3u) { case 0: return 0x89b0; case 1: return 0x8990; default: return (uint16_t)(v >> 2); } }
/* the pokes the recorders apply AFTER recording (the same function runs in the impl's and the model's world) */
static void poke(W *w, Cur *c) {
  unsigned k = c->pk++ % NSC, kind = g_sc.kind[k], v = g_sc.val[k];
  V34Obj *o = OBJ(w);
  if (!kind || !((g_pokemask >> kind) & 1u)) return;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: o->ctx_ = (v & 8) ? g_iw.b[B_C2] : g_iw.b[B_C]; break;
    case 3: o->k56_ = (v & 8) ? g_iw.b[B_K2] : g_iw.b[B_K]; break;
    case 4: o->params_ = (v & 8) ? g_iw.b[B_P2] : g_iw.b[B_P]; break;
    case 5: if (v & 8) o->k56Mode_ = (int32_t)(v >> 6 & 7u); else o->pcmMode_ = (int32_t)(v >> 6 & 0x1fu) - 1; break;
    case 6: if (v & 8) o->rxSig_ = pick_sig(v >> 4); else o->rxFlags_ = (uint16_t)((v >> 4 & 1u ? 0x400u : 0u) | (v >> 5 & 1u ? 0x10u : 0u) | (v >> 8 & 0xff)); break;
    case 7: switch ((v >> 3) % 6u) { case 0: o->txCnt_ = pick_h(v >> 6); break; case 1: o->txFlags_ = pick_h(v >> 6); break; case 2: o->txBits_ = (uint16_t)(v >> 6 & 3u); break;   /* 0..3: the reloaded value indexes vect16 unmasked (out of range = a read outside both tables) */
        case 3: o->k56Idx_ = pick_h(v >> 6); break; case 4: o->k56Word_ = pick_h(v >> 6); break; default: o->period_ = pick_h(v >> 6); } break;
    case 8: { V34ModParObj *p = (V34ModParObj *)P(w, g_iw.b[(v & 8) ? B_P2 : B_P]); if (v & 16) p->flags3_ = (uint8_t)(v >> 8); else p->flags50_ = (uint8_t)(v >> 8); break; }
    case 9: switch ((v >> 3) % 3u) { case 0: o->w3596_ = (v & 64) ? 0x45 : pick_h(v >> 8); break; case 1: o->b_abfe_ = (uint8_t)(v >> 8); break; default: o->txPoint_ = v >> 6; } break;
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
    f = hash_tbl(f, w->b[B_P + k], f_V34ModParObj, NF_V34ModParObj);
    f = hash_tbl(f, w->b[B_K + k], f_K56HandleObj, NF_K56HandleObj);
    f = fnv_upd(f, w->b[B_C + k], CTXSZ);
  }
  snprintf(out, cap, "|h=%08x f=%08x lvl=%u", h, f, dsplibs_debug_level);
}

/* ------------------------------------------------------------------------------------------------ the callee environment: record, effect, poke (the impl's recorders and the model call the same functions) */
static const char *nm02(const void *p) {
  static char b[40][32];
  static int n;
  char *o = b[n++ % 40];
  int id = tok_of(p);
  if (id >= 0 && id < g_nobj) return g_idname[id];
  if (id == ID_NULL) return "null";
  if (id == ID_WILD) return "wild";
  snprintf(o, 32, "A%d+%d", (id - 100) / 4096, (id - 100) % 4096);
  return o;
}
#define T_(p) nm02(p)
#define ENVREC(w, l, ...) do { rec_add((l), (w) == &g_mw, 'c', __VA_ARGS__); } while (0)
/* a pointer into the object, as an offset of the object (the impl and the model world differ by their block addresses) */
/* a pointer into the object as MEMBER + delta (V04: the x86-64 offsets of the members behind a pointer slot differ from the i386 ones, the member names do not) */
static const char *objoff(W *w, const void *p) {
  static char b[8][64];
  static int n;
  char *o = b[n++ & 7];
  size_t d = (size_t)((const uint8_t *)p - w->b[B_OBJ]);
  int i, best = 0;
  for (i = 0; i < NF_V34Obj; i++) if (f_V34Obj[i].off <= d && f_V34Obj[i].off >= f_V34Obj[best].off) best = i;
  snprintf(o, 64, "%s+%zu", f_V34Obj[best].name, d - f_V34Obj[best].off);
  return o;
}

static void env_dbg(W *w, Cur *c, Log *l, const char *s) { rec_add(l, w == &g_mw, 'd', "%s", s); poke(w, c); }
static void env_txmit(W *w, Cur *c, Log *l, void *obj) { ENVREC(w, l, "txmit(obj=%s)", T_(obj)); poke(w, c); }
static void env_dibit(W *w, Cur *c, Log *l, void *obj, int bits) { ENVREC(w, l, "txmitdibit(obj=%s bits=%d)", T_(obj), bits); poke(w, c); }
static void env_quad(W *w, Cur *c, Log *l, void *obj, int bits) { ENVREC(w, l, "txmitquadbit(obj=%s bits=%d)", T_(obj), bits); poke(w, c); }
static void env_initd(W *w, Cur *c, Log *l, void *obj) { ENVREC(w, l, "initdigital(obj=%s)", T_(obj)); poke(w, c); }
static void env_gmp(W *w, Cur *c, Log *l, void *obj) { ENVREC(w, l, "getMPrecvdBits(obj=%s)", T_(obj)); poke(w, c); }
/* V34scrambler(state, mode, in, nbits): the blob passes constants (0 / 1, 3, 2); the result is a 16-bit value read with cwde / mov %ax: the real function returns 0..3 for 2 bits (the table lookup is unmasked),
 * garbage above bit 15 (the upper half is not used by the callers) */
static unsigned env_scr(W *w, Cur *c, Log *l, unsigned *state, int mode, unsigned in, int n) {
  uint32_t r = sres(c);
  ENVREC(w, l, "V34scrambler(state=obj.%s mode=%d in=%u n=%d st=%08x)", objoff(w, state), (int)(int16_t)mode, in & 0xffffu, (int)(int16_t)n, *state);
  *state = (*state * 1664525u + 1013904223u) ^ r;   /* the real function advances its state */
  poke(w, c);
  return (r >> 16 << 16) | ((r >> 2) & 3u);
}
/* the bit getters: write the bits through `out` (sometimes), result 0 / garbage-only-upper (the blob sign-extends the low 16 bits) / a nonzero 16-bit value / one with garbage */
static int env_bits(W *w, Cur *c, Log *l, const char *nm, void *self, short *out) {
  uint32_t r = sres(c), v = sres(c);
  int res;
  ENVREC(w, l, "%s(this=%s out=obj.%s)", nm, T_(self), objoff(w, out));
  if (v & 1u) *(uint16_t *)out = (uint16_t)(v >> 4);
  switch (r & 3u) { case 0: res = 0; break; case 1: res = (int)(r & 0xffff0000u); break; case 2: res = (int)(r | 1u) & 0xffffu; break; default: res = (int)(r | 0x100u); }
  poke(w, c);
  return res;
}

/* ------------------------------------------------------------------------------------------------ the recorders (the blob's callees, weakened in the copy of dsplibs.o) */
#ifdef __i386__
#define RP1 __attribute__((regparm(1)))
#else
#define RP1
#endif
extern void rc_txmit(void *) L("txmit");
extern void rc_dibit(void *, int) L("txmitdibit");
extern void rc_quad(void *, int) L("txmitquadbit");
extern unsigned rc_scr(unsigned *, int, unsigned, int) L("V34scrambler");
extern void rc_initd(void *) L("initdigital");
extern int rc_cp(void *, short *) L("_ZN12VPcmFloModem12getV90CpBitsEPs");
extern int rc_ja(void *, short *) L("_ZN12VPcmFloModem12getV90JaBitsEPs");
extern int rc_kja(void *, short *) L("_ZN15K56FlexFloModem16getK56FlexJaBitsEPs");
extern int rc_kmp(void *, short *) L("_ZN15K56FlexFloModem16getK56FlexMpBitsEPs");
extern void rc_gmp(void *) L("_Z14getMPrecvdBitsP12tagV34Object") RP1;
void rc_txmit(void *o) { env_txmit(IW, IC, IL, o); }
void rc_dibit(void *o, int b) { env_dibit(IW, IC, IL, o, b); }
void rc_quad(void *o, int b) { env_quad(IW, IC, IL, o, b); }
unsigned rc_scr(unsigned *s, int m, unsigned i, int n) { return env_scr(IW, IC, IL, s, m, i, n); }
void rc_initd(void *o) { env_initd(IW, IC, IL, o); }
int rc_cp(void *t, short *o) { return env_bits(IW, IC, IL, "getV90CpBits", t, o); }
int rc_ja(void *t, short *o) { return env_bits(IW, IC, IL, "getV90JaBits", t, o); }
int rc_kja(void *t, short *o) { return env_bits(IW, IC, IL, "getK56FlexJaBits", t, o); }
int rc_kmp(void *t, short *o) { return env_bits(IW, IC, IL, "getK56FlexMpBits", t, o); }
RP1 void rc_gmp(void *o) { env_gmp(IW, IC, IL, o); }
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  env_dbg(IW, IC, IL, buf);
  return 0;
}

#ifdef V02_BLOB
/* the blob's getMPrecvdBits is a LOCAL regparm(1) function that its callers reach by a PC-relative call without a relocation: it cannot be interposed by a symbol.  Overwrite its entry with
 * `mov ecx, rc_gmp ; jmp ecx` (eax = obj goes straight through).  The address is derived from the GLOBAL v90Phase34 (ELF .text 0x9be0 vs 0x9250); the prologue is verified first. */
static void patch_gmp(void) {
  static const unsigned char sig[16] = { 0x55, 0x8d, 0x50, 0x04, 0x57, 0x8d, 0xb8, 0x0c, 0xaa, 0x00, 0x00, 0x56, 0x53, 0x83, 0xec, 0x1c };
  unsigned char *p = (unsigned char *)((uintptr_t)&f_ph34 - 0x9be0u + 0x9250u);
  uintptr_t pg = (uintptr_t)p & ~(uintptr_t)4095;
  uint32_t a = (uint32_t)(uintptr_t)&rc_gmp;
  if (memcmp(p, sig, sizeof sig)) { fprintf(stderr, "getMPrecvdBits address arithmetic failed (prologue mismatch)\n"); exit(3); }
  if (mprotect((void *)pg, 8192, PROT_READ | PROT_WRITE | PROT_EXEC)) { perror("mprotect"); exit(3); }
  p[0] = 0xb9; memcpy(p + 1, &a, 4); p[5] = 0xff; p[6] = 0xe1;
}
#else
static void patch_gmp(void) {}
#endif

/* ------------------------------------------------------------------------------------------------ the independent model (from porting/decompiled/dp/v02_v34_phase34_reneg.txt) */
static void mpf(const char *fmt, ...) {
  char b[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  env_dbg(MW, MC, ML, b);
}
static V34Obj *mo(void) { return OBJ(MW); }
static int sx(unsigned v) { return (int)(int16_t)(uint16_t)v; }
static void *ipt(int blk) { return g_iw.b[blk]; }          /* the impl address of a block: what the object's pointer slots hold and what the record tokens name */
static void *oobj(void) { return g_iw.b[B_OBJ]; }
/* the packed I/Q constellation words: (hi16 << 16) | lo16 */
static uint32_t pq(int lo, int hi) { return ((uint32_t)(uint16_t)hi << 16) | (uint16_t)lo; }
static uint32_t mv4(int k) { static const int lo[4] = { 4579, 4579, -4579, -4579 }, hi[4] = { 4579, -4579, -4579, 4579 }; return pq(lo[k], hi[k]); }
/* coverage: the model's branches are shared between the families, the sites are not: one site object per (family, name) */
#define CVP(g, nm) do { if ((g) == G_PH) CV(G_PH, nm); else CV(G_RR, nm); } while (0)
#define CVG(g, nm) do { switch (g) { case G_PH: CV(G_PH, nm); break; case G_K56: CV(G_K56, nm); break; default: CV(G_RR, nm); break; } } while (0)

/* the state table: per family, the role of every state and its successor */
enum { R_PA, R_PB, R_FIX, R_CP, R_CPDONE, R_SCR };
typedef struct { int st, role, next, flag; } Row;
enum { F_PRINT = 1, F_SIL = 2, F_FLAG4 = 4, F_DBLSCR = 8 };
static const Row rows_ph[] = { { 3, R_PA, 4, F_PRINT }, { 4, R_PB, 5, 0 }, { 5, R_SCR, 0, 0 }, { 6, R_PA, 7, 0 }, { 7, R_PB, 8, 0 }, { 8, R_CP, 9, 0 }, { 9, R_FIX, 0, 0 }, { 10, R_CPDONE, 2, 0 } };
static const Row rows_rr[] = { { 0xb, R_PA, 0xc, 0 }, { 0xc, R_PB, 0xd, 0 }, { 0xd, R_FIX, 0, 0 }, { 0xe, R_CPDONE, 2, 0 } };
static const Row rows_rrs[] = { { 0xf, R_PA, 0x10, 0 }, { 0x10, R_PB, 0x11, 0 }, { 0x11, R_FIX, 0, 0 }, { 0x12, R_CP, 0x13, F_SIL }, { 0x13, R_SCR, 0, 0 }, { 0x14, R_CPDONE, 2, F_FLAG4 } };

static void m_point(uint32_t pt) { mo()->txPoint_ = pt; }
static unsigned m_bump(void) { V34Obj *o = mo(); o->txCnt_ = (uint16_t)(o->txCnt_ + 1u); return o->txCnt_; }
/* the bits are in txBits_ (or the fixed pattern); a quad-bit symbol on the 16-point signature, else a di-bit */
static void m_send(int fixed, int g) {
  V34Obj *o = mo();
  int wide = (o->rxSig_ == 0x89b0);
  int bits = fixed ? (wide ? 0xf : 3) : sx(o->txBits_);
  if (fixed) CVP(g, "send: fixed pattern"); else CVG(g, "send: the bits buffer");
  if (wide) { CVG(g, "send: 16-point (quad-bit)"); env_quad(MW, MC, ML, oobj(), bits); } else { CVG(g, "send: 4-point (di-bit)"); env_dibit(MW, MC, ML, oobj(), bits); }
}
static void m_done_bits(int g) {
  V34Obj *o = mo();
  env_gmp(MW, MC, ML, oobj());
  env_initd(MW, MC, ML, oobj());
  if (o->w3596_ != 0x45) { CVP(g, "bits done: w3596 changed to 0x45"); o->w3596_ = 0x45; } else CVP(g, "bits done: w3596 already 0x45");
  o->pcmMode_ = 2;
}
static void m_scr_point(int g, int mode, int wide) {
  V34Obj *o = mo();
  int r1, r2;
  if (wide) {
    CVG(g, "scrambled point: 16-point (two scrambler calls)");
    r1 = sx(env_scr(MW, MC, ML, &o->scrState_, mode, 3, 2)); o->txBits_ = (uint16_t)r1;
    r2 = sx(env_scr(MW, MC, ML, &o->scrState_, mode, 3, 2));
    m_point((uint32_t)vect16[r2 + 4 * sx(o->txBits_)]);
  } else {
    CVG(g, "scrambled point: 4-point (one scrambler call)");
    r1 = sx(env_scr(MW, MC, ML, &o->scrState_, mode, 3, 2)); o->txBits_ = (uint16_t)r1;
    m_point(mv4(r1));
  }
}
static int bits_nonzero(int r) { return sx((unsigned)r) != 0; }

static int m_run(int op) {
  V34Obj *o = mo();
  const Row *rows = op == OP_PH ? rows_ph : op == OP_RR ? rows_rr : rows_rrs;
  int nrows = op == OP_PH ? 8 : op == OP_RR ? 4 : 6, g = op == OP_PH ? G_PH : G_RR, i;
  const Row *row = 0;
  void *ctxI = o->ctx_;
  unsigned flags0 = o->rxFlags_;
  if (op == OP_PH) {
    /* the gates in front of the state table: Ja bits first, then the gate flag */
    if (!(flags0 & 0x400u)) {
      int r;
      CV(G_PH, "ph34: Ja bits (flag 0x400 clear)");
      r = env_bits(MW, MC, ML, "getV90JaBits", ctxI, (short *)&o->txBits_);
      env_dibit(MW, MC, ML, oobj(), sx(o->txBits_));
      if (bits_nonzero(r)) {
        V34ModParObj *p;
        CV(G_PH, "ph34: Ja complete");
        o->rxFlags_ = (uint16_t)(o->rxFlags_ | 0x400u);
        p = (V34ModParObj *)P(MW, o->params_);
        if (p->flags50_ & 4u) {
          CV(G_PH, "ph34: backward clear parameter set");
          if (DBG()) { CV(G_PH, "ph34: backward clear print"); mpf("VPcmV34Main: tx buffer backward clear is enabled...\r\n"); }
          o->b_abfe_ = 1;
        }
      } else CV(G_PH, "ph34: Ja not complete");
      return 0;
    }
    if (!(flags0 & 0x10u)) {
      CV(G_PH, "ph34: gate closed: silence");
      m_point(0);
      env_txmit(MW, MC, ML, oobj());
      if (o->pcmMode_ > 2) { CV(G_PH, "ph34: gate opened"); o->rxFlags_ = (uint16_t)(o->rxFlags_ | 0x10u); o->txCnt_ = 0; } else CV(G_PH, "ph34: gate stays closed");
      return 0;
    }
  }
  for (i = 0; i < nrows; i++) if (rows[i].st == o->pcmMode_) row = &rows[i];
  if (!row) { CVP(g, "no such state"); return 0; }
  switch (row->role) {
    case R_PA: {
      unsigned c;
      CVP(g, "state: tone pair A (vect4[0], vect4[3])");
      m_point(mv4(0)); env_txmit(MW, MC, ML, oobj());
      m_bump();
      m_point(mv4(3)); env_txmit(MW, MC, ML, oobj());
      c = (o->txCnt_ + 1u) & 0xffffu;
      if (sx(c) <= 0x7f) { CVP(g, "tone pair A: counter below the limit"); o->txCnt_ = (uint16_t)c; }
      else {
        CVP(g, "tone pair A: limit reached");
        if (row->flag & F_PRINT) { o->txCnt_ = (uint16_t)c; if (DBG()) { CV(G_PH, "tone pair A: print"); mpf("Entered v34m->v90Receiver == V90RCV_P3_THIRD_S with tx->symcnt = %d period = %d\n", sx(c), sx(o->period_)); } }
        o->pcmMode_ = row->next; o->txCnt_ = 0;
      }
      return 0;
    }
    case R_PB: {
      unsigned c;
      CVP(g, "state: tone pair B (vect4[2], vect4[1])");
      m_point(mv4(2)); env_txmit(MW, MC, ML, oobj());
      m_bump();
      m_point(mv4(1)); env_txmit(MW, MC, ML, oobj());
      c = (o->txCnt_ + 1u) & 0xffffu;
      if (c != 0x10) { CVP(g, "tone pair B: counter below the limit"); o->txCnt_ = (uint16_t)c; }
      else { CVP(g, "tone pair B: limit reached"); o->pcmMode_ = row->next; o->txPrev_ = 0; o->txCnt_ = 0; o->scrState_ = 0; }
      return 0;
    }
    case R_FIX:
      CVP(g, "state: fixed symbol");
      m_send(1, g);
      m_bump();
      return 0;
    case R_CP: case R_CPDONE: {
      int r = env_bits(MW, MC, ML, "getV90CpBits", ctxI, (short *)&o->txBits_);
      CVP(g, "state: Cp bits");
      m_send(0, g);
      if (bits_nonzero(r)) {
        CVP(g, "Cp bits complete");
        if (row->role == R_CPDONE) m_done_bits(g);
        else {
          if (row->flag & F_SIL) {
            V34ModParObj *p;
            if (DBG()) { CV(G_RR, "Cp bits: silence print"); mpf("move to SCR on silence rrn (disabling SAS detector on silence)\r\n"); }
            o->pcmMode_ = row->next; o->txFlags_ = (uint16_t)(o->txFlags_ & ~4u);
            p = (V34ModParObj *)P(MW, o->params_);
            p->flags3_ = (uint8_t)(p->flags3_ & ~4u);
          } else o->pcmMode_ = row->next;
        }
      } else CVP(g, "Cp bits incomplete");
      return 0;
    }
    case R_SCR: {
      uint16_t sig = o->rxSig_;
      if (op == OP_PH) {
        if (sig == 0x89b0) m_scr_point(g, 0, 1); else if (sig == 0x8990) m_scr_point(g, 0, 0); else { CV(G_PH, "ph34 state 5: other signature: zero point"); m_point(0); }
      } else m_scr_point(g, 0, sig == 0x89b0);
      env_txmit(MW, MC, ML, oobj());
      m_bump();
      if (op != OP_PH) { CV(G_RR, "silence 0x13: previous bits"); o->txPrev_ = o->txBits_; }
      return 0;
    }
  }
  return 0;
}
/* 0x14 sets txFlags_ bit 4 before the getter call: handled by a wrapper so that the order (flag, getter) is kept */
static int m_rrs(void) {
  V34Obj *o = mo();
  if (o->pcmMode_ == 0x14) { CV(G_RR, "silence 0x14: flag set before the getter"); o->txFlags_ = (uint16_t)(o->txFlags_ | 4u); }
  return m_run(OP_RRS);
}
static int m_k56(void) {
  V34Obj *o = mo();
  void *kI = o->k56_;
  int g = G_K56;
  if (!(o->rxFlags_ & 0x400u)) {
    int r;
    CV(G_K56, "k56: Ja bits (flag 0x400 clear)");
    r = env_bits(MW, MC, ML, "getK56FlexJaBits", kI, (short *)&o->txBits_);
    env_dibit(MW, MC, ML, oobj(), sx(o->txBits_));
    if (bits_nonzero(r)) { CV(G_K56, "k56: Ja complete"); o->rxFlags_ = (uint16_t)(o->rxFlags_ | 0x400u); o->k56Word_ = 0x899f; o->k56Idx_ = 0; o->k56Mode_ = 3; } else CV(G_K56, "k56: Ja incomplete");
    return 0;
  }
  switch (o->k56Mode_) {
    case 3: {
      unsigned n = (2u * (unsigned)(uint16_t)o->k56Idx_) & 31u, w = o->k56Word_, k, ni;
      CV(G_K56, "k56 state 3: dibit of the JA word");
      for (k = 0; k < n; k++) w >>= 1;
      env_dibit(MW, MC, ML, oobj(), sx(w));
      ni = ((unsigned)o->k56Idx_ + 1u) & 7u;
      o->k56Idx_ = (uint16_t)ni;
      if (ni == 0) { CV(G_K56, "k56 state 3: word complete"); o->txPrev_ = 0; o->txCnt_ = 0; o->scrState_ = 0; o->k56Mode_ = 4; } else CV(G_K56, "k56 state 3: more dibits");
      return 0;
    }
    case 4:
      CV(G_K56, "k56 state 4: scrambled point");
      if (o->rxSig_ == 0x89b0) m_scr_point(g, 1, 1); else m_scr_point(g, 1, 0);
      env_txmit(MW, MC, ML, oobj());
      m_bump();
      return 0;
    case 5: {
      int r = env_bits(MW, MC, ML, "getK56FlexMpBits", kI, (short *)&o->txBits_);
      CV(G_K56, "k56 state 5: Mp bits");
      m_send(0, g);
      if (bits_nonzero(r)) {
        CV(G_K56, "k56 Mp bits complete");
        env_gmp(MW, MC, ML, oobj());
        env_initd(MW, MC, ML, oobj());
        if (o->w3596_ != 0x45) { CV(G_K56, "k56 done: w3596 changed to 0x45"); o->w3596_ = 0x45; } else CV(G_K56, "k56 done: w3596 already 0x45");
        o->k56Mode_ = 2;
      } else CV(G_K56, "k56 Mp bits incomplete");
      return 0;
    }
    default: CV(G_K56, "k56: no such state"); return 0;
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
static uint16_t pick_cnt(void) {
  static const uint16_t v[] = { 0, 1, 0xd, 0xe, 0xf, 0x10, 0x11, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x7fff, 0x8000, 0xfffd, 0xfffe, 0xffff, 0x7ffe };
  return h_rpct(80) ? v[h_rmod(sizeof v / sizeof v[0])] : (uint16_t)h_rnd();
}
static uint16_t pick_h16(void) { return (uint16_t)(h_rpct(25) ? 0 : h_rpct(30) ? 1 : h_rpct(25) ? 0xffffu : h_rnd()); }
static int32_t pick_state(int op) {
  int lo = op == OP_PH ? 3 : op == OP_RR ? 0xb : op == OP_RRS ? 0xf : 3, hi = op == OP_PH ? 10 : op == OP_RR ? 0xe : op == OP_RRS ? 0x14 : 5;
  if (h_rpct(88)) return lo + (int32_t)h_rmod((unsigned)(hi - lo + 1));
  switch (h_rmod(7)) { case 0: return 0; case 1: return 2; case 2: return lo - 1; case 3: return hi + 1; case 4: return -1; case 5: return (int32_t)h_rnd(); default: return (int32_t)h_rmod(0x20); }
}
static uint16_t pick_flags(void) {
  uint16_t f = (uint16_t)h_rnd();
  f = (uint16_t)(h_rpct(75) ? (f | 0x400u) : (f & ~0x400u));
  f = (uint16_t)(h_rpct(75) ? (f | 0x10u) : (f & ~0x10u));
  if (h_rpct(30)) f &= 0x410u;
  return f;
}
static uint16_t pick_sig16(void) { return (uint16_t)(h_rpct(40) ? 0x89b0 : h_rpct(66) ? 0x8990 : h_rpct(50) ? (0x89b0 ^ (1u << h_rmod(16))) : h_rnd()); }
static void new_world(int op) {
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
  newblk(B_C, CTXSZ); newblk(B_C2, CTXSZ);
  for (k = B_P; k <= B_P2; k++) {
    V34ModParObj *p = (V34ModParObj *)g_iw.b[k];
    fill_tbl((uint8_t *)p, f_V34ModParObj, NF_V34ModParObj);
    p->flags3_ = (uint8_t)(h_rpct(30) ? 0xff : h_rnd());
    p->flags50_ = (uint8_t)(h_rpct(60) ? 4u | (h_rnd() & 0xfbu) : h_rnd() & 0xfbu);
  }
  for (k = B_K; k <= B_K2; k++) fill_tbl((uint8_t *)g_iw.b[k], f_K56HandleObj, NF_K56HandleObj);
  fill_blk(g_iw.b[B_C], CTXSZ); fill_blk(g_iw.b[B_C2], CTXSZ);
  o = (V34Obj *)raw;
  o->ctx_ = g_iw.b[B_C]; o->k56_ = g_iw.b[B_K]; o->params_ = g_iw.b[B_P];
  o->pcmMode_ = pick_state(op == OP_K56 ? OP_PH : op);
  o->k56Mode_ = op == OP_K56 ? (h_rpct(88) ? 3 + (int32_t)h_rmod(3) : pick_state(OP_K56)) : (int32_t)h_rmod(8);
  o->rxSig_ = pick_sig16(); o->rxFlags_ = pick_flags();
  o->txCnt_ = pick_cnt(); o->txFlags_ = (uint16_t)h_rnd(); o->txPrev_ = (uint16_t)h_rnd(); o->txBits_ = pick_h16();
  o->scrState_ = h_rnd(); o->txPoint_ = h_rnd();
  o->k56Word_ = h_rpct(40) ? 0x899f : (uint16_t)h_rnd();
  o->k56Idx_ = h_rpct(70) ? (uint16_t)h_rmod(8) : pick_h16();
  o->w3596_ = h_rpct(40) ? 0x45 : (uint16_t)h_rnd();
  o->period_ = pick_h16(); o->b_abfe_ = (uint8_t)h_rnd();
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
  h_dump_obj(show, "P2", g_iw.b[B_P2], f_V34ModParObj, NF_V34ModParObj);
}
static void do_call(int op) {
  uint8_t *t = g_iw.b[B_OBJ];
  unsigned i, lv0;
  int ri = 0, rm = 0;
  blk_sync_model();
  world_model();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  cov_clear();
  g_what = opname[op];
  g_snapfn = obj_snapshot;
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
  lv0 = dsplibs_debug_level;
  h_emit(g_show, "call %s lvl=%u", g_what, dsplibs_debug_level);
  switch (op) { case OP_PH: ri = f_ph34(t); break; case OP_K56: ri = f_k56(t); break; case OP_RR: ri = f_rr(t); break; default: ri = f_rrs(t); }
  __asm__ volatile("" ::: "memory");
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  dsplibs_debug_level = lv0;
  switch (op) { case OP_PH: rm = m_run(OP_PH); break; case OP_K56: rm = m_k56(); break; case OP_RR: rm = m_run(OP_RR); break; default: rm = m_rrs(); }
  check_common(g_what);
  if (ri != rm) h_die3("%s: the model returns %d, the impl %d", g_what, rm, ri);
  h_emit(g_show, "ret %d", ri);
  dump_world(g_show);
  cov_flush();
  g_ncalls++;
}

/* ------------------------------------------------------------------------------------------------ the trials */
static unsigned pick_lv(void) { return h_rpct(65) ? 2u + h_rmod(2) : pick_level(); }
static int pick_op(int sc) { return sc == G_PH ? OP_PH : sc == G_K56 ? OP_K56 : sc == G_RR ? (h_rpct(40) ? OP_RR : OP_RRS) : (int)h_rmod(NOPS); }
/* a whole walk through a state machine from its first state: the counters are cleared, the flags open; each call advances the state through the recorders' scripted results */
static void start_walk(int op) {
  V34Obj *o = OBJ(&g_iw);
  o->txCnt_ = 0; o->rxFlags_ = (uint16_t)(o->rxFlags_ | 0x410u);
  if (op == OP_PH) o->pcmMode_ = 3; else if (op == OP_RR) o->pcmMode_ = 0xb; else if (op == OP_RRS) o->pcmMode_ = 0xf; else o->k56Mode_ = 3;
  if (op == OP_K56) { o->k56Idx_ = 0; o->k56Word_ = 0x899f; }
  blk_sync_model();
}
static void trial(int sc) {
  int k, nm, op;
  op = pick_op(sc);
  new_world(op);
  gen_script();
  dsplibs_debug_level = pick_lv();
  g_pokemask = sc == G_POKE ? 0xfffffffeu : sc == G_SEQ ? (1u << 1) : 0;
  if (sc != G_SEQ) do_call(op);
  else if (h_rpct(35)) {
    int len = (int)rnd_range(90, 220);
    start_walk(op);
    for (k = 0; k < len; k++) {
      if (!(k & 15)) gen_script();
      do_call(op);
    }
  } else {
    nm = (int)rnd_range(2, 6);
    for (k = 0; k < nm; k++) {
      if (k) { gen_script(); if (h_rpct(15)) dsplibs_debug_level = pick_lv(); }
      do_call(pick_op(sc));
    }
  }
  h_trial_end("v02");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr, k;
  const char *seed = getenv("V02_SEED");
  CovSite *cv;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("V02_ALL")) g_showall = 1;
  if (sc < 0 || sc > 4) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  v01_layout_check();
  h_x87_check();
  for (k = 0; k < 4; k++) if ((uint32_t)vect4[k] != mv4(k)) { fprintf(stderr, "vect4[%d] != the model's packed point\n", k); return 3; }
  patch_gmp();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("V02_TRIALS") ? atoi(getenv("V02_TRIALS")) : (sc == G_SEQ ? 1000 : sc == G_POKE ? 12000 : 15000);
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (cv = __start_covsites; cv < __stop_covsites; cv++) {
    if (cv->grp != sc) continue;
    h_emit(1, "COV %s %lu", cv->name, cv->n);
    if (cv->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", cv->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("V02_NOCOV")) ? 2 : 0;
}
