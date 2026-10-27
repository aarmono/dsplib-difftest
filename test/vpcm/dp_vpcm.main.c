/*
 * Differential test harness for port chunk Z01 (src/vpcm/dp_vpcm.cpp): VPCMXF_Create / VPCMXF_Delete / VPCMXF_SessionTermination, the LOCAL vpcm_create / vpcm_delete / vpcm_run (reached through the record
 * vpcm_op that dp_vpcm_init registers with modem_dp_register) and dp_vpcm_init / dp_vpcm_exit.  Derived from test/vpcm/vpcm_create.main.c (recorders + independent model + world snapshots + pokes); ONE C source, no #ifdef
 * around the code under test, built by test/vpcm/dp_vpcm.sh against the blob (dsplibs.o with the callees WEAK), against the port (-O0 / -O2: the port object alone) and for x86-64 (diffed against the
 * 32-bit BLOB's output = test/golden/vpcm/dp_vpcm.blob32.out).
 *
 * The world: the session state (Z01_State, allocated by vpcm_create through sysdep_malloc: every block sysdep_malloc returns is REGISTERED as a world block at that moment, so the independent model sees the
 * state before the call and the impl's mutations are compared against the model's), the modem handle (opaque), the dp parameter block (V34ModParObj) and the 2-word pair modem_get_param(modem, 0xb) returns,
 * the K56Flex handle, the VPcmFloModem (PcmFlo47Obj) VPCMXF_Create builds, the input / output sample buffers and the cleaned-samples buffer.  The sizes of the state and of the VPcmFloModem differ on x86-64
 * (pointers): the state is addressed by the member offsets of the REAL class (test/vpcm/dp_vpcm_layout_probe.cpp), the VPcmFloModem through the P47 mirror, the malloc log maps both to the i386 sizes.
 * Every callee is a RECORDER (sysdep_memset / memcpy, modem_*, dp_param_get, K56FLEX_*, VPcmV34Create / Progress / GetCleanedSamples / GetCurrent*, the VPcmFloModem constructor, the six member destructors,
 * V90Demodulator::sessionTermination, dsplibs_debug_printf): it records its arguments as tokens + a hash snapshot of the world, performs its effect (a function of the script) and may then POKE the world by script
 * (debug level, dp block words, state words): the time at which the blob / port reloads a member is observable.  The INDEPENDENT MODEL (m_xfcreate / m_xfdelete / m_xfterm / m_create / m_delete / m_run: the
 * control-flow graph of the disassembly as gotos, written from porting/decompiled/dp/z01_glue.txt) runs AFTER the implementation on copies of the world, through the same environment functions, and must reproduce every
 * record, the returned value and every block (exit 3 otherwise).
 *
 * Scenarios (own process each): 0 VPCMXF_Create / SessionTermination / Delete (side, mode, rate shapes incl. the x87 truncation of the buffer size), 1 vpcm_create + vpcm_delete (every failure branch, the rate
 * limits, delays, flag bits), 2 vpcm_run sequences on a created state with pokes, 3 dp_vpcm_init / dp_vpcm_exit.
 */

#include "../v34/v01_common.h"
#include "vpcm_flo_modem_layout.h"
#include "../v34/v01_layout.h"

extern void *f_xfcreate(int, void *, void *, unsigned, int) L("VPCMXF_Create");
extern void f_xfdelete(void *) L("VPCMXF_Delete");
extern int f_xfterm(void *) L("VPCMXF_SessionTermination");
extern long f_dpget(void *) L("dp_param_get");
extern int f_init(void) L("dp_vpcm_init");
extern void f_exit(void) L("dp_vpcm_exit");

/* ------------------------------------------------------------------------------------------------ the state, by the offsets of the real class */
extern const unsigned z01_probe_Z01StateObj[];
enum { ZS_SIZE, ZS_id, ZS_modem, ZS_unk08, ZS_op, ZS_dpData, ZS_status, ZS_link, ZS_bitsWanted, ZS_trainCnt, ZS_pair, ZS_dp, ZS_v34, ZS_inF, ZS_outF, ZS_bitsIn, ZS_bitsOut, ZS_txCnt, ZS_txBuf,
       ZS_pendCnt, ZS_pendBuf, ZS_silence, ZS_addDelay, ZS_N };
static const char *const zs_name[ZS_N] = { "size", "id", "modem", "unk08", "op", "dpData", "status", "link", "bitsWanted", "trainCnt", "pair", "dp", "v34", "inF", "outF", "bitsIn", "bitsOut", "txCnt",
                                           "txBuf", "pendCnt", "pendBuf", "silence", "addDelay" };
#define ZSIZE (z01_probe_Z01StateObj[ZS_SIZE])
#define XFSIZE (sizeof(PcmFlo47Obj))
#define ZO(s, m) ((uint8_t *)(s) + z01_probe_Z01StateObj[ZS_##m])
#define ZI(s, m) (*(int32_t *)ZO(s, m))
#define ZP(s, m) (*(void **)ZO(s, m))
#define ZV34(s) ((V34Obj *)ZO(s, v34))
#define DBG() (dsplibs_debug_level > 1u)

/* slmodemd's struct dp_operations (modem_dp.h) */
typedef struct { const char *name; int use; void *create; void *del; void *process; void *hangup; } Ops;
typedef void *(*CreateFn)(void *, int, int, int, int, void *);
typedef int (*DeleteFn)(void *);
typedef int (*RunFn)(void *, short *, short *, int);

void *h_sysdep_malloc(unsigned size);
void h_sysdep_free(void *q);

/* ------------------------------------------------------------------------------------------------ the world */
enum { NSTATIC = 8 };
/* g_pair is the host's `struct dsp_info`: connection_type at +0 and `long clock_deviation` at +sizeof(long) (+4 i386, +8 x86-64); the blob writes two 32-bit
 * words at +0 / +4.  PW1() is the 32-bit word at the clock_deviation member (the low half on x86-64, where the port stores the whole sign-extended long). */
#define PAIR_W1 sizeof(long)
#define PW1(p) (*(int32_t *)((uint8_t *)(p) + PAIR_W1))
static void *g_modem, *g_dp, *g_dp2, *g_pair, *g_in, *g_out, *g_clean, *g_xctx, *g_dem;
static void *g_op;                    /* the dp_operations record the blob / port registered (impl-world address) */
static uint8_t *g_can;
enum { INSZ = 256, OUTSZ = 256, CLEANSZ = 256 };
static int g_bfree[MAXB];
static const char *g_bname[MAXB];
static void *g_stc[2];                /* the live state block (impl address) as the impl [0] / the model [1] sees it, or NULL */
static void *g_xfc[2];                /* the VPcmFloModem VPCMXF_Delete is tearing down / was created (for the destructor `this` check and the snapshot) */
static int g_mode;                    /* world-model switch of the snapshot */
enum { G_XF = 0, G_CREATE = 1, G_RUN = 2, G_REG = 3 };

typedef struct { uint32_t kind[NSC], val[NSC], res[NSC]; } Script;
typedef struct { unsigned pk, rv; } Cur;
static Script g_sc;
static Cur g_ic, g_mc;
static Log g_il, g_ml;
static int g_show, g_showall;
static unsigned long g_symctr, g_ncalls;
static unsigned g_pokemask;
static unsigned g_failpct_v34, g_failpct_k56;
static const char *g_what = "setup";
static int g_sc_id;
#define CUR(mw) ((mw) ? &g_mc : &g_ic)
#define LOG(mw) ((mw) ? &g_ml : &g_il)

static int blk_find(const void *p) {
  int i;
  for (i = 0; i < g_nblk; i++) if ((const uint8_t *)p >= (const uint8_t *)g_blk[i].p && (const uint8_t *)p < (const uint8_t *)g_blk[i].p + g_blk[i].n) return i;
  return -1;
}
/* an impl-world pointer as seen from the model: the address inside the model's copy of the block */
static void *P(int mw, const void *p) {
  int i;
  if (!mw || !p) return (void *)p;
  i = blk_find(p);
  if (i < 0) return (void *)p;   /* a stack variable (a5 / a7 / the count of the cleaned samples): the model's own */
  return (uint8_t *)g_blk[i].m + ((const uint8_t *)p - (const uint8_t *)g_blk[i].p);
}
static void blk_gc(void) {   /* drop the blocks freed by the previous call */
  int i, j = 0;
  for (i = 0; i < g_nblk; i++) {
    if (g_bfree[i]) { free(g_blk[i].m); continue; }
    g_blk[j] = g_blk[i]; g_bfree[j] = 0; g_bname[j] = g_bname[i]; j++;
  }
  g_nblk = j;
}
static void blk_sync(void) { int i; for (i = 0; i < g_nblk; i++) memcpy(g_blk[i].m, g_blk[i].p, g_blk[i].n); }
static void blk_cmp(const char *what) {
  int i;
  for (i = 0; i < g_nblk; i++) {
    const uint8_t *x = (const uint8_t *)g_blk[i].m, *y = (const uint8_t *)g_blk[i].p;
    size_t k;
    if (g_bfree[i] || !memcmp(x, y, g_blk[i].n)) continue;
    for (k = 0; k < g_blk[i].n; k++) if (x[k] != y[k]) break;
    h_die3("%s: block \"%s\" differs from the model at offset 0x%zx (model %02x, impl %02x)", what, g_bname[i], k, x[k], y[k]);
  }
}
static int blk_reg(const char *name, void *p, size_t n) { int i = blk_add(name, p, n); g_bname[i] = name; g_bfree[i] = 0; return i; }

/* a pointer as a token: the name of the world block it lies in + the offset (arch-independent: the state is addressed by member) */
static const char *pname(const void *p) {
  static char b[24][64];
  static int n;
  char *o = b[n++ % 24];
  int i;
  const uint8_t *q = (const uint8_t *)p;
  if (!p) return "null";
  if (g_op && p == g_op) return "vpcm_op";
  i = blk_find(p);
  if (i < 0) return "stk";   /* a stack variable of the function under test / of the model */
  if (!strcmp(g_bname[i], "state")) {
    unsigned off = (unsigned)(q - (const uint8_t *)g_blk[i].p), k, best = 1;
    for (k = 1; k < ZS_N; k++) if (z01_probe_Z01StateObj[k] <= off && z01_probe_Z01StateObj[k] >= z01_probe_Z01StateObj[best]) best = k;
    if (off == z01_probe_Z01StateObj[best]) snprintf(o, 64, "state.%s", zs_name[best]);
    else snprintf(o, 64, "state.%s+%u", zs_name[best], off - z01_probe_Z01StateObj[best]);
    return o;
  }
  if (q == (const uint8_t *)g_blk[i].p) snprintf(o, 64, "%s", g_bname[i]);
  else snprintf(o, 64, "%s+%u", g_bname[i], (unsigned)(q - (const uint8_t *)g_blk[i].p));
  return o;
}
#define T_(p) pname(p)

/* ------------------------------------------------------------------------------------------------ script, pokes, snapshot */
static uint32_t sres(Cur *c) { return g_sc.res[c->rv++ % NSC]; }
static uint32_t pick_w(uint32_t v) { switch (v & 7u) { case 0: return 0; case 1: return 1; case 2: return 0xffffffffu; case 3: return 0x80000000u; case 4: return 0x7fffffffu; default: return v >> 3; } }
static void gen_script(void) {
  unsigned i;
  for (i = 0; i < NSC; i++) {
    g_sc.kind[i] = h_rpct(55) ? 0u : 1u + h_rmod(4);
    g_sc.val[i] = h_rnd();
    g_sc.res[i] = h_rnd();
  }
}
static void poke(int mw, Cur *c) {
  unsigned k = c->pk++ % NSC, kind = g_sc.kind[k], v = g_sc.val[k];
  if (!kind || !((g_pokemask >> kind) & 1u)) return;
  switch (kind) {
    case 1: { static const unsigned lvs[] = { 0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 3 }; dsplibs_debug_level = lvs[(v >> 2) % 8]; break; }
    case 2: {   /* the dp block (the words the function re-reads) */
      V34ModParObj *p = (V34ModParObj *)P(mw, (v & 4) ? g_dp2 : g_dp);
      switch ((v >> 4) % 7u) {
        case 0: p->w6c_ = pick_w(v >> 8); break;
        case 1: p->delay_ = (int32_t)pick_w(v >> 8); break;
        case 2: p->extDelay_ = (int32_t)pick_w(v >> 8); break;
        case 3: p->flags2_ = (uint8_t)(v >> 8); break;
        case 4: p->connType_ = pick_w(v >> 8); break;
        case 5: if (g_stc[mw]) ZP(P(mw, g_stc[mw]), dp) = (v >> 8) & 1 ? g_dp2 : g_dp; break;   /* the state's dp_ slot: the time at which a member is re-read through it is observable */
        default: p->regWord_ = pick_w(v >> 8); break;
      }
      break;
    }
    case 3: {   /* the state */
      void *s;
      if (!g_stc[mw]) break;
      s = P(mw, g_stc[mw]);
      switch ((v >> 4) % 7u) {
        case 0: ZI(s, addDelay) = (int32_t)pick_w(v >> 8); break;
        case 1: ZI(s, silence) = (v >> 8) & 1 ? (int32_t)(v >> 12) & 0x3f : (int32_t)pick_w(v >> 8); break;
        case 2: ZI(s, bitsWanted) = (int32_t)((v >> 8) % 0x403u) - 2; break;
        case 3: ZI(s, trainCnt) = (v >> 12) & 1 ? (int32_t)0xbb6 + (int32_t)((v >> 8) & 3u) : -1 - (int32_t)((v >> 8) & 3u); break;
        case 4: ZI(s, link) = (int32_t)((v >> 8) % 4u) - 1; break;
        case 5: ZI(s, status) = (int32_t)((v >> 8) % 20u) - 1; break;
        default: ZI(s, id) = (v >> 8) & 1 ? (int32_t)pick_w(v >> 9) : (int32_t)(34 + 56 * ((v >> 9) % 3u)); break;
      }
      break;
    }
    case 4: { int32_t *pr = (int32_t *)P(mw, g_pair); if ((v >> 4) & 1) PW1(pr) = (int32_t)pick_w(v >> 8); else pr[0] = (int32_t)pick_w(v >> 8); break; }
    default: break;
  }
}
static uint32_t hash_vals(uint32_t h, const uint8_t *t, const HFld *tbl, int nf) {   /* the value members only (pointer slots are compared byte-wise at the end of the call) */
  int i;
  for (i = 0; i < nf; i++) {
    const HFld *fl = &tbl[i];
    unsigned n = fl->count ? fl->count : 1, es;
    switch (fl->kind) {
      case 'p': case 'o': case 'e': case 'v': continue;
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'd': es = 8; break;
      default: es = 4; break;
    }
    h = fnv_upd(h, t + fl->off, (size_t)n * es);
  }
  return h;
}
static uint32_t hash_state(const void *s) {
  uint32_t h = 2166136261u;
  h = fnv_upd(h, ZO(s, id), 4);
  h = fnv_upd(h, ZO(s, status), 16);   /* status, link, bitsWanted, trainCnt */
  h = fnv_upd(h, ZO(s, txCnt), 4 + 104 + 4 + 104);   /* txCnt, txBuf, pendCnt, pendBuf */
  h = fnv_upd(h, ZO(s, silence), 8);
  h = hash_vals(h, ZO(s, v34), f_V34Obj, NF_V34Obj);
  return h;
}
static void snapshot(char *out, size_t cap, int mw) {
  uint32_t s = 0, d, p, x = 0, k = 0;
  void *st = g_stc[mw] ? P(mw, g_stc[mw]) : NULL;
  if (st) s = hash_state(st);
  d = hash_vals(2166136261u, (uint8_t *)P(mw, g_dp), f_V34ModParObj, NF_V34ModParObj);
  d = hash_vals(d, (uint8_t *)P(mw, g_dp2), f_V34ModParObj, NF_V34ModParObj);
  p = fnv_upd(2166136261u, P(mw, g_pair), 4);
  p = fnv_upd(p, (uint8_t *)P(mw, g_pair) + PAIR_W1, 4);
  if (g_xfc[mw] && blk_find(g_xfc[mw]) >= 0) x = hash_vals(2166136261u, (uint8_t *)P(mw, g_xfc[mw]), f_PcmFlo47Obj, NF_PcmFlo47Obj);
  (void)k;
  snprintf(out, cap, "|s=%08x d=%08x p=%08x x=%08x lvl=%u", s, d, p, x, dsplibs_debug_level);
}
#define ENVREC(mw, ...) rec_add(LOG(mw), (mw), 'c', __VA_ARGS__)

/* ------------------------------------------------------------------------------------------------ the callee environment (the impl's recorders and the model call the same functions) */
static const char *sizename(size_t n) {
  static char b[8][24];
  static int k;
  char *o = b[k++ & 7];
  if (n == ZSIZE) return "state";
  if (n == XFSIZE) return "ctx";
  snprintf(o, 24, "%u", (unsigned)n);
  return o;
}
static void fill_tbl(uint8_t *t, const HFld *tbl, int nf);
static struct { void *p; } g_alq[64];
static unsigned g_nalq, g_malq;
static void *env_malloc(int mw, unsigned size) {
  void *p;
  if (!mw) {
    if (size == ZSIZE && ZSIZE != 0xd258u) h_log_next_size = 0xd258u;
    if (size == XFSIZE && XFSIZE != 0x7f68u) h_log_next_size = 0x7f68u;
    {   /* a failed allocation: the library logs the raw size ("M NULL <size>"), which differs on x86-64; do the logging here with the i386 size */
      int fail = 0;
      unsigned lsz = size == ZSIZE ? 0xd258u : size == XFSIZE ? 0x7f68u : size;
      if (h_fail_next_malloc) { fail = 1; h_fail_next_malloc = 0; } else if (h_fail_malloc_n && --h_fail_malloc_n == 0) fail = 1;
      if (fail) { h_emit(0, "M NULL %u", lsz); h_log_next_size = 0; p = NULL; }
      else { unsigned sv = h_fail_malloc_n; h_fail_malloc_n = 0; p = h_sysdep_malloc(size); h_fail_malloc_n = sv; }
    }
    if (g_nalq >= 64) h_die3("more than 64 allocations in one call");
    g_alq[g_nalq++].p = p;
    if (p) {
      const char *nm = size == ZSIZE ? "state" : size == XFSIZE ? "ctx" : size == 0x14 ? "k56" : "blk";
      /* the allocator's fresh contents depend on the layout: give the block an arch-independent content (the snapshots hash it) */
      if (size == XFSIZE) fill_tbl((uint8_t *)p, f_PcmFlo47Obj, NF_PcmFlo47Obj); else memset(p, 0xa5, size);
      blk_reg(nm, p, size);
    }
  } else {
    if (g_malq >= g_nalq) h_die3("model: more sysdep_malloc calls than the impl made");
    p = g_alq[g_malq++].p;
  }
  if (p && size == ZSIZE) g_stc[mw] = p;
  if (p && size == XFSIZE) g_xfc[mw] = p;
  rec_add(LOG(mw), mw, 'a', "sysdep_malloc(%s)=%s", sizename(size), T_(p));
  return p;
}
static void env_free(int mw, void *p) {
  int i = blk_find(p);
  rec_add(LOG(mw), mw, 'a', "sysdep_free(%s)", T_(p));
  if (i < 0) h_die3("sysdep_free of a pointer outside the world");
  if (!mw) g_bfree[i] = 1;
  if (p == g_stc[mw]) g_stc[mw] = NULL;
  if (p == g_xfc[mw]) g_xfc[mw] = NULL;
  if (!mw) h_sysdep_free(p);
}
void *sysdep_malloc(unsigned size) { return env_malloc(0, size); }
void sysdep_free(void *q) { env_free(0, q); }

static void env_ed(int mw, char kind, const char *s) { rec_add(LOG(mw), mw, kind, "%s", s); poke(mw, CUR(mw)); }
static void *env_memset(int mw, void *dst, int ch, size_t n) {
  const char *nn = (dst == g_stc[mw] && n == ZSIZE) ? "sizeof" : NULL;
  if (nn) ENVREC(mw, "sysdep_memset(dst=%s c=%d n=sizeof)", T_(dst), ch);
  else ENVREC(mw, "sysdep_memset(dst=%s c=%d n=%u)", T_(dst), ch, (unsigned)n);
  if (blk_find(dst) < 0 || n > 4096u + ZSIZE) h_die3("sysdep_memset: unexpected destination / length");
  memset(P(mw, dst), ch, n);
  poke(mw, CUR(mw));
  return dst;
}
static void *env_memcpy(int mw, void *dst, const void *src, size_t n) {
  ENVREC(mw, "sysdep_memcpy(dst=%s src=%s n=%u)", T_(dst), T_(src), (unsigned)n);
  if (blk_find(dst) < 0 || blk_find(src) < 0 || n > 512u) h_die3("sysdep_memcpy: unexpected arguments (n = %lu)", (unsigned long)n);
  memmove(P(mw, dst), P(mw, src), n);
  poke(mw, CUR(mw));
  return dst;
}
static long env_gp(int mw, void *m, unsigned idx) {
  Cur *c = CUR(mw);
  uint32_t r = sres(c);
  long res;
  ENVREC(mw, "modem_get_param(m=%s idx=%u)", T_(m), idx);
  if (idx == 0xa) res = (long)(intptr_t)g_dp;
  else if (idx == 0xb) res = (long)(intptr_t)g_pair;
  else if (idx == 3) { static const uint32_t v[] = { 0, 4800, 9600, 28800, 33600, 56000, 0xdac0, 0xdac1, 0x80000000u, 0xffffffffu }; res = (long)(int32_t)((r >> 8) % 50u == 0 ? r : v[r % 10]); }
  else if (idx == 4) { static const uint32_t v[] = { 0, 4800, 33600, 0xdabf, 0xdac0, 0xdac1, 56000, 0x10000, 0x80000000u, 0xffffffffu }; res = (long)(int32_t)v[r % 10]; }
  else if (idx == 5) { static const int32_t v[] = { 0, 100, 235, 236, 237, 239, 240, 241, 242, 300, 1000, 0x7fffffff, -1, -300, (int32_t)0x80000000, 621, 622, 623, 624, 625, 626, 620 }; res = v[r % 22]; if (r >> 31) res = (int32_t)(r >> 8) % 400 - 100; }
  else res = (long)(int32_t)r;
  poke(mw, c);
  return res;
}
static long env_sp(int mw, void *m, unsigned name, int val) { ENVREC(mw, "modem_set_param(m=%s name=%u val=%d)", T_(m), name, val); poke(mw, CUR(mw)); return 0; }
static int env_getbits(int mw, void *m, int nb, uint8_t *buf, int n) {
  Cur *c = CUR(mw);
  uint32_t r = sres(c);
  int k, i;
  ENVREC(mw, "modem_get_bits(m=%s nbits=%d buf=%s n=%d)", T_(m), nb, T_(buf), n);
  if (n <= 0 || n > 0x400) h_die3("modem_get_bits: n = %d", n);
  switch (r & 7u) { case 0: k = 0; break; case 1: k = n / 2; break; case 2: k = -1; break; case 3: k = 1; break; default: k = n; }
  for (i = 0; i < k; i++) { uint32_t b = sres(c); ((uint8_t *)P(mw, buf))[i] = (uint8_t)(b >> 3); }
  poke(mw, c);
  return k;
}
static int env_putbits(int mw, void *m, int nb, uint8_t *buf, int n) {
  uint32_t h = 2166136261u;
  if (n < 0 || n > 0x400) h_die3("modem_put_bits: n = %d", n);
  h = fnv_upd(h, P(mw, buf), (size_t)n);
  ENVREC(mw, "modem_put_bits(m=%s nbits=%d buf=%s n=%d h=%08x)", T_(m), nb, T_(buf), n, h);
  poke(mw, CUR(mw));
  return n;
}
static int env_dlog(int mw, void *m, unsigned id, const void *data, int cnt) {
  uint32_t h = 2166136261u;
  if (cnt < 0 || cnt > 2 * CLEANSZ) h_die3("modem_debug_log_data: cnt = %d", cnt);
  h = fnv_upd(h, P(mw, data), (size_t)cnt);
  ENVREC(mw, "modem_debug_log_data(m=%s id=%u data=%s cnt=%d h=%08x)", T_(m), id, T_(data), cnt, h);
  poke(mw, CUR(mw));
  return 0;
}
static int env_dpreg(int mw, int dereg, int id, void *op) {
  Cur *c = CUR(mw);
  uint32_t r = sres(c);
  ENVREC(mw, "modem_dp_%sregister(id=%d op=%s)", dereg ? "de" : "", id, T_(op));
  poke(mw, c);
  return dereg ? 0 : (int)(r % 5u == 0 ? -1 : 0);
}
static void *env_k56create(int mw) {
  Cur *c = CUR(mw);
  uint32_t r = sres(c);
  void *p = NULL;
  ENVREC(mw, "K56FLEX_Create()");
  if ((r % 100u) >= g_failpct_k56) p = env_malloc(mw, 0x14);
  poke(mw, c);
  return p;
}
static void env_k56delete(int mw, void *p) {
  ENVREC(mw, "K56FLEX_Delete(p=%s)", T_(p));
  if (p) env_free(mw, p);
  poke(mw, CUR(mw));
}
static int env_v34create(int mw, void *obj, int quick, int w8, void *pm, int mode) {
  Cur *c = CUR(mw);
  uint32_t r = sres(c);
  int res = ((r % 100u) < g_failpct_v34) ? (int)(sres(c) | 1u) : 0;
  ENVREC(mw, "VPcmV34Create(obj=%s quick=%d w8=%d pm=%s mode=%d)", T_(obj), quick, w8, T_(pm), mode);
  poke(mw, c);
  return res;
}
static int32_t pick_status(uint32_t r) {
  switch (r & 15u) { case 0: return -1; case 1: return 0; case 2: return 1; case 3: return 17; case 4: return (int32_t)0x80000000u; case 5: return (int32_t)(r >> 8);
                     default: return (int32_t)((r >> 4) % 17u); }
}
static uint32_t pick_fbits(uint32_t r, uint32_t r2) {
  switch (r % 24u) {
    case 0: return 0x7fc00000u;               /* NaN */
    case 1: return 0xff800000u;               /* -inf */
    case 2: return 0x7f800000u;               /* +inf */
    case 3: return 0xc7000100u;               /* -32769.0 */
    case 4: return 0x47000000u;               /* 32768.0 */
    case 5: return 0xc7000000u;               /* -32768.0 */
    case 6: return 0x46fffe00u;               /* 32767.0 */
    case 7: return 0x4700ffffu;               /* just above 32768 */
    case 8: return 0x7149f2cau;               /* 1e30 */
    case 9: return 0xbf800000u | (r2 & 0x7fffffu);   /* about -1 */
    case 10: return 0x3f800000u | (r2 & 0x7fffffu);  /* about +1 */
    case 11: return 0x80000000u;              /* -0 */
    case 12: return 0x3f000000u | (r2 & 0x7fffffu);  /* about 0.5 */
    case 13: return 0xbf000000u | (r2 & 0x7fffffu);  /* about -0.5 */
    default: return 0x40000000u + ((r2 >> 3) & 0x0fffffffu) - (((r2 >> 2) & 1u) ? 0x20000000u : 0u) + ((r2 & 1u) << 31);   /* mixed magnitudes and signs */
  }
}
static int env_progress(int mw, void *obj, float *in, float *out, unsigned n, int *a4, int *a5, int *a6, int *a7) {
  Cur *c = CUR(mw);
  uint32_t r = sres(c);
  int status = pick_status(r), i, c5;
  uint32_t hin = 2166136261u, hb = 2166136261u;
  int w7 = *(int *)P(mw, a7);
  uint32_t *pbits;
  hin = fnv_upd(hin, P(mw, in), (size_t)n * 4u);
  if (w7 > 0 && w7 <= 0x400) hb = fnv_upd(hb, P(mw, a6), (size_t)w7 * 4u);
  ENVREC(mw, "VPcmV34Progress(obj=%s in=%s out=%s n=%u a4=%s a5=%s a6=%s a7=%s) *a7=%d hin=%08x hbits=%08x", T_(obj), T_(in), T_(out), n, T_(a4), T_(a5), T_(a6), T_(a7), w7, hin, hb);
  if (n > 52u) h_die3("VPcmV34Progress: n = %u", n);
  if (((r >> 4) & 7u) == 0) { poke(mw, c); return status; }   /* the gate of the real function (w262_ == 0): returns the state without touching the buffers or the counts */
  for (i = 0; i < (int)n; i++) { uint32_t b1 = sres(c), b2 = sres(c); ((uint32_t *)P(mw, out))[i] = pick_fbits(b1, b2); }
  c5 = (int)(sres(c) % 6u);
  c5 = c5 == 0 ? 0 : c5 == 1 ? 1 : c5 == 2 ? (int)(sres(c) % 40u) : c5 == 3 ? 0x400 : c5 == 4 ? -1 : (int)(sres(c) % 0x401u);
  *(int *)P(mw, a5) = c5;
  pbits = (uint32_t *)P(mw, a4);
  for (i = 0; i < c5 && i < 0x400; i++) pbits[i] = sres(c);
  { uint32_t q = sres(c);
    switch (q % 8u) { case 0: w7 = 0; break; case 1: w7 = 0x400; break; case 2: w7 = 0x401; break; case 3: w7 = -1; break; case 4: w7 = (int)(q >> 4); break; case 5: w7 = (int)(sres(c) % 0x500u); break; default: w7 = (int)(sres(c) % 100u); }
    *(int *)P(mw, a7) = w7; }
  poke(mw, c);
  return status;
}
static void *env_cleaned(int mw, void *obj, int *out) {
  Cur *c = CUR(mw);
  uint32_t r = sres(c);
  void *res;
  ENVREC(mw, "VPcmV34GetCleanedSamples(obj=%s out=%s)", T_(obj), T_(out));
  *(int *)P(mw, out) = (r & 3u) == 0 ? 0 : (r & 3u) == 1 ? -3 : (int)((r >> 4) % 130u);
  res = (r & 12u) == 0 ? NULL : g_clean;
  { int k; for (k = 0; k < 2 * CLEANSZ; k++) ((uint8_t *)P(mw, g_clean))[k] = (uint8_t)(k * 7 + (r >> 20)); }
  poke(mw, c);
  return res;
}
static int env_sess(int mw, void *obj) { uint32_t r = sres(CUR(mw)); ENVREC(mw, "VPcmV34GetCurrentSessionDP(obj=%s)", T_(obj)); poke(mw, CUR(mw)); return (r & 3u) == 0 ? 0x5a : (r & 3u) == 1 ? 0x5c : (r & 3u) == 2 ? 0x22 : (int)(r >> 8) % 200; }
static int env_rxr(int mw, void *obj) { uint32_t r = sres(CUR(mw)); ENVREC(mw, "VPcmV34GetCurrentRxBitRate(obj=%s)", T_(obj)); poke(mw, CUR(mw)); return (int)(r >> 3) % 60000 - 100; }
static int env_txr(int mw, void *obj) { uint32_t r = sres(CUR(mw)); ENVREC(mw, "VPcmV34GetCurrentTxBitRate(obj=%s)", T_(obj)); poke(mw, CUR(mw)); return (int)(r >> 3) % 40000 - 100; }

/* the VPcmFloModem constructor: the recorder scribbles the object (non-pointer members) with a private xorshift stream, so that a store of 0 right behind it is visible */
static uint32_t xs32(uint32_t *s) { *s ^= *s << 13; *s ^= *s >> 17; *s ^= *s << 5; return *s; }
static void scribble(uint8_t *o, uint32_t seed) {
  uint32_t s = seed | 1u;
  int i;
  for (i = 0; i < NF_PcmFlo47Obj; i++) {
    const HFld *fl = &f_PcmFlo47Obj[i];
    unsigned n = fl->count ? fl->count : 1, es, j;
    uint8_t *q = o + fl->off;
    switch (fl->kind) {
      case 'p': case 'o': case 'e': case 'v': continue;
      case 'b': case 'x': es = 1; break;
      case 'h': es = 2; break;
      case 'd': es = 8; break;
      default: es = 4; break;
    }
    for (j = 0; j < n * es; j++) q[j] = (uint8_t)(xs32(&s) >> 11);
  }
}
static void env_ctor(int mw, void *self, void *ctx, unsigned side, void *params, unsigned blk, unsigned m90, unsigned m92) {
  Cur *c = CUR(mw);
  uint32_t r = sres(c);
  ENVREC(mw, "VPcmFloModem::VPcmFloModem(this=%s ctx=%s side=%u params=%s blk=%u m90=%u m92=%u)", T_(self), T_(ctx), side, T_(params), blk, m90, m92);
  if (self) { scribble((uint8_t *)P(mw, self), r); ((PcmFlo47Obj *)P(mw, self))->v90Dem_ = g_dem; }
  poke(mw, c);
}
static void env_dtor(int mw, const char *nm, size_t off, void *self) {
  long d = g_xfc[mw] ? (long)((uint8_t *)self - (uint8_t *)g_xfc[mw]) - (long)off : 12345;
  if (d == 0) ENVREC(mw, "~%s(this=ctx.member)", nm);
  else ENVREC(mw, "~%s(this=ctx+%ld)", nm, d);
  poke(mw, CUR(mw));
}
static int env_term(int mw, void *self) {
  uint32_t r = sres(CUR(mw));
  ENVREC(mw, "V90Demodulator::sessionTermination(this=%s)", T_(self));
  poke(mw, CUR(mw));
  return (r & 1u) ? 0 : (int)(r >> 1);
}

/* the recorders (the blob's callees, weakened in the copy of dsplibs.o) */
extern void *rc_memset(void *, int, size_t) L("sysdep_memset");
extern void *rc_memcpy(void *, const void *, size_t) L("sysdep_memcpy");
extern long rc_gp(void *, unsigned) L("modem_get_param");
extern long rc_sp(void *, unsigned, int) L("modem_set_param");
extern int rc_getbits(void *, int, uint8_t *, int) L("modem_get_bits");
extern int rc_putbits(void *, int, uint8_t *, int) L("modem_put_bits");
extern int rc_dlog(void *, unsigned, const void *, int) L("modem_debug_log_data");
extern int rc_dpreg(int, void *) L("modem_dp_register");
extern void rc_dpdereg(int, void *) L("modem_dp_deregister");
extern void *rc_k56c(void) L("K56FLEX_Create");
extern void rc_k56d(void *) L("K56FLEX_Delete");
extern int rc_v34c(void *, int, int, void *, int) L("VPcmV34Create");
extern int rc_prog(void *, float *, float *, unsigned, int *, int *, int *, int *) L("VPcmV34Progress");
extern void *rc_clean(void *, int *) L("VPcmV34GetCleanedSamples");
extern int rc_sess(void *) L("VPcmV34GetCurrentSessionDP");
extern int rc_rxr(void *) L("VPcmV34GetCurrentRxBitRate");
extern int rc_txr(void *) L("VPcmV34GetCurrentTxBitRate");
extern void rc_ctor(void *, void *, unsigned, void *, unsigned, unsigned, unsigned) L("_ZN12VPcmFloModemC1EPv12V90ModemSideP19_tagModemParametersj20V90ComputationalMode20V92ComputationalMode");
extern void rc_d_iir(void *) L("_ZN10GenericIIRIfdED1Ev");
extern void rc_d_sine(void *) L("_ZN8SineWaveIffED1Ev");
extern void rc_d_ans(void *) L("_ZN17ANSamToneDetectorD1Ev");
extern void rc_d_ec(void *) L("_ZN16V92EchoCancellerD1Ev");
extern void rc_d_v92(void *) L("_ZN8V92ModemD1Ev");
extern void rc_d_v90(void *) L("_ZN8V90ModemD1Ev");
extern int rc_term(void *) L("_ZN14V90Demodulator18sessionTerminationEv");
void *rc_memset(void *d, int c, size_t n) { return env_memset(0, d, c, n); }
void *rc_memcpy(void *d, const void *s, size_t n) { return env_memcpy(0, d, s, n); }
long rc_gp(void *m, unsigned i) { return env_gp(0, m, i); }
long rc_sp(void *m, unsigned n, int v) { return env_sp(0, m, n, v); }
int rc_getbits(void *m, int nb, uint8_t *b, int n) { return env_getbits(0, m, nb, b, n); }
int rc_putbits(void *m, int nb, uint8_t *b, int n) { return env_putbits(0, m, nb, b, n); }
int rc_dlog(void *m, unsigned id, const void *d, int n) { return env_dlog(0, m, id, d, n); }
int rc_dpreg(int id, void *op) { if (!g_op) g_op = op; return env_dpreg(0, 0, id, op); }
void rc_dpdereg(int id, void *op) { if (!g_op) g_op = op; env_dpreg(0, 1, id, op); }
void *rc_k56c(void) { return env_k56create(0); }
void rc_k56d(void *p) { env_k56delete(0, p); }
int rc_v34c(void *o, int q, int w8, void *pm, int mode) { return env_v34create(0, o, q, w8, pm, mode); }
int rc_prog(void *o, float *in, float *out, unsigned n, int *a4, int *a5, int *a6, int *a7) { return env_progress(0, o, in, out, n, a4, a5, a6, a7); }
void *rc_clean(void *o, int *out) { return env_cleaned(0, o, out); }
int rc_sess(void *o) { return env_sess(0, o); }
int rc_rxr(void *o) { return env_rxr(0, o); }
int rc_txr(void *o) { return env_txr(0, o); }
void rc_ctor(void *t, void *ctx, unsigned side, void *pm, unsigned blk, unsigned m90, unsigned m92) { env_ctor(0, t, ctx, side, pm, blk, m90, m92); }
void rc_d_iir(void *t) { env_dtor(0, "GenericIIR", offsetof(PcmFlo47Obj, iirDen_), t); }
void rc_d_sine(void *t) { env_dtor(0, "SineWave", offsetof(PcmFlo47Obj, sineAmp_), t); }
void rc_d_ans(void *t) { env_dtor(0, "ANSamToneDetector", offsetof(PcmFlo47Obj, ansIir_), t); }
void rc_d_ec(void *t) { env_dtor(0, "V92EchoCanceller", offsetof(PcmFlo47Obj, ecParams_), t); }
void rc_d_v92(void *t) { env_dtor(0, "V92Modem", offsetof(PcmFlo47Obj, v92Mod_), t); }
void rc_d_v90(void *t) { env_dtor(0, "V90Modem", offsetof(PcmFlo47Obj, v90Mod_), t); }
int rc_term(void *t) { return env_term(0, t); }
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  env_ed(0, 'd', buf);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ the independent model (from porting/decompiled/dp/z01_glue.txt): the disassembly as gotos */
static void mpf(char kind, const char *fmt, ...) {
  char b[TXTMAX * 2];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  env_ed(1, kind, b);
}
#define MC_ (&g_mc)

/* n = the buffer size VPCMXF_Create derives from the rate: the blob computes rate * C + 0.5 in the x87 (64-bit mantissa; C = the double 9.6 for side == 0, else 8.0f) and stores with the truncating control word;
 * here: exact integer arithmetic.  side == 0: floor(rate * 5404319552844595 / 2^49 + 0.5) with 9.6 the DOUBLE 5404319552844595 / 2^49 (the fraction of the product is 0, .2, .4, .6, .8 minus a tiny
 * representation error, never within an extended-precision ulp of .5, so the truncation is that of the exact sum); else rate * 8 (+ 0.5 truncated away); a result outside int32 is the x87 integer
 * indefinite 0x80000000 */
static int32_t model_n(int side, uint32_t rate) {
  uint64_t r;
  if (side == 0) {
    const uint64_t M = 5404319552844595ull;
    uint64_t a = (uint64_t)rate * (M >> 32), b = (uint64_t)rate * (M & 0xffffffffu);
    r = (a + (b >> 32) + 0x10000u) >> 17;
  } else {
    r = (uint64_t)rate * 8u;
  }
  return r >= 0x80000000ull ? (int32_t)0x80000000u : (int32_t)r;
}
static void *m_xfcreate(int side, void *ctxI, void *paramsI, unsigned rate, int mode) {
  int32_t n = model_n(side, rate);
  unsigned m90 = 0, m92 = 0;
  void *objI;
  PcmFlo47Obj *o;
  if (mode == 2) { CV(G_XF, "xf: mode 2"); m92 = 1; }
  else if (mode > 2) { if (mode == 3) { CV(G_XF, "xf: mode 3"); m90 = 1; m92 = 1; } else CV(G_XF, "xf: mode > 3"); }
  else if (mode == 1) { CV(G_XF, "xf: mode 1"); m90 = 1; }
  else CV(G_XF, "xf: mode < 1");
  if (DBG()) { CV(G_XF, "xf: create print"); mpf('d', "VPCMXF_Create: side is %s, maxDataBuffer - %d\r\n", side ? "Digital" : "Analog", n); }
  objI = env_malloc(1, (unsigned)XFSIZE);
  env_ctor(1, objI, ctxI, side == 0 ? 1u : 0u, paramsI, (unsigned)n, m90, m92);
  if (!objI) { CV(G_XF, "xf: new failed"); if (DBG()) mpf('d', "VPCMXF_Create: new VPcmFloModem() failed.\n"); return NULL; }
  o = (PcmFlo47Obj *)P(1, objI);
  o->cpnotLen_ = 0; o->bitPtr_ = 0;
  o->trainConstel_ = 0; o->rrnConstel_ = 0; o->localRrn_ = 0; o->dropToV34_ = 0; o->clearDown_ = 0;
  o->v34Flags_[0] = 1; o->v34Flags_[1] = 0; o->v34Flags_[2] = 1; o->v34Flags_[3] = 1; o->v34Flags_[4] = 1; o->v34Flags_[5] = 0;
  o->cpLen_ = 0; o->jaFlag_ = 0; o->cpFlag_ = 0; o->cpnotFlag_ = 0; o->cpnotCopied_ = 0; o->nofBitsPhase4_ = 2; o->cpSeqCnt_ = 0; o->minTxSeq_ = 1;
  return objI;
}
static void m_xfdelete(void *pI) {
  if (!pI) { CV(G_XF, "xf: delete NULL"); return; }
  CV(G_XF, "xf: delete");
  env_dtor(1, "GenericIIR", offsetof(PcmFlo47Obj, iirDen_), (uint8_t *)pI + offsetof(PcmFlo47Obj, iirDen_));
  env_dtor(1, "SineWave", offsetof(PcmFlo47Obj, sineAmp_), (uint8_t *)pI + offsetof(PcmFlo47Obj, sineAmp_));
  env_dtor(1, "ANSamToneDetector", offsetof(PcmFlo47Obj, ansIir_), (uint8_t *)pI + offsetof(PcmFlo47Obj, ansIir_));
  env_dtor(1, "V92EchoCanceller", offsetof(PcmFlo47Obj, ecParams_), (uint8_t *)pI + offsetof(PcmFlo47Obj, ecParams_));
  env_dtor(1, "V92Modem", offsetof(PcmFlo47Obj, v92Mod_), (uint8_t *)pI + offsetof(PcmFlo47Obj, v92Mod_));
  env_dtor(1, "V90Modem", offsetof(PcmFlo47Obj, v90Mod_), (uint8_t *)pI + offsetof(PcmFlo47Obj, v90Mod_));
  env_free(1, pI);
}
static int m_xfterm(void *pI) { return env_term(1, ((PcmFlo47Obj *)P(1, pI))->v90Dem_); }

static void *m_create(void *modem, int id, int caller, int rate, int frag, void *op) {
  void *sI, *s, *dpI, *ctxI, *k56I;
  int t, bit4, session, d, eax_ext;
  unsigned lo, hi;
  V34ModParObj *dp;
  if (rate != 0x2580) { CV(G_CREATE, "create: rate != 9600"); return NULL; }
  if (DBG()) { CV(G_CREATE, "create: print"); mpf('d', "vpcm: create: dp %d, caller %d, frag %d (size %d).\n", id, caller, frag, 0xd258); }
  if (frag > 0x30) { CV(G_CREATE, "create: frag > 0x30"); return NULL; }
  sI = env_malloc(1, (unsigned)ZSIZE);
  if (!sI) { CV(G_CREATE, "create: malloc failed"); return NULL; }
  s = P(1, sI);
  env_memset(1, sI, 0, ZSIZE);
  ZP(s, dpData) = sI; ZI(s, txCnt) = 4; ZP(s, modem) = modem; ZI(s, id) = id; ZP(s, op) = op; ZI(s, status) = 0; ZI(s, link) = 0; ZI(s, trainCnt) = 0;
  ZP(s, pair) = (void *)(intptr_t)env_gp(1, modem, 0xb);
  dpI = (void *)(intptr_t)env_gp(1, modem, 0xa);   /* dp_param_get = modem_get_param(modem, 0xa) */
  ZP(s, dp) = dpI;
  ((V34ModParObj *)P(1, dpI))->w78_ = 0;
  t = frag * 1000 / rate;
  ctxI = m_xfcreate(0, (uint8_t *)sI + z01_probe_Z01StateObj[ZS_v34], dpI, (unsigned)t, 0);
  ZV34(s)->ctx_ = ctxI;
  if (!ctxI) { CV(G_CREATE, "create: ctx failed"); env_free(1, sI); return NULL; }
  k56I = env_k56create(1);
  ZV34(s)->k56_ = k56I;
  if (!k56I) { CV(G_CREATE, "create: k56 failed"); m_xfdelete(ctxI); env_free(1, sI); return NULL; }
  lo = (unsigned)env_gp(1, modem, 3);
  hi = (unsigned)env_gp(1, modem, 4);
  if (hi > 0xdac0u) { CV(G_CREATE, "create: max rate clamped"); hi = 0xdac0u; }
  if (DBG()) { CV(G_CREATE, "create: rate print"); mpf('d', "vpcm: VPCM rate limits: %d-%d\n", (int)lo, (int)hi); }
  dp = (V34ModParObj *)P(1, ZP(s, dp));
  dp->minRate_ = lo; dp->maxRate_ = hi; dp->upMinRate_ = 0x12c0; dp->upMaxRate_ = 0x8340;
  bit4 = (id == 0x5c) ? ((dp->flags2_ >> 4) & 1) : 0;
  if (id == 0x5c) CV(G_CREATE, "create: V.92 keeps bit 4"); else CV(G_CREATE, "create: bit 4 cleared");
  dp->flags2_ = (uint8_t)((dp->flags2_ & 0xef) | (bit4 << 4));
  ((V34ModParObj *)P(1, ZP(s, dp)))->flags2_ &= 0xdf;
  { unsigned f = (((V34ModParObj *)P(1, ZP(s, dp)))->flags2_ >> 4) & 1u; ZI(s, silence) = f < 1 ? 0x210 : 0; if (f) CV(G_CREATE, "create: silence 0"); else CV(G_CREATE, "create: silence 0x210"); }
  t = (int)env_gp(1, modem, 5) + 4;
  d = 0xf4 - t;
  if (d < 0) {
    int a;
    CV(G_CREATE, "create: hardware delay too large");
    env_sp(1, modem, 0xd, d);
    a = -d;
    if (a >= 0x180) CV(G_CREATE, "create: add delay > 0x180"); else a = 0x180;
    ZI(s, addDelay) = a;
    t = 0xf4;
  } else CV(G_CREATE, "create: hardware delay fits");
  dp = (V34ModParObj *)P(1, ZP(s, dp));                      /* 3c03: the reload before the delay stores */
  eax_ext = t - 0x30;
  dp->extDelay_ = eax_ext;                                   /* the blob stores t - 0x30 first, then adds addDelay (read after the stores) and stores again */
  dp->delay_ = t;
  eax_ext += ZI(s, addDelay);
  dp->extDelay_ = eax_ext;
  if (DBG()) {                                               /* 3d00: [4] = reloaded dp->delay_, [8] = the value just computed; 3d1a: dp reloaded again, only on this path */
    CV(G_CREATE, "create: delays print");
    mpf('d', "vpcm: Delays: HW %d, DMA %d\n", ((V34ModParObj *)P(1, ZP(s, dp)))->delay_, eax_ext);
    dp = (V34ModParObj *)P(1, ZP(s, dp));
  }
  dp->w6c_ = 0;                                              /* 3c2c: ecx of 3c03 (or the reload of 3d1a) */
  session = id == 0x5c ? 2 : (id == 0x5a ? 1 : 0);
  if (DBG()) { CV(G_CREATE, "create: session print"); mpf('d', "vpcm: initial dp V.%d, session type %d.\n", id, session); }
  if (env_v34create(1, (uint8_t *)sI + z01_probe_Z01StateObj[ZS_v34], caller == 0, frag, ZP(s, dp), session) != 0) {
    CV(G_CREATE, "create: VPcmV34Create failed");
    m_xfdelete(ZV34(s)->ctx_);
    env_k56delete(1, ZV34(s)->k56_);
    env_free(1, sI);
    return NULL;
  }
  CV(G_CREATE, "create: ok");
  return sI;
}
static int m_delete(void *wI) {
  void *w = P(1, wI);
  void *sI = ZP(w, dpData), *s = P(1, sI);
  V34ModParObj *dp;
  uint8_t *pair;
  if (DBG()) { CV(G_CREATE, "delete: print"); mpf('d', "vpcm: delete...\n"); }
  dp = (V34ModParObj *)P(1, ZP(s, dp));
  pair = (uint8_t *)P(1, ZP(s, pair));
  *(long *)(pair + PAIR_W1) = (long)(int32_t)dp->regWord_;   /* the whole long member, sign-extended */
  *(int32_t *)pair = (int32_t)dp->connType_;
  m_xfterm(ZV34(s)->ctx_);
  m_xfdelete(ZV34(s)->ctx_);
  env_k56delete(1, ZV34(s)->k56_);
  env_free(1, sI);
  CV(G_CREATE, "delete: done");
  return 0;
}

static int16_t m_trunc16(float f) { double d = f; return (d > -32769.0 && d < 32768.0) ? (int16_t)(int)d : (int16_t)0x8000; }
static int m_run(void *wI, short *rxI, short *txI, int n) {
  void *w = P(1, wI);
  void *sI = ZP(w, dpData), *s = P(1, sI);
  uint8_t *base = (uint8_t *)sI;
  int s44 = 0, s34 = 0, s38 = 0, s3c = 0, s58 = 0, s54 = 0, s50;
  int ecx, edx, eax, edi, esi;
  uint8_t *s40 = NULL;
  V34ModParObj *dp;
#define STB(m) (base + z01_probe_Z01StateObj[ZS_##m])
#define MS(m) ((uint8_t *)P(1, STB(m)))
#define MI(m) (*(int32_t *)MS(m))
#define MDP() ((V34ModParObj *)P(1, ZP(s, dp)))
  ecx = MI(pendCnt);
  edx = ecx + n; eax = edx;
  if (edx < 0) eax = edx + 3;
  edi = eax & ~3;
  if (ecx <= 0) goto L3eb6;
  esi = MI(pendCnt);
  env_memcpy(1, STB(pendBuf) + 2 * esi, rxI, (size_t)(2 * n));
  rxI = (short *)STB(pendBuf);
L3eb6:
  if (edi <= 0) { CV(G_RUN, "run: no whole block"); goto L3f19; }
  esi = MI(txCnt);
  s40 = STB(txBuf) + 2 * esi;
  s34 = 0;
  esi = MI(silence);
  s38 = MI(link);
  if (esi <= 0) goto L3fb7;
  CV(G_RUN, "run: silence");
  env_memset(1, s40, 0, (size_t)(2 * edi));
  eax = MI(silence) - edi;
  if (eax <= 0) { CV(G_RUN, "run: silence ends"); eax = 0; }
  MI(silence) = eax;
  goto L3f19;
L3fb7:
  s54 = 0;
  eax = MI(bitsWanted);
  s58 = eax;
  if (eax <= 0) { CV(G_RUN, "run: no bits wanted"); goto L4004; }
  CV(G_RUN, "run: get bits");
  eax = env_getbits(1, ZP(w, modem), 1, STB(bitsIn), eax);
  s58 = eax;
  for (edx = eax - 1; edx >= 0; edx--) ((uint32_t *)MS(bitsIn))[edx] = MS(bitsIn)[edx];
L4004:
  for (eax = 0; eax < edi; eax++) ((float *)MS(inF))[eax] = (float)((short *)P(1, rxI))[eax];
  s3c = env_progress(1, STB(v34), (float *)STB(inF), (float *)STB(outF), (unsigned)edi, (int *)STB(bitsOut), &s54, (int *)STB(bitsIn), &s58);
  s50 = 0;
  { void *cl = env_cleaned(1, STB(v34), &s50);
    if (cl && s50 > 0) { CV(G_RUN, "run: cleaned samples logged"); env_dlog(1, ZP(w, modem), 3, cl, s50 * 2); } }
  for (edx = 0; edx < edi; edx++) ((int16_t *)P(1, s40))[edx] = m_trunc16(((float *)MS(outF))[edx]);
  eax = s58;
  if ((unsigned)eax <= 0x400u) goto L4111;
  CV(G_RUN, "run: too many bits");
  if (DBG()) mpf('d', "vpcm: too many bits requested (%d)\n", eax);
  eax = 0x400; s58 = 0x400;
L4111:
  MI(bitsWanted) = eax;
  eax = s54;
  if (eax <= 0) goto L4157;
  CV(G_RUN, "run: put bits");
  for (edx = 0; edx < s54; edx++) MS(bitsOut)[edx] = (uint8_t)(MS(bitsOut)[4 * edx] & 1);
  env_putbits(1, ZP(w, modem), 1, STB(bitsOut), s54);
L4157:
  ecx = s3c;
  if (MI(status) == ecx) goto L4268;
  if (DBG()) mpf('d', "vpcm: New status %d\n", ecx);
  if ((unsigned)s3c > 0x10u) { CV(G_RUN, "run: status > 0x10"); goto L4210; }
  switch (s3c) {
    case 0: goto L430b;
    case 1: goto L418f;
    case 4: case 5: goto L42fd;
    case 6: case 7: goto L42f2;
    case 8: case 9: case 16: goto L428c;
    case 10: goto L42d4;
    default: CV(G_RUN, "run: status without action"); goto L4210;
  }
L4268:
  if (s3c != 0) { CV(G_RUN, "run: same status"); goto L4210; }
  esi = MI(trainCnt) + 1;
  MI(trainCnt) = esi;
  if (esi <= 0xbb8) { CV(G_RUN, "run: training counted"); goto L4210; }
  CV(G_RUN, "run: train timeout");
  if (DBG()) mpf('d', "vpcm: train timeout!\n");
L428c:
  s38 = -1;
  goto L4210;
L418f:
  CV(G_RUN, "run: status 1");
  if (DBG()) mpf('d', "vpcm: Phase II completed !!!\n");
  edx = MI(addDelay);
  if (edx == 0) goto L4210;
  dp = MDP();
  if (dp->w6c_ != 0) goto L4210;
  CV(G_RUN, "run: phase 2 completed: delay added");
  env_sp(1, ZP(w, modem), 0xd, edx);
  edx = MI(addDelay);
  MDP()->w6c_ = (uint32_t)edx;
  if (DBG()) mpf('d', "vpcm: P2 FINISHED: increase delay!! init %d, ext %d, add %d\n", MDP()->delay_, MDP()->extDelay_, edx);
  goto L4210;
L430b:
  CV(G_RUN, "run: status 0");
  if (DBG()) mpf('d', "vpcm: Re-starting phase II\n");
  MI(trainCnt) = 0;
  s38 = 0;
  if ((int32_t)MDP()->w6c_ <= 0) goto L4210;
  eax = MI(addDelay);
  if (eax == 0) goto L4210;
  CV(G_RUN, "run: restart: delay removed");
  env_sp(1, ZP(w, modem), 0xd, -eax);
  MDP()->w6c_ = 0;
  if (DBG()) mpf('d', "vpcm: P2 RESTART: decrease delay!! init %d, ext %d, add %d\n", MDP()->delay_, MDP()->extDelay_, 0);
  goto L4210;
L42fd:
  CV(G_RUN, "run: link up status");
  s38 = 1;
  goto L4210;
L42f2:
  CV(G_RUN, "run: link down status");
  s38 = 0;
  goto L4210;
L42d4:
  CV(G_RUN, "run: line verification status");
  if (DBG()) mpf('d', "vpcm: Same Line Verification Status\n");
L4210:
  edx = s38;
  eax = s3c;
  MI(status) = eax;
  if (MI(link) == edx) goto L4232;
  if (edx == -1) goto L42af;
  if (s38 != 1) goto L4232;
  CV(G_RUN, "run: link established");
  {
    int sess = env_sess(1, STB(v34)), rxr = env_rxr(1, STB(v34)), txr = env_txr(1, STB(v34));
    if (DBG()) mpf('d', "vpcm: Link: DP is V.%d, rate: rx %d, tx %d\n", sess, rxr, txr);
    if (sess == 0x5a) MI(id) = 0x5a;
    else if (sess == 0x5c) MI(id) = 0x5c;
    else MI(id) = 0x22;
    env_sp(1, ZP(w, modem), 2, txr);
    env_sp(1, ZP(w, modem), 1, rxr);
  }
  s34 = 1;
  goto L4232;
L42af:
  CV(G_RUN, "run: link error");
  if (DBG()) mpf('d', "vpcm: Link Error\n");
  s34 = -1;
L4232:
  MI(link) = s38;
  if (s58 != 0) goto L4248;
  if (s38 - 1 != 0) goto L4248;
  CV(G_RUN, "run: no symbols requested");
  if (DBG()) mpf('d', "vpcm: No symbols Requested!!!\n");
L4248:
  if (s34 == 0) goto L3f19;
  s44 = s34;
L3f19:
  eax = MI(pendCnt) + n;
  edx = MI(pendCnt);
  if (eax == edi) goto L3f52;
  CV(G_RUN, "run: pending samples kept");
  env_memcpy(1, STB(pendBuf), (uint8_t *)rxI + 2 * edi, (size_t)((eax - edi) * 2));
  edx = MI(pendCnt);
L3f52:
  MI(pendCnt) = edx + n - edi;
  env_memcpy(1, txI, STB(txBuf), (size_t)(2 * n));
  eax = MI(txCnt) + edi - n;
  env_memcpy(1, STB(txBuf), STB(txBuf) + 2 * n, (size_t)(2 * eax));
  MI(txCnt) = MI(txCnt) + edi - n;
  return s44;
}

/* ------------------------------------------------------------------------------------------------ world setup */
static void fill_blk(void *p, size_t n) { size_t j; uint32_t s = h_rnd(); for (j = 0; j + 4 <= n; j += 4) { s = s * 1664525u + 1013904223u; memcpy((uint8_t *)p + j, &s, 4); } }
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
      default: continue;
    }
    tot = n * es;
    for (j = 0; j < tot; j += 4) { uint32_t r = h_rnd(); memcpy(q + j, &r, tot - j < 4 ? tot - j : 4); }
  }
}
static void fill_dp(V34ModParObj *p) {
  fill_tbl((uint8_t *)p, f_V34ModParObj, NF_V34ModParObj);
  p->flags2_ = (uint8_t)(h_rpct(30) ? 0x10 : h_rpct(30) ? 0x20 : h_rpct(30) ? 0x30 : h_rpct(30) ? 0 : h_rnd());
  if (h_rpct(60)) p->w6c_ = h_rpct(50) ? 0 : h_rpct(50) ? 1 : (h_rnd() & 0xffff);
  p->delay_ = (int32_t)h_rmod(600) - 100; p->extDelay_ = (int32_t)h_rmod(900) - 100;
}
static void new_world(void) {
  size_t k;
  blk_reset();
  memset(g_bfree, 0, sizeof g_bfree);
  g_nalq = g_malq = 0;
  g_stc[0] = g_stc[1] = NULL; g_xfc[0] = g_xfc[1] = NULL;
  g_modem = h_obj_new(64); blk_reg("modem", g_modem, 64);
  g_dp = h_obj_new(sizeof(V34ModParObj)); blk_reg("dp", g_dp, sizeof(V34ModParObj));
  g_dp2 = h_obj_new(sizeof(V34ModParObj)); blk_reg("dp2", g_dp2, sizeof(V34ModParObj));
  g_pair = h_obj_new(16); blk_reg("pair", g_pair, 16);
  g_in = h_obj_new(2 * INSZ); blk_reg("in", g_in, 2 * INSZ);
  g_out = h_obj_new(2 * OUTSZ); blk_reg("out", g_out, 2 * OUTSZ);
  g_clean = h_obj_new(2 * CLEANSZ); blk_reg("cleaned", g_clean, 2 * CLEANSZ);
  g_xctx = h_obj_new(64); blk_reg("xctx", g_xctx, 64);
  g_dem = h_obj_new(64); blk_reg("dem", g_dem, 64);
  fill_dp((V34ModParObj *)g_dp); fill_dp((V34ModParObj *)g_dp2);
  fill_blk(g_pair, 16);
  if (PAIR_W1 != 4) memcpy((uint8_t *)g_pair + PAIR_W1, (uint8_t *)g_pair + 4, 4);   /* the same random word in the clock_deviation member as the 32-bit run has at +4 */
  for (k = 0; k < INSZ; k++) ((short *)g_in)[k] = (short)(h_rpct(8) ? (h_rpct(50) ? -32768 : 32767) : h_rpct(10) ? 0 : (int)(h_rnd() >> 16));
  fill_blk(g_out, 2 * OUTSZ);
  blk_sync();
}

/* ------------------------------------------------------------------------------------------------ one call */
static void dump_world(int show) {
  char b[256];
  h_emit(show, "world lvl=%u", dsplibs_debug_level);
  h_dump_obj(show, "DP", g_dp, f_V34ModParObj, NF_V34ModParObj);
  { const int32_t *pr = (const int32_t *)g_pair; h_emit(show, "pair %08x %08x", (unsigned)pr[0], (unsigned)PW1(g_pair)); }
  if (g_stc[0]) {
    void *s = g_stc[0];
    int i;
    uint32_t hi = 2166136261u, ho = 2166136261u, hbi = 2166136261u, hbo = 2166136261u;
    hi = fnv_upd(hi, ZO(s, inF), 160 * 4); ho = fnv_upd(ho, ZO(s, outF), 160 * 4); hbi = fnv_upd(hbi, ZO(s, bitsIn), 1024 * 4); hbo = fnv_upd(hbo, ZO(s, bitsOut), 1024 * 4);
    snprintf(b, sizeof b, "state id=%d status=%d link=%d bitsWanted=%d trainCnt=%d txCnt=%d pendCnt=%d silence=%d addDelay=%d", ZI(s, id), ZI(s, status), ZI(s, link), ZI(s, bitsWanted), ZI(s, trainCnt),
             ZI(s, txCnt), ZI(s, pendCnt), ZI(s, silence), ZI(s, addDelay));
    h_emit(show, "%s", b);
    h_emit(show, "state ptrs modem=%s op=%s self=%s pair=%s dp=%s ctx=%s k56=%s", T_(ZP(s, modem)), T_(ZP(s, op)), T_(ZP(s, dpData)), T_(ZP(s, pair)), T_(ZP(s, dp)), T_(ZV34(s)->ctx_), T_(ZV34(s)->k56_));
    for (i = 0; i < 52; i++) { snprintf(b + (i % 13) * 5, 6, "%04x ", ((uint16_t *)ZO(s, txBuf))[i]); if (i % 13 == 12) h_emit(show, "tx %s", b); }
    for (i = 0; i < 52; i++) { snprintf(b + (i % 13) * 5, 6, "%04x ", ((uint16_t *)ZO(s, pendBuf))[i]); if (i % 13 == 12) h_emit(show, "pend %s", b); }
    h_emit(show, "state bufs in=%08x out=%08x bitsIn=%08x bitsOut=%08x v34=%08x", hi, ho, hbi, hbo, (unsigned)hash_vals(2166136261u, ZO(s, v34), f_V34Obj, NF_V34Obj));
  }
}

static void begin_call(const char *what, int sc) {
  blk_gc();
  blk_sync();
  memset(&g_mc, 0, sizeof g_mc); memset(&g_ic, 0, sizeof g_ic);
  g_ml.n = 0; g_il.n = 0;
  g_nalq = g_malq = 0;
  cov_clear();
  g_what = what;
  g_sc_id = sc;
  g_snapfn = snapshot;
  g_show = h_sample(&g_symctr, 12, 300) || g_showall;
}
static unsigned g_lv0;
static void *g_st0, *g_xf0;
static void mid_call(void) {   /* between the impl and the model */
  unsigned i;
  for (i = 0; i < g_il.n && i < LOGMAX; i++) h_emit(g_show, "%s", g_il.r[i].t);
  dsplibs_debug_level = g_lv0;
  h_fail_next_malloc = 0; h_fail_malloc_n = 0;
  g_malq = 0;
  g_stc[1] = g_st0; g_xfc[1] = g_xf0;
}
static void end_call(void) {
  log_cmp(&g_ml, &g_il, g_what);
  if (memcmp(&g_mc, &g_ic, sizeof g_mc)) h_die3("%s: the script cursors differ", g_what);
  if (g_malq != g_nalq) h_die3("%s: the model made %u allocations, the impl %u", g_what, g_malq, g_nalq);
  if (g_stc[0] != g_stc[1] || g_xfc[0] != g_xfc[1]) h_die3("%s: the live state / ctx pointers differ", g_what);
  blk_cmp(g_what);
  cov_flush();
  g_ncalls++;
}
#define PRE() do { g_lv0 = dsplibs_debug_level; g_st0 = g_stc[0]; g_xf0 = g_xfc[0]; } while (0)

/* ---- the VPCMXF_* calls ---- */
static void *do_xfcreate(int side, int mode, unsigned rate) {
  void *ri, *rm;
  begin_call("VPCMXF_Create", G_XF);
  PRE();
  h_emit(g_show, "call %s side=%d mode=%d rate=%u lvl=%u", g_what, side, mode, rate, dsplibs_debug_level);
  ri = f_xfcreate(side, g_xctx, g_dp, rate, mode);
  mid_call();
  rm = m_xfcreate(side, g_xctx, g_dp, rate, mode);
  if (ri != rm) h_die3("%s: the model returns %s, the impl %s", g_what, T_(rm), T_(ri));
  end_call();
  h_emit(g_show, "ret %s", T_(ri));
  if (ri) h_dump_obj(g_show, "X", ri, f_PcmFlo47Obj, NF_PcmFlo47Obj);
  return ri;
}
static void do_xfdelete(void *p) {
  begin_call("VPCMXF_Delete", G_XF);
  g_xfc[0] = p;   /* the destructors check their `this` against it */
  PRE();
  h_emit(g_show, "call %s p=%s lvl=%u", g_what, T_(p), dsplibs_debug_level);
  f_xfdelete(p);
  mid_call();
  m_xfdelete(p);
  end_call();
}
static void do_xfterm(void *p) {
  int ri, rm;
  begin_call("VPCMXF_SessionTermination", G_XF);
  PRE();
  h_emit(g_show, "call %s p=%s lvl=%u", g_what, T_(p), dsplibs_debug_level);
  ri = f_xfterm(p);
  mid_call();
  rm = m_xfterm(p);
  if (ri != rm) h_die3("%s: the model returns %d, the impl %d", g_what, rm, ri);
  end_call();
  h_emit(g_show, "ret %d", ri);
}

/* ---- the dp_operations calls ---- */
static CreateFn g_createfn;
static DeleteFn g_deletefn;
static RunFn g_runfn;
static void get_ops(void) {
  Ops *op;
  if (!g_op) { f_init(); if (!g_op) h_die3("dp_vpcm_init did not register anything"); f_exit(); g_il.n = 0; }
  op = (Ops *)g_op;
  g_createfn = (CreateFn)op->create; g_deletefn = (DeleteFn)op->del; g_runfn = (RunFn)op->process;
}
static void *do_create(int id, int caller, int rate, int frag, void *op) {
  void *ri, *rm;
  begin_call("vpcm_create", G_CREATE);
  PRE();
  h_emit(g_show, "call %s id=%d caller=%d rate=%d frag=%d op=%s lvl=%u v34fail=%u k56fail=%u", g_what, id, caller, rate, frag, T_(op), dsplibs_debug_level, g_failpct_v34, g_failpct_k56);
  ri = g_createfn(g_modem, id, caller, rate, frag, op);
  mid_call();
  rm = m_create(g_modem, id, caller, rate, frag, op);
  if (ri != rm) h_die3("%s: the model returns %s, the impl %s", g_what, T_(rm), T_(ri));
  end_call();
  h_emit(g_show, "ret %s", T_(ri));
  dump_world(g_show);
  if (ri && g_xfc[0]) h_dump_obj(g_show, "X", g_xfc[0], f_PcmFlo47Obj, NF_PcmFlo47Obj);
  return ri;
}
static void do_delete(void *w) {
  int ri, rm;
  void *s = w;
  begin_call("vpcm_delete", G_CREATE);
  g_xfc[0] = ZV34(s)->ctx_;
  PRE();
  h_emit(g_show, "call %s w=%s lvl=%u", g_what, T_(w), dsplibs_debug_level);
  ri = g_deletefn(w);
  mid_call();
  rm = m_delete(w);
  if (ri != rm || ri != 0) h_die3("%s: the model returns %d, the impl %d", g_what, rm, ri);
  end_call();
  h_emit(g_show, "ret %d", ri);
  h_emit(g_show, "pair after %08x %08x", ((unsigned *)g_pair)[0], (unsigned)PW1(g_pair));
}
static int pick_n(void) {
  switch (h_rmod(10)) { case 0: return 0; case 1: return 1; case 2: return 3; case 3: return 4; case 4: return 5; case 5: return 47; case 6: return 48; default: return (int)h_rmod(49); }
}
static void fill_in(int n) {
  int k;
  (void)n;
  for (k = 0; k < INSZ; k++) ((short *)g_in)[k] = (short)(h_rpct(8) ? (h_rpct(50) ? -32768 : 32767) : h_rpct(10) ? 0 : (int)(h_rnd() >> 16));
  for (k = 0; k < OUTSZ; k++) ((short *)g_out)[k] = (short)(0x5a5a ^ (int)h_rnd());
}
static int do_run(void *w, int n) {
  int ri, rm;
  fill_in(n);
  begin_call("vpcm_run", G_RUN);
  PRE();
  h_emit(g_show, "call %s n=%d lvl=%u", g_what, n, dsplibs_debug_level);
  ri = g_runfn(w, (short *)g_in, (short *)g_out, n);
  mid_call();
  rm = m_run(w, (short *)g_in, (short *)g_out, n);
  if (ri != rm) h_die3("%s: the model returns %d, the impl %d", g_what, rm, ri);
  end_call();
  h_emit(g_show, "ret %d", ri);
  { uint32_t h = fnv_upd(2166136261u, g_out, (size_t)(2 * n)); h_emit(g_show, "out %08x", h); }
  dump_world(g_show);
  return ri;
}

/* ---- the trials ---- */
static unsigned pick_lv(void) { return h_rpct(65) ? 2u + h_rmod(2) : pick_level(); }
static int pick_side(void) { switch (h_rmod(8)) { case 0: return 0; case 1: return 1; case 2: return -1; case 3: return 0x100; case 4: return (int)0x80000000u; case 5: return 7; default: return h_rpct(50) ? 0 : 1; } }
static int pick_xmode(void) { static const int m[] = { -1, 0, 1, 2, 3, 4, 5, 0x7fffffff, (int)0x80000000u, 1, 2, 2, 3, 0 }; return h_rpct(90) ? m[h_rmod(sizeof m / sizeof m[0])] : (int)h_rnd(); }
static unsigned pick_xrate(void) {
  static const unsigned r[] = { 0, 1, 2, 9, 10, 100, 480, 500, 8000, 9600, 9601, 11025, 48000, 223696213u, 223696214u, 268435455u, 268435456u, 268435457u, 0x7fffffffu, 0x80000000u, 0xffffffffu, 96000u, 1000000u };
  return h_rpct(70) ? r[h_rmod(sizeof r / sizeof r[0])] : h_rpct(50) ? h_rnd() >> h_rmod(32) : h_rnd();
}
static void trial_xf(void) {
  int k, nk = (int)rnd_range(1, 4);
  void *o;
  new_world();
  gen_script();
  dsplibs_debug_level = pick_lv();
  g_pokemask = 2u;
  g_failpct_v34 = g_failpct_k56 = 0;
  if (h_rpct(8)) { do_xfdelete(NULL); }
  for (k = 0; k < nk; k++) {
    if (k) { gen_script(); if (h_rpct(30)) dsplibs_debug_level = pick_lv(); }
    if (h_rpct(4)) h_fail_next_malloc = 1;   /* the blob's real constructor would crash on NULL; with the recorder the failure branch is reachable */
    o = do_xfcreate(pick_side(), pick_xmode(), pick_xrate());
    if (o) {
      if (h_rpct(60)) { gen_script(); do_xfterm(o); }
      gen_script();
      do_xfdelete(o);
    }
  }
  h_trial_end("z01");
}
static int pick_id(void) { switch (h_rmod(10)) { case 0: return 0; case 1: return 91; case 2: return -1; case 3: return 0x5b; case 4: return 0x5d; case 5: case 6: return 0x5c; case 7: return 0x5a; default: return 0x22; } }
static int pick_frag(void) { if (h_rpct(8)) { static const int n[] = { -4799, -4800, -9600, -9599, -10000, -100000, -1000000, -48, -49, -4801 }; return n[h_rmod(10)]; } switch (h_rmod(10)) { case 0: return 0; case 1: return 0x30; case 2: return 0x31; case 3: return -1; case 4: return (int)0x80000000u; case 5: return 100; case 6: return 1; default: return (int)h_rmod(0x31); } }
static void trial_create(void) {
  int k, nk = (int)rnd_range(1, 3);
  new_world();
  get_ops();
  gen_script();
  dsplibs_debug_level = pick_lv();
  g_pokemask = (1u << 1) | (1u << 2) | (1u << 4);
  for (k = 0; k < nk; k++) {
    void *w;
    int rate = h_rpct(85) ? 0x2580 : h_rpct(50) ? 0 : h_rpct(50) ? 0x2581 : h_rpct(50) ? 8000 : -1;
    if (k) { gen_script(); fill_dp((V34ModParObj *)g_dp); if (h_rpct(30)) dsplibs_debug_level = pick_lv(); }
    g_failpct_v34 = h_rpct(60) ? 0u : h_rpct(50) ? 40u : 100u;
    g_failpct_k56 = h_rpct(70) ? 0u : h_rpct(50) ? 40u : 100u;
    if (h_rpct(6)) h_fail_next_malloc = 1;
    else if (h_rpct(6)) h_fail_malloc_n = 2;   /* the VPcmFloModem: the blob's real constructor would crash on NULL, the recorder lets the failure branch run */
    w = do_create(pick_id(), h_rpct(40) ? 0 : h_rpct(50) ? 1 : h_rpct(50) ? -1 : (int)h_rnd(), rate, pick_frag(), h_rpct(80) ? g_op : g_modem);
    if (w) { gen_script(); if (h_rpct(30)) fill_dp((V34ModParObj *)g_dp); do_delete(w); }
  }
  h_trial_end("z01");
}
static void trial_run(void) {
  int k, nk = (int)rnd_range(2, 9);
  void *w;
  new_world();
  get_ops();
  gen_script();
  dsplibs_debug_level = pick_lv();
  g_pokemask = (1u << 1) | (1u << 2) | (1u << 3) | (1u << 4);
  g_failpct_v34 = g_failpct_k56 = 0;
  w = do_create(h_rpct(60) ? (h_rpct(50) ? 0x5c : 0x5a) : pick_id(), h_rpct(50) ? 0 : 1, 0x2580, (int)h_rmod(0x31), g_op);
  if (!w) h_die3("trial_run: vpcm_create failed");
  for (k = 0; k < nk; k++) {
    gen_script();
    if (h_rpct(60)) {   /* steer the state: (almost) no silence left, link / status / counters around their thresholds */
      ZI(w, silence) = h_rpct(70) ? 0 : (int)h_rmod(60);
      if (h_rpct(30)) ZI(w, link) = (int)h_rmod(3) - 1;
      if (h_rpct(30)) ZI(w, status) = (int)h_rmod(18) - 1;
      if (h_rpct(25)) ZI(w, trainCnt) = 0xbb7 + (int)h_rmod(2);
      if (h_rpct(30)) ZI(w, bitsWanted) = h_rpct(30) ? 0 : (int)h_rmod(0x401);
      if (h_rpct(25)) ZI(w, addDelay) = h_rpct(50) ? 0 : (int)(0x180 + h_rmod(400));
      if (h_rpct(25)) ((V34ModParObj *)g_dp)->w6c_ = h_rpct(50) ? 0 : h_rnd() & 0xff;
      if (h_rpct(10)) ZI(w, id) = pick_id();
    }
    if (h_rpct(15)) dsplibs_debug_level = pick_lv();
    do_run(w, pick_n());
  }
  gen_script();
  do_delete(w);
  h_trial_end("z01");
}
static void m_init(void) {
  env_dpreg(1, 0, 0x22, g_op); env_dpreg(1, 0, 0x5a, g_op); env_dpreg(1, 0, 0x5c, g_op);
}
static void m_exit(void) {
  env_dpreg(1, 1, 0x22, g_op); env_dpreg(1, 1, 0x5a, g_op); env_dpreg(1, 1, 0x5c, g_op);
}
static void do_dpget(void) {
  long ri, rm;
  begin_call("dp_param_get", G_REG);
  PRE();
  h_emit(g_show, "call %s m=%s lvl=%u", g_what, T_(g_modem), dsplibs_debug_level);
  ri = f_dpget(g_modem);
  mid_call();
  rm = env_gp(1, g_modem, 0xa);
  if (ri != rm || (void *)(intptr_t)ri != g_dp) h_die3("%s: the model returns %ld, the impl %ld", g_what, rm, ri);
  CV(G_REG, "reg: dp_param_get");
  end_call();
  h_emit(g_show, "ret %s", T_((void *)(intptr_t)ri));
}
static void trial_reg(void) {
  int k, nk = (int)rnd_range(1, 4);
  new_world();
  gen_script();
  dsplibs_debug_level = pick_lv();
  g_pokemask = 2u;
  for (k = 0; k < nk; k++) {
    int ri, init = h_rpct(60);
    gen_script();
    if (h_rpct(30)) do_dpget();
    begin_call(init ? "dp_vpcm_init" : "dp_vpcm_exit", G_REG);
    PRE();
    h_emit(g_show, "call %s lvl=%u", g_what, dsplibs_debug_level);
    if (init) ri = f_init(); else { f_exit(); ri = 0; }
    mid_call();
    if (init) m_init(); else m_exit();
    if (init) CV(G_REG, "reg: init"); else CV(G_REG, "reg: exit");
    if (ri != 0) h_die3("%s returned %d", g_what, ri);
    end_call();
    h_emit(g_show, "ret %d", ri);
    if (g_op) { Ops *op = (Ops *)g_op; h_emit(g_show, "ops name=%s use=%d create=%d delete=%d process=%d hangup=%d", op->name, op->use, op->create != NULL, op->del != NULL, op->process != NULL, op->hangup != NULL); }
  }
  h_trial_end("z01");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, bad = 0, ntr;
  const char *seed = getenv("Z01_SEED");
  CovSite *cv;
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("Z01_ALL")) g_showall = 1;
  if (sc < 0 || sc > 3) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  p47_layout_check(); v01_layout_check();
  if (ZSIZE != 0xd258u && sizeof(void *) == 4) h_die3("state size %u", ZSIZE);
  h_x87_check();
  g_snapfn = snapshot;
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("Z01_TRIALS") ? atoi(getenv("Z01_TRIALS")) : (sc == G_XF ? 6000 : sc == G_CREATE ? 8000 : sc == G_RUN ? 3000 : 200);
  for (tr = 0; tr < ntr; tr++) {
    switch (sc) { case G_XF: trial_xf(); break; case G_CREATE: trial_create(); break; case G_RUN: trial_run(); break; default: trial_reg(); break; }
  }
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  for (cv = __start_covsites; cv < __stop_covsites; cv++) {
    if (cv->grp != sc) continue;
    h_emit(1, "COV %s %lu", cv->name, cv->n);
    if (cv->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", cv->name); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("Z01_NOCOV")) ? 2 : 0;
}
