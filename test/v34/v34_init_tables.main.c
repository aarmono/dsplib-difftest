/*
 * Differential test harness for V.34 "tier 3" group C (src/v34/v34_setup.c):
 *   initV34, V34SetupDemodulator, V34SetupModulator, v34setuptxmit
 * All four are GLOBAL in dsplibs.o (no address thunks needed).
 *
 * One source, two binaries (same convention as test/v34/v34_init.main.c):
 *   v34_tier3c_test_blob - linked against the real dsplibs.o (-DTEST_BLOB)
 *   v34_tier3c_test_port - linked against src/v34/v34_setup.c + src/v34/v34_init.c
 *                          + src/v34/v34_agc_demod.c + src/v34/v34_dsp_primitives.c + tables,
 *                          plus (INTERIM, group D not ported) a copy of the real
 *                          blob object in which every symbol except `settxlevel`
 *                          is local (objcopy --keep-global-symbol=settxlevel).
 * Each scenario runs in its OWN process (argv[1]).
 *
 * Objects: tagV34Object is not a struct. The harness keeps a random "template"
 * fake object of OBJSZ bytes; every call runs on a fresh copy of the template
 * and the result is the hash of the DIFF (word index + new value of every word
 * that differs from the template). Equal diffs from equal templates <=> equal
 * final objects, so stray writes anywhere are caught while sweeps stay cheap.
 * Pointer slots the functions write/read are hashed NORMALISED: in-object
 * pointers -> offsets, table addresses -> (table id, offset) via a registry,
 * external buffers -> (region id, offset). Raw addresses are never compared
 * across the two processes.
 *
 * Table registry: the 24 LOCAL modulator tables have no linkable name in the
 * blob, so their addresses in the blob process are derived from the GLOBAL
 * anchor `hsine1200` (.rodata 0x30a0) plus each table's ELF .rodata offset
 * (dsplibs.o is one contiguous .text/.rodata piece). At start-up EVERY
 * registered blob table (global and derived) is memcmp'ed against the port's
 * extracted copy (linked into the blob binary under a `port_` prefix via
 * objcopy --prefix-symbols), which proves the anchor arithmetic, the offsets
 * AND the extraction in one go. The process aborts (exit 3) if any differs.
 *
 * Every scenario keeps COVERAGE COUNTERS derived from the INPUTS (or from
 * tiny independent predicates) and exits 2 if a branch class was never hit.
 * Env V34C_VERBOSE=1 prints a line per call (debugging a divergence).
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v34_img.h" /* T04: x86-64 image <-> native V34Obj marshalling; empty at -m32 */

extern unsigned int dsplibs_debug_level;

/* Prototypes use int-width parameters on purpose: the harness passes garbage
 * in the upper 16 bits of every 16-bit argument (callee must ignore it). */
#if defined(__x86_64__)
#define V34P6(p) ((void *)(uintptr_t)(p)) /* T06: p6 is an ADDRESS stored in a pointer slot (initdigital passes obj+0x2a68): a real pointer parameter on x86-64 */
#define V34P6T void *
#else
#define V34P6(p) (p)
#define V34P6T uint32_t
#endif
extern int initV34(void *blk, unsigned int p2, unsigned int p3, int p4, int p5, V34P6T p6, unsigned int p7);
extern void V34SetupDemodulator(void *obj, int baud, int carrier);
extern int *V34SetupModulator(void *m, int baud, int carrier, int preemp, int v90, int fullReset);
extern void v34setuptxmit(void *obj);
/* already-verified (tier 2) helper used only to build a valid fake echo canceller */
extern void V34InitializeImplementationSpecific(void *obj);

/* ---- tables ---- */
#define GLOBAL_TBLS(X)                                                                                       \
  X(Convolve64, 128) X(Convolve32, 128) X(Convolve16, 128) X(hsine1200, 32) X(hsine1600, 24)                 \
  X(hsine1680, 160) X(hsine1800, 64) X(hsine1829, 84) X(hsine1867, 144) X(hsine1920, 20)                     \
  X(hsine1959, 196) X(hsine2000, 96) X(hsine2400, 32)

#define LOCAL_TBLS(X)                                                                                        \
  X(preemp0, 0x5300, 32) X(tx600c1, 0x3640, 256) X(txAllPass, 0x3740, 256) X(p2400, 0x3840, 320)            \
  X(ec_prem_coef_B2400High, 0x3980, 84) X(ec_prem_coef_B2400, 0x39e0, 84) X(tx2400c1, 0x3a40, 256)           \
  X(p2800, 0x3b40, 320) X(ec_prem_coef_B2800High, 0x3c80, 84) X(ec_prem_coef_B2800, 0x3ce0, 84)              \
  X(tx2800c1, 0x3d40, 1536) X(p3000, 0x4340, 320) X(ec_prem_coef_B3000High, 0x4480, 84)                      \
  X(ec_prem_coef_B3000, 0x44e0, 84) X(tx3000c1, 0x4540, 1024) X(p3200, 0x4940, 320)                          \
  X(ec_prem_coef_B3200High, 0x4a80, 84) X(ec_prem_coef_B3200, 0x4ae0, 84)                                    \
  X(tx3200c1_for_v34, 0x4b40, 192) X(V90EchoPrefilterCoeff, 0x4c00, 84) X(tx3200c1_for_v90, 0x4c60, 384)     \
  X(p3429, 0x4de0, 320) X(ec_prem_coef_B3429, 0x4f20, 84) X(tx3429c1, 0x4f80, 896)

#define X_DECL_G(n, sz) extern const short n[];
GLOBAL_TBLS(X_DECL_G)
#undef X_DECL_G
#ifdef TEST_BLOB
/* the port's extracted copies, prefixed by objcopy, for the content check */
#define X_DECL_PG(n, sz) extern const short port_##n[];
#define X_DECL_PL(n, off, sz) extern const short port_##n[];
GLOBAL_TBLS(X_DECL_PG)
LOCAL_TBLS(X_DECL_PL)
#undef X_DECL_PG
#undef X_DECL_PL
#else
#define X_DECL_L(n, off, sz) extern const short n[];
LOCAL_TBLS(X_DECL_L)
#undef X_DECL_L
#endif

typedef struct {
  const char *name;
  const unsigned char *addr;
  size_t size;
} tbl_t;
static tbl_t g_tbl[64];
static int g_ntbl;
static const unsigned char *g_preemp0;
static int g_tables_verified;

static void add_tbl(const char *name, const void *addr, size_t size) {
  g_tbl[g_ntbl].name = name;
  g_tbl[g_ntbl].addr = (const unsigned char *)addr;
  g_tbl[g_ntbl].size = size;
  g_ntbl++;
}

static const unsigned char *tbl_addr(const char *name) {
  int i;
  for (i = 0; i < g_ntbl; i++)
    if (!strcmp(g_tbl[i].name, name)) return g_tbl[i].addr;
  fprintf(stderr, "harness: unknown table %s\n", name);
  exit(3);
}

static void tables_init(void) {
#ifdef TEST_BLOB
  const unsigned char *anchor = (const unsigned char *)hsine1200 - 0x30a0; /* dsplibs.o .rodata base */
#define X_REG_G(n, sz)                                                                                       \
  add_tbl(#n, (const void *)n, sz);                                                                          \
  if (memcmp((const void *)n, (const void *)port_##n, sz)) { fprintf(stderr, "table %s: blob != port copy\n", #n); exit(3); }
#define X_REG_L(n, off, sz)                                                                                  \
  add_tbl(#n, anchor + (off), sz);                                                                           \
  if (memcmp(anchor + (off), (const void *)port_##n, sz)) { fprintf(stderr, "table %s: blob@rodata+0x%x != port copy\n", #n, off); exit(3); }
#else
#define X_REG_G(n, sz) add_tbl(#n, (const void *)n, sz);
#define X_REG_L(n, off, sz) add_tbl(#n, (const void *)n, sz);
#endif
  GLOBAL_TBLS(X_REG_G)
  LOCAL_TBLS(X_REG_L)
#undef X_REG_G
#undef X_REG_L
  g_preemp0 = tbl_addr("preemp0");
  g_tables_verified = g_ntbl;
}

/* ---- accessors ---- */
/* V34SHIMS BEGIN (generated by tools/tier_rewrite.py shims; do not edit) */
#if defined(__x86_64__)
static int V34SH_initV34(void *blk, unsigned int p2, unsigned int p3, int p4, int p5, V34P6T p6, unsigned int p7) { int r; v34i_enter(); r = initV34((void *)v34i_in((void *)blk), p2, p3, p4, p5, p6, p7); v34i_leave(); return r;}
static void V34SH_V34SetupDemodulator(void *obj, int baud, int carrier) { v34i_enter(); V34SetupDemodulator((void *)v34i_in((void *)obj), baud, carrier); v34i_leave(); }
static int * V34SH_V34SetupModulator(void *m, int baud, int carrier, int preemp, int v90, int fullReset) { int * r; v34i_enter(); r = V34SetupModulator((void *)v34i_in((void *)m), baud, carrier, preemp, v90, fullReset); v34i_leave(); return (int *)v34i_outp((void *)r);}
static void V34SH_v34setuptxmit(void *obj) { v34i_enter(); v34setuptxmit((void *)v34i_in((void *)obj)); v34i_leave(); }
static void V34SH_V34InitializeImplementationSpecific(void *obj) { v34i_enter(); V34InitializeImplementationSpecific((void *)v34i_in((void *)obj)); v34i_leave(); }
static void *const V34REAL_initV34 = (void *)initV34; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_V34SetupDemodulator = (void *)V34SetupDemodulator; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_V34SetupModulator = (void *)V34SetupModulator; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_v34setuptxmit = (void *)v34setuptxmit; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_V34InitializeImplementationSpecific = (void *)V34InitializeImplementationSpecific; /* the real address (the name is a macro for the shim below) */
#define initV34 V34SH_initV34
#define V34SetupDemodulator V34SH_V34SetupDemodulator
#define V34SetupModulator V34SH_V34SetupModulator
#define v34setuptxmit V34SH_v34setuptxmit
#define V34InitializeImplementationSpecific V34SH_V34InitializeImplementationSpecific
#endif
/* V34SHIMS END */
#define OB(o, off) ((unsigned char *)(o) + (off))
#define S16(o, off) (*(int16_t *)OB(o, off))
#define U16(o, off) (*(uint16_t *)OB(o, off))
#define I32(o, off) (*(int32_t *)OB(o, off))
#define U32(o, off) (*(uint32_t *)OB(o, off))
#if defined(__x86_64__)
#define PTR(o, off) (*(uint32_t *)OB(o, off)) /* x86-64: the image keeps a 32-bit slot (low memory address), marshalled to a real pointer (test/v34/v34_img.h) */
#else
#define PTR(o, off) (*(void **)OB(o, off))
#endif
#define PTRX(o, off) (*(void **)OB(o, off)) /* a pointer slot of the fake ctx (raw 32-bit offset, pointer-sized: the port reads it with PTR(ctx, off)) */
#if defined(__x86_64__) /* T05: the ctx is an IMAGE too (i386 layout, marshalled by test/v34/v34_img.h): its pointer slots are 32-bit words */
#define PTRX_SET(o, off, v) (*(uint32_t *)OB(o, off) = (uint32_t)(uintptr_t)(v))
#else
#define PTRX_SET(o, off, v) (PTRX(o, off) = (v))
#endif
#define OBJSZ 0xb400u
#define CTXSZ 0x6200u
#define CFGSZ 0x600u
#define MOFF 0x1450u
#define BANKSZ 4096u

/* ---- deterministic LCG ---- */
static unsigned int g_lcg = 0x5ca1ab1eu;
/* LCG state step + a bijective 32-bit finalizer, so EVERY output bit (incl. the low byte,
 * which is a period-256 sequence in a raw LCG) is well distributed. Still fully
 * deterministic; no rand(). */
static unsigned int lcg_next(void) {
  unsigned int x;
  g_lcg = g_lcg * 1103515245u + 12345u;
  x = g_lcg;
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}
static short lcg_short(void) { return (short)(lcg_next() >> 16); }
static unsigned int lcg_mod(unsigned int m) { return (lcg_next() >> 8) % m; }
static int lcg_range(int lo, int hi) { return lo + (int)lcg_mod((unsigned int)(hi - lo + 1)); }

static unsigned int fnv(unsigned int h, const void *p, size_t n) {
  const unsigned char *b = (const unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) {
    h ^= b[i];
    h *= 16777619u;
  }
  return h;
}
#define FNV0 2166136261u

static void fill_random(void *p, size_t n) {
  unsigned char *b = (unsigned char *)p;
  size_t i;
  for (i = 0; i + 4 <= n; i += 4) {
    uint32_t v = lcg_next();
    memcpy(b + i, &v, 4);
  }
  for (; i < n; i++) b[i] = (unsigned char)(lcg_next() >> 16);
}

static int g_fail = 0;
static int g_verbose = 0;
static void need(const char *name, long v) {
  printf("  coverage %s = %ld\n", name, v);
  if (v <= 0) {
    fprintf(stderr, "COVERAGE FAILURE: %s never reached\n", name);
    g_fail = 1;
  }
}

/* ---- pointer normalisation ---- */
typedef struct {
  const unsigned char *base;
  size_t size;
} ext_t;
static ext_t g_ext[8];
static int g_next;
static void ext_clear(void) { g_next = 0; }
static void ext_reg(const void *p, size_t sz) {
  g_ext[g_next].base = (const unsigned char *)p;
  g_ext[g_next].size = sz;
  g_next++;
}

static const unsigned char *g_ptbl; /* expected pNNNN table for the current call (or NULL) */
static int g_pre_nz;                /* current call's preemp != 0 (then slot m+0xcb0 = table-relative, else &preemp0) */

static uint32_t norm_val(const unsigned char *obj, uint32_t v) {
  int i;
  uintptr_t a = (uintptr_t)v;
  if (obj && a >= (uintptr_t)obj && a < (uintptr_t)obj + OBJSZ) return 0x80000000u | (uint32_t)(a - (uintptr_t)obj);
  for (i = 0; i < g_next; i++)
    if (a >= (uintptr_t)g_ext[i].base && a < (uintptr_t)g_ext[i].base + g_ext[i].size)
      return 0x7d000000u | ((uint32_t)i << 16) | (uint32_t)(a - (uintptr_t)g_ext[i].base);
  for (i = 0; i < g_ntbl; i++)
    if (a >= (uintptr_t)g_tbl[i].addr && a < (uintptr_t)g_tbl[i].addr + g_tbl[i].size)
      return 0x7e000000u | ((uint32_t)i << 16) | (uint32_t)(a - (uintptr_t)g_tbl[i].addr);
  return v;
}

/* offsets (in the fake object) of every 4-byte slot that may hold a pointer */
static const unsigned int k_slots[] = {
    0x268,  0x26c,  0x394,  0x2220, 0x2224, 0x2a28, 0xe48,  0x2608, 0xa28,  0x80b8, 0x80bc, 0x80c0, 0x80c4,
    0x80c8, 0x9138, 0x913c, 0x9140, 0x9144, 0x9148, 0x418,  0x1460, 0x2074, 0x20cc, 0x2100, 0x3548, 0xac3c};
#define NSLOTS (sizeof k_slots / sizeof k_slots[0])
#define SLOT_CB0 0x2100u /* m+0xcb0: tx pre-emphasis pointer */

static unsigned char g_isslot[OBJSZ / 4];
static void slots_init(void) {
  size_t i;
  for (i = 0; i < NSLOTS; i++) g_isslot[k_slots[i] / 4] = 1;
}

static uint32_t norm_slot(const unsigned char *obj, unsigned int off, uint32_t v) {
  if (off == SLOT_CB0) {
    /* NB: with preemp != 0 the pointer may be one row past the end of pNNNN, which in the port's
     * (linker-placed) layout can equal &preemp0 - so decide by the expectation, not by the value */
    if (g_pre_nz && g_ptbl) return 0x6c000000u | ((uint32_t)((uintptr_t)v - (uintptr_t)g_ptbl) & 0x00ffffffu);
    if ((uintptr_t)v == (uintptr_t)g_preemp0) return 0x6b000000u;
  }
  return norm_val(obj, v);
}

static void scrub_slots(unsigned char *o) {
  size_t i;
  for (i = 0; i < NSLOTS; i++) U32(o, k_slots[i]) = 0x0d000000u | (lcg_next() & 0xffffffu);
}

/* hash of the diff of a live object against its (pre-call) snapshot */
static unsigned int diff_obj(const unsigned char *obj, const unsigned char *snap) {
  unsigned int h = FNV0;
  size_t i;
  for (i = 0; i < OBJSZ / 4; i++) {
    uint32_t a, b;
    memcpy(&a, obj + 4 * i, 4);
    memcpy(&b, snap + 4 * i, 4);
    if (g_isslot[i]) {
      a = norm_slot(obj, (unsigned int)(4 * i), a);
      b = norm_slot(obj, (unsigned int)(4 * i), b);
    }
    if (a != b) {
      uint32_t idx = (uint32_t)i;
      h = fnv(h, &idx, 4);
      h = fnv(h, &a, 4);
      if (g_verbose > 1) printf("   diffword off=%05x new=%08x\n", (unsigned int)(4 * i), a);
    }
  }
  return h;
}

/* plain buffer diff (external buffers); `slots` = byte offsets of normalised pointer words */
static unsigned int diff_buf(const unsigned char *buf, const unsigned char *snap, size_t sz, const unsigned char *obj,
                             const unsigned int *slots, int nslots) {
  unsigned int h = FNV0;
  size_t i;
  for (i = 0; i + 4 <= sz; i += 4) {
    uint32_t a, b;
    int s, isslot = 0;
    memcpy(&a, buf + i, 4);
    memcpy(&b, snap + i, 4);
    for (s = 0; s < nslots; s++)
      if (slots[s] == i) isslot = 1;
    if (isslot) {
      a = norm_val(obj, a);
      b = norm_val(obj, b);
    }
    if (a != b) {
      uint32_t idx = (uint32_t)i;
      h = fnv(h, &idx, 4);
      h = fnv(h, &a, 4);
    }
  }
  return h;
}

/* ---- per-scenario rolling log ---- */
typedef struct {
  const char *name;
  unsigned int all;
  long calls;
  long every;
} logger_t;
static void log_call2(logger_t *l, unsigned int h, long a, long b, long c) {
  l->all = fnv(l->all, &h, 4);
  if (g_verbose || l->calls % l->every == 0) printf("%s c=%ld a=%ld b=%ld c2=%ld hash=%08x\n", l->name, l->calls, L32(a), L32(b), L32(c), h);
  l->calls++;
}
static void log_done(logger_t *l) { printf("%s calls=%ld all=%08x\n", l->name, l->calls, l->all); }

static unsigned char *xalloc(size_t n) {
  unsigned char *p = (unsigned char *)calloc(1, n);
  if (!p) exit(4);
  if (n == OBJSZ) V34I_REG(p, n);
  else if (n == CTXSZ) V34I_REGC(p, n, ctx);
  else if (n == CFGSZ) V34I_REGC(p, n, cfg);
  return p;
}

/* ===================== scenario 0: initV34 ===================== */

#define BLKSZ 0x480u

static unsigned int junk_hi(void) { return lcg_next() & 0xffff0000u; }

static void slotnorm_blk(uint32_t *a, uint32_t *b) {
  *a = norm_val(NULL, *a);
  *b = norm_val(NULL, *b);
}

static unsigned int diff_blk(const unsigned char *blk, const unsigned char *snap) {
  unsigned int h = FNV0;
  size_t i;
  for (i = 0; i < BLKSZ / 4; i++) {
    uint32_t a, b;
    memcpy(&a, blk + 4 * i, 4);
    memcpy(&b, snap + 4 * i, 4);
    if (i == 0x28 / 4) slotnorm_blk(&a, &b);
    if (a != b) {
      uint32_t idx = (uint32_t)i;
      h = fnv(h, &idx, 4);
      h = fnv(h, &a, 4);
    }
  }
  return h;
}

static struct {
  long n_calls;
  long nclass[4];       /* N = 0xc, 0xe, 0x10, 0xf */
  long k8, k7;
  long p5zero, p5one, p5other, p5neg;
  long p4zero, p4nz;
  long e_loop, e_direct, rows_le_c, rows_le37, rows_gt37;
  long seen_e[2][32];
  long seen_m[2][20];
  long seen_out44[20];
  long cmax;
  long p7_le80, p7_gt80;
  long copy_nonzero;
} g_i0;

static void run_initV34(logger_t *lg, unsigned int p2, unsigned int p3, unsigned int p4, unsigned int p5, uint32_t p6, unsigned int p7) {
  unsigned char tmpl[BLKSZ], blk[BLKSZ];
  unsigned int h;
  int rc;
  fill_random(tmpl, sizeof tmpl);
  U32(tmpl, 0x28) = 0x0d000000u | (lcg_next() & 0xffffffu);
  V34I_REGW(blk, BLKSZ, 0xa00); /* x86-64: the standalone block B is a window of the object (offset 0xa00) */
  memcpy(blk, tmpl, sizeof blk);
  {
    unsigned int a2 = (p2 & 0xffffu) | junk_hi();
    unsigned int a3 = (p3 & 0xffffu) | junk_hi();
    unsigned int a4 = (p4 & 0xffffu) | junk_hi();
    unsigned int a5 = (p5 & 0xffffu) | junk_hi();
    unsigned int a7 = (p7 & 0xffffu) | junk_hi();
    rc = initV34(blk, a2, a3, (int)a4, (int)a5, V34P6(p6), a7);
  }
  h = diff_blk(blk, tmpl);
  V34I_UNREG(blk);
  h = fnv(h, &rc, 4);
  log_call2(lg, h, (long)(p2 & 0xffff), (long)(p3 & 0xffff), (long)U16(blk, 0x12));

  /* coverage from the INPUTS (independent predicates) and observable outputs */
  {
    unsigned int q2 = p2 & 0xffff, q7 = p7 & 0xffff;
    int p4z = (p4 & 0xffff) == 0, p5v = (int16_t)(p5 & 0xffff);
    int cls;
    g_i0.n_calls++;
    if (q2 > 0xaf0) cls = (q2 == 0xc80) ? 2 : 3;
    else if (q2 == 0xaf0) cls = 1;
    else cls = 0;
    g_i0.nclass[cls]++;
    if (q2 == 0xab7 || q2 == 0xd65) g_i0.k8++; else g_i0.k7++;
    if (p5v == 0) g_i0.p5zero++; else if (p5v == 1) g_i0.p5one++; else g_i0.p5other++;
    if (p5v < 0) g_i0.p5neg++;
    if (p4z) g_i0.p4zero++; else g_i0.p4nz++;
    if (U16(blk, 0x14) > 0) g_i0.e_loop++; else g_i0.e_direct++;
    if (U16(blk, 0x4) <= 0xc) g_i0.rows_le_c++;
    if (U16(blk, 0x4) <= 0x37) g_i0.rows_le37++; else g_i0.rows_gt37++;
    if (U16(blk, 0xe) < 32) g_i0.seen_e[p4z ? 0 : 1][U16(blk, 0xe)]++;
    if (U16(blk, 0x12) < 20) g_i0.seen_m[p4z ? 0 : 1][U16(blk, 0x12)]++;
    if (U16(blk, 0x44) < 20) g_i0.seen_out44[U16(blk, 0x44)]++;
    if (q7 <= 0x80) g_i0.p7_le80++; else g_i0.p7_gt80++;
    if (U32(blk, 0x248) != U32(tmpl, 0x248)) g_i0.copy_nonzero++;
    if (U16(blk, 0x14) > g_i0.cmax) g_i0.cmax = U16(blk, 0x14);
  }
}

static void scen_initV34(void) {
  static const unsigned int p2set[] = {0xab7, 0xd65, 0x960, 0xaf0, 0xc80, 0xb00, 0xaef, 0xaf1, 0x0, 0xffff, 0x8000, 0xc7f, 0xc81, 0x1000};
  static const int p5set[] = {0, 1, 2, -1, 0x4000, -32768, 3};
  logger_t lg = {"initV34", FNV0, 0, 20011};
  unsigned int p2, p3, i, j;
  memset(&g_i0, 0, sizeof g_i0);

  /* A: every p3 (0..65535) x 14 interesting p2 values, p4/p5/p7 cycling */
  for (j = 0; j < sizeof p2set / sizeof p2set[0]; j++)
    for (p3 = 0; p3 < 65536; p3++) {
      unsigned int r4 = lcg_next(), r6 = lcg_next();
      run_initV34(&lg, p2set[j], p3, ((p3 >> 3) & 1) ? (r4 | 1u) : 0u, (unsigned int)p5set[(p3 + j) % 7], r6,
                  (unsigned int)(p3 * 13u + j));
    }
  /* B: every p2 (0..65535) with random p3/p4/p5/p6/p7 (x3) */
  for (i = 0; i < 3; i++)
    for (p2 = 0; p2 < 65536; p2++) {
      unsigned int r3 = lcg_next(), r4 = lcg_next(), r4b = lcg_next(), r5 = lcg_mod(7), r6 = lcg_next(), r7 = lcg_next();
      run_initV34(&lg, p2, r3, (r4 & 4) ? r4b : 0u, (unsigned int)p5set[r5], r6, r7);
    }
  /* C: every p7 (0..65535) on the four main baud classes */
  for (i = 0; i < 4; i++)
    for (p3 = 0; p3 < 65536; p3++) {
      unsigned int r3 = lcg_next() & 0xffffu, r4 = lcg_next() & 1u, r5 = lcg_mod(7), r6 = lcg_next();
      run_initV34(&lg, p2set[i], r3, r4, (unsigned int)p5set[r5], r6, p3);
    }
  /* D: small/edge p3 values explicitly, every p4 encoding variant */
  for (i = 0; i < 300; i++)
    for (j = 0; j < 5; j++)
      run_initV34(&lg, p2set[j], i, (j & 1) ? 0x10000u : 0u, (unsigned int)p5set[j], 0x80000000u | i, 0x80 + (i % 3));

  log_done(&lg);
  need("initV34 N=0xc (p2<0xaf0)", g_i0.nclass[0]);
  need("initV34 N=0xe (p2==0xaf0)", g_i0.nclass[1]);
  need("initV34 N=0x10 (p2==0xc80)", g_i0.nclass[2]);
  need("initV34 N=0xf (p2>0xaf0, !=0xc80)", g_i0.nclass[3]);
  need("initV34 K=8", g_i0.k8);
  need("initV34 K=7", g_i0.k7);
  need("initV34 p5==0", g_i0.p5zero);
  need("initV34 p5==1", g_i0.p5one);
  need("initV34 p5 other", g_i0.p5other);
  need("initV34 p5 negative", g_i0.p5neg);
  need("initV34 p4==0 (MMinTable)", g_i0.p4zero);
  need("initV34 p4!=0 (MMaxTable)", g_i0.p4nz);
  need("initV34 e via c-loop (blk+0x14>0)", g_i0.e_loop);
  need("initV34 e direct (blk+0x14==0)", g_i0.e_direct);
  need("initV34 rows<=0xc", g_i0.rows_le_c);
  need("initV34 rows<=0x37", g_i0.rows_le37);
  need("initV34 rows>0x37", g_i0.rows_gt37);
  need("initV34 p7<=0x80", g_i0.p7_le80);
  need("initV34 p7>0x80", g_i0.p7_gt80);
  need("initV34 xyz slice copied", g_i0.copy_nonzero);
  {
    int t, e, m;
    long miss_e = 0, miss_44 = 0;
    long seen_m_distinct[2] = {0, 0};
    for (t = 0; t < 2; t++) {
      for (e = 0; e < 32; e++)
        if (!g_i0.seen_e[t][e]) miss_e++;
      for (m = 1; m < 20; m++)
        if (g_i0.seen_m[t][m]) seen_m_distinct[t]++;
    }
    for (m = 8; m <= 17; m++)
      if (!g_i0.seen_out44[m]) miss_44++;
    printf("  e values not seen (of 64): %ld; p7 bit-count results 8..17 not seen: %ld; distinct m: min=%ld max=%ld; max c=%ld\n", miss_e, miss_44,
           seen_m_distinct[0], seen_m_distinct[1], g_i0.cmax);
    need("initV34 every e=0..31 for both MMin and MMax (64/64)", miss_e == 0);
    need("initV34 every p7 log2 result 8..17", miss_44 == 0);
    need("initV34 c-loop count >= 3", g_i0.cmax >= 3);
    need("initV34 distinct MMin m values >= 15", seen_m_distinct[0] >= 15);
    need("initV34 distinct MMax m values >= 17", seen_m_distinct[1] >= 17);
  }
}

/* ===================== fake-object plumbing shared by scenarios 1-4 ===================== */
static unsigned char *g_tmpl; /* OBJSZ */
static unsigned char *g_obj;  /* OBJSZ */

static void tmpl_refresh(void) {
  fill_random(g_tmpl, OBJSZ);
  scrub_slots(g_tmpl);
}

/* ===================== scenario 1: V34SetupDemodulator ===================== */
static struct {
  long baud[6], baud_other, car[8], car_other;
} g_d;

static void run_demod(logger_t *lg, int baud, int carrier) {
  unsigned int h;
  const unsigned char *b = OB(g_obj, 0x264);
  if (lg->calls % 128 == 0) tmpl_refresh();
  memcpy(g_obj, g_tmpl, OBJSZ);
  {
    unsigned int ab = (uint32_t)(uint16_t)baud | junk_hi();
    unsigned int ac = (uint32_t)(uint16_t)carrier | junk_hi();
    V34SetupDemodulator(g_obj, (int)ab, (int)ac);
  }
  h = diff_obj(g_obj, g_tmpl);
  log_call2(lg, h, baud, carrier, S16(b, 0x1ac));
  {
    int16_t bs = (int16_t)baud, cs = (int16_t)carrier;
    if (bs == 2400) g_d.baud[0]++; else if (bs == 2743) g_d.baud[1]++; else if (bs == 2800) g_d.baud[2]++;
    else if (bs == 3000) g_d.baud[3]++; else if (bs == 3200) g_d.baud[4]++; else if (bs == 3429) g_d.baud[5]++;
    else g_d.baud_other++;
    if (S16(g_obj, 0x264 + 0x128) != 4) { fprintf(stderr, "harness: demod did not write 4\n"); g_fail = 1; }
    switch (cs) {
      case 1600: g_d.car[0]++; break; case 1680: g_d.car[1]++; break; case 1800: g_d.car[2]++; break;
      case 1829: g_d.car[3]++; break; case 1867: g_d.car[4]++; break; case 1920: g_d.car[5]++; break;
      case 1959: g_d.car[6]++; break; case 2000: g_d.car[7]++; break; default: g_d.car_other++; break;
    }
  }
}

static void scen_demod(void) {
  static const int bset[] = {2400, 2743, 2800, 3000, 3200, 3429, 1, -1, 0, 4800, 32767, -32768};
  static const int cset[] = {1600, 1680, 1800, 1829, 1867, 1920, 1959, 2000, 1200, 2400, 0, -1, 1601, 32767, -32768};
  logger_t lg = {"V34SetupDemodulator", FNV0, 0, 5003};
  int i, j, v;
  g_tmpl = xalloc(OBJSZ);
  g_obj = xalloc(OBJSZ);
  memset(&g_d, 0, sizeof g_d);
  tmpl_refresh();
  /* grid */
  for (i = 0; i < (int)(sizeof bset / sizeof bset[0]); i++)
    for (j = 0; j < (int)(sizeof cset / sizeof cset[0]); j++) run_demod(&lg, bset[i], cset[j]);
  /* full baud sweep (every short) against cycling carriers */
  for (v = -32768; v < 32768; v++) run_demod(&lg, v, cset[(v + 32768) % (int)(sizeof cset / sizeof cset[0])]);
  /* full carrier sweep against cycling bauds */
  for (v = -32768; v < 32768; v++) run_demod(&lg, bset[(v + 32768) % (int)(sizeof bset / sizeof bset[0])], v);
  log_done(&lg);
  need("Demod baud 2400", g_d.baud[0]); need("Demod baud 2743", g_d.baud[1]); need("Demod baud 2800", g_d.baud[2]);
  need("Demod baud 3000", g_d.baud[3]); need("Demod baud 3200", g_d.baud[4]); need("Demod baud 3429", g_d.baud[5]);
  need("Demod unlisted baud", g_d.baud_other);
  need("Demod carrier 1600", g_d.car[0]); need("Demod carrier 1680", g_d.car[1]); need("Demod carrier 1800", g_d.car[2]);
  need("Demod carrier 1829", g_d.car[3]); need("Demod carrier 1867", g_d.car[4]); need("Demod carrier 1920", g_d.car[5]);
  need("Demod carrier 1959", g_d.car[6]); need("Demod carrier 2000", g_d.car[7]); need("Demod unlisted carrier", g_d.car_other);
}

/* ===================== scenarios 2/3: V34SetupModulator ===================== */
static unsigned char *g_bank_tmpl, *g_bank;

static struct {
  long baud[10]; /* 0:600 1:2400 2:2800 3:3000 4:3200v34 5:3200v90 6:3429 7:4800 8:invalid(other) 9:reserved */
  long inv_lt600, inv_gap_3000_3429, inv_gt4800, inv_low_gap;
  long sel_hi[4]; /* 2400/2800/3000/3200v34: carrier matched (non-High) */
  long sel_lo[4]; /* ... carrier not matched (High) */
  long pre_in[5], pre_out[5];
  long car[11], car_inv;
  long fr0, fr1, bank_in, bank_ext, bank_written, ret_q_neg, ret_q_pos, ret_m;
  long n64;
} g_m;

static const unsigned char *exp_ptable(int baud) {
  switch (baud) {
    case 2400: return tbl_addr("p2400");
    case 2800: return tbl_addr("p2800");
    case 3000: return tbl_addr("p3000");
    case 3200: return tbl_addr("p3200");
    case 3429: return tbl_addr("p3429");
    default: return NULL;
  }
}

/* one modulator call on a fresh copy of the template; returns diff hash */
static unsigned int mod_call(int baud, int carrier, int preemp, int v90, int fullReset, int bankext, int *retflag, long *retq) {
  unsigned char *m = OB(g_obj, MOFF);
  unsigned int h;
  int *ret;
  memcpy(g_obj, g_tmpl, OBJSZ);
  memcpy(g_bank, g_bank_tmpl, BANKSZ);
  ext_clear();
  ext_reg(g_bank, BANKSZ);
  PTR(g_obj, 0x2074) = bankext ? (void *)g_bank : (void *)OB(g_obj, MOFF + 0x1c);
  g_ptbl = exp_ptable(baud);
  g_pre_nz = (int16_t)preemp != 0;
  {
    unsigned int ab = (uint32_t)(uint16_t)baud | junk_hi();
    unsigned int ac = (uint32_t)(uint16_t)carrier | junk_hi();
    unsigned int ap = (uint32_t)(uint16_t)preemp | junk_hi();
    ret = V34SetupModulator(m, (int)ab, (int)ac, (int)ap, v90, fullReset);
  }
  h = diff_obj(g_obj, g_tmpl);
  {
    unsigned int hb = diff_buf(g_bank, g_bank_tmpl, BANKSZ, g_obj, NULL, 0);
    h = fnv(h, &hb, 4);
  }
  if (fullReset != 0) {
    unsigned int f = ((unsigned char *)ret == m) ? 0xA5A5A5A5u : (unsigned int)(uintptr_t)ret;
    h = fnv(h, &f, 4);
    *retflag = (unsigned char *)ret == m;
    *retq = 0;
  } else {
    unsigned int f = (unsigned int)(uintptr_t)ret;
    h = fnv(h, &f, 4);
    *retflag = 0;
    *retq = (long)(int)(intptr_t)ret;
  }
  g_ptbl = NULL;
  return h;
}

static void mod_cover(int baud, int carrier, int preemp, int v90, int fullReset, int bankext, int retflag, long retq) {
  int16_t bs = (int16_t)baud, cs = (int16_t)carrier, ps = (int16_t)preemp;
  int idx;
  switch (bs) {
    case 600: idx = 0; break; case 2400: idx = 1; break; case 2800: idx = 2; break; case 3000: idx = 3; break;
    case 3200: idx = v90 ? 5 : 4; break; case 3429: idx = 6; break; case 4800: idx = 7; break; default: idx = 8; break;
  }
  g_m.baud[idx]++;
  if (idx == 8) {
    if (bs < 600) g_m.inv_lt600++;
    else if (bs > 4800) g_m.inv_gt4800++;
    else if (bs > 3000 && bs < 3429) g_m.inv_gap_3000_3429++;
    else g_m.inv_low_gap++;
  }
  {
    int sel = -1, hit = 0;
    if (bs == 2400) { sel = 0; hit = cs == 0x640; }
    else if (bs == 2800) { sel = 1; hit = cs == 0x690; }
    else if (bs == 3000) { sel = 2; hit = cs == 0x708; }
    else if (bs == 3200 && !v90) { sel = 3; hit = cs == 0x725; }
    if (sel >= 0) { if (hit) g_m.sel_hi[sel]++; else g_m.sel_lo[sel]++; }
  }
  if (ps != 0) {
    int pt = (bs == 2400) ? 0 : (bs == 2800) ? 1 : (bs == 3000) ? 2 : (bs == 3200) ? 3 : (bs == 3429) ? 4 : -1;
    if (pt >= 0) { if (ps >= 1 && ps <= 10) g_m.pre_in[pt]++; else g_m.pre_out[pt]++; }
  }
  switch (cs) {
    case 1200: g_m.car[0]++; break; case 1600: g_m.car[1]++; break; case 1680: g_m.car[2]++; break; case 1800: g_m.car[3]++; break;
    case 1829: g_m.car[4]++; break; case 1867: g_m.car[5]++; break; case 1920: g_m.car[6]++; break; case 1959: g_m.car[7]++; break;
    case 2000: g_m.car[8]++; break; case 2400: g_m.car[9]++; break; default: g_m.car_inv++; break;
  }
  if (fullReset) g_m.fr1++; else g_m.fr0++;
  if (bankext) g_m.bank_ext++; else g_m.bank_in++;
  if (memcmp(g_bank, g_bank_tmpl, BANKSZ) != 0 || memcmp(OB(g_obj, MOFF + 0x1c), OB(g_tmpl, MOFF + 0x1c), 3072) != 0) g_m.bank_written++;
  if (fullReset) { if (retflag) g_m.ret_m++; }
  else { if (retq < 0) g_m.ret_q_neg++; else g_m.ret_q_pos++; }
  if ((bs == 3200 && v90)) g_m.n64++;
}

static void mod_setup(void) {
  g_tmpl = xalloc(OBJSZ);
  g_obj = xalloc(OBJSZ);
  g_bank_tmpl = xalloc(BANKSZ);
  g_bank = xalloc(BANKSZ);
  memset(&g_m, 0, sizeof g_m);
}

static void mod_refresh(void) {
  tmpl_refresh();
  fill_random(g_bank_tmpl, BANKSZ);
}

/* H01: with H01_POISON=<byte> in the environment, the stack below this frame is overwritten with that byte before every modulator call, so an
 * uninitialised local in the function under test (or a callee) changes the hash between two poison values.  Off by default (inert). */
static int g_h01_poison = -1;
static void __attribute__((noinline)) h01_poison_stack(void) {
  volatile unsigned char buf[65536];
  size_t k;
  for (k = 0; k < sizeof buf; k++) buf[k] = (unsigned char)g_h01_poison;
}

static void run_mod(logger_t *lg, int baud, int carrier, int preemp, int v90, int fullReset, int bankext) {
  int rf;
  long rq;
  unsigned int h;
  if (g_h01_poison >= 0) h01_poison_stack();
  if (lg->calls % 97 == 0) mod_refresh();
  h = mod_call(baud, carrier, preemp, v90, fullReset, bankext, &rf, &rq);
  mod_cover(baud, carrier, preemp, v90, fullReset, bankext, rf, rq);
  log_call2(lg, h, baud, carrier, preemp);
}

static void mod_report(void) {
  need("Mod baud 600 (silent default)", g_m.baud[0]);
  need("Mod baud 2400", g_m.baud[1]);
  need("Mod baud 2800", g_m.baud[2]);
  need("Mod baud 3000", g_m.baud[3]);
  need("Mod baud 3200 (v90==0)", g_m.baud[4]);
  need("Mod baud 3200 (v90!=0, N=0x40, no zero fill)", g_m.baud[5]);
  need("Mod baud 3429", g_m.baud[6]);
  need("Mod baud 4800", g_m.baud[7]);
  need("Mod invalid baud <600", g_m.inv_lt600);
  need("Mod invalid baud in (600,3000] other than listed", g_m.inv_low_gap);
  need("Mod invalid baud in (3000,3429)", g_m.inv_gap_3000_3429);
  need("Mod invalid baud >4800", g_m.inv_gt4800);
  need("Mod 2400 carrier==1600 (B2400)", g_m.sel_hi[0]); need("Mod 2400 carrier!=1600 (High)", g_m.sel_lo[0]);
  need("Mod 2800 carrier==1680 (B2800)", g_m.sel_hi[1]); need("Mod 2800 carrier!=1680 (High)", g_m.sel_lo[1]);
  need("Mod 3000 carrier==1800 (B3000)", g_m.sel_hi[2]); need("Mod 3000 carrier!=1800 (High)", g_m.sel_lo[2]);
  need("Mod 3200 v34 carrier==1829 (B3200)", g_m.sel_hi[3]); need("Mod 3200 v34 carrier!=1829 (High)", g_m.sel_lo[3]);
  {
    int t;
    static const char *pn[5] = {"p2400", "p2800", "p3000", "p3200", "p3429"};
    char nm[64];
    for (t = 0; t < 5; t++) {
      snprintf(nm, sizeof nm, "Mod preemp row 1..10 into %s", pn[t]);
      need(nm, g_m.pre_in[t]);
      snprintf(nm, sizeof nm, "Mod preemp out-of-range (no bounds check) into %s", pn[t]);
      need(nm, g_m.pre_out[t]);
    }
  }
  {
    static const char *cn[10] = {"1200", "1600", "1680", "1800", "1829", "1867", "1920", "1959", "2000", "2400"};
    char nm[64];
    int t;
    for (t = 0; t < 10; t++) {
      snprintf(nm, sizeof nm, "Mod carrier %s", cn[t]);
      need(nm, g_m.car[t]);
    }
  }
  need("Mod invalid carrier", g_m.car_inv);
  need("Mod fullReset==0", g_m.fr0);
  need("Mod fullReset!=0", g_m.fr1);
  need("Mod bank ptr in-object (m+0x1c)", g_m.bank_in);
  need("Mod bank ptr external buffer", g_m.bank_ext);
  need("Mod bank contents changed", g_m.bank_written);
  need("Mod return == m (fullReset)", g_m.ret_m);
  need("Mod return quotient negative", g_m.ret_q_neg);
  need("Mod return quotient non-negative", g_m.ret_q_pos);
}

static void scen_mod_grid(void) {
  static const int bset[] = {600, 2400, 2743, 2800, 3000, 3200, 3429, 4800, 0, 1, -1, 32767, -32768, 2401, 2399, 2799, 2801, 2999, 3001, 3199, 3201,
                             3428, 3430, 4799, 4801, 601, 599, 3100, 3300, 1000, 4000, 0x12c1};
  static const int cset[] = {1200, 1600, 1680, 1800, 1829, 1867, 1920, 1959, 2000, 2400, 0, 1, -1, 1199, 1201, 2401, 3200, 32767, -32768, 1828, 1830};
  static const int pset[] = {0, 1, 2, 5, 9, 10, 11, -1, -5, 100, -32768, 32767};
  static const int vset[] = {0, 1, -1, 0x10000};
  static const int fset[] = {0, 1, -1, 0x100};
  logger_t lg = {"V34SetupModulator-grid", FNV0, 0, 9973};
  unsigned int a, b, c, d, e;
  mod_setup();
  mod_refresh();
  for (a = 0; a < sizeof bset / sizeof bset[0]; a++)
    for (b = 0; b < sizeof cset / sizeof cset[0]; b++)
      for (c = 0; c < sizeof pset / sizeof pset[0]; c++)
        for (d = 0; d < 4; d++)
          for (e = 0; e < 4; e++) run_mod(&lg, bset[a], cset[b], pset[c], vset[d], fset[e], (int)((a + b + c + d + e) & 1));
  log_done(&lg);
  mod_report();
}

static void scen_mod_sweep(void) {
  static const int bset[] = {600, 2400, 2743, 2800, 3000, 3200, 3429, 4800, 0, 2500, 3100};
  static const int cset[] = {1200, 1600, 1680, 1800, 1829, 1867, 1920, 1959, 2000, 2400, 0, -1, 1601, 32767};
  logger_t lg = {"V34SetupModulator-sweep", FNV0, 0, 9973};
  int v, i;
  mod_setup();
  mod_refresh();
  /* every baud (short) against cycling carriers/preemp/v90/fullReset */
  for (v = -32768; v < 32768; v++) {
    int pr = lcg_range(-3, 13);
    unsigned int r1 = lcg_next(), r2 = lcg_next(), r3 = lcg_next(), r4 = lcg_next();
    run_mod(&lg, v, cset[(v + 32768) % 14], pr, (int)((r1 & 1u) ? r2 : 0u), (int)(r3 & 1u), (int)(r4 & 1u));
  }
  /* every carrier (short) against the valid bauds (both v90 states, both fullReset states) */
  for (v = -32768; v < 32768; v++) {
    int pr = lcg_range(-3, 13);
    unsigned int r1 = lcg_next(), r2 = lcg_next(), r3 = lcg_next();
    run_mod(&lg, bset[(v + 32768) % 11], v, pr, (int)(r1 & 1u), (int)(r2 & 1u), (int)(r3 & 1u));
  }
  /* every preemp (short) against the five preemp-bearing bauds */
  for (v = -32768; v < 32768; v++) {
    unsigned int ci = lcg_mod(10), r1 = lcg_next(), r2 = lcg_next(), r3 = lcg_next();
    run_mod(&lg, bset[1 + (v + 32768) % 5], cset[ci], v, (int)(r1 & 1u), (int)(r2 & 1u), (int)(r3 & 1u));
  }
  /* random everything */
  for (i = 0; i < 60000; i++) {
    short b = lcg_short(), c = lcg_short(), p = lcg_short();
    unsigned int r1 = lcg_next(), r2 = lcg_next(), r3 = lcg_next();
    run_mod(&lg, b, c, p, (int)r1, (int)r2, (int)(r3 & 1u));
  }
  log_done(&lg);
  mod_report();
}

/* ===================== scenario 4: v34setuptxmit ===================== */
static struct {
  long rx_eq, rx_ne, tx_eq, tx_ne, both_ne, v90_00, v90_10, v90_01, v90_11, v90_big, scale_changed, red_changed;
  long ptr_neg, ptr_pos, ctx6120_zero, ctx6120_nz, cfg4f8_zero, cfg4f8_nz, p54_4, p54_other, bauds_valid, bauds_invalid, txlevel_min_neg;
  long flag800_was_set, flag200_was_set;
} g_t;

#define CTXSZ 0x6200u
#define CFGSZ 0x600u
#define PSZ 0x80u

static void seed_echo_obj(unsigned char *o) {
  unsigned char near_hdr[0x20], far_hdr[0x20], p2074[4];
  fill_random(o, OBJSZ);
  V34InitializeImplementationSpecific(o); /* sets NEAR/FAR echo headers to in-object buffers */
  memcpy(near_hdr, OB(o, 0x80b8), 0x20);
  memcpy(far_hdr, OB(o, 0x9138), 0x20);
  memcpy(p2074, OB(o, 0x2074), 4);
  fill_random(o, OBJSZ);
  memcpy(OB(o, 0x80b8), near_hdr, 0x20);
  memcpy(OB(o, 0x9138), far_hdr, 0x20);
  memcpy(OB(o, 0x2074), p2074, 4);
  {
    static const unsigned int others[] = {0x268, 0x26c, 0x394, 0x2220, 0x2224, 0x2a28, 0xe48, 0x2608, 0xa28, 0x418, 0x1460, 0x20cc, 0x2100};
    size_t i;
    for (i = 0; i < sizeof others / sizeof others[0]; i++) U32(o, others[i]) = 0x0d000000u | (lcg_next() & 0xffffffu);
  }
}

static void run_txmit(logger_t *lg, int variant, int valid_states) {
  static const int bset[] = {600, 2400, 2743, 2800, 3000, 3200, 3429, 4800, 0, 3100, -1, 5000};
  static const int cset[] = {1200, 1600, 1680, 1800, 1829, 1867, 1920, 1959, 2000, 2400, 0, -1};
  unsigned char *o = g_obj, *snap = g_tmpl;
  unsigned char *ctx = xalloc(CTXSZ), *cfg = xalloc(CFGSZ), *P = xalloc(PSZ);
  unsigned char *ctx_s = xalloc(CTXSZ), *cfg_s = xalloc(CFGSZ), *P_s = xalloc(PSZ);
  V34I_UNREG(ctx_s); /* T05: the snapshots are never handed to the port: not images */
  V34I_UNREG(cfg_s);
  unsigned int h, hc, hf, hp;
  int baud = bset[variant % 12], carrier = cset[(variant / 12 + variant) % 12];
  int preemp = lcg_range(-2, 12);
  int grp = variant / 12; /* selectors below use the GROUP index so they are independent of baud = variant % 12 */
  int v90sel = (grp / 9) % 4;
  uint32_t r24c, r250;
  const unsigned int ctxslot[1] = {0x610c};
  int rx0, tx0, scale0, red0;

  seed_echo_obj(o);
  fill_random(ctx, CTXSZ);
  fill_random(cfg, CFGSZ);
  fill_random(P, PSZ);
  U32(o, 0xac3c) = 0; /* set below */
  PTR(o, 0x3548) = ctx;
  PTRX_SET(ctx, 0x610c, cfg);
  PTR(o, 0xac3c) = P;

  S16(o, 0xaa84) = (int16_t)baud;
  S16(o, 0xaa94) = (int16_t)carrier;
  S16(o, 0xaa8a) = (int16_t)preemp;
  {
    static const uint32_t nzvals[5] = {1u, 0x10000u, 0x80000000u, 0x100u, 0xfffe0000u};
    unsigned int i1 = lcg_mod(6), i2 = lcg_mod(6);
    r24c = (v90sel & 1) ? (i1 < 5 ? nzvals[i1] : (lcg_next() | 0x10000u)) : 0u; /* nonzero, low 16 bits often ZERO */
    r250 = (v90sel & 2) ? (i2 < 5 ? nzvals[i2] : (lcg_next() | 0x10000u)) : 0u;
  }
  if (grp % 7 == 3) { r24c = 0; r250 = 0; }
  U32(o, 0x24c) = r24c;
  U32(o, 0x250) = r250;
  /* rx/tx handshake states: force each combination; valid_states keeps the StateName index in 0..86 */
  {
    int rxsel = grp % 3, txsel = (grp / 3) % 3;
    if (rxsel == 0) U16(o, 0x3594) = 0x23; else U16(o, 0x3594) = valid_states ? (uint16_t)lcg_mod(87) : (uint16_t)lcg_next();
    if (txsel == 0) U16(o, 0x3596) = 0x12; else U16(o, 0x3596) = valid_states ? (uint16_t)lcg_mod(87) : (uint16_t)lcg_next();
    if (U16(o, 0x3594) == 0x23 && rxsel != 0) U16(o, 0x3594) = 0x22;
    if (U16(o, 0x3596) == 0x12 && txsel != 0) U16(o, 0x3596) = 0x13;
    if (valid_states) U16(o, 0x3592) = (uint16_t)lcg_mod(87);
  }
  S16(o, 0x25d4) = (int16_t)lcg_range(100, 30000);
  U16(o, 0x264 + 0x122) = (uint16_t)lcg_next();
  if (grp % 5 == 0) U16(o, 0x264 + 0x122) |= 0x0800;
  U16(o, 0x25c2) = (uint16_t)(lcg_next() & ((grp & 1) ? 0xffffu : 0xfdffu));
  /* settxlevel / GetVPcmMinimalTxPowerReduction inputs */
  S16(P, 0x44) = (int16_t)lcg_range(-20, 15);
  I32(P, 0x54) = (grp % 2) ? 4 : (int32_t)lcg_range(-3, 9);
  I32(ctx, 0x6120) = (grp % 5 == 1) ? 0 : (int32_t)lcg_next();
  I32(cfg, 0x4f8) = (grp % 7 == 2) ? 0 : (int32_t)(lcg_next() | 1u);

  memcpy(snap, o, OBJSZ);
  memcpy(ctx_s, ctx, CTXSZ);
  memcpy(cfg_s, cfg, CFGSZ);
  memcpy(P_s, P, PSZ);
  rx0 = U16(o, 0x3594); tx0 = U16(o, 0x3596); scale0 = S16(o, 0x25d4); red0 = S16(o, 0x25dc);

  ext_clear();
  ext_reg(ctx, CTXSZ);
  ext_reg(cfg, CFGSZ);
  ext_reg(P, PSZ);
  g_ptbl = exp_ptable(baud);
  g_pre_nz = (int16_t)preemp != 0;
  v34setuptxmit(o);

  h = diff_obj(o, snap);
  hc = diff_buf(ctx, ctx_s, CTXSZ, o, ctxslot, 1);
  hf = diff_buf(cfg, cfg_s, CFGSZ, o, NULL, 0);
  hp = diff_buf(P, P_s, PSZ, o, NULL, 0);
  h = fnv(h, &hc, 4);
  h = fnv(h, &hf, 4);
  h = fnv(h, &hp, 4);
  log_call2(lg, h, baud, carrier, preemp);

  /* coverage (inputs) */
  if (rx0 == 0x23) g_t.rx_eq++; else g_t.rx_ne++;
  if (tx0 == 0x12) g_t.tx_eq++; else g_t.tx_ne++;
  if (rx0 != 0x23 && tx0 != 0x12) g_t.both_ne++;
  if (!r24c && !r250) g_t.v90_00++; else if (r24c && !r250) g_t.v90_10++; else if (!r24c && r250) g_t.v90_01++; else g_t.v90_11++;
  if (S16(o, 0x25d4) != scale0) g_t.scale_changed++;
  if (S16(o, 0x25dc) != red0) g_t.red_changed++;
  if (S16(P, 0x44) < 0) g_t.ptr_neg++; else g_t.ptr_pos++;
  if (I32(ctx, 0x6120) == 0) g_t.ctx6120_zero++; else g_t.ctx6120_nz++;
  if (I32(cfg, 0x4f8) == 0) g_t.cfg4f8_zero++; else g_t.cfg4f8_nz++;
  if (I32(P, 0x54) == 4) g_t.p54_4++; else g_t.p54_other++;
  if (baud == 2400 || baud == 2800 || baud == 3000 || baud == 3200 || baud == 3429 || baud == 4800 || baud == 600) g_t.bauds_valid++; else g_t.bauds_invalid++;
  if (U16(o, 0x3594) != 0x23 || U16(o, 0x3596) != 0x12) { fprintf(stderr, "harness: states not set\n"); g_fail = 1; }
  if ((U16(o, 0x264 + 0x122) & 0x0800) == 0 && (U16(snap, 0x264 + 0x122) & 0x0800)) g_t.flag800_was_set++;
  if ((U16(o, 0x25c2) & 0x200) && !(U16(snap, 0x25c2) & 0x200)) g_t.flag200_was_set++;
  g_ptbl = NULL;
  free(ctx); free(cfg); free(P); free(ctx_s); free(cfg_s); free(P_s);
}

static void scen_txmit(void) {
  logger_t lg = {"v34setuptxmit", FNV0, 0, 61};
  int c;
  g_tmpl = xalloc(OBJSZ);
  g_obj = xalloc(OBJSZ);
  memset(&g_t, 0, sizeof g_t);
  for (c = 0; c < 1500; c++) run_txmit(&lg, c, 0);
  log_done(&lg);
  need("txmit rx state == 0x23 before", g_t.rx_eq);
  need("txmit rx state != 0x23 before", g_t.rx_ne);
  need("txmit tx state == 0x12 before", g_t.tx_eq);
  need("txmit tx state != 0x12 before", g_t.tx_ne);
  need("txmit both states differ before", g_t.both_ne);
  need("txmit 0x24c==0 && 0x250==0 (v90=0)", g_t.v90_00);
  need("txmit 0x24c!=0 only (v90=1)", g_t.v90_10);
  need("txmit 0x250!=0 only (v90=1)", g_t.v90_01);
  need("txmit both nonzero", g_t.v90_11);
  need("txmit settxlevel changed txscale", g_t.scale_changed);
  need("txmit settxlevel changed reduction", g_t.red_changed);
  need("txmit P+0x44 negative", g_t.ptr_neg);
  need("txmit P+0x44 non-negative", g_t.ptr_pos);
  need("txmit ctx+0x6120 == 0", g_t.ctx6120_zero);
  need("txmit ctx+0x6120 != 0", g_t.ctx6120_nz);
  need("txmit cfg+0x4f8 == 0", g_t.cfg4f8_zero);
  need("txmit cfg+0x4f8 != 0", g_t.cfg4f8_nz);
  need("txmit P+0x54 == 4", g_t.p54_4);
  need("txmit P+0x54 != 4", g_t.p54_other);
  need("txmit valid baud", g_t.bauds_valid);
  need("txmit invalid baud", g_t.bauds_invalid);
  need("txmit b+0x122 bit 0x800 cleared", g_t.flag800_was_set);
  need("txmit 0x25c2 bit 0x200 set", g_t.flag200_was_set);
}

/* ===================== scenario 5: debug level 2 output ===================== */
static void scen_debug(void) {
  static const int bset[] = {2400, 2743, 2800, 3000, 3200, 3429, 4800, 600, 1, 3100, 5000, -7};
  static const int cset[] = {1600, 1680, 1800, 1829, 1867, 1920, 1959, 2000, 1200, 2400, 0, -1, 777};
  logger_t lg = {"debug", FNV0, 0, 1};
  int i, j, c;
  g_tmpl = xalloc(OBJSZ);
  g_obj = xalloc(OBJSZ);
  g_bank_tmpl = xalloc(BANKSZ);
  g_bank = xalloc(BANKSZ);
  tmpl_refresh();
  fill_random(g_bank_tmpl, BANKSZ);
  dsplibs_debug_level = 2;
  for (i = 0; i < (int)(sizeof bset / sizeof bset[0]); i++)
    for (j = 0; j < (int)(sizeof cset / sizeof cset[0]); j += 3) {
      memcpy(g_obj, g_tmpl, OBJSZ);
      printf("-- demod baud=%d carrier=%d\n", bset[i], cset[j]);
      V34SetupDemodulator(g_obj, bset[i], cset[j]);
      printf("diff %08x\n", diff_obj(g_obj, g_tmpl));
    }
  for (i = 0; i < (int)(sizeof bset / sizeof bset[0]); i++)
    for (j = 0; j < (int)(sizeof cset / sizeof cset[0]); j++) {
      int rf;
      long rq;
      unsigned int h;
      int fr = (i + j) % 3 ? 1 : 0;
      memcpy(g_bank, g_bank_tmpl, BANKSZ);
      printf("-- mod baud=%d carrier=%d fr=%d\n", bset[i], cset[j], fr);
      h = mod_call(bset[i], cset[j], (i + j) % 4, (i * j) & 1, fr, (i + j) & 1, &rf, &rq);
      printf("diff %08x\n", h);
    }
  for (c = 0; c < 108; c++) {
    printf("-- txmit c=%d\n", c);
    run_txmit(&lg, 12 * c + (c % 12), 1); /* group index c: rx/tx selectors cycle through all 9 combos */
  }
  dsplibs_debug_level = 0;
}

int main(int argc, char **argv) {
  int s = argc > 1 ? atoi(argv[1]) : -1;
  g_verbose = getenv("V34C_VERBOSE") ? atoi(getenv("V34C_VERBOSE")) : 0;
  if (getenv("H01_POISON")) g_h01_poison = atoi(getenv("H01_POISON")) & 0xff;
  tables_init();
  slots_init();
  fprintf(stderr, "tables registered: %d (blob tables content-checked against port copies: %s)\n", g_ntbl,
#ifdef TEST_BLOB
         "yes"
#else
         "n/a (port build)"
#endif
  );
  switch (s) {
    case 0: scen_initV34(); break;
    case 1: scen_demod(); break;
    case 2: scen_mod_grid(); break;
    case 3: scen_mod_sweep(); break;
    case 4: scen_txmit(); break;
    case 5: scen_debug(); break;
    default: fprintf(stderr, "usage: %s <scenario 0-5>\n", argv[0]); return 1;
  }
  fflush(stdout);
  return g_fail ? 2 : 0;
}
