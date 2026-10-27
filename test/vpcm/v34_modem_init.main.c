/*
 * Differential test harness for V.34 "tier 4b" (src/vpcm/v34_modem_init.c):
 *   v34modeminit, VPcmV34SetTxScale, cEncodeChar, getMPrecvdBits
 *
 * One source, two binaries (same convention as test/v34/v34_init_tables.main.c):
 *   v34_tier4b_test_blob - linked against the real dsplibs.o (-DTEST_BLOB)
 *   v34_tier4b_test_port - linked against src/vpcm/v34_modem_init.c + tier 3/2 ports + tables
 * Each scenario runs in its OWN process (argv[1]).
 *
 * Reaching the blob's functions:
 *   - v34modeminit, VPcmV34SetTxScale, cEncodeChar are GLOBAL.
 *   - getMPrecvdBits is LOCAL, regparm(1) (obj in EAX). dsplibs.o is one contiguous
 *     .text piece, so its address is the address of the GLOBAL `bitreverse` (ELF .text
 *     0x5d770) minus the known offset difference (getMPrecvdBits is at 0x9250); the
 *     16 prologue bytes are verified at run time (abort exit 3 on mismatch).
 *   - LOCAL tables (modulator/echo-prefilter tables whose addresses v34modeminit stores
 *     into the object) are located from the GLOBAL anchor hsine1200 (.rodata 0x30a0) plus
 *     each table's ELF offset and memcmp'ed against the port's extraction at start-up.
 *   - edprintf's static state (temp/enc/iEncodeOffset) is located from the first
 *     sysdep_vsnprintf buffer (the stubs live in this file) exactly as test 3d does; the
 *     layout (enc at +0x100, iEncodeOffset at +0x210) is asserted.
 *
 * Objects: tagV34Object is not a struct. A random fake object (OBJSZ bytes) is refreshed per
 * call; the result is the hash of the DIFF (word index + new value of every changed word), so
 * stray writes anywhere are caught. Pointer slots are compared NORMALISED (in-object -> offset,
 * table/function -> (id, offset), external buffer -> (region id, offset)); raw addresses are
 * never compared across the two processes.
 *
 * Scenarios: 0 VPcmV34SetTxScale, 1 cEncodeChar, 2 getMPrecvdBits random, 3 getMPrecvdBits
 * debug levels + level flips, 4 v34modeminit random, 5 v34modeminit debug levels + flips,
 * 6 obj+0x25c2 probe (twin runs of each callee), 7 getMPrecvdBits boundary grid.
 * Every scenario keeps COVERAGE COUNTERS (from the INPUTS / independent predicates) and exits 2
 * if a branch class was never reached.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../v34/v34_img.h" /* T04: x86-64 image <-> native V34Obj marshalling; empty at -m32 */

extern unsigned int dsplibs_debug_level;

/* ---- functions under test ---- */
extern void v34modeminit(void *obj);
extern void VPcmV34SetTxScale(void *obj);
extern int cEncodeChar(int c); /* int-width: the harness passes garbage above the low byte */
extern void edprintf(const char *fmt, ...);
/* already-verified helpers */
extern void V34InitializeImplementationSpecific(void *obj);
extern unsigned short bitreverse(unsigned short x, short nbits);
extern void preinitdigital(void *obj);
extern void txinit(void *obj);
extern int rxinit(void *obj);
extern int *V34SetupModulator(void *m, int baud, int carrier, int preemp, int v90, int fullReset);
#if defined(__x86_64__)
#define V34PW(p) ((const void *)(uintptr_t)(p)) /* T06: detectorinit's p2 is a table ADDRESS stored in a pointer slot: a real pointer parameter on x86-64 */
#define V34PWV(p) ((uint32_t)(uintptr_t)(p))
#define V34PWT const void *
#else
#define V34PW(p) (p)
#define V34PWV(p) (p)
#define V34PWT uint32_t
#endif
extern void detectorinit(void *blk, V34PWT p2, int p3, int p4, int p5, int p6, int p7);
extern void scrambleGPC(void);
extern void scrambleGPA(void);
extern void descrambleGPA(void);
extern void descrambleGPC(void);

#ifdef TEST_BLOB
typedef void(__attribute__((regparm(1))) * gmp_fn_t)(void *);
static void call_gmp(void *obj) {
  static gmp_fn_t fn;
  if (!fn) {
    static const unsigned char sig[16] = {0x55, 0x8d, 0x50, 0x04, 0x57, 0x8d, 0xb8, 0x0c, 0xaa, 0x00, 0x00, 0x56, 0x53, 0x83, 0xec, 0x1c};
    unsigned char *p = (unsigned char *)&bitreverse - 0x5d770 + 0x9250;
    if (memcmp(p, sig, sizeof sig) != 0) {
      fprintf(stderr, "getMPrecvdBits address arithmetic failed (prologue mismatch)\n");
      exit(3);
    }
    fn = (gmp_fn_t)p;
  }
  fn(obj);
}
#else
extern void getMPrecvdBits(void *obj);
#if defined(__x86_64__)
#define call_gmp(obj) getMPrecvdBits(obj) /* x86-64: expands after the shim macro of getMPrecvdBits (test/v34/v34_img.h) */
#else
static void call_gmp(void *obj) { getMPrecvdBits(obj); }
#endif
#endif

/* ---- tables ---- */
#define GLOBAL_TBLS(X)                                                                                       \
  X(Convolve64, 128) X(Convolve32, 128) X(Convolve16, 128) X(hsine1200, 32) X(hsine1600, 24)                 \
  X(hsine1680, 160) X(hsine1800, 64) X(hsine1829, 84) X(hsine1867, 144) X(hsine1920, 20)                     \
  X(hsine1959, 196) X(hsine2000, 96) X(hsine2400, 32) X(c1200_, 16) X(c2400_, 16) X(bpv22high, 120) X(bpv22low, 120)

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
static tbl_t g_tbl[80];
static int g_ntbl;

static void add_tbl(const char *name, const void *addr, size_t size) {
  g_tbl[g_ntbl].name = name;
  g_tbl[g_ntbl].addr = (const unsigned char *)addr;
  g_tbl[g_ntbl].size = size;
  g_ntbl++;
}
static int tbl_index(const char *name) {
  int i;
  for (i = 0; i < g_ntbl; i++)
    if (!strcmp(g_tbl[i].name, name)) return i;
  fprintf(stderr, "harness: unknown table %s\n", name);
  exit(3);
}
static uint32_t tbl_id(const char *name) { return 0x7e000000u | ((uint32_t)tbl_index(name) << 16); }

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
  /* function pointers stored by preinitdigital (identity compared, never the address) */
  add_tbl("fn_scrambleGPC", (const void *)scrambleGPC, 1);
  add_tbl("fn_scrambleGPA", (const void *)scrambleGPA, 1);
  add_tbl("fn_descrambleGPA", (const void *)descrambleGPA, 1);
  add_tbl("fn_descrambleGPC", (const void *)descrambleGPC, 1);
}

/* ---- accessors ---- */
/* V34SHIMS BEGIN (generated by tools/tier_rewrite.py shims; do not edit) */
#if defined(__x86_64__)
static void V34SH_v34modeminit(void *obj) { v34i_enter(); v34modeminit((void *)v34i_in((void *)obj)); v34i_leave(); }
static void V34SH_VPcmV34SetTxScale(void *obj) { v34i_enter(); VPcmV34SetTxScale((void *)v34i_in((void *)obj)); v34i_leave(); }
static void V34SH_V34InitializeImplementationSpecific(void *obj) { v34i_enter(); V34InitializeImplementationSpecific((void *)v34i_in((void *)obj)); v34i_leave(); }
static void V34SH_preinitdigital(void *obj) { v34i_enter(); preinitdigital((void *)v34i_in((void *)obj)); v34i_leave(); }
static void V34SH_txinit(void *obj) { v34i_enter(); txinit((void *)v34i_in((void *)obj)); v34i_leave(); }
static int V34SH_rxinit(void *obj) { int r; v34i_enter(); r = rxinit((void *)v34i_in((void *)obj)); v34i_leave(); return r;}
static int * V34SH_V34SetupModulator(void *m, int baud, int carrier, int preemp, int v90, int fullReset) { int * r; v34i_enter(); r = V34SetupModulator((void *)v34i_in((void *)m), baud, carrier, preemp, v90, fullReset); v34i_leave(); return (int *)v34i_outp((void *)r);}
static void V34SH_detectorinit(void *blk, V34PWT p2, int p3, int p4, int p5, int p6, int p7) { v34i_enter(); detectorinit((void *)v34i_in((void *)blk), p2, p3, p4, p5, p6, p7); v34i_leave(); }
static void V34SH_getMPrecvdBits(void *obj) { v34i_enter(); getMPrecvdBits((void *)v34i_in((void *)obj)); v34i_leave(); }
static void *const V34REAL_v34modeminit = (void *)v34modeminit; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_VPcmV34SetTxScale = (void *)VPcmV34SetTxScale; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_V34InitializeImplementationSpecific = (void *)V34InitializeImplementationSpecific; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_preinitdigital = (void *)preinitdigital; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_txinit = (void *)txinit; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_rxinit = (void *)rxinit; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_V34SetupModulator = (void *)V34SetupModulator; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_detectorinit = (void *)detectorinit; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_getMPrecvdBits = (void *)getMPrecvdBits; /* the real address (the name is a macro for the shim below) */
#define v34modeminit V34SH_v34modeminit
#define VPcmV34SetTxScale V34SH_VPcmV34SetTxScale
#define V34InitializeImplementationSpecific V34SH_V34InitializeImplementationSpecific
#define preinitdigital V34SH_preinitdigital
#define txinit V34SH_txinit
#define rxinit V34SH_rxinit
#define V34SetupModulator V34SH_V34SetupModulator
#define detectorinit V34SH_detectorinit
#define getMPrecvdBits V34SH_getMPrecvdBits
#endif
/* V34SHIMS END */
#define OB(o, off) ((unsigned char *)(o) + (off))
#define S8(o, off) (*(int8_t *)OB(o, off))
#define U8(o, off) (*(uint8_t *)OB(o, off))
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
#define PSZ 0x80u

/* ---- deterministic LCG with a bijective finaliser ---- */
static unsigned int g_lcg = 0x4b1de5a7u;
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
static void die(const char *msg) {
  fflush(stdout);
  fprintf(stderr, "HARNESS ERROR: %s\n", msg);
  exit(3);
}

static unsigned char *xalloc(size_t n) {
  unsigned char *p = (unsigned char *)calloc(1, n);
  if (!p) exit(4);
  if (n == OBJSZ) V34I_REG(p, n);
  else if (n == CTXSZ) V34I_REGC(p, n, ctx);
  else if (n == CFGSZ) V34I_REGC(p, n, cfg);
  return p;
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
    0x268,  0x26c,  0x394,  0x2220, 0x2224, 0x2a28, 0xe48,  0x2608, 0xa28,  0x80b8, 0x80bc, 0x80c0, 0x80c4, 0x80c8,
    0x9138, 0x913c, 0x9140, 0x9144, 0x9148, 0x418,  0x1460, 0x2074, 0x20cc, 0x2100, 0x3548, 0xac3c, 0x508,  0x3564,
    0xaa6c};
#define NSLOTS (sizeof k_slots / sizeof k_slots[0])

static unsigned char g_isslot[OBJSZ / 4];
static void slots_init(void) {
  size_t i;
  for (i = 0; i < NSLOTS; i++) g_isslot[k_slots[i] / 4] = 1;
}

static void scrub_slots(unsigned char *o) {
  size_t i;
  for (i = 0; i < NSLOTS; i++) U32(o, k_slots[i]) = 0x0d000000u | (lcg_next() & 0xffffffu);
}

static unsigned int diff_obj(const unsigned char *obj, const unsigned char *snap) {
  unsigned int h = FNV0;
  size_t i;
  for (i = 0; i < OBJSZ / 4; i++) {
    uint32_t a, b;
    memcpy(&a, obj + 4 * i, 4);
    memcpy(&b, snap + 4 * i, 4);
    if (g_isslot[i]) {
      a = norm_val(obj, a);
      b = norm_val(obj, b);
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

/* hash of the WHOLE object (slots normalised) - for the 0x25c2 twin probe */
static unsigned int hash_obj(const unsigned char *obj) {
  unsigned int h = FNV0;
  size_t i;
  for (i = 0; i < OBJSZ / 4; i++) {
    uint32_t a;
    memcpy(&a, obj + 4 * i, 4);
    if (g_isslot[i]) a = norm_val(obj, a);
    h = fnv(h, &a, 4);
  }
  return h;
}

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

/* ============================================================================
 * sysdep_* stubs (defined here), the debug printer (with level-flip hook), and
 * the edprintf static-state accessors
 * ========================================================================== */
#define ENC_OFF 0x100
#define IDX_OFF 0x210
#define ENC_SZ 0x110
static char *g_temp; /* learned from the first vsnprintf buf */
static char *g_enc;  /* learned from the first strcpy dst   */

int sysdep_vsnprintf(char *buf, unsigned int n, const char *fmt, va_list ap) {
  int r = vsnprintf(buf, n, fmt, ap);
  if (!g_temp) g_temp = buf;
  return r;
}
size_t sysdep_strlen(const char *s) { return strlen(s); }
char *sysdep_strcpy(char *d, const char *s) {
  if (!g_enc) {
    g_enc = d;
    if (!g_temp || g_enc != g_temp + ENC_OFF) die("encoded buffer is not at temp+0x100 (layout assumption broken)");
  }
  return strcpy(d, s);
}
char *sysdep_strcat(char *d, const char *s) { return strcat(d, s); }

static int g_flip_after = -1;   /* after the k-th (0-based) print of a call, set the level to g_flip_to */
static unsigned int g_flip_to;
static int g_nprint;            /* prints since the last reset */
static long g_total_prints;
int dsplibs_debug_printf(const char *fmt, ...) {
  va_list ap;
  int ret;
  va_start(ap, fmt);
  ret = vprintf(fmt, ap);
  va_end(ap);
  g_total_prints++;
  if (g_flip_after >= 0 && g_nprint == g_flip_after) dsplibs_debug_level = g_flip_to;
  g_nprint++;
  return ret;
}

static int ed_idx_get(void) {
  int v;
  memcpy(&v, g_temp + IDX_OFF, 4);
  return v;
}
static void ed_idx_set(int v) { memcpy(g_temp + IDX_OFF, &v, 4); }
static unsigned int ed_state_hash(void) {
  unsigned int h = FNV0;
  if (!g_temp) return h;
  h = fnv(h, g_temp, 0x100);
  h = fnv(h, g_temp + ENC_OFF, ENC_SZ);
  h = fnv(h, g_temp + IDX_OFF, 4);
  return h;
}
/* make sure the static buffers are known (one harmless edprintf call at level 0) */
static void ed_learn(void) {
  unsigned int saved = dsplibs_debug_level;
  dsplibs_debug_level = 0;
  if (!g_temp) edprintf("learn");
  dsplibs_debug_level = saved;
  if (!g_temp || !g_enc) die("could not learn edprintf buffers");
}

/* ============================================================================
 * fake-object plumbing
 * ========================================================================== */
static unsigned char *g_obj;  /* live object (fixed address for the whole process) */
static unsigned char *g_snap; /* pre-call snapshot */

/* random object with a VALID fake echo canceller (txinit -> V34EchoCleanUp needs it) and the
 * pointer slots scrubbed to junk (so a random word never looks like a pointer into any range) */
static void seed_echo_obj(unsigned char *o) {
  unsigned char near_hdr[0x20], far_hdr[0x20], p2074[4];
  size_t i;
  fill_random(o, OBJSZ);
  V34InitializeImplementationSpecific(o); /* sets NEAR/FAR echo headers + bank pointer to in-object buffers */
  memcpy(near_hdr, OB(o, 0x80b8), 0x20);
  memcpy(far_hdr, OB(o, 0x9138), 0x20);
  memcpy(p2074, OB(o, 0x2074), 4);
  fill_random(o, OBJSZ);
  scrub_slots(o);
  memcpy(OB(o, 0x80b8), near_hdr, 0x20);
  memcpy(OB(o, 0x9138), far_hdr, 0x20);
  memcpy(OB(o, 0x2074), p2074, 4);
  /* V34InitializeImplementationSpecific leaves writePtr = the OLD (random) base word; a random 32-bit value can
   * land inside a registered address range in exactly one of the two processes and then normalise differently
   * (only matters for the whole-object hash of the 0x25c2 probe). Make the writePtr slots valid in-object pointers. */
  U32(o, 0x80b8) = U32(o, 0x80bc);
  U32(o, 0x9138) = U32(o, 0x913c);
  (void)i;
}

/* ============================================================================
 * scenario 0: VPcmV34SetTxScale
 * ========================================================================== */
static const unsigned int k_levels[6] = {0u, 1u, 2u, 3u, 0x80000000u, 0xffffffffu};

static void scen_txscale(void) {
  logger_t lg = {"VPcmV34SetTxScale", FNV0, 0, 97};
  long printed = 0, quiet = 0, lvl1 = 0, huge = 0, wasSame = 0;
  int c;
  g_obj = xalloc(OBJSZ);
  g_snap = xalloc(OBJSZ);
  ed_learn();
  for (c = 0; c < 3600; c++) {
    unsigned int lvl = k_levels[c % 6];
    unsigned int h, hs;
    long before = g_total_prints;
    fill_random(g_obj, OBJSZ);
    scrub_slots(g_obj);
    if (c % 5 == 0) U16(g_obj, 0x25d4) = 0x16a1;
    memcpy(g_snap, g_obj, OBJSZ);
    dsplibs_debug_level = lvl;
    VPcmV34SetTxScale(g_obj);
    dsplibs_debug_level = 0;
    h = diff_obj(g_obj, g_snap);
    hs = ed_state_hash();
    h = fnv(h, &hs, 4);
    log_call2(&lg, h, (long)lvl, (long)U16(g_obj, 0x25d4), (long)ed_idx_get());
    if (U16(g_obj, 0x25d4) != 0x16a1) die("0x25d4 not 0x16a1");
    if (g_total_prints > before) printed++; else quiet++;
    if (lvl == 1) lvl1++;
    if (lvl >= 0x80000000u) huge++;
    if (U16(g_snap, 0x25d4) == 0x16a1) wasSame++;
  }
  log_done(&lg);
  need("txscale printed (level>1)", printed);
  need("txscale quiet (level<=1)", quiet);
  need("txscale level==1 boundary", lvl1);
  need("txscale unsigned-huge level", huge);
  need("txscale value already 0x16a1", wasSame);
}

/* ============================================================================
 * scenario 1: cEncodeChar (shares iEncodeOffset with edprintf)
 * ========================================================================== */
static void scen_cencode(void) {
  logger_t lg = {"cEncodeChar", FNV0, 0, 1499};
  long wraps = 0, neg = 0, nonneg = 0, idxseen[10], hi_garbage = 0, chars_hi = 0, seq_ok = 0, ed_between = 0;
  int idx, c, k, variant;
  unsigned int hh = FNV0;
  memset(idxseen, 0, sizeof idxseen);
  ed_learn();
  /* A: exhaustive index (0..9) x char (0..255) x 3 garbage-in-upper-24-bits variants */
  for (idx = 0; idx < 10; idx++)
    for (c = 0; c < 256; c++)
      for (variant = 0; variant < 3; variant++) {
        unsigned int arg = (unsigned int)c;
        int r, after;
        if (variant == 1) arg |= (lcg_next() & 0xffffff00u);
        if (variant == 2) arg |= 0xffffff00u;
        ed_idx_set(idx);
        r = cEncodeChar((int)arg);
        after = ed_idx_get();
        if (after != (idx == 9 ? 0 : idx + 1)) die("cEncodeChar index update wrong (or iEncodeOffset address wrong)");
        hh = fnv(hh, &r, 4);
        hh = fnv(hh, &after, 4);
        log_call2(&lg, fnv(fnv(FNV0, &r, 4), &after, 4), idx, (long)arg, r);
        idxseen[idx]++;
        if (idx == 9) wraps++;
        if (r < 0) neg++; else nonneg++;
        if (c >= 0x80) chars_hi++;
        if (variant) hi_garbage++;
      }
  /* B: chained random sequence (index carried between calls), with occasional edprintf calls
   * that reset/advance the SHARED index */
  ed_idx_set(0);
  for (k = 0; k < 30000; k++) {
    unsigned int arg = lcg_next();
    int before = ed_idx_get(), r, after;
    if (k % 41 == 40) {
      /* edprintf resets the index to 0 and leaves (2*len) mod 10 */
      static const char *msgs[] = {"a", "hello", "hello world", "x%dy", "\x80\xff", "", "0123456789"};
      const char *m = msgs[lcg_mod(7)];
      if (strchr(m, '%')) edprintf(m, 5); else edprintf(m);
      ed_between++;
      before = ed_idx_get();
      log_call2(&lg, ed_state_hash(), -1, before, 0);
    }
    r = cEncodeChar((int)arg);
    after = ed_idx_get();
    if (after != (before == 9 ? 0 : before + 1)) die("cEncodeChar chained index update wrong");
    seq_ok++;
    log_call2(&lg, fnv(fnv(FNV0, &r, 4), &after, 4), before, (long)arg, r);
  }
  log_done(&lg);
  printf("cEncodeChar exhaustive-hash=%08x\n", hh);
  need("cEncodeChar index wrap (idx==9)", wraps);
  need("cEncodeChar negative result", neg);
  need("cEncodeChar non-negative result", nonneg);
  for (idx = 0; idx < 10; idx++) {
    char nm[48];
    snprintf(nm, sizeof nm, "cEncodeChar index %d", idx);
    need(nm, idxseen[idx]);
  }
  need("cEncodeChar garbage in upper 24 bits", hi_garbage);
  need("cEncodeChar chars >= 0x80", chars_hi);
  need("cEncodeChar chained calls", seq_ok);
  need("cEncodeChar interleaved edprintf", ed_between);
}

/* ============================================================================
 * getMPrecvdBits
 * ========================================================================== */
static unsigned char *g_ctx, *g_cfg, *g_P, *g_ctx_s, *g_cfg_s, *g_P_s;

static struct {
  long calls, f24c_le0, f24c_1, f24c_ge2, f24c_zero_lowhalf_pos, f24c_neg;
  long t_step1, t_step2, t_twice, t_step2_only;
  long reg_a, reg_b, reg_c, sens, clamp, noclamp, equal, ovf;
  long lowrate, highrate, boundary, negrate;
  long rpat[16];
  long w_bit0, w_2000, w_4000, w_8000, byte_neg, byte_hi, cfg_poison;
  long bits_set_k[4], bits_clr_k[4];
  long old_aa0c_bit0, old_aa3c_bit[4];
} g_g;

#define CTX_MP 0x1744

static void gmp_setup_random(int variant) {
  unsigned char *o = g_obj;
  int k;
  fill_random(o, OBJSZ);
  scrub_slots(o);
  fill_random(g_ctx, CTXSZ);
  fill_random(g_cfg, CFGSZ);
  fill_random(g_P, PSZ);
  PTR(o, 0x3548) = g_ctx;
  PTRX_SET(g_ctx, 0x610c, g_cfg);
  PTR(o, 0xac3c) = g_P;
  (void)k;
  (void)variant;
}

static int32_t pick_24c(void) {
  static const int32_t vals[] = {0,          1,           2,           3,          -1,           0x7fffffff, (int32_t)0x80000000,
                                 0x10000,    0x20000,     (int32_t)0xffff0000, 0x100,       -2,          100,        (int32_t)0xffff0001,
                                 0x10001,    0x1ffff,     (int32_t)0x80010000, 5,          -100000,     0x40000000};
  unsigned int r = lcg_mod(10);
  if (r < 7) return vals[lcg_mod(sizeof vals / sizeof vals[0])];
  return (int32_t)lcg_next();
}
static uint8_t pick_byte(void) {
  unsigned int r = lcg_mod(8);
  if (r == 0) return 0;
  if (r == 1) return 1;
  if (r == 2) return 0x80;
  if (r == 3) return 2;
  if (r == 4) return 0xff;
  return (uint8_t)lcg_next();
}
static int32_t pick_nz32(void) {
  static const int32_t vals[] = {1, -1, 0x10000, (int32_t)0x80000000, 0x100, 0x20000, (int32_t)0xffff0000, 0x7fffffff, 2};
  unsigned int r = lcg_mod(4);
  if (r < 2) return vals[lcg_mod(sizeof vals / sizeof vals[0])];
  return (int32_t)(lcg_next() | 1u);
}
static int32_t pick_rate(void) {
  unsigned int r = lcg_mod(20);
  if (r < 6) return (int32_t)lcg_mod(0x8340 + 100);        /* mostly in range */
  if (r < 8) return 0x8340 + lcg_range(-4, 4);             /* boundary */
  if (r < 10) return -(int32_t)lcg_mod(200000);            /* negative small */
  if (r < 11) return (int32_t)0x80000000 + lcg_range(-3, 3); /* INT_MIN neighbourhood */
  if (r < 12) return 0x7fffffff - lcg_range(0, 3);
  if (r < 14) return lcg_range(0, 15) * 0x960 + lcg_range(-2, 2); /* around multiples of 0x960 */
  return (int32_t)lcg_next();
}

/* one getMPrecvdBits call on a freshly randomised environment */
static unsigned int run_gmp(logger_t *lg, int variant, int want_debug) {
  unsigned char *o = g_obj;
  unsigned int h, hc, hf, hp, hs;
  int32_t f24c = pick_24c();
  int32_t c6120 = (lcg_mod(3) == 0) ? 0 : pick_nz32();
  int32_t c4f8 = (lcg_mod(3) == 0) ? 0 : pick_nz32();
  int32_t rate = pick_rate();
  int32_t c4fc;
  const unsigned int ctxslot[1] = {0x610c};
  int i;

  gmp_setup_random(variant);
  {
    unsigned int m = lcg_mod(4);
    if (m == 0) c4fc = (int32_t)lcg_next();                                   /* wraps in the imul */
    else if (m == 1) c4fc = lcg_range(-20, 20);
    else if (m == 2) c4fc = (int32_t)((uint32_t)rate / 0x960u) + lcg_range(-1, 1); /* product near the rate */
    else c4fc = lcg_range(0, 40);
  }
  /* MP info bytes */
  for (i = 0; i < 6; i++) U8(g_ctx, CTX_MP + i) = pick_byte();
  I32(o, 0x24c) = f24c;
  I32(g_ctx, 0x6120) = c6120;
  I32(g_cfg, 0x4f8) = c4f8;
  I32(g_cfg, 0x4fc) = c4fc;
  I32(g_P, 0x3c) = rate;
  if (lcg_mod(4) == 0) U16(o, 0xaa0c) = (uint16_t)(U16(o, 0xaa0c) | 1u); /* stale bit 0 for step 2 */
  if (lcg_mod(4) == 0) U16(o, 0xaa0c) = (uint16_t)(U16(o, 0xaa0c) & ~1u);
  /* lazy cfg dereference: when the first two tests already decide "regular ISP" the cfg pointer is never
   * loaded through; poison it in half of those calls so an eager dereference would crash */
  {
    int short_circuit = (c6120 == 0) || (f24c < 2);
    if (short_circuit && lcg_mod(2) == 0) {
      PTRX_SET(g_ctx, 0x610c, (void *)(uintptr_t)(0x0d100000u | (lcg_next() & 0xfffffu)));
      g_g.cfg_poison++;
    }
  }

  memcpy(g_snap, o, OBJSZ);
  memcpy(g_ctx_s, g_ctx, CTXSZ);
  memcpy(g_cfg_s, g_cfg, CFGSZ);
  memcpy(g_P_s, g_P, PSZ);

  ext_clear();
  ext_reg(g_ctx, CTXSZ);
  ext_reg(g_cfg, CFGSZ);
  ext_reg(g_P, PSZ);
  g_nprint = 0;
  call_gmp(o);

  h = diff_obj(o, g_snap);
  hc = diff_buf(g_ctx, g_ctx_s, CTXSZ, o, ctxslot, 1);
  hf = diff_buf(g_cfg, g_cfg_s, CFGSZ, o, NULL, 0);
  hp = diff_buf(g_P, g_P_s, PSZ, o, NULL, 0);
  hs = ed_state_hash();
  h = fnv(h, &hc, 4);
  h = fnv(h, &hf, 4);
  h = fnv(h, &hp, 4);
  h = fnv(h, &hs, 4);
  log_call2(lg, h, (long)f24c, (long)rate, (long)U16(o, 0xaa3c));

  /* independent coverage from the inputs */
  (void)want_debug;
  g_g.calls++;
  if (f24c <= 0) g_g.f24c_le0++;
  if (f24c == 1) g_g.f24c_1++;
  if (f24c >= 2) g_g.f24c_ge2++;
  if (f24c > 0 && (f24c & 0xffff) == 0) g_g.f24c_zero_lowhalf_pos++;
  if (f24c < 0) g_g.f24c_neg++;
  {
    int b0 = U8(g_ctx, CTX_MP) != 0;
    uint32_t w = 0;
    if (f24c > 0) {
      g_g.t_step1++;
      w = (uint32_t)b0 | ((uint32_t)(S8(g_ctx, CTX_MP + 1) & 0xf) << 6) | ((uint32_t)(S8(g_ctx, CTX_MP + 2) & 3) << 11);
      if (U8(g_ctx, CTX_MP + 3)) w |= 0x2000;
      if (U8(g_ctx, CTX_MP + 4)) w |= 0x4000;
      if (U8(g_ctx, CTX_MP + 5)) w |= 0x8000;
      if (w & 1) g_g.w_bit0++;
      if (w & 0x2000) g_g.w_2000++;
      if (w & 0x4000) g_g.w_4000++;
      if (w & 0x8000) g_g.w_8000++;
      if (S8(g_ctx, CTX_MP + 1) < 0 || S8(g_ctx, CTX_MP + 2) < 0) g_g.byte_neg++;
      if ((U8(g_ctx, CTX_MP + 1) & 0xf0) || (U8(g_ctx, CTX_MP + 2) & 0xfc)) g_g.byte_hi++;
      if (w & 1) {
        if (U16(g_snap, 0xaa0c) & 1) { /* irrelevant: the new word's bit 0 decides step 2 */ }
        g_g.t_twice++;
      }
    } else {
      w = U16(g_snap, 0xaa0c);
      if (w & 1) g_g.t_step2_only++;
    }
    if (U16(o, 0xaa0c) & 1) g_g.t_step2++;
    if (U16(g_snap, 0xaa0c) & 1) g_g.old_aa0c_bit0++;
  }
  {
    int reg = 0;
    int32_t r2 = rate;
    if (c6120 == 0) { g_g.reg_a++; reg = 1; }
    else if (f24c < 2) { g_g.reg_b++; reg = 1; }
    else if (c4f8 == 0) { g_g.reg_c++; reg = 1; }
    if (!reg) {
      int32_t m = (int32_t)((uint32_t)c4fc * 0x960u);
      g_g.sens++;
      if (r2 > m) { g_g.clamp++; r2 = m; }
      else if (r2 == m) g_g.equal++;
      else g_g.noclamp++;
      if ((int64_t)c4fc * 0x960 != (int64_t)m) g_g.ovf++;
    }
    if (r2 < 0x8340) {
      int32_t t = (int32_t)((uint32_t)r2 * 8u - (uint32_t)r2);
      unsigned int rv = (unsigned int)(int16_t)bitreverse((unsigned short)((t >> 14) & 0xffff), 4);
      g_g.rpat[rv & 15]++;
      g_g.lowrate++;
      if (r2 < 0) g_g.negrate++;
      for (i = 0; i < 4; i++) {
        if ((rv >> i) & 1) g_g.bits_set_k[i]++; else g_g.bits_clr_k[i]++;
        if ((U16(g_snap, 0xaa3c) >> (6 + i)) & 1) g_g.old_aa3c_bit[i]++;
      }
    } else {
      g_g.highrate++;
    }
    if (r2 >= 0x833e && r2 <= 0x8341) g_g.boundary++;
  }
  return h;
}

static void gmp_report(void) {
  int i;
  char nm[64];
  need("gmp flag [0x24c] <= 0", g_g.f24c_le0);
  need("gmp flag [0x24c] == 1", g_g.f24c_1);
  need("gmp flag [0x24c] >= 2", g_g.f24c_ge2);
  need("gmp flag [0x24c] positive with zero low half", g_g.f24c_zero_lowhalf_pos);
  need("gmp flag [0x24c] negative", g_g.f24c_neg);
  need("gmp step 1 executed", g_g.t_step1);
  need("gmp step 1 built word with bit 0 (txrxdmainit inside step 1)", g_g.w_bit0);
  need("gmp txrxdmainit twice (step 1 + step 2)", g_g.t_twice);
  need("gmp step 2 only (stale word bit 0, flag<=0)", g_g.t_step2_only);
  need("gmp step 2 taken (final word bit 0)", g_g.t_step2);
  need("gmp stale aa0c bit 0 set before", g_g.old_aa0c_bit0);
  need("gmp w bit 0x2000", g_g.w_2000);
  need("gmp w bit 0x4000", g_g.w_4000);
  need("gmp w bit 0x8000", g_g.w_8000);
  need("gmp negative signed bytes (drn/trellis)", g_g.byte_neg);
  need("gmp drn/trellis bytes with bits above the mask", g_g.byte_hi);
  need("gmp regular ISP via ctx+0x6120==0", g_g.reg_a);
  need("gmp regular ISP via flag<2", g_g.reg_b);
  need("gmp regular ISP via cfg+0x4f8==0", g_g.reg_c);
  need("gmp sensitive ISP", g_g.sens);
  need("gmp sensitive: rate clamped", g_g.clamp);
  need("gmp sensitive: rate == limit", g_g.equal);
  need("gmp sensitive: rate below limit", g_g.noclamp);
  need("gmp sensitive: 32-bit imul overflow", g_g.ovf);
  need("gmp cfg pointer poisoned on lazy path", g_g.cfg_poison);
  need("gmp rate < 0x8340 (mask loop)", g_g.lowrate);
  need("gmp rate >= 0x8340 (skip)", g_g.highrate);
  need("gmp rate near boundary 0x833e..0x8341", g_g.boundary);
  need("gmp negative rate", g_g.negrate);
  for (i = 0; i < 16; i++) {
    snprintf(nm, sizeof nm, "gmp bitreverse result pattern %d", i);
    need(nm, g_g.rpat[i]);
  }
  for (i = 0; i < 4; i++) {
    snprintf(nm, sizeof nm, "gmp mask bit %d set from r", 6 + i);
    need(nm, g_g.bits_set_k[i]);
    snprintf(nm, sizeof nm, "gmp mask bit %d cleared from r", 6 + i);
    need(nm, g_g.bits_clr_k[i]);
    snprintf(nm, sizeof nm, "gmp old aa3c bit %d was set", 6 + i);
    need(nm, g_g.old_aa3c_bit[i]);
  }
}

static void gmp_alloc(void) {
  g_obj = xalloc(OBJSZ);
  g_snap = xalloc(OBJSZ);
  g_ctx = xalloc(CTXSZ);
  g_cfg = xalloc(CFGSZ);
  g_P = xalloc(PSZ);
  g_ctx_s = xalloc(CTXSZ);
  g_cfg_s = xalloc(CFGSZ);
  V34I_UNREG(g_ctx_s); /* T05: the snapshots are never handed to the port: not images */
  V34I_UNREG(g_cfg_s);
  g_P_s = xalloc(PSZ);
}

static void scen_gmp(void) {
  logger_t lg = {"getMPrecvdBits", FNV0, 0, 4999};
  int c;
  gmp_alloc();
  ed_learn();
  memset(&g_g, 0, sizeof g_g);
  for (c = 0; c < 120000; c++) run_gmp(&lg, c, 0);
  log_done(&lg);
  gmp_report();
}

/* debug scenario: levels, and prints that flip the level mid-chain */
static void scen_gmp_debug(void) {
  logger_t lg = {"getMPrecvdBits-debug", FNV0, 0, 1};
  long chain_full = 0, chain_cut = 0, quiet = 0, cut_at[10];
  int c;
  gmp_alloc();
  ed_learn();
  memset(&g_g, 0, sizeof g_g);
  memset(cut_at, 0, sizeof cut_at);
  for (c = 0; c < 4000; c++) {
    unsigned int lvl = k_levels[c % 6];
    int flip = (c / 6) % 12; /* 0..8 = flip after that print, 9..11 = no flip */
    long before = g_total_prints;
    int prints;
    static const unsigned int to[3] = {0u, 1u, 0x80000000u};
    if (flip <= 8) {
      g_flip_after = flip;
      g_flip_to = to[(c / 72) % 3];
    } else {
      g_flip_after = -1;
    }
    dsplibs_debug_level = lvl;
    printf("-- call %d level=%u flip=%d\n", c, lvl, g_flip_after);
    run_gmp(&lg, c, 1);
    prints = (int)(g_total_prints - before);
    dsplibs_debug_level = 0;
    g_flip_after = -1;
    if (prints == 0) quiet++;
    if (prints >= 8) chain_full++;
    if (prints >= 1 && prints < 8) chain_cut++;
    if (prints < 10) cut_at[prints]++;
  }
  log_done(&lg);
  need("gmp debug: quiet calls", quiet);
  need("gmp debug: full chain (8 prints)", chain_full);
  need("gmp debug: chain cut short by a level flip", chain_cut);
  {
    int i;
    char nm[64];
    for (i = 1; i <= 8; i++) {
      snprintf(nm, sizeof nm, "gmp debug: calls with exactly %d prints", i);
      need(nm, cut_at[i]);
    }
  }
  need("gmp debug: step 1 executed", g_g.t_step1);
  need("gmp debug: sensitive path", g_g.sens);
}

/* boundary grid: rate x (limit relation) x flag x ...; deterministic enumerations that the random
 * scenario reaches only by chance */
static void scen_gmp_grid(void) {
  logger_t lg = {"getMPrecvdBits-grid", FNV0, 0, 997};
  static const int32_t rates[] = {0,     1,      0x960,  0x8000, 0x833e,  0x833f,     0x8340,     0x8341,     0x9000,     0x7fffffff,
                                  -1,    -0x960, -0x8000, (int32_t)0x80000000, (int32_t)0x80000001, 0x1fff, 0x2000, 0x3fff, 0x4000, 0x5b00,
                                  0x1234, 0x4ab0, 0x7ff0};
  static const int32_t f24cs[] = {0, 1, 2, 3, -1, 0x10000};
  int ri, fi, li, k;
  gmp_alloc();
  ed_learn();
  memset(&g_g, 0, sizeof g_g);
  for (ri = 0; ri < (int)(sizeof rates / sizeof rates[0]); ri++)
    for (fi = 0; fi < 6; fi++)
      for (li = 0; li < 6; li++)
        for (k = 0; k < 4; k++) {
          /* run_gmp randomises everything; force the selected fields after its setup by
           * pre-seeding the LCG so the picks are irrelevant: we call the lower level directly */
          unsigned char *o = g_obj;
          unsigned int h, hs;
          int32_t rate = rates[ri];
          int32_t limit = (li == 0) ? rate / 0x960 : (li == 1) ? rate / 0x960 + 1 : (li == 2) ? rate / 0x960 - 1
                          : (li == 3) ? 0x1000000 : (li == 4) ? -5 : 0;
          fill_random(o, OBJSZ);
          scrub_slots(o);
          fill_random(g_ctx, CTXSZ);
          fill_random(g_cfg, CFGSZ);
          fill_random(g_P, PSZ);
          PTR(o, 0x3548) = g_ctx;
          PTRX_SET(g_ctx, 0x610c, g_cfg);
          PTR(o, 0xac3c) = g_P;
          I32(o, 0x24c) = f24cs[fi];
          I32(g_ctx, 0x6120) = (k & 1) ? 0x10000 : 1;
          I32(g_cfg, 0x4f8) = (k & 2) ? 0x10000 : 1;
          I32(g_cfg, 0x4fc) = limit;
          I32(g_P, 0x3c) = rate;
          U16(o, 0xaa3c) = (uint16_t)((ri * 7 + fi + li) & 1 ? 0xffff : 0);
          U8(g_ctx, CTX_MP) = (uint8_t)((ri + k) & 1);
          U8(g_ctx, CTX_MP + 1) = (uint8_t)(ri * 5 + li);
          U8(g_ctx, CTX_MP + 2) = (uint8_t)(fi + k);
          U8(g_ctx, CTX_MP + 3) = (uint8_t)(li & 1);
          U8(g_ctx, CTX_MP + 4) = (uint8_t)(ri & 1);
          U8(g_ctx, CTX_MP + 5) = (uint8_t)((ri + li) & 1);
          memcpy(g_snap, o, OBJSZ);
          memcpy(g_ctx_s, g_ctx, CTXSZ);
          memcpy(g_cfg_s, g_cfg, CFGSZ);
          memcpy(g_P_s, g_P, PSZ);
          ext_clear();
          ext_reg(g_ctx, CTXSZ);
          ext_reg(g_cfg, CFGSZ);
          ext_reg(g_P, PSZ);
          call_gmp(o);
          h = diff_obj(o, g_snap);
          {
            const unsigned int ctxslot[1] = {0x610c};
            unsigned int hc = diff_buf(g_ctx, g_ctx_s, CTXSZ, o, ctxslot, 1);
            unsigned int hf = diff_buf(g_cfg, g_cfg_s, CFGSZ, o, NULL, 0);
            unsigned int hp = diff_buf(g_P, g_P_s, PSZ, o, NULL, 0);
            h = fnv(fnv(fnv(h, &hc, 4), &hf, 4), &hp, 4);
          }
          hs = ed_state_hash();
          h = fnv(h, &hs, 4);
          log_call2(&lg, h, (long)rate, (long)limit, (long)U16(o, 0xaa3c));
        }
  log_done(&lg);
  need("gmp grid: calls", lg.calls);
}

/* ============================================================================
 * v34modeminit
 * ========================================================================== */
static struct {
  long calls, arm65, arm_other, hi_garbage_65, hi_garbage_other, v0165, v8065, v0064, v0066;
  long fin4, fin5, bpv_high, bpv_low, det_c2400, det_c1200, rx8_set, rx8_clr, f386_hi_set, old25c2_bits;
  long old_dc0, ptr418_ok, prints_seen;
} g_m;

static void run_modeminit(logger_t *lg, int variant) {
  unsigned char *o = g_obj;
  unsigned int h, hs;
  uint32_t w359c;
  seed_echo_obj(o);
  switch (variant % 10) {
    case 0: w359c = 0x65; break;
    case 1: w359c = 0x65 | (lcg_next() & 0xffff0000u) | 0x10000u; break; /* upper-half garbage, low half 0x65 */
    case 2: w359c = (lcg_next() & 0xffffu); if (w359c == 0x65) w359c = 0x66; w359c |= (lcg_next() & 0xffff0000u); break;
    case 3: w359c = 0x0165; break;
    case 4: w359c = 0x8065; break;
    case 5: w359c = 0x0064; break;
    case 6: w359c = 0x0066; break;
    case 7: w359c = 0x65 | 0x80000000u; break;
    case 8: w359c = 0; break;
    default: w359c = lcg_next(); break;
  }
  U32(o, 0x359c) = w359c; /* low half = the 16-bit word the code compares; upper half = garbage it must ignore */
  memcpy(g_snap, o, OBJSZ);
  ext_clear();
  g_nprint = 0;
  v34modeminit(o);
  h = diff_obj(o, g_snap);
  hs = ed_state_hash();
  h = fnv(h, &hs, 4);
  log_call2(lg, h, (long)(w359c & 0xffff), (long)U16(o, 0x25c2), (long)U16(o, 0x386));

  g_m.calls++;
  if ((w359c & 0xffff) == 0x65) g_m.arm65++; else g_m.arm_other++;
  if ((w359c & 0xffff) == 0x65 && (w359c >> 16)) g_m.hi_garbage_65++;
  if ((w359c & 0xffff) != 0x65 && (w359c >> 16)) g_m.hi_garbage_other++;
  if (w359c == 0x0165) g_m.v0165++;
  if (w359c == 0x8065) g_m.v8065++;
  if (w359c == 0x0064) g_m.v0064++;
  if (w359c == 0x0066) g_m.v0066++;
  if (U16(o, 0x25c2) == 4) g_m.fin4++;
  if (U16(o, 0x25c2) == 5) g_m.fin5++;
  if (norm_val(o, U32(o, 0x508)) == tbl_id("bpv22high")) g_m.bpv_high++;
  if (norm_val(o, U32(o, 0x508)) == tbl_id("bpv22low")) g_m.bpv_low++;
  if (norm_val(o, U32(o, 0x3564)) == tbl_id("c2400_")) g_m.det_c2400++;
  if (norm_val(o, U32(o, 0x3564)) == tbl_id("c1200_")) g_m.det_c1200++;
  if (norm_val(o, U32(o, 0x418)) == tbl_id("hsine1800")) g_m.ptr418_ok++;
  if (U16(g_snap, 0x386) & 8) g_m.rx8_set++; else g_m.rx8_clr++;
  if (U16(g_snap, 0x386) & 0xf000) g_m.f386_hi_set++;
  if (U16(g_snap, 0x25c2) & 0xfff8) g_m.old25c2_bits++;
  if (U16(g_snap, 0x4c6) != 0 && U16(o, 0x39a) == U16(g_snap, 0x4c6)) g_m.old_dc0++;
}

static void modeminit_report(void) {
  need("modeminit [0x359c]==0x65 arm", g_m.arm65);
  need("modeminit [0x359c]!=0x65 arm", g_m.arm_other);
  need("modeminit 0x65 with garbage in upper half (16-bit compare)", g_m.hi_garbage_65);
  need("modeminit !=0x65 with garbage in upper half", g_m.hi_garbage_other);
  need("modeminit 0x0165", g_m.v0165);
  need("modeminit 0x8065", g_m.v8065);
  need("modeminit 0x0064", g_m.v0064);
  need("modeminit 0x0066", g_m.v0066);
  need("modeminit final 0x25c2 == 4", g_m.fin4);
  need("modeminit final 0x25c2 == 5", g_m.fin5);
  need("modeminit bpv22high pointer stored", g_m.bpv_high);
  need("modeminit bpv22low pointer stored", g_m.bpv_low);
  need("modeminit detector table c2400_", g_m.det_c2400);
  need("modeminit detector table c1200_", g_m.det_c1200);
  need("modeminit hsine1800 pointer stored", g_m.ptr418_ok);
  need("modeminit initial 0x386 bit 3 set (rxinit arm)", g_m.rx8_set);
  need("modeminit initial 0x386 bit 3 clear (rxinit arm)", g_m.rx8_clr);
  need("modeminit initial 0x386 high bits set", g_m.f386_hi_set);
  need("modeminit old 0x25c2 had bits outside 4/5 (whole-word store)", g_m.old25c2_bits);
  need("modeminit 0x39a copy of 0x4c6 non-trivial", g_m.old_dc0);
}

static void scen_modeminit(void) {
  logger_t lg = {"v34modeminit", FNV0, 0, 499};
  int c;
  g_obj = xalloc(OBJSZ);
  g_snap = xalloc(OBJSZ);
  ed_learn();
  memset(&g_m, 0, sizeof g_m);
  for (c = 0; c < 30000; c++) run_modeminit(&lg, c);
  log_done(&lg);
  modeminit_report();
}

static void scen_modeminit_debug(void) {
  logger_t lg = {"v34modeminit-debug", FNV0, 0, 1};
  long printed_full = 0, quiet = 0, cut = 0, max_prints = 0;
  int c;
  g_obj = xalloc(OBJSZ);
  g_snap = xalloc(OBJSZ);
  ed_learn();
  memset(&g_m, 0, sizeof g_m);
  for (c = 0; c < 1500; c++) {
    unsigned int lvl = k_levels[c % 6];
    int flip = (c / 6) % 16; /* 0..11 flip after that print, 12..15 none */
    static const unsigned int to[3] = {0u, 1u, 0x80000000u};
    long before = g_total_prints;
    int prints;
    if (flip <= 11) {
      g_flip_after = flip;
      g_flip_to = to[(c / 96) % 3];
    } else {
      g_flip_after = -1;
    }
    dsplibs_debug_level = lvl;
    printf("-- call %d level=%u flip=%d\n", c, lvl, g_flip_after);
    run_modeminit(&lg, c);
    dsplibs_debug_level = 0;
    g_flip_after = -1;
    prints = (int)(g_total_prints - before);
    if (prints == 0) quiet++;
    if (prints > max_prints) max_prints = prints;
    if (prints > 0 && g_flip_after < 0) { /* placeholder to keep counters simple */ }
  }
  (void)printed_full;
  (void)cut;
  log_done(&lg);
  need("modeminit debug: quiet calls", quiet);
  need("modeminit debug: max prints in a call", max_prints);
  need("modeminit debug: arm65", g_m.arm65);
  need("modeminit debug: arm other", g_m.arm_other);
}

/* ============================================================================
 * scenario 6: does anything modeminit calls read/write obj+0x25c2?
 * ========================================================================== */
typedef void (*probe_fn_t)(unsigned char *o, int trial);
static void pf_preinitdigital(unsigned char *o, int t) { (void)t; preinitdigital(o); }
static void pf_txinit(unsigned char *o, int t) { (void)t; txinit(o); }
static void pf_rxinit(unsigned char *o, int t) { (void)t; rxinit(o); }
static void pf_setupmod_a(unsigned char *o, int t) { (void)t; V34SetupModulator(OB(o, 0x1450), 0x960, 0x708, 0, 0, 0); }
static void pf_setupmod_b(unsigned char *o, int t) {
  V34SetupModulator(OB(o, 0x1450), 600, (t & 1) ? 0x4b0 : 0x960, 0, 0, 1);
}
static void pf_detector(unsigned char *o, int t) {
  detectorinit(OB(o, 0x3564), V34PW((t & 1) ? c2400_ : c1200_), 0, 200, 0x32, 0x600, 0);
}
static void pf_settxscale(unsigned char *o, int t) { (void)t; VPcmV34SetTxScale(o); }
static void pf_modeminit(unsigned char *o, int t) { (void)t; v34modeminit(o); }

static void scen_probe25c2(void) {
  static const struct {
    const char *name;
    probe_fn_t fn;
  } fns[] = {{"preinitdigital", pf_preinitdigital}, {"txinit", pf_txinit},         {"rxinit", pf_rxinit},
             {"V34SetupModulator(2400,fr=0)", pf_setupmod_a}, {"V34SetupModulator(600,fr=1)", pf_setupmod_b},
             {"detectorinit", pf_detector},         {"VPcmV34SetTxScale", pf_settxscale}, {"v34modeminit", pf_modeminit}};
  size_t f;
  int t;
  long total_indep = 0;
  g_obj = xalloc(OBJSZ);
  g_snap = xalloc(OBJSZ);
  ed_learn();
  for (f = 0; f < sizeof fns / sizeof fns[0]; f++) {
    long indep = 0, untouched = 0, n = 0;
    unsigned int all = FNV0;
    for (t = 0; t < 400; t++) {
      unsigned int hv[3];
      static const uint16_t vals[3] = {0x0000, 0xffff, 0x5a3c};
      int v;
      seed_echo_obj(g_obj);
      U32(g_obj, 0x359c) = lcg_next();
      if (t % 3 == 0) U16(g_obj, 0x359c) = 0x65;
      memcpy(g_snap, g_obj, OBJSZ);
      for (v = 0; v < 3; v++) {
        int changed;
        memcpy(g_obj, g_snap, OBJSZ);
        U16(g_obj, 0x25c2) = vals[v];
        fns[f].fn(g_obj, t);
        changed = U16(g_obj, 0x25c2) != vals[v];
        /* v34modeminit is expected to WRITE 0x25c2 (4 or 5, independent of the old value); for every
         * other callee the word must be untouched */
        if (!strcmp(fns[f].name, "v34modeminit")) {
          if (U16(g_obj, 0x25c2) != 4 && U16(g_obj, 0x25c2) != 5) die("modeminit 0x25c2 not 4/5");
          changed = 0;
        }
        if (changed) die("a callee modified obj+0x25c2");
        untouched++;
        U16(g_obj, 0x25c2) = 0;
        hv[v] = hash_obj(g_obj);
      }
      n++;
      if (hv[0] == hv[1] && hv[1] == hv[2]) indep++;
      all = fnv(all, &hv[0], 4);
    }
    printf("probe %s: trials=%ld independent-of-0x25c2=%ld word-untouched=%ld all=%08x\n", fns[f].name, n, indep, untouched, all);
    if (indep != n) {
      fprintf(stderr, "PROBE FAILURE: %s depends on obj+0x25c2\n", fns[f].name);
      g_fail = 1;
    }
    total_indep += indep;
  }
  need("25c2 probe: independent trials", total_indep);
}

int main(int argc, char **argv) {
  int s = argc > 1 ? atoi(argv[1]) : -1;
  g_verbose = getenv("V34B_VERBOSE") ? atoi(getenv("V34B_VERBOSE")) : 0;
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
    case 0: scen_txscale(); break;
    case 1: scen_cencode(); break;
    case 2: scen_gmp(); break;
    case 3: scen_gmp_debug(); break;
    case 4: scen_modeminit(); break;
    case 5: scen_modeminit_debug(); break;
    case 6: scen_probe25c2(); break;
    case 7: scen_gmp_grid(); break;
    default: fprintf(stderr, "usage: %s <scenario 0-7>\n", argv[0]); return 1;
  }
  fflush(stdout);
  return g_fail ? 2 : 0;
}
