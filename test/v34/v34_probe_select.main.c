/*
 * Differential test harness for V.34 "tier 8a" (src/v34/v34_probe_select.c):
 *   probeselect (6,173 B), chkForceBaudRate (389 B)
 *
 * One source, two binaries (same convention as test/vpcm/v34_modem_init.main.c):
 *   v34_tier8a_test_blob - linked against the real dsplibs.o (-DTEST_BLOB)
 *   v34_tier8a_test_port - linked against src/v34/v34_probe_select.c + src/v34/v34_agc_demod.c (bitreverse) + tier-5 tables
 * The ctx buffer is the i386 image of a VPcmFloModem (flags at +0x217) on the blob side; at x86-64 it is registered as a ctx image (test/v34/v34_img.h), so
 * the port sees a native-layout object (flags at +0x21b) marshalled from and back to the image around each call.
 * Each scenario runs in its OWN process (argv[1]).
 *
 * Which callees are real: in the blob binary probeselect calls the BLOB's own bitreverse and
 * chkForceBaudRate; in the port binaries the verified port's bitreverse (src/v34/v34_agc_demod.c) and the
 * PORT's chkForceBaudRate (which this test also checks DIRECTLY, scenario 3).  The blob's tables
 * (scale*, c1600..c2000 - the addresses probeselect stores into obj+0xaa90/0xaaac/0xaab0) are
 * memcmp'ed against the port's extraction (linked with a `port_` prefix) at start-up.
 *
 * Objects: tagV34Object is not a struct.  A random fake object (OBJSZ = 0xb400 bytes) is generated per
 * call; the result is the hash of the DIFF of the object, the ctx buffer and the P buffer (word index +
 * new value of every changed word, so a stray write anywhere is caught) plus the hash of every debug
 * print (formatted text) - raw addresses are never compared (in-object / external-buffer / table pointer
 * slots are normalised).  The two harness-owned pointer slots (obj+0x3548 ctx, obj+0xac3c P) point at
 * heap buffers registered as external regions; the three slots probeselect writes (obj+0xaa90, 0xaaac,
 * 0xaab0) hold table addresses and are normalised to (table id, offset).
 *
 * INDEPENDENT MODEL: `model()` below re-implements probeselect + chkForceBaudRate from scratch in the
 * structure of the Ghidra decompile (goto-free, table-driven arms, 64-bit arithmetic where the blob wraps
 * 32 bits explicitly) - NOT from the port source.  After every call the model's post-state of the WHOLE object
 * (and ctx) is compared word by word with the real result (exit 3 on any disagreement); the coverage
 * counters are derived from the model (input predicates), never from the code under test.  Debug prints
 * are covered by the blob-vs-port diff (the model has no prints).
 *
 * Scenarios: 0 random single calls (level 0), 1 random at debug levels (prints hashed), 2 debug-level flips
 * (print hook lowers the level after the k-th print: cached-vs-fresh level reads), 3 chkForceBaudRate direct,
 * 4 cascade boundary sweep (rec values 4..12 x mode x flag bits), 5 pre-emphasis steering (thr on the
 * trajectory: every index 6..10 + not found, both sides of `>`), 6 SNR / gain exhaustive grid at level 2,
 * 7 sensitive-RX / power-reduction branch classes at levels 0/2, 8 repeated calls on one persistent object.
 * Every scenario keeps COVERAGE COUNTERS and exits 2 if a branch class was never reached.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v34_img.h" /* T04: x86-64 image <-> native V34Obj marshalling; empty at -m32 */

extern unsigned int dsplibs_debug_level;

/* ---- functions under test / callees ---- */
extern void probeselect(void *obj);
extern void chkForceBaudRate(void *obj, void *R);
extern unsigned short bitreverse(unsigned short x, short nbits);

/* ---- tables ---- */
#define TBLS(X)                                                                                              \
  X(scale2400, 56) X(scale2800, 56) X(scale3000, 56) X(scale3200, 56) X(scale3429, 56) X(c1600, 16)          \
  X(c1680, 16) X(c1800_, 16) X(c1829, 16) X(c1867, 16) X(c1920, 16) X(c1959, 16) X(c2000, 16)

#define X_DECL(n, sz) extern const short n[];
TBLS(X_DECL)
#undef X_DECL
#ifdef TEST_BLOB
#define X_DECLP(n, sz) extern const short port_##n[];
TBLS(X_DECLP)
#undef X_DECLP
#endif

typedef struct {
  const char *name;
  const unsigned char *addr;
  size_t size;
} tbl_t;
static tbl_t g_tbl[32];
static int g_ntbl;
static void add_tbl(const char *name, const void *addr, size_t size) {
  g_tbl[g_ntbl].name = name;
  g_tbl[g_ntbl].addr = (const unsigned char *)addr;
  g_tbl[g_ntbl].size = size;
  g_ntbl++;
}
static void tables_init(void) {
#ifdef TEST_BLOB
#define X_REG(n, sz)                                                                                         \
  add_tbl(#n, (const void *)n, sz);                                                                          \
  if (memcmp((const void *)n, (const void *)port_##n, sz)) { fprintf(stderr, "table %s: blob != port copy\n", #n); exit(3); }
#else
#define X_REG(n, sz) add_tbl(#n, (const void *)n, sz);
#endif
  TBLS(X_REG)
#undef X_REG
}

/* ---- accessors ---- */
/* V34SHIMS BEGIN (generated by tools/tier_rewrite.py shims; do not edit) */
#if defined(__x86_64__)
static void V34SH_probeselect(void *obj) { v34i_enter(); probeselect((void *)v34i_in((void *)obj)); v34i_leave(); }
static void V34SH_chkForceBaudRate(void *obj, void *R) { v34i_enter(); chkForceBaudRate((void *)v34i_in((void *)obj), (void *)v34i_in((void *)R)); v34i_leave(); }
static void *const V34REAL_probeselect = (void *)probeselect; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_chkForceBaudRate = (void *)chkForceBaudRate; /* the real address (the name is a macro for the shim below) */
#define probeselect V34SH_probeselect
#define chkForceBaudRate V34SH_chkForceBaudRate
#endif
/* V34SHIMS END */
#define OB(o, off) ((unsigned char *)(o) + (off))
#define U8(o, off) (*(uint8_t *)OB(o, off))
#define S16(o, off) (*(int16_t *)OB(o, off))
#define U16(o, off) (*(uint16_t *)OB(o, off))
#define S32(o, off) (*(int32_t *)OB(o, off))
#define U32(o, off) (*(uint32_t *)OB(o, off))
#if defined(__x86_64__)
#define PTR(o, off) (*(uint32_t *)OB(o, off)) /* x86-64: the image keeps a 32-bit slot (low memory address), marshalled to a real pointer (test/v34/v34_img.h) */
#else
#define PTR(o, off) (*(void **)OB(o, off))
#endif

#define OBJSZ 0xb400u
#define CTXSZ 0x300u
#define PSZ 0x80u
#define RB 0xa320 /* R = obj + RB */

/* ---- deterministic LCG with a bijective finaliser ---- */
static unsigned int g_lcg = 0x8a3d51c9u;
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
static int lcg_pct(int pct) { return (int)lcg_mod(100) < pct; }

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

/* bulk filler: xorshift64 seeded from the LCG per call (fast; the LCG's finaliser is used for every steering decision) */
static void fill_random(void *p, size_t n) {
  unsigned char *b = (unsigned char *)p;
  uint64_t x = ((uint64_t)lcg_next() << 32) | lcg_next() | 1u;
  size_t i;
  for (i = 0; i + 8 <= n; i += 8) {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    memcpy(b + i, &x, 8);
  }
  for (; i < n; i++) {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    b[i] = (unsigned char)(x >> 24);
  }
}

static int g_fail = 0;
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
  return p;
}

/* ---- pointer normalisation ---- */
static unsigned char *g_obj, *g_snap, *g_ctx, *g_ctxs, *g_P, *g_Ps, *g_mobj, *g_mctx;
static uint32_t norm_val(uint32_t v) {
  int i;
  uintptr_t a = (uintptr_t)v;
  if (a >= (uintptr_t)g_obj && a < (uintptr_t)g_obj + OBJSZ) return 0x80000000u | (uint32_t)(a - (uintptr_t)g_obj);
  if (a >= (uintptr_t)g_ctx && a < (uintptr_t)g_ctx + CTXSZ) return 0x7d000000u | (uint32_t)(a - (uintptr_t)g_ctx);
  if (a >= (uintptr_t)g_P && a < (uintptr_t)g_P + PSZ) return 0x7c000000u | (uint32_t)(a - (uintptr_t)g_P);
  for (i = 0; i < g_ntbl; i++)
    if (a >= (uintptr_t)g_tbl[i].addr && a < (uintptr_t)g_tbl[i].addr + g_tbl[i].size)
      return 0x7e000000u | ((uint32_t)i << 16) | (uint32_t)(a - (uintptr_t)g_tbl[i].addr);
  return v;
}
static uint32_t tbl_norm(const void *t) { return norm_val((uint32_t)(uintptr_t)t); }

static const unsigned int k_slots[] = {0x3548, 0xac3c, 0xaa90, 0xaaac, 0xaab0};
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
      a = norm_val(a);
      b = norm_val(b);
    }
    if (a != b) {
      uint32_t idx = (uint32_t)i;
      h = fnv(h, &idx, 4);
      h = fnv(h, &a, 4);
    }
  }
  return h;
}
static unsigned int diff_buf(const unsigned char *buf, const unsigned char *snap, size_t sz) {
  unsigned int h = FNV0;
  size_t i;
  for (i = 0; i + 4 <= sz; i += 4) {
    uint32_t a, b;
    memcpy(&a, buf + i, 4);
    memcpy(&b, snap + i, 4);
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
static void log_call(logger_t *l, unsigned int h, long a, long b, long c) {
  l->all = fnv(l->all, &h, 4);
  if (l->calls % l->every == 0) printf("%s c=%ld a=%ld b=%ld c2=%ld hash=%08x\n", l->name, l->calls, L32(a), L32(b), L32(c), h);
  l->calls++;
}
static void log_done(logger_t *l) { printf("%s calls=%ld all=%08x\n", l->name, l->calls, l->all); }

/* ============================================================================
 * the debug printer: hashes every formatted line, counts per format, optional echo,
 * level-flip hook (after the k-th print of a call the level is set to g_flip_to)
 * ========================================================================== */
/* hash of everything probeselect / chkForceBaudRate write, taken at every print (slots normalised) */
static unsigned int state_hash(void) {
  unsigned int h = FNV0;
  size_t off;
  if (!g_obj) return h;
  h = fnv(h, g_obj + 0x39a, 2);
  h = fnv(h, g_obj + 0x4c6, 2);
  for (off = 0xa320; off < 0xa76c; off += 2) h = fnv(h, g_obj + off + 0, 2); /* the 25 records (a few 16-bit words at a time) */
  h = fnv(h, g_obj + 0xa9ac, 0x14);
  for (off = 0xaa84; off < 0xaab4; off += 4) {
    uint32_t w;
    memcpy(&w, g_obj + off, 4);
    if (off == 0xaa90 || off == 0xaaac || off == 0xaab0) w = norm_val(w);
    h = fnv(h, &w, 4);
  }
  h = fnv(h, g_ctx + 0x217, 8);
  return h;
}

static const char *k_fmts[] = {
    /*0*/ "V34PROBE, snr_L1=%d , snr_L2=%d , L2toL1ratio=%d  (all not in dB)\n",
    /*1*/ "V34PROBE, dBcnt=%d , powerReductionReq=%d , gain=%d\n",
    /*2*/ "Sensitive RX Power Reduction mechanism disabled!\r\n",
    /*3*/ "Sensitive RX Power Reduction mechanism enabled!\r\n",
    /*4*/ "V34PROBE, asking for a power reduction of %d\n",
    /*5*/ "V34PROBE, agc gainestimate of L1 signal is %d\n",
    /*6*/ "V34PROBE, agc gainestimate due to power reduction request is %d\n",
    /*7*/ "V34PROBE, not asking for power reduction\n",
    /*8*/ "V34PROBE, rx->gain=%d ,(obj->rxinfo0.data[1]&0x80)=%d\n",
    /*9*/ "V34PROBE,0=%d,%d,%d,%d,%d,5=%d,%d,%d,%d,%d,10=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,20=%d,%d,22=%d,%d,24=%d\n",
    /*10*/ "V34PROBE, min = %d, i= %d, so reducing band edge norm by 2\n",
    /*11*/ "V34PROBE, min = %d, i= %d, so reducing band edge norm by 1\n",
    /*12*/ "V34PREEMPHASIS, - index is %d, baudrate= %d\n",
    /*13*/ "V34PREEMPHASIS, - index is 10, baudrate= %d \n",
    /*14*/ "V34PREEMPHASIS, - index is 0, baudrate= %d\n",
    /*15*/ "VpcmV34Main: max V34 baud rate index = %d\r\n"};
#define NFMT (sizeof k_fmts / sizeof k_fmts[0])
static long g_fmt_count[NFMT];
static long g_unknown_fmt;
static unsigned int g_ph;        /* rolling hash of the prints of the current call */
static long g_np;                /* prints of the current call */
static long g_total_prints;
static int g_echo;               /* echo the formatted lines to stdout */
static int g_flip_after = -1;    /* after the k-th (0-based) print of a call, set the level to g_flip_to */
static unsigned int g_flip_to;
static int g_nprint_call;

int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[1024];
  va_list ap;
  size_t i;
  int len;
  va_start(ap, fmt);
  len = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  for (i = 0; i < NFMT; i++)
    if (fmt == k_fmts[i] || !strcmp(fmt, k_fmts[i])) break;
  if (i == NFMT) g_unknown_fmt++;
  else g_fmt_count[i]++;
  g_ph = fnv(g_ph, buf, (size_t)len);
  V34I_SYNC_IMG(); /* x86-64: the port works on a SHADOW; bring the image up to date before it is hashed */
  g_ph = fnv(g_ph, &(unsigned int){state_hash()}, 4); /* the object / ctx as they are AT the print: store-vs-print ORDER is compared */
  g_np++;
  g_total_prints++;
  if (g_echo) fputs(buf, stdout);
  if (g_flip_after >= 0 && g_nprint_call == g_flip_after) dsplibs_debug_level = g_flip_to;
  g_nprint_call++;
  return len;
}

/* ============================================================================
 * the INDEPENDENT MODEL (Ghidra-decompile structure; see the header)
 * ========================================================================== */
enum {
  C_S2_L1ZERO, C_S2_NEGL2, C_S2_CNTGT9, C_S2_NORMAL, C_S2_DZERO, C_S3_DBC1000, C_S3_DBCLOOP, C_S3_GAIN0,
  C_S3_CLAMP, C_S3_NOCLAMP, C_PRR_NEG, C_PRR_ZERO, C_PRR_1_7, C_PRR_GT7,
  C_SENS_DIS, C_SENS_EN, C_BR_A, C_BR_B, C_BR_REGULAR, C_BR_NOASK_FLAG, C_BR_NOASK_GAIN, C_BR_NOASK_PRR,
  C_REG_CLAMP7000, C_S5_MINNEG, C_S5_MINZEROEXT, C_S5_RED2, C_S5_RED1, C_S5_NONE, C_S5_I0,
  C_CHK_FORCE, C_CHK_CTXFLAGS, C_CHK_LOCALF5_1, C_CHK_LOCALF5_0, C_CHK_E5, C_CHK_E4, C_CHK_E3, C_CHK_E2,
  C_K_0, C_K_1, C_K_2, C_K_3, C_K_4, C_K_5, C_K_6, C_K_7,
  C_ARM_3429N, C_ARM_3429_65, C_ARM_3200N, C_ARM_3200_65, C_ARM_3000N, C_ARM_3000_65, C_ARM_2800N,
  C_ARM_2800_65, C_ARM_2400_65, C_ARM_2400N, C_ARM_2400_RET0, C_ARM_NONE, C_FORCED, C_TO_61643,
  C_PE_6, C_PE_7, C_PE_8, C_PE_9, C_PE_10, C_PE_TOTAL,
  C_A_REC1_GT5, C_A_REC23_GT6, C_A_REC24_GT10,
  C_ARM3200_65_REC23_LE5, C_ARM3200_65_REC23_GT7, C_ARM3000_65_REC22_LE5, C_ARM3000_65_REC22_EQ7, C_ARM3000_65_REC22_GT7,
  C_ARM2800_65_OR6, C_S5_SHIFT0, C_S5_SHIFTNZ, C_ARM3200N_REC23_GT5, C_ARM3000N_C2000, C_ARM2800N_C1867,
  C_ARM3200N_B40, C_ARM3000N_B80, C_ARM2800N_B1, C_ARM2400N_B4, C_ARM2400_65_NEWLINE,
  C_LVL_ON, C_LVL_OFF, C_REC24_NEG, C_REC23_NEG, C_REC1_NEG, C_REC22_NEG, C_REC2_NEG, C_REC20_NEG, C_REC19_NEG, C_REC21_NEG,
  C_CHAIN2, C_CHAIN3, C_CHAIN4, C_CHAIN5,
  C_COUNT
};
static const char *k_cnames[C_COUNT] = {
    "s2 l1==0", "s2 l2 negative", "s2 cnt>9 (division by l1)", "s2 cnt<=9", "s2 d==0", "s3 dBcnt=1000", "s3 dBcnt loop", "s3 gain==0",
    "s3 quotient clamped 0x4000", "s3 quotient not clamped", "prr<0", "prr==0", "prr 1..7", "prr>7",
    "sensitive disabled", "sensitive enabled", "branch A (l1<500)", "branch B (500..999)", "regular reduction", "not asking: flag clear",
    "not asking: gain>0xfff", "not asking: prr==0", "regular agc clamp 7000", "s5 min negative (zero-ext store)", "s5 min stored zero-extended",
    "s5 reduce by 2", "s5 reduce by 1", "s5 no reduction", "s5 i==0 -> 1", "chk: obj+0x24c != 0 (ctx flags)", "chk: ctx flag bytes used", "chk: local f5=1", "chk: local f5=0",
    "chk e5==0", "chk e4==0", "chk e3==0", "chk e2==0",
    "k=0", "k=1", "k=2", "k=3", "k=4", "k=5", "k=6", "k=7",
    "arm 3429 non-0x65", "arm 3429 0x65", "arm 3200 non-0x65", "arm 3200 0x65", "arm 3000 non-0x65", "arm 3000 0x65", "arm 2800 non-0x65",
    "arm 2800 0x65", "arm 2400 0x65", "arm 2400 non-0x65", "arm 2400 non-0x65 return (0x3c==0)", "no arm at all", "forced (0x359a != 0)", "->0x61643 by gate",
    "pe idx 6", "pe idx 7", "pe idx 8", "pe idx 9", "pe idx 10 (not found)", "pe searches",
    "rec1>5 gate", "rec23>6 gate", "rec24>10 gate",
    "3200/65 rec23<=5", "3200/65 rec23>7", "3000/65 rec22<=5", "3000/65 rec22==7", "3000/65 rec22>7",
    "2800/65 p6|=1", "s5 shift==0", "s5 shift!=0", "3200n rec23>5", "3000n c2000", "2800n c1867",
    "3200n bit0x40", "3000n bit0x80", "2800n bit1", "2400n bit4", "2400/65 (unused)", "level on", "level off", "rec24<0 at cascade", "rec23<0", "rec1<0", "rec22<0", "rec2<0", "rec20<0", "rec19<0", "rec21<0",
    "0x65 chain: >=2 fall-through arms + 2400", "0x65 chain: >=3", "0x65 chain: >=4", "0x65 chain: all 5 (3429,3200,3000,2800 + 2400)"};

typedef struct {
  long c[C_COUNT];
} mcov_t;
static mcov_t g_cov;
#define COV(id) (g_cov.c[id]++)

static int m_wrap16(int64_t v) { return (int16_t)v; }
static int m_pe2(const unsigned char *o, int koff, int mul) {
  int thr = S16(o, RB + 0xbc);
  int v = S16(o, RB + koff);
  int n;
  for (n = 1; n <= 5; n++) {
    v = m_wrap16(((int64_t)v * mul) >> 14);
    if (v > thr) {
      if (n == 1) COV(C_PE_6);
      else if (n == 2) COV(C_PE_7);
      else if (n == 3) COV(C_PE_8);
      else if (n == 4) COV(C_PE_9);
      else COV(C_PE_10);
      COV(C_PE_TOTAL);
      return 5 + n;
    }
  }
  COV(C_PE_10);
  COV(C_PE_TOTAL);
  return 10;
}

static void m_or16(unsigned char *o, int off, unsigned int v) { U16(o, off) = (uint16_t)(U16(o, off) | v); }

/* the chkForceBaudRate model (writes the clamped records, may write ctx bytes) */
static void m_chk(unsigned char *o, unsigned char *ctx, unsigned char *P) {
  unsigned char loc[6];
  unsigned char *fl;
  unsigned int k = (unsigned int)(U8(P, 0x50) >> 5);
  memset(loc, 0, sizeof loc);
  loc[0] = loc[1] = loc[2] = loc[3] = loc[4] = 1;
  COV(C_K_0 + (int)k);
  if (U32(o, 0x24c) != 0) {
    loc[5] = 1;
    fl = ctx + 0x217;
    COV(C_CHK_FORCE);
    COV(C_CHK_CTXFLAGS);
  } else {
    fl = loc;
    if (U32(o, 0x250) == 0) { loc[5] = 1; COV(C_CHK_LOCALF5_1); }
    else { loc[5] = 0; COV(C_CHK_LOCALF5_0); }
  }
  {
    int j;
    if (k != 0)
      for (j = 1; j <= 5; j++)
        if (k < (unsigned int)(j + 1)) fl[j] = 0; /* bVar2 < j+1  <=>  k <= j */
  }
  if (fl[5] == 0) { S16(o, RB + 0x42e) = 0xb; COV(C_CHK_E5); }
  if (fl[4] == 0) { S16(o, RB + 0x3d6) = 7; COV(C_CHK_E4); }
  if (fl[3] == 0) { S16(o, RB + 0x66) = 7; S16(o, RB + 0x3aa) = 7; COV(C_CHK_E3); }
  if (fl[2] == 0) { S16(o, RB + 0x37e) = 7; S16(o, RB + 0x352) = 7; COV(C_CHK_E2); }
}

/* returns the model's normalised pointer words through the object slots (table id form) */
static void m_setptr(unsigned char *o, int off, const void *t) { U32(o, off) = tbl_norm(t); }

static void model(unsigned char *o, unsigned char *ctx, unsigned char *P) {
  unsigned char *pw = o + 0xa9ac, *qw = o + 0xaa84;
  int i, mode, idx, n65 = 0;
  uint32_t uVar12 = U32(o, 0xaac4), uVar13 = 0, uVar5, uVar7;
  int iVar15, gain;
  int forced;

  for (i = 0; i < 10; i++) U16(pw, 2 * i) = 0;
  /* ---- S2 ---- */
  if (uVar12 != 0) {
    int cnt = 0;
    uVar7 = 0x80000000u;
    uVar5 = U32(o, 0xaac8);
    if ((int32_t)uVar5 >= 0) {
      do {
        cnt++;
        uVar7 >>= 1;
      } while ((uVar7 & uVar5) == 0 && cnt < 10);
      if (cnt > 9) {
        COV(C_S2_CNTGT9);
        uVar13 = (uint32_t)(uVar5 * 0x400u + (uVar12 >> 1)) / uVar12;
        goto s2done;
      }
    } else {
      COV(C_S2_NEGL2);
    }
    COV(C_S2_NORMAL);
    uVar7 = uVar12 >> ((10 - cnt) & 0x1f);
    uVar13 = 0;
    if (uVar7 != 0) uVar13 = ((uVar5 << (cnt & 0x1f)) + (uVar12 >> ((0xb - cnt) & 0x1f))) / uVar7;
    else COV(C_S2_DZERO);
  } else {
    COV(C_S2_L1ZERO);
  }
s2done:
  /* ---- S3 ---- */
  iVar15 = 0;
  if (uVar13 < 0x19000u) {
    uint32_t e = 0x47d;
    COV(C_S3_DBCLOOP);
    while (e < uVar13) {
      e = (e * 0x509u + 0x200u) >> 10;
      iVar15 = m_wrap16(iVar15 + 1);
    }
  } else {
    iVar15 = 1000;
    COV(C_S3_DBC1000);
  }
  uVar12 = 0;
  gain = S16(o, 0x39a);
  if (gain != 0) {
    int64_t q = ((int64_t)(gain >> 1) + 0x640000) / gain; /* C99 truncation toward zero, like idiv */
    uint32_t u = (uint32_t)q;
    if (u > 0x4000u) { u = 0x4000u; COV(C_S3_CLAMP); } else COV(C_S3_NOCLAMP);
    uVar12 = (uint32_t)(int32_t)(int16_t)((u * (uint32_t)iVar15) >> 14);
  } else {
    COV(C_S3_GAIN0);
  }
  if ((int16_t)uVar12 < 0) die("prr < 0: impossible (u <= 0x4000, dBcnt <= 1000)");
  else if ((int16_t)uVar12 == 0) COV(C_PRR_ZERO);
  else if ((int16_t)uVar12 <= 7) COV(C_PRR_1_7);
  else COV(C_PRR_GT7);
  /* ---- S4 ---- */
  {
    int sens = (U8(P, 0x50) & 0x10) != 0;
    int flag = (U8(o, 0xa97e) & 0x80) != 0;
    uint32_t l1 = U32(o, 0xaac4);
    int regular = 1;
    if (!sens) {
      COV(C_SENS_DIS);
    } else {
      COV(C_SENS_EN);
      if (l1 < 500 && flag) {
        int n;
        uint32_t v = U16(o, 0x4c6);
        COV(C_BR_A);
        m_or16(pw, 0, (uint32_t)(bitreverse(7, 3) & 0xffff) << 5);
        m_or16(pw, 0, (uint32_t)(bitreverse(2, 3) & 0xffff) << 2);
        for (n = 0; n < 9; n++) v = (uint32_t)(((int32_t)(int16_t)v * 0x47cf + 0x2000) >> 14);
        S16(o, 0x4c6) = (int16_t)v;
        regular = 0;
        goto noask; /* falls into the "not asking" block */
      } else if (!(l1 > 999 || !flag)) {
        int n;
        uint32_t v = U16(o, 0x4c6);
        COV(C_BR_B);
        m_or16(pw, 0, (uint32_t)(bitreverse(7, 3) & 0xffff) << 5);
        for (n = 0; n < 7; n++) v = (uint32_t)(((int32_t)(int16_t)v * 0x47cf + 0x2000) >> 14);
        S16(o, 0x4c6) = (int16_t)v;
        regular = 0;
        goto noask;
      }
    }
    if (regular) {
      if (!flag) { COV(C_BR_NOASK_FLAG); goto noask; }
      if (S16(o, 0x39a) > 0xfff) { COV(C_BR_NOASK_GAIN); goto noask; }
      if ((int16_t)uVar12 == 0) { COV(C_BR_NOASK_PRR); goto noask; }
      COV(C_BR_REGULAR);
      if ((int16_t)uVar12 > 7) uVar12 = 7;
      m_or16(pw, 0, (uint32_t)(bitreverse((uint16_t)uVar12, 3) & 0xffff) << 5);
      if ((int32_t)uVar12 > 0) {
        int n;
        uint32_t v = U16(o, 0x4c6);
        for (n = 0; n < (int)uVar12; n++) v = (uint32_t)(((int32_t)(int16_t)v * 0x47cf + 0x2000) >> 14);
        S16(o, 0x4c6) = (int16_t)v;
      }
      if (S16(o, 0x4c6) > 7000) { S16(o, 0x4c6) = 7000; COV(C_REG_CLAMP7000); }
    }
  }
noask:
  /* ---- S5 ---- */
  {
    int mn = 0x20;
    for (i = 0; i < 25; i++) {
      uint16_t c = U16(o, RB + 0xe + 0x2c * i);
      if ((int16_t)c < mn) mn = c;
    }
    if (mn < 0) COV(C_S5_MINNEG);
    if (mn > 0x20) COV(C_S5_MINZEROEXT);
    if (mn == 0) COV(C_S5_SHIFT0); else COV(C_S5_SHIFTNZ);
    for (i = 0; i < 25; i++) S16(o, RB + 0xe + 0x2c * i) = (int16_t)(U16(o, RB + 0xe + 0x2c * i) - mn);
    {
      uint32_t isum = (uint16_t)(S16(o, RB + 0xe + 0x2c * 2) + S16(o, RB + 0xe + 0x2c * 3) + S16(o, RB + 0xe + 0x2c * 4) + S16(o, RB + 0xe + 0x2c * 6));
      uint32_t mnn = (uint16_t)(S16(o, RB + 0xe + 0x2c * 12) + S16(o, RB + 0xe + 0x2c * 13) + S16(o, RB + 0xe + 0x2c * 14) + S16(o, RB + 0xe + 0x2c * 16));
      if (isum == 0) { isum = 1; COV(C_S5_I0); }
      if ((int64_t)mnn > 4 * (int64_t)isum) {
        COV(C_S5_RED2);
        for (i = 16; i < 24; i++)
          if (S16(o, RB + 0xe + 0x2c * i) > 2) S16(o, RB + 0xe + 0x2c * i) = (int16_t)(S16(o, RB + 0xe + 0x2c * i) - 2);
      } else if ((int64_t)mnn > 2 * (int64_t)isum) {
        COV(C_S5_RED1);
        for (i = 16; i < 24; i++)
          if (S16(o, RB + 0xe + 0x2c * i) > 1) S16(o, RB + 0xe + 0x2c * i) = (int16_t)(S16(o, RB + 0xe + 0x2c * i) - 1);
      } else {
        COV(C_S5_NONE);
      }
    }
  }
  /* ---- S6 ---- */
  m_chk(o, ctx, P);
  forced = U16(o, 0x359a) != 0;
  mode = U16(o, 0x359c);

  /* ---- S7: the cascade (Ghidra structure: LAB_00071194 / 00071643 / 000716a0 / 0007168f / 000714d2) ---- */
#define REC(k) ((int)S16(o, RB + 0xe + 0x2c * (k)))
  if (!forced) {
    if (REC(24) < 0) die("rec24 < 0 after S5: impossible (mn is the LAST record once a negative one was seen)");
    if (REC(23) < 0) COV(C_REC23_NEG);
    if (REC(1) < 0) COV(C_REC1_NEG);
    if (REC(22) < 0) COV(C_REC22_NEG);
    if (REC(2) < 0) COV(C_REC2_NEG);
    if (REC(20) < 0) COV(C_REC20_NEG);
    if (REC(19) < 0) COV(C_REC19_NEG);
    if (REC(21) < 0) COV(C_REC21_NEG);
  }
  if (forced) {
    COV(C_FORCED);
    goto g2400;
  }
  if (REC(24) >= 0xb) {
    COV(C_A_REC24_GT10);
    goto g71643;
  }
  if (6 < REC(23)) { COV(C_A_REC23_GT6); goto g71643; }
  if (5 < REC(1)) { COV(C_A_REC1_GT5); goto g71643; }
  if (mode == 0x65) {
    /* the 3429 0x65 arm: ORs and continues */
    COV(C_ARM_3429_65); n65++;
    m_or16(pw, 0x10, 0xe0);
    idx = m_pe2(o, 0x3d4, 0x6626);
    S16(qw, 0x20) = (int16_t)idx;
    m_or16(pw, 0xe, (uint32_t)(bitreverse((uint16_t)idx, 4) & 0xffff) * 2u);
    goto g71643;
  }
  if (((U8(o, 0xa9ea) & 1) == 0) && ((U8(o, 0xa9ec) & 0xe0) == 0)) goto g71643;
  COV(C_ARM_3429N);
  S16(qw, 0) = 0xd65;
  S16(qw, 0x10) = 0x7a7;
  m_setptr(o, 0xaa90, scale3429);
  S16(qw, 6) = (int16_t)bitreverse((uint16_t)((U16(o, 0xa9ea) >> 1) & 0xf), 4);
  m_setptr(o, 0xaaac, scale3429);
  m_setptr(o, 0xaab0, c1959);
  m_or16(pw, 4, 2);
  m_or16(pw, 6, 0xd0);
  S16(qw, 0x12) = 0xd65;
  S16(qw, 0x24) = 0x7a7;
  m_or16(pw, 4, 0x24);
  idx = m_pe2(o, 0x3d4, 0x6626);
  S16(qw, 0x20) = (int16_t)idx;
  goto common_tail;

g71643:
  COV(C_TO_61643);
  {
    int r22 = (int16_t)U16(o, RB + 0x3d6);
    /* ---- 3200 ---- */
    if (r22 < 7 && REC(1) < 7) {
      if (mode == 0x65) {
        int r23 = (int16_t)U16(o, RB + 0x402);
        COV(C_ARM_3200_65); n65++;
        if (r23 < 6) { m_or16(pw, 0xc, 0x40); COV(C_ARM3200_65_REC23_LE5); }
        if (r23 < 8) { m_or16(pw, 0xc, 2); m_or16(pw, 0xe, 0xc0); }
        else { m_or16(pw, 0xc, 3); m_or16(pw, 0xe, 0x40); COV(C_ARM3200_65_REC23_GT7); }
        idx = m_pe2(o, 0x37c, 0x639f);
        S16(qw, 0x1e) = (int16_t)idx;
        m_or16(pw, 0xc, (uint32_t)(bitreverse((uint16_t)idx, 4) & 0xffff) << 2);
      } else if ((U8(o, 0xa9e8) & 3) != 0 || (U8(o, 0xa9ea) & 0xc0) != 0) {
        COV(C_ARM_3200N);
        S16(qw, 0) = 0xc80;
        m_setptr(o, 0xaa90, scale3200);
        if ((U8(o, 0xa9e8) & 0x40) == 0) S16(qw, 0x10) = 0x725; else { S16(qw, 0x10) = 0x780; COV(C_ARM3200N_B40); }
        S16(qw, 6) = (int16_t)bitreverse((uint16_t)((U16(o, 0xa9e8) >> 2) & 0xf), 4);
        m_setptr(o, 0xaaac, scale3200);
        m_or16(pw, 6, 0x90);
        S16(qw, 0x12) = 0xc80;
        if (S16(o, RB + 0x402) < 6) {
          m_setptr(o, 0xaab0, c1920);
          m_or16(pw, 2, 4);
          S16(qw, 0x24) = 0x780;
        } else {
          S16(qw, 0x24) = 0x725;
          m_setptr(o, 0xaab0, c1829);
          COV(C_ARM3200N_REC23_GT5);
        }
        m_or16(pw, 4, 0x24);
        idx = m_pe2(o, 0x37c, 0x639f);
        S16(qw, 0x1e) = (int16_t)idx;
        goto common_tail;
      }
    }
    /* ---- 3000 ---- */
    if ((r22 < 7 && REC(2) < 7) || (REC(21) < 7 && REC(1) < 7)) {
      if (mode == 0x65) {
        COV(C_ARM_3000_65); n65++;
        if (r22 < 6 && REC(2) < 6) { m_or16(pw, 0xa, 0x80); COV(C_ARM3000_65_REC22_LE5); }
        if (r22 < 8) {
          if (r22 < 7) { m_or16(pw, 0xa, 1); m_or16(pw, 0xc, 0x80); }
          else { m_or16(pw, 0xa, 2); m_or16(pw, 0xc, 0x80); COV(C_ARM3000_65_REC22_EQ7); }
        } else { m_or16(pw, 0xa, 4); m_or16(pw, 0xc, 0x80); COV(C_ARM3000_65_REC22_GT7); }
        idx = m_pe2(o, 0x350, 0x656f);
        S16(qw, 0x1c) = (int16_t)idx;
        m_or16(pw, 0xa, (uint32_t)(bitreverse((uint16_t)idx, 4) & 0xffff) << 3);
      } else if ((U8(o, 0xa9e6) & 7) != 0 || (U8(o, 0xa9e8) & 0x80) != 0) {
        COV(C_ARM_3000N);
        S16(qw, 0) = 3000;
        m_setptr(o, 0xaa90, scale3000);
        if ((U8(o, 0xa9e6) & 0x80) == 0) S16(qw, 0x10) = 0x708; else { S16(qw, 0x10) = 2000; COV(C_ARM3000N_B80); }
        S16(qw, 6) = (int16_t)bitreverse((uint16_t)((U16(o, 0xa9e6) >> 3) & 0xf), 4);
        m_setptr(o, 0xaaac, scale3000);
        m_or16(pw, 4, 3);
        m_or16(pw, 6, 0x60);
        S16(qw, 0x12) = 3000;
        if (r22 < 6 && REC(2) < 6) {
          m_setptr(o, 0xaab0, c2000);
          m_or16(pw, 2, 4);
          S16(qw, 0x24) = 2000;
          COV(C_ARM3000N_C2000);
        } else {
          S16(qw, 0x24) = 0x708;
          m_setptr(o, 0xaab0, c1800_);
        }
        m_or16(pw, 4, 0x24);
        idx = m_pe2(o, 0x350, 0x656f);
        S16(qw, 0x1c) = (int16_t)idx;
        goto common_tail;
      }
    }
    /* ---- 2800 ---- */
    if ((REC(20) < 7 && REC(2) < 7) || (REC(19) < 7 && REC(1) < 7)) {
      if (mode == 0x65) {
        COV(C_ARM_2800_65); n65++;
        if (REC(20) < 6 && REC(2) < 6) { m_or16(pw, 6, 1); COV(C_ARM2800_65_OR6); }
        m_or16(pw, 8, 0xd);
        idx = m_pe2(o, 0x324, 0x6789);
        S16(qw, 0x18) = (int16_t)idx;
        m_or16(pw, 8, ((uint32_t)(bitreverse((uint16_t)idx, 4) & 0xf)) << 4);
      } else if ((U8(o, 0xa9e4) & 0xf) != 0) {
        COV(C_ARM_2800N);
        S16(qw, 0) = 0xaf0;
        m_setptr(o, 0xaa90, scale2800);
        if ((U8(o, 0xa9e2) & 1) == 0) S16(qw, 0x10) = 0x690; else { S16(qw, 0x10) = 0x74b; COV(C_ARM2800N_B1); }
        S16(qw, 6) = (int16_t)bitreverse((uint16_t)((U16(o, 0xa9e4) >> 4) & 0xf), 4);
        m_setptr(o, 0xaaac, scale2800);
        m_or16(pw, 4, 1);
        m_or16(pw, 6, 0x20);
        S16(qw, 0x12) = 0xaf0;
        if (REC(20) < 6 && REC(2) < 6) {
          m_setptr(o, 0xaab0, c1867);
          m_or16(pw, 2, 4);
          S16(qw, 0x24) = 0x74b;
          COV(C_ARM2800N_C1867);
        } else {
          S16(qw, 0x24) = 0x690;
          m_setptr(o, 0xaab0, c1680);
        }
        m_or16(pw, 4, 0x24);
        idx = m_pe2(o, 0x324, 0x6789);
        S16(qw, 0x18) = (int16_t)idx;
        goto common_tail;
      }
    }
  }
g2400:
  if (mode == 0x65) {
    COV(C_ARM_2400_65); n65++;
    if (n65 >= 2) COV(C_CHAIN2);
    if (n65 >= 3) COV(C_CHAIN3);
    if (n65 >= 4) COV(C_CHAIN4);
    if (n65 >= 5) COV(C_CHAIN5);
    m_or16(pw, 4, 0x24);
    idx = m_pe2(o, 0x324, 0x7da7);
    S16(qw, 0x16) = (int16_t)idx;
    {
      unsigned int r = bitreverse((uint16_t)idx, 4) & 0xffff;
      m_or16(pw, 2, (r >> 2) & 3);
      m_or16(pw, 4, (r << 6) & 0xc0);
    }
    return;
  }
  if ((U16(o, 0xa9e0) & 0x3c) == 0) { COV(C_ARM_2400_RET0); return; }
  COV(C_ARM_2400N);
  S16(qw, 0) = 0x960;
  m_setptr(o, 0xaa90, scale2400);
  if ((U8(o, 0xa9de) & 4) == 0) S16(qw, 0x10) = 0x640; else { S16(qw, 0x10) = 0x708; COV(C_ARM2400N_B4); }
  S16(qw, 6) = (int16_t)bitreverse((uint16_t)(((U16(o, 0xa9de) & 3) << 2) | ((U16(o, 0xa9e0) & 0xc0) >> 6)), 4);
  S16(qw, 0x12) = 0x960;
  S16(qw, 0x24) = 0x640;
  m_setptr(o, 0xaaac, scale2400);
  m_setptr(o, 0xaab0, c1600);
  m_or16(pw, 4, 0x24);
  idx = m_pe2(o, 0x324, 0x7da7);
  S16(qw, 0x16) = (int16_t)idx;
  {
    unsigned int r = bitreverse((uint16_t)idx, 4) & 0xffff;
    m_or16(pw, 2, r >> 2);
    m_or16(pw, 4, (r << 6) & 0xc0);
  }
  return;

common_tail : {
  /* the four non-0x65 arms 3429/3200/3000/2800 end in the same tail */
  unsigned int r = bitreverse((uint16_t)idx, 4) & 0xffff;
  m_or16(pw, 2, r >> 2);
  m_or16(pw, 4, (r << 6) & 0xff);
}
  return;
#undef REC
}

/* ============================================================================
 * the run wrapper: one call on a fresh (generated) state + model check + hash
 * ========================================================================== */


/* compare the WHOLE model object with the real one (slots normalised) and the ctx buffer */
static void compare_model(const unsigned char *real, const unsigned char *mod, const unsigned char *rctx, const unsigned char *mctx, const char *what, long call) {
  size_t i;
  for (i = 0; i < OBJSZ / 4; i++) {
    uint32_t a, b;
    memcpy(&a, real + 4 * i, 4);
    memcpy(&b, mod + 4 * i, 4);
    if (g_isslot[i]) {
      a = norm_val(a);
      b = norm_val(b);
    }
    if (a != b) {
      fprintf(stderr, "MODEL MISMATCH (%s, call %ld): word off=0x%05x real=%08x model=%08x\n", what, call, (unsigned int)(4 * i), a, b);
      exit(3);
    }
  }
  if (memcmp(rctx, mctx, CTXSZ)) {
    for (i = 0; i < CTXSZ; i++)
      if (rctx[i] != mctx[i]) {
        fprintf(stderr, "MODEL MISMATCH (%s, call %ld): ctx byte 0x%zx real=%02x model=%02x\n", what, call, i, rctx[i], mctx[i]);
        exit(3);
      }
  }
}

/* random value helpers */
static int16_t rs16(void) { return (int16_t)(lcg_next() >> 16); }

static uint32_t pick_l1(void) {
  switch (lcg_mod(20)) {
    case 0: return 0;
    case 1: case 2: case 3: return (uint32_t)lcg_range(1, 3);
    case 4: case 5: case 6: case 7: case 8: case 9: return (uint32_t)lcg_range(4, 499);
    case 10: return 499;
    case 11: return 500;
    case 12: return (uint32_t)lcg_range(500, 999);
    case 13: return 999;
    case 14: return 1000;
    case 15: return (uint32_t)lcg_range(1000, 100000);
    case 16: return 0x40000000u + (lcg_next() & 0xfffffu);
    case 17: return 0x01000000u + (lcg_next() & 0x7fffffu);
    case 18: return 0xffffffffu - (lcg_next() & 3);
    default: return lcg_next();
  }
}
static uint32_t pick_l2(uint32_t l1) {
  switch (lcg_mod(15)) {
    case 12: return 0xffffffffu;
    case 13: { static const uint32_t e[] = {0x80000000u, 0x7fffffffu, 0x40000000u, 0x3fffffffu, 0x80000001u, 0xfffffffeu}; return e[lcg_mod(6)]; }
    case 14: return 0x200000u + lcg_mod(3) - 1u;
    case 0: return 0;
    case 1: return 0x80000000u | lcg_next();
    case 2: return 1u << lcg_mod(31);
    case 3: return (1u << lcg_mod(31)) - 1u;
    case 4: case 5: case 6: case 7: { /* steer the ratio: l2 ~ target * l1 / 1024 */
      uint64_t tgt = ((uint64_t)(lcg_next() & 0x1ffffu)) >> lcg_mod(15);
      uint64_t v = (tgt * (uint64_t)l1) >> 10;
      v += lcg_mod(3);
      return (uint32_t)(v & 0x7fffffffu);
    }
    case 8: return lcg_next() & 0xffffu;
    case 9: return lcg_next() & 0x7fffffffu;
    case 10: return (lcg_next() & 0x1fffffu) | 0; /* < 2^21: the cnt>9 branch */
    default: return 0x200000u + (lcg_next() & 0x3fffffffu); /* >= 2^21: the shifted branch */
  }
}
static int pick_gain(void) {
  switch (lcg_mod(16)) {
    case 0: return 0;
    case 1: return 1;
    case 2: return -1;
    case 3: return lcg_range(-100, 100);
    case 4: return lcg_range(1, 400);
    case 5: return 0xfff;
    case 6: return 0x1000;
    case 7: return 0xffe;
    case 8: return 0x1001;
    case 9: return -0x1000;
    case 10: return lcg_range(400, 0x1000);
    case 11: return lcg_range(0x1000, 0x7fff);
    case 12: return -32768;
    case 13: return 32767;
    default: return rs16();
  }
}
static int pick_rec(int regime) {
  switch (regime) {
    case 0: return lcg_range(0, 12);
    case 1: { static const int v[] = {4, 5, 6, 7, 8, 10, 11, 12}; return v[lcg_mod(8)]; }
    case 2: return rs16();
    case 3: return lcg_range(-8, 12);
    case 5: return lcg_range(0x1e, 0x24);   /* around the S5 minimum's initial 0x20 */
    case 6: return lcg_range(0x20, 0x30);   /* every record >= 0x20 */
    default: return lcg_range(0, 40);
  }
}

/* boundary steering helpers: the ratio thresholds of the dBcnt loop and the 7000 clamp of the AGC estimate */
static uint32_t g_thr[64];
static int g_nthr;
static uint16_t g_clamp_v0[10][4][6]; /* [n iterations][target 6999..7002][candidates] */
static int g_clamp_n[10][4];
static void steer_init(void) {
  uint32_t e = 0x47d;
  int n, v0;
  while (e <= 0x19000u && g_nthr < 60) {
    g_thr[g_nthr++] = e;
    e = (e * 0x509u + 0x200u) >> 10;
  }
  g_thr[g_nthr++] = 0x18fffu;
  g_thr[g_nthr++] = 0x19000u;
  for (v0 = 0; v0 < 65536; v0++)
    for (n = 1; n <= 9; n++) {
      int32_t v = v0;
      int k, t;
      for (k = 0; k < n; k++) v = ((int32_t)(int16_t)v * 0x47cf + 0x2000) >> 14;
      for (t = 0; t < 4; t++)
        if ((int16_t)v == 6999 + t && g_clamp_n[n][t] < 6) g_clamp_v0[n][t][g_clamp_n[n][t]++] = (uint16_t)v0;
    }
}
/* the model's S2/S3: prr for the given inputs (no coverage bookkeeping) */
static int m_front_prr(uint32_t l1, uint32_t l2, int gain) {
  uint32_t ratio = 0, e;
  int dbc = 0;
  if (l1 != 0) {
    int cnt = 0;
    uint32_t mask = 0x80000000u;
    if ((int32_t)l2 >= 0) {
      do { cnt++; mask >>= 1; } while ((mask & l2) == 0 && cnt < 10);
      if (cnt > 9) { ratio = (uint32_t)(l2 * 0x400u + (l1 >> 1)) / l1; goto done; }
    }
    { uint32_t d = l1 >> ((10 - cnt) & 0x1f); ratio = d ? ((l2 << (cnt & 0x1f)) + (l1 >> ((0xb - cnt) & 0x1f))) / d : 0; }
  }
done:
  if (ratio > 0x18fffu) dbc = 1000;
  else for (e = 0x47d; e < ratio; e = (e * 0x509u + 0x200u) >> 10) dbc++;
  if (gain == 0) return 0;
  {
    uint32_t u = (uint32_t)(((int64_t)(gain >> 1) + 0x640000) / gain);
    if (u > 0x4000u) u = 0x4000u;
    return (int16_t)((u * (uint32_t)dbc) >> 14);
  }
}

/* generate one probeselect state into g_obj / g_ctx / g_P (level not set) */
static int g_force_arm_vars;  /* scenario 4: rec values from the boundary set */
static void gen_state(int cls) {
  int k, regime;
  fill_random(g_obj, OBJSZ);
  scrub_slots(g_obj);
  fill_random(g_ctx, CTXSZ);
  fill_random(g_P, PSZ);
  PTR(g_obj, 0x3548) = g_ctx;
  PTR(g_obj, 0xac3c) = g_P;
  U32(g_obj, 0xaac4) = pick_l1();
  U32(g_obj, 0xaac8) = pick_l2(U32(g_obj, 0xaac4));
  if (lcg_pct(12)) { /* ratio exactly on a dBcnt threshold (+-1): l1 = 1024 << k, l2 = t * 2^k  =>  ratio == t */
    uint32_t t = g_thr[lcg_mod((unsigned int)g_nthr)] + (uint32_t)lcg_range(-1, 1);
    uint32_t sh = lcg_mod(3);
    U32(g_obj, 0xaac4) = 1024u << sh;
    U32(g_obj, 0xaac8) = t << sh;
  }
  S16(g_obj, 0x39a) = (int16_t)pick_gain();
  if (lcg_pct(30)) S16(g_obj, 0x4c6) = (int16_t)lcg_range(0, 9000);
  else if (lcg_pct(20)) S16(g_obj, 0x4c6) = (int16_t)(0x7fff - lcg_range(0, 30));
  if (lcg_pct(60)) U16(g_obj, 0xa97e) = (uint16_t)(U16(g_obj, 0xa97e) | 0x80);
  else U16(g_obj, 0xa97e) = (uint16_t)(U16(g_obj, 0xa97e) & ~0x80);
  if (lcg_pct(50)) U8(g_P, 0x50) = (uint8_t)(U8(g_P, 0x50) | 0x10); else U8(g_P, 0x50) = (uint8_t)(U8(g_P, 0x50) & ~0x10);
  /* obj+0x24c / 0x250: zero / nonzero (incl. zero low half: a 16-bit test would see 0) */
  switch (lcg_mod(6)) { case 0: case 1: case 2: U32(g_obj, 0x24c) = 0; break; case 3: U32(g_obj, 0x24c) = 0xffu << (8 * lcg_mod(4)); break; case 4: U32(g_obj, 0x24c) = 1; break; default: break; }
  switch (lcg_mod(6)) { case 0: case 1: case 2: U32(g_obj, 0x250) = 0; break; case 3: U32(g_obj, 0x250) = 0x1u << (8 * lcg_mod(4) + lcg_mod(8)); break; case 4: U32(g_obj, 0x250) = 2; break; default: break; }
  /* obj+0x359a (u16, 0 -> the cascade runs; the upper neighbour 0x359c is the mode word), 0x359c mode */
  if (lcg_pct(50)) U16(g_obj, 0x359a) = 0;
  else if (U16(g_obj, 0x359a) == 0) U16(g_obj, 0x359a) = 1;
  switch (lcg_mod(10)) {
    case 0: case 1: case 2: case 3: case 4: U16(g_obj, 0x359c) = 0x65; break;
    case 5: U16(g_obj, 0x359c) = 0x165; break;
    case 6: U16(g_obj, 0x359c) = 0x8065; break;
    case 7: U16(g_obj, 0x359c) = 0x64; break;
    case 8: U16(g_obj, 0x359c) = 0x66; break;
    default: break;
  }
  /* records */
  regime = (int)lcg_mod(7);
  if (cls == 4) regime = 1;
  for (k = 0; k < 25; k++) {
    S16(g_obj, RB + 0xe + 0x2c * k) = (int16_t)pick_rec(regime);
    switch (lcg_mod(4)) { /* the +0xc field (thr / pre-emphasis v0) */
      case 0: S16(g_obj, RB + 0xc + 0x2c * k) = (int16_t)lcg_range(-5, 60); break;
      case 1: S16(g_obj, RB + 0xc + 0x2c * k) = (int16_t)lcg_range(1, 400); break;
      case 2: S16(g_obj, RB + 0xc + 0x2c * k) = (int16_t)lcg_range(-3000, 3000); break;
      default: break;
    }
  }
  if (lcg_pct(60)) S16(g_obj, RB + 0xe + 0x2c * (int)lcg_mod(25)) = 0; /* shift-free S5 */
  /* S5 reduction steering */
  if (lcg_pct(45)) {
    int cl = (int)lcg_mod(3), s = lcg_range(1, 5);
    S16(g_obj, RB + 0xe + 0x2c * 2) = (int16_t)lcg_range(0, 3);
    S16(g_obj, RB + 0xe + 0x2c * 3) = (int16_t)lcg_range(0, 3);
    S16(g_obj, RB + 0xe + 0x2c * 4) = (int16_t)lcg_range(0, 3);
    S16(g_obj, RB + 0xe + 0x2c * 6) = (int16_t)lcg_range(0, 3);
    {
      int i = S16(g_obj, RB + 0xe + 0x2c * 2) + S16(g_obj, RB + 0xe + 0x2c * 3) + S16(g_obj, RB + 0xe + 0x2c * 4) + S16(g_obj, RB + 0xe + 0x2c * 6);
      int tgt = (cl == 0) ? 0 : (cl == 1) ? 2 * i + s : 4 * i + s;
      S16(g_obj, RB + 0xe + 0x2c * 12) = (int16_t)(tgt / 4);
      S16(g_obj, RB + 0xe + 0x2c * 13) = (int16_t)(tgt / 4);
      S16(g_obj, RB + 0xe + 0x2c * 14) = (int16_t)(tgt / 4);
      S16(g_obj, RB + 0xe + 0x2c * 16) = (int16_t)(tgt - 3 * (tgt / 4));
    }
  }
  /* the INFO bits that gate the arms: force some bits on more often */
  if (lcg_pct(50)) U16(g_obj, 0xa9ea) = (uint16_t)(U16(g_obj, 0xa9ea) | 0x1);
  if (lcg_pct(30)) U16(g_obj, 0xa9ec) = (uint16_t)(U16(g_obj, 0xa9ec) & ~0xe0);
  if (lcg_pct(30)) { U16(g_obj, 0xa9e8) = (uint16_t)(U16(g_obj, 0xa9e8) & ~3); U16(g_obj, 0xa9ea) = (uint16_t)(U16(g_obj, 0xa9ea) & ~0xc0); }
  if (lcg_pct(30)) { U16(g_obj, 0xa9e6) = (uint16_t)(U16(g_obj, 0xa9e6) & ~7); U16(g_obj, 0xa9e8) = (uint16_t)(U16(g_obj, 0xa9e8) & ~0x80); }
  if (lcg_pct(30)) U16(g_obj, 0xa9e4) = (uint16_t)(U16(g_obj, 0xa9e4) & ~0xf);
  if (lcg_pct(30)) U16(g_obj, 0xa9e0) = (uint16_t)(U16(g_obj, 0xa9e0) & ~0x3c);
  if (lcg_pct(25)) { /* b+0x262 such that the scaled estimate lands on 6999..7002 (the 7000 clamp boundary) */
    int prr = m_front_prr(U32(g_obj, 0xaac4), U32(g_obj, 0xaac8), S16(g_obj, 0x39a));
    int n = (prr > 7) ? 7 : prr;
    if (n >= 1) {
      int t = (int)lcg_mod(4);
      if (g_clamp_n[n][t]) S16(g_obj, 0x4c6) = (int16_t)g_clamp_v0[n][t][lcg_mod((unsigned int)g_clamp_n[n][t])];
    }
  }
  if (cls == 4) {
    /* neutralise S5/S6 so the boundary values reach the cascade: one record == 0 (no shift), recs 12/13/14/16 == 0
     * (no reduction), P[0x50]>>5 == 7 and ctx flag bytes == 1 with obj+0x24c != 0 (chkForceBaudRate leaves the records alone) */
    static const int v[] = {4, 5, 6, 7, 8, 10, 11, 12};
    static const int vars[] = {1, 2, 19, 20, 21, 22, 23, 24};
    int n;
    for (n = 0; n < 8; n++) S16(g_obj, RB + 0xe + 0x2c * vars[n]) = (int16_t)v[lcg_mod(8)];
    S16(g_obj, RB + 0xe + 0x2c * 12) = 0; S16(g_obj, RB + 0xe + 0x2c * 13) = 0;
    S16(g_obj, RB + 0xe + 0x2c * 14) = 0; S16(g_obj, RB + 0xe + 0x2c * 16) = 0;
    S16(g_obj, RB + 0xe + 0x2c * 0) = 0;
    if (lcg_pct(85)) {
      U8(g_P, 0x50) = (uint8_t)((U8(g_P, 0x50) & 0x1f) | 0xe0);
      U32(g_obj, 0x24c) = 1;
      memset(g_ctx + 0x218, 1, 5);
    }
    if (lcg_pct(90)) U16(g_obj, 0x359a) = 0;
  }
  (void)g_force_arm_vars;
}

typedef struct {
  unsigned int hash;
  long nprints;
  unsigned int ph;
} callres_t;

/* one call: model + real + compare; `what` selects probeselect (0) or chkForceBaudRate direct (1) */
static long g_call_no;
static callres_t run_call(int what, unsigned int level, int flip_after, unsigned int flip_to, int use_model) {
  callres_t r;
  memcpy(g_snap, g_obj, OBJSZ);
  memcpy(g_ctxs, g_ctx, CTXSZ);
  memcpy(g_Ps, g_P, PSZ);
  if (use_model) {
    memcpy(g_mobj, g_obj, OBJSZ);
    memcpy(g_mctx, g_ctx, CTXSZ);
    if (what == 0) model(g_mobj, g_mctx, g_P);
    else m_chk(g_mobj, g_mctx, g_P);
  }
  g_ph = FNV0;
  g_np = 0;
  g_nprint_call = 0;
  g_flip_after = flip_after;
  g_flip_to = flip_to;
  dsplibs_debug_level = level;
  if (what == 0) probeselect(g_obj);
  else chkForceBaudRate(g_obj, g_obj + RB);
  dsplibs_debug_level = 0;
  g_flip_after = -1;
  if (use_model) compare_model(g_obj, g_mobj, g_ctx, g_mctx, what ? "chkForceBaudRate" : "probeselect", g_call_no);
  g_call_no++;
  r.hash = diff_obj(g_obj, g_snap);
  r.hash = fnv(r.hash, &(unsigned int){diff_buf(g_ctx, g_ctxs, CTXSZ)}, 4);
  r.hash = fnv(r.hash, &(unsigned int){diff_buf(g_P, g_Ps, PSZ)}, 4);
  r.hash = fnv(r.hash, &g_ph, 4);
  r.hash = fnv(r.hash, &g_np, 4); /* T04: 4 = sizeof(long) on the 32-bit golden */
  r.nprints = g_np;
  r.ph = g_ph;
  return r;
}

static void world_init(void) {
  steer_init();
  g_obj = xalloc(OBJSZ); g_snap = xalloc(OBJSZ); g_mobj = xalloc(OBJSZ);
  g_ctx = xalloc(CTXSZ); g_ctxs = xalloc(CTXSZ); g_mctx = xalloc(CTXSZ);
  V34I_REGC(g_ctx, CTXSZ, ctx); /* x86-64: the port reads the rate flags as the member v34Flags_ of a native VPcmFloModem (shadow of this image; 0x21b, not 0x217) */
  g_P = xalloc(PSZ); g_Ps = xalloc(PSZ);
  slots_init();
  tables_init();
}

static const unsigned int k_levels[6] = {2u, 3u, 0x80000000u, 0xffffffffu, 1u, 2u};

static void need_cov(int id) { need(k_cnames[id], g_cov.c[id]); }
static void need_cov_list(const int *ids, int n) {
  int i;
  for (i = 0; i < n; i++) need_cov(ids[i]);
}
static const int k_cov_frontend[] = {C_S2_L1ZERO, C_S2_NEGL2, C_S2_CNTGT9, C_S2_NORMAL, C_S2_DZERO, C_S3_DBC1000, C_S3_DBCLOOP, C_S3_GAIN0, C_S3_CLAMP, C_S3_NOCLAMP,
                                     C_PRR_ZERO, C_PRR_1_7, C_PRR_GT7, C_SENS_DIS, C_SENS_EN, C_BR_A, C_BR_B, C_BR_REGULAR, C_BR_NOASK_FLAG, C_BR_NOASK_GAIN,
                                     C_BR_NOASK_PRR, C_REG_CLAMP7000, C_S5_MINZEROEXT, C_S5_RED2, C_S5_RED1, C_S5_NONE, C_S5_I0, C_S5_SHIFT0, C_S5_SHIFTNZ};
static const int k_cov_chk[] = {C_CHK_FORCE, C_CHK_CTXFLAGS, C_CHK_LOCALF5_1, C_CHK_LOCALF5_0, C_CHK_E5, C_CHK_E4, C_CHK_E3, C_CHK_E2, C_K_0, C_K_1, C_K_2, C_K_3, C_K_4, C_K_5, C_K_6, C_K_7};
static const int k_cov_arms[] = {C_ARM_3429N, C_ARM_3429_65, C_ARM_3200N, C_ARM_3200_65, C_ARM_3000N, C_ARM_3000_65, C_ARM_2800N, C_ARM_2800_65, C_ARM_2400_65, C_ARM_2400N,
                                 C_ARM_2400_RET0, C_FORCED, C_TO_61643, C_PE_6, C_PE_7, C_PE_8, C_PE_9, C_PE_10, C_A_REC1_GT5, C_A_REC23_GT6, C_A_REC24_GT10,
                                 C_ARM3200_65_REC23_LE5, C_ARM3200_65_REC23_GT7, C_ARM3000_65_REC22_LE5, C_ARM3000_65_REC22_EQ7, C_ARM3000_65_REC22_GT7, C_ARM2800_65_OR6,
                                 C_ARM3200N_REC23_GT5, C_ARM3000N_C2000, C_ARM2800N_C1867, C_ARM3200N_B40, C_ARM3000N_B80, C_ARM2800N_B1, C_ARM2400N_B4};

static const int k_cov_chain[] = {C_CHAIN2, C_CHAIN3, C_CHAIN4, C_CHAIN5};
static const int k_cov_neg[] = {C_REC23_NEG, C_REC1_NEG, C_REC22_NEG, C_REC2_NEG, C_REC20_NEG, C_REC19_NEG, C_REC21_NEG};
static void reset_cov(void) { memset(&g_cov, 0, sizeof g_cov); }

static void need_formats(const int *req, int n) {
  int i;
  for (i = 0; i < n; i++) {
    char nm[64];
    snprintf(nm, sizeof nm, "print format #%d", req[i]);
    need(nm, g_fmt_count[req[i]]);
  }
  if (g_unknown_fmt) die("unknown print format seen");
  if (g_fmt_count[14]) die("the unreachable 'index is 0' print fired (dead-code assumption violated)");
}

/* ============================================================================
 * scenarios
 * ========================================================================== */
static void scen_random(int level_mode, int ncalls, const char *name) {
  logger_t lg = {name, FNV0, 0, 997};
  int c;
  world_init();
  reset_cov();
  for (c = 0; c < ncalls; c++) {
    unsigned int lvl = level_mode ? k_levels[c % 6] : 0;
    callres_t r;
    gen_state(0);
    g_echo = level_mode && (c % 500 == 0);
    r = run_call(0, lvl, -1, 0, 1);
    g_echo = 0;
    log_call(&lg, r.hash, (long)lvl, r.nprints, (long)U16(g_obj, 0xa9b0));
  }
  log_done(&lg);
  need_cov_list(k_cov_frontend, (int)(sizeof k_cov_frontend / sizeof k_cov_frontend[0]));
  need_cov_list(k_cov_chk, (int)(sizeof k_cov_chk / sizeof k_cov_chk[0]));
  need_cov_list(k_cov_arms, (int)(sizeof k_cov_arms / sizeof k_cov_arms[0]));
  need_cov_list(k_cov_neg, (int)(sizeof k_cov_neg / sizeof k_cov_neg[0]));
  need_cov_list(k_cov_chain, (int)(sizeof k_cov_chain / sizeof k_cov_chain[0]));
  if (level_mode) {
    static const int req[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15};
    need_formats(req, (int)(sizeof req / sizeof req[0]));
  }
}

static void scen_flips(int ncalls) {
  logger_t lg = {"flips", FNV0, 0, 499};
  int c;
  long cut[16], cutany = 0, samecount = 0;
  memset(cut, 0, sizeof cut);
  world_init();
  reset_cov();
  for (c = 0; c < ncalls; c++) {
    unsigned int lvl = k_levels[c % 6];
    static const unsigned int to[] = {0u, 1u, 2u, 0x80000000u, 0u, 3u};
    int fa = (int)lcg_mod(14);
    unsigned int ft = to[lcg_mod(6)];
    callres_t r0, r1;
    unsigned int h;
    gen_state(0);
    /* the same state twice: unflipped, then flipped (restore the pre-state in between) */
    {
      unsigned char *save = xalloc(OBJSZ), *savec = xalloc(CTXSZ), *saveP = xalloc(PSZ);
      memcpy(save, g_obj, OBJSZ); memcpy(savec, g_ctx, CTXSZ); memcpy(saveP, g_P, PSZ);
      r0 = run_call(0, lvl, -1, 0, 1);
      memcpy(g_obj, save, OBJSZ); memcpy(g_ctx, savec, CTXSZ); memcpy(g_P, saveP, PSZ);
      r1 = run_call(0, lvl, fa, ft, 0);
      free(save); free(savec); free(saveP);
    }
    if (r1.nprints < r0.nprints) { cutany++; if (fa < 16) cut[fa]++; }
    else if (r1.nprints == r0.nprints) samecount++;
    h = fnv(r0.hash, &r1.hash, 4);
    log_call(&lg, h, (long)lvl, r0.nprints, r1.nprints);
  }
  log_done(&lg);
  need("flip cut a chain", cutany);
  need("flip did not change the chain", samecount);
  {
    int k;
    for (k = 0; k < 10; k++) {
      char nm[48];
      snprintf(nm, sizeof nm, "chain cut after print #%d", k);
      need(nm, cut[k]);
    }
  }
  {
    static const int req[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15};
    need_formats(req, (int)(sizeof req / sizeof req[0]));
  }
}

static void scen_chk(int ncalls) {
  logger_t lg = {"chkForceBaudRate", FNV0, 0, 997};
  int c;
  world_init();
  reset_cov();
  for (c = 0; c < ncalls; c++) {
    unsigned int lvl = (c % 3 == 0) ? k_levels[(c / 3) % 6] : 0;
    callres_t r;
    int j;
    fill_random(g_obj, OBJSZ);
    scrub_slots(g_obj);
    fill_random(g_ctx, CTXSZ);
    fill_random(g_P, PSZ);
    PTR(g_obj, 0x3548) = g_ctx;
    PTR(g_obj, 0xac3c) = g_P;
    U8(g_P, 0x50) = (uint8_t)((lcg_next() >> 8) & 0xff);
    switch (lcg_mod(5)) { case 0: case 1: U32(g_obj, 0x24c) = 0; break; case 2: U32(g_obj, 0x24c) = 0x1u << (8 * lcg_mod(4) + lcg_mod(8)); break; case 3: U32(g_obj, 0x24c) = 0xffffffffu; break; default: break; }
    switch (lcg_mod(5)) { case 0: case 1: U32(g_obj, 0x250) = 0; break; case 2: U32(g_obj, 0x250) = 0x1u << (8 * lcg_mod(4) + lcg_mod(8)); break; case 3: U32(g_obj, 0x250) = 2; break; default: break; }
    for (j = 0; j < 8; j++) g_ctx[0x217 + j] = (uint8_t)(lcg_pct(50) ? 1 : (lcg_pct(30) ? 0 : (lcg_next() >> 9)));
    for (j = 0; j < 25; j++) S16(g_obj, RB + 0xe + 0x2c * j) = (int16_t)pick_rec((int)lcg_mod(5));
    g_echo = (lvl > 1) && (c % 300 == 0);
    r = run_call(1, lvl, (c % 7 == 0) ? 0 : -1, 0, 1);
    g_echo = 0;
    log_call(&lg, r.hash, (long)lvl, r.nprints, (long)(U8(g_P, 0x50) >> 5));
  }
  log_done(&lg);
  need_cov_list(k_cov_chk, (int)(sizeof k_cov_chk / sizeof k_cov_chk[0]));
  need("chk print (format #15)", g_fmt_count[15]);
}

static void scen_cascade(int ncalls) {
  logger_t lg = {"cascade", FNV0, 0, 1499};
  int c;
  world_init();
  reset_cov();
  for (c = 0; c < ncalls; c++) {
    callres_t r;
    gen_state(4);
    r = run_call(0, 0, -1, 0, 1);
    log_call(&lg, r.hash, (long)U16(g_obj, 0x359c), (long)U16(g_obj, 0xa9b0), (long)U16(g_obj, 0xaa84));
  }
  log_done(&lg);
  need_cov_list(k_cov_arms, (int)(sizeof k_cov_arms / sizeof k_cov_arms[0]));
  need("no-return fallthrough to 2400 non-0x65 return", g_cov.c[C_ARM_2400N]);
}

/* pre-emphasis steering: run the model once to learn which arms run, then put thr on / next to a trajectory value */
static int traj(const unsigned char *o, int koff, int mul, int n) {
  int v = S16(o, RB + koff), k;
  for (k = 0; k < n; k++) v = (int16_t)(((int64_t)v * mul) >> 14);
  return v;
}
static void scen_pe(int ncalls) {
  logger_t lg = {"pe-steer", FNV0, 0, 1499};
  static const int ks[5] = {0x324, 0x37c, 0x350, 0x3d4, 0x324};
  static const int ms[5] = {0x7da7, 0x639f, 0x656f, 0x6626, 0x6789};
  int c;
  long boundary_ge = 0, boundary_lt = 0;
  world_init();
  reset_cov();
  for (c = 0; c < ncalls; c++) {
    callres_t r;
    int arm = (int)lcg_mod(5), n = lcg_range(1, 5), off = lcg_range(-1, 1), kind = (int)lcg_mod(5);
    static const int K[5] = {0x324, 0x37c, 0x350, 0x3d4, 0x324};
    (void)ks; (void)ms;
    gen_state(4);
    /* v0 small so that the trajectory stays inside 16 bits for a while */
    S16(g_obj, RB + K[arm]) = (int16_t)lcg_range(-300, 300);
    if (kind == 0) S16(g_obj, RB + K[arm]) = (int16_t)lcg_range(-3, 3);
    {
      /* the arm order in the cascade for the steered arm is decided by the model; the steering applies to the
       * arm's own (K, M): thr = trajectory(n) + off (found at step n iff traj(n) > thr) */
      int M = (arm == 0) ? 0x7da7 : (arm == 1) ? 0x639f : (arm == 2) ? 0x656f : (arm == 3) ? 0x6626 : 0x6789;
      int t = traj(g_obj, K[arm], M, n);
      int thr = t + off;
      if (thr < -32768) thr = -32768;
      if (thr > 32767) thr = 32767;
      S16(g_obj, RB + 0xbc) = (int16_t)thr;
      if (off == 0) boundary_ge++; else boundary_lt++;
    }
    /* steer the gates so that the wanted arm is the first one that returns / runs */
#define SR(k, v) S16(g_obj, RB + 0xe + 0x2c * (k)) = (int16_t)(v)
    if (arm == 3) { SR(24, 5); SR(23, 5); SR(1, 5); U16(g_obj, 0xa9ea) = (uint16_t)(U16(g_obj, 0xa9ea) | 1); }
    if (arm == 1) { SR(24, 12); SR(23, 5); SR(22, 5); SR(1, 5); U16(g_obj, 0xa9e8) = (uint16_t)(U16(g_obj, 0xa9e8) | 1); }
    if (arm == 2) { SR(24, 12); SR(22, 5); SR(2, 5); SR(1, 12); U16(g_obj, 0xa9e6) = (uint16_t)(U16(g_obj, 0xa9e6) | 1); }
    if (arm == 4) { SR(24, 12); SR(22, 12); SR(21, 12); SR(20, 5); SR(2, 5); U16(g_obj, 0xa9e4) = (uint16_t)(U16(g_obj, 0xa9e4) | 1); }
    if (arm == 0) { SR(24, 12); SR(22, 12); SR(21, 12); SR(20, 12); SR(19, 12); U16(g_obj, 0xa9e0) = (uint16_t)(U16(g_obj, 0xa9e0) | 4); }
#undef SR
    r = run_call(0, (c % 4 == 0) ? 2u : 0u, -1, 0, 1);
    log_call(&lg, r.hash, (long)arm, (long)n, (long)off);
  }
  log_done(&lg);
  need("thr == traj (not found at n)", boundary_ge);
  need("thr < traj (found at n)", boundary_lt);
  {
    static const int ids[] = {C_PE_6, C_PE_7, C_PE_8, C_PE_9, C_PE_10, C_ARM_3429N, C_ARM_3200N, C_ARM_3000N, C_ARM_2800N, C_ARM_2400N, C_ARM_2400_65};
    need_cov_list(ids, (int)(sizeof ids / sizeof ids[0]));
  }
  {
    static const int req[] = {12, 13};
    need_formats(req, 2);
  }
}

/* exhaustive gain grid + SNR grid at debug level 2 */
static void scen_snr(void) {
  logger_t lg = {"snr-gain-grid", FNV0, 0, 4999};
  long c = 0;
  int g, cfg, s;
  world_init();
  reset_cov();
  for (cfg = 0; cfg < 3; cfg++)
    for (g = -32768; g <= 32767; g++) {
      callres_t r;
      gen_state(0);
      S16(g_obj, 0x39a) = (int16_t)g;
      U32(g_obj, 0xaac4) = (cfg == 0) ? 1000u + (uint32_t)(g & 0xff) : (cfg == 1) ? 2000u : 0x100000u;
      U32(g_obj, 0xaac8) = (cfg == 0) ? 3000u : (cfg == 1) ? 0x30000u : 0x9000000u;
      U16(g_obj, 0xa97e) = (uint16_t)(U16(g_obj, 0xa97e) | 0x80);
      U8(g_P, 0x50) = (uint8_t)(U8(g_P, 0x50) & ~0x10); /* regular path: gain / prr visible in the bits */
      r = run_call(0, 2, -1, 0, 1);
      log_call(&lg, r.hash, (long)g, (long)cfg, r.nprints);
      c++;
    }
  /* SNR grid: every leading-bit position of l2 x l1 classes x ratio boundaries */
  for (s = 0; s < 40000; s++) {
    callres_t r;
    gen_state(0);
    switch (s % 8) {
      case 0: U32(g_obj, 0xaac8) = 1u << (s / 8 % 32); break;
      case 1: U32(g_obj, 0xaac8) = (1u << (s / 8 % 32)) - 1u; break;
      case 2: U32(g_obj, 0xaac8) = (1u << (s / 8 % 31)) | (lcg_next() & ((1u << (s / 8 % 31)) - 1u)); break;
      default: break;
    }
    if (s % 5 == 0) U32(g_obj, 0xaac4) = 1u << (s / 5 % 32);
    if (s % 5 == 1) U32(g_obj, 0xaac4) = (1u << (s / 5 % 32)) + (lcg_next() & 7);
    r = run_call(0, 2, -1, 0, 1);
    log_call(&lg, r.hash, (long)s, (long)U32(g_obj, 0xaac4), r.nprints);
  }
  log_done(&lg);
  need_cov_list(k_cov_frontend, (int)(sizeof k_cov_frontend / sizeof k_cov_frontend[0]));
  need("grid calls", c);
}

static void scen_sens(int ncalls) {
  logger_t lg = {"sensitive-rx", FNV0, 0, 999};
  static const uint32_t l1v[] = {0, 1, 2, 250, 498, 499, 500, 501, 750, 998, 999, 1000, 1001, 5000, 0xffffffffu};
  int c;
  world_init();
  reset_cov();
  for (c = 0; c < ncalls; c++) {
    callres_t r;
    unsigned int lvl = (c & 1) ? k_levels[(c / 2) % 6] : 0;
    gen_state(0);
    U32(g_obj, 0xaac4) = l1v[lcg_mod(15)];
    if (lcg_pct(70)) U32(g_obj, 0xaac8) = (U32(g_obj, 0xaac4) * (uint32_t)lcg_range(0, 40)) >> 2;
    if (lcg_pct(70)) U8(g_P, 0x50) = (uint8_t)(U8(g_P, 0x50) | 0x10);
    if (lcg_pct(80)) U16(g_obj, 0xa97e) = (uint16_t)(U16(g_obj, 0xa97e) | 0x80);
    g_echo = (lvl > 1) && (c % 700 == 1);
    r = run_call(0, lvl, -1, 0, 1);
    g_echo = 0;
    log_call(&lg, r.hash, (long)lvl, (long)U32(g_obj, 0xaac4), (long)U16(g_obj, 0x4c6));
  }
  log_done(&lg);
  need_cov_list(k_cov_frontend, (int)(sizeof k_cov_frontend / sizeof k_cov_frontend[0]));
}

static void scen_chain(int nobj, int reps) {
  logger_t lg = {"persistent", FNV0, 0, 499};
  int o, k;
  world_init();
  reset_cov();
  for (o = 0; o < nobj; o++) {
    unsigned int h = FNV0;
    gen_state(o % 2 ? 4 : 0);
    for (k = 0; k < reps; k++) {
      callres_t r = run_call(0, (k & 1) ? 2u : 0u, -1, 0, 1);
      h = fnv(h, &r.hash, 4);
    }
    log_call(&lg, h, (long)o, (long)reps, (long)U16(g_obj, 0x4c6));
  }
  log_done(&lg);
  need("persistent runs", (long)nobj * reps);
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  if (getenv("T8A_SEED")) g_lcg ^= (unsigned int)strtoul(getenv("T8A_SEED"), NULL, 0) * 2654435761u;
  setvbuf(stdout, NULL, _IOFBF, 1 << 16);
  switch (sc) {
    case 0: scen_random(0, 60000, "probeselect-random"); break;
    case 1: scen_random(1, 30000, "probeselect-levels"); break;
    case 2: scen_flips(20000); break;
    case 3: scen_chk(50000); break;
    case 4: scen_cascade(150000); break;
    case 5: scen_pe(100000); break;
    case 6: scen_snr(); break;
    case 7: scen_sens(60000); break;
    case 8: scen_chain(2000, 8); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 4;
  }
  fflush(stdout);
  printf("total prints %ld\n", g_total_prints);
  if (g_fail) return 2;
  return 0;
}
