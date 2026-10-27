/*
 * Differential test harness for V.34 "tier 6a" (src/v34/v34_modulator.c): modulatevector
 * (TX vector modulator = shell mapper + constellation mapper + trellis encoder + precoder).
 *
 * One source, two binaries (same convention as test/v34/v34_tx_symbol.main.c / v34_tier5d):
 *   v34_tier6a_test_blob - linked against the real dsplibs.o (-DTEST_BLOB)
 *   v34_tier6a_test_port - linked against src/v34/v34_modulator.c + the ported callees + tables
 * Each scenario runs in its OWN process (argv[1]).
 *
 * WHICH CALLEES ARE REAL: modulatevector calls getFrame, txmit (and V34nlencoder).  In the blob
 * binary those are the BLOB'S OWN (dsplibs.o); in the port binary they are the already verified
 * ports (src/v34/v34_tx_symbol.c, src/v34/v34_dsp_primitives.c, ...).  getFrame's indirect callee (the
 * scrambler at obj+0x2a28) is either the harness RECORDER (deterministic bit source; hashes every
 * call) or the real scrambleGPC / scrambleGPA (blob's own / src/v34/v34_init.c).  txmit runs the
 * real modulator / echo pre-filter / delay lines on a valid configuration (setup_tx).
 *
 * Objects: tagV34Object is not a struct.  A random fake object (OBJSZ = 0xb400 bytes) is refreshed
 * per call (single-call scenarios; result = hash of the DIFF: word index + new value of every
 * changed word, so a stray write anywhere is caught) or kept across calls (chain scenarios; result =
 * hash of the WHOLE object after every call).  Pointer slots (coefficient set obj+0x2604, trellis
 * table obj+0x2608, scrambler fn 0x2a28, tx ring, bulk-delay base, modulator/echo pointers) are
 * compared NORMALISED (in-object -> offset, harness buffer / table -> (region id, offset),
 * function -> id); raw addresses are never compared across the two processes.
 *
 * VALID STATES + INDEX ORACLE (important, read this).  modulatevector has unchecked table reads and
 * unbounded loops that a legitimate producer never triggers (see the port header, hazards H1-H4):
 * quarter[idx] (valid 0..415), gInvertPat[(short)(Fc+1)] (valid 0..15), idiv by A[j1]/B[j2]/B[j3]
 * (zero divisor = SIGFPE in the blob) and the three peel loops (valid: never run past `lo`).  The
 * harness therefore builds VALID shell-mapper tables from its own math (g2 = 1..m..1 triangle,
 * g4 = g2*g2, g8 = g4*g4, T = prefix sums of g8 - bit-identical to initV34's tables for m <= 14 and
 * truncated at 2^32 for m >= 15, checked against the blob's own `xyz` at start-up) and runs every
 * candidate state through an ORACLE first: a private copy of the PRISTINE src/v34/v34_modulator.c is
 * compiled into this harness under oracle_* names with PORT_TRACE_INDEX defined (its hooks report
 * every unchecked index / divisor / loop overrun and the branch-class coverage; txmit and the
 * nlencoder are stubbed in the copy, getFrame is the real one, run on a scratch copy of the object).
 * A violating state is discarded and re-drawn (deterministically and identically in both processes:
 * the oracle sees the blob's tables in the blob process and the port's in the port process, which the
 * start-up memcmp proves equal).  The oracle source is always the pristine file (PORT_T6A_ORACLE_SRC),
 * so mutating the port under test (env PORT_T6A) cannot change the input stream.  Nothing outside
 * the oracle-accepted domain is claimed covered.
 *
 * INDEPENDENT MODEL: (a) the eight ring numbers the shell decode produces are checked against a
 * from-scratch decode written in this file (shell_decode: cumulative g8 / g4 / g2 block search, the
 * reflect rule for pair sums >= L) for every oracle-accepted state on the harness-built tables; (b)
 * the tail (k, k+1, obj+0x2aa2, the point handed to txmit at obj+0x25d0, the nlencoder arm) and the
 * e4c / aa74 / 0x2214 / 0x25c2 block are checked against an independent model on BOTH sides.
 *
 * Scenarios: 0 random single calls (recorder, v steered), 1 random single calls (real scrambleGPC and
 * GPA), 2 chains (recorder, GPC, GPA), 3 shell-index boundary sweep (T[k], T[k]-1, T[k+1]-1, total-1;
 * m = 2 exhaustive), 4 tail / k / e4c block, 5 real preinitV34 + initV34 built blocks, 6 zero-divisor
 * probe (blob traps in a forked child; the port must trap the same way), 7 legal V.34 rate/baud grid
 * (real initV34: the largest quarter[] index stays <= 415 for every legal combination).
 * Every scenario keeps COVERAGE COUNTERS (oracle branch classes + input classes) and exits 2 if a
 * branch class was never reached; exit 3 = harness assumption / model violated.
 */

#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v34_img.h" /* T04: x86-64 image <-> native V34Obj marshalling; empty at -m32 */
#include <sys/wait.h>
#include <unistd.h>

/* ---- functions under test / callees ---- */
extern void modulatevector(void *obj);
extern void getFrame(void *obj);
extern void txmit(void *obj);
extern void V34nlencoder(const short in[2], short out[2]);
extern int scrambleGPC(void *obj, int arg);
extern int scrambleGPA(void *obj, int arg);
extern void preinitV34(void *blk);
extern int initV34(void *blk, unsigned short p2, unsigned short p3, short p4, short p5, uint32_t p6,
                   unsigned short p7);

/* tables */
extern const unsigned short lsbMask[];
extern const short Convolve16[], Convolve32[], Convolve64[];
extern const unsigned int xyz[];
#ifdef TEST_BLOB
extern const short port_quarter[];
extern const short port_smIndex[];
extern const unsigned short port_gInvertPat[];
extern const unsigned int port_xyz[];
#endif

#define OBJSZ 0xb400u
#define OBJBIG 0x30000u /* the oracle's scratch object: a wild 16-bit index can reach 0x2628 + 2*65535 */

/* ------------------------------------------------------------------ oracle */
/* stubs for the oracle copy: getFrame is the real one on the scratch object, txmit / nlencoder do nothing */
#if defined(__x86_64__)
static void orc_real_getFrame(void *obj) { v34i_enter(); getFrame(v34i_in(obj)); v34i_leave(); } /* the real getFrame on the scratch image */
#else
#define orc_real_getFrame getFrame
#endif
struct tagV34Object;
static void oracle_getFrame(struct tagV34Object *obj) { orc_real_getFrame(obj); }
static void oracle_txmit(struct tagV34Object *obj) { (void)obj; }
static void oracle_nl(const short in[2], short out[2]) { (void)in; (void)out; }
#define PORT_TRACE_INDEX 1
#define modulatevector oracle_modulatevector
#define getFrame oracle_getFrame
#define txmit oracle_txmit
#define V34nlencoder oracle_nl
#ifndef PORT_T6A_ORACLE_SRC
#define PORT_T6A_ORACLE_SRC "../src/v34/v34_modulator.c"
#endif
#include PORT_T6A_ORACLE_SRC
#undef modulatevector
#undef getFrame
#undef txmit
#undef V34nlencoder
#undef PORT_TRACE_INDEX
#undef OBJ_BYTES
#undef S16
#undef U16
#undef S32
#undef U32
#undef U8
#undef PTR


extern void oracle_modulatevector(struct tagV34Object *obj);
/* V34SHIMS BEGIN (generated by tools/tier_rewrite.py shims; do not edit) */
#if defined(__x86_64__)
static void V34SH_modulatevector(void *obj) { v34i_enter(); modulatevector((void *)v34i_in((void *)obj)); v34i_leave(); }
static void V34SH_getFrame(void *obj) { v34i_enter(); getFrame((void *)v34i_in((void *)obj)); v34i_leave(); }
static void V34SH_txmit(void *obj) { v34i_enter(); txmit((void *)v34i_in((void *)obj)); v34i_leave(); }
static void V34SH_V34nlencoder(const short *in, short *out) { v34i_enter(); V34nlencoder((const short *)v34i_in((void *)in), (short *)v34i_in((void *)out)); v34i_leave(); }
static int V34SH_scrambleGPC(void *obj, int arg) { int r; v34i_enter(); r = scrambleGPC((void *)v34i_in((void *)obj), arg); v34i_leave(); return r;}
static int V34SH_scrambleGPA(void *obj, int arg) { int r; v34i_enter(); r = scrambleGPA((void *)v34i_in((void *)obj), arg); v34i_leave(); return r;}
static void V34SH_preinitV34(void *blk) { v34i_enter(); preinitV34((void *)v34i_in((void *)blk)); v34i_leave(); }
static int V34SH_initV34(void *blk, unsigned short p2, unsigned short p3, short p4, short p5, uint32_t p6, unsigned short p7) { int r; v34i_enter(); r = initV34((void *)v34i_in((void *)blk), p2, p3, p4, p5, p6, p7); v34i_leave(); return r;}
static void V34SH_oracle_modulatevector(void *obj) { v34i_enter(); oracle_modulatevector((void *)v34i_in((void *)obj)); v34i_leave(); }
#define modulatevector V34SH_modulatevector
#define getFrame V34SH_getFrame
#define txmit V34SH_txmit
#define V34nlencoder V34SH_V34nlencoder
#define scrambleGPC V34SH_scrambleGPC
#define scrambleGPA V34SH_scrambleGPA
#define preinitV34 V34SH_preinitV34
#define initV34 V34SH_initV34
#define oracle_modulatevector V34SH_oracle_modulatevector
#endif
/* V34SHIMS END */
#define OB(o, off) ((unsigned char *)(o) + (off))
#define U8(o, off) (*(uint8_t *)OB(o, off))
#define S16(o, off) (*(int16_t *)OB(o, off))
#define U16(o, off) (*(uint16_t *)OB(o, off))
#define I32(o, off) (*(int32_t *)OB(o, off))
#define U32(o, off) (*(uint32_t *)OB(o, off))
#if defined(__x86_64__)
#define PTRS(o, off) (*(uint32_t *)OB(o, off)) /* x86-64: the image keeps a 32-bit slot (low memory address), marshalled to a real pointer (test/v34/v34_img.h) */
#else
#define PTRS(o, off) (*(void **)OB(o, off))

#endif
#define OBJSZ 0xb400u

/* ---- deterministic LCG with a bijective finaliser (raw low bytes are period-256) ---- */
static unsigned int g_lcg = 0x7ab5e1e5u;
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
static int rr(int lo, int hi) { return lo + (int)lcg_mod((unsigned int)(hi - lo + 1)); }
static uint16_t rnd16(void) { return (uint16_t)(lcg_next() >> 16); }
static short rshort(void) { return (short)(lcg_next() >> 16); }

static unsigned int fnv(unsigned int h, const void *p, size_t n) {
  const unsigned char *b = (const unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) {
    h ^= b[i];
    h *= 16777619u;
  }
  return h;
}
static unsigned int fnv32(unsigned int h, uint32_t w) {
  h ^= w;
  h *= 16777619u;
  h ^= h >> 15;
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

typedef struct {
  const char *name;
  unsigned int all;
  long calls;
  long every;
} logger_t;
static void log_call(logger_t *l, unsigned int h, long a, long b) {
  l->all = fnv(l->all, &h, 4);
  if (g_verbose || l->calls % l->every == 0) printf("%s c=%ld a=%ld b=%ld hash=%08x\n", l->name, l->calls, L32(a), L32(b), h);
  l->calls++;
}
static void log_done(logger_t *l) { printf("%s calls=%ld all=%08x\n", l->name, l->calls, l->all); }


static unsigned int g_rec_h;
static long g_rec_n;
static int g_rec_on;
static void *g_rec_obj;
static int g_rec_callno;

static unsigned int mix32(unsigned int a, unsigned int b) {
  unsigned int x = a * 0x9e3779b1u + b * 0x85ebca6bu + 0x1234567u;
  x ^= x >> 15;
  x *= 0x2c1b3c6du;
  x ^= x >> 12;
  x *= 0x297a2d39u;
  x ^= x >> 15;
  return x;
}
/* pure function of (count, call number within this getFrame): the next count the recorder returns */
static int rec_next_count(int count, int callno) {
  int k = 1 + (int)(mix32((unsigned int)count, (unsigned int)callno) % 30u);
  return (int16_t)(count - k);
}
/* what the recorder writes into obj+0x2a64 during the fn(obj, 0) call of the n > 16 arm (the blob re-reads the
 * count from memory after that call, so a callee that changes it must be honoured) */
static int rec_zero_mem(int callno) { return (int)(mix32((unsigned int)callno, 999u) % 50u) - 5; }

/* The recorder stands in for scrambleGPA/GPC: hashes the full 32-bit (count) argument, refills the
 * bit buffer at obj+0x2a60 deterministically, returns the new count in AX with GARBAGE in the upper
 * half of EAX (the caller must use AX only). */
#if defined(__x86_64__)
#define RECORDER_BODY recorder_img /* x86-64: `recorder` is the callback the port calls on a SHADOW object; it syncs the image, runs the body on the image, syncs back */
static int recorder_img(void *obj, int count);
static int recorder(void *objn, int count) {
  int r;
  V34I_SYNC_IMG();
  r = recorder_img(V34I_IMGP(objn), count);
  V34I_SYNC_NAT();
  return r;
}
#else
#define RECORDER_BODY recorder
#endif
static int RECORDER_BODY(void *obj, int count) {
  unsigned int buf;
  int nc;
  if (!g_rec_on) return 0;
  if (obj != g_rec_obj) die("recorder: obj pointer mismatch");
  if (count != (int)(int16_t)count) die("recorder: count argument is not a sign-extended short");
  g_rec_h = fnv32(fnv32(g_rec_h, (uint32_t)count), (uint32_t)U32(obj, 0x2a60));
  /* the whole output block + bit reader as it is at this instant: makes the ORDER of getFrame's stores
   * relative to its indirect calls observable (a real callee could look at any of it) */
  g_rec_h = fnv(g_rec_h, OB(obj, 0x2a30), 0x36);
  g_rec_n++;
  buf = U32(obj, 0x2a60);
  buf = buf * 1664525u + 1013904223u + mix32((unsigned int)count, 77u + (unsigned int)g_rec_callno);
  U32(obj, 0x2a60) = buf;
  nc = rec_next_count(count, g_rec_callno);
  if (count == 0) S16(obj, 0x2a64) = (int16_t)rec_zero_mem(g_rec_callno); /* only the n > 16 arm passes 0 */
  g_rec_callno++;
  return (int)(((unsigned int)mix32((unsigned int)nc, 0x55u) << 16) | (uint16_t)nc);
}


/* ============================================================================
 * fake object, pointer normalisation
 * ========================================================================== */
static unsigned char g_objbuf[OBJSZ + 64];
static unsigned char g_snapbuf[OBJSZ + 64];
static unsigned char g_orcbuf[OBJBIG + 64];
#define G_OBJ (g_objbuf)
#define G_SNAP (g_snapbuf)
#define G_ORC (g_orcbuf)

/* harness-owned buffers that object pointer slots may reference */
static short g_tab[320];
static short g_bank[2560];
static short g_coef16[16 + 8];
static short g_pfcoef[42 + 8];
static short g_P[12 + 4];      /* precoder coefficient set (obj+0x2604) */
static uint16_t g_Q[64 + 4];   /* trellis output table (obj+0x2608) */

typedef struct {
  const unsigned char *base;
  size_t size;
} ext_t;
static ext_t g_ext[12];
static int g_next;
static void ext_reg(const void *p, size_t sz) {
  g_ext[g_next].base = (const unsigned char *)p;
  g_ext[g_next].size = sz;
  g_next++;
}

/* function identities (obj+0x2a28) */
static int RECORDER_BODY(void *obj, int count);
typedef struct {
  const void *addr;
  uint32_t id;
} fnid_t;
static fnid_t g_fn[3];

static uint32_t norm_val(const unsigned char *obj, uint32_t v) {
  int i;
  uintptr_t a = (uintptr_t)v;
  if (a >= (uintptr_t)obj && a < (uintptr_t)obj + OBJSZ) return 0x80000000u | (uint32_t)(a - (uintptr_t)obj);
  for (i = 0; i < g_next; i++)
    if (a >= (uintptr_t)g_ext[i].base && a < (uintptr_t)g_ext[i].base + g_ext[i].size)
      return 0x7d000000u | ((uint32_t)i << 16) | (uint32_t)(a - (uintptr_t)g_ext[i].base);
  for (i = 0; i < 3; i++)
    if (a == (uintptr_t)g_fn[i].addr) return g_fn[i].id;
  return v;
}

/* offsets (in the fake object) of every 4-byte slot that may hold a pointer */
static const unsigned int k_slots[] = {0x2a28, 0x2220, 0x2224, 0x35b0, 0x1460, 0x2074, 0x20cc, 0x2100, 0x80b8,
                                       0x80bc, 0x80c0, 0x80c4, 0x80c8, 0x9138, 0x913c, 0x9140, 0x9144, 0x9148,
                                       0x2604, 0x2608};
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

/* diff over the object: 256-byte blocks that are byte-identical are skipped (a raw-identical pointer slot is
 * identical after normalisation too); every other word is compared NORMALISED */
static unsigned int diff_obj(const unsigned char *obj, const unsigned char *snap) {
  unsigned int h = FNV0;
  size_t i, b;
  for (b = 0; b < OBJSZ; b += 256) {
    if (!memcmp(obj + b, snap + b, 256)) continue;
    for (i = b / 4; i < (b + 256) / 4; i++) {
      uint32_t a, c;
      memcpy(&a, obj + 4 * i, 4);
      memcpy(&c, snap + 4 * i, 4);
      if (g_isslot[i]) {
        a = norm_val(obj, a);
        c = norm_val(obj, c);
      }
      if (a != c) {
        uint32_t idx = (uint32_t)i;
        h = fnv(h, &idx, 4);
        h = fnv(h, &a, 4);
        if (g_verbose > 1) printf("   diffword off=%05x new=%08x\n", (unsigned int)(4 * i), a);
      }
    }
  }
  return h;
}
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
static unsigned int hash_ext(void) {
  unsigned int h = FNV0;
  h = fnv(h, g_tab, sizeof g_tab);
  h = fnv(h, g_bank, sizeof g_bank);
  h = fnv(h, g_coef16, sizeof g_coef16);
  h = fnv(h, g_pfcoef, sizeof g_pfcoef);
  h = fnv(h, g_P, sizeof g_P);
  h = fnv(h, g_Q, sizeof g_Q);
  return h;
}

/* ============================================================================
 * start-up table checks
 * ========================================================================== */
static void tables_check(void) {
#ifdef TEST_BLOB
  if (memcmp(quarter, port_quarter, 832)) die("quarter: blob != port copy");
  if (memcmp(smIndex, port_smIndex, 32)) die("smIndex: blob != port copy");
  if (memcmp(gInvertPat, port_gInvertPat, 32)) die("gInvertPat: blob != port copy");
  if (memcmp(xyz, port_xyz, 945 * 4)) die("xyz: blob != port copy");
#endif
  slots_init();
  g_fn[0].addr = (const void *)recorder;
  g_fn[0].id = 0x7f000001u;
  g_fn[1].addr = (const void *)scrambleGPC;
  g_fn[1].id = 0x7f000002u;
  g_fn[2].addr = (const void *)scrambleGPA;
  g_fn[2].id = 0x7f000003u;
  g_next = 0;
  ext_reg(g_tab, sizeof g_tab);
  ext_reg(g_bank, sizeof g_bank);
  ext_reg(g_coef16, sizeof g_coef16);
  ext_reg(g_pfcoef, sizeof g_pfcoef);
  ext_reg(g_P, sizeof g_P);
  ext_reg(g_Q, sizeof g_Q);
  ext_reg(Convolve16, 128);
  ext_reg(Convolve32, 128);
  ext_reg(Convolve64, 128);
}

/* ============================================================================
 * independent shell-mapping model (from scratch: no code shared with the port)
 * ========================================================================== */
typedef struct {
  int m;
  uint32_t g2[160]; /* zero beyond the support: the decode indexes them up to 8m-8 */
  uint64_t g4[160];
  uint64_t g8[160];
  uint64_t T[161]; /* T[k] = sum_{i<k} g8[i]; T[8m-7] = total = m^8 */
  int nk;          /* number of ring sums 8m-7 */
  int cnt;         /* number of T entries below 2^32 (what fits the object's dword table) */
} shell_t;

static void shell_build(shell_t *s, int m) {
  int i, j;
  memset(s, 0, sizeof *s);
  s->m = m;
  for (i = 0; i < 2 * m - 1; i++) s->g2[i] = (uint32_t)(i < m ? i + 1 : 2 * m - 1 - i);
  for (i = 0; i < 2 * m - 1; i++)
    for (j = 0; j < 2 * m - 1; j++) s->g4[i + j] += (uint64_t)s->g2[i] * s->g2[j];
  for (i = 0; i < 4 * m - 3; i++)
    for (j = 0; j < 4 * m - 3; j++) s->g8[i + j] += s->g4[i] * s->g4[j];
  s->nk = 8 * m - 7;
  s->T[0] = 0;
  for (i = 0; i < s->nk; i++) s->T[i + 1] = s->T[i] + s->g8[i];
  s->cnt = 0;
  for (i = 0; i < s->nk; i++)
    if (s->T[i] < 0x100000000ull) s->cnt = i + 1;
  if (s->cnt > 128) s->cnt = 128;
}

/* fill mode for the T entries beyond `cnt`: 0 = 0xffffffff (sorted), 1 = 0, 2 = random junk */
static void shell_install(unsigned char *o, const shell_t *s, int fill) {
  int i;
  uint16_t *B = (uint16_t *)(void *)OB(o, 0x2628);
  uint16_t *A = (uint16_t *)(void *)OB(o, 0x2728);
  uint32_t *T = (uint32_t *)(void *)OB(o, 0x2828);
  for (i = 0; i < 128; i++) {
    B[i] = (i < 2 * s->m - 1) ? (uint16_t)s->g2[i] : 0;
    A[i] = (i < 4 * s->m - 3) ? (uint16_t)s->g4[i] : 0;
    if (i < s->cnt) T[i] = (uint32_t)s->T[i];
    else T[i] = fill == 0 ? 0xffffffffu : (fill == 1 ? 0u : lcg_next());
  }
}

/* decode a shell index into eight ring numbers exactly as the V.34 shell mapping defines them:
 * ring sum m0 = the largest k with T[k] <= v; the remainder picks the (first-half sum j1) block in
 * ascending j1 with blocks of g4[j1]*g4[m0-j1] indices (slow index q selects the second half, fast index
 * rem1 the first half); each half is split again by g2 blocks; a pair (idx, sum s) is the idx-th pair
 * (a,b), a ascending, a,b in [0,L), a+b = s.  Returns 0 if v >= total. */
static int shell_decode(const shell_t *s, uint32_t v, int x[8], int *plo) {
  int m = s->m, lo = -1, k, t, j1 = -1, j2 = -1, j3 = -1, m2;
  uint64_t rem, blk, R1, q, rem1, R2, R3, qa, ra, qb, rb;
  int sums[4];
  uint64_t idx[4];
  if ((uint64_t)v >= s->T[s->nk]) return 0;
  for (k = 0; k < s->nk; k++)
    if (s->T[k] <= v) lo = k;
  rem = v - s->T[lo];
  for (t = 0; t <= lo; t++) {
    blk = s->g4[t] * s->g4[lo - t];
    if (rem < blk) { j1 = t; break; }
    rem -= blk;
  }
  if (j1 < 0) return 0;
  R1 = rem;
  q = R1 / s->g4[j1];
  rem1 = R1 % s->g4[j1];
  rem = rem1;
  for (t = 0; t <= j1; t++) {
    blk = (uint64_t)s->g2[t] * s->g2[j1 - t];
    if (rem < blk) { j2 = t; break; }
    rem -= blk;
  }
  if (j2 < 0) return 0;
  R2 = rem;
  qa = R2 / s->g2[j2];
  ra = R2 % s->g2[j2];
  m2 = lo - j1;
  rem = q;
  for (t = 0; t <= m2; t++) {
    blk = (uint64_t)s->g2[t] * s->g2[m2 - t];
    if (rem < blk) { j3 = t; break; }
    rem -= blk;
  }
  if (j3 < 0) return 0;
  R3 = rem;
  qb = R3 / s->g2[j3];
  rb = R3 % s->g2[j3];
  sums[0] = j2; idx[0] = ra;
  sums[1] = j1 - j2; idx[1] = qa;
  sums[2] = j3; idx[2] = rb;
  sums[3] = m2 - j3; idx[3] = qb;
  for (k = 0; k < 4; k++) {
    int amin = sums[k] - (m - 1);
    int a, b;
    if (amin < 0) amin = 0;
    a = amin + (int)idx[k];
    b = sums[k] - a;
    x[2 * k] = a;
    x[2 * k + 1] = b;
  }
  *plo = lo;
  return 1;
}

/* ============================================================================
 * oracle hooks + coverage
 * ========================================================================== */
static int g_viol;
static int g_viol_kind[4];
static long g_covtmp[CV_MAX];
static long g_cov[CV_MAX];
static uint16_t g_orc_x[8];
static uint32_t g_orc_v;
static int g_orc_lo, g_orc_m, g_orc_rings;

int tr_quarter(int idx) {
  if (idx < 0 || idx > 415) { g_viol = 1; g_viol_kind[0]++; return 0; }
  return idx;
}
int tr_ginv(int idx) {
  if (idx < 0 || idx > 15) { g_viol = 1; g_viol_kind[1]++; return 0; }
  return idx;
}
int tr_div(int d) {
  if (d == 0) { g_viol = 1; g_viol_kind[2]++; return 1; }
  return d;
}
int tr_peel(int t, int lo) {
  if (t > lo) { g_viol = 1; g_viol_kind[3]++; return 1; }
  return 0;
}
void tr_rings(const uint16_t x[8], uint32_t v, int lo, int m) {
  memcpy(g_orc_x, x, 16);
  g_orc_v = v;
  g_orc_lo = lo;
  g_orc_m = m;
  g_orc_rings = 1;
}
void tr_cov(int id) { g_covtmp[id]++; }

static long g_oracle_runs, g_oracle_rejects;

/* run the oracle on a scratch copy of `o`; returns 1 if the state is inside the valid domain.  The
 * recorder bookkeeping (hash, call number) is saved and restored so the real call is unaffected. */
static int oracle_ok(const unsigned char *o) {
  unsigned int sh = g_rec_h;
  long sn = g_rec_n;
  int son = g_rec_on, scn = g_rec_callno;
  void *sobj = g_rec_obj;
  memcpy(G_ORC, o, OBJSZ);
  g_viol = 0;
  g_orc_rings = 0;
  memset(g_covtmp, 0, sizeof g_covtmp);
  g_rec_obj = G_ORC;
  g_rec_on = 1;
  g_rec_callno = 0;
  g_rec_h = FNV0;
  g_rec_n = 0;
  oracle_modulatevector(G_ORC);
  g_rec_h = sh;
  g_rec_n = sn;
  g_rec_on = son;
  g_rec_callno = scn;
  g_rec_obj = sobj;
  g_oracle_runs++;
  if (g_viol) {
    g_oracle_rejects++;
    return 0;
  }
  return 1;
}

#define RING_BASE 0x2228u
#define RING_END 0x25c0u
#define RING_ENTRIES ((RING_END - RING_BASE) / 4u) /* 230 */

typedef struct {
  int nout;        /* predicted modulator output count */
  int ring_wrap;   /* the ring write pointer wrapped during the call */
  int count_wrap;  /* the 16-bit counter wrapped */
  int bulk;        /* bulk-delay path taken (flag && nout > 0) */
  int rd_wrap, wr_wrap, rd_ge_len, wr_ge_len;
  int flag;
} txm_t;

/* Configure a fake object so that txmit (and its callees) run on valid pointers. Returns a model
 * of what the call will do (independent of the code under test). */
static void setup_tx(unsigned char *o, txm_t *m) {
  int taps, stride, idx, limit, step, phase, len, tp;
  int i, ringk, bulk_on;
  unsigned int rd, wr, blen;
  int boff;

  memset(m, 0, sizeof *m);
  /* ---- modulator (m = obj+0x1450) ---- */
  switch (lcg_mod(8)) {
    case 0: taps = 0; break;
    case 1: taps = -rr(1, 3); break;
    default: taps = rr(1, 40); break;
  }
  idx = rr(0, 5);
  {
    int hi = 126 - idx - (taps > 0 ? taps : 0);
    int lo = taps > 1 ? taps : 1;
    stride = rr(lo, hi > lo ? hi : lo);
  }
  limit = rr(1, 36);
  step = rr(1, limit + 3);
  if (lcg_mod(6) == 0) step = 1;
  if (lcg_mod(4) == 0)
    phase = limit + rr(0, 3); /* zero-output path */
  else
    phase = rr(0, limit - 1);
  len = rr(1, 128);
  tp = (lcg_mod(4) == 0) ? len - 1 : rr(0, len - 1);
  I32(o, 0x1450) = taps;
  I32(o, 0x1454) = step;
  I32(o, 0x1458) = limit;
  I32(o, 0x145c) = phase;
  PTRS(o, 0x1460) = g_tab;
  I32(o, 0x1464) = len;
  I32(o, 0x1468) = tp;
  PTRS(o, 0x2074) = g_bank;
  PTRS(o, 0x2100) = g_coef16;
  I32(o, 0x220c) = idx;
  I32(o, 0x2210) = stride;
  {
    int amp = rr(0, 12);
    for (i = 0; i < 320; i++) g_tab[i] = (short)(rshort() >> (amp > 8 ? 3 : 0));
    for (i = 0; i < 2560; i++) g_bank[i] = (short)(rshort() >> rr(0, 5));
    for (i = 0; i < 16 + 8; i++) g_coef16[i] = (short)(rshort() >> rr(0, 3));
    for (i = 0; i < 42 + 8; i++) g_pfcoef[i] = (short)(rshort() >> rr(0, 4));
  }
  PTRS(o, 0x20cc) = g_pfcoef;
  U8(o, 0x20dc) = (unsigned char)lcg_next();
  m->nout = (phase < limit) ? (limit - phase + step - 1) / step : 0;

  /* ---- tx history ring ---- */
  switch (lcg_mod(4)) {
    case 0: ringk = (int)RING_ENTRIES - 1; break;
    case 1: ringk = (int)RING_ENTRIES - 1 - rr(0, 8); break;
    default: ringk = rr(0, (int)RING_ENTRIES - 1); break;
  }
  PTRS(o, 0x2224) = OB(o, RING_BASE + 4 * ringk);
  U16(o, 0x221c) = (lcg_mod(4) == 0) ? 0xffffu : rnd16();
  S16(o, 0x25d4) = (lcg_mod(6) == 0) ? (short)-32768 : rshort();
  m->ring_wrap = (m->nout > 0) && (ringk + m->nout >= (int)RING_ENTRIES);
  m->count_wrap = (m->nout > 0) && ((unsigned int)U16(o, 0x221c) + (unsigned int)m->nout > 0xffffu);

  /* ---- 0x25c2 word: bit 0 = tap select, byte 0x25c3 bit 1 = bulk-delay path enable ---- */
  bulk_on = lcg_mod(10) < 6;
  m->flag = bulk_on;
  U8(o, 0x25c3) = (unsigned char)((lcg_next() & 0xfd) | (bulk_on ? 2 : 0));
  U8(o, 0x25c2) = (unsigned char)((lcg_next() & 0xfe) | (unsigned char)(lcg_next() >> 31));

  /* ---- bulk-delay ring {rd 0x35a8, wr 0x35ac, base 0x35b0, len 0x35b4} ---- */
  boff = rr(0, 63);
  PTRS(o, 0x35b0) = OB(o, 0x35b8 + 2 * boff);
  switch (lcg_mod(6)) {
    case 0: blen = (unsigned int)rr(1, 4); break;
    case 1: blen = 0x2580u; break;
    case 2: blen = (unsigned int)rr(1, 40); break;
    case 3: blen = 0u; break;
    case 4: blen = 0x80000000u | (lcg_next() & 0x7fffffffu); break; /* "negative" as int: UNSIGNED compares */
    default: blen = (unsigned int)rr(1, 0x2580); break;
  }
  {
    unsigned int maxi = blen > 0x1fffu ? 0x1fffu : blen; /* keep every index inside the object */
    switch (lcg_mod(4)) {
      case 0: rd = maxi ? maxi - 1 : 0; break;
      case 1: rd = maxi; break;
      case 2: rd = (unsigned int)rr(0, (int)maxi + 1); if (rd > 0x1fffu) rd = 0x1fffu; break;
      default: rd = (unsigned int)rr(0, 0x1fff); break;
    }
    switch (lcg_mod(4)) {
      case 0: wr = maxi ? maxi - 1 : 0; break;
      case 1: wr = maxi; break;
      case 2: wr = (unsigned int)rr(0, (int)maxi + 1); if (wr > 0x1fffu) wr = 0x1fffu; break;
      default: wr = (unsigned int)rr(0, 0x1fff); break;
    }
  }
  U32(o, 0x35a8) = rd;
  U32(o, 0x35ac) = wr;
  U32(o, 0x35b4) = blen;
  m->bulk = bulk_on && m->nout > 0;
  if (m->bulk) {
    unsigned int r = rd, w = wr;
    if (rd >= blen) m->rd_ge_len = 1;
    if (wr >= blen) m->wr_ge_len = 1;
    for (i = 0; i < m->nout; i++) {
      r = r + 1;
      if (!(blen > r)) { r = 0; m->rd_wrap = 1; }
      w = w + 1;
      if (!(blen > w)) { w = 0; m->wr_wrap = 1; }
    }
  }

  /* ---- near / far echo cancellers: valid write pointers, wrap edges ---- */
  {
    int e;
    for (e = 0; e < 2; e++) {
      unsigned int st = e == 0 ? 0x80b8u : 0x9138u;
      unsigned int bb = st + 0x140u + 2u * (unsigned)rr(0, 60); /* base .. base + 2*dl within the region */
      int dl;
      unsigned int wp;
      switch (lcg_mod(3)) {
        case 0: dl = rr(1, 6); break;
        case 1: dl = rr(1, 60); break;
        default: dl = rr(1, 0x678); break;
      }
      switch (lcg_mod(3)) {
        case 0: wp = bb + 2u * (unsigned)(dl - 1); break;
        case 1: wp = bb + 2u * (unsigned)(dl > 1 ? dl - 2 : 0); break;
        default: wp = bb + 2u * (unsigned)rr(0, dl - 1); break;
      }
      PTRS(o, st + 0) = OB(o, wp);
      PTRS(o, st + 4) = OB(o, bb);
      I32(o, st + 0x18) = dl;
      if (e == 1 && lcg_mod(8) == 0) { /* far aliases near: the two updates hit the same slots, so their ORDER shows */
        PTRS(o, st + 0) = PTRS(o, 0x80b8 + 0);
        PTRS(o, st + 4) = PTRS(o, 0x80b8 + 4);
        I32(o, st + 0x18) = I32(o, 0x80b8 + 0x18);
      }
      /* the remaining pointer fields stay scrubbed junk (not read by V34EchoUpdateDelayLine) */
    }
  }
}


/* ============================================================================
 * state generation
 * ========================================================================== */
typedef struct {
  int m, W, n, fill, fnmode, steered, model_ok, k;
  shell_t sh;
} mvcfg_t;

static int floor_log2_u64(uint64_t x) {
  int r = -1;
  while (x) {
    r++;
    x >>= 1;
  }
  return r;
}
static int wmax_for(int m) {
  int w = 0;
  while (w < 8 && ((long)m << (w + 1)) <= 416) w++;
  return w;
}
static int pick_m(void) {
  switch (lcg_mod(10)) {
    case 0: return 1;
    case 1: case 2: return rr(2, 4);
    case 3: case 4: case 5: return rr(5, 14);
    case 6: return 15;
    case 7: return 16;
    case 8: return rr(17, 18);
    default: return rr(2, 14);
  }
}
static void fill_coefs(short *p, int n) {
  int i;
  for (i = 0; i < n; i++) p[i] = (short)(rshort() >> rr(0, 13));
}

static void gf_set_scrambler_state(unsigned char *o) {
  /* override buffer (obj+0x118 + 4*idx), enable word 0x2214, index 0x21c < limit 0x218 <= 64 */
  U32(o, 0x218) = (uint32_t)rr(0, 64);
  U32(o, 0x21c) = (lcg_mod(3) == 0) ? (uint32_t)rr(0, 70) : (uint32_t)rr(0, (int)U32(o, 0x218) + 1);
  U16(o, 0x2214) = (lcg_mod(3) == 0) ? 0 : rnd16() | 1;
}
static void gf_set_fn(unsigned char *o, int fnmode) {
  PTRS(o, 0x2a28) = fnmode == 0 ? (void *)recorder : (fnmode == 1 ? (void *)scrambleGPC : (void *)scrambleGPA);
}

/* dynamic state: trellis words, phase word, D, precoder history, e4c / aa74 block */
static void mv_dynamic_fields(unsigned char *o) {
  unsigned int N;
  int i;
  for (i = 0; i < 6; i++) {
    uint16_t w = rnd16();
    switch (lcg_mod(8)) {
      case 0: w = 0; break;
      case 1: w = 0xffff; break;
      case 2: w &= 0x3f; break;
      default: break;
    }
    U16(o, 0x260c + 2 * i) = w;
  }
  U16(o, 0x2618) = rnd16();
  U16(o, 0x261a) = rnd16();
  for (i = 0; i < 6; i++) S16(o, 0x25f8 + 2 * i) = (int16_t)(rshort() >> rr(0, 10));
  N = U16(o, 0x25e0);
  U16(o, 0xe4c) = (lcg_mod(4) == 0) ? 0 : (uint16_t)(lcg_next() | 1u);
  if (lcg_mod(6) == 0) U16(o, 0xe4c) = 0x100; /* nonzero with a zero low byte: a 16-bit test */
  switch (lcg_mod(8)) {
    case 0: I32(o, 0xaa74) = (int32_t)N; break;
    case 1: I32(o, 0xaa74) = (int32_t)N - 1; break;
    case 2: I32(o, 0xaa74) = (int32_t)N + 1; break;
    case 3: I32(o, 0xaa74) = (int32_t)(lcg_next() & 0x7fu); break;
    case 4: I32(o, 0xaa74) = -(int32_t)(lcg_next() & 0xffu); break;
    case 5: I32(o, 0xaa74) = (int32_t)0x7fffffff; break;
    case 6: I32(o, 0xaa74) = (int32_t)0x80000000u; break;
    default: I32(o, 0xaa74) = (int32_t)lcg_next(); break;
  }
}

/* counters E (mod word 0x25e2) and Fc (mod short 0x2620): drawn relative to the limits in the object */
static void mv_counters(unsigned char *o) {
  int N2 = U16(o, 0x25e2), K2 = S16(o, 0x2620);
  switch (lcg_mod(6)) {
    case 0: U16(o, 0x261c) = (uint16_t)(N2 > 0 ? N2 - 1 : 0); break;
    case 1: U16(o, 0x261c) = (uint16_t)(N2 > 1 ? N2 - 2 : 0); break;
    case 2: U16(o, 0x261c) = (uint16_t)rr(0, N2 + 1); break;
    case 3: U16(o, 0x261c) = (lcg_mod(2) ? 0x7fff : 0xffff); break; /* E+1 wraps as a short */
    default: U16(o, 0x261c) = (uint16_t)rr(0, 3); break;
  }
  switch (lcg_mod(5)) {
    case 0: S16(o, 0x261e) = (int16_t)(K2 > 1 ? K2 - 2 : 0); break; /* Fc+1 == 2K-1 (last valid index) */
    case 1: S16(o, 0x261e) = (int16_t)(K2 > 0 ? K2 - 1 : 0); break; /* Fc+1 == 2K: wraps to 0 */
    case 2: S16(o, 0x261e) = (int16_t)(K2 + 1); break;
    case 3: S16(o, 0x261e) = -1; break;                             /* Fc+1 == 0 */
    default: S16(o, 0x261e) = (int16_t)rr(0, K2 > 1 ? K2 - 1 : 1); break;
  }
}

/* configuration words: cfg, limits, scale, Rs, coefficient / trellis-table pointers */
static void mv_static_fields(unsigned char *o) {
  int N2, K2;
  /* cfg word: 0x40 = 64-state shift-register form, anything else the LFSR form (multiplier sext16(cfg)) */
  switch (lcg_mod(10)) {
    case 0: case 1: case 2: U16(o, 0x25f6) = 0x40; break;
    case 3: case 4: U16(o, 0x25f6) = 0x20; break;
    case 5: U16(o, 0x25f6) = 0; break;
    case 6: U16(o, 0x25f6) = 0x8040; break; /* != 0x40 as a 16-bit compare */
    case 7: U16(o, 0x25f6) = (uint16_t)(0xff00u | (lcg_next() & 0xff)); break; /* negative */
    case 8: U16(o, 0x25f6) = 0x140; break;
    default: U16(o, 0x25f6) = rnd16(); break;
  }
  switch (lcg_mod(6)) {
    case 0: N2 = 1; break;
    case 1: N2 = 2; break;
    case 2: N2 = rr(3, 5); break;
    case 3: N2 = 0; break;
    default: N2 = rr(1, 40); break;
  }
  if (lcg_mod(8) == 0) { /* limits with bit 15 set: the E compare zero-extends 0x25e2 (a WORD compared as int) */
    static const int wn[] = {0x7fff, 0x8000, 0xffff, 0x8001};
    N2 = wn[lcg_mod(4)];
  }
  U16(o, 0x25e2) = (uint16_t)N2;
  switch (lcg_mod(4)) {
    case 0: K2 = 14; break;
    case 1: K2 = 16; break;
    case 2: K2 = rr(1, 16); break;
    default: K2 = 0; break;
  }
  if (lcg_mod(8) == 0) { /* short limit that is negative / large: the Fc compare is a SIGNED 16-bit compare */
    static const int wk[] = {-1, -32768, 0x7fff, -16, 100};
    K2 = wk[lcg_mod(5)];
  }
  S16(o, 0x2620) = (int16_t)K2;
  switch (lcg_mod(6)) {
    case 0: S16(o, 0x2622) = (int16_t)0x4000; break;
    case 1: S16(o, 0x2622) = (int16_t)rr(-300, 300); break;
    case 2: S16(o, 0x2622) = (int16_t)0x7fff; break;
    case 3: S16(o, 0x2622) = (int16_t)0x8000; break;
    default: S16(o, 0x2622) = rshort(); break;
  }
  switch (lcg_mod(10)) {
    case 0: case 1: case 2: case 3: S16(o, 0x2626) = 1; break;
    case 4: case 5: case 6: case 7: S16(o, 0x2626) = 2; break;
    case 8: S16(o, 0x2626) = (int16_t)rr(-4, 6); break;
    default: S16(o, 0x2626) = rshort(); break;
  }
  /* coefficient set: harness buffer or (like initdigital) the object's own obj+0x2a68 */
  if (lcg_mod(2)) {
    fill_coefs(g_P, 12);
    PTRS(o, 0x2604) = g_P;
  } else {
    fill_coefs((short *)(void *)OB(o, 0x2a68), 12);
    PTRS(o, 0x2604) = OB(o, 0x2a68);
  }
  /* trellis output table: harness buffer or the blob's / port's own Convolve tables */
  switch (lcg_mod(10)) {
    case 0: PTRS(o, 0x2608) = (void *)Convolve16; break;
    case 1: PTRS(o, 0x2608) = (void *)Convolve32; break;
    case 2: PTRS(o, 0x2608) = (void *)Convolve64; break;
    default: {
      int i;
      for (i = 0; i < 64 + 4; i++) g_Q[i] = rnd16();
      PTRS(o, 0x2608) = g_Q;
    } break;
  }
}

/* getFrame programme: N / a6 / a8 / a4 with boundary steering (as in tier 5a), n, W */
static int pick_t1_source(unsigned char *o) {
  int a4;
  switch (lcg_mod(6)) {
    case 0: a4 = 8; break;
    case 1: a4 = 9; break;
    case 2: a4 = 0; break;
    case 3: a4 = 0xffff; break;
    default: a4 = rr(0, 20); break;
  }
  U16(o, 0x25e4) = (uint16_t)a4;
  return a4 < 9 ? 1 : 2; /* t1 */
}
static void mv_program(unsigned char *o, int n, int W) {
  unsigned int N, a6, a8;
  N = (lcg_mod(3) == 0) ? (unsigned int)rr(1, 40) : (unsigned int)rr(1, 0xffff);
  a6 = (lcg_mod(3) == 0) ? (unsigned int)rr(0, 40) : rnd16();
  switch (lcg_mod(6)) {
    case 0: a8 = (N - a6) & 0xffffu; break;
    case 1: a8 = (N - 1u - a6) & 0xffffu; break;
    default: a8 = rnd16(); break;
  }
  U16(o, 0x25e0) = (uint16_t)N;
  U16(o, 0x25e6) = (uint16_t)a6;
  U16(o, 0x25e8) = (uint16_t)a8;
  S16(o, 0x25ee) = (int16_t)n;
  S16(o, 0x25f0) = (int16_t)n;
  S16(o, 0x25f4) = (int16_t)W;
}

/* build a valid state for an 8-point block generation.  fnmode: 0 recorder, 1 GPC, 2 GPA. */
static void mv_build(unsigned char *o, mvcfg_t *c, int fnmode, int steer_kind) {
  txm_t tm;
  int t1, nmax, n;
  uint64_t total;

  fill_random(o, OBJSZ);
  scrub_slots(o);
  setup_tx(o, &tm);
  U32(o, 0x25d0) = lcg_next();

  memset(c, 0, sizeof *c);
  c->fnmode = fnmode;
  c->m = pick_m();
  c->W = (lcg_mod(4) == 0) ? 0 : ((lcg_mod(3) == 0) ? wmax_for(c->m) : rr(0, wmax_for(c->m)));
  shell_build(&c->sh, c->m);
  c->fill = (lcg_mod(10) == 0) ? (int)(1 + lcg_mod(2)) : 0;
  c->model_ok = (c->fill == 0);
  shell_install(o, &c->sh, c->fill);
  S16(o, 0x25f2) = (int16_t)c->m; /* the ring limit L (block A +0x12) */

  t1 = pick_t1_source(o);
  total = c->sh.T[c->sh.nk];
  nmax = floor_log2_u64(total);
  if (nmax > 32) nmax = 32;
  if (nmax <= 0 || lcg_mod(12) == 0) {
    n = -(int)lcg_mod((unsigned int)t1 + 1u); /* 0, -1, (-2): v = 0 */
  } else {
    switch (lcg_mod(10)) {
      case 0: case 1: case 2: n = nmax; break;
      case 3: case 4: n = nmax < 16 ? nmax : 16; break;
      default: n = rr(1, nmax); break;
    }
  }
  c->n = n;
  mv_program(o, n, c->W);
  mv_static_fields(o);
  mv_counters(o);
  mv_dynamic_fields(o);
  gf_set_fn(o, fnmode);
  if (fnmode) gf_set_scrambler_state(o);
  S16(o, 0x2aa2) = 8;
  c->k = 8;

  /* bit reader: steer v = (buf >> count) & lsbMask[n] for the recorder when the first extraction is the plain arm */
  U32(o, 0x2a60) = lcg_next();
  S16(o, 0x2a64) = (int16_t)rr(0, 15);
  if (fnmode == 0 && n >= 1 && n <= 16 && steer_kind != 0) {
    int count = rr(0, 15);
    uint32_t v = 0, buf;
    uint64_t lim = (uint64_t)1 << n;
    if (lim > total) lim = total;
    switch (steer_kind) {
      case 1: v = (uint32_t)(lcg_next() % lim); break;
      default: { /* boundary: T[k], T[k]-1, T[k+1]-1, 0, total-1 when they fit in n bits */
        int kk = (int)lcg_mod((unsigned int)c->sh.nk);
        uint64_t cand;
        switch (lcg_mod(5)) {
          case 0: cand = c->sh.T[kk]; break;
          case 1: cand = c->sh.T[kk] ? c->sh.T[kk] - 1 : 0; break;
          case 2: cand = c->sh.T[kk + 1] - 1; break;
          case 3: cand = 0; break;
          default: cand = total - 1; break;
        }
        v = (cand < lim) ? (uint32_t)cand : (uint32_t)(lcg_next() % lim);
      } break;
    }
    buf = (v << count) | (lcg_next() & (((uint32_t)1 << count) - 1u));
    if (count + n < 32) buf |= lcg_next() << (count + n);
    U32(o, 0x2a60) = buf;
    S16(o, 0x2a64) = (int16_t)count;
    c->steered = 1;
  }
}

/* ============================================================================
 * post-call model of the tail and of the e4c / aa74 block (checked on BOTH sides)
 * ========================================================================== */
static long g_post_calls, g_post_gen, g_post_nl, g_post_e4c_set, g_post_e4c_inc;
static void post_check(const unsigned char *o, const unsigned char *snap, const char *what) {
  unsigned int k16 = U16(snap, 0x2aa2);
  int gen = (uint16_t)k16 == 8;
  unsigned int kk = gen ? 0u : k16;
  unsigned int w25c2 = U16(snap, 0x25c2);
  int32_t exp_aa74 = I32(snap, 0xaa74);
  unsigned int exp_2214 = U16(snap, 0x2214), exp_25c2 = w25c2;
  const short *in = (const short *)(const void *)(o + 0x2a80 + 4 * (int)(int16_t)kk);
  int nl = (U8(snap, 0x25c3) & 0x40) != 0;
  uint32_t ref;

  if (gen && !(w25c2 & 0x10u) && U16(snap, 0xe4c) != 0) {
    exp_aa74 = (int32_t)((uint32_t)exp_aa74 + 1u);
    g_post_e4c_inc++;
    if (I32(snap, 0xaa74) >= (int32_t)U16(snap, 0x25e0)) {
      exp_2214 = 1;
      exp_25c2 = (uint16_t)(w25c2 | 0x10u);
      g_post_e4c_set++;
    }
  }
  if (I32(o, 0xaa74) != exp_aa74 || U16(o, 0x2214) != exp_2214 || U16(o, 0x25c2) != exp_25c2) {
    fprintf(stderr, "%s: e4c/aa74 block differs from the harness model (aa74 %08x/%08x 2214 %04x/%04x 25c2 %04x/%04x)\n",
            what, (unsigned)I32(o, 0xaa74), (unsigned)exp_aa74, U16(o, 0x2214), exp_2214, U16(o, 0x25c2), exp_25c2);
    exit(3);
  }
  if (U16(o, 0x2aa2) != (uint16_t)(kk + 1u)) {
    fprintf(stderr, "%s: k+1 word %04x, expected %04x\n", what, U16(o, 0x2aa2), (uint16_t)(kk + 1u));
    exit(3);
  }
  if (nl) {
    short out[2];
    V34nlencoder(in, out);
    memcpy(&ref, out, 4);
    g_post_nl++;
  } else {
    memcpy(&ref, in, 4);
  }
  if (U32(o, 0x25d0) != ref) {
    fprintf(stderr, "%s: point handed to txmit %08x, expected %08x (nl=%d)\n", what, U32(o, 0x25d0), ref, nl);
    exit(3);
  }
  g_post_calls++;
  if (gen) g_post_gen++;
}

/* ============================================================================
 * one call: oracle-validate (re-draw while invalid), then run the code under test
 * ========================================================================== */
static long g_model_checked, g_model_skipped, g_redraws, g_v_steered, g_n_gt16, g_n_le0, g_v_ge31;
static long g_m_cnt[20], g_w_cnt[10];

/* returns the log hash of the call; `snap` holds the pre-call object afterwards */
static unsigned int call_once(unsigned char *o, int fnmode, int gen_expected) {
  unsigned int h;
  memcpy(G_SNAP, o, OBJSZ);
  g_rec_h = FNV0;
  g_rec_n = 0;
  g_rec_callno = 0;
  g_rec_obj = o;
  g_rec_on = 1;
  modulatevector(o);
  g_rec_on = 0;
  post_check(o, G_SNAP, "modulatevector");
  h = diff_obj(o, G_SNAP);
  h = fnv32(h, hash_ext());
  if (fnmode == 0) {
    h = fnv32(fnv32(h, g_rec_h), (uint32_t)g_rec_n);
    (void)gen_expected;
  }
  return h;
}

static void merge_cov(void) {
  int i;
  for (i = 0; i < CV_MAX; i++) g_cov[i] += g_covtmp[i];
}

/* validate a freshly built state with the oracle, check the ring model; re-draw on rejection */
static void model_check(const unsigned char *o, const mvcfg_t *c) {
  int x[8], lo, i;
  shell_t sh;
  (void)o;
  if (!c->model_ok || !g_orc_rings) { g_model_skipped++; return; }
  sh = c->sh;
  if (!shell_decode(&sh, g_orc_v, x, &lo)) {
    /* the oracle accepted a v the model calls invalid (>= total): the peel loops ran off the table without
     * tripping a hook - only possible if T is truncated (m >= 17); count it, do not check */
    g_model_skipped++;
    return;
  }
  if (lo != g_orc_lo) { fprintf(stderr, "MODEL: lo %d (port oracle) vs %d (model), v=%u m=%d\n", g_orc_lo, lo, g_orc_v, c->m); exit(3); }
  for (i = 0; i < 8; i++)
    if (x[i] != (int)g_orc_x[i]) {
      fprintf(stderr, "MODEL: ring[%d] = %d (port oracle) vs %d (model), v=%u m=%d\n", i, g_orc_x[i], x[i], g_orc_v, c->m);
      exit(3);
    }
  g_model_checked++;
}

/* build states until one passes the oracle */
static void build_valid(unsigned char *o, mvcfg_t *c, int fnmode, int steer_kind) {
  int tries = 0;
  for (;;) {
    mv_build(o, c, fnmode, steer_kind);
    if (oracle_ok(o)) break;
    g_redraws++;
    if (++tries > 100000) die("oracle rejected 100000 consecutive states");
  }
  merge_cov();
  model_check(o, c);
  g_m_cnt[c->m]++;
  g_w_cnt[c->W]++;
  if (c->steered) g_v_steered++;
  if (c->n > 16) g_n_gt16++;
  if (c->n <= 0) g_n_le0++;
  if (g_orc_rings && g_orc_v >= 0x80000000u) g_v_ge31++;
}

/* ============================================================================
 * coverage report
 * ========================================================================== */
static void cov_full(int want_nogen, int want_eq) {
  int i;
  char nm[64];
  need("oracle: generation path (k == 8)", g_cov[CV_GEN]);
  if (want_nogen) need("oracle: plain path (k != 8)", g_cov[CV_NOGEN]);
  need("oracle: e4c block skipped, bit 4 of 0x25c2 set", g_cov[CV_E4C_BIT4]);
  need("oracle: e4c block skipped, word 0xe4c == 0", g_cov[CV_E4C_ZERO]);
  need("oracle: e4c counter < N (no flag)", g_cov[CV_E4C_LT]);
  need("oracle: e4c counter >= N (flag set)", g_cov[CV_E4C_GE]);
  need("oracle: binary search T[mid] > v", g_cov[CV_BS_ARM1]);
  need("oracle: binary search T[mid] <= v", g_cov[CV_BS_ARM2]);
  if (want_eq) need("oracle: binary search T[mid] == v", g_cov[CV_BS_EQ]);
  for (i = 0; i < 9; i++) {
    sprintf(nm, "oracle: peel phase %d, %s", i / 3 + 1, (i % 3) == 0 ? "0 iterations" : ((i % 3) == 1 ? "1 iteration" : ">= 2 iterations"));
    need(nm, g_cov[CV_PEEL + i]);
  }
  for (i = 0; i < 4; i++) {
    sprintf(nm, "oracle: pair %d sum < L (plain)", i);
    need(nm, g_cov[CV_PAIR_NORM + i]);
    sprintf(nm, "oracle: pair %d sum >= L (reflected)", i);
    need(nm, g_cov[CV_PAIR_ALT + i]);
  }
  need("oracle: precoder acc negative", g_cov[CV_QUANT_NEG]);
  need("oracle: precoder acc non-negative", g_cov[CV_QUANT_POS]);
  need("oracle: quantiser adds t", g_cov[CV_QUANT_ADDT]);
  need("oracle: quantiser does not add t", g_cov[CV_QUANT_NOADD]);
  for (i = 0; i < 4; i++) {
    sprintf(nm, "oracle: first-point rotation %d", i);
    need(nm, g_cov[CV_ROT1 + i]);
    sprintf(nm, "oracle: second-point rotation %d", i);
    need(nm, g_cov[CV_ROT2 + i]);
  }
  need("oracle: second-point parity bit 0", g_cov[CV_B0]);
  need("oracle: second-point parity bit 1", g_cov[CV_B1]);
  need("oracle: trellis cfg == 0x40 (64-state form)", g_cov[CV_CFG64]);
  need("oracle: trellis LFSR form, C0 odd", g_cov[CV_LFSR_ODD]);
  need("oracle: trellis LFSR form, C0 even", g_cov[CV_LFSR_EVEN]);
  need("oracle: trellis LFSR form, negative cfg", g_cov[CV_LFSR_CFGNEG]);
  need("oracle: counter E+1 < N2", g_cov[CV_CNT_INC]);
  need("oracle: counter wrap, Fc+1 < 2K (gInvertPat read)", g_cov[CV_CNT_FC]);
  need("oracle: counter wrap, Fc wraps to 0", g_cov[CV_CNT_ZERO]);
  need("oracle: tail with nlencoder", g_cov[CV_TAIL_NL]);
  need("oracle: tail plain copy", g_cov[CV_TAIL_PLAIN]);
  need("oracle: W == 0", g_cov[CV_W0]);
  need("oracle: W > 0", g_cov[CV_WPOS]);
}
static void cov_inputs(void) {
  int i;
  char nm[64];
  printf("  oracle runs=%ld rejects=%ld (kinds: quarter %d, gInvertPat %d, zero-divisor %d, peel overrun %d)\n",
         g_oracle_runs, g_oracle_rejects, g_viol_kind[0], g_viol_kind[1], g_viol_kind[2], g_viol_kind[3]);
  printf("  model-checked ring decodes=%ld skipped=%ld\n", g_model_checked, g_model_skipped);
  need("oracle rejected some states (the oracle demonstrably fires)", g_oracle_rejects);
  need("ring model checked", g_model_checked);
  for (i = 1; i <= 18; i++) {
    sprintf(nm, "m == %d", i);
    need(nm, g_m_cnt[i]);
  }
  need("W == 0 blocks", g_w_cnt[0]);
  need("W >= 1 blocks", g_w_cnt[1] + g_w_cnt[2] + g_w_cnt[3] + g_w_cnt[4]);
  need("n > 16 (word arm of getFrame)", g_n_gt16);
  need("n <= 0 (v == 0)", g_n_le0);
  need("oracle: binary search compared a table entry >= 2^31 (unsigned compare matters)", g_cov[CV_BS_TBIG]);
  printf("  v >= 2^31 reached: %ld (getFrame yields at most 16 + (n&15) bits, so v < 2^31)\n", g_v_ge31);
  need("post-model: e4c increment", g_post_e4c_inc);
  need("post-model: e4c flag set", g_post_e4c_set);
  need("post-model: nlencoder arm", g_post_nl);
}

/* ============================================================================
 * scenarios
 * ========================================================================== */
static void scen_random(int fnmode, long calls, const char *name) {
  logger_t lg = {name, FNV0, 0, 431};
  unsigned char *o = G_OBJ;
  mvcfg_t c;
  long i;
  for (i = 0; i < calls; i++) {
    unsigned int h;
    build_valid(o, &c, fnmode, (int)lcg_mod(3));
    h = call_once(o, fnmode, 1);
    log_call(&lg, h, c.m, c.n);
  }
  log_done(&lg);
}

static void scen0(void) {
  scen_random(0, 40000, "mv(recorder)");
  cov_full(0, 0);
  cov_inputs();
  need("recorder-steered v", g_v_steered);
}
static void scen1(void) {
  scen_random(1, 20000, "mv(GPC)");
  scen_random(2, 20000, "mv(GPA)");
  cov_full(0, 0);
  cov_inputs();
}

/* chain: the object persists; k walks 0..8; the shell state is redrawn at generation time only when the
 * oracle rejects the natural continuation */
/* a chain lets the bulk-delay ring indices run for thousands of samples: keep the ring geometry the real
 * object has (base = obj+0x35b8, len <= 0x2580, indices < len) so the walk never leaves its own storage */
static void chain_fix_bulk(unsigned char *o) {
  unsigned int len;
  switch (lcg_mod(3)) {
    case 0: len = (unsigned int)rr(1, 40); break;
    case 1: len = 0x2580u; break;
    default: len = (unsigned int)rr(1, 0x2580); break;
  }
  PTRS(o, 0x35b0) = OB(o, 0x35b8);
  U32(o, 0x35b4) = len;
  U32(o, 0x35a8) = lcg_next() % len;
  U32(o, 0x35ac) = lcg_next() % len;
}

/* wmode 0: force the offset width W = 0, 1: force W in 1..wmax(m) (both keep (m << W) <= 416, i.e. quarter[] in range) */
static void chain_fix_w(unsigned char *o, const mvcfg_t *c, int wmode) {
  if (wmode == 0) S16(o, 0x25f4) = 0;
  else if (wmode == 1) S16(o, 0x25f4) = (int16_t)rr(1, wmax_for(c->m));
}

static void scen_chain(int fnmode, long calls, const char *name, int wmode) {
  logger_t lg = {name, FNV0, 0, 997};
  unsigned char *o = G_OBJ;
  mvcfg_t c;
  long i, gens = 0, redo_bits = 0, redo_full = 0;
  build_valid(o, &c, fnmode, 0);
  chain_fix_bulk(o);
  chain_fix_w(o, &c, wmode);
  S16(o, 0x2aa2) = 0; /* start with a plain call, generation on the 9th */
  for (i = 0; i < calls; i++) {
    unsigned int h;
    if (U16(o, 0x2aa2) == 8) {
      int tries = 0;
      /* the natural continuation: bits keep coming from the scrambler / recorder; re-poke the bit buffer if invalid */
      while (!oracle_ok(o)) {
        g_redraws++;
        if (++tries <= 30) {
          U32(o, 0x2a60) = lcg_next();
          S16(o, 0x2a64) = (int16_t)rr(0, 15);
          if (fnmode) gf_set_scrambler_state(o);
          redo_bits++;
        } else {
          /* rebuild from scratch, keeping the tx pointers valid: fresh object */
          build_valid(o, &c, fnmode, 0);
          chain_fix_bulk(o);
          chain_fix_w(o, &c, wmode);
          S16(o, 0x2aa2) = 8;
          redo_full++;
          break;
        }
      }
      merge_cov();
      gens++;
    } else {
      (void)oracle_ok(o); /* plain path: counts the tail coverage (cannot violate) */
      merge_cov();
    }
    if (i % 401 == 400) { /* the caller pokes other tx-side fields now and then */
      unsigned int w = U16(o, 0x25c2);
      mv_static_fields(o); /* limits, scale, Rs, coefficient / trellis-table pointers re-drawn mid-stream */
      mv_counters(o);
      switch ((i / 401) % 5) { /* trellis configuration / rounding shift / scale change mid-stream */
        case 0: U16(o, 0x25f6) = 0x40; break;
        case 1: U16(o, 0x25f6) = 0x20; break;
        case 2: U16(o, 0x25f6) = 0; S16(o, 0x2626) = (int16_t)rr(1, 2); break;
        case 3: U16(o, 0x25f6) = 0xffe0; S16(o, 0x2622) = rshort(); break;
        default: U16(o, 0x25f6) = 0x40; S16(o, 0x2626) = (int16_t)rr(-2, 3); break;
      }
      U16(o, 0x25c2) = (uint16_t)(lcg_mod(2) ? (w & ~0x10u) : (w | 0x10u)); /* bit 4 both ways */
      U8(o, 0x25c3) = (unsigned char)(U8(o, 0x25c3) ^ (lcg_mod(3) == 0 ? 0x40 : 0));
      if (lcg_mod(2)) { I32(o, 0xaa74) = (int32_t)(U16(o, 0x25e0) - lcg_mod(3)); U16(o, 0xe4c) = (lcg_mod(3) == 0) ? 0 : (uint16_t)(lcg_next() | 1u); }
    }
    h = call_once(o, fnmode, 0);
    h = fnv32(h, hash_obj(o)); /* the WHOLE object after every call */
    log_call(&lg, h, (long)U16(o, 0x2aa2), 0);
  }
  log_done(&lg);
  need("chain generations", gens);
  printf("  chain %s: generations=%ld bit re-pokes=%ld full rebuilds=%ld\n", name, gens, redo_bits, redo_full);
}
static void scen2(void) {
  scen_chain(0, 30000, "mv-chain(recorder)", 0);
  scen_chain(1, 20000, "mv-chain(GPC)", 1);
  scen_chain(2, 20000, "mv-chain(GPA)", 1);
  cov_full(1, 0);
}

/* boundary sweep of the shell index (recorder-steered v).  m = 2..4: every T[k], T[k]-1, T[k+1]-1 within
 * 16 bits; m = 2 is also swept EXHAUSTIVELY over v in 0..255 (x 6 random trellis states each). */
static void scen3(void) {
  logger_t lg = {"mv-boundary", FNV0, 0, 199};
  unsigned char *o = G_OBJ;
  mvcfg_t c;
  long i, hit_eq = 0, exh = 0;
  int v;
  for (i = 0; i < 6000; i++) {
    unsigned int h;
    int tries = 0;
    for (;;) {
      mv_build(o, &c, 0, 2);
      /* small m only so the boundaries fit in 16 bits */
      if (c.m <= 8 && c.steered && oracle_ok(o)) break;
      if (++tries > 100000) die("scen3: no valid boundary state");
    }
    merge_cov();
    model_check(o, &c);
    g_m_cnt[c.m]++;
    g_w_cnt[c.W]++;
    g_v_steered++;
    h = call_once(o, 0, 1);
    log_call(&lg, h, c.m, c.n);
  }
  /* exhaustive m = 2 */
  for (v = 0; v < 256; v++) {
    int rep;
    for (rep = 0; rep < 6; rep++) {
      unsigned int h;
      unsigned int total = 256;
      int tries = 0;
      for (;;) {
        mv_build(o, &c, 0, 0);
        if (c.m != 2) {
          /* force m == 2 by rebuilding the shell tables and the block on the same object */
          shell_build(&c.sh, 2);
          shell_install(o, &c.sh, 0);
          S16(o, 0x25f2) = 2;
          c.m = 2;
          c.W = rr(0, wmax_for(2));
          c.fill = 0;
          c.model_ok = 1;
        }
        {
          int count = rr(0, 15);
          uint32_t buf = ((uint32_t)v << count) | (lcg_next() & (((uint32_t)1 << count) - 1u)) | (lcg_next() << (count + 8));
          S16(o, 0x25ee) = 8;
          S16(o, 0x25f0) = 8;
          S16(o, 0x25f4) = (int16_t)c.W;
          U32(o, 0x2a60) = buf;
          S16(o, 0x2a64) = (int16_t)count;
          c.n = 8;
        }
        if (oracle_ok(o)) break;
        if (++tries > 100000) die("scen3: exhaustive m=2 state never valid");
      }
      merge_cov();
      if (g_orc_rings && g_orc_v != (uint32_t)v) die("scen3: steered v differs from the requested v (getFrame extraction assumption)");
      model_check(o, &c);
      g_m_cnt[2]++;
      g_w_cnt[c.W]++;
      (void)total;
      h = call_once(o, 0, 1);
      log_call(&lg, h, v, c.W);
      exh++;
    }
  }
  hit_eq = g_cov[CV_BS_EQ];
  log_done(&lg);
  cov_full(0, 1);
  printf("  exhaustive m=2 calls=%ld  binary-search equality hits=%ld  ring model checked=%ld\n", exh, hit_eq, g_model_checked);
  need("recorder-steered v", g_v_steered);
  need("ring model checked", g_model_checked);
}

/* k / tail / e4c block scenario: k in every class, nlencoder flag both ways, bit 4 / e4c / aa74 classes */
/* T07: scen4(1) = scenario 8: the same walk with k restricted to the block's 0..7 (and 8 = generate): no read outside the 8-point block
 * (obj+0x2a80 + 4k), so the result does not depend on the members around it (i386 vs x86-64 layout) */
static void scen4(int inrange) {
  logger_t lg = {inrange ? "mv-tail-inrange" : "mv-tail", FNV0, 0, 431};
  unsigned char *o = G_OBJ;
  mvcfg_t c;
  long i, kplain = 0, kgen = 0, kneg = 0, khi = 0, kneigh = 0;
  for (i = 0; i < 30000; i++) {
    unsigned int h;
    int fn = (int)lcg_mod(3), kmode = (int)lcg_mod(8);
    build_valid(o, &c, fn, 1);
    switch (kmode) {
      case 0: case 1: case 2: S16(o, 0x2aa2) = 8; break;
      case 3: case 4: S16(o, 0x2aa2) = (int16_t)rr(0, 7); break;
      case 5: S16(o, 0x2aa2) = (int16_t)(inrange ? rr(0, 7) : rr(9, 60)); break;
      case 6: S16(o, 0x2aa2) = (int16_t)(inrange ? rr(0, 7) : -rr(1, 100)); break;
      default: S16(o, 0x2aa2) = (int16_t)(inrange ? 7 : lcg_mod(2) ? 7 : 9); break; /* the neighbours of 8 */
    }
    {
      unsigned int kw16 = U16(o, 0x2aa2);
      if ((uint16_t)kw16 == 8) kgen++;
      else {
        kplain++;
        if ((int16_t)kw16 < 0) kneg++;
        if ((int16_t)kw16 > 8) khi++;
        if ((int16_t)kw16 == 7 || (int16_t)kw16 == 9) kneigh++;
      }
      /* k == -22 makes the tail read obj+0x2a28, the scrambler FUNCTION-POINTER slot, as the "point" (an address:
       * different in the two binaries, so not comparable): nudge it (documented exclusion) */
      if ((int16_t)kw16 == -22) U16(o, 0x2aa2) = (uint16_t)-23;
      /* an out-of-range k reads the block index outside 0..7: keep the read inside the object */
      if (!((uint16_t)kw16 == 8) && ((int16_t)kw16 > 200 || (int16_t)kw16 < -200)) U16(o, 0x2aa2) = 3;
    }
    if ((uint16_t)U16(o, 0x2aa2) != 8) {
      if (!oracle_ok(o)) die("scen4: the plain path cannot violate");
      merge_cov();
    }
    /* the plain path needs no valid shell state, but the object must still pass through txmit safely: setup_tx did that */
    h = call_once(o, fn, 0);
    log_call(&lg, h, (long)U16(G_SNAP, 0x2aa2), fn);
  }
  log_done(&lg);
  cov_full(1, 0);
  cov_inputs();
  need("k == 8 calls", kgen);
  need("k in 0..7 or above 8 (plain)", kplain);
  if (!inrange) {
    need("negative k (index below the block)", kneg);
    need("k > 8", khi);
    need("k == 7 or 9 (neighbours of the generation value)", kneigh);
  } else {
    need("k == 7 (neighbour of the generation value)", kneigh);
    if (kneg || khi) die("scen4(in-range): k left 0..8");
  }
}

/* blocks built by the REAL preinitV34 + initV34 (blob's own in the blob binary, the port's in the port
 * binary): realistic tables, N / rows / step, cfg, Q pointer (Convolve16/32/64), p6 coefficient pointer
 * (harness buffer), scale.  Only the dynamic state (trellis words, phase, history, counters, e4c block,
 * running position a8) is randomised. */
static void scen5(void) {
  logger_t lg = {"mv-initV34", FNV0, 0, 199};
  unsigned char *o = G_OBJ;
  static const unsigned short bauds[] = {0x960, 0xaf0, 0xab7, 0xbb8, 0xc80, 0xd65};
  long i, accepted = 0, tries = 0, m_seen[20], q64 = 0, q32 = 0, q16 = 0, w_pos = 0;
  memset(m_seen, 0, sizeof m_seen);
  for (i = 0; i < 12000;) {
    unsigned int h;
    int fnmode = (int)lcg_mod(3);
    unsigned short p2 = bauds[lcg_mod(6)];
    unsigned short p3 = (unsigned short)rr(2400, 33600);
    short p4 = (short)lcg_mod(2), p5 = (short)lcg_mod(3);
    unsigned short p7 = (unsigned short)rr(1, 400);
    unsigned char *blk = o + 0x25e0;
    int m;

    tries++;
    fill_random(o, OBJSZ);
    scrub_slots(o);
    {
      txm_t tm;
      setup_tx(o, &tm);
    }
    U32(o, 0x25d0) = lcg_next();
    preinitV34(blk);
    fill_coefs(g_P, 12);
    initV34(blk, p2, p3, p4, p5, (uint32_t)(uintptr_t)g_P, p7);
    m = S16(o, 0x25f2);
    if (m < 1 || m > 18) continue; /* not a shell configuration */
    U16(o, 0x25e8) = (uint16_t)rr(0, U16(o, 0x25e0) + 2); /* running position */
    mv_dynamic_fields(o);
    mv_counters(o);
    gf_set_fn(o, fnmode);
    if (fnmode) gf_set_scrambler_state(o);
    S16(o, 0x2aa2) = 8;
    U32(o, 0x2a60) = lcg_next();
    S16(o, 0x2a64) = (int16_t)rr(0, 15);
    if (!oracle_ok(o)) { g_redraws++; continue; }
    merge_cov();
    m_seen[m]++;
    if (S16(o, 0x25f4) > 0) w_pos++;
    {
      unsigned int cfgw = U16(o, 0x25f6);
      if (cfgw == 0x40) q64++; else if (cfgw == 0x20) q32++; else q16++;
    }
    h = call_once(o, fnmode, 0);
    log_call(&lg, h, m, S16(o, 0x25f4));
    accepted++;
    i++;
  }
  log_done(&lg);
  printf("  initV34 states: tried=%ld accepted=%ld oracle rejects=%ld\n", tries, accepted, g_oracle_rejects);
  {
    int mm, distinct = 0;
    for (mm = 1; mm <= 18; mm++) if (m_seen[mm]) distinct++;
    printf("  distinct m values from real initV34 blocks: %d (%s)\n", distinct, "list follows");
    for (mm = 1; mm <= 18; mm++) if (m_seen[mm]) printf("    m=%d: %ld\n", mm, m_seen[mm]);
    need("distinct m values from initV34 (>= 4)", distinct >= 4 ? distinct : 0);
  }
  need("initV34 blocks accepted", accepted);
  need("initV34 cfg 0x40 (p5 >= 2)", q64);
  need("initV34 cfg 0x20 (p5 == 1)", q32);
  need("initV34 default cfg (p5 == 0)", q16);
  need("initV34 blocks with W > 0", w_pos);
  need("oracle: generation path", g_cov[CV_GEN]);
  need("oracle: binary search T[mid] > v", g_cov[CV_BS_ARM1]);
  need("oracle: binary search T[mid] <= v", g_cov[CV_BS_ARM2]);
  need("oracle: peel phase 1, >= 1 iteration", g_cov[CV_PEEL + 1] + g_cov[CV_PEEL + 2]);
  need("oracle: pair reflected", g_cov[CV_PAIR_ALT] + g_cov[CV_PAIR_ALT + 1] + g_cov[CV_PAIR_ALT + 2] + g_cov[CV_PAIR_ALT + 3]);
  need("oracle: cfg 0x40 path", g_cov[CV_CFG64]);
  need("oracle: LFSR path", g_cov[CV_LFSR_ODD] + g_cov[CV_LFSR_EVEN]);
}

/* zero-divisor probe: T[] all 0x7fffffff makes the binary search end at lo = 0; r = v - T[0] (v == 0) is
 * negative as a signed int, so the peel loop is skipped, j1 = 0 and the divisor is A[0] = 0 -> idiv by zero
 * -> SIGFPE.  The blob traps; the port must too (a state no producer ever builds). */
static void scen6(void) {
  unsigned char *o = G_OBJ;
  mvcfg_t c;
  pid_t pid;
  int st = 0;
  build_valid(o, &c, 0, 1);
  {
    int i;
    uint32_t *T = (uint32_t *)(void *)OB(o, 0x2828);
    for (i = 0; i < 128; i++) T[i] = 0xffffffffu;
    U16(o, 0x2728) = 0; /* A[0] = 0 */
    for (i = 0; i < 128; i++) T[i] = 0x7fffffffu; /* the search ends at lo = 0; r = v - T[0] is NEGATIVE as an int */
    S16(o, 0x25ee) = 1; /* one plain-arm bit: v = 0 */
    S16(o, 0x25f0) = 1;
    U32(o, 0x2a60) = 0;
    S16(o, 0x2a64) = 0;
  }
  fflush(stdout);
  pid = fork();
  if (pid == 0) {
    g_rec_h = FNV0;
    g_rec_n = 0;
    g_rec_callno = 0;
    g_rec_obj = o;
    g_rec_on = 1;
    modulatevector(o);
    _exit(0);
  }
  if (pid < 0) die("fork failed");
  waitpid(pid, &st, 0);
  if (WIFSIGNALED(st)) printf("zero-divisor probe: child killed by signal %d (SIGFPE=%d)\n", WTERMSIG(st), SIGFPE);
  else printf("zero-divisor probe: child exited normally with status %d\n", WEXITSTATUS(st));
  if (!(WIFSIGNALED(st) && WTERMSIG(st) == SIGFPE)) {
    fprintf(stderr, "zero-divisor probe: expected SIGFPE\n");
    g_fail = 1;
  }
}


/* legal V.34 rate / baud grid: for every baud and every rate (multiple of 2400) the REAL preinitV34 + initV34
 * (blob's / port's) build the block; the largest quarter[] index modulatevector can form is
 * ((m-1) << c) + (2^c - 1) = (m << c) - 1 (ring m-1 at shift c with a c-bit offset).  Inside V.34's rate table
 * (max rate per baud: 2400 -> 21600, 2743 / 2800 -> 26400, 3000 -> 28800, 3200 -> 31200, 3429 -> 33600) this is
 * always <= 415 (asserted); above the table's rates the index can overrun quarter[] (the blob then reads the
 * neighbouring .rodata tables; the port returns 0) - listed, and unreachable by any legal caller. */
static void scen7(void) {
  static const unsigned short bauds[] = {0x960, 0xab7, 0xaf0, 0xbb8, 0xc80, 0xd65};
  static const int maxrate[] = {21600, 26400, 26400, 28800, 31200, 33600};
  unsigned char *o = G_OBJ;
  int bi, p4, rate;
  long legal = 0, over_legal = 0, over_illegal = 0, total = 0;
  unsigned int h = FNV0;
  for (bi = 0; bi < 6; bi++)
    for (p4 = 0; p4 < 2; p4++)
      for (rate = 2400; rate <= 33600; rate += 2400) {
        unsigned char *blk = o + 0x25e0;
        int m, c, maxidx, islegal = rate <= maxrate[bi];
        memset(o, 0, OBJSZ);
        preinitV34(blk);
        initV34(blk, bauds[bi], (unsigned short)rate, (short)p4, 0, 0, 100);
        m = S16(o, 0x25f2);
        c = S16(o, 0x25f4);
        maxidx = (m << c) - 1;
        total++;
        h = fnv32(fnv32(fnv32(h, (uint32_t)m), (uint32_t)c), (uint32_t)maxidx);
        if (islegal) {
          legal++;
          if (maxidx > 415) {
            over_legal++;
            fprintf(stderr, "LEGAL combo baud %d rate %d p4 %d: m=%d c=%d max quarter index %d > 415\n", bauds[bi], rate, p4, m, c, maxidx);
          }
        } else if (maxidx > 415) {
          over_illegal++;
        }
        printf("grid baud=%d rate=%d p4=%d m=%d c=%d maxidx=%d %s%s\n", bauds[bi], rate, p4, m, c, maxidx, islegal ? "legal" : "above-table",
               maxidx > 415 ? " OVERRUN" : "");
      }
  printf("grid total=%ld legal=%ld legal-with-overrun=%ld above-table-with-overrun=%ld hash=%08x\n", total, legal, over_legal, over_illegal, h);
  if (over_legal) die("a legal V.34 rate/baud combination can overrun quarter[] (index > 415)");
  need("grid combinations", total);
  need("above-table combinations that would overrun quarter[]", over_illegal);
}

int main(int argc, char **argv) {
  V34I_REG(g_objbuf, OBJSZ);
  V34I_REG(g_orcbuf, OBJBIG);
  int s = argc > 1 ? atoi(argv[1]) : -1;
  const char *v = getenv("V34T6_VERBOSE");
  const char *sd = getenv("V34T6_SEED"); /* alternative input streams (mutation-survivor cross-checks) */
  if (v) g_verbose = atoi(v);
  if (sd) g_lcg ^= (unsigned int)strtoul(sd, NULL, 0) * 0x9e3779b1u;
  tables_check();
  {
    /* start-up: the harness's own shell math reproduces the blob's initV34 tables for m <= 14 (xyz slices) */
    int m;
    for (m = 1; m <= 18; m++) {
      shell_t sh;
      unsigned int i, len = xyz[m + 1] - xyz[m];
      shell_build(&sh, m);
      if (len > (unsigned int)sh.nk) die("xyz slice longer than the model's ring-sum count");
      if (m <= 14 && len != (unsigned int)sh.nk) die("xyz slice length != 8m-7 for m <= 14");
      for (i = 0; i < len; i++)
        if ((uint64_t)xyz[xyz[m] + i] != (sh.T[i] & 0xffffffffull)) die("xyz slice != harness g8 prefix sums");
    }
  }
  memset(g_orcbuf, 0, sizeof g_orcbuf);
  switch (s) {
    case 0: scen0(); break;
    case 1: scen1(); break;
    case 2: scen2(); break;
    case 3: scen3(); break;
    case 4: scen4(0); break;
    case 5: scen5(); break;
    case 6: scen6(); break;
    case 7: scen7(); break;
    case 8: scen4(1); break;
    default: fprintf(stderr, "usage: %s <scenario 0-8>\n", argv[0]); return 1;
  }
  fflush(stdout);
  return g_fail ? 2 : 0;
}

