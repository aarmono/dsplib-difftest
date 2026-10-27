/*
 * R30 harness library (header only; used by test/fax/fax_api.main.c): derived from test/r17_common.h (modem scripting, recorders) and test/fax/r29_common.h (the heap with a canary, the typed
 * object sizes of the fax stack).  Canonical output (every line is FNV-hashed, only selected lines are printed unless R30_VERBOSE=1, then "HASH <h> lines <n>"), a PRNG, required
 * coverage counters (a zero exits 2; 3 = harness violation), a deterministic heap (red zones, poisoned frees, id-patterned fresh blocks; the sizes of the typed objects mapped to the
 * i386 ones) with the callee recorders sysdep_malloc / free / memset / memcpy, dsplibs_debug_printf (the formatted message + a hash of the object under test at that moment: g_snap),
 * modem_get_sreg (scripted table), modem_recv_from_tty / modem_send_to_tty (scripted byte queues, every call logged).  The code under test (blob or port) calls these through its normal
 * external symbols.  Everything printed is architecture independent: the 32-bit BLOB run is the golden of x86-64.
 */
#ifndef R30_COMMON_H
#define R30_COMMON_H
#include <limits.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "fax/fax_class1.h"

typedef unsigned long long u64;

static u64 g_hash = 1469598103934665603ULL, g_lines;
static int g_verbose;
static void hash_bytes(const void *p, size_t n)
{
  const unsigned char *b = (const unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) {
    g_hash ^= b[i];
    g_hash *= 1099511628211ULL;
  }
}
static u64 fnv_bytes(const void *p, size_t n)
{
  u64 h = 1469598103934665603ULL;
  const unsigned char *b = (const unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) {
    h ^= b[i];
    h *= 1099511628211ULL;
  }
  return h;
}
static u64 fnv_mix(u64 h, unsigned v)
{
  int k;
  for (k = 0; k < 4; k++) {
    h ^= (v >> (8 * k)) & 0xffu;
    h *= 1099511628211ULL;
  }
  return h;
}
/* a float array as its bit patterns */
static u64 fnv_floats(u64 h, const float *p, unsigned n)
{
  unsigned i;
  for (i = 0; i < n; i++) {
    uint32_t u;
    memcpy(&u, &p[i], 4);
    h = fnv_mix(h, u);
  }
  return h;
}
static u64 fnv_shorts(u64 h, const short *p, unsigned n)
{
  unsigned i;
  for (i = 0; i < n; i++) h = fnv_mix(h, (unsigned)(unsigned short)p[i]);
  return h;
}
static void emit(int show, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void emit(int show, const char *fmt, ...)
{
  char buf[2048];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  hash_bytes(buf, strlen(buf));
  hash_bytes("\n", 1);
  g_lines++;
  if (show || g_verbose) puts(buf);
}
static void die3(const char *fmt, ...) __attribute__((noreturn, format(printf, 1, 2)));
static void die3(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  fprintf(stderr, "HARNESS/MODEL VIOLATION: ");
  vfprintf(stderr, fmt, ap);
  fprintf(stderr, "\n");
  va_end(ap);
  exit(3);
}
static unsigned g_lcg = 0x2545f491u;
static unsigned rnd(void)
{
  unsigned x;
  g_lcg = g_lcg * 1664525u + 1013904223u;
  x = g_lcg;
  x ^= x >> 15;
  x *= 0x2c1b3c6du;
  x ^= x >> 12;
  x *= 0x297a2d39u;
  x ^= x >> 15;
  return x;
}
static unsigned rmod(unsigned m) { return m ? rnd() % m : 0; }
static int rpct(int pct) { return (int)rmod(100) < pct; }
static int rrange(int lo, int hi) { return lo + (int)rmod((unsigned)(hi - lo + 1)); }

#define COV_MAX 300
static unsigned long g_cov[COV_MAX];
static const char *g_covname[COV_MAX];
static void cov_def(int id, const char *name) { g_covname[id] = name; }
static void cov_hit(int id) { g_cov[id]++; }
static void cov_finish(void)
{
  int i, bad = 0;
  for (i = 0; i < COV_MAX; i++) {
    if (!g_covname[i]) continue;
    emit(1, "COV %-52s %lu", g_covname[i], g_cov[i]);
    if (g_cov[i] == 0) {
      fprintf(stderr, "COVERAGE FAIL: %s was never reached\n", g_covname[i]);
      bad = 1;
    }
  }
  emit(1, "HASH %016llx lines %llu", g_hash, g_lines);
  fflush(stdout);
  if (bad) exit(2);
}


/* ------------------------------------------------------------------------------------------------ deterministic heap + callee recorders */
#define ARENA_SZ (16u << 20)
#define REDZ 48u
#define MAXBLK 4096
typedef struct {
  unsigned char *p;
  unsigned size, id;
  int freed;
} Blk;
static unsigned char g_arena[ARENA_SZ] __attribute__((aligned(64)));
static size_t g_off = 64;
static Blk g_blk[MAXBLK];
static unsigned g_nblk, g_next_id = 1, g_live;
static unsigned long g_nmalloc, g_nfree, g_nmemset, g_nmemcpy;
static int g_fail_malloc = -1; /* the n-th (0 based, counted from the last trial_end) sysdep_malloc returns NULL; -1 = never */
static unsigned g_malloc_ctr;
static unsigned g_msz[256], g_nmsz; /* the raw sizes of the sysdep_malloc calls since the last trial_end */
static u64 (*g_snap)(void);                 /* hash of the object under test (called at every debug print) */
static void (*g_dbg_hook)(const char *msg); /* coverage observer / state mutator of the debug text */

/* the logged size of a block: the sizes of the typed objects are the i386 ones on both hosts.  A plain buffer the harness allocates never has the size of a typed object on either host. */
static unsigned log_size(unsigned size)
{
#ifdef HARNESS_ENG
  (void)size; /* the engine objects behind the ops tables have host dependent sizes: only the block ids are logged */
  return 0;
#else
  if (size == sizeof(FaxT30)) return 0x12f4;
  if (size == sizeof(FaxGlue)) return 0x28bc;
  if (size == sizeof(FaxVmiCfg)) return 0x18; /* 0x18 at -m32, 32 on x86-64 */
  if (size == sizeof(FaxVmi)) return 0x2c;
  if (size == sizeof(RcFixed)) return 8;
  return size;
#endif
}
static Blk *find_blk(const void *q)
{
  unsigned i;
  const unsigned char *p = (const unsigned char *)q;
  for (i = 0; i < g_nblk; i++)
    if (p >= g_blk[i].p && p < g_blk[i].p + (g_blk[i].size ? g_blk[i].size : 1u)) return &g_blk[i]; /* a size 0 block is the base address only */
  return NULL;
}
static unsigned blk_id_of(const void *p)
{
  Blk *b = find_blk(p);
  return b ? b->id : 0;
}
/* an offset inside a block, as the i386 build sees it (x86-64: the members of the glue block move) */
static unsigned log_off(const Blk *b, unsigned off)
{
  if (b->size == sizeof(FaxGlue)) {
#define M(m, i386) \
  if (off >= offsetof(FaxGlue, m) && off < offsetof(FaxGlue, m) + sizeof(((FaxGlue *)0)->m)) return (unsigned)(i386) + (off - (unsigned)offsetof(FaxGlue, m))
    M(tty_out, 0x10); M(tty_in, 0x1010); M(rxbuf, 0x2018); M(txbuf, 0x2158); M(out, 0x229c); M(in, 0x25ac);
#undef M
  }
  if (b->size == sizeof(FaxT30) && off == (unsigned)offsetof(FaxT30, hpState)) return 0x12e0;
  return off;
}
static int g_ainit;
static void arena_init(void)
{
  if (!g_ainit) {
    memset(g_arena, 0x5A, sizeof g_arena);
    g_ainit = 1;
  }
}
void *sysdep_malloc(unsigned int size)
{
  size_t asz = ((size_t)size + 15u) & ~(size_t)15u, i;
  unsigned id = g_next_id++;
  unsigned char *p;
  arena_init();
  if (g_nmsz < 256) g_msz[g_nmsz] = size;
  g_nmsz++;
  if (g_fail_malloc >= 0 && g_malloc_ctr++ == (unsigned)g_fail_malloc) {
    emit(0, "M %u %u NULL", id, log_size(size));
    return NULL;
  }
  if (g_off + REDZ + asz + REDZ > ARENA_SZ || g_nblk >= MAXBLK) die3("arena exhausted");
  memset(g_arena + g_off, 0xCC, REDZ);
  p = g_arena + g_off + REDZ;
  for (i = 0; i < asz; i++) p[i] = (unsigned char)(0x35u + id * 13u + (unsigned)i * 7u + ((unsigned)i >> 6)); /* fresh blocks hold deterministic garbage */
  memset(p + size, 0xCC, asz - size); /* the alignment padding after the block is a red zone too: a one-element overrun is seen */
  memset(p + asz, 0xCC, REDZ);
  g_off += REDZ + asz + REDZ;
  g_blk[g_nblk].p = p;
  g_blk[g_nblk].size = size;
  g_blk[g_nblk].id = id;
  g_blk[g_nblk].freed = 0;
  g_nblk++;
  g_live++;
  g_nmalloc++;
  emit(0, "M %u %u", id, log_size(size));
  return p;
}
void sysdep_free(void *q)
{
  Blk *b;
  if (!q) {
    emit(0, "F NULL");
    g_nfree++;
    return;
  }
  b = find_blk(q);
  if (!b || b->p != (unsigned char *)q) die3("sysdep_free of a pointer that is not a block base");
  if (b->freed) die3("double free of block %u", b->id);
  {
    size_t k, asz = ((size_t)b->size + 15u) & ~(size_t)15u;
    for (k = b->size; k < asz; k++)
      if (b->p[k] != 0xCC) die3("block %u: a byte beyond its size was written (before the free)", b->id);
  }
  b->freed = 1;
  g_live--;
  g_nfree++;
  memset(b->p, 0xDD, ((size_t)b->size + 15u) & ~(size_t)15u);
  emit(0, "F %u", b->id);
}
void *sysdep_memset(void *dst, int c, size_t len)
{
  Blk *b = find_blk(dst);
  if (b && b->freed) die3("memset of a freed block");
  g_nmemset++;
#ifndef HARNESS_ENG
  if (b) {
    unsigned off = log_off(b, (unsigned)((unsigned char *)dst - b->p));
    emit(0, "S %u %u %d %u", b->id, off, c, (unsigned)(b->size == len ? log_size((unsigned)len) : (unsigned)len));
  } else emit(0, "S - %d %u", c, (unsigned)len);
#endif
  return memset(dst, c, len);
}
void *sysdep_memcpy(void *dst, const void *src, size_t len)
{
  Blk *b = find_blk(dst);
  if (b && b->freed) die3("memcpy into a freed block");
  g_nmemcpy++;
  memcpy(dst, src, len);
#ifndef HARNESS_ENG
  emit(0, "C %u %u %u %016llx", b ? b->id : 0u, b ? log_off(b, (unsigned)((unsigned char *)dst - b->p)) : 0u, (unsigned)len, fnv_bytes(dst, len));
#endif
  return dst;
}
/* non-fatal: 1 when a byte outside some block was written (the probes of the blob's own overruns; the child process reports it) */
static int redzones_dirty(void)
{
  unsigned i;
  size_t k;
  for (i = 0; i < g_nblk; i++) {
    const Blk *b = &g_blk[i];
    size_t asz = ((size_t)b->size + 15u) & ~(size_t)15u;
    for (k = b->freed ? asz : b->size; k < asz + REDZ; k++)
      if (b->p[k] != 0xCC) return 1;
  }
  return 0;
}
static void trial_end(const char *what)
{
  unsigned i;
  size_t k;
  arena_init();
  for (i = 0; i < g_nblk; i++) {
    const Blk *b = &g_blk[i];
    size_t asz = ((size_t)b->size + 15u) & ~(size_t)15u;
    if (!b->freed)
      for (k = b->size; k < asz; k++)
        if (b->p[k] != 0xCC) die3("block %u: a byte beyond its size was written (%s)", b->id, what);
    for (k = 0; k < REDZ; k++)
      if (b->p[asz + k] != 0xCC || b->p[(size_t)0 - REDZ + k] != 0xCC) die3("block %u: a byte outside the block was written (%s)", b->id, what);
    if (b->freed) {
      for (k = 0; k < asz; k++)
        if (b->p[k] != 0xDD) die3("block %u written after free (%s)", b->id, what);
    }
  }
  if (g_live != 0) emit(1, "LEAK %s %u live blocks", what, g_live);
  { /* nothing beyond the top of the heap may have been written (a far overrun misses the red zones): the arena above g_off keeps its 0x5A canary */
    size_t lim = g_off + (256u << 10) < ARENA_SZ ? g_off + (256u << 10) : ARENA_SZ;
    for (k = g_off; k < lim; k++)
      if (g_arena[k] != 0x5A) die3("a byte beyond the top of the heap was written (%s, +%u)", what, (unsigned)(k - g_off));
    memset(g_arena, 0x5A, g_off);
  }
  g_nblk = 0;
  g_live = 0;
  g_off = 64;
  g_malloc_ctr = 0;
  g_nmsz = 0;
  g_fail_malloc = -1;
}


/* ------------------------------------------------------------------------------------------------ the scripted modem */
static int g_fake_modem, g_fake_modem2;
#define FAKEM ((struct modem *)&g_fake_modem)
#define FAKEM2 ((struct modem *)&g_fake_modem2)
#define MID(m) ((m) == FAKEM ? 1 : (m) == FAKEM2 ? 2 : 0)
#define SREG_MAX 256
static int g_sreg[SREG_MAX]; /* modem_get_sreg(m, n) results */
static unsigned long g_ngs;
int modem_get_sreg(struct modem *m, unsigned int num)
{
  int v;
  if (m != FAKEM && m != FAKEM2) die3("modem_get_sreg with a foreign modem pointer");
  g_ngs++;
  v = num < SREG_MAX ? g_sreg[num] : -1;
  emit(0, "SREG m%d %u -> %d", MID(m), num, v);
  return v;
}


/* tty queues (one per modem: index 1 = FAKEM, 2 = FAKEM2): modem_recv_from_tty copies from the script (in, in_n bytes, in_pos consumed), modem_send_to_tty appends to out */
typedef struct {
  unsigned char in[1 << 16];
  unsigned in_n, in_pos;
  unsigned char out[1 << 16];
  unsigned out_n;
  unsigned long out_total; /* bytes ever sent (out keeps the first 64 KB) */
} TtyQ;
static TtyQ g_q[3];
#define g_tty_in (g_q[1].in)
#define g_tty_in_n (g_q[1].in_n)
#define g_tty_in_pos (g_q[1].in_pos)
#define g_tty_out (g_q[1].out)
#define g_tty_out_n (g_q[1].out_n)
static int g_recv_cap = -1; /* >= 0: modem_recv_from_tty returns at most this many bytes (a script of its own limit) */
static unsigned long g_nrecv, g_nsend;
static int g_last_recv_n;
static int g_recv_neg; /* nonzero: modem_recv_from_tty fails with this (negative) value */
static void (*g_recv_hook)(void); /* called after every modem_recv_from_tty (a test that rewrites the caller's state from inside the call) */
int modem_recv_from_tty(struct modem *m, char *buf, int n)
{
  unsigned k;
  int r;
  TtyQ *q;
  if (m != FAKEM && m != FAKEM2) die3("modem_recv_from_tty with a foreign modem pointer");
  q = &g_q[MID(m)];
  g_nrecv++;
  g_last_recv_n = n;
  k = q->in_n - q->in_pos;
  if (n < 0) k = 0;
  if ((int)k > n && n >= 0) k = (unsigned)n;
  if (g_recv_cap >= 0 && (int)k > g_recv_cap) k = (unsigned)g_recv_cap;
  if (g_recv_neg) {
    emit(0, "RECV m%d %d -> %d (failure)", MID(m), n, g_recv_neg);
    return g_recv_neg;
  }
  if (k) memcpy(buf, q->in + q->in_pos, k);
  q->in_pos += k;
  r = (int)k;
  emit(0, "RECV m%d %d -> %d %016llx", MID(m), n, r, fnv_bytes(buf, k));
  if (g_recv_hook) g_recv_hook();
  return r;
}
int modem_send_to_tty(struct modem *m, const char *buf, int n)
{
  TtyQ *q;
  if (m != FAKEM && m != FAKEM2) die3("modem_send_to_tty with a foreign modem pointer");
  q = &g_q[MID(m)];
  g_nsend++;
  if (n > 0 && q->out_n + (unsigned)n <= sizeof q->out) {
    memcpy(q->out + q->out_n, buf, (size_t)n);
    q->out_n += (unsigned)n;
  }
  if (n > 0) q->out_total += (unsigned)n;
  emit(0, "SEND m%d %d %016llx", MID(m), n, n > 0 ? fnv_bytes(buf, (size_t)n) : 0ULL);
  return n;
}

int sysdep_sprintf(char *buf, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
int sysdep_sprintf(char *buf, const char *fmt, ...)
{
  int n;
  va_list ap;
  va_start(ap, fmt);
  n = vsprintf(buf, fmt, ap);
  va_end(ap);
  emit(0, "SPRINTF %s", buf);
  return n;
}

/* ------------------------------------------------------------------------------------------------ debug printf recorder */
unsigned int dsplibs_debug_level = 0;
static unsigned lvl_value(int k)
{
  static const unsigned v[8] = {0u, 1u, 2u, 3u, 4u, 0x80000000u, 0xffffffffu, 2u};
  return v[(unsigned)k & 7u];
}
static unsigned long g_dbg_count;
/* the debug level a trial starts with: the level-changing hooks of the tests lower dsplibs_debug_level for the rest of the call only (script_reset restores it) */
static unsigned g_lvl_base;
#define SETLVL(x) (dsplibs_debug_level = g_lvl_base = (x))
static void dbg_cov(const char *msg); /* the harness' coverage observer of the debug text */
int dsplibs_debug_printf(const char *fmt, ...)
{
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  if (strstr(fmt, "%p")) {
    void *p = va_arg(ap, void *);
    snprintf(buf, sizeof buf, "%.*s m%d", (int)(strstr(fmt, "%p") - fmt), fmt, MID((struct modem *)p));
  } else if (strstr(fmt, "%lX")) {
    unsigned long p = va_arg(ap, unsigned long);
    snprintf(buf, sizeof buf, "%.*s blk%u", (int)(strstr(fmt, "%lX") - fmt), fmt, blk_id_of((const void *)(uintptr_t)p));
  } else
    vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  {
    char *e = buf + strlen(buf);
    while (e > buf && (e[-1] == '\n' || e[-1] == '\r')) *--e = 0;
  }
  g_dbg_count++;
  emit(0, "DBG %s @%016llx", buf, g_snap ? g_snap() : 0ULL);
  dbg_cov(buf);
  if (g_dbg_hook) g_dbg_hook(buf);
  return 0;
}


/* a handle as a block id (0 = NULL, 255 = a pointer into no block) */
static unsigned hid(const void *p) { return p ? (blk_id_of(p) ? blk_id_of(p) : 255u) : 0u; }

/* ------------------------------------------------------------------------------------------------ guarded children (paths on which the blob crashes) */
/* run fn in a child: 0 = returned normally, 1 = killed by a signal, 3 = harness violation (die3) */
static int guarded(void (*fn)(void *), void *arg)
{
  int st = 0;
  pid_t c;
  fflush(stdout);
  c = fork();
  if (c < 0) die3("fork");
  if (c == 0) {
    if (!g_verbose && !freopen("/dev/null", "w", stderr)) _exit(9);
    fn(arg);
    _exit(0);
  }
  if (waitpid(c, &st, 0) != c) die3("waitpid");
  if (WIFSIGNALED(st)) return 1;
  return WEXITSTATUS(st);
}
/* the blob may die in the child (what it does on these paths is documented in NOTES / BUGS), the port must return normally */
static void guarded_report(const char *what, int r)
{
  if (g_verbose) fprintf(stderr, "guarded %s: child status %d\n", what, r);
#ifdef TEST_BLOB
  if (r != 0 && r != 1) die3("%s: blob child status %d", what, r);
#else
  if (r != 0) die3("%s: the port child failed (status %d)", what, r);
#endif
  emit(1, "GUARDED %s handled", what);
}

#endif
