/*
 * R01 differential harness: the RcFixed polyphase sample-rate converter (RcFixed_Create / _Delete / _Reset / _Resample / _Check_Combination), real dsplibs.o
 * vs src/core/rcfixed_resampler.c.  ONE source, no #ifdef around the code under test; the same object type (src/core/rcfixed_resampler.h: i386 layout at -m32, wider pointers on
 * x86-64) is used on both sides and the printed output is architecture independent, so the 32-bit BLOB run is the golden of the x86-64 build
 * (test/golden/rcfixed_blob32.out.gz, test/lib/tier_golden64.sh).
 *
 * Self-contained (no test/harness library: tier_golden64.sh builds only the main + the port files + stub_env.c): PRNG, canonical output (every line is hashed with
 * FNV-1a 64, only selected lines are printed, then "HASH <h> lines <n>"), required coverage counters (a zero exits 2), a deterministic heap with red zones /
 * poisoned frees / id-patterned fresh blocks (the blob reads uninitialised heap behind the 4 bytes it zeroes), a log of every sysdep_malloc / sysdep_free /
 * sysdep_memset (the callee recorders: id, size mapped to the i386 size, offset normalised) and fork probes for the inputs that crash the real blob.
 * Run as:  rcfixed_test <scenario>.  Exit 2 = a coverage counter is zero, 3 = harness / model violation.
 *
 * Scenarios: 0 Check_Combination (exhaustive grid, every legal pair x gcd multiples, random, extremes, fork probes (0,0) / (INT_MIN,-1)) + an independent model;
 *   1 lifecycle (Create every index, > 999, Delete NULL / odd types, interleaved objects, Reset, crash probes 20 .. 999);
 *   2 polyphase Resample (idx 2..19), random blocks / streaks / caps / resets / state pokes;
 *   3 4x converters (idx 0, 1): same;
 *   4 designed grid (every block length around the 200-sample history and the 4-sample groups, every extreme signal kind, per index);
 *   5 caps (*pnOut capacity 0 / 1 .. 9 / exact / exact-1 / huge) and non-0/1 object types.
 */
#include <limits.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include "core/rcfixed_resampler.h"
#include "core/rcfixed_resampler_tables.h"

typedef unsigned long long u64;

/* ------------------------------------------------------------------------------------------------ output / hash / PRNG / coverage */
static u64 g_hash = 1469598103934665603ULL, g_lines;
static void hash_bytes(const void *p, size_t n)
{
  const unsigned char *b = (const unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) {
    g_hash ^= b[i];
    g_hash *= 1099511628211ULL;
  }
}
static void emit(int show, const char *fmt, ...)
{
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  hash_bytes(buf, strlen(buf));
  hash_bytes("\n", 1);
  g_lines++;
  if (show) puts(buf);
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

#define COV_MAX 128
static unsigned long g_cov[COV_MAX];
static const char *g_covname[COV_MAX];
static char g_covbuf[COV_MAX][64];
static void cov_def(int id, const char *name) { g_covname[id] = name; }
static void cov_defn(int id, const char *name, int n)
{
  snprintf(g_covbuf[id], sizeof g_covbuf[id], "%s %d", name, n);
  g_covname[id] = g_covbuf[id];
}
static void cov_hit(int id) { g_cov[id]++; }
static void cov_finish(void)
{
  int i, bad = 0;
  for (i = 0; i < COV_MAX; i++) {
    if (!g_covname[i]) continue;
    emit(1, "COV %-40s %lu", g_covname[i], g_cov[i]);
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
#define ARENA_SZ (8u << 20)
#define REDZ 48u
#define MAXBLK 4096
typedef struct {
  unsigned char *p;
  unsigned size, id;
  int freed;
} Blk;
static unsigned char g_arena_static[ARENA_SZ] __attribute__((aligned(64)));
static unsigned char *g_arena = g_arena_static;
static size_t g_off = 64;
static Blk g_blk[MAXBLK];
static unsigned g_nblk, g_next_id = 1, g_live;
static unsigned long g_nmalloc, g_nfree, g_nmemset;
static int g_probe_fd = -1; /* fork probes: the child reports the arguments of a memset that hits no heap block (the crash site of Create(20..999)) */
typedef struct {
  u64 dst;
  unsigned len;
  int c;
} MemsetRec;

static void arena_init(void)
{
  /* as the shared harness: map the arena so that 0x80000000 lies inside it (signed pointer compares); static buffer if refused */
  void *want = (void *)(uintptr_t)(0x80000000u - 0x400000u);
  void *m = mmap(want, ARENA_SZ, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | 0x100000 /* MAP_FIXED_NOREPLACE */, -1, 0);
  if (m == want) g_arena = (unsigned char *)m;
  else if (m != MAP_FAILED) munmap(m, ARENA_SZ);
  g_off = 64;
}
static unsigned log_size(unsigned size)
{
  if (size == sizeof(RcFixed)) return 8;
  if (size == sizeof(RcPoly)) return 0x1a4;
  if (size == sizeof(RcDec)) return 0x28;
  return size;
}
void *sysdep_malloc(unsigned int size)
{
  size_t asz = ((size_t)size + 15u) & ~(size_t)15u, i;
  unsigned id = g_next_id++;
  unsigned char *p;
  if (g_off + REDZ + asz + REDZ > ARENA_SZ || g_nblk >= MAXBLK) die3("arena exhausted");
  memset(g_arena + g_off, 0xCC, REDZ);
  p = g_arena + g_off + REDZ;
  for (i = 0; i < asz; i++) p[i] = (unsigned char)(0x35u + id * 13u + (unsigned)i * 7u + ((unsigned)i >> 6)); /* fresh blocks hold deterministic garbage */
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
static Blk *find_blk(const void *q)
{
  unsigned i;
  const unsigned char *p = (const unsigned char *)q;
  for (i = 0; i < g_nblk; i++)
    if (p >= g_blk[i].p && p < g_blk[i].p + g_blk[i].size) return &g_blk[i];
  return NULL;
}
void sysdep_free(void *q)
{
  Blk *b = find_blk(q);
  if (!q) die3("sysdep_free(NULL)");
  if (!b || b->p != (unsigned char *)q) die3("sysdep_free of a pointer that is not a block base");
  if (b->freed) die3("double free of block %u", b->id);
  b->freed = 1;
  g_live--;
  g_nfree++;
  memset(b->p, 0xDD, ((size_t)b->size + 15u) & ~(size_t)15u);
  emit(0, "F %u", b->id);
}
void *sysdep_memset(void *dst, int c, unsigned int len)
{
  Blk *b = find_blk(dst);
  if (b) { /* normalised: block id, offset (the history array of the polyphase state is at +4 on i386 and +8 on x86-64), value, length */
    size_t off = (size_t)((unsigned char *)dst - b->p);
    if (b->size == sizeof(RcPoly)) off -= offsetof(RcPoly, hist);
    if (b->freed) die3("memset of a freed block");
    g_nmemset++;
    emit(0, "S %u %u %d %u", b->id, (unsigned)off, c, len);
  }
  if (!b && g_probe_fd >= 0) {
    MemsetRec r;
    r.dst = (u64)(uintptr_t)dst;
    r.len = len;
    r.c = c;
    if (write(g_probe_fd, &r, sizeof r) != (ssize_t)sizeof r) _exit(9);
  }
  return memset(dst, c, len); /* a block-less (NULL + 4) destination faults here, as in the blob */
}
static void trial_end(const char *what)
{
  unsigned i;
  size_t k;
  for (i = 0; i < g_nblk; i++) {
    const Blk *b = &g_blk[i];
    size_t asz = ((size_t)b->size + 15u) & ~(size_t)15u;
    for (k = 0; k < REDZ; k++)
      if (b->p[asz + k] != 0xCC || b->p[(size_t)0 - REDZ + k] != 0xCC) die3("block %u: a byte outside the block was written (%s)", b->id, what);
    if (b->freed) {
      for (k = 0; k < asz; k++) if (b->p[k] != 0xDD) die3("block %u written after free (%s)", b->id, what);
    } else {
      for (k = b->size; k < asz; k++)
        if (b->p[k] != (unsigned char)(0x35u + b->id * 13u + (unsigned)k * 7u + ((unsigned)k >> 6))) die3("block %u: the gap behind it was written (%s)", b->id, what);
    }
  }
  if (g_live != 0) emit(1, "LEAK %s %u live blocks", what, g_live);
  g_nblk = 0;
  g_live = 0;
  g_off = 64;
}

/* ------------------------------------------------------------------------------------------------ the tables: identify what a state points to by CONTENT */
typedef struct {
  const char *name;
  const short *p;
  unsigned bytes;
} Tab;
#define T_(n) {#n, rcfx_##n, sizeof(rcfx_##n)}
static const Tab g_filt[16] = {T_(rc80to72filter), T_(rc72to80filter), T_(rc320to96filter), T_(rc96to320filter), T_(rc120to80filter), T_(rc80to120filter), T_(rc120to96filter),
                               T_(rc96to120filter), T_(rc384to80filter), T_(rc80to384filter), T_(rc384to96filter), T_(rc96to384filter), T_(rc480to96filter), T_(rc480to80filter),
                               T_(rc96to80filter), T_(rc80to96filter)};
static const Tab g_ci[3] = {T_(CI_bDroop), T_(CI_b1), T_(CI_b2)};
/* the candidate whose size is `bytes` (the size the state itself implies: up * taps * 2) and whose content equals what p points to; -1 none, -2 NULL */
static int tab_id(const short *p, const Tab *t, int n, unsigned bytes)
{
  int i;
  if (!p) return -2;
  for (i = 0; i < n; i++)
    if (t[i].bytes == bytes && memcmp(p, t[i].p, t[i].bytes) == 0) return i;
  return -1;
}

/* ------------------------------------------------------------------------------------------------ canonical state dump */
static u64 fnv_shorts(const short *p, unsigned n)
{
  u64 h = 1469598103934665603ULL;
  unsigned i;
  for (i = 0; i < n; i++) {
    h ^= (unsigned short)p[i] & 0xffu;
    h *= 1099511628211ULL;
    h ^= ((unsigned short)p[i] >> 8) & 0xffu;
    h *= 1099511628211ULL;
  }
  return h;
}
static void dump(int show, const char *tag, const RcFixed *o)
{
  if (!o) {
    emit(show, "%s NULL", tag);
    return;
  }
  if (o->state.any == NULL) {
    emit(show, "%s type=%d state=NULL", tag, o->type);
    return;
  }
  if (o->type == 0) {
    const RcPoly *p = o->state.poly;
    emit(show, "%s poly coef=%d phase=%d down=%d up=%d taps=%d pos=%d skip=%d hist=%016llx", tag, tab_id(p->coef, g_filt, 16, 2u * (unsigned)(unsigned short)p->up * (unsigned)(unsigned short)p->taps), p->phase, p->down, p->up, p->taps, p->pos, p->skip,
         fnv_shorts(p->hist, RC_HIST));
  } else if (o->type == 1) {
    const RcDec *d = o->state.dec;
    emit(show, "%s dec mode=%d ca=%d cb=%d cc=%d ia=%d ib=%d ic=%d ha=%016llx hb=%016llx hc=%016llx", tag, d->mode, tab_id(d->coef_a, &g_ci[0], 1, 6), tab_id(d->coef_b, &g_ci[1], 1, 174),
         tab_id(d->coef_c, &g_ci[2], 1, 30), d->idx_a, d->idx_b, d->idx_c, fnv_shorts(d->hist_a, 3), fnv_shorts(d->hist_b, 87), fnv_shorts(d->hist_c, 15));
  } else {
    emit(show, "%s type=%d", tag, o->type);
  }
}

static int h_sample_(unsigned long *ctr, unsigned long first, unsigned long k)
{
  unsigned long c = (*ctr)++;
  return c < first || (c % k) == 0;
}

/* ------------------------------------------------------------------------------------------------ coverage ids */
enum {
  CV_CHK_IDX = 0,       /* 18 ids: 0 .. 17 = Check_Combination returned idx 2 .. 19 */
  CV_CHK_NONE = 18, CV_CHK_NEG, CV_CHK_ZERO_A, CV_CHK_ZERO_B, CV_CHK_BIG, CV_CHK_FPE_00, CV_CHK_FPE_MIN, CV_CHK_MODEL,
  CV_LC_ALL = 30, CV_LC_NULL_CREATE, CV_LC_NULL_DELETE, CV_LC_CRASH20, CV_LC_CRASH999, CV_LC_DEL_TYPE2, CV_LC_DEL_NOSTATE, CV_LC_RESET_POLY, CV_LC_RESET_DEC, CV_LC_RESET_TYPE2,
  CV_LC_MULTI, CV_LC_REV, CV_LC_BOUNDARY999, CV_LC_RS_TYPE2, CV_LC_ODD_NEG, CV_LC_ODD_EQ, CV_LC_ODD_GT,
  CV_P_IDX = 50,        /* 18 ids: poly resample per idx 2 .. 19 (50 .. 67) */
  CV_P_WRAP = 68, CV_P_PARTIAL, CV_P_EXACTSKIP, CV_P_CAPSTOP, CV_P_CAP0, CV_P_SKIPMULTI, CV_P_NOSKIP, CV_P_SKIP1INIT, CV_P_N0, CV_P_OVERFLOW16, CV_P_POKE, CV_P_RESET_MID, CV_P_POSDROP,
  CV_P_MANYCALLS, CV_P_N1,
  CV_D_UP = 90, CV_D_DOWN, CV_D_UPCAP, CV_D_DOWNCAP, CV_D_DOWNPARTIAL, CV_D_N0, CV_D_OVERFLOW16, CV_D_POKE, CV_D_RESET_MID, CV_D_WRAPA, CV_D_WRAPB, CV_D_WRAPC, CV_D_DOWNOUT, CV_D_UPCAP_SMALL,
  CV_D_N1, CV_D_GARBAGE_RESET
};

/* ------------------------------------------------------------------------------------------------ signals */
#define NMAX 4100
static short g_in[NMAX + 16];
static short g_out[NMAX * 8 + 128];

static void gen(short *p, unsigned n, int kind)
{
  unsigned i;
  for (i = 0; i < n; i++) {
    int v;
    switch (kind) {
      case 0: v = (short)rnd(); break;                                   /* full-range random */
      case 1: v = (rnd() & 1) ? 32767 : -32768; break;                   /* worst-case random sign */
      case 2: v = -32768; break;
      case 3: v = 32767; break;
      case 4: v = 0; break;
      case 5: v = (i == 0) ? 32767 : 0; break;                           /* impulse */
      case 6: v = (int)(rnd() % 201) - 100; break;                       /* small */
      case 7: v = (int)((i % 37) * 1080) - 20000; break;                 /* sawtooth */
      case 8: v = (i & 1) ? -32768 : 32767; break;                       /* Nyquist, saturating */
      default: v = (short)(rnd() >> 3); break;
    }
    p[i] = (short)v;
  }
}

/* one Resample call with all the checks and the canonical lines; returns the output count */
static unsigned do_resample(RcFixed *o, unsigned n, unsigned cap, const char *tag, int show)
{
  unsigned nout = cap, i, lim = n * 8 + 64, k;
  u64 h;
  if (lim > sizeof g_out / sizeof g_out[0]) die3("buffer");
  for (i = 0; i < lim; i++) g_out[i] = 0x5a5a;
  RcFixed_Resample(o, g_in, n, g_out, &nout);
  if (nout > lim - 8) die3("%s: nout %u out of range", tag, nout);
  for (k = nout; k < nout + 8 && k < lim; k++)
    if (g_out[k] != 0x5a5a) die3("%s: output written past the reported count %u", tag, nout);
  h = fnv_shorts(g_out, nout);
  emit(show, "%s n=%u cap=%u -> nout=%u out=%016llx", tag, n, cap, nout, h);
  return nout;
}

/* ------------------------------------------------------------------------------------------------ fork probes (inputs that crash the blob) */
static int probe(const char *name, int which, unsigned arg)
{
  pid_t pid;
  int st = 0, fd[2];
  MemsetRec r;
  ssize_t got = 0;
  fflush(stdout);
  if (pipe(fd) != 0) die3("pipe");
  pid = fork();
  if (pid == 0) {
    FILE *z = freopen("/dev/null", "w", stdout);
    (void)z;
    close(fd[0]);
    g_probe_fd = fd[1];
    if (which == 0) RcFixed_Create(arg);
    else if (which == 1) RcFixed_Check_Combination(0, 0);
    else RcFixed_Check_Combination(INT_MIN, -1);
    _exit(0);
  }
  if (pid < 0) die3("fork");
  close(fd[1]);
  waitpid(pid, &st, 0);
  memset(&r, 0, sizeof r);
  got = read(fd[0], &r, sizeof r);
  close(fd[0]);
  {
    int sig = WIFSIGNALED(st) ? WTERMSIG(st) : 0, ex = WIFEXITED(st) ? WEXITSTATUS(st) : -1;
    if (got == (ssize_t)sizeof r) emit(1, "PROBE %s signal=%d exit=%d memset(dst=%llu, c=%d, len=%u)", name, sig, ex, r.dst, r.c, r.len);
    else emit(1, "PROBE %s signal=%d exit=%d", name, sig, ex);
    return sig;
  }
}

/* ================================================================================================ scenario 0: Check_Combination */
static const int k_down[20] = {1, 4, 5, 6, 1, 6, 1, 5, 1, 4, 5, 24, 4, 5, 2, 3, 3, 10, 9, 10};
static const int k_up[20] = {4, 1, 6, 5, 6, 1, 5, 1, 4, 1, 24, 5, 5, 4, 3, 2, 10, 3, 10, 9};
static long long gcd_model(long long a, long long b)
{
  while (b) {
    long long t = a % b;
    a = b;
    b = t;
  }
  return a < 0 ? -a : a;
}
/* independent model for positive arguments: reduce with 64-bit arithmetic, search the legal list by its ratios (the INDEX is the table position) */
static int model_check(long long a, long long b)
{
  long long g = gcd_model(a, b);
  int i;
  for (i = 2; i < 20; i++)
    if (b / g == k_up[i] && a / g == k_down[i]) return i;
  return 20;
}
static void sc_check(void)
{
  int a, b, i, g;
  unsigned long ctr = 0;
  cov_def(CV_CHK_NONE, "check: no combination (20)");
  for (i = 0; i < 18; i++) cov_defn(CV_CHK_IDX + i, "check: legal index", i + 2);
  cov_def(CV_CHK_NEG, "check: negative argument");
  cov_def(CV_CHK_ZERO_A, "check: a == 0");
  cov_def(CV_CHK_ZERO_B, "check: b == 0");
  cov_def(CV_CHK_BIG, "check: large arguments");
  cov_def(CV_CHK_FPE_00, "check: (0,0) traps");
  cov_def(CV_CHK_FPE_MIN, "check: (INT_MIN,-1) traps");
  cov_def(CV_CHK_MODEL, "check: model-compared results");
  arena_init();
  for (a = -20; a <= 700; a++)
    for (b = -20; b <= 700; b++) {
      int r;
      if (a == 0 && b == 0) continue;
      r = RcFixed_Check_Combination(a, b);
      if (r < 2 || r > 20) die3("Check(%d,%d) = %d", a, b, r);
      if (a > 0 && b > 0) {
        if (r != model_check(a, b)) die3("Check(%d,%d) = %d, model %d", a, b, r, model_check(a, b));
        cov_hit(CV_CHK_MODEL);
      }
      if (r == 20) cov_hit(CV_CHK_NONE); else cov_hit(CV_CHK_IDX + r - 2);
      if (a < 0 || b < 0) cov_hit(CV_CHK_NEG);
      if (a == 0) cov_hit(CV_CHK_ZERO_A);
      if (b == 0) cov_hit(CV_CHK_ZERO_B);
      emit(h_sample_(&ctr, 400, 997), "CHK %d %d %d", a, b, r);
    }
  /* every legal pair at every multiple (the rate pairs of real callers are (1000 * k, ...) in Hz) */
  for (i = 2; i < 20; i++)
    for (g = 1; g <= 400; g++) {
      int r = RcFixed_Check_Combination(k_down[i] * g, k_up[i] * g);
      if (r != i) die3("legal pair idx %d g %d -> %d", i, g, r);
      emit(g == 1 || g == 400, "LEGAL %d %d", i, r);
    }
  /* large and negative values (INT_MIN excluded: INT_MIN % -1 traps, probed below) */
  for (i = 0; i < 40000; i++) {
    long long x, y;
    int r;
    switch (i & 3) {
      case 0: a = (int)rnd(); b = (int)rnd(); break;
      case 1: a = (int)(rnd() >> (rmod(32))); b = (int)(rnd() >> rmod(32)); break;
      case 2: g = (int)(rnd() % 4000) + 1; a = k_down[2 + (int)rmod(18)] * g; b = k_up[2 + (int)rmod(18)] * g; break;   /* legal and illegal mixtures */
      default: a = (int)(rnd() % 3000) - 1500; b = (int)(rnd() % 3000) - 1500; break;
    }
    if (a == INT_MIN || b == INT_MIN || (a == 0 && b == 0)) continue;
    x = a; y = b;
    r = RcFixed_Check_Combination(a, b);
    if (x > 0 && y > 0 && r != model_check(x, y)) die3("Check(%d,%d) = %d, model %d", a, b, r, model_check(x, y));
    if (a > 100000 || b > 100000 || a < -100000) cov_hit(CV_CHK_BIG);
    emit(0, "BIG %d %d %d", a, b, r);
  }
  {
    static const int ex[][2] = {{INT_MAX, INT_MAX}, {INT_MAX, 1}, {1, INT_MAX}, {INT_MAX, -INT_MAX}, {-INT_MAX, INT_MAX}, {INT_MIN, 1}, {1, INT_MIN}, {INT_MIN, 2}, {INT_MIN, INT_MIN}, {INT_MIN, 0}, {0, INT_MIN}, {INT_MAX, 0}, {0, INT_MAX}, {-1, -1}, {-5, 6}, {5, -6}, {-5, -6}, {80, 96}, {96, 80}, {8000, 9600},
                                {7200, 8000}, {48000, 8000}, {44100, 8000}, {8000, 48000}};
    for (i = 0; i < (int)(sizeof ex / sizeof ex[0]); i++) {
      int r = RcFixed_Check_Combination(ex[i][0], ex[i][1]);
      emit(1, "EXTREME %d %d %d", ex[i][0], ex[i][1], r);
      cov_hit(CV_CHK_BIG);
    }
  }
  if (probe("check(0,0)", 1, 0) == SIGFPE) cov_hit(CV_CHK_FPE_00);
  if (probe("check(INT_MIN,-1)", 2, 0) == SIGFPE) cov_hit(CV_CHK_FPE_MIN);
  trial_end("check");
}

/* ================================================================================================ scenario 1: lifecycle */
static void sc_lifecycle(void)
{
  static const unsigned null_idx[] = {1000, 1001, 4096, 65535, 0x7fffffffu, 0x80000000u, 0xfffffffeu, 0xffffffffu};
  RcFixed *o[20], *x;
  unsigned i, j, r;
  char tag[64];
  cov_def(CV_LC_ALL, "lifecycle: Create 0..19 dumped");
  cov_def(CV_LC_NULL_CREATE, "lifecycle: Create > 999 -> NULL");
  cov_def(CV_LC_NULL_DELETE, "lifecycle: Delete(NULL)");
  cov_def(CV_LC_CRASH20, "lifecycle: Create(20) crashes (SIGSEGV)");
  cov_def(CV_LC_CRASH999, "lifecycle: Create(999) crashes (SIGSEGV)");
  cov_def(CV_LC_DEL_TYPE2, "lifecycle: Delete with type 2");
  cov_def(CV_LC_DEL_NOSTATE, "lifecycle: Delete with state NULL");
  cov_def(CV_LC_RESET_POLY, "lifecycle: Reset polyphase");
  cov_def(CV_LC_RESET_DEC, "lifecycle: Reset 4x");
  cov_def(CV_LC_RESET_TYPE2, "lifecycle: Reset with type 2 (no-op)");
  cov_def(CV_LC_MULTI, "lifecycle: 20 live objects");
  cov_def(CV_LC_REV, "lifecycle: deleted in reverse order");
  cov_def(CV_LC_BOUNDARY999, "lifecycle: Create(1000) -> NULL, idx 999 probed");
  cov_def(CV_LC_RS_TYPE2, "lifecycle: Resample with type 2 clears *pnOut");
  cov_def(CV_LC_ODD_NEG, "lifecycle: Reset with negative down (remainder fix-up)"); cov_def(CV_LC_ODD_EQ, "lifecycle: Reset with down == up"); cov_def(CV_LC_ODD_GT, "lifecycle: Reset with down > up");
  cov_def(CV_D_GARBAGE_RESET, "lifecycle: 4x Reset over random history");
  cov_def(CV_P_RESET_MID, "poly: Reset after Resample");
  arena_init();
  g_lcg ^= 0x1111u;
  /* every index on its own: dump after Create, after Reset, Delete; the callee log (malloc sizes / order, the memset of the history / 4 bytes) is in the hash */
  for (i = 0; i < 20; i++) {
    x = RcFixed_Create(i);
    if (!x) die3("Create(%u) = NULL", i);
    snprintf(tag, sizeof tag, "CREATE %u", i);
    dump(1, tag, x);
    RcFixed_Reset(x);
    snprintf(tag, sizeof tag, "RESET %u", i);
    dump(1, tag, x);
    RcFixed_Delete(x);
    trial_end("lc1");
  }
  cov_hit(CV_LC_ALL);
  for (i = 0; i < sizeof null_idx / sizeof null_idx[0]; i++) {
    x = RcFixed_Create(null_idx[i]);
    emit(1, "CREATE %u -> %s", null_idx[i], x ? "object" : "NULL");
    if (x) die3("Create(%u) did not return NULL", null_idx[i]);
    cov_hit(CV_LC_NULL_CREATE);
  }
  cov_hit(CV_LC_BOUNDARY999);
  RcFixed_Delete(NULL);
  cov_hit(CV_LC_NULL_DELETE);
  trial_end("lc2");
  /* all objects alive together, different delete orders */
  for (r = 0; r < 3; r++) {
    for (i = 0; i < 20; i++) o[i] = RcFixed_Create(i);
    cov_hit(CV_LC_MULTI);
    for (i = 0; i < 20; i++) {
      RcFixed_Reset(o[i]);
      snprintf(tag, sizeof tag, "MULTI %u", i);
      dump(r == 0, tag, o[i]);
    }
    if (r == 0) for (i = 0; i < 20; i++) RcFixed_Delete(o[i]);
    else if (r == 1) { for (i = 20; i-- > 0;) RcFixed_Delete(o[i]); cov_hit(CV_LC_REV); }
    else {
      unsigned order[20];
      for (i = 0; i < 20; i++) order[i] = i;
      for (i = 19; i > 0; i--) { j = rmod(i + 1); { unsigned t = order[i]; order[i] = order[j]; order[j] = t; } }
      for (i = 0; i < 20; i++) RcFixed_Delete(o[order[i]]);
    }
    trial_end("lc3");
  }
  /* Reset over used state: polyphase (history, phase, skip) and 4x (random history behind the 4 zeroed bytes) */
  for (i = 0; i < 20; i++) {
    unsigned n = 50 + rmod(300), k;
    x = RcFixed_Create(i);
    gen(g_in, n, 0);
    do_resample(x, n, 0, "LCRS", 0);
    if (x->type == 1) {
      for (k = 0; k < 3; k++) x->state.dec->hist_a[k] = (short)rnd();
      for (k = 0; k < 87; k++) x->state.dec->hist_b[k] = (short)rnd();
      for (k = 0; k < 15; k++) x->state.dec->hist_c[k] = (short)rnd();
      x->state.dec->idx_a = (short)rmod(3); x->state.dec->idx_b = (short)rmod(87); x->state.dec->idx_c = (short)rmod(15);
    }
    snprintf(tag, sizeof tag, "USED %u", i);
    dump(1, tag, x);
    RcFixed_Reset(x);
    snprintf(tag, sizeof tag, "RESET-USED %u", i);
    dump(1, tag, x);
    if (x->type == 0) { cov_hit(CV_LC_RESET_POLY); cov_hit(CV_P_RESET_MID); } else { cov_hit(CV_LC_RESET_DEC); cov_hit(CV_D_GARBAGE_RESET); }
    gen(g_in, 60, 0);
    do_resample(x, 60, 0, "LCRS2", 1);
    dump(1, "AFTER", x);
    RcFixed_Delete(x);
    trial_end("lc4");
  }
  /* Reset over states the tables never produce: down <= 0 / == up / > up (the unsigned-looking negative remainder fix-up, up <= down), garbage phase / skip */
  for (i = 2; i < 20; i++) {
    static const int dv[] = {-32768, -300, -7, -3, -1, 0, 1, 2, 3, 5, 6, 9, 10, 24, 119, 32767};
    int up;
    x = RcFixed_Create(i);
    up = x->state.poly->up;
    for (j = 0; j < sizeof dv / sizeof dv[0] + 3; j++) {
      int d = j < sizeof dv / sizeof dv[0] ? dv[j] : (j == sizeof dv / sizeof dv[0] ? up : (j == sizeof dv / sizeof dv[0] + 1 ? up - 1 : up + 1));
      x->state.poly->down = (short)d;
      x->state.poly->phase = (short)(rnd() % 40);
      x->state.poly->skip = (int)rmod(50);
      x->state.poly->pos = (int)rmod(200);
      RcFixed_Reset(x);
      snprintf(tag, sizeof tag, "ODD %u down=%d", i, d);
      dump(j % 5 == 0, tag, x);
      if (d < 0 && d % up != 0) cov_hit(CV_LC_ODD_NEG);
      if (d == up) cov_hit(CV_LC_ODD_EQ);
      if (d > up) cov_hit(CV_LC_ODD_GT);
    }
    RcFixed_Delete(x);
    trial_end("lc8");
  }
  /* odd object types and a missing state */
  for (i = 0; i < 2; i++) {
    unsigned nout = 12345;
    x = RcFixed_Create(i ? 0 : 5);
    x->type = 2 + (int)i * -3; /* 2, then -1 */
    RcFixed_Reset(x);
    cov_hit(CV_LC_RESET_TYPE2);
    dump(1, "TYPE2", x);
    RcFixed_Resample(x, g_in, 10, g_out, &nout);
    emit(1, "TYPE2 resample nout=%u", nout);
    if (nout != 0) die3("odd type: *pnOut not cleared");
    cov_hit(CV_LC_RS_TYPE2);
    RcFixed_Delete(x); /* type != 1: frees the state and the object */
    cov_hit(CV_LC_DEL_TYPE2);
    trial_end("lc5");
  }
  for (i = 0; i < 3; i++) {
    x = RcFixed_Create(i == 0 ? 0 : (i == 1 ? 1 : 7));
    if (x->type == 1) { sysdep_free(x->state.dec->hist_a); sysdep_free(x->state.dec->hist_b); sysdep_free(x->state.dec->hist_c); }
    sysdep_free(x->state.any);
    x->state.any = NULL;
    RcFixed_Delete(x); /* state NULL: only the object */
    cov_hit(CV_LC_DEL_NOSTATE);
    trial_end("lc6");
  }
  /* indices 20 .. 999 crash the blob (state NULL, memset(state + 4)); the port reproduces it */
  if (probe("create(20)", 0, 20) == SIGSEGV) cov_hit(CV_LC_CRASH20);
  probe("create(21)", 0, 21);
  probe("create(100)", 0, 100);
  if (probe("create(999)", 0, 999) == SIGSEGV) cov_hit(CV_LC_CRASH999);
  trial_end("lc7");
}

/* ================================================================================================ scenarios 2 / 3: Resample, random streaks */
static const unsigned k_nlist[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 13, 15, 16, 17, 31, 63, 64, 65, 100, 127, 128, 129, 150, 199, 200, 201, 202, 250, 333, 398, 399, 400, 401, 402, 777, 1000, 1500, 2047, 4000};
#define NLIST ((unsigned)(sizeof k_nlist / sizeof k_nlist[0]))

static void poke_poly(RcFixed *o, int deep)
{
  RcPoly *p = o->state.poly;
  unsigned i;
  for (i = 0; i < RC_HIST; i++) p->hist[i] = (short)rnd();
  p->phase = (short)rmod((unsigned)p->up);
  p->skip = (int)rmod(6);
  if (deep && rpct(30)) p->skip = (int)(rmod(900));
  p->pos = p->taps + (int)rmod((unsigned)(RC_HIST - p->taps));
  if (deep && rpct(25)) p->down = (short)(rmod(120));   /* more input per output (skip loops longer than one step) */
  cov_hit(CV_P_POKE);
}
static void poke_dec(RcFixed *o)
{
  RcDec *d = o->state.dec;
  unsigned k;
  for (k = 0; k < 3; k++) d->hist_a[k] = (short)rnd();
  for (k = 0; k < 87; k++) d->hist_b[k] = (short)rnd();
  for (k = 0; k < 15; k++) d->hist_c[k] = (short)rnd();
  d->idx_a = (short)rmod(3);
  d->idx_b = (short)rmod(87);
  d->idx_c = (short)rmod(15);
  cov_hit(CV_D_POKE);
}

/* the observable-state coverage of one call (before / after) */
static void cover_poly(const RcPoly *b, const RcPoly *a, unsigned n, unsigned cap, unsigned nout)
{
  if (n == 0) cov_hit(CV_P_N0);
  if (n == 1) cov_hit(CV_P_N1);
  if (n > 0 && a->pos < b->pos) cov_hit(CV_P_WRAP);
  if (n > 0 && (unsigned)b->skip > n) cov_hit(CV_P_PARTIAL);
  if (n > 0 && (unsigned)b->skip == n) cov_hit(CV_P_EXACTSKIP);
  if (cap > 0 && nout == cap) cov_hit(CV_P_CAPSTOP);
  if (cap == 0 && nout > 0) cov_hit(CV_P_CAP0);
  if (nout > 0 && a->skip > 1) cov_hit(CV_P_SKIPMULTI);
  if (nout > 0 && a->skip == 0) cov_hit(CV_P_NOSKIP);
}
static void cover_dec(const RcDec *b, const RcDec *a, unsigned n, unsigned cap, unsigned nout)
{
  if (n == 0) cov_hit(CV_D_N0);
  if (n == 1) cov_hit(CV_D_N1);
  if (a->mode == 0) {
    if (nout > 0) cov_hit(CV_D_UP);
    if (cap > 0 && nout >= cap && nout < 4u * n) cov_hit(CV_D_UPCAP);
    if (cap > 0 && cap < 4 && nout == 4) cov_hit(CV_D_UPCAP_SMALL);
  } else {
    if (nout > 0) { cov_hit(CV_D_DOWN); cov_hit(CV_D_DOWNOUT); }
    if (cap > 0 && nout == cap && n >= 4u * cap + 4u) cov_hit(CV_D_DOWNCAP);
    if (n % 4 != 0) cov_hit(CV_D_DOWNPARTIAL);
  }
  if (n > 0 && a->idx_a < b->idx_a) cov_hit(CV_D_WRAPA);
  if (n > 0 && a->idx_b < b->idx_b) cov_hit(CV_D_WRAPB);
  if (n > 0 && a->idx_c < b->idx_c) cov_hit(CV_D_WRAPC);
}
static void call_cov(RcFixed *o, unsigned n, unsigned cap, const char *tag, int show)
{
  RcPoly bp = {0};
  RcDec bd = {0};
  unsigned nout;
  if (o->type == 0) bp = *o->state.poly; else bd = *o->state.dec;
  nout = do_resample(o, n, cap, tag, show);
  if (o->type == 0) cover_poly(&bp, o->state.poly, n, cap, nout); else cover_dec(&bd, o->state.dec, n, cap, nout);
  dump(show, "  st", o);
}
static unsigned pick_cap(unsigned n)
{
  switch (rmod(10)) {
    case 0: case 1: case 2: return 0;             /* unbounded */
    case 3: return 1 + rmod(9);                   /* small */
    case 4: return 1 + rmod(n + 4);               /* around the expected count */
    case 5: return n / 2 + 1;
    default: return 100000;                       /* effectively unbounded but nonzero */
  }
}
static void sc_resample(int dec)
{
  unsigned lo = dec ? 0 : 2, hi = dec ? 2 : 20, idx, t, c, ncalls;
  char tag[48];
  unsigned long ctr = 0;
  int i;
  if (!dec) {
    for (i = 0; i < 18; i++) cov_defn(CV_P_IDX + i, "poly: idx resampled", i + 2);
    cov_def(CV_P_WRAP, "poly: history wrap (pos 200)"); cov_def(CV_P_PARTIAL, "poly: nIn < skip"); cov_def(CV_P_EXACTSKIP, "poly: nIn == skip");
    cov_def(CV_P_CAPSTOP, "poly: stopped by capacity"); cov_def(CV_P_CAP0, "poly: capacity 0 (unbounded)"); cov_def(CV_P_SKIPMULTI, "poly: skip > 1 after an output");
    cov_def(CV_P_NOSKIP, "poly: skip 0 after an output"); cov_def(CV_P_N0, "poly: nIn == 0"); cov_def(CV_P_N1, "poly: nIn == 1");
    cov_def(CV_P_POKE, "poly: state poke"); cov_def(CV_P_RESET_MID, "poly: Reset mid-stream"); cov_def(CV_P_MANYCALLS, "poly: >= 10 calls on one object");
  } else {
    cov_def(CV_D_UP, "4x: interpolate"); cov_def(CV_D_DOWN, "4x: decimate"); cov_def(CV_D_UPCAP, "4x up: stopped by capacity");
    cov_def(CV_D_UPCAP_SMALL, "4x up: capacity < 4"); cov_def(CV_D_DOWNCAP, "4x down: stopped by capacity");
    cov_def(CV_D_DOWNPARTIAL, "4x down: nIn % 4 != 0"); cov_def(CV_D_DOWNOUT, "4x down: output produced"); cov_def(CV_D_N0, "4x: nIn == 0"); cov_def(CV_D_N1, "4x: nIn == 1");
    cov_def(CV_D_POKE, "4x: state poke"); cov_def(CV_D_RESET_MID, "4x: Reset mid-stream"); cov_def(CV_D_WRAPA, "4x: stage A index wrap");
    cov_def(CV_D_WRAPB, "4x: stage B index wrap"); cov_def(CV_D_WRAPC, "4x: stage C index wrap");
  }
  arena_init();
  g_lcg ^= dec ? 0x3333u : 0x2222u;
  for (idx = lo; idx < hi; idx++)
    for (t = 0; t < 26; t++) {
      RcFixed *o = RcFixed_Create(idx);
      ncalls = 3 + rmod(18);
      if (!dec && o->state.poly->skip == 1) cov_hit(CV_P_SKIP1INIT);
      if (t == 0) { snprintf(tag, sizeof tag, "NEW %u", idx); dump(1, tag, o); }
      if (!dec) cov_hit(CV_P_IDX + (int)idx - 2);
      if (ncalls >= 10) cov_hit(CV_P_MANYCALLS);
      for (c = 0; c < ncalls; c++) {
        unsigned n = rpct(70) ? k_nlist[rmod(NLIST)] : rmod(601);
        unsigned cap = pick_cap(n);
        if (rpct(8)) { RcFixed_Reset(o); cov_hit(dec ? CV_D_RESET_MID : CV_P_RESET_MID); emit(0, "RESET"); }
        if (rpct(10)) { if (dec) poke_dec(o); else poke_poly(o, 1); }
        gen(g_in, n, rpct(55) ? 0 : (int)rmod(10));
        snprintf(tag, sizeof tag, "RS %u.%u.%u", idx, t, c);
        call_cov(o, n, cap, tag, h_sample_(&ctr, 40, 61));
      }
      RcFixed_Delete(o);
      trial_end("rs");
    }
}

/* ================================================================================================ scenario 4: designed grid */
static void sc_grid(void)
{
  unsigned idx, k, kind, c;
  unsigned long ctr = 0;
  char tag[48];
  static const unsigned gn[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 15, 16, 17, 31, 32, 33, 65, 100, 128, 199, 200, 201, 202, 203, 204, 205, 239, 240, 241, 398, 399, 400, 401, 402, 403, 600, 1001, 2047};
  int i;
  for (i = 0; i < 18; i++) cov_defn(CV_P_IDX + i, "poly: idx resampled", i + 2);
  cov_def(CV_P_WRAP, "poly: history wrap (pos 200)"); cov_def(CV_P_PARTIAL, "poly: nIn < skip"); cov_def(CV_P_EXACTSKIP, "poly: nIn == skip");
  cov_def(CV_P_SKIPMULTI, "poly: skip > 1 after an output"); cov_def(CV_P_NOSKIP, "poly: skip 0 after an output"); cov_def(CV_P_N0, "poly: nIn == 0");
  cov_def(CV_P_N1, "poly: nIn == 1"); cov_def(CV_P_SKIP1INIT, "poly: initial skip 1 (up <= down)");
  cov_def(CV_D_UP, "4x: interpolate"); cov_def(CV_D_DOWN, "4x: decimate"); cov_def(CV_D_DOWNPARTIAL, "4x down: nIn % 4 != 0"); cov_def(CV_D_DOWNOUT, "4x down: output produced");
  cov_def(CV_D_N0, "4x: nIn == 0"); cov_def(CV_D_N1, "4x: nIn == 1");
  cov_def(CV_D_WRAPA, "4x: stage A index wrap"); cov_def(CV_D_WRAPB, "4x: stage B index wrap"); cov_def(CV_D_WRAPC, "4x: stage C index wrap");
  arena_init();
  g_lcg ^= 0x4444u;
  for (idx = 0; idx < 20; idx++)
    for (k = 0; k < sizeof gn / sizeof gn[0]; k++)
      for (kind = 0; kind < 9; kind++) {
        unsigned n = gn[k];
        RcFixed *o = RcFixed_Create(idx);
        if (o->type == 0) { cov_hit(CV_P_IDX + (int)idx - 2); if (o->state.poly->skip == 1) cov_hit(CV_P_SKIP1INIT); }
        gen(g_in, n, (int)kind);
        snprintf(tag, sizeof tag, "GR %u.%u.%u", idx, n, kind);
        call_cov(o, n, 0, tag, h_sample_(&ctr, 40, 97));
        /* carry-over: more calls with other signals (the second block continues the state of the first) */
        for (c = 0; c < 3; c++) {
          unsigned m = k_nlist[rmod(NLIST)] % 450;
          gen(g_in, m, (int)((kind + c + 1) % 9));
          snprintf(tag, sizeof tag, "GR2 %u.%u.%u.%u", idx, n, kind, c);
          call_cov(o, m, rpct(25) ? 0 : 100000, tag, h_sample_(&ctr, 40, 197));
        }
        RcFixed_Delete(o);
        trial_end("grid");
      }
}

/* ================================================================================================ scenario 5: capacities and odd types */
static void sc_caps(void)
{
  static const unsigned gn[] = {1, 2, 3, 4, 5, 8, 9, 50, 201, 400};
  unsigned idx, k, c, expect, seed;
  unsigned long ctr = 0;
  char tag[48];
  int i;
  for (i = 0; i < 18; i++) cov_defn(CV_P_IDX + i, "poly: idx resampled", i + 2);
  cov_def(CV_P_CAPSTOP, "poly: stopped by capacity"); cov_def(CV_P_CAP0, "poly: capacity 0 (unbounded)");
  cov_def(CV_D_UPCAP, "4x up: stopped by capacity"); cov_def(CV_D_UPCAP_SMALL, "4x up: capacity < 4"); cov_def(CV_D_DOWNCAP, "4x down: stopped by capacity");
  cov_def(CV_D_DOWNOUT, "4x down: output produced"); cov_def(CV_D_UP, "4x: interpolate");
  arena_init();
  g_lcg ^= 0x5555u;
  for (idx = 0; idx < 20; idx++)
    for (k = 0; k < sizeof gn / sizeof gn[0]; k++) {
      unsigned n = gn[k] * (idx < 2 ? 4 : 3), caps[24], nc = 0;
      seed = rnd();
      {
        RcFixed *o = RcFixed_Create(idx);
        g_lcg ^= 0; gen(g_in, n, 0);
        expect = do_resample(o, n, 0, "REF", 0);
        RcFixed_Delete(o);
        trial_end("caps0");
      }
      for (c = 0; c <= 9; c++) caps[nc++] = c;
      caps[nc++] = expect ? expect - 1 : 0;
      caps[nc++] = expect;
      caps[nc++] = expect + 1;
      caps[nc++] = expect / 2;
      caps[nc++] = 100000;
      caps[nc++] = 0xffffffffu;
      for (c = 0; c < nc; c++) {
        RcFixed *o = RcFixed_Create(idx);
        (void)seed;
        if (o->type == 0) cov_hit(CV_P_IDX + (int)idx - 2);
        snprintf(tag, sizeof tag, "CAP %u.%u.%u", idx, n, caps[c]);
        call_cov(o, n, caps[c], tag, h_sample_(&ctr, 40, 31));
        /* a second call on the same state, other capacity */
        gen(g_in, 37, 0);
        call_cov(o, 37, caps[(c + 3) % nc], "CAP2", 0);
        RcFixed_Delete(o);
        trial_end("caps");
      }
    }
}

int main(int argc, char **argv)
{
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("RCFIXED_SEED");
  if (seed) g_lcg ^= (unsigned)strtoul(seed, NULL, 0) * 2654435761u;
  switch (sc) {
    case 0: sc_check(); break;
    case 1: sc_lifecycle(); break;
    case 2: sc_resample(0); break;
    case 3: sc_resample(1); break;
    case 4: sc_grid(); break;
    case 5: sc_caps(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  emit(1, "calls: malloc %lu free %lu memset %lu", g_nmalloc, g_nfree, g_nmemset);
  cov_finish();
  return 0;
}
