/*
 * R04 harness library (header only, used by test/r04_fse_diff_main.c and test/r04_sre_diff_main.c): canonical output (every line is FNV-hashed, only selected lines
 * are printed, then "HASH <h> lines <n>"), a PRNG, required coverage counters (a zero exits 2), a deterministic heap (red zones, poisoned frees, id-patterned fresh
 * blocks: the blob reads uninitialised heap) and the callee recorders sysdep_malloc / sysdep_free / sysdep_memcpy (each call is logged with block ids and OFFSETS IN
 * SHORTS, so the log is the same on i386 and x86-64).  The FPM_atan / FPM_phasor callees are the real ones of each side (blob: dsplibs.o; port: src/v32/v32_math.c,
 * src/fpm/fpm_math.c).  Everything printed is architecture independent: the 32-bit BLOB run is the golden of the x86-64 build.
 */
#ifndef R04_COMMON_H
#define R04_COMMON_H
#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned long long u64;

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
  char buf[1024];
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
static int rrange(int lo, int hi) { return lo + (int)rmod((unsigned)(hi - lo + 1)); }
static short rshort(void) { return (short)rnd(); }

#define COV_MAX 160
static unsigned long g_cov[COV_MAX];
static const char *g_covname[COV_MAX];
static void cov_def(int id, const char *name) { g_covname[id] = name; }
static void cov_hit(int id) { g_cov[id]++; }
static void cov_finish(void)
{
  int i, bad = 0;
  for (i = 0; i < COV_MAX; i++) {
    if (!g_covname[i]) continue;
    emit(1, "COV %-48s %lu", g_covname[i], g_cov[i]);
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
#define ARENA_SZ (1u << 20)
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
static unsigned long g_nmalloc, g_nfree, g_nmemcpy;
static const short *g_in_base; /* the harness input buffer of the current call (memcpy sources are logged relative to it) */
static unsigned g_in_len;
static int g_log_memcpy = 1;

void *sysdep_malloc(unsigned int size)
{
  size_t asz = ((size_t)size + 15u) & ~(size_t)15u, i;
  unsigned id = g_next_id++;
  unsigned char *p;
  if (g_off + REDZ + asz + REDZ > ARENA_SZ || g_nblk >= MAXBLK) die3("arena exhausted");
  memset(g_arena + g_off, 0xCC, REDZ);
  p = g_arena + g_off + REDZ;
  for (i = 0; i < asz; i++) p[i] = (unsigned char)(0x35u + id * 13u + (unsigned)i * 7u + ((unsigned)i >> 6));
  memset(p + asz, 0xCC, REDZ);
  g_off += REDZ + asz + REDZ;
  g_blk[g_nblk].p = p;
  g_blk[g_nblk].size = size;
  g_blk[g_nblk].id = id;
  g_blk[g_nblk].freed = 0;
  g_nblk++;
  g_live++;
  g_nmalloc++;
  emit(0, "M %u %u", id, size);
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
  { /* an overrun is checked HERE: the poisoning below would erase the evidence in the gap behind the block */
    size_t asz = ((size_t)b->size + 15u) & ~(size_t)15u, k;
    for (k = 0; k < REDZ; k++)
      if (b->p[asz + k] != 0xCC || b->p[(size_t)0 - REDZ + k] != 0xCC) die3("block %u: a byte outside the block was written (at free)", b->id);
    for (k = b->size; k < asz; k++)
      if (b->p[k] != (unsigned char)(0x35u + b->id * 13u + (unsigned)k * 7u + ((unsigned)k >> 6))) die3("block %u: the gap behind it was written (at free)", b->id);
  }
  b->freed = 1;
  g_live--;
  g_nfree++;
  memset(b->p, 0xDD, ((size_t)b->size + 15u) & ~(size_t)15u);
  emit(0, "F %u", b->id);
}
/* where a pointer lives: "b<id>+<shorts>" inside a heap block, "in+<shorts>" in the input buffer, "NULL" */
static void ptr_desc(char *buf, size_t n, const void *q)
{
  Blk *b;
  if (!q) {
    snprintf(buf, n, "NULL");
    return;
  }
  b = find_blk(q);
  if (!b && g_nblk) { /* one past the end of a block (a zero-length copy) */
    unsigned i;
    for (i = 0; i < g_nblk; i++)
      if ((const unsigned char *)q == g_blk[i].p + g_blk[i].size) {
        snprintf(buf, n, "b%u+%u", g_blk[i].id, g_blk[i].size / 2);
        return;
      }
  }
  if (b) {
    snprintf(buf, n, "b%u+%u", b->id, (unsigned)(((const unsigned char *)q - b->p) / 2));
    return;
  }
  if (g_in_base && (const short *)q >= g_in_base && (const short *)q <= g_in_base + g_in_len) {
    snprintf(buf, n, "in+%u", (unsigned)((const short *)q - g_in_base));
    return;
  }
  snprintf(buf, n, "?");
}
void *sysdep_memcpy(void *dst, const void *src, size_t len)
{
  char d[40], s[40];
  Blk *b = find_blk(dst);
  if (!b && len != 0) die3("memcpy to a pointer outside the heap");
  if (b && b->freed) die3("memcpy to a freed block");
  if (b && (const unsigned char *)dst + len > b->p + b->size) die3("memcpy of %u bytes overruns block %u (size %u)", (unsigned)len, b->id, b->size);
  ptr_desc(d, sizeof d, dst);
  ptr_desc(s, sizeof s, src);
  g_nmemcpy++;
  if (g_log_memcpy) emit(0, "C %s %s %u", d, s, (unsigned)len);
  return memcpy(dst, src, len);
}
/* end of a trial: no byte outside a block may have been written, freed blocks stay poisoned, the gap behind a block is untouched */
static void trial_end(const char *what, int expect_leak)
{
  unsigned i;
  size_t k;
  for (i = 0; i < g_nblk; i++) {
    const Blk *b = &g_blk[i];
    size_t asz = ((size_t)b->size + 15u) & ~(size_t)15u;
    for (k = 0; k < REDZ; k++)
      if (b->p[asz + k] != 0xCC || b->p[(size_t)0 - REDZ + k] != 0xCC) die3("block %u: a byte outside the block was written (%s)", b->id, what);
    if (b->freed) {
      for (k = 0; k < asz; k++)
        if (b->p[k] != 0xDD) die3("block %u written after free (%s)", b->id, what);
    } else {
      for (k = b->size; k < asz; k++)
        if (b->p[k] != (unsigned char)(0x35u + b->id * 13u + (unsigned)k * 7u + ((unsigned)k >> 6))) die3("block %u: the gap behind it was written (%s)", b->id, what);
    }
  }
  if (g_live != 0 && !expect_leak) emit(1, "LEAK %s %u live blocks", what, g_live);
  g_nblk = 0;
  g_live = 0;
  g_off = 64;
}

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
#endif
