/*
 * R20 harness library (header only, used by test/fax/fax_v27_tx.main.c; a copy of r19_common.h with the V.27ter transmit object sizes mapped): canonical output (every line is FNV-hashed, only selected lines are printed unless R18_VERBOSE=1, then
 * "HASH <h> lines <n>"), a PRNG, required coverage counters (a zero exits 2; 3 = harness violation), a deterministic heap (red zones, poisoned frees, id-patterned fresh blocks)
 * with the callee recorders sysdep_malloc / sysdep_free (block ids and sizes; sizeof(FaxFifo) / sizeof(FaxSgd) are logged as their i386 sizes 0x14 / 0x5c, so the log is the same
 * on i386 and x86-64).  The code under test (blob or port) calls these through its normal external symbols.  Everything printed is architecture independent: the 32-bit BLOB
 * run is the golden of x86-64.
 */
#ifndef R20_COMMON_H
#define R20_COMMON_H
#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fax/fax_common_tables.h"
#include "fax/fax_v27_tx.h"

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
static u64 fnv_ints(const int *p, unsigned n)
{
  u64 h = 1469598103934665603ULL;
  unsigned i;
  int k;
  for (i = 0; i < n; i++)
    for (k = 0; k < 4; k++) {
      h ^= ((unsigned)p[i] >> (8 * k)) & 0xffu;
      h *= 1099511628211ULL;
    }
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

#define COV_MAX 200
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
#define ARENA_SZ (4u << 20)
#define REDZ 48u
#define MAXBLK 2048
typedef struct {
  unsigned char *p;
  unsigned size, id;
  int freed;
} Blk;
static unsigned char g_arena[ARENA_SZ] __attribute__((aligned(64)));
static size_t g_off = 64;
static Blk g_blk[MAXBLK];
static unsigned g_nblk, g_next_id = 1, g_live;
static unsigned long g_nmalloc, g_nfree, g_nmemset, g_ngp;
static int g_fail_malloc = -1; /* the n-th (0 based, counted from the last trial_end) sysdep_malloc returns NULL; -1 = never */
static unsigned g_malloc_ctr;

/* the logged size of a block.  The sizes of the typed objects (V27Tx / hdx / bundle / FaxFifo / FaxSgd) depend on the host's pointer width; a plain buffer can have
 * the same byte size as such an object on the OTHER host (24 bytes: the x86-64 FaxFifo and a 12-short PPS history), so every size that is the size of a typed object on EITHER host
 * is logged as 0x0fff (the i386 sizes are pinned by _Static_asserts in the object headers, the x86-64 ones by the layout). */
static unsigned log_size(unsigned size)
{
  static const unsigned masked[] = {44, 56, 24, 40, 148, 216, 20, 112, 92};
  unsigned i;
  for (i = 0; i < sizeof masked / sizeof masked[0]; i++)
    if (size == masked[i]) return 0x0fffu;
  return size;
}
static Blk *find_blk(const void *q)
{
  unsigned i;
  const unsigned char *p = (const unsigned char *)q;
  for (i = 0; i < g_nblk; i++)
    if (p >= g_blk[i].p && p < g_blk[i].p + g_blk[i].size) return &g_blk[i];
  return NULL;
}
void *sysdep_malloc(unsigned int size)
{
  size_t asz = ((size_t)(size ? size : 1u) + 15u) & ~(size_t)15u, i;
  static int ainit;
  if (!ainit) {
    memset(g_arena, 0x5A, sizeof g_arena);
    ainit = 1;
  }
  unsigned id = g_next_id++;
  unsigned char *p;
  if (g_fail_malloc >= 0 && g_malloc_ctr++ == (unsigned)g_fail_malloc) {
    emit(0, "M %u %u NULL", id, log_size(size));
    return NULL;
  }
  if (g_off + REDZ + asz + REDZ > ARENA_SZ || g_nblk >= MAXBLK) die3("arena exhausted");
  memset(g_arena + g_off, 0xCC, REDZ);
  p = g_arena + g_off + REDZ;
  for (i = 0; i < asz; i++) p[i] = (unsigned char)(0x35u + id * 13u + (unsigned)i * 7u + ((unsigned)i >> 6)); /* fresh blocks hold deterministic garbage */
  memset(p + (size ? size : 1u), 0xCC, asz - (size ? size : 1u)); /* the alignment padding after the block is a red zone too: a one-element overrun is seen */
  memset(p + asz, 0xCC, REDZ);
  g_off += REDZ + asz + REDZ;
  g_blk[g_nblk].p = p;
  g_blk[g_nblk].size = size ? size : 1; /* a size-0 block still owns one byte (find_blk) */
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
static unsigned blk_id_of(const void *p)
{
  Blk *b = find_blk(p);
  return b ? b->id : 0;
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
  { /* nothing beyond the top of the heap may have been written (a far overrun misses the red zones): the arena above g_off keeps its 0x5A canary (written lazily by the first sysdep_malloc) */
    size_t lim = g_off + (256u << 10) < ARENA_SZ ? g_off + (256u << 10) : ARENA_SZ;
    for (k = g_off; k < lim; k++)
      if (g_arena[k] != 0x5A) die3("a byte beyond the top of the heap was written (%s, +%u)", what, (unsigned)(k - g_off));
    memset(g_arena, 0x5A, g_off);
  }
  g_nblk = 0;
  g_live = 0;
  g_off = 64;
  g_malloc_ctr = 0;
  g_fail_malloc = -1;
}


/* ------------------------------------------------------------------------------------------------ layout independent pointer descriptions */
typedef struct {
  const char *name;
  const void *base;
  unsigned bytes;
} HTab;
static HTab g_htab[64];
static int g_nhtab;
static void htab_add(const char *name, const void *base, unsigned bytes)
{
  if (g_nhtab >= 64) die3("htab");
  g_htab[g_nhtab].name = name;
  g_htab[g_nhtab].base = base;
  g_htab[g_nhtab].bytes = bytes;
  g_nhtab++;
}
static const char *dptr(const void *p, char *buf, size_t cap)
{
  int i;
  unsigned id;
  if (!p) {
    snprintf(buf, cap, "NULL");
    return buf;
  }
  for (i = 0; i < g_nhtab; i++) {
    const char *b = (const char *)g_htab[i].base;
    if ((const char *)p >= b && (const char *)p < b + g_htab[i].bytes) {
      snprintf(buf, cap, "%s+%u", g_htab[i].name, (unsigned)(((const char *)p - b) / 2));
      return buf;
    }
  }
  id = blk_id_of(p);
  if (id) {
    snprintf(buf, cap, "blk%u", id);
    return buf;
  }
  snprintf(buf, cap, "?");
  return buf;
}
#endif
