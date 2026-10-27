/*
 * R09 harness library (header only, used by test/tone/tone.main.c): canonical output (every line is FNV-hashed, only selected lines are printed unless R09_VERBOSE=1, then
 * "HASH <h> lines <n>"), a PRNG, required coverage counters (a zero exits 2; 3 = harness violation), a deterministic heap (red zones, poisoned frees, id-patterned fresh blocks:
 * the blob's toneiir_create reads one uninitialised field of a freshly allocated object) with the callee recorders sysdep_malloc / sysdep_free / sysdep_memset (block ids, offsets
 * and sizes mapped to the i386 sizes: sizeof(Cadence) -> 0x2dc), dsplibs_debug_printf (the formatted message) and modem_get_param (a scripted parameter table; every call is logged).
 * The code under test (blob or port) calls these through its normal external symbols.  Everything printed is architecture independent: the 32-bit BLOB run is the golden of x86-64.
 */
#ifndef R09_COMMON_H
#define R09_COMMON_H
#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tone/tone_detect.h"

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

static unsigned log_size(unsigned size)
{
  if (size == sizeof(Cadence)) return 0x2dc;
  return size; /* ToneIir: 0xa8 on every host */
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
  size_t asz = ((size_t)size + 15u) & ~(size_t)15u, i;
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
void *sysdep_memset(void *dst, int c, size_t len)
{
  Blk *b = find_blk(dst);
  if (!b) die3("memset outside the heap");
  if (b->freed) die3("memset of a freed block");
  g_nmemset++;
  emit(0, "S %u %u %d %u", b->id, (unsigned)((unsigned char *)dst - b->p), c, (unsigned)(b->size == len ? log_size((unsigned)len) : (unsigned)len));
  return memset(dst, c, len);
}
static unsigned blk_id_of(const void *p)
{
  Blk *b = find_blk(p);
  return b ? b->id : 0;
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
      for (k = 0; k < asz; k++)
        if (b->p[k] != 0xDD) die3("block %u written after free (%s)", b->id, what);
    }
  }
  if (g_live != 0) emit(1, "LEAK %s %u live blocks", what, g_live);
  g_nblk = 0;
  g_live = 0;
  g_off = 64;
  g_malloc_ctr = 0;
  g_fail_malloc = -1;
}

/* ------------------------------------------------------------------------------------------------ debug printf recorder */
unsigned int dsplibs_debug_level = 0;
static void (*g_dbg_hook)(const char *msg); /* coverage observer of the debug text (set by the harness main) */
static unsigned lvl_value(int k)
{
  static const unsigned v[7] = {0u, 1u, 2u, 3u, 4u, 0x80000000u, 0xffffffffu};
  return v[(unsigned)k % 7u];
}
int dsplibs_debug_printf(const char *fmt, ...)
{
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  {
    char *e = buf + strlen(buf);
    while (e > buf && (e[-1] == '\n' || e[-1] == '\r')) *--e = 0;
  }
  emit(0, "DBG %s", buf);
  if (g_dbg_hook) g_dbg_hook(buf);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ the scripted modem parameters */
static int g_fake_modem;
#define FAKEM ((struct modem *)&g_fake_modem)
#define PRM_MAX 128
static long g_prm[PRM_MAX]; /* modem_get_param(m, id) results */
long modem_get_param(struct modem *m, unsigned name)
{
  long v = name < PRM_MAX ? g_prm[name] : -1;
  if (m != FAKEM) die3("modem_get_param with a foreign modem pointer");
  g_ngp++;
  emit(0, "GP %u -> %ld", name, v);
  return v;
}

/* ------------------------------------------------------------------------------------------------ layout independent descriptions */
typedef struct {
  const char *name;
  const short *base;
  unsigned bytes;
} HTab;
static HTab g_htab[64];
static int g_nhtab;
static void htab_add(const char *name, const void *base, unsigned bytes)
{
  if (g_nhtab >= 64) die3("htab");
  g_htab[g_nhtab].name = name;
  g_htab[g_nhtab].base = (const short *)base;
  g_htab[g_nhtab].bytes = bytes;
  g_nhtab++;
  tone_tab_register(name, base, bytes); /* x86-64: addressable by a table word */
}
static const char *dptr(const void *p, char *buf, size_t cap)
{
  int i;
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
  snprintf(buf, cap, "?");
  return buf;
}
static const char *dword(int w, char *buf, size_t cap) { return dptr(w ? (const void *)tone_ptr(w) : NULL, buf, cap); }
#endif
