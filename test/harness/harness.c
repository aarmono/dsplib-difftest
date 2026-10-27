/*
 * Shared differential-test harness library (see harness.h).  Extracted from test/p01_diff_main.c (P01) without changing a
 * single output byte: the P01 golden dump (test/golden/p01_blob32.out) is produced identically through this library.
 * Plain C99; links with -m32 and -m64 builds.  Compile with the same flags as the harness main.
 */
#include "harness.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

/* ----------------------------------------------------------------------------------------------------------- PRNG */
static unsigned int g_lcg = 0x4c9e2b71u;
void h_seed_xor(unsigned int seed_word) { g_lcg ^= seed_word * 2654435761u; }
unsigned int h_rnd(void) {
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
unsigned int h_rmod(unsigned int m) { return (h_rnd() >> 8) % m; }
int h_rrange(int lo, int hi) { return lo + (int)h_rmod((unsigned int)(hi - lo + 1)); }
int h_rpct(int pct) { return (int)h_rmod(100) < pct; }

/* ----------------------------------------------------------------------------------------------- canonical output */
static u64 g_hash = 1469598103934665603ULL;
static u64 g_lines;
static int g_showall = -1;
void h_hash_bytes(const void *p, size_t n) {
  const unsigned char *b = (const unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) {
    g_hash ^= b[i];
    g_hash *= 1099511628211ULL;
  }
}
void h_die3(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  fprintf(stderr, "HARNESS/MODEL VIOLATION: ");
  vfprintf(stderr, fmt, ap);
  fprintf(stderr, "\n");
  va_end(ap);
  exit(3);
}
/* every line is hashed; `show` lines are printed too */
void h_emit(int show, const char *fmt, ...) {
  char line[4096];
  va_list ap;
  int n;
  va_start(ap, fmt);
  n = vsnprintf(line, sizeof line - 1, fmt, ap);
  va_end(ap);
  if (n < 0 || n >= (int)sizeof line - 1) h_die3("emit overflow");
  line[n++] = '\n';
  h_hash_bytes(line, (size_t)n);
  g_lines++;
  if (g_showall < 0) g_showall = getenv("H_SHOWALL") != NULL; /* debugging: print EVERY line (the hash line then differs from the golden) */
  if (show || g_showall) fwrite(line, 1, (size_t)n, stdout);
}
/* sampling: print a line when the counter hits a multiple of `k` (or the first `first` times) */
int h_sample(unsigned long *ctr, unsigned long first, unsigned long k) {
  unsigned long c = (*ctr)++;
  return c < first || (c % k) == 0;
}
uint32_t h_garbage_hi(uint32_t lo8) { return lo8 | (h_rnd() << 8); }
uint32_t h_bits_of(float f) { uint32_t b; memcpy(&b, &f, 4); return b; }
float h_float_from_bits(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }
float h_float_from_bits_quiet(uint32_t b) { /* keeps bit patterns exactly except that a signalling NaN is made quiet (x87 loads would) */
  if ((b & 0x7f800000u) == 0x7f800000u && (b & 0x007fffffu)) b |= 0x00400000u;
  return h_float_from_bits(b);
}

void h_x87_check(void) {
#if defined(__i386__) || defined(__x86_64__)
  unsigned short cw;
  __asm__ volatile("fnstcw %0" : "=m"(cw));
  if ((cw & 0x300) != 0x300 || (cw & 0xc00) != 0) h_die3("x87 control word is 0x%04x (expected 64-bit mantissa, round to nearest)", (unsigned)cw);
#endif
}

/* ------------------------------------------------------------------------------------------------ coverage counters */
static unsigned long g_cov[H_COV_MAX];
static const char *g_covname[H_COV_MAX];
unsigned char h_covreq[H_COV_MAX];
static int g_ncov;
void h_cov_define(int id, const char *name) {
  g_covname[id] = name;
  h_covreq[id] = 1;
  if (id >= g_ncov) g_ncov = id + 1;
}
void h_cov_hit(int id) { g_cov[id]++; }
void h_cov_finish(void) {
  int i, bad = 0;
  for (i = 0; i < g_ncov; i++) {
    if (!g_covname[i]) continue;
    h_emit(1, "COV %-28s %lu", g_covname[i], g_cov[i]);
    if (h_covreq[i] && g_cov[i] == 0) {
      fprintf(stderr, "COVERAGE FAIL: %s was never reached\n", g_covname[i]);
      bad = 1;
    }
  }
  h_emit(1, "HASH %016llx lines %llu", g_hash, g_lines);
  fflush(stdout);
  if (bad) exit(2);
}

/* ------------------------------------------------------------------------ deterministic allocator, table and log */
#define ARENA_SZ (48u << 20)
#define REDZ 48u
/* The arena is mapped so that the 2^31 boundary lies 4 MB inside it: heap blocks placed around 0x80000000 make pointer compares
 * that are accidentally SIGNED (an `(int32_t)p < (int32_t)q`) differ from the unsigned ones the blob uses (32-bit heaps do
 * live above 2 GB).  If the fixed mapping is refused, a static buffer is used and the straddling coverage counter reports it. */
#define ARENA_BOUNDARY_OFF 0x400000u
static uint8_t g_arena_static[ARENA_SZ] __attribute__((aligned(64)));
static uint8_t *g_arena = g_arena_static;
static int g_arena_straddles;
static size_t g_arena_off = 64;
static unsigned g_trial_no;
static unsigned g_align_mask = ~0u; /* 3 when the blocks hold 4-byte elements (int / float), else any byte offset */
void h_arena_init(void) {
  void *want = (void *)(uintptr_t)(0x80000000u - ARENA_BOUNDARY_OFF);
  void *m = mmap(want, ARENA_SZ, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | 0x100000 /* MAP_FIXED_NOREPLACE */, -1, 0);
  if (m == want) { g_arena = (uint8_t *)m; g_arena_straddles = 1; }
  else if (m != MAP_FAILED) munmap(m, ARENA_SZ);
}
HAl *h_al;
static size_t g_nal, g_capal;
static uint32_t g_next_id = 1;
unsigned h_live;
unsigned long h_nmalloc, h_nfree;
unsigned h_fail_malloc_n;  /* test hook: the n-th next sysdep_malloc returns NULL (once; 1 = the next one); default 0 = off */
int h_fail_next_malloc;   /* test hook: when set, the next sysdep_malloc returns NULL once (logged as "M NULL <size>", not counted); default 0 = no change for P01 / P02 */
unsigned h_last_alloc = 0;
static unsigned g_log_tab_from[8], g_log_tab_to[8], g_log_tab_n;
void h_log_map_add(unsigned from, unsigned to) { if (g_log_tab_n >= 8) h_die3("h_log_map_add: table full"); g_log_tab_from[g_log_tab_n] = from; g_log_tab_to[g_log_tab_n++] = to; }
void h_log_map_clear(void) { g_log_tab_n = 0; }
static unsigned log_size_of(unsigned size) {
  unsigned i;
  if (h_log_next_size) return h_log_next_size;
  if (h_log_map_from && size == h_log_map_from) return h_log_map_to;
  for (i = 0; i < g_log_tab_n; i++) if (size == g_log_tab_from[i]) return g_log_tab_to[i];
  return size;
}
unsigned h_log_map_from, h_log_map_to;  /* persistent: every sysdep_malloc of exactly h_log_map_from bytes is logged as h_log_map_to (several objects with pointers, e.g. two heap FloatFIR of P16: sizeof differs on i386 / x86-64); 0 = off */
unsigned h_log_next_size;  /* one-shot: the next sysdep_malloc logs this size instead of the real one (an object that holds pointers has a different sizeof on i386 / x86-64); default 0 = off */
unsigned h_wild;
/* "store order" probe: while a constructor runs, a sysdep_malloc call records which fields of the object under construction were already
 * written at that moment (compared with a shadow copy of the random fill) - the blob stores some fields before and some after the allocation */
const uint8_t *h_mp_obj, *h_mp_shadow;
const void *h_mp_flds;
int h_mp_nf;
static void mp_report(void);

/* optionally (h_rz_by_id) the red zone bytes of a block depend on its id: a stray write that copies another block's red zone (e.g. an over-read of one block stored past the end of the next) must not
 * look like an unchanged red zone */
int h_rz_by_id;   /* 0 (default, P01 - P04: their goldens contain red-zone bytes read past a block): every red-zone byte is 0xCC; 1: the byte depends on the block id */
static uint8_t rz_byte(uint32_t id) { return h_rz_by_id ? (uint8_t)(0xCCu ^ ((id * 37u) & 0x3fu)) : 0xCCu; }
void *sysdep_malloc(unsigned size) {
  if (h_fail_next_malloc || (h_fail_malloc_n && --h_fail_malloc_n == 0)) { h_fail_next_malloc = 0; h_emit(0, "M NULL %u", size); return NULL; }
  size_t asz = ((size_t)size + 15u) & ~(size_t)15u;
  HAl *a;
  uint8_t *p;
  uint32_t id = g_next_id++;
  size_t i;
  if (g_arena_off + REDZ + asz + REDZ > ARENA_SZ) h_die3("arena exhausted");
  if (g_nal == g_capal) {
    g_capal = g_capal ? g_capal * 2 : 4096;
    h_al = (HAl *)realloc(h_al, g_capal * sizeof(HAl));
    if (!h_al) h_die3("table realloc");
  }
  memset(g_arena + g_arena_off, rz_byte(id), REDZ);
  p = g_arena + g_arena_off + REDZ;
  for (i = 0; i < asz; i++) p[i] = (uint8_t)(0x35u + id * 13u + (unsigned)i * 7u + ((unsigned)i >> 6));
  memset(p + asz, rz_byte(id), REDZ);
  g_arena_off += REDZ + asz + REDZ;
  a = &h_al[g_nal];
  a->p = p;
  a->size = size;
  a->id = id;
  a->freed = 0;
  h_last_alloc = (unsigned)g_nal++;
  h_live++;
  h_nmalloc++;
  h_emit(0, "M %u %u", id, log_size_of(size));
  h_log_next_size = 0;
  if (h_mp_obj) mp_report();
  return p;
}
HAl *h_find_al(const void *q, int allow_end) {
  size_t lo = 0, hi = g_nal;
  const uint8_t *p = (const uint8_t *)q;
  while (lo < hi) { /* last block with base <= p */
    size_t mid = (lo + hi) / 2;
    if (h_al[mid].p <= p) lo = mid + 1; else hi = mid;
  }
  if (lo == 0) return NULL;
  {
    HAl *a = &h_al[lo - 1];
    size_t off = (size_t)(p - a->p);
    if (off < a->size || (allow_end && off <= a->size)) return a;
  }
  return NULL;
}
void sysdep_free(void *q) {
  HAl *a = h_find_al(q, 1);
  if (!q) h_die3("sysdep_free(NULL)");
  if (!a || a->p != (uint8_t *)q) h_die3("sysdep_free of a pointer that is not a block base");
  if (a->freed) h_die3("double free of block %u", a->id);
  a->freed = 1;
  h_live--;
  h_nfree++;
  memset(a->p, 0xDD, ((size_t)a->size + 15u) & ~(size_t)15u);
  h_emit(0, "F %u", a->id);
}
/* dtor with a NULL block pointer: (sometimes) release the block by hand and clear the object's first pointer, then the destructor must
 * not call sysdep_free at all (the stub dies on sysdep_free(NULL)).  Returns 1 if it did. */
int h_maybe_null_first_ptr(void *o, int pct) {
  void *p, *z = NULL;
  if (!h_rpct(pct)) return 0;
  memcpy(&p, o, sizeof p);
  sysdep_free(p);
  memcpy(o, &z, sizeof z);
  h_cov_hit(H_COV_DTOR_NULL);
  return 1;
}
/* start of a new independent trial: forget all blocks (leaks are reported by the caller first).  When the arena straddles 2^31 the
 * first block of the trial starts `delta` bytes below 0x80000000 (delta = 0 every 5th trial, else spread over 0..96) */
static void heap_trial_reset(void) {
  g_nal = 0;
  h_live = 0;
  g_trial_no++;
  if (g_arena_straddles) {
    unsigned delta;
    h_cov_hit(H_COV_STRADDLE);
    delta = (g_trial_no % 5 == 0) ? 0u : (g_trial_no * 13u) % 97u;
    delta &= g_align_mask;
    g_arena_off = ARENA_BOUNDARY_OFF - delta - REDZ;
  } else {
    g_arena_off = 64;
  }
}
void h_scenario_begin(int elem4) {
  h_cov_define(H_COV_STRADDLE, "trials with the block around 2^31");
  h_cov_define(H_COV_DTOR_NULL, "dtor with a NULL block pointer");
  g_align_mask = elem4 == 2 ? ~1u : elem4 ? ~3u : ~0u;   /* elem4 == 2 (P42): 16-bit elements: only 2-byte alignment (the aligned-pointer arithmetic of V90Equalizer::C1 sees offsets 0 .. 3) */
  heap_trial_reset();
}
/* integrity of everything the allocator handed out: the blob never writes outside its blocks, never writes freed memory */
void h_check_redzones(void) {
  size_t i, k;
  for (i = 0; i < g_nal; i++) {
    const HAl *a = &h_al[i];
    size_t asz = ((size_t)a->size + 15u) & ~(size_t)15u;
    for (k = 0; k < REDZ; k++)
      if (a->p[asz + k] != rz_byte(a->id) || a->p[(size_t)0 - REDZ + k] != rz_byte(a->id)) h_die3("block %u: a byte outside the block was written (red zone, offset %zu)", a->id, k);
    if (a->freed) {
      for (k = 0; k < asz; k++) if (a->p[k] != 0xDD) h_die3("block %u: written after free", a->id);
    } else {
      for (k = a->size; k < asz; k++)
        if (a->p[k] != (uint8_t)(0x35u + a->id * 13u + (unsigned)k * 7u + ((unsigned)k >> 6))) h_die3("block %u: the alignment gap after the block was written", a->id);
    }
  }
}
/* object memory (NOT logged: the objects are what the harness owns, the heap blocks are what the classes allocate) */
static uint8_t g_pool[1u << 20] __attribute__((aligned(16)));
static size_t g_pool_off;
static unsigned int g_fill = 0x1234567u;
void *h_obj_new(size_t sz) {
  size_t i;
  uint8_t *p;
  g_pool_off = (g_pool_off + 15u) & ~(size_t)15u;
  if (g_pool_off + sz > sizeof g_pool) h_die3("object pool exhausted");
  p = g_pool + g_pool_off;
  for (i = 0; i < sz; i++) { /* random-filled: a ctor must define every field it owns (own generator: sizeof differs on x86-64) */
    g_fill = g_fill * 1664525u + 1013904223u;
    p[i] = (uint8_t)(g_fill >> 24);
  }
  g_pool_off += sz;
  return p;
}
void h_pool_reset(void) { g_pool_off = 0; }

/* pointer -> "A<id>+<byte offset>" / "NULL" / "WILD" (a pointer outside every block); raw addresses never appear */
void h_fmt_ptr(char *out, const void *p) {
  HAl *a;
  size_t lo, hi;
  const uint8_t *q = (const uint8_t *)p;
  if (!p) { strcpy(out, "NULL"); return; }
  a = h_find_al(p, 1);
  if (a) { sprintf(out, "A%u+%u%s", a->id, (unsigned)(q - a->p), a->freed ? "F" : ""); return; }
  /* inside a red zone (an over- / under-read of a block by a few elements, e.g. a tap beyond the history): "A<id>+off" with
   * off > size, or "A<id>-off" before the base; still architecture independent */
  lo = 0; hi = g_nal;
  while (lo < hi) { size_t mid = (lo + hi) / 2; if (h_al[mid].p <= q) lo = mid + 1; else hi = mid; }
  if (lo > 0 && (size_t)(q - h_al[lo - 1].p) <= (size_t)h_al[lo - 1].size + REDZ) {
    a = &h_al[lo - 1];
    sprintf(out, "A%u+%u", a->id, (unsigned)(q - a->p));
    return;
  }
  if (lo < g_nal && (size_t)(h_al[lo].p - q) <= REDZ) {
    a = &h_al[lo];
    sprintf(out, "A%u-%u", a->id, (unsigned)(a->p - q));
    return;
  }
  strcpy(out, "WILD");
  h_wild++;
}

/* ------------------------------------------------------------------------------------------------ vtable registry */
static struct { const uint8_t *vt; const char *name; } g_vt[16];
static int g_nvt;
void h_register_vtable(const void *vt, const char *name) {
  if (g_nvt >= 16) h_die3("vtable registry full");
  g_vt[g_nvt].vt = (const uint8_t *)vt;
  g_vt[g_nvt].name = name;
  g_nvt++;
}

/* --------------------------------------------------------------------------------------------- field-wise dump */
static size_t fld_unit(char kind) {
  switch (kind) {
    case 'b': return 1;
    case 'h': return 2;
    case 'u': case 'i': case 'f': return 4;
    case 'd': return 8;
    case 'p': case 'o': case 'e': case 'v': return sizeof(void *);
    default: return 1;
  }
}
static size_t fld_width(const HFld *f) {
  if (f->kind == 'x') return f->count;
  return fld_unit(f->kind) * (f->count > 1 ? f->count : 1);
}
/* one scalar element, printed like the P01 dump did for 'p' / 'u' */
static int fmt_elem(char *line, const HFld *f, const uint8_t *q, const uint8_t *objbase) {
  char pb[48];
  switch (f->kind) {
    case 'b': return sprintf(line, "%u", (unsigned)*q);
    case 'h': { uint16_t v; memcpy(&v, q, 2); return sprintf(line, "%u", (unsigned)v); }
    case 'u': { uint32_t v; memcpy(&v, q, 4); return sprintf(line, "%u", v); }
    case 'i': { int32_t v; memcpy(&v, q, 4); return sprintf(line, "%d", v); }
    case 'f': { uint32_t v; memcpy(&v, q, 4); return sprintf(line, "%08x", v); }
    case 'd': { u64 v; memcpy(&v, q, 8); return sprintf(line, "%016llx", v); }
    case 'p': case 'o': { void *p; memcpy(&p, q, sizeof p); h_fmt_ptr(pb, p); return sprintf(line, "%s", pb); }
    case 'e': {
      const uint8_t *p;
      memcpy(&p, q, sizeof p);
      if (!p) return sprintf(line, "NULL");
      if (p >= objbase && (size_t)(p - objbase) < 0x100000u) return sprintf(line, "E+%u", (unsigned)(p - objbase));
      return sprintf(line, "E?");
    }
    case 'v': {
      const uint8_t *p;
      int k;
      memcpy(&p, q, sizeof p);
      if (!p) return sprintf(line, "V:NULL");
      {
        int best = -1; /* the registered table with the largest base <= p (vtables of several classes can lie close together in memory) */
        for (k = 0; k < g_nvt; k++)
          if (p >= g_vt[k].vt && p < g_vt[k].vt + 64 && (best < 0 || g_vt[k].vt > g_vt[best].vt)) best = k;
        if (best >= 0) return sprintf(line, "V:%s+%u", g_vt[best].name, (unsigned)(p - g_vt[best].vt));
      }
      return sprintf(line, "V:?");
    }
    default: return sprintf(line, "?");
  }
}
void h_dump_obj(int show, const char *tag, const void *obj, const HFld *f, int nf) {
  char line[6000];
  int n = 0, i, nowners = 0, k;
  const uint8_t *o = (const uint8_t *)obj;
  h_check_redzones();
  for (i = 0; i < nf; i++) if (f[i].kind == 'o') nowners++;
  n += sprintf(line + n, "%s", tag);
  for (i = 0; i < nf; i++) {
    n += sprintf(line + n, " %s=", f[i].name);
    if (f[i].kind == 'x') {
      if (f[i].count <= 64) { for (k = 0; k < (int)f[i].count; k++) n += sprintf(line + n, "%02x", o[f[i].off + (size_t)k]); }
      else { u64 h = 1469598103934665603ULL; for (k = 0; k < (int)f[i].count; k++) { h ^= o[f[i].off + (size_t)k]; h *= 1099511628211ULL; } n += sprintf(line + n, "#%016llx", h); }
    } else if (f[i].count > 1) {
      n += sprintf(line + n, "[");
      for (k = 0; k < (int)f[i].count; k++) {
        if (k) line[n++] = ',';
        n += fmt_elem(line + n, &f[i], o + f[i].off + (size_t)k * fld_unit(f[i].kind), o);
        if (n > 5000) { n += sprintf(line + n, "..."); break; }
      }
      n += sprintf(line + n, "]");
    } else {
      n += fmt_elem(line + n, &f[i], o + f[i].off, o);
    }
  }
  for (i = 0; i < nf; i++) {
    const void *blk;
    HAl *a;
    if (f[i].kind != 'o') continue;
    memcpy(&blk, o + f[i].off, sizeof blk);
    if (!blk) continue;
    if (nowners > 1) n += sprintf(line + n, " heap.%s", f[i].name); else n += sprintf(line + n, " heap");
    a = h_find_al(blk, 1);
    if (a && a->p == (const uint8_t *)blk) {
      uint32_t kk;
      if (a->size <= 192) {
        n += sprintf(line + n, "(%u)=", a->size);
        for (kk = 0; kk < a->size; kk++) n += sprintf(line + n, "%02x", a->p[kk]);
      } else {
        u64 h = 1469598103934665603ULL;
        for (kk = 0; kk < a->size; kk++) { h ^= a->p[kk]; h *= 1099511628211ULL; }
        n += sprintf(line + n, "(%u)#%016llx", a->size, h);
      }
    } else {
      n += sprintf(line + n, "=?");
    }
  }
  h_emit(show, "%s", line);
}
static void mp_report(void) {
  const HFld *f = (const HFld *)h_mp_flds;
  char line[1200];
  int i, n = 0;
  for (i = 0; i < h_mp_nf; i++) {
    size_t w = fld_width(&f[i]);
    int changed = memcmp(h_mp_obj + f[i].off, h_mp_shadow + f[i].off, w) != 0;
    if (!changed) n += sprintf(line + n, " %s=unset", f[i].name);
    else if (f[i].kind == 'u' && f[i].count <= 1) { uint32_t v; memcpy(&v, h_mp_obj + f[i].off, 4); n += sprintf(line + n, " %s=%u", f[i].name, v); }
    else if (f[i].kind == 'p' || f[i].kind == 'o' || f[i].kind == 'e' || f[i].kind == 'v') { void *q; memcpy(&q, h_mp_obj + f[i].off, sizeof q); n += sprintf(line + n, " %s=%s", f[i].name, q ? "set" : "NULL"); }
    else n += sprintf(line + n, " %s=set", f[i].name);
    if (n > 1000) break;
  }
  h_emit(0, "MALLOC-TIME%s", line);
}
void h_check_not_wild(const char *what) {
  if (h_wild) h_die3("%s: a pointer field holds a value outside every heap block (%u)", what, h_wild);
}
void h_trial_end(const char *what) {
  h_check_redzones();
  if (h_live != 0) h_emit(1, "LEAK %s %u live blocks", what, h_live);
  heap_trial_reset();
  h_pool_reset();
}

void h_layout_compare(const char *cls, const unsigned *mirror, const unsigned *probe, unsigned n) {
  unsigned i;
  for (i = 0; i < n; i++)
    if (mirror[i] != probe[i]) h_die3("layout of %s: entry %u (0 = sizeof, then the member offsets in table order) is %u in the harness mirror, %u in the port class", cls, i, mirror[i], probe[i]);
}
