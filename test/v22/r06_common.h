/*
 * R06 differential harness library (header-only, a derivative of test/r02_common.h: included by the mains of test/v22/): the pieces the three harnesses share.
 * Differences to r02_common.h: fresh heap blocks hold garbage that depends on the block id ONLY (not on the byte offset: the V.22 engine reads never-initialised words,
 * e.g. the SMC word of the FSE, whose offset differs between i386 and x86-64), the M line prints the size only for the blocks whose size is the same on every host
 * (the typed engine objects print their class, the sizes that collide with an FPM object print "?"), and the heap log can be switched off.
 *
 *  - canonical output: every emitted line is hashed (FNV-1a 64) and only selected lines are printed, then "HASH <h> lines <n>" (R02_VERBOSE=1 prints every line);
 *    the printed text is architecture independent, so the 32-bit BLOB run is the golden of the x86-64 build (test/golden/<id>_blob32.out.gz).
 *  - required coverage counters (a zero exits 2), harness / model violations exit 3.
 *  - a deterministic heap with red zones / poisoned frees / id-patterned fresh blocks, the callee recorders sysdep_malloc / free / memset / memcpy (sizes mapped to the
 *    i386 sizes, pointers described by block id + field, never printed raw), dsplibs_debug_printf (the formatted message), and a fake slmodemd `struct modem`:
 *    modem_get_param / set_param / get_bits / put_bits / modem_dp_register / deregister recorders with scripted results.
 *  - the fake RcFixed (create / delete / resample recorders) and `dp_param_get`.
 * Nothing here is under test.  The code under test (blob or port) calls these through its normal external symbols.
 */
#ifndef R06_COMMON_H
#define R06_COMMON_H
#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "v22/dp_v22.h"
#include "v22/v22_tables.h"

typedef unsigned long long u64;

/* ------------------------------------------------------------------------------------------------ output / hash / PRNG / coverage */
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
static u64 fnv_shorts(const short *p, size_t n)
{
  u64 h = 1469598103934665603ULL;
  size_t i;
  for (i = 0; i < n; i++) {
    h ^= (unsigned short)p[i] & 0xffu;
    h *= 1099511628211ULL;
    h ^= ((unsigned short)p[i] >> 8) & 0xffu;
    h *= 1099511628211ULL;
  }
  return h;
}
static u64 fnv_ints(const int *p, size_t n)
{
  u64 h = 1469598103934665603ULL;
  size_t i;
  for (i = 0; i < n; i++) {
    unsigned v = (unsigned)p[i];
    int k;
    for (k = 0; k < 4; k++) {
      h ^= (v >> (8 * k)) & 0xffu;
      h *= 1099511628211ULL;
    }
  }
  return h;
}
static void emit(int show, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
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

#define COV_MAX 160
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
    emit(1, "COV %-44s %lu", g_covname[i], g_cov[i]);
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
static unsigned long g_nmalloc, g_nfree, g_nmemset, g_nmemcpy;
static int g_fail_malloc = -1; /* the n-th (0 based, counted from the last heap_reset) sysdep_malloc returns NULL; -1 = never */
static unsigned g_malloc_ctr;
static char g_szbuf[16];

static void arena_init(void)
{
  /* as the shared harness: map the arena so that 0x80000000 lies inside it (signed pointer compares); static buffer if refused */
  void *want = (void *)(uintptr_t)(0x80000000u - 0x200000u);
  void *m = mmap(want, ARENA_SZ, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | 0x100000 /* MAP_FIXED_NOREPLACE */, -1, 0);
  if (m == want) g_arena = (unsigned char *)m;
  else if (m != MAP_FAILED) munmap(m, ARENA_SZ);
  g_off = 64;
}
/* the i386 size of every block type this suite allocates */
static unsigned log_size(unsigned size)
{
  /* sizes that are ambiguous between hosts: the i386 and the x86-64 sizes of the FPM_MTD object (0x18 / 0x20), of RcFixed / RdObj (8 / 16), RcDec (0x28 / 80) and RcPoly (420 / 424) also occur as plain
   * buffer sizes (the circular buffer 0x18, the IIR object 0x20, the FSE buffers 0x28, kstate 8, ...): printed as "?" on both hosts */
  switch (size) {
    case 8: case 16: case 0x18: case 0x20: case 0x28: case 80: case 420: case 424: return 0;
    default: break;
  }
  if (size == sizeof(V22Engine)) return 0x5c;
  if (size == sizeof(V22Core)) return 0x1f8;
  if (size == sizeof(V22Tones)) return 0x40;
  if (size == sizeof(V22Glue)) return 0x344;
  if (size == sizeof(FpmTone)) return 0x108;
  if (size == sizeof(DpWrapper)) return 0x938;
  if (size == sizeof(B103Glue)) return 0x348;
  if (size == sizeof(V8Glue)) return 0x34;
  if (size == sizeof(RdObj) || size == sizeof(RcFixed)) return 8;
  if (size == sizeof(RcPoly)) return 0x1a4;
  if (size == sizeof(RcDec)) return 0x28;
  return size; /* DpRuntime 0x88, DcrObj 0x20: the same on every host */
}
/* R03 (r03_e2e): with R02_HEAP_LOG_FILTER only the glue / wrapper blocks are logged (a real V.32 engine behind the glue allocates blocks whose x86-64 sizes differ) */
static int g_heap_log = 1;
static int blk_logged(unsigned size)
{
  (void)size;
  return g_heap_log;
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
    if (blk_logged(size)) emit(0, "M %u %s NULL", id, log_size(size) ? (snprintf(g_szbuf, sizeof g_szbuf, "%u", log_size(size)), g_szbuf) : "?");
    return NULL;
  }
  if (g_off + REDZ + asz + REDZ > ARENA_SZ || g_nblk >= MAXBLK) die3("arena exhausted");
  memset(g_arena + g_off, 0xCC, REDZ);
  p = g_arena + g_off + REDZ;
  for (i = 0; i < asz; i++) p[i] = (unsigned char)(0x35u + id * 13u); /* fresh blocks hold deterministic garbage that depends on the block id only */
  memset(p + asz, 0xCC, REDZ);
  g_off += REDZ + asz + REDZ;
  g_blk[g_nblk].p = p;
  g_blk[g_nblk].size = size;
  g_blk[g_nblk].id = id;
  g_blk[g_nblk].freed = 0;
  g_nblk++;
  g_live++;
  g_nmalloc++;
  if (blk_logged(size)) {
    if (log_size(size)) emit(0, "M %u %u", id, log_size(size));
    else emit(0, "M %u ?", id);
  }
  return p;
}
void sysdep_free(void *q)
{
  Blk *b = find_blk(q);
  if (!q) die3("sysdep_free(NULL)");
  if (!b || b->p != (unsigned char *)q) die3("sysdep_free of a pointer that is not a block base");
  if (b->freed) die3("double free of block %u", b->id);
  { /* an overrun into the gap behind the block (size .. size rounded up to 16) is checked here: the poisoning below would erase the evidence */
    size_t asz = ((size_t)b->size + 15u) & ~(size_t)15u, k;
    for (k = b->size; k < asz; k++)
      if (b->p[k] != (unsigned char)(0x35u + b->id * 13u)) die3("block %u: the gap behind it was written (at free)", b->id);
  }
  /* the state a block is freed in is observable here (a field the code clears just before freeing is then not "dead"): the wrapper's owner, a glue block's dp_data */
  if (!blk_logged(b->size)) {}
  else if (b->size == sizeof(DpWrapper)) emit(0, "F %u wrapper owner=%d", b->id, ((const DpWrapper *)(const void *)b->p)->owner != NULL);
  else if (b->size == sizeof(V22Glue)) emit(0, "F %u glue dp_data=%d", b->id, ((const struct dp *)(const void *)b->p)->dp_data != NULL);
  b->freed = 1;
  g_live--;
  g_nfree++;
  memset(b->p, 0xDD, ((size_t)b->size + 15u) & ~(size_t)15u);
  if (blk_logged(b->size)) emit(0, "F %u", b->id);
}
/* a block id of an address, 0 when it is not inside a live harness-heap block */
static unsigned blk_id_of(const void *p)
{
  Blk *b = find_blk(p);
  return b ? b->id : 0;
}
static const Blk *blk_of(const void *p) { return find_blk(p); }

/* named caller buffers: describe() prints "name+index" for a pointer into one of them */
typedef struct {
  const void *base;
  size_t bytes;
  size_t esz;
  const char *name;
} Region;
static Region g_reg[16];
static int g_nreg;
static void reg_clear(void) { g_nreg = 0; }
static void reg_add(const void *base, size_t bytes, size_t esz, const char *name)
{
  if (g_nreg >= 16) die3("regions");
  g_reg[g_nreg].base = base;
  g_reg[g_nreg].bytes = bytes;
  g_reg[g_nreg].esz = esz;
  g_reg[g_nreg].name = name;
  g_nreg++;
}
/* describes a pointer layout-independently: a registered buffer, a field of a harness-heap glue block, or "?" (raw addresses are never printed) */
static void describe(const void *p, char *out, size_t cap)
{
  int i;
  const Blk *b;
  if (!p) {
    snprintf(out, cap, "NULL");
    return;
  }
  for (i = 0; i < g_nreg; i++) {
    const unsigned char *base = (const unsigned char *)g_reg[i].base;
    if ((const unsigned char *)p >= base && (const unsigned char *)p < base + g_reg[i].bytes) {
      snprintf(out, cap, "%s+%zu", g_reg[i].name, (size_t)((const unsigned char *)p - base) / g_reg[i].esz);
      return;
    }
  }
  b = find_blk(p);
  if (b) {
    const unsigned char *q = (const unsigned char *)p;
    const unsigned char *base = b->p;
    if (b->size == sizeof(DpWrapper)) {
      const DpWrapper *w = (const DpWrapper *)base;
      if (q >= (const unsigned char *)w->in.data && q < (const unsigned char *)w->in.data + sizeof w->in.data) snprintf(out, cap, "W%u.in+%zu", b->id, (size_t)(q - (const unsigned char *)w->in.data) / 2);
      else if (q >= (const unsigned char *)w->out.data && q < (const unsigned char *)w->out.data + sizeof w->out.data) snprintf(out, cap, "W%u.out+%zu", b->id, (size_t)(q - (const unsigned char *)w->out.data) / 2);
      else if (q >= (const unsigned char *)w->inbuf && q < (const unsigned char *)w->inbuf + sizeof w->inbuf) snprintf(out, cap, "W%u.inbuf+%zu", b->id, (size_t)(q - (const unsigned char *)w->inbuf) / 2);
      else if (q >= (const unsigned char *)w->outbuf && q < (const unsigned char *)w->outbuf + sizeof w->outbuf) snprintf(out, cap, "W%u.outbuf+%zu", b->id, (size_t)(q - (const unsigned char *)w->outbuf) / 2);
      else snprintf(out, cap, "W%u.?", b->id);
      return;
    }
    if (b->size == sizeof(V22Glue)) {
      const V22Glue *g = (const V22Glue *)base;
      if (q >= (const unsigned char *)g->txbits && q < (const unsigned char *)g->txbits + sizeof g->txbits) snprintf(out, cap, "G%u.tx+%zu", b->id, (size_t)(q - (const unsigned char *)g->txbits));
      else if (q >= (const unsigned char *)g->rxbits && q < (const unsigned char *)g->rxbits + sizeof g->rxbits) snprintf(out, cap, "G%u.rx+%zu", b->id, (size_t)(q - (const unsigned char *)g->rxbits));
      else snprintf(out, cap, "G%u.?", b->id);
      return;
    }
    if (b->size == sizeof(B103Glue)) { /* B103Glue and V23Glue have the same layout (asserted) */
      const B103Glue *g = (const B103Glue *)base;
      if (q >= (const unsigned char *)g->txbits && q < (const unsigned char *)g->txbits + sizeof g->txbits) snprintf(out, cap, "G%u.tx+%zu", b->id, (size_t)(q - (const unsigned char *)g->txbits));
      else if (q >= (const unsigned char *)g->rxbits && q < (const unsigned char *)g->rxbits + sizeof g->rxbits) snprintf(out, cap, "G%u.rx+%zu", b->id, (size_t)(q - (const unsigned char *)g->rxbits));
      else snprintf(out, cap, "G%u.?", b->id);
      return;
    }
    snprintf(out, cap, "B%u+%zu", b->id, (size_t)(q - base));
    return;
  }
  snprintf(out, cap, "?");
}
void *sysdep_memset(void *dst, int c, size_t len)
{
  Blk *b = find_blk(dst);
  if (b) {
    size_t off = (size_t)((unsigned char *)dst - b->p);
    if (b->size == sizeof(RcPoly)) off -= offsetof(RcPoly, hist); /* the history array is at +4 on i386 and +8 on x86-64 */
    if (b->size == sizeof(DpWrapper) && off >= offsetof(DpWrapper, inbuf)) off -= offsetof(DpWrapper, inbuf) - 0x14; /* R03: a memset of the wrapper's staging buffers (inbuf is at +0x14 on i386) */
    if (b->freed) die3("memset of a freed block");
    g_nmemset++;
    if (blk_logged(b->size)) emit(0, "S %u %u %d %u", b->id, (unsigned)off, c, (unsigned)(b->size == len ? log_size((unsigned)len) : (unsigned)len));
  } else {
    /* a stack object (the V.8 config block): length mapped to the i386 size */
    unsigned l = (unsigned)len;
    if (len == sizeof(V8Cfg)) l = 0x18;
    g_nmemset++;
    emit(0, "S stack %d %u", c, l);
  }
  return memset(dst, c, len);
}
void *sysdep_memcpy(void *dst, const void *src, size_t len)
{
  char d[96], s[96];
  describe(dst, d, sizeof d);
  describe(src, s, sizeof s);
  g_nmemcpy++;
  emit(0, "C %s <- %s n=%u h=%016llx", d, s, (unsigned)len, fnv_bytes(src, len));
  return memcpy(dst, src, len);
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
        if (b->p[k] != (unsigned char)(0x35u + b->id * 13u)) die3("block %u: the gap behind it was written (%s)", b->id, what);
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
/* debug levels the scenarios use: the code compares `1 < dsplibs_debug_level` UNSIGNED, so 0x80000000 / 0xffffffff must print (a signed compare would not) */
static unsigned lvl_value(int k)
{
  static const unsigned v[5] = {0u, 1u, 2u, 0x80000000u, 0xffffffffu};
  return v[(unsigned)k % 5u];
}
static void (*g_dbg_hook)(const char *msg); /* R07: a harness may count the messages */
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

/* ------------------------------------------------------------------------------------------------ the fake slmodemd modem */
typedef struct FakeModem {
  int id;
  int dummy[3];
} FakeModem;
static FakeModem g_modem[3] = {{0, {0}}, {1, {0}}, {2, {0}}};
static int modem_no(const struct modem *m)
{
  int i;
  for (i = 0; i < 3; i++)
    if ((const void *)m == (const void *)&g_modem[i]) return i;
  return -1;
}
#define M_(i) ((struct modem *)&g_modem[i])
#define PRM_MAX 32
static long g_param[PRM_MAX];              /* scripted modem_get_param results (pointer parameters are patched by the harness) */
static int g_param_ptr[PRM_MAX];           /* 1 = the value is an address: printed as "ptr" */
static struct dsp_info g_dsp_info;
static unsigned long g_ngp, g_nsp, g_ngb, g_npb;
static long g_set_log[8][3];
long modem_get_param(struct modem *m, unsigned name)
{
  long v = name < PRM_MAX ? g_param[name] : -1;
  g_ngp++;
  if (name < PRM_MAX && g_param_ptr[name]) emit(0, "GP m%d %u -> ptr", modem_no(m), name);
  else emit(0, "GP m%d %u -> %ld", modem_no(m), name, v);
  return v;
}
long modem_set_param(struct modem *m, unsigned name, int val)
{
  g_nsp++;
  emit(0, "SP m%d %u %d", modem_no(m), name, val);
  return 0;
}
/* scripted bit supply: get_bits fills `bytes` of the buffer from the LCG and returns min(n, g_bits_give) (or g_bits_give raw when g_bits_raw) */
static int g_bits_give = 6, g_bits_raw;
static int g_bits_log_hash = 1;
int modem_get_bits(struct modem *m, int nbits, unsigned char *buf, int n)
{
  int r = g_bits_raw ? g_bits_give : (n < g_bits_give ? n : g_bits_give), i;
  char d[96];
  g_ngb++;
  describe(buf, d, sizeof d);
  for (i = 0; i < r && i < 100; i++) buf[i] = (unsigned char)rnd();
  emit(0, "GB m%d bits=%d buf=%s n=%d -> %d h=%016llx", modem_no(m), nbits, d, n, r, fnv_bytes(buf, (size_t)(r > 0 && r < 100 ? r : (r >= 100 ? 100 : 0))));
  return r;
}
int modem_put_bits(struct modem *m, int nbits, unsigned char *buf, int n)
{
  char d[96];
  g_npb++;
  describe(buf, d, sizeof d);
  emit(0, "PB m%d bits=%d buf=%s n=%d h=%016llx", modem_no(m), nbits, d, n, fnv_bytes(buf, (size_t)(n > 0 && n < 400 ? n : 0)));
  return n;
}
/* registration recorder: the record is described by its name, use_count and which slots are set (a function address is never printed; the process slot is named by
 * the harness' `g_op_slot_name`); the record itself is captured so that the harness can call create / delete / process through it, as slmodemd does */
static const char *(*g_op_slot_name)(const void *fn);
static struct dp_operations *g_reg_op[8];
static int g_reg_id[8], g_nreg_op;
int modem_dp_register(enum DP_ID id, struct dp_operations *op)
{
  emit(0, "REG %d name=%s use=%d create=%d delete=%d process=%s hangup=%d", (int)id, op->name, op->use_count, op->create != 0, op->delete != 0,
       g_op_slot_name ? g_op_slot_name((const void *)op->process) : "?", op->hangup != 0);
  if (g_nreg_op < 8) {
    g_reg_op[g_nreg_op] = op;
    g_reg_id[g_nreg_op] = (int)id;
    g_nreg_op++;
  }
  return 0;
}
void modem_dp_deregister(enum DP_ID id, struct dp_operations *op)
{
  int i, same = 0;
  for (i = 0; i < g_nreg_op; i++)
    if (g_reg_op[i] == op) same = 1;
  emit(0, "DEREG %d name=%s same=%d", (int)id, op->name, same);
}

/* the dp parameter block recorder (dp_param_get = modem_get_param(m, 0xa)) */
static DpRuntime g_rt_block;
long dp_param_get(void *m)
{
  emit(0, "DPG m%d", modem_no((struct modem *)m));
  return (long)(intptr_t)&g_rt_block;
}

#ifdef R02_REAL_RC
/* r02_wrapper_rc: the REAL RcFixed (the blob's own on the blob side, src/core/rcfixed_resampler.c on the port side) behind the wrapper: no fake, no failure injection.  rc_idx maps the
 * converter to the Create index the wrapper chose from the polyphase (up, down) of the object */
static unsigned g_rc_fail_mask;
static void rc_reset(void) {}
static int rc_idx(const RcFixed *h)
{
  int up, down;
  if (!h) return -1;
  if (h->type != 0 || !h->state.poly) return 100 + h->type;
  up = h->state.poly->up;
  down = h->state.poly->down;
  if (up == 6 && down == 5) return 2;
  if (up == 5 && down == 6) return 3;
  if (up == 6 && down == 1) return 4;
  if (up == 1 && down == 6) return 5;
  if (up == 5 && down == 1) return 6;
  if (up == 1 && down == 5) return 7;
  return 50 + up * 10 + down;
}
#else
/* ------------------------------------------------------------------------------------------------ the fake RcFixed (a deterministic stand-in: the glue only needs a handle) */
typedef struct FakeRc {
  int idx;
  int live;
  unsigned calls;
  int pad[3];
} FakeRc;
static FakeRc g_rcpool[16];
static int g_nrc;
static unsigned g_rc_fail_mask; /* bit k: the k-th RcFixed_Create since reset returns NULL */
static unsigned g_rc_create_ctr;
static unsigned long g_rc_resample;
static unsigned g_rc_out_div = 0; /* resample output count: nIn * up / down by index (below) */
static int rc_id(const RcFixed *h) { return (int)(((const FakeRc *)(const void *)h) - g_rcpool); }
RcFixed *RcFixed_Create(unsigned int idx)
{
  FakeRc *r;
  if (g_rc_fail_mask & (1u << g_rc_create_ctr++)) {
    emit(0, "RC create %u -> NULL", idx);
    return NULL;
  }
  if (g_nrc >= 16) die3("rc pool");
  r = &g_rcpool[g_nrc++];
  r->idx = (int)idx;
  r->live = 1;
  r->calls = 0;
  emit(0, "RC create %u -> r%d", idx, rc_id((RcFixed *)(void *)r));
  return (RcFixed *)(void *)r;
}
void RcFixed_Delete(RcFixed *o)
{
  FakeRc *r = (FakeRc *)(void *)o;
  if (!r->live) die3("RcFixed_Delete twice");
  r->live = 0;
  emit(0, "RC delete r%d idx=%d", rc_id(o), r->idx);
}
/* up / down by Create index 2..7 (the converter ratios the wrapper uses): 8000->9600 6/5, 9600->8000 5/6, 8000->48000 6, 48000->8000 1/6, 9600->48000 5, 48000->9600 1/5 */
static void rc_ratio(int idx, unsigned *up, unsigned *down)
{
  switch (idx) {
    case 2: *up = 6; *down = 5; break;
    case 3: *up = 5; *down = 6; break;
    case 4: *up = 6; *down = 1; break;
    case 5: *up = 1; *down = 6; break;
    case 6: *up = 5; *down = 1; break;
    default: *up = 1; *down = 5; break;
  }
}
void RcFixed_Resample(RcFixed *o, const short *in, unsigned int nIn, short *out, unsigned int *pnOut)
{
  FakeRc *r = (FakeRc *)(void *)o;
  unsigned up, down, n, cap = *pnOut, j;
  char di[96], dout[96];
  rc_ratio(r->idx, &up, &down);
  n = (unsigned)(((unsigned long long)nIn * up) / down);
  if (cap != 0 && n > cap) n = cap;
  describe(in, di, sizeof di);
  describe(out, dout, sizeof dout);
  for (j = 0; j < n; j++) {
    unsigned k = nIn ? (unsigned)(((unsigned long long)j * down) / up) : 0;
    out[j] = (short)((nIn ? in[k < nIn ? k : nIn - 1] : 0) + (short)(r->calls * 3u + j));
  }
  g_rc_resample++;
  emit(0, "RC resample r%d idx=%d in=%s n=%u h=%016llx cap=%u out=%s -> %u h=%016llx", rc_id(o), r->idx, di, nIn, fnv_shorts(in, nIn), cap, dout, n, fnv_shorts(out, n));
  r->calls++;
  *pnOut = n;
}
static int rc_idx(const RcFixed *h) { return h ? ((const FakeRc *)(const void *)h)->idx : -1; }
static void rc_reset(void)
{
  g_nrc = 0;
  g_rc_create_ctr = 0;
  g_rc_fail_mask = 0;
  memset(g_rcpool, 0, sizeof g_rcpool);
}

#endif

#endif
