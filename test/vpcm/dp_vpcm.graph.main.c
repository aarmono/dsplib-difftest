/*
 * Z01 INTEGRATION test (32-bit only): the nine glue functions of src/vpcm/dp_vpcm.cpp inside the REAL object graph of dsplibs.o.  Two binaries are built from this ONE source:
 *   blob           test/vpcm/dp_vpcm.graph.main.c + the unmodified dsplibs.o
 *   port over blob test/vpcm/dp_vpcm.graph.main.c + a copy of dsplibs.o in which the six GLOBAL glue symbols (VPCMXF_Create / Delete / SessionTermination, dp_param_get, dp_vpcm_init / exit) are WEAK, + the port's dp_vpcm.o
 *                  (the strong definitions win at link time; the blob's LOCAL vpcm_create / vpcm_delete / vpcm_run stay in the object, unreferenced: dp_vpcm_init of the port registers the PORT's vpcm_op)
 * Nothing else is interposed: VPcmFloModem's constructor / destructors, VPcmV34Create / Progress and the whole V.34 / V.90 / V.92 / K56Flex machinery are the blob's own.  The environment (the slmodemd side:
 * modem_get_param / set_param / get_bits / put_bits / dp_register, sysdep_*, dsplibs_debug_printf) is a deterministic fake.  Two sessions (an answerer and a caller of one DP id, 0x22 / 0x5a / 0x5c) are wired
 * back to back through vpcm_run (each one's output samples are the other's input of the next call) and driven for thousands of calls; every call's results (return value, output samples, bit traffic, the
 * state words and the dp parameter block) are hashed into the canonical output, the malloc log shows every allocation / free: the two binaries must print identical output.
 * Usage: z01_graph_test <variant 0..5>   (variants: DP id x which side starts; own process each)
 */
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "../harness/harness.h"

#define L(m) __asm__(m)
extern unsigned int dsplibs_debug_level;
extern int f_init(void) L("dp_vpcm_init");
extern void f_exit(void) L("dp_vpcm_exit");
void *h_sysdep_malloc(unsigned size);
void h_sysdep_free(void *q);
void *sysdep_malloc(unsigned size) { return h_sysdep_malloc(size); }
void sysdep_free(void *q) { h_sysdep_free(q); }
void *sysdep_memset(void *d, int c, size_t n) { return memset(d, c, n); }
void *sysdep_memcpy(void *d, const void *s, size_t n) { return memmove(d, s, n); }

int sysdep_vsnprintf(char *b, size_t n, const char *fmt, va_list ap) { return vsnprintf(b, n, fmt, ap); }
int sysdep_sprintf(char *b, const char *fmt, ...) { int r; va_list ap; va_start(ap, fmt); r = vsprintf(b, fmt, ap); va_end(ap); return r; }
char *sysdep_strcat(char *d, const char *s) { return strcat(d, s); }
char *sysdep_strcpy(char *d, const char *s) { return strcpy(d, s); }
size_t sysdep_strlen(const char *s) { return strlen(s); }

typedef struct { const char *name; int use; void *create, *del, *process, *hangup; } Ops;
typedef void *(*CreateFn)(void *, int, int, int, int, void *);
typedef int (*DeleteFn)(void *);
typedef int (*RunFn)(void *, short *, short *, int);

typedef struct {
  const char *name;
  uint32_t rng;
  uint8_t *dp;       /* the modem parameter block (0x400 bytes, zero, a few fields set) */
  int32_t pair[4];
  int32_t minRate, maxRate, hwDelay;
  unsigned long nget, nput, nset, nlog;
  uint32_t hput;
  int shown;
} Fake;
static Fake g_f[2];
static Ops *g_ops;
static int g_show;

static uint32_t frng(Fake *f) { f->rng ^= f->rng << 13; f->rng ^= f->rng >> 17; f->rng ^= f->rng << 5; return f->rng; }

long modem_get_param(void *m, unsigned idx) {
  Fake *f = (Fake *)m;
  switch (idx) {
    case 0xa: return (long)(intptr_t)f->dp;
    case 0xb: return (long)(intptr_t)f->pair;
    case 3: return f->minRate;
    case 4: return f->maxRate;
    case 5: return f->hwDelay;
    default: return 0;
  }
}
long modem_set_param(void *m, unsigned name, int val) {
  Fake *f = (Fake *)m;
  f->nset++;
  h_emit(0, "set_param %s %u %d", f->name, name, val);
  if (name == 0xd) f->hwDelay = val;
  return 0;
}
int modem_get_bits(void *m, int nbits, uint8_t *buf, int n) {
  Fake *f = (Fake *)m;
  int i;
  f->nget++;
  (void)nbits;
  for (i = 0; i < n; i++) buf[i] = (uint8_t)(frng(f) >> 7) & 1u;
  h_emit(0, "get_bits %s n=%d", f->name, n);
  return n;
}
int modem_put_bits(void *m, int nbits, uint8_t *buf, int n) {
  Fake *f = (Fake *)m;
  int i;
  f->nput++;
  (void)nbits;
  for (i = 0; i < n; i++) f->hput = (f->hput ^ buf[i]) * 16777619u;
  h_emit(0, "put_bits %s n=%d", f->name, n);
  return n;
}
int modem_debug_log_data(void *m, unsigned id, const void *d, int cnt) { Fake *f = (Fake *)m; f->nlog++; h_emit(0, "log_data %s %u %d", f->name, id, cnt); (void)d; return 0; }
int modem_dp_register(int id, void *op) { if (!g_ops) g_ops = (Ops *)op; h_emit(1, "dp_register %d %s", id, ((Ops *)op)->name); return 0; }
void modem_dp_deregister(int id, void *op) { h_emit(1, "dp_deregister %d %s", id, ((Ops *)op)->name); }
int dsplibs_debug_printf(const char *fmt, ...) {
  char b[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  h_emit(0, "dbg %s", b);
  return 0;
}

static void init_fake(Fake *f, const char *name, uint32_t seed) {
  memset(f, 0, sizeof *f);
  f->name = name;
  f->rng = seed | 1u;
  f->dp = (uint8_t *)h_obj_new(0x400);
  memset(f->dp, 0, 0x400);
  *(int32_t *)(f->dp + 0x70) = -1;              /* entrance filter: according to the hardware */
  *(int32_t *)(f->dp + 0x54) = 0xe;
  f->minRate = 2400; f->maxRate = 33600; f->hwDelay = 100;
}

enum { NCALLS = 6000, FRAG = 40 };

int main(int argc, char **argv) {
  int variant = argc > 1 ? atoi(argv[1]) : 0, k, call;
  static const int dpid[6] = { 0x22, 0x22, 0x5a, 0x5a, 0x5c, 0x5c };
  CreateFn cr;
  DeleteFn de;
  RunFn rn;
  void *w[2];
  short *in[2], *out[2], *prev[2];
  uint32_t hout[2] = { 2166136261u, 2166136261u };
  int rc = 0, lst[2] = { 0, 0 }, llk[2] = { 0, 0 }, lid[2] = { 0, 0 };
  if (variant < 0 || variant > 5) { fprintf(stderr, "unknown variant %d\n", variant); return 2; }
  if (getenv("Z01G_SEED")) h_seed_xor((unsigned)strtoul(getenv("Z01G_SEED"), NULL, 0));
  h_x87_check();
  h_arena_init();
  h_scenario_begin(0);
  dsplibs_debug_level = getenv("Z01G_LEVEL") ? (unsigned)atoi(getenv("Z01G_LEVEL")) : 2u;
  init_fake(&g_f[0], "A", 0x1234567u + (uint32_t)variant); init_fake(&g_f[1], "B", 0x7654321u + (uint32_t)variant);
  if (variant & 2) { g_f[0].hwDelay = 300; g_f[1].hwDelay = 400; }   /* the hardware delay exceeds the budget: vpcm_create adds delay (0x180+), phase II completed / restart change it */
  f_init();
  if (!g_ops) { fprintf(stderr, "no registration\n"); return 3; }
  h_emit(1, "ops %s use=%d hangup=%d", g_ops->name, g_ops->use, g_ops->hangup != NULL);
  cr = (CreateFn)g_ops->create; de = (DeleteFn)g_ops->del; rn = (RunFn)g_ops->process;
  for (k = 0; k < 2; k++) {
    /* side 0 = the answerer (caller 0), side 1 = the caller; variant & 1 swaps them */
    int caller = (k ^ (variant & 1)) ? 1 : 0;
    w[k] = cr(&g_f[k], dpid[variant], caller, 0x2580, FRAG, g_ops);
    h_emit(1, "create %s dp=0x%x caller=%d -> %s", g_f[k].name, dpid[variant], caller, w[k] ? "ok" : "NULL");
    if (!w[k]) return 3;
    in[k] = (short *)h_obj_new(2 * 64); out[k] = (short *)h_obj_new(2 * 64); prev[k] = (short *)h_obj_new(2 * 64);
    memset(in[k], 0, 128); memset(out[k], 0, 128); memset(prev[k], 0, 128);
  }
  for (call = 0; call < NCALLS; call++) {
    int n = FRAG, r[2];
    for (k = 0; k < 2; k++) {
      /* the other side's output of the previous call is this side's input (a digital loop-back with one call of delay) */
      memcpy(in[k], prev[1 - k], 2 * (size_t)n);
      if (call % 701 == 700) { int i; for (i = 0; i < n; i++) in[k][i] = (short)(frng(&g_f[k]) >> 12); }   /* a burst of noise now and then */
    }
    for (k = 0; k < 2; k++) {
      uint32_t h = 2166136261u;
      int i;
      r[k] = rn(w[k], in[k], out[k], n);
      for (i = 0; i < n; i++) { h = (h ^ (uint16_t)out[k][i]) * 16777619u; hout[k] = (hout[k] ^ (uint16_t)out[k][i]) * 16777619u; }
      h_emit(call % 250 == 0, "call %d %s ret=%d out=%08x state: id=%d status=%d link=%d bits=%d train=%d tx=%d pend=%d silence=%d add=%d", call, g_f[k].name, r[k], h,
             *(int *)((uint8_t *)w[k] + 0), *(int *)((uint8_t *)w[k] + 0x14), *(int *)((uint8_t *)w[k] + 0x18), *(int *)((uint8_t *)w[k] + 0x1c), *(int *)((uint8_t *)w[k] + 0x20),
             *(int *)((uint8_t *)w[k] + 0xd178), *(int *)((uint8_t *)w[k] + 0xd1e4), *(int *)((uint8_t *)w[k] + 0xd250), *(int *)((uint8_t *)w[k] + 0xd254));
    }
    for (k = 0; k < 2; k++) {
      int st = *(int *)((uint8_t *)w[k] + 0x14), ln = *(int *)((uint8_t *)w[k] + 0x18), id = *(int *)((uint8_t *)w[k] + 0);
      if (st != lst[k] || ln != llk[k] || id != lid[k] || r[k] != 0) { h_emit(1, "change at call %d %s: status %d -> %d, link %d -> %d, id %d -> %d, ret %d", call, g_f[k].name, lst[k], st, llk[k], ln, lid[k], id, r[k]); lst[k] = st; llk[k] = ln; lid[k] = id; }
      memcpy(prev[k], out[k], 2 * (size_t)n);
    }
  }
  for (k = 0; k < 2; k++) {
    int rr = de(w[k]);
    h_emit(1, "final %s outhash=%08x get=%lu put=%lu set=%lu log=%lu hput=%08x pair=%d,%d delete=%d", g_f[k].name, hout[k], g_f[k].nget, g_f[k].nput, g_f[k].nset, g_f[k].nlog, g_f[k].hput, g_f[k].pair[0],
           g_f[k].pair[1], rr);
  }
  f_exit();
  h_emit(1, "calls: malloc %lu free %lu", h_nmalloc, h_nfree);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  h_cov_finish();
  return rc;
}
