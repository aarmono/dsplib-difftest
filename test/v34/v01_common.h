/*
 * V01 copy of test/vpcm/p47_common.h (MAXB 64): the record log (callee records with a snapshot of the world), the coverage-site mechanism, the world-block table (impl blocks + the model's copies, compared byte-wise
 * after every call) and small helpers.  Included once per program; static everything.  Derived from test/vpcm/v90_phase4_demod.main.c (same recorder / model / snapshot design).
 */
#ifndef V01_COMMON_H
#define V01_COMMON_H

#include <limits.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../harness/harness.h"

#define L(m) __asm__(m)
extern unsigned int dsplibs_debug_level;

#ifndef V01_LOGMAX
#define V01_LOGMAX 768   /* V07 raises it: its loops produce hundreds of records per call */
#endif
enum { LOGMAX = V01_LOGMAX, TXTMAX = 640, NSC = 32, CAN = 64, MAXB = 64 };

/* ------------------------------------------------------------------------------------------------ the world: blocks (impl) and their copies (model) */
typedef struct { void *p; void *m; size_t n; const char *name; int own_model; } Blk;
static Blk g_blk[MAXB];
static int g_nblk;
static int blk_add(const char *name, void *impl, size_t n) {
  Blk *b = &g_blk[g_nblk];
  b->p = impl; b->n = n; b->name = name;
  b->m = malloc(n ? n : 1);
  b->own_model = 1;
  memcpy(b->m, impl, n);
  return g_nblk++;
}
static void blk_reset(void) { int i; for (i = 0; i < g_nblk; i++) free(g_blk[i].m); g_nblk = 0; }
static void blk_sync_model(void) { int i; for (i = 0; i < g_nblk; i++) memcpy(g_blk[i].m, g_blk[i].p, g_blk[i].n); }
static void blk_compare(const char *what) {
  int i;
  for (i = 0; i < g_nblk; i++) {
    if (memcmp(g_blk[i].m, g_blk[i].p, g_blk[i].n)) {
      const uint8_t *x = (const uint8_t *)g_blk[i].m, *y = (const uint8_t *)g_blk[i].p;
      size_t k;
      for (k = 0; k < g_blk[i].n; k++) if (x[k] != y[k]) break;
      h_die3("%s: block \"%s\" differs from the model at offset 0x%zx (model %02x, impl %02x)", what, g_blk[i].name, k, x[k], y[k]);
    }
  }
}

/* ------------------------------------------------------------------------------------------------ records */
typedef struct { char t[TXTMAX]; } Rec;
typedef struct { Rec r[LOGMAX]; unsigned n; unsigned long total; } Log;
typedef void (*SnapFn)(char *out, size_t cap, int model);
static SnapFn g_snapfn;

static uint32_t fnv_upd(uint32_t h, const void *p, size_t n) { const uint8_t *q = (const uint8_t *)p; while (n--) { h ^= *q++; h *= 16777619u; } return h; }
static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
static void rec_add(Log *lg, int model, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2], snap[TXTMAX * 2];
  va_list ap;
  Rec *r = &lg->r[lg->n < LOGMAX ? lg->n : LOGMAX - 1];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  g_snapfn(snap, sizeof snap, model);
  r->t[0] = kind; r->t[1] = ' '; r->t[2] = 0;
  esc_cat(r->t, sizeof r->t, buf);
  if (strlen(r->t) + strlen(snap) + 1 >= sizeof r->t) h_die3("record text does not fit (%zu + %zu)", strlen(r->t), strlen(snap));
  strcat(r->t, snap);
  lg->n++; lg->total++;
  if (lg->n > LOGMAX) h_die3("more than %d records in one call", LOGMAX);
}

/* ------------------------------------------------------------------------------------------------ coverage: per-site counters in their own section */
typedef struct { unsigned long n; unsigned char hit, grp; const char *name; } CovSite;
#define CV(g, nm) do { static CovSite cs_ __attribute__((section("covsites"), used, aligned(sizeof(void *)))) = { 0, 0, (g), (nm) }; cs_.hit = 1; } while (0)
#define CVD(g, nm) do { static CovSite cs_ __attribute__((section("covsites"), used, aligned(sizeof(void *)))) = { 0, 0, 0, (nm) }; cs_.grp = (unsigned char)(g); cs_.hit = 1; } while (0)
extern CovSite __start_covsites[], __stop_covsites[];
static void cov_flush(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) if (c->hit) { c->n++; c->hit = 0; } }
static void cov_clear(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) c->hit = 0; }

/* ------------------------------------------------------------------------------------------------ pointer tokens: known objects by id, heap blocks by allocation id, else WILD */
enum { ID_NULL = -1, ID_WILD = 99 };
static void *g_obj[MAXB];       /* id -> impl pointer (0 = unknown yet) */
static int g_nobj;
static const char *g_idname[MAXB];
static int tok_of(const void *p) {
  int k;
  HAl *a;
  if (!p) return ID_NULL;
  for (k = 0; k < g_nobj; k++) if (g_obj[k] == p) return k;
  a = h_find_al(p, 1);
  if (a) return 100 + (int)a->id * 4096 + (int)((const uint8_t *)p - a->p > 4095 ? 4095 : (const uint8_t *)p - a->p);
  return ID_WILD;
}
static const char *idn(int id) {
  static char b[8][24];
  static int n;
  char *o = b[n++ & 7];
  if (id == ID_NULL) return "null";
  if (id == ID_WILD) return "wild";
  if (id >= 0 && id < g_nobj) return g_idname[id];
  snprintf(o, 24, "A%d+%d", (id - 100) / 4096, (id - 100) % 4096);
  return o;
}

/* the canonical output line is limited to 4 KB: a field table longer than 48 members (the V.34 object, V04+) is dumped in slices D0, D1 ... */
static void dump_sliced(int show, const char *tag, const void *obj, const HFld *f, int nf) {
  int i, n;
  char t[16];
  if (nf <= 48) { h_dump_obj(show, tag, obj, f, nf); return; }
  for (i = 0; i < nf; i += 48) {
    n = nf - i < 48 ? nf - i : 48;
    snprintf(t, sizeof t, "%s%d", tag, i / 48);
    h_dump_obj(show, t, obj, f + i, n);
  }
}

static unsigned rnd_range(unsigned lo, unsigned hi) { return lo + h_rmod(hi - lo + 1); }
static uint32_t pick_word(void) { return h_rpct(50) ? h_rnd() : h_rpct(50) ? 0u : h_rpct(50) ? 0xffffffffu : 0x80000000u; }
static uint32_t pick_flag(void) { return h_rpct(45) ? 0u : h_rpct(70) ? 1u : h_rnd(); }
static unsigned pick_level(void) {
  static const unsigned lv[] = { 0, 1, 2, 3, 4, 0x80000000u, 0xffffffffu, 0x80000001u };
  return h_rpct(40) ? 2u + h_rmod(2) : lv[h_rmod(sizeof lv / sizeof lv[0])];
}
static void *opaque_new(size_t n) { return h_obj_new(n); }

static void log_cmp(Log *ml, Log *il, const char *what) {
  unsigned i, n = ml->n < LOGMAX ? ml->n : LOGMAX;
  if (ml->n != il->n) {
    for (i = 0; i < n && i < il->n && i < LOGMAX; i++) fprintf(stderr, "m: %s\ni: %s\n", ml->r[i].t, il->r[i].t);
    h_die3("%s: %u model records, %u impl records", what, ml->n, il->n);
  }
  for (i = 0; i < n; i++) if (strcmp(ml->r[i].t, il->r[i].t)) h_die3("%s: record %u differs:\n model: %s\n impl:  %s", what, i, ml->r[i].t, il->r[i].t);
}

#endif
