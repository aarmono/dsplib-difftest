/*
 * R10 harness library (header only, used by test/callprog/dialer.main.c): canonical output (every line is FNV-hashed, only selected lines are printed unless R10_VERBOSE=1, then
 * "HASH <h> lines <n>"), a PRNG, required coverage counters (a zero exits 2; 3 = harness violation), and the callee recorders the dialer talks to (all of them log their
 * arguments in call order, so the ORDER and the arguments of every callback are compared):
 *   modem_get_param(m, id)   a scripted table g_prm[id]; id 0xe (MDMPRM_DP_ADDR) returns the harness' struct dp (NULL when g_nodp)
 *   modem_set_param(m, id, v), sysdep_strlen / sysdep_strcpy, dsplibs_debug_printf (the formatted text + a hash of the object under test at that very moment: g_snap)
 *   TONE_read / FP_Pow (R11 functions: recorders; in the 32-bit blob run they are the WEAKENED blob functions, so the strong recorders win; deterministic pseudo values)
 * The code under test (blob or port) calls these through its normal external symbols.  Everything printed is architecture independent: the 32-bit BLOB run is the golden of x86-64.
 */
#ifndef R10_COMMON_H
#define R10_COMMON_H
#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "callprog/callprog.h"

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
static u64 fnv_mix(u64 h, unsigned v)
{
  int k;
  for (k = 0; k < 4; k++) {
    h ^= (v >> (8 * k)) & 0xffu;
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

/* ------------------------------------------------------------------------------------------------ the call glue block + the scripted modem */
static int g_fake_modem, g_fake_modem2; /* two distinct modem objects: the pulse helpers call modem_set_param with the dp's modem (dp.modem), not their argument */
#define FAKEM ((struct modem *)&g_fake_modem)
#define FAKEM2 ((struct modem *)&g_fake_modem2)
#define MID(m) ((m) == FAKEM ? 1 : 2)
static CallGlue g_glue;  /* the block modem_get_param(m, 0xe)->dp_data points to */
static int g_nodp;       /* modem_get_param(m, 0xe) returns NULL */
#define PRM_MAX 128
static long g_prm[PRM_MAX]; /* modem_get_param(m, id) results (within the int range: the golden is the 32-bit run) */
static unsigned long g_ngp, g_nsp;

/* hooks: the harness may watch / disturb the code under test inside a callback (level poked, ...) */
static u64 (*g_snap)(void);                 /* hash of the object under test (called at every debug print) */
static void (*g_dbg_hook)(const char *msg); /* coverage observer of the debug text */
static void (*g_event_hook)(void);          /* called after every callback (get/set param, printf, strlen, strcpy, TONE_read, FP_Pow) */
static unsigned long g_events;

unsigned int dsplibs_debug_level = 0;
static unsigned lvl_value(int k)
{
  static const unsigned v[8] = {0u, 1u, 2u, 3u, 4u, 0x80000000u, 0xffffffffu, 2u};
  return v[(unsigned)k & 7u];
}
static void ev(void)
{
  g_events++;
  if (g_event_hook) g_event_hook();
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
  emit(0, "DBG %s @%016llx", buf, g_snap ? g_snap() : 0ULL);
  if (g_dbg_hook) g_dbg_hook(buf);
  ev();
  return 0;
}
long modem_get_param(struct modem *m, unsigned name)
{
  long v;
  if (m != FAKEM && m != FAKEM2) die3("modem_get_param with a foreign modem pointer");
  g_ngp++;
  if (name == 0xe) {
    emit(0, "GP m%d 14 -> %s @%016llx", MID(m), g_nodp ? "NULL" : "DP", g_snap ? g_snap() : 0ULL);
    ev();
    return g_nodp ? 0L : (long)(intptr_t)&g_glue.dp;
  }
  v = name < PRM_MAX ? g_prm[name] : -1;
  emit(0, "GP m%d %u -> %ld", MID(m), name, v);
  ev();
  return v;
}
long modem_set_param(struct modem *m, unsigned name, int val)
{
  if (m != FAKEM && m != FAKEM2) die3("modem_set_param with a foreign modem pointer");
  g_nsp++;
  emit(0, "SP m%d %u %d @%016llx", MID(m), name, val, g_snap ? g_snap() : 0ULL);
  ev();
  return 0;
}
size_t sysdep_strlen(const char *s)
{
  size_t n = strlen(s);
  emit(0, "SL %u", (unsigned)n);
  ev();
  return n;
}
char *sysdep_strcpy(char *d, const char *s)
{
  emit(0, "SC %u", (unsigned)strlen(s));
  ev();
  return strcpy(d, s);
}
static short pseudo(unsigned x)
{
  x = x * 2654435761u + 0x9e3779b9u;
  x ^= x >> 15;
  x *= 0x2c1b3c6du;
  x ^= x >> 13;
  return (short)(x >> 7);
}
static void quick(unsigned tag, unsigned v)
{ /* a cheap logged event (per-sample callbacks): the hash sees it, the line count too */
  if (g_verbose) { emit(0, "Q%u %d", tag, (int)v); return; }
  g_hash ^= tag;
  g_hash *= 1099511628211ULL;
  g_hash ^= v;
  g_hash *= 1099511628211ULL;
  g_lines++;
}
int TONE_read(int idx)
{
  quick(1, (unsigned)idx);
  ev();
  return pseudo((unsigned)(idx & 0xffff));
}
int FP_Pow(int x)
{
  quick(2, (unsigned)x);
  ev();
  return pseudo((unsigned)x ^ 0x5a5a5a5au);
}

/* ------------------------------------------------------------------------------------------------ layout independent object descriptions */
static u64 hash_pulse(void)
{
  u64 h = 1469598103934665603ULL;
  const CallPulse *p = &g_glue.pulse;
  h = fnv_mix(h, p->makeTime);
  h = fnv_mix(h, p->breakTime);
  h = fnv_mix(h, p->w5bc);
  h = fnv_mix(h, p->count);
  h = fnv_mix(h, p->counter);
  h = fnv_mix(h, (unsigned)p->hookOn);
  return h;
}
static u64 hash_dialer(const Dialer *d)
{
  u64 h = fnv_bytes(d->str, sizeof d->str);
  const DialerCfg *c = &d->cfg;
  h = fnv_mix(h, (unsigned)c->toneDigitLen);
  h = fnv_mix(h, (unsigned)c->toneBetween);
  h = fnv_mix(h, (unsigned)(unsigned short)c->gain1);
  h = fnv_mix(h, (unsigned)(unsigned short)c->gain2);
  h = fnv_mix(h, (unsigned)c->pulseOnHookTime);
  h = fnv_mix(h, (unsigned)c->pulseOffHookTime);
  h = fnv_mix(h, (unsigned)c->pulseBetween);
  h = fnv_mix(h, (unsigned)c->dialPause);
  h = fnv_mix(h, (unsigned)c->flashTime);
  h = fnv_mix(h, (unsigned)c->toneOrPulse);
  h = fnv_mix(h, (unsigned)c->modValid);
  h = fnv_mix(h, (unsigned)c->abcd);
  h = fnv_mix(h, (unsigned)c->pulseAndTone);
  h = fnv_mix(h, (unsigned)c->callingTone);
  h = fnv_mix(h, (unsigned)c->commaLimit);
  h = fnv_mix(h, (unsigned)c->digitPattern);
  h = fnv_mix(h, (unsigned)d->lastIdx);
  h = fnv_mix(h, (unsigned)d->syntax);
  h = fnv_mix(h, (unsigned)d->commaCount);
  h = fnv_mix(h, (unsigned)d->cur);
  h = fnv_mix(h, (unsigned)d->row);
  h = fnv_mix(h, (unsigned)d->col);
  h = fnv_mix(h, d->state);
  h = fnv_mix(h, (unsigned)d->left);
  h = fnv_mix(h, d->ph1);
  h = fnv_mix(h, d->ph2);
  h = fnv_mix(h, d->inc1);
  h = fnv_mix(h, d->inc2);
  h = fnv_mix(h, (unsigned)d->digitSent);
  h = fnv_mix(h, (unsigned)d->pulseActive);
  h = fnv_mix(h, (unsigned)d->pulseDone);
  h = fnv_mix(h, d->modem == FAKEM ? 1u : (d->modem == NULL ? 0u : (d->modem == FAKEM2 ? 3u : 2u)));
  return h;
}
#endif
