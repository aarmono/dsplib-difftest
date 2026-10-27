/*
 * R17 harness library (header only; used by test/r17_*_diff_main.c): derived from test/r11_common.h (kept separate: the R11 job stamps must not change).  Canonical output (every
 * line is FNV-hashed, only selected lines are printed unless R17_VERBOSE=1, then "HASH <h> lines <n>"), a PRNG, required coverage counters (a zero exits 2; 3 = harness
 * violation), a deterministic heap (red zones, poisoned frees, id-patterned fresh blocks; sizes mapped to the i386 sizes) with the callee recorders sysdep_malloc / free / memset /
 * memcpy, dsplibs_debug_printf (the formatted message + a hash of the object under test at that moment: g_snap; pointers in the text are mapped to block / modem ids),
 * modem_get_param (scripted table; parameter 0xc = the harness' `struct voice_info`), modem_recv_from_tty / modem_send_to_tty (scripted byte queues, every call logged), and
 * integer-only test signal generators (Q14 resonators with hard-coded 2cos(w) constants: no libm).  The code under test (blob or port) calls these through its normal external
 * symbols.  Everything printed is architecture independent: the 32-bit BLOB run is the golden of x86-64.
 */
#ifndef R17_COMMON_H
#define R17_COMMON_H
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

#include "voice/voice.h"

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
static u64 (*g_snap)(void);                 /* hash of the object under test (called at every debug print) */
static void (*g_dbg_hook)(const char *msg); /* coverage observer / state mutator of the debug text */

static unsigned log_size(unsigned size)
{
  /* the blob sizes of the typed blocks (sizeof differs on x86-64 and by the tail slot at -m32); the harness never allocates a buffer of one of these sizes itself */
  if (size == sizeof(Voice)) return 0x7dc;
  if (size == sizeof(VoiceGlue)) return 0x1484;
  if (size == sizeof(BeepGen)) return 0x12c;
  if (size == sizeof(Detector)) return 0x38;
  if (size == sizeof(Silence)) return 0x14;
  if (size == sizeof(ToneObj)) return 0x1c8;
  if (size == sizeof(FdspObj)) return FDSP_OBJ_BLOB_SIZE;
  if (size == sizeof(FdspChan)) return FDSP_CHAN_BLOB_SIZE;
  if (size == sizeof(FifoObj)) return FIFO8_OBJ_BLOB_SIZE;
  if (size == sizeof(Cadence)) return 0x2dc;
  if (size == sizeof(RcFixed)) return 8;
  if (size == sizeof(RcPoly)) return 0x1a4;
  if (size == sizeof(RcDec)) return 0x28;
  return size;
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
/* an offset inside a block, as the i386 build sees it (x86-64: the members of the glue / voice blocks move) */
static unsigned log_off(const Blk *b, unsigned off)
{
  if (b->size == sizeof(VoiceGlue)) {
#define M(m, i386) if (off >= offsetof(VoiceGlue, m) && off < offsetof(VoiceGlue, m) + sizeof(((VoiceGlue *)0)->m)) return (unsigned)(i386) + (off - (unsigned)offsetof(VoiceGlue, m))
    M(tty, 0x20); M(outb, 0x420); M(sbuf, 0x820); M(fin, 0x960); M(fout, 0xbe0); M(out, 0xe64); M(in, 0x1174);
#undef M
  }
  if (b->size == sizeof(Voice)) {
    if (off >= offsetof(Voice, area) && off < offsetof(Voice, area) + VOICE_AREA) return 0x38 + (off - (unsigned)offsetof(Voice, area));
  }
  if (b->size == sizeof(RcPoly)) return off - (unsigned)offsetof(RcPoly, hist); /* the history array is at +4 on i386 and +8 on x86-64 */
  return off;
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
  if (b && b->freed) die3("memset of a freed block");
  g_nmemset++;
  if (b)
    emit(0, "S %u %u %d %u", b->id, log_off(b, (unsigned)((unsigned char *)dst - b->p)), c, (unsigned)(b->size == len ? log_size((unsigned)len) : (unsigned)len));
  else
    emit(0, "S - %d %u", c, (unsigned)len);
  return memset(dst, c, len);
}
void *sysdep_memcpy(void *dst, const void *src, size_t len)
{
  Blk *b = find_blk(dst);
  if (b && b->freed) die3("memcpy into a freed block");
  g_nmemcpy++;
  memcpy(dst, src, len);
  emit(0, "C %u %u %u %016llx", b ? b->id : 0u, b ? log_off(b, (unsigned)((unsigned char *)dst - b->p)) : 0u, (unsigned)len, fnv_bytes(dst, len));
  return dst;
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

/* ------------------------------------------------------------------------------------------------ the scripted modem */
static int g_fake_modem, g_fake_modem2;
#define FAKEM ((struct modem *)&g_fake_modem)
#define FAKEM2 ((struct modem *)&g_fake_modem2)
#define MID(m) ((m) == FAKEM ? 1 : (m) == FAKEM2 ? 2 : 0)
#define PRM_MAX 128
static long g_prm[PRM_MAX]; /* modem_get_param(m, id) results (inside the int range: the golden is the 32-bit run) */
static struct voice_info g_vi; /* modem_get_param(m, 0xc): slmodemd's voice_info (the real struct: native layout) */
static unsigned long g_ngp;
/* voice_tone.h (R15) declares `int modem_get_param(struct modem *, int)`: the harness defines the real function under an asm label */
long h_get_param(struct modem *m, unsigned name) __asm__("modem_get_param");
long h_get_param(struct modem *m, unsigned name)
{
  long v;
  if (m != FAKEM && m != FAKEM2) die3("modem_get_param with a foreign modem pointer");
  g_ngp++;
  if (name == 0xc) {
    emit(0, "GP m%d 12 -> VI %u %u %u %u %u %u %u %u %u %u %u", MID(m), g_vi.comp_method, g_vi.sample_rate, g_vi.rx_gain, g_vi.tx_gain, g_vi.dtmf_symbol, g_vi.tone1_freq,
         g_vi.tone2_freq, g_vi.tone_duration, g_vi.inactivity_timer, g_vi.silence_detect_sensitivity, g_vi.silence_detect_period);
    return (long)(intptr_t)&g_vi;
  }
  v = name < PRM_MAX ? g_prm[name] : -1;
  emit(0, "GP m%d %u -> %ld", MID(m), name, v);
  return v;
}

/* tty queues: modem_recv_from_tty copies from the script (g_tty_in, g_tty_in_n bytes, g_tty_in_pos consumed), modem_send_to_tty appends to g_tty_out */
static unsigned char g_tty_in[1 << 16];
static unsigned g_tty_in_n, g_tty_in_pos;
static unsigned char g_tty_out[1 << 16];
static unsigned g_tty_out_n;
static int g_recv_cap = -1; /* >= 0: modem_recv_from_tty returns at most this many bytes (a script of its own limit) */
static unsigned long g_nrecv, g_nsend;
static void (*g_recv_hook)(void); /* called after every modem_recv_from_tty (a test that rewrites the caller's state from inside the call) */
int modem_recv_from_tty(struct modem *m, char *buf, int n)
{
  unsigned k;
  int r;
  if (m != FAKEM && m != FAKEM2) die3("modem_recv_from_tty with a foreign modem pointer");
  g_nrecv++;
  k = g_tty_in_n - g_tty_in_pos;
  if (n < 0) k = 0;
  if ((int)k > n && n >= 0) k = (unsigned)n;
  if (g_recv_cap >= 0 && (int)k > g_recv_cap) k = (unsigned)g_recv_cap;
  if (k) memcpy(buf, g_tty_in + g_tty_in_pos, k);
  g_tty_in_pos += k;
  r = (int)k;
  emit(0, "RECV m%d %d -> %d %016llx", MID(m), n, r, fnv_bytes(buf, k));
  if (g_recv_hook) g_recv_hook();
  return r;
}
int modem_send_to_tty(struct modem *m, const char *buf, int n)
{
  if (m != FAKEM && m != FAKEM2) die3("modem_send_to_tty with a foreign modem pointer");
  g_nsend++;
  if (n > 0 && g_tty_out_n + (unsigned)n <= sizeof g_tty_out) {
    memcpy(g_tty_out + g_tty_out_n, buf, (size_t)n);
    g_tty_out_n += (unsigned)n;
  }
  emit(0, "SEND m%d %d %016llx", MID(m), n, n > 0 ? fnv_bytes(buf, (size_t)n) : 0ULL);
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
/* the debug level a trial starts with: the level-changing hooks of the tests lower dsplibs_debug_level for the rest of the call only (script_reset / mk_voice restore it) */
static unsigned g_lvl_base;
#define SETLVL(x) (dsplibs_debug_level = g_lvl_base = (x))
int dsplibs_debug_printf(const char *fmt, ...)
{
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  if (strcmp(fmt, "%X  %X\n") == 0) {
    /* BeepGen's callback addresses: the callbacks are the blob's / port's own functions (unknown addresses): print only whether they are set */
    unsigned a = va_arg(ap, unsigned), b = va_arg(ap, unsigned);
    snprintf(buf, sizeof buf, "CBPTRS %s %s", a ? "set" : "NULL", b ? "set" : "NULL");
  } else if (strncmp(fmt, "voice: StrmVCE old:", 19) == 0) {
    /* STRM_VCE_GetFDSPEnvironmentalParams prints the OLD values of its two output words, which are the callers' UNINITIALISED stack variables (voice_create / voice_command): masked */
    snprintf(buf, sizeof buf, "voice: StrmVCE old: (uninitialised caller values masked)");
  } else if (strstr(fmt, "%p")) {
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
  if (g_dbg_hook) g_dbg_hook(buf);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ canonical description of the objects under test */
/* a handle as a block id (0 = NULL, 255 = a pointer into no block), a function pointer by the small id the harness gives it */
static unsigned hid(const void *p) { return p ? (blk_id_of(p) ? blk_id_of(p) : 255u) : 0u; }
static u64 snap_voice(const Voice *v, unsigned fnid)
{
  u64 h = 1469598103934665603ULL;
  h = fnv_mix(h, (unsigned)MID(v->ctx));
  h = fnv_mix(h, v->cb != NULL);
  h = fnv_mix(h, v->hook_on != NULL);
  h = fnv_mix(h, v->hook_off != NULL);
  h = fnv_mix(h, (unsigned)v->mode);
  h = fnv_mix(h, (unsigned)v->mode_prev);
  h = fnv_mix(h, hid(v->beep));
  h = fnv_mix(h, hid(v->det));
  h = fnv_mix(h, fnid);
  h = fnv_mix(h, hid(v->fifo));
  h = fnv_mix(h, hid(v->sil));
  h = fnv_mix(h, (unsigned)v->w2c);
  h = fnv_mix(h, (unsigned)v->w30);
  h = fnv_mix(h, hid(v->fdsp));
  h = fnv_mix(h, (unsigned)fnv_bytes(v->area.b, VOICE_AREA));
  h = fnv_mix(h, (unsigned)(fnv_bytes(v->area.b, VOICE_AREA) >> 32));
  h = fnv_mix(h, (unsigned)v->beep_idle);
  h = fnv_mix(h, (unsigned)v->etx);
  h = fnv_mix(h, (unsigned)v->abort);
  h = fnv_mix(h, (unsigned)v->vls);
  h = fnv_mix(h, (unsigned)v->w750);
  h = fnv_mix(h, v->playvol);
  h = fnv_mix(h, v->rx_on);
  h = fnv_mix(h, v->rate);
  h = fnv_mix(h, v->bits);
  h = fnv_mix(h, v->underrun);
  h = fnv_mix(h, v->en_tx);
  h = fnv_mix(h, v->en_rx);
  h = fnv_mix(h, v->en_on);
  h = fnv_mix(h, v->tmark);
  h = fnv_mix(h, v->tcount);
  h = fnv_mix(h, (unsigned)fnv_bytes(v->pad768, sizeof v->pad768));
  h = fnv_mix(h, (unsigned)v->first);
  h = fnv_floats(h, &v->dc, 1);
  h = fnv_floats(h, &v->rxGainVls1, 1);
  h = fnv_floats(h, &v->rxGainLine, 1);
  h = fnv_floats(h, &v->rxGainVls3, 1);
  return h;
}
static u64 snap_glue(const VoiceGlue *g)
{
  u64 h = 1469598103934665603ULL;
  h = fnv_mix(h, (unsigned)MID(g->modem));
  h = fnv_mix(h, hid(g->voice));
  h = fnv_mix(h, g->vi == &g_vi);
  h = fnv_mix(h, (unsigned)g->state);
  h = fnv_mix(h, (unsigned)g->last);
  h = fnv_mix(h, (unsigned)g->cnt);
  h = fnv_mix(h, hid(g->rcIn));
  h = fnv_mix(h, hid(g->rcOut));
  h = fnv_mix(h, (unsigned)fnv_bytes(g->tty, sizeof g->tty));
  h = fnv_mix(h, (unsigned)fnv_bytes(g->outb, sizeof g->outb));
  h = fnv_shorts(h, g->sbuf, 0xa0);
  h = fnv_floats(h, g->fin, 160);
  h = fnv_floats(h, g->fout, 160);
  h = fnv_mix(h, g->blk);
  h = fnv_mix(h, g->out.count);
  h = fnv_mix(h, g->out.wr);
  h = fnv_mix(h, g->out.rd);
  h = fnv_mix(h, g->out.half);
  h = fnv_shorts(h, g->out.data, VRING);
  h = fnv_mix(h, g->in.count);
  h = fnv_mix(h, g->in.wr);
  h = fnv_mix(h, g->in.rd);
  h = fnv_mix(h, g->in.half);
  h = fnv_shorts(h, g->in.data, VRING);
  return h;
}


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

/* ------------------------------------------------------------------------------------------------ integer-only test signals */
/* a Q14 resonator: y[n] = (c * y[n-1] >> 14) - y[n-2], c = round(2 cos(2 pi f / fs) * 16384) for fs 8000 and 9600 (a table of integers: no libm at run time) */
typedef struct {
  int c, y1, y2;
} Osc;
/* generated offline (python math, not part of the harness run): c(f) = round(2 cos(2 pi f / fs) * 16384) for f = 0, 5, 10 ... 3500 Hz */
static const int osc_tab8000[701] = {
32768, 32768, 32767, 32766, 32764, 32762, 32759, 32756, 32752, 32748, 32743, 32737, 32732, 32725,
32718, 32711, 32703, 32695, 32686, 32677, 32667, 32657, 32646, 32634, 32623, 32610, 32597, 32584,
32570, 32556, 32541, 32525, 32510, 32493, 32476, 32459, 32441, 32423, 32404, 32384, 32365, 32344,
32323, 32302, 32280, 32258, 32235, 32211, 32188, 32163, 32138, 32113, 32087, 32061, 32034, 32007,
31979, 31951, 31922, 31892, 31863, 31832, 31802, 31770, 31739, 31706, 31674, 31640, 31607, 31572,
31538, 31503, 31467, 31431, 31394, 31357, 31319, 31281, 31243, 31204, 31164, 31124, 31084, 31043,
31001, 30959, 30917, 30874, 30831, 30787, 30743, 30698, 30653, 30607, 30561, 30514, 30467, 30419,
30371, 30323, 30274, 30224, 30174, 30124, 30073, 30022, 29970, 29918, 29865, 29812, 29758, 29704,
29649, 29594, 29539, 29483, 29427, 29370, 29312, 29255, 29197, 29138, 29079, 29019, 28959, 28899,
28838, 28777, 28715, 28653, 28590, 28527, 28463, 28399, 28335, 28270, 28205, 28139, 28073, 28006,
27939, 27872, 27804, 27736, 27667, 27598, 27528, 27458, 27388, 27317, 27246, 27174, 27102, 27029,
26956, 26883, 26809, 26735, 26660, 26585, 26510, 26434, 26358, 26281, 26204, 26127, 26049, 25970,
25892, 25813, 25733, 25653, 25573, 25492, 25411, 25330, 25248, 25166, 25083, 25000, 24917, 24833,
24749, 24665, 24580, 24494, 24409, 24323, 24236, 24149, 24062, 23975, 23887, 23799, 23710, 23621,
23532, 23442, 23352, 23261, 23170, 23079, 22988, 22896, 22804, 22711, 22618, 22525, 22431, 22337,
22243, 22148, 22053, 21958, 21862, 21766, 21670, 21573, 21476, 21379, 21281, 21183, 21085, 20986,
20887, 20788, 20688, 20588, 20488, 20387, 20286, 20185, 20084, 19982, 19880, 19777, 19675, 19572,
19468, 19365, 19261, 19156, 19052, 18947, 18842, 18736, 18631, 18525, 18418, 18312, 18205, 18098,
17990, 17883, 17775, 17666, 17558, 17449, 17340, 17231, 17121, 17011, 16901, 16791, 16680, 16569,
16458, 16347, 16235, 16123, 16011, 15899, 15786, 15673, 15560, 15447, 15333, 15219, 15105, 14991,
14876, 14762, 14647, 14531, 14416, 14300, 14184, 14068, 13952, 13835, 13719, 13602, 13485, 13367,
13250, 13132, 13014, 12896, 12777, 12659, 12540, 12421, 12302, 12182, 12063, 11943, 11823, 11703,
11583, 11462, 11342, 11221, 11100, 10979, 10857, 10736, 10614, 10492, 10370, 10248, 10126, 10003,
9881, 9758, 9635, 9512, 9389, 9265, 9142, 9018, 8895, 8771, 8647, 8522, 8398, 8274,
8149, 8024, 7900, 7775, 7650, 7524, 7399, 7274, 7148, 7022, 6897, 6771, 6645, 6519,
6393, 6266, 6140, 6014, 5887, 5760, 5634, 5507, 5380, 5253, 5126, 4999, 4872, 4744,
4617, 4490, 4362, 4235, 4107, 3979, 3851, 3724, 3596, 3468, 3340, 3212, 3084, 2956,
2827, 2699, 2571, 2443, 2314, 2186, 2058, 1929, 1801, 1672, 1544, 1415, 1286, 1158,
1029, 901, 772, 643, 515, 386, 257, 129, 0, -129, -257, -386, -515, -643,
-772, -901, -1029, -1158, -1286, -1415, -1544, -1672, -1801, -1929, -2058, -2186, -2314, -2443,
-2571, -2699, -2827, -2956, -3084, -3212, -3340, -3468, -3596, -3724, -3851, -3979, -4107, -4235,
-4362, -4490, -4617, -4744, -4872, -4999, -5126, -5253, -5380, -5507, -5634, -5760, -5887, -6014,
-6140, -6266, -6393, -6519, -6645, -6771, -6897, -7022, -7148, -7274, -7399, -7524, -7650, -7775,
-7900, -8024, -8149, -8274, -8398, -8522, -8647, -8771, -8895, -9018, -9142, -9265, -9389, -9512,
-9635, -9758, -9881, -10003, -10126, -10248, -10370, -10492, -10614, -10736, -10857, -10979, -11100, -11221,
-11342, -11462, -11583, -11703, -11823, -11943, -12063, -12182, -12302, -12421, -12540, -12659, -12777, -12896,
-13014, -13132, -13250, -13367, -13485, -13602, -13719, -13835, -13952, -14068, -14184, -14300, -14416, -14531,
-14647, -14762, -14876, -14991, -15105, -15219, -15333, -15447, -15560, -15673, -15786, -15899, -16011, -16123,
-16235, -16347, -16458, -16569, -16680, -16791, -16901, -17011, -17121, -17231, -17340, -17449, -17558, -17666,
-17775, -17883, -17990, -18098, -18205, -18312, -18418, -18525, -18631, -18736, -18842, -18947, -19052, -19156,
-19261, -19365, -19468, -19572, -19675, -19777, -19880, -19982, -20084, -20185, -20286, -20387, -20488, -20588,
-20688, -20788, -20887, -20986, -21085, -21183, -21281, -21379, -21476, -21573, -21670, -21766, -21862, -21958,
-22053, -22148, -22243, -22337, -22431, -22525, -22618, -22711, -22804, -22896, -22988, -23079, -23170, -23261,
-23352, -23442, -23532, -23621, -23710, -23799, -23887, -23975, -24062, -24149, -24236, -24323, -24409, -24494,
-24580, -24665, -24749, -24833, -24917, -25000, -25083, -25166, -25248, -25330, -25411, -25492, -25573, -25653,
-25733, -25813, -25892, -25970, -26049, -26127, -26204, -26281, -26358, -26434, -26510, -26585, -26660, -26735,
-26809, -26883, -26956, -27029, -27102, -27174, -27246, -27317, -27388, -27458, -27528, -27598, -27667, -27736,
-27804, -27872, -27939, -28006, -28073, -28139, -28205, -28270, -28335, -28399, -28463, -28527, -28590, -28653,
-28715, -28777, -28838, -28899, -28959, -29019, -29079, -29138, -29197, -29255, -29312, -29370, -29427, -29483,
-29539, -29594, -29649, -29704, -29758, -29812, -29865, -29918, -29970, -30022, -30073, -30124, -30174, -30224,
-30274};
static const int osc_tab9600[701] = {
32768, 32768, 32767, 32766, 32765, 32764, 32762, 32759, 32757, 32754, 32750, 32747, 32743, 32738,
32734, 32729, 32723, 32717, 32711, 32705, 32698, 32691, 32683, 32675, 32667, 32658, 32649, 32640,
32631, 32621, 32610, 32600, 32588, 32577, 32565, 32553, 32541, 32528, 32515, 32501, 32488, 32473,
32459, 32444, 32429, 32413, 32397, 32381, 32365, 32348, 32330, 32313, 32295, 32276, 32258, 32239,
32219, 32200, 32180, 32159, 32138, 32117, 32096, 32074, 32052, 32029, 32007, 31984, 31960, 31936,
31912, 31887, 31863, 31837, 31812, 31786, 31760, 31733, 31706, 31679, 31651, 31624, 31595, 31567,
31538, 31508, 31479, 31449, 31419, 31388, 31357, 31326, 31294, 31262, 31230, 31197, 31164, 31131,
31097, 31063, 31029, 30994, 30959, 30924, 30888, 30853, 30816, 30780, 30743, 30705, 30668, 30630,
30592, 30553, 30514, 30475, 30435, 30395, 30355, 30315, 30274, 30232, 30191, 30149, 30107, 30064,
30022, 29979, 29935, 29891, 29847, 29803, 29758, 29713, 29668, 29622, 29576, 29530, 29483, 29436,
29389, 29341, 29293, 29245, 29197, 29148, 29099, 29049, 28999, 28949, 28899, 28848, 28797, 28746,
28694, 28642, 28590, 28537, 28485, 28431, 28378, 28324, 28270, 28216, 28161, 28106, 28051, 27995,
27939, 27883, 27827, 27770, 27713, 27655, 27598, 27540, 27482, 27423, 27364, 27305, 27246, 27186,
27126, 27066, 27005, 26944, 26883, 26821, 26760, 26698, 26635, 26573, 26510, 26447, 26383, 26320,
26255, 26191, 26127, 26062, 25997, 25931, 25865, 25800, 25733, 25667, 25600, 25533, 25466, 25398,
25330, 25262, 25193, 25125, 25056, 24986, 24917, 24847, 24777, 24707, 24636, 24565, 24494, 24423,
24351, 24279, 24207, 24135, 24062, 23989, 23916, 23843, 23769, 23695, 23621, 23546, 23472, 23397,
23322, 23246, 23170, 23095, 23018, 22942, 22865, 22788, 22711, 22634, 22556, 22478, 22400, 22322,
22243, 22164, 22085, 22006, 21926, 21846, 21766, 21686, 21605, 21525, 21444, 21363, 21281, 21199,
21118, 21035, 20953, 20871, 20788, 20705, 20622, 20538, 20454, 20371, 20286, 20202, 20118, 20033,
19948, 19863, 19777, 19692, 19606, 19520, 19434, 19347, 19261, 19174, 19087, 18999, 18912, 18824,
18736, 18648, 18560, 18472, 18383, 18294, 18205, 18116, 18026, 17937, 17847, 17757, 17666, 17576,
17485, 17395, 17304, 17213, 17121, 17030, 16938, 16846, 16754, 16662, 16569, 16477, 16384, 16291,
16198, 16105, 16011, 15917, 15824, 15730, 15636, 15541, 15447, 15352, 15257, 15162, 15067, 14972,
14876, 14781, 14685, 14589, 14493, 14397, 14300, 14204, 14107, 14010, 13913, 13816, 13719, 13621,
13524, 13426, 13328, 13230, 13132, 13033, 12935, 12836, 12738, 12639, 12540, 12441, 12341, 12242,
12142, 12043, 11943, 11843, 11743, 11643, 11543, 11442, 11342, 11241, 11140, 11039, 10938, 10837,
10736, 10634, 10533, 10431, 10330, 10228, 10126, 10024, 9922, 9819, 9717, 9615, 9512, 9409,
9307, 9204, 9101, 8998, 8895, 8791, 8688, 8585, 8481, 8377, 8274, 8170, 8066, 7962,
7858, 7754, 7650, 7545, 7441, 7336, 7232, 7127, 7022, 6918, 6813, 6708, 6603, 6498,
6393, 6288, 6182, 6077, 5971, 5866, 5760, 5655, 5549, 5444, 5338, 5232, 5126, 5020,
4914, 4808, 4702, 4596, 4490, 4383, 4277, 4171, 4064, 3958, 3851, 3745, 3638, 3532,
3425, 3319, 3212, 3105, 2998, 2892, 2785, 2678, 2571, 2464, 2357, 2250, 2143, 2036,
1929, 1822, 1715, 1608, 1501, 1394, 1286, 1179, 1072, 965, 858, 751, 643, 536,
429, 322, 214, 107, 0, -107, -214, -322, -429, -536, -643, -751, -858, -965,
-1072, -1179, -1286, -1394, -1501, -1608, -1715, -1822, -1929, -2036, -2143, -2250, -2357, -2464,
-2571, -2678, -2785, -2892, -2998, -3105, -3212, -3319, -3425, -3532, -3638, -3745, -3851, -3958,
-4064, -4171, -4277, -4383, -4490, -4596, -4702, -4808, -4914, -5020, -5126, -5232, -5338, -5444,
-5549, -5655, -5760, -5866, -5971, -6077, -6182, -6288, -6393, -6498, -6603, -6708, -6813, -6918,
-7022, -7127, -7232, -7336, -7441, -7545, -7650, -7754, -7858, -7962, -8066, -8170, -8274, -8377,
-8481, -8585, -8688, -8791, -8895, -8998, -9101, -9204, -9307, -9409, -9512, -9615, -9717, -9819,
-9922, -10024, -10126, -10228, -10330, -10431, -10533, -10634, -10736, -10837, -10938, -11039, -11140, -11241,
-11342, -11442, -11543, -11643, -11743, -11843, -11943, -12043, -12142, -12242, -12341, -12441, -12540, -12639,
-12738, -12836, -12935, -13033, -13132, -13230, -13328, -13426, -13524, -13621, -13719, -13816, -13913, -14010,
-14107, -14204, -14300, -14397, -14493, -14589, -14685, -14781, -14876, -14972, -15067, -15162, -15257, -15352,
-15447, -15541, -15636, -15730, -15824, -15917, -16011, -16105, -16198, -16291, -16384, -16477, -16569, -16662,
-16754, -16846, -16938, -17030, -17121, -17213, -17304, -17395, -17485, -17576, -17666, -17757, -17847, -17937,
-18026, -18116, -18205, -18294, -18383, -18472, -18560, -18648, -18736, -18824, -18912, -18999, -19087, -19174,
-19261, -19347, -19434, -19520, -19606, -19692, -19777, -19863, -19948, -20033, -20118, -20202, -20286, -20371,
-20454, -20538, -20622, -20705, -20788, -20871, -20953, -21035, -21118, -21199, -21281, -21363, -21444, -21525,
-21605};
static int osc_c(int hz, int fs) { return (fs == 9600 ? osc_tab9600 : osc_tab8000)[(unsigned)(hz / 5) % 701u]; }
static void osc_init(Osc *o, int c, int amp)
{
  o->c = c;
  o->y2 = 0;
  o->y1 = amp;
}
static int osc_next(Osc *o)
{
  int y = (int)(((long long)o->c * o->y1) >> 14) - o->y2;
  if (y > 32000) y = 32000;
  if (y < -32000) y = -32000;
  o->y2 = o->y1;
  o->y1 = y;
  return y;
}
#endif
