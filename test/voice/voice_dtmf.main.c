/*
 * R16 differential harness: the voice DTMF detector (create_dtmf, dtmf_detect, dtmf_test, dtmf_progress) and the beep / DTMF generator (beepgen_create / delete / start_beep /
 * start_dtmf / sample) of the real dsplibs.o vs src/voice/voice_dtmf.c (+ voice_dtmf_tables.c).  The port build links R15's voice_tone.c for notch / voice_getgain (the callees of dtmf_detect /
 * beepgen_*; their cadence_* callees are abort stubs here, the blob keeps its own).  Library test/voice/r16_common.h (hashed canonical output, coverage counters exit 2, 3 = harness violation,
 * deterministic heap, recorders sysdep_malloc / free / memset / dsplibs_debug_printf).  The 32-bit BLOB run is the x86-64 golden: every INPUT float is built from integers (bit patterns,
 * exact int -> float conversions, integer oscillators: NO libm sin / cos) so the x86-64 run feeds the same inputs; floats are printed / hashed as bit patterns.
 *
 * The blob's callees: modem_get_param (stub_env's abort stub renamed), the BeepGen callbacks cb11c / cb120 / cb124 (recorders cbA .. cbF, which can also rewrite the object's count / cur:
 * the blob reloads them after each call), the debug printf (a hook lowers dsplibs_debug_level inside a chosen message: the blob re-reads the level at fixed points, the port must too).
 *
 * Scenarios (each its own process; argv[1]):
 *   0 dtmf_test: tables (us_coef / eur_coef / biascoef), structured digit patterns, exact boundary constructions (powers of two make the products with the blob's float constants exact),
 *     random mixtures, every special value (NaN / sNaN / inf / denormal / 0) against every other;
 *   1 create_dtmf (NULL / failing malloc / dirty existing objects) and dtmf_detect (sequences of integer signals, random object states, preloaded energies that drive the
 *     history / busy / easy logic exactly);
 *   2 dtmf_progress over synthetic DTMF bursts (integer oscillators, +-2 % detuned, missing / doubled tones, 6 amplitudes, chunked), noise and special streams, count <= 0;
 *   3 beepgen_create (every callback combination, debug levels, hooks, failing malloc), beepgen_delete, start_dtmf (every character x durations x queue states), start_beep (frequency
 *     pairs, queues up to 20 entries, callbacks that rewrite the object), the debug text with the level-lowering hooks, extreme GetGain parameters, guarded children;
 *   4 beepgen_sample: real sequences sampled to the end, random / wild object states (NaN / sNaN / inf phases and frequencies), controlled transitions at all debug levels with hooks.
 * R16_VERBOSE=1 prints every line, R16_SEED selects another input stream, R16_SCALE=n divides every counted loop, R16_SWEEP=n the big random sweeps (mutation runs).
 */
#include "r16_common.h"

static float fb(unsigned u)
{
  float f;
  memcpy(&f, &u, 4);
  return f;
}
/* the bits of a float OBJECT, read from memory (a float passed by value through the i386 x87 would quiet a signalling NaN) */
static unsigned bm(const float *p)
{
  unsigned u;
  memcpy(&u, p, 4);
  return u;
}
#define bf(x) bm(&(x))
static unsigned scale_div = 1, sweep_div = 1;
static unsigned SC(unsigned n) /* R16_SCALE: every counted loop (debugging) */
{
  unsigned v = n / scale_div;
  return v ? v : 1;
}
static unsigned SW(unsigned n) /* R16_SWEEP: the big random sweeps only (mutation runs); R16_SCALE divides these too */
{
  unsigned v = n / (sweep_div * scale_div);
  return v ? v : 1;
}
static void set_pc(int dbl)
{
#if defined(__i386__) || defined(__x86_64__)
  unsigned short cw;
  __asm__ volatile("fnstcw %0" : "=m"(cw));
  cw = (unsigned short)((cw & ~0x300) | (dbl ? 0x200 : 0x300));
  __asm__ volatile("fldcw %0" : : "m"(cw));
#else
  (void)dbl;
#endif
}
typedef struct {
  u64 h;
  unsigned long n;
} Acc;
static void acc_init(Acc *a)
{
  a->h = 1469598103934665603ULL;
  a->n = 0;
}
static void acc_bytes(Acc *a, const void *p, size_t n)
{
  const unsigned char *b = (const unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) {
    a->h ^= b[i];
    a->h *= 1099511628211ULL;
  }
  a->n++;
}
static void acc_u32(Acc *a, unsigned v) { acc_bytes(a, &v, 4); }
static void acc_emit(const char *name, const Acc *a) { emit(1, "ACC %-28s n=%lu h=%016llx", name, a->n, a->h); }

static const unsigned special_bits[] = {
    0x00000000u, 0x80000000u, 0x00000001u, 0x80000001u, 0x007fffffu, 0x807fffffu, 0x00800000u, 0x80800000u, 0x7f7fffffu, 0xff7fffffu, 0x7f800000u, 0xff800000u, 0x7fc00000u, 0xffc00000u,
    0x7fc12345u, 0xffc54321u, 0x7f800001u, 0xff812345u, 0x3f800000u, 0xbf800000u, 0x40000000u, 0x3f000000u, 0x3e800000u, 0x40490fdbu, 0x40c90fdbu, 0x40c90fdau, 0x40c90fdcu, 0x40490fdau,
    0x40490fdcu, 0x3fc90fdbu, 0xc0490fdbu, 0xc0c90fdbu, 0x3f7fffffu, 0x3f800001u, 0x4b000000u, 0x4b7fffffu, 0x4f000000u, 0xcf000000u, 0x477fff00u, 0x47000000u, 0x46fffe00u, 0x4effffffu,
    0x38d1b717u, 0x3a83126fu, 0x3d4104fbu, 0x44a28000u, 0x45034000u, 0x3dcccccdu, 0xbdcccccdu, 0x42c80000u, 0x7f000000u, 0x01000000u, 0x00400000u, 0x3b800000u, 0x3b83126fu};
#define NSPECIAL (sizeof special_bits / sizeof special_bits[0])
static float rspecial(void) { return fb(special_bits[rmod((unsigned)NSPECIAL)]); }
/* a random normal float with biased exponent in [lo, hi] */
static float rfl(unsigned lo, unsigned hi)
{
  unsigned u = rnd();
  return fb((u & 0x80000000u) | ((lo + rmod(hi - lo + 1)) << 23) | (u & 0x7fffffu));
}
/* Floats that may be a signalling NaN are handled as BIT PATTERNS only (a float returned, passed by value or copied through the i386 x87 would turn a signalling NaN into a quiet one
 * on the 32-bit side and not on the x86-64 side): putb stores bits into a float object */
static void putb(float *d, unsigned b) { memcpy(d, &b, 4); }
static unsigned rspecialb(void) { return special_bits[rmod((unsigned)NSPECIAL)]; }
static unsigned rflb(unsigned lo, unsigned hi)
{
  unsigned u = rnd();
  return (u & 0x80000000u) | ((lo + rmod(hi - lo + 1)) << 23) | (u & 0x7fffffu);
}
static float rfl_pos(unsigned lo, unsigned hi) { return fb(rflb(lo, hi) & 0x7fffffffu); }
/* a pattern that is mostly "ordinary" and sometimes special (weighted) */
static unsigned rmixb(void)
{
  unsigned k = rmod(100);
  if (k < 60) return rflb(0x6c, 0x84);
  if (k < 75) return rflb(0x40, 0xa0);
  if (k < 90) return rspecialb();
  return rnd();
}

/* run fn in a child: 0 = returned normally, 1 = killed by a signal, 3 = harness violation (die3), 4 = leak */
static int guarded(void (*fn)(void *), void *arg)
{
  int st = 0;
  pid_t c;
  fflush(stdout);
  c = fork();
  if (c < 0) die3("fork");
  if (c == 0) {
    fn(arg);
    _exit(0);
  }
  if (waitpid(c, &st, 0) != c) die3("waitpid");
  if (WIFSIGNALED(st)) return 1;
  return WEXITSTATUS(st);
}
static void guarded_report(const char *what, int r)
{
  if (g_verbose) fprintf(stderr, "guarded %s: child status %d\n", what, r);
#ifdef TEST_BLOB
  if (r != 0 && r != 1 && r != 3) die3("%s: blob child status %d", what, r);
#else
  if (r != 0) die3("%s: the port child failed (status %d)", what, r);
#endif
  emit(1, "GUARDED %s handled", what);
}


/* ---------------------------------------------------------------------------------------------- the callees under test: blob / port calling conventions */
/* dtmf_detect(float x, ...) / dtmf_test(e, float eb, ..) take a float BY VALUE: on i386 the argument is 4 stack bytes, so the harness passes the bit pattern as an unsigned (a float passed
 * through the i386 x87 would quiet a signalling NaN before the callee sees it; the blob loads it with fld, which quiets it anyway, but the harness must not decide that).  The same
 * call through a cast function pointer works for the blob and for the port. */
#if defined(__i386__)
extern int dtmf_detect_bits(unsigned xbits, Dtmf *d, short mode) __asm__("dtmf_detect");
extern int dtmf_test_bits(const float *e, unsigned ebits, short mode) __asm__("dtmf_test");
#define DETECT(xb, d, m) dtmf_detect_bits((xb), (d), (short)(m))
#define DTEST(e, ebb, m) dtmf_test_bits((e), (ebb), (short)(m))
#else
#define DETECT(xb, d, m) dtmf_detect(fb(xb), (d), (short)(m))
#define DTEST(e, ebb, m) dtmf_test((e), fb(ebb), (short)(m))
#endif

enum {
  CV_T_TABLES, CV_T_CODE0, /* 16 ids: CV_T_CODE0 .. CV_T_CODE0 + 15 */
  CV_T_M1 = CV_T_CODE0 + 16, CV_T_ODD, CV_T_NANIN, CV_T_BOUND, CV_T_EBLOW, CV_T_MODE1, CV_T_MODEX, CV_T_SPECIAL,
  CV_D_CREATE_NULL, CV_D_CREATE_FAIL, CV_D_CREATE_FAIL_DBG, CV_D_CREATE_EXIST, CV_D_SKIP, CV_D_BLOCK, CV_D_VALID, CV_D_EASY, CV_D_RELEASE, CV_D_BUSY_HOLD, CV_D_BUSY_NONE, CV_D_NONE,
  CV_D_PREFILTER, CV_D_PROGRESS, CV_D_DIGIT_RET, CV_D_NEG_COUNT, CV_D_MODE1, CV_D_NANX, CV_D_ALL_DIGITS,
  CV_B_CREATE_NULL, CV_B_CREATE_EXIST, CV_B_CREATE_FAIL, CV_B_DBG, CV_B_CBNULL, CV_B_DTMF_ALL, CV_B_DTMF_BANG, CV_B_DTMF_COMMA, CV_B_DTMF_DEFAULT, CV_B_BEEP, CV_B_QUEUE, CV_B_QUEUE_FULL,
  CV_B_FIRST_ZERO, CV_B_FIRST_M1, CV_B_SECOND_ZERO, CV_B_HOOKS, CV_B_DELETE, CV_B_GUARDED, CV_B_REENT, CV_B_DURBIG, CV_B_CB124,
  CV_S_RUN, CV_S_END, CV_S_NEXT, CV_S_F1ZERO, CV_S_F1M1, CV_S_F2ZERO, CV_S_HOOK120, CV_S_NANF1, CV_S_RANDSTATE, CV_S_SNAN, CV_S_DBG, CV_S_LIMIT, CV_S_CB11C, CV_S_REENT, CV_S_HUGEPHASE, CV_S_OBJHOOK, CV_S_NEGCOUNT, CV_S_CBHIT_cbA, CV_S_CBHIT_cbB, CV_S_CBHIT_cbC, CV_S_CBHIT_cbD
};
static void cov_setup(int scen)
{
  int i;
  char *nm;
  switch (scen) {
    case 0:
      cov_def(CV_T_TABLES, "tables");
      for (i = 0; i < 16; i++) {
        nm = (char *)malloc(32);
        snprintf(nm, 32, "dtmf_test result %d", i);
        cov_def(CV_T_CODE0 + i, nm);
      }
      cov_def(CV_T_M1, "dtmf_test result -1");
      cov_def(CV_T_ODD, "dtmf_test result outside -1 .. 15 (no minimum index found)");
      cov_def(CV_T_NANIN, "dtmf_test NaN inputs");
      cov_def(CV_T_BOUND, "dtmf_test boundary constructions");
      cov_def(CV_T_EBLOW, "dtmf_test eb below the threshold");
      cov_def(CV_T_MODE1, "dtmf_test mode 1");
      cov_def(CV_T_MODEX, "dtmf_test other modes");
      cov_def(CV_T_SPECIAL, "dtmf_test special values");
      break;
    case 1:
      cov_def(CV_D_CREATE_NULL, "create_dtmf NULL");
      cov_def(CV_D_CREATE_FAIL, "create_dtmf malloc failure");
      cov_def(CV_D_CREATE_FAIL_DBG, "create_dtmf malloc failure with the debug text");
      cov_def(CV_D_CREATE_EXIST, "create_dtmf existing");
      cov_def(CV_D_SKIP, "dtmf_detect -2 (sample skipped)");
      cov_def(CV_D_BLOCK, "dtmf_detect block evaluated");
      cov_def(CV_D_VALID, "dtmf_detect new digit (check_for_valid)");
      cov_def(CV_D_EASY, "dtmf_detect new digit (easy)");
      cov_def(CV_D_RELEASE, "dtmf_detect digit released");
      cov_def(CV_D_BUSY_HOLD, "dtmf_detect busy, digit still there");
      cov_def(CV_D_BUSY_NONE, "dtmf_detect busy, tone gone");
      cov_def(CV_D_NONE, "dtmf_detect no digit");
      cov_def(CV_D_PREFILTER, "dtmf_detect mode 1 prefilter");
      cov_def(CV_D_NANX, "dtmf_detect special input");
      break;
    case 2:
      cov_def(CV_D_PROGRESS, "dtmf_progress calls");
      cov_def(CV_D_DIGIT_RET, "dtmf_progress digit result");
      cov_def(CV_D_NEG_COUNT, "dtmf_progress count <= 0");
      cov_def(CV_D_MODE1, "dtmf_progress mode 1");
      cov_def(CV_D_ALL_DIGITS, "dtmf_progress all 16 digits");
      break;
    case 3:
      cov_def(CV_B_CREATE_NULL, "beepgen_create NULL");
      cov_def(CV_B_CREATE_EXIST, "beepgen_create existing");
      cov_def(CV_B_CREATE_FAIL, "beepgen_create malloc failure");
      cov_def(CV_B_DBG, "debug text");
      cov_def(CV_B_CBNULL, "callbacks NULL");
      cov_def(CV_B_DTMF_ALL, "start_dtmf every character");
      cov_def(CV_B_DTMF_BANG, "start_dtmf '!'");
      cov_def(CV_B_DTMF_COMMA, "start_dtmf ','");
      cov_def(CV_B_DTMF_DEFAULT, "start_dtmf unmapped character");
      cov_def(CV_B_BEEP, "start_beep");
      cov_def(CV_B_QUEUE, "queued second entry");
      cov_def(CV_B_QUEUE_FULL, "queue of 20 entries");
      cov_def(CV_B_FIRST_ZERO, "first entry f1 == 0");
      cov_def(CV_B_FIRST_M1, "first entry f1 == -1");
      cov_def(CV_B_SECOND_ZERO, "first entry f2 == 0");
      cov_def(CV_B_HOOKS, "level lowered inside a message");
      cov_def(CV_B_DELETE, "beepgen_delete");
      cov_def(CV_B_GUARDED, "guarded children");
      cov_def(CV_B_REENT, "callback modifies the object");
      cov_def(CV_B_DURBIG, "huge / negative duration");
      cov_def(CV_B_CB124, "'!' asks cb124");
      break;
    case 4:
      cov_def(CV_S_RUN, "sample inside an entry");
      cov_def(CV_S_END, "sample: last entry ended (1)");
      cov_def(CV_S_NEXT, "sample: next entry started");
      cov_def(CV_S_F1ZERO, "next entry f1 == 0");
      cov_def(CV_S_F1M1, "next entry f1 == -1");
      cov_def(CV_S_F2ZERO, "next entry f2 == 0");
      cov_def(CV_S_NANF1, "NaN frequency");
      cov_def(CV_S_RANDSTATE, "random object state");
      cov_def(CV_S_SNAN, "signalling NaN in the queue");
      cov_def(CV_S_DBG, "debug text");
      cov_def(CV_S_LIMIT, "limit reached");
      cov_def(CV_S_REENT, "callback modifies the object");
      cov_def(CV_S_HUGEPHASE, "huge / infinite phase");
      cov_def(CV_S_OBJHOOK, "a debug message rewrites the object");
      cov_def(CV_S_NEGCOUNT, "negative queue count");
      cov_def(CV_S_CBHIT_cbA, "cb11c (cbA) called");
      cov_def(CV_S_CBHIT_cbB, "cb11c (cbB) called");
      cov_def(CV_S_CBHIT_cbC, "cb120 (cbC) called");
      cov_def(CV_S_CBHIT_cbD, "cb120 (cbD) called");
      break;
    default: break;
  }
}

/* ---------------------------------------------------------------------------------------------- recorders: modem_get_param, the BeepGen callbacks */
static int g_quiet;
static char g_modem_obj[16];
static struct modem *const g_modem = (struct modem *)(void *)g_modem_obj; /* only compared with ==, never dereferenced */
static int g_mgp[256];
long modem_get_param(struct modem *m, unsigned id)
{
  if (!g_quiet) emit(0, "MGP %d %s", (int)id, m == g_modem ? "ctx" : "BADCTX");
  return g_mgp[id & 0xff];
}
static const unsigned lvl_pool[6] = {0u, 1u, 2u, 3u, 0x80000000u, 0xffffffffu}; /* the level is compared as unsigned: 0x80000000 / 0xffffffff print everything */
static BeepGen *g_reent_bg; /* when set, cbB / cbD modify this object (the blob reloads count / cur after the call) */
static int g_reent_count, g_reent_cur;
static unsigned g_cb124_ret[2] = {500u, 3u};
#define HOOK_CB(name, reent)                                                    \
  static void name(struct modem *m)                                             \
  {                                                                             \
    emit(0, #name " %s", m == g_modem ? "ctx" : "BADCTX");                      \
    cov_hit(CV_S_CBHIT_##name);                                                 \
    if (reent && g_reent_bg) {                                                  \
      g_reent_bg->count = g_reent_count;                                        \
      g_reent_bg->cur = g_reent_cur;                                            \
      cov_hit(CV_B_REENT);                                                      \
      cov_hit(CV_S_REENT);                                                      \
    }                                                                           \
  }
HOOK_CB(cbA, 0)
HOOK_CB(cbB, 1)
HOOK_CB(cbC, 0)
HOOK_CB(cbD, 1)
static unsigned cbE(struct modem *m, int id)
{
  emit(0, "cbE %d %s", id, m == g_modem ? "ctx" : "BADCTX");
  cov_hit(CV_B_CB124);
  return g_cb124_ret[0];
}
static unsigned cbF(struct modem *m, int id)
{
  emit(0, "cbF %d %s", id, m == g_modem ? "ctx" : "BADCTX");
  cov_hit(CV_B_CB124);
  return g_cb124_ret[1];
}
static BeepHookFn const g_h11c[3] = {NULL, cbA, cbB};
static BeepHookFn const g_h120[3] = {NULL, cbC, cbD};
static VoiceParamFn const g_h124[3] = {NULL, cbE, cbF};
static void cb_register(void)
{
  g_cbaddr[1] = (unsigned)(uintptr_t)cbA;
  g_cbaddr[2] = (unsigned)(uintptr_t)cbB;
  g_cbaddr[3] = (unsigned)(uintptr_t)cbC;
  g_cbaddr[4] = (unsigned)(uintptr_t)cbD;
  g_cbaddr[5] = (unsigned)(uintptr_t)cbE;
  g_cbaddr[6] = (unsigned)(uintptr_t)cbF;
}
static const char *cbn(const void *fn) { return cbname((unsigned)(uintptr_t)fn); }

/* the debug level hook: lowers dsplibs_debug_level when a message contains the pattern */
static int g_hook_mode;
static const char *g_hook_pat;
static unsigned g_hook_to;
static BeepGen *g_hook_obj; /* a hook that rewrites the object from inside a message (the blob re-reads the member after the printf) */
static const char *g_hook_objpat;
static int g_hook_setf1, g_hook_cur = -1;
static unsigned g_hook_f1;
static void dbg_hook(const char *msg)
{
  if (g_hook_obj && g_hook_objpat && strstr(msg, g_hook_objpat)) {
    if (g_hook_setf1) putb(&g_hook_obj->f1, g_hook_f1);
    if (g_hook_cur >= 0) g_hook_obj->cur = g_hook_cur;
    cov_hit(CV_S_OBJHOOK);
  }
  if (g_hook_mode && g_hook_pat && strstr(msg, g_hook_pat)) {
    dsplibs_debug_level = g_hook_to;
    cov_hit(CV_B_HOOKS);
  }
}

#ifndef TEST_BLOB
/* R15's detector (voice_tone.c, linked by the port build for notch / voice_getgain) needs the cadence functions; the blob has its own */
Cadence *cadence_create(Cadence *c, CadenceParams *p, int extra, struct modem *m)
{
  (void)c; (void)p; (void)extra; (void)m;
  abort();
}
int cadence_progress(Cadence *c, short s)
{
  (void)c; (void)s;
  abort();
}
void cadence_delete(Cadence *c)
{
  (void)c;
  abort();
}
#endif

/* float helpers built from integers only (exact, identical on x87 and SSE) */
static unsigned fbits_int(int v) /* the bits of (float)v, v < 2^24 in magnitude */
{
  float f = (float)v;
  unsigned u;
  memcpy(&u, &f, 4);
  return u;
}
/* the bits of v * num / den (v a positive normal float, rounded to nearest by integer arithmetic) */
static unsigned scale_bits(unsigned v, unsigned num, unsigned den)
{
  unsigned long long m = (v & 0x7fffffu) | 0x800000u, p = m * num, q, r;
  int e = (int)((v >> 23) & 0xff);
  q = p / den;
  r = p % den;
  if (r * 2 >= den) q++;
  while (q >= (1ull << 24)) {
    q = (q + 1) >> 1;
    e++;
  }
  while (q < (1ull << 23)) {
    q <<= 1;
    e--;
  }
  if (q >= (1ull << 24)) { q >>= 1; e++; }
  return ((unsigned)e << 23) | (unsigned)(q & 0x7fffffu);
}

/* ---------------------------------------------------------------------------------------------- scenario 0: dtmf_test and the tables */
static void dt_one(Acc *a, const unsigned eb8[8], unsigned ebb, int mode, int show)
{
  float e[8];
  int i, r;
  for (i = 0; i < 8; i++) putb(&e[i], eb8[i]);
  r = DTEST(e, ebb, mode);
  acc_u32(a, (unsigned)r);
  for (i = 0; i < 8; i++) acc_u32(a, bf(e[i]));
  if (r >= 0 && r <= 15) cov_hit(CV_T_CODE0 + r);
  else if (r == -1) cov_hit(CV_T_M1);
  else cov_hit(CV_T_ODD);
  if (mode == 1) cov_hit(CV_T_MODE1);
  if (mode != 0 && mode != 1) cov_hit(CV_T_MODEX);
  if (show || g_verbose)
    emit(0, "DT %08x %08x %08x %08x %08x %08x %08x %08x eb=%08x m=%d -> %d", eb8[0], eb8[1], eb8[2], eb8[3], eb8[4], eb8[5], eb8[6], eb8[7], ebb, mode, r);
}
static const short mode_pool[] = {0, 1, 0, 1, 0, 1, 2, -1, 0x7fff, -32768, 257, 3};
static short rmode(void) { return mode_pool[rmod((unsigned)(sizeof mode_pool / sizeof mode_pool[0]))]; }
static unsigned pos_bits(unsigned lo, unsigned hi) { return rflb(lo, hi) & 0x7fffffffu; }
/* a random int of random magnitude: the two draws are SEQUENCED (an expression `rnd() >> rmod(32)` has an unspecified evaluation order: it differed between build flags) */
static int rwild(void)
{
  unsigned v = rnd();
  unsigned sh = rmod(32);
  return (int)(v >> sh);
}
/* e[lo] and e[4 + hi] get the (small) tone values, the other six hover around the sum of the two (what the decision wants) */
static void gen_pattern(unsigned out[8], int lo, int hi, unsigned elo, unsigned ehi)
{
  unsigned z = scale_bits(elo, 1, 1), k;
  int i;
  /* z = elo + ehi in float: use the larger one scaled by 1 + ratio via integer arithmetic (approximate, the harness only needs "around") */
  {
    unsigned a = elo > ehi ? elo : ehi, b = elo > ehi ? ehi : elo;
    int ea = (int)((a >> 23) & 0xff), eb = (int)((b >> 23) & 0xff);
    int d = ea - eb;
    if (d > 20) z = a;
    else {
      unsigned long long ma = (a & 0x7fffffu) | 0x800000u, mb = ((b & 0x7fffffu) | 0x800000u) >> d, s = ma + mb;
      int e = ea;
      if (s >= (1ull << 24)) { s >>= 1; e++; }
      z = ((unsigned)e << 23) | (unsigned)(s & 0x7fffffu);
    }
  }
  for (i = 0; i < 8; i++) {
    k = rmod(100);
    if (k < 60) out[i] = scale_bits(z, 70 + rmod(60), 100);
    else if (k < 85) out[i] = scale_bits(z, 40 + rmod(130), 100);
    else out[i] = scale_bits(z, 20 + rmod(400), 100);
  }
  out[lo] = elo;
  out[4 + hi] = ehi;
}
/* the exact boundary constructions (powers of two make the products with the blob's float constants EXACT in extended precision): cond 0: e_lo * 7.94 vs e_hi, 1: e_hi * 2.82 vs e_lo,
 * 2: max(low) * 0.96 vs e_lo, 3: max(high) * 0.96 vs e_hi, 4: eb vs the threshold */
static void gen_boundary(unsigned out[8], unsigned *ebb, int mode, int cond, int delta)
{
  int lo = (int)rmod(4), hi = (int)rmod(4), a = (int)rmod(20) - 24, i;
  unsigned p2 = (unsigned)(127 + a) << 23, elo = p2, ehi = p2, big;
  *ebb = (unsigned)(127 - 2 + (int)rmod(3)) << 23;
  switch (cond) {
    case 0: ehi = 0x40fe147bu + ((unsigned)a << 23) + (unsigned)delta; break; /* 7.94f * 2^a */
    case 1: elo = 0x40347ae1u + ((unsigned)a << 23) + (unsigned)delta; break; /* 2.82f * 2^a */
    case 2: elo = 0x3f75c28fu + ((unsigned)a << 23) + (unsigned)delta; break; /* 0.96f * 2^a (the maximum of the low group is 2^a) */
    case 3: ehi = 0x3f75c28fu + ((unsigned)a << 23) + (unsigned)delta; break;
    default: break;
  }
  gen_pattern(out, lo, hi, elo, ehi);
  if (cond == 2) {
    /* three low entries equal to the maximum 2^a (>= e_lo) */
    for (i = 0; i < 4; i++)
      if (i != lo) out[i] = p2;
    big = scale_bits(p2, 300, 100);
    for (i = 4; i < 8; i++)
      if (i != 4 + hi) out[i] = big;
  } else if (cond == 3) {
    for (i = 4; i < 8; i++)
      if (i != 4 + hi) out[i] = p2;
    big = scale_bits(p2, 300, 100);
    for (i = 0; i < 4; i++)
      if (i != lo) out[i] = big;
  }
  if (cond == 4) {
    unsigned th = (mode == 1) ? 0x3b102de0u : 0x3b03126eu;
    *ebb = th + (unsigned)delta;
  }
  cov_hit(CV_T_BOUND);
}
static void scen0(void)
{
  Acc a;
  unsigned i, j, eb8[8], ebb;
  int k, cond, delta, mode;
  extern const float us_coef[32], eur_coef[32], biascoef[4];
  /* the tables (the blob's GLOBALs and the port's) */
  acc_init(&a);
  acc_bytes(&a, us_coef, 128);
  acc_bytes(&a, eur_coef, 128);
  acc_bytes(&a, biascoef, 16);
  acc_emit("tables", &a);
  cov_hit(CV_T_TABLES);
  /* structured digit patterns */
  acc_init(&a);
  for (i = 0; i < SW(300000u); i++) {
    int lo = (int)rmod(4), hi = (int)rmod(4);
    unsigned elo = pos_bits(0x68, 0x7c), ehi = rmod(3) ? scale_bits(elo, 20 + rmod(300), 100) : pos_bits(0x68, 0x7c);
    gen_pattern(eb8, lo, hi, elo, ehi);
    ebb = rmod(8) ? (0x3b000000u + (rnd() & 0x00ffffffu)) & 0x7fffffffu : pos_bits(0x60, 0x7e);
    dt_one(&a, eb8, ebb, rmod(2) ? 1 : (int)rmode(), 0);
  }
  acc_emit("dtmf_test structured", &a);
  /* boundary constructions: each condition at the exact boundary and +-3 ulps, both thresholds, every group position */
  acc_init(&a);
  for (i = 0; i < SW(120000u); i++) {
    cond = (int)rmod(5);
    delta = (int)rmod(7) - 3;
    mode = rmod(2) ? 1 : 0;
    gen_boundary(eb8, &ebb, mode, cond, delta);
    if (cond == 4) cov_hit(CV_T_EBLOW);
    dt_one(&a, eb8, ebb, mode, 0);
    /* the same inputs with one entry (the tone filters excluded) replaced by a larger / smaller value */
    if (rmod(4) == 0) {
      {
        unsigned dst = rmod(8), src = rmod(8), pct = 50 + rmod(300); /* sequenced draws */
        eb8[dst] = scale_bits(eb8[src] | 0x3f800000u, pct, 100);
      }
      dt_one(&a, eb8, ebb, mode, 0);
    }
  }
  acc_emit("dtmf_test boundaries", &a);
  /* random mixtures and the special values */
  acc_init(&a);
  for (i = 0; i < SW(300000u); i++) {
    for (k = 0; k < 8; k++) eb8[k] = rmixb();
    ebb = rmixb();
    if (rmod(3) == 0) {
      /* make the decision interesting: tame positive energies with a few specials */
      for (k = 0; k < 8; k++)
        if (rmod(4)) eb8[k] = pos_bits(0x70, 0x78);
      ebb = pos_bits(0x70, 0x7a);
    }
    dt_one(&a, eb8, ebb, rmode(), 0);
  }
  acc_emit("dtmf_test random", &a);
  acc_init(&a);
  for (i = 0; i < NSPECIAL; i++)
    for (j = 0; j < NSPECIAL; j++) {
      for (k = 0; k < 8; k++) eb8[k] = pos_bits(0x70, 0x78);
      eb8[rmod(8)] = special_bits[i];
      eb8[rmod(8)] = special_bits[j];
      dt_one(&a, eb8, special_bits[(i + j) % NSPECIAL], (int)rmode(), 0);
      for (k = 0; k < 8; k++) eb8[k] = special_bits[(i * 5 + (unsigned)k * j) % NSPECIAL];
      dt_one(&a, eb8, special_bits[j], (int)(i & 1), 0);
      for (k = 0; k < 8; k++) eb8[k] = special_bits[i];
      dt_one(&a, eb8, special_bits[j], 0, 0);
      cov_hit(CV_T_SPECIAL);
      if ((special_bits[i] & 0x7f800000u) == 0x7f800000u && (special_bits[i] & 0x7fffffu)) cov_hit(CV_T_NANIN);
    }
  /* all equal / zeros */
  for (j = 0; j < 40; j++) {
    for (k = 0; k < 8; k++) eb8[k] = j < 20 ? 0u : (0x3c000000u + (j << 20));
    dt_one(&a, eb8, 0x3c000000u, (int)(j & 1), 0);
  }
  /* values around the "no minimum yet" start value (0x7e967699) and the region above it: a filter group whose energies are ALL >= it never updates the minimum index (esi / ebx keep their
   * initial 0), and a decision on such energies still passes (the energies are within the blob's ratios): the code is then esi + 4 * ebx - 16 = -16 .. -1 */
  for (j = 0; j < SW(60000u); j++) {
    static const unsigned bigs[] = {0x7e967699u, 0x7e967698u, 0x7e96769au, 0x7e967697u, 0x7e96769bu, 0x7f000000u, 0x7e967700u, 0x7f400000u, 0x7e000000u, 0x7f7fffffu, 0x7f800000u};
    unsigned pat = rmod(4);
    for (k = 0; k < 8; k++) {
      unsigned c = rmod(100);
      eb8[k] = pat == 0 ? bigs[rmod(5)] : (pat == 1 ? 0x7e967699u + (rmod(2) ? 0u : rmod(0x300000u)) : (c < 70 ? bigs[rmod(6)] : bigs[rmod(11)]));
      if (pat == 3 && (k == 0 || k == 4)) eb8[k] = 0x7e967699u + rmod(3) - 1u; /* tone entries at the start value (+-1) */
      if (pat == 3 && k != 0 && k != 4) {
        unsigned sb = scale_bits(0x7e967699u, 190 + rmod(120), 100);
        eb8[k] = sb < 0x7f800000u ? sb : 0x7f000000u;
      }
    }
    {
      unsigned ebv = rmod(2) ? 0x3c000000u : rmixb();
      int mv = rmod(2) ? 1 : (int)rmode();
      dt_one(&a, eb8, ebv, mv, 0);
    }
  }
  acc_emit("dtmf_test specials", &a);
  trial_end("s0");
}

/* ---------------------------------------------------------------------------------------------- scenario 1: create_dtmf, dtmf_detect */
static void dt_hash(Acc *a, const Dtmf *d) { acc_bytes(a, d, 0x98); }
static void dt_line(const char *tag, const Dtmf *d, int ret)
{
  if (!g_verbose) return;
  emit(0, "%s ret=%d sub=%u cnt=%u busy=%u code=%04x easy=%u h=%04x %04x %04x %04x %04x %04x %04x %04x eb=%08x e=%08x %08x %08x %08x %08x %08x %08x %08x", tag, ret, d->sub, d->cnt, d->busy,
       d->code, d->easy, d->hist[0], d->hist[1], d->hist[2], d->hist[3], d->hist[4], d->hist[5], d->hist[6], d->hist[7], bf(d->eb), bf(d->e[0]), bf(d->e[1]), bf(d->e[2]), bf(d->e[3]),
       bf(d->e[4]), bf(d->e[5]), bf(d->e[6]), bf(d->e[7]));
}
/* a plausible random state (every member, in the blob's order, the unused bytes too) */
static void dt_fill(Dtmf *d, int level)
{
  int i;
  unsigned char *p = (unsigned char *)d;
  for (i = 0; i < 0x98; i++) p[i] = (unsigned char)rnd(); /* garbage everywhere first (also the never touched bytes) */
  for (i = 0; i < 16; i++) {
    unsigned k = rmod(100);
    putb(&d->st[i >> 1][i & 1], k < 70 ? rflb(0x68, 0x7c) : (k < 85 ? 0u : rmixb()));
  }
  for (i = 0; i < 8; i++) putb(&d->e[i], rmod(10) ? pos_bits(0x60, 0x7c) : rmixb());
  putb(&d->eb, rmod(10) ? pos_bits(0x60, 0x7c) : rmixb());
  for (i = 0; i < 8; i++) d->hist[i] = (unsigned short)(rmod(4) == 0 ? 0xffffu : rmod(level ? 16u : 3u));
  d->cnt = (unsigned short)(rmod(2) ? rmod(50) : rnd());
  d->sub = (unsigned short)(rmod(2) ? rmod(4) : rnd());
  putb(&d->bias[0], rmod(4) ? rflb(0x68, 0x7c) : rmixb());
  putb(&d->bias[1], rmod(4) ? rflb(0x68, 0x7c) : rmixb());
  d->busy = (unsigned short)(rmod(2) ? 0u : (rmod(3) ? 1u : rnd()));
  d->code = (unsigned short)(rmod(2) ? 0xffffu : rmod(16));
  d->easy = (unsigned short)(rmod(3) == 0 ? 1u : (rmod(8) == 0 ? rnd() : 0u));
}
/* an x input from integer classes */
static unsigned x_bits(int kind)
{
  switch (kind) {
    case 0: return 0u;
    case 1: return 0x80000000u;
    case 2: return rflb(0x68, 0x7e);
    case 3: return fbits_int((int)rmod(2001) - 1000) | 0u; /* exact small integers */
    case 4: return rspecialb();
    default: return rmixb();
  }
}
static void scen1(void)
{
  Acc a;
  unsigned i, j, lvl;
  int k, mode, ret;
  Dtmf *d;
  Dtmf *dd;
  g_dbg_hook = dbg_hook;
  /* create_dtmf: NULL (allocates), failures (with / without the text), existing dirty objects */
  acc_init(&a);
  for (k = 0; k < 6; k++) {
    lvl = lvl_pool[k];
    dsplibs_debug_level = lvl;
    d = create_dtmf(NULL);
    if (!d) die3("create_dtmf(NULL) failed");
    cov_hit(CV_D_CREATE_NULL);
    dt_hash(&a, d);
    dt_line("CREATE", d, 0);
    g_fail_malloc = 0;
    dd = create_dtmf(NULL);
    if (dd) die3("create_dtmf: failing malloc returned an object");
    cov_hit(CV_D_CREATE_FAIL);
    if (lvl > 1) cov_hit(CV_D_CREATE_FAIL_DBG);
    acc_u32(&a, (unsigned)(dd == NULL));
    g_fail_malloc = -1;
    sysdep_free(d);
    g_malloc_ctr = 0;
  }
  dsplibs_debug_level = 0;
  for (j = 0; j < SC(400u); j++) {
    Dtmf obj;
    dt_fill(&obj, 1);
    d = create_dtmf(&obj);
    if (d != &obj) die3("create_dtmf(existing) returned another pointer");
    dt_hash(&a, d);
    cov_hit(CV_D_CREATE_EXIST);
  }
  acc_emit("create_dtmf", &a);
  /* a text lowering hook inside the failure message: nothing after it */
  g_hook_mode = 1;
  g_hook_pat = "could not allocate";
  g_hook_to = 0;
  dsplibs_debug_level = 2;
  g_fail_malloc = 0;
  (void)create_dtmf(NULL);
  g_fail_malloc = -1;
  g_malloc_ctr = 0;
  g_hook_mode = 0;
  dsplibs_debug_level = 0;
  /* dtmf_detect from a fresh object: sample after sample (every second one evaluates), signals of integer classes, both modes */
  acc_init(&a);
  for (j = 0; j < SC(300u); j++) {
    Dtmf obj;
    unsigned kind = rmod(6);
    mode = rmod(2) ? 1 : (int)rmode();
    memset(&obj, 0xa5, sizeof obj); /* the bytes create_dtmf never writes (w80, pad96) must not hold stack garbage: they are hashed */
    (void)create_dtmf(&obj);
    if (rmod(2)) obj.easy = 1;
    for (i = 0; i < 400; i++) {
      unsigned xb = kind == 3 ? (fbits_int((int)((i * 37u + j * 11u) % 41u) - 20) & 0xffffffffu) : x_bits((int)(kind == 4 ? rmod(6) : kind));
      ret = DETECT(xb, &obj, mode);
      acc_u32(&a, (unsigned)ret);
      acc_bytes(&a, &obj, 0x98);
      if (ret == -2) cov_hit(CV_D_SKIP);
      if (ret == -1) cov_hit(CV_D_NONE);
      if (ret >= 0) cov_hit(CV_D_RELEASE);
      if (mode == 1) cov_hit(CV_D_PREFILTER);
      if (kind >= 4) cov_hit(CV_D_NANX);
      if (g_verbose && i < 12) dt_line("SEQ", &obj, ret);
    }
  }
  acc_emit("dtmf_detect sequences", &a);
  /* random states: a call that evaluates (sub == 1) from every kind of object state, x from the integer classes */
  acc_init(&a);
  for (j = 0; j < SW(150000u); j++) {
    Dtmf obj;
    int calls = 1 + (int)rmod(3);
    dt_fill(&obj, rmod(2));
    mode = (int)rmode();
    if (rmod(5)) {
      obj.sub = 1;
      obj.cnt = (unsigned short)((mode == 1 ? 45 : 40) - 1 - (rmod(8) == 0 ? rmod(2) : 0u));
    }
    for (k = 0; k < calls; k++) {
      unsigned xb = x_bits((int)rmod(7));
      ret = DETECT(xb, &obj, mode);
      acc_u32(&a, (unsigned)ret);
      dt_hash(&a, &obj);
      if (g_verbose) dt_line("RS", &obj, ret);
      if (ret == -2) cov_hit(CV_D_SKIP);
      else cov_hit(CV_D_BLOCK);
    }
  }
  acc_emit("dtmf_detect random states", &a);
  /* preloaded energies: x = 0 with zero filter states leaves e / eb as loaded, so dtmf_test sees the pattern and the history / busy / easy logic is driven exactly */
  acc_init(&a);
  for (j = 0; j < SW(150000u); j++) {
    Dtmf obj;
    unsigned eb8[8], ebb;
    int lo = (int)rmod(4), hi = (int)rmod(4), code = lo + 4 * hi, mm, h;
    unsigned cond = rmod(6);
    mm = rmod(2) ? 1 : (int)rmode();
    memset(&obj, 0, sizeof obj);
    if (cond < 5 && rmod(2)) {
      gen_boundary(eb8, &ebb, mm == 1, (int)cond, (int)rmod(5) - 2);
    } else {
      unsigned elo = pos_bits(0x68, 0x7c);
      gen_pattern(eb8, lo, hi, elo, scale_bits(elo, 20 + rmod(300), 100));
      ebb = pos_bits(0x70, 0x7a);
    }
    for (h = 0; h < 8; h++) putb(&obj.e[h], eb8[h]);
    putb(&obj.eb, ebb);
    /* the code the detector will compute: found by running dtmf_test itself (both sides compute their own, they agree or the test fails) */
    {
      float e[8];
      for (h = 0; h < 8; h++) putb(&e[h], eb8[h]);
      code = DTEST(e, ebb, mm);
      if (code < 0 || code > 15) code = (int)rmod(16);
    }
    for (h = 0; h < 8; h++) {
      unsigned r = rmod(100);
      obj.hist[h] = (unsigned short)(r < 35 ? (unsigned)code : (r < 45 ? 0xffffu : (r < 70 ? (unsigned)((code + 1) & 15) : rmod(16))));
    }
    if (rmod(2)) obj.hist[0] = obj.hist[1] = (unsigned short)code; /* (the history shifts before the new result is stored) */
    if (rmod(2)) obj.hist[2] = obj.hist[3] = obj.hist[4] = obj.hist[5] = (unsigned short)((code + 5) & 15);
    obj.busy = (unsigned short)(rmod(3) == 0 ? (rmod(2) ? 1u : 0x8000u) : 0u);
    obj.code = (unsigned short)(obj.busy ? (rmod(2) ? (unsigned)code : 0xffffu) : (rmod(2) ? 0xffffu : rmod(16)));
    obj.easy = (unsigned short)(rmod(2) ? 1u : 0u);
    obj.sub = 1;
    obj.cnt = (unsigned short)((mm == 1 ? 45 : 40) - 1);
    ret = DETECT(rmod(2) ? 0u : 0x80000000u, &obj, mm);
    acc_u32(&a, (unsigned)ret);
    dt_hash(&a, &obj);
    dt_line("PRE", &obj, ret);
    if (ret == -1) cov_hit(CV_D_NONE);
    if (ret >= 0) cov_hit(CV_D_RELEASE);
    cov_hit(CV_D_BLOCK);
  }
  /* the valid / easy / busy transitions are counted from a second pass through the same generator (the results are in the hashes above) */
  for (j = 0; j < 4000; j++) {
    Dtmf obj;
    float e[8];
    unsigned eb8[8], ebb;
    int lo = (int)rmod(4), hi = (int)rmod(4), code, h, mm = (int)(j & 1);
    unsigned elo = pos_bits(0x68, 0x7c);
    gen_pattern(eb8, lo, hi, elo, scale_bits(elo, 50 + rmod(200), 100));
    ebb = pos_bits(0x70, 0x7a);
    for (h = 0; h < 8; h++) putb(&e[h], eb8[h]);
    code = DTEST(e, ebb, mm);
    if (code < 0) continue;
    memset(&obj, 0, sizeof obj);
    for (h = 0; h < 8; h++) putb(&obj.e[h], eb8[h]);
    putb(&obj.eb, ebb);
    for (h = 0; h < 8; h++) obj.hist[h] = 0xffff;
    obj.sub = 1;
    obj.cnt = (unsigned short)((mm == 1 ? 45 : 40) - 1);
    obj.code = 0xffff;
    switch (j % 5) {
      case 0: obj.hist[0] = obj.hist[1] = (unsigned short)code; break;                     /* valid */
      case 1: obj.hist[0] = (unsigned short)code; obj.easy = 1; break;                       /* easy only */
      case 2: obj.busy = 1; obj.code = (unsigned short)code; break;                          /* held, tone still there */
      case 3: obj.busy = 1; obj.code = (unsigned short)code; code = -1; break;               /* held, tone gone (see below) */
      default: obj.hist[0] = obj.hist[1] = (unsigned short)code; obj.hist[3] = (unsigned short)code; break;
    }
    if (j % 5 == 3) { /* tone gone: energies that fail the decision */
      for (h = 0; h < 8; h++) putb(&obj.e[h], 0x3f800000u);
      putb(&obj.eb, 0u);
    }
    ret = DETECT(0u, &obj, mm);
    acc_u32(&a, (unsigned)ret);
    dt_hash(&a, &obj);
    if (obj.busy && ret == -1 && j % 5 == 0) cov_hit(CV_D_VALID);
    if (obj.busy && ret == -1 && j % 5 == 1) cov_hit(CV_D_EASY);
    if (j % 5 == 2 && ret == -1) cov_hit(CV_D_BUSY_HOLD);
    if (j % 5 == 3 && ret >= 0) cov_hit(CV_D_BUSY_NONE);
  }
  acc_emit("dtmf_detect preloaded", &a);
  trial_end("s1");
}

/* ---------------------------------------------------------------------------------------------- scenario 2: dtmf_progress over synthetic DTMF signals */
/* integer oscillators y[n] = (2 c y[n-1] >> 30) - y[n-2] (c = cos(w) in Q30, y[1] = 4096 sin(w)): NO libm (sin / cos differ between the x87 and the SSE libm) */
static const int osc_tab[24][2] = {
    {916834755, 2132}, {883303530, 2329}, {842182412, 2541}, {793605035, 2759}, {624973589, 3331}, {534922184, 3552}, {428754983, 3755}, {305228673, 3927},
    {910661169, 2170}, {875855160, 2369}, {833193109, 2584}, {782828008, 2803}, {608280628, 3375}, {515267803, 3594}, {405802355, 3792}, {278724757, 3956},
    {922898443, 2094}, {890622681, 2288}, {851020874, 2498}, {804208675, 2714}, {641441156, 3285}, {554340990, 3508}, {451476835, 3716}, {331531766, 3896}};
typedef struct {
  int c, y1, y2;
} Osc;
static void osc_init(Osc *o, int idx)
{
  o->c = osc_tab[idx][0];
  o->y2 = 0;
  o->y1 = osc_tab[idx][1];
}
static int osc_next(Osc *o) /* returns the sample AT n then advances */
{
  int out = o->y2;
  int next = (int)(((long long)2 * o->c * o->y1) >> 30) - o->y2;
  o->y2 = o->y1;
  o->y1 = next;
  return out;
}
static void scen2(void)
{
  Acc a;
  unsigned j;
  int k;
  Dtmf *d;
  static const int amp_div[] = {1, 2, 4, 8, 16, 32};
  d = create_dtmf(NULL);
  if (!d) die3("create_dtmf");
  acc_init(&a);
  for (j = 0; j < SC(900u); j++) {
    int mode = (int)(j & 1);
    unsigned seq = 1 + rmod(4), s;
    int variant = (int)rmod(3); /* 0 exact, 1 +2 %, 2 -2 % */
    unsigned digits_seen = 0;
    static unsigned all_digits;
    (void)create_dtmf(d);
    if (rmod(4) == 0) d->easy = 1;
    for (s = 0; s < seq; s++) {
      int lo = (int)rmod(4), hi = (int)rmod(4), shift = amp_div[rmod(6)];
      unsigned len = 160u + rmod(520u), gap = 200u + rmod(500u), n, pos = 0;
      Osc olo, ohi;
      float buf[640];
      int two_low = rmod(8) == 0, none_hi = rmod(10) == 0, sh2 = 0;
      while ((1 << sh2) < shift) sh2++;
      osc_init(&olo, variant * 8 + lo);
      osc_init(&ohi, variant * 8 + 4 + hi);
      if (two_low) osc_init(&ohi, variant * 8 + (hi + 1) % 4);
      for (n = 0; n < len + gap;) {
        unsigned chunk = rmod(6) == 0 ? 100 + rmod(500) : 1 + rmod(37); /* some chunks hold several blocks (80 samples each): the result is the LAST digit */
        unsigned c;
        int ret;
        short cnt;
        if (n + chunk > len + gap) chunk = len + gap - n;
        for (c = 0; c < chunk; c++) {
          int v = 0;
          if (n + c < len) {
            v = osc_next(&olo);
            if (!none_hi) v += osc_next(&ohi);
            v *= 4; /* the oscillators have amplitude 4096: x = v / 2^16 */
          } else {
            v = (int)(rnd() & 0xff) - 128;
          }
          {
            unsigned bb = fbits_int(v);
            if (bb & 0x7fffffffu) bb -= (unsigned)(16 + sh2) << 23; /* v * 2^-(16 + sh2): an exponent change, exact for a normal float */
            putb(&buf[c], bb);
          }
        }
        cnt = (short)chunk;
        ret = dtmf_progress(d, buf, cnt, mode);
        acc_u32(&a, (unsigned)ret);
        acc_bytes(&a, d, 0x98);
        cov_hit(CV_D_PROGRESS);
        if (mode == 1) cov_hit(CV_D_MODE1);
        if (ret >= 0) {
          cov_hit(CV_D_DIGIT_RET);
          digits_seen |= 1u << ret;
          all_digits |= 1u << ret;
          if (g_verbose) emit(0, "DIGIT %d mode=%d at %u", ret, mode, n);
        }
        n += chunk;
        pos++;
      }
      /* count <= 0 */
      (void)pos;
      k = dtmf_progress(d, buf, 0, mode);
      acc_u32(&a, (unsigned)k);
      k = dtmf_progress(d, buf, -5, mode);
      acc_u32(&a, (unsigned)k);
      cov_hit(CV_D_NEG_COUNT);
    }
    (void)digits_seen;
    if (all_digits == 0xffffu) cov_hit(CV_D_ALL_DIGITS);
  }
  acc_emit("dtmf_progress signals", &a);
  /* white noise, specials in the stream, big amplitudes */
  acc_init(&a);
  for (j = 0; j < SC(400u); j++) {
    float buf[40];
    int n, ret, mode = (int)rmode();
    (void)create_dtmf(d);
    for (n = 0; n < 1000; n += 40) {
      int c;
      unsigned kind = rmod(5);
      for (c = 0; c < 40; c++) putb(&buf[c], kind == 0 ? rflb(0x68, 0x7e) : (kind == 1 ? rmixb() : (kind == 2 ? (rmod(30) ? rflb(0x70, 0x7a) : rspecialb()) : (kind == 3 ? fbits_int((int)rmod(65535) - 32768) : 0u))));
      ret = dtmf_progress(d, buf, 40, mode);
      acc_u32(&a, (unsigned)ret);
      acc_bytes(&a, d, 0x98);
      cov_hit(CV_D_PROGRESS);
    }
  }
  acc_emit("dtmf_progress noise", &a);
  sysdep_free(d);
  trial_end("s2");
}

/* ---------------------------------------------------------------------------------------------- BeepGen helpers */
static unsigned q_freq_bits(void)
{
  unsigned k = rmod(100);
  if (k < 14) return 0u;
  if (k < 28) return 0xbf800000u;                                        /* -1.0 */
  if (k < 70) return fbits_int((int)(300 + rmod(1800)));                  /* a frequency */
  if (k < 78) return 0x7f800001u | (rnd() & 0x3fffffu);                   /* signalling NaN */
  if (k < 84) return 0x7fc00000u | (rnd() & 0x3fffffu);                   /* quiet NaN */
  if (k < 90) return rspecialb();
  if (k < 93) { static const unsigned nb[] = {0xbfc00000u, 0xbf800001u, 0xbf7fffffu, 0xc0000000u, 0xbf000000u, 0xc00fffffu, 0xbfffffffu}; return nb[rmod(7)]; } /* around -1, between -2 and 0 */
  return fbits_int(-(int)rmod(5000));
}
static int q_dur(void)
{
  unsigned k = rmod(100);
  if (k < 70) return (int)(1 + rmod(4));
  if (k < 80) return 0;
  if (k < 88) return -(int)rmod(20);
  return (int)rnd();
}
/* every member of a BeepGen from the PRNG in a fixed order (wild = floats of every class) */
static void bg_fill(BeepGen *b, int wild)
{
  int i;
  unsigned k;
  b->ctx = g_modem;
  putb(&b->phase1, wild ? (rmod(4) ? rmixb() : 0x7f800000u) : (rmod(3) ? rflb(0x68, 0x7f) : 0u));
  putb(&b->phase2, wild ? rmixb() : (rmod(3) ? rflb(0x68, 0x7f) : 0u));
  putb(&b->f1, wild ? q_freq_bits() : (rmod(2) ? fbits_int((int)(300 + rmod(1800))) : q_freq_bits()));
  putb(&b->f2, wild ? q_freq_bits() : fbits_int((int)(300 + rmod(1800))));
  putb(&b->gain1, wild ? rmixb() : rflb(0x60, 0x7e));
  putb(&b->gain2, wild ? rmixb() : rflb(0x60, 0x7e));
  b->n = (int)rmod(3000);
  b->limit = wild ? rwild() : (int)rmod(3000);
  if (rmod(3) == 0) b->limit = -(int)rmod(5);
  b->count = (int)rmod(BEEP_QMAX + 1);
  b->cur = b->count ? (int)rmod((unsigned)b->count) : 0;
  b->div = rmod(2) ? 10 : 100;
  for (i = 0; i < BEEP_QMAX; i++) {
    putb(&b->q[i].f1, q_freq_bits());
    putb(&b->q[i].f2, q_freq_bits());
    b->q[i].dur = q_dur();
  }
  k = rmod(3);
  b->hookOn = g_h11c[k];
  k = rmod(3);
  b->hookOff = g_h120[k];
  k = 1 + rmod(2); /* never NULL: a '!' digit calls it (the blob crashes on NULL: the guarded child tests that) */
  b->getParam = g_h124[k];
  if (wild && rmod(3) == 0) {
    /* near a limit crossing */
    b->n = b->limit - (int)rmod(3);
  }
}
/* the members create initialises */
static void bg_hash_init(Acc *a, const BeepGen *b)
{
  acc_u32(a, b->ctx == g_modem);
  acc_u32(a, bf(b->phase1));
  acc_u32(a, bf(b->phase2));
  acc_u32(a, bf(b->f1));
  acc_u32(a, bf(b->f2));
  acc_u32(a, bf(b->gain1));
  acc_u32(a, bf(b->gain2));
  acc_u32(a, (unsigned)b->count);
  acc_u32(a, (unsigned)b->cur);
  acc_bytes(a, cbn((const void *)b->hookOn), strlen(cbn((const void *)b->hookOn)));
  acc_bytes(a, cbn((const void *)b->hookOff), strlen(cbn((const void *)b->hookOff)));
  acc_bytes(a, cbn((const void *)b->getParam), strlen(cbn((const void *)b->getParam)));
}
/* all of them (every queue entry too: stray writes behind count are seen) */
static void bg_hash(Acc *a, const BeepGen *b)
{
  int i;
  bg_hash_init(a, b);
  acc_u32(a, (unsigned)b->n);
  acc_u32(a, (unsigned)b->limit);
  acc_u32(a, (unsigned)b->div);
  for (i = 0; i < BEEP_QMAX; i++) {
    acc_u32(a, bf(b->q[i].f1));
    acc_u32(a, bf(b->q[i].f2));
    acc_u32(a, (unsigned)b->q[i].dur);
  }
}
static void bg_line(const char *tag, const BeepGen *b)
{
  if (!g_verbose) return;
  emit(0, "%s p=%08x %08x f=%08x %08x g=%08x %08x n=%d lim=%d cnt=%d cur=%d div=%d q0=%08x %08x %d q1=%08x %08x %d cb=%s %s %s", tag, bf(b->phase1), bf(b->phase2), bf(b->f1), bf(b->f2), bf(b->gain1),
       bf(b->gain2), b->n, b->limit, b->count, b->cur, b->div, bf(b->q[0].f1), bf(b->q[0].f2), b->q[0].dur, bf(b->q[1].f1), bf(b->q[1].f2), b->q[1].dur, cbn((const void *)b->hookOn),
       cbn((const void *)b->hookOff), cbn((const void *)b->getParam));
}
/* after create(NULL): the members it leaves alone get PRNG values (so the following steps do not depend on malloc garbage laid out differently on x86-64) */
static void bg_fill_rest(BeepGen *b)
{
  int i;
  b->n = (int)rmod(3000);
  b->limit = (int)rmod(3000);
  b->div = rmod(2) ? 10 : 100;
  for (i = 0; i < BEEP_QMAX; i++) {
    putb(&b->q[i].f1, q_freq_bits());
    putb(&b->q[i].f2, q_freq_bits());
    b->q[i].dur = q_dur();
  }
}
static void set_params(int wild)
{
  g_mgp[0x14] = wild ? rwild() : (int)rmod(40) - 5;
  g_mgp[0x24] = wild ? -rwild() : (int)rmod(60);
  g_mgp[0x3c] = wild ? rwild() : (int)rmod(30);
}

/* ---------------------------------------------------------------------------------------------- scenario 3: beepgen_create / delete / start_beep / start_dtmf */
static void do_create_existing(Acc *a, unsigned i, unsigned j, unsigned k)
{
  BeepGen obj;
  BeepCfg cfg;
  BeepGen *r;
  cfg.ctx = g_modem;
  cfg.hookOn = g_h11c[i];
  cfg.hookOff = g_h120[j];
  cfg.getParam = g_h124[k];
  bg_fill(&obj, 0);
  r = beepgen_create(&obj, &cfg);
  if (r != &obj) die3("beepgen_create(existing) returned another pointer");
  bg_hash(a, &obj);
  bg_line("CREATE_EXIST", &obj);
  cov_hit(CV_B_CREATE_EXIST);
}
static const int dur_pool[] = {1, 3, 0, -5, 250, 0x7fffffff, 0x40000001, 0x20000000, 7, 100, -0x7fffffff - 1, 8000, 4};
static const int fq_pool[] = {0, -1, 1, 2, 440, 697, 1633, 0x7fffffff, -0x7fffffff - 1, -2, 16777217, 2000, 350, 1209, 8000, -1000};
static const char *const hook_pats[] = {"beepgen_create", "beepgen start dtmf", "start !!!", "digit ','", "start beep", "End of BEEP", "Hook on proc", "Start beep[", "CBPTRS", "could not"};
static void start_one(Acc *a, int kind, int ch, int f2, int dur, int prequeue, int reent)
{
  BeepGen b;
  int i;
  bg_fill(&b, 0);
  b.count = prequeue;
  b.cur = 0;
  if (reent) {
    g_reent_bg = &b;
    g_reent_count = (int)rmod(4);
    g_reent_cur = 0;
    b.hookOn = cbB;
  }
  set_params(rmod(8) == 0);
  for (i = 0; i < 1; i++) {
    if (kind == 0) beepgen_start_dtmf(&b, ch, dur);
    else beepgen_start_beep(&b, ch, f2, dur);
  }
  g_reent_bg = NULL;
  bg_hash(a, &b);
  bg_line(kind == 0 ? "DTMF" : "BEEP", &b);
}
#ifndef TEST_BLOB
static void port_overflow_check(void)
{
  BeepGen b;
  int i;
  bg_fill(&b, 0);
  b.count = 0;
  b.hookOn = cbA;
  b.hookOff = cbC;
  b.getParam = cbE;
  g_quiet = 1; /* the port-only check must not change the hashed output */
  for (i = 0; i < BEEP_QMAX + 5; i++) beepgen_start_beep(&b, 440 + i, 880, 1);
  g_quiet = 0;
  if (b.count != BEEP_QMAX || b.hookOn != cbA || b.hookOff != cbC || b.getParam != cbE) die3("the port's queue overflow guard");
}
#endif
static void guard_bang(void *arg)
{
  BeepGen b;
  (void)arg;
  bg_fill(&b, 0);
  b.getParam = NULL;
  b.count = 0;
  beepgen_start_dtmf(&b, '!', 5);
}
static void guard_create_null_cfg(void *arg)
{
  (void)arg;
  (void)beepgen_create(NULL, NULL);
}
static void scen3(void)
{
  Acc a;
  unsigned i, j, k, lvl, rep;
  int ch, d, f;
  BeepGen *b;
  BeepCfg cfg;
  g_dbg_hook = dbg_hook;
  cb_register();
  /* beepgen_create: every callback combination at the debug levels 0 .. 3, fresh (NULL) and existing objects */
  acc_init(&a);
  for (rep = 0; rep < 6; rep++)
    for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
        for (k = 0; k < 3; k++) {
          lvl = lvl_pool[rep];
          dsplibs_debug_level = lvl;
          set_params(0);
          cfg.ctx = g_modem;
          cfg.hookOn = g_h11c[i];
          cfg.hookOff = g_h120[j];
          cfg.getParam = g_h124[k];
          b = beepgen_create(NULL, &cfg);
          if (!b) die3("beepgen_create(NULL) failed");
          bg_hash_init(&a, b);
          bg_line("CREATE", b);
          cov_hit(CV_B_CREATE_NULL);
          if (lvl > 1) cov_hit(CV_B_DBG);
          if (!i && !j && !k) cov_hit(CV_B_CBNULL);
          bg_fill_rest(b);
          beepgen_delete(b);
          cov_hit(CV_B_DELETE);
          do_create_existing(&a, i, j, k);
        }
  dsplibs_debug_level = 0;
  acc_emit("beepgen_create", &a);
  /* a failing allocation: NULL, with and without the text; the text hooks (the level lowered inside a message) */
  acc_init(&a);
  for (rep = 0; rep < 6; rep += 1) {
    lvl = lvl_pool[rep];
    dsplibs_debug_level = lvl;
    cfg.ctx = g_modem;
    cfg.hookOn = cbA;
    cfg.hookOff = cbC;
    cfg.getParam = cbE;
    g_fail_malloc = 0;
    b = beepgen_create(NULL, &cfg);
    if (b) die3("beepgen_create: failing malloc returned an object");
    acc_u32(&a, 1);
    g_fail_malloc = -1;
    g_malloc_ctr = 0;
    cov_hit(CV_B_CREATE_FAIL);
  }
  for (i = 0; i < sizeof hook_pats / sizeof hook_pats[0]; i++)
    for (k = 0; k < 3; k++) {
      dsplibs_debug_level = 2 + (k == 2);
      g_hook_mode = 1;
      g_hook_pat = hook_pats[i];
      g_hook_to = k == 1 ? 1u : 0u;
      cfg.hookOn = cbA;
      cfg.hookOff = cbC;
      cfg.getParam = cbE;
      set_params(0);
      b = beepgen_create(NULL, &cfg);
      bg_hash_init(&a, b);
      bg_fill_rest(b);
      beepgen_delete(b);
    }
  g_hook_mode = 0;
  dsplibs_debug_level = 0;
  acc_emit("beepgen_create failures / hooks", &a);
  /* start_dtmf: every character x durations x object states (count 0: starts; > 0: queues) */
  acc_init(&a);
  for (rep = 0; rep < SC(3u); rep++)
    for (ch = 0; ch < 256; ch++)
      for (j = 0; j < 4; j++) {
        d = dur_pool[(unsigned)(ch * 3 + (int)j + (int)rep * 5) % (sizeof dur_pool / sizeof dur_pool[0])];
        start_one(&a, 0, ch, 0, d, j == 3 ? (int)(1 + rmod(19)) : 0, 0);
        if (ch == '!') cov_hit(CV_B_DTMF_BANG);
        if (ch == ',') cov_hit(CV_B_DTMF_COMMA);
        if (ch == 'x') cov_hit(CV_B_DTMF_DEFAULT);
        if (j == 3) cov_hit(CV_B_QUEUE);
        if (d == 0x7fffffff || d < 0) cov_hit(CV_B_DURBIG);
      }
  cov_hit(CV_B_DTMF_ALL);
  {
    static const int extra[] = {0x121, 0x12c, 0x1ff, -1, 0x7fffffff, -0x7fffffff - 1, 0x100000b2, 0x1000021, 0x2c00, 0xffff, 0x10030, 0x144};
    for (i = 0; i < sizeof extra / sizeof extra[0]; i++)
      for (j = 0; j < 6; j++) start_one(&a, 0, extra[i], 0, dur_pool[j * 2], j == 5 ? 3 : 0, 0);
  }
  /* '!' asks the callback: cbE / cbF, durations of the callback including 0 / negative / huge */
  for (j = 0; j < 12; j++) {
    unsigned v[] = {500u, 0u, 0xffffffffu, 0x7fffffffu, 0x80000000u, 3u, 1u, 100u, 0x40000001u, 0x20000000u, 8u, 2u};
    g_cb124_ret[0] = v[j];
    g_cb124_ret[1] = v[(j + 5) % 12];
    start_one(&a, 0, '!', 0, 7, (int)(j % 3), 0);
  }
  g_cb124_ret[0] = 500u;
  g_cb124_ret[1] = 3u;
  acc_emit("start_dtmf", &a);
  /* start_beep: frequency pairs x durations; queues up to 20 entries */
  acc_init(&a);
  for (i = 0; i < sizeof fq_pool / sizeof fq_pool[0]; i++)
    for (j = 0; j < sizeof fq_pool / sizeof fq_pool[0]; j++) {
      f = fq_pool[i];
      start_one(&a, 1, f, fq_pool[j], dur_pool[(i * 7 + j) % (sizeof dur_pool / sizeof dur_pool[0])], (int)(rmod(4) == 0 ? 1 + rmod(19) : 0), 0);
      if (i == 0) cov_hit(CV_B_FIRST_ZERO);
      if (i == 1) cov_hit(CV_B_FIRST_M1);
      if (j == 0) cov_hit(CV_B_SECOND_ZERO);
      cov_hit(CV_B_BEEP);
    }
  for (j = 0; j < SC(600u); j++) {
    BeepGen obj;
    int n = 1 + (int)rmod(22), t;
    bg_fill(&obj, 0);
    obj.count = 0;
    cfg.ctx = g_modem;
    set_params(rmod(10) == 0);
    for (t = 0; t < n && t < BEEP_QMAX; t++) {
      int ff1 = rmod(5) ? (int)(300 + rmod(1800)) : fq_pool[rmod((unsigned)(sizeof fq_pool / sizeof fq_pool[0]))];
      int ff2 = rmod(5) ? (int)(300 + rmod(1800)) : fq_pool[rmod((unsigned)(sizeof fq_pool / sizeof fq_pool[0]))];
      {
        int isbeep = (int)rmod(2);
        unsigned ci = rmod(19), di = rmod((unsigned)(sizeof dur_pool / sizeof dur_pool[0])); /* sequenced draws */
        if (isbeep) beepgen_start_beep(&obj, ff1, ff2, dur_pool[di]);
        else beepgen_start_dtmf(&obj, "0123456789*#ABCD,!x"[ci], dur_pool[di]);
      }
      bg_hash(&a, &obj);
    }
    if (obj.count == BEEP_QMAX) cov_hit(CV_B_QUEUE_FULL);
    bg_line("QUEUE", &obj);
  }
  /* callbacks that modify count / cur (the blob reloads them after the call) */
  for (j = 0; j < SC(300u); j++) {
    int fx = (int)(300 + rmod(1000));
    int dx = (int)(1 + rmod(3));
    start_one(&a, 1, -1, fx, dx, 0, 1);
  }
  acc_emit("start_beep", &a);
  /* the text: levels 2 / 3 for a subset, the level lowered inside each message in turn */
  acc_init(&a);
  g_quiet = 0;
  for (rep = 1; rep < 6; rep++)
    for (i = 0; i < sizeof hook_pats / sizeof hook_pats[0]; i++)
      for (k = 0; k < 3; k++) {
        static const char chars[] = "!,5x#AD0";
        unsigned c;
        for (c = 0; c < sizeof chars - 1; c++) {
          dsplibs_debug_level = lvl_pool[rep];
          g_hook_mode = k != 0;
          g_hook_pat = hook_pats[i];
          g_hook_to = k == 2 ? 1u : 0u;
          start_one(&a, 0, chars[c], 0, 5, 0, 0);
          start_one(&a, 1, chars[c] == '!' ? -1 : 440, 880, 4, 0, 0);
          cov_hit(CV_B_DBG);
        }
      }
  g_hook_mode = 0;
  dsplibs_debug_level = 0;
  acc_emit("start text / hooks", &a);
  /* extreme GetGain parameters through the start path */
  acc_init(&a);
  for (j = 0; j < SC(300u); j++) {
    BeepGen obj;
    bg_fill(&obj, 0);
    obj.count = 0;
    set_params(1);
    beepgen_start_beep(&obj, (int)(1 + rmod(2000)), 400, 2);
    bg_hash(&a, &obj);
  }
  acc_emit("start extreme params", &a);
  /* guarded: a '!' digit with a NULL cb124 (the blob calls NULL: SIGSEGV; the port keeps the duration) */
  guarded_report("start_dtmf('!') with a NULL callback", guarded(guard_bang, NULL));
  cov_hit(CV_B_GUARDED);
#ifndef TEST_BLOB
  port_overflow_check();
#endif
  (void)guard_create_null_cfg;
  trial_end("s3");
}

/* ---------------------------------------------------------------------------------------------- scenario 4: beepgen_sample */
static void sample_run(Acc *a, BeepGen *b, int maxn, int hash_every, unsigned *ends)
{
  int i;
  float out;
  for (i = 0; i < maxn; i++) {
    int r;
    putb(&out, 0x7fa5a5a5u); /* an old value: written exactly once per call */
    r = beepgen_sample(b, &out);
    acc_u32(a, bf(out));
    acc_u32(a, (unsigned)r);
    if (r == 0) cov_hit(CV_S_RUN);
    if (r == 1) {
      cov_hit(CV_S_END);
      if (ends) (*ends)++;
      bg_hash(a, b);
      break;
    }
    if (hash_every && (i % hash_every) == 0) bg_hash(a, b);
  }
}
static void scen4(void)
{
  Acc a;
  unsigned j, i;
  BeepCfg cfg;
  BeepGen *b;
  unsigned ends = 0;
  g_dbg_hook = dbg_hook;
  cb_register();
  /* real sequences */
  acc_init(&a);
  for (j = 0; j < SC(260u); j++) {
    int n = 1 + (int)rmod(5), t;
    cfg.ctx = g_modem;
    cfg.hookOn = g_h11c[rmod(3)];
    cfg.hookOff = g_h120[rmod(3)];
    cfg.getParam = g_h124[1 + rmod(2)];
    set_params(0);
    b = beepgen_create(NULL, &cfg);
    if (!b) die3("create");
    bg_hash_init(&a, b);
    bg_fill_rest(b);
    for (t = 0; t < n; t++) {
      unsigned c = rmod(100);
      {
        unsigned ci = rmod(19);
        int fa = (int)(300 + rmod(1800));
        int fb2 = rmod(3) ? (int)(300 + rmod(1800)) : 0;
        int fc = rmod(2) ? 0 : 400;
        int dd = (int)(1 + rmod(3)); /* every draw sequenced (the order of the arguments of a call is unspecified) */
        if (c < 50) beepgen_start_dtmf(b, "0123456789*#ABCD,!x"[ci], dd);
        else if (c < 80) beepgen_start_beep(b, fa, fb2, dd);
        else if (c < 92) beepgen_start_beep(b, -1, fc, dd);
        else beepgen_start_beep(b, 0, fa, dd);
      }
    }
    sample_run(&a, b, 40000, 97, &ends);
    bg_hash(&a, b);
    bg_line("SEQ", b);
    beepgen_delete(b);
  }
  acc_emit("sample sequences", &a);
  /* random states: forced and natural transitions, wild floats, signalling NaNs in the queue, callbacks that rewrite count / cur */
  acc_init(&a);
  for (j = 0; j < SW(40000u); j++) {
    BeepGen obj;
    int calls = 1 + (int)rmod(5), t, wild = (int)rmod(2);
    bg_fill(&obj, wild);
    if (rmod(3)) obj.n = obj.limit - (int)rmod(2); /* the next sample crosses the limit */
    if (wild) cov_hit(CV_S_RANDSTATE);
    if (rmod(10) == 0) {
      obj.count = rmod(2) ? -(int)(1 + rmod(3)) : (int)0x80000000u; /* a negative count: `n >= count` is a SIGNED compare (the queue is then over at once) */
      cov_hit(CV_S_NEGCOUNT);
    }
    set_params(rmod(10) == 0);
    if (rmod(8) == 0) {
      g_reent_bg = &obj;
      g_reent_count = (int)rmod(5);
      g_reent_cur = (int)rmod(4);
    }
    for (t = 0; t < calls; t++) {
      float out;
      int r;
      putb(&out, 0x7fa5a5a5u);
      r = beepgen_sample(&obj, &out);
      acc_u32(&a, bf(out));
      acc_u32(&a, (unsigned)r);
      bg_hash(&a, &obj);
      if (r == 1) cov_hit(CV_S_END);
      if (r == 0) cov_hit(CV_S_RUN);
      if (g_verbose) bg_line("RS", &obj);
      if (obj.n <= obj.limit) {
        /* nothing crossed */
      } else {
        cov_hit(CV_S_LIMIT);
      }
    }
    g_reent_bg = NULL;
  }
  acc_emit("sample random states", &a);
  /* the transition code: a controlled queue, the entry f1 / f2 classes and the callbacks, at every debug level with the level-lowering hooks */
  acc_init(&a);
  for (j = 0; j < SC(2500u); j++) {
    BeepGen obj;
    int t, cnt = 2 + (int)rmod(3);
    bg_fill(&obj, 0);
    obj.count = cnt;
    obj.cur = 0;
    obj.limit = 3;
    obj.n = 3;
    putb(&obj.f1, rmod(3) == 0 ? 0xbf800000u : (rmod(2) ? 0u : q_freq_bits()));
    putb(&obj.f2, q_freq_bits());
    for (t = 0; t < BEEP_QMAX; t++) {
      unsigned c = rmod(100);
      putb(&obj.q[t].f1, c < 25 ? 0u : (c < 50 ? 0xbf800000u : (c < 60 ? (0x7f800001u | (rnd() & 0x3fffffu)) : (c < 65 ? 0x7fc00001u : (c < 70 ? 0x80000000u : fbits_int((int)(300 + rmod(1800))))))));
      c = rmod(100);
      putb(&obj.q[t].f2, c < 30 ? 0u : (c < 40 ? 0x80000000u : (c < 45 ? (0x7f800001u | (rnd() & 0x3fffffu)) : (c < 50 ? 0x7fc00000u : fbits_int((int)(300 + rmod(1800)))))));
      obj.q[t].dur = rmod(5) ? (int)(1 + rmod(3)) : q_dur();
    }
    dsplibs_debug_level = rmod(3) == 0 ? lvl_pool[1 + rmod(5)] : 0u;
    g_hook_mode = dsplibs_debug_level != 0 && rmod(2);
    g_hook_pat = hook_pats[5 + rmod(3)];
    g_hook_to = rmod(2);
    g_hook_obj = NULL;
    g_hook_setf1 = 0;
    g_hook_cur = -1;
    if (rmod(5) == 0) {
      /* a message that rewrites the object: f1 after "End of BEEP" (the blob loads it again after the printf), cur after "Start beep[" (reloaded too) */
      static const unsigned f1v[] = {0xbf800000u, 0u, 0x3f800000u, 0x7fc00000u, 0x7f800001u, 0xbfc00000u};
      dsplibs_debug_level = rmod(2) ? 2u : 3u;
      g_hook_mode = 0;
      g_hook_obj = &obj;
      if (rmod(2)) {
        g_hook_objpat = "End of BEEP";
        g_hook_setf1 = 1;
        g_hook_f1 = f1v[rmod(6)];
      } else {
        g_hook_objpat = "Start beep[";
        g_hook_cur = (int)rmod(BEEP_QMAX);
      }
    }
    if (rmod(6) == 0) {
      g_reent_bg = &obj;
      g_reent_count = (int)rmod(5);
      g_reent_cur = (int)rmod(4);
    }
    set_params(0);
    for (t = 0; t < 6; t++) {
      float out;
      int r, cur0 = obj.cur;
      unsigned long msgs0 = g_dbg_count;
      putb(&out, 0x7fa5a5a5u);
      r = beepgen_sample(&obj, &out);
      acc_u32(&a, bf(out));
      acc_u32(&a, (unsigned)r);
      bg_hash(&a, &obj);
      if (g_dbg_count != msgs0) cov_hit(CV_S_DBG);
      if (r == 0 && obj.cur != cur0 && obj.n == 0) { /* a new entry started: what it looked like */
        unsigned f1b = bf(obj.f1);
        cov_hit(CV_S_NEXT);
        if (f1b == 0u) cov_hit(CV_S_F1ZERO);
        if (f1b == 0xbf800000u) cov_hit(CV_S_F1M1);
        if (bf(obj.f2) == 0u) cov_hit(CV_S_F2ZERO);
        if ((f1b & 0x7f800000u) == 0x7f800000u && (f1b & 0x7fffffu)) cov_hit((f1b & 0x400000u) ? CV_S_NANF1 : CV_S_SNAN);
      }
      if (r == 1) break;
      if (obj.cur > 0 && rmod(2)) {
        obj.n = obj.limit; /* the next sample ends this entry again */
      }
    }
    g_hook_mode = 0;
    g_hook_obj = NULL;
    g_hook_setf1 = 0;
    g_hook_cur = -1;
    dsplibs_debug_level = 0;
    g_reent_bg = NULL;
  }
  acc_emit("sample transitions", &a);
  /* huge / infinite phases: fsin / fcos on every class */
  acc_init(&a);
  for (j = 0; j < SC(4000u); j++) {
    BeepGen obj;
    float out;
    bg_fill(&obj, 0);
    obj.count = 1;
    obj.cur = 0;
    obj.n = 0;
    obj.limit = 1000000;
    putb(&obj.phase1, rmod(2) ? rspecialb() : rflb(0x90, 0xfe));
    putb(&obj.phase2, rmod(2) ? rspecialb() : rflb(0x90, 0xfe));
    cov_hit(CV_S_HUGEPHASE);
    for (i = 0; i < 3; i++) {
      putb(&out, 0x7fa5a5a5u);
      (void)beepgen_sample(&obj, &out);
      acc_u32(&a, bf(out));
      bg_hash(&a, &obj);
    }
  }
  acc_emit("sample phases", &a);
  (void)ends;
  trial_end("s4");
}

int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : -1;
  const char *e;
  if (scen < 0 || scen > 4) {
    fprintf(stderr, "usage: %s scenario(0..4)\n", argv[0]);
    return 2;
  }
  g_verbose = (e = getenv("R16_VERBOSE")) != NULL && *e == '1';
  if ((e = getenv("R16_SEED")) != NULL) g_lcg ^= (unsigned)strtoul(e, NULL, 0);
  if ((e = getenv("R16_SCALE")) != NULL && atoi(e) > 0) scale_div = (unsigned)atoi(e);
  if ((e = getenv("R16_SWEEP")) != NULL && atoi(e) > 0) sweep_div = (unsigned)atoi(e);
  g_lcg += (unsigned)scen * 0x9e3779b9u;
  g_dbg_hook = dbg_hook;
  cov_setup(scen);
  switch (scen) {
    case 0: scen0(); break;
    case 1: scen1(); break;
    case 2: scen2(); break;
    case 3: scen3(); break;
    case 4: scen4(); break;
    default: break;
  }
  cov_finish();
  return 0;
}
