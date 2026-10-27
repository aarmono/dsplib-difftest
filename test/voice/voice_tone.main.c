/*
 * R15 differential harness: TONE_create / TONE_detect / TONE_delete, MTK_phasor, notch, silence_*, detector_*, _status and GetGain (the blob's LOCAL, called through a regparm(2) pointer;
 * the port's voice_getgain) of the real dsplibs.o vs src/voice/voice_tone.c + voice_tone_tables.c.  Library test/voice/r15_common.h (hashed canonical output, coverage counters exit 2, 3 = harness
 * violation, deterministic heap, recorders sysdep_malloc / free / memset, dsplibs_debug_printf).  The 32-bit BLOB run is the x86-64 golden: every INPUT float is built from integers
 * (bit patterns, exact int -> float conversions, powers of two) so the x86-64 run feeds the same inputs; floats are printed as bit patterns.
 *
 * Callee recorders (R16 / R09 functions, NOT ported here; the blob's own definitions are weakened by the build script): create_dtmf, dtmf_progress (R16), cadence_create /
 * cadence_progress / cadence_delete (R09; the real ones are ported in src/tone/tone_cadence.c but a recorder gives scripted events), modem_get_param (stub_env's abort stub renamed), and the
 * voice object's callback `cb(ctx, id)`.  Every call is logged (hashed) and returns scripted values, so a deviation in call order / arguments changes the hash.
 *
 * Scenarios (each its own process; argv[1]):
 *   0 MTK_phasor (phases: grid, +-ulps around 0 / pi / 2 pi, the (-6e-8, 0) range where the phase rounds up to 2 pi (table quadrant 4), huge, denormal, +-0, inf, NaN payloads, random bit
 *     patterns; steps: 0, small, +-pi, +-2 pi, huge, inf, NaN) and notch (random / special x, state, coefficients; the returned EXTENDED value is compared bit for bit), each also with the x87
 *     precision control set to double; the MTK / TONE_CFG tables;
 *   1 TONE: create (NULL / NULL, TONE_CFG, the detector's four configs, random configs; fresh and existing objects), TONE_detect over integer-generated signals (silence, tones at the
 *     object's frequency and beside it, noise, mixtures, impulses, specials NaN / inf / denormal / huge, counts 0 / 1 / many / negative), delete; PC = double pass;
 *   2 silence_* (create, progress with every level 0 .. 5, window / limit / heard states, thresholds swept +-ulps, noise at the thresholds, debug text and a level-lowering hook,
 *     silence_is_more_then over all special values, _status);
 *   3 detector_* (create / existing / failures (guarded), set_*, progress with scripted DTMF / cadence events, all enable masks, both output modes, tones at the four frequencies, delete);
 *   4 GetGain over the parameter cube and extremes, debug levels and the level-lowering hook.
 * R15_VERBOSE=1 prints every line, R15_SEED selects another input stream, R15_SCALE=n divides the big sweeps by n (debugging only).
 */
#include <math.h>

#include "r15_common.h"

#define PHASE_VAL(v) v
extern const ToneCfg TONE_CFG;

/* ---------------------------------------------------------------------------------------------- the callee under test: blob / port naming */
#ifdef TEST_BLOB
/* the blob's LOCAL GetGain(eax = beepgen, edx = out_b, stack = out_a), globalised by the build script (objcopy --globalize-symbol=GetGain) */
extern void __attribute__((regparm(2))) GetGain(void *beep, float *out_b, float *out_a);
#define CALL_GETGAIN(b, ob, oa) GetGain((b), (ob), (oa))
/* the blob's own GLOBAL pow (pow.S): called through an asm label so that the compiler does not treat it as the libm builtin */
extern long double blob_pow(double x, double y) __asm__("pow");
#define CALL_POW(x, y) blob_pow((x), (y))
#else
#define CALL_POW(x, y) dsplibs_pow((x), (y))
#define CALL_GETGAIN(b, ob, oa) voice_getgain((b), (ob), (oa))
#endif

enum {
  CV_P_GRID, CV_P_ULPS, CV_P_NEGTINY, CV_P_HUGE, CV_P_DENORM, CV_P_ZERO, CV_P_INF, CV_P_NAN, CV_P_RAND, CV_P_STEP0, CV_P_STEPBIG, CV_P_STEPINF, CV_P_STEPNAN, CV_P_PCDBL,
  CV_N_RAND, CV_N_SPECIAL, CV_N_NAN, CV_N_INF, CV_N_PCDBL, CV_T_TABLES,
  CV_T_CREATE_NULL, CV_T_CREATE_CFG, CV_T_CREATE_EXIST, CV_T_CREATE_RAND, CV_T_DET_0, CV_T_DET_1, CV_T_DET_2, CV_T_ZEROCNT, CV_T_NEGCNT, CV_T_ONECNT, CV_T_SPECIAL, CV_T_NANIN, CV_T_INFIN,
  CV_T_DENORM, CV_T_HUGE, CV_T_WRAP, CV_T_IDXRESET, CV_T_DELETE, CV_T_PCDBL, CV_T_CLAMP, CV_T_NEGCFG, CV_T_NANCFG, CV_T_FAIL_GUARDED, CV_T_N0,
  CV_S_LVL0, CV_S_LVL1, CV_S_LVL3, CV_S_LVLBIG, CV_S_SOUND, CV_S_SILENT, CV_S_REPORT71, CV_S_REPORT73, CV_S_LIMITBIG, CV_S_DBG, CV_S_HOOK, CV_S_SWEEP, CV_S_NOISE, CV_S_CNT0, CV_S_EXIST,
  CV_S_NEW, CV_S_MORE_NAN, CV_S_MORE_BIG, CV_S_STATUS, CV_S_PCDBL, CV_S_FAIL0, CV_S_WRAPN, CV_S_DELETE,
  CV_D_NEW, CV_D_EXIST, CV_D_CBNULL, CV_D_CB, CV_D_FAIL0, CV_D_FAIL_GUARDED, CV_D_DTMF_VALID, CV_D_DTMF_NEG, CV_D_DTMF_SKIP, CV_D_DTMF_BIG, CV_D_BUSY90, CV_D_TONE_STATUS, CV_D_TONE_STREAM,
  CV_D_TONE_CNTRESET, CV_D_CAD_A, CV_D_CAD_B, CV_D_CAD_STATUS, CV_D_CAD_STREAM, CV_D_MASKS, CV_D_DBG, CV_D_DELETE, CV_D_DELETE_CAD8, CV_D_SETS, CV_D_BUFADV, CV_D_SPECIALX,
  CV_G_CUBE, CV_G_EXTREME, CV_G_DBG0, CV_G_DBG2, CV_G_HOOK1, CV_G_HOOK2,
  CV_W_INTY, CV_W_FRACY, CV_W_ZEROY, CV_W_INFY, CV_W_NANY, CV_W_ZEROX, CV_W_INFX, CV_W_NANX, CV_W_NEGX, CV_W_NEARONE, CV_W_BIGINT, CV_W_NEGINT, CV_W_GETGAINY, CV_W_ODD, CV_W_PCDBL
};
static void cov_setup(int scen)
{
  switch (scen) {
    case 0:
      cov_def(CV_P_GRID, "phasor phase grid");
      cov_def(CV_P_ULPS, "phasor +-ulps around 0 / pi / 2 pi");
      cov_def(CV_P_NEGTINY, "phasor tiny negative (quadrant 4)");
      cov_def(CV_P_HUGE, "phasor huge");
      cov_def(CV_P_DENORM, "phasor denormal");
      cov_def(CV_P_ZERO, "phasor +-0");
      cov_def(CV_P_INF, "phasor +-inf");
      cov_def(CV_P_NAN, "phasor NaN");
      cov_def(CV_P_RAND, "phasor random bit patterns");
      cov_def(CV_P_STEP0, "phasor step 0");
      cov_def(CV_P_STEPBIG, "phasor huge step");
      cov_def(CV_P_STEPINF, "phasor infinite step");
      cov_def(CV_P_STEPNAN, "phasor NaN step");
      cov_def(CV_P_PCDBL, "phasor / notch precision control = double pass");
      cov_def(CV_N_RAND, "notch random");
      cov_def(CV_N_SPECIAL, "notch special values");
      cov_def(CV_N_NAN, "notch NaN");
      cov_def(CV_N_INF, "notch inf");
      cov_def(CV_T_TABLES, "tables");
      break;
    case 1:
      cov_def(CV_T_CREATE_NULL, "TONE_create NULL / NULL");
      cov_def(CV_T_CREATE_CFG, "TONE_create NULL obj, explicit cfg");
      cov_def(CV_T_CREATE_EXIST, "TONE_create existing object");
      cov_def(CV_T_CREATE_RAND, "TONE_create random cfg");
      cov_def(CV_T_DET_0, "TONE_detect returned 0");
      cov_def(CV_T_DET_1, "TONE_detect returned 1");
      cov_def(CV_T_DET_2, "TONE_detect returned 2");
      cov_def(CV_T_ZEROCNT, "TONE_detect count 0");
      cov_def(CV_T_NEGCNT, "TONE_detect count < 0");
      cov_def(CV_T_ONECNT, "TONE_detect count 1");
      cov_def(CV_T_SPECIAL, "TONE_detect special inputs");
      cov_def(CV_T_NANIN, "TONE_detect NaN input");
      cov_def(CV_T_INFIN, "TONE_detect inf input");
      cov_def(CV_T_DENORM, "TONE_detect denormal input");
      cov_def(CV_T_HUGE, "TONE_detect huge input");
      cov_def(CV_T_WRAP, "ring index wraps");
      cov_def(CV_T_IDXRESET, "ring index out of range resets");
      cov_def(CV_T_DELETE, "TONE_delete");
      cov_def(CV_T_PCDBL, "precision control = double pass");
      cov_def(CV_T_CLAMP, "accumulator A clamped (A became 0 with B > 0)");
      cov_def(CV_T_NEGCFG, "negative cfg values");
      cov_def(CV_T_NANCFG, "NaN cfg values");
      cov_def(CV_T_FAIL_GUARDED, "TONE_create malloc failure (guarded)");
      cov_def(CV_T_N0, "cfg.n <= 0 on an existing object");
      break;
    case 2:
      cov_def(CV_S_LVL0, "silence level 0");
      cov_def(CV_S_LVL1, "silence level 1");
      cov_def(CV_S_LVL3, "silence level 3");
      cov_def(CV_S_LVLBIG, "silence level > 3");
      cov_def(CV_S_SOUND, "window above threshold");
      cov_def(CV_S_SILENT, "window at / below threshold");
      cov_def(CV_S_REPORT71, "DLE 0x71");
      cov_def(CV_S_REPORT73, "DLE 0x73");
      cov_def(CV_S_LIMITBIG, "limit > INT_MAX");
      cov_def(CV_S_DBG, "debug text");
      cov_def(CV_S_HOOK, "level lowered inside a message");
      cov_def(CV_S_SWEEP, "threshold sweep");
      cov_def(CV_S_NOISE, "noise at the thresholds");
      cov_def(CV_S_CNT0, "count <= 0");
      cov_def(CV_S_EXIST, "silence_create existing");
      cov_def(CV_S_NEW, "silence_create NULL");
      cov_def(CV_S_MORE_NAN, "is_more_then NaN");
      cov_def(CV_S_MORE_BIG, "is_more_then overflow");
      cov_def(CV_S_STATUS, "_status");
      cov_def(CV_S_PCDBL, "precision control = double pass");
      cov_def(CV_S_FAIL0, "silence_create malloc failure");
      cov_def(CV_S_WRAPN, "window counter already >= 800");
      cov_def(CV_S_DELETE, "silence_delete");
      break;
    case 3:
      cov_def(CV_D_NEW, "detector_create NULL");
      cov_def(CV_D_EXIST, "detector_create existing");
      cov_def(CV_D_CBNULL, "detector_create cb NULL");
      cov_def(CV_D_CB, "detector_create cb");
      cov_def(CV_D_FAIL0, "detector_create malloc #0 fails");
      cov_def(CV_D_FAIL_GUARDED, "detector_create later malloc fails (guarded)");
      cov_def(CV_D_DTMF_VALID, "dtmf valid digit");
      cov_def(CV_D_DTMF_NEG, "dtmf false detect");
      cov_def(CV_D_DTMF_SKIP, "dtmf -2 / -1");
      cov_def(CV_D_DTMF_BIG, "dtmf > 15");
      cov_def(CV_D_BUSY90, "dtmf busy word skips the tones");
      cov_def(CV_D_TONE_STATUS, "tone reported as status code");
      cov_def(CV_D_TONE_STREAM, "tone reported in the stream");
      cov_def(CV_D_TONE_CNTRESET, "tone counter reset");
      cov_def(CV_D_CAD_A, "cadence A event");
      cov_def(CV_D_CAD_B, "cadence B event");
      cov_def(CV_D_CAD_STATUS, "cadence status");
      cov_def(CV_D_CAD_STREAM, "cadence stream");
      cov_def(CV_D_MASKS, "enable masks");
      cov_def(CV_D_DBG, "debug text");
      cov_def(CV_D_DELETE, "detector_delete");
      cov_def(CV_D_DELETE_CAD8, "detector_delete with cad8");
      cov_def(CV_D_SETS, "set_* functions");
      cov_def(CV_D_BUFADV, "stream write after a dtmf write");
      cov_def(CV_D_SPECIALX, "cadence samples special floats");
      break;
    case 5:
      cov_def(CV_W_INTY, "pow integer y");
      cov_def(CV_W_FRACY, "pow non-integer y");
      cov_def(CV_W_ZEROY, "pow y = +-0");
      cov_def(CV_W_INFY, "pow y = +-inf");
      cov_def(CV_W_NANY, "pow y NaN");
      cov_def(CV_W_ZEROX, "pow x = +-0");
      cov_def(CV_W_INFX, "pow x = +-inf");
      cov_def(CV_W_NANX, "pow x NaN");
      cov_def(CV_W_NEGX, "pow x < 0");
      cov_def(CV_W_NEARONE, "pow |x - 1| <= 0.29");
      cov_def(CV_W_BIGINT, "pow huge integer y");
      cov_def(CV_W_NEGINT, "pow negative integer y");
      cov_def(CV_W_GETGAINY, "pow(10, y) over GetGain's exponents");
      cov_def(CV_W_ODD, "pow odd integer y with +-0 / +-inf x");
      cov_def(CV_W_PCDBL, "precision control = double pass");
      break;
    case 4:
      cov_def(CV_G_CUBE, "GetGain parameter cube");
      cov_def(CV_G_EXTREME, "GetGain extreme parameters");
      cov_def(CV_G_DBG0, "GetGain no text");
      cov_def(CV_G_DBG2, "GetGain all text");
      cov_def(CV_G_HOOK1, "GetGain level lowered after the first text");
      cov_def(CV_G_HOOK2, "GetGain level lowered after the second text");
      break;
    default: break;
  }
}

/* ---------------------------------------------------------------------------------------------- helpers */
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
static unsigned SC(unsigned n) /* R15_SCALE: every counted loop (debugging) */
{
  unsigned v = n / scale_div;
  return v ? v : 1;
}
static unsigned SW(unsigned n) /* R15_SWEEP: the big random sweeps only (mutation runs); R15_SCALE divides these too */
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

/* ---------------------------------------------------------------------------------------------- recorders of the callees that are not under test */
static int g_quiet; /* skip the recorder logging (the big sweeps) */
static char g_modem_obj[16];
static struct modem *const g_modem = (struct modem *)(void *)g_modem_obj; /* only compared with ==, never dereferenced */
static int g_mgp[256];
static unsigned g_cbv[256]; /* values the voice callback returns per id */
static int g_cb_calls;
long modem_get_param(struct modem *m, unsigned id)
{
  if (!g_quiet) emit(0, "MGP %d %s", (int)id, m == g_modem ? "ctx" : "BADCTX");
  g_cb_calls++;
  return g_mgp[id & 0xff];
}
static unsigned vp_cb(struct modem *m, int id)
{
  emit(0, "CB %d %s", id, m == g_modem ? "ctx" : "BADCTX");
  g_cb_calls++;
  return g_cbv[id & 0xff];
}

/* DTMF object recorder: a 0x100-byte block; the u16 at +0x90 is the "busy" word the detector reads */
static unsigned short g_w90;
static int g_dtmf_script; /* 0: valid digits, 1: mixed */
static unsigned long g_dtmf_calls;
struct Dtmf *create_dtmf(struct Dtmf *d)
{
  emit(0, "CREATE_DTMF %s", d ? "existing" : "NULL");
  if (!d) {
    d = (struct Dtmf *)sysdep_malloc(0x100);
    if (!d) return NULL;
    memset(d, 0, 0x100);
  }
  *(unsigned short *)((char *)d + 0x90) = g_w90;
  return d;
}
static short dtmf_next(void)
{
  static const short mixed[] = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 0x100, 0x7fff, -3, -32768, -4, -5, 4, 5, -2, 11};
  unsigned k = rmod(100);
  if (g_dtmf_script == 0) return (short)rmod(16);
  if (k < 55) return (short)(-2 + (int)rmod(2));
  return mixed[rmod((unsigned)(sizeof mixed / sizeof mixed[0]))];
}
short dtmf_progress(struct Dtmf *d, const float *x, short count, int mode)
{
  short r = dtmf_next();
  Acc a;
  int i, m = count < 0 ? 0 : (count > 8 ? 8 : count);
  acc_init(&a);
  for (i = 0; i < m; i++) acc_u32(&a, bf(x[i]));
  g_dtmf_calls++;
  if ((unsigned short)(r + 2) <= 1) cov_hit(CV_D_DTMF_SKIP);
  else if ((unsigned short)r <= 15) cov_hit(CV_D_DTMF_VALID);
  else {
    cov_hit(CV_D_DTMF_NEG);
    if ((unsigned short)r > 15 && r > 15) cov_hit(CV_D_DTMF_BIG);
  }
  emit(0, "DTMF_PROGRESS count=%d mode=%d busy=%u x=%016llx -> %d", (int)count, mode, (unsigned)*(unsigned short *)((char *)d + 0x90), a.h, (int)r);
  return r;
}

/* cadence recorders: the handle is a 0x40-byte sysdep block; the script decides the result */
static unsigned g_cad_script; /* 0 never, 1 sparse hits, 2 dense hits, 3 mixed */
static unsigned long g_cad_calls[2];
Cadence *cadence_create(Cadence *c, CadenceParams *p, int extra, struct modem *m)
{
  emit(0, "CADENCE_CREATE %s w0=%d w4=%d w8=%d type=%u w18=%d extra=%d %s", c ? "existing" : "NULL", p->w0, p->w4, p->w8, p->type, p->callerParamB, extra, m == g_modem ? "ctx" : "BADCTX");
  if (!c) {
    c = (Cadence *)sysdep_malloc(0x40);
    if (!c) return NULL;
    memset(c, 0, 0x40);
  }
  return c;
}
static Cadence *g_cadA, *g_cadB;
int cadence_progress(Cadence *c, short sample)
{
  int which = (c == g_cadB);
  unsigned k = rmod(100), r;
  g_cad_calls[which]++;
  emit(0, "CADENCE_PROGRESS %c %d", which ? 'B' : 'A', (int)sample);
  switch (g_cad_script) {
    case 0: r = 0; break;
    case 1: r = k < 3 ? 1u : 0u; break;
    case 2: r = k < 60 ? 1u : 0u; break;
    default: r = k < 20 ? 1u : (k < 30 ? 7u : (k < 40 ? 0x10001u : (k < 45 ? 0x20000u : (k < 50 ? 0xffff0001u : 0u)))); break;
  }
  if ((short)r == 1) cov_hit(which ? CV_D_CAD_B : CV_D_CAD_A);
  return (int)r;
}
void cadence_delete(Cadence *c)
{
  emit(0, "CADENCE_DELETE");
  sysdep_free(c);
}

/* ---------------------------------------------------------------------------------------------- scenario 0: MTK_phasor / notch / tables */
/* every float input travels as a bit pattern (a float passed BY VALUE through the i386 x87 would turn a signalling NaN into a quiet one before the callee sees it) */
static void phasor_one_b(Acc *a, unsigned p0, unsigned p3)
{
  float p[4];
  memcpy(&p[0], &p0, 4);
  p[1] = fb(0x7f8abcdeu); /* garbage in the outputs: never read */
  p[2] = fb(0x3f123456u);
  memcpy(&p[3], &p3, 4);
  MTK_phasor(p);
  acc_u32(a, bf(p[0]));
  acc_u32(a, bf(p[1]));
  acc_u32(a, bf(p[2]));
  acc_u32(a, bf(p[3]));
  if (g_verbose) emit(0, "PH %08x %08x -> %08x %08x %08x %08x", p0, p3, bf(p[0]), bf(p[1]), bf(p[2]), bf(p[3]));
}

static void phasor_all(const char *tag, unsigned scale)
{
  static const unsigned steps[] = {0x00000000u, 0x80000000u, 0x3d4104fbu, 0x3a83126fu, 0x40490fdbu, 0xc0490fdbu, 0x40c90fdbu, 0xc0c90fdbu, 0x40490fdau, 0x40c90fdau, 0x7f7fffffu, 0x7f800000u,
                                   0xff800000u, 0x7fc00000u, 0x7fc12345u, 0x7f800001u, 0x00000001u, 0x3f800000u, 0xbf800000u, 0x3dcccccdu, 0x4b000000u};
  Acc a;
  unsigned i, j, k;
  char nm[64];
  /* grid: multiples of 1/256 in [-16, 16] (exact int -> float), each with 6 steps */
  acc_init(&a);
  for (i = 0; i < 8193u; i++) {
    float ph = (float)((int)i - 4096) * 0.00390625f;
    for (j = 0; j < 6; j++) phasor_one_b(&a, bf(ph), steps[(i + j) % (sizeof steps / sizeof steps[0])]);
  }
  cov_hit(CV_P_GRID);
  snprintf(nm, sizeof nm, "%s grid", tag);
  acc_emit(nm, &a);
  /* ulps around 0, pi, 2 pi (+-3 pi, 4 pi) */
  acc_init(&a);
  {
    static const unsigned base[] = {0x40490fdbu, 0x40c90fdbu, 0x40c90fdau, 0x4116cbe4u, 0x41490fdbu, 0x3fc90fdbu, 0x4096cbe4u, 0x41c90fdbu, 0x42490fdbu};
    for (k = 0; k < sizeof base / sizeof base[0]; k++)
      for (i = 0; i < 129; i++)
        for (j = 0; j < 4; j++) {
          unsigned u = base[k] + i - 64u;
          phasor_one_b(&a, u, steps[(i + j) % (sizeof steps / sizeof steps[0])]);
          phasor_one_b(&a, u | 0x80000000u, steps[(i + j + 5) % (sizeof steps / sizeof steps[0])]);
        }
    for (i = 0; i < 400; i++) {
      phasor_one_b(&a, i, 0u);
      phasor_one_b(&a, 0x80000000u | i, 0u);
      phasor_one_b(&a, i, 0x40490fdbu);
    }
  }
  cov_hit(CV_P_ULPS);
  cov_hit(CV_P_ZERO);
  cov_hit(CV_P_DENORM);
  snprintf(nm, sizeof nm, "%s ulps", tag);
  acc_emit(nm, &a);
  /* the range where 2 pi + rem rounds up to 2 pi as a float: tiny negative phases */
  acc_init(&a);
  for (i = 0; i < SW(60000u * scale); i++) {
    unsigned u = 0x80000000u | (0x30000000u + rmod(0x08000000u)); /* -2^-31 .. -2^-21 */
    phasor_one_b(&a, u, steps[rmod(8)]);
  }
  for (i = 0; i < 4096; i++) {
    unsigned u = 0xb3000000u + i * 0x40u; /* sweep through -2^-25 */
    phasor_one_b(&a, u, 0u);
    phasor_one_b(&a, u, 0x40490fdbu);
  }
  cov_hit(CV_P_NEGTINY);
  snprintf(nm, sizeof nm, "%s negtiny", tag);
  acc_emit(nm, &a);
  /* huge, infinite, NaN phases and the special step set */
  acc_init(&a);
  for (i = 0; i < NSPECIAL; i++)
    for (j = 0; j < sizeof steps / sizeof steps[0]; j++) phasor_one_b(&a, special_bits[i], steps[j]);
  for (i = 0; i < SW(40000u * scale); i++) phasor_one_b(&a, rflb(0xa0, 0xfe), rflb(0x60, 0xfe));
  for (i = 0; i < SW(2000u * scale); i++) {
    phasor_one_b(&a, 0x7f800000u | (rnd() & 0x007fffffu), steps[rmod(sizeof steps / sizeof steps[0])]);
    phasor_one_b(&a, 0xff800000u | (rnd() & 0x007fffffu), 0u);
  }
  cov_hit(CV_P_HUGE);
  cov_hit(CV_P_INF);
  cov_hit(CV_P_NAN);
  cov_hit(CV_P_STEPBIG);
  cov_hit(CV_P_STEPINF);
  cov_hit(CV_P_STEPNAN);
  snprintf(nm, sizeof nm, "%s special", tag);
  acc_emit(nm, &a);
  /* random */
  acc_init(&a);
  for (i = 0; i < SW(1500000u * scale); i++) {
    unsigned ph, st;
    switch (rmod(8)) {
      case 0: ph = rflb(0x6e, 0x82); st = 0u; break;
      case 1: ph = rflb(0x6e, 0x82); st = rflb(0x6e, 0x82); break;
      case 2: ph = rflb(0x78, 0x84); st = rflb(0x78, 0x80) & 0x7fffffffu; break;
      case 3: ph = rnd(); st = rnd(); break;
      case 4: ph = rflb(0x60, 0x90); st = rspecialb(); break;
      case 5: ph = rspecialb(); st = rflb(0x60, 0x90); break;
      case 6: ph = rflb(0x7f, 0x83); st = rflb(0x7f, 0x83); break;
      default: ph = rflb(0x70, 0x7f); st = rflb(0x70, 0x90); break;
    }
    phasor_one_b(&a, ph, st);
  }
  cov_hit(CV_P_RAND);
  cov_hit(CV_P_STEP0);
  snprintf(nm, sizeof nm, "%s random", tag);
  acc_emit(nm, &a);
}
/* notch(float x, ...) is called with x as a raw 32-bit argument at -m32 (a float passed by value would be converted by the x87 and a signalling NaN quieted); x86-64 passes it in an xmm
 * register through a bit-exact move */
static long double call_notch(unsigned xb, float *s, const float *c)
{
#if defined(__i386__)
  return ((long double (*)(unsigned, float *, const float *))(void *)notch)(xb, s, c);
#else
  float x;
  memcpy(&x, &xb, 4);
  return notch(x, s, c);
#endif
}
static void notch_one(Acc *a, unsigned xb, const unsigned *sb, const unsigned *cb)
{
  float s[2], c[4];
  long double r;
  unsigned char raw[16];
  memcpy(&s[0], &sb[0], 4);
  memcpy(&s[1], &sb[1], 4);
  memcpy(c, cb, 16);
  r = call_notch(xb, s, c);
  memset(raw, 0, sizeof raw);
  memcpy(raw, &r, 10); /* the 80-bit value */
  acc_bytes(a, raw, 10);
  acc_u32(a, bf(s[0]));
  acc_u32(a, bf(s[1]));
  if (g_verbose) emit(0, "NOTCH x=%08x s=%08x %08x c=%08x %08x %08x %08x -> %08x %08x", xb, sb[0], sb[1], cb[0], cb[1], cb[2], cb[3], bf(s[0]), bf(s[1]));
}
static void notch_all(const char *tag, unsigned scale)
{
  Acc a;
  unsigned i, k, s[2], c[4], x;
  char nm[64];
  acc_init(&a);
  for (i = 0; i < SW(1500000u * scale); i++) {
    switch (rmod(4)) {
      case 0: x = rflb(0x78, 0x86); s[0] = rflb(0x78, 0x86); s[1] = rflb(0x78, 0x86); for (k = 0; k < 4; k++) c[k] = rflb(0x70, 0x80); break;
      case 1: x = rflb(0x60, 0x8e); s[0] = rflb(0x60, 0x8e); s[1] = rflb(0x60, 0x8e); for (k = 0; k < 4; k++) c[k] = rflb(0x60, 0x8e); break;
      case 2: x = rflb(0x7e, 0x80); s[0] = rflb(0x7e, 0x80); s[1] = rflb(0x7e, 0x80); for (k = 0; k < 4; k++) c[k] = rflb(0x7c, 0x80); break;
      default: x = rnd(); s[0] = rnd(); s[1] = rnd(); for (k = 0; k < 4; k++) c[k] = rnd(); break;
    }
    notch_one(&a, x, s, c);
  }
  cov_hit(CV_N_RAND);
  snprintf(nm, sizeof nm, "%s random", tag);
  acc_emit(nm, &a);
  acc_init(&a);
  for (i = 0; i < SW(400000u * scale); i++) {
    x = rmixb();
    s[0] = rmixb();
    s[1] = rmixb();
    for (k = 0; k < 4; k++) c[k] = rmixb();
    notch_one(&a, x, s, c);
  }
  for (i = 0; i < NSPECIAL; i++)
    for (k = 0; k < NSPECIAL; k++) {
      s[0] = special_bits[k];
      s[1] = special_bits[(k + 7) % NSPECIAL];
      c[0] = special_bits[(i + k) % NSPECIAL];
      c[1] = special_bits[(i * 3 + k) % NSPECIAL];
      c[2] = special_bits[(i + 11 * k) % NSPECIAL];
      c[3] = special_bits[(i + 1) % NSPECIAL];
      notch_one(&a, special_bits[i], s, c);
    }
  cov_hit(CV_N_SPECIAL);
  cov_hit(CV_N_NAN);
  cov_hit(CV_N_INF);
  snprintf(nm, sizeof nm, "%s mixed", tag);
  acc_emit(nm, &a);
}
static void table_hash(const char *name, const float *t, unsigned n)
{
  Acc a;
  unsigned i;
  acc_init(&a);
  for (i = 0; i < n; i++) acc_u32(&a, bf(t[i]));
  acc_emit(name, &a);
}
static void scen0(void)
{
  Acc a;
  unsigned i;
  table_hash("MTK_sin_table", MTK_sin_table, 257);
  table_hash("MTK_cos_table", MTK_cos_table, 257);
  table_hash("MTK_sin_sign", MTK_sin_sign, 4);
  table_hash("MTK_cos_sign", MTK_cos_sign, 4);
  acc_init(&a);
  {
    const unsigned *w = (const unsigned *)(const void *)&TONE_CFG;
    (void)w;
    acc_u32(&a, bf(TONE_CFG.freq));
    acc_u32(&a, bf(TONE_CFG.f4));
    acc_u32(&a, bf(TONE_CFG.f8));
    acc_u32(&a, bf(TONE_CFG.fc));
    acc_u32(&a, bf(TONE_CFG.f10));
    acc_u32(&a, bf(TONE_CFG.f14));
    acc_u32(&a, bf(TONE_CFG.f18));
    acc_u32(&a, (unsigned)(unsigned short)TONE_CFG.n);
    acc_u32(&a, (unsigned)(unsigned short)TONE_CFG.n_hi);
    acc_u32(&a, (unsigned)TONE_CFG.w24);
    acc_u32(&a, bf(TONE_CFG.f28));
    acc_u32(&a, (unsigned)TONE_CFG.w2c);
  }
  acc_emit("TONE_CFG fields", &a);
  table_hash("TONE_CFG.lpf", TONE_CFG.lpf, 53);
  cov_hit(CV_T_TABLES);
  set_pc(0);
  phasor_all("phasor", 1);
  notch_all("notch", 1);
  set_pc(1);
  cov_hit(CV_P_PCDBL);
  emit(1, "== precision control double");
  phasor_all("phasor-pcd", 1);
  notch_all("notch-pcd", 1);
  set_pc(0);
  for (i = 0; i < 1; i++) trial_end("s0");
}

/* ---------------------------------------------------------------------------------------------- integer signal generators */
#define SIGN_MAX 70000
static float g_sig[SIGN_MAX];
/* sin(2 pi ph / 2^32) in Q16 (+-65536) by an integer parabola-ish approximation: 1.5 u - 0.5 u^3 per quadrant (a 1.7% error: harmonics do not matter here) */
static int isin(unsigned ph)
{
  unsigned q = ph >> 30, u = (ph >> 14) & 0xffffu;
  long long u2, u3, v;
  int r;
  if (q & 1) u = 0x10000u - u;
  u2 = ((long long)u * u) >> 16;
  u3 = (u2 * u) >> 16;
  v = (3LL * u - u3) / 2; /* 0 .. 65536 */
  r = (int)v;
  return (q & 2) ? -r : r;
}
static unsigned freq_inc(unsigned hz, unsigned fs) { return (unsigned)((((unsigned long long)hz) << 32) / fs); }
/* kinds: 0 zeros, 1 sine, 2 noise, 3 sine + noise, 4 impulses, 5 square, 6 ramp; amp = mant * 2^-shift (mant <= 2^8 so the sample is exact in a float) */
static unsigned g_ph;
static void gen(float *out, unsigned n, int kind, unsigned hz, int mant, int shift)
{
  unsigned i, inc = freq_inc(hz, 8000);
  float sc = 1.0f;
  int k;
  for (k = 0; k < shift; k++) sc *= 0.5f; /* exact */
  for (i = 0; i < n; i++) {
    int v;
    switch (kind) {
      case 0: v = 0; break;
      case 1: v = isin(g_ph); break;
      case 2: v = (int)(rnd() & 0xffffu) - 32768; break;
      case 3: v = isin(g_ph) / 2 + ((int)(rnd() & 0x7fffu) - 16384) / 2; break;
      case 4: v = (rmod(40) == 0) ? 65536 : 0; break;
      case 5: v = (g_ph & 0x80000000u) ? -65536 : 65536; break;
      default: v = (int)((i * 977u) & 0xffffu) - 32768; break;
    }
    g_ph += inc;
    out[i] = (float)(v >> 4) * sc * (float)mant; /* |v >> 4| <= 2^12, mant < 2^8: 20 bits, exact; times a power of two */
  }
}

/* ---------------------------------------------------------------------------------------------- scenario 1: TONE */
static float g_lpf[96];
static void make_lpf(void)
{
  unsigned i;
  for (i = 0; i < 96; i++) g_lpf[i] = (float)((int)(rnd() & 0xfffffu) - 0x80000) * 0.00000095367431640625f; /* int * 2^-20, exact: +-0.5 */
}
static void dump_tone(int show, const char *tag, const ToneObj *o)
{
  Acc a;
  int i, n = o->cfg.n;
  emit(show, "%s cfg %08x %08x %08x %08x %08x %08x %08x n=%d hi=%d w24=%08x f28=%08x w2c=%08x lpf=%s", tag, bf(o->cfg.freq), bf(o->cfg.f4), bf(o->cfg.f8), bf(o->cfg.fc), bf(o->cfg.f10),
       bf(o->cfg.f14), bf(o->cfg.f18), (int)o->cfg.n, (int)o->cfg.n_hi, (unsigned)o->cfg.w24, bf(o->cfg.f28), (unsigned)o->cfg.w2c,
       o->cfg.lpf == g_lpf ? "g_lpf" : (o->cfg.lpf == TONE_CFG.lpf ? "TONE_CFG" : "OTHER"));
  emit(show, "%s state w30=%08x ph=%08x w38=%08x idx=%d k=%08x %08x %08x s=%08x %08x A=%08x B=%08x w64=%d w68=%d w6c=%d w1b0=%d w1c0=%d w1c4=%d selfk=%s", tag, bf(o->w30), bf(o->ph), bf(o->w38), (int)o->idx,
       bf(o->k0), bf(o->k1), bf(o->k2), bf(o->s0), bf(o->s1), bf(o->accA), bf(o->accB), (int)o->w64, o->w68, o->w6c, (int)o->w1b0, o->w1c0, o->w1c4, o->selfk == &o->k0 ? "ok" : "BAD");
  acc_init(&a);
  for (i = 0; i < 80; i++) acc_u32(&a, (unsigned)o->z70[i]);
  emit(show, "%s z70 %016llx", tag, a.h);
  if (n > 0 && o->coef && o->ring) {
    acc_init(&a);
    for (i = 0; i < n; i++) acc_u32(&a, bf(o->coef[i]));
    emit(show, "%s coef %016llx first %08x last %08x", tag, a.h, bf(o->coef[0]), bf(o->coef[n - 1]));
    acc_init(&a);
    for (i = 0; i < n; i++) acc_u32(&a, bf(o->ring[i]));
    emit(show, "%s ring %016llx", tag, a.h);
  }
  if (o->b14) emit(show, "%s b14 %08x %08x %08x %08x %08x", tag, bf(o->b14[0]), bf(o->b14[1]), bf(o->b14[2]), bf(o->b14[3]), bf(o->b14[4]));
  if (o->b8) emit(show, "%s b8 %08x %08x", tag, bf(o->b8[0]), bf(o->b8[1]));
  emit(show, "%s blocks coef=A%u ring=A%u b14=A%u b8=A%u", tag, blk_id_of(o->coef), blk_id_of(o->ring), blk_id_of(o->b14), blk_id_of(o->b8));
}
/* an existing object: dirty bytes (the harness checks that the padding members are not written), its four blocks allocated by the harness */
static ToneObj *make_existing(unsigned *padsum)
{
  ToneObj *o = (ToneObj *)sysdep_malloc((unsigned)sizeof(ToneObj));
  unsigned i;
  unsigned char *b = (unsigned char *)o;
  for (i = 0; i < 0x1c8; i++) b[i] = (unsigned char)rnd(); /* the i386 size: the number of PRNG draws must not depend on sizeof */
  o->coef = (float *)sysdep_malloc(96 * 4);
  o->ring = (float *)sysdep_malloc(96 * 4);
  o->b14 = (float *)sysdep_malloc(0x14);
  o->b8 = (float *)sysdep_malloc(8);
  for (i = 0; i < 96; i++) {
    putb(&o->coef[i], rnd());
    putb(&o->ring[i], rnd());
  }
  o->idx = (short)rmod(60);
  *padsum = ((unsigned)(unsigned short)o->pad46 << 16) ^ (unsigned)(unsigned short)o->pad66 ^ ((unsigned)(unsigned short)o->pad1b2 << 8);
  return o;
}
static void check_pads(const ToneObj *o, unsigned padsum)
{
  unsigned now = ((unsigned)(unsigned short)o->pad46 << 16) ^ (unsigned)(unsigned short)o->pad66 ^ ((unsigned)(unsigned short)o->pad1b2 << 8);
  if (now != padsum) die3("a padding member of ToneObj was written");
}
static int g_det_results[3];
static void det_call(ToneObj *o, const float *in, short count, const char *tag)
{
  int r = TONE_detect(o, in, count);
  if (r < 0 || r > 2) die3("TONE_detect returned %d", r);
  g_det_results[r]++;
  emit(0, "DET %s n=%d in0=%08x -> %d A=%08x B=%08x idx=%d s=%08x %08x", tag, (int)count, count > 0 ? bf(in[0]) : 0u, r, bf(o->accA), bf(o->accB), (int)o->idx, bf(o->s0), bf(o->s1));
  if (r == 0) cov_hit(CV_T_DET_0);
  if (r == 1) cov_hit(CV_T_DET_1);
  if (r == 2) cov_hit(CV_T_DET_2);
  if (bf(o->accA) == 0u && o->accB != 0.0f) cov_hit(CV_T_CLAMP);
}
/* feed a series of blocks of every kind to one object */
static void det_series(ToneObj *o, unsigned hz, unsigned blocks, const char *tag)
{
  static const unsigned sizes[] = {1, 2, 3, 7, 8, 40, 53, 54, 80, 160, 200, 400, 800};
  unsigned b;
  int n = o->cfg.n;
  for (b = 0; b < blocks; b++) {
    unsigned cnt = sizes[rmod((unsigned)(sizeof sizes / sizeof sizes[0]))];
    int kind = (int)rmod(7), mant = 1 + (int)rmod(255), shift = 4 + (int)rmod(12);
    unsigned usehz = hz;
    unsigned k = rmod(100);
    if (k < 30) usehz = hz + 40u + rmod(400);
    else if (k < 40 && hz > 300u) usehz = hz - 40u - rmod(250);
    if (k > 85) kind = 1;
    if (kind == 0 && rmod(3)) kind = 1;
    gen(g_sig, cnt, kind, usehz, mant, shift);
    if (rmod(25) == 0) {
      unsigned p = rmod(cnt), v = rmod(10);
      putb(&g_sig[p], v < 3 ? (0x7fc00000u | rnd() >> 12) : (v < 5 ? (0x7f800000u | ((rnd() & 1u) << 31)) : (v < 7 ? (1u + rmod(0x7fffffu)) : (0x7f000000u | rmod(0x800000u)))));
      cov_hit(CV_T_SPECIAL);
      if ((bf(g_sig[p]) & 0x7fffffffu) > 0x7f800000u) cov_hit(CV_T_NANIN);
      if ((bf(g_sig[p]) & 0x7fffffffu) == 0x7f800000u) cov_hit(CV_T_INFIN);
      if ((bf(g_sig[p]) & 0x7fffffffu) < 0x00800000u && (bf(g_sig[p]) & 0x7fffffffu)) cov_hit(CV_T_DENORM);
      if ((bf(g_sig[p]) & 0x7fffffffu) >= 0x7f000000u && (bf(g_sig[p]) & 0x7fffffffu) < 0x7f800000u) cov_hit(CV_T_HUGE);
    }
    if (o->idx + (int)cnt >= n && n > 0) cov_hit(CV_T_WRAP);
    det_call(o, g_sig, (short)cnt, tag);
    if (cnt == 1) cov_hit(CV_T_ONECNT);
  }
}
static void make_cfg(ToneCfg *c, int mode)
{
  memset(c, 0, sizeof *c);
  c->freq = mode ? fb(0x44000000u + (rnd() % 0x01000000u)) : fb(0x45034000u);
  putb(&c->f4, rmixb());
  putb(&c->f8, rmixb());
  c->fc = mode ? rfl(0x70, 0x80) : fb(0x3f400000u);
  putb(&c->f10, rmixb());
  c->f14 = mode ? rfl_pos(0x60, 0x7a) : fb(0x3bf5c28fu);
  putb(&c->f18, mode ? (rmod(4) ? (rflb(0x76, 0x7f) & 0x7fffffffu) : rmixb()) : 0x3f700000u);
  c->lpf = g_lpf;
  c->n = (short)(mode ? 1 + (int)rmod(60) : 53);
  /* the heap recorder maps a block of sizeof(Silence / Detector) bytes (20 / 56 at -m32, 32 / 96 on x86-64) to the blob size: keep the n * 4 byte blocks away from those sizes */
  if (c->n == 5 || c->n == 8 || c->n == 14 || c->n == 24) c->n++;
  c->n_hi = (short)rnd();
  c->w24 = (int)rnd();
  putb(&c->f28, rmixb());
  c->w2c = (int)rnd();
}
static void scen1_pass(unsigned scale)
{
  unsigned t, i;
  ToneObj *o;
  ToneCfg c;
  unsigned pad;
  static const unsigned det_freq[4] = {1296, 1100, 2100, 2225};
  make_lpf();
  /* A: NULL / NULL: TONE_CFG (2100 Hz, n = 53) */
  o = TONE_create(NULL, NULL);
  cov_hit(CV_T_CREATE_NULL);
  dump_tone(1, "T-null", o);
  for (i = 0; i < SC(300u * scale); i++) det_series(o, 2100, 1, "T-null");
  TONE_delete(o);
  cov_hit(CV_T_DELETE);
  /* B: the detector's four configs through an explicit cfg */
  for (t = 0; t < 4; t++) {
    ToneCfg d = TONE_CFG;
    char nm[24];
    d.freq = (t == 0) ? fb(0x44a28000u) : (t == 1) ? fb(0x44898000u) : (t == 2) ? fb(0x45034000u) : fb(0x450b1000u);
    d.fc = fb(0x3f400000u);
    snprintf(nm, sizeof nm, "T-cfg%u", t);
    o = TONE_create(NULL, &d);
    cov_hit(CV_T_CREATE_CFG);
    dump_tone(1, nm, o);
    for (i = 0; i < SC(120u * scale); i++) det_series(o, det_freq[t], 1, nm);
    putb(&o->accA, 0x80000000u);
    det_call(o, g_sig, 0, nm);
    o->accB = o->cfg.f14;
    det_call(o, g_sig, 0, nm);
    dump_tone(1, nm, o);
    TONE_delete(o);
  }
  /* C: random configs, fresh and existing objects, one object per config */
  for (t = 0; t < SC(260u * scale); t++) {
    char nm[24];
    int mode = (t % 5) != 0;
    int existing = (t & 1);
    unsigned hz;
    emit(0, "RNG t=%u %08x", t, g_lcg);
    make_cfg(&c, mode);
    if (t % 17 == 0) c.f18 = fb(0xbf000000u);
    if (t % 23 == 0) c.f14 = fb(0xbbcccccdu);
    if (t % 29 == 0) c.fc = fb(0xbf400000u);
    if (t % 31 == 0) c.f18 = fb(0x7fc00000u);
    if (t % 37 == 0) c.fc = fb(0x7fc00001u);
    if ((bf(c.f18) | bf(c.fc) | bf(c.f14)) >> 31) cov_hit(CV_T_NEGCFG);
    if ((bf(c.f18) & 0x7fffffffu) > 0x7f800000u || (bf(c.fc) & 0x7fffffffu) > 0x7f800000u) cov_hit(CV_T_NANCFG);
    snprintf(nm, sizeof nm, "T-rnd%u", t);
    hz = (c.freq > 100.0f && c.freq < 3900.0f) ? (unsigned)c.freq : 1000u;
    if (existing) {
      o = make_existing(&pad);
      if (t % 11 == 3) c.n = (short)(t % 3 == 0 ? 0 : -(int)(t % 7)); /* n <= 0: no loop, the blocks are untouched */
      if (c.n <= 0) cov_hit(CV_T_N0);
      o = TONE_create(o, &c);
      check_pads(o, pad);
      cov_hit(CV_T_CREATE_EXIST);
    } else {
      o = TONE_create(NULL, &c);
      cov_hit(CV_T_CREATE_RAND);
    }
    dump_tone(0, nm, o);
    if (c.n > 0) {
      det_series(o, hz, 6 + rmod(10), nm);
      if (existing && rmod(3) == 0) { /* an out of range ring index resets to 0 */
        o->idx = (short)(c.n + 1 + (int)rmod(100));
        cov_hit(CV_T_IDXRESET);
        gen(g_sig, 5, 1, 1000, 100, 8);
        det_call(o, g_sig, 5, nm);
      }
      det_call(o, g_sig, 0, nm); /* count 0 */
      cov_hit(CV_T_ZEROCNT);
      if (t % 3 == 0) { /* A = -0.0 is not clamped (A < 0 is false), B == f14 is not "below the minimum" */
        putb(&o->accA, 0x80000000u);
        det_call(o, g_sig, 0, nm);
        o->accB = o->cfg.f14;
        det_call(o, g_sig, 0, nm);
        putb(&o->accA, 0x00000001u);
        det_call(o, g_sig, 0, nm);
      }
      if (t % 6 == 1) { /* negative counts: 65536 + count samples */
        gen(g_sig, 65535, 3, 1500, 10, 10);
        det_call(o, g_sig, -1, nm);
        gen(g_sig, 65530, 1, 2100, 4, 6);
        det_call(o, g_sig, -6, nm);
        cov_hit(CV_T_NEGCNT);
      }
    }
    dump_tone(0, nm, o);
    if (existing) {
      /* the harness owns the blocks of an existing object: TONE_delete frees them only when n > 0 */
      if (c.n > 0)
        TONE_delete(o);
      else {
        sysdep_free(o->b8);
        sysdep_free(o->b14);
        sysdep_free(o->ring);
        sysdep_free(o->coef);
        TONE_delete(o);
      }
    } else
      TONE_delete(o);
    cov_hit(CV_T_DELETE);
    trial_end("tone");
  }
}
static void fail_create(void *arg)
{
  int point = (int)(intptr_t)arg;
  ToneObj *o;
  g_fail_malloc = point;
  g_malloc_ctr = 0;
  o = TONE_create(NULL, NULL);
  if (o) die3("TONE_create returned an object after a failure at point %d", point);
  if (g_live != 0) _exit(4);
}
static void scen1(void)
{
  unsigned pcd;
  int p;
  for (pcd = 0; pcd < 2; pcd++) {
    set_pc((int)pcd);
    if (pcd) {
      cov_hit(CV_T_PCDBL);
      emit(1, "== precision control double");
    }
    scen1_pass(1u);
    emit(1, "DETECT results %d %d %d", g_det_results[0], g_det_results[1], g_det_results[2]);
  }
  set_pc(0);
  /* malloc failures: the blob crashes (it never checks), the port returns NULL */
  for (p = 0; p < 5; p++) {
    char nm[40];
    snprintf(nm, sizeof nm, "TONE_create fail %d", p);
    guarded_report(nm, guarded(fail_create, (void *)(intptr_t)p));
    cov_hit(CV_T_FAIL_GUARDED);
  }
  trial_end("s1");
}

/* ---------------------------------------------------------------------------------------------- scenario 2: silence */
static unsigned g_hook_mode;
static const char *g_hook_pat;
static void dbg_hook(const char *msg)
{
  if (g_hook_mode && g_hook_pat && strstr(msg, g_hook_pat)) {
    dsplibs_debug_level = g_hook_mode - 1u; /* mode 1: level 0, mode 2: level 1 */
    cov_hit(CV_S_HOOK);
  }
}
static unsigned char g_buf[80];
static unsigned short g_len;
static void sil_dump(int show, const char *tag, const Silence *s)
{
  emit(show, "%s S ctx=%s cb=%s count=%u n=%u heard=%u E=%08x", tag, s->ctx == g_modem ? "ctx" : "BAD", s->cb == vp_cb ? "cb" : "BAD", (unsigned)s->count, (unsigned)s->n, (unsigned)s->heard, bf(s->energy));
}
static void sil_call(Silence *s, const float *x, short n, const char *tag)
{
  unsigned i;
  unsigned short l0;
  memset(g_buf, 0xA5, sizeof g_buf);
  switch (rmod(4)) {
    case 0: g_len = 0; break;
    case 1: g_len = (unsigned short)(0xfffe + rmod(2)); break;
    default: g_len = (unsigned short)rnd(); break;
  }
  l0 = g_len;
  silence_progress(s, x, n, g_buf, &g_len);
  for (i = 2; i < sizeof g_buf; i++)
    if (g_buf[i] != 0xA5) die3("silence_progress wrote beyond the 2 status bytes");
  emit(0, "SIL %s n=%d len %u -> %u buf %02x %02x", tag, (int)n, (unsigned)l0, (unsigned)g_len, g_buf[0], g_buf[1]);
  sil_dump(0, tag, s);
  if ((unsigned short)(g_len - l0) != 0) {
    if (g_buf[1] == 0x71) cov_hit(CV_S_REPORT71);
    if (g_buf[1] == 0x73) cov_hit(CV_S_REPORT73);
  }
  if (0) {
  } else if (((unsigned short)(g_len - l0)) & 1u)
    die3("silence_progress changed len by %u", (unsigned)(unsigned short)(g_len - l0));
  if ((unsigned short)(g_len - l0) != 0 && (g_buf[0] != 0x10 || (g_buf[1] != 0x71 && g_buf[1] != 0x73))) die3("silence_progress wrote a wrong status");
}
static void gen_noise_r(float *out, unsigned n, int R)
{
  unsigned i;
  for (i = 0; i < n; i++) out[i] = (float)((int)rmod((unsigned)(2 * R + 1)) - R) * 0.00000095367431640625f; /* r * 2^-20, exact */
}
static void gen_const(float *out, unsigned n, float a)
{
  unsigned i;
  for (i = 0; i < n; i++) out[i] = a;
}
static void sil_fresh(Silence *s, unsigned level, unsigned limit)
{
  g_cbv[0x52] = level;
  g_cbv[0x53] = limit;
  silence_create(s, g_modem, vp_cb);
}
static const unsigned lvl_amp[4] = {0u, 0x3acfda39u, 0x3b2d35dbu, 0x3b9be3abu};
static const int lvl_R[4] = {0, 2880, 4800, 8640};
static void scen2_pass(unsigned scale)
{
  Silence s, *p;
  unsigned t, lvl, k, i;
  int d;
  static const unsigned limits[] = {0u, 1u, 2u, 3u, 10u, 100u, 65535u, 65536u, 0x7fffffffu, 0x80000000u, 0x80000001u, 0x80000002u, 0xffffffffu, 0xfffffff0u, 0x7ffffffeu};
  static const unsigned lens[] = {1, 5, 799, 800, 801, 1600, 2400, 3000, 9000};
  static const unsigned levels[] = {0u, 1u, 2u, 3u, 4u, 5u, 0x10001u, 0xffff0002u, 0xffff0000u, 0x30003u};
  static const unsigned n_init[] = {0u, 1u, 400u, 790u, 799u, 800u, 801u, 1000u, 65535u};
  static const unsigned c_init[] = {0u, 1u, 2u, 5u, 65534u, 65535u};
  /* create */
  p = silence_create(NULL, g_modem, vp_cb);
  cov_hit(CV_S_NEW);
  sil_dump(1, "S-new", p);
  silence_delete(p);
  cov_hit(CV_S_DELETE);
  memset(&s, 0x5a, sizeof s);
  silence_create(&s, g_modem, vp_cb);
  cov_hit(CV_S_EXIST);
  sil_dump(1, "S-exist", &s);
  /* thresholds: constant amplitudes within +-40 ulps of sqrt(threshold), a window at a time */
  for (lvl = 1; lvl <= 3; lvl++)
    for (d = -40; d <= 40; d++)
      for (k = 0; k < 2; k++) {
        unsigned ab = lvl_amp[lvl] + (unsigned)d;
        float a = fb(ab | (k ? 0x80000000u : 0u));
        sil_fresh(&s, lvl, 0);
        gen_const(g_sig, 800, a);
        sil_call(&s, g_sig, 800, "S-sweep");
        if (s.heard) cov_hit(CV_S_SOUND);
        cov_hit(CV_S_SWEEP);
      }
  /* noise windows at the thresholds (the extended accumulator vs a float one decides here) */
  for (lvl = 1; lvl <= 3; lvl++)
    for (i = 0; i < SC(400u * scale); i++) {
      int R = lvl_R[lvl] - 160 + (int)rmod(321);
      sil_fresh(&s, lvl, 0);
      s.energy = 0.0f;
      gen_noise_r(g_sig, 1600, R);
      sil_call(&s, g_sig, 1600, "S-noise");
      cov_hit(CV_S_NOISE);
    }
  /* state grid */
  for (t = 0; t < SC(9000u * scale); t++) {
    unsigned L = levels[rmod((unsigned)(sizeof levels / sizeof levels[0]))];
    unsigned len = lens[rmod((unsigned)(sizeof lens / sizeof lens[0]))];
    unsigned lv = L & 0xffffu;
    int kind = (int)rmod(4);
    short cnt = (short)len;
    sil_fresh(&s, L, limits[rmod((unsigned)(sizeof limits / sizeof limits[0]))]);
    s.n = (unsigned short)n_init[rmod((unsigned)(sizeof n_init / sizeof n_init[0]))];
    s.count = (unsigned short)c_init[rmod((unsigned)(sizeof c_init / sizeof c_init[0]))];
    s.heard = (unsigned short)(rmod(3) == 0 ? 0 : (rmod(2) ? 1 : 0x100));
    putb(&s.energy, rmod(4) ? 0u : rmixb());
    if (s.n >= 800) cov_hit(CV_S_WRAPN);
    if (lv == 0) cov_hit(CV_S_LVL0);
    if (lv == 1) cov_hit(CV_S_LVL1);
    if (lv == 3) cov_hit(CV_S_LVL3);
    if (lv > 3) cov_hit(CV_S_LVLBIG);
    if (g_cbv[0x53] > 0x7fffffffu) cov_hit(CV_S_LIMITBIG);
    switch (kind) {
      case 0: gen_const(g_sig, len, 0.0f); break;
      case 1: gen_const(g_sig, len, fb(lvl_amp[1 + rmod(3)] + rmod(16)));  break;
      case 2: gen_noise_r(g_sig, len, lvl_R[1 + rmod(3)] + (int)rmod(60) - 30); break;
      default: gen_noise_r(g_sig, len, 40000); break;
    }
    if (rmod(8) == 0) putb(&g_sig[rmod(len)], rspecialb());
    if (rmod(40) == 0) cnt = (short)(rmod(2) ? 0 : -(int)(1 + rmod(5)));
    if (cnt <= 0) cov_hit(CV_S_CNT0);
    sil_call(&s, g_sig, cnt, "S-grid");
    if (s.energy == 0.0f && s.n == 0) cov_hit(CV_S_SILENT);
  }
  /* long runs: windows of silence, then sound, then silence again (the heard flag) */
  for (t = 0; t < SC(60u * scale); t++) {
    sil_fresh(&s, 1 + (t % 3), (t % 5) * 2u);
    for (k = 0; k < 14; k++) {
      unsigned len = 800 + 800 * rmod(3);
      if (rmod(3) == 0) gen_noise_r(g_sig, len, lvl_R[1 + (t % 3)] * 3);
      else gen_const(g_sig, len, 0.0f);
      sil_call(&s, g_sig, (short)len, "S-run");
    }
  }
  /* large counts */
  sil_fresh(&s, 2, 5);
  gen_noise_r(g_sig, 32767, 20);
  sil_call(&s, g_sig, 32767, "S-big");
  /* debug text and the level-lowering hook */
  for (d = 0; d < 4; d++) {
    unsigned hm;
    for (hm = 0; hm < 3; hm++) {
      static const char *const pats[] = {NULL, "energy", "silence counter"};
      for (k = 0; k < 3; k++) {
        dsplibs_debug_level = (unsigned)d;
        g_hook_mode = hm;
        g_hook_pat = pats[k];
        if (hm == 0 && k > 0) continue;
        sil_fresh(&s, 1 + k, 1);
        s.heard = (unsigned short)(k & 1);
        gen_noise_r(g_sig, 4000, k == 1 ? 8000 : 100);
        sil_call(&s, g_sig, 4000, "S-dbg");
        if (d > 1) cov_hit(CV_S_DBG);
      }
    }
  }
  /* the debug text with the huge limits: "silence counter %d, level %d" prints the int32 conversion of the callback value (> INT_MAX: the indefinite) */
  for (d = 2; d <= 3; d++)
    for (k = 0; k < sizeof limits / sizeof limits[0]; k++) {
      dsplibs_debug_level = (unsigned)d;
      sil_fresh(&s, 1 + (k % 3), limits[k]);
      gen_const(g_sig, 2400, 0.0f);
      sil_call(&s, g_sig, 2400, "S-dbglimit");
    }
  dsplibs_debug_level = 0;
  g_hook_mode = 0;
  /* is_more_then */
  {
    static const unsigned vb[] = {0x00000000u, 0x80000000u, 0x3dcccccdu, 0x41200000u, 0x4120c000u, 0x40a00000u, 0x40a00001u, 0x40a66666u, 0x40a66667u, 0x42c80000u, 0x45cccccdu, 0x45cccc00u,
                                  0x7f800000u, 0xff800000u, 0x7fc00000u, 0x7fc12345u, 0x7f7fffffu, 0x4effffffu, 0x4f000000u, 0xcf000000u, 0xcf000001u, 0x4cffffffu, 0x4d000000u, 0xbf800000u, 0xc1200000u, 0x3f800000u};
    static const unsigned cb[] = {0u, 1u, 2u, 9u, 10u, 11u, 99u, 100u, 101u, 65535u, 65534u, 32768u};
    for (i = 0; i < sizeof vb / sizeof vb[0]; i++)
      for (k = 0; k < sizeof cb / sizeof cb[0]; k++) {
        int r;
        silence_create(&s, g_modem, vp_cb);
        s.count = (unsigned short)cb[k];
        r = silence_is_more_then(&s, fb(vb[i]));
        emit(0, "MORE v=%08x count=%u -> %d", vb[i], cb[k], r);
        if ((vb[i] & 0x7fffffffu) > 0x7f800000u) cov_hit(CV_S_MORE_NAN);
        if ((vb[i] & 0x7fffffffu) >= 0x4f000000u) cov_hit(CV_S_MORE_BIG);
      }
    for (t = 0; t < SC(20000u * scale); t++) {
      unsigned vbits = rmixb();
      float v;
      memcpy(&v, &vbits, 4);
      silence_create(&s, g_modem, vp_cb);
      s.count = (unsigned short)rnd();
      emit(0, "MORE v=%08x count=%u -> %d", vbits, (unsigned)s.count, silence_is_more_then(&s, v));
    }
  }
  /* _status */
  for (d = 0; d < 4; d++)
    for (i = 0; i < 300; i++) {
      unsigned short l;
      int code = (i < 256) ? (int)i - (i >= 128 ? 256 : 0) : (int)(rnd() & 0xffffu) - 20000;
      dsplibs_debug_level = (unsigned)d;
      memset(g_buf, 0x3c, sizeof g_buf);
      l = (unsigned short)(rmod(3) == 0 ? 0xffffu - rmod(3) : rnd());
      g_len = l;
      _status(g_buf, &g_len, code);
      emit(0, "STATUS code=%d lvl=%d len %u -> %u buf %02x %02x %02x", code, d, (unsigned)l, (unsigned)g_len, g_buf[0], g_buf[1], g_buf[2]);
      cov_hit(CV_S_STATUS);
    }
  dsplibs_debug_level = 0;
}
static void sil_fail(void *arg)
{
  Silence *s;
  (void)arg;
  g_fail_malloc = 0;
  g_malloc_ctr = 0;
  s = silence_create(NULL, g_modem, vp_cb);
  if (s) die3("silence_create returned an object after a failure");
}
static void scen2(void)
{
  unsigned pcd;
  for (pcd = 0; pcd < 2; pcd++) {
    set_pc((int)pcd);
    if (pcd) {
      cov_hit(CV_S_PCDBL);
      emit(1, "== precision control double");
    }
    scen2_pass(1u);
  }
  set_pc(0);
  /* a failing allocation: the blob checks it too, compared directly (not guarded) */
  g_fail_malloc = 0;
  g_malloc_ctr = 0;
  {
    Silence *s = silence_create(NULL, g_modem, vp_cb);
    emit(1, "silence_create malloc failure -> %s", s ? "object" : "NULL");
    cov_hit(CV_S_FAIL0);
    g_fail_malloc = -1;
  }
  (void)sil_fail;
  trial_end("s2");
}

/* ---------------------------------------------------------------------------------------------- scenario 3: detector */
static void det_dump(int show, const char *tag, const Detector *d)
{
  int i;
  emit(show, "%s D enable=%04x dtmf=A%u cad8=A%u cadA=A%u cadB=A%u cnt=%u %u %u %u w2c=%d w30=%u out=%d pad02=%u", tag, (unsigned)d->enable, blk_id_of(d->dtmf), blk_id_of(d->cad8), blk_id_of(d->cadBusy),
       blk_id_of(d->cadDial), (unsigned)d->cnt[0], (unsigned)d->cnt[1], (unsigned)d->cnt[2], (unsigned)d->cnt[3], d->w2c, d->w30, d->out_mode, (unsigned)d->pad02);
  for (i = 0; i < 4; i++) {
    char nm[48];
    snprintf(nm, sizeof nm, "%s tone%d A%u", tag, i, blk_id_of(d->tone[i]));
    if (d->tone[i]) dump_tone(0, nm, d->tone[i]);
  }
}
static int g_dp_results[8];
static int dp_call(Detector *d, const float *x, short n, const char *tag)
{
  unsigned short l0, cnt0[4];
  int r, i;
  memset(g_buf, 0xA5, sizeof g_buf);
  g_len = (unsigned short)(rmod(4) == 0 ? 0xfffeu + rmod(2) : rnd());
  l0 = g_len;
  for (i = 0; i < 4; i++) cnt0[i] = d->cnt[i];
  r = detector_progress(d, x, n, g_buf, &g_len);
  for (i = 4; i < (int)sizeof g_buf; i++)
    if (g_buf[i] != 0xA5) die3("detector_progress wrote beyond 4 bytes");
  if (g_dtmf_calls == 0xffffffffu) die3("counter");
  emit(0, "DP %s n=%d -> %d len %u -> %u buf %02x %02x %02x %02x cnt %u %u %u %u", tag, (int)n, r, (unsigned)l0, (unsigned)g_len, g_buf[0], g_buf[1], g_buf[2], g_buf[3], (unsigned)d->cnt[0],
       (unsigned)d->cnt[1], (unsigned)d->cnt[2], (unsigned)d->cnt[3]);
  if (r >= 0 && r < 8) g_dp_results[r]++;
  for (i = 0; i < 4; i++)
    if (cnt0[i] != 0 && d->cnt[i] == 0) cov_hit(CV_D_TONE_CNTRESET);
  if (d->out_mode == 0 && r >= 3 && r <= 6) cov_hit(CV_D_TONE_STATUS);
  if (d->out_mode != 0 && (g_len - l0) != 0 && g_buf[0] == 0x10 && (strchr("ecaf", g_buf[1]) || strchr("ecaf", g_buf[3])))
    cov_hit(CV_D_TONE_STREAM);
  if (d->out_mode == 0 && (r == 1)) cov_hit(CV_D_CAD_STATUS);
  if (d->out_mode == 0 && (r == 2)) cov_hit(CV_D_CAD_STATUS);
  if (d->out_mode != 0 && (g_buf[1] == 0x62 || g_buf[1] == 0x64 || g_buf[3] == 0x62 || g_buf[3] == 0x64)) cov_hit(CV_D_CAD_STREAM);
  if (g_buf[1] != 0xA5 && g_buf[1] != 'f' && g_buf[1] != 'a' && g_buf[1] != 'c' && g_buf[1] != 'e' && g_buf[1] != 0x62 && g_buf[1] != 0x64 && g_buf[0] == 0x10 && g_buf[2] != 0xA5) cov_hit(CV_D_BUFADV);
  return r;
}
static Detector *new_detector(unsigned cbv49, int withcb)
{
  Detector *d;
  g_cbv[0x49] = cbv49;
  g_dtmf_script = 0;
  d = detector_create(NULL, g_modem, withcb ? vp_cb : NULL);
  if (!d) die3("detector_create failed");
  g_cadA = d->cadBusy;
  g_cadB = d->cadDial;
  return d;
}
static void del_detector(Detector *d)
{
  detector_delete(d);
  cov_hit(CV_D_DELETE);
  trial_end("detector");
}
/* an existing detector object: a dirty heap block with existing dtmf / tones / cadences (the harness owns them; detector_delete frees all of it) */
static Detector *make_existing_det(unsigned *pads, int with_cad8)
{
  Detector *d = (Detector *)sysdep_malloc((unsigned)sizeof(Detector));
  unsigned i;
  unsigned char *bb = (unsigned char *)d;
  CadenceParams cp;
  for (i = 0; i < 0x38; i++) bb[i] = (unsigned char)rnd();
  memset(&cp, 0, sizeof cp);
  d->dtmf = create_dtmf(NULL);
  for (i = 0; i < 4; i++) d->tone[i] = make_existing(&pads[i]);
  d->cadBusy = cadence_create(NULL, &cp, 0, g_modem);
  d->cadDial = cadence_create(NULL, &cp, 0, g_modem);
  d->cad8 = with_cad8 ? cadence_create(NULL, &cp, 0, g_modem) : NULL;
  return d;
}
static void tone_blocks(Detector *d, unsigned hz, unsigned blocks, const char *tag, int amp_shift)
{
  unsigned b;
  for (b = 0; b < blocks; b++) {
    unsigned cnt = 80 + 40 * rmod(8);
    gen(g_sig, cnt, 1, hz, 200, amp_shift);
    dp_call(d, g_sig, (short)cnt, tag);
  }
}
static void det_fail(void *arg)
{
  int point = (int)(intptr_t)arg;
  Detector *d;
  g_fail_malloc = point;
  g_malloc_ctr = 0;
  g_cbv[0x49] = 8000;
  d = detector_create(NULL, g_modem, vp_cb);
  if (d) detector_delete(d);
  if (g_live != 0) _exit(4);
}
static void scen3_pass(unsigned scale)
{
  static const unsigned hz[4] = {1296, 1100, 2100, 2225};
  static const unsigned cbvals[] = {0u, 1u, 8000u, 12345u, 0x7fffffffu, 0x80000000u, 0xffffffffu, 0x04000001u, 3u};
  static const float cad_special[] = {0.0f, -0.0f};
  Detector *d;
  unsigned t, m, i, k;
  (void)cad_special;
  /* create: NULL, with / without the callback */
  for (t = 0; t < sizeof cbvals / sizeof cbvals[0]; t++) {
    d = new_detector(cbvals[t], 1);
    cov_hit(CV_D_NEW);
    cov_hit(CV_D_CB);
    det_dump(1, "D-new", d);
    del_detector(d);
  }
  d = new_detector(5, 0);
  cov_hit(CV_D_CBNULL);
  det_dump(1, "D-nocb", d);
  del_detector(d);
  /* create on an existing (dirty) object with existing tones / dtmf / cadences; delete with and without cad8 */
  for (t = 0; t < 4; t++) {
    unsigned padv[4];
    d = make_existing_det(padv, t & 1);
    g_cbv[0x49] = 77u + t;
    g_cadA = d->cadBusy;
    g_cadB = d->cadDial;
    detector_create(d, g_modem, t == 3 ? NULL : vp_cb);
    cov_hit(CV_D_EXIST);
    det_dump(1, "D-exist", d);
    g_cad_script = 1;
    tone_blocks(d, hz[t & 3], 4, "D-exist", 6);
    for (i = 0; i < 4; i++) check_pads(d->tone[i], padv[i]);
    if (t & 1) cov_hit(CV_D_DELETE_CAD8);
    del_detector(d);
  }
  /* the set_* functions */
  d = new_detector(100, 1);
  detector_set_output_status(d);
  emit(1, "set_output_status out=%d", d->out_mode);
  detector_set_output_in_stream(d);
  emit(1, "set_output_in_stream out=%d", d->out_mode);
  for (t = 0; t < 12; t++) {
    unsigned short v = (unsigned short)(t < 6 ? t * 0x0d : rnd());
    detector_set_enable(d, v);
    emit(0, "set_enable %04x -> %04x", (unsigned)v, (unsigned)d->enable);
  }
  cov_hit(CV_D_SETS);
  del_detector(d);
  /* progress: every enable mask x both output modes x tones at the four frequencies */
  for (m = 0; m < 64; m++) {
    for (k = 0; k < 2; k++) {
      d = new_detector(8000, 1);
      detector_set_enable(d, (unsigned short)m);
      if (k) detector_set_output_status(d);
      g_cad_script = (m % 4);
      g_dtmf_script = (int)(m & 1);
      g_w90 = 0;
      for (t = 0; t < 4; t++) tone_blocks(d, hz[t], SW(36u * scale), "D-mask", 6 + (int)(m % 3));
      det_dump(0, "D-mask", d);
      cov_hit(CV_D_MASKS);
      del_detector(d);
    }
  }
  /* the dtmf busy word, the dtmf scripts, special samples, high enable bits */
  for (t = 0; t < SC(40u * scale); t++) {
    unsigned blocks = 30 + rmod(30);
    d = new_detector(8000, 1);
    detector_set_enable(d, (unsigned short)(rmod(3) == 0 ? rnd() : 0x3f));
    if (rmod(2)) detector_set_output_status(d);
    g_cad_script = rmod(4);
    g_dtmf_script = (int)rmod(2);
    g_w90 = 0;
    for (k = 0; k < blocks; k++) {
      unsigned cnt = 1 + rmod(300), kind = rmod(4);
      unsigned short w = (unsigned short)(rmod(4) == 0 ? 1 + rmod(3) : 0);
      *(unsigned short *)((char *)d->dtmf + 0x90) = w;
      if (w) cov_hit(CV_D_BUSY90);
      if (kind == 0) gen(g_sig, cnt, 1, hz[rmod(4)], 200, 6);
      else if (kind == 1) gen(g_sig, cnt, 3, hz[rmod(4)], 100, 6);
      else if (kind == 2) gen(g_sig, cnt, 2, 0, 1 + (int)rmod(255), 4 + (int)rmod(10));
      else gen(g_sig, cnt, 0, 0, 1, 1);
      if (rmod(6) == 0) {
        static const unsigned sp[] = {0x4003126fu, 0x4003126eu, 0x40031370u, 0xc003126fu, 0xc0031375u, 0xc0031376u, 0x7fc00000u, 0x7f800000u, 0xff800000u, 0x00000001u, 0x80000001u, 0x3f800000u, 0x40000000u,
                                      0x4effffffu, 0x7f7fffffu, 0x3a83126fu, 0xba83126fu, 0x3a800000u, 0x7f800001u};
        unsigned ii;
        for (ii = 0; ii < 3; ii++) putb(&g_sig[rmod(cnt)], sp[rmod((unsigned)(sizeof sp / sizeof sp[0]))]);
        cov_hit(CV_D_SPECIALX);
      }
      dp_call(d, g_sig, (short)cnt, "D-rand");
    }
    det_dump(0, "D-rand", d);
    del_detector(d);
  }
  /* cadence samples: the 16000 scale and the int16 truncation boundaries, +-64 ulps around +-2.048 */
  g_w90 = 0;
  for (t = 0; t < 4; t++) {
    static const unsigned base[4] = {0x4003126fu, 0xc003126fu, 0x40031375u, 0xc0031375u};
    d = new_detector(1, 1);
    detector_set_enable(d, 0x20);
    detector_set_output_status(d);
    g_cad_script = 3;
    for (i = 0; i < 129; i++) {
      unsigned cnt = 8;
      for (k = 0; k < cnt; k++) putb(&g_sig[k], base[t] + i - 64u + k);
      dp_call(d, g_sig, (short)cnt, "D-cadsweep");
    }
    del_detector(d);
  }
  cov_hit(CV_D_SPECIALX);
  /* debug text + the hooks */
  for (t = 0; t < 3; t++) {
    unsigned lvl;
    for (lvl = 0; lvl < 4; lvl++) {
      d = new_detector(8000, 1);
      dsplibs_debug_level = lvl;
      detector_set_enable(d, 0x3f);
      g_cad_script = 2;
      g_dtmf_script = 1;
      g_hook_mode = t ? 1 : 0;
      g_hook_pat = t == 1 ? "dtmf" : "detected";
      for (k = 0; k < 40; k++) {
        unsigned cnt = 160;
        gen(g_sig, cnt, 1, hz[k & 3], 200, 6);
        if (k % 2) detector_set_output_status(d);
        else detector_set_output_in_stream(d);
        dp_call(d, g_sig, (short)cnt, "D-dbg");
      }
      if (lvl > 1) cov_hit(CV_D_DBG);
      dsplibs_debug_level = 0;
      g_hook_mode = 0;
      del_detector(d);
    }
  }
}
static void scen3_extra(void)
{
  static const unsigned hz[4] = {1296, 1100, 2100, 2225};
  Detector *d;
  unsigned t, lvl, k;
  /* a tone held for 60 blocks with the debug text on (the "detected %c" message), both output modes */
  for (lvl = 2; lvl <= 3; lvl++)
    for (t = 0; t < 4; t++) {
      d = new_detector(8000, 1);
      dsplibs_debug_level = lvl;
      g_dtmf_script = 0;
      g_cad_script = 0;
      g_w90 = 0;
      detector_set_enable(d, (unsigned short)(2u << t));
      if (lvl == 3) detector_set_output_status(d);
      tone_blocks(d, hz[t], 60, "D-held", 6);
      dsplibs_debug_level = 0;
      del_detector(d);
    }
  /* a negative count with the cadences enabled: no sample is processed */
  d = new_detector(8000, 1);
  detector_set_enable(d, 0x20);
  g_cad_script = 1;
  for (k = 0; k < 4; k++) {
    gen(g_sig, 8, 1, 1296, 100, 8);
    dp_call(d, g_sig, (short)(-1 - (int)k), "D-negcount");
  }
  dp_call(d, g_sig, 0, "D-zerocount");
  del_detector(d);
}
static void scen3(void)
{
  unsigned pcd;
  int p;
  Detector *d;
  g_dbg_hook = dbg_hook;
  g_hook_pat = NULL;
  for (pcd = 0; pcd < 2; pcd++) {
    set_pc((int)pcd);
    if (pcd) emit(1, "== precision control double");
    scen3_pass(pcd ? 1u : 1u);
    scen3_extra();
    emit(1, "DETECTOR results %d %d %d %d %d %d %d %d", g_dp_results[0], g_dp_results[1], g_dp_results[2], g_dp_results[3], g_dp_results[4], g_dp_results[5], g_dp_results[6], g_dp_results[7]);
  }
  set_pc(0);
  /* allocation failures: #0 (the detector itself), #1 (create_dtmf's block), #22 / #23 (the cadence recorders' blocks) are compared; #2 .. #21 (TONE_create) crash the blob */
  {
    static const int cmp_points[] = {0, 1, 22, 23};
    for (p = 0; p < 4; p++) {
      g_fail_malloc = cmp_points[p];
      g_malloc_ctr = 0;
      g_cbv[0x49] = 8000;
      d = detector_create(NULL, g_modem, vp_cb);
      emit(1, "detector_create fail %d -> %s", cmp_points[p], d ? "object" : "NULL");
      if (p == 0) {
        cov_hit(CV_D_FAIL0);
        if (d) die3("detector_create returned an object after malloc #0 failed");
      }
      g_fail_malloc = -1;
      if (d) {
        int q;
        det_dump(1, "D-fail", d);
        /* detector_delete would sysdep_free(NULL) (a harness violation) when a recorder block failed: release the pieces by hand */
        if (d->dtmf) sysdep_free(d->dtmf);
        for (q = 0; q < 4; q++) TONE_delete(d->tone[q]);
        if (d->cadBusy) cadence_delete(d->cadBusy);
        if (d->cadDial) cadence_delete(d->cadDial);
        sysdep_free(d);
      }
    }
  }
  for (p = 2; p <= 21; p++) {
    char nm[40];
    snprintf(nm, sizeof nm, "detector_create fail %d", p);
    guarded_report(nm, guarded(det_fail, (void *)(intptr_t)p));
    cov_hit(CV_D_FAIL_GUARDED);
  }
  trial_end("s3");
}

/* ---------------------------------------------------------------------------------------------- scenario 4: GetGain */
typedef struct {
  struct modem *m;
  unsigned char rest[0x40];
} FakeBeep;
static void gg_one(Acc *a, int p14, int p24, int p3c, int show)
{
  FakeBeep b;
  float ob = fb(0x7fa5a5a5u), oa = fb(0x7fa5a5a5u);
  b.m = g_modem;
  memset(b.rest, 0, sizeof b.rest);
  g_mgp[0x14] = p14;
  g_mgp[0x24] = p24;
  g_mgp[0x3c] = p3c;
  CALL_GETGAIN(&b, &ob, &oa);
  acc_u32(a, bf(ob));
  acc_u32(a, bf(oa));
  if (show || g_verbose) emit(0, "GG %d %d %d -> %08x %08x", p14, p24, p3c, bf(ob), bf(oa));
}
static void scen4(void)
{
  Acc a;
  int p14, p24, p3c, lo = -100, hi = 100;
  unsigned i, t, lvl;
  static const int ext[] = {0x80000000, 0x80000001, -1073741824, -100000, -1000, -401, -400, -399, -64, -7, -1, 0, 1, 5, 6, 7, 63, 64, 399, 400, 401, 1000, 100000, 1073741824, 0x7ffffffe, 0x7fffffff};
  g_dbg_hook = dbg_hook;
  /* the debug text first (few calls: every text is compared) */
  for (lvl = 0; lvl < 4; lvl++) {
    unsigned hm;
    for (hm = 0; hm < 3; hm++) {
      static const char *const pats[] = {NULL, "Additional", "GAIN1"};
      unsigned k;
      for (k = 0; k < 3; k++) {
        if (hm == 0 && k > 0) continue;
        dsplibs_debug_level = lvl;
        g_hook_mode = hm;
        g_hook_pat = pats[k];
        acc_init(&a);
        gg_one(&a, 10, -30, 12, 1);
        gg_one(&a, 3, 0, -5, 1);
        gg_one(&a, 0x7fffffff, 0x80000000, 7, 1);
        if (lvl == 0) cov_hit(CV_G_DBG0);
        if (lvl > 1 && hm == 0) cov_hit(CV_G_DBG2);
        if (hm && k == 1) cov_hit(CV_G_HOOK1);
        if (hm && k == 2) cov_hit(CV_G_HOOK2);
      }
    }
  }
  dsplibs_debug_level = 0;
  g_hook_mode = 0;
  g_quiet = 1;
  /* the extreme parameters (pow overflow / underflow, INT_MIN wraps) */
  acc_init(&a);
  for (i = 0; i < sizeof ext / sizeof ext[0]; i++)
    for (t = 0; t < sizeof ext / sizeof ext[0]; t++)
      for (p3c = 0; p3c < 8; p3c++) gg_one(&a, ext[i], ext[t], ext[(i * 7 + t * 3 + (unsigned)p3c * 5) % (sizeof ext / sizeof ext[0])], 0);
  for (i = 0; i < SW(300000u); i++) {
    int v[3], k;
    for (k = 0; k < 3; k++) {
      unsigned sh = rmod(32);
      v[k] = (int)(rnd() >> sh);
      if (rmod(2)) v[k] = -v[k];
    }
    gg_one(&a, v[0], v[1], v[2], 0);
  }
  cov_hit(CV_G_EXTREME);
  acc_emit("getgain extremes", &a);
  /* the cube: plausible parameter values */
  acc_init(&a);
  if (scale_div * sweep_div > 1) {
    lo = -100 / (int)(scale_div * sweep_div > 40 ? 40 : scale_div * sweep_div);
    hi = -lo;
  }
  for (p14 = lo; p14 <= hi; p14++)
    for (p3c = lo; p3c <= hi; p3c++)
      for (p24 = lo; p24 <= hi; p24++) gg_one(&a, p14, p24, p3c, 0);
  cov_hit(CV_G_CUBE);
  acc_emit("getgain cube", &a);
  g_quiet = 0;
  trial_end("s4");
}

/* ---------------------------------------------------------------------------------------------- scenario 5: the blob's own pow */
static void pow_one(Acc *a, double x, double y)
{
  long double r = CALL_POW(x, y);
  unsigned char raw[10];
  memcpy(raw, &r, 10);
  acc_bytes(a, raw, 10);
  if (g_verbose) {
    unsigned long long ux, uy;
    memcpy(&ux, &x, 8);
    memcpy(&uy, &y, 8);
    emit(0, "POW %016llx %016llx -> %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", ux, uy, raw[9], raw[8], raw[7], raw[6], raw[5], raw[4], raw[3], raw[2], raw[1], raw[0]);
  }
}
static double dbl_bits(unsigned long long u)
{
  double d;
  memcpy(&d, &u, 8);
  return d;
}
static double rdbl(unsigned lo, unsigned hi)
{
  unsigned long long m = ((unsigned long long)rnd() << 32 | rnd()) & 0x000fffffffffffffULL;
  unsigned long long e = lo + rmod(hi - lo + 1);
  unsigned long long sgn = (unsigned long long)(rnd() & 1u) << 63;
  return dbl_bits(sgn | (e << 52) | m);
}
static const unsigned long long dspecial[] = {0x0000000000000000ULL, 0x8000000000000000ULL, 0x0000000000000001ULL, 0x8000000000000001ULL, 0x000fffffffffffffULL, 0x0010000000000000ULL, 0x7fefffffffffffffULL,
                                              0xffefffffffffffffULL, 0x7ff0000000000000ULL, 0xfff0000000000000ULL, 0x7ff8000000000000ULL, 0xfff8000000000000ULL, 0x7ff0000000000001ULL, 0xfff4000000000000ULL,
                                              0x3ff0000000000000ULL, 0xbff0000000000000ULL, 0x4000000000000000ULL, 0xc000000000000000ULL, 0x3fe0000000000000ULL, 0xbfe0000000000000ULL, 0x4008000000000000ULL,
                                              0xc008000000000000ULL, 0x4024000000000000ULL, 0x4340000000000000ULL, 0x43e0000000000000ULL, 0xc3e0000000000000ULL, 0x43dfffffffffffffULL, 0x4330000000000000ULL,
                                              0x4320000000000000ULL, 0xc330000000000000ULL, 0x3fd28f5c28f5c28fULL, 0x3fd28f5c28f5c290ULL, 0x3fd28f5c28f5c28eULL, 0x3ff28f5c28f5c28fULL, 0x3fe6666666666666ULL,
                                              0x4090000000000000ULL, 0xc090000000000000ULL, 0x40d0000000000000ULL, 0x4059000000000000ULL, 0x3ff0000000000002ULL, 0x3fefffffffffffffULL, 0x3ff0000000000001ULL};
#define NDSPECIAL (sizeof dspecial / sizeof dspecial[0])
static void pow_all(const char *tag, unsigned scale)
{
  Acc a;
  unsigned i, j, k;
  char nm[48];
  acc_init(&a);
  for (i = 0; i < NDSPECIAL; i++)
    for (j = 0; j < NDSPECIAL; j++) {
      double x = dbl_bits(dspecial[i]), y = dbl_bits(dspecial[j]);
      pow_one(&a, x, y);
      if (y == 0) cov_hit(CV_W_ZEROY);
      if (y != y) cov_hit(CV_W_NANY);
      if (y == dbl_bits(0x7ff0000000000000ULL) || y == dbl_bits(0xfff0000000000000ULL)) cov_hit(CV_W_INFY);
      if (x == 0) cov_hit(CV_W_ZEROX);
      if (x != x) cov_hit(CV_W_NANX);
      if (x == dbl_bits(0x7ff0000000000000ULL) || x == dbl_bits(0xfff0000000000000ULL)) cov_hit(CV_W_INFX);
      if (x < 0) cov_hit(CV_W_NEGX);
    }
  snprintf(nm, sizeof nm, "%s specials", tag);
  acc_emit(nm, &a);
  /* integer exponents, +-0 / +-inf bases with odd / even / huge integers */
  acc_init(&a);
  for (i = 0; i < NDSPECIAL; i++)
    for (k = 0; k < 400; k++) {
      int e = (int)k - 200;
      pow_one(&a, dbl_bits(dspecial[i]), (double)e);
      pow_one(&a, dbl_bits(dspecial[i]), (double)(e * 5000011));
      pow_one(&a, dbl_bits(dspecial[i]), dbl_bits(0x4330000000000000ULL + (unsigned long long)k * 3ULL));
      pow_one(&a, dbl_bits(dspecial[i]), dbl_bits(0xc330000000000000ULL + (unsigned long long)k * 3ULL));
      pow_one(&a, dbl_bits(dspecial[i]), dbl_bits(0x43d0000000000000ULL + (unsigned long long)k * 0x100000000ULL));
      pow_one(&a, dbl_bits(dspecial[i]), dbl_bits(0xc3d0000000000000ULL + (unsigned long long)k * 0x100000000ULL));
      pow_one(&a, dbl_bits(dspecial[i]), dbl_bits(0x41f0000000000000ULL + (unsigned long long)k * 0x80000000ULL));
    }
  cov_hit(CV_W_INTY);
  cov_hit(CV_W_BIGINT);
  cov_hit(CV_W_NEGINT);
  cov_hit(CV_W_ODD);
  /* large ODD integers (hi dword of the int64 near the 0x200000 / 0xffe00000 thresholds, 2^32 + 1, 2^53 - 1) against +-0 / +-inf bases and ordinary bases */
  {
    static const unsigned long long odd[] = {4294967297ULL, 8589934591ULL, 1099511627777ULL, 4503599627370497ULL, 9007199254740991ULL, 9007199254740989ULL, 9007194959773697ULL, 9007194959773699ULL,
                                             9007190664806401ULL, 9007190664806403ULL, 562949953421311ULL, 140737488355327ULL, 2147483649ULL, 4294967295ULL, 6442450943ULL, 3ULL, 5ULL, 4611686018427387903ULL};
    static const unsigned long long bases[] = {0x0000000000000000ULL, 0x8000000000000000ULL, 0x7ff0000000000000ULL, 0xfff0000000000000ULL, 0x4024000000000000ULL, 0xc008000000000000ULL, 0x3fe0000000000000ULL};
    for (i = 0; i < sizeof odd / sizeof odd[0]; i++)
      for (j = 0; j < sizeof bases / sizeof bases[0]; j++)
        for (k = 0; k < 4; k++) {
          double y = (double)odd[i];
          if (k & 1) y = -y;
          if (k & 2) y = (double)(odd[i] - 2ULL * (odd[i] > 1000 ? 1ULL : 0ULL)) * (k & 1 ? -1.0 : 1.0);
          pow_one(&a, dbl_bits(bases[j]), y);
        }
  }
  snprintf(nm, sizeof nm, "%s integer y", tag);
  acc_emit(nm, &a);
  /* bases near one (the fyl2xp1 / fyl2x switch at |x - 1| = 0.29): exact grid points 1 + k * 2^-21 and +-ulps around 0.71 / 1.29 with fractional exponents */
  acc_init(&a);
  for (i = 0; i < SW(120000u * scale); i++) {
    double x = 1.0 + (double)((int)(rnd() % 1300001u) - 650000) * 0.000000476837158203125;
    double y = (double)((int)(rnd() % 4001u) - 2000) * 0.015625 + 0.0078125;
    pow_one(&a, x, y);
    if (x > 0.7 && x < 1.3) cov_hit(CV_W_NEARONE);
  }
  for (i = 0; i < 400; i++) {
    double y = (double)((int)(i % 40) - 20) * 0.1171875 + 0.00390625;
    pow_one(&a, dbl_bits(0x3ff28f5c28f5c28fULL + i - 200u), y);
    pow_one(&a, dbl_bits(0x3fe6b851eb851eb8ULL + i - 200u), y);
    pow_one(&a, dbl_bits(0x3ff0000000000000ULL + i), y);
    pow_one(&a, dbl_bits(0x3fefffffffffffffULL - i), y);
  }
  snprintf(nm, sizeof nm, "%s near one", tag);
  acc_emit(nm, &a);
  /* GetGain's exponents: pow(10, k * 0.05 + m * 0.05 ...) as doubles, and the neighbours */
  acc_init(&a);
  /* the exponents are exact binary fractions (k / 64, k / 256): double arithmetic of the harness itself must not differ between the x87 (-m32) and SSE builds */
  for (i = 0; i < 8001; i++) {
    double y = (double)((int)i - 4000) * 0.015625;
    pow_one(&a, 10.0, y);
    pow_one(&a, 10.0, dbl_bits(0x3ff0000000000000ULL + i));
    pow_one(&a, 10.0, dbl_bits(0xc010000000000000ULL + i));
  }
  for (i = 0; i < SW(80000u * scale); i++) pow_one(&a, 10.0, (double)((int)(rnd() % 160001u) - 80000) * 0.00390625);
  cov_hit(CV_W_GETGAINY);
  snprintf(nm, sizeof nm, "%s getgain", tag);
  acc_emit(nm, &a);
  /* near one (the fyl2xp1 path) and general bases / exponents */
  acc_init(&a);
  for (i = 0; i < SW(300000u * scale); i++) {
    double x, y;
    switch (rmod(6)) {
      case 0: x = dbl_bits(0x3fd0000000000000ULL + (((unsigned long long)rnd() << 20) & 0x3ffffffffffffffULL) % 0x0030000000000000ULL); y = rdbl(0x3f0, 0x410); cov_hit(CV_W_NEARONE); break;
      case 1: x = rdbl(0x3fe, 0x401); y = rdbl(0x3f0, 0x405); break;
      case 2: x = rdbl(0x380, 0x480); y = rdbl(0x3e0, 0x400); break;
      case 3: x = rdbl(0x3ff, 0x400); y = (double)((int)(rnd() % 301u) - 150); break;
      case 4: x = rdbl(0x3e0, 0x420); y = rdbl(0x400, 0x40d); break;
      default: x = dbl_bits(((unsigned long long)rnd() << 32) | rnd()); y = dbl_bits(((unsigned long long)rnd() << 32) | rnd()); break;
    }
    pow_one(&a, x, y);
    if (y == (double)(long long)y) cov_hit(CV_W_INTY); else cov_hit(CV_W_FRACY);
  }
  snprintf(nm, sizeof nm, "%s random", tag);
  acc_emit(nm, &a);
}
static void scen5(void)
{
  set_pc(0);
  pow_all("pow", 1);
  set_pc(1);
  cov_hit(CV_W_PCDBL);
  emit(1, "== precision control double");
  pow_all("pow-pcd", 1);
  set_pc(0);
  trial_end("s5");
}

int main(int argc, char **argv)
{
  int scen = argc > 1 ? atoi(argv[1]) : -1;
  const char *e;
  if (scen < 0 || scen > 5) {
    fprintf(stderr, "usage: %s scenario(0..5)\n", argv[0]);
    return 2;
  }
  g_verbose = (e = getenv("R15_VERBOSE")) != NULL && *e == '1';
  if ((e = getenv("R15_SEED")) != NULL) g_lcg ^= (unsigned)strtoul(e, NULL, 0);
  if ((e = getenv("R15_SCALE")) != NULL && atoi(e) > 0) scale_div = (unsigned)atoi(e);
  if ((e = getenv("R15_SWEEP")) != NULL && atoi(e) > 0) sweep_div = (unsigned)atoi(e);
  g_lcg += (unsigned)scen * 0x9e3779b9u;
  g_dbg_hook = dbg_hook;
  cov_setup(scen);
  switch (scen) {
    case 0: scen0(); break;
    case 1: scen1(); break;
    case 2: scen2(); break;
    case 3: scen3(); break;
    case 4: scen4(); break;
    case 5: scen5(); break;
    default: break;
  }
  cov_finish();
  return 0;
}
