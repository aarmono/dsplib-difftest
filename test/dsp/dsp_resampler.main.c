/*
 * Differential test harness for port chunk P05 (src/dsp/dsp_resampler.cpp): the Resampler hierarchy of dsplibs.o - Resampler,
 * ResamplerTimingOffset, ResamplerTiming, V90Resampler (51 functions, 4 vtables, the only virtual dispatch of the V.90 / V.92 cluster).
 *
 * ONE C source, no #ifdef for the code under test, built by test/dsp/dsp_resampler.sh against the blob (a copy of dsplibs.o in which
 * `edprintf` is WEAK so that the recorder below wins), against the port (-O0 / -O2; the P03 port object supplies LowPassFIR / mean / Var in
 * the port builds) and for x86-64 (diffed against the 32-bit BLOB's output = the golden, test/golden/p05_blob32*.out).  The functions are
 * reached through __asm__ labels with the Itanium-mangled names (identical in blob and port, C1 AND C2 / D1 AND D2 / D0), the object
 * layouts are C mirror structs GENERATED from test/p05.layout (REAL pointer members: they widen on x86-64 like the port's classes).
 *
 * Objects are built by the constructors under test (never by hand) on pool memory, or on sysdep_malloc memory when the deleting
 * destructor (D0, vtable slot 1) frees them.  What is compared: every return value, every object (all fields field-wise, floats as bit
 * patterns, the heap history, the coefficient table), the malloc / free sequence (objects logged with their i386 size: h_log_next_size),
 * the constructor store order (malloc-time snapshot), the red zones, every `edprintf` / `dsplibs_debug_printf` call (format text and
 * arguments; recorders), and every VIRTUAL call of Resampler::resample (VIRTUAL CALL INTERPOSITION: the vtable words are copied into a
 * harness table, slot 3 is replaced by a recorder and the vptr is pointed at the copy, so the exact sequence of timingCorrection calls,
 * their `this` / argument / the object's state at the call is part of the output; the recorder can also change phase_ like the real
 * hooks do).  The real (un-interposed) classes are tested end to end too.
 *
 * INDEPENDENT MODELS (written from the disassembly facts, not from the port): a stream model of Resampler::resample (a sliding window
 * of the consumed samples, a queue of pending samples, explicit even / odd accumulators), the polyphase table rearrangement, the
 * timing loop (named intermediate quantities), the half-baud DFT, the gain normalisation (with its debug text), the gain schedule of
 * setBllState, the timing-offset arithmetic.
 *
 * Scenarios (each its own process): 0 Resampler (pointer form) interposed hook vs the stream model, 1 float-form tables vs the LowPassFIR
 * rearrangement + resample, 2 ResamplerTimingOffset (real hook, setTimingOffset, reset, deleting dtor), 3 ResamplerTiming (real hook vs
 * the loop model, SdHalfBaudDft, adjustHalfBaudBpfGain, reset(unsigned), invertPhase / addPhase), 4 V90Resampler (setBllState, reset, history,
 * mean / std), 5 constructors / destructors / degenerate arguments / allocation failure / odd states, 6 designed dot products (accumulation
 * order), 7 the EXTENDED return values (i386 only: 80-bit patterns), 8 special values (NaN payloads, inf, denormals).
 *
 * Excluded because the BLOB itself is undefined / crashes there: resample with a NULL history (allocation failure), a NaN or out-of-range
 * phase_ at entry, a coefficient pointer shorter than L * taps, pendCount_ > 4, L == 0 with resample, infinite wrap loops (phase +inf / L == 0).
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <signal.h>
#include <unistd.h>
#include "../harness/harness.h"
#include "../p05_layout.h"

#define L(m) __asm__(m)
typedef long double LD;
typedef uint64_t u64_;

/* float arguments by value: a plain 4-byte stack word on i386 (the bits can be passed as uint32_t WITHOUT a trip through st(0), which would quiet a
 * signalling NaN); on x86-64 floats travel in SSE registers, where a movd is exact too */
#if defined(__i386__)
typedef uint32_t farg_t;
#define FARG(bits) ((uint32_t)(bits))
typedef LD XRET;   /* extended results of the blob / port (i386): unrounded, in st(0) */
#else
typedef float farg_t;
static float farg_from_bits(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }
#define FARG(bits) farg_from_bits(bits)
typedef float XRET;
#endif

/* ---- Resampler ---- */
extern void rs_c1f(void *, unsigned, farg_t, unsigned, farg_t, unsigned) L("_ZN9ResamplerC1Ejfjfj");
extern void rs_c2f(void *, unsigned, farg_t, unsigned, farg_t, unsigned) L("_ZN9ResamplerC2Ejfjfj");
extern void rs_c1p(void *, unsigned, farg_t, unsigned, float *, unsigned) L("_ZN9ResamplerC1EjfjPfj");
extern void rs_c2p(void *, unsigned, farg_t, unsigned, float *, unsigned) L("_ZN9ResamplerC2EjfjPfj");
extern void rs_d0(void *) L("_ZN9ResamplerD0Ev");
extern void rs_d1(void *) L("_ZN9ResamplerD1Ev");
extern void rs_d2(void *) L("_ZN9ResamplerD2Ev");
extern void rs_reset(void *) L("_ZN9Resampler5resetEv");
extern void rs_rhi(void *) L("_ZN9Resampler17resetHistoryIndexEv");
extern void rs_cht(void *) L("_ZN9Resampler15copyHistoryTailEv");
extern void rs_resample(void *, const float *, unsigned, float *, unsigned *) L("_ZN9Resampler8resampleEPKfjPfRj");
extern float rs_getnp(const void *) L("_ZNK9Resampler18getNormalizedPhaseEv");
extern void rs_setnp(void *, farg_t) L("_ZN9Resampler18setNormalizedPhaseEf");
extern void rs_tc(void *, farg_t) L("_ZN9Resampler16timingCorrectionEf");
/* ---- ResamplerTimingOffset ---- */
extern void ro_c1f(void *, unsigned, farg_t, unsigned, farg_t, farg_t, unsigned) L("_ZN21ResamplerTimingOffsetC1Ejfjffj");
extern void ro_c2f(void *, unsigned, farg_t, unsigned, farg_t, farg_t, unsigned) L("_ZN21ResamplerTimingOffsetC2Ejfjffj");
extern void ro_c1p(void *, unsigned, farg_t, unsigned, float *, farg_t, unsigned) L("_ZN21ResamplerTimingOffsetC1EjfjPffj");
extern void ro_c2p(void *, unsigned, farg_t, unsigned, float *, farg_t, unsigned) L("_ZN21ResamplerTimingOffsetC2EjfjPffj");
extern void ro_d0(void *) L("_ZN21ResamplerTimingOffsetD0Ev");
extern void ro_d1(void *) L("_ZN21ResamplerTimingOffsetD1Ev");
extern void ro_d2(void *) L("_ZN21ResamplerTimingOffsetD2Ev");
extern void ro_reset(void *) L("_ZN21ResamplerTimingOffset5resetEv");
extern void ro_tc(void *, farg_t) L("_ZN21ResamplerTimingOffset16timingCorrectionEf");
extern void ro_setto(void *, farg_t) L("_ZN21ResamplerTimingOffset15setTimingOffsetEf");
extern XRET ro_getppm(const void *) L("_ZNK21ResamplerTimingOffset18getTimingOffsetPPMEv");
/* ---- ResamplerTiming ---- */
extern void rt_c1f(void *, unsigned, farg_t, unsigned, farg_t, farg_t, unsigned) L("_ZN15ResamplerTimingC1Ejfjffj");
extern void rt_c2f(void *, unsigned, farg_t, unsigned, farg_t, farg_t, unsigned) L("_ZN15ResamplerTimingC2Ejfjffj");
extern void rt_c1p(void *, unsigned, farg_t, unsigned, float *, farg_t, unsigned) L("_ZN15ResamplerTimingC1EjfjPffj");
extern void rt_c2p(void *, unsigned, farg_t, unsigned, float *, farg_t, unsigned) L("_ZN15ResamplerTimingC2EjfjPffj");
extern void rt_d0(void *) L("_ZN15ResamplerTimingD0Ev");
extern void rt_d1(void *) L("_ZN15ResamplerTimingD1Ev");
extern void rt_d2(void *) L("_ZN15ResamplerTimingD2Ev");
extern void rt_tc(void *, farg_t) L("_ZN15ResamplerTiming16timingCorrectionEf");
extern void rt_reset(void *, unsigned) L("_ZN15ResamplerTiming5resetEj");
extern void rt_sd(void *, farg_t) L("_ZN15ResamplerTiming13SdHalfBaudDftEf");
extern void rt_adj(void *, farg_t) L("_ZN15ResamplerTiming21adjustHalfBaudBpfGainEf");
extern void rt_rsd(void *) L("_ZN15ResamplerTiming18resetSdHalfBaudDftEv");
extern void rt_inv(void *) L("_ZN15ResamplerTiming11invertPhaseEv");
extern void rt_add(void *, farg_t) L("_ZN15ResamplerTiming8addPhaseEf");
/* ---- V90Resampler ---- */
extern void vr_c1f(void *, unsigned, farg_t, unsigned, farg_t, void *, farg_t, unsigned) L("_ZN12V90ResamplerC1EjfjfP13V90Parametersfj");
extern void vr_c2f(void *, unsigned, farg_t, unsigned, farg_t, void *, farg_t, unsigned) L("_ZN12V90ResamplerC2EjfjfP13V90Parametersfj");
extern void vr_c1p(void *, unsigned, farg_t, unsigned, float *, void *, farg_t, unsigned) L("_ZN12V90ResamplerC1EjfjPfP13V90Parametersfj");
extern void vr_c2p(void *, unsigned, farg_t, unsigned, float *, void *, farg_t, unsigned) L("_ZN12V90ResamplerC2EjfjPfP13V90Parametersfj");
extern void vr_d0(void *) L("_ZN12V90ResamplerD0Ev");
extern void vr_d1(void *) L("_ZN12V90ResamplerD1Ev");
extern void vr_d2(void *) L("_ZN12V90ResamplerD2Ev");
extern void vr_reset(void *) L("_ZN12V90Resampler5resetEv");
extern void vr_setbll(void *, unsigned, unsigned) L("_ZN12V90Resampler11setBllStateE11V90BllStatej");
extern void vr_resample(void *, const float *, unsigned, float *, unsigned *) L("_ZN12V90Resampler8resampleEPKfjPfRj");
extern XRET vr_mean(void *) L("_ZN12V90Resampler20getTimingHistoryMeanEv");
extern XRET vr_std(void *) L("_ZN12V90Resampler19getTimingHistoryStdEv");
/* ---- the blob / port neighbours the harness needs ---- */
typedef struct { float *coef_; unsigned n_; } LpfObj;  /* LowPassFIR<float> (P03): heap coef_, n_ */
extern void lpf_c1(void *, unsigned, farg_t, int, farg_t) L("_ZN10LowPassFIRIfEC1Ejf10WindowTypef");
extern void lpf_d1(void *) L("_ZN10LowPassFIRIfED1Ev");
extern char vtbl_rs[] L("_ZTV9Resampler");
extern char vtbl_ro[] L("_ZTV21ResamplerTimingOffset");
extern char vtbl_rt[] L("_ZTV15ResamplerTiming");
extern char vtbl_vr[] L("_ZTV12V90Resampler");
extern unsigned int dsplibs_debug_level;

/* ------------------------------------------------------------------------------------------------ recorders (edprintf / dsplibs_debug_printf) */
static unsigned g_nedp, g_ndbg;
extern unsigned int dsplibs_debug_level;
static int g_lvl_plan_on;                /* the printf recorder changes dsplibs_debug_level after print i (the blob re-reads the level between its prints) */
static unsigned g_lvl_plan[3], g_call_prints;
static char g_last_edp[200];
static int g_last_dbg_sign, g_last_dbg_ip, g_last_dbg_fp;
static char g_last_dbg_fmt[200];
static struct { char fmt[200]; int sign, ip, fp; } g_dbg_ring[8];
static int g_ed_show, g_dbg_show;
static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
void edprintf(const char *fmt, ...) {
  char buf[400];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  g_nedp++;
  g_last_edp[0] = 0;
  esc_cat(g_last_edp, sizeof g_last_edp, buf);
  h_emit(g_ed_show, "edprintf %s", g_last_edp);
}
int dsplibs_debug_printf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  g_last_dbg_sign = va_arg(ap, int);
  g_last_dbg_ip = va_arg(ap, int);
  g_last_dbg_fp = va_arg(ap, int);
  va_end(ap);
  g_last_dbg_fmt[0] = 0;
  esc_cat(g_last_dbg_fmt, sizeof g_last_dbg_fmt, fmt);
  strcpy(g_dbg_ring[g_ndbg & 7u].fmt, g_last_dbg_fmt);
  g_dbg_ring[g_ndbg & 7u].sign = g_last_dbg_sign; g_dbg_ring[g_ndbg & 7u].ip = g_last_dbg_ip; g_dbg_ring[g_ndbg & 7u].fp = g_last_dbg_fp;
  g_ndbg++;
  if (g_lvl_plan_on && g_call_prints < 3) dsplibs_debug_level = g_lvl_plan[g_call_prints];
  g_call_prints++;
  h_emit(g_dbg_show, "dbgprintf %s sign=%d ip=%d fp=%d", g_last_dbg_fmt, g_last_dbg_sign, g_last_dbg_ip, g_last_dbg_fp);
  return 0;
}

/* ------------------------------------------------------------------------------------------------ bit helpers */
static uint32_t bits_f(float f) { return h_bits_of(f); }
static float f_of(uint32_t b) { float f; memcpy(&f, &b, 4); return f; } /* only fed to arithmetic when finite; never returned by value for signalling NaNs */
static uint64_t bits_d(double d) { uint64_t b; memcpy(&b, &d, 8); return b; }
static double d_of(uint64_t b) { double d; memcpy(&d, &b, 8); return d; }
static int is_nan_w(uint32_t b) { return (b & 0x7f800000u) == 0x7f800000u && (b & 0x007fffffu) != 0; }
static int is_snan_w(uint32_t b) { return is_nan_w(b) && !(b & 0x00400000u); }
static int is_fin_w(uint32_t b) { return (b & 0x7f800000u) != 0x7f800000u; }
static float pow2f(int e) { return f_of((uint32_t)(e + 127) << 23); }  /* 2^e, e in [-126, 127] */
static uint64_t fnv_blk(const void *p, size_t n) {
  const uint8_t *q = (const uint8_t *)p;
  uint64_t h = 0xcbf29ce484222325ull;
  size_t i;
  for (i = 0; i < n; i++) { h ^= q[i]; h *= 0x100000001b3ull; }
  return h;
}

/* float words by regime (bit constructions only: no float arithmetic in the data path, i386 and x86-64 must see the same stream) */
enum { R_UNIT = 0, R_WIDE, R_INT, R_SPECIAL, R_TINY };
static uint32_t rbits(int regime) {
  uint32_t r = h_rnd(), s = (r >> 31) << 31;
  switch (regime) {
    case R_UNIT: return s | ((uint32_t)(126 - (int)h_rmod(8)) << 23) | (h_rnd() & 0x7fffffu);
    case R_WIDE: return s | ((uint32_t)(127 + h_rrange(-30, 30)) << 23) | (h_rnd() & 0x7fffffu);
    case R_INT: { int v = h_rrange(-64, 64); float f = (float)v; return bits_f(f); }
    case R_TINY: return s | ((uint32_t)(h_rmod(60)) << 23) | (h_rnd() & 0x7fffffu);  /* denormals and small normals */
    default: {
      static const uint32_t sp[] = {0x00000000u, 0x80000000u, 0x3f800000u, 0xbf800000u, 0x7f800000u, 0xff800000u, 0x7fc00000u, 0xffc00000u,
                                    0x7fa00001u, 0xffbfffffu, 0x7fffffffu, 0x00000001u, 0x007fffffu, 0x80000001u, 0x7f7fffffu, 0xff7fffffu,
                                    0x3f000000u, 0x40000000u, 0x00800000u};
      return sp[h_rmod((unsigned)(sizeof sp / sizeof sp[0]))];
    }
  }
}

/* ------------------------------------------------------------------------------------------------ coverage ids */
enum {
  CV_TRIAL = 0, CV_RS, CV_RO, CV_RT, CV_VR, CV_C2, CV_PTRFORM, CV_FLOATFORM, CV_D0, CV_D1, CV_D2, CV_HOOK_INTERPOSED, CV_HOOK_REAL,
  CV_POL_JITTER, CV_POL_JUMP, CV_POL_NEG, CV_POL_BOUND,
  CV_NEED_GT_TOTAL, CV_TAIL, CV_N0, CV_N1, CV_WRAP, CV_REMAINDER, CV_LAST_PHASE, CV_NOT_LAST_PHASE, CV_PHASE_CLAMP, CV_PHASE_MULTIWRAP, CV_PHASE_WRAP1,
  CV_PEND_SHIFT, CV_PEND_MULTI, CV_NEEDGE2, CV_STEP_LT_1, CV_STEP_GT_L,
  CV_FLOAT_TABLE, CV_TABLE_TAPS_NON4, CV_ALLOC_FAIL, CV_HIST0, CV_TAPS0,
  CV_OFF_NEG, CV_OFF_POS, CV_SETTO, CV_GETPPM, CV_RESET_VIRT,
  CV_TM_A, CV_TM_B, CV_TM_PARITY_HIGH, CV_SD0, CV_SD1, CV_SD2, CV_SD3, CV_SD_DONE, CV_SD_AFTER_DONE, CV_ADJ_NOTDONE, CV_ADJ_P0, CV_ADJ_NAN, CV_ADJ_N_GT4,
  CV_ADJ_N_2_4, CV_ADJ_N_1_2, CV_ADJ_N_LT1, CV_ADJ_N_EQ4, CV_ADJ_N_EQ2 = 62 /* ids 60 / 61 are reserved by the harness library */, CV_ADJ_N_EQ1, CV_DBG1, CV_DBG2, CV_DBG3, CV_DBG_NEG, CV_DBG_BIG, CV_RESET_UNS, CV_INVERT, CV_ADD, CV_ADD_NEG,
  CV_BLL_SAME, CV_BLL_OOR, CV_BLL_NEWFLAG, CV_HIST_REC, CV_HIST_WRAP, CV_P160_0, CV_MEAN, CV_STD, CV_STD_NEGVAR,
  CV_SNAN, CV_DESIGN, CV_NORMPH, CV_ZERO_L, CV_END_
};
#define CV_BLL_COUNT 16
#define CV_BLL0 (CV_END_)   /* the setBllState cases follow the table */

static void define_cov(void) {
  int i;
  if (CV_BLL0 + CV_BLL_COUNT >= H_COV_MAX) h_die3("too many coverage ids");
  h_cov_define(CV_TRIAL, "trials");
  h_cov_define(CV_RS, "Resampler objects"); h_cov_define(CV_RO, "ResamplerTimingOffset objects"); h_cov_define(CV_RT, "ResamplerTiming objects");
  h_cov_define(CV_VR, "V90Resampler objects"); h_cov_define(CV_C2, "C2 constructors"); h_cov_define(CV_PTRFORM, "coefficient-pointer constructors");
  h_cov_define(CV_FLOATFORM, "float (designed table) constructors");
  h_cov_define(CV_D0, "deleting destructor through vtable slot 1"); h_cov_define(CV_D1, "D1"); h_cov_define(CV_D2, "D2");
  h_cov_define(CV_HOOK_INTERPOSED, "interposed virtual call recorded"); h_cov_define(CV_HOOK_REAL, "real timingCorrection override ran in resample");
  h_cov_define(CV_POL_JITTER, "hook policy: jitter"); h_cov_define(CV_POL_JUMP, "hook policy: jumps"); h_cov_define(CV_POL_NEG, "hook policy: negative phase");
  h_cov_define(CV_POL_BOUND, "hook policy: boundary phases");
  h_cov_define(CV_NEED_GT_TOTAL, "resample: need > total exit"); h_cov_define(CV_TAIL, "resample: tail store"); h_cov_define(CV_N0, "resample: n == 0");
  h_cov_define(CV_N1, "resample: n == 1"); h_cov_define(CV_WRAP, "history wrap (w == histLen)"); h_cov_define(CV_REMAINDER, "taps % 4 != 0 (dot remainder)");
  h_cov_define(CV_LAST_PHASE, "idx + 1 >= L (look-ahead row 0)"); h_cov_define(CV_NOT_LAST_PHASE, "idx + 1 < L (next row)");
  h_cov_define(CV_PHASE_CLAMP, "phase clamped to 0 (negative)"); h_cov_define(CV_PHASE_MULTIWRAP, "phase wrapped more than once"); h_cov_define(CV_PHASE_WRAP1, "phase wrapped once");
  h_cov_define(CV_PEND_SHIFT, "pending samples shifted"); h_cov_define(CV_PEND_MULTI, "pendCount_ >= 2 consumed"); h_cov_define(CV_NEEDGE2, "need_ >= 2 consumed in one output");
  h_cov_define(CV_STEP_LT_1, "step < 1 (outputs without input)"); h_cov_define(CV_STEP_GT_L, "step > L (several samples per output)");
  h_cov_define(CV_FLOAT_TABLE, "designed table checked against the LowPassFIR rearrangement"); h_cov_define(CV_TABLE_TAPS_NON4, "float form with taps % 4 != 0");
  h_cov_define(CV_ALLOC_FAIL, "history allocation failure"); h_cov_define(CV_HIST0, "histLen == 0 (taps == 0)"); h_cov_define(CV_TAPS0, "taps == 0");
  h_cov_define(CV_OFF_NEG, "negative timing offset"); h_cov_define(CV_OFF_POS, "positive timing offset"); h_cov_define(CV_SETTO, "setTimingOffset");
  h_cov_define(CV_GETPPM, "getTimingOffsetPPM"); h_cov_define(CV_RESET_VIRT, "reset() through vtable slot 2");
  h_cov_define(CV_TM_A, "timingCorrection branch A (parity even)"); h_cov_define(CV_TM_B, "timingCorrection branch B (parity odd)"); h_cov_define(CV_TM_PARITY_HIGH, "parity_ > 2 (poked)");
  h_cov_define(CV_SD0, "SdHalfBaudDft n&3 == 0"); h_cov_define(CV_SD1, "SdHalfBaudDft n&3 == 1"); h_cov_define(CV_SD2, "SdHalfBaudDft n&3 == 2"); h_cov_define(CV_SD3, "SdHalfBaudDft n&3 == 3");
  h_cov_define(CV_SD_DONE, "SdHalfBaudDft completed 256 samples"); h_cov_define(CV_SD_AFTER_DONE, "SdHalfBaudDft after completion (ignored)");
  h_cov_define(CV_ADJ_NOTDONE, "adjustHalfBaudBpfGain not done (ignored)"); h_cov_define(CV_ADJ_P0, "adjust: P == 0"); h_cov_define(CV_ADJ_NAN, "adjust: P NaN");
  h_cov_define(CV_ADJ_N_GT4, "adjust: N > 4"); h_cov_define(CV_ADJ_N_2_4, "adjust: 2 < N <= 4"); h_cov_define(CV_ADJ_N_1_2, "adjust: 1 <= N <= 2"); h_cov_define(CV_ADJ_N_LT1, "adjust: N < 1");
  h_cov_define(CV_ADJ_N_EQ4, "adjust: N == 4"); h_cov_define(CV_ADJ_N_EQ2, "adjust: N == 2"); h_cov_define(CV_ADJ_N_EQ1, "adjust: N == 1");
  h_cov_define(CV_DBG1, "debug print 1"); h_cov_define(CV_DBG2, "debug print 2"); h_cov_define(CV_DBG3, "debug print 3"); h_cov_define(CV_DBG_NEG, "debug print of a negative value");
  h_cov_define(CV_DBG_BIG, "debug print of an out-of-int32-range value"); h_cov_define(CV_RESET_UNS, "ResamplerTiming::reset(unsigned)");
  h_cov_define(CV_INVERT, "invertPhase"); h_cov_define(CV_ADD, "addPhase a > 0"); h_cov_define(CV_ADD_NEG, "addPhase a <= 0 / NaN");
  h_cov_define(CV_BLL_SAME, "setBllState same state (early return)"); h_cov_define(CV_BLL_OOR, "setBllState state > 15"); h_cov_define(CV_BLL_NEWFLAG, "setBllState flag changed");
  h_cov_define(CV_HIST_REC, "timing history record"); h_cov_define(CV_HIST_WRAP, "timing history index wrap"); h_cov_define(CV_P160_0, "params->0x160 == 0 (no history)");
  h_cov_define(CV_MEAN, "getTimingHistoryMean"); h_cov_define(CV_STD, "getTimingHistoryStd"); h_cov_define(CV_STD_NEGVAR, "getTimingHistoryStd with a negative variance (rounding)");
  h_cov_define(CV_SNAN, "signalling-NaN words"); h_cov_define(CV_DESIGN, "designed dot products"); h_cov_define(CV_NORMPH, "get/setNormalizedPhase"); h_cov_define(CV_ZERO_L, "L == 0");
  for (i = 0; i < CV_BLL_COUNT; i++) h_cov_define(CV_BLL0 + i, "setBllState case");
}
static void cov_clear(const int *ids) { while (*ids >= 0) h_covreq[*ids++] = 0; }

/* ------------------------------------------------------------------------------------------------ objects */
typedef struct {
  int kind;            /* 0 Resampler, 1 ResamplerTimingOffset, 2 ResamplerTiming, 3 V90Resampler */
  int ptrForm;         /* 1: the coefficient-pointer constructor */
  int useC2;
  unsigned L, taps, histLen;
  uint32_t stepW, cutW, ppmW;
  float *coef;         /* pointer form: a harness array (sysdep_malloc) */
  void *params;        /* V90Resampler: the harness' V90Parameters block */
  int heap;            /* the object itself is a logged sysdep_malloc block (deleting destructor) */
  int probe;           /* malloc-time store-order probe */
  int failFirst;       /* the first sysdep_malloc of the constructor returns NULL (the blob never tests it) */
  int failLast;        /* the LAST sysdep_malloc of the constructor returns NULL (the polyphase table of the float form); the count comes from a dry-run construction */
} Cfg;

static size_t obj_size(int kind) { static const size_t t[4] = {sizeof(RsObj), sizeof(RoObj), sizeof(RtObj), sizeof(VrObj)}; return t[kind]; }
static unsigned obj_size_i386(int kind) { static const unsigned t[4] = {0x48, 0x4c, 0x94, 0xb4}; return t[kind]; }

static void zero_pads(int kind, void *o) {
  if (kind >= 2) { memset((uint8_t *)o + offsetof(RtObj, pad64_), 0, sizeof(((RtObj *)o)->pad64_)); memset((uint8_t *)o + offsetof(RtObj, pad8d_), 0, sizeof(((RtObj *)o)->pad8d_)); }
}

static void call_ctor(const Cfg *c, void *o) {
  switch (c->kind) {
    case 0:
      if (c->ptrForm) (c->useC2 ? rs_c2p : rs_c1p)(o, c->L, FARG(c->stepW), c->taps, c->coef, c->histLen);
      else (c->useC2 ? rs_c2f : rs_c1f)(o, c->L, FARG(c->stepW), c->taps, FARG(c->cutW), c->histLen);
      break;
    case 1:
      if (c->ptrForm) (c->useC2 ? ro_c2p : ro_c1p)(o, c->L, FARG(c->stepW), c->taps, c->coef, FARG(c->ppmW), c->histLen);
      else (c->useC2 ? ro_c2f : ro_c1f)(o, c->L, FARG(c->stepW), c->taps, FARG(c->cutW), FARG(c->ppmW), c->histLen);
      break;
    case 2:
      if (c->ptrForm) (c->useC2 ? rt_c2p : rt_c1p)(o, c->L, FARG(c->stepW), c->taps, c->coef, FARG(c->ppmW), c->histLen);
      else (c->useC2 ? rt_c2f : rt_c1f)(o, c->L, FARG(c->stepW), c->taps, FARG(c->cutW), FARG(c->ppmW), c->histLen);
      break;
    default:
      if (c->ptrForm) (c->useC2 ? vr_c2p : vr_c1p)(o, c->L, FARG(c->stepW), c->taps, c->coef, c->params, FARG(c->ppmW), c->histLen);
      else (c->useC2 ? vr_c2f : vr_c1f)(o, c->L, FARG(c->stepW), c->taps, FARG(c->cutW), c->params, FARG(c->ppmW), c->histLen);
      break;
  }
}

static const HFld *kind_flds(int kind, int *nf) {
  switch (kind) {
    case 0: *nf = NF_RsObj; return f_RsObj;
    case 1: *nf = NF_RoObj; return f_RoObj;
    case 2: *nf = NF_RtObj; return f_RtObj;
    default: *nf = NF_VrObj; return f_VrObj;
  }
}

static void *make_obj(const Cfg *c) {
  size_t sz = obj_size(c->kind);
  uint8_t *o;
  int nf;
  const HFld *fl = kind_flds(c->kind, &nf);
  if (c->heap) { h_log_next_size = obj_size_i386(c->kind); o = (uint8_t *)sysdep_malloc((unsigned)sz); h_log_next_size = 0; }
  else o = (uint8_t *)h_obj_new(sz);
  zero_pads(c->kind, o);
  if (c->kind >= 2) ((RtObj *)o)->dftDone_ = 0xA5;   /* the constructor stores 0: the random fill byte must not be 0 (store-order probe) */
  h_wild = 0;
  if (c->failFirst) { h_fail_next_malloc = 1; h_cov_hit(CV_ALLOC_FAIL); }
  if (c->failLast) {   /* dry run on a scratch object: how many allocations does this constructor make? */
    Cfg d = *c;
    void *scratch;
    unsigned long m0;
    d.heap = 0; d.probe = 0; d.failFirst = 0; d.failLast = 0;
    scratch = h_obj_new(sz);
    zero_pads(c->kind, scratch);
    m0 = h_nmalloc;
    call_ctor(&d, scratch);
    h_fail_malloc_n = (unsigned)(h_nmalloc - m0);
    { void **vt = *(void ***)scratch; (void)vt; }
    switch (c->kind) { case 0: rs_d1(scratch); break; case 1: ro_d1(scratch); break; case 2: rt_d1(scratch); break; default: vr_d1(scratch); }
    h_cov_hit(CV_ALLOC_FAIL);
  }
  if (c->probe) { H_CTOR_PROBE(o, fl, nf, sizeof(VrObj), call_ctor(c, o)); }
  else call_ctor(c, o);
  h_fail_next_malloc = 0; h_fail_malloc_n = 0;
  if (((RsObj *)o)->histLen_ == 0) h_cov_hit(CV_HIST0);
  if (((RsObj *)o)->taps_ == 0) h_cov_hit(CV_TAPS0);
  if (c->L == 0) h_cov_hit(CV_ZERO_L);
  h_cov_hit(CV_RS + c->kind);
  if (c->useC2) h_cov_hit(CV_C2);
  h_cov_hit(c->ptrForm ? CV_PTRFORM : CV_FLOATFORM);
  return o;
}

static void dump_obj(int show, const char *tag, int kind, const void *o) {
  int nf;
  const HFld *fl = kind_flds(kind, &nf);
  const RsObj *r = (const RsObj *)o;
  h_dump_obj(show, tag, o, fl, nf);
  /* the coefficient table (owned or borrowed) */
  {
    unsigned long n = (unsigned long)r->taps_ * (unsigned long)r->L_;
    HAl *a = r->coef_ ? h_find_al(r->coef_, 1) : NULL;
    if (a && a->p == (const uint8_t *)r->coef_ && (unsigned long)a->size >= n * 4ul) h_emit(show, "%s.coef n=%lu #%016llx", tag, n, (unsigned long long)fnv_blk(r->coef_, n * 4ul));
    else if (r->coef_) h_emit(show, "%s.coef ?", tag);
  }
  h_check_not_wild(tag);
}

/* destroy: dk 0 D1, 1 D2, 2 the deleting destructor through the vtable (heap objects), 3 D1 through the vtable (slot 0) */
static void destroy_obj(int kind, void *o, int dk) {
  void **vt = *(void ***)o;
  switch (dk) {
    case 0: switch (kind) { case 0: rs_d1(o); break; case 1: ro_d1(o); break; case 2: rt_d1(o); break; default: vr_d1(o); } h_cov_hit(CV_D1); break;
    case 1: switch (kind) { case 0: rs_d2(o); break; case 1: ro_d2(o); break; case 2: rt_d2(o); break; default: vr_d2(o); } h_cov_hit(CV_D2); break;
    case 2: ((void (*)(void *))vt[1])(o); h_cov_hit(CV_D0); break;
    default: ((void (*)(void *))vt[0])(o); h_cov_hit(CV_D1); break;
  }
}
static void direct_d0(int kind, void *o) { switch (kind) { case 0: rs_d0(o); break; case 1: ro_d0(o); break; case 2: rt_d0(o); break; default: vr_d0(o); } h_cov_hit(CV_D0); }

static void register_vtables(void) {
  h_register_vtable(vtbl_rs + 2 * sizeof(void *), "RS"); h_register_vtable(vtbl_ro + 2 * sizeof(void *), "RO");
  h_register_vtable(vtbl_rt + 2 * sizeof(void *), "RT"); h_register_vtable(vtbl_vr + 2 * sizeof(void *), "VR");
}

/* ------------------------------------------------------------------------------------------------ virtual call interposition */
static void *g_fake[4][8];       /* one copy of each class' vtable (offset-to-top, typeinfo, slots), slot 3 replaced by the recorder */
static int g_fake_made;
static int g_pol;                /* the recorder's phase policy */
static unsigned g_hook_no;
static void *g_cur_obj;
static unsigned g_opno; static int g_oprr; static int g_opw;
static const RsObj *g_watch;
static int g_watch_kind;   /* the object of the call in progress (for the runaway report) */
static unsigned g_hook_calls;

static double policy_phase(int pol, unsigned n, double ph, unsigned Lval) {
  uint32_t h = n * 2654435761u;
  h ^= h >> 15; h *= 2246822519u; h ^= h >> 13;
  switch (pol) {
    case 1: return ph + (double)((int)(h % 129u) - 64) / 64.0;
    case 2: if (h % 5u == 0) return ph + (double)Lval * (double)(h % 4u) + 0.25; return ph;
    case 3: if (h % 4u == 0) return -(double)(h % 9u) - 0.5; return ph;
    case 4:
      switch (h % 6u) {
        case 0: return (double)Lval;
        case 1: return (double)Lval - 1.0 / 1048576.0;
        case 2: return 0.0;
        case 3: return -0.0;
        case 4: return 2.0 * (double)Lval;
        default: return ph;
      }
    default: return ph;
  }
}

static void rec_slot3(void *self, farg_t xw) {
  uint32_t xb;
  RsObj *r = (RsObj *)self;
#if defined(__i386__)
  xb = xw;
#else
  memcpy(&xb, &xw, 4);
#endif
  g_hook_no++;
  g_hook_calls++;
  h_emit(0, "hook %u this=%s x=%08x phase=%016llx need=%u L=%u", g_hook_no, self == g_cur_obj ? "OBJ" : "OTHER", xb, (unsigned long long)bits_d(r->phase_), r->need_, r->L_);
  if (self != g_cur_obj) h_die3("virtual call: this != the object under test");
  h_cov_hit(CV_HOOK_INTERPOSED);
  if (g_pol) r->phase_ = policy_phase(g_pol, g_hook_no, r->phase_, r->L_);
}

static void make_fakes(void) {
  const int nslots[4] = {4, 4, 5, 5};
  char *vts[4];
  int k, i;
  if (g_fake_made) return;
  vts[0] = vtbl_rs; vts[1] = vtbl_ro; vts[2] = vtbl_rt; vts[3] = vtbl_vr;
  for (k = 0; k < 4; k++) {
    void **vt = (void **)(vts[k] + 2 * sizeof(void *));
    g_fake[k][0] = ((void **)vts[k])[0];
    g_fake[k][1] = ((void **)vts[k])[1];
    for (i = 0; i < nslots[k]; i++) g_fake[k][2 + i] = vt[i];
    g_fake[k][2 + 3] = (void *)rec_slot3;
  }
  h_register_vtable(&g_fake[0][2], "F0"); h_register_vtable(&g_fake[1][2], "F1"); h_register_vtable(&g_fake[2][2], "F2"); h_register_vtable(&g_fake[3][2], "F3");
  g_fake_made = 1;
}
static void interpose(int kind, void *o, int pol) {
  make_fakes();
  *(void ***)o = &g_fake[kind][2];
  g_cur_obj = o;
  g_pol = pol;
  if (pol == 1) h_cov_hit(CV_POL_JITTER); else if (pol == 2) h_cov_hit(CV_POL_JUMP); else if (pol == 3) h_cov_hit(CV_POL_NEG); else if (pol == 4) h_cov_hit(CV_POL_BOUND);
}
static void restore_vptr(int kind, void *o) {
  char *vts[4];
  vts[0] = vtbl_rs; vts[1] = vtbl_ro; vts[2] = vtbl_rt; vts[3] = vtbl_vr;
  *(void **)o = vts[kind] + 2 * sizeof(void *);
  g_pol = 0;
}

/* ------------------------------------------------------------------------------------------------ the models */
#define MAXTAPS 130
typedef struct {            /* the timing loop: named quantities (written from the disassembly facts, not from the port) */
  float gA, gB, errOut, lastCorr, energy, energyPrev, bpf, bpfPrev, errAcc, gain, off;
  unsigned parity;
} TmModel;
typedef struct {            /* the resampler: a stream model with a sliding window of the consumed samples */
  unsigned L, taps;
  const float *coef;
  float step;
  double phase;
  unsigned need, pc;
  uint32_t q[8];
  uint32_t win[MAXTAPS + 4];
  int hookKind;             /* 0 empty hook, 1 interposed recorder with a policy, 2 ResamplerTimingOffset, 3 ResamplerTiming / V90Resampler loop */
  int pol;
  unsigned hookNo;
  TmModel tm;
  unsigned nhook;           /* hooks executed since the last check */
} RsModel;

/* the model's own x87 helpers: explicit extended arithmetic */
static float F_(uint32_t b) { return f_of(b); }
static LD dotM(const uint32_t *xw, const float *c, unsigned n) {
  unsigned full = n & ~3u, i;
  LD ev = 0.0L, od = 0.0L;
  for (i = 0; i < full; i++) {
    LD t = (LD)F_(xw[i]) * (LD)c[i];
    if (i & 1u) od += t; else ev += t;
  }
  for (; i < n; i++) ev += (LD)F_(xw[i]) * (LD)c[i];
  return od + ev;
}

static void tm_correct(RsModel *m, float x) {
  TmModel *t = &m->tm;
  LD e = (LD)t->gain * (LD)x - (LD)0.9604f * (LD)t->bpfPrev;
  LD kEnergy = (LD)t->energy;
  float oldBpf = t->bpf;
  LD e2;
  LD r;
  t->bpfPrev = oldBpf;
  t->bpf = (float)e;
  e2 = e * e;
  r = ((e2 * (LD)0.00025f) - (kEnergy * (LD)0.0005f)) + ((LD)0.00025f * (LD)t->energyPrev);
  t->energy = (float)e2;
  t->energyPrev = (float)kEnergy;
  t->errOut = (float)r;
  if ((t->parity & 1u) == 0) {
    LD off = (LD)t->off;
    t->lastCorr = (float)off;
    t->errAcc = (float)(-r);
    t->parity = 1;
    m->phase = (double)((LD)m->phase + off);
    h_cov_hit(CV_TM_A);
  } else {
    LD q = r + (LD)t->errAcc;
    LD half = q * (LD)0.5f;
    LD integ = ((LD)t->gB * half) + (LD)t->off;
    LD prop = ((LD)t->gA * half) + integ;
    t->parity = t->parity + 1;
    t->errAcc = (float)q;
    t->off = (float)integ;
    t->lastCorr = (float)prop;
    m->phase = (double)((LD)m->phase + prop);
    h_cov_hit(CV_TM_B);
    if (t->parity > 2) h_cov_hit(CV_TM_PARITY_HIGH);
  }
}
static void rm_hook(RsModel *m, float out) {
  m->nhook++;
  switch (m->hookKind) {
    case 1: m->hookNo++; m->phase = policy_phase(m->pol, m->hookNo, m->phase, m->L); break;
    case 2: m->phase = (double)((LD)m->phase + (LD)m->tm.off); h_cov_hit(CV_HOOK_REAL); break;
    case 3: tm_correct(m, out); h_cov_hit(CV_HOOK_REAL); break;
    default: break;
  }
}

/* the model of Resampler::resample.  in[] is the stream (word view); the function may read in[n] (the look-ahead). */
static unsigned rm_resample(RsModel *m, const uint32_t *in, unsigned n, uint32_t *out) {
  unsigned nout = 0, total = n + m->pc, pos = 0;
  while (total > 1) {
    if (nout > 2000000u) h_die3("model: no end of the output loop (phase %g need %u L %u step %g)", m->phase, m->need, m->L, (double)m->step);
    unsigned need0 = m->need, k = need0 <= total ? need0 : total, i;
    for (i = 0; i < k; i++) {
      uint32_t s;
      if (m->pc > 0) {
        unsigned j;
        s = m->q[0];
        for (j = 1; j < m->pc; j++) m->q[j - 1] = m->q[j];
        m->pc--;
        h_cov_hit(CV_PEND_SHIFT);
      } else {
        s = in[pos++];
      }
      if (m->taps > 0) { memmove(m->win, m->win + 1, (m->taps - 1) * 4u); m->win[m->taps - 1] = s; }
    }
    if (need0 > total) {
      m->need = need0 - total;
      m->q[m->pc++] = in[pos];
      h_cov_hit(CV_NEED_GT_TOTAL);
      return nout;
    }
    if (need0 >= 2) h_cov_hit(CV_NEEDGE2);
    total -= need0;
    {
      LD ph = (LD)m->phase;
      int idx = (int)ph;
      float y0, y1, fracf, of;
      LD d, o;
      y0 = (float)dotM(m->win, m->coef + (unsigned)idx * m->taps, m->taps);
      if ((unsigned)(idx + 1) < m->L) {
        y1 = (float)dotM(m->win, m->coef + (unsigned)(idx + 1) * m->taps, m->taps);
        h_cov_hit(CV_NOT_LAST_PHASE);
      } else {
        uint32_t tmp[MAXTAPS + 4];
        if (m->taps > 0) { memcpy(tmp, m->win + 1, (m->taps - 1) * 4u); tmp[m->taps - 1] = in[pos]; }
        y1 = (float)dotM(tmp, m->coef, m->taps);
        h_cov_hit(CV_LAST_PHASE);
      }
      fracf = (float)(ph - (LD)idx);
      d = (LD)y1 - (LD)y0;
      o = (LD)y0 + d * (LD)fracf;
      of = (float)o;
      if (getenv("P05_DEBUG")) fprintf(stderr, "  model out %u: idx %d y0 %08x y1 %08x frac %08x of %08x win0 %08x winlast %08x taps %u need0 %u pc %u phase %.6f\n", nout, idx, bits_f(y0), bits_f(y1), bits_f(fracf), bits_f(of), m->taps ? m->win[0] : 0, m->taps ? m->win[m->taps - 1] : 0, m->taps, need0, m->pc, m->phase);
      out[nout++] = bits_f(of);
      m->phase = (double)(ph + (LD)m->step);
      rm_hook(m, of);
      m->need = 0;
      {
        LD Lf = (LD)m->L, p = (LD)m->phase;
        if (!(p >= Lf)) {
          if (0.0L > p) { m->phase = 0.0; h_cov_hit(CV_PHASE_CLAMP); }
        } else {
          unsigned cnt = 0;
          do { cnt++; p -= Lf; if (cnt > 1000000u) h_die3("model: the phase wrap loop does not end (phase %Lg L %u)", (LD)m->phase, m->L); } while (p >= Lf);
          m->need = cnt;
          m->phase = (double)p;
          if (0.0L > p) m->phase = 0.0;
          h_cov_hit(cnt > 1 ? CV_PHASE_MULTIWRAP : CV_PHASE_WRAP1);
        }
      }
    }
  }
  m->q[m->pc++] = in[pos];
  h_cov_hit(CV_TAIL);
  return nout;
}

static void rm_init(RsModel *m, int kind, const void *obj, int hookKind, int pol) {
  const RsObj *r = (const RsObj *)obj;
  memset(m, 0, sizeof *m);
  m->L = r->L_; m->taps = r->taps_; m->coef = r->coef_; m->step = r->step_; m->phase = r->phase_; m->need = r->need_;
  m->hookKind = hookKind; m->pol = pol;
  if (kind >= 1) m->tm.off = ((const RoObj *)obj)->off_;
  if (kind >= 2) {
    const RtObj *t = (const RtObj *)obj;
    m->tm.gA = t->gainA_; m->tm.gB = t->gainB_; m->tm.errOut = t->errOut_; m->tm.lastCorr = t->lastCorr_; m->tm.energy = t->energy_;
    m->tm.energyPrev = t->energyPrev_; m->tm.bpf = t->bpf_; m->tm.bpfPrev = t->bpfPrev_; m->tm.errAcc = t->errAcc_; m->tm.gain = t->bpfGain_; m->tm.parity = t->parity_;
  }
  /* the window: the newest taps samples of the history (right after the constructor / reset: zeros) */
  if (r->hist_ && r->wr_ >= r->taps_) memcpy(m->win, (const float *)r->hist_ + (r->wr_ - r->taps_), r->taps_ * 4u);
  m->pc = r->pendCount_;
  memcpy(m->q, r->pend_, sizeof r->pend_);
}

/* word equality; NaN results only need to be NaN */
static int word_eq(uint32_t a, uint32_t b) { return a == b || (is_nan_w(a) && is_nan_w(b)); }
static void rm_check(const RsModel *m, int kind, const void *obj, const char *when) {
  const RsObj *r = (const RsObj *)obj;
  unsigned i;
  uint64_t pb = bits_d(r->phase_), mb = bits_d(m->phase);
  int pnan = ((mb >> 52) & 0x7ff) == 0x7ff && (mb & 0xfffffffffffffull);
  if (!(pb == mb || (pnan && ((pb >> 52) & 0x7ff) == 0x7ff))) h_die3("%s: phase_ %016llx, model %016llx", when, (unsigned long long)pb, (unsigned long long)mb);
  if (r->need_ != m->need) h_die3("%s: need_ %u, model %u", when, r->need_, m->need);
  if (r->pendCount_ != m->pc) h_die3("%s: pendCount_ %u, model %u", when, r->pendCount_, m->pc);
  for (i = 0; i < m->pc && i < 5; i++) if (bits_f(r->pend_[i]) != m->q[i]) h_die3("%s: pend_[%u] %08x, model %08x", when, i, bits_f(r->pend_[i]), m->q[i]);
  if (r->hist_ && r->wr_ >= r->taps_) {
    for (i = 0; i < r->taps_ && i < MAXTAPS; i++) { const float *hh = (const float *)r->hist_; if (bits_f(hh[r->wr_ - r->taps_ + i]) != m->win[i]) h_die3("%s: history window[%u] %08x, model %08x", when, i, bits_f(hh[r->wr_ - r->taps_ + i]), m->win[i]); }
  }
  if (kind >= 1 && !word_eq(bits_f(((const RoObj *)obj)->off_), bits_f(m->tm.off))) h_die3("%s: off_ %08x, model %08x", when, bits_f(((const RoObj *)obj)->off_), bits_f(m->tm.off));
  if (kind >= 2 && m->hookKind == 3) {
    const RtObj *t = (const RtObj *)obj;
#define TMC(f, mf) if (!word_eq(bits_f(t->f), bits_f(m->tm.mf))) h_die3("%s: %s %08x, model %08x", when, #f, bits_f(t->f), bits_f(m->tm.mf))
    TMC(gainA_, gA); TMC(gainB_, gB); TMC(errOut_, errOut); TMC(lastCorr_, lastCorr); TMC(energy_, energy); TMC(energyPrev_, energyPrev);
    TMC(bpf_, bpf); TMC(bpfPrev_, bpfPrev); TMC(errAcc_, errAcc); TMC(bpfGain_, gain);
#undef TMC
    if (t->parity_ != m->tm.parity) h_die3("%s: parity_ %u, model %u", when, t->parity_, m->tm.parity);
  }
}

/* ------------------------------------------------------------------------------------------------ data */
static float *new_floats(unsigned n) { return (float *)sysdep_malloc(n * 4u); }
static void put_word(float *p, unsigned i, uint32_t w) { memcpy(&p[i], &w, 4); }
static void fill_regime(float *p, unsigned n, int regime) { unsigned i; for (i = 0; i < n; i++) put_word(p, i, rbits(regime)); }
/* coefficient tables */
static void fill_coef(float *c, unsigned n, int mode) {
  unsigned i;
  for (i = 0; i < n; i++) {
    switch (mode) {
      case 0: put_word(c, i, rbits(R_UNIT)); break;
      case 1: put_word(c, i, rbits(R_INT)); break;
      case 2: put_word(c, i, (i & 3u) == 0 ? 0x3f800000u : 0u); break;  /* selects every 4th sample */
      default: put_word(c, i, rbits(R_WIDE)); break;
    }
  }
}
/* a V90Parameters stand-in: 0x170 bytes in the i386 layout; the gain words (0x88 .. 0xf7) are float bit patterns, 0x160 / 0x164 / 0x168 control the history.
 * The first member of V90Parameters is a pointer (4 bytes i386, 8 bytes x86-64), so on a 64-bit build every word after it is PAD64 bytes further: PO() maps an
 * i386 offset to the offset in the stand-in (the port reads the real members).  The random stream is the same at both widths. */
#define PAD64 (sizeof(void *) - 4)
#define PO(off) ((off) + PAD64)
static void *new_params(unsigned histCount, uint32_t flag160, uint32_t thr168, int special) {
  uint8_t *p;
  h_log_next_size = 0x170;   /* logged as the i386 stand-in size at both widths */
  p = (uint8_t *)sysdep_malloc(0x170 + PAD64);
  unsigned off;
  { unsigned q; for (q = 0; q < 0x170; q++) p[q < 4 ? q : q + PAD64] = (uint8_t)(h_rnd() >> 11) | 0x01; for (q = 4; q < 4 + PAD64; q++) p[q] = 1; }   /* garbage everywhere (nonzero bytes) except the words below */
  if (histCount == 0) flag160 = 0;
  if (flag160 == 2) flag160 = h_rpct(50) ? 0x10000u : (h_rpct(50) ? 0x1000000u : 0x100u);   /* nonzero flags with the low half / bytes clear */   /* the blob writes tHist_[idx] with idx growing without bound when the history is empty: excluded */
  for (off = 0x88; off < 0xf8; off += 4) {   /* gains: small floats (a large gain would make the timing loop runaway), or specials */
    uint32_t w = special ? rbits(R_SPECIAL) : (((h_rnd() & 1u) << 31) | ((uint32_t)(127 - 3 - (int)h_rmod(10)) << 23) | (h_rnd() & 0x7fffffu));
    memcpy(p + PO(off), &w, 4);
  }
  memcpy(p + PO(0x160), &flag160, 4);
  memcpy(p + PO(0x164), &histCount, 4);
  memcpy(p + PO(0x168), &thr168, 4);
  return p;
}
static const uint32_t *as_words(const float *p) { return (const uint32_t *)p; }

static LD sqrtLD(LD x) { __asm__("fsqrt" : "+t"(x)); return x; }
static float fv(uint32_t w) { return f_of(w); }                   /* finite words only */
static int fistp32(LD v) { if (!(v > -2147483649.0L && v < 2147483648.0L)) return (int)0x80000000u; return (int)v; }
static const float K_MICRO = 1e-6f;

static void expect_word(const char *what, uint32_t got, uint32_t want) { if (!word_eq(got, want)) h_die3("%s: %08x, expected %08x", what, got, want); }
static void expect_u(const char *what, unsigned got, unsigned want) { if (got != want) h_die3("%s: %u, expected %u", what, got, want); }
static void expect_d(const char *what, double got, double want) { uint64_t g = bits_d(got), w = bits_d(want); int wn = ((w >> 52) & 0x7ff) == 0x7ff && (w & 0xfffffffffffffull); if (!(g == w || (wn && ((g >> 52) & 0x7ff) == 0x7ff))) h_die3("%s: %016llx, expected %016llx", what, (unsigned long long)g, (unsigned long long)w); }

static uint32_t pword(const void *params, unsigned off) { uint32_t w; memcpy(&w, (const uint8_t *)params + PO(off), 4); return w; }   /* off: the i386 offset */

/* V90Resampler::resample bookkeeping (independent expectation from the pre-call state) */
typedef struct { unsigned stateFlag, stateCount, tCount, tIdx, tLen, flag160, thr168; uint32_t th[64]; } V90Pre;
static void v90_pre(const void *o, V90Pre *p) {
  const VrObj *v = (const VrObj *)o;
  p->stateFlag = v->stateFlag_; p->stateCount = v->stateCount_; p->tCount = v->tCount_; p->tIdx = v->tHistIdx_; p->tLen = v->tHistLen_;
  p->flag160 = pword(v->params_, 0x160); p->thr168 = pword(v->params_, 0x168);
  if (v->tHist_ && p->tLen <= 64) memcpy(p->th, v->tHist_, p->tLen * 4u);
}
static void v90_post(const void *o, const V90Pre *p, unsigned n) {
  const VrObj *v = (const VrObj *)o;
  unsigned i;
  unsigned cnt = p->tCount, idx = p->tIdx;
  uint32_t exp[64];
  if (p->tLen <= 64) memcpy(exp, p->th, p->tLen * 4u);
  expect_u("V90 resample: stateCount_", v->stateCount_, p->stateFlag ? p->stateCount + n : p->stateCount);
  if (p->flag160 == 0) {
    h_cov_hit(CV_P160_0);
    expect_u("V90 resample: tCount_ (no history)", v->tCount_, p->tCount);
  } else {
    cnt += n;
    if (cnt >= p->thr168) {
      LD ppm = ((LD)1000000.0f * (LD)v->off_) / (LD)v->step_;
      if (p->tLen <= 64) { float f = (float)ppm; memcpy(&exp[idx], &f, 4); }
      idx = idx + 1 == p->tLen ? 0 : idx + 1;
      if (idx == 0) h_cov_hit(CV_HIST_WRAP);
      cnt = 0;
      h_cov_hit(CV_HIST_REC);
    }
    expect_u("V90 resample: tCount_", v->tCount_, cnt);
    expect_u("V90 resample: tHistIdx_", v->tHistIdx_, idx);
    if (p->tLen <= 64) for (i = 0; i < p->tLen; i++) { uint32_t w; memcpy(&w, (const float *)v->tHist_ + i, 4); if (!word_eq(w, exp[i])) h_die3("V90 resample: tHist_[%u] %08x, expected %08x", i, w, exp[i]); }
  }
}

/* ------------------------------------------------------------------------------------------------ resample sessions */
static unsigned pick_L(void) { static const unsigned t[] = {1, 1, 2, 2, 3, 3, 4, 5, 6, 8, 12, 16}; return t[h_rmod((unsigned)(sizeof t / sizeof t[0]))]; }
/* step = L * k / 64 (exact), k in [kmin, kmax] */
static uint32_t step_word(unsigned Lv, unsigned kmin, unsigned kmax) {
  unsigned k = kmin + h_rmod(kmax - kmin + 1);
  float f = (float)(k * Lv) * 0.015625f;
  return bits_f(f);
}
static uint32_t step_irregular(unsigned Lv, unsigned kmin) { /* a random mantissa: 1.m * 2^e with 2^e in [L * kmin / 64 .. 2L] (approximately) */
  int e = 0, lg = 0;
  uint32_t m = h_rnd() & 0x7fffffu;
  unsigned t = Lv;
  while (t > 1) { t >>= 1; lg++; }
  (void)kmin;
  e = lg - 3 + (int)h_rmod(5);   /* about L/8 .. 2L */
  return ((uint32_t)(127 + e) << 23) | m;
}

typedef struct { int interposed, pol; unsigned ncalls, maxblock; int regime; int poke; int dumpEvery; int objCheckEvery; } Sess;

static void poke_state(int kind, void *o, int regime) {
  RsObj *r = (RsObj *)o;
  unsigned i;
  if (!r->hist_ || r->taps_ + 1 > r->histLen_ || r->L_ == 0) return;
  for (i = 0; i < r->histLen_; i++) put_word(r->hist_, i, rbits(regime));
  r->wr_ = r->taps_ + h_rmod(r->histLen_ - r->taps_);
  r->pendCount_ = h_rmod(5);
  for (i = 0; i < 5; i++) put_word(r->pend_, i, rbits(regime));
  r->need_ = h_rmod(5);
  r->phase_ = (double)h_rmod(r->L_ * 64u) / 64.0;
  if (kind >= 2) {
    RtObj *t = (RtObj *)o;
    {
      uint32_t w;
      w = ((uint32_t)(120 - (int)h_rmod(6)) << 23) | (h_rnd() & 0x7fffffu) | ((h_rnd() & 1u) << 31); memcpy(&t->gainA_, &w, 4);
      w = ((uint32_t)(118 - (int)h_rmod(8)) << 23) | (h_rnd() & 0x7fffffu) | ((h_rnd() & 1u) << 31); memcpy(&t->gainB_, &w, 4);
      w = rbits(R_UNIT); memcpy(&t->energy_, &w, 4); w = rbits(R_UNIT); memcpy(&t->energyPrev_, &w, 4); w = rbits(R_UNIT); memcpy(&t->bpf_, &w, 4);
      w = rbits(R_UNIT); memcpy(&t->bpfPrev_, &w, 4); w = rbits(R_UNIT); memcpy(&t->errAcc_, &w, 4);
      t->parity_ = h_rmod(4);
    }
  }
}

/* one resample session on an object built by Cfg: ncalls calls with random block sizes, the model in lock step; returns the object */
static void *session(const Cfg *c, const Sess *s, unsigned long *ctr) {
  void *o = make_obj(c);
  RsModel m;
  RsObj *r = (RsObj *)o;
  unsigned total = s->ncalls * s->maxblock + 16, pos = 0, call;
  float *st = new_floats(total + 8);
  int hookKind = s->interposed ? 1 : (c->kind == 1 ? 2 : (c->kind >= 2 ? 3 : 0));
  char tag[64];
  dump_obj(h_sample(ctr, 6, 1999), "init", c->kind, o);
  fill_regime(st, total + 8, s->regime);
  g_hook_no = 0;
  if (s->interposed) interpose(c->kind, o, s->pol);
  rm_init(&m, c->kind, o, hookKind, s->pol);
  for (call = 0; call < s->ncalls; call++) {
    unsigned n, nout = 0xdeadbeefu, nm, cap, i, h0;
    uint32_t *mout;
    float *out;
    uint32_t pre_pc;
    unsigned wr0;
    int rr = (int)h_rmod(100);
    n = rr < 6 ? 0u : rr < 14 ? 1u : rr < 22 ? 2u : 1u + h_rmod(s->maxblock);
    if (n == 0) h_cov_hit(CV_N0); else if (n == 1) h_cov_hit(CV_N1);
    if (s->poke && h_rpct(15)) {
      poke_state(c->kind, o, s->regime);
      rm_init(&m, c->kind, o, hookKind, s->pol);
      m.hookNo = g_hook_no;
    }
    if (pos + n + 4 > total) pos = 0;
    if (r->pendCount_ > 3) { r->pendCount_ = 3; rm_init(&m, c->kind, o, hookKind, s->pol); m.hookNo = g_hook_no; } /* the call may push one pending sample: pend_[] has 5 cells */
    cap = (n + 3) * (unsigned)((double)r->L_ / (double)r->step_ * 2.0 + 4.0) + 16;
    out = new_floats(cap);
    mout = (uint32_t *)malloc(cap * 4u);
    memset(mout, 0, cap * 4u);
    h0 = g_hook_calls;
    pre_pc = r->pendCount_;
    wr0 = r->wr_;
    if (pre_pc >= 2 && n >= 2 && r->need_ >= 2) h_cov_hit(CV_PEND_MULTI);
    m.hookNo = g_hook_no;
    if (getenv("P05_DEBUG")) fprintf(stderr, "call %u kind %d L %u taps %u histLen %u wr %u n %u pc %u need %u phase %.6f step %.6f pol %d interposed %d\n", call, c->kind, r->L_, r->taps_, r->histLen_, r->wr_, n, r->pendCount_, r->need_, r->phase_, (double)r->step_, s->pol, s->interposed);
    g_watch = r;
    { V90Pre vp; if (c->kind == 3) v90_pre(o, &vp);
      if (c->kind == 3) vr_resample(o, st + pos, n, out, &nout); else rs_resample(o, st + pos, n, out, &nout);
      if (c->kind == 3) v90_post(o, &vp, n); }
    g_watch = NULL;
    nm = rm_resample(&m, as_words(st + pos), n, mout);
    if (nout != nm) h_die3("call %u: nOut %u, model %u (n %u)", call, nout, nm, n);
    for (i = 0; i < nout; i++) if (!word_eq(bits_f(out[i]), mout[i])) h_die3("call %u: out[%u] %08x, model %08x", call, i, bits_f(out[i]), mout[i]);
    if (r->wr_ < wr0) h_cov_hit(CV_WRAP);
    if (s->interposed && g_hook_calls - h0 != nout) h_die3("call %u: %u virtual calls for %u outputs", call, g_hook_calls - h0, nout);
    sprintf(tag, "call %u", call);
    rm_check(&m, c->kind, o, tag);
    h_emit(0, "resample n=%u nOut=%u pos=%u #%016llx", n, nout, pos, (unsigned long long)fnv_blk(out, nout * 4ul));
    if (r->taps_ > 0 && r->wr_ + 0 > r->histLen_) h_die3("wr_ beyond the history");
    dump_obj(s->dumpEvery <= 1 || call % (unsigned)s->dumpEvery == 0 ? h_sample(ctr, 6, 1999) : 0, "obj", c->kind, o);
    sysdep_free(out);
    free(mout);
    pos += n;
    if (h_rpct(10) && pos > 3) pos -= 1 + h_rmod(3);  /* overlapping feeds: some samples are delivered twice */
  }
  if (s->interposed) restore_vptr(c->kind, o);
  sysdep_free(st);
  return o;
}

/* a random Cfg for a pointer-form object with a harness-owned table */
static void cfg_ptr(Cfg *c, int kind, unsigned Lv, unsigned taps) {
  memset(c, 0, sizeof *c);
  c->kind = kind; c->ptrForm = 1; c->useC2 = h_rpct(40);
  c->L = Lv; c->taps = taps;
  c->histLen = h_rpct(40) ? 0u : (h_rpct(50) ? taps + 1 + h_rmod(2 * taps + 3) : taps * 10 + 1 + h_rmod(40));
  c->coef = new_floats(Lv * taps + 1);
  fill_coef(c->coef, Lv * taps + 1, (int)h_rmod(3));
}

static unsigned pick_taps(void) { static const unsigned t[] = {1, 2, 3, 4, 4, 5, 6, 7, 8, 8, 9, 10, 12, 13, 16, 17, 20, 24, 31, 32, 33, 40, 64, 65, 100, 128}; return t[h_rmod((unsigned)(sizeof t / sizeof t[0]))]; }
static uint32_t ppm_word(void) { /* a float in about +-2^9 with a random mantissa */
  return ((h_rnd() & 1u) << 31) | ((uint32_t)(127 + 2 + (int)h_rmod(8)) << 23) | (h_rnd() & 0x7fffffu);
}
static void free_cfg(Cfg *c) {
  if (c->coef) sysdep_free(c->coef);
  if (c->params) sysdep_free(c->params);
  c->coef = NULL; c->params = NULL;
}
/* destroy an object: heap objects through the deleting destructor (vtable slot 1 or the direct symbol), pool objects through D1 / D2 / D1 via the vtable */
static void finish_obj(Cfg *c, void *o) {
  if (h_rpct(8)) { /* the destructors test their block pointers: release the blocks by hand and clear the pointers first */
    RsObj *r = (RsObj *)o;
    if (r->hist_) { sysdep_free(r->hist_); r->hist_ = NULL; }
    if (!r->borrowed_ && r->coef_) { sysdep_free(r->coef_); r->coef_ = NULL; }
    if (c->kind == 3) { VrObj *v = (VrObj *)o; if (v->tHist_) { sysdep_free(v->tHist_); v->tHist_ = NULL; } }
    h_cov_hit(H_COV_DTOR_NULL);
  }
  if (c->heap) { if (h_rpct(25)) direct_d0(c->kind, o); else destroy_obj(c->kind, o, 2); }
  else destroy_obj(c->kind, o, (int)h_rmod(3) == 2 ? 3 : (int)h_rmod(2));
  free_cfg(c);
}

/* scenario 0: pointer-form objects, the interposed virtual call (the recorder + phase policies) vs the stream model */
static void sc0(unsigned ntrials) {
  unsigned long ctr = 0;
  unsigned t;
  define_cov();
  h_scenario_begin(1);
  register_vtables();
  {
    static const int off[] = {CV_FLOATFORM, CV_HOOK_REAL, CV_FLOAT_TABLE, CV_TABLE_TAPS_NON4, CV_ALLOC_FAIL, CV_HIST0, CV_TAPS0, CV_OFF_NEG, CV_OFF_POS, CV_SETTO, CV_GETPPM, CV_RESET_VIRT,
      CV_TM_A, CV_TM_B, CV_TM_PARITY_HIGH, CV_SD0, CV_SD1, CV_SD2, CV_SD3, CV_SD_DONE, CV_SD_AFTER_DONE, CV_ADJ_NOTDONE, CV_ADJ_P0, CV_ADJ_NAN, CV_ADJ_N_GT4, CV_ADJ_N_2_4, CV_ADJ_N_1_2,
      CV_ADJ_N_LT1, CV_ADJ_N_EQ4, CV_ADJ_N_EQ2, CV_ADJ_N_EQ1, CV_DBG1, CV_DBG2, CV_DBG3, CV_DBG_NEG, CV_DBG_BIG, CV_RESET_UNS, CV_INVERT, CV_ADD, CV_ADD_NEG, CV_BLL_SAME, CV_BLL_OOR,
      CV_BLL_NEWFLAG, CV_HIST_REC, CV_HIST_WRAP, CV_P160_0, CV_MEAN, CV_STD, CV_STD_NEGVAR, CV_SNAN, CV_DESIGN, CV_NORMPH, CV_ZERO_L, CV_STEP_LT_1, -1};
    int i;
    cov_clear(off);
    for (i = 0; i < CV_BLL_COUNT; i++) h_covreq[CV_BLL0 + i] = 0;
  }
  for (t = 0; t < ntrials; t++) {
    Cfg c;
    Sess s;
    void *o;
    unsigned Lv = pick_L(), taps = pick_taps();
    int kind = h_rpct(45) ? 0 : (int)h_rmod(4);
    int pol = (int)h_rmod(5);
    cfg_ptr(&c, kind, Lv, taps);
    c.heap = h_rpct(30);
    c.probe = h_rpct(15);
    c.ppmW = ppm_word();
    if (kind == 3) c.params = new_params(h_rmod(8), 0, 100, 0);
    c.stepW = pol >= 3 ? step_word(Lv, 32, 256) : (h_rpct(25) ? step_irregular(Lv, 4) : step_word(Lv, 8, 256));
    memset(&s, 0, sizeof s);
    s.interposed = 1; s.pol = pol; s.ncalls = 8 + h_rmod(14); s.maxblock = 1 + h_rmod(60);
    s.regime = h_rpct(60) ? R_UNIT : (h_rpct(50) ? R_INT : R_WIDE);
    s.poke = 1; s.dumpEvery = 3;
    h_cov_hit(CV_TRIAL);
    if (c.taps % 4) h_cov_hit(CV_REMAINDER);
    { float sf = f_of(c.stepW); if (sf > (float)Lv) h_cov_hit(CV_STEP_GT_L); }
    o = session(&c, &s, &ctr);
    dump_obj(0, "end", kind, o);
    finish_obj(&c, o);
    h_trial_end("s0");
  }
}

/* ------------------------------------------------------------------------------------------------ single operations with pre / post expectations */
/* the checks of "the resampler part is in its reset state" */
static void expect_rs_reset(const RsObj *r, const char *what) {
  unsigned i;
  expect_d(what, r->phase_, 0.0); expect_u(what, r->need_, 0); expect_u(what, r->wr_, r->taps_); expect_u(what, r->pendCount_, 0);
  for (i = 0; i < 5; i++) expect_word(what, bits_f(r->pend_[i]), 0);
  if (r->hist_) for (i = 0; i < r->histLen_; i++) expect_word(what, bits_f(((const float *)r->hist_)[i]), 0);
}
static void expect_timing_clear(const RtObj *t, const char *what) {
  expect_u(what, t->parity_, 0);
  expect_word(what, bits_f(t->gainA_), 0); expect_word(what, bits_f(t->gainB_), 0); expect_word(what, bits_f(t->errOut_), 0); expect_word(what, bits_f(t->f58_), 0);
  expect_word(what, bits_f(t->lastCorr_), 0); expect_word(what, bits_f(t->energy_), 0); expect_word(what, bits_f(t->energyPrev_), 0); expect_word(what, bits_f(t->bpf_), 0);
  expect_word(what, bits_f(t->bpfPrev_), 0); expect_word(what, bits_f(t->errAcc_), 0); expect_u(what, t->dftN_, 0); expect_word(what, bits_f(t->dftMag_), 0);
  expect_word(what, bits_f(t->dftRe_), 0); expect_word(what, bits_f(t->dftIm_), 0); expect_u(what, t->dftDone_, 0); expect_word(what, bits_f(t->bpfGain_), 0x3d230fd0u);
}

/* the gain schedule of setBllState (independent table: state -> params byte offsets and the debug text) */
static const struct { unsigned offA, offB; const char *text; } BLL[16] = {
  {0, 0, "FROZEN"}, {0, 0, "SECOND ORDER FROZEN"}, {0x88, 0x8c, "INITIAL"}, {0x90, 0x94, "FAST"}, {0x98, 0x9c, "MEDIUM"}, {0xa0, 0xa4, "SLOW"}, {0xa8, 0xac, "SLOW2"},
  {0xb0, 0xb4, "DIL"}, {0xb8, 0xbc, "TRN2 INITIAL"}, {0xc0, 0xc4, "TRN2 "}, {0xc8, 0xcc, "STEADY_STATE"}, {0xd0, 0xd4, "PRE_ANSPCM"}, {0xd8, 0xdc, "TRN1_QC_INITIAL"},
  {0xe0, 0xe4, "TRN1_QC_FAST"}, {0xe8, 0xec, "TRN1_QC_MEDIUM"}, {0xf0, 0xf4, "TRN1_QC_SLOW"}};

static void op_setbll(void *o, unsigned state, unsigned flag, unsigned long *ctr) {
  VrObj *v = (VrObj *)o;
  unsigned st0 = v->state_, nedp0 = g_nedp;
  uint32_t gA0 = bits_f(v->gainA_), gB0 = bits_f(v->gainB_), flag0 = v->stateFlag_;
  char want[200];
  vr_setbll(o, state, flag);
  if (state == st0) {
    h_cov_hit(CV_BLL_SAME);
    expect_u("setBllState same: state_", v->state_, st0); expect_u("same: stateFlag_", v->stateFlag_, flag0); expect_u("same: edprintf", g_nedp, nedp0);
    expect_word("same: gainA_", bits_f(v->gainA_), gA0); expect_word("same: gainB_", bits_f(v->gainB_), gB0);
    return;
  }
  if (flag != flag0) h_cov_hit(CV_BLL_NEWFLAG);
  expect_u("setBllState: stateFlag_", v->stateFlag_, flag);
  expect_u("setBllState: state_", v->state_, state);
  expect_u("setBllState: stateCount_", v->stateCount_, 0);
  if (state > 15) {
    h_cov_hit(CV_BLL_OOR);
    expect_u("oor: edprintf", g_nedp, nedp0); expect_word("oor: gainA_", bits_f(v->gainA_), gA0); expect_word("oor: gainB_", bits_f(v->gainB_), gB0);
  } else {
    uint32_t eA, eB;
    h_cov_hit(CV_BLL0 + (int)state);
    eA = state == 0 ? 0u : state == 1 ? gA0 : pword(v->params_, BLL[state].offA);
    eB = state == 0 ? 0u : pword(v->params_, BLL[state].offB);
    if (state == 1) eB = 0;
    expect_word("setBllState: gainA_", bits_f(v->gainA_), eA); expect_word("setBllState: gainB_", bits_f(v->gainB_), eB);
    expect_u("setBllState: one edprintf", g_nedp, nedp0 + 1);
    want[0] = 0;
    { char t[120]; sprintf(t, "V90Resampler: state = %s\r\n", BLL[state].text); esc_cat(want, sizeof want, t); }
    if (strcmp(want, g_last_edp)) h_die3("setBllState %u: text '%s', expected '%s'", state, g_last_edp, want);
  }
  (void)ctr;
}

/* getNormalizedPhase / setNormalizedPhase */
static void op_setnp(void *o, uint32_t fw) {
  RsObj *r = (RsObj *)o;
  LD v = (LD)fv(fw);   /* NaN words are quiet NaNs here (never signalling: the harness' regimes) */
  double want;
  if (is_nan_w(fw)) want = 0.0;
  else if (!(v >= 0.0L)) want = 0.0;
  else if (v < 1.0L) want = (double)(v * (LD)r->L_);
  else want = 0.0;
  rs_setnp(o, FARG(fw));
  expect_d("setNormalizedPhase", r->phase_, want);
  h_cov_hit(CV_NORMPH);
}
static void op_getnp(void *o) {
  RsObj *r = (RsObj *)o;
  float got = rs_getnp(o);
  float want = (float)((LD)r->phase_ / (LD)r->L_);
  expect_word("getNormalizedPhase", bits_f(got), bits_f(want));
  h_emit(0, "getNP %08x", bits_f(got));
  h_cov_hit(CV_NORMPH);
}

/* setTimingOffset / getTimingOffsetPPM */
static void op_setto(int kind, void *o, uint32_t ppmw) {
  RoObj *r = (RoObj *)o;
  float want = (float)(((LD)r->step_ * (LD)fv(ppmw)) * (LD)K_MICRO);
  (void)kind;
  ro_setto(o, FARG(ppmw));
  expect_word("setTimingOffset: off_", bits_f(r->off_), bits_f(want));
  h_cov_hit(CV_SETTO);
  h_cov_hit(r->off_ < 0 ? CV_OFF_NEG : CV_OFF_POS);
}
static void op_getppm(void *o, unsigned long *ctr) {
  RoObj *r = (RoObj *)o;
  XRET ret = ro_getppm(o);
  float want = (float)(((LD)1000000.0f * (LD)r->off_) / (LD)r->step_);
  float got = (float)ret;
  expect_word("getTimingOffsetPPM", bits_f(got), bits_f(want));
  h_emit(h_sample(ctr, 6, 1999), "getppm %08x", bits_f(got));
  h_cov_hit(CV_GETPPM);
}

/* the blob's resample() needs 0 <= phase_ < L at entry; a direct call of a timing hook (or a poke) can leave it outside: put it back (the harness' own invariant) */
static void fix_phase(void *o) {
  RsObj *r = (RsObj *)o;
  if (r->L_ != 0 && r->L_ < 100000u && !(r->phase_ >= 0.0 && r->phase_ < (double)r->L_)) r->phase_ = (double)h_rmod(r->L_ * 64u) / 64.0;
}
/* direct timingCorrection through the class' own symbol */
static void op_tc(int kind, void *o, uint32_t xw) {
  RsModel m;
  RsObj *r = (RsObj *)o;
  uint64_t ph0 = bits_d(r->phase_);
  if (kind == 0) { rs_tc(o, FARG(xw)); expect_d("Resampler::timingCorrection changes nothing", r->phase_, d_of(ph0)); return; }
  rm_init(&m, kind, o, kind == 1 ? 2 : 3, 0);
  if (kind == 1) { ro_tc(o, FARG(xw)); m.phase = (double)((LD)m.phase + (LD)m.tm.off); h_cov_hit(CV_HOOK_REAL); }
  else { rt_tc(o, FARG(xw)); tm_correct(&m, fv(xw)); }
  rm_check(&m, kind, o, "timingCorrection");
  fix_phase(o);
}

/* the half-baud DFT bin */
static void op_sd(void *o, uint32_t xw) {
  RtObj *t = (RtObj *)o;
  unsigned n0 = t->dftN_;
  uint32_t re0 = bits_f(t->dftRe_), im0 = bits_f(t->dftIm_), mag0 = bits_f(t->dftMag_);
  uint32_t done0 = t->dftDone_;
  LD x = (LD)fv(xw);
  float wre = fv(re0), wim = fv(im0);
  rt_sd(o, FARG(xw));
  if (done0) {
    h_cov_hit(CV_SD_AFTER_DONE);
    expect_u("Sd after done: dftN_", t->dftN_, n0); expect_word("Sd after done: re", bits_f(t->dftRe_), re0); expect_word("Sd after done: im", bits_f(t->dftIm_), im0);
    expect_word("Sd after done: mag", bits_f(t->dftMag_), mag0);
    return;
  }
  switch (n0 & 3u) {
    case 0: wre = (float)(x + (LD)wre); h_cov_hit(CV_SD0); break;
    case 1: wim = (float)((LD)wim - x); h_cov_hit(CV_SD1); break;
    case 2: wre = (float)((LD)wre - x); h_cov_hit(CV_SD2); break;
    default: wim = (float)(x + (LD)wim); h_cov_hit(CV_SD3); break;
  }
  expect_u("Sd: dftN_", t->dftN_, n0 + 1);
  expect_word("Sd: re", bits_f(t->dftRe_), bits_f(wre)); expect_word("Sd: im", bits_f(t->dftIm_), bits_f(wim));
  if (n0 + 1 == 256) {
    float mag = (float)sqrtLD((LD)wre * (LD)wre + (LD)wim * (LD)wim);
    h_cov_hit(CV_SD_DONE);
    expect_u("Sd: done", t->dftDone_, 1); expect_word("Sd: mag", bits_f(t->dftMag_), bits_f(mag));
  } else {
    expect_u("Sd: not done", t->dftDone_, 0); expect_word("Sd: mag unchanged", bits_f(t->dftMag_), mag0);
  }
}

/* the gain normalisation and its debug text */
/* the print `back` calls ago (1 = the last one) */
static void expect_print(unsigned absn, const char *fmt0, LD v, float f) {
  const char *gf = g_dbg_ring[absn & 7u].fmt;
  int gs = g_dbg_ring[absn & 7u].sign, gi = g_dbg_ring[absn & 7u].ip, gp = g_dbg_ring[absn & 7u].fp;
  int sign = (0.0L >= (LD)f) ? '-' : '+';
  LD av = v < 0 ? -v : v;
  int ip = fistp32(av), tv = fistp32(v);
  LD d = (v - (LD)tv) * 1000.0L;
  int fp = fistp32(d);
  unsigned u = (unsigned)fp, mm = (unsigned)(fp >> 31);
  fp = (int)((u ^ mm) - mm);
  if (strcmp(gf, fmt0) || gs != sign || gi != ip || gp != fp)
    h_die3("debug print: '%s' %d %d %d, expected '%s' %d %d %d", gf, gs, gi, gp, fmt0, sign, ip, fp);
  if (sign == '-' && v < 0) h_cov_hit(CV_DBG_NEG);
  if (!(v > -2147483649.0L && v < 2147483648.0L)) h_cov_hit(CV_DBG_BIG);
}
static void op_adj(void *o, uint32_t xw) {
  RtObj *t = (RtObj *)o;
  uint32_t done0 = t->dftDone_, mag0 = bits_f(t->dftMag_), gain0 = bits_f(t->bpfGain_);
  unsigned nd0 = g_ndbg;
  unsigned level = dsplibs_debug_level, L1, L2, L3, p1, p2, p3;
  LD P;
  LD Y;
  float magf, gainf, yf;
  g_lvl_plan_on = h_rpct(40);                    /* the recorder lowers / raises the level between the prints (the blob re-reads it before each one) */
  g_lvl_plan[0] = h_rpct(60) ? level : h_rmod(4);
  g_lvl_plan[1] = h_rpct(60) ? g_lvl_plan[0] : h_rmod(4);
  g_lvl_plan[2] = level;
  L1 = level; L2 = g_lvl_plan_on ? g_lvl_plan[0] : level; L3 = g_lvl_plan_on ? g_lvl_plan[1] : level;
  g_call_prints = 0;
  rt_adj(o, FARG(xw));
  g_lvl_plan_on = 0;
  dsplibs_debug_level = level;                   /* restore */
  if (!done0) { h_cov_hit(CV_ADJ_NOTDONE); expect_word("adjust (not done): mag", bits_f(t->dftMag_), mag0); expect_word("adjust (not done): gain", bits_f(t->bpfGain_), gain0); expect_u("adjust (not done): no print", g_ndbg, nd0); return; }
  P = (LD)fv(xw) * (LD)fv(mag0);
  magf = (float)P;
  if (P > 0.0L || P < 0.0L) {
    LD N = (1.0L / P) * 350000.0L;
    if (N == 4.0L) h_cov_hit(CV_ADJ_N_EQ4);
    if (N == 2.0L) h_cov_hit(CV_ADJ_N_EQ2);
    if (N == 1.0L) h_cov_hit(CV_ADJ_N_EQ1);
    if (N > 4.0L) { Y = 1.0L; h_cov_hit(CV_ADJ_N_GT4); }
    else if (N > 2.0L) { Y = 2.0L; h_cov_hit(CV_ADJ_N_2_4); }
    else if (N >= 1.0L) { Y = N; h_cov_hit(CV_ADJ_N_1_2); }
    else if (N < 1.0L) { Y = 1.0L; h_cov_hit(CV_ADJ_N_LT1); }
    else { Y = N; h_cov_hit(CV_ADJ_NAN); }       /* N is NaN */
  } else if (P == 0.0L) { Y = 1.0L; h_cov_hit(CV_ADJ_P0); }
  else { Y = 1.0L; h_cov_hit(CV_ADJ_NAN); }
  gainf = (float)(Y * (LD)0.03981f);
  yf = (float)Y;
  expect_word("adjust: dftMag_", bits_f(t->dftMag_), bits_f(magf));
  expect_word("adjust: bpfGain_", bits_f(t->bpfGain_), bits_f(gainf));
  p1 = L1 > 1; p2 = p1 && L2 > 1; p3 = p2 && L3 > 1;
  expect_u("adjust: number of debug prints", g_ndbg, nd0 + p1 + p2 + p3);
  if (p1) { expect_print(nd0, "ResamplerTiming::adjustHalfBaudBpfGain  baud/2 dft bin = %c%d.%03d\\r\\n", P, magf); h_cov_hit(CV_DBG1); }
  if (p2) { expect_print(nd0 + 1, "ResamplerTiming::adjustHalfBaudBpfGain  normFactor = %c%d.%03d\\r\\n", (LD)yf, yf); h_cov_hit(CV_DBG2); }
  if (p3) { expect_print(nd0 + 2, "ResamplerTiming::adjustHalfBaudBpfGain  normBPFhBaudB0coef = %c%d.%03d\\r\\n", (LD)gainf, gainf); h_cov_hit(CV_DBG3); }
}

/* invertPhase / addPhase (dead in the blob) */
static void reduce_phase(LD P, unsigned L, unsigned *need, double *phase, int ge_exit) {
  LD Lf = (LD)L;
  if (P >= Lf) {   /* the blob's jb skips the loop for P < L and for NaN */
    unsigned n = *need;
    if (ge_exit) { do { P -= Lf; n++; } while (P >= Lf); }
    else { do { P -= Lf; n++; } while (Lf <= P); }
    *need = n;
  }
  *phase = (double)P;
}
static void op_invert(void *o) {
  RtObj *t = (RtObj *)o;
  unsigned need = t->need_;
  double ph;
  LD P = (LD)t->phase_ + (LD)t->step_;
  reduce_phase(P, t->L_, &need, &ph, 0);
  rt_inv(o);
  expect_d("invertPhase: phase_", t->phase_, ph); expect_u("invertPhase: need_", t->need_, need);
  h_cov_hit(CV_INVERT);
}
static void op_add(void *o, uint32_t aw) {
  RtObj *t = (RtObj *)o;
  unsigned need = t->need_;
  double ph = t->phase_;
  LD a = (LD)fv(aw);
  rt_add(o, FARG(aw));
  if (!(a > 0.0L)) { h_cov_hit(CV_ADD_NEG); expect_d("addPhase (a <= 0): phase_", t->phase_, ph); expect_u("addPhase (a <= 0): need_", t->need_, need); return; }
  {
    LD P = (LD)ph + (a * (LD)t->step_) * (LD)0.0055555556900799274f;
    double wph;
    reduce_phase(P, t->L_, &need, &wph, 1);
    expect_d("addPhase: phase_", t->phase_, wph); expect_u("addPhase: need_", t->need_, need);
    h_cov_hit(CV_ADD);
  }
}

/* reset through the vtable (slot 2) and reset(unsigned) */
static void op_reset_virtual(int kind, void *o) {
  void **vt = *(void ***)o;
  RsObj *r = (RsObj *)o;
  unsigned ne0 = g_nedp;
  ((void (*)(void *))vt[2])(o);
  h_cov_hit(CV_RESET_VIRT);
  expect_rs_reset(r, "reset(): resampler part");
  if (kind >= 1) expect_word("reset(): off_", bits_f(((RoObj *)o)->off_), 0);
  if (kind == 3) {
    VrObj *v = (VrObj *)o;
    unsigned i;
    expect_timing_clear((RtObj *)o, "V90Resampler::reset(): timing part");
    expect_u("V90 reset: state_", v->state_, 0); expect_u("V90 reset: stateCount_", v->stateCount_, 0); expect_u("V90 reset: stateFlag_", v->stateFlag_, 1);
    expect_u("V90 reset: tHistIdx_", v->tHistIdx_, 0); expect_u("V90 reset: tCount_", v->tCount_, 0);
    if (v->tHist_) for (i = 0; i < v->tHistLen_; i++) expect_word("V90 reset: tHist_", bits_f(((float *)v->tHist_)[i]), 0);
    expect_u("V90 reset: one edprintf (FROZEN)", g_nedp, ne0 + 1);
    if (strcmp(g_last_edp, "V90Resampler: state = FROZEN\\r\\n")) h_die3("V90 reset: edprintf '%s'", g_last_edp);
  }
}
static void op_reset_uns(int kind, void *o, unsigned arg) {
  RsObj *r = (RsObj *)o;
  uint32_t st = 0;
  VrObj snap;
  if (kind == 3) memcpy(&snap, o, sizeof snap);
  (void)st;
  rt_reset(o, arg);
  h_cov_hit(CV_RESET_UNS);
  expect_rs_reset(r, "reset(unsigned): resampler part");
  expect_word("reset(unsigned): off_", bits_f(((RoObj *)o)->off_), 0);
  expect_timing_clear((RtObj *)o, "reset(unsigned): timing part");
  if (kind == 3) {
    VrObj *v = (VrObj *)o;
    expect_u("reset(unsigned): V90 state_ unchanged", v->state_, snap.state_); expect_u("reset(unsigned): stateCount_ unchanged", v->stateCount_, snap.stateCount_);
    expect_u("reset(unsigned): tCount_ unchanged", v->tCount_, snap.tCount_); expect_u("reset(unsigned): tHistIdx_ unchanged", v->tHistIdx_, snap.tHistIdx_);
  }
}

static void op_rhi(void *o) { RsObj *r = (RsObj *)o; rs_rhi(o); expect_u("resetHistoryIndex", r->wr_, r->taps_); }
static void op_cht(void *o) {
  RsObj *r = (RsObj *)o;
  unsigned i, t = r->taps_;
  uint32_t *exp;
  if (!r->hist_ || t == 0 || t > r->histLen_) { rs_cht(o); return; }
  exp = (uint32_t *)malloc(r->histLen_ * 4u);
  memcpy(exp, r->hist_, r->histLen_ * 4u);
  for (i = 0; i < t; i++) { uint32_t w; memcpy(&w, (const float *)r->hist_ + (r->histLen_ - t + i), 4); exp[i] = w; }
  rs_cht(o);
  for (i = 0; i < r->histLen_; i++) { uint32_t w; memcpy(&w, (const float *)r->hist_ + i, 4); if (w != exp[i]) h_die3("copyHistoryTail: hist[%u] %08x, expected %08x", i, w, exp[i]); }
  free(exp);
}

/* V90: timing history mean / std (extended: the model sums in explicit long double) */
static void op_hist_stats(void *o, unsigned long *ctr, int ext) {
  VrObj *v = (VrObj *)o;
  unsigned n = v->tHistLen_, i;
  const float *h = (const float *)v->tHist_;
  LD sum = 0.0L, sq = 0.0L, mean, var, stdv;
  float sqf;
  XRET rm, rs;
  if (!h) return;
  for (i = 0; i < n; i++) sum += (LD)h[i];
  mean = sum / (LD)n;
  for (i = 0; i < n; i++) { LD x = (LD)h[i]; sq += x * x; }
  sqf = (float)(sq / (LD)n);
  var = (LD)sqf - mean * mean;
  { LD k = (var >= 0.0L) ? 1.0L : -1.0L; __asm__("" : "+m"(k)); stdv = sqrtLD(k * var); }
  if (!(var >= 0.0L)) h_cov_hit(CV_STD_NEGVAR);
  rm = vr_mean(o);
  rs = vr_std(o);
  h_cov_hit(CV_MEAN); h_cov_hit(CV_STD);
  if (!word_eq(bits_f((float)rm), bits_f((float)mean))) h_die3("getTimingHistoryMean: %08x, expected %08x", bits_f((float)rm), bits_f((float)mean));
  if (!word_eq(bits_f((float)rs), bits_f((float)stdv))) h_die3("getTimingHistoryStd: %08x, expected %08x", bits_f((float)rs), bits_f((float)stdv));
  h_emit(h_sample(ctr, 6, 1999), "stats mean %08x std %08x", bits_f((float)rm), bits_f((float)rs));
  (void)ext;
}

/* ------------------------------------------------------------------------------------------------ the generic operation trial */
static uint32_t word_unit_small(void) { return ((h_rnd() & 1u) << 31) | ((uint32_t)(126 - (int)h_rmod(10)) << 23) | (h_rnd() & 0x7fffffu); }
static uint32_t word_float_int(int v) { float f = (float)v; return bits_f(f); }
static void fill_dft_samples(unsigned long *ctr, void *o, unsigned n, int regime) {
  unsigned i;
  for (i = 0; i < n; i++) op_sd(o, rbits(regime));
  (void)ctr;
}

/* a float for setNormalizedPhase: around 0 and 1 and the boundaries; never a signalling NaN (the model reads it by value) */
static uint32_t np_word(void) {
  static const uint32_t t[] = {0x00000000u, 0x80000000u, 0x3f800000u, 0x3f7fffffu, 0x3f800001u, 0xbf800000u, 0x00000001u, 0x007fffffu, 0x3f000000u, 0x7fc00000u, 0xffc00000u, 0x7f800000u, 0xff800000u, 0x3e800000u, 0x40000000u};
  return t[h_rmod((unsigned)(sizeof t / sizeof t[0]))];
}
typedef struct { int kind, ptrForm, floatCut, nops, regime, level; } OpsPlan;
static void ops_run(const Cfg *cp, const OpsPlan *pl, unsigned long *ctr);
static void ops_trial(const OpsPlan *pl, unsigned long *ctr) {
  Cfg c;
  unsigned Lv = pick_L(), taps = pick_taps();
  int kind = pl->kind;
  memset(&c, 0, sizeof c);
  c.kind = kind; c.ptrForm = pl->ptrForm; c.useC2 = h_rpct(40); c.L = Lv; c.taps = taps;
  c.histLen = h_rpct(40) ? 0u : (h_rpct(50) ? taps + 1 + h_rmod(2 * taps + 3) : taps * 10 + 1 + h_rmod(40));
  if (h_rpct(8)) c.histLen = c.ptrForm ? taps : (taps & ~3u);   /* histLen == taps: the boundary of the history size rule */
  if (!c.ptrForm) { unsigned t4 = taps & ~3u; if (t4 == 0) { taps = 4 + 4 * h_rmod(8) + h_rmod(4); c.taps = taps; } c.cutW = bits_f((float)(1 + h_rmod(16 * Lv)) * 0.0625f); if (c.taps % 4) h_cov_hit(CV_TABLE_TAPS_NON4); }
  else { c.coef = new_floats(Lv * taps + 1); fill_coef(c.coef, Lv * taps + 1, kind >= 2 ? 0 : (int)h_rmod(3)); }  /* a timing loop fed with large samples can stall the blob's loop: unit tables / data */
  c.stepW = h_rpct(25) ? step_irregular(Lv, 8) : step_word(Lv, 8, 256);
  { int k = h_rrange(-400, 400); c.ppmW = word_float_int(k * 1000); }
  if (kind == 3) c.params = new_params(h_rpct(30) ? 0u : 1u + h_rmod(20), h_rpct(70) ? 1u : h_rmod(3), h_rpct(30) ? 0u : 1u + h_rmod(60), 0);
  c.heap = h_rpct(25);
  c.probe = h_rpct(10);
  ops_run(&c, pl, ctr);
}
/* the operation loop on an object built by Cfg (also used with degenerate configurations by scenario 5) */
static void ops_run(const Cfg *cp, const OpsPlan *pl, unsigned long *ctr) {
  Cfg c = *cp;
  void *o;
  RsObj *r;
  RsModel m;
  unsigned op;
  int kind = pl->kind, tmp_regime = kind >= 2 ? R_UNIT : pl->regime;
  unsigned total = 0, pos = 0;
  float *st;
  int maxblock = 1 + (int)h_rmod(40);
  g_ed_show = 0;
  dsplibs_debug_level = (unsigned)pl->level;
  o = make_obj(&c);
  r = (RsObj *)o;
  g_watch = r; g_watch_kind = kind;
  if (kind >= 2) { /* gains for the timing loop (small, like the V90 parameter block's) */
    RtObj *t = (RtObj *)o;
    uint32_t w = ((uint32_t)(125 - (int)h_rmod(8)) << 23) | (h_rnd() & 0x7fffffu); memcpy(&t->gainA_, &w, 4);
    w = ((uint32_t)(122 - (int)h_rmod(8)) << 23) | (h_rnd() & 0x7fffffu); memcpy(&t->gainB_, &w, 4);
  }
  dump_obj(h_sample(ctr, 6, 1999), "init", kind, o);
  total = (unsigned)pl->nops * (unsigned)maxblock + 32;
  st = new_floats(total + 8);
  fill_regime(st, total + 8, tmp_regime);
  for (op = 0; op < (unsigned)pl->nops; op++) {
    int rr = (int)h_rmod(100);
    char tag[48];
    sprintf(tag, "op %u", op);
    g_opno = op; g_oprr = rr; g_opw = -1;
    if ((kind == 0 ? rr < 62 : kind == 1 ? rr < 50 : kind == 2 ? rr < 28 : rr < 30) && r->hist_ && r->L_ != 0 && r->coef_) { /* resample (the real hook runs); a NULL history crashes the blob, L == 0 never leaves its wrap loop: excluded */
      unsigned n = rr % 7 == 0 ? 0u : 1u + h_rmod((unsigned)maxblock), nout = 0xdeadbeefu, nm, cap, i, wr0;
      uint32_t *mout;
      float *out;
      int hk = kind == 0 ? 0 : kind == 1 ? 2 : 3;
      if (pos + n + 4 > total) pos = 0;
      if (r->hist_ && r->taps_ + 1 <= r->histLen_ && h_rpct(20)) poke_state(kind, o, tmp_regime);
      if (r->pendCount_ > 3) r->pendCount_ = 3;
      rm_init(&m, kind, o, hk, 0);
      cap = (n + 3) * (unsigned)((double)r->L_ / (double)r->step_ * 2.0 + 6.0) + 16;
      out = new_floats(cap);
      mout = (uint32_t *)malloc(cap * 4u);
      if (getenv("P05_DEBUG")) fprintf(stderr, "%s kind %d L %u taps %u histLen %u wr %u n %u pc %u need %u phase %.6f step %.6f off %.9g hk %d\n", tag, kind, r->L_, r->taps_, r->histLen_, r->wr_, n, r->pendCount_, r->need_, r->phase_, (double)r->step_, kind >= 1 ? (double)((RoObj *)o)->off_ : 0.0, hk);
      wr0 = r->wr_;
      if (n == 0) h_cov_hit(CV_N0); else if (n == 1) h_cov_hit(CV_N1);
      g_watch = r; g_watch_kind = kind;
      { V90Pre vp; if (kind == 3) v90_pre(o, &vp);
        if (kind == 3) vr_resample(o, st + pos, n, out, &nout); else rs_resample(o, st + pos, n, out, &nout);
        if (kind == 3) v90_post(o, &vp, n); }
      if (r->wr_ < wr0) h_cov_hit(CV_WRAP);
      nm = rm_resample(&m, as_words(st + pos), n, mout);
      if (nout != nm) h_die3("%s: nOut %u, model %u (n %u)", tag, nout, nm, n);
      for (i = 0; i < nout; i++) if (!word_eq(bits_f(out[i]), mout[i])) h_die3("%s: out[%u] %08x, model %08x", tag, i, bits_f(out[i]), mout[i]);
      rm_check(&m, kind, o, tag);
      h_emit(0, "resample n=%u nOut=%u #%016llx", n, nout, (unsigned long long)fnv_blk(out, nout * 4ul));
      sysdep_free(out);
      free(mout);
      pos += n;
    } else if (rr < 66) { op_reset_virtual(kind, o); }
    else if (rr < 71) { op_setnp(o, h_rpct(25) ? np_word() : (rbits(R_UNIT) & 0xbfffffffu)); }
    else if (rr < 76) { op_getnp(o); }
    else if (rr < 78) { op_rhi(o); }
    else if (rr < 80 && r->hist_) { op_cht(o); }
    else if (kind == 0) { op_tc(0, o, rbits(R_UNIT)); }
    else if (kind == 1 || rr < 84) {
      int w = (int)h_rmod(3);
      if (w == 0) op_setto(kind, o, word_float_int(h_rrange(-400, 400) * 1000));
      else if (w == 1) op_getppm(o, ctr);
      else op_tc(kind, o, rbits(R_UNIT));
    } else if (kind == 2 || (kind == 3 && rr < 90)) {
      int w = (int)h_rmod(10);
      RtObj *t = (RtObj *)o;
      g_opw = w;
      if (w < 3) { unsigned k = 1 + h_rmod(300); while (k--) op_sd(o, rbits(h_rpct(50) ? R_UNIT : R_WIDE)); }
      else if (w == 3 && h_rpct(35)) { op_adj(o, rbits(h_rpct(50) ? R_UNIT : R_WIDE)); }
      else if (w == 3 && h_rpct(15)) { /* exact products: P = 350000, 175000, 87500 (N = 1, 2, 4 when the extended reciprocal rounds well) and neighbours */
        static const uint32_t xs[] = {0x48aae600u, 0x482ae600u, 0x47aae600u, 0xc8aae600u};
        uint32_t magw = 0x3f800000u, xw = xs[h_rmod(4)] + (uint32_t)((int)h_rmod(3) - 1);
        t->dftDone_ = 1; memcpy(&t->dftMag_, &magw, 4);
        op_adj(o, xw);
      }
      else if (w == 3) { /* a designed pair (mag, x): P = x * mag in a chosen binade, so that N = 350000 / P falls in every range */
        static const int eps[] = {10, 16, 16, 17, 17, 17, 18, 18, 19, 22};
        int e = eps[h_rmod(10)], a = h_rrange(-6, 6);
        uint32_t magw = (uint32_t)(127 + a) << 23, xw = ((h_rpct(15) ? 1u : 0u) << 31) | ((uint32_t)(127 + e - a) << 23) | (h_rnd() & 0x7fffffu);
        t->dftDone_ = 1; memcpy(&t->dftMag_, &magw, 4);
        op_adj(o, xw);
      }
      else if (w == 4) { if (t->dftDone_) op_adj(o, rbits(R_WIDE)); else { fill_dft_samples(ctr, o, 256 - (t->dftN_ & 255u), R_UNIT); op_adj(o, rbits(R_UNIT)); } }
      else if (w == 5) { rt_rsd(o); expect_u("resetSdHalfBaudDft: dftN_", t->dftN_, 0); expect_word("resetSdHalfBaudDft: mag", bits_f(t->dftMag_), 0); expect_word("resetSdHalfBaudDft: re", bits_f(t->dftRe_), 0); expect_word("resetSdHalfBaudDft: im", bits_f(t->dftIm_), 0); }
      else if (w == 6 && r->L_ != 0) { if (h_rpct(30) && (LD)r->step_ <= (LD)r->L_ && r->L_ < 100000u) r->phase_ = (double)((LD)r->L_ - (LD)r->step_);   /* phase + step == L exactly (dyadic values): the boundary of the wrap test */ op_invert(o); }
      else if (w == 7 && r->L_ != 0) { if (h_rpct(15)) r->phase_ = -0.0; op_add(o, h_rpct(12) ? (h_rpct(50) ? 0u : 0x80000000u) : h_rpct(30) ? rbits(R_UNIT) | 0x80000000u : h_rpct(35) ? (((uint32_t)(127 + (int)h_rmod(15)) << 23) | (h_rnd() & 0x7fffffu)) : rbits(R_UNIT) & 0x7fffffffu);   /* some large a: several wraps */ }
      else if (w == 8 || w == 6 || w == 7) { op_reset_uns(kind, o, h_rnd()); }
      else { op_tc(kind, o, rbits(R_UNIT)); }
    } else if (kind == 3) {
      if (h_rpct(70)) op_setbll(o, h_rpct(15) ? 16u + h_rmod(50) : h_rmod(16), h_rpct(50) ? 1u : h_rnd() & 3u, ctr);
      else op_hist_stats(o, ctr, 0);
    } else {
      op_tc(kind, o, rbits(R_UNIT));
    }
    dump_obj(op % 5 == 0 ? h_sample(ctr, 6, 1999) : 0, "obj", kind, o);
  }
  sysdep_free(st);
  g_pol = 0;
  g_watch = NULL;
  finish_obj(&c, o);
}

/* ------------------------------------------------------------------------------------------------ scenario setup helpers */
static void sc_begin(const int *notreq) {
  define_cov();
  h_scenario_begin(1);
  register_vtables();
  { int i; for (i = 0; i < CV_BLL_COUNT; i++) h_covreq[CV_BLL0 + i] = 0; }
  h_covreq[H_COV_DTOR_NULL] = 0;
  { static const int always[] = {CV_FLOATFORM, CV_PTRFORM, -1}; (void)always; }
  cov_clear(notreq);
}
static const int NR_ALL[] = {CV_FLOATFORM, CV_PTRFORM, CV_C2, CV_RS, CV_RO, CV_RT, CV_VR, CV_D0, CV_D1, CV_D2, CV_HOOK_INTERPOSED, CV_HOOK_REAL, CV_POL_JITTER, CV_POL_JUMP, CV_POL_NEG, CV_POL_BOUND,
  CV_NEED_GT_TOTAL, CV_TAIL, CV_N0, CV_N1, CV_WRAP, CV_REMAINDER, CV_LAST_PHASE, CV_NOT_LAST_PHASE, CV_PHASE_CLAMP, CV_PHASE_MULTIWRAP, CV_PHASE_WRAP1, CV_PEND_SHIFT, CV_PEND_MULTI, CV_NEEDGE2,
  CV_STEP_LT_1, CV_STEP_GT_L, CV_FLOAT_TABLE, CV_TABLE_TAPS_NON4, CV_ALLOC_FAIL, CV_HIST0, CV_TAPS0, CV_OFF_NEG, CV_OFF_POS, CV_SETTO, CV_GETPPM, CV_RESET_VIRT, CV_TM_A, CV_TM_B,
  CV_TM_PARITY_HIGH, CV_SD0, CV_SD1, CV_SD2, CV_SD3, CV_SD_DONE, CV_SD_AFTER_DONE, CV_ADJ_NOTDONE, CV_ADJ_P0, CV_ADJ_NAN, CV_ADJ_N_GT4, CV_ADJ_N_2_4, CV_ADJ_N_1_2, CV_ADJ_N_LT1,
  CV_ADJ_N_EQ4, CV_ADJ_N_EQ2, CV_ADJ_N_EQ1, CV_DBG1, CV_DBG2, CV_DBG3, CV_DBG_NEG, CV_DBG_BIG, CV_RESET_UNS, CV_INVERT, CV_ADD, CV_ADD_NEG, CV_BLL_SAME, CV_BLL_OOR, CV_BLL_NEWFLAG,
  CV_HIST_REC, CV_HIST_WRAP, CV_P160_0, CV_MEAN, CV_STD, CV_STD_NEGVAR, CV_SNAN, CV_DESIGN, CV_NORMPH, CV_ZERO_L, CV_TRIAL, -1};
/* the counters that a scenario must reach are the ones NOT listed in its `notreq`: build it as NR_ALL minus the required ones */
static void sc_begin_req(const int *req) {
  int notreq[128], n = 0, i;
  const int *a;
  for (a = NR_ALL; *a >= 0; a++) {
    int keep = 0;
    for (i = 0; req[i] >= 0; i++) if (req[i] == *a) keep = 1;
    if (!keep) notreq[n++] = *a;
  }
  notreq[n] = -1;
  sc_begin(notreq);
}

/* ------------------------------------------------------------------------------------------------ scenario 0 (pointer form, interposed hook) is above; scenario 1 */
static void table_check_trial(unsigned long *ctr) {
  Cfg c;
  void *o;
  RsObj *r;
  unsigned Lv = 1 + h_rmod(24), taps = h_rpct(60) ? 4 * (1 + h_rmod(16)) : 4 + h_rmod(64), t4, hl;   /* L > 12: the over-read of a polyphase row leaves the red zone of the LowPassFIR block */
  memset(&c, 0, sizeof c);
  c.kind = (int)h_rmod(4); c.ptrForm = 0; c.useC2 = h_rpct(50); c.L = Lv; c.taps = taps;
  c.histLen = h_rpct(40) ? 0u : taps + h_rmod(3 * taps + 10);
  c.cutW = bits_f((float)h_rmod(16 * Lv + 1) * 0.0625f);
  c.stepW = step_word(Lv, 8, 256); c.ppmW = word_float_int(h_rrange(-400, 400) * 1000);
  if (c.kind == 3) c.params = new_params(1 + h_rmod(5), 1, 1, 0);
  c.heap = h_rpct(25); c.probe = h_rpct(20);
  o = make_obj(&c);
  r = (RsObj *)o;
  t4 = taps & ~3u;
  hl = t4 < c.histLen ? c.histLen : t4 * 10u;
  expect_u("float ctor: L_", r->L_, Lv); expect_u("float ctor: taps_", r->taps_, t4); expect_u("float ctor: histLen_", r->histLen_, hl);
  expect_u("float ctor: borrowed_", r->borrowed_, 0); expect_u("float ctor: wr_", r->wr_, t4); expect_word("float ctor: step_ word", bits_f(r->step_), c.stepW);
  if (taps % 4) h_cov_hit(CV_TABLE_TAPS_NON4);
  if (!r->coef_) h_die3("float ctor: coef_ NULL");
  if (t4 > 0) {
    LpfObj lpf;
    LD Lf = (LD)Lv;
    float fc = (float)((LD)f_of(c.cutW) / Lf), gain = (float)Lf;
    unsigned p, k;
    const float *tab = (const float *)r->coef_;
    memset(&lpf, 0, sizeof lpf);
    lpf_c1(&lpf, Lv * t4, FARG(bits_f(fc)), 3, FARG(bits_f(gain)));
    if (!lpf.coef_) h_die3("reference LowPassFIR: design failed (n %u fc %08x)", Lv * t4, bits_f(fc));
    for (p = 0; p < Lv; p++)
      for (k = 0; k < t4; k++)
        if (bits_f(tab[p * t4 + k]) != bits_f(lpf.coef_[(t4 - 1 - k) * Lv + p])) h_die3("table[%u][%u] %08x, reference %08x", p, k, bits_f(tab[p * t4 + k]), bits_f(lpf.coef_[(t4 - 1 - k) * Lv + p]));
    lpf_d1(&lpf);
    h_cov_hit(CV_FLOAT_TABLE);
  } else h_cov_hit(CV_TAPS0);
  dump_obj(h_sample(ctr, 6, 1999), "obj", c.kind, o);
  finish_obj(&c, o);
  if (getenv("P05_DEBUG") && h_live) fprintf(stderr, "table_check leak: kind %d L %u taps %u histLen %u cut %08x heap %d C2 %d live %u\n", c.kind, Lv, taps, c.histLen, c.cutW, c.heap, c.useC2, h_live);
}
static void sc1(unsigned ntrials) {
  static const int req[] = {CV_FLOATFORM, CV_C2, CV_RS, CV_RO, CV_RT, CV_VR, CV_D0, CV_D1, CV_D2, CV_FLOAT_TABLE, CV_TABLE_TAPS_NON4, CV_TRIAL, CV_WRAP, CV_N0, CV_N1, CV_TAIL, CV_NEED_GT_TOTAL,
    CV_LAST_PHASE, CV_NOT_LAST_PHASE, CV_PHASE_WRAP1, CV_HOOK_INTERPOSED, CV_HOOK_REAL, -1};
  unsigned long ctr = 0;
  unsigned t;
  sc_begin_req(req);
  for (t = 0; t < ntrials; t++) {
    h_cov_hit(CV_TRIAL);
    table_check_trial(&ctr);
    h_trial_end("s1a");
    {   /* sessions on designed tables: interposed and real hooks */
      OpsPlan pl;
      pl.kind = (int)h_rmod(4); pl.ptrForm = 0; pl.floatCut = 1; pl.nops = 12 + (int)h_rmod(12);
      pl.regime = h_rpct(60) ? R_UNIT : R_INT; pl.level = (int)h_rmod(4);
      ops_trial(&pl, &ctr);
      h_trial_end("s1b");
    }
    {
      Cfg c;
      Sess s;
      void *o;
      unsigned Lv = pick_L(), taps = 4 * (1 + h_rmod(10)) + (h_rpct(30) ? h_rmod(4) : 0);
      int kind = (int)h_rmod(4);
      memset(&c, 0, sizeof c);
      c.kind = kind; c.ptrForm = 0; c.useC2 = h_rpct(60); c.L = Lv; c.taps = taps; c.histLen = h_rpct(50) ? 0u : taps + 4 + h_rmod(40);
      c.cutW = bits_f((float)(1 + h_rmod(16 * Lv)) * 0.0625f); c.ppmW = ppm_word();
      if (kind == 3) c.params = new_params(h_rmod(6), 0, 10, 0);
      c.stepW = step_word(Lv, 8, 256);
      c.heap = h_rpct(25);
      memset(&s, 0, sizeof s);
      s.interposed = 1; s.pol = (int)h_rmod(3); s.ncalls = 6 + h_rmod(8); s.maxblock = 1 + h_rmod(50); s.regime = R_UNIT; s.poke = 1; s.dumpEvery = 2;
      o = session(&c, &s, &ctr);
      finish_obj(&c, o);
      h_trial_end("s1c");
    }
  }
}

/* scenarios 2 / 3 / 4: the operation trials of ResamplerTimingOffset / ResamplerTiming / V90Resampler */
static void sc_ops(int kind, unsigned ntrials) {
  static const int req_all[] = {CV_FLOATFORM, CV_PTRFORM, CV_C2, CV_D0, CV_D1, CV_D2, CV_RESET_VIRT, CV_NORMPH, CV_TRIAL, CV_N0, CV_TAIL, CV_WRAP, CV_NEED_GT_TOTAL, CV_LAST_PHASE, CV_NOT_LAST_PHASE, CV_PHASE_WRAP1, -1};
  unsigned long ctr = 0;
  unsigned t;
  int req[100], n = 0, i;
  for (i = 0; req_all[i] >= 0; i++) req[n++] = req_all[i];
  req[n++] = CV_RO + kind - 1;
  if (kind >= 1) { req[n++] = CV_OFF_POS; req[n++] = CV_OFF_NEG; req[n++] = CV_SETTO; req[n++] = CV_GETPPM; req[n++] = CV_HOOK_REAL; }
  if (kind >= 2) {
    int ids[] = {CV_TM_A, CV_TM_B, CV_SD0, CV_SD1, CV_SD2, CV_SD3, CV_SD_DONE, CV_SD_AFTER_DONE, CV_ADJ_NOTDONE, CV_ADJ_N_GT4, CV_ADJ_N_2_4, CV_ADJ_N_1_2, CV_ADJ_N_LT1, CV_DBG1, CV_DBG2, CV_DBG3,
                 CV_DBG_NEG, CV_RESET_UNS, CV_INVERT, CV_ADD, CV_ADD_NEG, -1};
    for (i = 0; ids[i] >= 0; i++) req[n++] = ids[i];
  }
  if (kind == 3) {
    int ids[] = {CV_BLL_SAME, CV_BLL_OOR, CV_BLL_NEWFLAG, CV_MEAN, CV_STD, CV_HIST_REC, CV_HIST_WRAP, CV_P160_0, -1};
    for (i = 0; ids[i] >= 0; i++) req[n++] = ids[i];
    for (i = 0; i < CV_BLL_COUNT; i++) req[n++] = CV_BLL0 + i;
  }
  req[n] = -1;
  sc_begin_req(req);
  if (kind == 3) { for (i = 0; i < CV_BLL_COUNT; i++) h_covreq[CV_BLL0 + i] = 1; }
  for (t = 0; t < ntrials; t++) {
    OpsPlan pl;
    h_cov_hit(CV_TRIAL);
    pl.kind = kind; pl.ptrForm = h_rpct(55); pl.floatCut = 0; pl.nops = 30 + (int)h_rmod(40);
    pl.regime = kind >= 2 ? (h_rpct(60) ? R_UNIT : R_INT) : (h_rpct(50) ? R_UNIT : h_rpct(50) ? R_INT : R_WIDE);
    pl.level = (int)h_rmod(4);
    ops_trial(&pl, &ctr);
    h_trial_end("ops");
  }
}

static void on_alarm(int sig) {
  char b[400];
  (void)sig;
  { int n0 = snprintf(b, sizeof b, "ALARM: op %u rr %d w %d\n", g_opno, g_oprr, g_opw); if (write(2, b, (size_t)n0) < 0) _exit(3); }
  if (g_watch) {
    int n = snprintf(b, sizeof b, "RUNAWAY: a call did not return (blob / port loop): phase_ %016llx need_ %u L_ %u taps_ %u step_ %08x pendCount_ %u hooks %u\n", (unsigned long long)bits_d(g_watch->phase_), g_watch->need_, g_watch->L_, g_watch->taps_, bits_f(g_watch->step_), g_watch->pendCount_, g_hook_calls);
    if (write(2, b, (size_t)n) < 0) _exit(3);
    if (g_watch_kind >= 2) {
      const RtObj *t = (const RtObj *)g_watch;
      n = snprintf(b, sizeof b, "  off_ %08x gainA %08x gainB %08x errOut %08x lastCorr %08x parity %u energy %08x energyPrev %08x bpf %08x bpfPrev %08x errAcc %08x bpfGain %08x\n",
                   bits_f(t->off_), bits_f(t->gainA_), bits_f(t->gainB_), bits_f(t->errOut_), bits_f(t->lastCorr_), t->parity_, bits_f(t->energy_), bits_f(t->energyPrev_), bits_f(t->bpf_), bits_f(t->bpfPrev_), bits_f(t->errAcc_), bits_f(t->bpfGain_));
      if (write(2, b, (size_t)n) < 0) _exit(3);
    }
  }
  _exit(3);
}

/* ------------------------------------------------------------------------------------------------ scenario 5: constructors / destructors / degenerate arguments / allocation failure */
static uint32_t special_word(void) { return rbits(R_SPECIAL); }
/* the same without any NaN: NaNs are only used where no tie can arise.  Two NaNs of EQUAL significand and DIFFERENT sign meeting in an x87 operation are resolved by the
 * instruction form the compiler picked (float rule 6 / P02), and a signalling NaN in a MEMORY operand meets another NaN differently from the port's `fld`-quieted operand; the
 * operation scenario therefore uses finite values, infinities and denormals only (the NaNs they create are the default NaN, significand 0x400000), constructor words
 * may be anything (nothing is computed with them before the dump), and the NaNs in the resample data are positive with a significand above 0x400000. */
static uint32_t special_word_q(void) { uint32_t w; do { w = rbits(R_SPECIAL); } while (is_nan_w(w)); return w; }
static uint32_t data_nan_word(void) { static const uint32_t t[] = {0x7fa00001u, 0x7fffffffu, 0x7fe00001u}; return t[h_rmod(3)]; }

static void sc5(unsigned ntrials) {
  static const int req[] = {CV_TRIAL, CV_ALLOC_FAIL, CV_HIST0, CV_TAPS0, CV_D0, CV_D1, CV_D2, CV_FLOATFORM, CV_PTRFORM, CV_C2, CV_RS, CV_RO, CV_RT, CV_VR, CV_NORMPH, CV_RESET_VIRT, CV_ZERO_L, -1};
  unsigned long ctr = 0;
  unsigned t;
  sc_begin_req(req);
  for (t = 0; t < ntrials; t++) {
    static const unsigned tapsT[] = {0, 0, 1, 2, 3, 4, 5, 8}, histT[] = {0, 0, 1, 2, 3, 7, 100}, LT[] = {0, 1, 1, 2, 3, 5, 0x80000000u, 0xffffffffu};
    Cfg c;
    OpsPlan pl;
    unsigned Lv = LT[h_rmod(8)], taps = tapsT[h_rmod(8)], hl, t4;
    memset(&c, 0, sizeof c);
    c.kind = (int)h_rmod(4); c.ptrForm = h_rpct(50); c.useC2 = h_rpct(50); c.L = Lv; c.taps = taps; c.histLen = histT[h_rmod(7)];
    if (Lv >= 0x80000000u) { c.ptrForm = 1; c.taps = taps = 0; c.histLen = 0; }   /* L beyond int32: no table, no history; the unsigned conversions of L in getNormalizedPhase / setNormalizedPhase */
    t4 = c.ptrForm ? taps : (taps & ~3u);
    hl = c.histLen > t4 ? c.histLen : t4 * 10u;
    if (c.ptrForm) { c.coef = new_floats(Lv * taps + 1); fill_coef(c.coef, Lv * taps + 1, (int)h_rmod(3)); }
    else c.cutW = bits_f((float)(1 + h_rmod(16 * (Lv ? Lv : 1))) * 0.0625f);
    c.stepW = step_word((Lv && Lv < 1000) ? Lv : 1, 8, 256); c.ppmW = word_float_int(h_rrange(-400, 400) * 1000);
    if (c.kind == 3) c.params = new_params(h_rpct(30) ? 0u : 1u + h_rmod(5), h_rpct(70) ? 1u : h_rmod(3), 1 + h_rmod(5), 0);
    c.heap = h_rpct(30); c.probe = h_rpct(30);
    c.failFirst = hl > 0 && h_rpct(35);
    c.failLast = !c.ptrForm && !c.failFirst && c.kind != 3 && Lv < 0x80000000u && h_rpct(30);   /* kind 3: the last allocation is the timing history, which the blob writes unchecked */
    memset(&pl, 0, sizeof pl);
    pl.kind = c.kind; pl.nops = 8 + (int)h_rmod(14); pl.regime = R_UNIT; pl.level = (int)h_rmod(4);
    h_cov_hit(CV_TRIAL);
    ops_run(&c, &pl, &ctr);
    h_trial_end("s5");
  }
}

/* ------------------------------------------------------------------------------------------------ scenario 6: designed dot products (the accumulation order) */
static uint32_t pow2w(int sign, int e) { return ((uint32_t)(sign ? 1 : 0) << 31) | ((uint32_t)(127 + e) << 23); }
static void sc6(unsigned ntrials) {
  static const int req[] = {CV_TRIAL, CV_DESIGN, CV_REMAINDER, CV_LAST_PHASE, CV_NOT_LAST_PHASE, -1};
  static const unsigned tapsT[] = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 20, 23, 24};
  unsigned long ctr = 0;
  unsigned t;
  sc_begin_req(req);
  for (t = 0; t < ntrials; t++) {
    Cfg c;
    RsObj *r;
    void *o;
    unsigned Lv = 1 + h_rmod(2), n = tapsT[h_rmod(15)], i, row, nout = 0, nm;
    float *st = new_floats(8), out[16];
    uint32_t xw[40], cw[2][40], mout[16];
    uint32_t sx[1];
    RsModel m;
    LD alt = 0.0L, blobg;
    memset(&c, 0, sizeof c);
    c.kind = 0; c.ptrForm = 1; c.useC2 = h_rpct(50); c.L = Lv; c.taps = n; c.histLen = n + 1 + h_rmod(6);
    c.coef = new_floats(Lv * n + 1); c.stepW = bits_f((float)Lv);
    for (i = 0; i < n; i++) { int a = h_rrange(-8, 8); xw[i] = pow2w((int)h_rmod(2), a); }
    sx[0] = pow2w((int)h_rmod(2), h_rrange(-8, 8));
    for (row = 0; row < Lv; row++) {
      unsigned bp = 1 + h_rmod(2), k;
      int e[40], sg[40];
      for (i = 0; i < n; i++) { e[i] = h_rrange(-10, 5); sg[i] = (int)h_rmod(2); }
      for (k = 0; k < bp; k++) {   /* a big term and its exact negative at two random positions */
        unsigned i1 = h_rmod(n), i2 = h_rmod(n);
        int big = 60 + (int)h_rmod(8), s1 = (int)h_rmod(2);
        if (i1 == i2) i2 = (i1 + 1) % n;
        e[i1] = big; sg[i1] = s1; e[i2] = big; sg[i2] = !s1;
      }
      for (i = 0; i < n; i++) {
        int a = (int)((xw[i] >> 23) & 0xff) - 127, sx_ = (int)(xw[i] >> 31);
        cw[row][i] = pow2w(sg[i] ^ sx_, e[i] - a);
      }
      memcpy(c.coef + row * n, cw[row], n * 4u);
    }
    o = make_obj(&c);
    r = (RsObj *)o;
    memcpy(st, sx, 4);
    { uint32_t z = 0; unsigned j; for (j = 1; j < 8; j++) memcpy(&st[j], &z, 4); }
    { unsigned j; for (j = 0; j < r->histLen_; j++) put_word(r->hist_, j, rbits(R_UNIT)); }
    for (i = 0; i < n; i++) memcpy((float *)r->hist_ + i, &xw[i], 4);
    r->wr_ = n; r->pendCount_ = 0; r->need_ = 0; r->phase_ = 0.5;
    interpose(0, o, 0);
    rm_init(&m, 0, o, 1, 0);
    g_hook_no = 0; m.hookNo = 0;
    g_watch = r; g_watch_kind = 0;
    rs_resample(o, st, 2, out, &nout);
    g_watch = NULL;
    nm = rm_resample(&m, as_words(st), 2, mout);
    if (nout != nm) h_die3("designed: nOut %u, model %u", nout, nm);
    for (i = 0; i < nout; i++) if (!word_eq(bits_f(out[i]), mout[i])) h_die3("designed: out[%u] %08x, model %08x", i, bits_f(out[i]), mout[i]);
    /* the design must be sensitive to the grouping: the row-0 dot product with a single accumulator differs from the blob's even / odd structure */
    { for (i = 0; i < n; i++) alt += (LD)f_of(xw[i]) * (LD)f_of(cw[0][i]);
      blobg = dotM(xw, (const float *)cw[0], n);
      if (bits_f((float)alt) != bits_f((float)blobg)) h_cov_hit(CV_DESIGN); }
    h_emit(0, "designed n=%u L=%u nOut=%u out0 %08x", n, Lv, nout, nout ? bits_f(out[0]) : 0u);
    dump_obj(h_sample(&ctr, 6, 499), "obj", 0, o);
    restore_vptr(0, o);
    h_cov_hit(CV_TRIAL);
    if (n % 4) h_cov_hit(CV_REMAINDER);
    destroy_obj(0, o, 0);
    free_cfg(&c);
    sysdep_free(st);
    h_trial_end("s6");
  }
}

/* ------------------------------------------------------------------------------------------------ scenario 7: the extended return values (i386: 80-bit patterns from st(0)) */
static void ext_hex(const char *tag, XRET v) {
#if defined(__i386__)
  unsigned char b[10];
  memcpy(b, &v, 10);
  h_emit(1, "%s %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", tag, b[9], b[8], b[7], b[6], b[5], b[4], b[3], b[2], b[1], b[0]);
#else
  (void)tag; (void)v;
#endif
}
static void sc7(unsigned ntrials) {
  unsigned long ctr = 0;
  unsigned t;
  static const int req[] = {CV_TRIAL, CV_GETPPM, CV_MEAN, CV_STD, CV_STD_NEGVAR, -1};
  sc_begin_req(req);
#if defined(__i386__)
  for (t = 0; t < ntrials; t++) {
    Cfg c;
    void *o;
    RoObj *ro;
    unsigned Lv = pick_L(), taps = pick_taps(), i;
    LD wantp;
    XRET gp;
    memset(&c, 0, sizeof c);
    c.kind = h_rpct(50) ? 1 : 3; c.ptrForm = 1; c.useC2 = h_rpct(50); c.L = Lv; c.taps = taps; c.histLen = 0;
    c.coef = new_floats(Lv * taps + 1); fill_coef(c.coef, Lv * taps + 1, 0);
    c.stepW = h_rpct(30) ? step_irregular(Lv, 4) : step_word(Lv, 4, 256); c.ppmW = word_float_int(h_rrange(-400, 400) * 1000);
    if (c.kind == 3) c.params = new_params(1 + h_rmod(40), 1, 1, 0);
    o = make_obj(&c);
    ro = (RoObj *)o;
    h_cov_hit(CV_TRIAL);
    for (i = 0; i < 4; i++) {
      /* the timing offset: step * ppm * 1e-6 stored as a float; then 1e6 * off / step in extended precision, unrounded in st(0) */
      uint32_t w = h_rpct(20) ? rbits(R_WIDE) : (((h_rnd() & 1u) << 31) | ((uint32_t)(127 - 8 - (int)h_rmod(8)) << 23) | (h_rnd() & 0x7fffffu));
      memcpy(&ro->off_, &w, 4);
      wantp = ((LD)1000000.0f * (LD)ro->off_) / (LD)ro->step_;
      gp = ro_getppm(o);
      if (memcmp(&gp, &wantp, 10)) h_die3("getTimingOffsetPPM: extended result differs from (1e6 * off) / step");
      ext_hex("ppm", gp);
      h_cov_hit(CV_GETPPM);
    }
    if (c.kind == 3) {
      VrObj *v = (VrObj *)o;
      for (i = 0; i < 4; i++) {
        unsigned j;
        LD sum = 0, sq = 0, mean, var, stdv, k;
        float sqf;
        XRET rm, rs;
        for (j = 0; j < v->tHistLen_; j++) put_word(v->tHist_, j, h_rpct(80) ? rbits(R_UNIT) : rbits(R_WIDE));
        for (j = 0; j < v->tHistLen_; j++) sum += (LD)((const float *)v->tHist_)[j];
        mean = sum / (LD)v->tHistLen_;
        for (j = 0; j < v->tHistLen_; j++) { LD x = (LD)((const float *)v->tHist_)[j]; sq += x * x; }
        sqf = (float)(sq / (LD)v->tHistLen_);
        var = (LD)sqf - mean * mean;
        k = (var >= 0.0L) ? 1.0L : -1.0L; __asm__("" : "+m"(k));
        stdv = sqrtLD(k * var);
        if (!(var >= 0.0L)) h_cov_hit(CV_STD_NEGVAR);
        rm = vr_mean(o); rs = vr_std(o);
        if (memcmp(&rm, &mean, 10)) h_die3("getTimingHistoryMean: extended result differs from the model");
        if (memcmp(&rs, &stdv, 10)) h_die3("getTimingHistoryStd: extended result differs from the model");
        ext_hex("mean", rm); ext_hex("std", rs);
        h_cov_hit(CV_MEAN); h_cov_hit(CV_STD);
      }
    }
    finish_obj(&c, o);
    h_trial_end("s7");
  }
#else
  (void)ctr; (void)t; (void)ntrials;
  h_covreq[CV_TRIAL] = h_covreq[CV_GETPPM] = h_covreq[CV_MEAN] = h_covreq[CV_STD] = h_covreq[CV_STD_NEGVAR] = 0;
#endif
}

/* ------------------------------------------------------------------------------------------------ scenario 8: special values (signalling NaNs, infinities, denormals); blob vs port only */
static void sc8(unsigned ntrials) {
  static const int req[] = {CV_TRIAL, CV_SNAN, CV_RS, CV_RO, CV_RT, CV_VR, -1};
  unsigned long ctr = 0;
  unsigned t;
  sc_begin_req(req);
  for (t = 0; t < ntrials; t++) {
    Cfg c;
    void *o;
    RsObj *r;
    unsigned Lv = pick_L(), taps = pick_taps(), k, nops;
    int kind = (int)h_rmod(4), ctorOnly;
    memset(&c, 0, sizeof c);
    c.kind = kind; c.ptrForm = h_rpct(60); c.useC2 = h_rpct(50); c.L = Lv; c.taps = taps; c.histLen = 0;
    if (c.ptrForm) { c.coef = new_floats(Lv * taps + 1); if (h_rpct(50)) fill_coef(c.coef, Lv * taps + 1, 0); else { unsigned q; for (q = 0; q < Lv * taps + 1; q++) put_word(c.coef, q, h_rpct(15) ? (data_nan_word() | 0x00400000u) : special_word_q()); } }
    else { c.cutW = bits_f((float)(1 + h_rmod(16 * Lv)) * 0.0625f); if (taps < 4) c.taps = taps = 4; }
    ctorOnly = h_rpct(35);
    c.stepW = ctorOnly ? special_word() : special_word_q(); c.ppmW = ctorOnly ? special_word() : special_word_q();
    if (is_nan_w(c.stepW) && is_nan_w(c.ppmW)) c.ppmW = word_float_int(1000);   /* two NaNs in one x87 product: which payload wins is outside the model (float rule 6) */
    if (is_snan_w(c.stepW) || is_snan_w(c.ppmW)) h_cov_hit(CV_SNAN);
    if (kind == 3) c.params = new_params(h_rmod(5), 0, 1, 0);
    c.heap = h_rpct(25); c.probe = h_rpct(20);
    h_cov_hit(CV_TRIAL);
    o = make_obj(&c);
    r = (RsObj *)o;
    g_watch = r; g_watch_kind = kind;
    dump_obj(h_sample(&ctr, 6, 199), "obj", kind, o);
    expect_word("step_ word", bits_f(r->step_), c.stepW);
    nops = ctorOnly ? 0 : 6 + h_rmod(10);
    for (k = 0; k < nops; k++) {
      int w = (int)h_rmod(9);
      uint32_t xw = special_word_q();
      if (is_snan_w(xw)) h_cov_hit(CV_SNAN);
      if (w == 0 && kind >= 1) ro_setto(o, FARG(special_word_q()));
      else if (w == 1 && kind >= 1) { XRET v = ro_getppm(o); h_emit(0, "getppm %08x", bits_f((float)v)); }
      else if (w == 2) rs_setnp(o, FARG(special_word_q()));
      else if (w == 3) h_emit(0, "getnp %08x", bits_f(rs_getnp(o)));
      else if (w == 4 && kind == 1) ro_tc(o, FARG(xw));
      else if (w == 4 && kind >= 2) rt_tc(o, FARG(xw));
      else if (w == 5 && kind >= 2) { RtObj *tt = (RtObj *)o; tt->dftDone_ = (uint8_t)h_rmod(2); rt_sd(o, FARG(xw)); }
      else if (w == 6 && kind >= 2) { RtObj *tt = (RtObj *)o; uint32_t mw = special_word_q(); tt->dftDone_ = 1; memcpy(&tt->dftMag_, &mw, 4); dsplibs_debug_level = h_rmod(4); rt_adj(o, FARG(xw)); }
      else if (w == 7 && kind >= 2 && r->L_ != 0 && is_fin_w(bits_f(r->step_)) && fv(bits_f(r->step_)) < 1000.0f && fv(bits_f(r->step_)) > -1000.0f && (r->phase_ >= -1e6 && r->phase_ <= 1e6) && (is_nan_w(xw) || (is_fin_w(xw) && ((xw >> 23) & 0xff) < 127 + 10))) rt_add(o, FARG(xw));   /* +inf / huge never leaves the wrap loop of the blob */
      else if (w == 8 && kind == 3) { vr_setbll(o, h_rmod(20), h_rnd() & 1u); }
      else if (kind <= 1 && r->hist_ && r->L_ && w == 4 && h_rpct(50) && is_fin_w(bits_f(r->step_)) && fv(bits_f(r->step_)) > 0.25f && fv(bits_f(r->step_)) < 1000.0f && (kind == 0 || (is_fin_w(bits_f(((RoObj *)o)->off_)) && fv(bits_f(((RoObj *)o)->off_)) > -0.1f && fv(bits_f(((RoObj *)o)->off_)) < 0.1f))) { /* data-dependent only for kind 0 / 1: a hook that ignores the samples */
        unsigned n8 = 1 + h_rmod(20), cap8 = (n8 + 3) * (unsigned)((double)r->L_ / (double)fv(bits_f(r->step_)) * 2.0 + 4.0) + 16;
        float *st = new_floats(40), *out = new_floats(cap8);
        unsigned nout = 0;
        { unsigned q; for (q = 0; q < 40; q++) put_word(st, q, h_rpct(25) ? data_nan_word() : special_word_q()); }   /* samples may be signalling NaNs (history / pending moves must keep the bits); the coefficients are quiet */
        if (!(r->phase_ >= 0.0 && r->phase_ < (double)r->L_)) r->phase_ = 0.0;
        r->pendCount_ = 0;
        rs_resample(o, st, n8, out, &nout);
        h_emit(0, "resample nOut=%u #%016llx", nout, (unsigned long long)fnv_blk(out, nout * 4ul));
        if (getenv("P05_DEBUG")) { unsigned q; fprintf(stderr, "S8 resample L=%u taps=%u kind %d step %08x:", r->L_, r->taps_, kind, bits_f(r->step_)); for (q = 0; q < nout; q++) fprintf(stderr, " %08x", bits_f(out[q])); fprintf(stderr, "\n  st:"); for (q = 0; q < 40; q++) fprintf(stderr, " %08x", bits_f(st[q])); fprintf(stderr, "\n  coef:"); for (q = 0; q < r->L_ * r->taps_ && q < 40; q++) fprintf(stderr, " %08x", bits_f(((float *)r->coef_)[q])); fprintf(stderr, "\n"); }
        sysdep_free(st); sysdep_free(out);
      }
      dump_obj(h_sample(&ctr, 6, 199), "obj", kind, o);
    }
    g_watch = NULL;
    finish_obj(&c, o);
    h_trial_end("s8");
  }
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P05_SEED");
  signal(SIGALRM, on_alarm);
  alarm(getenv("P05_ALARM") ? (unsigned)atoi(getenv("P05_ALARM")) : 120u);
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  p05_layout_check();
  h_x87_check();
  h_rz_by_id = 1;   /* per-block red zone bytes: an over-read of one block copied past the end of the next is not invisible */
  h_arena_init();
  switch (sc) {
    case 0: sc0(2400); break;
    case 1: sc1(900); break;
    case 2: sc_ops(1, 1200); break;
    case 3: sc_ops(2, 1200); break;
    case 4: sc_ops(3, 1200); break;
    case 5: sc5(3000); break;
    case 6: sc6(7500); break;
    case 7: sc7(500); break;
    case 8: sc8(2400); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu hooks %u edprintf %u dbgprintf %u", h_nmalloc, h_nfree, g_hook_calls, g_nedp, g_ndbg);
  h_cov_finish();
  return 0;
}
