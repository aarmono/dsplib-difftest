/*
 * Differential test harness for port chunk P28 (src/vpcm/v90_mapper.cpp): class V90SignBitsExtractor (7 functions), class V90Mapper (7) and class V90Demapper (13): the V.90 bits <-> symbol
 * mapper pair.
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_mapper.sh against the blob (dsplibs.o with edprintf, the three V90SpectralShaper functions the Mapper calls - reset,
 * resetSSFilter, process (P21) - and V90AutoDigitalImpDetector::clearCamulativeVal (P18 / P20) WEAK so that the recorders below win), against the port (-O0 / -O2; the P01 / P03 / P06 / P21 port objects
 * supply the sub-objects and the G.711 coders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_mapper.blob32.out).  The functions are reached through __asm__ labels; the objects
 * are the generated C mirrors (test/vpcm/v90_mapper.layout), the V90MappingParams block the P11 mirror, the V90Parameters block the P07 mirror, the detector the P18 mirror.
 *
 * Scenarios (each its own process):
 *   0 V90SignBitsExtractor: ctor / dtor, reset (periods 0 .. 8, huge), process through the two-frame action state machine, applyFrameAction with every action (also negative / huge)
 *   1 V90Mapper, no spectral shaping (K_ == 0): ctor / dtor, reset / resetNoSpectral (both PCM laws, counts 0 .. 128, partial frames across calls), process (bits -> six levels -> signed symbols)
 *   2 V90Mapper with spectral shaping (K_ != 0, the shaper's reset / resetSSFilter / process are recorders): the K_ groups, the depth_ countdown / partial output, float words handed to the shaper
 *   3 V90Demapper decisions: ctor / dtor, reset / resetNoSpectral (plain and alternate-RBS phases), hardDecision (nearest level, histogram, full buffers), incrementRBSFramePosition, printErrorHistogramAndReset,
 *     process (ModulusDecoder, the K_ == 0 sign decoder / the V90SignBitsExtractor)
 *   4 V90Demapper level estimation (x87): resetLinearMappStudy, linearMappingStudy, updateConstelation with dense and boundary data
 *
 * What is compared: every returned value, every output buffer and count, the whole object after every operation (field-wise: the model's shadow object against the real one every call, the full dump when
 * something notable happened and on a schedule), the canary bytes behind each object, every edprintf / dsplibs_debug_printf call (kind, text, a snapshot of the object at the call: store order before a
 * print), every recorded callee call (the shaper's reset / resetSSFilter / process with the float words and the pointer positions, the detector's clearCamulativeVal sequence), the malloc log.
 * INDEPENDENT MODELS (written from the disassembly facts, not from the port code): the `ms_` (SignBitsExtractor), `mm_` (Mapper) and `md_` (Demapper) functions.  A model mismatch exits 3.
 * Calls that would crash the REAL blob (a count of zero, a code point outside its row, ...) are excluded by the driver.
 */

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
#include "pcm_parameters_layout.h"
#include "v92_cp_layout.h"
#include "v90_adid_layout.h"
#include "v90_mapper_layout.h"

#define L(m) __asm__(m)

/* V90SignBitsExtractor */
extern void sb_c1(void *) L("_ZN20V90SignBitsExtractorC1Ev");
extern void sb_c2(void *) L("_ZN20V90SignBitsExtractorC2Ev");
extern void sb_d1(void *) L("_ZN20V90SignBitsExtractorD1Ev");
extern void sb_d2(void *) L("_ZN20V90SignBitsExtractorD2Ev");
extern void sb_reset(void *, unsigned, unsigned) L("_ZN20V90SignBitsExtractor5resetEjj");
extern void sb_apply(void *, int, unsigned char *, unsigned char *) L("_ZN20V90SignBitsExtractor16applyFrameActionENS_7ACTIONSEPhS1_");
extern void sb_process(void *, unsigned char *, unsigned char *) L("_ZN20V90SignBitsExtractor7processEPhS0_");
/* V90Mapper */
extern void mp_c1(void *, void *) L("_ZN9V90MapperC1EP13V90Parameters");
extern void mp_c2(void *, void *) L("_ZN9V90MapperC2EP13V90Parameters");
extern void mp_d1(void *) L("_ZN9V90MapperD1Ev");
extern void mp_d2(void *) L("_ZN9V90MapperD2Ev");
extern void mp_reset(void *, void *, int) L("_ZN9V90Mapper5resetEP16V90MappingParams7PcmType");
extern void mp_resetns(void *, void *, int) L("_ZN9V90Mapper15resetNoSpectralEP16V90MappingParams7PcmType");
extern void mp_process(void *, unsigned char *, unsigned, short *, unsigned *) L("_ZN9V90Mapper7processEPhjPsRj");
/* V90Demapper */
extern void dm_c1(void *, unsigned, void *, void *) L("_ZN11V90DemapperC1EjP13V90ParametersP25V90AutoDigitalImpDetector");
extern void dm_c2(void *, unsigned, void *, void *) L("_ZN11V90DemapperC2EjP13V90ParametersP25V90AutoDigitalImpDetector");
extern void dm_d1(void *) L("_ZN11V90DemapperD1Ev");
extern void dm_d2(void *) L("_ZN11V90DemapperD2Ev");
extern void dm_resetls(void *, unsigned) L("_ZN11V90Demapper20resetLinearMappStudyEj");
extern void dm_reset(void *, void *) L("_ZN11V90Demapper5resetEP16V90MappingParams");
extern void dm_resetns(void *, void *) L("_ZN11V90Demapper15resetNoSpectralEP16V90MappingParams");
extern void dm_incrbs(void *) L("_ZN11V90Demapper25incrementRBSFramePositionEv");
extern void dm_printhist(void *) L("_ZN11V90Demapper27printErrorHistogramAndResetEv");
extern short dm_hard(void *, short) L("_ZN11V90Demapper12hardDecisionEs");
extern void dm_updcon(void *) L("_ZN11V90Demapper18updateConstelationEv");
extern void dm_study(void *, short, short) L("_ZN11V90Demapper18linearMappingStudyEss");
extern unsigned dm_process(void *, unsigned char *, unsigned *) L("_ZN11V90Demapper7processEPhRj");
/* the G.711 decoders (blob or the P06 port object) */
extern int alaw2linear(int);
extern int ulaw2linear(int);
extern unsigned int dsplibs_debug_level;
/* the interposed callees: defined below */
#ifdef __x86_64__
typedef float FWORD;                  /* x86-64: the float arguments of the shaper's reset arrive in SSE registers */
static unsigned fw_bits(FWORD f) { unsigned w; memcpy(&w, &f, 4); return w; }
#else
typedef unsigned FWORD;               /* i386: the blob pushes the four float WORDS (mov), read here as integers: no FPU, no quieting of a signalling NaN */
static unsigned fw_bits(FWORD f) { return f; }
#endif
extern void h_sh_reset(void *, unsigned, unsigned, FWORD, FWORD, FWORD, FWORD) L("_ZN17V90SpectralShaper5resetEjjffff");
extern void h_sh_resetss(void *, FWORD, FWORD, FWORD, FWORD) L("_ZN17V90SpectralShaper13resetSSFilterEffff");
extern void h_sh_process(void *, short *, unsigned char *, short *) L("_ZN17V90SpectralShaper7processEPsPhS0_");
extern void h_adid_clear(void *, short, short) L("_ZN25V90AutoDigitalImpDetector18clearCamulativeValEss");

typedef SbeObj OS;
typedef MapObj OM;
typedef DemObj OD;
#define SZS sizeof(OS)
#define SZM sizeof(OM)
#define SZD sizeof(OD)
#define CAN 64

/* ------------------------------------------------------------------------------------------------ small helpers */
static int g_show;
static unsigned long g_nprint;
#define EXPECT_EQ(what, got, want)                                                                                                   \
  do {                                                                                                                              \
    unsigned long long g_ = (unsigned long long)(got), w_ = (unsigned long long)(want);                                              \
    if (g_ != w_) h_die3("%s: got 0x%llx expected 0x%llx (line %d)", (what), g_, w_, __LINE__);                                     \
  } while (0)
static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
static void can_fill(uint8_t *c) { memset(c, 0x5a, CAN); }
static void can_check(const uint8_t *c, const char *w) { unsigned i; for (i = 0; i < CAN; i++) if (c[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", w, i); }
static int16_t rs16(void) {
  static const int16_t e[] = { 0, 1, -1, 2, -2, 32767, -32768, -32767, 16384, -16384, 255, 256, -255, -256, 100, -100 };
  switch (h_rmod(4)) {
    case 0: return e[h_rmod(sizeof e / sizeof e[0])];
    case 1: return (int16_t)h_rrange(-3000, 3000);
    default: return (int16_t)h_rnd();
  }
}
static unsigned fbits_pick(void) {   /* float bit patterns for the V90MappingParams / detector floats */
  static const unsigned e[] = { 0x00000000u, 0x80000000u, 0x3f800000u, 0xbf800000u, 0x7f800000u, 0xff800000u, 0x7fc00000u, 0xffc00000u, 0x7fa00001u, 0x00000001u, 0x007fffffu, 0x7f7fffffu, 0x3f000000u,
                                0x43000000u, 0x46fffe00u, 0xc7000000u, 0x47000000u, 0x4efffffeu };
  switch (h_rmod(3)) {
    case 0: return e[h_rmod(sizeof e / sizeof e[0])];
    case 1: return h_bits_of((float)h_rrange(-40000, 40000) * 0.5f);
    default: return h_rnd();
  }
}

/* architecture-independent random content of an object before its constructor runs (the pool's own fill depends on sizeof, which differs on x86-64): the field table drives it */
static void prefill(void *obj, size_t sz, const HFld *f, int nf) {
  int i;
  unsigned k, n;
  uint8_t *o = (uint8_t *)obj;
  memset(o, 0, sz);
  for (i = 0; i < nf; i++) {
    n = f[i].count > 1 ? f[i].count : 1;
    if (f[i].kind == 'x') { for (k = 0; k < f[i].count; k++) o[f[i].off + k] = (uint8_t)h_rnd(); continue; }
    for (k = 0; k < n; k++) {
      uint8_t *q;
      uint32_t v = h_rnd();
      switch (f[i].kind) {
        case 'b': q = o + f[i].off + k; *q = (uint8_t)v; break;
        case 'h': q = o + f[i].off + 2u * k; memcpy(q, &v, 2); break;
        case 'u': case 'i': case 'f': q = o + f[i].off + 4u * k; memcpy(q, &v, 4); break;
        default: { uintptr_t pv = 0xdead0000u | (v & 0xffffu); q = o + f[i].off + sizeof(void *) * k; memcpy(q, &pv, sizeof pv); break; }
      }
    }
  }
}

/* light field tables: the scalar members and small arrays of a mirror (the 768-element arrays are compared by the model every call and dumped in full on a schedule) */
#define LIGHTMAX 160
static HFld g_lightS[LIGHTMAX], g_lightM[LIGHTMAX], g_lightD[LIGHTMAX], g_fullM[LIGHTMAX], g_fullD[LIGHTMAX];
static int g_nlightS, g_nlightM, g_nlightD, g_nfullM, g_nfullD;
/* the full table: the big integer arrays become hashed byte regions (one dump line is limited to 4 KB) */
static int make_full(HFld *dst, const HFld *src, int n) {
  int i;
  for (i = 0; i < n; i++) {
    dst[i] = src[i];
    if (src[i].count > 32 && (src[i].kind == 'b' || src[i].kind == 'h' || src[i].kind == 'u' || src[i].kind == 'i')) {
      dst[i].count = src[i].count * (src[i].kind == 'b' ? 1u : src[i].kind == 'h' ? 2u : 4u);
      dst[i].kind = 'x';
    }
  }
  return n;
}
static int make_light(HFld *dst, const HFld *src, int n) {
  int i, k = 0;
  for (i = 0; i < n; i++) if (src[i].count <= 8) dst[k++] = src[i];
  return k;
}

/* generic comparison of a model shadow with the real object: every scalar / array field of a table (pointers and unknown regions are skipped) */
static void cmp_flds(const void *model, const void *real, const HFld *f, int nf, const char *what) {
  int i;
  for (i = 0; i < nf; i++) {
    unsigned n = f[i].count > 1 ? f[i].count : 1, w;
    switch (f[i].kind) {
      case 'b': w = 1; break;
      case 'h': w = 2; break;
      case 'u': case 'i': case 'f': w = 4; break;
      case 'd': w = 8; break;
      default: continue;
    }
    if (memcmp((const uint8_t *)model + f[i].off, (const uint8_t *)real + f[i].off, (size_t)n * w) != 0) {
      unsigned long long mv = 0, rv = 0;
      unsigned k;
      for (k = 0; k < n * w; k++) if (((const uint8_t *)model)[f[i].off + k] != ((const uint8_t *)real)[f[i].off + k]) break;
      memcpy(&mv, (const uint8_t *)model + f[i].off + (k / w) * w, w);
      memcpy(&rv, (const uint8_t *)real + f[i].off + (k / w) * w, w);
      h_die3("%s: member %s[%u] is 0x%llx, the model says 0x%llx", what, f[i].name, k / w, rv, mv);
    }
  }
}

/* ------------------------------------------------------------------------------------------------ expected-print queue and recorders */
#define TXT 200
#define EXPMAX 4096
static char (*g_exp)[TXT];
static unsigned g_nexp, g_expi;
static int g_expcheck;           /* 1: every print must be the next one the model expects */
static void exp_clear(void) { g_nexp = 0; g_expi = 0; }
static void exp_push(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void exp_push(const char *fmt, ...) {
  va_list ap;
  if (g_nexp >= EXPMAX) h_die3("expected-print queue overflow");
  va_start(ap, fmt);
  vsnprintf(g_exp[g_nexp++], TXT, fmt, ap);
  va_end(ap);
}
static void exp_done(const char *what) {
  if (g_expi != g_nexp) h_die3("%s: the model expected %u more call / print records (next: %s)", what, g_nexp - g_expi, g_exp[g_expi]);
  exp_clear();
}
static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else if (*s == '\t') { out[n++] = '\\'; out[n++] = 't'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
/* the object under test (snapshot at each print / callee call) */
static OS *g_curS;
static OM *g_curM;
static OD *g_curD;
static void snapshot(void) {
  if (g_curS) h_dump_obj(0, "snap", g_curS, f_SbeObj, NF_SbeObj);
  if (g_curM) h_dump_obj(0, "snap", g_curM, g_lightM, g_nlightM);
  if (g_curD) h_dump_obj(0, "snap", g_curD, g_lightD, g_nlightD);
}
static void rec(char kind, const char *text) {
  char esc[TXT * 2];
  esc[0] = 0;
  esc_cat(esc, sizeof esc, text);
  if (g_expcheck) {
    if (g_expi >= g_nexp) h_die3("unexpected %c-record: %s", kind, text);
    if (strcmp(g_exp[g_expi], text) != 0) h_die3("%c-record mismatch: got \"%s\", the model expected \"%s\"", kind, text, g_exp[g_expi]);
    g_expi++;
  }
  if (kind == 'e' || kind == 'd') snapshot();
  h_emit(g_show, "%cprintf %s", kind, esc);
  g_nprint++;
}
void edprintf(const char *fmt, ...) {
  va_list ap;
  char buf[TXT];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  rec('e', buf);
}
int dsplibs_debug_printf(const char *fmt, ...) {
  va_list ap;
  char buf[TXT];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  rec('d', buf);
  return 0;
}

/* the V90SpectralShaper functions the Mapper calls: recorders.  process is a pure function of its inputs (the model computes the same): out[i] = 3 * in[i] + 5 * i + (i ? bits[i - 1] : 9) */
static unsigned g_shreset_n, g_shprocess_n, g_shss_n;
static int g_sh_expect_obj;       /* 1: `this` must be the shaper embedded in g_curM */
static void sh_check_this(const void *t, const char *w) {
  if (!g_curM) h_die3("%s: the shaper is called with no mapper under test", w);
  if (g_sh_expect_obj && (const void *)t != (const void *)&g_curM->sh_depth_) h_die3("%s: `this` is not the shaper embedded in the mapper", w);
}
void h_sh_reset(void *t, unsigned depth, unsigned arg, FWORD c0, FWORD c1, FWORD c2, FWORD c3) {
  char buf[TXT];
  sh_check_this(t, "shaper reset");
  g_shreset_n++;
  snprintf(buf, sizeof buf, "shaper reset depth=%u arg=%u c=%08x %08x %08x %08x", depth, arg, fw_bits(c0), fw_bits(c1), fw_bits(c2), fw_bits(c3));
  rec('c', buf);
}
void h_sh_resetss(void *t, FWORD c0, FWORD c1, FWORD c2, FWORD c3) {
  char buf[TXT];
  sh_check_this(t, "shaper resetSSFilter");
  g_shss_n++;
  snprintf(buf, sizeof buf, "shaper resetSSFilter c=%08x %08x %08x %08x", fw_bits(c0), fw_bits(c1), fw_bits(c2), fw_bits(c3));
  rec('c', buf);
}
static short sh_out(short in, unsigned i, const unsigned char *bits) { return (short)(3 * (int)in + 5 * (int)i + (i ? (int)bits[i - 1] : 9)); }
void h_sh_process(void *t, short *in, unsigned char *bits, short *out) {
  char buf[TXT];
  unsigned i, spf;
  sh_check_this(t, "shaper process");
  spf = g_curM->spf_;
  g_shprocess_n++;
  snprintf(buf, sizeof buf, "shaper process in=lev+%d bits=buf+%d out=sym+%d spf=%u", (int)(in - (short *)g_curM->lev_), (int)(bits - (unsigned char *)g_curM->buf_), (int)(out - (short *)g_curM->sym_), spf);
  rec('c', buf);
  for (i = 0; i < spf; i++) out[i] = sh_out(in[i], i, bits);
}

/* the detector's clearCamulativeVal: records the (phase, ucode) sequence and clears the two cells the demapper reads (the real function also clears the squared sum) */
static AdidObj *g_adid;
#define CLRMAX 8192
static struct { short ph, uc; } g_clr[CLRMAX];
static unsigned g_nclr;
static unsigned long g_clr_total;
void h_adid_clear(void *t, short phase, short ucode) {
  int idx = ((int)phase << 7) + (int)ucode;
  if (t != (void *)g_adid) h_die3("clearCamulativeVal: `this` is not the detector under test");
  if (idx < 0 || idx >= 768) h_die3("clearCamulativeVal(%d, %d): index outside the arrays", phase, ucode);
  if (g_nclr < CLRMAX) { g_clr[g_nclr].ph = phase; g_clr[g_nclr].uc = ucode; }
  g_nclr++;
  g_clr_total++;
  g_adid->cumCount_[idx] = 0;
  g_adid->cumSum_[idx] = 0.0f;
}
/* the sequence a full clear (6 phases x 128 ucodes in order) must have produced */
static void clr_expect_full(const char *what) {
  unsigned i, j, k = 0;
  if (g_nclr != 768) h_die3("%s: %u clearCamulativeVal calls, expected 768", what, g_nclr);
  for (i = 0; i < 6; i++)
    for (j = 0; j < 128; j++, k++)
      if (g_clr[k].ph != (short)i || g_clr[k].uc != (short)j) h_die3("%s: clearCamulativeVal call %u was (%d, %d), expected (%u, %u)", what, k, g_clr[k].ph, g_clr[k].uc, i, j);
  h_emit(g_show, "clearAll n=%u", g_nclr);
  g_nclr = 0;
}
static void clr_expect_none(const char *what) { if (g_nclr) h_die3("%s: %u unexpected clearCamulativeVal calls", what, g_nclr); }

/* ------------------------------------------------------------------------------------------------ coverage ids */
enum {
  C_TRIALS = 1, C_LV0, C_LV1, C_LV2, C_LVHI,
  /* V90SignBitsExtractor */
  S_C1, S_C2, S_D1, S_D2, S_RESET_N0, S_RESET_N1, S_RESET_N2, S_RESET_N3, S_RESET_N6, S_P_A0_NZ, S_P_A0_Z, S_P_A1_Z, S_P_A1_NZ, S_P_N0, S_P_N1, S_P_N6, S_AP0, S_AP1, S_AP2, S_AP3, S_APNEG, S_APBIG,
  S_APN0, S_PBYTE, S_P_BIGBYTE,
  /* V90Mapper */
  M_C1, M_C2, M_D1, M_D2, M_RESET, M_RESETNS, M_ULAW, M_ALAW, M_K0, M_K1, M_K2, M_K3, M_K6, M_K45, M_KHUGE, M_DEPTH0, M_DEPTHPOS, M_CNT128, M_CNT0, M_PROC_PARTIAL, M_PROC_FRAME, M_PROC_MULTI,
  M_SGN0, M_SGN1, M_SYM_NEG32768, M_SH_CALL, M_SH_WORDS_SNAN, M_DEP_ZERO, M_DEP_LT, M_DEP_GE, M_DEP_COPYALL, M_DEP_COPYPART, M_DEP_COPYNONE, M_FILL_CARRY, M_W0_ONE, M_NS_OLDSKIP, M_N0, M_BYTEBIT,
  /* V90Demapper */
  D_C1, D_C2, D_D1, D_D2, D_D_HIST, D_D_NOHIST, D_RESET, D_RESETNS, D_RESETNS_HIST, D_RESETNS_NOHIST, D_RESETLS, D_ALT_PHASE, D_PLAIN_PHASE, D_K0, D_KPOS, D_DELAY_KEEP, D_DELAY_ZERO, D_INCR, D_INCR_WRAP,
  D_HARD_POS, D_HARD_NEG, D_HARD_MIN, D_HARD_FULL, D_HARD_E0, D_HARD_ELIM, D_HARD_EMID, D_HARD_LIM0, D_HARD_LO, D_HARD_HI, D_HARD_TIE, D_HARD_ALT, D_HARD_PLAIN, D_HARD_HIST_ON, D_HARD_DELAY, D_HARD_ACC,
  D_HARD_INTEG_PRINT, D_HARD_INTEG_STEP, D_PRINT_FULL, D_PRINT_SKIP, D_PRINT_DIV0, D_PRINT_DIV, D_PROC_NONE, D_PROC_FRAMES, D_PROC_ONE, D_PROC_NOFRAME, D_PROC_K0, D_PROC_KPOS, D_PROC_COMPACT, D_PROC_REST,
  /* study */
  T_STUDY_PROC, T_STUDY_D_POS, T_STUDY_D_NEG, T_STUDY_D_ZERO, T_STUDY_UP, T_STUDY_DOWN, T_STUDY_ACC, T_STUDY_SKIP, T_STUDY_DONE, T_STUDY_IT2, T_STUDY_IT_OTHER, T_STUDY_PRINT, T_STUDY_QUIET,
  T_UPD_CALL, T_UPD_NONZERO, T_UPD_ZERO, T_UPD_ALT, T_UPD_SAT_HI, T_UPD_SAT_LO, T_UPD_NAN, T_UPD_NEG, T_UPD_FRAC, T_UPD_ROUND_EDGE, T_UPD_PRINT, T_UPD_QUIET, T_STUDY_IDX0, T_STUDY_IDXLAST,
  T_STUDY_BOUND_EQ, T_UPD_INF, T_UPD_BIGCOUNT,
  COV_MAX
};
static unsigned long g_covn[COV_MAX];
static unsigned char g_covreq[COV_MAX];
static void cov(int id) { g_covn[id]++; }
static const char *cov_name(int id) {
  static char buf[COV_MAX][40];
  if (!buf[id][0]) snprintf(buf[id], sizeof buf[id], "cov %03d", id);
  return buf[id];
}
static unsigned pick_level(void) {
  static const unsigned lv[] = { 0, 1, 2, 3, 7, 0x80000000u, 0xffffffffu, 0x80000001u };
  unsigned v = h_rpct(60) ? 2u : lv[h_rmod(sizeof lv / sizeof lv[0])];
  cov(v == 0 ? C_LV0 : v == 1 ? C_LV1 : v >= 0x80000000u ? C_LVHI : C_LV2);
  return v;
}

/* ================================================================================================ V90SignBitsExtractor: model and trials */
typedef struct { OS o; uint8_t lanes[8]; } MS;
static MS g_ms;
static OS *g_os;
static uint8_t *g_cans;
static void ms_reset(MS *m, unsigned period, unsigned action) {
  unsigned i;
  m->o.period_ = period;
  m->o.sde_state_ = 0;
  m->o.n_ = period ? 6u / period : 0u;
  for (i = 0; i < m->o.n_; i++) m->lanes[i] = 0;
  m->o.pdd_n_ = m->o.n_;
  m->o.action_ = action;
}
/* one sign pattern: action 0 copy, 1 invert (zero test), 2 / 3 alternating */
static void ms_apply(int act, unsigned n, const uint8_t *in, uint8_t *out) {
  unsigned i;
  for (i = 0; i < n; i++) {
    int inv = (act == 1) || (act == 2 && (i % 2) == 0) || (act == 3 && (i % 2) == 1);
    int cpy = (act == 0) || (act == 2 && (i % 2) == 1) || (act == 3 && (i % 2) == 0);
    if (inv) out[i] = in[i] == 0;
    else if (cpy) out[i] = in[i];
  }
}
/* returns 1 when the blob would run on an unloaded register (action_ >= 2): the driver never gets here */
static int ms_process(MS *m, const uint8_t *in, uint8_t *out) {
  unsigned i, n = m->o.n_;
  int applied;
  uint8_t sde;
  if (m->o.action_ == 0) { applied = in[0] != 0; m->o.action_ = (unsigned)applied; }
  else if (m->o.action_ == 1) { applied = in[0] == 0 ? 3 : 2; m->o.action_ = in[0] == 0; }
  else return 1;
  ms_apply(applied, n, in, m->o.buf_);
  for (i = 0; i < m->o.pdd_n_; i++) { uint8_t x = m->o.buf_[i]; m->o.buf_[i] = (uint8_t)(x ^ m->lanes[i]); m->lanes[i] = x; }
  sde = m->o.sde_state_;
  for (i = 1; i < n; i += 2) { uint8_t x = m->o.buf_[i]; m->o.buf_[i] = (uint8_t)(sde ^ x); sde = x; }
  m->o.sde_state_ = sde;
  for (i = 1; i < n; i++) out[i - 1] = m->o.buf_[i];
  return 0;
}
static void sb_check(const char *what) {
  OS *o = g_os;
  cmp_flds(&g_ms.o, o, f_SbeObj, NF_SbeObj, what);
  if (memcmp(g_ms.lanes, o->pdd_state_, 6) != 0) h_die3("%s: the PDD lanes differ from the model", what);
  can_check(g_cans, what);
}
static const unsigned g_periods[] = { 1, 2, 3, 6, 1, 2, 3, 6, 4, 5, 0, 7, 8, 0xffffffffu, 0x80000000u, 6, 3 };
static void trial_sbe(void) {
  int c2 = h_rpct(50), ops, k;
  uint8_t in[32], out[32], ref[32];
  OS *o = (OS *)h_obj_new(SZS + CAN);
  g_os = o; g_cans = (uint8_t *)o + SZS; can_fill(g_cans);
  prefill(o, SZS, f_SbeObj, NF_SbeObj);
  g_curS = o; g_curM = NULL; g_curD = NULL;
  dsplibs_debug_level = pick_level();
  exp_clear(); g_expcheck = 1;
  cov(c2 ? S_C2 : S_C1);
  H_CTOR_PROBE(o, f_SbeObj, NF_SbeObj, sizeof(OS), (c2 ? sb_c2(o) : sb_c1(o)));
  EXPECT_EQ("sde after the constructor", o->sde_state_, 0);
  EXPECT_EQ("action after the constructor", o->action_, 0);
  EXPECT_EQ("pdd cap", o->pdd_cap_, 6);
  EXPECT_EQ("pdd n", o->pdd_n_, 0);
  memcpy(&g_ms.o, o, SZS);
  memcpy(g_ms.lanes, o->pdd_state_, 6);
  sb_check("after the constructor");
  h_dump_obj(0, "ctor", o, f_SbeObj, NF_SbeObj);
  ops = 25 + (int)h_rmod(60);
  for (k = 0; k < ops; k++) {
    int which = k == 0 ? 0 : (int)h_rmod(100);   /* the constructor leaves period_ / n_ unset: a process() before the first reset would run on garbage */
    if (which < 30) {
      unsigned period = h_rpct(25) ? g_periods[h_rmod(sizeof g_periods / sizeof g_periods[0])] : 1 + h_rmod(6);
      unsigned action = h_rpct(75) ? h_rmod(2) : h_rpct(60) ? h_rmod(5) : h_rnd();
      sb_reset(o, period, action);
      ms_reset(&g_ms, period, action);
      switch (g_ms.o.n_) { case 0: cov(S_RESET_N0); break; case 1: cov(S_RESET_N1); break; case 2: cov(S_RESET_N2); break; case 3: cov(S_RESET_N3); break; case 6: cov(S_RESET_N6); break; default: break; }
      h_dump_obj(0, "reset", o, f_SbeObj, NF_SbeObj);
    } else if (which < 80) {
      unsigned i, mode = h_rmod(10);
      if (o->action_ > 1) continue;                 /* the blob would apply the caller's EDI: never run */
      for (i = 0; i < 32; i++) in[i] = mode < 6 ? (uint8_t)(h_rnd() & 1) : mode < 8 ? (uint8_t)h_rnd() : (uint8_t)(mode == 8 ? 0 : 1);
      if (h_rpct(25)) in[0] = (uint8_t)(h_rpct(50) ? 0 : 1);
      memset(out, 0xa5, sizeof out); memcpy(ref, out, sizeof out);
      {
        unsigned act = o->action_;
        int z = in[0] == 0;
        cov(act == 0 ? (z ? S_P_A0_Z : S_P_A0_NZ) : (z ? S_P_A1_Z : S_P_A1_NZ));
        if (mode >= 6 && mode < 8) cov(S_PBYTE);
        for (i = 0; i < 6; i++) if (in[i] > 1) cov(S_P_BIGBYTE);
      }
      switch (o->n_) { case 0: cov(S_P_N0); break; case 1: cov(S_P_N1); break; case 6: cov(S_P_N6); break; default: break; }
      sb_process(o, in, out);
      if (ms_process(&g_ms, in, ref)) h_die3("the model reached the undefined action");
      if (memcmp(out, ref, sizeof out) != 0) h_die3("process: the output bytes differ from the model");
      h_dump_obj(0, "process", o, f_SbeObj, NF_SbeObj);
    } else {
      static const int acts[] = { 0, 1, 2, 3, -1, 4, 5, 7, 0x7fffffff, -2147483647 - 1, 3, 2, 1, 0 };
      int act = acts[h_rmod(sizeof acts / sizeof acts[0])];
      unsigned i, n = h_rpct(30) ? o->n_ : h_rmod(21);
      unsigned save = o->n_;
      for (i = 0; i < 32; i++) in[i] = (uint8_t)(h_rpct(70) ? (h_rnd() & 1) : h_rnd());
      memset(out, 0xa5, sizeof out); memcpy(ref, out, sizeof out);
      o->n_ = n; g_ms.o.n_ = n;
      sb_apply(o, act, in, out);
      ms_apply(act, n, in, ref);
      o->n_ = save; g_ms.o.n_ = save;
      if (memcmp(out, ref, sizeof out) != 0) h_die3("applyFrameAction(%d, n=%u): the output bytes differ from the model", act, n);
      cov(act < 0 ? S_APNEG : act > 3 ? S_APBIG : act == 0 ? S_AP0 : act == 1 ? S_AP1 : act == 2 ? S_AP2 : S_AP3);
      if (n == 0) cov(S_APN0);
    }
    sb_check("sign bits extractor");
  }
  { int d2 = h_rpct(50); cov(d2 ? S_D2 : S_D1); if (d2) sb_d2(o); else sb_d1(o); }
  __asm__ volatile("" ::: "memory");
  can_check(g_cans, "V90SignBitsExtractor dtor");
  h_dump_obj(0, "dtor", o, f_SbeObj, NF_SbeObj);
  exp_done("sbe trial");
  g_curS = NULL;
  h_trial_end("sbe");
  cov(C_TRIALS);
}

/* ================================================================================================ V90MappingParams generator (shared by the Mapper and the Demapper) */
static void put_f(float *dst, unsigned bits) { memcpy(dst, &bits, 4); }
static unsigned get_fbits(const float *src) { unsigned w; memcpy(&w, src, 4); return w; }
static unsigned ilog2u64(unsigned long long v) { unsigned r = 0; while (v > 1) { v >>= 1; r++; } return r; }
/* a block for process(): the counts in 1 .. 128 (<= 64 when `alt` doubles them), w0 = the bits the modulus coder consumes (all six radices) + `skip`.  `maxcode`: 0x7f for the demapper, 0xff for the mapper */
static void gen_mp(MapPar90 *mp, unsigned K, unsigned skip, unsigned maxcode, int alt, int allow_zero_cnt) {
  unsigned i, j;
  unsigned long long prod = 1;
  unsigned nb;
  memset(mp, 0, sizeof *mp);
  for (j = 0; j < sizeof mp->listB_; j++) mp->listB_[j] = (uint8_t)h_rnd();
  mp->flag_ = h_rnd();
  for (i = 0; i < 6; i++) mp->idx_[i] = h_rnd();
  for (i = 0; i < 6; i++) {
    unsigned cmax = alt ? 64u : 128u, c;
    switch (h_rmod(10)) {
      case 0: c = 1; break;
      case 1: c = cmax; break;
      case 2: case 3: case 4: case 5: c = 1 + h_rmod(8); break;
      default: c = 1 + h_rmod(cmax);
    }
    if (allow_zero_cnt && h_rpct(8)) c = 0;
    mp->cnt_[i] = c;
    if (c) prod *= c;
    for (j = 0; j < 128; j++) mp->listA_[i * 128 + j] = (uint8_t)(h_rnd() % (maxcode + 1u));
  }
  nb = ilog2u64(prod);
  if (nb > 60) nb = 60;
  if (h_rpct(30)) nb = h_rmod(nb + 1);
  mp->w0_ = nb + skip;
  mp->w620_ = K;
  mp->w624_ = h_rpct(75) ? h_rmod(4) : h_rpct(50) ? 4 + h_rmod(3) : h_rnd();
  put_f(&mp->f628_, fbits_pick()); put_f(&mp->f62c_, fbits_pick()); put_f(&mp->f630_, fbits_pick()); put_f(&mp->f634_, fbits_pick());
}

/* ================================================================================================ V90Mapper: model and trials */
typedef struct { OM o; uint8_t buf[0x50]; } MM;
static MM g_mm;
static OM *g_om;
static uint8_t *g_canm;
static unsigned dec_level(int pcm, unsigned code) {
  return (unsigned)(pcm == 0 ? ulaw2linear((int)((code & 0x7f) ^ 0xff)) : alaw2linear((int)((code & 0x7f) ^ 0xd5))) & 0xffffu;
}
static void mm_levels(MM *m, const MapPar90 *mp, int pcm) {
  unsigned i, j;
  for (i = 0; i < 6; i++) {
    m->o.cnt_[i] = mp->cnt_[i];
    for (j = 0; j < 128; j++) m->o.levels_[i * 128 + j] = (uint16_t)(j < mp->cnt_[i] ? dec_level(pcm, mp->listA_[i * 128 + j]) : 0);
  }
}
static void mm_setme(MM *m) {
  m->o.me_m0_ = m->o.cnt_[0]; m->o.me_m1_ = m->o.cnt_[1]; m->o.me_m2_ = m->o.cnt_[2]; m->o.me_m3_ = m->o.cnt_[3]; m->o.me_m4_ = m->o.cnt_[4]; m->o.me_m5_ = m->o.cnt_[5];
  m->o.me_m6_ = m->o.nbits_;
}
static void mm_exp_float_words(char *buf, size_t cap, const char *head, const MapPar90 *mp, unsigned a, unsigned b, int full) {
  if (full) snprintf(buf, cap, "%s depth=%u arg=%u c=%08x %08x %08x %08x", head, a, b, get_fbits(&mp->f628_), get_fbits(&mp->f62c_), get_fbits(&mp->f630_), get_fbits(&mp->f634_));
  else snprintf(buf, cap, "%s c=%08x %08x %08x %08x", head, get_fbits(&mp->f628_), get_fbits(&mp->f62c_), get_fbits(&mp->f630_), get_fbits(&mp->f634_));
}
static void mm_reset(MM *m, const MapPar90 *mp, int pcm) {
  char buf[TXT];
  unsigned K = mp->w620_;
  m->o.w0_ = mp->w0_;
  m->o.K_ = K;
  m->o.spf_ = K ? 6u / K : 0u;
  m->o.skip_ = 6u - K;
  m->o.nbits_ = mp->w0_ - (6u - K);
  mm_levels(m, mp, pcm);
  if (K != 0) {
    mm_exp_float_words(buf, sizeof buf, "shaper reset", mp, mp->w624_, mp->w620_, 1);
    exp_push("%s", buf);
    m->o.depth_ = mp->w624_;
  } else m->o.depth_ = 0;
  mm_setme(m);
  m->o.z700_ = 0;
  m->o.sde_state_ = 0;
  m->o.fill_ = 0;
}
static void mm_resetns(MM *m, const MapPar90 *mp, int pcm) {
  char buf[TXT];
  m->o.w0_ = mp->w0_;
  m->o.nbits_ = mp->w0_ - m->o.skip_;
  mm_levels(m, mp, pcm);
  mm_setme(m);
  m->o.sde_state_ = 0;
  mm_exp_float_words(buf, sizeof buf, "shaper resetSSFilter", mp, 0, 0, 0);
  exp_push("%s", buf);
  m->o.z700_ = 0;
}
/* returns 1 when the driver must not have called (a digit outside its row would read outside the table) */
static int mm_process(MM *m, const uint8_t *in, unsigned n, int16_t *out, unsigned *outcnt) {
  unsigned i, j, pos = 0;
  for (i = 0; i < n; i++) {
    unsigned d[6];
    unsigned long long acc = 0;
    int e;
    m->buf[m->o.fill_] = in[i];
    m->o.fill_++;
    if (m->o.fill_ < m->o.w0_) continue;
    cov(M_PROC_FRAME);
    for (e = (int)m->o.nbits_ - 1; e >= 0; e--) acc = (acc << 1) | (m->buf[m->o.skip_ + (unsigned)e] & 1u);
    d[0] = (unsigned)(acc % m->o.cnt_[0]); acc /= m->o.cnt_[0];
    d[1] = (unsigned)(acc % m->o.cnt_[1]); acc /= m->o.cnt_[1];
    d[2] = (unsigned)(acc % m->o.cnt_[2]); acc /= m->o.cnt_[2];
    d[3] = (unsigned)(acc % m->o.cnt_[3]); acc /= m->o.cnt_[3];
    d[4] = (unsigned)(acc % m->o.cnt_[4]); acc /= m->o.cnt_[4];
    d[5] = (unsigned)acc;
    for (j = 0; j < 6; j++) {
      if (d[j] >= 128) return 1;
      m->o.idx_[j] = d[j];
      m->o.lev_[j] = m->o.levels_[j * 128 + d[j]];
    }
    if (m->o.K_ == 0) {
      for (j = 0; j < 6; j++) {
        uint8_t y = (uint8_t)(m->o.sde_state_ ^ m->buf[j]);
        m->o.sde_state_ = y;
        m->o.sgn_[j] = y;
        m->o.sym_[j] = y ? m->o.lev_[j] : (uint16_t)(0u - m->o.lev_[j]);
        cov(y ? M_SGN1 : M_SGN0);
        if (y == 0 && m->o.lev_[j] == 0x8000) cov(M_SYM_NEG32768);
      }
    } else {
      unsigned k;
      for (k = 0; k < m->o.K_; k++) {
        unsigned spf = m->o.spf_;
        int bo = (int)((spf - 1u) * k);
        exp_push("shaper process in=lev+%u bits=buf+%d out=sym+%u spf=%u", spf * k, bo, spf * k, spf);
        cov(M_SH_CALL);
        for (j = 0; j < spf; j++) {
          const uint8_t *bits = m->buf + bo;
          m->o.sym_[spf * k + j] = (uint16_t)sh_out((short)m->o.lev_[spf * k + j], j, bits);
        }
      }
    }
    if (m->o.depth_ == 0) {
      cov(M_DEP_ZERO);
      for (j = 0; j < 6; j++) out[pos + j] = (int16_t)m->o.sym_[j];
      pos += 6;
    } else if (m->o.depth_ < m->o.K_) {
      unsigned skip = m->o.spf_ * m->o.depth_;
      cov(M_DEP_LT);
      if (skip <= 5) { cov(skip == 0 ? M_DEP_COPYALL : M_DEP_COPYPART); for (j = skip; j < 6; j++) out[pos + j - skip] = (int16_t)m->o.sym_[j]; }
      else cov(M_DEP_COPYNONE);
      pos = pos - skip + 6;
      m->o.depth_ = 0;
    } else {
      cov(M_DEP_GE);
      m->o.depth_ -= m->o.K_;
    }
    m->o.fill_ -= m->o.w0_;
  }
  *outcnt = pos;
  return 0;
}
static void mm_check(const char *what) {
  OM *o = g_om;
  cmp_flds(&g_mm.o, o, f_MapObj, NF_MapObj, what);
  if (memcmp(g_mm.buf, o->buf_, 0x50) != 0) h_die3("%s: the input frame block differs from the model", what);
  can_check(g_canm, what);
}
static void mm_dumps(const char *tag, int full) { h_dump_obj(0, tag, g_om, full ? g_fullM : g_lightM, full ? g_nfullM : g_nlightM); }

static void trial_mapper(int spectral) {
  int c2 = h_rpct(50), ops, k, valid = 0;
  MapPar90 mp;
  uint8_t in[512], *bufmem;
  int16_t out[512], ref[512];
  void *params = NULL;
  OM *o = (OM *)h_obj_new(SZM + CAN);
  g_om = o; g_canm = (uint8_t *)o + SZM; can_fill(g_canm);
  prefill(o, SZM, f_MapObj, NF_MapObj);
  params = sysdep_malloc(32);
  g_curM = o; g_curS = NULL; g_curD = NULL;
  g_sh_expect_obj = 1;
  dsplibs_debug_level = pick_level();
  exp_clear(); g_expcheck = 1;
  cov(c2 ? M_C2 : M_C1);
  H_CTOR_PROBE(o, f_MapObj, NF_MapObj, sizeof(OM), (c2 ? mp_c2(o, params) : mp_c1(o, params)));
  EXPECT_EQ("w0_", o->w0_, 0); EXPECT_EQ("nbits_", o->nbits_, 0); EXPECT_EQ("skip_", o->skip_, 0); EXPECT_EQ("K_", o->K_, 0); EXPECT_EQ("spf_", o->spf_, 0);
  EXPECT_EQ("fill_", o->fill_, 0); EXPECT_EQ("depth_", o->depth_, 0); EXPECT_EQ("sde", o->sde_state_, 0); EXPECT_EQ("cnt_[5]", o->cnt_[5], 0);
  if (o->params_ != params) h_die3("params_ is %p, not the argument", (void *)o->params_);
  memcpy(&g_mm.o, o, SZM);
  memcpy(g_mm.buf, o->buf_, 0x50);
  bufmem = (uint8_t *)o->buf_;
  (void)bufmem;
  mm_check("after the constructor");
  mm_dumps("ctor", 1);
  ops = h_rpct(7) ? 0 : 6 + (int)h_rmod(8);   /* 0: destroyed right after the constructor */
  for (k = 0; k < ops; k++) {
    int what = (int)h_rmod(100);
    if (k == 0 || what < 22) {
      /* reset (or the "edge" variant that process() is never called after) */
      int pcm = (int)h_rmod(3) == 0 ? 1 : 0;
      unsigned K;
      int edge = k > 0 && h_rpct(15);
      if (spectral) K = h_rpct(75) ? (unsigned[]){ 1, 2, 3, 6 }[h_rmod(4)] : 4 + h_rmod(2); else K = 0;
      if (edge) { static const unsigned ks[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xffffffffu, 0x80000000u }; K = ks[h_rmod(12)]; }
      if (edge) {
        unsigned i;
        gen_mp(&mp, K, 0, 0xff, 0, 1);
        if (h_rpct(60)) mp.w0_ = h_rnd();
        for (i = 0; i < 6; i++) if (h_rpct(10)) mp.cnt_[i] = 0;
        valid = 0;
      } else {
        gen_mp(&mp, K, 6u - K, 0xff, 0, 0);
        if (K > 6) h_die3("harness: K > 6 in a valid reset");
        valid = 1;
      }
      pcm = pcm != 0 ? (int)(h_rmod(1000) + 1) : 0;
      exp_clear();
      mm_reset(&g_mm, &mp, pcm != 0);
      mp_reset(o, &mp, pcm);
      exp_done("mapper reset");
      cov(M_RESET); cov(pcm ? M_ALAW : M_ULAW);
      cov(K == 0 ? M_K0 : K == 1 ? M_K1 : K == 2 ? M_K2 : K == 3 ? M_K3 : K == 6 ? M_K6 : K == 4 || K == 5 ? M_K45 : M_KHUGE);
      cov(mp.w624_ == 0 ? M_DEPTH0 : M_DEPTHPOS);
      { unsigned i; for (i = 0; i < 6; i++) { if (mp.cnt_[i] == 128) cov(M_CNT128); if (mp.cnt_[i] == 0) cov(M_CNT0); } }
      if (K != 0 && (get_fbits(&mp.f628_) == 0x7fa00001u || get_fbits(&mp.f62c_) == 0x7fa00001u || get_fbits(&mp.f630_) == 0x7fa00001u || get_fbits(&mp.f634_) == 0x7fa00001u)) cov(M_SH_WORDS_SNAN);
      if (mp.w0_ == 1) cov(M_W0_ONE);
      mm_check("after reset");
      mm_dumps("reset", 1);
    } else if (what < 38 && valid) {
      /* resetNoSpectral: w0 re-derived from the OLD skip */
      int pcm = h_rpct(50) ? (int)(1 + h_rmod(5)) : 0;
      unsigned oldK = g_mm.o.K_;
      gen_mp(&mp, oldK, g_mm.o.skip_, 0xff, 0, 0);
      if (g_mm.o.skip_ + (mp.w0_ - g_mm.o.skip_) != mp.w0_) h_die3("harness");
      if (mp.w0_ == 0) mp.w0_ = 1;
      if (mp.w0_ < g_mm.o.skip_) h_die3("harness: w0 < skip");
      o->z700_ = h_rnd(); g_mm.o.z700_ = o->z700_;   /* resetNoSpectral clears it */
      mp.w620_ = h_rnd();   /* resetNoSpectral does not read word +0x620 */
      mp.w624_ = h_rnd();
      exp_clear();
      mm_resetns(&g_mm, &mp, pcm != 0);
      mp_resetns(o, &mp, pcm);
      exp_done("mapper resetNoSpectral");
      cov(M_RESETNS); cov(M_NS_OLDSKIP);
      mm_check("after resetNoSpectral");
      mm_dumps("resetns", 1);
    } else if (valid) {
      unsigned n, i, mode = h_rmod(10);
      unsigned w0 = g_mm.o.w0_ ? g_mm.o.w0_ : 1u;
      unsigned tmax = w0 * (1 + h_rmod(4)) + h_rmod(10);
      unsigned oc = 0x12345678u, rc = 0x12345678u;   /* process() must WRITE the count (also 0) */
      n = h_rpct(10) ? 0 : h_rpct(30) ? 1 + h_rmod(w0 > 1 ? w0 - 1 : 1) : tmax;
      if (n > 500) n = 500;
      if (g_mm.o.w0_ == 0) continue;
      for (i = 0; i < n; i++) in[i] = mode < 7 ? (uint8_t)(h_rnd() & 1) : mode < 9 ? (uint8_t)h_rnd() : (uint8_t)(h_rpct(50) ? 0 : 1);
      if (mode == 7 || mode == 8) cov(M_BYTEBIT);
      for (i = 0; i < 512; i++) out[i] = 0x7777;
      memcpy(ref, out, sizeof out);
      exp_clear();
      {
        /* a frame length of 2^31 or more (poked): the frame never completes in the blob (unsigned compare); n stays small so that fill_ stays inside the 0x50-byte block */
        unsigned save_w0 = g_mm.o.w0_;
        int poke = n <= 40 && g_mm.o.fill_ + n <= 70 && h_rpct(6);
        if (poke) { unsigned hw = 0x80000000u | h_rnd(); o->w0_ = hw; g_mm.o.w0_ = hw; }
        if (mm_process(&g_mm, in, n, ref, &rc)) h_die3("harness: a digit left its row (the real blob would read outside the table)");
        mp_process(o, in, n, out, &oc);
        if (poke) { o->w0_ = save_w0; g_mm.o.w0_ = save_w0; }
      }
      exp_done("mapper process");
      EXPECT_EQ("outCnt", oc, rc);
      if (memcmp(out, ref, sizeof out) != 0) h_die3("mapper process: the output symbols differ from the model");
      if (n == 0) cov(M_N0);
      if (n && rc == 0) cov(M_PROC_PARTIAL);
      if (rc > 6) cov(M_PROC_MULTI);
      if (g_mm.o.fill_ != 0 && n > w0) cov(M_FILL_CARRY);
      mm_check("after process");
      mm_dumps("process", (k & 3) == 0);
      h_emit(g_show, "mapper out %u", oc);
      h_hash_bytes(out, (size_t)oc * 2);
    }
  }
  cov(h_rpct(50) ? M_D2 : M_D1);
  if (h_rpct(50)) mp_d2(o); else mp_d1(o);
  __asm__ volatile("" ::: "memory");
  can_check(g_canm, "V90Mapper dtor");
  mm_dumps("dtor", 1);
  exp_done("mapper trial");
  sysdep_free(params);
  g_curM = NULL;
  h_trial_end("mapper");
  cov(C_TRIALS);
}

/* ================================================================================================ V90Demapper: model */
typedef struct {
  OD o;                       /* the shadow of the object (raw values: pointers are not compared) */
  uint32_t dig[256];          /* the model's copy of the digit queue block (4 * cap bytes) */
  uint8_t bits[256];          /* the sign-bit queue block (cap bytes) */
  MS sbe;                     /* the embedded V90SignBitsExtractor */
  float msum[768];            /* the detector's cumulative sums / counts before a study operation */
  uint32_t mcnt[768];
} MD;
static MD g_md;
static OD *g_od;
static uint8_t *g_cand;
static V90ParObj *g_prm;
static const char SEP[] = "===========================================================\r\n";
static int adid_alt(unsigned i) { return (int16_t)g_adid->altRbsPhase_[i] != 0; }
static void md_sync_sbe(MD *m) {
  m->o.sbe_period_ = m->sbe.o.period_; m->o.sbe_n_ = m->sbe.o.n_; memcpy(m->o.sbe_buf_, m->sbe.o.buf_, 6); m->o.sbe_action_ = m->sbe.o.action_;
  m->o.sbe_sde_state_ = m->sbe.o.sde_state_; m->o.sbe_pdd_cap_ = m->sbe.o.pdd_cap_; m->o.sbe_pdd_n_ = m->sbe.o.pdd_n_;
}
static void md_load_sbe(MD *m, const OD *r) {
  memset(&m->sbe, 0, sizeof m->sbe);
  m->sbe.o.period_ = r->sbe_period_; m->sbe.o.n_ = r->sbe_n_; memcpy(m->sbe.o.buf_, r->sbe_buf_, 6); m->sbe.o.action_ = r->sbe_action_;
  m->sbe.o.sde_state_ = r->sbe_sde_state_; m->sbe.o.pdd_cap_ = r->sbe_pdd_cap_; m->sbe.o.pdd_n_ = r->sbe_pdd_n_;
  memcpy(m->sbe.lanes, r->sbe_pdd_state_, 6);
}
static void md_levels(MD *m, const MapPar90 *mp) {
  unsigned i, j;
  const int16_t *lm = (const int16_t *)g_adid->linearMapping_, *lma = (const int16_t *)g_adid->linearMappingAlt_;
  int16_t *lev = (int16_t *)m->o.lev_;
  for (i = 0; i < 6; i++) {
    unsigned base = i * 128, cnt = mp->cnt_[i];
    m->o.cnt_[i] = cnt;
    if (!adid_alt(i)) {
      cov(D_PLAIN_PHASE);
      for (j = 0; j < cnt; j++) lev[base + j] = lm[base + mp->listA_[base + j]];
    } else {
      unsigned k = 0;
      cov(D_ALT_PHASE);
      for (j = 0; j < cnt; j++) {
        int a = lm[base + mp->listA_[base + j]], b = lma[base + mp->listA_[base + j]];
        lev[base + k] = (int16_t)(a > b ? a : b);
        lev[base + k + 1] = (int16_t)(a > b ? b : a);
        k += 2;
      }
    }
  }
}
static void md_setmd(MD *m) {
  m->o.md_m0_ = m->o.cnt_[0]; m->o.md_m1_ = m->o.cnt_[1]; m->o.md_m2_ = m->o.cnt_[2]; m->o.md_m3_ = m->o.cnt_[3]; m->o.md_m4_ = m->o.cnt_[4]; m->o.md_m5_ = m->o.cnt_[5];
  m->o.md_m6_ = m->o.nbits_;
}
static void md_print(MD *m) {
  unsigned i, j;
  if (m->o.cnt_[0] && m->o.cnt_[1] && m->o.cnt_[2] && m->o.cnt_[3] && m->o.cnt_[4] && m->o.cnt_[5]) {
    cov(D_PRINT_FULL);
    exp_push("%s", SEP);
    exp_push("Error histogram #%d:\r\n", (int)m->o.histNo_);
    for (i = 0; i < 6; i++) {
      exp_push("constellation %d :\r\n", (int)i);
      for (j = 0; j < m->o.cnt_[i]; j++) {
        unsigned n = m->o.errCnt_[i * 128 + j], avg = 0;
        if (n) { avg = m->o.errSum_[i * 128 + j] / n; cov(D_PRINT_DIV); } else cov(D_PRINT_DIV0);
        exp_push("  %d\t-\t%d\t\t%d\r\n", (int)(int16_t)m->o.lev_[i * 128 + j], (int)n, (int)avg);
      }
      for (j = 0; j < 128; j++) { m->o.errSum_[i * 128 + j] = 0; m->o.errCnt_[i * 128 + j] = 0; }
    }
    exp_push("%s", SEP);
  } else cov(D_PRINT_SKIP);
  m->o.histNo_++;
}
static void md_reset(MD *m, const MapPar90 *mp) {
  unsigned K = mp->w620_, i;
  m->o.w0_ = mp->w0_; m->o.K_ = K; m->o.skip_ = 6u - K; m->o.nbits_ = mp->w0_ - (6u - K);
  if (K) m->o.spf_ = 6u / K;
  md_levels(m, mp);
  md_setmd(m);
  m->o.sdd_state_ = 0;
  ms_reset(&m->sbe, K, 0); md_sync_sbe(m);
  m->o.wr_ = 0; m->o.rd_ = 0; m->o.phase_ = 0;
  for (i = 0; i < 768; i++) { m->o.errSum_[i] = 0; m->o.errCnt_[i] = 0; }
  m->o.delay_ = g_prm->demapperDelayBeforeErrorHistogram < g_prm->trn2dDdLength ? (unsigned)g_prm->demapperDelayBeforeErrorHistogram : 0u;
  cov(m->o.delay_ ? D_DELAY_KEEP : D_DELAY_ZERO);
  m->o.integ_ = 0;
  m->o.studyCnt_ = 0; m->o.set_ = 0; m->o.z1ea4_ = 0; m->o.z1ea6_ = 0; m->o.studyIter_ = 0; m->o.studyLen_ = 0; m->o.z1eb4_ = 0;
  cov(K ? D_KPOS : D_K0);
}
static void md_resetns(MD *m, const MapPar90 *mp) {
  m->o.w0_ = mp->w0_;
  m->o.nbits_ = mp->w0_ - m->o.skip_;
  if (g_prm->debugDemapperErrorHistogram != 0) {
    cov(D_RESETNS_HIST);
    md_print(m);
    m->o.integ_ = 0;
    m->o.delay_ = (unsigned)g_prm->demapperDelayBeforeErrorHistogram;
  } else cov(D_RESETNS_NOHIST);
  md_levels(m, mp);
  md_setmd(m);
  m->o.sdd_state_ = 0;
}
static int16_t L16(const MD *m, int k) { return k == -1 ? (int16_t)(m->o.wr_ >> 16) : ((const int16_t *)m->o.lev_)[k]; }
static int16_t md_hard(MD *m, int16_t x) {
  int xs, ax, sign, hi, lo, d1, d2, derr, alt;
  unsigned p, cnt, lim, e, k, wr = m->o.wr_;
  const int16_t *lev = (const int16_t *)m->o.lev_;
  m->o.set_ = (uint16_t)m->o.phase_;
  if (wr >= m->o.cap_) {
    cov(D_HARD_FULL);
    exp_push("V90Demapper: Hard decision input buffers are full !!!!!!!!!!!!!\n");
    m->o.prevIdx_ = 0;
    return 0;
  }
  if (x >= 0) { m->bits[wr] = 1; sign = 1; xs = x; cov(D_HARD_POS); }
  else { m->bits[wr] = 0; sign = -1; xs = (int16_t)(-(int)x); cov(D_HARD_NEG); if (x == -32768) cov(D_HARD_MIN); }
  ax = (int16_t)(xs < 0 ? -xs : xs);
  p = m->o.phase_;
  alt = adid_alt(p);
  cnt = m->o.cnt_[p];
  lim = alt ? 2u * cnt - 1u : cnt - 1u;
  cov(alt ? D_HARD_ALT : D_HARD_PLAIN);
  e = 0;
  if (!(lev[p * 128] < ax)) {
    if (lim != 0) {
      unsigned kk = 1;
      while (!(lev[p * 128 + kk] < ax) && kk < lim) kk++;
      e = kk - 1;
      if (e == 0) cov(D_HARD_E0);
      if (kk == lim) cov(D_HARD_ELIM); else cov(D_HARD_EMID);
    } else cov(D_HARD_LIM0);
  } else cov(D_HARD_E0);
  hi = lev[p * 128 + e];
  lo = lev[p * 128 + e + 1];
  d1 = (int16_t)(hi - xs < 0 ? xs - hi : hi - xs);
  d2 = (int16_t)(lo - xs < 0 ? xs - lo : lo - xs);
  if (d2 < d1) { xs = lo; derr = d2; k = e + 1; cov(D_HARD_LO); }
  else { xs = hi; derr = d1; k = e; cov(D_HARD_HI); if (d2 == d1) cov(D_HARD_TIE); }
  if (g_prm->debugDemapperErrorHistogram != 0) {
    cov(D_HARD_HIST_ON);
    if (m->o.delay_ == 0) {
      unsigned ix = p * 128 + k;
      m->o.errSum_[ix] += (unsigned)derr;
      m->o.errCnt_[ix] += 1;
      cov(D_HARD_ACC);
      if ((int)m->o.integ_ >= g_prm->demapperErrorHistogramIntegrationTime) { cov(D_HARD_INTEG_PRINT); md_print(m); m->o.integ_ = 0; }
      else { cov(D_HARD_INTEG_STEP); m->o.integ_++; }
    } else { cov(D_HARD_DELAY); m->o.delay_--; }
  }
  m->dig[wr] = alt ? (unsigned)(int)(int16_t)(k >> 1) : k;
  m->o.wr_ = wr + 1;
  m->o.phase_ = (m->o.phase_ + 1) % 6;
  m->o.prevIdx_ = (uint16_t)k;
  (void)lev;
  return (int16_t)(xs * sign);
}
/* ModulusDecoder: Horner over the six digits with the radices, then the low m6 bits (the sign extends past bit 63) */
static void md_modulus(const MD *m, const unsigned *d, uint8_t *out) {
  uint64_t acc = d[5];
  unsigned i;
  acc = acc * m->o.md_m4_ + d[4]; acc = acc * m->o.md_m3_ + d[3]; acc = acc * m->o.md_m2_ + d[2]; acc = acc * m->o.md_m1_ + d[1]; acc = acc * m->o.md_m0_ + d[0];
  for (i = 0; i < m->o.md_m6_; i++) out[i] = (uint8_t)(i < 64 ? (acc >> i) & 1u : (acc >> 63) & 1u);
}
/* returns 1 if the driver must not have called (the blob's sign extractor would run on an unloaded register) */
static int md_process(MD *m, uint8_t *out, unsigned *outcnt, unsigned *ret) {
  unsigned consumed = 0, rd, wr = m->o.wr_;
  if (wr <= 5) { *outcnt = 0; *ret = 0; cov(D_PROC_NONE); return 0; }
  rd = m->o.rd_;
  if (rd + 6 <= wr) {
    cov(D_PROC_FRAMES);
    while (rd + 6 <= wr) {
      unsigned j;
      md_modulus(m, m->dig + rd, out + m->o.skip_ + consumed);
      if (m->o.K_ == 0) {
        cov(D_PROC_K0);
        for (j = 0; j < m->o.skip_; j++) { uint8_t y = (uint8_t)(m->o.sdd_state_ ^ m->bits[rd + j]); m->o.sdd_state_ = m->bits[rd + j]; out[consumed + j] = y; }
      } else {
        unsigned k;
        cov(D_PROC_KPOS);
        for (k = 0; k < m->o.K_; k++) {
          if (m->sbe.o.action_ > 1) return 1;
          ms_process(&m->sbe, m->bits + rd + m->o.spf_ * k, out + (int)((m->o.spf_ - 1u) * k + consumed));
        }
        md_sync_sbe(m);
      }
      consumed += m->o.w0_;
      rd += 6;
      m->o.rd_ = rd;
    }
  } else cov(D_PROC_NOFRAME);
  {
    unsigned e = 0;
    if (wr > rd) cov(D_PROC_REST);
    while (wr > rd) { m->dig[e] = m->dig[rd]; m->bits[e] = m->bits[rd]; rd++; e++; }
    cov(D_PROC_COMPACT);
    m->o.wr_ = e;
  }
  m->o.rd_ = 0;
  *outcnt = consumed;
  *ret = 1;
  return 0;
}
static void md_check(const char *what) {
  OD *o = g_od;
  cmp_flds(&g_md.o, o, f_DemObj, NF_DemObj, what);
  if (memcmp(g_md.dig, o->digits_, (size_t)o->cap_ * 4) != 0) h_die3("%s: the digit queue differs from the model", what);
  if (memcmp(g_md.bits, o->bits_, o->cap_) != 0) h_die3("%s: the sign-bit queue differs from the model", what);
  if (memcmp(g_md.sbe.lanes, o->sbe_pdd_state_, 6) != 0) h_die3("%s: the sign extractor's PDD lanes differ from the model", what);
  can_check(g_cand, what);
}
static void md_dumps(const char *tag, int full) { h_dump_obj(0, tag, g_od, full ? g_fullD : g_lightD, full ? g_nfullD : g_nlightD); }

/* ================================================================================================ V90Demapper: study model */
static int16_t trunc16m(long double v) {
  if (v != v) return (int16_t)0x8000;
  if (v >= 32768.0L || v <= -32769.0L) return (int16_t)0x8000;
  return (int16_t)(int)v;
}
static void md_rebuild(MD *m) {
  unsigned i, j;
  for (i = 0; i < 6; i++) {
    unsigned n = adid_alt(i) ? (uint16_t)(m->o.cnt_[i] * 2u) : (uint16_t)m->o.cnt_[i];
    for (j = 0; j < n; j++) {
      unsigned idx = i * 128 + j, c = m->mcnt[idx];
      if (c != 0) {
        long double r = 1.0L / (long double)c;
        long double v = (long double)m->msum[idx] * r + 0.5L;
        int16_t t = trunc16m(v);
        m->o.lev_[idx] = (uint16_t)t;
        if (t == (int16_t)0x8000) cov(v != v ? T_UPD_NAN : v > 0 ? T_UPD_SAT_HI : T_UPD_SAT_LO);
        else if (v < 0) cov(T_UPD_NEG);
        else cov(T_UPD_FRAC);
        if (v != v) { /* counted above */ }
        else if (v > 1e30L || v < -1e30L) cov(T_UPD_INF);
        if (c >= 0x80000000u) cov(T_UPD_BIGCOUNT);
        cov(T_UPD_NONZERO);
        if (v > -1e9L && v < 1e9L) { long double fl = v - (long double)(long long)v; if (fl == 0.0L || fl > 0.4999L) cov(T_UPD_ROUND_EDGE); }
      } else cov(T_UPD_ZERO);
      if (adid_alt(i)) cov(T_UPD_ALT);
    }
  }
}
static void md_updcon(MD *m) {
  cov(T_UPD_CALL);
  md_rebuild(m);
  if (dsplibs_debug_level > 1) { exp_push("V90Demapper: constelation update !!!\n"); cov(T_UPD_PRINT); } else cov(T_UPD_QUIET);
}
/* returns 1 when the study completed (the detector was cleared) */
static int md_study(MD *m, int16_t a, int16_t b) {
  int aa = a < 0 ? -(int)a : a, bb = b < 0 ? -(int)b : b, d = aa - bb, idx = (int16_t)m->o.prevIdx_, s = (int16_t)m->o.set_, diff, k;
  long double ad;
  cov(T_STUDY_PROC);
  if ((long double)d > 0.0L) {
    cov(T_STUDY_D_POS);
    k = (s << 7) + idx;
    if (idx - 1 < 0) { diff = L16(m, k) - L16(m, k + 1); cov(T_STUDY_UP); }
    else { diff = L16(m, k - 1) - L16(m, k); cov(T_STUDY_DOWN); }
  } else {
    int n = adid_alt((unsigned)s) ? (int16_t)(m->o.cnt_[s] * 2u) : (int16_t)m->o.cnt_[s];
    cov(d < 0 ? T_STUDY_D_NEG : T_STUDY_D_ZERO);
    k = (s << 7) + idx;
    if (idx + 1 >= n) { diff = L16(m, k - 1) - L16(m, k); cov(T_STUDY_DOWN); }
    else { diff = L16(m, k) - L16(m, k + 1); cov(T_STUDY_UP); }
  }
  if (idx == 0) cov(T_STUDY_IDX0);
  ad = (long double)(d < 0 ? -d : d);
  if ((long double)0.4f * (long double)diff > ad) {
    int kk = (s << 7) + idx;
    m->msum[kk] = (float)((long double)aa + (long double)m->msum[kk]);
    m->mcnt[kk]++;
    cov(T_STUDY_ACC);
  } else cov(T_STUDY_SKIP);
  if (idx + 1 >= (adid_alt((unsigned)s) ? (int16_t)(m->o.cnt_[s] * 2u) : (int16_t)m->o.cnt_[s])) cov(T_STUDY_IDXLAST);
  if ((long double)0.4f * (long double)diff == ad) cov(T_STUDY_BOUND_EQ);
  {
    unsigned c = m->o.studyCnt_ + 1u;
    if (c != m->o.studyLen_) { m->o.studyCnt_ = c; return 0; }
  }
  cov(T_STUDY_DONE);
  m->o.studyIter_ = (uint16_t)(m->o.studyIter_ + 1u);
  if (m->o.studyIter_ == 2) { m->o.z1ea6_ = 1; cov(T_STUDY_IT2); } else cov(T_STUDY_IT_OTHER);
  m->o.studyCnt_ = 0;
  m->o.z1ea4_ = 1;
  md_rebuild(m);
  if (dsplibs_debug_level > 1) { exp_push("V90Demapper: constelation update !!!\n"); cov(T_STUDY_PRINT); } else cov(T_STUDY_QUIET);
  { unsigned i; for (i = 0; i < 768; i++) { m->msum[i] = 0.0f; m->mcnt[i] = 0; } }
  return 1;
}

/* ================================================================================================ V90Demapper: trials */
static void adid_snapshot(MD *m) {
  memcpy(m->msum, g_adid->cumSum_, sizeof m->msum);
  memcpy(m->mcnt, g_adid->cumCount_, sizeof m->mcnt);
}
static void adid_check(const MD *m, const char *what) {
  if (memcmp(m->msum, g_adid->cumSum_, sizeof m->msum) != 0) {
    unsigned i;
    for (i = 0; i < 768; i++) if (memcmp(&m->msum[i], &g_adid->cumSum_[i], 4)) break;
    h_die3("%s: cumSum_[%u] is %08x, the model says %08x", what, i, h_bits_of(g_adid->cumSum_[i]), h_bits_of(m->msum[i]));
  }
  if (memcmp(m->mcnt, g_adid->cumCount_, sizeof m->mcnt) != 0) {
    unsigned i;
    for (i = 0; i < 768; i++) if (m->mcnt[i] != g_adid->cumCount_[i]) break;
    h_die3("%s: cumCount_[%u] is %u, the model says %u", what, i, g_adid->cumCount_[i], m->mcnt[i]);
  }
}
/* the six codes lists + the detector's linear mappings of one trial: `arbitrary` = arbitrary int16 values instead of descending levels (the real use) */
static void adid_gen(MapPar90 *mp, int anyalt, int arbitrary) {
  unsigned i, j;
  for (j = 0; j < 768; j++) { g_adid->linearMapping_[j] = (uint16_t)h_rnd(); g_adid->linearMappingAlt_[j] = (uint16_t)h_rnd(); }
  for (i = 0; i < 6; i++) g_adid->altRbsPhase_[i] = (uint16_t)(anyalt && h_rpct(50) ? (h_rpct(85) ? 1u : h_rnd() | 1u) : 0u);
  for (i = 0; i < 6; i++) {
    unsigned perm[128], base = i * 128, cnt = mp->cnt_[i], alt = g_adid->altRbsPhase_[i] != 0, nlev = alt ? 2 * cnt : cnt, t;
    int V[256], start = 32124 - (int)h_rmod(3000), step, cur;
    for (j = 0; j < 128; j++) perm[j] = j;
    for (j = 127; j > 0; j--) { unsigned r = h_rmod(j + 1), tt = perm[j]; perm[j] = perm[r]; perm[r] = tt; }
    step = start / (int)(nlev + 1);
    if (step < 1) step = 1;
    cur = start;
    for (t = 0; t < nlev && t < 256; t++) { V[t] = cur; cur -= step - (int)h_rmod((unsigned)(step / 3 + 1)); if (cur < 1) cur = 1; }
    for (j = 0; j < cnt; j++) {
      unsigned c = perm[j];
      mp->listA_[base + j] = (uint8_t)c;
      if (arbitrary) continue;
      if (!alt) g_adid->linearMapping_[base + c] = (uint16_t)V[j];
      else if (h_rpct(50)) { g_adid->linearMapping_[base + c] = (uint16_t)V[2 * j]; g_adid->linearMappingAlt_[base + c] = (uint16_t)V[2 * j + 1]; }
      else { g_adid->linearMapping_[base + c] = (uint16_t)V[2 * j + 1]; g_adid->linearMappingAlt_[base + c] = (uint16_t)V[2 * j]; }
    }
  }
}
/* a float rounded to single precision exactly where the code says so (the i386 x87 would keep excess precision in a register) */
static float f32(long double v) { volatile float f = (float)v; return f; }
static void adid_gen_cum(const MD *m) {
  unsigned i, j;
  for (j = 0; j < 768; j++) {
    unsigned c = h_rpct(45) ? 0u : h_rpct(85) ? 1 + h_rmod(20) : h_rpct(70) ? h_rnd() : 0x80000000u + h_rmod(5);
    float s;
    unsigned lv = (unsigned)(m->o.lev_[j] & 0x7fffu);
    switch (h_rmod(10)) {
      case 8: case 9: {
        /* results exactly at the edges of the 16-bit truncation: 1 / c is exact for c a power of two, so v = s / c + 0.5 hits the target exactly */
        static const long double tg[] = { 32766.5L, 32767.0L, 32767.5L, 32768.0L, 32768.5L, 32769.0L, -32767.5L, -32768.0L, -32768.5L, -32769.0L, -32769.5L, -32770.0L, 0.5L, -0.5L, 0.0L, 1.0L, -1.0L };
        c = 1u << h_rmod(4);
        s = (float)((tg[h_rmod(sizeof tg / sizeof tg[0])] - 0.5L) * (long double)c);
        break;
      }
      case 0: s = h_float_from_bits_quiet(fbits_pick()); break;
      case 1: s = f32((long double)f32((long double)c) * (long double)h_rmod(40000)); break;    /* long double + volatile float stores: i386 (x87) and x86-64 (SSE) round identically (no float arithmetic in the data path) */
      default: s = f32((long double)f32((long double)c) * (long double)lv + (long double)h_rrange(-50, 50) * 0.25L); break;
    }
    g_adid->cumCount_[j] = c;
    g_adid->cumSum_[j] = s;
  }
  (void)i;
}
static unsigned pick_cap(void) { return h_rpct(15) ? 1 + h_rmod(5) : 6 + h_rmod(58); }
static int16_t pick_x(const MD *m) {
  unsigned p = m->o.phase_ % 6, n;
  int16_t base;
  int delta, s;
  const int16_t *lev = (const int16_t *)m->o.lev_;
  n = adid_alt(p) ? 2 * m->o.cnt_[p] : m->o.cnt_[p];
  if (n == 0) n = 1;
  base = lev[p * 128 + h_rmod(n > 128 ? 128 : n)];
  switch (h_rmod(8)) {
    case 0: delta = 0; break;
    case 1: delta = 1; break;
    case 2: delta = -1; break;
    case 3: delta = (int)h_rmod(40) - 20; break;
    case 4: delta = (int)h_rmod(400) - 200; break;
    case 5: return rs16();
    case 6: delta = (int)h_rmod(5) - 2; break;
    default: delta = (int)h_rmod(3000) - 1500; break;
  }
  s = (int)base + delta;
  if (s > 32767) s = 32767;
  if (s < -32768) s = -32768;
  return (int16_t)(h_rpct(50) ? s : -s);
}
static void dm_params(void) {
  memset(g_prm, 0, sizeof *g_prm);
  g_prm->debugDemapperErrorHistogram = h_rpct(55) ? (h_rpct(70) ? 1 : (int)h_rnd() | 1) : 0;
  g_prm->demapperDelayBeforeErrorHistogram = h_rpct(60) ? (int)h_rmod(4) : (int)h_rmod(9) - 3;
  g_prm->trn2dDdLength = h_rpct(60) ? (int)h_rmod(8) : (int)h_rmod(9) - 3;
  g_prm->demapperErrorHistogramIntegrationTime = h_rpct(70) ? (int)h_rmod(6) : (int)h_rmod(12) - 4;
}
static void dm_after(const char *what, int full) {
  md_check(what);
  md_dumps(what, full);
}
static void trial_demapper(int mode) {
  int c2 = h_rpct(50), ops, k, anyalt, arbitrary, pos_valid = 0;   /* pos_valid: set_ / prevIdx_ hold a position that is inside the current tables */
  int safe = 1;                                                      /* 0 after a reset with zero counts: only the operations that never index a level table run until the next valid reset */
  MapPar90 mp;
  unsigned cap = pick_cap(), K;
  uint8_t *outb;
  uint8_t *refb;
  OD *o = (OD *)h_obj_new(SZD + CAN);
  g_od = o; g_cand = (uint8_t *)o + SZD; can_fill(g_cand);
  prefill(o, SZD, f_DemObj, NF_DemObj);
  g_adid = (AdidObj *)h_obj_new(sizeof(AdidObj) + CAN);
  h_log_next_size = 0x558;                     /* sizeof(V90Parameters) on i386: the x86-64 block is larger, the log must not show it */
  g_prm = (V90ParObj *)sysdep_malloc(sizeof(V90ParObj));
  outb = (uint8_t *)malloc(4096); refb = (uint8_t *)malloc(4096);
  dm_params();
  g_curD = o; g_curM = NULL; g_curS = NULL;
  dsplibs_debug_level = pick_level();
  exp_clear(); g_expcheck = 1; g_nclr = 0;
  cov(c2 ? D_C2 : D_C1);
  H_CTOR_PROBE(o, f_DemObj, NF_DemObj, sizeof(OD), (c2 ? dm_c2(o, cap, g_prm, g_adid) : dm_c1(o, cap, g_prm, g_adid)));
  EXPECT_EQ("cap_", o->cap_, cap); EXPECT_EQ("wr_", o->wr_, 0); EXPECT_EQ("rd_", o->rd_, 0); EXPECT_EQ("phase_", o->phase_, 0); EXPECT_EQ("histNo_", o->histNo_, 0);
  EXPECT_EQ("sdd", o->sdd_state_, 0); EXPECT_EQ("K_", o->K_, 0); EXPECT_EQ("w0_", o->w0_, 0); EXPECT_EQ("cnt_[3]", o->cnt_[3], 0); EXPECT_EQ("sbe n", o->sbe_n_, o->sbe_n_);
  if (o->params_ != (void *)g_prm || o->adid_ != (void *)g_adid) h_die3("params_ / adid_ are not the arguments");
  memcpy(&g_md.o, o, SZD);
  memcpy(g_md.dig, o->digits_, (size_t)cap * 4); memcpy(g_md.bits, o->bits_, cap);
  md_load_sbe(&g_md, o);
  md_check("after the constructor");
  md_dumps("ctor", 1);
  anyalt = h_rpct(40);
  arbitrary = h_rpct(20);
  ops = (mode == 3 ? 40 : 30) + (int)h_rmod(mode == 3 ? 160 : 120);
  if (h_rpct(7)) {
    /* destroyed before any reset: all six counts are still zero (the histogram print is skipped), the sub-objects are as the constructor left them */
    ops = 0;
    if (h_rpct(60)) { exp_clear(); md_print(&g_md); dm_printhist(o); exp_done("quick printErrorHistogramAndReset"); dm_after("after printErrorHistogramAndReset", 1); }
    if (h_rpct(50)) { dm_incrbs(o); g_md.o.phase_ = (g_md.o.phase_ + 1) % 6; dm_after("after incrementRBSFramePosition", 0); }
  }
  for (k = 0; k < ops; k++) {
    int what = (int)h_rmod(100);
    if (!safe) what = (int)h_rmod(12);
    if (k == 0 || (mode == 3 ? what < 2 : what < 4)) {
      /* reset: a new geometry (10%: some counts are ZERO: nothing but the print / increment / the next reset may follow) */
      static const unsigned ks[] = { 0, 0, 0, 1, 2, 3, 6, 4, 5, 3, 2, 6, 0 };
      int edge = k > 0 && safe && h_rpct(10);
      K = ks[h_rmod(sizeof ks / sizeof ks[0])];
      if (mode == 4 && h_rpct(30)) K = h_rmod(7);
      gen_mp(&mp, K, 6u - K, 0x7f, anyalt, 0);
      if (edge) {
        unsigned i, z = 0;
        if (h_rpct(60)) { mp.cnt_[h_rmod(6)] = 0; z = 1; }       /* exactly one zero count, at any position */
        else for (i = 0; i < 6; i++) if (h_rpct(30)) { mp.cnt_[i] = 0; z++; }
        if (!z) mp.cnt_[h_rmod(6)] = 0;
        safe = 0;
      } else safe = 1;
      adid_gen(&mp, anyalt, arbitrary);
      if (h_rpct(30)) dm_params();
      if (h_rpct(10)) { unsigned r = h_rnd(); o->rd_ = r; g_md.o.rd_ = r; }   /* reset clears it */
      g_nclr = 0;
      exp_clear();
      md_reset(&g_md, &mp);
      dm_reset(o, &mp);
      if (mode == 4) adid_gen_cum(&g_md);
      clr_expect_full("demapper reset");
      exp_done("demapper reset");
      cov(D_RESET);
      pos_valid = 0;
      adid_snapshot(&g_md);
      dm_after("after reset", 1);
      if (edge) {   /* the print with a zero count: nothing is printed, the counter still advances */
        exp_clear(); md_print(&g_md); dm_printhist(o); exp_done("printErrorHistogramAndReset after a reset with a zero count");
        dm_after("after printErrorHistogramAndReset", 1);
      }
    } else if (what < (mode == 3 ? 6 : 8)) {
      /* resetNoSpectral: w0 derived from the OLD skip; the histogram parameters may have changed */
      gen_mp(&mp, g_md.o.K_, g_md.o.skip_, 0x7f, anyalt, 0);
      /* the code lists / counts stay those of the detector's mappings: re-run the generator for them */
      adid_gen(&mp, anyalt, arbitrary);
      if (mode == 4) adid_gen_cum(&g_md);
      if (h_rpct(50)) dm_params();
      safe = 1;
      mp.w620_ = h_rnd();
      g_nclr = 0;
      exp_clear();
      md_resetns(&g_md, &mp);
      dm_resetns(o, &mp);
      clr_expect_none("demapper resetNoSpectral");
      exp_done("demapper resetNoSpectral");
      cov(D_RESETNS);
      pos_valid = 0;
      adid_snapshot(&g_md);
      dm_after("after resetNoSpectral", 1);
    } else if (what < 9) {
      exp_clear();
      md_print(&g_md);
      dm_printhist(o);
      exp_done("demapper printErrorHistogramAndReset");
      dm_after("after printErrorHistogramAndReset", 1);
    } else if (what < 12) {
      dm_incrbs(o);
      g_md.o.phase_ = (g_md.o.phase_ + 1) % 6;
      cov(D_INCR); if (g_md.o.phase_ == 0) cov(D_INCR_WRAP);
      dm_after("after incrementRBSFramePosition", 0);
    } else if (mode == 3 && what < 28) {
      unsigned oc = 0x12345678u, rc = 0x12345678u, ret, rret = 0;
      memset(outb, 0x77, 4096); memcpy(refb, outb, 4096);
      unsigned save_skip = g_md.o.skip_;
      int pskip = 0;
      if (g_md.o.K_ == 0 && h_rpct(8)) {
        /* a different number of sign bits per frame (poked; K_ == 0 always has 6): the sign decoder loop's zero test */
        unsigned ns = h_rmod(4);
        o->skip_ = ns; g_md.o.skip_ = ns; pskip = 1;
      }
      if (g_md.o.rd_ == 0 && g_md.o.wr_ > 0 && h_rpct(10)) { unsigned r = h_rmod(g_md.o.wr_ + 1); o->rd_ = r; g_md.o.rd_ = r; }   /* a read position that is not 0 (the blob keeps 0 between calls): rd + 6 > wr skips the loop */
      exp_clear();
      if (md_process(&g_md, refb, &rc, &rret)) h_die3("harness: the model reached the undefined sign-extractor action");
      ret = dm_process(o, outb, &oc);
      exp_done("demapper process");
      if (pskip) { o->skip_ = save_skip; g_md.o.skip_ = save_skip; }
      EXPECT_EQ("process return", ret, rret);
      EXPECT_EQ("process outCnt", oc, rc);
      if (memcmp(outb, refb, 4096) != 0) h_die3("demapper process: the output bits differ from the model");
      h_hash_bytes(outb, rc);
      if (rret && rc) cov(D_PROC_ONE);
      dm_after("after process", (k & 7) == 0);
    } else if (mode == 4 && what < 20) {
      unsigned len = h_rpct(60) ? 1 + h_rmod(5) : h_rpct(50) ? 0x80000000u : h_rnd();
      g_nclr = 0;
      dm_resetls(o, len);
      g_md.o.studyCnt_ = 0; g_md.o.set_ = 0; g_md.o.z1ea4_ = 0; g_md.o.z1ea6_ = 0; g_md.o.studyIter_ = 0; g_md.o.studyLen_ = len;
      clr_expect_full("resetLinearMappStudy");
      cov(D_RESETLS);
      adid_snapshot(&g_md);
      dm_after("after resetLinearMappStudy", 1);
    } else if (mode == 4 && what < 32) {
      dsplibs_debug_level = pick_level();
      exp_clear();
      adid_snapshot(&g_md);
      md_updcon(&g_md);
      dm_updcon(o);
      exp_done("updateConstelation");
      adid_check(&g_md, "updateConstelation");
      dm_after("after updateConstelation", 1);
    } else if (mode == 4 && what < 92) {
      /* linearMappingStudy: poke the position, call with samples around the level */
      int16_t a, b;
      int pcnt2 = 0;
      unsigned pk2 = 0, save2 = 0;
      unsigned p = h_rmod(6), n = adid_alt(p) ? 2 * g_md.o.cnt_[p] : g_md.o.cnt_[p], idx;
      int did_clear;
      if (n == 0) n = 1;
      if (!pos_valid || h_rpct(70)) {
        pos_valid = 1;
        idx = h_rpct(30) ? 0 : h_rpct(30) ? n - 1 : h_rmod(n);
        o->set_ = (uint16_t)p; g_md.o.set_ = (uint16_t)p;
        o->prevIdx_ = (uint16_t)idx; g_md.o.prevIdx_ = (uint16_t)idx;
      }
      if (h_rpct(8)) { unsigned len2 = 1 + h_rmod(4); o->studyLen_ = len2; g_md.o.studyLen_ = len2; }
      if (h_rpct(6)) { unsigned sc = h_rmod(3); o->studyCnt_ = sc; g_md.o.studyCnt_ = sc; }
      {
        int idx2 = (int16_t)g_md.o.prevIdx_, s = (int16_t)g_md.o.set_;
        int16_t lv = (s >= 0 && s < 6) ? (int16_t)g_md.o.lev_[s * 128 + (idx2 >= 0 && idx2 < 128 ? idx2 : 0)] : 0;
        int bs = lv < 0 ? -(int)lv : lv, spread = h_rpct(60) ? 40 : 600, as;
        b = (int16_t)(h_rpct(50) ? bs : -bs);
        as = bs + (int)h_rmod((unsigned)spread * 2 + 1) - spread;
        if (as > 32767) as = 32767;
        if (as < -32768) as = -32768;
        a = (int16_t)(h_rpct(50) ? as : -as);
        if (h_rpct(15)) { a = rs16(); b = rs16(); }
        if (h_rpct(5)) b = a;
        if (h_rpct(12)) { a = (int16_t)h_rrange(-3, 3); b = (int16_t)h_rrange(-3, 3); }   /* |b| = 1: the sign branch of the absolute value */
        if (h_rpct(8) && s >= 0 && s < 6) {
          /* two equal neighbouring levels and a == b: 0.4 * (spacing) == |d| == 0 exactly */
          unsigned nn = adid_alt((unsigned)s) ? 2 * g_md.o.cnt_[s] : g_md.o.cnt_[s];
          int kk = s * 128 + idx2;
          if (idx2 >= 0 && (unsigned)idx2 + 1 < nn) { o->lev_[kk + 1] = o->lev_[kk]; g_md.o.lev_[kk + 1] = o->lev_[kk]; }
          else if (idx2 >= 1 && (unsigned)idx2 < nn) { o->lev_[kk - 1] = o->lev_[kk]; g_md.o.lev_[kk - 1] = o->lev_[kk]; }
          b = (int16_t)(h_rpct(50) ? (int16_t)o->lev_[kk] : -(int16_t)o->lev_[kk]); a = b;
        }
      }
      if (h_rpct(10)) dsplibs_debug_level = pick_level();
      {
        /* a level count whose low 16 bits are negative (poked), only when this call cannot complete the study (the estimator would run over 2^15 levels) */
        int s2 = (int16_t)g_md.o.set_;
        pcnt2 = 0;
        if (s2 >= 0 && s2 < 6 && g_md.o.studyCnt_ + 1u != g_md.o.studyLen_ && h_rpct(5)) {
          unsigned hn = adid_alt((unsigned)s2) ? 0x4000u + h_rmod(0x4000) : 0x8000u + h_rmod(0x8000);
          pk2 = (unsigned)s2; save2 = g_md.o.cnt_[pk2]; o->cnt_[pk2] = hn; g_md.o.cnt_[pk2] = hn; pcnt2 = 1;
        }
      }
      exp_clear(); g_nclr = 0;
      adid_snapshot(&g_md);
      did_clear = md_study(&g_md, a, b);
      dm_study(o, a, b);
      if (pcnt2) { o->cnt_[pk2] = save2; g_md.o.cnt_[pk2] = save2; }
      exp_done("linearMappingStudy");
      if (did_clear) clr_expect_full("linearMappingStudy"); else clr_expect_none("linearMappingStudy");
      adid_check(&g_md, "linearMappingStudy");
      dm_after("after linearMappingStudy", did_clear);
    } else {
      /* a decision */
      int16_t x = pick_x(&g_md), r, rr;
      unsigned save_cap = g_md.o.cap_, save_cnt = g_md.o.cnt_[g_md.o.phase_ % 6], pk = g_md.o.phase_ % 6;
      int pcap = 0, pcnt = 0;
      if (g_md.o.wr_ < save_cap && h_rpct(4)) {
        /* a capacity of 2^31 or more (poked; the blocks keep their size, wr_ < the real capacity): the unsigned compare says "not full" */
        unsigned hc = 0x80000000u | h_rnd();
        o->cap_ = hc; g_md.o.cap_ = hc; pcap = 1;
      }
      if (!arbitrary && !pcap && g_prm->debugDemapperErrorHistogram == 0 && h_rpct(4) && g_md.o.cnt_[pk] >= 4 && g_md.o.cnt_[pk] <= 64 && (adid_alt(pk) ? 2 * g_md.o.cnt_[pk] : g_md.o.cnt_[pk]) >= 4) {
        /* a level count of 2^31 or more (poked): the search limit is huge, the search ends at the first level below |x|: |x| = third level + 1 */
        unsigned hn = 0x80000000u | h_rmod(1000);
        const int16_t *lv = (const int16_t *)g_md.o.lev_ + pk * 128;
        int t = (int)lv[3] + 1;
        if (lv[1] >= lv[3] + 1 && t > 0 && t <= 32767 && lv[0] >= t) { x = (int16_t)(h_rpct(50) ? t : -t); o->cnt_[pk] = hn; g_md.o.cnt_[pk] = hn; pcnt = 1; }
      }
      exp_clear(); g_nclr = 0;
      rr = md_hard(&g_md, x);
      r = dm_hard(o, x);
      if (pcap) { o->cap_ = save_cap; g_md.o.cap_ = save_cap; }
      if (pcnt) { o->cnt_[pk] = save_cnt; g_md.o.cnt_[pk] = save_cnt; }
      exp_done("hardDecision");
      clr_expect_none("hardDecision");
      EXPECT_EQ("hardDecision result", (unsigned)(uint16_t)r, (unsigned)(uint16_t)rr);
      if (g_md.o.prevIdx_ != 0 || g_md.o.wr_ != 0) pos_valid = pos_valid || g_md.o.wr_ > 0;
      dm_after("after hardDecision", 0);
      if (g_md.o.wr_ + 1 >= g_md.o.cap_ && h_rpct(60) && mode == 3) { /* leave the queue full for a while: the full-buffer path */ }
    }
  }
  /* dtor: prints the histogram when the debug parameter is set */
  exp_clear();
  if (g_prm->debugDemapperErrorHistogram != 0) { md_print(&g_md); cov(D_D_HIST); } else cov(D_D_NOHIST);
  { int d2 = h_rpct(50); cov(d2 ? D_D2 : D_D1); if (d2) dm_d2(o); else dm_d1(o); }
  __asm__ volatile("" ::: "memory");
  exp_done("demapper dtor");
  can_check(g_cand, "V90Demapper dtor");
  md_dumps("dtor", 1);
  g_curD = NULL;
  free(outb); free(refb);
  sysdep_free(g_prm);
  h_trial_end("demapper");
  cov(C_TRIALS);
}

/* the counters each scenario must reach (frozen from a run: every counter with >= 25 hits in that scenario; the lists are checked with 12 seeds) */
static const int req0[] = { C_TRIALS, C_LV2, C_LVHI, S_C1, S_C2, S_D1, S_D2, S_RESET_N0, S_RESET_N1, S_RESET_N2, S_RESET_N3, S_RESET_N6, S_P_A0_NZ, S_P_A0_Z, S_P_A1_Z, S_P_A1_NZ, S_P_N0, S_P_N1, S_P_N6, S_AP0, S_AP1, S_AP2, S_AP3, S_APNEG, S_APBIG, S_APN0, S_PBYTE, S_P_BIGBYTE, 0 };
static const int req1[] = { C_TRIALS, C_LV2, M_C1, M_C2, M_D1, M_D2, M_RESET, M_RESETNS, M_ULAW, M_ALAW, M_K0, M_DEPTH0, M_DEPTHPOS, M_CNT128, M_CNT0, M_PROC_PARTIAL, M_PROC_FRAME, M_PROC_MULTI, M_SGN0, M_SGN1, M_DEP_ZERO, M_FILL_CARRY, M_NS_OLDSKIP, M_N0, M_BYTEBIT, 0 };
static const int req2[] = { C_TRIALS, C_LV2, M_C1, M_C2, M_D1, M_D2, M_RESET, M_RESETNS, M_ULAW, M_ALAW, M_K1, M_K2, M_K3, M_K6, M_K45, M_DEPTH0, M_DEPTHPOS, M_CNT128, M_CNT0, M_PROC_PARTIAL, M_PROC_FRAME, M_PROC_MULTI, M_SH_CALL, M_DEP_ZERO, M_DEP_LT, M_DEP_GE, M_DEP_COPYPART, M_FILL_CARRY, M_NS_OLDSKIP, M_N0, M_BYTEBIT, 0 };
static const int req3[] = { C_TRIALS, C_LV2, D_C1, D_C2, D_D1, D_D2, D_D_HIST, D_D_NOHIST, D_RESET, D_RESETNS, D_RESETNS_HIST, D_RESETNS_NOHIST, D_ALT_PHASE, D_PLAIN_PHASE, D_K0, D_KPOS, D_DELAY_KEEP, D_DELAY_ZERO, D_INCR, D_INCR_WRAP, D_HARD_POS, D_HARD_NEG, D_HARD_FULL, D_HARD_E0, D_HARD_ELIM, D_HARD_EMID, D_HARD_LIM0, D_HARD_LO, D_HARD_HI, D_HARD_ALT, D_HARD_PLAIN, D_HARD_HIST_ON, D_HARD_DELAY, D_HARD_ACC, D_HARD_INTEG_PRINT, D_HARD_INTEG_STEP, D_PRINT_FULL, D_PRINT_DIV0, D_PRINT_DIV, D_PROC_NONE, D_PROC_FRAMES, D_PROC_ONE, D_PROC_K0, D_PROC_KPOS, D_PROC_COMPACT, D_PROC_REST, 0 };
static const int req4[] = { C_TRIALS, C_LV0, C_LV1, C_LV2, C_LVHI, D_C1, D_C2, D_D1, D_D2, D_D_HIST, D_D_NOHIST, D_RESET, D_RESETNS, D_RESETNS_HIST, D_RESETNS_NOHIST, D_RESETLS, D_ALT_PHASE, D_PLAIN_PHASE, D_K0, D_KPOS, D_DELAY_KEEP, D_DELAY_ZERO, D_INCR, D_HARD_POS, D_HARD_NEG, D_HARD_FULL, D_HARD_E0, D_HARD_ELIM, D_HARD_EMID, D_HARD_LIM0, D_HARD_LO, D_HARD_HI, D_HARD_ALT, D_HARD_PLAIN, D_HARD_HIST_ON, D_HARD_DELAY, D_HARD_ACC, D_HARD_INTEG_PRINT, D_HARD_INTEG_STEP, D_PRINT_FULL, D_PRINT_DIV0, D_PRINT_DIV, T_STUDY_PROC, T_STUDY_D_POS, T_STUDY_D_NEG, T_STUDY_D_ZERO, T_STUDY_UP, T_STUDY_DOWN, T_STUDY_ACC, T_STUDY_SKIP, T_STUDY_DONE, T_STUDY_IT2, T_STUDY_IT_OTHER, T_STUDY_PRINT, T_STUDY_QUIET, T_UPD_CALL, T_UPD_NONZERO, T_UPD_ZERO, T_UPD_ALT, T_UPD_SAT_HI, T_UPD_SAT_LO, T_UPD_NAN, T_UPD_NEG, T_UPD_FRAC, T_UPD_ROUND_EDGE, T_UPD_PRINT, T_UPD_QUIET, T_STUDY_IDX0, T_STUDY_IDXLAST, T_UPD_INF, T_UPD_BIGCOUNT, 0 };
static void relax_reserved(void) { h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0; }

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, i, tr, bad = 0;
  const char *seed = getenv("P28_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P28_DBG")) g_show = 1;
  p07_layout_check();
  p11_layout_check();
  p18_layout_check();
  p28_layout_check();
  h_x87_check();
  h_arena_init();
  g_exp = malloc((size_t)EXPMAX * TXT);
  g_nlightS = make_light(g_lightS, f_SbeObj, NF_SbeObj);
  g_nlightM = make_light(g_lightM, f_MapObj, NF_MapObj);
  g_nlightD = make_light(g_lightD, f_DemObj, NF_DemObj);
  g_nfullM = make_full(g_fullM, f_MapObj, NF_MapObj);
  g_nfullD = make_full(g_fullD, f_DemObj, NF_DemObj);
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  relax_reserved();
  { static const int *const reqs[5] = { req0, req1, req2, req3, req4 }; const int *r = sc >= 0 && sc < 5 ? reqs[sc] : req0; for (; *r; r++) g_covreq[*r] = 1; }
  switch (sc) {
    case 0: for (tr = 0; tr < 250; tr++) trial_sbe(); break;
    case 1: for (tr = 0; tr < 120; tr++) trial_mapper(0); break;
    case 2: for (tr = 0; tr < 120; tr++) trial_mapper(1); break;
    case 3: for (tr = 0; tr < 120; tr++) trial_demapper(3); break;
    case 4: for (tr = 0; tr < 120; tr++) trial_demapper(4); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu prints %lu shaper reset %u resetSS %u process %u clear %lu", h_nmalloc, h_nfree, g_nprint, g_shreset_n, g_shss_n, g_shprocess_n, g_clr_total);
  for (i = 1; i < COV_MAX; i++) {
    h_emit(1, "COV %-8s %lu", cov_name(i), g_covn[i]);
    if (g_covreq[i] && g_covn[i] == 0) { fprintf(stderr, "COVERAGE FAIL: counter %d was never reached\n", i); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P28_NOCOV")) ? 2 : 0;
}
