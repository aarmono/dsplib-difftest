/*
 * Differential test harness for port chunk P06 (src/vpcm/pcm_leaf_classes.cpp): the small V.90 / V.92 leaves of dsplibs.o - the G.711 functions, the K56Flex stubs,
 * the parser stubs, print_echo_coeffs (sub-chunk A), V90Phase2Info / V92Phase2Info / V90ConstellationPower / ModulusEncoder / ModulusDecoder / V92Mapper (B)
 * and V90SdDetector / V90RDetector / V90SpectralShapingFilter (C): 85 functions.
 *
 * ONE C source, no #ifdef for the code under test, built by test/vpcm/pcm_leaf_classes.sh against the blob (a copy of dsplibs.o in which `edprintf` is WEAK so that
 * the recorder below wins), against the port (-O0 / -O2) and for x86-64 (diffed against the 32-bit BLOB's output = the golden, test/golden/p06_blob32*.out).  The
 * functions are reached through __asm__ labels with the mangled / plain names (identical in blob and port, C1 AND C2 / D1 AND D2), the object layouts are C
 * mirror structs GENERATED from test/vpcm/pcm_leaf_classes.layout (REAL pointer members: they widen on x86-64 like the port's classes).
 *
 * Objects are built by the constructors under test (never by hand) on pool memory.  What is compared: every return value, every object (all fields field-wise,
 * floats as bit patterns, heap blocks), the malloc / free sequence, the constructor store order (malloc-time snapshot), the red zones, every `edprintf` /
 * `dsplibs_debug_printf` call (format text and arguments; recorders).  INDEPENDENT MODELS (written from the disassembly facts / the ITU definitions, not from
 * the port) are checked in every scenario that has one; a model violation exits 3.
 *
 * Scenarios (each its own process): 0 G.711 (all 65,536 / 256 inputs, the 32-bit extremes, garbage in the upper argument bits), 1 stubs (K56Flex, parser,
 * K56FLEX_Create / Delete) + print_echo_coeffs.   [B and C: see the scenario table in main()]
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../harness/harness.h"
#include "pcm_leaf_classes_layout.h"

#define L(m) __asm__(m)
typedef long double LD;
typedef uint64_t u64_;

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

/* ---- (A) free functions ---- */
extern int l2a(int) L("linear2alaw");
extern int l2u(int) L("linear2ulaw");
extern int a2l(int) L("alaw2linear");
extern int u2l(int) L("ulaw2linear");
extern int a2u(int) L("alaw2ulaw");
extern int u2a(int) L("ulaw2alaw");
extern unsigned char tab_a2u[128] L("_a2u");
extern unsigned char tab_u2a[128] L("_u2a");
extern void *k56_create(void) L("K56FLEX_Create");
extern void k56_delete(void *) L("K56FLEX_Delete");
extern int k56_term(void) L("K56FLEX_SessionTermination");
extern int vp_int(void *, const char *, int *) L("Vparser_read_int");
extern int vp_flt(void *, const char *, float *) L("Vparser_read_float");
extern void print_echo(float *, unsigned) L("_Z17print_echo_coeffsPfj");
/* ---- K56FlexFloModem ---- */
extern void kf_c1(void *, void *, int, void *) L("_ZN15K56FlexFloModemC1EPviP19_tagModemParameters");
extern void kf_c2(void *, void *, int, void *) L("_ZN15K56FlexFloModemC2EPviP19_tagModemParameters");
extern void kf_d1(void *) L("_ZN15K56FlexFloModemD1Ev");
extern void kf_d2(void *) L("_ZN15K56FlexFloModemD2Ev");
extern void kf_ext(void *) L("_ZN15K56FlexFloModem13externalResetEv");
extern void kf_int(void *) L("_ZN15K56FlexFloModem13internalResetEv");
extern void kf_ep3(void *) L("_ZN15K56FlexFloModem18k56FlexEnterPhase3Ev");
extern void kf_epfd(void *) L("_ZN15K56FlexFloModem21enterPhase3FullDuplexEv");
extern void kf_mm(void *, int, int) L("_ZN15K56FlexFloModem14setMinMaxRatesEii");
extern int kf_run(void *, float *, unsigned, int *, int *) L("_ZN15K56FlexFloModem21k56FlexRunDemodulatorEPfjPiS1_");
extern int kf_gc(void *, void *, unsigned long) L("_ZN15K56FlexFloModem16getConstellationEP11int_complexm");
extern int kf_gle(void *, void *, unsigned long) L("_ZN15K56FlexFloModem18getLinearEqualizerEP11int_complexm");
extern int kf_gdfe(void *, void *, unsigned long) L("_ZN15K56FlexFloModem6getDFEEP11int_complexm");
extern int kf_gde(void *, void *, unsigned long) L("_ZN15K56FlexFloModem17getDecisionErrorsEP11int_complexm");
extern int kf_grp(void *, void *, unsigned long) L("_ZN15K56FlexFloModem17getResamplerPhaseEP11int_complexm");
extern int kf_gro(void *, void *, unsigned long) L("_ZN15K56FlexFloModem18getResamplerOffsetEP11int_complexm");
extern int kf_gja(void *, short *) L("_ZN15K56FlexFloModem16getK56FlexJaBitsEPs");
extern int kf_gmp(void *, short *) L("_ZN15K56FlexFloModem16getK56FlexMpBitsEPs");
extern int kf_gmps(void *) L("_ZN15K56FlexFloModem17getK56MPsReceiverEv");
extern unsigned int dsplibs_debug_level;




/* ---- (B) V92Mapper / ModulusEncoder / ModulusDecoder / V90Phase2Info / V92Phase2Info / V90ConstellationPower ---- */
extern void vm_c1(void *) L("_ZN9V92MapperC1Ev");
extern void vm_c2(void *) L("_ZN9V92MapperC2Ev");
extern void vm_d1(void *) L("_ZN9V92MapperD1Ev");
extern void vm_d2(void *) L("_ZN9V92MapperD2Ev");
extern void vm_reset(void *, int, int) L("_ZN9V92Mapper5resetEsh");              /* short / uchar arguments passed as int words with garbage in the upper bits */
extern int16_t vm_proc(void *, unsigned char *) L("_ZN9V92Mapper7processEPh");
extern int vm_tab[16] L("_ZN9V92Mapper21constelAmplitudeTableE");
extern void me_c1(void *) L("_ZN14ModulusEncoderC1Ev");
extern void me_c2(void *) L("_ZN14ModulusEncoderC2Ev");
extern void me_c1n(void *, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned) L("_ZN14ModulusEncoderC1Ejjjjjjj");
extern void me_c2n(void *, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned) L("_ZN14ModulusEncoderC2Ejjjjjjj");
extern void me_prog(void *, unsigned char *, unsigned *) L("_ZN14ModulusEncoder8progressEPhPj");
extern void md_c1(void *) L("_ZN14ModulusDecoderC1Ev");
extern void md_c2(void *) L("_ZN14ModulusDecoderC2Ev");
extern void md_c1n(void *, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned) L("_ZN14ModulusDecoderC1Ejjjjjjj");
extern void md_c2n(void *, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned) L("_ZN14ModulusDecoderC2Ejjjjjjj");
extern void md_prog(void *, unsigned char *, unsigned *) L("_ZN14ModulusDecoder8progressEPhPj");
extern void p2v_c1(void *, void *) L("_ZN13V90Phase2InfoC1EP13V90Parameters");
extern void p2v_c2(void *, void *) L("_ZN13V90Phase2InfoC2EP13V90Parameters");
extern void p2v_def(void *) L("_ZN13V90Phase2Info12setToDefaultEv");
extern void p2v_print(const void *) L("_ZNK13V90Phase2Info9printInfoEv");
extern void p2w_c1(void *, void *) L("_ZN13V92Phase2InfoC1EP13V92Parameters");
extern void p2w_c2(void *, void *) L("_ZN13V92Phase2InfoC2EP13V92Parameters");
extern void p2w_def(void *) L("_ZN13V92Phase2Info12setToDefaultEv");
extern void p2w_print(const void *) L("_ZNK13V92Phase2Info9printInfoEv");
extern void cp_c1(void *) L("_ZN21V90ConstellationPowerC1Ev");
extern void cp_c2(void *) L("_ZN21V90ConstellationPowerC2Ev");
extern void cp_d1(void *) L("_ZN21V90ConstellationPowerD1Ev");
extern void cp_d2(void *) L("_ZN21V90ConstellationPowerD2Ev");
extern void cp_calc(void *, void *) L("_ZN21V90ConstellationPower21calcModulusParametersEP16V90MappingParams");
extern void cp_info(void *, void *, int, unsigned) L("_ZN21V90ConstellationPower20getConstellationInfoEP16V90MappingParams26V90TxPowerMeasurementPointj");
extern float cp_power(void *, void *, int, int) L("_ZN21V90ConstellationPower8getPowerEP16V90MappingParams26V90TxPowerMeasurementPoint7PcmType");
extern unsigned cp_pidx(void *, farg_t) L("_ZN21V90ConstellationPower21getPowerIndexForPowerEf");
extern unsigned char cp_legal(void *, unsigned) L("_ZN21V90ConstellationPower17isLegalPowerIndexEj");
extern unsigned cp_limits[35] L("_ZN21V90ConstellationPower18averagePowerLimitsE");

/* ---- (C) V90SdDetector / V90RDetector / V90SpectralShapingFilter ---- */
extern void sd_c1(void *, farg_t, farg_t, farg_t, unsigned) L("_ZN13V90SdDetectorC1Efffj");
extern void sd_c2(void *, farg_t, farg_t, farg_t, unsigned) L("_ZN13V90SdDetectorC2Efffj");
extern void sd_d1(void *) L("_ZN13V90SdDetectorD1Ev");
extern void sd_d2(void *) L("_ZN13V90SdDetectorD2Ev");
extern void sd_reset(void *) L("_ZN13V90SdDetector5resetEv");
extern int sd_proc(void *, farg_t) L("_ZN13V90SdDetector7processEf");
extern void rd_c1(void *, void *) L("_ZN12V90RDetectorC1EP13V90Parameters");
extern void rd_c2(void *, void *) L("_ZN12V90RDetectorC2EP13V90Parameters");
extern void rd_d1(void *) L("_ZN12V90RDetectorD1Ev");
extern void rd_d2(void *) L("_ZN12V90RDetectorD2Ev");
extern void rd_reset(void *, unsigned, unsigned) L("_ZN12V90RDetector5resetEjj");
extern int rd_r(void *, int) L("_ZN12V90RDetector7detectREs");
extern int rd_rn(void *, int) L("_ZN12V90RDetector10detectRNotEs");
extern int rd_rf(void *, int) L("_ZN12V90RDetector8detectRfEs");
extern int rd_rfn(void *, int) L("_ZN12V90RDetector11detectRfNotEs");
extern void sf_c1(void *) L("_ZN24V90SpectralShapingFilterC1Ev");
extern void sf_c2(void *) L("_ZN24V90SpectralShapingFilterC2Ev");
extern void sf_reset(void *) L("_ZN24V90SpectralShapingFilter5resetEv");
extern void sf_set(void *, farg_t, farg_t, farg_t, farg_t) L("_ZN24V90SpectralShapingFilter14setFilterCoeffEffff");
extern XRET sf_metric(const void *, const short *, unsigned) L("_ZNK24V90SpectralShapingFilter9getMetricEPKsj");
extern void sf_prog(void *, const short *) L("_ZN24V90SpectralShapingFilter8progressEPKs");

/* ------------------------------------------------------------------------------------------------ recorders (edprintf / dsplibs_debug_printf) */
static unsigned g_nedp, g_ndbg;
static char g_last_edp[300];
static char g_ed_text[64][300];   /* the texts of the edprintf calls of the current call window (g_ed_in_call counts them) */
static unsigned g_ed_in_call;
static char g_last_dbg_fmt[200];
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
  if (g_ed_in_call < 64) strcpy(g_ed_text[g_ed_in_call], g_last_edp);
  g_ed_in_call++;
  h_emit(g_ed_show, "edprintf %s", g_last_edp);
}
/* dsplibs_debug_printf: the formats in P06 take (const char *) or up to four ints; the recorder reads the four int slots (the callee may pass fewer: the
 * remaining slots are whatever the caller's frame holds, so only the number of arguments the FORMAT names is recorded) */
typedef void (*dbg_hook_t)(unsigned idx);
static dbg_hook_t g_dbg_hook;   /* called after each recorded dbgprintf: lets a scenario change dsplibs_debug_level between the prints of one call */
static char g_dbg_text[64][300];/* the formatted text of the last prints of the current call */
static unsigned g_dbg_in_call;
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[400];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  g_last_dbg_fmt[0] = 0;
  esc_cat(g_last_dbg_fmt, sizeof g_last_dbg_fmt, buf);
  if (g_dbg_in_call < 64) strcpy(g_dbg_text[g_dbg_in_call], g_last_dbg_fmt);
  g_dbg_in_call++;
  g_ndbg++;
  h_emit(g_dbg_show, "dbgprintf %s", g_last_dbg_fmt);
  if (g_dbg_hook) g_dbg_hook(g_dbg_in_call - 1);
  return 0;
}

/* pool memory with a CANONICAL fill (the library's random fill is one stream over all objects, so a class whose sizeof differs between i386 and x86-64 shifts the fill of every later object) */
static void *onew(size_t n) { void *p = h_obj_new(n); memset(p, 0xa5, n); return p; }

/* ------------------------------------------------------------------------------------------------ bit helpers */
static uint32_t bits_f(float f) { return h_bits_of(f); }
static float f_of(uint32_t b) { float f; memcpy(&f, &b, 4); return f; } /* only fed to arithmetic when finite; never returned by value for signalling NaNs */
static uint64_t bits_d(double d) { uint64_t b; memcpy(&b, &d, 8); return b; }
static double d_of(uint64_t b) { double d; memcpy(&d, &b, 8); return d; }
static int is_nan_w(uint32_t b) { return (b & 0x7f800000u) == 0x7f800000u && (b & 0x007fffffu) != 0; }
static int is_snan_w(uint32_t b) { return is_nan_w(b) && !(b & 0x00400000u); }
static int is_fin_w(uint32_t b) { return (b & 0x7f800000u) != 0x7f800000u; }
static uint64_t fnv_blk(const void *p, size_t n) {
  const uint8_t *q = (const uint8_t *)p;
  uint64_t h = 0xcbf29ce484222325ull;
  size_t i;
  for (i = 0; i < n; i++) { h ^= q[i]; h *= 0x100000001b3ull; }
  return h;
}
#define EXPECT_EQ(what, got, want)                                                                                                   \
  do {                                                                                                                              \
    unsigned long long g_ = (unsigned long long)(got), w_ = (unsigned long long)(want);                                              \
    if (g_ != w_) h_die3("%s: got 0x%llx expected 0x%llx (line %d)", (what), g_, w_, __LINE__);                                     \
  } while (0)

/* ------------------------------------------------------------------------------------------------ coverage ids */
enum {
  /* A: G.711 */
  CV_L2A_NEG = 0, CV_L2A_POS, CV_L2A_NF_POS, CV_L2A_NF_NEG, CV_L2A_SEG0, /* .. SEG0 + 7 = 11 */
  CV_L2U_NEG = 12, CV_L2U_POS, CV_L2U_NF_POS, CV_L2U_NF_NEG, CV_L2U_SEG0, /* .. + 7 = 23 */
  CV_A2L_SEG0 = 24, CV_A2L_SEG1, CV_A2L_SEGHI, CV_A2L_NEG, CV_A2L_POS,
  CV_U2L_SEG0 = 29, CV_U2L_SEGHI, CV_U2L_NEG, CV_U2L_POS,
  CV_A2U_POS = 33, CV_A2U_NEG, CV_U2A_POS, CV_U2A_NEG, CV_GARBAGE_HI, CV_INT_MIN, CV_INT_MAX, CV_L2U_WRAP_NEG, CV_L2U_WRAP_POS, CV_L2A_NEG_SMALL
};
/* counter ids are per scenario (each scenario is its own process); ids 60 / 61 are reserved by the harness library */
enum {
  /* A: stubs */
  CV_K56_METHODS = 0, CV_K56_RUN, CV_K56_CREATE, CV_K56_DELETE_NULL, CV_K56_DELETE, CV_K56_TERM, CV_VP_INT, CV_VP_FLT,
  CV_PE_EMPTY, CV_PE_POS, CV_PE_NEG, CV_PE_ZERO, CV_PE_NAN, CV_PE_INF, CV_PE_BIG, CV_PE_FRAC, CV_PE_NEGZERO, CV_PE_SMALL, CV_PE_INTMIN, CV_PE_N_MANY
};
static const char *g_covname[H_COV_MAX];
static void cov_def(int id, const char *name) { h_cov_define(id, name); g_covname[id] = name; }
/* the two reserved counters (2^31 straddle of the arena blocks, dtor with a NULL block) are not required by scenarios without an owning heap block */
static void relax_reserved(void) { h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0; }




/* B: counter ids per scenario */
enum { CV_VM_C2 = 0, CV_VM_RESET0, CV_VM_RESET1, CV_VM_RESETN, CV_VM_NB2, CV_VM_NB3, CV_VM_NB0, CV_VM_NB4, CV_VM_WRAP16, CV_VM_BYTE, CV_VM_NEGSCALE, CV_VM_CLIP, CV_VM_ZEROENTRY, CV_VM_FLAG1TAB,
       CV_VM_POWPOKE, CV_VM_NEGENTRY, CV_VM_GARBAGE, CV_VM_INRANGE, CV_VM_NAN };
enum { CV_MC_CTOR0 = 0, CV_MC_CTOR7, CV_MC_C2, CV_MC_ENC, CV_MC_DEC, CV_MC_ROUNDTRIP, CV_MC_M6_0, CV_MC_M6_64, CV_MC_RADIX1, CV_MC_BIGRADIX, CV_MC_NEGACC, CV_MC_M6_HUGE, CV_MC_M5, CV_MC_D5BIG, CV_MC_SAR };
enum { CV_P2_C2 = 0, CV_P2_DEF, CV_P2_PRINT, CV_P2_LVL_FULL, CV_P2_LVL_OFF, CV_P2_LVL_CUT, CV_P2_BIGLEVEL, CV_P2_PCM1, CV_P2_PCM0, CV_P2_TX1, CV_P2_TX0, CV_P2_L2SPECIAL, CV_P2_L2NEG, CV_P2_L2FRAC,
       CV_P2_WORD100, CV_P2_BYTETRUNC, CV_P2_MAXTX_ODD, CV_P2_LEVELPLAN_LATE, CV_P2_POKE };
enum { CV_CP_CTOR = 0, CV_CP_C2, CV_CP_CALC, CV_CP_SHIFT_WRAP, CV_CP_MNEG, CV_CP_R5, CV_CP_D2BIG, CV_CP_INFO, CV_CP_INFO_OOR, CV_CP_INFO_HI, CV_CP_POWER, CV_CP_P_ALAW, CV_CP_P_ULAW, CV_CP_P_HI, CV_CP_P_GT,
       CV_CP_P_EQ, CV_CP_P_LT, CV_CP_PIDX, CV_CP_PIDX_TOP, CV_CP_PIDX_ZERO, CV_CP_PIDX_MID, CV_CP_PIDX_NAN, CV_CP_LEGAL, CV_CP_LEGAL_NO, CV_CP_LEGAL_EDGE, CV_CP_RADIX_BIG, CV_CP_PIDX_EQ, CV_CP_PCM_ODD,
       CV_CP_POINT_ODD, CV_CP_RADIX1 };


/* C: counter ids per scenario */
enum { CV_SD_C2 = 0, CV_SD_CTOR, CV_SD_PATH0, CV_SD_PATH1, CV_SD_PATH2, CV_SD_PATH3A, CV_SD_PATH3B, CV_SD_EQ_A, CV_SD_EQ_B, CV_SD_EQ_C, CV_SD_COUNT_EQ, CV_SD_COUNT_GT, CV_SD_COUNT_LT, CV_SD_RESET, CV_SD_NAN,
       CV_SD_ZEROA, CV_SD_NSMALL, CV_SD_NULLDTOR, CV_SD_SNAN, CV_SD_INF, CV_SD_DENORM, CV_SD_REQ0, CV_SD_REQBIG, CV_SD_PROBE };
enum { CV_RD_CTOR = 0, CV_RD_C2, CV_RD_RESET, CV_RD_GARBAGE, CV_RD_PRE_RESET, CV_RD_SR_HIGH, CV_RD_DIR_NEG, CV_RD_DIR_ZERO, CV_RD_DIR_BIG, CV_RD_LIM_EQ, CV_RD_XNEG, CV_RD_XZERO, CV_RD_XMIN, CV_RD_XMAX, CV_RD_RET1,
       CV_RD_RETM1, CV_RD_CNTPOKE, CV_RD_LIMPOKE };
enum { CV_SF_CTOR = 0, CV_SF_C2, CV_SF_RESET, CV_SF_SET, CV_SF_PROG, CV_SF_METRIC, CV_SF_N0, CV_SF_N1, CV_SF_N2, CV_SF_NBIG, CV_SF_NM0, CV_SF_NM1, CV_SF_NMBIG, CV_SF_EXTDIFF, CV_SF_NONFIN, CV_SF_BIGIN, CV_SF_MININ,
       CV_SF_ZERO, CV_SF_CONST, CV_SF_DEFAULTS, CV_SF_SPECIAL, CV_SF_SNAN, CV_SF_STATE_POKE };

/* ================================================================================================ (A) scenario 0: G.711 */
/* independent models: the segment search by arithmetic (no table), floor division for the mantissa, the decoders from the closed forms
 *   A-law   magnitude = seg == 0 ? (2 m + 1) << 3 : (2 m + 33) << (seg + 2)               (after the 0x55 inversion; bit 7 set = positive)
 *   mu-law  magnitude = ((2 m + 33) << (seg + 2)) - 132                                    (after the complement; bit 7 set = NEGATIVE) */
static int64_t floordiv(int64_t a, int64_t b) { int64_t q = a / b; if ((a % b != 0) && ((a < 0) != (b < 0))) q--; return q; }
static int model_seg(int32_t v) { int s = 0; while (s < 8 && (int64_t)v > ((int64_t)256 << s) - 1) s++; return s; }
static int model_l2a(int32_t x) {
  int32_t v = x;
  unsigned mask = 0xd5;
  int seg, m;
  if (x < 0) { mask = 0x55; v = (int32_t)((uint32_t)0 - (uint32_t)x - 8u); }
  seg = model_seg(v);
  if (seg == 8) return (int)(mask ^ 0x7f);
  m = (int)(((floordiv(v, (int64_t)1 << (seg <= 1 ? 4 : seg + 3)) % 16) + 16) % 16);
  return (int)((((unsigned)seg << 4 | (unsigned)m) ^ mask) & 0xff);
}
static int model_l2u(int32_t x) {
  int32_t v;
  unsigned mask = 0xff;
  int seg, m;
  if (x < 0) { mask = 0x7f; v = (int32_t)((uint32_t)0x84 - (uint32_t)x); } else v = (int32_t)((uint32_t)x + 0x84u);
  seg = model_seg(v);
  if (seg == 8) return (int)(mask ^ 0x7f);
  m = (int)(((floordiv(v, (int64_t)1 << (seg + 3)) % 16) + 16) % 16);
  return (int)((((unsigned)seg << 4 | (unsigned)m) ^ mask) & 0xff);
}
static int model_a2l(unsigned a) {
  unsigned b = (a & 0xff) ^ 0x55;
  unsigned seg = (b & 0x70) >> 4, m = b & 0xf;
  int mag = seg == 0 ? (int)((2 * m + 1) << 3) : (int)((2 * m + 33) << (seg + 2));
  return (b & 0x80) ? mag : -mag;
}
static int model_u2l(unsigned u) {
  unsigned b = ~u & 0xff;
  unsigned seg = (b & 0x70) >> 4, m = b & 0xf;
  int mag = (int)((2 * m + 33) << (seg + 2)) - 132;
  return (b & 0x80) ? -mag : mag;
}
static int a_step(unsigned a) { unsigned b = (a & 0xff) ^ 0x55; unsigned seg = (b & 0x70) >> 4; return seg <= 1 ? 16 : 16 << (seg - 1); }
static int u_step(unsigned u) { unsigned b = ~u & 0xff; unsigned seg = (b & 0x70) >> 4; return 8 << seg; }

static void sc_g711(void) {
  uint64_t h = 0xcbf29ce484222325ull;
  unsigned long ctr = 0;
  int64_t x;
  unsigned i;
  cov_def(CV_L2A_NEG, "l2a negative"); cov_def(CV_L2A_POS, "l2a positive"); cov_def(CV_L2A_NF_POS, "l2a segment search fails, positive"); cov_def(CV_L2A_NF_NEG, "l2a segment search fails, negative");
  for (i = 0; i < 8; i++) cov_def(CV_L2A_SEG0 + (int)i, "l2a segment found");
  cov_def(CV_L2U_NEG, "l2u negative"); cov_def(CV_L2U_POS, "l2u positive"); cov_def(CV_L2U_NF_POS, "l2u segment search fails, positive"); cov_def(CV_L2U_NF_NEG, "l2u segment search fails, negative");
  for (i = 0; i < 8; i++) cov_def(CV_L2U_SEG0 + (int)i, "l2u segment found");
  cov_def(CV_A2L_SEG0, "a2l segment 0"); cov_def(CV_A2L_SEG1, "a2l segment 1"); cov_def(CV_A2L_SEGHI, "a2l segment >= 2"); cov_def(CV_A2L_NEG, "a2l negative"); cov_def(CV_A2L_POS, "a2l positive");
  cov_def(CV_U2L_SEG0, "u2l segment 0"); cov_def(CV_U2L_SEGHI, "u2l segment >= 1"); cov_def(CV_U2L_NEG, "u2l negative"); cov_def(CV_U2L_POS, "u2l positive");
  cov_def(CV_A2U_POS, "a2u positive code"); cov_def(CV_A2U_NEG, "a2u negative code"); cov_def(CV_U2A_POS, "u2a positive code"); cov_def(CV_U2A_NEG, "u2a negative code");
  cov_def(CV_GARBAGE_HI, "garbage in the upper argument bits"); cov_def(CV_INT_MIN, "INT_MIN input"); cov_def(CV_INT_MAX, "INT_MAX input");
  cov_def(CV_L2U_WRAP_NEG, "l2u: 0x84 - x wraps"); cov_def(CV_L2U_WRAP_POS, "l2u: x + 0x84 wraps"); cov_def(CV_L2A_NEG_SMALL, "l2a: -x - 8 in -7 .. 0");
  h_scenario_begin(1);
  relax_reserved();
  /* the tables, by content (independent copies of the readelf-extracted bytes are NOT needed here: every entry is exercised through a2u / u2a and checked by
   * the property model below; blob == port makes the extraction exact) */
  /* encoders: all 16-bit values, then every value of +-2^21 around the segment boundaries, the 32-bit extremes, random 32-bit values */
  for (x = -70000; x <= 70000; x++) {
    int32_t v = (int32_t)x;
    int ra = l2a(v), ru = l2u(v);
    if (ra != model_l2a(v)) h_die3("linear2alaw(%d) = 0x%x, model 0x%x", v, ra, model_l2a(v));
    if (ru != model_l2u(v)) h_die3("linear2ulaw(%d) = 0x%x, model 0x%x", v, ru, model_l2u(v));
    h ^= (uint64_t)(unsigned)ra | ((uint64_t)(unsigned)ru << 8); h *= 0x100000001b3ull;
    if (v < 0) { h_cov_hit(CV_L2A_NEG); h_cov_hit(CV_L2U_NEG); if (-(int64_t)v - 8 >= -7 && -(int64_t)v - 8 <= 0) h_cov_hit(CV_L2A_NEG_SMALL); } else { h_cov_hit(CV_L2A_POS); h_cov_hit(CV_L2U_POS); }
    { int sg = model_seg(v < 0 ? (int32_t)(-(int64_t)v - 8) : v); if (sg < 8) h_cov_hit(CV_L2A_SEG0 + sg); else h_cov_hit(v < 0 ? CV_L2A_NF_NEG : CV_L2A_NF_POS); }
    { int32_t u = v < 0 ? (int32_t)((uint32_t)0x84 - (uint32_t)v) : (int32_t)((uint32_t)v + 0x84u); int sg = model_seg(u); if (sg < 8) h_cov_hit(CV_L2U_SEG0 + sg); else h_cov_hit(v < 0 ? CV_L2U_NF_NEG : CV_L2U_NF_POS); }
    if ((x & 0xfff) == 0) h_emit(0, "l2 %d %02x %02x", v, ra, ru);
  }
  h_emit(1, "encoders -70000..70000 hash %016llx", (unsigned long long)h);
  h = 0xcbf29ce484222325ull;
  for (i = 0; i < 4000000; i++) { /* 32-bit values: the powers of two and their neighbours, the extremes, uniform and log-uniform random */
    int32_t v;
    unsigned k = i % 8;
    if (i < 64 * 8) { int sh = (int)(i / 8); v = (int32_t)((int64_t)(k < 4 ? 1 : -1) * ((int64_t)1 << (sh % 32)) + (int)(k % 4) - 1); }
    else if (k < 3) v = (int32_t)h_rnd();
    else if (k < 6) v = (int32_t)(h_rnd() >> (h_rmod(32)));
    else v = (int32_t)(0 - (h_rnd() >> h_rmod(32)));
    switch (i) {
      case 3: v = (int32_t)0x80000000; break;
      case 4: v = 0x7fffffff; break;
      case 5: v = (int32_t)0x80000001; break;
      case 6: v = 0x7ffffffe; break;
      case 7: v = (int32_t)(0x80000000u + 0x84u); break;
      case 8: v = (int32_t)(0x7fffffffu - 0x83u); break;
      case 9: v = (int32_t)(0x7fffffffu - 0x84u); break;
      case 10: v = (int32_t)(0x7fffffffu - 0x82u); break;
      default: break;
    }
    {
      int ra = l2a(v), ru = l2u(v);
      if (ra != model_l2a(v)) h_die3("linear2alaw(%d) = 0x%x, model 0x%x", v, ra, model_l2a(v));
      if (ru != model_l2u(v)) h_die3("linear2ulaw(%d) = 0x%x, model 0x%x", v, ru, model_l2u(v));
      h ^= (uint64_t)(unsigned)ra | ((uint64_t)(unsigned)ru << 8); h ^= (uint64_t)(uint32_t)v << 16; h *= 0x100000001b3ull;
      if (v == (int32_t)0x80000000) h_cov_hit(CV_INT_MIN);
      if (v == 0x7fffffff) h_cov_hit(CV_INT_MAX);
      if (v < 0 && (int32_t)((uint32_t)0x84 - (uint32_t)v) < 0) h_cov_hit(CV_L2U_WRAP_NEG);
      if (v >= 0 && (int32_t)((uint32_t)v + 0x84u) < 0) h_cov_hit(CV_L2U_WRAP_POS);
      if (v < 0) { h_cov_hit(CV_L2A_NEG); h_cov_hit(CV_L2U_NEG); } else { h_cov_hit(CV_L2A_POS); h_cov_hit(CV_L2U_POS); }
      { int32_t va = v < 0 ? (int32_t)((uint32_t)0 - (uint32_t)v - 8u) : v; int sg = model_seg(va); if (sg < 8) h_cov_hit(CV_L2A_SEG0 + sg); else h_cov_hit(v < 0 ? CV_L2A_NF_NEG : CV_L2A_NF_POS); }
      { int32_t u = v < 0 ? (int32_t)((uint32_t)0x84 - (uint32_t)v) : (int32_t)((uint32_t)v + 0x84u); int sg = model_seg(u); if (sg < 8) h_cov_hit(CV_L2U_SEG0 + sg); else h_cov_hit(v < 0 ? CV_L2U_NF_NEG : CV_L2U_NF_POS); }
      if (h_sample(&ctr, 0, 400000)) h_emit(0, "l2x %d %02x %02x", v, ra, ru);
    }
  }
  h_emit(1, "encoders 32-bit hash %016llx", (unsigned long long)h);
  /* decoders and the table converters: all 256 codes, each with 16 different garbage patterns in the upper 24 bits of the (int) argument */
  h = 0xcbf29ce484222325ull;
  for (i = 0; i < 256; i++) {
    unsigned g, r;
    int ref_a = model_a2l(i), ref_u = model_u2l(i);
    for (g = 0; g < 16; g++) {
      int arg = (int)(i | (g == 0 ? 0u : (h_rnd() & 0xffffff00u)));
      int ra = a2l(arg), ru = u2l(arg), rau = a2u(arg), rua = u2a(arg);
      if (ra != ref_a) h_die3("alaw2linear(0x%x) = %d, model %d", (unsigned)arg, ra, ref_a);
      if (ru != ref_u) h_die3("ulaw2linear(0x%x) = %d, model %d", (unsigned)arg, ru, ref_u);
      if (g) h_cov_hit(CV_GARBAGE_HI);
      if (g == 0) {
        h ^= (uint64_t)(unsigned)ra * 31u + (uint64_t)(unsigned)ru * 131u + ((uint64_t)rau << 40) + ((uint64_t)rua << 48) + i; h *= 0x100000001b3ull;
        h_emit(0, "dec %02x a2l %d u2l %d a2u %02x u2a %02x", i, ra, ru, rau, rua);
      } else {
        /* the same results whatever the upper bits */
        if (rau != a2u((int)i) || rua != u2a((int)i)) h_die3("a2u / u2a depend on the upper argument bits (0x%x)", (unsigned)arg);
      }
    }
    r = i;
    { unsigned b = (r & 0xff) ^ 0x55; unsigned seg = (b & 0x70) >> 4; h_cov_hit(seg == 0 ? CV_A2L_SEG0 : seg == 1 ? CV_A2L_SEG1 : CV_A2L_SEGHI); h_cov_hit(ref_a < 0 ? CV_A2L_NEG : CV_A2L_POS); }
    { unsigned b = ~r & 0xff; unsigned seg = (b & 0x70) >> 4; h_cov_hit(seg == 0 ? CV_U2L_SEG0 : CV_U2L_SEGHI); h_cov_hit(ref_u < 0 ? CV_U2L_NEG : CV_U2L_POS); }
    h_cov_hit((i & 0x80) ? CV_A2U_NEG : CV_A2U_POS);
    h_cov_hit((i & 0x80) ? CV_U2A_NEG : CV_U2A_POS);
  }
  /* property model of the converters (the tables are ITU data, no formula): the converted code decodes to the same SIGN and to a value within the larger of the
   * two quantisation steps of the source value (a2u / u2a: 256 codes each) */
  for (i = 0; i < 256; i++) {
    int la = model_a2l(i), lu = model_u2l(a2u((int)i)), d;
    int lu2 = model_u2l(i), la2 = model_a2l(u2a((int)i)), d2;
    int s1 = a_step(i) > u_step(a2u((int)i)) ? a_step(i) : u_step(a2u((int)i));
    int s2 = u_step(i) > a_step(u2a((int)i)) ? u_step(i) : a_step(u2a((int)i));
    d = la > lu ? la - lu : lu - la;
    d2 = lu2 > la2 ? lu2 - la2 : la2 - lu2;
    if (d > s1 + 8 || d2 > s2 + 8) h_die3("converter property: code %02x: alaw %d -> ulaw %d (step %d), ulaw %d -> alaw %d (step %d)", i, la, lu, s1, lu2, la2, s2);
    if ((la > 0 && lu < 0) || (la < 0 && lu > 0) || (lu2 > 0 && la2 < 0) || (lu2 < 0 && la2 > 0)) h_die3("converter property: sign of code %02x", i);
  }
  h_emit(1, "decoders / converters hash %016llx", (unsigned long long)h);
  /* the two byte tables themselves, as the harness' own copies of the readelf-extracted bytes' hash (the blob / port objects' table symbols are compared too) */
  h_emit(1, "tables a2u %016llx u2a %016llx", (unsigned long long)fnv_blk(tab_a2u, 128), (unsigned long long)fnv_blk(tab_u2a, 128));
  h_trial_end("g711");
}

/* ================================================================================================ (A) scenario 1: stubs + print_echo_coeffs */
/* the expected text of one print_echo_coeffs line, by EXACT integer arithmetic (independent of the x87 sequence): for |x| < 2^50 the value x = m 2^k is
 * decomposed into integer / fraction parts; out-of-range cases follow the blob's `integer indefinite` rule (INT_MIN) */
static void model_split(uint32_t bits, int64_t scale, int *sign, int *ip, int *fp) {
  int neg = (int)(bits >> 31);
  uint32_t ex = (bits >> 23) & 0xff, mant = bits & 0x7fffff;
  const int64_t I32MIN = -((int64_t)1 << 31), I32MAX = ((int64_t)1 << 31) - 1;
  int64_t M, ipart, t, q;
  int e;
  if (ex == 0xff) { /* inf / NaN: every conversion is the indefinite; sign: NaN '+', -inf '-', +inf '+' */
    *ip = (int)0x80000000u; *fp = (int)0x80000000u;
    *sign = (mant == 0 && neg) ? '-' : '+';
    return;
  }
  *sign = (bits == 0u || bits == 0x80000000u) ? '-' : (neg ? '-' : '+');   /* zero prints '-' (the compare is 0 < x) */
  /* value = (mant | hidden) * 2^(ex - 150) (ex == 0: denormal, no hidden bit, exponent -149) */
  M = (int64_t)(mant | (ex ? 0x800000u : 0u));
  e = ex ? (int)ex - 150 : -149;
  if (e > 30) { *ip = (int)0x80000000u; *fp = (int)0x80000000u; return; }   /* |x| >= 2^54: every digit is the indefinite (the fraction is far out of range) */
  if (e < -62) { *ip = 0; *fp = 0; return; }                                   /* |x| < 2^-38: all digits 0 */
  if (e >= 0) {
    int64_t num = M << e, x = neg ? -num : num, fnum;
    ipart = num;
    *ip = (ipart > I32MAX) ? (int)0x80000000u : (int)ipart;                  /* fabs; fistp: |x| = 2^31 does not fit */
    t = (x < I32MIN || x > I32MAX) ? I32MIN : x;                             /* fist of x: out of range = the indefinite */
    fnum = x - t;                                                            /* the exact fraction * 1 (an integer here) */
    if (fnum > 4294 || fnum < -4294) q = I32MIN - 1;                         /* |frac * 1e6| >= 2^32: out of range */
    else q = fnum * scale;
  } else {
    int64_t den = (int64_t)1 << (-e), fr;
    ipart = M >> (-e);
    *ip = (int)ipart;
    t = neg ? -ipart : ipart;
    fr = M - ipart * den;                                                    /* < den and < 2^24 */
    q = (neg ? -fr : fr) * scale / den;                                      /* truncation toward zero */
  }
  if (q < I32MIN || q > I32MAX) q = I32MIN;
  *fp = (int)(q < 0 && q != I32MIN ? -q : q);
}

static void pe_check(const uint32_t *w, unsigned n, const char *tag) {
  float *c = (float *)onew(n * sizeof(float) + 16);
  unsigned i;
  unsigned calls0 = g_nedp;
  for (i = 0; i < n; i++) memcpy(&c[i], &w[i], 4);
  g_ed_show = 1;
  print_echo(c, n);
  g_ed_show = 0;
  if (g_nedp != calls0 + 1 + n) h_die3("print_echo_coeffs(%s): %u edprintf calls for n = %u", tag, g_nedp - calls0, n);
  /* the recorder only keeps the LAST line: the per-line comparison with the model happens in pe_one (single coefficient calls); the n-coefficient calls
   * are compared blob-vs-port through the emitted lines */
}
static void pe_one(uint32_t w, const char *tag) {
  char exp[300] = "";
  char buf[200];
  int sign, ip, fp;
  float *c = (float *)onew(32);
  unsigned calls0 = g_nedp;
  memcpy(c, &w, 4);
  g_ed_show = 1;
  print_echo(c, 1);
  g_ed_show = 0;
  if (g_nedp != calls0 + 2) h_die3("print_echo_coeffs(%s %08x): %u edprintf calls", tag, w, g_nedp - calls0);
  model_split(w, 1000000, &sign, &ip, &fp);
  { uint32_t ex = (w >> 23) & 0xff;
    if (is_nan_w(w)) h_cov_hit(CV_PE_NAN); else if (ex == 0xff) h_cov_hit(CV_PE_INF);
    else if (w == 0u) h_cov_hit(CV_PE_ZERO); else if (w == 0x80000000u) h_cov_hit(CV_PE_NEGZERO);
    else {
      h_cov_hit((w >> 31) ? CV_PE_NEG : CV_PE_POS);
      if (ex >= 158) h_cov_hit(CV_PE_BIG);
      if (ex < 107) h_cov_hit(CV_PE_SMALL);
      if (fp != 0 && ip != (int)0x80000000u) h_cov_hit(CV_PE_FRAC);
      if (w == 0xcf000000u) h_cov_hit(CV_PE_INTMIN);
    }
  }
  snprintf(buf, sizeof buf, "   = %c%d.%06d\r\n", sign, ip, fp);
  esc_cat(exp, sizeof exp, buf);
  if (strcmp(exp, g_last_edp)) h_die3("print_echo_coeffs(%s %08x): '%s', model '%s'", tag, w, g_last_edp, exp);
}

static void sc_stubs(void) {
  unsigned t;
  uint8_t *blk = (uint8_t *)onew(256);
  uint8_t snap[256];
  uint64_t h0;
  cov_def(CV_K56_METHODS, "K56Flex method calls"); cov_def(CV_K56_RUN, "k56FlexRunDemodulator"); cov_def(CV_K56_CREATE, "K56FLEX_Create"); cov_def(CV_K56_DELETE_NULL, "K56FLEX_Delete(NULL)");
  cov_def(CV_K56_DELETE, "K56FLEX_Delete(p)"); cov_def(CV_K56_TERM, "K56FLEX_SessionTermination"); cov_def(CV_VP_INT, "Vparser_read_int"); cov_def(CV_VP_FLT, "Vparser_read_float");
  cov_def(CV_PE_EMPTY, "print_echo n = 0"); cov_def(CV_PE_POS, "positive"); cov_def(CV_PE_NEG, "negative"); cov_def(CV_PE_ZERO, "+0"); cov_def(CV_PE_NAN, "NaN"); cov_def(CV_PE_INF, "inf");
  cov_def(CV_PE_BIG, "|x| >= 2^31"); cov_def(CV_PE_FRAC, "fraction"); cov_def(CV_PE_NEGZERO, "-0"); cov_def(CV_PE_SMALL, "|x| < 1e-6"); cov_def(CV_PE_INTMIN, "x = -2^31"); cov_def(CV_PE_N_MANY, "n >= 8");
  h_scenario_begin(1);
  relax_reserved();
  /* --- K56Flex: nothing is stored anywhere, the stubs return their constants, no allocation happens */
  memset(blk, 0xa5, 256);
  for (t = 0; t < 256; t++) blk[t] = (uint8_t)(h_rnd() | 1);
  memcpy(snap, blk, 256);
  h0 = fnv_blk(blk, 256);
  { unsigned long m0 = h_nmalloc, f0 = h_nfree;
    int ra, rb, rc, rd, re, rf, rg, rh, ri;
    short sh[4] = {1, 2, 3, 4};
    int out1[4] = {0x11, 0x22, 0x33, 0x44}, out2[4] = {0x55, 0x66, 0x77, 0x88};
    float fl[8] = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f};
    for (t = 0; t < 40; t++) {
      kf_c1(blk, blk + 64, (int)h_rnd(), blk + 128); kf_c2(blk, blk + 64, (int)h_rnd(), blk + 128); kf_d1(blk); kf_d2(blk);
      kf_ext(blk); kf_int(blk); kf_ep3(blk); kf_epfd(blk); kf_mm(blk, (int)h_rnd(), (int)h_rnd());
      ra = kf_run(blk, fl, h_rnd(), out1, out2);
      rb = kf_gc(blk, blk + 32, h_rnd()); rc = kf_gle(blk, blk + 32, h_rnd()); rd = kf_gdfe(blk, blk + 32, h_rnd()); re = kf_gde(blk, blk + 32, h_rnd());
      rf = kf_grp(blk, blk + 32, h_rnd()); rg = kf_gro(blk, blk + 32, h_rnd()); rh = kf_gja(blk, sh); ri = kf_gmp(blk, sh);
      { int rj = kf_gmps(blk);
        if (ra != 5) h_die3("k56FlexRunDemodulator returned %d", ra);
        if (rb || rc || rd || re || rf || rg || rh || ri || rj) h_die3("a K56Flex getter returned non-zero");
        h_cov_hit(CV_K56_METHODS); h_cov_hit(CV_K56_RUN); }
      h_emit(0, "k56 %d %d%d%d%d%d%d%d%d", ra, rb, rc, rd, re, rf, rg, rh, ri);
    }
    if (fnv_blk(blk, 256) != h0 || memcmp(blk, snap, 256)) h_die3("a K56Flex stub modified the object");
    if (memcmp(sh, (short[4]){1, 2, 3, 4}, sizeof sh) || memcmp(out1, (int[4]){0x11, 0x22, 0x33, 0x44}, sizeof out1) || memcmp(out2, (int[4]){0x55, 0x66, 0x77, 0x88}, sizeof out2)) h_die3("a K56Flex stub modified an argument");
    for (t = 0; t < 8; t++) if (fl[t] != (float)(t + 1)) h_die3("a K56Flex stub modified the sample buffer");
    if (h_nmalloc != m0 || h_nfree != f0) h_die3("a K56Flex stub allocated / freed");
  }
  /* --- K56FLEX_Create / Delete: one malloc of 0x14 bytes, never initialised; Delete(NULL) does nothing */
  { unsigned long m0 = h_nmalloc, f0 = h_nfree;
    void *p = k56_create();
    HAl *a;
    if (!p) h_die3("K56FLEX_Create returned NULL");
    a = h_find_al(p, 0);
    if (!a || a->size != 0x14 || a->p != (uint8_t *)p) h_die3("K56FLEX_Create: block size %u (expected 0x14)", a ? a->size : 0);
    h_cov_hit(CV_K56_CREATE);
    h_emit(1, "K56FLEX_Create: malloc %lu", h_nmalloc - m0);
    k56_delete(NULL);
    if (h_nfree != f0) h_die3("K56FLEX_Delete(NULL) freed");
    h_cov_hit(CV_K56_DELETE_NULL);
    k56_delete(p);
    if (h_nfree != f0 + 1) h_die3("K56FLEX_Delete(p) did not free");
    h_cov_hit(CV_K56_DELETE);
    h_emit(1, "K56FLEX_Delete: free %lu", h_nfree - f0);
  }
  { int r = k56_term(); if (r) h_die3("K56FLEX_SessionTermination returned %d", r); h_cov_hit(CV_K56_TERM); h_emit(1, "K56FLEX_SessionTermination %d", r); }
  /* --- parser stubs: return 0, touch nothing */
  { int iv = 0x1234; float fv = 3.5f; char name[16] = "SomeParameter"; unsigned long m0 = h_nmalloc;
    for (t = 0; t < 20; t++) {
      int r1 = vp_int(blk + 64, name, &iv), r2 = vp_flt(blk + 64, name, &fv), r3 = vp_int(NULL, NULL, NULL), r4 = vp_flt(NULL, NULL, NULL);
      if (r1 || r2 || r3 || r4) h_die3("a parser stub returned non-zero");
      if (iv != 0x1234 || fv != 3.5f) h_die3("a parser stub wrote its output argument");
      h_cov_hit(CV_VP_INT); h_cov_hit(CV_VP_FLT);
    }
    if (memcmp(blk, snap, 256) || h_nmalloc != m0) h_die3("a parser stub touched memory");
    h_emit(1, "parser stubs ok");
  }
  /* --- print_echo_coeffs: single coefficients against the exact model; many coefficients blob vs port */
  { static const uint32_t sp[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7f800000u, 0xff800000u, 0x7f800001u, 0xff800001u, 0x7fa00000u, 0x00000001u, 0x80000001u, 0x007fffffu,
      0x3f800000u, 0xbf800000u, 0x3f000000u, 0xbf000000u, 0x3dcccccdu, 0xbdcccccdu, 0x358637bdu, 0xb58637bdu, 0x358637beu, 0x3f7fffffu, 0xbf7fffffu, 0x3effffffu, 0x4b000000u, 0x4b7fffffu,
      0x4efffffeu, 0x4f000000u, 0xcf000000u, 0xcf000001u, 0xcf000100u, 0x4f000100u, 0x4f800000u, 0x5f000000u, 0xdf000000u, 0x7f7fffffu, 0xff7fffffu, 0x2edbe6ffu, 0x2edbe700u, 0x49742400u, 0x497423ffu, 0x4b189680u };
    static const uint32_t many[] = { 0x3f800000u, 0xc0000000u, 0x40490fdbu, 0x00000000u, 0x7fc00000u, 0xbf000000u, 0x3a83126fu, 0xc47a0000u, 0x5f000000u, 0x80000000u };
    unsigned k;
    g_ed_show = 1;
    print_echo(NULL, 0);
    g_ed_show = 0;
    if (g_nedp != 1) h_die3("print_echo_coeffs(NULL, 0): %u edprintf calls", g_nedp);
    h_cov_hit(CV_PE_EMPTY);
    for (k = 0; k < sizeof sp / sizeof sp[0]; k++) pe_one(sp[k], "special");
    for (k = 0; k < 6000; k++) {
      uint32_t w, r = h_rnd();
      switch (k % 6) {
        case 0: w = r; break;                                                             /* anything */
        case 1: w = (r & 0x80000000u) | ((uint32_t)(100 + h_rmod(40)) << 23) | (h_rnd() & 0x7fffffu); break; /* |x| in 2^-27 .. 2^12 */
        case 2: w = (r & 0x80000000u) | ((uint32_t)(127 + h_rmod(33)) << 23) | (h_rnd() & 0x7fffffu); break; /* up to 2^32 */
        case 3: w = (r & 0x80000000u) | ((uint32_t)(60 + h_rmod(60)) << 23) | (h_rnd() & 0x7fffffu); break;
        case 4: w = (r & 0x80000000u) | ((uint32_t)(120 + h_rmod(8)) << 23) | (h_rnd() & 0x7fffffu); break;   /* ~1e-3 .. 1 */
        default: w = (r & 0x80000000u) | ((uint32_t)(127 + h_rmod(24)) << 23) | (h_rnd() & 0x7fffffu); break; /* integers and fractions up to 2^24 */
      }
      if (((w >> 23) & 0xff) < 190 || is_nan_w(w) || !is_fin_w(w)) pe_one(w, "random");   /* |x| < 2^63: the model's exact range */
      else pe_one(w & 0x807fffffu, "random");
    }
    for (k = 0; k < 40; k++) {
      uint32_t w[40];
      unsigned n = k < 20 ? k : 20 + h_rmod(20), j;
      for (j = 0; j < n; j++) w[j] = (j < sizeof many / sizeof many[0] && k % 3 == 0) ? many[j] : (h_rnd() & 0x807fffffu) | ((uint32_t)(110 + h_rmod(30)) << 23);
      pe_check(w, n, "block");
      if (n >= 8) h_cov_hit(CV_PE_N_MANY);
    }
  }
  h_trial_end("stubs");
}



/* ================================================================================================ (B) helpers */
#define DUMP(ctr, tag, obj, T) h_dump_obj(h_sample(&(ctr), 12, 997), (tag), (obj), f_##T, NF_##T)
static uint32_t rnd_word_kind(void) { /* an arbitrary 32-bit word with interesting classes */
  static const uint32_t sp[] = { 0u, 1u, 2u, 0xffu, 0x100u, 0x1ffu, 0x80u, 0x7fu, 0x8000u, 0x10000u, 0x80000000u, 0xffffffffu, 0x7fffffffu };
  unsigned k = h_rmod(4);
  if (k == 0) return sp[h_rmod(sizeof sp / sizeof sp[0])];
  if (k == 1) return h_rmod(300);
  return h_rnd();
}

/* ================================================================================================ (B) scenario 2: V92Mapper */
/* extended square root (IEEE: correctly rounded on every x86 / x86-64 host; an asm statement because the 32-bit link has no libm) */
static LD ld_sqrt(LD x) { __asm__("fsqrt" : "+t"(x)); return x; }
static int16_t model_mapper(const uint8_t *bits, int16_t nbits, uint8_t flag, uint32_t powerw, int16_t scale, int *idx_out) {
  int n = (int)nbits, i, acc = 0, idx;
  float pw_;
  LD q, r;
  static const int T[16] = { 1, 3, -1, -3, 0, 0, 0, 0, 1, 3, 5, 7, -1, -3, -5, -7 };   /* independent copy of the table */
  for (i = n - 1; i >= 0; i--) acc = (int16_t)(((2u * (unsigned)acc) | bits[i]) & 0xffffu);
  idx = acc + 8 * (int)flag;
  *idx_out = idx;
  if (idx < 0 || idx > 15) return 0;
  memcpy(&pw_, &powerw, 4);
  q = (LD)T[idx] / ld_sqrt((LD)pw_);
  r = q * (LD)scale;
  if (!(r > -32769.0L && r < 32768.0L)) return (int16_t)0x8000;
  return (int16_t)(int)r;
}

static void sc_mapper(void) {
  static const int16_t scales[] = { 0, 1, -1, 7, 100, 1000, 20000, 32767, -32768, -12345, 255, 256, 16384, -16384, 10923, -10923, 16385, -16385 };   /* 16384 / 0.25 gives 32768 exactly, -3 * 10923 / 1 gives -32769 exactly: the integer-indefinite boundaries */
  static const uint32_t powers[] = { 0x40a00000u, 0x41a80000u, 0x3f800000u, 0x00000000u, 0x80000000u, 0x7f800000u, 0xff800000u, 0x7fc00000u, 0xffc00000u, 0x00000001u, 0x7f7fffffu, 0x34000000u,
                                     0x42c80000u, 0xbf800000u, 0x3dcccccdu, 0x40000000u, 0x3e800000u, 0x3f800000u, 0x3e800000u, 0x3f800000u };
  unsigned long ctr = 0;
  unsigned t;
  uint64_t acc = 0;
  cov_def(CV_VM_C2, "V92Mapper C2 / D2 used"); cov_def(CV_VM_RESET0, "reset flag 0"); cov_def(CV_VM_RESET1, "reset flag 1"); cov_def(CV_VM_RESETN, "reset flag >= 2 (nonzero byte)");
  cov_def(CV_VM_NB2, "process nbits 2"); cov_def(CV_VM_NB3, "process nbits 3"); cov_def(CV_VM_NB0, "process nbits <= 0"); cov_def(CV_VM_NB4, "process nbits >= 4");
  cov_def(CV_VM_WRAP16, "process: more than 16 bits (the 16-bit accumulator wraps)"); cov_def(CV_VM_BYTE, "process: a bit byte other than 0 / 1"); cov_def(CV_VM_NEGSCALE, "negative scale");
  cov_def(CV_VM_CLIP, "result clipped to the integer indefinite"); cov_def(CV_VM_ZEROENTRY, "table entry 0"); cov_def(CV_VM_FLAG1TAB, "flag 1 table half"); cov_def(CV_VM_POWPOKE, "power_ poked");
  cov_def(CV_VM_NEGENTRY, "negative table entry"); cov_def(CV_VM_GARBAGE, "garbage in the upper argument bits"); cov_def(CV_VM_INRANGE, "process result in range");
  cov_def(CV_VM_NAN, "NaN / inf power");
  h_scenario_begin(0);
  relax_reserved();
  for (t = 0; t < 12000; t++) {
    MapObj *o = (MapObj *)onew(sizeof(MapObj));
    uint8_t *bits = (uint8_t *)onew(64);
    int c2 = h_rpct(30), flag = h_rpct(60) ? (int)h_rmod(2) : (h_rpct(50) ? (int)h_rmod(256) : 0);
    int16_t sc = h_rpct(50) ? scales[h_rmod(sizeof scales / sizeof scales[0])] : (int16_t)h_rnd();
    unsigned i, calls = 1 + h_rmod(6);
    uint32_t pwd;
    uint32_t argS = (uint32_t)(uint16_t)sc | (h_rpct(60) ? (h_rnd() << 16) : 0u), argF = (uint32_t)(uint8_t)flag | (h_rpct(60) ? (h_rnd() << 8) : 0u);
    if (argS >> 16 || argF >> 8) h_cov_hit(CV_VM_GARBAGE);
    if (c2) { vm_c2(o); h_cov_hit(CV_VM_C2); } else vm_c1(o);
    DUMP(ctr, "vm ctor", o, MapObj);
    vm_reset(o, (int)argS, (int)argF);
    DUMP(ctr, "vm reset", o, MapObj);
    /* model of reset */
    if (o->scale_ != (uint16_t)sc || o->flag_ != (uint8_t)flag) h_die3("V92Mapper::reset: scale_ %04x flag_ %02x", o->scale_, o->flag_);
    if (flag == 0) { if (o->nbits_ != 2 || bits_f(o->power_) != 0x40a00000u) h_die3("V92Mapper::reset(flag 0): nbits_ %u power %08x", o->nbits_, bits_f(o->power_)); h_cov_hit(CV_VM_RESET0); }
    else { if (o->nbits_ != 3 || bits_f(o->power_) != 0x41a80000u) h_die3("V92Mapper::reset(flag %d): nbits_ %u power %08x", flag, o->nbits_, bits_f(o->power_)); h_cov_hit(flag == 1 ? CV_VM_RESET1 : CV_VM_RESETN); }
    if (sc < 0) h_cov_hit(CV_VM_NEGSCALE);
    for (i = 0; i < calls; i++) {
      int16_t nb;
      uint32_t pword;
      int idx, k;
      int16_t got, want;
      /* the constellation index must stay inside the table: flag 0 -> 0..15, flag 1 -> 0..7; other flags only as reset checks */
      if (flag >= 2) break;
      memset(bits, 0, 64);
      if (h_rpct(70)) {
        nb = (int16_t)(flag ? 3 : 2);
        if (h_rpct(25)) nb = (int16_t)(h_rrange(-3, 4));
        if (h_rpct(10)) nb = (int16_t)(flag ? 3 : 4);
        for (k = 0; k < 8; k++) bits[k] = (uint8_t)h_rmod(2);
      } else if (h_rpct(40)) {   /* more than 16 bits: only the last 16 processed bits survive in the 16-bit accumulator */
        nb = (int16_t)(17 + h_rmod(20));
        for (k = 0; k < 64; k++) bits[k] = (k >= 4 && k < 16) ? 0 : (uint8_t)h_rmod(2);
        for (k = 16; k < 64; k++) bits[k] = (uint8_t)h_rmod(256);   /* shifted out whatever they are */
      } else {                   /* full bytes (merged with `movzx dx, byte`: not masked to one bit) */
        nb = (int16_t)(1 + h_rmod(3));
        for (k = 0; k < 8; k++) bits[k] = (uint8_t)h_rmod(8);
      }
      o->nbits_ = (uint16_t)nb;
      pword = h_rpct(60) ? (flag ? 0x41a80000u : 0x40a00000u) : powers[h_rmod(sizeof powers / sizeof powers[0])];
      if (pword != (flag ? 0x41a80000u : 0x40a00000u)) { memcpy(&o->power_, &pword, 4); h_cov_hit(CV_VM_POWPOKE); }
      if (h_rpct(20)) { o->scale_ = (uint16_t)(sc = scales[h_rmod(sizeof scales / sizeof scales[0])]); }
      pwd = bits_f(o->power_);
      want = model_mapper(bits, nb, (uint8_t)flag, pwd, (int16_t)o->scale_, &idx);
      if (idx < 0 || idx > 15) continue;
      got = vm_proc(o, bits);
      if (got != want) h_die3("V92Mapper::process: got %d, model %d (nbits %d flag %d idx %d scale %d power %08x)", (int)got, (int)want, (int)nb, flag, idx, (int)(int16_t)o->scale_, pwd);
      DUMP(ctr, "vm proc", o, MapObj);
      h_emit(h_sample(&ctr, 12, 499), "vm proc ret %d idx %d", (int)got, idx);
      acc = acc * 31 + (uint64_t)(uint16_t)got;
      h_cov_hit(nb == 2 ? CV_VM_NB2 : nb == 3 ? CV_VM_NB3 : nb <= 0 ? CV_VM_NB0 : nb >= 4 ? CV_VM_NB4 : CV_VM_NB2);
      if (nb > 16) h_cov_hit(CV_VM_WRAP16);
      for (k = 0; k < 8 && nb <= 16; k++) if (bits[k] > 1) { h_cov_hit(CV_VM_BYTE); break; }
      if (got == (int16_t)0x8000 && want == (int16_t)0x8000) h_cov_hit(CV_VM_CLIP); else h_cov_hit(CV_VM_INRANGE);
      if (vm_tab[idx] == 0) h_cov_hit(CV_VM_ZEROENTRY);
      if (vm_tab[idx] < 0) h_cov_hit(CV_VM_NEGENTRY);
      if (flag) h_cov_hit(CV_VM_FLAG1TAB);
      if (is_nan_w(pwd) || (pwd & 0x7f800000u) == 0x7f800000u) h_cov_hit(CV_VM_NAN);
    }
    if (c2) vm_d2(o); else vm_d1(o);
    DUMP(ctr, "vm dtor", o, MapObj);
    h_trial_end("vm");
  }
  /* the table symbol of the object: the harness' own copy */
  { static const int T[16] = { 1, 3, -1, -3, 0, 0, 0, 0, 1, 3, 5, 7, -1, -3, -5, -7 };
    if (memcmp(vm_tab, T, sizeof T)) h_die3("constelAmplitudeTable differs from the harness' copy");
  }
  h_emit(1, "vm hash %016llx", (unsigned long long)acc);
}

/* ================================================================================================ (B) scenario 3: ModulusEncoder / ModulusDecoder */
/* the schoolbook reference of the encoder for m6 <= 63 (the accumulator stays non-negative): digit k = (acc / (m0 ... m(k-1))) % m_k, digit 5 = the rest; for the
 * m6 >= 64 regime the signed 64-bit semantics of the blob (negative accumulator, the zero-extended remainder subtracted) are written out step by step */
static void model_enc(const unsigned *m, const uint8_t *bits, unsigned nbits, unsigned *out) {
  int64_t acc = 0;
  int64_t e;
  int k;
  for (e = (int64_t)(int32_t)nbits - 1; e >= 0; e--) acc = (int64_t)((uint64_t)acc * 2u + (bits[e] & 1u));
  for (k = 0; k < 5; k++) {
    int64_t d = (int64_t)(uint64_t)m[k];
    int64_t r = acc % d;
    out[k] = (uint32_t)r;
    acc = (int64_t)((uint64_t)acc - (uint64_t)out[k]) / d;
  }
  out[5] = (uint32_t)acc;
}
static void model_dec(const unsigned *m, const unsigned *digits, unsigned nbits, uint8_t *bits) {
  uint64_t acc = digits[5];
  unsigned i;
  int k;
  for (k = 4; k >= 0; k--) acc = acc * (uint64_t)m[k] + (uint64_t)digits[k];
  for (i = 0; i < nbits; i++) bits[i] = (uint8_t)((acc >> (i < 63 ? i : 63)) & 1u);   /* the arithmetic shift keeps the sign bit from bit 63 on */
}
static unsigned rnd_radix(void) {
  unsigned k = h_rmod(10);
  if (k == 0) return 1;
  if (k < 5) return 2 + h_rmod(15);
  if (k < 7) return 2 + h_rmod(1000);
  if (k < 8) return 0x10000u + h_rmod(0x10000);
  if (k < 9) return 0x7fffffffu - h_rmod(3) + (h_rpct(30) ? 1u : 0u);
  return 0x80000000u + h_rmod(0x7fffffffu);
}

static void sc_modulus(void) {
  unsigned long ctr = 0;
  unsigned t;
  uint64_t acc = 0;
  cov_def(CV_MC_CTOR0, "ctor zero fill"); cov_def(CV_MC_CTOR7, "ctor 7 arguments"); cov_def(CV_MC_C2, "C2 used"); cov_def(CV_MC_ENC, "encoder progress"); cov_def(CV_MC_DEC, "decoder progress");
  cov_def(CV_MC_ROUNDTRIP, "round trip (m6 <= 63)"); cov_def(CV_MC_M6_0, "m6 == 0"); cov_def(CV_MC_M6_64, "m6 >= 64 (the accumulator wraps / goes negative)"); cov_def(CV_MC_RADIX1, "radix 1");
  cov_def(CV_MC_BIGRADIX, "radix >= 2^31"); cov_def(CV_MC_NEGACC, "negative accumulator"); cov_def(CV_MC_M6_HUGE, "encoder m6 >= 2^31 (the signed counter ends the loop at once)");
  cov_def(CV_MC_M5, "m5 varied"); cov_def(CV_MC_D5BIG, "digit 5 >= 2^32 truncated"); cov_def(CV_MC_SAR, "decoder: bits beyond 64 repeat the sign");
  h_scenario_begin(0);
  relax_reserved();
  for (t = 0; t < 40000; t++) {
    MeObj *e = (MeObj *)onew(sizeof(MeObj));
    MdObj *d = (MdObj *)onew(sizeof(MdObj));
    unsigned m[7], digits[8], digits2[8], i, nbits;
    uint8_t bits[260], back[260];
    int mode = (int)h_rmod(10);
    if (t < 2) {   /* the default constructors */
      if (t == 0) { me_c1(e); md_c1(d); } else { me_c2(e); md_c2(d); h_cov_hit(CV_MC_C2); }
      DUMP(ctr, "mc ctor0 e", e, MeObj); DUMP(ctr, "mc ctor0 d", d, MdObj);
      h_cov_hit(CV_MC_CTOR0);
      if (e->m0_ | e->m1_ | e->m2_ | e->m3_ | e->m4_ | e->m5_ | e->m6_ | d->m0_ | d->m1_ | d->m2_ | d->m3_ | d->m4_ | d->m5_ | d->m6_) h_die3("modulus default ctor did not clear");
      h_trial_end("mc");
      continue;
    }
    for (i = 0; i < 7; i++) m[i] = rnd_radix();
    m[5] = h_rnd();
    if (mode < 6) nbits = 1 + h_rmod(63); else if (mode < 8) nbits = h_rmod(64); else if (mode == 8) nbits = 64 + h_rmod(40); else nbits = h_rmod(3) == 0 ? 0 : 1 + h_rmod(12);
    if (h_rpct(8)) nbits = 63;
    m[6] = nbits;
    if (h_rpct(50)) { me_c1n(e, m[0], m[1], m[2], m[3], m[4], m[5], m[6]); md_c1n(d, m[0], m[1], m[2], m[3], m[4], m[5], m[6]); }
    else { me_c2n(e, m[0], m[1], m[2], m[3], m[4], m[5], m[6]); md_c2n(d, m[0], m[1], m[2], m[3], m[4], m[5], m[6]); h_cov_hit(CV_MC_C2); }
    h_cov_hit(CV_MC_CTOR7);
    DUMP(ctr, "mc ctor e", e, MeObj); DUMP(ctr, "mc ctor d", d, MdObj);
    if (e->m0_ != m[0] || e->m1_ != m[1] || e->m2_ != m[2] || e->m3_ != m[3] || e->m4_ != m[4] || e->m5_ != m[5] || e->m6_ != m[6]) h_die3("ModulusEncoder 7-argument ctor stored the wrong words");
    if (d->m0_ != m[0] || d->m1_ != m[1] || d->m2_ != m[2] || d->m3_ != m[3] || d->m4_ != m[4] || d->m5_ != m[5] || d->m6_ != m[6]) h_die3("ModulusDecoder 7-argument ctor stored the wrong words");
    /* encode a random bit string: random bits, or an all-ones string (the largest value), or a single bit */
    for (i = 0; i < 260; i++) bits[i] = (uint8_t)(h_rnd());   /* the encoder masks to one bit: any byte */
    if (h_rpct(10)) for (i = 0; i < 260; i++) bits[i] = 1;
    if (h_rpct(10)) for (i = 0; i < 260; i++) bits[i] = (uint8_t)(i == nbits / 2);
    for (i = 0; i < 8; i++) digits[i] = digits2[i] = 0xdeadbeefu ^ (uint32_t)i;
    { unsigned want[6];
      model_enc(m, bits, nbits, want);
      me_prog(e, bits, digits);
      h_cov_hit(CV_MC_ENC);
      for (i = 0; i < 6; i++) if (digits[i] != want[i]) h_die3("ModulusEncoder::progress: digit %u = 0x%x, model 0x%x (m6 %u, radices %u %u %u %u %u)", i, digits[i], want[i], nbits, m[0], m[1], m[2], m[3], m[4]);
      if (digits[6] != (0xdeadbeefu ^ 6u) || digits[7] != (0xdeadbeefu ^ 7u)) h_die3("ModulusEncoder::progress wrote beyond digit 5");
      DUMP(ctr, "mc enc obj", e, MeObj);
      h_emit(h_sample(&ctr, 12, 499), "mc enc %u: %x %x %x %x %x %x", nbits, digits[0], digits[1], digits[2], digits[3], digits[4], digits[5]);
      for (i = 0; i < 6; i++) acc = acc * 1000003u + digits[i];
      if (nbits >= 64) { h_cov_hit(CV_MC_M6_64); }
      if (nbits == 0) h_cov_hit(CV_MC_M6_0);
      for (i = 0; i < 5; i++) { if (m[i] == 1) h_cov_hit(CV_MC_RADIX1); if (m[i] >= 0x80000000u) h_cov_hit(CV_MC_BIGRADIX); }
      if (nbits >= 64) { int64_t a = 0; int64_t k; for (k = (int64_t)nbits - 1; k >= 0; k--) a = (int64_t)((uint64_t)a * 2u + (bits[k] & 1u)); if (a < 0) h_cov_hit(CV_MC_NEGACC); }
      if (nbits < 64 && digits[5] != 0) { uint64_t ac2 = 0; int64_t k; for (k = (int64_t)nbits - 1; k >= 0; k--) ac2 = ac2 * 2u + (bits[k] & 1u); (void)ac2; }
    }
    /* decode arbitrary digits (any 6 words) and the digits of the encoder */
    for (i = 0; i < 6; i++) digits2[i] = (mode >= 5 && i < 5 && m[i] != 0) ? (digits[i] ? digits[i] : h_rnd() % m[i]) : h_rnd();
    if (mode < 5) for (i = 0; i < 6; i++) digits2[i] = digits[i];
    for (i = 0; i < 260; i++) back[i] = 0xa5;
    { uint8_t want[260];
      memset(want, 0xa5, sizeof want);
      model_dec(m, digits2, nbits, want);
      md_prog(d, back, digits2);
      h_cov_hit(CV_MC_DEC);
      if (memcmp(back, want, sizeof back)) h_die3("ModulusDecoder::progress differs from the model (m6 %u)", nbits);
      if (nbits >= 64) { uint64_t ac2 = digits2[5]; int k; for (k = 4; k >= 0; k--) ac2 = ac2 * m[k] + digits2[k]; if ((int64_t)ac2 < 0) h_cov_hit(CV_MC_SAR); }
      DUMP(ctr, "mc dec obj", d, MdObj);
      h_emit(h_sample(&ctr, 12, 499), "mc dec %u: %02x%02x%02x%02x %02x%02x", nbits, back[0], back[1], back[2], back[3], back[nbits ? nbits - 1 : 0], back[nbits]);
      acc = acc * 1000003u + fnv_blk(back, nbits + 1);
    }
    /* the round trip: the encoder's digits decode back to the bits whenever the accumulator stayed positive and below 2^63 */
    if (mode < 5 && nbits <= 63) {
      uint64_t q = 0;
      int64_t kk;
      for (kk = (int64_t)nbits - 1; kk >= 0; kk--) q = q * 2u + (bits[kk] & 1u);
      for (i = 0; i < 5; i++) q /= m[i];
      if (q >> 32) goto no_roundtrip;                      /* digit 5 would be truncated to 32 bits: no round trip */
      md_prog(d, back, digits);
      for (i = 0; i < nbits; i++) if (back[i] != (bits[i] & 1u)) h_die3("round trip failed at bit %u (m6 %u)", i, nbits);
      h_cov_hit(CV_MC_ROUNDTRIP);
    }
  no_roundtrip:
    if (m[5] != 0) h_cov_hit(CV_MC_M5);
    if (digits[5] > 0xffffu) h_cov_hit(CV_MC_D5BIG);
    h_trial_end("mc");
  }
  /* encoder with a huge m6 (>= 2^31): the SIGNED counter makes the bit loop empty (the accumulator stays 0) */
  { MeObj *e = (MeObj *)onew(sizeof(MeObj));
    unsigned digits[6];
    uint8_t bits[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };
    unsigned k;
    for (k = 0; k < 4; k++) {
      unsigned m6 = k == 0 ? 0x80000001u : k == 1 ? 0xffffffffu : k == 2 ? 0xfffffffeu : 0x80000002u;   /* (0x80000000 and 0x7fffffff would run the bit loop 2^31 times in the blob: excluded) */
      me_c1n(e, 3, 5, 7, 11, 13, 17, m6);
      me_prog(e, bits, digits);
      h_emit(1, "mc enc m6 %08x: %u %u %u %u %u %u", m6, digits[0], digits[1], digits[2], digits[3], digits[4], digits[5]);
      if (digits[0] | digits[1] | digits[2] | digits[3] | digits[4] | digits[5]) h_die3("ModulusEncoder with m6 = 0x%x: non-zero digits", m6);
      h_cov_hit(CV_MC_M6_HUGE);
    }
  }
  h_emit(1, "mc hash %016llx", (unsigned long long)acc);
  h_trial_end("mc");
}

/* ================================================================================================ (B) scenarios 4 / 5: V90Phase2Info / V92Phase2Info */
static unsigned g_plan[64], g_plan_len;
static int g_plan_on;
static void plan_hook(unsigned idx) { if (g_plan_on && g_plan_len) dsplibs_debug_level = g_plan[idx < g_plan_len ? idx : g_plan_len - 1]; }
/* expected event stream of one printInfo call */
typedef struct { char dbg[64][300]; unsigned ndbg; char ed[64][300]; unsigned ned; unsigned lvl; } Expect;
static void ex_dbg(Expect *x, const char *fmt, ...) {
  char buf[400];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  x->dbg[x->ndbg][0] = 0;
  esc_cat(x->dbg[x->ndbg], 300, buf);
  if (g_plan_on && g_plan_len) x->lvl = g_plan[x->ndbg < g_plan_len ? x->ndbg : g_plan_len - 1];
  x->ndbg++;
}
static void ex_ed(Expect *x, const char *fmt, ...) {
  char buf[400];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  x->ed[x->ned][0] = 0;
  esc_cat(x->ed[x->ned], 300, buf);
  x->ned++;
}
static void model_phase2_print(Expect *x, int v92, unsigned pcm, unsigned rtd, unsigned maxtx, unsigned tx, const uint8_t *b10to17, unsigned uinfo, const uint32_t *l2w, unsigned lvl0) {
  const char *nm = v92 ? "V92Phase2Info" : "V90Phase2Info";
  unsigned i;
  x->lvl = lvl0; x->ndbg = x->ned = 0;
  if (x->lvl > 1) {
    ex_dbg(x, "%s: pcmType = %s\r\n", nm, pcm == 1 ? "A_LAW" : "MU_LAW");
    if (x->lvl > 1) {
      ex_dbg(x, "%s: rtd = %d\r\n", nm, (int)rtd);
      if (x->lvl > 1) {
        unsigned h2 = (maxtx & 0xffu) + 1u;     /* maxTxPower = -(b + 1) / 2 dBm0: integer part (b + 1) / 2, fraction digit 5 or 0 */
        ex_dbg(x, "%s: maxTxPower [dBm0]  = %c%d.%01d\r\n", nm, '-', (int)(h2 / 2u), (int)((h2 % 2u) * 5u));
        if (x->lvl > 1) ex_dbg(x, "%s: txPowerMeasurementPoint = %s\r\n", nm, tx == 1 ? "CodecOutput" : "DigitalModemTerminal");
      }
    }
  }
  if (v92) {
    ex_ed(x, "V92Phase2Info: ShortPhase2: local=%d , remote=%d\r\n", (int)b10to17[0], (int)b10to17[2]);
    ex_ed(x, "V92Phase2Info: v92Capabilities: local=%d , remote=%d\r\n", (int)b10to17[1], (int)b10to17[3]);
    ex_ed(x, "V92Phase2Info: v90UseHighCarrier = %d\r\n", (int)b10to17[7]);
  }
  ex_ed(x, "%s: Uinfo = %d\r\n", nm, (int)(uinfo & 0xffu));
  for (i = 0; i <= 20; i++) {
    if (x->lvl > 1) {
      int sg, ip, fp;
      model_split(l2w[i], 1000, &sg, &ip, &fp);
      ex_dbg(x, "%s: L2[%d] = %c%d.%03d\r\n", nm, (int)i, sg, ip, fp);
    }
  }
}

static void sc_phase2(int v92) {
  unsigned long ctr = 0;
  unsigned t;
  uint64_t acc = 0;
  const size_t osz = v92 ? sizeof(P2wObj) : sizeof(P2vObj);
  const size_t OFF_PARAMS = v92 ? offsetof(P2wObj, params_) : offsetof(P2vObj, params_), OFF_L2 = v92 ? offsetof(P2wObj, l2_) : offsetof(P2vObj, l2_);
  const HFld *ft = v92 ? f_P2wObj : f_P2vObj;
  const int nf = v92 ? NF_P2wObj : NF_P2vObj;
  void (*c1)(void *, void *) = v92 ? p2w_c1 : p2v_c1, (*c2)(void *, void *) = v92 ? p2w_c2 : p2v_c2;
  void (*def)(void *) = v92 ? p2w_def : p2v_def;
  void (*prt)(const void *) = v92 ? p2w_print : p2v_print;
  cov_def(CV_P2_C2, "C2 used"); cov_def(CV_P2_DEF, "setToDefault"); cov_def(CV_P2_PRINT, "printInfo"); cov_def(CV_P2_LVL_FULL, "level > 1 for the whole call"); cov_def(CV_P2_LVL_OFF, "level <= 1");
  cov_def(CV_P2_LVL_CUT, "level dropped between the prints"); cov_def(CV_P2_BIGLEVEL, "level >= 2^31 (unsigned compare)"); cov_def(CV_P2_PCM1, "pcm A_LAW"); cov_def(CV_P2_PCM0, "pcm MU_LAW");
  cov_def(CV_P2_TX1, "tx point CodecOutput"); cov_def(CV_P2_TX0, "tx point DigitalModemTerminal"); cov_def(CV_P2_L2SPECIAL, "L2 value NaN / inf / huge"); cov_def(CV_P2_L2NEG, "L2 negative"); cov_def(CV_P2_L2FRAC, "L2 fraction");
  cov_def(CV_P2_WORD100, "parameter word with a zero low byte but non-zero (flag test is on the dword)"); cov_def(CV_P2_BYTETRUNC, "uinfo / maxTx word wider than a byte"); cov_def(CV_P2_MAXTX_ODD, "odd maxTx");
  cov_def(CV_P2_LEVELPLAN_LATE, "level cut inside the L2 loop"); cov_def(CV_P2_POKE, "flag words / byte fields poked");
  h_scenario_begin(0);
  relax_reserved();
  g_dbg_hook = plan_hook;
/* the parameter block stand-in: words of the i386 layout.  The first member of V90Parameters / V92Parameters is a pointer (4 bytes i386, 8 bytes x86-64), so on a
 * 64-bit build the word that is number n on i386 is number n + 1: PW(n) is the word the port reads.  The random stream is the same at both widths. */
#define PW(n) par[(n) + sizeof(void *) / 4 - 1]
  for (t = 0; t < 4000; t++) {
    uint8_t *o = (uint8_t *)onew(osz);
    uint8_t *pre = (uint8_t *)onew(osz);
    uint32_t *par = (uint32_t *)(h_log_next_size = 0x80, sysdep_malloc(0x80 + sizeof(void *) - 4));   /* logged as the i386 size at both widths */
    uint32_t *l2 = (uint32_t *)sysdep_malloc(21 * 4);
    uint32_t pcmw, rtdw, uiw, mxw, txw, w5c = 0, w60 = 0, w64 = 0;
    unsigned i, lvl0;
    uint8_t exp[96];
    int usec2 = h_rpct(30);
    for (i = 0; i < 0x20; i++) PW(i) = rnd_word_kind();
    /* the parameter words the constructor reads */
    if (v92) { PW(2) = pcmw = rnd_word_kind(); PW(3) = rtdw = rnd_word_kind(); PW(4) = uiw = rnd_word_kind(); PW(5) = mxw = rnd_word_kind(); PW(6) = txw = rnd_word_kind(); w5c = PW(0x17); w60 = PW(0x18); w64 = PW(0x19); }
    else { PW(6) = pcmw = rnd_word_kind(); PW(7) = rtdw = rnd_word_kind(); PW(8) = uiw = rnd_word_kind(); PW(9) = mxw = rnd_word_kind(); PW(10) = txw = rnd_word_kind(); }
    memcpy(pre, o, osz);
    if (usec2) { c2(o, par); h_cov_hit(CV_P2_C2); } else c1(o, par);
    h_dump_obj(h_sample(&ctr, 12, 997), v92 ? "p2w ctor" : "p2v ctor", o, ft, nf);
    /* expected object: start from the pre-ctor fill, set the defined fields */
    memcpy(exp, pre, osz);
    {
      unsigned pcm = pcmw != 0, tx = txw != 0;
      memcpy(exp + 0, &pcm, 4); memcpy(exp + 4, &rtdw, 4); exp[8] = (uint8_t)uiw; exp[9] = (uint8_t)mxw; memcpy(exp + 0xc, &tx, 4);
      if (v92) { exp[0x10] = 0; exp[0x11] = 1; exp[0x12] = 0; exp[0x13] = 0; exp[0x14] = (uint8_t)w5c; exp[0x15] = (uint8_t)w60; exp[0x16] = (uint8_t)w64; exp[0x17] = 0; }
      /* params_ is a pointer: compared through the dump; here only the non-pointer part of the object */
      if (memcmp(o, exp, v92 ? 0x18 : 0x10)) h_die3("%s ctor: fields differ from the model", v92 ? "V92Phase2Info" : "V90Phase2Info");
      /* the untouched regions keep the fill */
      if (v92) { if (memcmp(o + 0x18, pre + 0x18, 8) || memcmp(o + offsetof(P2wObj, unk24_), pre + offsetof(P2wObj, unk24_), 4) || memcmp(o + offsetof(P2wObj, l2_), pre + offsetof(P2wObj, l2_), sizeof(void *))) h_die3("V92Phase2Info ctor touched an unknown region / l2_"); }
      else { if (memcmp(o + 0x10, pre + 0x10, 8) || memcmp(o + offsetof(P2vObj, unk1c_), pre + offsetof(P2vObj, unk1c_), 4) || memcmp(o + offsetof(P2vObj, l2_), pre + offsetof(P2vObj, l2_), sizeof(void *))) h_die3("V90Phase2Info ctor touched an unknown region / l2_"); }
    }
    { void *pp; memcpy(&pp, o + OFF_PARAMS, sizeof pp); if (pp != (void *)par) h_die3("params_ not stored"); }
    if (pcmw & 0xffu) h_cov_hit(CV_P2_PCM1); else h_cov_hit(CV_P2_PCM0);
    if ((pcmw != 0) && !(pcmw & 0xffu)) h_cov_hit(CV_P2_WORD100);
    if ((uiw >> 8) || (mxw >> 8)) h_cov_hit(CV_P2_BYTETRUNC);
    if (txw != 0) h_cov_hit(CV_P2_TX1); else h_cov_hit(CV_P2_TX0);
    if (mxw & 1) h_cov_hit(CV_P2_MAXTX_ODD);
    /* setToDefault re-reads the (changed) parameter words */
    if (h_rpct(50)) {
      for (i = 0; i < 0x20; i++) if (h_rpct(60)) PW(i) = rnd_word_kind();
      if (v92) { pcmw = PW(2); rtdw = PW(3); uiw = PW(4); mxw = PW(5); txw = PW(6); w5c = PW(0x17); w60 = PW(0x18); w64 = PW(0x19); }
      else { pcmw = PW(6); rtdw = PW(7); uiw = PW(8); mxw = PW(9); txw = PW(10); }
      memcpy(pre, o, osz);
      for (i = 0; i < 0x18; i++) if (h_rpct(30)) o[i] ^= (uint8_t)h_rnd();   /* the scalar part only (offsets and the number of random draws must not depend on sizeof) */   /* stale fields: setToDefault must overwrite the defined ones */
      memcpy(pre, o, osz);
      def(o);
      h_cov_hit(CV_P2_DEF);
      h_dump_obj(h_sample(&ctr, 12, 997), v92 ? "p2w def" : "p2v def", o, ft, nf);
      memcpy(exp, pre, osz);
      { unsigned pcm = pcmw != 0, tx = txw != 0;
        memcpy(exp + 0, &pcm, 4); memcpy(exp + 4, &rtdw, 4); exp[8] = (uint8_t)uiw; exp[9] = (uint8_t)mxw; memcpy(exp + 0xc, &tx, 4);
        if (v92) { exp[0x10] = 0; exp[0x11] = 1; exp[0x12] = 0; exp[0x13] = 0; exp[0x14] = (uint8_t)w5c; exp[0x15] = (uint8_t)w60; exp[0x16] = (uint8_t)w64; exp[0x17] = 0; }
        if (memcmp(o, exp, osz)) h_die3("%s setToDefault differs from the model", v92 ? "V92Phase2Info" : "V90Phase2Info");
      }
    }
    /* odd flag words (the prints compare == 1), odd byte fields */
    if (h_rpct(45)) {
      static const uint32_t flw[] = { 0u, 1u, 2u, 0xffffffffu, 0x100u, 0x80000001u };
      uint32_t a = flw[h_rmod(6)], b = flw[h_rmod(6)];
      memcpy(o + 0, &a, 4); memcpy(o + 0xc, &b, 4); o[8] = (uint8_t)h_rnd(); o[9] = (uint8_t)h_rnd();
      h_cov_hit(CV_P2_POKE);
    }
    /* printInfo: l2_ points to 21 arena floats (set by the owner in the blob; poked here) */
    for (i = 0; i < 21; i++) {
      unsigned k = h_rmod(12);
      l2[i] = k == 0 ? 0x7fc00000u : k == 1 ? 0x7f800000u : k == 2 ? 0xff800000u : k == 3 ? 0x4f000000u : k == 4 ? 0xcf000000u : k == 5 ? 0x80000000u : k == 6 ? 0x3a83126fu :
              ((h_rnd() & 0x80000000u) | ((uint32_t)(105 + h_rmod(40)) << 23) | (h_rnd() & 0x7fffffu));
      if (k < 7 && k != 6 && !h_rpct(15)) l2[i] = ((h_rnd() & 0x80000000u) | ((uint32_t)(110 + h_rmod(24)) << 23) | (h_rnd() & 0x7fffffu));
    }
    { void *pp = l2; memcpy(o + OFF_L2, &pp, sizeof pp); }
    { static const unsigned lv[] = { 0, 1, 2, 3, 2, 2, 5, 0xffffffffu, 0x80000000u, 0x80000001u, 7 };
      lvl0 = lv[h_rmod(sizeof lv / sizeof lv[0])];
      g_plan_on = h_rpct(55);
      g_plan_len = 1 + h_rmod(40);
      for (i = 0; i < g_plan_len; i++) g_plan[i] = h_rpct(75) ? 2u + h_rmod(3) : lv[h_rmod(sizeof lv / sizeof lv[0])];
      if (g_plan_on && h_rpct(40)) { unsigned cut = h_rmod(g_plan_len); g_plan[cut] = 1; if (cut >= 5) h_cov_hit(CV_P2_LEVELPLAN_LATE); }
    }
    { Expect *x = (Expect *)malloc(sizeof(Expect));
      uint8_t b10[8];
      uint32_t l2w[21];
      unsigned pcm, rtd, mx, tx, ui;
      memcpy(b10, o + 0x10, 8);
      for (i = 0; i < 21; i++) l2w[i] = l2[i];
      memcpy(&pcm, o, 4); memcpy(&rtd, o + 4, 4); mx = o[9]; tx = 0; memcpy(&tx, o + 0xc, 4); ui = o[8];
      memcpy(pre, o, osz);
      model_phase2_print(x, v92, pcm, rtd, mx, tx, b10, ui, l2w, lvl0);
      dsplibs_debug_level = lvl0;
      g_dbg_in_call = 0; g_ed_in_call = 0;
      g_ed_show = h_sample(&ctr, 12, 331); g_dbg_show = g_ed_show;
      prt(o);
      g_ed_show = g_dbg_show = 0;
      h_cov_hit(CV_P2_PRINT);
      dsplibs_debug_level = 0;
      if (g_dbg_in_call != x->ndbg) h_die3("%s printInfo: %u debug prints, model %u (level0 %u, plan %d)", v92 ? "V92" : "V90", g_dbg_in_call, x->ndbg, lvl0, g_plan_on);
      if (g_ed_in_call != x->ned) h_die3("%s printInfo: %u edprintf calls, model %u", v92 ? "V92" : "V90", g_ed_in_call, x->ned);
      for (i = 0; i < x->ndbg; i++) if (strcmp(g_dbg_text[i], x->dbg[i])) h_die3("printInfo debug print %u: '%s', model '%s'", i, g_dbg_text[i], x->dbg[i]);
      for (i = 0; i < x->ned; i++) if (strcmp(g_ed_text[i], x->ed[i])) h_die3("printInfo edprintf %u: '%s', model '%s'", i, g_ed_text[i], x->ed[i]);
      if (memcmp(o, pre, osz)) h_die3("printInfo modified the object");
      if (x->ndbg == 0) h_cov_hit(CV_P2_LVL_OFF);
      else if (x->ndbg == 4 + 21) h_cov_hit(CV_P2_LVL_FULL);
      else h_cov_hit(CV_P2_LVL_CUT);
      if (lvl0 >= 0x80000000u) h_cov_hit(CV_P2_BIGLEVEL);
      for (i = 0; i < 21; i++) { if ((l2w[i] & 0x7f800000u) == 0x7f800000u || (l2w[i] & 0x7f800000u) >= 0x4f000000u) h_cov_hit(CV_P2_L2SPECIAL); if (l2w[i] >> 31) h_cov_hit(CV_P2_L2NEG); if ((l2w[i] & 0x7f800000u) < 0x4b000000u && (l2w[i] & 0x7f800000u) > 0) h_cov_hit(CV_P2_L2FRAC); }
      for (i = 0; i < x->ndbg; i++) acc = acc * 1000003u + fnv_blk(x->dbg[i], strlen(x->dbg[i]));
      for (i = 0; i < x->ned; i++) acc = acc * 1000003u + fnv_blk(x->ed[i], strlen(x->ed[i]));
      free(x);
    }
    sysdep_free(par); sysdep_free(l2);
    h_trial_end("p2");
  }
  g_dbg_hook = NULL;
  dsplibs_debug_level = 0;
  h_emit(1, "p2 hash %016llx", (unsigned long long)acc);
}

/* ================================================================================================ (B) scenario 6: V90ConstellationPower */
static const uint32_t T_LIMITS[35] = { 228735376u, 203804176u, 181710400u, 161900176u, 144288144u, 128595600u, 114661264u, 102171664u, 91087936u, 81144064u, 72318016u, 64448784u,
                                       57456400u, 51208336u, 45643536u, 40704400u, 36240400u, 32307856u, 28815424u, 25684624u, 22886656u, 20394256u, 18181696u, 16192576u,
                                       14440000u, 12873744u, 11478544u, 10214416u, 9120400u, 8133904u, 7246864u, 6451600u, 5740816u, 5112121u, 4549689u };
#define PM_SIZE 0x700u
static uint32_t pmw(const uint8_t *pm, unsigned off) { uint32_t v; memcpy(&v, pm + off, 4); return v; }
typedef struct { int64_t M, A[7]; uint32_t r[6]; double D[6]; } CpModel;
static unsigned pm_radix(const uint8_t *pm, unsigned k) { return pmw(pm, 0x604 + 4 * k); }
/* the model of calcModulusParameters from the archaeology facts: floor divisions of non-negative values, the 64-bit shift by (count & 63), the extended product chain */
static void model_calc(const uint8_t *pm, CpModel *c) {
  unsigned s = (pmw(pm, 0x620) + pmw(pm, 0) - 6u) & 63u, k;
  uint64_t a;
  LD x;
  c->M = (int64_t)((uint64_t)1 << s);
  a = (uint64_t)c->M - 1u;
  c->A[0] = (int64_t)a;
  for (k = 0; k < 5; k++) {
    uint64_t d = pm_radix(pm, k);
    c->r[k] = (uint32_t)(a % d);
    a = a / d;
    c->A[k + 1] = (int64_t)a;
  }
  c->r[5] = (uint32_t)a;                                       /* the last "remainder" is the low word of the quotient */
  a = ((a >> 32) << 32) / pm_radix(pm, 5);                     /* (a - lo32(a)) / d5 */
  c->A[6] = (int64_t)a;
  c->D[0] = 1.0;
  x = (LD)pm_radix(pm, 0);
  c->D[1] = (double)x;
  for (k = 1; k < 5; k++) { x = x * (LD)pm_radix(pm, k); c->D[k + 1] = (double)x; }
}
static int cp_obj_check(const CpObj *o, const CpModel *m, const char *what) {
  unsigned k;
  if ((int64_t)bits_d(o->M_) != m->M) h_die3("%s: M_ 0x%llx, model 0x%llx", what, (unsigned long long)bits_d(o->M_), (unsigned long long)m->M);
  for (k = 0; k < 7; k++) if ((int64_t)bits_d(o->A_[k]) != m->A[k]) h_die3("%s: A_[%u] 0x%llx, model 0x%llx", what, k, (unsigned long long)bits_d(o->A_[k]), (unsigned long long)m->A[k]);
  for (k = 0; k < 6; k++) if (o->r_[k] != m->r[k]) h_die3("%s: r_[%u] 0x%x, model 0x%x", what, k, o->r_[k], m->r[k]);
  for (k = 0; k < 6; k++) if (bits_d(o->D_[k]) != bits_d(m->D[k])) h_die3("%s: D_[%u] %016llx, model %016llx", what, k, (unsigned long long)bits_d(o->D_[k]), (unsigned long long)bits_d(m->D[k]));
  return 0;
}
/* the model of getPower from the x87 stack traces: named quantities, extended precision except at the double stores */
static uint32_t model_power(const uint8_t *pm, const CpModel *c, int point, int pcm) {
  double total = 0.0;
  unsigned k, j;
  for (k = 0; k <= 5; k++) {
    unsigned n = pm_radix(pm, k);
    const uint8_t *tab = pm + (point == 1 ? 0x304u : 0x4u) + 0x80u * k;
    for (j = 0; j < n; j++) {
      double t2;
      unsigned b = tab[j] & 0x7fu;
      int v;
      LD Mx = (LD)c->M;
      if (c->r[k] > j) { double t = (double)((LD)c->D[k] / Mx); t2 = (double)((LD)(int64_t)((uint64_t)c->A[k + 1] + 1u) * (LD)t); }
      else if (c->r[k] == j) { LD s = ((LD)1 / Mx) * (LD)c->D[k]; LD d = (LD)(int64_t)((uint64_t)c->A[k] - (uint64_t)c->A[k + 1]); t2 = (double)((LD)1 - s * d); }
      else { double t = (double)((LD)c->D[k] / Mx); t2 = (double)((LD)c->A[k + 1] * (LD)t); }
      v = pcm ? ((b ^ 0xd5u) & 0xff) : (~b & 0xff);
      /* the decoded sample: the closed forms of the G.711 decoders (sc_g711's models) */
      v = pcm ? model_a2l((unsigned)v) : model_u2l((unsigned)v);
      total = (double)((LD)v * (LD)v * (LD)t2 + (LD)total);
    }
  }
  { float f = (float)((LD)total * (LD)0.16666666666666666); return bits_f(f); }
}

static void sc_power(void) {
  unsigned long ctr = 0;
  unsigned t, k;
  uint64_t acc = 0;
  cov_def(CV_CP_CTOR, "C1 / D1"); cov_def(CV_CP_C2, "C2 / D2"); cov_def(CV_CP_CALC, "calcModulusParameters"); cov_def(CV_CP_SHIFT_WRAP, "shift count >= 64 (masked)"); cov_def(CV_CP_MNEG, "M negative (shift 63)");
  cov_def(CV_CP_R5, "r_[5] != A_[5] (A_[5] >= 2^32)"); cov_def(CV_CP_D2BIG, "D_ product > 2^64 (extended rounding)"); cov_def(CV_CP_INFO, "getConstellationInfo"); cov_def(CV_CP_INFO_OOR, "getConstellationInfo k > 5");
  cov_def(CV_CP_INFO_HI, "getConstellationInfo point 1"); cov_def(CV_CP_POWER, "getPower"); cov_def(CV_CP_P_ALAW, "getPower A-law"); cov_def(CV_CP_P_ULAW, "getPower mu-law"); cov_def(CV_CP_P_HI, "getPower point 1");
  cov_def(CV_CP_P_GT, "r > j branch"); cov_def(CV_CP_P_EQ, "r == j branch"); cov_def(CV_CP_P_LT, "r < j branch"); cov_def(CV_CP_PIDX, "getPowerIndexForPower"); cov_def(CV_CP_PIDX_TOP, "index 34 (limit >= x at the top)");
  cov_def(CV_CP_PIDX_ZERO, "index 0"); cov_def(CV_CP_PIDX_MID, "index in 1..33"); cov_def(CV_CP_PIDX_NAN, "NaN power"); cov_def(CV_CP_LEGAL, "isLegalPowerIndex"); cov_def(CV_CP_LEGAL_NO, "isLegalPowerIndex false");
  cov_def(CV_CP_LEGAL_EDGE, "index 34 / 35"); cov_def(CV_CP_RADIX_BIG, "radix > 128 (tables overlap)"); cov_def(CV_CP_PIDX_EQ, "limit == x exactly"); cov_def(CV_CP_PCM_ODD, "pcm other than 0 / 1");
  cov_def(CV_CP_POINT_ODD, "point other than 0 / 1"); cov_def(CV_CP_RADIX1, "radix 1");
  h_scenario_begin(0);
  relax_reserved();
  /* the limits table of the object under test against the harness' copy */
  for (k = 0; k < 35; k++) if (cp_limits[k] != T_LIMITS[k]) h_die3("averagePowerLimits[%u] = %u, harness copy %u", k, cp_limits[k], T_LIMITS[k]);
  h_emit(1, "averagePowerLimits ok");
  for (t = 0; t < 1500; t++) {
    CpObj *o = (CpObj *)onew(sizeof(CpObj));
    uint8_t *pm = (uint8_t *)sysdep_malloc(PM_SIZE);
    CpModel m;
    unsigned c2 = h_rpct(30), i;
    uint32_t w0, w620;
    for (i = 0; i < PM_SIZE; i++) pm[i] = (uint8_t)h_rnd();
    w0 = h_rpct(70) ? h_rmod(24) : rnd_word_kind();
    w620 = h_rpct(70) ? h_rmod(24) : rnd_word_kind();
    if (h_rpct(10)) { w0 = 40 + h_rmod(40); w620 = 6 + h_rmod(80); }       /* shift count >= 64: masked to 6 bits */
    if (h_rpct(8)) { w620 = 63 + 6 - w0; }                                 /* shift 63: M negative */
    memcpy(pm + 0, &w0, 4); memcpy(pm + 0x620, &w620, 4);
    for (k = 0; k < 6; k++) {
      unsigned d = h_rpct(60) ? 1 + h_rmod(128) : h_rpct(50) ? 1 + h_rmod(300) : h_rpct(40) ? 1 : 2 + h_rmod(8);
      if (h_rpct(4)) d = 0x80000000u + h_rmod(0x7fffffffu);               /* radix >= 2^31: n_ huge, only for calc (getPower would run for ever) */
      memcpy(pm + 0x604 + 4 * k, &d, 4);
      if (d == 1) h_cov_hit(CV_CP_RADIX1);
      if (d > 128 && d < 301) h_cov_hit(CV_CP_RADIX_BIG);
    }
    if (c2) { cp_c2(o); h_cov_hit(CV_CP_C2); } else { cp_c1(o); h_cov_hit(CV_CP_CTOR); }
    DUMP(ctr, "cp ctor", o, CpObj);
    cp_calc(o, pm);
    h_cov_hit(CV_CP_CALC);
    model_calc(pm, &m);
    cp_obj_check(o, &m, "calcModulusParameters");
    DUMP(ctr, "cp calc", o, CpObj);
    if ((w620 + w0 - 6u) >= 64u) h_cov_hit(CV_CP_SHIFT_WRAP);
    if (m.M < 0) h_cov_hit(CV_CP_MNEG);
    if (m.A[5] >= 0x100000000ll) h_cov_hit(CV_CP_R5);
    if ((long double)pm_radix(pm, 0) * pm_radix(pm, 1) * pm_radix(pm, 2) * pm_radix(pm, 3) * pm_radix(pm, 4) > 18446744073709551616.0L) h_cov_hit(CV_CP_D2BIG);
    for (k = 0; k < 6; k++) acc = acc * 1000003u + (uint64_t)(uint32_t)o->r_[k] + (uint64_t)o->A_[k];
    /* getConstellationInfo: every k incl. the out-of-range ones, every point value */
    for (i = 0; i < 4; i++) {
      unsigned kk = h_rpct(25) ? (h_rpct(30) ? (h_rpct(50) ? 0xffffffffu : 0x80000000u + h_rmod(8)) : 6 + h_rmod(5)) : h_rmod(6);   /* k > 5 unsigned: also the values a signed compare would let through */
      int point = (int)(i == 0 ? 0 : i == 1 ? 1 : h_rpct(50) ? 2 : -1);
      const uint8_t *before_tab = (const uint8_t *)o->tab_;
      unsigned before_n = o->n_;
      cp_info(o, pm, point, kk);
      h_cov_hit(CV_CP_INFO);
      if (kk <= 5) {
        if ((const uint8_t *)o->tab_ != pm + (point == 1 ? 0x304u : 0x4u) + 0x80u * kk || o->n_ != pm_radix(pm, kk)) h_die3("getConstellationInfo(k %u, point %d): tab_ / n_ wrong", kk, point);
      } else {
        if ((const uint8_t *)o->tab_ != before_tab || o->n_ != before_n) h_die3("getConstellationInfo(k %u) modified the object", kk);
        h_cov_hit(CV_CP_INFO_OOR);
      }
      if (point == 1) h_cov_hit(CV_CP_INFO_HI);
      if (point != 0 && point != 1) h_cov_hit(CV_CP_POINT_ODD);
      DUMP(ctr, "cp info", o, CpObj);
    }
    /* getPower: only for radices that keep the loop short (every radix < 2^31 is fine: the loop length is the radix, so skip >= 2^31) */
    { int ok = 1; for (k = 0; k < 6; k++) if (pm_radix(pm, k) > 300) ok = 0;
      if (ok) {
        int point = h_rpct(50) ? (int)h_rmod(2) : (h_rpct(50) ? 2 : -1), pcm = h_rpct(50) ? (int)h_rmod(2) : (h_rpct(50) ? 2 : -1);
        float fr;
        uint32_t got, want;
        CpModel m2;
        if (h_rpct(60)) {   /* a STALE object: the state of another parameter block (or the canonical garbage): getPower must recompute everything itself */
          if (h_rpct(50)) { uint8_t *pm2 = (uint8_t *)sysdep_malloc(PM_SIZE); unsigned q2; for (q2 = 0; q2 < PM_SIZE; q2++) pm2[q2] = (uint8_t)h_rnd();
            for (q2 = 0; q2 < 6; q2++) { unsigned d2 = 1 + h_rmod(100); memcpy(pm2 + 0x604 + 4 * q2, &d2, 4); }
            { uint32_t a2 = h_rmod(30), b2 = h_rmod(30); memcpy(pm2, &a2, 4); memcpy(pm2 + 0x620, &b2, 4); }
            cp_calc(o, pm2); sysdep_free(pm2); }
          else memset(o, 0xa5, sizeof(CpObj));
        }
        fr = cp_power(o, pm, point, pcm);
        got = bits_f(fr);
        h_cov_hit(CV_CP_POWER);
        model_calc(pm, &m2);
        cp_obj_check(o, &m2, "getPower (object after)");
        want = model_power(pm, &m2, point, pcm);
        if (got != want) h_die3("getPower(point %d, pcm %d): %08x, model %08x (M %lld)", point, pcm, got, want, (long long)m2.M);
        h_emit(h_sample(&ctr, 12, 97), "cp power point %d pcm %d -> %08x", point, pcm, got);
        acc = acc * 1000003u + got;
        DUMP(ctr, "cp power obj", o, CpObj);
        h_cov_hit(pcm ? CV_CP_P_ALAW : CV_CP_P_ULAW);
        if (point == 1) h_cov_hit(CV_CP_P_HI);
        if (pcm != 0 && pcm != 1) h_cov_hit(CV_CP_PCM_ODD);
        if (point != 0 && point != 1) h_cov_hit(CV_CP_POINT_ODD);
        for (k = 0; k < 6; k++) { unsigned n = pm_radix(pm, k), rr = m2.r[k]; if (rr > 0 && n > 0) h_cov_hit(CV_CP_P_GT); if (rr < n) h_cov_hit(CV_CP_P_EQ); if (n > 0 && rr + 1 < n) h_cov_hit(CV_CP_P_LT); }
      }
    }
    if (c2) cp_d2(o); else cp_d1(o);
    DUMP(ctr, "cp dtor", o, CpObj);
    sysdep_free(pm);
    h_trial_end("cp");
  }
  /* getPowerIndexForPower: around every limit, specials, random */
  { uint32_t xs[600];
    unsigned nx = 0, q;
    static const uint32_t spec[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7fc12345u, 0x7f800000u, 0xff800000u, 0x00000001u, 0x3f800000u, 0xbf800000u, 0x7f7fffffu, 0x4b189680u, 0x4d000000u, 0x4e000000u,
                                     0x5f000000u };
    for (q = 0; q < 35; q++) { float f = (float)T_LIMITS[q]; uint32_t b = bits_f(f); xs[nx++] = b; xs[nx++] = b - 1; xs[nx++] = b + 1; xs[nx++] = b + 2; xs[nx++] = b - 2; }
    for (q = 0; q < sizeof spec / sizeof spec[0]; q++) xs[nx++] = spec[q];
    while (nx < 600) { uint32_t w = (h_rnd() & 0x80000000u) | ((uint32_t)(145 + h_rmod(20)) << 23) | (h_rnd() & 0x7fffffu); if (h_rpct(10)) w &= 0x7fffffffu; xs[nx++] = w; }
    CpObj *o = (CpObj *)onew(sizeof(CpObj));
    cp_c1(o);
    for (q = 0; q < nx; q++) {
      uint32_t w = xs[q];
      unsigned got = cp_pidx(o, FARG(w)), want = 0, i;
      float fx;
      memcpy(&fx, &w, 4);
      for (i = 34; i > 0; i--) if ((LD)T_LIMITS[i] >= (LD)fx) break;
      want = i;
      /* NaN: (LD)limit >= NaN is false for every i: the loop runs down to 0 */
      if (got != want) h_die3("getPowerIndexForPower(%08x) = %u, model %u", w, got, want);
      h_cov_hit(CV_CP_PIDX);
      h_cov_hit(got == 34 ? CV_CP_PIDX_TOP : got == 0 ? CV_CP_PIDX_ZERO : CV_CP_PIDX_MID);
      if (is_nan_w(w)) h_cov_hit(CV_CP_PIDX_NAN);
      for (i = 0; i < 35; i++) if ((float)T_LIMITS[i] == fx) { h_cov_hit(CV_CP_PIDX_EQ); break; }
      acc = acc * 1000003u + got;
      h_emit(h_sample(&ctr, 12, 53), "cp pidx %08x -> %u", w, got);
    }
    DUMP(ctr, "cp pidx obj", o, CpObj);
    for (q = 0; q < 80; q++) {
      unsigned idx = q < 70 ? q : q == 70 ? 0xffffffffu : q == 71 ? 0x80000000u : q == 72 ? 0x7fffffffu : q == 73 ? 0x22u : q == 74 ? 0x23u : q == 75 ? 0x21u : 100u + q;
      unsigned char r = cp_legal(o, idx);
      if (r != (idx <= 34u ? 1u : 0u)) h_die3("isLegalPowerIndex(%u) = %u", idx, (unsigned)r);
      h_cov_hit(CV_CP_LEGAL);
      if (!r) h_cov_hit(CV_CP_LEGAL_NO);
      if (idx == 34 || idx == 35) h_cov_hit(CV_CP_LEGAL_EDGE);
      h_emit(h_sample(&ctr, 12, 17), "cp legal %u -> %u", idx, (unsigned)r);
    }
  }
  h_emit(1, "cp hash %016llx", (unsigned long long)acc);
  h_trial_end("cp");
}

/* ================================================================================================ (C) float word helpers */
static uint32_t w_of_ld_near(LD v, int delta) {   /* the float word nearest to v (round to nearest even), moved by `delta` ulps in the bit pattern */
  float f = (float)v;
  uint32_t w = bits_f(f);
  if (is_nan_w(w)) return w;
  w = (uint32_t)((int64_t)w + delta);
  return w;
}
static uint32_t rnd_float_word(int regime) {
  uint32_t r = h_rnd(), s = r & 0x80000000u;
  switch (regime) {
    case 0: return s | ((uint32_t)(126 - h_rmod(6)) << 23) | (h_rnd() & 0x7fffffu);              /* ~ 0.03 .. 1 */
    case 1: return bits_f((float)(int)h_rrange(-9, 9));                                          /* small integers: exact sums */
    case 2: return s | ((uint32_t)(127 + h_rrange(-3, 3)) << 23);                                /* powers of two */
    case 3: return s | ((uint32_t)(127 + h_rrange(-30, 30)) << 23) | (h_rnd() & 0x7fffffu);      /* wide */
    case 4: return 0u;
    default: return s | ((uint32_t)(150 + h_rmod(60)) << 23) | (h_rnd() & 0x7fffffu);            /* huge */
  }
}

/* ================================================================================================ (C) scenario 7: V90SdDetector */
typedef struct {
  uint32_t h[12];
  unsigned n, count, required;
  uint32_t thA, thB, thC;
} SdModel;
/* the decision procedure of the archaeology file, on extended values; returns the result, updates count; A / q are reported for the coverage */
static int sd_model_step(SdModel *m, uint32_t xw, LD *Aout, LD *qout, int *path) {
  LD A = 0, B = 0, q;
  unsigned i, k;
  float thA, thB, thC;
  if (m->n >= 2) for (k = m->n - 1; k >= 1; k--) m->h[k] = m->h[k - 1];
  m->h[0] = xw;
  for (i = 0; i < 6; i++) { float a = f_of(m->h[i]), b = f_of(m->h[i + 6]); A += (LD)a * (LD)a; B += (LD)a * (LD)b; }
  thA = f_of(m->thA); thB = f_of(m->thB); thC = f_of(m->thC);
  *Aout = A;
  q = B / A;                 /* always computed here (the blob divides only after the thA test: the quotient is not observable before) */
  *qout = q;
  if (thA > A) { m->count = 0; *path = 0; return 0; }
  if (thB >= q) {
    if (thC > q) { *path = 1; return -1; }
    m->count = 0; *path = 2; return 0;
  }
  m->count++;
  *path = 3;
  return m->count >= m->required ? 1 : 0;
}
static void sd_check(const SdObj *o, const SdModel *m, const char *what) {
  unsigned i;
  uint32_t hw[12];
  if (o->count_ != m->count) h_die3("%s: count_ %u, model %u", what, o->count_, m->count);
  if (o->required_ != m->required || bits_f(o->thA_) != m->thA || bits_f(o->thB_) != m->thB || bits_f(o->thC_) != m->thC) h_die3("%s: thresholds / required differ from the model", what);
  memcpy(hw, o->hist_, 48);
  for (i = 0; i < 12; i++) if (hw[i] != m->h[i]) h_die3("%s: hist[%u] = %08x, model %08x", what, i, hw[i], m->h[i]);
}

static void sc_sd(int specials) {
  unsigned long ctr = 0;
  unsigned t;
  uint64_t acc = 0;
  cov_def(CV_SD_C2, "C2 / D2 used"); cov_def(CV_SD_CTOR, "ctor"); cov_def(CV_SD_PATH0, "thA > A: count cleared, 0"); cov_def(CV_SD_PATH1, "thB >= q and thC > q: -1"); cov_def(CV_SD_PATH2, "thB >= q, thC <= q: count cleared, 0");
  cov_def(CV_SD_PATH3A, "count path, below required: 0"); cov_def(CV_SD_PATH3B, "count path, reached required: 1"); cov_def(CV_SD_EQ_A, "thA == A exactly"); cov_def(CV_SD_EQ_B, "thB == q exactly");
  cov_def(CV_SD_EQ_C, "thC == q exactly"); cov_def(CV_SD_COUNT_EQ, "count + 1 == required"); cov_def(CV_SD_COUNT_GT, "count + 1 > required"); cov_def(CV_SD_COUNT_LT, "count + 1 < required");
  cov_def(CV_SD_RESET, "reset"); cov_def(CV_SD_NAN, "NaN q / threshold"); cov_def(CV_SD_ZEROA, "A == 0"); cov_def(CV_SD_NSMALL, "n_ poked below 12"); cov_def(CV_SD_NULLDTOR, "dtor with a NULL history");
  cov_def(CV_SD_SNAN, "signalling NaN words (threshold arguments / samples)"); cov_def(CV_SD_INF, "inf sample / threshold"); cov_def(CV_SD_DENORM, "denormal sample"); cov_def(CV_SD_REQ0, "required == 0");
  cov_def(CV_SD_REQBIG, "required == 0xffffffff"); cov_def(CV_SD_PROBE, "store-order probe");
  h_scenario_begin(1);
  if (!specials) { h_covreq[CV_SD_SNAN] = 0; h_covreq[CV_SD_INF] = 0; }
  for (t = 0; t < (specials ? 600u : 1500u); t++) {
    SdObj *o = (SdObj *)onew(sizeof(SdObj));
    SdModel m;
    unsigned i, steps = 40 + h_rmod(300), regime = h_rmod(6), c2 = h_rpct(25), req;
    uint32_t wa, wb, wc;
    static const uint32_t spw[] = { 0x7f800001u, 0xff800001u, 0x7fa00000u, 0x7fc00000u, 0xffc00001u, 0x7f800000u, 0xff800000u, 0x00000001u, 0x80000000u, 0x00000000u };
    wa = rnd_float_word(h_rmod(4)); wb = rnd_float_word(h_rmod(4)); wc = rnd_float_word(h_rmod(4));
    if (specials) { wa = spw[h_rmod(10)]; wb = spw[h_rmod(10)]; wc = spw[h_rmod(10)]; }
    req = h_rpct(60) ? 1 + h_rmod(6) : h_rpct(30) ? h_rmod(3) : h_rpct(50) ? 0xffffffffu : h_rnd();
    if (is_snan_w(wa) || is_snan_w(wb) || is_snan_w(wc)) h_cov_hit(CV_SD_SNAN);
    if (req == 0) h_cov_hit(CV_SD_REQ0);
    if (req == 0xffffffffu) h_cov_hit(CV_SD_REQBIG);
    { int probe = h_rpct(15);
      if (probe) { H_CTOR_PROBE(o, f_SdObj, NF_SdObj, sizeof(SdObj), (c2 ? sd_c2 : sd_c1)(o, FARG(wa), FARG(wb), FARG(wc), req)); h_cov_hit(CV_SD_PROBE); }
      else (c2 ? sd_c2 : sd_c1)(o, FARG(wa), FARG(wb), FARG(wc), req);
    }
    if (c2) h_cov_hit(CV_SD_C2);
    h_cov_hit(CV_SD_CTOR);
    DUMP(ctr, "sd ctor", o, SdObj);
    memset(&m, 0, sizeof m);
    m.n = 12; m.required = req; m.thA = wa; m.thB = wb; m.thC = wc;
    sd_check(o, &m, "SdDetector ctor");
    for (i = 0; i < steps; i++) {
      uint32_t xw;
      LD A, q;
      int path, want, got;
      unsigned kind = h_rmod(40);
      if (kind == 0) {   /* reset: also with a smaller n_ */
        if (h_rpct(30)) { unsigned nn = h_rpct(30) ? h_rmod(2) : 2 + h_rmod(11); o->n_ = nn; m.n = nn; h_cov_hit(CV_SD_NSMALL); }   /* n_ = 0 / 1: process() would run away, so n_ is restored right after the reset */
        sd_reset(o);
        if (m.n > 0) { unsigned k; for (k = 0; k < m.n; k++) m.h[k] = 0; }
        m.count = 0;
        h_cov_hit(CV_SD_RESET);
        sd_check(o, &m, "SdDetector reset");
        if (m.n < 2 || h_rpct(60)) { o->n_ = 12; m.n = 12; }
        DUMP(ctr, "sd reset", o, SdObj);
        continue;
      }
      if (kind == 1 && !specials) { /* rarely: the whole history from other data */ regime = h_rmod(6); }
      if (specials && h_rpct(25)) xw = spw[h_rmod(10)];
      else xw = rnd_float_word(h_rpct(8) ? 4 : (int)regime);
      if (!specials && h_rpct(3)) xw = 0x00000001u + h_rmod(3);   /* denormals */
      if ((xw & 0x7f800000u) == 0x7f800000u) h_cov_hit(CV_SD_INF);
      if ((xw & 0x7f800000u) == 0 && (xw & 0x7fffffu)) h_cov_hit(CV_SD_DENORM);
      if (is_snan_w(xw)) h_cov_hit(CV_SD_SNAN);
      /* preview the next step on a copy to place the thresholds on the boundaries of A and q (neither depends on the thresholds) */
      { SdModel pv = m; LD pa, pq; int pp;
        (void)sd_model_step(&pv, xw, &pa, &pq, &pp);
        A = pa; q = pq;
        if (h_rpct(60) && !specials) {
          unsigned sel = h_rmod(5), sb = h_rmod(6), sc = h_rmod(6);
          m.thA = sel == 0 ? w_of_ld_near(A, 0) : sel == 1 ? w_of_ld_near(A, h_rrange(-2, 2)) : sel == 2 ? 0u : sel == 3 ? m.thA : bits_f(1e30f);
          m.thB = sb == 0 ? w_of_ld_near(q, 0) : sb == 1 ? w_of_ld_near(q, h_rrange(-2, 2)) : sb == 2 ? bits_f(1e30f) : sb == 3 ? bits_f(-1e30f) : sb == 4 ? m.thB : 0x00000000u;
          m.thC = sc == 0 ? w_of_ld_near(q, 0) : sc == 1 ? w_of_ld_near(q, h_rrange(-2, 2)) : sc == 2 ? bits_f(1e30f) : sc == 3 ? bits_f(-1e30f) : sc == 4 ? m.thC : 0x00000000u;
          memcpy(&o->thA_, &m.thA, 4); memcpy(&o->thB_, &m.thB, 4); memcpy(&o->thC_, &m.thC, 4);
        }
        if (specials && h_rpct(30)) { m.thA = spw[h_rmod(10)]; m.thB = spw[h_rmod(10)]; m.thC = spw[h_rmod(10)]; memcpy(&o->thA_, &m.thA, 4); memcpy(&o->thB_, &m.thB, 4); memcpy(&o->thC_, &m.thC, 4); }
        if (h_rpct(25)) { unsigned r2 = m.count + 1; unsigned pick = h_rmod(6); m.required = pick == 0 ? r2 : pick == 1 ? r2 + 1 : pick == 2 ? (r2 > 0 ? r2 - 1 : 0) : pick == 3 ? 0u : pick == 4 ? 0xffffffffu : 1 + h_rmod(5); o->required_ = m.required; if (m.required == 0) h_cov_hit(CV_SD_REQ0); }
        if (h_rpct(10)) { m.count = h_rmod(7); o->count_ = m.count; }
        if (h_rpct(2)) { m.count = 0xfffffffeu + h_rmod(2); o->count_ = m.count; }
      }
      want = sd_model_step(&m, xw, &A, &q, &path);
      got = sd_proc(o, FARG(xw));
      if (got != want) h_die3("V90SdDetector::process: got %d, model %d (path %d, A %Lg q %Lg)", got, want, path, A, q);
      sd_check(o, &m, "SdDetector process");
      h_emit(h_sample(&ctr, 12, 1499), "sd step %08x -> %d cnt %u", xw, got, o->count_);
      acc = acc * 1000003u + (uint64_t)(unsigned)got * 7u + o->count_;
      h_cov_hit(path == 0 ? CV_SD_PATH0 : path == 1 ? CV_SD_PATH1 : path == 2 ? CV_SD_PATH2 : (got ? CV_SD_PATH3B : CV_SD_PATH3A));
      {
        float thA = f_of(m.thA), thB = f_of(m.thB), thC = f_of(m.thC);
        if (thA == A) h_cov_hit(CV_SD_EQ_A);
        if (path >= 1 && thB == q) h_cov_hit(CV_SD_EQ_B);
        if (path >= 1 && thC == q) h_cov_hit(CV_SD_EQ_C);
        if (path >= 1 && (q != q || thB != thB || thC != thC)) h_cov_hit(CV_SD_NAN);
        if (path == 0 && thA != thA) h_cov_hit(CV_SD_NAN);
        if (A == 0) h_cov_hit(CV_SD_ZEROA);
      }
      if (path == 3) { if (m.count == m.required) h_cov_hit(CV_SD_COUNT_EQ); else if (m.count > m.required) h_cov_hit(CV_SD_COUNT_GT); else h_cov_hit(CV_SD_COUNT_LT); }
    }
    DUMP(ctr, "sd end", o, SdObj);
    /* destruction: D1 / D2, sometimes with a NULL history (the blob frees only a non-NULL block) */
    if (h_rpct(15)) { sysdep_free(o->hist_); o->hist_ = NULL; h_cov_hit(H_COV_DTOR_NULL); h_cov_hit(CV_SD_NULLDTOR); }
    if (c2) sd_d2(o); else sd_d1(o);
    h_trial_end("sd");
  }
  h_emit(1, "sd hash %016llx", (unsigned long long)acc);
}

/* ================================================================================================ (C) scenario 8: V90RDetector */
typedef struct { unsigned cnt, lim[4], c[3]; uint16_t sr; int32_t dir; } RdModel;
static void rd_model_load(RdModel *m, const RdObj *o) {
  m->cnt = o->cnt_; m->lim[0] = o->lim0_; m->lim[1] = o->lim1_; m->lim[2] = o->lim2_; m->lim[3] = o->lim3_;
  m->c[0] = o->c0_; m->c[1] = o->c1_; m->c[2] = o->c2_; m->sr = o->sr_; m->dir = o->dir_;
}
/* the four decision procedures, table driven (which: 0 detectR, 1 detectRNot, 2 detectRf, 3 detectRfNot); *cls = the outcome class for the coverage */
static int rd_model_call(RdModel *m, int which, int16_t x, int *cls) {
  const int fine = (which >= 2);                         /* 12 decisions instead of 6 */
  const unsigned len = fine ? 12u : 6u;
  const unsigned patA = fine ? 0xccc : 0x38, patB = fine ? 0x333 : 0x7;   /* patA: "positive" (dir = +1), patB: "negative" (dir = -1) */
  const unsigned lim = which == 0 ? m->lim[0] : which == 1 ? m->lim[1] : which == 2 ? m->lim[2] : m->lim[3];
  int ret = 0;
  m->sr = (uint16_t)((unsigned)m->sr * 2u + (x > 0 ? 1u : 0u));
  if (m->cnt + 1 != len) { m->cnt = m->cnt + 1; *cls = 0; return 0; }
  if (which % 2 == 0) {                                  /* detectR / detectRf */
    if (m->sr == patA) { m->c[1] = 0; m->c[0] += len; if (m->c[0] == lim) { m->dir = 1; ret = 1; *cls = 2; } else *cls = 1; }
    else if (m->sr == patB) { m->c[0] = 0; m->c[1] += len; if (m->c[1] == lim) { m->dir = -1; ret = 1; *cls = 4; } else *cls = 3; }
    else { m->c[0] = 0; m->c[1] = 0; *cls = 5; }
  } else {                                               /* detectRNot / detectRfNot: target pattern by the direction */
    const unsigned target = m->dir > 0 ? patB : patA;
    if (m->sr == target) { m->c[2] += len; if (m->c[2] == lim) { ret = -1; *cls = 2; } else *cls = 1; }
    else { m->c[2] = 0; *cls = 5; }
  }
  m->cnt = 0;
  m->sr = 0;
  return ret;
}
static void rd_check(const RdObj *o, const RdModel *m, const char *what) {
  RdModel a;
  rd_model_load(&a, o);
  if (a.cnt != m->cnt || a.c[0] != m->c[0] || a.c[1] != m->c[1] || a.c[2] != m->c[2] || a.sr != m->sr || a.dir != m->dir || a.lim[0] != m->lim[0] || a.lim[1] != m->lim[1] || a.lim[2] != m->lim[2] || a.lim[3] != m->lim[3])
    h_die3("%s: object (cnt %u c %u %u %u sr %04x dir %d lim %u %u %u %u) differs from the model (cnt %u c %u %u %u sr %04x dir %d lim %u %u %u %u)", what, a.cnt, a.c[0], a.c[1], a.c[2], a.sr, a.dir, a.lim[0], a.lim[1], a.lim[2], a.lim[3],
           m->cnt, m->c[0], m->c[1], m->c[2], m->sr, m->dir, m->lim[0], m->lim[1], m->lim[2], m->lim[3]);
}
static void rd_model_reset(RdModel *m, unsigned a, unsigned b) {
  m->cnt = 0; m->c[0] = m->c[1] = m->c[2] = 0; m->dir = 1; m->sr = 0;
  m->lim[0] = (a / 6) * 6; m->lim[2] = (a / 12) * 12; m->lim[1] = (b / 6) * 6; m->lim[3] = (b / 12) * 12;
}

static void sc_rd(void) {
  static const unsigned avals[] = { 0, 1, 5, 6, 7, 11, 12, 13, 17, 18, 23, 24, 25, 35, 36, 60, 71, 72, 100, 1000, 0xfffffffbu, 0xffffffffu, 0x80000000u };
  unsigned long ctr = 0;
  unsigned t;
  uint64_t acc = 0;
  cov_def(CV_RD_CTOR, "ctor (stores params_ only)"); cov_def(CV_RD_C2, "C2 / D2 used"); cov_def(CV_RD_RESET, "reset"); cov_def(CV_RD_GARBAGE, "garbage in the upper argument bits");
  cov_def(CV_RD_PRE_RESET, "detect on the un-reset object"); cov_def(CV_RD_SR_HIGH, "sr_ with bits above the window"); cov_def(CV_RD_DIR_NEG, "dir_ < 0"); cov_def(CV_RD_DIR_ZERO, "dir_ == 0");
  cov_def(CV_RD_DIR_BIG, "dir_ INT_MIN / > 1"); cov_def(CV_RD_LIM_EQ, "counter == limit"); cov_def(CV_RD_XNEG, "x < 0"); cov_def(CV_RD_XZERO, "x == 0"); cov_def(CV_RD_XMIN, "x == -32768");
  cov_def(CV_RD_XMAX, "x == 32767"); cov_def(CV_RD_RET1, "returns 1"); cov_def(CV_RD_RETM1, "returns -1"); cov_def(CV_RD_CNTPOKE, "cnt_ poked"); cov_def(CV_RD_LIMPOKE, "counter poked next to its limit");
  { int f, c;
    for (f = 0; f < 4; f++) for (c = 0; c < 6; c++) cov_def(32 + f * 6 + c, "outcome class");
    for (f = 0; f < 4; f++) { h_covreq[32 + f * 6 + 0] = 1; }
    /* the classes that do not exist: the "Not" functions have no class 3 / 4 (negative-pattern) */
    for (f = 1; f < 4; f += 2) { h_covreq[32 + f * 6 + 3] = 0; h_covreq[32 + f * 6 + 4] = 0; }
  }
  h_scenario_begin(0);
  relax_reserved();
  for (t = 0; t < 4000; t++) {
    RdObj *o = (RdObj *)onew(sizeof(RdObj));
    uint8_t *prm = (uint8_t *)sysdep_malloc(64);
    RdModel m;
    unsigned c2 = h_rpct(30), steps = 80 + h_rmod(400), i, a = avals[h_rmod(sizeof avals / sizeof avals[0])], b = avals[h_rmod(sizeof avals / sizeof avals[0])], pref = h_rmod(4);
    unsigned pending = 0, pat = 0, patlen = 0;
    if (h_rpct(30)) { a = 6 * (1 + h_rmod(4)) + (h_rpct(30) ? 6 * h_rmod(2) : 0); b = 6 * (1 + h_rmod(4)); }
    if (c2) { rd_c2(o, prm); h_cov_hit(CV_RD_C2); } else rd_c1(o, prm);
    h_cov_hit(CV_RD_CTOR);
    DUMP(ctr, "rd ctor", o, RdObj);
    { void *pp; memcpy(&pp, &o->params_, sizeof pp); if (pp != (void *)prm) h_die3("V90RDetector ctor: params_ not stored"); }
    { unsigned k; for (k = 0; k < 0x28; k++) if (((uint8_t *)o)[k] != 0xa5) h_die3("V90RDetector ctor modified a field other than params_"); }
    if (h_rpct(8)) { rd_model_load(&m, o); h_cov_hit(CV_RD_PRE_RESET); }   /* detect on the garbage fill */
    else {
      rd_reset(o, a, b);
      rd_model_reset(&m, a, b);
      h_cov_hit(CV_RD_RESET);
      rd_check(o, &m, "V90RDetector reset");
      DUMP(ctr, "rd reset", o, RdObj);
    }
    for (i = 0; i < steps; i++) {
      int which = h_rpct(70) ? (int)pref : (int)h_rmod(4), cls, got, want, ix, sr_new_hi = 0;
      int16_t x;
      unsigned bit;
      if (h_rpct(4)) { unsigned k = h_rmod(7); o->cnt_ = m.cnt = k == 0 ? 5 : k == 1 ? 11 : k == 2 ? 0 : k == 3 ? 6 : k == 4 ? 12 : k == 5 ? 7 : h_rnd(); h_cov_hit(CV_RD_CNTPOKE); }
      if (h_rpct(4)) { unsigned k = h_rmod(5); o->sr_ = m.sr = (uint16_t)(k == 0 ? 0x8038 : k == 1 ? 0x0100 : k == 2 ? 0xffff : k == 3 ? 0x0007 : h_rnd()); }
      if (h_rpct(4)) { unsigned k = h_rmod(6); int32_t dv = k == 0 ? 1 : k == 1 ? -1 : k == 2 ? 0 : k == 3 ? (int32_t)0x80000000 : k == 4 ? 0x7fffffff : 2; o->dir_ = m.dir = dv; }
      if (h_rpct(6)) { /* a counter next to its limit */
        unsigned k = h_rmod(3);
        unsigned len = which >= 2 ? 12u : 6u, lim = m.lim[which];
        unsigned *cp = &m.c[k];
        *cp = lim - len + (h_rpct(30) ? 1u : 0u);
        if (k == 0) o->c0_ = *cp; else if (k == 1) o->c1_ = *cp; else o->c2_ = *cp;
        h_cov_hit(CV_RD_LIMPOKE);
      }
      /* the next decision: from a pattern block with probability 1/2, else a coin */
      if (pending == 0 && h_rpct(55)) {
        int fine = which >= 2;
        patlen = fine ? 12 : 6;
        pat = h_rpct(50) ? (fine ? 0xccc : 0x38) : (fine ? 0x333 : 0x7);
        if (h_rpct(10)) pat ^= 1u << h_rmod(patlen);
        pending = patlen;
      }
      if (pending) { bit = (pat >> (pending - 1)) & 1u; pending--; } else bit = h_rmod(2);
      { unsigned k = h_rmod(20);
        x = bit ? (int16_t)(k == 0 ? 32767 : k == 1 ? 1 : 1 + h_rmod(32767)) : (int16_t)(k == 0 ? -32768 : k == 1 ? 0 : k == 2 ? -1 : -(int)h_rmod(32769)); }
      ix = (int)((uint32_t)(uint16_t)x | (h_rpct(60) ? (h_rnd() << 16) : 0u));
      if (ix >> 16 != (x < 0 ? -1 : 0) && (uint32_t)ix >> 16) h_cov_hit(CV_RD_GARBAGE);
      { unsigned srnew = (((unsigned)m.sr * 2u) + (x > 0 ? 1u : 0u)) & 0xffffu; sr_new_hi = srnew > 0x3f; }
      want = rd_model_call(&m, which, x, &cls);
      got = which == 0 ? rd_r(o, ix) : which == 1 ? rd_rn(o, ix) : which == 2 ? rd_rf(o, ix) : rd_rfn(o, ix);
      if (got != want) h_die3("V90RDetector function %d: got %d, model %d (x %d)", which, got, want, (int)x);
      rd_check(o, &m, "V90RDetector detect");
      h_emit(h_sample(&ctr, 12, 1999), "rd f%d x %d -> %d cnt %u sr %04x", which, (int)x, got, o->cnt_, o->sr_);
      acc = acc * 1000003u + (uint64_t)(unsigned)(got + 2) * 13u + o->cnt_ + o->c0_ + o->c1_ * 3u + o->c2_ * 5u;
      h_cov_hit(32 + which * 6 + cls);
      if (sr_new_hi && cls != 0) h_cov_hit(CV_RD_SR_HIGH);
      if (m.dir < 0) h_cov_hit(CV_RD_DIR_NEG); else if (m.dir == 0) h_cov_hit(CV_RD_DIR_ZERO); else if (m.dir > 1) h_cov_hit(CV_RD_DIR_BIG);
      if (m.dir == (int32_t)0x80000000) h_cov_hit(CV_RD_DIR_BIG);
      if (got == 1) h_cov_hit(CV_RD_RET1);
      if (got == -1) h_cov_hit(CV_RD_RETM1);
      if (x < 0) h_cov_hit(CV_RD_XNEG); else if (x == 0) h_cov_hit(CV_RD_XZERO);
      if (x == -32768) h_cov_hit(CV_RD_XMIN);
      if (x == 32767) h_cov_hit(CV_RD_XMAX);
      if (cls == 2 || cls == 4) h_cov_hit(CV_RD_LIM_EQ);
      if (which % 2 == 1 && cls == 2) h_cov_hit(CV_RD_LIM_EQ);
    }
    DUMP(ctr, "rd end", o, RdObj);
    if (c2) rd_d2(o); else rd_d1(o);
    sysdep_free(prm);
    h_trial_end("rd");
  }
  h_emit(1, "rd hash %016llx", (unsigned long long)acc);
}

/* ================================================================================================ (C) scenarios 9 / 10 / 12: V90SpectralShapingFilter */
typedef struct { LD s0, s1, s2, s3; } SfM;
/* the difference equation of the archaeology file, one input sample, all extended */
static void sf_model_step(SfM *st, LD c0, LD c1, LD c2, LD c3, int16_t in) {
  LD x = (LD)in;
  LD t6 = c0 * st->s1 + (x - st->s0 * c2);
  LD t8 = st->s2 * c1 + (t6 - st->s1 * c3);
  st->s3 = st->s3 + t8 * t8;
  st->s0 = x; st->s1 = t6; st->s2 = t8;
}
static LD sf_model_metric(const SfObj *o, const int16_t *in, unsigned n) {
  SfM st;
  unsigned k, b;
  st.s0 = f_of(bits_f(o->s0_)); st.s1 = o->s1_; st.s2 = o->s2_; st.s3 = o->s3_;
  for (b = 0; b < n && o->N_ > 0; b++) for (k = 0; k < o->N_; k++) sf_model_step(&st, o->c0_, o->c1_, o->c2_, o->c3_, *in++);
  return st.s3;
}
static void sf_model_progress(SfObj *dst, const SfObj *o, const int16_t *in) {   /* the expected object after progress() (floats rounded at the end) */
  SfM st;
  unsigned k;
  *dst = *o;
  if (o->N_ == 0) return;
  st.s0 = o->s0_; st.s1 = o->s1_; st.s2 = o->s2_; st.s3 = o->s3_;
  for (k = 0; k < o->N_; k++) sf_model_step(&st, o->c0_, o->c1_, o->c2_, o->c3_, in[k]);
  dst->s0_ = (float)st.s0; dst->s1_ = (float)st.s1; dst->s2_ = (float)st.s2; dst->s3_ = (float)st.s3;
}
static int sf_same(const SfObj *a, const SfObj *b, int nan_by_class) {   /* object comparison: bit patterns (NaN payload only compared when !nan_by_class) */
  const float *fa = &a->c0_, *fb = &b->c0_;
  int i;
  if (a->N_ != b->N_) return 0;
  for (i = 0; i < 8; i++) {
    uint32_t wa = bits_f(fa[i]), wb = bits_f(fb[i]);
    if (wa == wb) continue;
    if (nan_by_class && is_nan_w(wa) && is_nan_w(wb)) continue;
    return 0;
  }
  return 1;
}

static void sc_sf(int mode) {   /* mode 0: normal data, results compared as FLOAT bits; 1: i386 only, the EXTENDED metric as an 80-bit pattern; 2: special coefficients (inf / NaN / denormal) */
  unsigned long ctr = 0;
  unsigned t;
  uint64_t acc = 0;
  cov_def(CV_SF_CTOR, "ctor"); cov_def(CV_SF_C2, "C2 used"); cov_def(CV_SF_RESET, "reset"); cov_def(CV_SF_SET, "setFilterCoeff"); cov_def(CV_SF_PROG, "progress"); cov_def(CV_SF_METRIC, "getMetric");
  cov_def(CV_SF_N0, "N_ == 0"); cov_def(CV_SF_N1, "N_ == 1"); cov_def(CV_SF_N2, "N_ == 2 (default)"); cov_def(CV_SF_NBIG, "N_ >= 3"); cov_def(CV_SF_NM0, "getMetric n == 0"); cov_def(CV_SF_NM1, "getMetric n == 1");
  cov_def(CV_SF_NMBIG, "getMetric n >= 3"); cov_def(CV_SF_EXTDIFF, "extended metric differs from its float rounding"); cov_def(CV_SF_NONFIN, "non-finite state"); cov_def(CV_SF_BIGIN, "|input| >= 16384");
  cov_def(CV_SF_MININ, "input -32768"); cov_def(CV_SF_ZERO, "all-zero coefficients"); cov_def(CV_SF_CONST, "obj unchanged by getMetric"); cov_def(CV_SF_DEFAULTS, "ctor defaults checked"); cov_def(CV_SF_SPECIAL, "special coefficient words");
  cov_def(CV_SF_SNAN, "signalling NaN coefficient words"); cov_def(CV_SF_STATE_POKE, "state poked");
  if (mode != 1) h_covreq[CV_SF_EXTDIFF] = 0;
  if (mode != 2) { h_covreq[CV_SF_SPECIAL] = 0; h_covreq[CV_SF_SNAN] = 0; }
  h_scenario_begin(0);
  relax_reserved();
  for (t = 0; t < (mode == 2 ? 1500u : 2500u); t++) {
    SfObj *o = (SfObj *)onew(sizeof(SfObj)), exp;
    int16_t *in = (int16_t *)sysdep_malloc(64 * 2 + 16);
    unsigned c2 = h_rpct(30), steps = 8 + h_rmod(30), i, k, regime = h_rmod(5), amp = h_rmod(4);
    /* NaN words with a significand above 0x400000 only: the default NaN produced by inf - inf / 0 * inf (0xffc00000) then never TIES with an input NaN, whose choice depends on the operand order of the instruction (float rule 6) */
    static const uint32_t spw[] = { 0x7f800000u, 0xff800000u, 0x7fc12345u, 0xffe00000u, 0x00000001u, 0x007fffffu, 0x80000001u, 0x7f7fffffu, 0x7fa54321u, 0x7f800001u, 0x00000000u, 0x80000000u };
    uint32_t cw[4];
    if (c2) { sf_c2(o); h_cov_hit(CV_SF_C2); } else sf_c1(o);
    h_cov_hit(CV_SF_CTOR);
    DUMP(ctr, "sf ctor", o, SfObj);
    { unsigned kk; const uint32_t *w = (const uint32_t *)o;
      for (kk = 0; kk < 8; kk++) if (w[kk] != 0) h_die3("V90SpectralShapingFilter ctor: word %u is 0x%x", kk, w[kk]);
      if (o->N_ != 2) h_die3("V90SpectralShapingFilter ctor: N_ = %u", o->N_);
      h_cov_hit(CV_SF_DEFAULTS);
    }
    for (k = 0; k < 4; k++) cw[k] = mode == 2 ? (h_rpct(45) ? spw[h_rmod(12)] : rnd_float_word(h_rmod(4))) : (regime == 0 ? rnd_float_word(0) & 0xbfffffffu : regime == 1 ? rnd_float_word(1) : regime == 2 ? rnd_float_word(2) : regime == 3 ? rnd_float_word(h_rmod(4)) : rnd_float_word(h_rmod(5)));
    if (mode == 0 && regime == 0) for (k = 0; k < 4; k++) cw[k] = (cw[k] & 0x80000000u) | ((cw[k] & 0x7fffffffu) - (1u << 23) < 0x7f800000u ? (cw[k] & 0x7fffffffu) - (1u << 23) : (cw[k] & 0x7fffffffu));   /* halve: keeps the loop from blowing up */
    if (h_rpct(8)) { for (k = 0; k < 4; k++) cw[k] = 0; h_cov_hit(CV_SF_ZERO); }
    if (h_rpct(8)) { sf_reset(o); h_cov_hit(CV_SF_RESET); }
    sf_set(o, FARG(cw[0]), FARG(cw[1]), FARG(cw[2]), FARG(cw[3]));
    h_cov_hit(CV_SF_SET);
    { const uint32_t *w = (const uint32_t *)o; for (k = 0; k < 4; k++) if (w[k] != cw[k]) h_die3("setFilterCoeff: word %u is 0x%x, argument 0x%x", k, w[k], cw[k]); }
    for (k = 0; k < 4; k++) { if (is_snan_w(cw[k])) h_cov_hit(CV_SF_SNAN); if (mode == 2 && (is_nan_w(cw[k]) || !is_fin_w(cw[k]) || (cw[k] & 0x7f800000u) == 0)) h_cov_hit(CV_SF_SPECIAL); }
    DUMP(ctr, "sf set", o, SfObj);
    for (i = 0; i < steps; i++) {
      unsigned op = h_rmod(10);
      int16_t buf[64];
      unsigned nn, kk;
      for (kk = 0; kk < 64; kk++) {
        unsigned a = amp == 0 ? 10 : amp == 1 ? 1000 : amp == 2 ? 32767 : 4;
        buf[kk] = h_rpct(6) ? (int16_t)(h_rpct(50) ? -32768 : 32767) : (int16_t)((int)h_rmod(2 * a + 1) - (int)a);
        if (mode == 0 && amp == 3) buf[kk] = (int16_t)(h_rmod(3) - 1);
      }
      memcpy(in, buf, 128);
      if (h_rpct(6)) { unsigned nv = h_rpct(50) ? h_rmod(5) : 1 + h_rmod(8); o->N_ = nv; }   /* N_ other than the default */
      if (h_rpct(4)) { uint32_t *w = (uint32_t *)((uint8_t *)o + 0x10); for (kk = 0; kk < 4; kk++) if (h_rpct(60)) { w[kk] = mode == 2 ? spw[h_rmod(12)] : rnd_float_word(h_rmod(4)); if (is_snan_w(w[kk])) w[kk] |= 0x00400000u; } h_cov_hit(CV_SF_STATE_POKE); }   /* a stored state is never a signalling NaN (an x87 store quiets it) */
      if (o->N_ == 0) h_cov_hit(CV_SF_N0); else if (o->N_ == 1) h_cov_hit(CV_SF_N1); else if (o->N_ == 2) h_cov_hit(CV_SF_N2); else h_cov_hit(CV_SF_NBIG);
      for (kk = 0; kk < 64; kk++) { if (buf[kk] == -32768) h_cov_hit(CV_SF_MININ); if (buf[kk] >= 16384 || buf[kk] <= -16384) h_cov_hit(CV_SF_BIGIN); }
      if (op == 9 && h_rpct(35)) {   /* reset in the middle of a run: the four state words go back to zero, nothing else changes */
        SfObj before = *o;
        sf_reset(o);
        h_cov_hit(CV_SF_RESET);
        if (bits_f(o->s0_) | bits_f(o->s1_) | bits_f(o->s2_) | bits_f(o->s3_)) h_die3("V90SpectralShapingFilter::reset left a non-zero state word");
        if (memcmp(&before.c0_, &o->c0_, 16) || before.N_ != o->N_) h_die3("V90SpectralShapingFilter::reset modified the coefficients / N_");
        DUMP(ctr, "sf reset", o, SfObj);
        continue;
      }
      if (op < 4) {   /* progress */
        sf_model_progress(&exp, o, buf);
        sf_prog(o, in);
        h_cov_hit(CV_SF_PROG);
        if (!sf_same(o, &exp, mode == 2)) h_die3("V90SpectralShapingFilter::progress: state differs from the model (s %08x %08x %08x %08x, model %08x %08x %08x %08x)", bits_f(o->s0_), bits_f(o->s1_), bits_f(o->s2_), bits_f(o->s3_),
                                                bits_f(exp.s0_), bits_f(exp.s1_), bits_f(exp.s2_), bits_f(exp.s3_));
        DUMP(ctr, "sf prog", o, SfObj);
        acc = acc * 1000003u + bits_f(o->s3_) + bits_f(o->s1_) * 3u + bits_f(o->s2_) * 5u;
        for (kk = 0; kk < 4; kk++) { float f = (&o->s0_)[kk]; if (!is_fin_w(bits_f(f))) h_cov_hit(CV_SF_NONFIN); }
      } else {        /* getMetric */
        unsigned char before[sizeof(SfObj)];
        XRET r;
        LD want, got;
        nn = op < 6 ? h_rmod(2) : h_rmod(8);
        if (nn * (o->N_ > 0 ? o->N_ : 1) > 64) nn = 64 / (o->N_ ? o->N_ : 1);
        if (o->N_ > 0 && nn * o->N_ > 64) nn = 64 / o->N_;
        memcpy(before, o, sizeof before);
        r = sf_metric(o, in, nn);
        h_cov_hit(CV_SF_METRIC);
        if (memcmp(before, o, sizeof before)) h_die3("getMetric modified the object");
        h_cov_hit(CV_SF_CONST);
        want = sf_model_metric(o, buf, nn);
        got = (LD)r;
        if (nn == 0) h_cov_hit(CV_SF_NM0); else if (nn == 1) h_cov_hit(CV_SF_NM1); else h_cov_hit(CV_SF_NMBIG);
        if (mode == 1) {
          unsigned char pg[10], pw_[10];
          memset(pg, 0, 10); memset(pw_, 0, 10);
          memcpy(pg, &got, 10); memcpy(pw_, &want, 10);
          if (memcmp(pg, pw_, 10) && !(got != got && want != want)) h_die3("getMetric(n %u): extended result %Lg (%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x) differs from the model", nn, got, pg[9], pg[8], pg[7], pg[6], pg[5], pg[4], pg[3], pg[2], pg[1], pg[0]);
          h_emit(h_sample(&ctr, 12, 211), "sf metric n %u -> %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", nn, pg[9], pg[8], pg[7], pg[6], pg[5], pg[4], pg[3], pg[2], pg[1], pg[0]);
          if ((LD)(float)got != got) h_cov_hit(CV_SF_EXTDIFF);
          { uint64_t hh; memcpy(&hh, pg, 8); acc = acc * 1000003u + hh; }
        } else {
          float fg = (float)got, fw = (float)want;
          uint32_t wg = bits_f(fg), ww = bits_f(fw);
          if (wg != ww && !(is_nan_w(wg) && is_nan_w(ww) && mode == 2)) h_die3("getMetric(n %u): float result %08x, model %08x", nn, wg, ww);
          h_emit(h_sample(&ctr, 12, 211), "sf metric n %u -> %08x", nn, wg);
          acc = acc * 1000003u + wg;
        }
      }
    }
    sysdep_free(in);
    h_trial_end("sf");
  }
  h_emit(1, "sf hash %016llx", (unsigned long long)acc);
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0;
  const char *seed = getenv("P06_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  p06_layout_check();
  h_x87_check();
  h_arena_init();
  switch (sc) {
    case 0: sc_g711(); break;
    case 1: sc_stubs(); break;
    case 2: sc_mapper(); break;
    case 3: sc_modulus(); break;
    case 4: sc_phase2(0); break;
    case 5: sc_phase2(1); break;
    case 6: sc_power(); break;
    case 7: sc_sd(0); break;
    case 8: sc_rd(); break;
    case 9: sc_sf(0); break;
    case 10: sc_sf(1); break;
    case 11: sc_sd(1); break;
    case 12: sc_sf(2); break;

    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu edprintf %u dbgprintf %u", h_nmalloc, h_nfree, g_nedp, g_ndbg);
  h_cov_finish();
  return 0;
}
