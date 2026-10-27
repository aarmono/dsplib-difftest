/*
 * Differential test harness for port chunk P27 (src/vpcm/pcm_phase3_modulators.cpp): class V90Phase3Modulator (22 functions) and class V92Phase3Modulator (16): the phase-3 TX symbol generators.
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/pcm_phase3_modulators.sh against the blob (dsplibs.o with edprintf and the three Jd callees - V90Jd::getBitVector,
 * V92Jd::getJdBitVector, V92Jd::getJdPhaseBitVector (P14) - WEAK so that the recorders below win), against the port (-O0 / -O2; the P01 port object supplies Scrambler<uchar,int>, the P06 port
 * object the G.711 coders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/pcm_phase3_modulators.blob32.out).  The functions are reached through __asm__ labels; the objects are the generated
 * C mirrors (test/vpcm/pcm_phase3_modulators.layout), the V92Parameters block the P07 mirror, the DIL descriptor the P13 mirror.
 *
 * Scenarios (each its own process):
 *   0 V90Phase3Modulator V.90 session (flag 0): the whole state machine - natural runs through the states with the owner's exit* calls, forced states / counters around every threshold, debug levels
 *   1 V90Phase3Modulator V.92 session (flag != 0): the V.92 states, generateV92Symbol, the Jd / Jd-phase bit vectors, the time-outs
 *   2 the DIL generator: resetDILGenerator with random / edge descriptors, the DIL states 9 and 0xa of both symbol generators, generateDIL, updateCodeSegmentPointer, exitDIL
 *   3 V92Phase3Modulator: generateSymbol through every state, the exit* functions, reset (TRN1u length from the V92Parameters, Ja / DIL arguments, warm-up symbols), the dead generators
 *   4 constructors / destructors / setSessionFlag / reset argument edge cases / the dead V90 generators (generateSd, generateSdNot, generateJd, generateJdNot, generateTRN1d, generateJdPhase, generateV92Jd, exitJd, exitJdPhase)
 *
 * What is compared: every returned symbol (16 bit), the whole object after every operation (field-wise dump: sampled; the model fields every call), the raw DIL region (0x54 .. 0x394) against the model's
 * image, the canary bytes behind the object, every edprintf / dsplibs_debug_printf call (kind, text, a snapshot of the object at the call: store order before a print), every recorded callee call.
 * INDEPENDENT MODELS (written from the disassembly facts, raw byte offsets for the DIL region, a delay-line model of the scrambler, not from the port code): m9_* (V90Phase3Modulator) and m2_* (V92Phase3Modulator).
 * A model mismatch exits 3.  Calls that would crash the REAL blob (a NULL bit vector read, a zero Ja length) are excluded by the driver.
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
#include "pcm_dil_layout.h"
#include "pcm_phase3_modulators_layout.h"

#define L(m) __asm__(m)

/* the functions under test: V90Phase3Modulator */
extern void v9_c1(void *, void *, unsigned) L("_ZN18V90Phase3ModulatorC1EP13V90Parametersj");
extern void v9_c2(void *, void *, unsigned) L("_ZN18V90Phase3ModulatorC2EP13V90Parametersj");
extern void v9_d1(void *) L("_ZN18V90Phase3ModulatorD1Ev");
extern void v9_d2(void *) L("_ZN18V90Phase3ModulatorD2Ev");
extern void v9_setflag(void *, unsigned) L("_ZN18V90Phase3Modulator14setSessionFlagEj");
extern short v9_sd(void *) L("_ZN18V90Phase3Modulator10generateSdEv");
extern short v9_sdnot(void *) L("_ZN18V90Phase3Modulator13generateSdNotEv");
extern void v9_exitjd(void *) L("_ZN18V90Phase3Modulator6exitJdEv");
extern void v9_exitjdph(void *) L("_ZN18V90Phase3Modulator11exitJdPhaseEv");
extern void v9_exitdil(void *) L("_ZN18V90Phase3Modulator7exitDILEv");
extern void v9_updseg(void *) L("_ZN18V90Phase3Modulator24updateCodeSegmentPointerEv");
extern void v9_resetdil(void *, const void *) L("_ZN18V90Phase3Modulator17resetDILGeneratorEPK19tagV90DILdescriptor");
extern short v9_gendil(void *) L("_ZN18V90Phase3Modulator11generateDILEv");
extern short v9_jdnot(void *) L("_ZN18V90Phase3Modulator13generateJdNotEv");
extern short v9_jd(void *) L("_ZN18V90Phase3Modulator10generateJdEv");
extern short v9_trn1d(void *) L("_ZN18V90Phase3Modulator13generateTRN1dEv");
extern short v9_sym90(void *) L("_ZN18V90Phase3Modulator17generateV90SymbolEv");
extern short v9_jdph(void *) L("_ZN18V90Phase3Modulator15generateJdPhaseEv");
extern short v9_v92jd(void *) L("_ZN18V90Phase3Modulator13generateV92JdEv");
extern short v9_sym92(void *) L("_ZN18V90Phase3Modulator17generateV92SymbolEv");
extern short v9_sym(void *) L("_ZN18V90Phase3Modulator14generateSymbolEv");
extern void v9_reset(void *, int, unsigned, int, unsigned, void *, void *, const void *, unsigned)
    L("_ZN18V90Phase3Modulator5resetE7PcmTypeh20Phase3ModulatorStatejP5V90JdP5V92JdPK19tagV90DILdescriptorj");
extern int v9_table[16] L("_ZN18V90Phase3Modulator32codeSegmentsBoundriesLookupTableE");
/* V92Phase3Modulator */
extern void v2_c1(void *, void *) L("_ZN18V92Phase3ModulatorC1EP13V92Parameters");
extern void v2_c2(void *, void *) L("_ZN18V92Phase3ModulatorC2EP13V92Parameters");
extern void v2_d1(void *) L("_ZN18V92Phase3ModulatorD1Ev");
extern void v2_d2(void *) L("_ZN18V92Phase3ModulatorD2Ev");
extern void v2_exitja(void *) L("_ZN18V92Phase3Modulator6exitJaEv");
extern void v2_exitsil(void *) L("_ZN18V92Phase3Modulator11exitSilenceEv");
extern void v2_exitsu2(void *) L("_ZN18V92Phase3Modulator12exitSuSecondEv");
extern void v2_exittrn(void *) L("_ZN18V92Phase3Modulator9exitTRN1uEv");
extern short v2_ru(void *) L("_ZN18V92Phase3Modulator10generateRuEv");
extern short v2_runot(void *) L("_ZN18V92Phase3Modulator13generateRuNotEv");
extern short v2_su(void *) L("_ZN18V92Phase3Modulator10genereteSuEv");
extern short v2_sunot(void *) L("_ZN18V92Phase3Modulator13genereteSuNotEv");
extern short v2_ja(void *) L("_ZN18V92Phase3Modulator10generateJaEv");
extern short v2_trn1u(void *) L("_ZN18V92Phase3Modulator13generateTRN1uEv");
extern short v2_sym(void *) L("_ZN18V92Phase3Modulator14generateSymbolEv");
extern void v2_reset(void *, int, unsigned, unsigned, void *, const void *, unsigned) L("_ZN18V92Phase3Modulator5resetEs23V92Phase3ModulatorStatejP5V92JaPK19tagV90DILdescriptorj");
extern int linear2alaw(int);
extern int linear2ulaw(int);
extern int alaw2linear(int);
extern int ulaw2linear(int);
extern unsigned int dsplibs_debug_level;
/* the interposed callees (P14): defined below */
extern unsigned char *h_getbv(void *) L("_ZN5V90Jd12getBitVectorEv");
extern unsigned char *h_getjdbv(void *) L("_ZN5V92Jd14getJdBitVectorEv");
extern unsigned char *h_getjdphbv(void *) L("_ZN5V92Jd19getJdPhaseBitVectorEv");

typedef Ph3V90Obj O9;
typedef Ph3V92Obj O2;

#define SZ9 sizeof(O9)
#define SZ2 sizeof(O2)

/* ------------------------------------------------------------------------------------------------ recorders */
#define LOGMAX 24
#define TXTMAX 160
typedef struct { char kind; char text[TXTMAX]; unsigned st, cnt, ev, jdb; } LogEnt;
static LogEnt g_log[LOGMAX];
static unsigned g_nlog;
static unsigned long g_ntot_e, g_ntot_d, g_ntot_c;
static int g_show;
/* the object under test (snapshot at each print) */
static O9 *g_cur9;
static O2 *g_cur2;
static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}
static void record(char kind, const char *fmt, va_list ap) {
  char buf[TXTMAX * 2];
  LogEnt *e = &g_log[g_nlog < LOGMAX ? g_nlog : LOGMAX - 1];
  vsnprintf(buf, sizeof buf, fmt, ap);
  e->kind = kind; e->text[0] = 0; esc_cat(e->text, sizeof e->text, buf);
  e->st = e->cnt = e->ev = e->jdb = 0;
  if (kind != 'c') {
    if (g_cur9) { e->st = g_cur9->state_; e->cnt = g_cur9->cnt_; e->ev = g_cur9->event_; e->jdb = g_cur9->jdBit_; h_dump_obj(0, "snap", g_cur9, f_Ph3V90Obj, NF_Ph3V90Obj); }
    if (g_cur2) { e->st = g_cur2->state_; e->cnt = g_cur2->cnt_; e->ev = g_cur2->event_; e->jdb = g_cur2->jaBit_; h_dump_obj(0, "snap", g_cur2, f_Ph3V92Obj, NF_Ph3V92Obj); }
  }
  h_emit(g_show, "%cprintf %s st=%u cnt=%u ev=%u jb=%u", kind, e->text, e->st, e->cnt, e->ev, e->jdb);
  g_nlog++;
}
void edprintf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  record('e', fmt, ap);
  va_end(ap);
  g_ntot_e++;
}
int dsplibs_debug_printf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  record('d', fmt, ap);
  va_end(ap);
  g_ntot_d++;
  return 0;
}
static void log_begin(void) { g_nlog = 0; }

/* the Jd callees: the bit vectors are harness blocks, scripted per trial; the fake Jd objects identify the argument */
static unsigned char g_fakejd[3][16];
static unsigned char *g_bv, *g_jdbv, *g_jdphbv;
static uint8_t *g_ja;
static unsigned char *g_ret_bv, *g_ret_jdbv, *g_ret_jdphbv;
static int g_exp_jd = -1;
static void rec_c(const char *fmt, ...) {
  va_list ap;
  char buf[TXTMAX * 2];
  LogEnt *e = &g_log[g_nlog < LOGMAX ? g_nlog : LOGMAX - 1];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  e->kind = 'c'; e->text[0] = 0; esc_cat(e->text, sizeof e->text, buf);
  e->st = e->cnt = e->ev = e->jdb = 0;
  h_emit(g_show, "cprintf %s", e->text);
  g_nlog++;
  g_ntot_c++;
}
static int jd_id(void *p) { int k; for (k = 0; k < 3; k++) if (p == (void *)g_fakejd[k]) return k; h_die3("a Jd callee got an unknown object %p", p); return -1; }
unsigned char *h_getbv(void *jd) { rec_c("V90Jd::getBitVector obj=%d", jd_id(jd)); return g_ret_bv; }
unsigned char *h_getjdbv(void *jd) { rec_c("V92Jd::getJdBitVector obj=%d", jd_id(jd)); return g_ret_jdbv; }
unsigned char *h_getjdphbv(void *jd) { rec_c("V92Jd::getJdPhaseBitVector obj=%d", jd_id(jd)); return g_ret_jdphbv; }

/* ------------------------------------------------------------------------------------------------ small helpers */
#define EXPECT_EQ(what, got, want)                                                                                                   \
  do {                                                                                                                              \
    unsigned long long g_ = (unsigned long long)(got), w_ = (unsigned long long)(want);                                              \
    if (g_ != w_) h_die3("%s: got 0x%llx expected 0x%llx (line %d)", (what), g_, w_, __LINE__);                                     \
  } while (0)
static void relax_reserved(void) { h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0; }
static unsigned pick_level(void) {
  static const unsigned lv[] = { 0, 1, 2, 3, 7, 0x80000000u, 0xffffffffu, 0x80000001u };
  return h_rpct(65) ? 2u : lv[h_rmod(sizeof lv / sizeof lv[0])];
}
static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }
static unsigned rd32(const uint8_t *p) { unsigned v; memcpy(&v, p, 4); return v; }
static unsigned rd16(const uint8_t *p) { unsigned v = p[0] | ((unsigned)p[1] << 8); return v; }
static void wr32(uint8_t *p, unsigned v) { memcpy(p, &v, 4); }
static void wr16(uint8_t *p, unsigned v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

/* ------------------------------------------------------------------------------------------------ coverage ids */
enum {
  C_TRIALS = 1, C_SYMBOLS, C_LV0, C_LV1, C_LV2, C_LVHI,
  /* V90 class symbol generator */
  V_SD_PAT0, V_SD_PAT1, V_SD_PAT3, V_SD_PAT4, V_SD_END, V_SDN_PAT, V_SDN_END, V_SDN_STAY, V_TRN_BIT0, V_TRN_BIT1, V_TRN_END_JD, V_TRN_END_NOJD, V_TRN_NOJD_PRINT, V_TRN_NOJD_QUIET,
  V_JD_X0, V_JD_X1, V_JD_TIMEOUT, V_JD_TIMEOUT64, V_JD_NORMAL, V_JDN_X0, V_JDN_X1, V_JDN_END, V_JDN_NOEND, V_Z_X0, V_Z_X1, V_Z_END_DIL, V_Z_END_NODIL, V_DIL_TIMEOUT, V_DIL_NORMAL, V_DIL_TIMEOUT_WRAP,
  V_DIL_END_PRINT, V_DIL_END_STAY, V_ST_QUIET11, V_ST_QUIET12, V_ST_QUIET14, V_ST_QUIET15, V_ILL4, V_ILL5, V_ILL6, V_ILL13, V_ILL_HI, V_ILL_PRINT, V_ILL_QUIET,
  /* V92 mode of the V90 class */
  W_TRN_END_JD, W_TRN_END_NOJD, W_JD_X0, W_JD_X1, W_JD_TIMEOUT, W_JD_NORMAL, W_JD4_END, W_JD4_NOEND, W_PH5_TIMEOUT, W_PH5_NORMAL, W_PH6_END, W_PH6_NOEND, W_ILL7, W_QUIET13, W_Z_X0, W_Z_X1,
  W_Z_END_DIL, W_Z_END_NODIL,
  /* DIL */
  D_RESET_NULL, D_RESET_ALAW, D_RESET_ULAW, D_TP_ZERO, D_TP_NONZERO, D_SP_NEG, D_SP_POS, D_SEG_CHANGE, D_SEG_STAY, D_TO_WRAP, D_TP_WRAP, D_SP_WRAP, D_SEG8, D_SEG0, D_SEGMID, D_STEP_ALAW, D_STEP_ULAW,
  D_LTP_OVER128, D_N_ZERO, D_GENDIL, D_UPDSEG, D_EXITDIL_EARLY, D_EXITDIL_BUSY, D_EXITDIL_END, D_EXITDIL_PRINT,
  /* exits (V90 class) */
  E_JD_NOSTATE, E_JD_ZERO, E_JD_FRAME_V90, E_JD_FRAME_V92, E_JD_MID_V90, E_JD_MID_V92, E_PH_NOSTATE, E_PH_ZERO, E_PH_FRAME, E_PH_MID,
  /* reset (V90 class) */
  R9_ALAW, R9_ULAW, R9_V90_JD, R9_V90_NOJD, R9_V92_JD, R9_V92_NOJD, R9_WARM90, R9_WARM92, R9_CODE_WRAP, R9_FLAG_GARBAGE,
  /* V92 class */
  Q_RU, Q_RUN_END, Q_RUN_STAY, Q_TRN_B0, Q_TRN_B1, Q_TRN_END, Q_JA_FIRST, Q_JA_BITS, Q_JA_X0, Q_JA_X1, Q_JA5_END, Q_JA5_STAY, Q_SU_END, Q_SUN_END, Q_SUN_STAY, Q_SU2, Q_SU3_END, Q_SU3_STAY,
  Q_SU4_END, Q_SU4_STAY, Q_T12_B0, Q_T12_B1, Q_T13_B0, Q_T13_B1, Q_T13_END, Q_T13_EARLY, Q_T13_NOMUL, Q_SIL6, Q_SIL14, Q_SIL15, Q_ILL2, Q_ILL_HI, Q_ILL_PRINT, Q_ILL_QUIET,
  QE_JA_NOSTATE, QE_JA_ZERO, QE_JA_END, QE_JA_MID, QE_SIL_NOSTATE, QE_SIL_ZERO, QE_SIL_GO, QE_SU_NOSTATE, QE_SU_ZERO, QE_SU_END, QE_SU_MID, QE_TRN_NOSTATE, QE_TRN_ZERO, QE_TRN_GO,
  QR_LEN_MIN, QR_LEN_COMPUTED, QR_LEN_NEG, QR_JA, QR_NOJA_DIL_PRINT, QR_NOJA_DIL_QUIET, QR_NOJA_NODIL, QR_WARM, QR_NOWARM,
  /* ctor / dtor / small */
  S_C1, S_C2, S_D1, S_D2, S_SETFLAG, S_SD, S_SDNOT, S_JD, S_JDNOT, S_TRN1D, S_JDPH, S_V92JD, S_RU, S_RUNOT, S_SU, S_SUNOT, S_JA, S_TRN1U,
  COV_MAX
};
static const char *cov_name(int id) {
  static char buf[COV_MAX][40];
  if (!buf[id][0]) snprintf(buf[id], sizeof buf[id], "cov %03d", id);
  return buf[id];
}
typedef struct { unsigned char c[COV_MAX]; } CovSet;
#define COV(m, id) ((m)->cov.c[id] = 1)
static unsigned long g_covn[COV_MAX];
static unsigned char g_covreq[COV_MAX];
static void cov_hit(int id) { g_covn[id]++; }
static void cov_flush(const CovSet *c) { int i; for (i = 0; i < COV_MAX; i++) if (c->c[i]) cov_hit(i); }

static int okp(const uint8_t *p) { return p && (p == g_bv || p == g_jdbv || p == g_jdphbv); }   /* a bit-vector pointer the harness handed out (anything else would be a wild read in the blob) */
/* ------------------------------------------------------------------------------------------------ model of V90Phase3Modulator */
enum { OF_FLAG = 0x00, OF_ALAW = 0x04, OF_BASE = 0x08, OF_A = 0x0c, OF_B = 0x0e, OF_C = 0x12, OF_ST = 0x14, OF_CNT = 0x18, OF_EV = 0x1c, OF_JDB = 0x40,
       OF_N = 0x54, OF_LSP = 0x55, OF_LTP = 0x56, OF_SP = 0x57, OF_TP = 0xd7, OF_SEGLEN = 0x158, OF_REF = 0x178, OF_TO = 0x188, OF_SPI = 0x388, OF_TPI = 0x389, OF_TOI = 0x38a, OF_SEGCNT = 0x38c,
       OF_SEG = 0x390, OF_ISREF = 0x392, OF_CODE = 0x394, OF_DILEND = 0x398 };
typedef struct {
  uint8_t mo[0x398];              /* the model's image of the object (raw byte offsets of the 32-bit blob) */
  uint8_t hist[24];               /* scrambler delay line: hist[k-1] = the symbol k steps ago */
  const uint8_t *pJd, *pV92, *pPh;/* the bit vectors behind +0x44 / +0x48 / +0x4c (NULL = NULL pointer) */
  unsigned level;
  LogEnt log[LOGMAX];
  unsigned nlog;
  int crash;
  CovSet cov;
} M9;
static unsigned g32(const M9 *m, unsigned o) { return rd32(m->mo + o); }
static unsigned g16(const M9 *m, unsigned o) { return rd16(m->mo + o); }
static void s32(M9 *m, unsigned o, unsigned v) { wr32(m->mo + o, v); }
static void s16(M9 *m, unsigned o, unsigned v) { wr16(m->mo + o, v); }
static int16_t sgn(int pos, unsigned lvl) { int16_t v = (int16_t)lvl; return pos ? v : (int16_t)(-(int)v); }
static void m9_print(M9 *m, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  LogEnt *e = &m->log[m->nlog < LOGMAX ? m->nlog : LOGMAX - 1];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  e->kind = kind; e->text[0] = 0; esc_cat(e->text, sizeof e->text, buf);
  e->st = g32(m, OF_ST); e->cnt = g32(m, OF_CNT); e->ev = g32(m, OF_EV); e->jdb = g32(m, OF_JDB);
  m->nlog++;
}
static unsigned m_dec(unsigned alaw, unsigned code) { return (unsigned)(alaw ? alaw2linear((int)((code & 0x7f) ^ 0xd5)) : ulaw2linear((int)((code & 0x7f) ^ 0xff))) & 0xffffu; }
static unsigned m9_scr(M9 *m, unsigned p1, unsigned in) {
  unsigned y = (unsigned)m->hist[p1 - 1] ^ (in & 0xffu) ^ (unsigned)m->hist[22];
  memmove(m->hist + 1, m->hist, 23);
  m->hist[0] = (uint8_t)y;
  return y;
}
static void m9_scr_reset(M9 *m, unsigned v) { memset(m->hist, (int)(v & 1), 23); }
/* the segment (0 .. 8) of the level to[toi]: the first limit of the row alaw * 8 that the zero-extended 16-bit value does not exceed */
static unsigned m_segof(const M9 *m) {
  unsigned lvl = g16(m, OF_TO + 2u * m->mo[OF_TOI]), k;
  for (k = 0; k < 8; k++) if ((int)lvl <= v9_table[g32(m, OF_ALAW) * 8 + k]) return k;
  return 8;
}
/* resetDILGenerator from the raw bytes of a descriptor */
static void m9_dilreset(M9 *m, const uint8_t *d) {
  unsigned i, k, alaw = g32(m, OF_ALAW);
  if (!d) { m->mo[OF_N] = 0; COV(m, D_RESET_NULL); return; }
  COV(m, alaw ? D_RESET_ALAW : D_RESET_ULAW);
  if (d[1] > 128 || d[2] > 128) COV(m, D_LTP_OVER128);
  if (d[0] == 0) COV(m, D_N_ZERO);
  m->mo[OF_N] = d[0]; m->mo[OF_LSP] = d[1]; m->mo[OF_LTP] = d[2];
  for (i = 0; i < d[1]; i++) m->mo[OF_SP + i] = d[3 + i];
  for (i = 0; i < d[2]; i++) m->mo[OF_TP + i] = d[0x83 + i];
  for (k = 0; k < 8; k++) { s32(m, OF_SEGLEN + 4 * k, 6u + 6u * d[0x103 + k]); s16(m, OF_REF + 2 * k, m_dec(alaw, d[0x10b + k])); }
  for (i = 0; i < m->mo[OF_N]; i++) s16(m, OF_TO + 2 * i, m_dec(alaw, d[0x113 + i]));
  m->mo[OF_SPI] = 0; s32(m, OF_SEGCNT, 0); m->mo[OF_TPI] = 0; m->mo[OF_TOI] = 0;
  m->mo[OF_SEG] = (uint8_t)m_segof(m);
}
/* one DIL symbol (generateDIL = the inlined steps of states 9 / 0xa) */
static int16_t m9_dilstep(M9 *m) {
  if (m->mo[OF_SEG] > 143) { m->crash = 1; return 0; }   /* segLen[seg] beyond the object: not modelled */
  unsigned tpi = m->mo[OF_TPI], spi = m->mo[OF_SPI], toi = m->mo[OF_TOI], seg = m->mo[OF_SEG], alaw = g32(m, OF_ALAW);
  unsigned tp = m->mo[OF_TP + tpi], sp = m->mo[OF_SP + spi], arg = g16(m, OF_TO + 2 * toi), cnt1 = g32(m, OF_SEGCNT) + 1u;
  int16_t v = (int16_t)(tp != 0 ? arg : g16(m, OF_REF + 2 * seg));
  unsigned ntp = (tpi + 1u) & 0xffu, nsp = (spi + 1u) & 0xffu;
  if (tp == 0) COV(m, D_TP_ZERO); else COV(m, D_TP_NONZERO);
  s16(m, OF_ISREF, tp == 0);
  m->mo[OF_CODE] = (uint8_t)(alaw ? (linear2alaw((int)arg) ^ 0xd5) : ~linear2ulaw((int)arg));
  COV(m, alaw ? D_STEP_ALAW : D_STEP_ULAW);
  if (ntp == m->mo[OF_LTP]) { ntp = 0; COV(m, D_TP_WRAP); }
  if (sp == 0) { v = (int16_t)(-(int)v); COV(m, D_SP_NEG); } else COV(m, D_SP_POS);
  if (nsp == m->mo[OF_LSP]) { nsp = 0; COV(m, D_SP_WRAP); }
  if (seg == 8) COV(m, D_SEG8);
  if (cnt1 == g32(m, OF_SEGLEN + 4 * seg)) {
    unsigned nt = (toi + 1u) & 0xffu;
    COV(m, D_SEG_CHANGE);
    s32(m, OF_SEGCNT, 0); m->mo[OF_TPI] = 0; m->mo[OF_SPI] = 0;
    if (nt == m->mo[OF_N]) { nt = 0; COV(m, D_TO_WRAP); }
    m->mo[OF_TOI] = (uint8_t)nt;
    m->mo[OF_SEG] = (uint8_t)m_segof(m);
    if (m->mo[OF_SEG] == 0) COV(m, D_SEG0); else if (m->mo[OF_SEG] < 8) COV(m, D_SEGMID);
  } else {
    COV(m, D_SEG_STAY);
    m->mo[OF_SPI] = (uint8_t)nsp; s32(m, OF_SEGCNT, cnt1); m->mo[OF_TPI] = (uint8_t)ntp;
  }
  return v;
}
/* the V.90 / V.92 symbol of V90Phase3Modulator; `v92` selects the table (flag != 0 for generateSymbol, the function itself for generateV90Symbol / generateV92Symbol) */
static int16_t m9_symbol(M9 *m, int v92) {
  unsigned st = g32(m, OF_ST), cnt, mod, A = g16(m, OF_A), B = g16(m, OF_B), C = g16(m, OF_C), base = g32(m, OF_BASE);
  const uint8_t *jd = v92 ? m->pV92 : m->pJd;
  unsigned x;
  int16_t sym = 0;
  /* a read of a NULL bit vector would crash the blob */
  if (v92 ? (st == 3 || st == 4) && !okp(m->pV92) : (st == 3 || st == 7) && !okp(m->pJd)) { m->crash = 1; return 0; }
  if (v92 && (st == 5 || st == 6) && !okp(m->pPh)) { m->crash = 1; return 0; }
  if (st == 2 && (v92 ? m->pV92 : m->pJd) && !okp(v92 ? m->pV92 : m->pJd)) { /* the end-of-TRN1d test only compares with NULL: a garbage pointer is fine here, but a later state 3 would read it */ }
  cnt = g32(m, OF_CNT) + 1u;
  s32(m, OF_CNT, cnt);
  mod = (cnt - 1u) % 6u;
  switch (st) {
    case 0:   /* Sd */
      s32(m, OF_EV, 0);
      switch (mod) { case 0: case 2: sym = sgn(1, B); COV(m, V_SD_PAT0); break; case 1: sym = sgn(1, C); COV(m, V_SD_PAT1); break; case 3: case 5: sym = sgn(0, B); COV(m, V_SD_PAT3); break; default: sym = sgn(0, C); COV(m, V_SD_PAT4); }
      if (cnt == 0x180) { s32(m, OF_ST, 1); s32(m, OF_CNT, 0); COV(m, V_SD_END); }
      return sym;
    case 1:   /* SdNot */
      switch (mod) { case 0: case 2: sym = sgn(0, B); break; case 1: sym = sgn(0, C); break; case 3: case 5: sym = sgn(1, B); break; default: sym = sgn(1, C); }
      COV(m, V_SDN_PAT);
      if (cnt == 0x30) { s32(m, OF_ST, 2); s32(m, OF_EV, 1); s32(m, OF_CNT, 0); m9_scr_reset(m, 0); COV(m, V_SDN_END); }
      else { s32(m, OF_EV, 0); COV(m, V_SDN_STAY); }
      return sym;
    case 2:   /* TRN1d: scrambled ones; the sign is positive for a nonzero bit */
      s32(m, OF_EV, 0);
      x = m9_scr(m, 18, 1);
      sym = sgn(x != 0, A);
      COV(m, x ? V_TRN_BIT1 : V_TRN_BIT0);
      if (cnt != 0x3e7c) return sym;
      if (!(v92 ? m->pV92 : m->pJd)) {
        s32(m, OF_ST, 0xf);
        COV(m, v92 ? W_TRN_END_NOJD : V_TRN_END_NOJD);
        if (m->level > 1) { m9_print(m, 'd', v92 ? "V90Phase3Modulator: ERROR: Null v92JdBits @ end of TRN1d\r\n" : "V90Phase3Modulator: ERROR: Null JdBits @ end of TRN1d\r\n"); COV(m, V_TRN_NOJD_PRINT); }
        else COV(m, V_TRN_NOJD_QUIET);
      } else {
        s32(m, OF_ST, 3); s32(m, OF_EV, 2); s32(m, OF_JDB, sym > 0);
        COV(m, v92 ? W_TRN_END_JD : V_TRN_END_JD);
      }
      s32(m, OF_CNT, 0);
      return sym;
    case 3:   /* Jd / V92Jd frame bits */
      s32(m, OF_EV, 0);
      x = m9_scr(m, 18, jd[(cnt - 1u) % 72u]) ^ g32(m, OF_JDB);
      s32(m, OF_JDB, x);
      sym = sgn(x == 0 ? 0 : 1, A);   /* x != 0: +level */
      if (x == 0) sym = sgn(0, A);
      COV(m, v92 ? (x ? W_JD_X1 : W_JD_X0) : (x ? V_JD_X1 : V_JD_X0));
      if ((unsigned)(base + 24804u) == cnt && (unsigned long long)base + 24804ull > 0xffffffffull) COV(m, V_JD_TIMEOUT64);   /* a 32-bit sum would have timed out here */
      if ((unsigned long long)cnt == (unsigned long long)base + 24804ull) {
        s32(m, OF_ST, 0xc); s32(m, OF_CNT, 0);
        COV(m, v92 ? W_JD_TIMEOUT : V_JD_TIMEOUT);
        m9_print(m, 'e', v92 ? "V90Phase3Modulator: V92Jd TimeOut\r\n" : "V90Phase3Modulator: Jd TimeOut\r\n");
      } else COV(m, v92 ? W_JD_NORMAL : V_JD_NORMAL);
      return sym;
    case 4:
      if (v92) {   /* V92Jd, frames of 72 */
        s32(m, OF_EV, 0);
        x = m9_scr(m, 18, m->pV92[(cnt - 1u) % 72u]) ^ g32(m, OF_JDB);
        s32(m, OF_JDB, x);
        sym = sgn(x != 0, A);
        if (cnt % 72u == 0) { s32(m, OF_ST, 5); s32(m, OF_CNT, 0); s32(m, OF_EV, 3); COV(m, W_JD4_END); } else COV(m, W_JD4_NOEND);
        return sym;
      }
      break;
    case 5:
      if (v92) {   /* V92JdPhase: time-out at exactly 24804 */
        s32(m, OF_EV, 0);
        x = m9_scr(m, 18, m->pPh[(cnt - 1u) % 72u]) ^ g32(m, OF_JDB);
        s32(m, OF_JDB, x);
        sym = sgn(x != 0, A);
        if (cnt == 24804u) { s32(m, OF_ST, 0xd); s32(m, OF_CNT, 0); COV(m, W_PH5_TIMEOUT); m9_print(m, 'e', "V90Phase3Modulator: V92JdPhase TimeOut\r\n"); } else COV(m, W_PH5_NORMAL);
        return sym;
      }
      break;
    case 6:
      if (v92) {
        s32(m, OF_EV, 0);
        x = m9_scr(m, 18, m->pPh[(cnt - 1u) % 72u]) ^ g32(m, OF_JDB);
        s32(m, OF_JDB, x);
        sym = sgn(x != 0, A);
        if (cnt % 72u == 0) { s32(m, OF_ST, 8); s32(m, OF_CNT, 0); COV(m, W_PH6_END); } else COV(m, W_PH6_NOEND);
        return sym;
      }
      break;
    case 7:
      if (!v92) {   /* Jd NOT: frames of 72 */
        s32(m, OF_EV, 0);
        x = m9_scr(m, 18, jd[(cnt - 1u) % 72u]) ^ g32(m, OF_JDB);
        s32(m, OF_JDB, x);
        sym = sgn(x != 0, A);
        if (cnt % 72u == 0) { s32(m, OF_ST, 8); s32(m, OF_CNT, 0); COV(m, V_JDN_END); } else COV(m, V_JDN_NOEND);
        COV(m, x ? V_JDN_X1 : V_JDN_X0);
        return sym;
      }
      COV(m, W_ILL7);
      break;
    case 8:
      s32(m, OF_EV, 0);
      x = m9_scr(m, 18, 0) ^ g32(m, OF_JDB);
      s32(m, OF_JDB, x);
      sym = sgn(x != 0, A);
      COV(m, v92 ? (x ? W_Z_X1 : W_Z_X0) : (x ? V_Z_X1 : V_Z_X0));
      if (cnt != 0xc) return sym;
      if (m->mo[OF_N] == 0) {
        s32(m, OF_ST, 0xf);
        COV(m, v92 ? W_Z_END_NODIL : V_Z_END_NODIL);
        m9_print(m, 'e', "V90Phase3Modulator: ERROR: Null DIL @ end of JdNOT\r\n");
      } else { s32(m, OF_ST, 9); COV(m, v92 ? W_Z_END_DIL : V_Z_END_DIL); }
      s32(m, OF_CNT, 0);
      return sym;
    case 9:
      s32(m, OF_EV, 0);
      sym = m9_dilstep(m);
      if (cnt == base + 40000u) {
        s32(m, OF_ST, 0xe); s32(m, OF_CNT, 0); COV(m, V_DIL_TIMEOUT);
        if ((unsigned long long)base + 40000ull > 0xffffffffull) COV(m, V_DIL_TIMEOUT_WRAP);
        m9_print(m, 'e', "V90Phase3Modulator: DIL TimeOut\r\n");
      } else COV(m, V_DIL_NORMAL);
      return sym;
    case 10:
      s32(m, OF_EV, 0);
      sym = m9_dilstep(m);
      if (g32(m, OF_SEGCNT) == 0) {
        COV(m, V_DIL_END_PRINT);
        m9_print(m, 'e', "V90Phase3Modulator: Phase3 Terminated @ %d\r\n", (int)cnt);
        s32(m, OF_ST, 0xb); s32(m, OF_CNT, 0); s32(m, OF_EV, 6);
      } else COV(m, V_DIL_END_STAY);
      return sym;
    case 11: COV(m, V_ST_QUIET11); s32(m, OF_EV, 0); return 0;
    case 12: COV(m, V_ST_QUIET12); s32(m, OF_EV, 0); return 0;
    case 14: COV(m, V_ST_QUIET14); s32(m, OF_EV, 0); return 0;
    case 15: COV(m, V_ST_QUIET15); s32(m, OF_EV, 0); return 0;
    case 13:
      if (v92) { COV(m, W_QUIET13); s32(m, OF_EV, 0); return 0; }
      COV(m, V_ILL13);
      break;
    default: COV(m, V_ILL_HI); break;
  }
  /* illegal state */
  if (st == 4) COV(m, V_ILL4); else if (st == 5) COV(m, V_ILL5); else if (st == 6) COV(m, V_ILL6);
  s32(m, OF_EV, 0);
  if (m->level > 1) { m9_print(m, 'd', "V90Phase3Modulator: Illegal state\r\n"); COV(m, V_ILL_PRINT); } else COV(m, V_ILL_QUIET);
  return 0;
}

static void m9_callee(M9 *m, const char *txt, int id) {
  char buf[TXTMAX];
  LogEnt *e = &m->log[m->nlog < LOGMAX ? m->nlog : LOGMAX - 1];
  snprintf(buf, sizeof buf, "%s obj=%d", txt, id);
  e->kind = 'c'; e->text[0] = 0; esc_cat(e->text, sizeof e->text, buf);
  e->st = e->cnt = e->ev = e->jdb = 0;
  m->nlog++;
}
static void m9_exitjd(M9 *m) {
  unsigned c = g32(m, OF_CNT), flag = g32(m, OF_FLAG);
  if (g32(m, OF_ST) != 3) { COV(m, E_JD_NOSTATE); return; }
  if (c == 0) { COV(m, E_JD_ZERO); return; }
  if (c % 72u == 0) { s32(m, OF_CNT, 0); s32(m, OF_ST, flag == 0 ? 8u : 5u); COV(m, flag ? E_JD_FRAME_V92 : E_JD_FRAME_V90); }
  else { s32(m, OF_ST, flag == 0 ? 7u : 4u); COV(m, flag ? E_JD_MID_V92 : E_JD_MID_V90); }
}
static void m9_exitjdph(M9 *m) {
  unsigned c = g32(m, OF_CNT);
  if (g32(m, OF_ST) != 5) { COV(m, E_PH_NOSTATE); return; }
  if (c == 0) { COV(m, E_PH_ZERO); return; }
  if (c % 72u == 0) { s32(m, OF_ST, 8); s32(m, OF_CNT, 0); COV(m, E_PH_FRAME); } else { s32(m, OF_ST, 6); COV(m, E_PH_MID); }
}
static void m9_exitdil(M9 *m) {
  unsigned c = g32(m, OF_CNT);
  if (g32(m, OF_ST) != 9) { COV(m, D_EXITDIL_EARLY); return; }
  if (c == 0) { COV(m, D_EXITDIL_EARLY); return; }
  if (g32(m, OF_SEGCNT) != 0) { s32(m, OF_ST, 10); COV(m, D_EXITDIL_BUSY); return; }
  COV(m, D_EXITDIL_PRINT);
  m9_print(m, 'e', "V90Phase3Modulator: Phase3 Terminated @ %d\r\n", (int)c);
  s32(m, OF_ST, 0xb); s32(m, OF_CNT, 0); s32(m, OF_EV, 6);
}
/* the pure pattern functions (no counter increment): which = 0 Sd, 1 SdNot */
static int16_t m9_pat(M9 *m, int which) {
  unsigned mod = (g32(m, OF_CNT) - 1u) % 6u, B = g16(m, OF_B), C = g16(m, OF_C);
  int pos = which == 0;
  switch (mod) { case 0: case 2: return sgn(pos, B); case 1: return sgn(pos, C); case 3: case 5: return sgn(!pos, B); case 4: return sgn(!pos, C); default: return 0; }
}
/* generateJd (src 0: +0x44), generateV92Jd (1: +0x48), generateJdPhase (2: +0x4c), generateJdNot (3: zeros), generateTRN1d (4: scrambled ones, no differential state) */
static int16_t m9_dead(M9 *m, int src) {
  unsigned A = g16(m, OF_A), x, c = g32(m, OF_CNT);
  const uint8_t *p = src == 0 ? m->pJd : src == 1 ? m->pV92 : m->pPh;
  if (src <= 2 && !okp(p)) { m->crash = 1; return 0; }
  if (src == 4) { x = m9_scr(m, 18, 1); return sgn(x != 0, A); }
  x = m9_scr(m, 18, src == 3 ? 0u : p[(c - 1u) % 72u]) ^ g32(m, OF_JDB);
  s32(m, OF_JDB, x);
  return sgn(x != 0, A);
}
static void m9_reset(M9 *m, unsigned pcm, unsigned code, unsigned st, unsigned n, int jdid, int v92id, const uint8_t *dil, unsigned base) {
  unsigned alaw, flag = g32(m, OF_FLAG), i;
  m9_scr_reset(m, 0);
  s32(m, OF_ALAW, pcm); s32(m, OF_CNT, 0); s32(m, OF_EV, 0); s32(m, OF_BASE, base); s32(m, OF_ST, st);
  alaw = pcm;
  COV(m, alaw ? R9_ALAW : R9_ULAW);
  if ((code & 0xffu) + 0x10u > 0xffu) COV(m, R9_CODE_WRAP);
  if (flag > 1) COV(m, R9_FLAG_GARBAGE);
  if (n) COV(m, flag ? R9_WARM92 : R9_WARM90);
  s16(m, OF_A, m_dec(alaw, code & 0xffu));
  s16(m, OF_B, m_dec(alaw, ((code & 0xffu) + 0x10u) & 0xffu));
  s16(m, OF_C, (unsigned)(alaw ? alaw2linear(0xd5) : ulaw2linear(0xff)) & 0xffffu);
  s32(m, OF_JDB, 0);
  if (flag == 0) {
    if (jdid >= 0) { m9_callee(m, "V90Jd::getBitVector", jdid); m->pJd = g_ret_bv; COV(m, R9_V90_JD); } else { m->pJd = NULL; COV(m, R9_V90_NOJD); }
  } else if (v92id >= 0) {
    m9_callee(m, "V92Jd::getJdBitVector", v92id); m->pV92 = g_ret_jdbv;
    m9_callee(m, "V92Jd::getJdPhaseBitVector", v92id); m->pPh = g_ret_jdphbv;
    COV(m, R9_V92_JD);
  } else { m->pV92 = NULL; m->pPh = NULL; COV(m, R9_V92_NOJD); }
  m9_dilreset(m, dil);
  for (i = 0; i < n; i++) { m9_symbol(m, flag != 0); if (m->crash) return; }
}
/* load the scalar fields / the DIL region / the pointer fields of the real object into the model image */
static void m9_load(M9 *m, const O9 *o) {
  s32(m, OF_FLAG, o->flag_); s32(m, OF_ALAW, o->alaw_); s32(m, OF_BASE, o->timeBase_); s16(m, OF_A, o->levelA_); s16(m, OF_B, o->levelB_); s16(m, OF_C, o->levelC_);
  s32(m, OF_ST, o->state_); s32(m, OF_CNT, o->cnt_); s32(m, OF_EV, o->event_); s32(m, OF_JDB, o->jdBit_);
  memcpy(m->mo + OF_N, (const uint8_t *)o + offsetof(O9, dilN_), OF_DILEND - OF_N);
  m->pJd = (const uint8_t *)o->jdBits_; m->pV92 = (const uint8_t *)o->v92JdBits_; m->pPh = (const uint8_t *)o->v92JdPhaseBits_;
  m->level = dsplibs_debug_level;
  m->nlog = 0; m->crash = 0;
}
static void m9_cmp(const M9 *m, const O9 *o, const char *what) {
#define C9(name, got, want) do { unsigned long long g_ = (unsigned long long)(got), w_ = (unsigned long long)(want); if (g_ != w_) h_die3("%s: %s is 0x%llx, model 0x%llx", what, name, g_, w_); } while (0)
  C9("flag_", o->flag_, g32(m, OF_FLAG)); C9("alaw_", o->alaw_, g32(m, OF_ALAW)); C9("timeBase_", o->timeBase_, g32(m, OF_BASE));
  C9("levelA_", o->levelA_, g16(m, OF_A)); C9("levelB_", o->levelB_, g16(m, OF_B)); C9("levelC_", o->levelC_, g16(m, OF_C));
  C9("state_", o->state_, g32(m, OF_ST)); C9("cnt_", o->cnt_, g32(m, OF_CNT)); C9("event_", o->event_, g32(m, OF_EV)); C9("jdBit_", o->jdBit_, g32(m, OF_JDB));
  C9("jdBits_", (uintptr_t)o->jdBits_, (uintptr_t)m->pJd); C9("v92JdBits_", (uintptr_t)o->v92JdBits_, (uintptr_t)m->pV92); C9("v92JdPhaseBits_", (uintptr_t)o->v92JdPhaseBits_, (uintptr_t)m->pPh);
  if (memcmp((const uint8_t *)o + offsetof(O9, dilN_), m->mo + OF_N, OF_DILEND - OF_N)) {
    unsigned k;
    for (k = 0; k < OF_DILEND - OF_N; k++) if (((const uint8_t *)o)[offsetof(O9, dilN_) + k] != m->mo[OF_N + k]) h_die3("%s: DIL region byte at +0x%x is 0x%02x, model 0x%02x", what, OF_N + k, ((const uint8_t *)o)[offsetof(O9, dilN_) + k], m->mo[OF_N + k]);
  }
#undef C9
}
static void log_cmp(const LogEnt *ml, unsigned mn, const char *what) {
  unsigned i;
  if (mn != g_nlog) h_die3("%s: %u prints / callee calls, model %u", what, g_nlog, mn);
  for (i = 0; i < mn && i < LOGMAX; i++) {
    if (ml[i].kind != g_log[i].kind || strcmp(ml[i].text, g_log[i].text)) h_die3("%s: log entry %u is '%c %s', model '%c %s'", what, i, g_log[i].kind, g_log[i].text, ml[i].kind, ml[i].text);
    if (ml[i].kind != 'c' && (ml[i].st != g_log[i].st || ml[i].cnt != g_log[i].cnt || ml[i].ev != g_log[i].ev || ml[i].jdb != g_log[i].jdb))
      h_die3("%s: log entry %u snapshot st=%u cnt=%u ev=%u jb=%u, model st=%u cnt=%u ev=%u jb=%u", what, i, g_log[i].st, g_log[i].cnt, g_log[i].ev, g_log[i].jdb, ml[i].st, ml[i].cnt, ml[i].ev, ml[i].jdb);
  }
}

/* ------------------------------------------------------------------------------------------------ model of V92Phase3Modulator */
enum { Q_ARG6 = 0x00, Q_A = 0x04, Q_B = 0x06, Q_ST = 0x08, Q_CNT = 0x0c, Q_LEN = 0x10, Q_EV = 0x14, Q_JB = 0x38, Q_JLEN = 0x40 };
typedef struct {
  uint8_t mo[0x50];
  uint8_t hist[24];
  const uint8_t *pJa;
  unsigned level;
  LogEnt log[LOGMAX];
  unsigned nlog;
  int crash;
  CovSet cov;
} M2;
static int okja(const uint8_t *p) { return p && g_ja && p == g_ja + 4; }
static unsigned q32(const M2 *m, unsigned o) { return rd32(m->mo + o); }
static unsigned q16(const M2 *m, unsigned o) { return rd16(m->mo + o); }
static void qs32(M2 *m, unsigned o, unsigned v) { wr32(m->mo + o, v); }
static void qs16(M2 *m, unsigned o, unsigned v) { wr16(m->mo + o, v); }
static void m2_print(M2 *m, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2];
  va_list ap;
  LogEnt *e = &m->log[m->nlog < LOGMAX ? m->nlog : LOGMAX - 1];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  e->kind = kind; e->text[0] = 0; esc_cat(e->text, sizeof e->text, buf);
  e->st = q32(m, Q_ST); e->cnt = q32(m, Q_CNT); e->ev = q32(m, Q_EV); e->jdb = q32(m, Q_JB);
  m->nlog++;
}
static unsigned m2_scr(M2 *m, unsigned in) {
  unsigned y = (unsigned)m->hist[4] ^ (in & 0xffu) ^ (unsigned)m->hist[22];   /* taps 5 and 23 */
  memmove(m->hist + 1, m->hist, 23);
  m->hist[0] = (uint8_t)y;
  return y;
}
static void m2_scr_reset(M2 *m, unsigned v) { memset(m->hist, (int)(v & 1), 23); }
/* the period-6 Ru / Su sequences: pattern index mod = (cnt - 1) % 6; table of signs * levels */
static int16_t m2_ru(const M2 *m, int not_) {
  unsigned mod = (q32(m, Q_CNT) - 1u) % 6u, A = q16(m, Q_A);
  return sgn((mod < 3) != (not_ != 0), A);
}
/* Su: the sequence over a period of 6 is {+B, 0, +B, -B, 0, -B} (not_: negated) */
static int16_t m2_su(const M2 *m, int not_) {
  static const int sg[6] = { 1, 0, 1, -1, 0, -1 };
  unsigned mod = (q32(m, Q_CNT) - 1u) % 6u, B = q16(m, Q_B);
  int s = sg[mod] * (not_ ? -1 : 1);
  return s == 0 ? 0 : sgn(s > 0, B);
}
static int m2_would_crash(const M2 *m, unsigned st, unsigned cnt1) {
  return (st == 4 || st == 5) && cnt1 > 0x18 && (q32(m, Q_JLEN) == 0 || !okja(m->pJa));
}
static int16_t m2_symbol(M2 *m) {
  unsigned st = q32(m, Q_ST), cnt = q32(m, Q_CNT) + 1u, x, b;
  int16_t sym = 0;
  unsigned A = q16(m, Q_A);
  if (m2_would_crash(m, st, cnt)) { m->crash = 1; return 0; }
  qs32(m, Q_CNT, cnt);
  switch (st) {
    case 0:
      qs32(m, Q_EV, 0); sym = m2_ru(m, 0); COV(m, Q_RU);
      if (cnt == 0x180) { qs32(m, Q_ST, 1); qs32(m, Q_CNT, 0); }
      return sym;
    case 1:
      qs32(m, Q_EV, 0); sym = m2_ru(m, 1);
      if (cnt == 0x18) { qs32(m, Q_ST, 3); qs32(m, Q_CNT, 0); m2_scr_reset(m, 0); qs32(m, Q_EV, 2); COV(m, Q_RUN_END); } else COV(m, Q_RUN_STAY);
      return sym;
    case 3:
      qs32(m, Q_EV, 0);
      x = m2_scr(m, 1);
      sym = sgn(x == 0, A); COV(m, x ? Q_TRN_B1 : Q_TRN_B0);
      if (cnt == q32(m, Q_LEN)) { qs32(m, Q_ST, 4); qs32(m, Q_CNT, 0); qs32(m, Q_EV, 3); qs32(m, Q_JB, sym <= 0); COV(m, Q_TRN_END); }
      return sym;
    case 4: case 5:
      qs32(m, Q_EV, 0);
      b = cnt > 0x18 ? m->pJa[(cnt - 0x19u) % q32(m, Q_JLEN)] : 1u;
      COV(m, cnt > 0x18 ? Q_JA_BITS : Q_JA_FIRST);
      x = m2_scr(m, b) ^ q32(m, Q_JB);
      qs32(m, Q_JB, x);
      sym = sgn(x == 0, A); COV(m, x ? Q_JA_X1 : Q_JA_X0);
      if (st == 5) {
        if (cnt % 12u == 0) { qs32(m, Q_ST, 6); qs32(m, Q_CNT, 0); COV(m, Q_JA5_END); } else COV(m, Q_JA5_STAY);
      }
      return sym;
    case 6: qs32(m, Q_EV, 0); COV(m, Q_SIL6); return 0;
    case 14: qs32(m, Q_EV, 0); COV(m, Q_SIL14); return 0;
    case 15: qs32(m, Q_EV, 0); COV(m, Q_SIL15); return 0;
    case 7:
      qs32(m, Q_EV, 0); sym = m2_su(m, 0);
      if (cnt == 0x90) { qs32(m, Q_ST, 8); qs32(m, Q_CNT, 0); COV(m, Q_SU_END); }
      return sym;
    case 8:
      sym = m2_su(m, 1);
      if (cnt == 0x18) { qs32(m, Q_ST, 9); qs32(m, Q_CNT, 0); qs32(m, Q_EV, 5); COV(m, Q_SUN_END); } else { qs32(m, Q_EV, 0); COV(m, Q_SUN_STAY); }
      return sym;
    case 9: qs32(m, Q_EV, 0); COV(m, Q_SU2); return m2_su(m, 0);
    case 10:
      qs32(m, Q_EV, 0); sym = m2_su(m, 0);
      if (cnt % 12u == 0) { qs32(m, Q_ST, 0xb); qs32(m, Q_CNT, 0); COV(m, Q_SU3_END); } else COV(m, Q_SU3_STAY);
      return sym;
    case 11:
      qs32(m, Q_EV, 0); sym = m2_su(m, 1);
      if (cnt == 0x18) { qs32(m, Q_ST, 0xc); qs32(m, Q_CNT, 0); m2_scr_reset(m, 0); qs32(m, Q_EV, 7); COV(m, Q_SU4_END); } else COV(m, Q_SU4_STAY);
      return sym;
    case 12:
      qs32(m, Q_EV, 0); x = m2_scr(m, 1); COV(m, x ? Q_T12_B1 : Q_T12_B0);
      return sgn(x == 0, A);
    case 13:
      qs32(m, Q_EV, 0); x = m2_scr(m, 1); sym = sgn(x == 0, A); COV(m, x ? Q_T13_B1 : Q_T13_B0);
      if (cnt < 0x7f8u) { COV(m, Q_T13_EARLY); return sym; }
      if (cnt % 12u != 0) { COV(m, Q_T13_NOMUL); return sym; }
      qs32(m, Q_ST, 0xe); qs32(m, Q_CNT, 0); qs32(m, Q_EV, 8); qs32(m, Q_JB, sym <= 0); COV(m, Q_T13_END);
      return sym;
    default:
      qs32(m, Q_EV, 0);
      if (st == 2) COV(m, Q_ILL2); else COV(m, Q_ILL_HI);
      if (m->level > 1) { m2_print(m, 'd', "V92Phase3Modulator: Illegal state\r\n"); COV(m, Q_ILL_PRINT); } else COV(m, Q_ILL_QUIET);
      return 0;
  }
}
static void m2_exitja(M2 *m) {
  unsigned c = q32(m, Q_CNT);
  if (q32(m, Q_ST) != 4) { COV(m, QE_JA_NOSTATE); return; }
  if (c == 0) { COV(m, QE_JA_ZERO); return; }
  if (c % 12u == 0) { qs32(m, Q_ST, 6); qs32(m, Q_CNT, 0); COV(m, QE_JA_END); } else { qs32(m, Q_ST, 5); COV(m, QE_JA_MID); }
}
static void m2_exitsil(M2 *m) {
  if (q32(m, Q_ST) != 6) { COV(m, QE_SIL_NOSTATE); return; }
  if (q32(m, Q_CNT) == 0) { COV(m, QE_SIL_ZERO); return; }
  qs32(m, Q_ST, 7); qs32(m, Q_CNT, 0); COV(m, QE_SIL_GO);
}
static void m2_exitsu2(M2 *m) {
  unsigned c = q32(m, Q_CNT);
  if (q32(m, Q_ST) != 9) { COV(m, QE_SU_NOSTATE); return; }
  if (c == 0) { COV(m, QE_SU_ZERO); return; }
  if (c % 12u == 0) { qs32(m, Q_ST, 0xb); qs32(m, Q_CNT, 0); COV(m, QE_SU_END); } else { qs32(m, Q_ST, 0xa); COV(m, QE_SU_MID); }
}
static void m2_exittrn(M2 *m) {
  if (q32(m, Q_ST) != 0xc) { COV(m, QE_TRN_NOSTATE); return; }
  if (q32(m, Q_CNT) == 0) { COV(m, QE_TRN_ZERO); return; }
  qs32(m, Q_ST, 0xd); COV(m, QE_TRN_GO);
}
/* generateJa / generateTRN1u (dead): no counter increment */
static int16_t m2_ja(M2 *m) {
  unsigned cnt = q32(m, Q_CNT), b, x;
  if (cnt > 0x18 && (q32(m, Q_JLEN) == 0 || !okja(m->pJa))) { m->crash = 1; return 0; }
  b = cnt > 0x18 ? m->pJa[(cnt - 0x19u) % q32(m, Q_JLEN)] : 1u;
  x = m2_scr(m, b) ^ q32(m, Q_JB);
  qs32(m, Q_JB, x);
  return sgn(x == 0, q16(m, Q_A));
}
static int16_t m2_trn1u(M2 *m) { unsigned x = m2_scr(m, 1); return sgn(x == 0, q16(m, Q_A)); }
/* reset: sum = fast + slow + 12, length = 12 * (sum / 12) (signed divide) unless below 0x1fe0 as an unsigned value */
static void m2_reset(M2 *m, unsigned st, unsigned n, const uint8_t *ja, unsigned jalen, int hasdil, unsigned arg6, int fast, int slow) {
  unsigned i;
  int sum = (int)((unsigned)fast + (unsigned)slow + 12u), len;
  m2_scr_reset(m, 0);
  qs32(m, Q_CNT, 0); qs32(m, Q_ARG6, arg6); qs32(m, Q_ST, st);
  len = (sum / 12) * 12;
  if ((unsigned)len > 0x1fdfu) { qs32(m, Q_LEN, (unsigned)len); COV(m, (unsigned)len > 0x7fffffffu ? QR_LEN_NEG : QR_LEN_COMPUTED); } else { qs32(m, Q_LEN, 0x1fe0u); COV(m, QR_LEN_MIN); }
  m2_print(m, 'e', "V92Phase3Modulator: TRN1u state length set to %d\r\n", (int)q32(m, Q_LEN));
  qs32(m, Q_EV, 0);
  qs16(m, Q_A, 0xfa0);
  if (ja) { m->pJa = ja + 4; qs32(m, Q_JLEN, jalen); COV(m, QR_JA); }
  else {
    if (hasdil && m->level > 1) { m2_print(m, 'd', "BUG BUG BUG BUG BUG - V92Phase3Modulator: no Ja Object, DIL descriptor available !\r\n"); COV(m, QR_NOJA_DIL_PRINT); }
    else if (hasdil) COV(m, QR_NOJA_DIL_QUIET); else COV(m, QR_NOJA_NODIL);
    qs32(m, Q_JLEN, 0); m->pJa = NULL;
  }
  qs16(m, Q_B, 4899);   /* trunc((float)(4000 * sqrt(1.5)) + 0.5f) */
  if (n) COV(m, QR_WARM); else COV(m, QR_NOWARM);
  for (i = 0; i < n; i++) { m2_symbol(m); if (m->crash) return; }
}
static void m2_load(M2 *m, const O2 *o) {
  qs32(m, Q_ARG6, o->arg6_); qs16(m, Q_A, o->levelA_); qs16(m, Q_B, o->levelB_); qs32(m, Q_ST, o->state_); qs32(m, Q_CNT, o->cnt_); qs32(m, Q_LEN, o->trn1uLen_); qs32(m, Q_EV, o->event_);
  qs32(m, Q_JB, o->jaBit_); qs32(m, Q_JLEN, o->jaLen_);
  m->pJa = (const uint8_t *)o->jaBits_;
  m->level = dsplibs_debug_level; m->nlog = 0; m->crash = 0;
}
static void m2_cmp(const M2 *m, const O2 *o, const char *what) {
#define C2(name, got, want) do { unsigned long long g_ = (unsigned long long)(got), w_ = (unsigned long long)(want); if (g_ != w_) h_die3("%s: %s is 0x%llx, model 0x%llx", what, name, g_, w_); } while (0)
  C2("arg6_", o->arg6_, q32(m, Q_ARG6)); C2("levelA_", o->levelA_, q16(m, Q_A)); C2("levelB_", o->levelB_, q16(m, Q_B)); C2("state_", o->state_, q32(m, Q_ST));
  C2("cnt_", o->cnt_, q32(m, Q_CNT)); C2("trn1uLen_", o->trn1uLen_, q32(m, Q_LEN)); C2("event_", o->event_, q32(m, Q_EV)); C2("jaBit_", o->jaBit_, q32(m, Q_JB));
  C2("jaLen_", o->jaLen_, q32(m, Q_JLEN)); C2("jaBits_", (uintptr_t)o->jaBits_, (uintptr_t)m->pJa);
#undef C2
}

/* ------------------------------------------------------------------------------------------------ the objects of a trial */
static O9 *g_o9;
static O2 *g_o2;
static uint8_t *g_can9, *g_can2;
static M9 g_m9;
static M2 g_m2;
static unsigned g_jalen;
static DilDesc g_dil;
static unsigned long g_symctr, g_dumpctr;
static unsigned g_ntrials, g_nskipped;
enum { CAN = 64 };
static void can_fill(uint8_t *c) { memset(c, 0x5a, CAN); }
static void can_check(const uint8_t *c, const char *w) { unsigned i; for (i = 0; i < CAN; i++) if (c[i] != 0x5a) h_die3("%s: canary byte %u behind the object was overwritten", w, i); }
static void fill_bytes(uint8_t *p, unsigned n, int mode) {
  unsigned i;
  for (i = 0; i < n; i++) p[i] = mode == 0 ? (uint8_t)(h_rnd() & 1) : mode == 1 ? (uint8_t)h_rnd() : mode == 2 ? 1 : 0;
}
/* architecture-independent random content of an object before its constructor runs (the pool's own fill depends on sizeof / the offsets, which differ on x86-64): the field table
 * drives it, one value per field element, in table order */
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
        case 'u': q = o + f[i].off + 4u * k; memcpy(q, &v, 4); break;
        default: { uintptr_t pv = 0xdead0000u | (v & 0xffffu); q = o + f[i].off + sizeof(void *) * k; memcpy(q, &pv, sizeof pv); break; }
      }
    }
  }
}
static void dump9(const char *tag) { h_dump_obj(0, tag, g_o9, f_Ph3V90Obj, NF_Ph3V90Obj); }
static void dump2(const char *tag) { h_dump_obj(0, tag, g_o2, f_Ph3V92Obj, NF_Ph3V92Obj); }
static void flush9(void) { cov_flush(&g_m9.cov); memset(&g_m9.cov, 0, sizeof g_m9.cov); }
static void flush2(void) { cov_flush(&g_m2.cov); memset(&g_m2.cov, 0, sizeof g_m2.cov); }

/* the heap blocks the harness hands to the object (bit vectors of 72 bytes, a Ja block): allocated with sysdep_malloc so that the pointers print as A<id>+off */
static void alloc_bufs(int bitmode) {
  g_bv = (uint8_t *)sysdep_malloc(72); g_jdbv = (uint8_t *)sysdep_malloc(72); g_jdphbv = (uint8_t *)sysdep_malloc(72);
  fill_bytes(g_bv, 72, bitmode); fill_bytes(g_jdbv, 72, bitmode); fill_bytes(g_jdphbv, 72, bitmode);
  g_ret_bv = g_bv; g_ret_jdbv = g_jdbv; g_ret_jdphbv = g_jdphbv;
}
static void free_bufs(void) { sysdep_free(g_bv); sysdep_free(g_jdbv); sysdep_free(g_jdphbv); }

/* A9: arguments of the V90Phase3Modulator::reset / resetDILGenerator call */
typedef struct { unsigned pcm, code, st, n, base; int jdid, v92id; const DilDesc *dil; } A9;
enum { OP_SYM, OP_S90, OP_S92, OP_EXITJD, OP_EXITJDPH, OP_EXITDIL, OP_UPDSEG, OP_RESETDIL, OP_GENDIL, OP_SD, OP_SDNOT, OP_JD, OP_JDNOT, OP_TRN1D, OP_JDPH, OP_V92JD, OP_RESET, OP_SETFLAG };

/* run one operation on the model (copy first: a call that would crash the blob is skipped), then on the real object, and compare everything.  Returns 0 when skipped. */
static void cov_ptrs(void) {}
static M9 g_pre9;   /* the model before the last operation (debugging a violation) */
static int do9(int op, const A9 *a, unsigned flagarg, int16_t *symout) {
  static M9 tmp;
  int16_t exp = 0, got = 0;
  int has_sym = 0;
  O9 *o = g_o9;
  unsigned long ctr;
  memcpy(&tmp, &g_m9, sizeof tmp);
  tmp.nlog = 0; tmp.crash = 0; tmp.level = dsplibs_debug_level;
  switch (op) {
    case OP_SYM: exp = m9_symbol(&tmp, g32(&tmp, OF_FLAG) != 0); has_sym = 1; break;
    case OP_S90: exp = m9_symbol(&tmp, 0); has_sym = 1; break;
    case OP_S92: exp = m9_symbol(&tmp, 1); has_sym = 1; break;
    case OP_EXITJD: m9_exitjd(&tmp); break;
    case OP_EXITJDPH: m9_exitjdph(&tmp); break;
    case OP_EXITDIL: m9_exitdil(&tmp); break;
    case OP_UPDSEG: tmp.mo[OF_SEG] = (uint8_t)m_segof(&tmp); COV(&tmp, D_UPDSEG); break;
    case OP_RESETDIL: m9_dilreset(&tmp, (const uint8_t *)a->dil); break;
    case OP_GENDIL: exp = m9_dilstep(&tmp); has_sym = 1; COV(&tmp, D_GENDIL); break;
    case OP_SD: exp = m9_pat(&tmp, 0); has_sym = 1; COV(&tmp, S_SD); break;
    case OP_SDNOT: exp = m9_pat(&tmp, 1); has_sym = 1; COV(&tmp, S_SDNOT); break;
    case OP_JD: exp = m9_dead(&tmp, 0); has_sym = 1; COV(&tmp, S_JD); break;
    case OP_JDNOT: exp = m9_dead(&tmp, 3); has_sym = 1; COV(&tmp, S_JDNOT); break;
    case OP_TRN1D: exp = m9_dead(&tmp, 4); has_sym = 1; COV(&tmp, S_TRN1D); break;
    case OP_JDPH: exp = m9_dead(&tmp, 2); has_sym = 1; COV(&tmp, S_JDPH); break;
    case OP_V92JD: exp = m9_dead(&tmp, 1); has_sym = 1; COV(&tmp, S_V92JD); break;
    case OP_RESET: m9_reset(&tmp, a->pcm, a->code, a->st, a->n, a->jdid, a->v92id, (const uint8_t *)a->dil, a->base); break;
    case OP_SETFLAG: s32(&tmp, OF_FLAG, flagarg); COV(&tmp, S_SETFLAG); break;
    default: h_die3("do9: op %d", op);
  }
  if (tmp.crash) { g_nskipped++; return 0; }
  log_begin(); g_cur9 = o;
  switch (op) {
    case OP_SYM: got = v9_sym(o); break;
    case OP_S90: got = v9_sym90(o); break;
    case OP_S92: got = v9_sym92(o); break;
    case OP_EXITJD: v9_exitjd(o); break;
    case OP_EXITJDPH: v9_exitjdph(o); break;
    case OP_EXITDIL: v9_exitdil(o); break;
    case OP_UPDSEG: v9_updseg(o); break;
    case OP_RESETDIL: v9_resetdil(o, a->dil); break;
    case OP_GENDIL: got = v9_gendil(o); break;
    case OP_SD: got = v9_sd(o); break;
    case OP_SDNOT: got = v9_sdnot(o); break;
    case OP_JD: got = v9_jd(o); break;
    case OP_JDNOT: got = v9_jdnot(o); break;
    case OP_TRN1D: got = v9_trn1d(o); break;
    case OP_JDPH: got = v9_jdph(o); break;
    case OP_V92JD: got = v9_v92jd(o); break;
    case OP_RESET:
      v9_reset(o, (int)a->pcm, a->code, a->st, a->n, a->jdid >= 0 ? (void *)g_fakejd[a->jdid] : NULL, a->v92id >= 0 ? (void *)g_fakejd[a->v92id] : NULL, a->dil, a->base);
      break;
    case OP_SETFLAG: v9_setflag(o, flagarg); break;
  }
  g_cur9 = NULL;
  memcpy(&g_pre9, &g_m9, sizeof g_pre9);
  memcpy(&g_m9, &tmp, sizeof g_m9);
  if (has_sym && got != exp) h_die3("op %d: symbol %d, model %d (state %u cnt %u)", op, got, exp, o->state_, o->cnt_);
  { char lab[48]; snprintf(lab, sizeof lab, "after op %d (state %u cnt %u)", op, o->state_, o->cnt_); m9_cmp(&g_m9, o, lab); }
  log_cmp(tmp.log, tmp.nlog, "prints");
  can_check(g_can9, "V90Phase3Modulator");
  ctr = g_symctr++;
  h_emit(0, "G%d %d %u %u %u %u", op, has_sym ? got : 0, o->state_, o->cnt_, o->event_, o->jdBit_);
  if (g_nlog || o->event_ != 0 || !(ctr & 1023u) || op == OP_RESET || op == OP_RESETDIL) { g_dumpctr++; dump9("o"); }
  if (symout) *symout = got;
  flush9();
  return 1;
}

/* V92Phase3Modulator operations */
enum { QO_SYM, QO_EXITJA, QO_EXITSIL, QO_EXITSU2, QO_EXITTRN, QO_RU, QO_RUNOT, QO_SU, QO_SUNOT, QO_JA, QO_TRN1U, QO_RESET };
typedef struct { unsigned st, n, arg6; int hasja, hasdil; int fast, slow; unsigned short arg1; } A2;
static V92ParObj *g_par2;
static int do2(int op, const A2 *a) {
  static M2 tmp;
  int16_t exp = 0, got = 0;
  int has_sym = 0;
  O2 *o = g_o2;
  unsigned long ctr;
  memcpy(&tmp, &g_m2, sizeof tmp);
  tmp.nlog = 0; tmp.crash = 0; tmp.level = dsplibs_debug_level;
  switch (op) {
    case QO_SYM: exp = m2_symbol(&tmp); has_sym = 1; break;
    case QO_EXITJA: m2_exitja(&tmp); break;
    case QO_EXITSIL: m2_exitsil(&tmp); break;
    case QO_EXITSU2: m2_exitsu2(&tmp); break;
    case QO_EXITTRN: m2_exittrn(&tmp); break;
    case QO_RU: exp = m2_ru(&tmp, 0); has_sym = 1; COV(&tmp, S_RU); break;
    case QO_RUNOT: exp = m2_ru(&tmp, 1); has_sym = 1; COV(&tmp, S_RUNOT); break;
    case QO_SU: exp = m2_su(&tmp, 0); has_sym = 1; COV(&tmp, S_SU); break;
    case QO_SUNOT: exp = m2_su(&tmp, 1); has_sym = 1; COV(&tmp, S_SUNOT); break;
    case QO_JA: exp = m2_ja(&tmp); has_sym = 1; COV(&tmp, S_JA); break;
    case QO_TRN1U: exp = m2_trn1u(&tmp); has_sym = 1; COV(&tmp, S_TRN1U); break;
    case QO_RESET: m2_reset(&tmp, a->st, a->n, a->hasja ? g_ja : NULL, g_jalen, a->hasdil, a->arg6, a->fast, a->slow); break;
    default: h_die3("do2: op %d", op);
  }
  if (tmp.crash) { g_nskipped++; return 0; }
  log_begin(); g_cur2 = o;
  switch (op) {
    case QO_SYM: got = v2_sym(o); break;
    case QO_EXITJA: v2_exitja(o); break;
    case QO_EXITSIL: v2_exitsil(o); break;
    case QO_EXITSU2: v2_exitsu2(o); break;
    case QO_EXITTRN: v2_exittrn(o); break;
    case QO_RU: got = v2_ru(o); break;
    case QO_RUNOT: got = v2_runot(o); break;
    case QO_SU: got = v2_su(o); break;
    case QO_SUNOT: got = v2_sunot(o); break;
    case QO_JA: got = v2_ja(o); break;
    case QO_TRN1U: got = v2_trn1u(o); break;
    case QO_RESET: v2_reset(o, (int)(short)a->arg1 | (int)(h_rnd() & 0xffff0000u), a->st, a->n, a->hasja ? (void *)g_ja : NULL, a->hasdil ? (const void *)&g_dil : NULL, a->arg6); break;
  }
  g_cur2 = NULL;
  memcpy(&g_m2, &tmp, sizeof g_m2);
  if (has_sym && got != exp) h_die3("V92 op %d: symbol %d, model %d (state %u cnt %u)", op, got, exp, o->state_, o->cnt_);
  m2_cmp(&g_m2, o, "after V92 op");
  log_cmp(tmp.log, tmp.nlog, "V92 prints");
  can_check(g_can2, "V92Phase3Modulator");
  ctr = g_symctr++;
  h_emit(0, "H%d %d %u %u %u %u", op, has_sym ? got : 0, o->state_, o->cnt_, o->event_, o->jaBit_);
  if (g_nlog || o->event_ != 0 || !(ctr & 1023u) || op == QO_RESET) { g_dumpctr++; dump2("o"); }
  flush2();
  return 1;
}

/* ------------------------------------------------------------------------------------------------ trial setup */
static void *g_params9;
static unsigned g_flag9;
static void new9(unsigned flag, int use_c2, int bitmode) {
  uint8_t *raw = (uint8_t *)h_obj_new(SZ9 + CAN);
  O9 *o = (O9 *)raw;
  g_o9 = o; g_can9 = raw + SZ9; can_fill(g_can9);
  prefill(o, SZ9, f_Ph3V90Obj, NF_Ph3V90Obj);
  alloc_bufs(bitmode);
  g_params9 = sysdep_malloc(32);
  memset(&g_m9, 0, sizeof g_m9);
  dsplibs_debug_level = pick_level();
  m9_load(&g_m9, o);
  g_flag9 = flag;
  cov_hit(use_c2 ? S_C2 : S_C1);
  H_CTOR_PROBE(o, f_Ph3V90Obj, NF_Ph3V90Obj, sizeof(O9), (use_c2 ? v9_c2(o, g_params9, flag) : v9_c1(o, g_params9, flag)));
  s32(&g_m9, OF_FLAG, flag);
  g_m9.level = dsplibs_debug_level;
  m9_reset(&g_m9, 0, 0x40, 0, 0, -1, -1, NULL, 0);
  if (g_m9.nlog) h_die3("the constructor's reset logged %u calls in the model", g_m9.nlog);
  m9_cmp(&g_m9, o, "after the constructor");
  can_check(g_can9, "V90Phase3Modulator ctor");
  dump9("ctor");
  if (o->params_ != g_params9) h_die3("params_ is %p, not the argument", o->params_);
  flush9();
}
static void end9(int use_d2) {
  O9 *o = g_o9;
  cov_hit(use_d2 ? S_D2 : S_D1);
  if (use_d2) v9_d2(o); else v9_d1(o);
  __asm__ volatile("" ::: "memory");
  can_check(g_can9, "V90Phase3Modulator dtor");
  dump9("dtor");
  free_bufs();
  sysdep_free(g_params9);
  h_trial_end("v90");
  g_ntrials++;
  cov_hit(C_TRIALS);
}
static void new2(int use_c2, int bitmode, int fast, int slow) {
  uint8_t *raw = (uint8_t *)h_obj_new(SZ2 + CAN);
  O2 *o = (O2 *)raw;
  g_o2 = o; g_can2 = raw + SZ2; can_fill(g_can2);
  prefill(o, SZ2, f_Ph3V92Obj, NF_Ph3V92Obj);
  alloc_bufs(bitmode);
  g_jalen = 1 + h_rmod(h_rpct(30) ? 400 : 60);
  g_ja = (uint8_t *)sysdep_malloc(4 + g_jalen);
  fill_bytes(g_ja + 4, g_jalen, bitmode);
  memcpy(g_ja, &g_jalen, 4);
  h_log_next_size = 0xdc;
  g_par2 = (V92ParObj *)sysdep_malloc(sizeof(V92ParObj));
  g_par2->v92EchoFastUpdateDuration = fast;
  g_par2->v92EchoSlowUpdateDuration = slow;
  memset(&g_m2, 0, sizeof g_m2);
  dsplibs_debug_level = pick_level();
  m2_load(&g_m2, o);
  g_m2.level = dsplibs_debug_level;
  log_begin(); g_cur2 = o;
  cov_hit(use_c2 ? S_C2 : S_C1);
  H_CTOR_PROBE(o, f_Ph3V92Obj, NF_Ph3V92Obj, sizeof(O2), (use_c2 ? v2_c2(o, g_par2) : v2_c1(o, g_par2)));
  g_cur2 = NULL;
  m2_reset(&g_m2, 0, 0, NULL, 0, 0, 0, fast, slow);
  qs32(&g_m2, Q_ARG6, 0);
  m2_cmp(&g_m2, o, "after the V92 constructor");
  log_cmp(g_m2.log, g_m2.nlog, "V92 ctor prints");
  can_check(g_can2, "V92Phase3Modulator ctor");
  dump2("ctor");
  if (o->params_ != (void *)g_par2) h_die3("V92 params_ is %p", o->params_);
  flush2();
}
static void end2(int use_d2) {
  O2 *o = g_o2;
  cov_hit(use_d2 ? S_D2 : S_D1);
  if (use_d2) v2_d2(o); else v2_d1(o);
  __asm__ volatile("" ::: "memory");
  can_check(g_can2, "V92Phase3Modulator dtor");
  dump2("dtor");
  free_bufs();
  sysdep_free(g_ja);
  sysdep_free(g_par2);
  h_trial_end("v92");
  g_ntrials++;
  cov_hit(C_TRIALS);
}

/* ------------------------------------------------------------------------------------------------ generators */
static void gen_dil(int regime) {
  uint8_t *b = (uint8_t *)&g_dil;
  unsigned i;
  for (i = 0; i < sizeof g_dil; i++) b[i] = (uint8_t)h_rnd();
  g_dil.N_ = (uint8_t)(regime == 0 ? 1 + h_rmod(10) : regime == 1 ? 1 + h_rmod(255) : h_rpct(30) ? 0 : h_rpct(50) ? 255 : 1 + h_rmod(255));
  g_dil.Lsp_ = (uint8_t)(regime == 0 ? 1 + h_rmod(8) : regime == 1 ? 1 + h_rmod(255) : h_rpct(30) ? 0 : h_rpct(50) ? 255 : 1 + h_rmod(255));
  g_dil.Ltp_ = (uint8_t)(regime == 0 ? 1 + h_rmod(8) : regime == 1 ? 1 + h_rmod(255) : h_rpct(30) ? 0 : h_rpct(50) ? 255 : 1 + h_rmod(255));
  if (regime == 1 && h_rpct(60)) { g_dil.Lsp_ = (uint8_t)(1 + h_rmod(128)); g_dil.Ltp_ = (uint8_t)(1 + h_rmod(128)); }
  if (h_rpct(88)) {   /* patterns of ones and zeros (the real descriptors hold 0 / 1) */
    for (i = 0; i < 128; i++) { g_dil.sp_[i] = (uint8_t)(h_rnd() & 1); g_dil.tp_[i] = (uint8_t)(h_rnd() & 1); }
    if (h_rpct(30)) for (i = 0; i < 128; i++) g_dil.tp_[i] = (uint8_t)(h_rpct(10) ? 1 : 0);
  }
  for (i = 0; i < 8; i++) g_dil.hr_[i] = (uint8_t)(regime == 0 ? h_rmod(5) : regime == 1 ? h_rmod(h_rpct(50) ? 12 : 256) : h_rmod(256));
  if (h_rpct(25)) for (i = 0; i < 8; i++) g_dil.hr_[8 + i] = (uint8_t)(h_rpct(50) ? 0x7f : 0xff);   /* extreme reference codes */
  if (h_rpct(25)) for (i = 0; i < 256; i++) g_dil.to_[i] = (uint8_t)(h_rpct(50) ? (h_rnd() & 0x80) | 0x7f : h_rnd());
}
static unsigned pick_base(void) {
  switch (h_rmod(9)) {
    case 0: return 0;
    case 1: return 1;
    case 2: case 3: return h_rmod(200);
    case 4: return 0xffffffffu - 24804u + h_rmod(40);
    case 5: return 0xffffffffu - 40000u + h_rmod(40);
    case 6: return 0xffffffffu;
    case 7: return h_rnd();
    default: return h_rmod(5000);
  }
}
/* a counter value near a threshold of the given state (V.90 session: v92 == 0) */
static unsigned near_threshold(unsigned st, unsigned base, int v92) {
  unsigned d = h_rmod(4);   /* 0 .. 3 below / at / above the threshold */
  unsigned t;
  switch (st) {
    case 0: t = 0x17e; break;
    case 1: t = 0x2e; break;
    case 2: t = 0x3e7a; break;
    case 3: t = base + 24802u; break;
    case 4: t = v92 ? 70u + 72u * h_rmod(3) : 5u; break;
    case 5: t = v92 ? 24802u : 5u; break;
    case 6: t = v92 ? 70u + 72u * h_rmod(3) : 5u; break;
    case 7: t = 70u + 72u * h_rmod(3); break;
    case 8: t = 10u; break;
    case 9: t = base + 39998u; break;
    default: t = h_rmod(40); break;
  }
  return t + d;
}
static unsigned pick_cnt(unsigned base, int v92) {
  static const unsigned th[] = { 0x17f, 0x180, 0x2f, 0x30, 0x3e7b, 0x3e7c, 0x17, 0x18, 0xb, 0xc, 71, 72, 143, 144, 0x7f7, 0x7f8, 0x8f, 0x90, 0x5f, 0x60, 0x1fdf, 0x1fe0 };
  switch (h_rmod(9)) {
    case 0: return 0;
    case 1: case 2: return th[h_rmod(sizeof th / sizeof th[0])];
    case 3: return base + 24803u + h_rmod(3);
    case 4: return base + 39999u + h_rmod(3);
    case 5: return 24803u + h_rmod(3);
    case 6: return 0xffffffffu - h_rmod(3);
    case 7: return h_rmod(300);
    default: return v92 ? h_rmod(30000) : h_rmod(50);
  }
}
static unsigned pick_state(int v92) {
  static const unsigned odd[] = { 16, 17, 31, 100, 0x80000000u, 0xffffffffu };
  if (h_rpct(10)) return odd[h_rmod(sizeof odd / sizeof odd[0])];
  (void)v92;
  return h_rmod(16);
}
static void rand_exit9(void) {
  switch (h_rmod(3)) { case 0: do9(OP_EXITJD, NULL, 0, NULL); break; case 1: do9(OP_EXITJDPH, NULL, 0, NULL); break; default: do9(OP_EXITDIL, NULL, 0, NULL); }
}
/* run symbols (the owner's loop): `maxs` symbols or until a silent / error state; exit functions at random moments and at the frame ends */
static void run9(unsigned maxs, int exits) {
  unsigned i;
  for (i = 0; i < maxs; i++) {
    int op = h_rpct(88) ? OP_SYM : h_rpct(50) ? OP_S90 : OP_S92;
    unsigned st;
    if (!do9(op, NULL, 0, NULL)) break;
    st = g_o9->state_;
    if (exits && (g_o9->event_ != 0 || h_rmod(400) == 0 || (g_o9->cnt_ % 72u == 0 && h_rpct(35)))) rand_exit9();
    st = g_o9->state_;
    if ((st == 0xb || st == 0xc || st == 0xd || st == 0xe || st == 0xf) && h_rpct(5)) break;
    if (st > 0xf) break;
  }
}
static void force9(int v92) {
  O9 *o = g_o9;
  unsigned base = o->timeBase_;
  o->state_ = pick_state(v92);
  if (h_rpct(10)) {   /* the time-out states with a base near the 32-bit wrap, the end of TRN1d without a bit vector */
    unsigned which = h_rmod(3);
    o->state_ = which == 0 ? 3u : which == 1 ? 9u : 2u;
    if (which != 2 || h_rpct(50)) o->timeBase_ = base = pick_base();
    if (which == 2) { o->cnt_ = 0x3e7bu + h_rmod(2); o->jdBits_ = NULL; o->v92JdBits_ = NULL; dsplibs_debug_level = h_rpct(50) ? h_rmod(2) : pick_level(); }
  }
  if (h_rpct(7)) {   /* the end of TRN1d with a bit vector, small levels: jdBit = sym > 0 */
    static const unsigned short lv[] = { 0, 1, 2, 0xffff, 0xfffe, 0x8000, 0x7fff };
    o->state_ = 2; o->cnt_ = 0x3e7bu + h_rmod(2);
    o->jdBits_ = g_bv; o->v92JdBits_ = g_jdbv;
    if (h_rpct(70)) o->levelA_ = lv[h_rmod(7)];
  }
  if (h_rpct(8)) { switch (h_rmod(4)) { case 0: o->state_ = 5; break; case 1: o->state_ = 3; break; case 2: o->state_ = 9; break; default: o->state_ = 8; } o->cnt_ = o->state_ == 8 ? 10u + h_rmod(4) : 1u + h_rmod(71) + 72u * h_rmod(5); if (h_rpct(30)) o->segCnt_ = 0; }
  if (o->state_ != 2 || o->cnt_ < 0x3e7b) o->cnt_ = h_rpct(55) ? near_threshold(o->state_, base, v92) : pick_cnt(base, v92);
  if (h_rpct(12)) { switch (h_rmod(3)) { case 0: o->jdBits_ = NULL; break; case 1: o->v92JdBits_ = NULL; break; default: o->v92JdPhaseBits_ = NULL; } }
  else if (h_rpct(25)) { o->jdBits_ = g_bv; o->v92JdBits_ = g_jdbv; o->v92JdPhaseBits_ = g_jdphbv; }
  if (h_rpct(25)) o->jdBit_ = h_rnd();
  if (h_rpct(25)) o->event_ = h_rnd();
  if (h_rpct(10)) dsplibs_debug_level = pick_level();
  if (h_rpct(10)) { o->tpIdx_ = (uint8_t)h_rnd(); o->spIdx_ = (uint8_t)h_rnd(); o->toIdx_ = (uint8_t)h_rnd(); o->segCnt_ = h_rmod(40); o->seg_ = (uint8_t)h_rmod(9); }
  if (h_rpct(12)) {   /* levels the G.711 decoders never give: zero, +-1, +-2, the 16-bit extremes (the sign of a level, jdBit = sym > 0, a negated -32768) */
    static const unsigned short lv[] = { 0, 1, 2, 0xffff, 0xfffe, 0x8000, 0x7fff, 0x8001 };
    if (h_rpct(60)) o->levelA_ = h_rpct(75) ? lv[h_rmod(8)] : (unsigned short)h_rnd();
    if (h_rpct(40)) o->levelB_ = h_rpct(75) ? lv[h_rmod(8)] : (unsigned short)h_rnd();
    if (h_rpct(40)) o->levelC_ = h_rpct(75) ? lv[h_rmod(8)] : (unsigned short)h_rnd();
  }
  m9_load(&g_m9, o);
}
static void trial_v90class(int flag, int dilp) {
  A9 a;
  int use_c2 = h_rpct(12), bitmode = h_rpct(88) ? 0 : 1 + (int)h_rmod(3);
  int regime = h_rmod(3), nat = h_rpct(35);
  new9((unsigned)flag, use_c2, bitmode);
  memset(&a, 0, sizeof a);
  a.pcm = h_rmod(2);
  a.code = h_garbage_hi(h_rnd() & 0xffu);
  a.st = nat ? 0u : h_rpct(60) ? 0u : h_rmod(16);
  a.n = h_rpct(75) ? 0u : 1u + h_rmod(60);
  a.base = nat ? h_rmod(100) : pick_base();
  a.jdid = h_rpct(92) ? 0 : -1;
  a.v92id = h_rpct(92) ? 1 : -1;
  if (h_rpct(dilp)) { gen_dil(regime); a.dil = &g_dil; }
  if (!a.dil && h_rpct(15)) a.pcm = h_rmod(6);
  if (h_rpct(10)) g_flag9 = flag ? (unsigned)(h_rpct(50) ? 0x80000000u : h_rnd() | 1u) : 0u;
  if (g_flag9 != (unsigned)flag) do9(OP_SETFLAG, NULL, g_flag9, NULL);
  if (!do9(OP_RESET, &a, 0, NULL)) { /* the warm-up symbols would have crashed the blob: reset without them */ a.n = 0; do9(OP_RESET, &a, 0, NULL); }
  if (nat) {
    if (h_rpct(70)) run9(18000 + h_rmod(8000), 0); else run9(20000, 1);
    run9(60000, 1);
  } else {
    unsigned k, nforce = 3 + h_rmod(12);
    for (k = 0; k < nforce; k++) {
      force9(flag != 0);
      if (h_rpct(40)) { do9(OP_EXITJD, NULL, 0, NULL); do9(OP_EXITJDPH, NULL, 0, NULL); do9(OP_EXITDIL, NULL, 0, NULL); }
      run9(1 + h_rmod(h_rpct(20) ? 3000 : 120), 1);
      if (h_rpct(15)) { A9 b = a; b.st = h_rmod(16); b.n = 0; do9(OP_RESET, &b, 0, NULL); }
    }
  }
  end9(h_rpct(30));
}

/* ------------------------------------------------------------------------------------------------ scenario drivers */
static void trial_dil(void) {
  int flag = h_rpct(50), regime = h_rmod(3), k, nk = 2 + (int)h_rmod(4);
  A9 a;
  new9((unsigned)flag, h_rpct(10), 0);
  for (k = 0; k < nk; k++) {
    int what = (int)h_rmod(8);
    memset(&a, 0, sizeof a);
    a.pcm = h_rmod(2);
    a.code = h_garbage_hi(h_rnd() & 0xffu);
    a.st = h_rpct(70) ? 9u : h_rpct(60) ? 10u : h_rmod(16);
    a.n = h_rpct(85) ? 0u : 1u + h_rmod(40);
    a.base = pick_base();
    a.jdid = 0; a.v92id = 1;
    if (h_rpct(92)) { gen_dil(h_rpct(60) ? 0 : regime); a.dil = &g_dil; }
    if (!do9(OP_RESET, &a, 0, NULL)) { a.n = 0; do9(OP_RESET, &a, 0, NULL); }
    if (what == 0) { do9(OP_RESETDIL, &a, 0, NULL); }
    if (what == 1) { A9 b = a; b.dil = h_rpct(30) ? NULL : (gen_dil(regime), &g_dil); do9(OP_RESETDIL, &b, 0, NULL); }
    if (what == 2) { force9(flag); }
    if (h_rpct(30) && a.dil) {   /* levels exactly at / next to the segment limits (the segment search compares <=) */
      unsigned j, nn = g_o9->dilN_ ? g_o9->dilN_ : 1u;
      for (j = 0; j < nn && j < 12; j++) {
        int d = (int)h_rmod(3) - 1;
        unsigned lim = (unsigned)v9_table[h_rmod(16)];
        g_o9->to_[h_rmod(nn)] = (unsigned short)((int)lim + d);
      }
      m9_load(&g_m9, g_o9);
      do9(OP_UPDSEG, NULL, 0, NULL);
    }
    {
      unsigned i, nsym = 40 + h_rmod(h_rpct(30) ? 6000 : 700);
      for (i = 0; i < nsym; i++) {
        unsigned r = h_rmod(100);
        if (r < 55) do9(OP_SYM, NULL, 0, NULL);
        else if (r < 63) do9(OP_S90, NULL, 0, NULL);
        else if (r < 71) do9(OP_S92, NULL, 0, NULL);
        else if (r < 90) do9(OP_GENDIL, NULL, 0, NULL);
        else if (r < 94) do9(OP_UPDSEG, NULL, 0, NULL);
        else if (r < 98) do9(OP_EXITDIL, NULL, 0, NULL);
        else { force9(flag); }
        if (g_o9->state_ > 0xf && h_rpct(50)) break;
      }
    }
  }
  end9(h_rpct(30));
}

static void trial_v92class(void) {
  A2 a;
  static const int durs[] = { 0, 1, 100, 1000, 3000, 4080, 4090, 5000, 8160, 8170, 20000, -1, -12, -13, -100000, 2147483647, (int)0x80000000, 0x7ffffff0 };
  int fast = h_rpct(50) ? durs[h_rmod(sizeof durs / sizeof durs[0])] : (int)h_rmod(12000);
  int slow = h_rpct(50) ? durs[h_rmod(sizeof durs / sizeof durs[0])] : (int)h_rmod(12000);
  int nat = h_rpct(40), k;
  new2(h_rpct(12), h_rpct(85) ? 0 : 1 + (int)h_rmod(3), fast, slow);
  memset(&a, 0, sizeof a);
  a.st = nat ? 0u : h_rpct(50) ? 0u : h_rmod(16);
  a.n = h_rpct(80) ? 0u : 1u + h_rmod(50);
  a.arg6 = h_rnd();
  a.hasja = h_rpct(85); a.hasdil = h_rpct(50);
  if (h_rpct(12)) { a.hasja = 0; a.hasdil = 1; dsplibs_debug_level = h_rpct(60) ? h_rmod(2) : pick_level(); g_m2.level = dsplibs_debug_level; }
  a.fast = fast; a.slow = slow; a.arg1 = (unsigned short)h_rnd();
  if (!do2(QO_RESET, &a)) { a.n = 0; do2(QO_RESET, &a); }
  if (nat) {
    unsigned i;
    for (i = 0; i < 40000; i++) {
      unsigned st;
      if (!do2(QO_SYM, NULL)) break;
      st = g_o2->state_;
      if (st == 4 && g_o2->cnt_ > 0x18 && h_rmod(60) == 0) do2(QO_EXITJA, NULL);
      if (st == 4 && g_o2->event_ == 3) { if (h_rpct(30)) do2(QO_EXITJA, NULL); }
      if (st == 6 && h_rmod(30) == 0) do2(QO_EXITSIL, NULL);
      if (st == 9 && h_rmod(40) == 0) do2(QO_EXITSU2, NULL);
      if (st == 0xc && h_rmod(40) == 0) do2(QO_EXITTRN, NULL);
      if (h_rmod(300) == 0) { switch (h_rmod(4)) { case 0: do2(QO_EXITJA, NULL); break; case 1: do2(QO_EXITSIL, NULL); break; case 2: do2(QO_EXITSU2, NULL); break; default: do2(QO_EXITTRN, NULL); } }
      if (st >= 0xe && h_rpct(10)) break;
      if (st == 6 && g_o2->cnt_ > 4000) break;
    }
  } else {
    unsigned nf = 3 + h_rmod(14);
    for (k = 0; k < (int)nf; k++) {
      static const unsigned ths[] = { 0x17f, 0x180, 0x17, 0x18, 0x8f, 0x90, 0xb, 0xc, 0x7f7, 0x7f8, 0x1fdf, 0x1fe0, 11, 12, 23, 24, 25, 0x18 + 12, 0xffffffffu, 0xfffffff0u };
      O2 *o = g_o2;
      unsigned i, ns = 1 + h_rmod(h_rpct(15) ? 2500 : 80);
      o->state_ = h_rpct(10) ? (h_rpct(50) ? 16u + h_rmod(5) : 0x80000000u) : h_rmod(16);
      o->cnt_ = h_rpct(30) ? ths[h_rmod(sizeof ths / sizeof ths[0])] : h_rpct(30) ? o->trn1uLen_ - h_rmod(3) : h_rpct(50) ? h_rmod(300) : h_rnd();
      if (h_rpct(12)) {   /* the ends of TRN1u / the second TRN1u with small levels: jaBit = sym <= 0 */
        static const unsigned short lv2[] = { 0, 1, 2, 0xffff, 0xfffe, 0x8000, 0x7fff };
        if (h_rpct(50)) { o->state_ = 3; o->cnt_ = o->trn1uLen_ - h_rmod(2); }
        else { o->state_ = 0xd; o->cnt_ = 12u * (170u + h_rmod(3)) - h_rmod(2); }
        if (h_rpct(70)) o->levelA_ = lv2[h_rmod(7)];
      }
      if (h_rpct(20)) o->jaBit_ = h_rnd();
      if (h_rpct(20)) o->event_ = h_rnd();
      if (h_rpct(10)) dsplibs_debug_level = pick_level();
      if (h_rpct(12)) {
        static const unsigned short lv[] = { 0, 1, 2, 0xffff, 0xfffe, 0x8000, 0x7fff, 0x8001 };
        if (h_rpct(60)) o->levelA_ = h_rpct(75) ? lv[h_rmod(8)] : (unsigned short)h_rnd();
        if (h_rpct(40)) o->levelB_ = h_rpct(75) ? lv[h_rmod(8)] : (unsigned short)h_rnd();
      }
      m2_load(&g_m2, o);
      for (i = 0; i < ns; i++) {
        unsigned r = h_rmod(100);
        if (r < 82) { if (!do2(QO_SYM, NULL)) break; }
        else if (r < 86) do2(QO_EXITJA, NULL);
        else if (r < 90) do2(QO_EXITSIL, NULL);
        else if (r < 94) do2(QO_EXITSU2, NULL);
        else if (r < 97) do2(QO_EXITTRN, NULL);
        else { A2 b = a; b.st = h_rmod(16); b.n = 0; do2(QO_RESET, &b); }
      }
    }
  }
  end2(h_rpct(30));
}

/* small functions: the dead generators of both classes, setSessionFlag, reset argument edge cases */
static void trial_small(void) {
  int which = (int)h_rmod(3);
  if (which == 0 || which == 1) {
    unsigned k, n = 40 + h_rmod(300);
    int flag = h_rpct(50);
    A9 a;
    new9((unsigned)flag, h_rpct(50), h_rpct(80) ? 0 : 1);
    memset(&a, 0, sizeof a);
    a.pcm = h_rmod(6);
    a.code = h_garbage_hi(h_rnd() & 0xffu);
    a.st = h_rmod(16); a.n = 0; a.base = pick_base();
    a.jdid = h_rpct(85) ? (int)h_rmod(3) : -1; a.v92id = h_rpct(85) ? (int)h_rmod(3) : -1;
    do9(OP_RESET, &a, 0, NULL);
    for (k = 0; k < n; k++) {
      static const int ops[] = { OP_SD, OP_SDNOT, OP_JD, OP_JDNOT, OP_TRN1D, OP_JDPH, OP_V92JD, OP_EXITJD, OP_EXITJDPH, OP_EXITDIL, OP_SETFLAG, OP_SYM, OP_RESET };
      int op = ops[h_rmod(sizeof ops / sizeof ops[0])];
      O9 *o = g_o9;
      if (h_rpct(40)) { o->cnt_ = h_rpct(40) ? 0 : pick_cnt(o->timeBase_, flag); o->state_ = h_rpct(60) ? (h_rpct(50) ? 3u : 5u) : pick_state(0); if (h_rpct(10)) o->segCnt_ = h_rmod(3); m9_load(&g_m9, o); }
      if (op == OP_RESET) { a.pcm = h_rmod(2); a.st = h_rmod(16); a.n = h_rpct(80) ? 0u : h_rmod(5); a.jdid = h_rpct(80) ? (int)h_rmod(3) : -1; a.v92id = h_rpct(80) ? (int)h_rmod(3) : -1; a.dil = NULL; if (h_rpct(30)) { gen_dil(0); a.dil = &g_dil; } if (!do9(op, &a, 0, NULL)) { a.n = 0; do9(op, &a, 0, NULL); } }
      else do9(op, &a, h_rpct(50) ? 0u : h_rpct(50) ? 1u : h_rnd(), NULL);
    }
    end9(h_rpct(50));
  } else {
    unsigned k, n = 40 + h_rmod(300);
    A2 a;
    int fast = (int)h_rmod(9000), slow = (int)h_rmod(9000);
    new2(h_rpct(50), h_rpct(80) ? 0 : 1, fast, slow);
    memset(&a, 0, sizeof a);
    a.fast = fast; a.slow = slow;
    a.hasja = 1; a.hasdil = (int)h_rmod(2);
    do2(QO_RESET, &a);
    if (h_rpct(30)) { a.hasja = 0; a.hasdil = 1; }
    for (k = 0; k < n; k++) {
      static const int ops[] = { QO_RU, QO_RUNOT, QO_SU, QO_SUNOT, QO_JA, QO_TRN1U, QO_EXITJA, QO_EXITSIL, QO_EXITSU2, QO_EXITTRN, QO_SYM };
      O2 *o = g_o2;
      if (h_rpct(60)) { o->cnt_ = h_rpct(40) ? 0 : pick_cnt(0, 1); o->state_ = h_rpct(70) ? (h_rpct(50) ? 4u : 5u) : pick_state(1); if (h_rpct(20)) o->jaBit_ = h_rnd(); m2_load(&g_m2, o); }
      if (h_rpct(15)) { static const unsigned short lv[] = { 0, 1, 2, 0xffff, 0xfffe, 0x8000, 0x7fff }; o->levelA_ = lv[h_rmod(7)]; o->levelB_ = lv[h_rmod(7)]; m2_load(&g_m2, o); }
      do2(ops[h_rmod(sizeof ops / sizeof ops[0])], &a);
    }
    end2(h_rpct(50));
  }
}

/* the counters each scenario must reach (frozen from a run: every counter with >= 3 hits; the model branches / exits / resets that scenario is about) */
static const int req0[] = { C_TRIALS, V_SD_PAT0, V_SD_PAT1, V_SD_PAT3, V_SD_PAT4, V_SD_END, V_SDN_PAT, V_SDN_END, V_SDN_STAY, V_TRN_BIT0, V_TRN_BIT1, V_TRN_END_JD, V_JD_X0, V_JD_X1, V_JD_NORMAL, V_JDN_X0, V_JDN_X1, V_JDN_END, V_JDN_NOEND, V_Z_X0, V_Z_X1, V_Z_END_NODIL, V_DIL_NORMAL, V_DIL_END_STAY, V_ST_QUIET11, V_ST_QUIET12, V_ST_QUIET14, V_ST_QUIET15, V_ILL4, V_ILL5, V_ILL6, V_ILL13, V_ILL_HI, V_ILL_PRINT, V_ILL_QUIET, E_JD_NOSTATE, E_JD_MID_V90, E_PH_NOSTATE, E_PH_MID, R9_ALAW, R9_ULAW, R9_V90_JD, R9_V90_NOJD, R9_WARM90, S_C1, S_C2, S_D1, S_D2, 0 };
static const int req1[] = { C_TRIALS, W_TRN_END_JD, W_JD_X0, W_JD_X1, W_JD_NORMAL, W_JD4_END, W_JD4_NOEND, W_PH5_NORMAL, W_PH6_END, W_PH6_NOEND, W_ILL7, W_QUIET13, W_Z_X0, W_Z_X1, W_Z_END_NODIL, E_JD_NOSTATE, E_JD_MID_V92, E_PH_NOSTATE, R9_ALAW, R9_ULAW, R9_V92_JD, R9_V92_NOJD, R9_WARM92, R9_CODE_WRAP, R9_FLAG_GARBAGE, S_C1, S_C2, S_D1, S_D2, S_SETFLAG, 0 };
static const int req2[] = { C_TRIALS, V_SD_PAT0, V_SD_PAT1, V_SD_PAT3, V_SD_PAT4, V_SD_END, V_SDN_PAT, V_SDN_END, V_SDN_STAY, V_TRN_BIT0, V_TRN_BIT1, V_TRN_END_JD, V_TRN_NOJD_PRINT, V_TRN_NOJD_QUIET, V_JD_X0, V_JD_X1, V_JD_TIMEOUT, V_JD_TIMEOUT64, V_JD_NORMAL, V_JDN_X0, V_JDN_X1, V_JDN_END, V_JDN_NOEND, V_Z_X0, V_Z_X1, V_Z_END_DIL, V_DIL_TIMEOUT, V_DIL_NORMAL, V_DIL_TIMEOUT_WRAP, V_DIL_END_PRINT, V_DIL_END_STAY, V_ST_QUIET11, V_ST_QUIET12, V_ST_QUIET14, V_ST_QUIET15, V_ILL4, V_ILL5, V_ILL6, V_ILL13, V_ILL_HI, V_ILL_PRINT, V_ILL_QUIET, W_TRN_END_JD, W_TRN_END_NOJD, W_JD_X0, W_JD_X1, W_JD_TIMEOUT, W_JD_NORMAL, W_JD4_END, W_JD4_NOEND, W_PH5_TIMEOUT, W_PH5_NORMAL, W_PH6_END, W_PH6_NOEND, W_ILL7, W_QUIET13, W_Z_X0, W_Z_X1, W_Z_END_DIL, D_RESET_NULL, D_RESET_ALAW, D_RESET_ULAW, D_TP_ZERO, D_TP_NONZERO, D_SP_NEG, D_SP_POS, D_SEG_CHANGE, D_SEG_STAY, D_TO_WRAP, D_TP_WRAP, D_SP_WRAP, D_SEG8, D_SEG0, D_SEGMID, D_STEP_ALAW, D_STEP_ULAW, D_LTP_OVER128, D_GENDIL, D_UPDSEG, D_EXITDIL_EARLY, D_EXITDIL_BUSY, R9_ALAW, R9_ULAW, R9_V90_JD, R9_V90_NOJD, R9_V92_JD, R9_V92_NOJD, R9_WARM90, R9_WARM92, S_C1, S_C2, S_D1, S_D2, 0 };
static const int req3[] = { C_TRIALS, Q_RU, Q_RUN_END, Q_RUN_STAY, Q_TRN_B0, Q_TRN_B1, Q_TRN_END, Q_JA_FIRST, Q_JA_BITS, Q_JA_X0, Q_JA_X1, Q_JA5_END, Q_JA5_STAY, Q_SU_END, Q_SUN_END, Q_SUN_STAY, Q_SU2, Q_SU3_END, Q_SU3_STAY, Q_SU4_END, Q_SU4_STAY, Q_T12_B0, Q_T12_B1, Q_T13_B0, Q_T13_B1, Q_T13_END, Q_T13_EARLY, Q_T13_NOMUL, Q_SIL6, Q_SIL14, Q_SIL15, Q_ILL2, Q_ILL_HI, Q_ILL_PRINT, Q_ILL_QUIET, QE_JA_NOSTATE, QE_JA_ZERO, QE_JA_MID, QE_SIL_NOSTATE, QE_SIL_ZERO, QE_SIL_GO, QE_SU_NOSTATE, QE_SU_END, QE_SU_MID, QE_TRN_NOSTATE, QE_TRN_ZERO, QE_TRN_GO, QR_LEN_MIN, QR_LEN_COMPUTED, QR_LEN_NEG, QR_JA, QR_NOJA_DIL_PRINT, QR_NOJA_DIL_QUIET, QR_NOJA_NODIL, QR_WARM, QR_NOWARM, S_C1, S_C2, S_D1, S_D2, 0 };
static const int req4[] = { C_TRIALS, V_SD_PAT0, V_SD_PAT1, V_SDN_PAT, V_SDN_STAY, V_TRN_BIT1, V_JD_X0, V_JD_X1, V_JD_NORMAL, V_JDN_X0, V_JDN_X1, V_JDN_NOEND, V_Z_X0, V_DIL_NORMAL, V_DIL_END_STAY, V_ST_QUIET11, V_ST_QUIET12, V_ST_QUIET14, V_ST_QUIET15, V_ILL4, V_ILL5, V_ILL6, V_ILL13, V_ILL_HI, V_ILL_PRINT, V_ILL_QUIET, W_JD_X0, W_JD_X1, W_JD_NORMAL, W_JD4_NOEND, W_PH5_NORMAL, W_PH6_NOEND, W_ILL7, W_QUIET13, W_Z_X0, E_JD_NOSTATE, E_JD_ZERO, E_JD_MID_V90, E_JD_MID_V92, E_PH_NOSTATE, E_PH_ZERO, E_PH_MID, R9_ALAW, R9_ULAW, R9_V90_JD, R9_V90_NOJD, R9_V92_JD, R9_V92_NOJD, R9_WARM90, R9_WARM92, R9_CODE_WRAP, R9_FLAG_GARBAGE, Q_RU, Q_RUN_STAY, Q_JA_FIRST, Q_JA_BITS, Q_JA_X1, Q_JA5_END, Q_JA5_STAY, Q_SU2, Q_SU3_STAY, Q_SIL6, Q_SIL14, Q_ILL_HI, Q_ILL_PRINT, QE_JA_NOSTATE, QE_JA_ZERO, QE_JA_END, QE_JA_MID, QE_SIL_NOSTATE, QE_SU_NOSTATE, QE_TRN_NOSTATE, QR_LEN_MIN, QR_LEN_COMPUTED, QR_JA, QR_NOJA_NODIL, QR_NOWARM, S_C1, S_C2, S_D1, S_D2, S_SETFLAG, S_SD, S_SDNOT, S_JD, S_JDNOT, S_TRN1D, S_JDPH, S_V92JD, S_RU, S_RUNOT, S_SU, S_SUNOT, S_JA, S_TRN1U, 0 };

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, i, tr, bad = 0;
  const char *seed = getenv("P27_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P27_DBG")) g_show = 1;
  p07_layout_check();
  p13_layout_check();
  p27_layout_check();
  h_x87_check();
  h_arena_init();
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(0);
  relax_reserved();
  { static const int *const reqs[5] = { req0, req1, req2, req3, req4 }; const int *r = sc >= 0 && sc < 5 ? reqs[sc] : req0; for (; *r; r++) g_covreq[*r] = 1; }
  switch (sc) {
    case 0: for (tr = 0; tr < 90; tr++) trial_v90class(0, 30); break;
    case 1: for (tr = 0; tr < 90; tr++) trial_v90class(1, 30); break;
    case 2: for (tr = 0; tr < 80; tr++) trial_dil(); break;
    case 3: for (tr = 0; tr < 120; tr++) trial_v92class(); break;
    case 4: for (tr = 0; tr < 150; tr++) trial_small(); break;
    default: fprintf(stderr, "unknown scenario %d\n", sc); return 2;
  }
  h_emit(1, "calls: malloc %lu free %lu edprintf %lu dbgprintf %lu callee %lu skipped %u symbols %lu dumps %lu", h_nmalloc, h_nfree, g_ntot_e, g_ntot_d, g_ntot_c, g_nskipped, g_symctr, g_dumpctr);
  for (i = 1; i < COV_MAX; i++) {
    h_emit(1, "COV %-8s %lu", cov_name(i), g_covn[i]);
    if (g_covreq[i] && g_covn[i] == 0) { fprintf(stderr, "COVERAGE FAIL: counter %d was never reached\n", i); bad = 1; }
  }
  h_cov_finish();
  return (bad && !getenv("P27_NOCOV")) ? 2 : 0;
}
