/*
 * Differential test harness for port chunk P43 (src/vpcm/v90_demodulator_progress.cpp): V90Demodulator::progress(int *out, unsigned &outCnt, float *in, unsigned n), 7,276 B, 235 x87 instructions, four jump
 * tables.  Derived from test/vpcm/v90_equalizer_process.main.c (same recorder / snapshot design).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_demodulator_progress.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port (-O0 / -O2:
 * the port object alone, the callees are the recorders) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_demodulator_progress.blob32.out).  The function is reached through an __asm__ label; the
 * demodulator is the generated C mirror (test/vpcm/v90_demodulator_progress.layout, flattened sub-objects), the objects it points to are the mirrors of P07 (V90Parameters + the modem parameter block), P09 (V90MP), P11
 * (V90MappingParams, tagV90AdditionalCPinfo), P17 (V90ConnectionEvaluator), P18 (V90AutoDigitalImpDetector), P24 (V90ConstellationDesigner), P28 (V90Demapper), P34 (V90Phase3Demodulator), P37
 * (V90Phase4Demodulator), P40 (V90Equalizer): member-wise access only (their layouts differ on x86-64); V90Phase2Info is a local mirror (two members read).  The V90Jd / V92Jd / DIL / CP / TRN2 designer
 * are opaque blocks (only their addresses matter).
 *
 * Every callee is a RECORDER with scripted results (the equalizer's phase event, the Agc power, the resampler counts, the spectral verifier flag, the connection evaluator verdicts, the designer result,
 * the timing offsets ...); some also WRITE the state the function reads afterwards (setBllState writes the bll state, setFilter the filter index, the P44 methods poke state_).  The results carry garbage
 * in the bits the function must not look at.  What is compared: the returned symbols / counts / output buffer, the whole demodulator object with the buffers it owns (the generated field table), every
 * foreign word the function reads or writes (selective hashes), every record (callee, arguments, and a SNAPSHOT hash of the world at the moment of the call: the store order before every call / print), the
 * canary behind the object.
 *
 * Excluded (documented): signalling NaNs in every float that the blob moves as a word or passes by value (the harness feeds quiet NaNs only: g++ -O0 on i386 would quiet them), a bll state / count in the
 * range where the blob would run an unbounded loop (none: the function has no loops), n > the buffer size the object was built for.
 *
 * Scenarios (each its own process): 0 phase 1 (the spectral verifier and the Sd / TRN1d / DIL state handlers, the BLL machine), 1 phase 2 (the timeout, the constellation design events), 2 phases 0 / 3 / 4
 * and garbage phases (the data path, evaluateConnection, the remote retrain detector, the periodic prints, the tail counters).
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
#include "v90_mp_layout.h"
#include "v92_cp_layout.h"
#include "v90_connection_evaluator_layout.h"
#include "v90_adid_layout.h"
#include "v90_constellation_noise_layout.h"
#include "v90_mapper_layout.h"
#include "v90_phase3_demod_v92_decision_layout.h"
#include "v90_phase4_demod_decisions_layout.h"
#include "v90_equalizer_process_layout.h"
#include "v90_demodulator_progress_layout.h"

#define L(m) __asm__(m)

extern unsigned int dsplibs_debug_level;

#if defined(__i386__)
typedef uint32_t FW;                        /* by-value float arguments are raw words on i386 (the x87 would quiet a signalling NaN in the harness' own call; the stack slot is identical) */
static inline uint32_t fw_bits(FW v) { return v; }
static inline FW fw_make(uint32_t b) { return b; }
#else
typedef float FW;                           /* x86-64: SSE keeps the bits */
static inline uint32_t fw_bits(FW v) { uint32_t b; memcpy(&b, &v, 4); return b; }
static inline FW fw_make(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }
#endif

/* the function under test */
extern void f_progress(void *, int *, unsigned *, float *, unsigned) L("_ZN14V90Demodulator8progressEPiRjPfj");

/* the interposed callees */
extern void c_fir(void *, const float *, float *, unsigned) L("_ZN8FloatFIR7processEPKfPfj");
extern void c_agc(void *, const float *, float *, unsigned) L("_ZN3AgcIfE7processEPKfPfj");
extern void c_agcfreeze(void *) L("_ZN3AgcIfE6freezeEv");
extern void c_resample(void *, const float *, unsigned, float *, unsigned *) L("_ZN12V90Resampler8resampleEPKfjPfRj");
extern void c_eqproc(void *, float *, unsigned, short *, float *, unsigned *) L("_ZN12V90Equalizer7processEPfjPsS0_Rj");
extern float c_calc(void *) L("_ZN12V90Equalizer23calcMeanErrorStatisticsEv");
extern void c_restore(void *) L("_ZN12V90Equalizer23restoreEqualizerToFloatEv");
extern unsigned c_demap(void *, unsigned char *, unsigned *) L("_ZN11V90Demapper7processEPhRj");
extern void c_descr(void *, const unsigned char *, int *, unsigned) L("_ZN11DescramblerIhiE7processEPKhPij");
extern unsigned c_svproc(void *, float *, unsigned) L("_ZN19V90SpectralVerifier7processEPfj");
extern void c_svstart(void *) L("_ZN19V90SpectralVerifier17startAccumulationEv");
extern void c_setfilter(void *, unsigned) L("_ZN12V90PreFilter9setFilterEj");
extern int c_v90cap(void *) L("_ZN12V90PreFilter16getV90CapabilityEv");
extern int c_eia6(void *) L("_ZNK12V90PreFilter13isV90WithEia6Ev");
extern void c_bll(void *, unsigned, unsigned) L("_ZN12V90Resampler11setBllStateE11V90BllStatej");
extern long double c_ppm(const void *) L("_ZNK21ResamplerTimingOffset18getTimingOffsetPPMEv");
extern long double c_ppmLD(const void *) L("_ZNK21ResamplerTimingOffset21getTimingOffsetPPM_LDEv");
extern void c_adjbpf(void *, FW) L("_ZN15ResamplerTiming21adjustHalfBaudBpfGainEf");
extern unsigned c_evalconn(void *) L("_ZN22V90ConnectionEvaluator18evaluateConnectionEv");
extern unsigned c_evalp3(void *) L("_ZN22V90ConnectionEvaluator14evaluatePhase3Ev");
extern unsigned c_evalp4(void *, FW) L("_ZN22V90ConnectionEvaluator14evaluatePhase4Ef");
extern unsigned c_evalmean(void *, FW, FW) L("_ZN22V90ConnectionEvaluator26evaluateMeanErrorStdPhase4Eff");
extern unsigned c_indlocal(void *) L("_ZN22V90ConnectionEvaluator20indicateLocalRetrainEv");
extern unsigned c_indremote(void *) L("_ZN22V90ConnectionEvaluator21indicateRemoteRetrainEv");
extern void c_updconst(void *, short, FW, FW, FW) L("_ZN22V90ConnectionEvaluator30updateCurrentConstellationDataEsfff");
extern int c_design(void *, unsigned, void *, FW, int, void *, void *, void *, void *, void *, void *, unsigned char, unsigned, unsigned, unsigned)
    L("_ZN24V90ConstellationDesigner7processEjP25V90AutoDigitalImpDetectorfiP16V90MappingParamsPA128_sS5_PsPhS7_h23__tHardwareCodecTypes__j28V90SpecialSpectralConditions");
extern void c_setconn(void *, short) L("_ZN25V90AutoDigitalImpDetector17setConnectionTypeEs");
extern int c_altrbs(void *) L("_ZN25V90AutoDigitalImpDetector21isThereAnyAltRbsPhaseEv");
extern int c_v90rates(void *) L("_ZN5V90Jd12getRatesMaskEv");
extern unsigned c_v90look(void *) L("_ZN5V90Jd15getMaxLookaheadEv");
extern int c_v92rates(void *) L("_ZN5V92Jd12getRatesMaskEv");
extern unsigned c_v92look(void *) L("_ZN5V92Jd15getMaxLookaheadEv");
extern unsigned char *c_maxucode(void *) L("_ZN20V90Phase3Demodulator11getMaxUcodeEv");
extern void c_setdil(void *) L("_ZN20V90Phase3Demodulator24setDigitalImairmentsInfoEv");
extern void c_p4reset(void *) L("_ZN20V90Phase4Demodulator13resetBeforRRNEv");
extern void c_p4cp(void *) L("_ZN20V90Phase4Demodulator14enterWaitForCPEv");
extern void c_p4ed(void *) L("_ZN20V90Phase4Demodulator14enterWaitForEdEv");
extern void c_p4mp(void *) L("_ZN20V90Phase4Demodulator14enterWaitForMPEv");
#ifndef P44_INTEG   /* P44 integration build (test/vpcm/v90_demodulator.integ.sh): the four P44 methods are the REAL functions (blob / P44 port), not recorders; their callees are the extra recorders below */
extern void c_exit3(void *) L("_ZN14V90Demodulator10exitPhase3Ev");
extern void c_edp(void *) L("_ZN14V90Demodulator14enterDataPhaseEv");
extern void c_eds(void *) L("_ZN14V90Demodulator20enterDataSteadyStateEv");
extern void c_rrn(void *) L("_ZN14V90Demodulator8enterRRNEv");
#else
extern long double c_tmean(void *) L("_ZN12V90Resampler20getTimingHistoryMeanEv");
extern long double c_tstd(void *) L("_ZN12V90Resampler19getTimingHistoryStdEv");
extern long double c_tmeanLD(void *) L("_ZN12V90Resampler22getTimingHistoryMeanLDEv");
extern long double c_tstdLD(void *) L("_ZN12V90Resampler21getTimingHistoryStdLDEv");
extern void c_demlin(void *, unsigned) L("_ZN11V90Demapper20resetLinearMappStudyEj");
extern void c_eqep4(void *) L("_ZN12V90Equalizer11enterPhase4Ev");
extern void c_p3exitdil(void *) L("_ZN20V90Phase3Demodulator7exitDILEv");
extern void c_trnnof(void *, short) L("_ZN15V90TRN2Designer18setNofUcodesInTrn2Es");
extern int c_trndesign(void *, void *, void *, void *, void *, void *, int, int, short, void *, unsigned, unsigned char, unsigned)
    L("_ZN15V90TRN2Designer13V90TRN2DesignEP16V90MappingParamsPA128_sS3_PA128_hPs7PcmTypeS7_sPhjh28V90SpecialSpectralConditions");
extern void c_p4reset2(void *, unsigned char, unsigned, unsigned, unsigned) L("_ZN20V90Phase4Demodulator5resetEh22Phase4DemodulatorStatejj");
#endif
extern void c_dispspec(void *) L("displaySpectralParams");

typedef DmdObj T;
#define SZ sizeof(T)
enum { CAN = 64, NSC = 96, LOGMAX = 400, TXTMAX = 640, NMAX = 48, MAXREC = 4000 };

static uint32_t fnv_add(uint32_t h, const void *p, size_t n) { const uint8_t *q = (const uint8_t *)p; while (n--) { h ^= *q++; h *= 16777619u; } return h; }
static uint32_t fbits(float f) { uint32_t b; memcpy(&b, &f, 4); return b; }
static float ffb(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }

/* local mirror of V90Phase2Info (0x24 bytes): the two members progress reads */
typedef struct { uint32_t pcm_; uint32_t rtd_; uint8_t uinfo_; uint8_t maxTx_; uint8_t pad_[2]; uint32_t rest_[6]; } InfoObj;

/* ------------------------------------------------------------------------------------------------ the world */
typedef struct {
  T *d;
  V90ParObj *par;
  ModParObj *mpar;
  EvObj *ce;
  Ph3DemObj *p3;
  Ph4DemObj *p4;
  DemObj *dem;
  CdObj *cd;
  EquObj *eq;
  AdidObj *adid;
  MpObj *mp;
  MapPar90 *mp1, *mp2;
  AddCp *cpi;
  InfoObj *info;
  void *jd, *jd92, *dil, *cp, *trn2d;
  unsigned char *ucode;      /* the array getMaxUcode returns */
  float *in;                 /* the caller's input (n + 4 floats) */
  int *out;                  /* the caller's output buffer */
  unsigned n, ocap;
  unsigned outCnt;           /* the `unsigned &` argument */
} W;
static W g_w;
static uint8_t *g_can;
static HFld g_fd[160];       /* the demodulator's field table: no pointers except the five owned buffers */
static int g_nfd;

/* ------------------------------------------------------------------------------------------------ scripts, cursors, records */
enum { K_EQ, K_EQ2, K_BUF, K_AGC, K_RS, K_SV, K_SVF, K_BLL, K_CE, K_CE2, K_CD, K_JD, K_PPM, K_PF, K_ADID, K_POKE, K_PRN, K_MISC, K_N };
static uint32_t g_sc[K_N][NSC];
static unsigned g_cur[K_N];
static uint32_t sw(int k) { return g_sc[k][g_cur[k]++ % NSC]; }
typedef struct { char t[TXTMAX]; } Rec;
typedef struct { Rec r[LOGMAX]; unsigned n; unsigned long total; } Log;
static Log g_il;
static int g_show;
static int g_rz;
static unsigned long g_symctr;

static void esc_cat(char *out, size_t cap, const char *s) {
  size_t n = strlen(out);
  for (; *s && n + 6 < cap; s++) {
    if (*s == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
    else if (*s == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
    else out[n++] = *s;
  }
  out[n] = 0;
}

/* the hash of the non-pointer members of an object (member tables are arch independent) */
static uint32_t hash_flds(uint32_t h, const void *obj, const HFld *f, int nf) {
  int i;
  for (i = 0; i < nf; i++) {
    const uint8_t *q = (const uint8_t *)obj + f[i].off;
    unsigned cnt = f[i].count > 1 ? f[i].count : 1u;
    switch (f[i].kind) {
      case 'b': h = fnv_add(h, q, cnt); break;
      case 'h': h = fnv_add(h, q, 2u * cnt); break;
      case 'u': case 'i': case 'f': h = fnv_add(h, q, 4u * cnt); break;
      case 'd': h = fnv_add(h, q, 8u * cnt); break;
      case 'x': h = fnv_add(h, q, f[i].count); break;
      default: break;
    }
  }
  return h;
}
static uint32_t hash_u16(uint32_t h, const uint16_t *p) { return fnv_add(h, p, 2); }
static uint32_t hash_u32(uint32_t h, const void *p) { return fnv_add(h, p, 4); }

/* the snapshot: the demodulator (with its owned buffers), the foreign words the function reads / writes, the caller's buffers */
static uint32_t snap_hash(const W *w) {
  uint32_t h = 2166136261u;
  int i;
  for (i = 0; i < g_nfd; i++) {
    const HFld *f = &g_fd[i];
    const uint8_t *q = (const uint8_t *)w->d + f->off;
    unsigned cnt = f->count > 1 ? f->count : 1u;
    switch (f->kind) {
      case 'b': h = fnv_add(h, q, cnt); break;
      case 'h': h = fnv_add(h, q, 2u * cnt); break;
      case 'u': case 'i': case 'f': h = fnv_add(h, q, 4u * cnt); break;
      case 'd': h = fnv_add(h, q, 8u * cnt); break;
      case 'x': h = fnv_add(h, q, f->count); break;
      case 'o': { void *p; HAl *a; memcpy(&p, q, sizeof p); a = h_find_al(p, 0); if (a) h = fnv_add(h, a->p, a->size); break; }
      default: break;
    }
  }
  h = hash_flds(h, w->ce, f_EvObj, NF_EvObj);
  h = hash_flds(h, w->cd, f_CdObj, NF_CdObj);
  h = hash_flds(h, w->par, f_V90ParObj, NF_V90ParObj);
  h = hash_flds(h, w->mpar, f_ModParObj, NF_ModParObj);
  h = hash_flds(h, w->mp, f_MpObj, NF_MpObj);
  h = hash_flds(h, w->cpi, f_AddCp, NF_AddCp);
  h = hash_u32(h, &w->p3->state_); h = hash_u32(h, &w->p3->cnt_); h = fnv_add(h, &w->p3->impairInfo_, 1); h = fnv_add(h, &w->p3->flag3f9_, 1); h = hash_u16(h, &w->p3->flag400_);
  h = hash_u32(h, &w->p4->state_); h = hash_u32(h, &w->p4->w38_); h = hash_u32(h, &w->p4->w3c_); h = hash_u32(h, &w->p4->w40_); h = hash_u32(h, &w->p4->keepRate_);
  h = hash_u16(h, &w->dem->z1eb4_);
  h = hash_u32(h, &w->eq->event_); h = hash_u16(h, &w->eq->w8_); h = hash_u32(h, &w->eq->meanErr_); h = hash_u32(h, &w->eq->statMean_); h = hash_u32(h, &w->eq->ratio_);
  h = hash_u32(h, &w->adid->padGain_);
  h = hash_u32(h, &w->mp2->w0_); h = hash_u32(h, &w->info->rtd_); h = fnv_add(h, &w->info->maxTx_, 1);
  h = fnv_add(h, w->out, w->ocap * 4u);
  h = hash_u32(h, &w->outCnt);
  return h;
}
static void rec_add(Log *lg, const W *w, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2], snap[64];
  va_list ap;
  Rec *r = &lg->r[lg->n < LOGMAX ? lg->n : LOGMAX - 1];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (g_rz) { fprintf(stderr, "rz check before record: %s\n", buf); h_check_redzones(); }
  snprintf(snap, sizeof snap, "|S=%08x", snap_hash(w));
  r->t[0] = kind; r->t[1] = ' '; r->t[2] = 0;
  esc_cat(r->t, sizeof r->t, buf);
  if (strlen(r->t) + strlen(snap) + 1 >= sizeof r->t) h_die3("record text does not fit (%zu + %zu)", strlen(r->t), strlen(snap));
  strcat(r->t, snap);
  lg->n++; lg->total++;
  if (lg->n > LOGMAX) h_die3("more than %d records in one call", LOGMAX);
}
#define CHECK(p, what) do { if ((void *)(p) != (void *)(what)) h_die3("callee %s got an unexpected object %p (expected %p)", __func__, (void *)(p), (void *)(what)); } while (0)

/* a pointer argument as text: the buffer it points into (and the offset), never a raw address */
static const char *idp(const void *p, char *b) {
  const W *w = &g_w;
  const T *d = w->d;
  const uint8_t *q = (const uint8_t *)p;
  if (!p) return "NULL";
#define IN(base_, sz_, nm_) if (q >= (const uint8_t *)(base_) && q <= (const uint8_t *)(base_) + (sz_)) { snprintf(b, 40, nm_ "+%d", (int)(q - (const uint8_t *)(base_))); return b; }
  IN(w->in, (NMAX + 4) * 4, "in");
  IN(d->inBuf_, NMAX * 4, "b1");
  IN(d->rsBuf_, NMAX * 12, "b2");
  IN(d->decBuf_, NMAX * 4, "b3");
  IN(d->eqBuf_, NMAX * 8, "b4");
  IN(d->byteBuf_, NMAX * 8, "b5");
  IN(w->out, w->ocap * 4, "out");
  IN(w->adid, sizeof(AdidObj), "adid");
  IN(w->ucode, 32, "ucode");
  IN(w->mp2, sizeof(MapPar90), "mp2");
  IN(w->d, SZ, "d");
  return "?";
#undef IN
}

/* ------------------------------------------------------------------------------------------------ pickers (every float is built from bit patterns: no float arithmetic in the data path) */
static uint32_t pick_fb(void) {   /* the BITS of a float parameter: finite most of the time, quiet NaN / inf / 0 / negative sometimes */
  static const uint32_t b[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7f800000u, 0xff800000u, 0x3f800000u, 0xbf800000u, 0x00000001u, 0x7f7fffffu, 0x3a83126fu, 0x358637bdu, 0x38d1b717u,
                                0x3f000000u, 0x40000000u, 0x3f7fffffu, 0x3f800001u, 0x42c80000u, 0x461c4000u, 0x49742400u };
  uint32_t v;
  if (h_rpct(35)) v = b[h_rmod(sizeof b / sizeof b[0])];
  else { v = ((uint32_t)(0x31u + h_rmod(24)) << 23) | h_rmod(1u << 23); if (h_rpct(25)) v |= 0x80000000u; }
  if ((v & 0x7f800000u) == 0x7f800000u && (v & 0x007fffffu)) v |= 0x00400000u;
  return v;
}
static float pick_f(void) { return ffb(pick_fb()); }
static uint32_t pick_cnt(void) { return h_rpct(70) ? h_rmod(200) : h_rpct(50) ? h_rnd() : (h_rpct(50) ? 0xffffffffu : 0x80000000u); }
static uint32_t pick_flag(void) { return h_rpct(40) ? 0u : h_rpct(70) ? 1u : h_rnd(); }
static uint32_t pick_i(void) { return h_rpct(60) ? h_rmod(300) : h_rpct(40) ? (uint32_t)(0u - h_rmod(50)) : h_rnd(); }
static uint32_t pick_u(void) { return h_rpct(80) ? h_rmod(40) : h_rnd(); }
static unsigned rnd_range(unsigned lo, unsigned hi) { return lo + h_rmod(hi - lo + 1); }
static uint32_t quiet_bits(uint32_t b) { return ((b & 0x7f800000u) == 0x7f800000u && (b & 0x007fffffu)) ? (b | 0x00400000u) : b; }

/* ------------------------------------------------------------------------------------------------ the scripted behaviour of the callees (the record text, the world it writes, the result) */
static void maybe_poke(void);
static Log *g_log = &g_il;
#define REC(kind, ...) rec_add(g_log, &g_w, (kind), __VA_ARGS__)

/* the scripted timing offset: an extended value that is NOT a float most of the time (the blob rounds only the first call's value) */
static long double ppm_val(uint32_t a, uint32_t b) {
  switch (a % 24u) {
    case 0: return 0.0L;
    case 1: return -0.0L;
    case 2: return __builtin_nanl("");
    case 3: return -__builtin_nanl("");
    case 4: return __builtin_infl();
    case 5: return -__builtin_infl();
    case 6: return (long double)(int)(b % 2001u) - 1000.0L;
    case 7: return (long double)(int32_t)b * 4294967296.0L;            /* beyond the int range */
    case 8: return (long double)(int32_t)b / 16.0L;                   /* many integers, large */
    case 9: return ((long double)(int32_t)b) / 2147483648.0L;         /* |x| < 1: sign and fraction only */
    case 10: return -(long double)(b % 1000u) / 1048576.0L;           /* tiny negative */
    case 11: return ((long double)(int32_t)b) / 3.0L;                 /* not representable as a float */
    case 12: return ((long double)(int32_t)b) / 1000.0L;
    case 13: return (long double)(b % 100000u) / 7.0L;
    case 14: return -(long double)(b % 100000u) / 7.0L;
    case 15: return 0.9999999L;
    case 16: return -0.9999999L;
    case 17: return 2147483647.0L + (long double)(b % 4u);
    case 18: return -2147483648.0L - (long double)(b % 4u);
    case 19: return (long double)(int32_t)b / 1048576.0L;
    case 20: return ((long double)(int32_t)b) * 1.0e-6L;
    default: return ((long double)(int32_t)b) / 8.0L;
  }
}
static long double env_ppm(void) { uint32_t a = sw(K_PPM), b = sw(K_PPM); REC('c', "getTimingOffsetPPM"); return ppm_val(a, b); }

static void env_fir(const float *in, float *out, unsigned n) {
  unsigned k;
  char b1[48], b2[48];
  REC('c', "FIR(%s,%s,%u)", idp(in, b1), idp(out, b2), n);
  for (k = 0; k < n; k++) out[k] = ffb(quiet_bits(fbits(in[k]) ^ 0x01010101u));
}
static void env_agc(const float *in, float *out, unsigned n) {
  uint32_t r = sw(K_AGC);
  unsigned k;
  char b1[48], b2[48];
  REC('c', "Agc(%s,%s,%u)", idp(in, b1), idp(out, b2), n);
  for (k = 0; k < n; k++) out[k] = ffb(quiet_bits(fbits(in[k]) + 0x00010001u));
  if ((r & 3u) == 0) g_w.d->agc_power_ = ffb(quiet_bits(sw(K_AGC)));
  if (((r >> 2) & 3u) == 0) g_w.d->agc_gain_ = ffb(quiet_bits(sw(K_AGC)));
}
static void env_agcfreeze(void) { REC('c', "Agc.freeze"); g_w.d->agc_mixSaved_ = g_w.d->agc_mix_; g_w.d->agc_mix_ = 1.0f; }
static void env_resample(const float *in, unsigned n, float *out, unsigned *cnt) {
  uint32_t r = sw(K_RS);
  unsigned k, c = (r % 8u == 0) ? 0u : (r >> 4) % (3u * n + 1u);
  char b1[48], b2[48];
  REC('c', "resample(%s,%u,%s)", idp(in, b1), n, idp(out, b2));
  for (k = 0; k < c; k++) out[k] = ffb(quiet_bits(0x3f800000u + k * 0x10001u + (n << 8)));
  *cnt = c;
  g_w.d->rs_stateCount_ += (r >> 20) % 5u == 0 ? (r ^ 0x80008000u) : ((r >> 19) & 1u) ? 0u : (r >> 24) & 0x3fu;   /* half of the calls leave the count alone: the BLL thresholds are hit exactly */
}
static void env_eqproc(float *in, unsigned n, short *dec, float *out, unsigned *cnt) {
  uint32_t r = sw(K_EQ), q = sw(K_EQ2);
  unsigned k, c = (q % 6u == 0) ? 0u : (q >> 4) % (2u * g_w.n + 3u);
  char b1[48], b2[48], b3[48];
  EquObj *e = g_w.eq;
  REC('c', "eqProcess(%s,%u,%s,%s)", idp(in, b1), n, idp(dec, b2), idp(out, b3));
  for (k = 0; k < c && k < NMAX * 2; k++) { dec[k] = (short)(r + k * 37u); out[k] = ffb(quiet_bits(0x40000000u + k * 0x101u + r)); }
  *cnt = c < NMAX * 2u ? c : NMAX * 2u;
  e->event_ = r;
  if ((q >> 16) % 4u == 0) e->w8_ = (uint16_t)q;
  if ((q >> 20) % 3u == 0) e->meanErr_ = ffb(quiet_bits(r * 2654435761u));
  if ((q >> 22) % 3u == 0) e->statMean_ = ffb(quiet_bits(q * 2246822519u));
  if ((q >> 24) % 3u == 0) e->ratio_ = ffb(quiet_bits(r ^ q));
}
static uint32_t env_calc(void) { uint32_t r = sw(K_EQ); REC('c', "calcMeanErrorStatistics"); g_w.eq->statMean_ = ffb(r); return r; }
static void env_restore(void) { REC('c', "restoreEqualizerToFloat"); maybe_poke(); }
static unsigned env_demap(unsigned char *out, unsigned *cnt) {
  uint32_t r = sw(K_BUF);
  unsigned k, c = (r % 5u == 0) ? 0u : (r >> 8) % (8u * g_w.n + 1u);
  char b1[48];
  REC('c', "demap(%s)", idp(out, b1));
  for (k = 0; k < c && k < NMAX * 8; k++) out[k] = (unsigned char)(r + k * 5u);
  *cnt = c < NMAX * 8u ? c : NMAX * 8u;
  return r * 2654435761u;
}
static void env_descr(const unsigned char *in, int *out, unsigned n) {
  unsigned k;
  char b1[48], b2[48];
  REC('c', "descr(%s,%s,%u)", idp(in, b1), idp(out, b2), n);
  for (k = 0; k < n && k < g_w.ocap; k++) out[k] = (int)(0x1000u + in[k] + k * 3u);
}
static unsigned env_svproc(float *in, unsigned n) {
  uint32_t r = sw(K_SV), f = sw(K_SVF);
  char b1[48];
  unsigned res = (r % 3u == 0) ? 0u : ((r & 0x80000000u) ? (r | 1u) : 1u);
  REC('c', "svProcess(%s,%u)", idp(in, b1), n);
  if ((f >> 24) % 6u == 0) g_w.d->phase_ = (f >> 28) % 5u;   /* the real verifier cannot change phase_: the function re-reads it in the state 0x14 handler, which this exercises */
  if (res) g_w.d->sv_flag_ = (f % 7u < 5u) ? f % 7u : (f % 7u == 5u ? 0xffffffffu : f);
  return res;
}
static void env_svstart(void) { REC('c', "svStartAccumulation"); maybe_poke(); }
static void env_setfilter(unsigned idx) { uint32_t r = sw(K_PF); REC('c', "setFilter(%u)", idx); g_w.d->pf_filterIdx_ = (int32_t)((r % 5u == 0) ? r : idx); }
static int env_v90cap(void) { uint32_t r = sw(K_PF); REC('c', "getV90Capability"); return (int)((r % 3u == 0) ? 0u : r | 1u); }
static int env_eia6(void) { uint32_t r = sw(K_PF); REC('c', "isV90WithEia6"); return (int)((r >> 4) & 1u); }
static void env_bll(unsigned st, unsigned fl) {
  uint32_t r = sw(K_BLL);
  REC('c', "setBllState(%u,%u)", st, fl);
  g_w.d->rs_state_ = st;
  g_w.d->rs_stateCount_ = (r & 1u) ? 0u : (r >> 1);
}
static void env_adjbpf(uint32_t b) { REC('c', "adjustHalfBaudBpfGain(%08x)", b); }
static unsigned env_ceverdict(const char *nm, const char *args) {
  uint32_t r = sw(K_CE);
  static const unsigned t[] = { 0, 1, 2, 3, 4, 5, 4, 5, 0, 5, 4, 6, 0x80000000u, 0xffffffffu, 0, 4, 5 };
  unsigned v = (r & 0xffu) < 150u ? t[(r >> 8) % (sizeof t / sizeof t[0])] : t[(r >> 8) % 6u];
  REC('c', "%s(%s) -> %u", nm, args, v);
  return v;
}
static void env_updconst(int s, uint32_t a, uint32_t b, uint32_t c) { REC('c', "updateCurrentConstellationData(%d,%08x,%08x,%08x)", s, a, b, c); }
static int env_design(unsigned la, void *adid, uint32_t noise, int rateMask, void *mp, void *A, void *B, void *dual, void *fixedU, void *cur, unsigned pl, unsigned codec, unsigned a13, unsigned sp) {
  uint32_t r = sw(K_CD);
  char b0[48], b1[48], b2[48], b3[48], b4[48], b5[48], b6[48];
  REC('c', "design(%u,%s,%08x,%d,%s,%s,%s,%s,%s,%s,%u,%u,%u,%u)", la, idp(adid, b0), noise, rateMask, idp(mp, b1), idp(A, b2), idp(B, b3), idp(dual, b4), idp(fixedU, b5), idp(cur, b6), pl, codec, a13, sp);
  return (int)((r % 3u == 0) ? 1u : (r % 3u == 1) ? 2u : r);
}
static void env_setconn(int t) { REC('c', "setConnectionType(%d)", t); maybe_poke(); }
static int env_altrbs(void) { uint32_t r = sw(K_ADID); REC('c', "isThereAnyAltRbsPhase"); return (int)((r % 3u == 0) ? (r & 0xffff0000u) : (r % 3u == 1) ? (r | 0x10000u) | 1u : (r & 0xffffu) | 0x8000u); }
static int env_rates(const char *nm) { uint32_t r = sw(K_JD); REC('c', "%s", nm); maybe_poke(); return (int)(r & 0xffffu); }
static unsigned env_look(const char *nm) { uint32_t r = sw(K_JD); REC('c', "%s", nm); maybe_poke(); return r; }
static unsigned char *env_maxucode(void) { REC('c', "getMaxUcode"); maybe_poke(); return g_w.ucode; }
static void env_setdil(void) { REC('c', "setDigitalImairmentsInfo"); maybe_poke(); }
static void env_p4(const char *nm) { REC('c', "p4.%s", nm); maybe_poke(); }
/* the P44 methods of V90Demodulator: they poke the object (the function reloads state_ after enterRRN) */
static void poke_state(uint32_t r) {
  static const uint32_t st[] = { 0x23, 0x25, 0x1f, 0x21, 0x22, 0x26, 0, 0x24, 0xff, 0x80000000u };
  if ((r >> 8) % 3u == 0) g_w.d->state_ = st[(r >> 12) % (sizeof st / sizeof st[0])];
  if ((r >> 16) % 4u == 0) g_w.d->phase_ = (r >> 20) % 5u;   /* the real methods enter / leave phases */
}
/* a callee that changes a word the function reads (again) after the call: the blob caches some words in callee-saved registers across calls and reloads others; the pokes (script words) make the load timing of every
 * such word observable.  Applied by the callees of the design / phase-4 / print blocks. */
static void maybe_poke(void) {
  uint32_t r = sw(K_POKE), v = sw(K_POKE);
  W *w = &g_w;
  if (r % 9u != 0) return;
  switch ((r >> 8) % 14u) {
    case 0: w->d->sv_flag_ = v % 5u; break;
    case 1: w->d->session_ ^= 1u; break;
    case 2: w->d->retrainOk_ ^= 1u; break;
    case 3: w->ce->retrainInsteadOfRateDown_ ^= 1; break;
    case 4: w->ce->rrnSilence_ ^= 1; break;
    case 5: w->info->maxTx_ = (uint8_t)v; break;
    case 6: w->cd->w24_ ^= 1u; break;
    case 7: w->d->hwCodec_ = (v & 1u) ? 4u : 0u; break;
    case 8: w->p4->w3c_ ^= 1u; break;
    case 9: w->p4->w38_ ^= 1u; break;
    case 10: w->eq->statMean_ = ffb(quiet_bits(v)); break;
    case 11: w->par->probingMode ^= 1; break;
    case 12: w->mp->type_ = (uint8_t)(v & 3u); break;
    default: w->par->rrnSilenceRequested ^= 1; break;
  }
}
static void env_exit3(void) { uint32_t r = sw(K_POKE); REC('c', "exitPhase3"); poke_state(r); if ((r >> 24) % 4u == 0) g_w.d->flag40_ = r >> 28; }
static void env_edp(void) { uint32_t r = sw(K_POKE); REC('c', "enterDataPhase"); poke_state(r); }
static void env_eds(void) { uint32_t r = sw(K_POKE); REC('c', "enterDataSteadyState"); poke_state(r); }
static void env_rrn(void) { uint32_t r = sw(K_POKE); REC('c', "enterRRN"); poke_state(r); }
static void env_dispspec(void *mp) { char b[48]; REC('c', "displaySpectralParams(%s)", idp(mp, b)); maybe_poke(); }
static void env_print(char kind, const char *fmt, va_list ap) {
  char buf[TXTMAX * 2];
  uint32_t r = sw(K_PRN);
  vsnprintf(buf, sizeof buf, fmt, ap);
  REC(kind, "%s", buf);
  maybe_poke();
  if (r % 5u == 0) dsplibs_debug_level = (r >> 8) % 3u;   /* a print that changes the level: the function re-tests it before every print */
}

/* the impl's recorders */
void c_fir(void *p, const float *in, float *out, unsigned n) { CHECK(p, &g_w.d->pf_fir_coef_); env_fir(in, out, n); }
void c_agc(void *p, const float *in, float *out, unsigned n) { CHECK(p, &g_w.d->agc_mix_); env_agc(in, out, n); }
void c_agcfreeze(void *p) { CHECK(p, &g_w.d->agc_mix_); env_agcfreeze(); }
void c_resample(void *p, const float *in, unsigned n, float *out, unsigned *cnt) { CHECK(p, &g_w.d->rs_vptr_); env_resample(in, n, out, cnt); }
void c_eqproc(void *p, float *in, unsigned n, short *dec, float *out, unsigned *cnt) { CHECK(p, g_w.eq); env_eqproc(in, n, dec, out, cnt); }
float c_calc(void *p) { CHECK(p, g_w.eq); return ffb(env_calc()); }
void c_restore(void *p) { CHECK(p, g_w.eq); env_restore(); }
unsigned c_demap(void *p, unsigned char *out, unsigned *cnt) { CHECK(p, g_w.dem); return env_demap(out, cnt); }
void c_descr(void *p, const unsigned char *in, int *out, unsigned n) { CHECK(p, &g_w.d->scr_buf_); env_descr(in, out, n); }
unsigned c_svproc(void *p, float *in, unsigned n) { CHECK(p, &g_w.d->sv_params_); return env_svproc(in, n); }
void c_svstart(void *p) { CHECK(p, &g_w.d->sv_params_); env_svstart(); }
void c_setfilter(void *p, unsigned idx) { CHECK(p, &g_w.d->pf_fir_coef_); env_setfilter(idx); }
int c_v90cap(void *p) { CHECK(p, &g_w.d->pf_fir_coef_); return env_v90cap(); }
int c_eia6(void *p) { CHECK(p, &g_w.d->pf_fir_coef_); return env_eia6(); }
void c_bll(void *p, unsigned st, unsigned fl) { CHECK(p, &g_w.d->rs_vptr_); env_bll(st, fl); }
long double c_ppm(const void *p) { CHECK(p, &g_w.d->rs_vptr_); return env_ppm(); }
long double c_ppmLD(const void *p) { CHECK(p, &g_w.d->rs_vptr_); return env_ppm(); }
void c_adjbpf(void *p, FW x) { CHECK(p, &g_w.d->rs_vptr_); env_adjbpf(fw_bits(x)); }
unsigned c_evalconn(void *p) { CHECK(p, g_w.ce); return env_ceverdict("evaluateConnection", ""); }
unsigned c_evalp3(void *p) { CHECK(p, g_w.ce); return env_ceverdict("evaluatePhase3", ""); }
unsigned c_evalp4(void *p, FW x) { char b[32]; CHECK(p, g_w.ce); snprintf(b, sizeof b, "%08x", fw_bits(x)); return env_ceverdict("evaluatePhase4", b); }
unsigned c_evalmean(void *p, FW a, FW c) { char b[40]; CHECK(p, g_w.ce); snprintf(b, sizeof b, "%08x,%08x", fw_bits(a), fw_bits(c)); return env_ceverdict("evaluateMeanErrorStdPhase4", b); }
unsigned c_indlocal(void *p) { CHECK(p, g_w.ce); return env_ceverdict("indicateLocalRetrain", ""); }
unsigned c_indremote(void *p) { CHECK(p, g_w.ce); return env_ceverdict("indicateRemoteRetrain", ""); }
void c_updconst(void *p, short s, FW a, FW b, FW c) { CHECK(p, g_w.ce); env_updconst((int)s, fw_bits(a), fw_bits(b), fw_bits(c)); }
int c_design(void *p, unsigned la, void *adid, FW noise, int rm, void *mp, void *A, void *B, void *dual, void *fixedU, void *cur, unsigned char pl, unsigned codec, unsigned a13, unsigned sp) {
  CHECK(p, g_w.cd);
  return env_design(la, adid, fw_bits(noise), rm, mp, A, B, dual, fixedU, cur, (unsigned)pl, codec, a13, sp);
}
void c_setconn(void *p, short t) { CHECK(p, g_w.adid); env_setconn((int)t); }
int c_altrbs(void *p) { CHECK(p, g_w.adid); return env_altrbs(); }
int c_v90rates(void *p) { CHECK(p, g_w.jd); return env_rates("V90Jd.getRatesMask"); }
unsigned c_v90look(void *p) { CHECK(p, g_w.jd); return env_look("V90Jd.getMaxLookahead"); }
int c_v92rates(void *p) { CHECK(p, g_w.jd92); return env_rates("V92Jd.getRatesMask"); }
unsigned c_v92look(void *p) { CHECK(p, g_w.jd92); return env_look("V92Jd.getMaxLookahead"); }
unsigned char *c_maxucode(void *p) { CHECK(p, g_w.p3); return env_maxucode(); }
void c_setdil(void *p) { CHECK(p, g_w.p3); env_setdil(); }
void c_p4reset(void *p) { CHECK(p, g_w.p4); env_p4("resetBeforRRN"); }
void c_p4cp(void *p) { CHECK(p, g_w.p4); env_p4("enterWaitForCP"); }
void c_p4ed(void *p) { CHECK(p, g_w.p4); env_p4("enterWaitForEd"); }
void c_p4mp(void *p) { CHECK(p, g_w.p4); env_p4("enterWaitForMP"); }
#ifndef P44_INTEG
void c_exit3(void *p) { CHECK(p, g_w.d); env_exit3(); }
void c_edp(void *p) { CHECK(p, g_w.d); env_edp(); }
void c_eds(void *p) { CHECK(p, g_w.d); env_eds(); }
void c_rrn(void *p) { CHECK(p, g_w.d); env_rrn(); }
#else
long double c_tmean(void *p) { uint32_t a = sw(K_PPM), b = sw(K_PPM); CHECK(p, &g_w.d->rs_vptr_); REC('c', "getTimingHistoryMean"); return ppm_val(a, b); }
long double c_tstd(void *p) { uint32_t a = sw(K_PPM), b = sw(K_PPM); CHECK(p, &g_w.d->rs_vptr_); REC('c', "getTimingHistoryStd"); return ppm_val(a, b); }
long double c_tmeanLD(void *p) { return c_tmean(p); }
long double c_tstdLD(void *p) { return c_tstd(p); }
void c_demlin(void *p, unsigned len) { CHECK(p, g_w.dem); REC('c', "dem.resetLinearMappStudy(%u)", len); maybe_poke(); }
void c_eqep4(void *p) { CHECK(p, g_w.eq); REC('c', "eq.enterPhase4"); maybe_poke(); }
void c_p3exitdil(void *p) { uint32_t r = sw(K_MISC); CHECK(p, g_w.p3); REC('c', "p3.exitDIL"); if (r & 3u) g_w.p3->event_ = (r & 4u) ? 0x14u : (r >> 8); maybe_poke(); }
void c_trnnof(void *p, short n) { CHECK(p, g_w.trn2d); REC('c', "setNofUcodesInTrn2(%d)", (int)n); maybe_poke(); }
int c_trndesign(void *p, void *mp, void *A, void *B, void *C, void *D, int pcmA, int pcm, short unused, void *cur, unsigned lim, unsigned char pidx, unsigned ssc) {
  uint32_t r = sw(K_MISC);
  char b0[48], b1[48], b2[48], b3[48], b4[48], b5[48];
  CHECK(p, g_w.trn2d);
  REC('c', "V90TRN2Design(%s,%s,%s,%s,%s,%d,%d,%d,%s,%u,%u,%u)", idp(mp, b0), idp(A, b1), idp(B, b2), idp(C, b3), idp(D, b4), pcmA, pcm, (int)unused, idp(cur, b5), lim, (unsigned)pidx, ssc);
  maybe_poke();
  return (int)((r % 3u == 0) ? (r & 0xffff0000u) : (r % 3u == 1) ? (r | 1u) : r);
}
void c_p4reset2(void *p, unsigned char code, unsigned st, unsigned a3, unsigned a4) { CHECK(p, g_w.p4); REC('c', "p4.reset(%u,%u,%u,%u)", (unsigned)code, st, a3, a4); maybe_poke(); }
#endif
void c_dispspec(void *p) { env_dispspec(p); }
void edprintf(const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print('e', fmt, ap); va_end(ap); }
int dsplibs_debug_printf(const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print('d', fmt, ap); va_end(ap); return 0; }

/* ------------------------------------------------------------------------------------------------ the independent model */
/* written from porting/decompiled/dp/p43_demodulator_progress.txt in a different shape than the port (one function per phase, if / else, no gotos); its own extended-precision helpers.  It runs on the LIVE
 * world first (the world is saved before and restored afterwards), through the same callee environment, and must leave the same world, outputs and records (with the same snapshot hashes) as the impl. */
typedef long double XL;
enum { G_ALL = 7, G_S0 = 1, G_S1 = 2, G_S2 = 4, G_S01 = 3, G_S02 = 5, G_S12 = 6 };   /* the scenarios that must reach a site */
typedef struct { unsigned long n; unsigned char hit, grp; const char *name; } CovSite;
#define CV(g, nm) do { static CovSite cs_ __attribute__((section("covsites"), used, aligned(sizeof(void *)))) = { 0, 0, (g), (nm) }; cs_.hit = 1; } while (0)
extern CovSite __start_covsites[], __stop_covsites[];
static void cov_flush(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) if (c->hit) { c->n++; c->hit = 0; } }
static void cov_clear(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) c->hit = 0; }

static int m_f32(XL v) { if (v != v || v >= 2147483648.0L || v <= -2147483649.0L) return INT_MIN; return (int)v; }
static int m_f16(XL v) { if (v != v || v >= 32768.0L || v <= -32769.0L) return -32768; return (int)v; }
static int m_abs(int v) { return v < 0 ? (int)(0u - (unsigned)v) : v; }
static int m_sgn(XL v) { return (v > 0.0L || v != v) ? '+' : '-'; }
static float m_rf(XL v) { volatile float t = (float)v; return t; }
static XL m_fabs(XL v) { return v < 0.0L ? -v : v; }
static void m_pf(char kind, const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print(kind, fmt, ap); va_end(ap); }
static int m_dbg(void) { return dsplibs_debug_level > 1; }
static void m_fix(char kind, const char *fmt, XL v, XL scale) { int t = m_f32(v); m_pf(kind, fmt, m_sgn(v), m_f32(m_fabs(v)), m_abs(m_f32((v - (XL)t) * scale))); }
static void m_means(const char *fmt, XL a, XL b) {
  int ta = m_f32(a), tb = m_f32(b);
  m_pf('e', fmt, m_sgn(a), m_f32(m_fabs(a)), m_abs(m_f32((a - (XL)ta) * 10000.0L)), m_sgn(b), m_f32(m_fabs(b)), m_abs(m_f32((b - (XL)tb) * 10000.0L)));
}
static uint32_t m_w(const float *p) { uint32_t b; memcpy(&b, p, 4); return b; }   /* the bits of a float in memory (no float copy: the x87 would quiet a signalling NaN) */
static uint32_t m_wv(float f) { volatile float t = f; return fbits(t); }
static unsigned m_retrain_state(void) { return g_w.d->retrainOk_ == 0 ? 0x1fu : 0x21u; }
#define PFX "V90Demodulator: "

static void m_set_timing_period(T *d) { d->errPrintPeriod_ = (uint32_t)g_w.par->errorEnergyPrintPeriodPhase4; d->tmPrintPeriod_ = (uint32_t)g_w.par->timingOffsetPrintPeriodPhase4; }

/* ---- the BLL machine of the resampler (phase 3) ---- */
static void m_bll_machine(void) {
  T *d = g_w.d;
  V90ParObj *p = g_w.par;
  unsigned bs = d->rs_state_, sc = d->rs_stateCount_;
  unsigned ro = d->retrainOk_;
  if (ro == 0) {
    if (bs == 2 && p->bllTrn1dInitialToFastDuration < (int32_t)sc) { CV(G_S0, "bll 2 -> 3"); env_bll(3, 1); }
    else if (bs == 3 && p->bllTrn1dFastToSlowDuration < (int32_t)sc) { CV(G_S0, "bll 3 -> 4"); env_bll(4, 1); }
    else if (bs == 5 && p->unk100 < (int32_t)sc) { CV(G_S0, "bll 5 -> 6"); env_bll(6, 1); }
  } else {
    unsigned ro2 = d->retrainOk_;
    (void)ro2;
    if (bs == 0xc && p->unk104 < (int32_t)sc) { CV(G_S0, "bll c -> d"); env_bll(0xd, 1); }
    else if (bs == 0xd && p->unk108 < (int32_t)sc) { CV(G_S0, "bll d -> e"); env_bll(0xe, 1); }
    else if (bs == 0xe && p->unk10c < (int32_t)sc) { CV(G_S0, "bll e -> f"); env_bll(0xf, 1); }
  }
}

/* ---- phase 3: the Agc freeze + DIL protection (p3 state 3), then evaluatePhase3 ---- */
static void m_phase1_eval(void) {
  T *d = g_w.d;
  V90ParObj *p = g_w.par;
  if (g_w.p3->state_ == 3 && p->agcAdaptationDuration < (int32_t)g_w.p3->cnt_ && (XL)d->agc_mix_ != 1.0L && (XL)d->agc_mix_ == (XL)d->agc_mix_) {
    CV(G_S0, "agc freeze");
    env_agcfreeze();
    m_pf('e', PFX "Agc Frozen\r\n");
    m_fix('e', PFX "Agc Energy = %c%d.%02d\r\n", (XL)d->agc_power_, 100.0L);
    m_fix('e', PFX "Agc Gain = %c%d.%06d\r\n", (XL)d->agc_gain_, 1000000.0L);
    if (d->retrainOk_ == 0) { if (env_eia6() == 0) { CV(G_S0, "agc: adjust bpf gain"); env_adjbpf(fbits(d->agc_gain_)); } else CV(G_S0, "agc: eia6 no adjust"); }
    d->flag40_ = 1;
    {
      XL gain = (XL)d->agc_gain_, u = (XL)p->unk6c;
      if (u > gain) {
        CV(G_S0, "dil: u6c > gain");
        if (u * 0.85L > gain) {
          CV(G_S0, "dil: 0.85 u6c > gain");
          g_w.p3->flag400_ = 1;
        } else {
          unsigned short v = (unsigned short)m_f16((250.0L * (1.0L - (1.0L / u) * gain)));
          CV(G_S0, "dil: divisor of the equalizer");
          g_w.eq->w8_ = (unsigned short)(v + 1u);
        }
        g_w.p3->impairInfo_ = 0x74;
        if ((XL)d->agc_gain_ >= (XL)p->unk70) { CV(G_S0, "dil: gain >= u70"); if (m_dbg()) m_pf('d', PFX "Agc Gain low > Setting DIL overflow protection !!!\r\n"); }
        else { CV(G_S0, "dil: gain < u70"); p->unk300 = p->unk300 - p->unk304; if (m_dbg()) m_pf('d', PFX "Agc Gain very low > Setting DIL extreme overflow protection !!!\r\n"); }
      } else { CV(G_S0, "dil: u6c <= gain"); g_w.p3->impairInfo_ = 0x74; }
      m_pf('e', PFX "Dil max ucode = %d\n", (unsigned)g_w.p3->impairInfo_);
    }
  }
  {
    unsigned r = env_ceverdict("evaluatePhase3", "");
    if (r == 4) { CV(G_S0, "eval p3 = 4"); d->state_ = 0x21; }
    else if (r == 5) { CV(G_S0, "eval p3 = 5"); d->state_ = m_retrain_state(); }
  }
}

/* ---- phase 1 ---- */
static void m_phase1(float *in, unsigned n) {
  T *d = g_w.d;
  V90ParObj *p = g_w.par;
  W *w = &g_w;
  int f;
  unsigned st;
  if (env_svproc(in, n) != 0) {
    f = (int)d->sv_flag_;
    if (f == 1) {
      CV(G_S0, "sv flag 1");
      env_setfilter((unsigned)p->germanIsdnNt1BoxFilterGain); m_pf('e', PFX "PreFilter adjusted to: %d\r\n", d->pf_filterIdx_);
      memcpy(&p->linearEquDataBeta, &p->germanIsdnNt1LinearEquDataBeta, 4);
      env_setconn(1); w->mpar->connectionType = 1;
    } else if (f == 0) {
      CV(G_S0, "sv flag 0");
      env_setconn(0); w->mpar->connectionType = 0;
    } else if (f == 2) {
      CV(G_S0, "sv flag 2");
      env_setfilter((unsigned)p->germanPbxPreFilterGain); m_pf('e', PFX "PreFilter adjusted to: %d\r\n", d->pf_filterIdx_);
      memcpy(&p->linearEquDataBeta, &p->germanPbxLinearEquDataBeta, 4); memcpy(&p->dfeDataBeta, &p->germanPbxDfeDataBeta, 4);
      memcpy(&p->linearEquDilBeta, &p->germanPbxLinearEquDilBeta, 4); memcpy(&p->linearEquDilMedUcodeBeta, &p->germanPbxLinearEquDilMedUcodeBeta, 4);
      memcpy(&p->linearEquDilHighUcodeBeta, &p->germanPbxLinearEquDilHighUcodeBeta, 4);
      env_setconn(2); w->mpar->connectionType = 2;
    } else if (f == 3) {
      if (env_eia6() != 0) { CV(G_S0, "sv flag 3: eia6"); m_pf('e', PFX "Severe Codec conditions were detected due to EIA6 loop type...\r\n"); }
      else if (p->enableDrop2V34OnSevereCodec == 0) { CV(G_S0, "sv flag 3: masked"); m_pf('e', PFX "Severe Codec conditions were detected NOT on EIA6, drop is masked, doing nothing...\r\n"); }
      else { CV(G_S0, "sv flag 3: drop"); m_pf('e', PFX "Severe Codec conditions were detected NOT on EIA6, initiating drop 2 V34...\r\n"); d->state_ = 0x1f; }
    } else { CV(G_S0, "sv flag illegal"); m_pf('e', PFX "!!! ERROR !!! : Spectral Verifier returned illegal value !!!\r\n"); }
  }
  st = d->state_;
  switch (st) {
    case 1:
      CV(G_S0, "p3 state 1");
      if (env_v90cap() == 0) { CV(G_S0, "p3 state 1: no capability"); d->state_ = 0x1f; if (m_dbg()) m_pf('d', PFX "Request FallBack to V.34 due to line conditions\r\n"); }
      break;
    case 3:
      CV(G_S0, "p3 state 3");
      env_svstart();
      if (d->retrainOk_ == 0) { CV(G_S0, "p3 state 3: agc activated"); memcpy(&d->agc_mix_, &p->agcK, 4); m_pf('e', PFX "Agc Activated\r\n"); }
      else d->flag40_ = 1;
      break;
    case 4:
      CV(G_S0, "p3 state 4");
      if (d->retrainOk_ == 0) { CV(G_S0, "p3 state 4: bll 5"); env_bll(5, 1); }
      break;
    case 5: case 6:
      CV(G_S0, "p3 state 5/6");
      if (p->probingMode == 0) {
        if (d->retrainOk_ == 0) {
          CV(G_S0, "p3 state 5/6: enable trn1d");
          w->ce->trn1dEvalActive_ = 1; w->ce->weight_ = 0; w->ce->avePdsnr_ = 0.0f;
          m_pf('e', PFX "enabling connectionEvaluator of TRN1d\r\n");
        } else {
          if ((int16_t)env_altrbs() != 0) { CV(G_S0, "p3 state 5/6: alt rbs"); w->ce->altRbsDetectedOnQc_ = 1; m_pf('e', PFX "setAltRbsDetectedOnQC was called !!!\r\n"); }
          m_pf('e', PFX "connectionEvaluator of TRN1d is NOT ENABLED due to quick connect...\r\n");
        }
      }
      break;
    case 8: {
      unsigned la;
      CV(G_S0, "p3 state 8");
      if (p->probingMode == 0) {
        CV(G_S0, "p3 state 8: disable eval");
        w->ce->phase3EvalActive_ = 0; w->ce->trn1dEvalActive_ = 0; w->ce->weight_ = 0; w->ce->avePdsnr_ = 0.0f;
        m_pf('e', PFX "disabling connectionEvaluator of Phase3\r\n");
      }
      la = d->session_ ? env_look("V92Jd.getMaxLookahead") : env_look("V90Jd.getMaxLookahead");
      m_pf('e', PFX "Jd maxLookAhead = %d\r\n", la & 0xffu);
      env_bll(p->probingMode == 0 ? 7u : 0u, 1);
      break; }
    case 0x11: {
      XL q;
      CV(G_S0, "p3 state 0x11");
      if (m_dbg()) m_pf('d', PFX "Dil study Terminated. Enter error relaxation period.\n");
      env_setdil();
      d->dilScale_ = w->adid->padGain_;
      q = (XL)d->agc_gain_ / (XL)w->adid->padGain_;
      d->agc_gain_ = m_rf(q);
      m_fix('e', PFX "Agc Gain = %c%d.%06d\r\n", q, 1000000.0L);
      w->p3->flag3f9_ = 1;
      w->ce->phase3EvalActive_ = 0; w->ce->trn1dEvalActive_ = 0; w->ce->weight_ = 0; w->ce->avePdsnr_ = 0.0f;
      break; }
    case 0x12: CV(G_S0, "p3 state 0x12"); env_exit3(); break;
    case 0x13:
      CV(G_S0, "p3 state 0x13");
      if (p->probingMode != 0) {
        if (m_dbg()) { m_pf('d', "---------------------------------------------------------------\r\n"); if (m_dbg()) { m_pf('d', PFX "tearing down connection, probing mode ended...\r\n"); if (m_dbg()) m_pf('d', "---------------------------------------------------------------\r\n"); } }
        d->state_ = 0x2b;
      }
      break;
    case 0x14:
      CV(G_S0, "p3 state 0x14");
      if (d->phase_ != 2) {
        if (m_dbg()) m_pf('d', PFX "enter Phase 4\r\n");
        d->phase_ = 2; d->timeCnt_ += d->symCnt_; d->symCnt_ = 0; d->timeLimit_ = 0x28230u + 5u * w->info->rtd_; m_set_timing_period(d);
      }
      break;
    case 0x15: {
      unsigned r;
      CV(G_S0, "p3 state 0x15");
      r = env_ceverdict("indicateLocalRetrain", "");
      if (r == 4) { CV(G_S0, "p3 0x15: local 4"); d->state_ = 0x21; }
      else if (r == 5) { CV(G_S0, "p3 0x15: local 5"); d->state_ = m_retrain_state(); }
      break; }
    default: break;
  }
  m_bll_machine();
  m_phase1_eval();
}

/* the design call shared by the two silence / mean-error design sites */
static unsigned m_design(uint32_t noiseBits) {
  T *d = g_w.d;
  W *w = &g_w;
  unsigned a13 = (unsigned)w->ce->retrainInsteadOfRateDown_, sp = d->sv_flag_, la;
  unsigned char *cur = env_maxucode();
  int rm = d->session_ ? env_rates("V92Jd.getRatesMask") : env_rates("V90Jd.getRatesMask");
  (void)noiseBits;
  la = d->session_ ? env_look("V92Jd.getMaxLookahead") : env_look("V90Jd.getMaxLookahead");
  return (unsigned)env_design(la & 0xffu, w->adid, noiseBits, rm, w->mp2, w->adid, (uint8_t *)w->adid + 0x600, (uint8_t *)w->adid + 0x2800, (uint8_t *)w->adid + 0x280c, cur,
                              (unsigned)(uint8_t)(w->info->maxTx_ + 1), d->hwCodec_, a13, sp);
}
static void m_updconst(void) {
  CdObj *c = g_w.cd;
  env_updconst((int16_t)c->dMin_, m_w(&c->threshUp_), m_w(&c->threshDown_), m_w(&c->threshRetrain_));
}

static void m_eval4(void) {
  char a[16];
  unsigned r;
  snprintf(a, sizeof a, "%08x", m_w(&g_w.eq->ratio_));
  r = env_ceverdict("evaluatePhase4", a);
  if (r == 4) { CV(G_S1, "eval p4 = 4"); g_w.d->state_ = 0x21; }
  else if (r == 5) { CV(G_S1, "eval p4 = 5"); g_w.d->state_ = m_retrain_state(); }
}

/* ---- phase 2 ---- */
static void m_phase2(int *out, unsigned *oc) {
  T *d = g_w.d;
  V90ParObj *p = g_w.par;
  W *w = &g_w;
  unsigned ev;
  *oc = 0;
  d->timeCnt_ += g_w.n;
  if (d->timeCnt_ > d->timeLimit_) {
    CV(G_S1, "p4 timeout");
    if (m_dbg()) m_pf('d', PFX "Phase4 TimeOut\r\n");
    if (w->ce->rrnSilence_ != 0 && w->p4->w3c_ != 0 && d->hwCodec_ != 4) {
      CV(G_S1, "timeout: silence rrn not finished");
      m_pf('e', PFX "Silence rrn not finished on platform other then USB, masking silence rrn...\r\n");
      p->rrnSilenceRequested = 0;
    }
    p->sensitiveIspDetected = 1; p->silenceScr = 0;
    m_pf('e', PFX "On phase4 timeout: Assuming Sensitive ISP - use non-silence & 6dB up & maixmal rate 12000 for next time...\r\n");
    { unsigned r = env_ceverdict("indicateLocalRetrain", "");
      if (r == 4) { CV(G_S1, "timeout: local 4"); d->state_ = 0x21; }
      else if (r == 5) { CV(G_S1, "timeout: local 5"); d->state_ = m_retrain_state(); } }
  }
  ev = d->state_;
  if (ev == 0x19) {
    float me;
    uint32_t w1;
    CV(G_S1, "ev 0x19");
    me = m_rf((XL)ffb(env_calc()));
    w1 = m_w(&w->eq->statMean_);
    if (w->ce->rrnSilence_ != 0 && w->p4->w3c_ != 0) {
      CV(G_S1, "ev 0x19: silence");
      m_pf('e', PFX "Constellation design on silence rrn...\r\n"); w->cpi->w4_ = 1;
      if (d->session_) env_p4("enterWaitForCP"); else env_p4("enterWaitForMP");
    } else {
      unsigned r;
      char a[40];
      uint32_t w0 = env_eia6() ? 0u : m_wv(me);
      snprintf(a, sizeof a, "%08x,%08x", w0, w1);
      r = env_ceverdict("evaluateMeanErrorStdPhase4", a);
      if (r == 4) { CV(G_S1, "ev 0x19: eval 4"); d->state_ = 0x21; }
      else if (r == 5) { CV(G_S1, "ev 0x19: eval 5"); d->state_ = d->retrainOk_ != 0 ? 0x21u : 0x1fu; }
      else {
        uint32_t noise;
        unsigned dr;
        /* the design: the call order is getMaxUcode, the rates mask, (the noise), the lookahead, then process */
        {
          unsigned sp = d->sv_flag_, a13 = (unsigned)w->ce->retrainInsteadOfRateDown_, la;
          unsigned char *cur = env_maxucode();
          int rm = d->session_ ? env_rates("V92Jd.getRatesMask") : env_rates("V90Jd.getRatesMask");
          if (d->retrainOk_ != 0 && w->cd->w24_ == 0) { CV(G_S1, "ev 0x19: noise scaled"); noise = m_wv(m_rf((XL)p->unk328 * (XL)w->eq->statMean_)); }
          else { CV(G_S1, "ev 0x19: noise plain"); noise = m_wv(m_rf((XL)w->eq->statMean_)); }
          la = d->session_ ? env_look("V92Jd.getMaxLookahead") : env_look("V90Jd.getMaxLookahead");
          dr = (unsigned)env_design(la & 0xffu, w->adid, noise, rm, w->mp2, w->adid, (uint8_t *)w->adid + 0x600, (uint8_t *)w->adid + 0x2800, (uint8_t *)w->adid + 0x280c, cur,
                                    (unsigned)(uint8_t)(w->info->maxTx_ + 1), d->hwCodec_, a13, sp);
        }
        w->ce->enableRrnDown_ = p->enableRrnDown; w->ce->enableRrnUp_ = p->enableRrnUp; d->designed_ = 1;
        if (dr != 1) {
          CV(G_S1, "ev 0x19: design ok");
          m_pf('e', PFX "Data Phase spectral parameters:\r\n"); env_dispspec(w->mp2); w->cpi->w4_ = 1;
          if (d->session_) env_p4("enterWaitForCP"); else env_p4("enterWaitForMP");
          m_updconst();
        } else {
          unsigned r2;
          CV(G_S1, "ev 0x19: design error");
          if (m_dbg()) m_pf('d', PFX "connection design error, initiating retrain\r\n");
          r2 = env_ceverdict("indicateLocalRetrain", "");
          if (r2 == 5) { CV(G_S1, "ev 0x19: design error: local 5"); d->state_ = d->retrainOk_ != 0 ? 0x21u : 0x1fu; }
          else d->state_ = 0x26;
        }
      }
    }
  } else if (ev == 0x1a || ev == 0x1b) {
    CV(G_S1, "ev 0x1a/b");
    if (w->p4->state_ == 5) {
      int masked = p->maskRrnSilenceOnProblematicIsp != 0 && p->rrnSilenceRequested != 0 && (w->mp->trellis_ == 0 && w->mp->nonLin_ == 0) && w->mp->shaping_ == 0 && w->mp->type_ == 1 &&
                   w->mp->h1Re_ == 0 && w->mp->h1Im_ == 0 && w->mp->h2Re_ == 0 && w->mp->h2Im_ == 0 && w->mp->h3Re_ == 0 && w->mp->h3Im_ == 0;
      if (masked) {
        CV(G_S1, "ev 0x1a: problematic isp");
        m_pf('e', "###########################################################################\r\n");
        if (d->hwCodec_ == 4) { CV(G_S1, "ev 0x1a: usb"); p->maxNofV90Retrains = 0x96; m_pf('e', PFX "'Problematic' ISP Modem detected on USB. Masking drop to V34 (MAX_NOF_V90_RETRAINS = %d)...\r\n", 0x96); w->ce->retrainInsteadOfRateDown_ = 1; }
        else { CV(G_S1, "ev 0x1a: not usb"); p->rrnSilenceRequested = 0; m_pf('e', PFX "'Problematic' ISP Modem detected NOT on USB. Masking Silence RRN...\r\n"); }
        m_pf('e', "###########################################################################\r\n");
        env_p4("enterWaitForEd");
      } else {
        CV(G_S1, "ev 0x1a: not masked");
        m_pf('e', PFX "Silence RRN is not masked (silence flag = %d)\r\n", p->rrnSilenceRequested);
        env_p4("enterWaitForEd");
      }
    }
  } else if (ev == 0x1c) {
    CV(G_S1, "ev 0x1c");
    if (w->ce->rrnSilence_ != 0 && w->p4->w3c_ != 0 && w->p4->w38_ != 0) {
      CV(G_S1, "ev 0x1c: freeze timing");
      m_pf('e', PFX "freezing timing on silence rrn...\r\n");
      env_bll(0, 1); d->flag40_ = 0;
      m_means(PFX "Ed received on Silence RRN... current Mean Error = %c%d.%04d,    average Mean Error = %c%d.%04d\r\n", (XL)w->eq->meanErr_, (XL)w->eq->statMean_);
      memcpy(&d->designErr_, &w->eq->statMean_, 4);
    }
    d->flag270_ = 1;
  } else if (ev == 0x1d) {
    CV(G_S1, "ev 0x1d");
    env_edp(); env_demap(d->byteBuf_, oc); env_descr(d->byteBuf_, out, *oc);
  } else if (ev == 0x28) {
    CV(G_S1, "ev 0x28"); m_pf('e', PFX "RtNot detected.\r\n"); env_bll(9, 1); d->flag40_ = 1;
  } else if (ev == 0x2a) {
    unsigned prev;
    unsigned prod;
    CV(G_S1, "ev 0x2a");
    prod = w->mp2->w0_ * 0x1f40u;
    prev = (unsigned)(uint64_t)(int64_t)(0.5L + (XL)(uint64_t)prod * (XL)0.1666666716337204f);
    m_pf('e', PFX "on silence RRN redesign, prevRate = %d\r\n", prev);
    if (w->ce->noRestrictionRrn_ != 0) { CV(G_S1, "ev 0x2a: no restriction"); w->cd->state_ = 3; m_pf('e', PFX "FORCED rate down on silence rrn\r\n"); }
    else if (w->p4->keepRate_ != 0 && (unsigned)p->minRateForSilenceRrnKeepRate >= prev) { CV(G_S1, "ev 0x2a: keep rate"); w->cd->state_ = 1; m_pf('e', PFX "keeping rate on silence rrn\r\n"); }
    else if (p->debugConnectionEvaluatorRateDown == 2) { CV(G_S1, "ev 0x2a: forced keep"); w->cd->state_ = 1; m_pf('e', PFX "FORCED keep rate on silence rrn\r\n"); }
    else { CV(G_S1, "ev 0x2a: one down"); w->cd->state_ = 3; m_pf('e', PFX "one rate down on silence rrn\r\n"); }
    m_means(PFX "about to redesign... current Mean Error = %c%d.%04d,    average Mean Error = %c%d.%04d\r\n", (XL)w->eq->meanErr_, (XL)w->eq->statMean_);
    {
      unsigned dr = m_design(m_w(&d->designErr_));
      w->ce->enableRrnDown_ = p->enableRrnDown; w->ce->enableRrnUp_ = p->enableRrnUp; d->designed_ = 1;
      if (dr == 1) {
        unsigned r2;
        CV(G_S1, "ev 0x2a: design error");
        if (m_dbg()) m_pf('d', PFX "connection design error, initiating retrain\r\n");
        r2 = env_ceverdict("indicateLocalRetrain", "");
        d->state_ = r2 == 5 ? m_retrain_state() : 0x26u;
      } else CV(G_S1, "ev 0x2a: design ok");
      m_updconst();
    }
  } else if (ev == 0x31) { CV(G_S1, "ev 0x31"); d->flag270_ = 1; }
  else if (ev == 0x33) { CV(G_S1, "ev 0x33"); d->flag270_ = 1; }
  else if (ev == 0x35) { CV(G_S1, "ev 0x35"); m_pf('e', PFX "freezing timing on silence rrn...\r\n"); env_bll(0, 1); d->flag40_ = 0; }
  else CV(G_S1, "ev other");
  m_eval4();
}

/* ---- phases 3 / 4: the data path ---- */
static void m_data(int *out, unsigned *oc) {
  T *d = g_w.d;
  W *w = &g_w;
  unsigned r;
  env_demap(d->byteBuf_, oc);
  env_descr(d->byteBuf_, out, *oc);
  if (d->state_ == 0x23) { CV(G_S2, "data: state 0x23"); env_rrn(); }
  if (d->state_ == 0x25 && d->phase_ != 2) {
    CV(G_S2, "data: enter phase 4");
    if (m_dbg()) m_pf('d', PFX "FPE detected: enter Phase 4\r\n");
    d->symCnt_ = 0; d->timeCnt_ = 0; d->phase_ = 2; d->timeLimit_ = 0x10680u + 2u * w->info->rtd_; m_set_timing_period(d);
  }
  r = env_ceverdict("evaluateConnection", "");
  switch (r) {
    case 1: CV(G_S2, "evalconn 1"); d->state_ = 0x22; w->cd->state_ = 2; env_restore(); break;
    case 2: CV(G_S2, "evalconn 2"); w->cd->state_ = w->par->debugConnectionEvaluatorRateDown == 2 ? 1u : 3u; d->state_ = 0x22; env_restore(); break;
    case 3: CV(G_S2, "evalconn 3"); w->cd->state_ = 0; d->state_ = 0x22; env_restore(); break;
    case 4: CV(G_S2, "evalconn 4"); d->state_ = 0x21; break;
    case 5: CV(G_S2, "evalconn 5"); d->state_ = 0x1f; break;
    default: CV(G_S2, "evalconn 0 / other"); break;
  }
  if (d->session_ != 0 && d->state_ - 0x22u <= 1u) {
    CV(G_S2, "data: rrn silence reset");
    env_p4("resetBeforRRN");
    w->p4->w40_ = (uint32_t)w->ce->rrnSilence_;
    w->dem->z1eb4_ = 0;
    if (m_dbg()) m_pf('d', PFX "disable linear mapping study in data\n");
  }
}

/* ---- the remote retrain detector, the periodic prints and the outcome counters (every phase) ---- */
static void m_common_tail(unsigned n) {
  T *d = g_w.d;
  W *w = &g_w;
  V90ParObj *p = w->par;
  if (d->flag40_ != 0) {
    if ((XL)d->agc_power_ >= (XL)p->energyDropDetectorThreshold) { CV(G_ALL, "tail: power ok"); d->quietCnt_ = 0; }
    else {
      d->quietCnt_ += n;
      if (d->quietCnt_ >= (unsigned)p->noEnergyDurationForRemoteRetrain) {
        unsigned r;
        CV(G_ALL, "tail: energy drop");
        if (m_dbg()) { m_pf('d', "-------------------------------------------------------------\r\n"); if (m_dbg()) { m_pf('d', PFX "REMOTE RETRAIN - Energy Drop detected\r\n"); if (m_dbg()) m_pf('d', "-------------------------------------------------------------\r\n"); } }
        r = env_ceverdict("indicateRemoteRetrain", "");
        if (r == 5) { CV(G_ALL, "tail: remote 5"); d->state_ = (d->retrainOk_ == 0 || (int)d->phase_ > 2) ? 0x1fu : 0x21u; }
        else { CV(G_ALL, "tail: remote not 5"); d->state_ = 0x26; }
      }
    }
  }
  {
    unsigned t = n + d->errPrintCnt_;
    if (t >= d->errPrintPeriod_) {
      CV(G_ALL, "tail: error energy period");
      d->errPrintCnt_ = 0;
      if (m_dbg()) m_fix('d', PFX "Error Energy = %c%d.%03d\r\n", (XL)w->eq->meanErr_, 1000.0L);
    } else d->errPrintCnt_ = t;
  }
  {
    unsigned t = n + d->tmPrintCnt_;
    if (t >= d->tmPrintPeriod_) {
      CV(G_ALL, "tail: timing period");
      d->tmPrintCnt_ = 0;
      if (m_dbg()) {
        XL p1 = env_ppm(), p2, p3, p4;
        float tq = m_rf(p1), tu;
        int ti, tw, tip;
        p2 = env_ppm(); ti = m_f32(p2);
        tu = m_rf((XL)tq - (XL)ti);
        tw = m_abs(m_f32(1000.0L * (XL)tu));
        p3 = env_ppm(); tip = m_f32(m_fabs(p3));
        p4 = env_ppm();
        m_pf('d', PFX "Timing Offset [ppm]  = %c%d.%03d\r\n", m_sgn(p4), tip, tw);
      }
    } else d->tmPrintCnt_ = t;
  }
  d->framePos_ = (d->eqCnt_ + d->framePos_) % 6u;
  {
    int st = (int)d->state_;
    if (st == 0x23) { CV(G_ALL, "tail: cnt23"); d->cnt23_++; }
    else if (st < 0x23) { if ((unsigned)(st - 0x1f) <= 2u) { CV(G_ALL, "tail: cnt1f"); d->cnt1f_++; } }
    else if (st == 0x26) { CV(G_ALL, "tail: cnt26"); d->cnt26_++; }
  }
}

static void m_progress(int *out, unsigned *oc, float *in, unsigned n) {
  T *d = g_w.d;
  d->symCnt_ += n;
  env_fir(in, d->inBuf_, n);
  env_agc(d->inBuf_, d->inBuf_, n);
  env_resample(d->inBuf_, n, d->rsBuf_, &d->rsCnt_);
  env_eqproc(d->rsBuf_, d->rsCnt_, d->decBuf_, d->eqBuf_, &d->eqCnt_);
  d->state_ = g_w.eq->event_;
  if (d->phase_ > 4) { CV(G_S2, "phase > 4"); }
  else switch (d->phase_) {
    case 0: CV(G_S2, "phase 0"); *oc = 0; break;
    case 1: *oc = 0; m_phase1(in, n); break;
    case 2: m_phase2(out, oc); break;
    case 3: CV(G_S2, "phase 3"); if (d->symCnt_ >= (unsigned)g_w.par->minimumDurationInDataBeforeEcRrn) { CV(G_S2, "phase 3: steady state"); env_eds(); } m_data(out, oc); break;
    default: CV(G_S2, "phase 4"); m_data(out, oc); break;
  }
  m_common_tail(n);
}

/* ------------------------------------------------------------------------------------------------ the world: saved before the model, restored for the impl, compared afterwards */
typedef struct { void *p; size_t n; } Reg;
static Reg g_reg[40];
static int g_nreg;
static uint8_t *g_sv0[40], *g_sv1[40];
static Log g_ml;
static void add_reg(void *p, size_t n) { if (g_nreg >= 40) h_die3("too many regions"); g_reg[g_nreg].p = p; g_reg[g_nreg].n = n; g_nreg++; }
static void build_regions(void) {
  W *w = &g_w;
  T *d = w->d;
  g_nreg = 0;
  add_reg(d, SZ);
  add_reg(d->inBuf_, NMAX * 4u); add_reg(d->rsBuf_, NMAX * 12u); add_reg(d->decBuf_, NMAX * 4u); add_reg(d->eqBuf_, NMAX * 8u); add_reg(d->byteBuf_, NMAX * 8u);
  add_reg(w->par, sizeof(V90ParObj)); add_reg(w->mpar, sizeof(ModParObj)); add_reg(w->ce, sizeof(EvObj)); add_reg(w->p3, sizeof(Ph3DemObj)); add_reg(w->p4, sizeof(Ph4DemObj));
  add_reg(w->dem, sizeof(DemObj)); add_reg(w->cd, sizeof(CdObj)); add_reg(w->eq, sizeof(EquObj)); add_reg(w->adid, sizeof(AdidObj)); add_reg(w->mp, sizeof(MpObj));
  add_reg(w->mp2, sizeof(MapPar90)); add_reg(w->cpi, sizeof(AddCp)); add_reg(w->info, sizeof(InfoObj)); add_reg(w->out, w->ocap * 4u); add_reg(w->in, (NMAX + 4) * 4u);
  add_reg(&w->outCnt, sizeof w->outCnt);
}
static void save_to(uint8_t **dst) { int i; for (i = 0; i < g_nreg; i++) { dst[i] = (uint8_t *)realloc(dst[i], g_reg[i].n); memcpy(dst[i], g_reg[i].p, g_reg[i].n); } }
static void restore_from(uint8_t **src) { int i; for (i = 0; i < g_nreg; i++) memcpy(g_reg[i].p, src[i], g_reg[i].n); }
static void log_cmp(void) {
  unsigned i, n = g_ml.n < LOGMAX ? g_ml.n : LOGMAX;
  if (g_ml.n != g_il.n) {
    for (i = 0; i < n && i < g_il.n && i < LOGMAX; i++) fprintf(stderr, "m: %s\ni: %s\n", g_ml.r[i].t, g_il.r[i].t);
    h_die3("%u model records, %u impl records", g_ml.n, g_il.n);
  }
  for (i = 0; i < n; i++) if (strcmp(g_ml.r[i].t, g_il.r[i].t)) h_die3("record %u differs:\n model: %s\n impl:  %s", i, g_ml.r[i].t, g_il.r[i].t);
}
static void world_cmp(void) {
  int i;
  for (i = 0; i < g_nreg; i++) if (memcmp(g_sv1[i], g_reg[i].p, g_reg[i].n)) {
    size_t k;
    for (k = 0; k < g_reg[i].n; k++) if (g_sv1[i][k] != ((uint8_t *)g_reg[i].p)[k]) break;
    h_die3("region %d (%zu bytes) differs from the model at +0x%zx (model %02x, impl %02x)", i, g_reg[i].n, k, g_sv1[i][k], ((uint8_t *)g_reg[i].p)[k]);
  }
}

/* ------------------------------------------------------------------------------------------------ the world */
static void *new_blk(unsigned bytes) { return sysdep_malloc(bytes); }
static void fill_flds(void *obj, const HFld *f, int nf) {   /* member-wise random fill of the non-pointer members (arch independent) */
  int i;
  unsigned j;
  for (i = 0; i < nf; i++) {
    uint8_t *q = (uint8_t *)obj + f[i].off;
    unsigned cnt = f[i].count > 1 ? f[i].count : 1u, one = 0;
    switch (f[i].kind) {
      case 'b': one = 1; break;
      case 'h': one = 2; break;
      case 'u': case 'i': case 'f': one = 4; break;
      case 'd': one = 8; break;
      case 'x': one = 1; cnt = f[i].count; break;
      default: break;
    }
    for (j = 0; j < one * cnt; j++) q[j] = (uint8_t)h_rnd();
  }
}
#define PARF(m_) do { uint32_t v_ = pick_fb(); memcpy(&p->m_, &v_, 4); } while (0)

static int g_class;      /* the world class of this trial */
static int g_snan;       /* P43_SNAN=1: signalling NaNs in every float word the function only MOVES (copyWord / WORDARG sites); valid for the port at -O1 and above and on x86-64 (g++ -O0 on i386 quiets them) */
static void snan_poke(void) {
  W *w = &g_w;
  uint32_t v;
#define SN(f_) do { if (h_rpct(30)) { v = 0x7f800001u + h_rmod(1u << 22); if (h_rpct(30)) v |= 0x80000000u; memcpy(&(f_), &v, 4); } } while (0)
  SN(w->par->agcK); SN(w->par->germanIsdnNt1LinearEquDataBeta); SN(w->par->germanPbxLinearEquDataBeta); SN(w->par->germanPbxDfeDataBeta); SN(w->par->germanPbxLinearEquDilBeta);
  SN(w->par->germanPbxLinearEquDilMedUcodeBeta); SN(w->par->germanPbxLinearEquDilHighUcodeBeta);
  SN(w->d->designErr_); SN(w->eq->statMean_); SN(w->cd->threshUp_); SN(w->cd->threshDown_); SN(w->cd->threshRetrain_); SN(w->eq->ratio_); SN(w->d->agc_gain_);
#undef SN
}
static void set_inputs(int sc);   /* re-randomises the inputs the function reads: every call */

static void new_world(int sc) {
  W *w = &g_w;
  T *d;
  uint8_t *raw = (uint8_t *)h_obj_new(SZ + CAN);
  unsigned N = NMAX;
  memset(raw, 0, SZ);
  memset(raw + SZ, 0x5a, CAN);
  w->d = d = (T *)raw; g_can = raw + SZ;
  w->par = (V90ParObj *)h_obj_new(sizeof(V90ParObj));
  w->mpar = (ModParObj *)h_obj_new(sizeof(ModParObj));
  w->ce = (EvObj *)h_obj_new(sizeof(EvObj));
  w->p3 = (Ph3DemObj *)h_obj_new(sizeof(Ph3DemObj));
  w->p4 = (Ph4DemObj *)h_obj_new(sizeof(Ph4DemObj));
  w->dem = (DemObj *)h_obj_new(sizeof(DemObj));
  w->cd = (CdObj *)h_obj_new(sizeof(CdObj));
  w->eq = (EquObj *)h_obj_new(sizeof(EquObj));
  w->adid = (AdidObj *)h_obj_new(sizeof(AdidObj));
  w->mp = (MpObj *)h_obj_new(sizeof(MpObj));
  w->mp1 = (MapPar90 *)h_obj_new(sizeof(MapPar90));
  w->mp2 = (MapPar90 *)h_obj_new(sizeof(MapPar90));
  w->cpi = (AddCp *)h_obj_new(sizeof(AddCp));
  w->info = (InfoObj *)h_obj_new(sizeof(InfoObj));
  w->jd = h_obj_new(64); w->jd92 = h_obj_new(64); w->dil = h_obj_new(64); w->cp = h_obj_new(64); w->trn2d = h_obj_new(16);
  w->ucode = (unsigned char *)h_obj_new(32);
  memset(w->par, 0, sizeof *w->par); memset(w->mpar, 0, sizeof *w->mpar); memset(w->ce, 0, sizeof *w->ce); memset(w->p3, 0, sizeof *w->p3); memset(w->p4, 0, sizeof *w->p4);
  memset(w->dem, 0, sizeof *w->dem); memset(w->cd, 0, sizeof *w->cd); memset(w->eq, 0, sizeof *w->eq); memset(w->adid, 0, sizeof *w->adid); memset(w->mp, 0, sizeof *w->mp);
  memset(w->mp1, 0, sizeof *w->mp1); memset(w->mp2, 0, sizeof *w->mp2); memset(w->cpi, 0, sizeof *w->cpi); memset(w->info, 0, sizeof *w->info);
  { unsigned i; for (i = 0; i < 32; i++) w->ucode[i] = (uint8_t)h_rnd(); }
  /* the demodulator: random garbage in every non-pointer member, the pointers the function dereferences, the heap buffers (sizes as the constructor allocates them for n = N) */
  fill_flds(d, f_DmdObj, NF_DmdObj);
  d->info_ = w->info; d->jd_ = w->jd; d->jd92_ = w->jd92; d->dil_ = w->dil; d->mp1_ = w->mp1; d->mp2_ = w->mp2; d->trn2d_ = w->trn2d; d->cpInfo_ = w->cpi; d->cp_ = w->cp; d->mp_ = w->mp;
  d->params_ = w->par; d->eq_ = w->eq; d->p3_ = w->p3; d->p4_ = w->p4; d->dem_ = w->dem; d->cdes_ = w->cd; d->ce_ = w->ce; d->adid_ = w->adid;
  d->inBuf_ = new_blk(N * 4); d->rsBuf_ = new_blk(N * 12); d->decBuf_ = new_blk(N * 4); d->eqBuf_ = new_blk(N * 8); d->byteBuf_ = new_blk(N * 8);
  w->par->params = w->mpar;
  w->ocap = N * 8 + 8;
  w->out = (int *)new_blk(w->ocap * 4);
  w->in = (float *)new_blk((N + 4) * 4);
  g_class = (int)h_rmod(6);
  (void)sc;
}
static void free_world(void) {
  W *w = &g_w;
  sysdep_free(w->d->inBuf_); sysdep_free(w->d->rsBuf_); sysdep_free(w->d->decBuf_); sysdep_free(w->d->eqBuf_); sysdep_free(w->d->byteBuf_);
  sysdep_free(w->out); sysdep_free(w->in);
}

/* the inputs of one call: every member the function reads is re-drawn (some of them also at 'interesting' values: boundaries of the comparisons) */
static void set_inputs(int sc) {
  W *w = &g_w;
  T *d = w->d;
  V90ParObj *p = w->par;
  unsigned i;
  /* the demodulator's own scalars */
  d->hwCodec_ = h_rpct(35) ? 4u : (h_rpct(70) ? h_rmod(9) : h_rnd());
  d->session_ = h_rpct(45) ? 0u : (h_rpct(70) ? 1u : h_rnd());
  if (sc == 0) d->phase_ = h_rpct(88) ? 1u : h_rmod(5);
  else if (sc == 1) d->phase_ = h_rpct(88) ? 2u : h_rmod(5);
  else { static const uint32_t ph[] = { 0, 3, 4, 3, 4, 3, 4, 5, 7, 0x80000000u, 0xffffffffu, 0, 1, 2 }; d->phase_ = h_rpct(85) ? ph[h_rmod(7)] : ph[h_rmod(sizeof ph / sizeof ph[0])]; }
  d->symCnt_ = h_rpct(60) ? h_rmod(300) : h_rnd();
  d->state_ = h_rnd();
  d->flag40_ = pick_flag();
  d->timeCnt_ = pick_cnt(); d->timeLimit_ = h_rpct(50) ? h_rmod(400) : h_rnd();
  d->agc_mix_ = h_rpct(60) ? 1.0f : pick_f();
  d->agc_gain_ = pick_f(); d->agc_power_ = pick_f(); d->agc_mixSaved_ = pick_f();
  d->rs_state_ = h_rpct(75) ? ((uint32_t[]){ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xc, 0xd, 0xe, 0xf, 0x10 })[h_rmod(15)] : h_rnd();
  d->rs_stateCount_ = h_rpct(60) ? h_rmod(400) : (h_rpct(50) ? (uint32_t)(0u - h_rmod(100)) : h_rnd());
  d->pf_filterIdx_ = (int32_t)h_rnd();
  d->sv_flag_ = h_rpct(80) ? h_rmod(4) : h_rnd();
  d->retrainOk_ = pick_flag();
  d->framePos_ = h_rpct(70) ? h_rmod(6) : h_rnd();
  d->cnt23_ = h_rnd(); d->cnt1f_ = h_rnd(); d->cnt26_ = h_rnd();
  d->flag270_ = h_rnd();
  d->designErr_ = ffb(pick_fb()); d->dilScale_ = ffb(pick_fb());
  d->quietCnt_ = h_rpct(60) ? h_rmod(300) : h_rnd();
  d->designed_ = (uint8_t)h_rnd();
  d->errPrintCnt_ = h_rpct(60) ? h_rmod(60) : h_rnd(); d->errPrintPeriod_ = h_rpct(70) ? 1u + h_rmod(60) : h_rnd();
  d->tmPrintCnt_ = h_rpct(60) ? h_rmod(60) : h_rnd(); d->tmPrintPeriod_ = h_rpct(70) ? 1u + h_rmod(60) : h_rnd();
  d->rsCnt_ = h_rnd(); d->eqCnt_ = h_rnd();
  /* the objects it points to */
  w->info->rtd_ = h_rpct(70) ? h_rmod(2000) : h_rnd(); w->info->maxTx_ = (uint8_t)(h_rpct(80) ? h_rmod(30) : h_rnd());
  w->mp2->w0_ = h_rpct(70) ? h_rmod(60) : h_rnd();
  w->cpi->w4_ = h_rnd();
  w->mp->type_ = (uint8_t)(h_rpct(60) ? 1u : h_rmod(4)); w->mp->trellis_ = (uint8_t)(h_rpct(75) ? 0u : h_rnd()); w->mp->nonLin_ = (uint8_t)(h_rpct(80) ? 0u : h_rnd()); w->mp->shaping_ = (uint8_t)(h_rpct(80) ? 0u : h_rnd());
  w->mp->h1Re_ = (uint16_t)(h_rpct(88) ? 0u : h_rnd()); w->mp->h1Im_ = (uint16_t)(h_rpct(90) ? 0u : h_rnd()); w->mp->h2Re_ = (uint16_t)(h_rpct(90) ? 0u : h_rnd());
  w->mp->h2Im_ = (uint16_t)(h_rpct(92) ? 0u : h_rnd()); w->mp->h3Re_ = (uint16_t)(h_rpct(92) ? 0u : h_rnd()); w->mp->h3Im_ = (uint16_t)(h_rpct(94) ? 0u : h_rnd());
  w->ce->enableRrnDown_ = (int32_t)h_rnd(); w->ce->enableRrnUp_ = (int32_t)h_rnd();
  w->ce->avePdsnr_ = ffb(pick_fb()); w->ce->weight_ = h_rnd(); w->ce->phase3EvalActive_ = (int32_t)h_rnd(); w->ce->trn1dEvalActive_ = (int32_t)h_rnd();
  w->ce->rrnSilence_ = (int32_t)pick_flag(); w->ce->retrainInsteadOfRateDown_ = (int32_t)pick_flag(); w->ce->noRestrictionRrn_ = (int32_t)pick_flag(); w->ce->altRbsDetectedOnQc_ = (uint16_t)h_rnd();
  w->p3->state_ = h_rpct(60) ? 3u : h_rmod(8); w->p3->cnt_ = h_rpct(60) ? h_rmod(400) : (h_rpct(50) ? (uint32_t)(0u - h_rmod(100)) : h_rnd());
  w->p3->impairInfo_ = (uint8_t)h_rnd(); w->p3->flag3f9_ = (uint8_t)h_rnd(); w->p3->flag400_ = (uint16_t)h_rnd();
  w->p4->state_ = h_rpct(60) ? 5u : h_rmod(8); w->p4->w38_ = pick_flag(); w->p4->w3c_ = pick_flag(); w->p4->w40_ = h_rnd(); w->p4->keepRate_ = pick_flag();
  w->dem->z1eb4_ = (uint16_t)h_rnd();
  w->cd->dMin_ = (uint16_t)h_rnd(); w->cd->threshUp_ = ffb(pick_fb()); w->cd->threshDown_ = ffb(pick_fb()); w->cd->threshRetrain_ = ffb(pick_fb());
  w->cd->w24_ = h_rpct(50) ? 0u : h_rnd(); w->cd->state_ = h_rnd();
  w->eq->event_ = h_rnd(); w->eq->w8_ = (uint16_t)h_rnd();
  w->eq->meanErr_ = ffb(pick_fb()); w->eq->statMean_ = ffb(pick_fb()); w->eq->ratio_ = ffb(pick_fb());
#ifdef P44_INTEG
  w->adid->pcmType_ = (int32_t)(h_rpct(60) ? h_rmod(2) : h_rnd()); w->adid->codecIsAlaw_ = (int32_t)(h_rpct(60) ? h_rmod(2) : h_rnd()); w->adid->unsuspectedPhase_ = (uint16_t)h_rnd();
  { unsigned i6; for (i6 = 0; i6 < 6; i6++) { w->adid->suspectedPhase_[i6] = (uint8_t)h_rnd(); w->adid->altRbsPhase_[i6] = (uint16_t)h_rnd(); } }
  w->info->pcm_ = h_rnd(); w->info->uinfo_ = (uint8_t)h_rnd();
  w->par->analogRateMask = (int32_t)h_rnd(); w->par->timingHistoryEvaluationEnabled = (int32_t)pick_flag(); PARF(timingOffesetMinStdForSave);
  w->par->errorEnergyPrintPeriodData = (int32_t)h_rnd(); w->par->timingOffsetPrintPeriodData = (int32_t)h_rnd();
  w->ce->delayedRetrainRequest_ = (int32_t)h_rnd();
  w->dem->z1eb4_ = (uint16_t)h_rnd();
#endif
  w->adid->padGain_ = h_rpct(60) ? (h_rpct(60) ? 1.0f : ffb(pick_fb() | 0x00800000u)) : ffb(pick_fb());
  /* the parameter block: boundaries around the compares (counts as signed / unsigned), random floats */
  p->probingMode = (int32_t)(h_rpct(50) ? 0u : h_rpct(70) ? 1u : h_rnd());
  p->germanIsdnNt1BoxFilterGain = (int32_t)h_rmod(30); p->germanPbxPreFilterGain = (int32_t)h_rmod(30);
  PARF(agcK); PARF(unk6c); PARF(unk70); PARF(germanPbxLinearEquDilBeta); PARF(germanPbxLinearEquDilMedUcodeBeta); PARF(germanPbxLinearEquDilHighUcodeBeta); PARF(germanIsdnNt1LinearEquDataBeta);
  PARF(germanPbxLinearEquDataBeta); PARF(germanPbxDfeDataBeta); PARF(linearEquDilBeta); PARF(linearEquDilMedUcodeBeta); PARF(linearEquDilHighUcodeBeta); PARF(linearEquDataBeta); PARF(dfeDataBeta);
  PARF(energyDropDetectorThreshold); PARF(unk328);
  if (h_rpct(30)) p->unk6c = ffb(0x3f800000u + (h_rpct(50) ? h_rmod(1u << 22) : 0u));
  if (h_rpct(20)) {   /* the DIL divisor window: 0.85 * unk6c <= gain < unk6c (gain up to 12.5% below unk6c) */
    uint32_t u = 0x3f000000u + h_rmod(1u << 24);
    p->unk6c = ffb(u);
    d->agc_gain_ = ffb(u - (h_rpct(10) ? 0u : h_rmod(1u << 20)));
    if (h_rpct(30)) d->agc_gain_ = ffb(u - 1u);
  }
  if (h_rpct(25)) p->unk70 = ffb(fbits(d->agc_gain_) + (h_rpct(50) ? 0u : (uint32_t)(h_rmod(3)) - 1u));
  p->agcAdaptationDuration = (int32_t)(h_rpct(60) ? h_rmod(400) : (h_rpct(50) ? (uint32_t)(0u - h_rmod(100)) : h_rnd()));
  p->bllTrn1dInitialToFastDuration = (int32_t)pick_i(); p->bllTrn1dFastToSlowDuration = (int32_t)pick_i(); p->unk100 = (int32_t)pick_i(); p->unk104 = (int32_t)pick_i(); p->unk108 = (int32_t)pick_i();
  p->unk10c = (int32_t)pick_i();
  p->errorEnergyPrintPeriodPhase4 = (int32_t)(h_rpct(70) ? 1u + h_rmod(60) : h_rnd()); p->timingOffsetPrintPeriodPhase4 = (int32_t)(h_rpct(70) ? 1u + h_rmod(60) : h_rnd());
  p->noEnergyDurationForRemoteRetrain = (int32_t)(h_rpct(60) ? h_rmod(200) : h_rnd());
  p->unk300 = (int32_t)h_rnd(); p->unk304 = (int32_t)h_rnd();
  p->rrnSilenceRequested = (int32_t)pick_flag(); p->maskRrnSilenceOnProblematicIsp = (int32_t)pick_flag(); p->silenceScr = (int32_t)h_rnd(); p->sensitiveIspDetected = (int32_t)h_rnd();
  p->maxNofV90Retrains = (int32_t)h_rnd();
  p->minRateForSilenceRrnKeepRate = (int32_t)(h_rpct(60) ? h_rmod(60) : h_rnd());
  p->enableRrnUp = (int32_t)h_rnd(); p->enableRrnDown = (int32_t)h_rnd();
  p->minimumDurationInDataBeforeEcRrn = (int32_t)(h_rpct(60) ? h_rmod(300) : (h_rpct(50) ? (uint32_t)(0u - h_rmod(100)) : h_rnd()));
  p->enableDrop2V34OnSevereCodec = (int32_t)pick_flag();
  p->debugConnectionEvaluatorRateDown = (int32_t)(h_rpct(50) ? 2u : h_rmod(4));
  w->mpar->connectionType = h_rnd();
  /* the bll block is exercised at the boundary: the count equals the threshold +- 1 */
  if (h_rpct(35)) {
    int32_t *thr[] = { &p->bllTrn1dInitialToFastDuration, &p->bllTrn1dFastToSlowDuration, &p->unk100, &p->unk104, &p->unk108, &p->unk10c };
    static const uint32_t bs[] = { 2, 3, 5, 0xc, 0xd, 0xe };
    unsigned k = h_rmod(6);
    d->rs_state_ = bs[k]; d->retrainOk_ = k < 3 ? 0u : 1u;
    d->rs_stateCount_ = (uint32_t)(*thr[k] + (int32_t)h_rmod(3) - 1);
  }
  /* the power / quiet counter boundaries */
  if (h_rpct(30)) d->agc_power_ = p->energyDropDetectorThreshold;
  if (h_rpct(30)) d->quietCnt_ = (uint32_t)(p->noEnergyDurationForRemoteRetrain - (int32_t)g_w.n + (int32_t)h_rmod(3) - 1);
  if (h_rpct(30)) d->errPrintCnt_ = d->errPrintPeriod_ - g_w.n + h_rmod(3) - 1u;
  if (h_rpct(30)) d->tmPrintCnt_ = d->tmPrintPeriod_ - g_w.n + h_rmod(3) - 1u;
  if (h_rpct(25)) d->timeCnt_ = d->timeLimit_ - g_w.n + h_rmod(3) - 1u;
  if (h_rpct(25)) d->symCnt_ = (uint32_t)p->minimumDurationInDataBeforeEcRrn - g_w.n + h_rmod(3) - 1u;
  if (h_rpct(30)) w->p3->cnt_ = (uint32_t)p->agcAdaptationDuration + h_rmod(3) - 1u;
  if (h_rpct(25)) w->p4->keepRate_ = 1u;
  if (h_rpct(35)) {   /* the silence RRN redesign: minRateForSilenceRrnKeepRate at the previous rate +- 1 (the print's formula; long double is the x87 type on both architectures) */
    uint32_t x = w->mp2->w0_ * 0x1f40u;
    unsigned long long pr = (unsigned long long)(long long)(0.5L + (long double)(unsigned long long)x * (long double)0.1666666716337204f);
    w->p4->keepRate_ = 1u;
    p->minRateForSilenceRrnKeepRate = (int32_t)((uint32_t)pr + h_rmod(3) - 1u);
  }
  (void)i;
}

/* the scripted results: the bits the function must not look at are garbage; the events are drawn from the ones the function acts on */
static uint32_t pick_from(const uint32_t *t, unsigned n) { return t[h_rmod(n)]; }
static void gen_script(void) {
  static const uint32_t ev1[] = { 0, 1, 3, 4, 5, 6, 8, 0x11, 0x12, 0x13, 0x14, 0x15, 2, 7, 9, 0x10, 0x16, 0x1f, 0x21, 3, 4, 5, 6, 8, 0x11, 0x13, 0x14, 0x15, 0xff, 0xffffffffu, 3, 8,
                                  0x10001u, 0x10003u, 0x10011u, 0x100000e8u, 0x80000001u, 0x20013u };   /* the last six differ from a state only in the upper bits: the function compares whole words */
  static const uint32_t ev2[] = { 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x28, 0x2a, 0x31, 0x33, 0x35, 0x18, 0x1e, 0x2b, 0x23, 0x25, 0, 0xffffffffu, 0x19, 0x19, 0x1a, 0x1c, 0x1c, 0x1d, 0x2a, 0x2a, 0x31, 0x28, 0x35, 0x33, 0x34,
                                  0x10019u, 0x1001cu, 0x1002au, 0x80000019u, 0x20035u };
  static const uint32_t ev3[] = { 0, 0, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x23, 0x24, 0x25, 0x25, 0x26, 0x27, 0x19, 0x3, 0xffffffffu, 0x80000000u, 0x23, 0x25, 0x1f, 0x21, 0x26, 0x22,
                                  0x10023u, 0x10025u, 0x10026u, 0x1001fu, 0x10021u, 0x80000023u, 0x20026u };
  unsigned i, ph = g_w.d->phase_;
  for (i = 0; i < NSC; i++) {
    uint32_t e;
    if (ph == 1) e = h_rpct(85) ? pick_from(ev1, sizeof ev1 / sizeof ev1[0]) : h_rnd();
    else if (ph == 2) e = h_rpct(88) ? pick_from(ev2, sizeof ev2 / sizeof ev2[0]) : h_rnd();
    else e = h_rpct(88) ? pick_from(ev3, sizeof ev3 / sizeof ev3[0]) : h_rnd();
    g_sc[K_EQ][i] = e;
    g_sc[K_EQ2][i] = h_rnd();
    g_sc[K_BUF][i] = h_rnd();
    g_sc[K_AGC][i] = (i & 1u) ? fbits(pick_f()) : h_rnd();
    g_sc[K_RS][i] = h_rnd();
    g_sc[K_SV][i] = h_rnd();
    g_sc[K_SVF][i] = h_rnd();
    g_sc[K_BLL][i] = h_rnd();
    g_sc[K_CE][i] = h_rnd();
    g_sc[K_CE2][i] = h_rnd();
    g_sc[K_CD][i] = h_rnd();
    g_sc[K_JD][i] = h_rnd();
    g_sc[K_PPM][i] = h_rnd();
    g_sc[K_PF][i] = h_rnd();
    g_sc[K_ADID][i] = h_rnd();
    g_sc[K_POKE][i] = h_rnd();
    g_sc[K_PRN][i] = h_rnd();
    g_sc[K_MISC][i] = h_rnd();
  }
  memset(g_cur, 0, sizeof g_cur);
}

static float pick_in(void) {
  uint32_t r = h_rnd();
  if ((r & 0x3f) < 2) { static const uint32_t b[] = { 0, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7f800000u, 0xff800000u, 0x47000000u, 0xc7000100u, 0x00000001u, 0x43960000u }; return h_float_from_bits_quiet(b[h_rmod(sizeof b / sizeof b[0])]); }
  return (float)((int)h_rmod(40001) - 20000);
}

/* ------------------------------------------------------------------------------------------------ one call */
static unsigned long g_ncalls;
static int g_showall;
static void do_call(int sc) {
  W *w = &g_w;
  T *d = w->d;
  unsigned n, k, cnt;
  n = h_rpct(5) ? 0u : h_rpct(60) ? 1u + h_rmod(10) : 1u + h_rmod(NMAX);
  w->n = n;
  for (k = 0; k < NMAX + 4; k++) w->in[k] = pick_in();
  for (k = 0; k < w->ocap; k++) w->out[k] = (int)0xdeadbeefu;
  memset(d->inBuf_, 0xa5, NMAX * 4); memset(d->rsBuf_, 0xa5, NMAX * 12); memset(d->decBuf_, 0xa5, NMAX * 4); memset(d->eqBuf_, 0xa5, NMAX * 8); memset(d->byteBuf_, 0xa5, NMAX * 8);
  set_inputs(sc);
  dsplibs_debug_level = h_rpct(50) ? 2u + h_rmod(2) : (h_rpct(60) ? 0u : (h_rpct(60) ? 1u : h_rnd()));
  if (g_snan) snan_poke();
  gen_script();
  g_show = h_sample(&g_symctr, 12, 250) || g_showall;
  w->outCnt = h_rpct(50) ? 0u : h_rnd();
  /* the model on the live world first (saved and restored), then the impl */
  build_regions();
  save_to(g_sv0);
#ifndef P44_INTEG   /* the independent model scripts the P44 methods (recorders) and cannot follow the real ones: in the integration build only blob vs port is compared */
  { unsigned lv0 = dsplibs_debug_level;
    g_log = &g_ml; g_ml.n = 0; memset(g_cur, 0, sizeof g_cur); cov_clear();
    { unsigned mc = w->outCnt; m_progress(w->out, &mc, w->in, n); w->outCnt = mc; }
    save_to(g_sv1);
    restore_from(g_sv0);
    dsplibs_debug_level = lv0; }
#endif
  g_log = &g_il; g_il.n = 0; memset(g_cur, 0, sizeof g_cur);
  h_emit(g_show, "call sc=%d ph=%u st=%u n=%u lvl=%u sess=%u ro=%u", sc, d->phase_, d->state_, n, dsplibs_debug_level, d->session_, d->retrainOk_);
  cnt = w->outCnt;
  f_progress(d, w->out, &cnt, w->in, n);
  w->outCnt = cnt;
  __asm__ volatile("" ::: "memory");
#ifndef P44_INTEG
  log_cmp(); world_cmp();
  cov_flush();
#endif
  for (k = 0; k < g_il.n && k < LOGMAX; k++) h_emit(g_show, "%s", g_il.r[k].t);
  h_emit(g_show, "ret outCnt=%u", cnt);
  { char b[1024]; size_t o = 0; unsigned q; for (q = 0; q < w->ocap && q < 40 && o + 24 < sizeof b; q++) o += (size_t)snprintf(b + o, sizeof b - o, "%08x ", (unsigned)w->out[q]); h_emit(g_show, "out %s", b); }
  { uint32_t h = fnv_add(2166136261u, w->out, w->ocap * 4u); h_emit(g_show, "outhash %08x", h); }
  h_dump_obj(g_show, "D", d, g_fd, g_nfd);
  h_wild = 0;
  h_emit(g_show, "foreign %08x", snap_hash(w));
  { unsigned i; for (i = 0; i < CAN; i++) if (g_can[i] != 0x5a) h_die3("canary byte %u behind the object was overwritten", i); }
  g_ncalls++;
}

static void trial(int sc) {
  unsigned ncalls = rnd_range(1, 3), k;
  new_world(sc);
  for (k = 0; k < ncalls; k++) do_call(sc);
  free_world();
  h_trial_end("p43");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, ntr, i;
  const char *seed = getenv("P43_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P43_ALL")) g_showall = 1;
  if (getenv("P43_RZ")) g_rz = 1;
  if (getenv("P43_SNAN")) g_snan = 1;
  p07_layout_check(); p09_layout_check(); p11_layout_check(); p17_layout_check(); p18_layout_check(); p24_layout_check(); p28_layout_check(); p34_layout_check(); p37_layout_check(); p40_layout_check();
  p43_layout_check();
  for (i = 0; i < NF_DmdObj; i++) {
    const char *nm = f_DmdObj[i].name;
    char k = f_DmdObj[i].kind;
    int keep = !(k == 'p' || k == 'o' || k == 'e' || k == 'v');
    if (k == 'o' && (!strcmp(nm, "inBuf_") || !strcmp(nm, "rsBuf_") || !strcmp(nm, "decBuf_") || !strcmp(nm, "eqBuf_") || !strcmp(nm, "byteBuf_"))) keep = 1;
    if (keep) g_fd[g_nfd++] = f_DmdObj[i];
  }
#ifdef P44_INTEG
  CV(0, "p44 integration: model disabled");   /* keeps the covsites section (and its start / stop symbols) alive: the model is not run */
#endif
  h_x87_check();
  h_arena_init();
  h_rz_by_id = 1;
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(1);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P43_TRIALS") ? atoi(getenv("P43_TRIALS")) : 4000;
  if (sc < 0 || sc > 2) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  for (tr = 0; tr < ntr; tr++) trial(sc);
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  { CovSite *c; int bad = 0;
    for (c = __start_covsites; c < __stop_covsites; c++) {
      if (!(c->grp & (1u << sc))) continue;
      h_emit(1, "COV %s %lu", c->name, c->n);
      if (c->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", c->name); bad = 1; }
    }
    h_cov_finish();
    return (bad && !getenv("P43_NOCOV")) ? 2 : 0; }
}
