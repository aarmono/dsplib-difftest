/*
 * Differential test harness for port chunk P44 (src/vpcm/v90_demodulator.cpp): the 20 methods of V90Demodulator that are not progress (P43): constructor C1 / C2, destructor D1 / D2,
 * sessionTermination, enterPhase4 (dead), enterDataSteadyState, enterRRN, enterFPE (dead), setSessionFlag, enterPhase3, getBitRate, getRbsPattern (dead), indicateRemoteRateReneg, getAT_UD
 * (dead), exitPhase3, enterDataPhase, reInit, reset, enterChannelVerification.  Derived from test/vpcm/v90_demodulator_progress.main.c (same recorder / snapshot / independent-model design).
 *
 * ONE C source, no #ifdef around the code under test, built by test/vpcm/v90_demodulator.main.sh against the blob (dsplibs.o with the callees WEAK so that the recorders below win), against the port
 * (-O0 / -O2: the port object alone) and for x86-64 (diffed against the 32-bit BLOB's output = test/golden/vpcm/v90_demodulator.main.blob32.out).  The functions are reached through __asm__ labels; the demodulator is
 * the generated C mirror (test/vpcm/v90_demodulator_progress.layout, flattened sub-objects), the objects it points to are the mirrors of P07, P11, P17, P18, P24, P26, P28, P34, P37, P40; V90Phase2Info is a local mirror.
 *
 * Every callee is a RECORDER with scripted results: the 14 sub-object constructors and 13 destructors, reset / setters of the sub-objects, the timing history mean / std (extended values), the
 * Jd lookahead (garbage in the upper bits), the TRN2 design result (garbage in the upper 16 bits), displaySpectralParams ...  Some recorders also WRITE the state the method reads afterwards (p3
 * event_, adid words, sv_ flag, retrainOk_, session_, phase_ ...: `poke_*`), so the load time of every word the blob caches in a register or reloads is observable.  What is compared: the return value
 * of the method, the whole demodulator object (generated field table), every foreign word the method reads or writes (selective hashes), every record (callee, arguments and a SNAPSHOT hash of the
 * world at the moment of the call: the store order before every call / print), the canary behind the object, the malloc / free log of the constructor / destructor, the store-order probe at every
 * malloc of the constructor (H_CTOR_PROBE).  The INDEPENDENT MODEL (one function per method, if / else, own extended-precision helpers) runs first on the LIVE world (saved before, restored
 * afterwards) through the same callee environment and must leave the same world, return value and records as the implementation (exit 3 otherwise); the constructor / destructor are checked against
 * an EXPECTED record list built from the arguments / the object (the order of the sub-constructors, mallocs, frees, the stores before each).  Coverage sites in the model (exit 2 if one is not reached).
 *
 * Scenarios (own process each): 0 sessionTermination + destructor, 1 constructor, 2 the small methods (enterPhase3 / 4 / FPE / RRN / DataPhase / DataSteadyState, setSessionFlag, getBitRate,
 * getRbsPattern, indicateRemoteRateReneg, getAT_UD, reInit), 3 exitPhase3 / reset / enterChannelVerification, 4 lifecycles (constructor, random calls on the built object, destructor).
 * Excluded (documented): signalling NaNs in the default run (P44_SNAN=1 experiment), n above 48.
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
#include "v90_trn2_designer_layout.h"
#include "v90_mapper_layout.h"
#include "v90_phase3_demod_v92_decision_layout.h"
#include "v90_phase4_demod_decisions_layout.h"
#include "v90_equalizer_process_layout.h"
#include "v90_demodulator_progress_layout.h"
#include "v90_demodulator_layout.h"

#define L(m) __asm__(m)

extern unsigned int dsplibs_debug_level;

#if defined(__i386__)
typedef uint32_t FW;                        /* by-value float arguments are raw words on i386 */
static inline uint32_t fw_bits(FW v) { return v; }
#else
typedef float FW;
static inline uint32_t fw_bits(FW v) { uint32_t b; memcpy(&b, &v, 4); return b; }
#endif

/* ---- the functions under test (the blob's own symbols / the port's) ---- */
extern int f_sessterm(void *) L("_ZN14V90Demodulator18sessionTerminationEv");
extern void f_ep4(void *) L("_ZN14V90Demodulator11enterPhase4Ev");
extern void f_steady(void *) L("_ZN14V90Demodulator20enterDataSteadyStateEv");
extern void f_rrn(void *) L("_ZN14V90Demodulator8enterRRNEv");
extern void f_fpe(void *) L("_ZN14V90Demodulator8enterFPEEv");
extern void f_setsess(void *, unsigned) L("_ZN14V90Demodulator14setSessionFlagEj");
extern void f_ep3(void *) L("_ZN14V90Demodulator11enterPhase3Ev");
extern int f_bitrate(const void *) L("_ZNK14V90Demodulator10getBitRateEv");
extern void f_rbs(const void *, unsigned *) L("_ZNK14V90Demodulator13getRbsPatternEPj");
extern void f_indic(const void *) L("_ZNK14V90Demodulator23indicateRemoteRateRenegEv");
extern void f_atud(const void *, void *) L("_ZNK14V90Demodulator8getAT_UDEP21TAG_DiagnosticResults");
extern void f_exit3(void *) L("_ZN14V90Demodulator10exitPhase3Ev");
extern void f_edp(void *) L("_ZN14V90Demodulator14enterDataPhaseEv");
extern void f_reinit(void *) L("_ZN14V90Demodulator6reInitEv");
extern void f_reset(void *, unsigned) L("_ZN14V90Demodulator5resetEj");
extern void f_ecv(void *, short, short) L("_ZN14V90Demodulator24enterChannelVerificationEss");
extern void f_ctor(void *, unsigned, void *, void *, void *, void *, void *, void *, void *, void *, void *, unsigned, void *, unsigned, unsigned)
    L("_ZN14V90DemodulatorC1EjP13V90Phase2InfoP5V90JdP5V92JdP19tagV90DILdescriptorP16V90MappingParamsS9_P22tagV90AdditionalCPinfoP5V90CPP5V90MP23__tHardwareCodecTypes__P13V90Parameters20V90ComputationalModej");
extern void f_dtor(void *) L("_ZN14V90DemodulatorD1Ev");

/* ---- the interposed callees ---- */
extern void c_scrreset(void *, unsigned char) L("_ZN11DescramblerIhiE5resetEh");
extern void c_agcreset(void *) L("_ZN3AgcIfE5resetEv");
extern void c_pfreset(void *) L("_ZN12V90PreFilter5resetEv");
extern int c_eia6(void *) L("_ZNK12V90PreFilter13isV90WithEia6Ev");
extern void c_pfsel(void *) L("_ZN12V90PreFilter12selectFilterEv");
extern void c_pfsetp(void *) L("_ZN12V90PreFilter12setParamEia6Ev");
extern void c_pfdisp(void *) L("_ZN12V90PreFilter16displayParamEia6Ev");
extern void c_rsreset(void *) L("_ZN12V90Resampler5resetEv");
extern void c_bll(void *, unsigned, unsigned) L("_ZN12V90Resampler11setBllStateE11V90BllStatej");
extern long double c_tmean(void *) L("_ZN12V90Resampler20getTimingHistoryMeanEv");
extern long double c_tstd(void *) L("_ZN12V90Resampler19getTimingHistoryStdEv");
extern long double c_tmeanLD(void *) L("_ZN12V90Resampler22getTimingHistoryMeanLDEv");
extern long double c_tstdLD(void *) L("_ZN12V90Resampler21getTimingHistoryStdLDEv");
extern void c_settoff(void *, FW) L("_ZN21ResamplerTimingOffset15setTimingOffsetEf");
extern void c_svreset(void *) L("_ZN19V90SpectralVerifier5resetEv");
extern void c_eqreset(void *, unsigned) L("_ZN12V90Equalizer5resetEj");
extern void c_eqep3(void *) L("_ZN12V90Equalizer11enterPhase3Ev");
extern void c_eqep4(void *) L("_ZN12V90Equalizer11enterPhase4Ev");
extern void c_eqecv(void *) L("_ZN12V90Equalizer24enterChannelVerificationEv");
extern void c_p3clear(void *) L("_ZN20V90Phase3Demodulator23clearVerificationStatusEv");
extern void c_p3sess(void *, unsigned) L("_ZN20V90Phase3Demodulator14setSessionFlagEj");
extern void c_p3reset(void *, unsigned, unsigned char, unsigned, unsigned, void *, void *, void *, short, short, FW, unsigned)
    L("_ZN20V90Phase3Demodulator5resetE7PcmTypeh22Phase3DemodulatorStatejP5V90JdP5V92JdP19tagV90DILdescriptorssfj");
extern void c_p3exitdil(void *) L("_ZN20V90Phase3Demodulator7exitDILEv");
extern unsigned char *c_maxucode(void *) L("_ZN20V90Phase3Demodulator11getMaxUcodeEv");
extern void c_p4sess(void *, unsigned) L("_ZN20V90Phase4Demodulator14setSessionFlagEj");
extern void c_p4reset(void *, unsigned char, unsigned, unsigned, unsigned) L("_ZN20V90Phase4Demodulator5resetEh22Phase4DemodulatorStatejj");
extern void c_cereset(void *) L("_ZN22V90ConnectionEvaluator5resetEv");
extern void c_ceremote(void *) L("_ZN22V90ConnectionEvaluator23indicateRemoteRateRenegEv");
extern void c_cdreset(void *) L("_ZN24V90ConstellationDesigner5resetEv");
extern void c_demlin(void *, unsigned) L("_ZN11V90Demapper20resetLinearMappStudyEj");
extern void c_printinfo(const void *) L("_ZNK13V90Phase2Info9printInfoEv");
extern int c_altrbs(void *) L("_ZN25V90AutoDigitalImpDetector21isThereAnyAltRbsPhaseEv");
extern void c_trnnof(void *, short) L("_ZN15V90TRN2Designer18setNofUcodesInTrn2Es");
extern int c_trndesign(void *, void *, void *, void *, void *, void *, int, int, short, void *, unsigned, unsigned char, unsigned)
    L("_ZN15V90TRN2Designer13V90TRN2DesignEP16V90MappingParamsPA128_sS3_PA128_hPs7PcmTypeS7_sPhjh28V90SpecialSpectralConditions");
extern unsigned c_v90look(void *) L("_ZN5V90Jd15getMaxLookaheadEv");
extern unsigned c_v92look(void *) L("_ZN5V92Jd15getMaxLookaheadEv");
extern void c_dispspec(void *) L("displaySpectralParams");
/* constructors / destructors of the sub-objects */
extern void k_agc(void *) L("_ZN3AgcIfEC1Ev");
extern void k_pf(void *, unsigned, void *, void *) L("_ZN12V90PreFilterC1E23__tHardwareCodecTypes__P13V90Phase2InfoP13V90Parameters");
extern void k_rs(void *, unsigned, FW, unsigned, FW, void *, FW, unsigned) L("_ZN12V90ResamplerC1EjfjfP13V90Parametersfj");
extern void k_cpow(void *) L("_ZN21V90ConstellationPowerC1Ev");
extern void k_scr(void *, unsigned, unsigned, unsigned) L("_ZN11DescramblerIhiEC1Ejjj");
extern void k_sv(void *, void *) L("_ZN19V90SpectralVerifierC1EP13V90Parameters");
extern void k_adid(void *, void *) L("_ZN25V90AutoDigitalImpDetectorC1EP13V90Parameters");
extern void k_ce(void *, void *) L("_ZN22V90ConnectionEvaluatorC1EP13V90Parameters");
extern void k_p3(void *, void *, void *, unsigned, void *) L("_ZN20V90Phase3DemodulatorC1EP13V90ParametersP19V90SpectralVerifierjP25V90AutoDigitalImpDetector");
extern void k_dem(void *, unsigned, void *, void *) L("_ZN11V90DemapperC1EjP13V90ParametersP25V90AutoDigitalImpDetector");
extern void k_p4(void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, unsigned)
    L("_ZN20V90Phase4DemodulatorC1EP16V90MappingParamsS1_P11V90DemapperP5V90CPP5V90MPP11DescramblerIhiEP22V90ConnectionEvaluatorP13V90ParametersP20V90Phase3DemodulatorP25V90AutoDigitalImpDetectorj");
extern void k_eq(void *, unsigned, unsigned, void *, void *, void *, void *, void *, void *, void *, void *, unsigned)
    L("_ZN12V90EqualizerC1EjjP20V90Phase3DemodulatorP20V90Phase4DemodulatorP11V90DemapperP22V90ConnectionEvaluatorP19V90SpectralVerifierP13V90ParametersP12V90ResamplerP12V90PreFilter20V90ComputationalMode");
extern void k_trn(void *, void *, void *) L("_ZN15V90TRN2DesignerC1EP13V90ParametersP21V90ConstellationPower");
extern void k_cd(void *, void *, void *, void *) L("_ZN24V90ConstellationDesignerC1EP13V90ParametersP12V90PreFilterP21V90ConstellationPower");
extern void x_pf(void *) L("_ZN12V90PreFilterD1Ev");
extern void x_rs(void *) L("_ZN12V90ResamplerD1Ev");
extern void x_cpow(void *) L("_ZN21V90ConstellationPowerD1Ev");
extern void x_scr(void *) L("_ZN11DescramblerIhiED1Ev");
extern void x_sv(void *) L("_ZN19V90SpectralVerifierD1Ev");
extern void x_adid(void *) L("_ZN25V90AutoDigitalImpDetectorD1Ev");
extern void x_ce(void *) L("_ZN22V90ConnectionEvaluatorD1Ev");
extern void x_p3(void *) L("_ZN20V90Phase3DemodulatorD1Ev");
extern void x_dem(void *) L("_ZN11V90DemapperD1Ev");
extern void x_p4(void *) L("_ZN20V90Phase4DemodulatorD1Ev");
extern void x_eq(void *) L("_ZN12V90EqualizerD1Ev");
extern void x_trn(void *) L("_ZN15V90TRN2DesignerD1Ev");
extern void x_cd(void *) L("_ZN24V90ConstellationDesignerD1Ev");

typedef DmdObj T;
#define SZ sizeof(T)
enum { CAN = 64, NSC = 96, LOGMAX = 300, TXTMAX = 640, NMAX = 48 };

static uint32_t fnv_add(uint32_t h, const void *p, size_t n) { const uint8_t *q = (const uint8_t *)p; while (n--) { h ^= *q++; h *= 16777619u; } return h; }
static uint32_t fbits(float f) { uint32_t b; memcpy(&b, &f, 4); return b; }
static float ffb(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }

/* local mirror of V90Phase2Info (0x24 bytes) */
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
  DiagObj *diag;             /* getAT_UD's record (NULL otherwise) */
  unsigned *rbs;             /* getRbsPattern's output (6 words) */
  unsigned n;                /* the buffer size the object was built for */
} W;
static W g_w;
static uint8_t *g_can;
static HFld g_fd[160];       /* the demodulator's field table of the method scenarios: no pointers except the five owned buffers */
static int g_nfd;
static HFld g_fdp[160];      /* the field table of the ctor / dtor scenarios: the object pointers are normalised ('p') */
static int g_nfdp;
static int g_foreign = 1;    /* snapshot hashes the foreign objects (0 in the destructor / constructor scenarios: the world is partial or freed) */
static const T *g_snap_d;   /* the constructor scenario: the expected records are hashed against a shadow object */
static int g_blocks = 1;     /* snapshot hashes the contents of the owned sample buffers (0 in the destructor: freed blocks) */

/* ------------------------------------------------------------------------------------------------ scripts, cursors, records */
enum { K_PF, K_TM, K_JD, K_ADID, K_POKE, K_PRN, K_TRN, K_BLL, K_MISC, K_N };
static uint32_t g_sc[K_N][NSC];
static unsigned g_cur[K_N];
static uint32_t sw(int k) { return g_sc[k][g_cur[k]++ % NSC]; }
typedef struct { char t[TXTMAX]; } Rec;
typedef struct { Rec r[LOGMAX]; unsigned n; unsigned long total; } Log;
static Log g_il, g_ml;
static Log *g_log = &g_il;
static int g_show;
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

/* the snapshot: the demodulator, the foreign words the methods read / write */
static uint32_t snap_hash(const W *w) {
  uint32_t h = 2166136261u;
  int i;
  const uint8_t *dp = (const uint8_t *)(g_snap_d ? g_snap_d : w->d);
  for (i = 0; i < g_nfd; i++) {
    const HFld *f = &g_fd[i];
    const uint8_t *q = dp + f->off;
    unsigned cnt = f->count > 1 ? f->count : 1u;
    switch (f->kind) {
      case 'b': h = fnv_add(h, q, cnt); break;
      case 'h': h = fnv_add(h, q, 2u * cnt); break;
      case 'u': case 'i': case 'f': h = fnv_add(h, q, 4u * cnt); break;
      case 'd': h = fnv_add(h, q, 8u * cnt); break;
      case 'x': h = fnv_add(h, q, f->count); break;
      case 'o': if (g_blocks) { void *p; HAl *a; memcpy(&p, q, sizeof p); a = h_find_al(p, 0); if (a && !a->freed) h = fnv_add(h, a->p, a->size); } break;
      default: break;
    }
  }
  if (g_foreign && w->ce) {
    h = hash_flds(h, w->ce, f_EvObj, NF_EvObj);
    h = hash_flds(h, w->cd, f_CdObj, NF_CdObj);
    h = hash_flds(h, w->par, f_V90ParObj, NF_V90ParObj);
    h = hash_flds(h, w->mpar, f_ModParObj, NF_ModParObj);
    h = hash_flds(h, w->cpi, f_AddCp, NF_AddCp);
    h = hash_u32(h, &w->p3->state_); h = hash_u32(h, &w->p3->event_); h = hash_u32(h, &w->p3->cnt_); h = hash_u32(h, &w->p3->lineMode_);
    h = hash_u32(h, &w->p4->state_); h = hash_u32(h, &w->p4->w38_); h = hash_u32(h, &w->p4->w3c_); h = hash_u32(h, &w->p4->w40_);
    h = hash_u16(h, &w->dem->z1eb4_);
    h = hash_u32(h, &w->eq->event_); h = hash_u16(h, &w->eq->w8_); h = hash_u32(h, &w->eq->meanErr_); h = hash_u32(h, &w->eq->flag148_); h = hash_u32(h, &w->eq->linLen_);
    h = hash_u32(h, &w->adid->pcmType_); h = hash_u32(h, &w->adid->codecIsAlaw_); h = hash_u16(h, &w->adid->unsuspectedPhase_); h = fnv_add(h, w->adid->suspectedPhase_, 6);
    h = hash_u32(h, &w->mp2->w0_); h = fnv_add(h, w->info, sizeof(InfoObj));
    if (w->diag) h = fnv_add(h, w->diag, sizeof(DiagObj));
    if (w->rbs) h = fnv_add(h, w->rbs, 24);
  }
  return h;
}
static void rec_add(Log *lg, const W *w, char kind, const char *fmt, ...) {
  char buf[TXTMAX * 2], snap[64];
  va_list ap;
  Rec *r = &lg->r[lg->n < LOGMAX ? lg->n : LOGMAX - 1];
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  snprintf(snap, sizeof snap, "|S=%08x", snap_hash(w));
  r->t[0] = kind; r->t[1] = ' '; r->t[2] = 0;
  esc_cat(r->t, sizeof r->t, buf);
  if (strlen(r->t) + strlen(snap) + 1 >= sizeof r->t) h_die3("record text does not fit (%zu + %zu)", strlen(r->t), strlen(snap));
  strcat(r->t, snap);
  lg->n++; lg->total++;
  if (lg->n > LOGMAX) h_die3("more than %d records in one call", LOGMAX);
}
#define REC(kind, ...) rec_add(g_log, &g_w, (kind), __VA_ARGS__)
#define CHECK(p, what) do { if ((void *)(p) != (void *)(what)) h_die3("callee %s got an unexpected object %p (expected %p)", __func__, (void *)(p), (void *)(what)); } while (0)

/* a pointer argument as text: the object it points into (never a raw address); heap blocks by their allocation id */
static const char *idp(const void *p, char *b) {
  const W *w = &g_w;
  const T *d = w->d;
  const uint8_t *q = (const uint8_t *)p;
  HAl *a;
  if (!p) return "NULL";
#define IN(base_, sz_, nm_) if ((base_) && q >= (const uint8_t *)(base_) && q < (const uint8_t *)(base_) + (sz_)) { snprintf(b, 40, nm_ "+%d", (int)(q - (const uint8_t *)(base_))); return b; }
  if (q == (const uint8_t *)&d->agc_mix_) return "agc";
  if (q == (const uint8_t *)&d->pf_fir_coef_) return "pf";
  if (q == (const uint8_t *)&d->rs_vptr_) return "rs";
  if (q == (const uint8_t *)&d->cpow_tab_) return "cpow";
  if (q == (const uint8_t *)&d->scr_buf_) return "scr";
  if (q == (const uint8_t *)&d->sv_params_) return "sv";
  IN(w->adid, sizeof(AdidObj), "adid");
  IN(w->ucode, 32, "ucode");
  IN(w->mp1, sizeof(MapPar90), "mp1");
  IN(w->mp2, sizeof(MapPar90), "mp2");
  IN(w->par, sizeof(V90ParObj), "par");
  IN(w->info, sizeof(InfoObj), "info");
  IN(w->jd, 64, "jd"); IN(w->jd92, 64, "jd92"); IN(w->dil, 64, "dil"); IN(w->cp, 64, "cp"); IN(w->mp, sizeof(MpObj), "mp");
  IN(w->p3, sizeof(Ph3DemObj), "p3"); IN(w->p4, sizeof(Ph4DemObj), "p4"); IN(w->dem, sizeof(DemObj), "dem"); IN(w->eq, sizeof(EquObj), "eq");
  IN(w->ce, sizeof(EvObj), "ce"); IN(w->cd, sizeof(CdObj), "cd"); IN(w->trn2d, 16, "trn");
  IN(w->diag, sizeof(DiagObj), "diag");
  a = h_find_al(p, 1);
  if (a) { snprintf(b, 40, "A%u+%d", (unsigned)a->id, (int)(q - a->p)); return b; }
  IN(w->d, SZ, "d");
  return "?";
#undef IN
}

/* ------------------------------------------------------------------------------------------------ pickers (every float is built from bit patterns) */
static uint32_t pick_fb(void) {
  static const uint32_t b[] = { 0x00000000u, 0x80000000u, 0x7fc00000u, 0xffc00000u, 0x7f800000u, 0xff800000u, 0x3f800000u, 0xbf800000u, 0x00000001u, 0x7f7fffffu, 0x3a83126fu, 0x358637bdu, 0x38d1b717u,
                                0x3f000000u, 0x40000000u, 0x3f7fffffu, 0x3f800001u, 0x42c80000u, 0x461c4000u, 0x49742400u, 0xc61c4000u, 0x3dcccccdu, 0x4cbebc20u };
  uint32_t v;
  if (h_rpct(35)) v = b[h_rmod(sizeof b / sizeof b[0])];
  else { v = ((uint32_t)(0x31u + h_rmod(24)) << 23) | h_rmod(1u << 23); if (h_rpct(25)) v |= 0x80000000u; }
  if ((v & 0x7f800000u) == 0x7f800000u && (v & 0x007fffffu)) v |= 0x00400000u;
  return v;
}
static uint32_t pick_cnt(void) { return h_rpct(70) ? h_rmod(200) : h_rpct(50) ? h_rnd() : (h_rpct(50) ? 0xffffffffu : 0x80000000u); }
static uint32_t pick_flag(void) { return h_rpct(40) ? 0u : h_rpct(70) ? 1u : h_rnd(); }
static unsigned rnd_range(unsigned lo, unsigned hi) { return lo + h_rmod(hi - lo + 1); }
static uint32_t quiet_bits(uint32_t b) { return ((b & 0x7f800000u) == 0x7f800000u && (b & 0x007fffffu)) ? (b | 0x00400000u) : b; }

/* an extended value that is NOT a float most of the time (the blob rounds the timing statistics with fstp DWORD) */
static long double ext_val(uint32_t a, uint32_t b) {
  switch (a % 29u) {
    case 0: return 0.0L;
    case 1: return -0.0L;
    case 2: return __builtin_nanl("");
    case 3: return -__builtin_nanl("");
    case 4: return __builtin_infl();
    case 5: return -__builtin_infl();
    case 6: return (long double)(int)(b % 2001u) - 1000.0L;
    case 7: return (long double)(int32_t)b * 4294967296.0L;
    case 8: return (long double)(int32_t)b / 16.0L;
    case 9: return ((long double)(int32_t)b) / 2147483648.0L;
    case 10: return -(long double)(b % 1000u) / 1048576.0L;
    case 11: return ((long double)(int32_t)b) / 3.0L;
    case 12: return ((long double)(int32_t)b) / 1000.0L;
    case 13: return (long double)(b % 100000u) / 7.0L;
    case 14: return -(long double)(b % 100000u) / 7.0L;
    case 15: return 0.9999999L;
    case 16: return -0.9999999L;
    case 17: return 2147483647.0L + (long double)(b % 4u);
    case 18: return -2147483648.0L - (long double)(b % 4u);
    case 19: return (long double)(int32_t)b / 1048576.0L;
    case 20: return ((long double)(int32_t)b) * 1.0e-6L;
    case 21: return (long double)(b % 20000u) / 10.0L;              /* near 1000 * std thresholds */
    case 22: return (long double)(b % 1000u) / 1000.0L;
    case 23: return 3.4028234e38L * (long double)(1 + (b & 1u));    /* beyond float: rounds to infinity */
    case 24: return -2147483648.0L - (long double)(b % 400000u) - 0.3L;   /* chop = INT_MIN (indefinite), the fraction product up to 2^31 (the abs of a value >= 2^29) */
    case 25: return -2147483648.0L - (long double)(b % 300000u) / 7.0L;
    case 26: return (long double)(int32_t)b * 2.0L;
    default: return ((long double)(int32_t)b) / 8.0L;
  }
}

/* ------------------------------------------------------------------------------------------------ pokes: callees that change a word the method reads (again) afterwards */
enum { PK_SVFLAG, PK_SESSION, PK_RETRAIN, PK_MAXTX, PK_UINFO, PK_RTD, PK_PHASE, PK_P3EVENT, PK_PCM, PK_CODEC, PK_UNSUS, PK_CDSTATE, PK_SYMCNT, PK_FLAG40, PK_N };
static void poke_one(unsigned which, uint32_t v) {
  W *w = &g_w;
  switch (which) {
    case PK_SVFLAG: w->d->sv_flag_ = v % 5u; break;
    case PK_SESSION: w->d->session_ ^= 1u; break;
    case PK_RETRAIN: w->d->retrainOk_ ^= 1u; break;
    case PK_MAXTX: w->info->maxTx_ = (uint8_t)v; break;
    case PK_UINFO: w->info->uinfo_ = (uint8_t)v; break;
    case PK_RTD: w->info->rtd_ = v % 200u; break;
    case PK_PHASE: w->d->phase_ = v % 6u; break;
    case PK_P3EVENT: w->p3->event_ = (v & 1u) ? 0x14u : (v >> 1); break;
    case PK_PCM: w->adid->pcmType_ ^= 1; break;
    case PK_CODEC: w->adid->codecIsAlaw_ ^= 1; break;
    case PK_UNSUS: w->adid->unsuspectedPhase_ = (uint16_t)v; break;
    case PK_SYMCNT: w->d->symCnt_ = v | 1u; break;
    case PK_FLAG40: w->d->flag40_ = v | 1u; break;
    default: w->cd->state_ = v; break;
  }
}
static int g_nopoke;   /* the constructor / destructor scenarios: the world is partial or freed */
static void poke_from(const unsigned *set, unsigned n) {
  uint32_t r = sw(K_POKE), v = sw(K_POKE);
  if (g_nopoke) return;
  if (r % 100u >= 40u) return;
  poke_one(set[(r >> 8) % n], v);
}
static void poke_any(void) {
  uint32_t r = sw(K_POKE), v = sw(K_POKE);
  if (g_nopoke) return;
  if (r % 12u != 0) return;
  poke_one((r >> 8) % PK_N, v);
}
static const unsigned pk_phase[] = { PK_PHASE }, pk_look[] = { PK_SVFLAG, PK_SESSION }, pk_ucode[] = { PK_MAXTX, PK_PCM, PK_CODEC, PK_UNSUS, PK_SVFLAG, PK_MAXTX },
                      pk_disp[] = { PK_RETRAIN, PK_UINFO }, pk_exitdil[] = { PK_P3EVENT, PK_P3EVENT, PK_PHASE }, pk_ep4[] = { PK_PHASE, PK_RTD }, pk_cdreset[] = { PK_SYMCNT, PK_FLAG40, PK_SYMCNT }, pk_flag40[] = { PK_FLAG40 };

/* ------------------------------------------------------------------------------------------------ the scripted behaviour of the callees */
static void env_print(char kind, const char *fmt, va_list ap) {
  char buf[TXTMAX * 2];
  uint32_t r = sw(K_PRN);
  vsnprintf(buf, sizeof buf, fmt, ap);
  REC(kind, "%s", buf);
  poke_any();
  if (r % 6u == 0) dsplibs_debug_level = (r >> 8) % 3u;   /* a print that changes the level: the method re-tests it before every print */
}
static int env_eia6(void) { uint32_t r = sw(K_PF); REC('c', "isV90WithEia6"); poke_from(pk_phase, 1); return (int)((r >> 4) & 1u); }
static long double env_tm(const char *nm) { uint32_t a = sw(K_TM), b = sw(K_TM); REC('c', "%s", nm); poke_any(); return ext_val(a, b); }
static unsigned env_look(const char *nm) { uint32_t r = sw(K_JD); REC('c', "%s", nm); poke_from(pk_look, 2); return r; }
static unsigned char *env_maxucode(void) { REC('c', "getMaxUcode"); poke_from(pk_ucode, 6); return g_w.ucode; }
static int env_altrbs(void) { uint32_t r = sw(K_ADID); REC('c', "isThereAnyAltRbsPhase"); poke_any(); return (int)((r % 3u == 0) ? (r & 0xffff0000u) : (r % 3u == 1) ? (r | 0x10000u) | 1u : (r & 0xffffu) | 0x8000u); }
static void env_trnnof(int n) { REC('c', "setNofUcodesInTrn2(%d)", n); poke_any(); }
static int env_trndesign(void *mp, void *A, void *B, void *C, void *D, int pcmA, int pcm, int unused, void *cur, unsigned lim, unsigned pidx, unsigned ssc) {
  uint32_t r = sw(K_TRN);
  char b0[48], b1[48], b2[48], b3[48], b4[48], b5[48];
  REC('c', "V90TRN2Design(%s,%s,%s,%s,%s,%d,%d,%d,%s,%u,%u,%u)", idp(mp, b0), idp(A, b1), idp(B, b2), idp(C, b3), idp(D, b4), pcmA, pcm, unused, idp(cur, b5), lim, pidx, ssc);
  poke_from(pk_disp, 2);
  return (int)((r % 3u == 0) ? (r & 0xffff0000u) : (r % 3u == 1) ? (r | 1u) : r);
}
static void env_p3reset(unsigned pcm, unsigned code, unsigned st, unsigned n, void *jd, void *jd92, void *dil, int a8, int a9, uint32_t f, unsigned tb) {
  char b0[48], b1[48], b2[48];
  REC('c', "p3.reset(%u,%u,%u,%u,%s,%s,%s,%d,%d,%08x,%u)", pcm, code, st, n, idp(jd, b0), idp(jd92, b1), idp(dil, b2), a8, a9, f, tb);
  poke_any();
}
static void env_p4reset(unsigned code, unsigned st, unsigned a3, unsigned a4) { REC('c', "p4.reset(%u,%u,%u,%u)", code, st, a3, a4); poke_any(); }
static void env_cdreset(void) { REC('c', "cd.reset"); poke_from(pk_cdreset, 3); }
static void env_eqecv(void) { REC('c', "eq.enterChannelVerification"); poke_from(pk_flag40, 1); }
static void env_plain(const char *nm) { REC('c', "%s", nm); poke_any(); }
static void env_u(const char *nm, unsigned v) { REC('c', "%s(%u)", nm, v); poke_any(); }
static void env_dispspec(void *mp) { char b[48]; REC('c', "displaySpectralParams(%s)", idp(mp, b)); poke_from(pk_disp, 2); }

/* the impl's recorders */
void c_scrreset(void *p, unsigned char v) { CHECK(p, &g_w.d->scr_buf_); env_u("scr.reset", (unsigned)v); }
void c_agcreset(void *p) { CHECK(p, &g_w.d->agc_mix_); env_plain("agc.reset"); }
void c_pfreset(void *p) { CHECK(p, &g_w.d->pf_fir_coef_); env_plain("pf.reset"); }
int c_eia6(void *p) { CHECK(p, &g_w.d->pf_fir_coef_); return env_eia6(); }
void c_pfsel(void *p) { CHECK(p, &g_w.d->pf_fir_coef_); env_plain("pf.selectFilter"); }
void c_pfsetp(void *p) { CHECK(p, &g_w.d->pf_fir_coef_); env_plain("pf.setParamEia6"); }
void c_pfdisp(void *p) { CHECK(p, &g_w.d->pf_fir_coef_); env_plain("pf.displayParamEia6"); }
void c_rsreset(void *p) { CHECK(p, &g_w.d->rs_vptr_); env_plain("rs.reset"); }
void c_bll(void *p, unsigned st, unsigned fl) { uint32_t r = sw(K_BLL); CHECK(p, &g_w.d->rs_vptr_); REC('c', "setBllState(%u,%u)", st, fl); g_w.d->rs_state_ = st; g_w.d->rs_stateCount_ = r; poke_any(); }
long double c_tmean(void *p) { CHECK(p, &g_w.d->rs_vptr_); return env_tm("getTimingHistoryMean"); }
long double c_tstd(void *p) { CHECK(p, &g_w.d->rs_vptr_); return env_tm("getTimingHistoryStd"); }
long double c_tmeanLD(void *p) { CHECK(p, &g_w.d->rs_vptr_); return env_tm("getTimingHistoryMean"); }
long double c_tstdLD(void *p) { CHECK(p, &g_w.d->rs_vptr_); return env_tm("getTimingHistoryStd"); }
void c_settoff(void *p, FW x) { CHECK(p, &g_w.d->rs_vptr_); REC('c', "setTimingOffset(%08x)", fw_bits(x)); poke_any(); }
void c_svreset(void *p) { CHECK(p, &g_w.d->sv_params_); env_plain("sv.reset"); }
void c_eqreset(void *p, unsigned a) { CHECK(p, g_w.eq); env_u("eq.reset", a); }
void c_eqep3(void *p) { CHECK(p, g_w.eq); env_plain("eq.enterPhase3"); }
void c_eqep4(void *p) { CHECK(p, g_w.eq); REC('c', "eq.enterPhase4"); poke_from(pk_ep4, 2); }
void c_eqecv(void *p) { CHECK(p, g_w.eq); env_eqecv(); }
void c_p3clear(void *p) { CHECK(p, g_w.p3); env_plain("p3.clearVerificationStatus"); }
void c_p3sess(void *p, unsigned f) { CHECK(p, g_w.p3); env_u("p3.setSessionFlag", f); }
void c_p3reset(void *p, unsigned pcm, unsigned char code, unsigned st, unsigned n, void *jd, void *jd92, void *dil, short a8, short a9, FW f, unsigned tb) {
  CHECK(p, g_w.p3); env_p3reset(pcm, (unsigned)code, st, n, jd, jd92, dil, (int)a8, (int)a9, fw_bits(f), tb);
}
void c_p3exitdil(void *p) { CHECK(p, g_w.p3); REC('c', "p3.exitDIL"); poke_from(pk_exitdil, 3); }
unsigned char *c_maxucode(void *p) { CHECK(p, g_w.p3); return env_maxucode(); }
void c_p4sess(void *p, unsigned f) { CHECK(p, g_w.p4); env_u("p4.setSessionFlag", f); }
void c_p4reset(void *p, unsigned char code, unsigned st, unsigned a3, unsigned a4) { CHECK(p, g_w.p4); env_p4reset((unsigned)code, st, a3, a4); }
void c_cereset(void *p) { CHECK(p, g_w.ce); env_plain("ce.reset"); }
void c_ceremote(void *p) { CHECK(p, g_w.ce); env_plain("ce.indicateRemoteRateReneg"); }
void c_cdreset(void *p) { CHECK(p, g_w.cd); env_cdreset(); }
void c_demlin(void *p, unsigned len) { CHECK(p, g_w.dem); env_u("dem.resetLinearMappStudy", len); }
void c_printinfo(const void *p) { CHECK(p, g_w.info); env_plain("info.printInfo"); }
int c_altrbs(void *p) { CHECK(p, g_w.adid); return env_altrbs(); }
void c_trnnof(void *p, short n) { CHECK(p, g_w.trn2d); env_trnnof((int)n); }
int c_trndesign(void *p, void *mp, void *A, void *B, void *C, void *D, int pcmA, int pcm, short unused, void *cur, unsigned lim, unsigned char pidx, unsigned ssc) {
  CHECK(p, g_w.trn2d);
  return env_trndesign(mp, A, B, C, D, pcmA, pcm, (int)unused, cur, lim, (unsigned)pidx, ssc);
}
unsigned c_v90look(void *p) { CHECK(p, g_w.jd); return env_look("V90Jd.getMaxLookahead"); }
unsigned c_v92look(void *p) { CHECK(p, g_w.jd92); return env_look("V92Jd.getMaxLookahead"); }
void c_dispspec(void *p) { env_dispspec(p); }
void edprintf(const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print('e', fmt, ap); va_end(ap); }
int dsplibs_debug_printf(const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print('d', fmt, ap); va_end(ap); return 0; }

/* ---- constructors / destructors of the sub-objects: recorders (the embedded ones by member name, the heap ones by allocation id) ---- */
static int g_check_stamp = 1;
static const char *cls_name[] = { "adid", "ce", "p3", "dem", "p4", "eq", "trn", "cd" };
enum { CL_ADID, CL_CE, CL_P3, CL_DEM, CL_P4, CL_EQ, CL_TRN, CL_CD };
static void stamp_put(void *o, unsigned cls) { uint32_t s = 0xc7a00000u | (cls << 8) | 0x5au; memcpy(o, &s, 4); }
static void stamp_chk(void *o, unsigned cls) {
  uint32_t s;
  memcpy(&s, o, 4);
  if (g_check_stamp && s != (0xc7a00000u | (cls << 8) | 0x5au)) h_die3("destructor of %s got an object that no constructor stamped (%08x)", cls_name[cls], s);
}
static void env_k(const char *nm, void *o, int heap, unsigned cls, const char *fmt, ...) {
  char b[48], a[400];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(a, sizeof a, fmt, ap);
  va_end(ap);
  if (heap) stamp_put(o, cls);
  REC('c', "ctor %s(%s%s%s)", nm, idp(o, b), a[0] ? "," : "", a);
  poke_any();
}
static void env_x(const char *nm, void *o, int heap, unsigned cls) {
  char b[48];
  if (heap) stamp_chk(o, cls);
  REC('c', "dtor %s(%s)", nm, idp(o, b));
}
void k_agc(void *p) { env_k("Agc", p, 0, 0, ""); }
void k_pf(void *p, unsigned codec, void *info, void *par) { char b1[48], b2[48]; env_k("V90PreFilter", p, 0, 0, "%u,%s,%s", codec, idp(info, b1), idp(par, b2)); }
void k_rs(void *p, unsigned L, FW step, unsigned taps, FW cut, void *par, FW ppm, unsigned hist) {
  char b1[48];
  env_k("V90Resampler", p, 0, 0, "%u,%08x,%u,%08x,%s,%08x,%u", L, fw_bits(step), taps, fw_bits(cut), idp(par, b1), fw_bits(ppm), hist);
}
void k_cpow(void *p) { env_k("V90ConstellationPower", p, 0, 0, ""); }
void k_scr(void *p, unsigned a, unsigned b, unsigned c) { env_k("Descrambler", p, 0, 0, "%u,%u,%u", a, b, c); }
void k_sv(void *p, void *par) { char b1[48]; env_k("V90SpectralVerifier", p, 0, 0, "%s", idp(par, b1)); }
void k_adid(void *p, void *par) { char b1[48]; env_k("V90AutoDigitalImpDetector", p, 1, CL_ADID, "%s", idp(par, b1)); }
void k_ce(void *p, void *par) { char b1[48]; env_k("V90ConnectionEvaluator", p, 1, CL_CE, "%s", idp(par, b1)); }
void k_p3(void *p, void *par, void *sv, unsigned session, void *adid) { char b1[48], b2[48], b3[48]; env_k("V90Phase3Demodulator", p, 1, CL_P3, "%s,%s,%u,%s", idp(par, b1), idp(sv, b2), session, idp(adid, b3)); }
void k_dem(void *p, unsigned cap, void *par, void *adid) { char b1[48], b2[48]; env_k("V90Demapper", p, 1, CL_DEM, "%u,%s,%s", cap, idp(par, b1), idp(adid, b2)); }
void k_p4(void *p, void *mp1, void *mp2, void *dem, void *cp, void *mp, void *scr, void *ce, void *par, void *p3, void *adid, unsigned session) {
  char b[10][48];
  env_k("V90Phase4Demodulator", p, 1, CL_P4, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%u", idp(mp1, b[0]), idp(mp2, b[1]), idp(dem, b[2]), idp(cp, b[3]), idp(mp, b[4]), idp(scr, b[5]), idp(ce, b[6]), idp(par, b[7]), idp(p3, b[8]),
        idp(adid, b[9]), session);
}
void k_eq(void *p, unsigned ll, unsigned dl, void *p3, void *p4, void *dem, void *ce, void *sv, void *par, void *rs, void *pf, unsigned mode) {
  char b[8][48];
  env_k("V90Equalizer", p, 1, CL_EQ, "%u,%u,%s,%s,%s,%s,%s,%s,%s,%s,%u", ll, dl, idp(p3, b[0]), idp(p4, b[1]), idp(dem, b[2]), idp(ce, b[3]), idp(sv, b[4]), idp(par, b[5]), idp(rs, b[6]), idp(pf, b[7]), mode);
}
void k_trn(void *p, void *par, void *pw) { char b1[48], b2[48]; env_k("V90TRN2Designer", p, 1, CL_TRN, "%s,%s", idp(par, b1), idp(pw, b2)); }
void k_cd(void *p, void *par, void *pf, void *pw) { char b1[48], b2[48], b3[48]; env_k("V90ConstellationDesigner", p, 1, CL_CD, "%s,%s,%s", idp(par, b1), idp(pf, b2), idp(pw, b3)); }
void x_pf(void *p) { env_x("V90PreFilter", p, 0, 0); }
void x_rs(void *p) { env_x("V90Resampler", p, 0, 0); }
void x_cpow(void *p) { env_x("V90ConstellationPower", p, 0, 0); }
void x_scr(void *p) { env_x("Descrambler", p, 0, 0); }
void x_sv(void *p) { env_x("V90SpectralVerifier", p, 0, 0); }
void x_adid(void *p) { env_x("V90AutoDigitalImpDetector", p, 1, CL_ADID); }
void x_ce(void *p) { env_x("V90ConnectionEvaluator", p, 1, CL_CE); }
void x_p3(void *p) { env_x("V90Phase3Demodulator", p, 1, CL_P3); }
void x_dem(void *p) { env_x("V90Demapper", p, 1, CL_DEM); }
void x_p4(void *p) { env_x("V90Phase4Demodulator", p, 1, CL_P4); }
void x_eq(void *p) { env_x("V90Equalizer", p, 1, CL_EQ); }
void x_trn(void *p) { env_x("V90TRN2Designer", p, 1, CL_TRN); }
void x_cd(void *p) { env_x("V90ConstellationDesigner", p, 1, CL_CD); }

/* ------------------------------------------------------------------------------------------------ the independent model */
/* written from the disassembly (porting/decompiled/dp/p44_demodulator_rest.txt) in a different shape than the port; its own extended-precision helpers.  It runs on the LIVE world first (saved before and restored
 * afterwards) through the same callee environment and must leave the same world, return value and records (with the same snapshot hashes) as the impl. */
typedef long double XL;
enum { G_ALL = 31, G_S0 = 1, G_S1 = 2, G_S2 = 4, G_S3 = 8, G_S4 = 16, G_S23 = 12, G_S234 = 28, G_S02 = 5, G_S04 = 17 };   /* the scenarios that must reach a site */
typedef struct { unsigned long n; unsigned char hit, grp; const char *name; } CovSite;
#define CV(g, nm) do { static CovSite cs_ __attribute__((section("covsites"), used, aligned(sizeof(void *)))) = { 0, 0, (g), (nm) }; cs_.hit = 1; } while (0)
extern CovSite __start_covsites[], __stop_covsites[];
static void cov_flush(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) if (c->hit) { c->n++; c->hit = 0; } }
static void cov_clear(void) { CovSite *c; for (c = __start_covsites; c < __stop_covsites; c++) c->hit = 0; }

static int m_f32(XL v) { if (v != v || v >= 2147483648.0L || v <= -2147483649.0L) return INT_MIN; return (int)v; }
static int m_abs(int v) { return v < 0 ? (int)(0u - (unsigned)v) : v; }
static int m_sgn(XL v) { return (v > 0.0L || v != v) ? '+' : '-'; }
static float m_rf(XL v) { volatile float t = (float)v; return t; }
static XL m_fabs(XL v) { return v < 0.0L ? -v : v; }
static void m_pf(char kind, const char *fmt, ...) { va_list ap; va_start(ap, fmt); env_print(kind, fmt, ap); va_end(ap); }
static int m_dbg(void) { return dsplibs_debug_level > 1; }
static void m_fix(char kind, const char *fmt, XL v, XL scale) { int t = m_f32(v); m_pf(kind, fmt, m_sgn(v), m_f32(m_fabs(v)), m_abs(m_f32((v - (XL)t) * scale))); }
static uint32_t m_w(const float *p) { uint32_t b; memcpy(&b, p, 4); return b; }
/* log10 the way the blob does it: fldlg2; fyl2x (st1 * log2(st0)) */
static XL m_log10(XL x) { XL r; __asm__("fldlg2\n\tfxch\n\tfyl2x" : "=t"(r) : "0"(x) : "st(1)"); return r; }
static XL m_ld(float v) { XL r = v; __asm__("" : "+t"(r)); return r; }   /* fld DWORD: quiets a signalling NaN (x86-64: gcc would fold the round trip) */
static unsigned m_rate(void) { T *d = g_w.d; unsigned x; if (!d->designed_) return 0; x = g_w.mp2->w0_ * 8000u; return (unsigned)(long long)(0.5L + (XL)(unsigned long long)x * (XL)0.1666666716337204f); }
#define S_ST "V90Demodulator on sessionTermination: "
#define PFX "V90Demodulator: "

/* sessionTermination / enterDataSteadyState: the timing statistics */
static void m_timing(int session) {
  T *d = g_w.d;
  V90ParObj *p = g_w.par;
  XL mean = (XL)m_rf(env_tm("getTimingHistoryMean"));
  XL sd = (XL)m_rf(env_tm("getTimingHistoryStd"));
  m_fix('e', session ? S_ST "mean of timing offset History  = %c%d.%04d\r\n" : PFX "mean of timing offset History  = %c%d.%04d\r\n", mean, 10000.0L);
  m_fix('e', session ? S_ST "std of timing offset History  = %c%d.%04d\r\n" : PFX "std of timing offset History  = %c%d.%04d\r\n", sd, 10000.0L);
  m_pf('e', session ? S_ST "1000* std = %d\r\n" : PFX "1000* std = %d\r\n", m_f32(sd * 1000.0L));
  (void)d;
  if ((XL)p->timingOffesetMinStdForSave >= sd) {
    int v;
    CV(G_S02, "timing: saved");
    m_pf('e', session ? S_ST "Timing offset saved in Registry!\r\n" : PFX "Timing offset saved in Registry!\r\n");
    v = m_f32(1000.0L * mean);
    memcpy(g_w.mpar->unk4c, &v, 4);
  } else CV(G_S02, "timing: not saved (min < std or unordered)");
}
static int m_sessterm(void) {
  T *d = g_w.d;
  int notsaved = 1;
  if (d->phase_ == 3) { CV(G_S0, "sessterm: phase 3"); notsaved = env_eia6() != 0; if (notsaved) CV(G_S0, "sessterm: phase 3 + eia6"); }
  else CV(G_S0, "sessterm: other phase");
  if (notsaved) {
    int e = env_eia6();
    m_pf('e', S_ST "Timing offset NOT saved to registry (isDataState = %d, isEia6 = %d)\r\n", d->phase_ == 3 ? 1 : 0, e);
    if (d->phase_ == 3) CV(G_S0, "sessterm: print after phase 3 + eia6");
  } else if (g_w.par->timingHistoryEvaluationEnabled == 0) {
    CV(G_S0, "sessterm: evaluation disabled");
    m_pf('e', S_ST "Timing offset NOT saved to registry, EVALUATION DISABLED !\n");
  } else {
    CV(G_S0, "sessterm: statistics");
    m_timing(1);
  }
  env_plain("p3.clearVerificationStatus");
  return 0;
}

static void m_set_p4_periods(T *d) { d->errPrintPeriod_ = (uint32_t)g_w.par->errorEnergyPrintPeriodPhase4; d->tmPrintPeriod_ = (uint32_t)g_w.par->timingOffsetPrintPeriodPhase4; }
static void m_phase4_inline(T *d) {
  d->phase_ = 2; d->timeCnt_ += d->symCnt_; d->symCnt_ = 0; d->timeLimit_ = 0x28230u + 5u * g_w.info->rtd_; m_set_p4_periods(d);
}
static void m_ep4(void) {
  T *d = g_w.d;
  if (d->phase_ == 2) { CV(G_S2, "ep4: already"); return; }
  CV(G_S2, "ep4: enter");
  if (m_dbg()) { CV(G_S2, "ep4: print"); m_pf('d', PFX "enter Phase 4\r\n"); }
  m_phase4_inline(d);
}
static void m_fpe(void) {
  T *d = g_w.d;
  if (d->phase_ == 2) { CV(G_S2, "fpe: already"); return; }
  CV(G_S2, "fpe: enter");
  if (m_dbg()) { CV(G_S2, "fpe: print"); m_pf('d', PFX "FPE detected: enter Phase 4\r\n"); }
  d->symCnt_ = 0; d->timeCnt_ = 0; d->phase_ = 2; d->timeLimit_ = 0x10680u + 2u * g_w.info->rtd_; m_set_p4_periods(d);
}
static void m_steady(void) {
  T *d = g_w.d;
  unsigned lv;
  if (d->phase_ == 4) { CV(G_S2, "steady: already"); return; }
  CV(G_S2, "steady: enter");
  d->phase_ = 4;
  m_pf('e', PFX "enter Data steady state\r\n");
  d->w278_ = (uint32_t)g_w.par->timingHistoryEvaluationEnabled;
  if (g_w.par->timingHistoryEvaluationEnabled != 0) { CV(G_S2, "steady: statistics"); m_timing(0); }
  lv = dsplibs_debug_level;
  g_w.dem->z1eb4_ = 0;
  if (lv > 1) { CV(G_S2, "steady: print"); m_pf('d', PFX "disable linear mapping study.\n"); }
}
static void m_rrn(void) {
  T *d = g_w.d;
  unsigned flag = 0, lv;
  if (d->phase_ == 2) { CV(G_S2, "rrn: already"); return; }
  CV(G_S2, "rrn: enter");
  if (m_dbg()) { CV(G_S2, "rrn: print"); m_pf('d', PFX "RRN detected: enter Phase 4\r\n"); }
  d->symCnt_ = 0; d->timeCnt_ = 0; d->phase_ = 2; d->timeLimit_ = 0x10680u + 2u * g_w.info->rtd_; m_set_p4_periods(d);
  if (g_w.ce->rrnSilence_ != 0) { if (g_w.p4->w3c_ != 0) { flag = g_w.p4->w38_ != 0; if (flag) CV(G_S2, "rrn: silence + w3c + w38"); else CV(G_S2, "rrn: silence + w3c, w38 = 0"); } else CV(G_S2, "rrn: silence, w3c = 0"); }
  else CV(G_S2, "rrn: no silence");
  g_w.cpi->w10_ = flag;
  d->designed_ = 0;
  lv = dsplibs_debug_level;
  g_w.dem->z1eb4_ = 0;
  if (lv > 1) { CV(G_S2, "rrn: print 2"); m_pf('d', PFX "disable linear mapping study\n"); }
}
static void m_setsess(unsigned f) {
  m_pf('e', PFX "setSessionFlag, flag = %d\r\n", f);
  g_w.d->session_ = f;
  env_u("p3.setSessionFlag", f);
  env_u("p4.setSessionFlag", f);
}
static void m_p3reset_call(unsigned st, int a8, int a9) {
  W *w = &g_w;
  env_p3reset(w->info->pcm_, w->info->uinfo_, st, 0, w->jd, w->jd92, w->dil, a8, a9, 0u, w->info->rtd_);
}
static void m_ep3(void) {
  T *d = g_w.d;
  W *w = &g_w;
  unsigned s;
  int e;
  if (d->phase_ == 1) { CV(G_S2, "ep3: already"); return; }
  CV(G_S2, "ep3: enter");
  if (m_dbg()) { CV(G_S2, "ep3: print"); m_pf('d', PFX "enter Phase 3\r\n"); }
  d->phase_ = 1; s = d->symCnt_; d->symCnt_ = 0; d->state_ = 0; d->timeCnt_ = s; d->flag40_ = 0;
  env_plain("info.printInfo");
  env_plain("pf.selectFilter");
  if (env_eia6()) {
    CV(G_S2, "ep3: eia6");
    env_plain("pf.setParamEia6"); env_plain("pf.displayParamEia6");
    REC('c', "setTimingOffset(%08x)", m_w(&w->par->initialBaudOffset)); poke_any();
  } else CV(G_S2, "ep3: no eia6");
  env_plain("sv.reset");
  e = (int16_t)env_eia6();
  m_p3reset_call(0, e, 1);
  w->p3->lineMode_ = d->retrainOk_;
  env_plain("eq.enterPhase3");
  d->errPrintPeriod_ = (uint32_t)w->par->errorEnergyPrintPeriodPhase3; d->tmPrintPeriod_ = (uint32_t)w->par->timingOffsetPrintPeriodPhase3;
  w->ce->phase3EvalActive_ = 0; w->ce->avePdsnr_ = 0.0f; w->ce->trn1dEvalActive_ = 0; w->ce->weight_ = 0;
  d->designed_ = 0;
  if ((int8_t)w->mpar->unk01[1] < 0) { CV(G_S2, "ep3: modem flag bit 7"); return; }
  if (d->session_ != 0) { CV(G_S2, "ep3: v92lite"); m_pf('e', PFX "we got PCM upstream under V.92Lite, retraining to V.34 upstream...\r\n"); d->state_ = 0x20; }
  else CV(G_S2, "ep3: session 0");
}
static int m_bitrate(void) { return (int)m_rate(); }
static void m_rbs(unsigned *out) { unsigned i; for (i = 0; i < 6; i++) out[i] = g_w.adid->suspectedPhase_[i]; }
static void m_indic(void) { env_plain("ce.indicateRemoteRateReneg"); g_w.cd->state_ = 1; }
static void m_atud(DiagObj *r) {
  T *d = g_w.d;
  EquObj *e = g_w.eq;
  unsigned rbs[6], i;
  uint64_t x;
  r->cnt23_ = d->cnt23_; r->cnt1f_ = d->cnt1f_; r->cnt26_ = d->cnt26_;
  r->bitRate_ = m_rate(); r->zeroBc_ = 0; r->symbolRate_ = 8000;
  memcpy(&r->meanErrWord_, &e->meanErr_, 4);
  { XL l = 10.0L * (XL)m_log10((XL)e->meanErr_); r->meanErrDb_ = m_rf(l); }
  { XL l = 10.0L * (XL)m_log10((XL)d->agc_power_); r->powerDb_ = m_rf(l); }
  x = (uint64_t)g_w.info->rtd_ * 10u; if (x > 0xffffffffull) x &= 0xffffffffull;
  r->rtdUnits_ = (uint32_t)(((x * 0xaaaaaaabull) >> 32) >> 6);
  for (i = 0; i < 6; i++) rbs[i] = g_w.adid->suspectedPhase_[i];
  r->rbs_ = rbs[0] + 2u * (rbs[1] + 2u * (rbs[2] + 2u * (rbs[3] + 2u * (rbs[4] + 2u * rbs[5]))));
  m_pf('e', "V90 Diagnostics\r\n");
  m_pf('e', "-------------------------\r\n");
  m_pf('e', "RBS : %d (%d%d%d%d%d%d)\r\n", r->rbs_, rbs[0], rbs[1], rbs[2], rbs[3], rbs[4], rbs[5]);
}
static void m_exit3(void) {
  T *d = g_w.d;
  W *w = &g_w;
  XL g;
  unsigned ssc, la, lv;
  short alt;
  int r;
  unsigned char cur_maxtx;
  if (d->phase_ != 1) { CV(G_S3, "exit3: not phase 1"); return; }
  CV(G_S3, "exit3: enter");
  g = m_ld(d->dilScale_);
  { float f = (float)g; memcpy(&w->cpi->f8_, &f, 4); }
  w->cpi->w0_ = 0; w->cpi->w4_ = 0;
  m_fix('e', PFX "TRN1d RMS Ratio = %c%d.%08d\r\n", g, 100000000.0L);
  w->cpi->w10_ = 0; w->cpi->wc_ = (uint32_t)w->adid->codecIsAlaw_;
  { uint16_t m = (uint16_t)w->par->analogRateMask; memcpy(&w->cpi->w14_, &m, 2); }
  REC('c', "p3.exitDIL"); poke_from(pk_exitdil, 3);
  if (w->p3->event_ == 0x14) {
    CV(G_S3, "exit3: p3 event 0x14");
    if (d->phase_ != 2) { CV(G_S3, "exit3: enter phase 4"); if (m_dbg()) { CV(G_S3, "exit3: print enter phase 4"); m_pf('d', PFX "enter Phase 4\r\n"); } m_phase4_inline(d); }
    else CV(G_S3, "exit3: already phase 4");
    REC('c', "eq.enterPhase4"); poke_from(pk_ep4, 2);
  } else CV(G_S3, "exit3: event != 0x14");
  alt = (int16_t)env_altrbs();
  env_trnnof((int)alt);
  ssc = d->sv_flag_;
  la = (d->session_ == 0 ? env_look("V90Jd.getMaxLookahead") : env_look("V92Jd.getMaxLookahead")) & 0xffu;
  if (d->session_) CV(G_S3, "exit3: V92 lookahead"); else CV(G_S3, "exit3: V90 lookahead");
  (void)env_maxucode();
  cur_maxtx = (uint8_t)(w->info->maxTx_ + 1u);
  r = (int16_t)env_trndesign(w->mp1, w->adid->linearMapping_, w->adid->linearMappingAlt_, w->adid->ucodeAllowed_, w->adid->altRbsPhase_, w->adid->pcmType_, w->adid->codecIsAlaw_, (int16_t)w->adid->unsuspectedPhase_,
                             w->ucode, la, cur_maxtx, ssc);
  if (r == 0) {
    CV(G_S3, "exit3: design result 0");
    lv = dsplibs_debug_level;
    w->ce->delayedRetrainRequest_ = 1;
    if (lv > 1) { CV(G_S3, "exit3: delayed retrain print"); m_pf('d', "V90Demodulator::exitPhase3() delayedRetrainRequest !!!\r\n"); }
  } else CV(G_S3, "exit3: design result != 0");
  m_pf('e', PFX "TRN2d spectral parameters:\r\n");
  env_dispspec(w->mp1);
  env_p4reset(w->info->uinfo_, 0, 0, d->retrainOk_);
}
static void m_edp(void) {
  T *d = g_w.d;
  unsigned lv;
  if (d->phase_ == 3) { CV(G_S2, "edp: already"); return; }
  CV(G_S2, "edp: enter");
  d->symCnt_ = 0; d->phase_ = 3; d->state_ = 0x1e;
  d->errPrintPeriod_ = (uint32_t)g_w.par->errorEnergyPrintPeriodData; d->tmPrintPeriod_ = (uint32_t)g_w.par->timingOffsetPrintPeriodData;
  { uint32_t r = sw(K_BLL); REC('c', "setBllState(%u,%u)", 10u, 1u); d->rs_state_ = 10u; d->rs_stateCount_ = r; poke_any(); }
  if (m_dbg()) { CV(G_S2, "edp: print"); m_pf('d', PFX "enter Data Phase, Rate = %d [bps]\r\n", (int)m_rate()); }
  env_u("dem.resetLinearMappStudy", d->retrainOk_ != 0 ? 0x3138u : 0x7530u);
  if (d->retrainOk_ != 0) CV(G_S2, "edp: retrainOk"); else CV(G_S2, "edp: not retrainOk");
  lv = dsplibs_debug_level;
  g_w.dem->z1eb4_ = 1;
  if (lv > 1) { CV(G_S2, "edp: print 2"); m_pf('d', PFX "reset and enable linear mapping study in data\n"); }
}
static void m_reinit(void) { env_plain("ce.reset"); env_plain("p3.clearVerificationStatus"); }
static void m_reset(unsigned arg) {
  T *d = g_w.d;
  V90ParObj *p = g_w.par;
  int cur;
  if (m_dbg()) { CV(G_S3, "reset: print"); m_pf('d', "V90Demodulator reset, quick connect flag = %d\r\n", arg); }
  d->phase_ = 0; d->quietCnt_ = 0; d->flag270_ = 0; d->symCnt_ = 0; d->state_ = 0; d->timeCnt_ = 0; d->flag40_ = 0;
  env_u("scr.reset", 0);
  env_plain("agc.reset");
  d->agc_period_ = (uint32_t)p->agcBlockLen;
  memcpy(&d->agc_target_, &p->agcNominalEnergy, 4);
  env_plain("pf.reset");
  m_fix('e', "V90Demodulator reset: Baud Offset = %c%d.%03d\r\n", (XL)p->initialBaudOffset, 1000.0L);
  env_plain("rs.reset");
  REC('c', "setTimingOffset(%08x)", m_w(&p->initialBaudOffset)); poke_any();
  cur = p->linearEquCursorPlace;
  if (cur < 0) { CV(G_S3, "reset: cursor < 0"); env_u("eq.reset", g_w.eq->linLen_ >> 1); } else { CV(G_S3, "reset: cursor >= 0"); env_u("eq.reset", (unsigned)cur); }
  env_cdreset();
  d->tmPrintPeriod_ = 0x4b00; d->errPrintCnt_ = 0; d->errPrintPeriod_ = 0x4b00; d->tmPrintCnt_ = 0; d->rsCnt_ = 0; d->eqCnt_ = 0; d->framePos_ = 0; d->w278_ = 0; d->designed_ = 0;
  d->retrainOk_ = arg; g_w.eq->flag148_ = arg;
}
static void m_ecv(short a, short b) {
  T *d = g_w.d;
  (void)a;
  if (m_dbg()) { CV(G_S3, "ecv: print"); m_pf('d', PFX "Enter Channel Verification called !\r\n"); }
  m_reset(1);
  d->phase_ = 5; d->symCnt_ = 0;
  env_plain("p3.clearVerificationStatus");
  m_p3reset_call(0x1a, 0, (int)b);
  env_eqecv();
  d->flag40_ = 0;
}

/* ------------------------------------------------------------------------------------------------ the world: saved before the model, restored for the impl, compared afterwards */
typedef struct { void *p; size_t n; } Reg;
static Reg g_reg[40];
static int g_nreg;
static uint8_t *g_sv0[40], *g_sv1[40];
static void add_reg(void *p, size_t n) { if (!p) return; if (g_nreg >= 40) h_die3("too many regions"); g_reg[g_nreg].p = p; g_reg[g_nreg].n = n; g_nreg++; }
static void build_regions(void) {
  W *w = &g_w;
  g_nreg = 0;
  add_reg(w->d, SZ);
  add_reg(w->par, sizeof(V90ParObj)); add_reg(w->mpar, sizeof(ModParObj)); add_reg(w->ce, sizeof(EvObj)); add_reg(w->p3, sizeof(Ph3DemObj)); add_reg(w->p4, sizeof(Ph4DemObj));
  add_reg(w->dem, sizeof(DemObj)); add_reg(w->cd, sizeof(CdObj)); add_reg(w->eq, sizeof(EquObj)); add_reg(w->adid, sizeof(AdidObj));
  add_reg(w->mp2, sizeof(MapPar90)); add_reg(w->cpi, sizeof(AddCp)); add_reg(w->info, sizeof(InfoObj));
  add_reg(w->diag, sizeof(DiagObj)); add_reg(w->rbs, 24);
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

/* ------------------------------------------------------------------------------------------------ the worlds */
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

/* the buffer size the object is built for: avoid the sizes the x86-64 log mapping would confuse (the 64-bit sizeof of the eight heap classes: 0x10 0x70 0xc0 0x1d0 ...) */
static unsigned pick_n(void) {
  unsigned n;
  for (;;) {
    n = h_rpct(4) ? 0u : h_rpct(50) ? 1u + h_rmod(8) : 1u + h_rmod(NMAX);
    if (n == 4 || n == 2 || n == 48 || n == 24 || n == 16 || n == 28 || n == 14) continue;
    return n;
  }
}

static void *obj_blk(unsigned heap, size_t sz) { void *p = heap ? sysdep_malloc((unsigned)sz) : h_obj_new(sz); memset(p, 0, sz); return p; }
/* the argument objects (heap = 1: sysdep_malloc blocks, logged identically on both sides; sizes are arch independent constants) */
static void new_args(unsigned heap) {
  W *w = &g_w;
  if (sizeof(V90ParObj) > 0x600 || sizeof(MpObj) > 0x140 || sizeof(MapPar90) > 0x700 || sizeof(AddCp) > 0x40) h_die3("argument block too small");
  w->par = (V90ParObj *)obj_blk(heap, 0x600);
  w->mpar = (ModParObj *)obj_blk(heap, 0x80);
  w->mp1 = (MapPar90 *)obj_blk(heap, 0x700);
  w->mp2 = (MapPar90 *)obj_blk(heap, 0x700);
  w->cpi = (AddCp *)obj_blk(heap, 0x40);
  w->info = (InfoObj *)obj_blk(heap, 0x30);
  w->mp = (MpObj *)obj_blk(heap, 0x140);
  w->jd = obj_blk(heap, 64); w->jd92 = obj_blk(heap, 64); w->dil = obj_blk(heap, 64); w->cp = obj_blk(heap, 64);
  w->par->params = w->mpar;
}
static void new_world(void) {   /* the method worlds: every object from the pool (nothing logged) */
  W *w = &g_w;
  uint8_t *raw = (uint8_t *)h_obj_new(SZ + CAN);
  memset(&g_w, 0, sizeof g_w);
  memset(raw, 0, SZ);
  memset(raw + SZ, 0x5a, CAN);
  w->d = (T *)raw; g_can = raw + SZ;
  new_args(0);
  w->ce = (EvObj *)obj_blk(0, sizeof(EvObj)); w->p3 = (Ph3DemObj *)obj_blk(0, sizeof(Ph3DemObj)); w->p4 = (Ph4DemObj *)obj_blk(0, sizeof(Ph4DemObj));
  w->dem = (DemObj *)obj_blk(0, sizeof(DemObj)); w->cd = (CdObj *)obj_blk(0, sizeof(CdObj)); w->eq = (EquObj *)obj_blk(0, sizeof(EquObj)); w->adid = (AdidObj *)obj_blk(0, sizeof(AdidObj));
  w->trn2d = obj_blk(0, 16);
  w->ucode = (unsigned char *)obj_blk(0, 32);
  w->diag = NULL; w->rbs = NULL;
  w->n = NMAX;
  w->d->info_ = (void *)w->info; w->d->jd_ = w->jd; w->d->jd92_ = w->jd92; w->d->dil_ = w->dil; w->d->mp1_ = w->mp1; w->d->mp2_ = w->mp2; w->d->trn2d_ = w->trn2d; w->d->cpInfo_ = w->cpi; w->d->cp_ = w->cp;
  w->d->mp_ = w->mp; w->d->params_ = w->par; w->d->eq_ = w->eq; w->d->p3_ = w->p3; w->d->p4_ = w->p4; w->d->dem_ = w->dem; w->d->cdes_ = w->cd; w->d->ce_ = w->ce; w->d->adid_ = w->adid;
  w->d->inBuf_ = obj_blk(0, NMAX * 4); w->d->rsBuf_ = obj_blk(0, NMAX * 12); w->d->decBuf_ = obj_blk(0, NMAX * 4); w->d->eqBuf_ = obj_blk(0, NMAX * 8); w->d->byteBuf_ = obj_blk(0, NMAX * 8);
}

/* the inputs of one call: every member the methods read is re-drawn */
static void set_inputs(void) {
  W *w = &g_w;
  T *d = w->d;
  V90ParObj *p = w->par;
  unsigned i;
  /* the demodulator's own scalars: random garbage in every non-pointer member, then the ones the methods look at */
  d->hwCodec_ = h_rpct(40) ? h_rmod(9) : h_rnd();
  d->session_ = h_rpct(45) ? 0u : (h_rpct(70) ? 1u : h_rnd());
  d->phase_ = h_rpct(60) ? h_rmod(6) : ((uint32_t[]){ 1u, 2u, 3u, 4u, 5u, 0x80000000u, 0xffffffffu, 7u })[h_rmod(8)];
  d->symCnt_ = h_rpct(60) ? h_rmod(300) : h_rnd();
  d->state_ = h_rnd();
  d->flag40_ = pick_flag();
  d->timeCnt_ = pick_cnt(); d->timeLimit_ = h_rnd();
  d->agc_mix_ = ffb(pick_fb()); d->agc_mixSaved_ = ffb(pick_fb()); d->agc_gain_ = ffb(pick_fb()); d->agc_target_ = ffb(pick_fb()); d->agc_power_ = ffb(pick_fb());
  if (h_rpct(30)) d->agc_power_ = ffb(0x3f800000u + h_rmod(1u << 22) * (h_rpct(50) ? 1u : 0u));
  d->agc_acc_ = ffb(pick_fb()); d->agc_period_ = h_rnd(); d->agc_count_ = h_rnd();
  d->rs_state_ = h_rnd(); d->rs_stateCount_ = h_rnd();
  d->pf_filterIdx_ = (int32_t)h_rnd();
  d->sv_flag_ = h_rpct(80) ? h_rmod(4) : h_rnd();
  d->retrainOk_ = pick_flag();
  d->framePos_ = h_rnd();
  d->cnt23_ = h_rpct(60) ? h_rmod(50) : h_rnd(); d->cnt1f_ = h_rpct(60) ? h_rmod(50) : h_rnd(); d->cnt26_ = h_rpct(60) ? h_rmod(50) : h_rnd();
  d->flag270_ = h_rnd(); d->w278_ = h_rnd();
  d->designErr_ = ffb(pick_fb()); d->dilScale_ = ffb(pick_fb());
  d->quietCnt_ = h_rnd();
  d->designed_ = (uint8_t)(h_rpct(50) ? 1u : h_rpct(50) ? 0u : h_rnd());
  d->errPrintCnt_ = h_rnd(); d->errPrintPeriod_ = h_rnd(); d->tmPrintCnt_ = h_rnd(); d->tmPrintPeriod_ = h_rnd();
  d->rsCnt_ = h_rnd(); d->eqCnt_ = h_rnd();
  /* the objects it points to */
  w->info->pcm_ = h_rpct(50) ? h_rmod(2) : h_rnd();
  w->info->rtd_ = h_rpct(70) ? h_rmod(2000) : h_rnd();
  w->info->uinfo_ = (uint8_t)(h_rpct(80) ? h_rmod(30) : h_rnd());
  w->info->maxTx_ = (uint8_t)(h_rpct(70) ? h_rmod(30) : h_rpct(30) ? 255u : h_rnd());
  w->mp2->w0_ = h_rpct(50) ? h_rmod(60) : h_rpct(50) ? h_rmod(1u << 20) : h_rnd();
  w->cpi->w0_ = h_rnd(); w->cpi->w4_ = h_rnd(); w->cpi->f8_ = ffb(h_rnd()); w->cpi->wc_ = h_rnd(); w->cpi->w10_ = h_rnd(); w->cpi->w14_ = h_rnd();
  w->ce->rrnSilence_ = (int32_t)pick_flag(); w->ce->delayedRetrainRequest_ = (int32_t)h_rnd(); w->ce->phase3EvalActive_ = (int32_t)h_rnd(); w->ce->trn1dEvalActive_ = (int32_t)h_rnd();
  w->ce->avePdsnr_ = ffb(h_rnd()); w->ce->weight_ = h_rnd();
  w->cd->state_ = h_rnd();
  w->p3->state_ = h_rnd(); w->p3->cnt_ = h_rnd(); w->p3->event_ = h_rpct(50) ? 0x14u : h_rpct(50) ? h_rmod(0x30) : h_rnd(); w->p3->lineMode_ = h_rnd();
  w->p4->state_ = h_rnd(); w->p4->w38_ = pick_flag(); w->p4->w3c_ = pick_flag(); w->p4->w40_ = h_rnd();
  w->dem->z1eb4_ = (uint16_t)h_rnd();
  w->eq->event_ = h_rnd(); w->eq->w8_ = (uint16_t)h_rnd(); w->eq->meanErr_ = ffb(pick_fb()); w->eq->flag148_ = h_rnd(); w->eq->linLen_ = h_rpct(50) ? h_rmod(30) : h_rnd();
  w->adid->pcmType_ = (int32_t)(h_rpct(60) ? h_rmod(2) : h_rnd()); w->adid->codecIsAlaw_ = (int32_t)(h_rpct(60) ? h_rmod(2) : h_rnd());
  w->adid->unsuspectedPhase_ = (uint16_t)(h_rpct(60) ? h_rmod(8) : h_rnd());
  for (i = 0; i < 6; i++) w->adid->suspectedPhase_[i] = (uint8_t)(h_rpct(85) ? h_rmod(2) : h_rnd());
  for (i = 0; i < 6; i++) w->adid->altRbsPhase_[i] = (uint16_t)h_rnd();
  for (i = 0; i < 32; i++) w->ucode[i] = (uint8_t)h_rnd();
  /* the parameter block */
  p->timingHistoryEvaluationEnabled = (int32_t)(h_rpct(40) ? 0u : h_rpct(70) ? 1u : h_rnd());
  PARF(timingOffesetMinStdForSave); PARF(initialBaudOffset); PARF(agcNominalEnergy);
  p->agcBlockLen = (int32_t)h_rnd();
  p->linearEquCursorPlace = (int32_t)(h_rpct(40) ? (uint32_t)(0u - h_rmod(10)) : h_rpct(70) ? h_rmod(40) : h_rnd());
  p->errorEnergyPrintPeriodPhase3 = (int32_t)h_rnd(); p->errorEnergyPrintPeriodPhase4 = (int32_t)h_rnd(); p->errorEnergyPrintPeriodData = (int32_t)h_rnd();
  p->timingOffsetPrintPeriodPhase3 = (int32_t)h_rnd(); p->timingOffsetPrintPeriodPhase4 = (int32_t)h_rnd(); p->timingOffsetPrintPeriodData = (int32_t)h_rnd();
  p->analogRateMask = (int32_t)(h_rpct(50) ? h_rnd() : h_rmod(0x20000));
  p->linearEquLength = (int32_t)h_rmod(30); p->dfeLength = (int32_t)h_rmod(30);
  w->mpar->unk01[1] = (uint8_t)(h_rpct(20) ? 0u : h_rpct(20) ? 0x80u : h_rpct(20) ? 0x7fu : h_rpct(20) ? 0xffu : h_rnd());
  for (i = 0; i < 4; i++) w->mpar->unk4c[i] = (uint8_t)h_rnd();
  /* the print thresholds: the min std for save at the std of the next statistics call is hit by the ext values (case 21 / 22) */
  if (h_rpct(30)) p->timingOffesetMinStdForSave = ffb(0x3f800000u + h_rmod(1u << 23));
}

/* the scripted results */
static void gen_script(void) {
  unsigned k, i;
  for (k = 0; k < K_N; k++) for (i = 0; i < NSC; i++) g_sc[k][i] = h_rnd();
  for (i = 0; i < NSC; i += 2) if (h_rpct(20)) g_sc[K_TM][i] = 29u * h_rmod(1u << 20) + 24u;   /* ext_val case 24: the abs of a fraction product >= 2^29 */
  memset(g_cur, 0, sizeof g_cur);
}

/* ------------------------------------------------------------------------------------------------ one method call */
enum { MT_SESS, MT_EP4, MT_STEADY, MT_RRN, MT_FPE, MT_SETSESS, MT_EP3, MT_BITRATE, MT_RBS, MT_INDIC, MT_ATUD, MT_EXIT3, MT_EDP, MT_REINIT, MT_RESET, MT_ECV, MT_N };
static const char *mt_name[MT_N] = { "sessionTermination", "enterPhase4", "enterDataSteadyState", "enterRRN", "enterFPE", "setSessionFlag", "enterPhase3", "getBitRate", "getRbsPattern",
                                     "indicateRemoteRateReneg", "getAT_UD", "exitPhase3", "enterDataPhase", "reInit", "reset", "enterChannelVerification" };
static unsigned g_arg;
static short g_sa, g_sb;
static unsigned long g_ncalls;
static int g_showall;
static int g_snan;   /* P44_SNAN=1: signalling NaNs in the float words the methods only MOVE (setTimingOffset argument, agc target, mean error word) or quiet (dilScale_) */
static void snan_poke(void) {
  W *w = &g_w;
  uint32_t v;
#define SN(f_) do { if (h_rpct(30)) { v = 0x7f800001u + h_rmod(1u << 22); if (h_rpct(30)) v |= 0x80000000u; memcpy(&(f_), &v, 4); } } while (0)
  SN(w->par->initialBaudOffset); SN(w->par->agcNominalEnergy); SN(w->eq->meanErr_); SN(w->d->dilScale_);
#undef SN
}

static int run_model(int mt) {
  W *w = &g_w;
  switch (mt) {
    case MT_SESS: return m_sessterm();
    case MT_EP4: m_ep4(); return 0;
    case MT_STEADY: m_steady(); return 0;
    case MT_RRN: m_rrn(); return 0;
    case MT_FPE: m_fpe(); return 0;
    case MT_SETSESS: m_setsess(g_arg); return 0;
    case MT_EP3: m_ep3(); return 0;
    case MT_BITRATE: return m_bitrate();
    case MT_RBS: m_rbs(w->rbs); return 0;
    case MT_INDIC: m_indic(); return 0;
    case MT_ATUD: m_atud(w->diag); return 0;
    case MT_EXIT3: m_exit3(); return 0;
    case MT_EDP: m_edp(); return 0;
    case MT_REINIT: m_reinit(); return 0;
    case MT_RESET: m_reset(g_arg); return 0;
    default: m_ecv(g_sa, g_sb); return 0;
  }
}
static int run_impl(int mt) {
  W *w = &g_w;
  T *d = w->d;
  switch (mt) {
    case MT_SESS: return f_sessterm(d);
    case MT_EP4: f_ep4(d); return 0;
    case MT_STEADY: f_steady(d); return 0;
    case MT_RRN: f_rrn(d); return 0;
    case MT_FPE: f_fpe(d); return 0;
    case MT_SETSESS: f_setsess(d, g_arg); return 0;
    case MT_EP3: f_ep3(d); return 0;
    case MT_BITRATE: return f_bitrate(d);
    case MT_RBS: f_rbs(d, w->rbs); return 0;
    case MT_INDIC: f_indic(d); return 0;
    case MT_ATUD: f_atud(d, w->diag); return 0;
    case MT_EXIT3: f_exit3(d); return 0;
    case MT_EDP: f_edp(d); return 0;
    case MT_REINIT: f_reinit(d); return 0;
    case MT_RESET: f_reset(d, g_arg); return 0;
    default: f_ecv(d, g_sa, g_sb); return 0;
  }
}

static void bias_for(int mt) {   /* steer the state so that both branches of the method are reached often */
  W *w = &g_w;
  T *d = w->d;
  switch (mt) {
    case MT_SESS: d->phase_ = h_rpct(60) ? 3u : d->phase_; break;
    case MT_EP4: case MT_FPE: case MT_RRN: d->phase_ = h_rpct(25) ? 2u : (h_rpct(70) ? 1u : d->phase_); break;
    case MT_STEADY: d->phase_ = h_rpct(25) ? 4u : (h_rpct(70) ? 3u : d->phase_); break;
    case MT_EP3: d->phase_ = h_rpct(25) ? 1u : (h_rpct(70) ? 0u : d->phase_); break;
    case MT_EDP: d->phase_ = h_rpct(25) ? 3u : (h_rpct(70) ? 2u : d->phase_); break;
    case MT_EXIT3: d->phase_ = h_rpct(85) ? 1u : d->phase_; break;
    default: break;
  }
  if (mt == MT_RRN && h_rpct(60)) { w->ce->rrnSilence_ = 1; w->p4->w3c_ = 1; w->p4->w38_ = h_rpct(50); }
  if (mt == MT_SESS || mt == MT_STEADY) {
    if (h_rpct(60)) w->par->timingHistoryEvaluationEnabled = 1;
    if (h_rpct(30)) w->par->timingOffesetMinStdForSave = ffb(0x3f800000u + (h_rpct(50) ? 0u : 0x300000u));
  }
  if (mt == MT_EXIT3 && h_rpct(50)) w->p3->event_ = 0x14;
  if (mt == MT_ECV) { g_sa = (short)h_rnd(); g_sb = (short)(h_rpct(30) ? 0 : h_rnd()); }
}

static void do_method(int mt) {
  W *w = &g_w;
  T *d = w->d;
  unsigned k;
  int rm, ri;
  set_inputs();
  bias_for(mt);
  if (g_snan) snan_poke();
  g_arg = h_rpct(50) ? h_rmod(2) : h_rnd();
  if (mt != MT_ECV) { g_sa = (short)h_rnd(); g_sb = (short)h_rnd(); }
  dsplibs_debug_level = h_rpct(50) ? 2u + h_rmod(2) : (h_rpct(60) ? 0u : (h_rpct(60) ? 1u : h_rnd()));
  gen_script();
  g_show = h_sample(&g_symctr, 12, 250) || g_showall;
  w->diag = NULL; w->rbs = NULL;
  if (mt == MT_ATUD) { w->diag = (DiagObj *)obj_blk(0, sizeof(DiagObj) + CAN); for (k = 0; k < sizeof(DiagObj); k++) ((uint8_t *)w->diag)[k] = (uint8_t)h_rnd(); memset((uint8_t *)w->diag + sizeof(DiagObj), 0x5a, CAN); }
  if (mt == MT_RBS) { w->rbs = (unsigned *)obj_blk(0, 24 + CAN); for (k = 0; k < 6; k++) w->rbs[k] = h_rnd(); memset((uint8_t *)w->rbs + 24, 0x5a, CAN); }
  build_regions();
  save_to(g_sv0);
  { unsigned lv0 = dsplibs_debug_level;
    g_log = &g_ml; g_ml.n = 0; memset(g_cur, 0, sizeof g_cur); cov_clear();
    rm = run_model(mt);
    save_to(g_sv1);
    restore_from(g_sv0);
    dsplibs_debug_level = lv0; }
  g_log = &g_il; g_il.n = 0; memset(g_cur, 0, sizeof g_cur);
  h_emit(g_show, "call %s ph=%u lvl=%u sess=%u ro=%u arg=%u", mt_name[mt], d->phase_, dsplibs_debug_level, d->session_, d->retrainOk_, g_arg);
  ri = run_impl(mt);
  __asm__ volatile("" ::: "memory");
  if (mt == MT_SESS || mt == MT_BITRATE) { if (rm != ri) h_die3("%s: model returned %d, impl %d", mt_name[mt], rm, ri); }
  log_cmp(); world_cmp();
  cov_flush();
  for (k = 0; k < g_il.n && k < LOGMAX; k++) h_emit(g_show, "%s", g_il.r[k].t);
  if (mt == MT_SESS || mt == MT_BITRATE) h_emit(g_show, "ret %d", ri);
  h_dump_obj(g_show, "D", d, g_fd, g_nfd);
  h_emit(g_show, "foreign %08x", snap_hash(w));
  if (w->diag) { for (k = 0; k < CAN; k++) if (((uint8_t *)w->diag)[sizeof(DiagObj) + k] != 0x5a) h_die3("canary behind the diagnostics record overwritten"); h_emit(g_show, "diag %08x", fnv_add(2166136261u, w->diag, sizeof(DiagObj))); }
  if (w->rbs) { for (k = 0; k < CAN; k++) if (((uint8_t *)w->rbs)[24 + k] != 0x5a) h_die3("canary behind the RBS output overwritten"); h_emit(g_show, "rbs %08x", fnv_add(2166136261u, w->rbs, 24)); }
  h_wild = 0;
  for (k = 0; k < CAN; k++) if (g_can[k] != 0x5a) h_die3("canary byte %u behind the object was overwritten", k);
  w->diag = NULL; w->rbs = NULL;
  g_ncalls++;
}

/* ------------------------------------------------------------------------------------------------ destructor */
static void build_regions_min(void) { W *w = &g_w; g_nreg = 0; add_reg(w->d, SZ); add_reg(w->par, sizeof(V90ParObj)); add_reg(w->mpar, sizeof(ModParObj)); }
typedef struct { void **pp; unsigned cls; const char *nm; unsigned size; } HeapMember;
/* the destructor's owner pointers in the blob's order (the objects with a destructor), then the five sample buffers */
static void heap_members(T *d, HeapMember *h, unsigned n) {
  HeapMember t[13] = {
    { &d->eq_, CL_EQ, "V90Equalizer", sizeof(EquObj) }, { &d->p3_, CL_P3, "V90Phase3Demodulator", sizeof(Ph3DemObj) }, { &d->p4_, CL_P4, "V90Phase4Demodulator", sizeof(Ph4DemObj) },
    { &d->dem_, CL_DEM, "V90Demapper", sizeof(DemObj) }, { &d->trn2d_, CL_TRN, "V90TRN2Designer", sizeof(TrnObj) }, { &d->cdes_, CL_CD, "V90ConstellationDesigner", sizeof(CdObj) },
    { &d->adid_, CL_ADID, "V90AutoDigitalImpDetector", sizeof(AdidObj) }, { &d->ce_, CL_CE, "V90ConnectionEvaluator", sizeof(EvObj) },
    { &d->inBuf_, 0, 0, 4 * 1 }, { &d->rsBuf_, 0, 0, 12 }, { &d->decBuf_, 0, 0, 4 }, { &d->eqBuf_, 0, 0, 8 }, { &d->byteBuf_, 0, 0, 8 } };
  unsigned i;
  for (i = 0; i < 13; i++) { h[i] = t[i]; if (i >= 8) h[i].size *= n; }
}
/* the expected records of the destructor: the model's sessionTermination, then the sub-destructors in the blob's order (no frees: the allocator log has them) */
static void exp_dtor_log(void) {
  T *d = g_w.d;
  HeapMember hm[13];
  unsigned i;
  heap_members(d, hm, 1);
  m_sessterm();
  for (i = 0; i < 8; i++) if (*hm[i].pp) { env_x(hm[i].nm, *hm[i].pp, 0, 0); CV(G_S04, "dtor: object block"); } else CV(G_S0, "dtor: NULL pointer");
  env_x("V90SpectralVerifier", &d->sv_params_, 0, 0);
  env_x("Descrambler", &d->scr_buf_, 0, 0);
  env_x("V90ConstellationPower", &d->cpow_tab_, 0, 0);
  env_x("V90Resampler", &d->rs_vptr_, 0, 0);
  env_x("V90PreFilter", &d->pf_fir_coef_, 0, 0);
}
static void run_dtor(void) {
  W *w = &g_w;
  T *d = w->d;
  HeapMember hm[13];
  void *before[13];
  unsigned i, k;
  heap_members(d, hm, 1);
  for (i = 0; i < 13; i++) before[i] = *hm[i].pp;
  g_foreign = 0; g_blocks = 0; g_nopoke = 1;
  dsplibs_debug_level = h_rpct(60) ? 0u : 2u + h_rmod(2);
  gen_script();
  g_show = h_sample(&g_symctr, 12, 250) || g_showall;
  build_regions_min();
  save_to(g_sv0);
  { unsigned lv0 = dsplibs_debug_level;
    g_log = &g_ml; g_ml.n = 0; memset(g_cur, 0, sizeof g_cur); cov_clear();
    exp_dtor_log();
    save_to(g_sv1);
    restore_from(g_sv0);
    dsplibs_debug_level = lv0; }
  g_log = &g_il; g_il.n = 0; memset(g_cur, 0, sizeof g_cur);
  h_emit(g_show, "dtor lvl=%u ph=%u", dsplibs_debug_level, d->phase_);
  f_dtor(d);
  __asm__ volatile("" ::: "memory");
  log_cmp(); world_cmp();
  for (k = 0; k < g_il.n && k < LOGMAX; k++) h_emit(g_show, "%s", g_il.r[k].t);
  for (i = 0; i < 13; i++) {
    if (*hm[i].pp != before[i]) h_die3("the destructor changed the pointer member %u", i);
    if (before[i]) { HAl *a = h_find_al(before[i], 1); if (!a || !a->freed) h_die3("the block of heap member %u was not freed", i); }
    else CV(G_S0, "dtor: NULL block pointer left alone");
  }
  cov_flush();
  h_dump_obj(g_show, "D", d, g_fdp, g_nfdp);          /* the object after the destructor (-flifetime-dse: a store the compiler would delete shows here) */
  for (k = 0; k < CAN; k++) if (g_can[k] != 0x5a) h_die3("canary byte %u behind the object was overwritten", k);
  g_foreign = 1; g_blocks = 1; g_nopoke = 0;
  g_ncalls++;
}
static void new_dtor_world(void) {
  W *w;
  T *d;
  HeapMember hm[13];
  unsigned i, n = pick_n();
  new_world();
  w = &g_w; d = w->d;
  fill_flds(d, f_DmdObj, NF_DmdObj);
  heap_members(d, hm, n);
  for (i = 0; i < 13; i++) {
    if (i != 1 && h_rpct(25)) { *hm[i].pp = NULL; continue; }
    *hm[i].pp = sysdep_malloc(hm[i].size);
    memset(*hm[i].pp, 0, hm[i].size);
    if (hm[i].nm) stamp_put(*hm[i].pp, hm[i].cls);
  }
  w->p3 = (Ph3DemObj *)d->p3_;       /* sessionTermination calls clearVerificationStatus on it */
  d->params_ = w->par;
  set_inputs();                       /* (writes the p3 block's words: the stamp at +0 is not touched) */
}

/* ------------------------------------------------------------------------------------------------ constructor */
static void zero_ptrs(T *d) {
  int i;
  for (i = 0; i < NF_DmdObj; i++) { char k = f_DmdObj[i].kind; if (k == 'p' || k == 'o' || k == 'e' || k == 'v') memset((uint8_t *)d + f_DmdObj[i].off, 0, sizeof(void *)); }
}
/* the expected records of the constructor (shadow object for the snapshots: the scalar stores happen after the sub-objects and before the first malloc) */
static void exp_ctor_log(const T *init, unsigned n, unsigned codec, unsigned mode, unsigned session) {
  W *w = &g_w;
  T *d = w->d;
  T sh;
  char sp[2][48];
  memcpy(&sh, init, SZ);
  g_snap_d = &sh;
  env_k("Agc", &d->agc_mix_, 0, 0, "");
  env_k("V90PreFilter", &d->pf_fir_coef_, 0, 0, "%u,%s,%s", codec, idp(w->info, sp[0]), idp(w->par, sp[1]));
  env_k("V90Resampler", &d->rs_vptr_, 0, 0, "%u,%08x,%u,%08x,%s,%08x,%u", 100u, 0x42700000u, 16u, 0x3f7ae148u, idp(w->par, sp[0]), 0u, 0u);
  env_k("V90ConstellationPower", &d->cpow_tab_, 0, 0, "");
  env_k("Descrambler", &d->scr_buf_, 0, 0, "%u,%u,%u", 0x12u, 0x17u, 0x63u);
  env_k("V90SpectralVerifier", &d->sv_params_, 0, 0, "%s", idp(w->par, sp[0]));
  sh.hwCodec_ = codec; sh.session_ = session; sh.rsCnt_ = 0; sh.eqCnt_ = 0;
  { char b[10][48]; unsigned ll = (unsigned)w->par->linearEquLength, dl = (unsigned)w->par->dfeLength;
    env_k("V90AutoDigitalImpDetector", d->adid_, 0, 0, "%s", idp(w->par, b[0]));
    env_k("V90ConnectionEvaluator", d->ce_, 0, 0, "%s", idp(w->par, b[0]));
    env_k("V90Phase3Demodulator", d->p3_, 0, 0, "%s,%s,%u,%s", idp(w->par, b[0]), idp(&d->sv_params_, b[1]), session, idp(d->adid_, b[2]));
    env_k("V90Demapper", d->dem_, 0, 0, "%u,%s,%s", n + n, idp(w->par, b[0]), idp(d->adid_, b[1]));
    env_k("V90Phase4Demodulator", d->p4_, 0, 0, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%u", idp(w->mp1, b[0]), idp(w->mp2, b[1]), idp(d->dem_, b[2]), idp(w->cp, b[3]), idp(w->mp, b[4]), idp(&d->scr_buf_, b[5]),
          idp(d->ce_, b[6]), idp(w->par, b[7]), idp(d->p3_, b[8]), idp(d->adid_, b[9]), session);
    env_k("V90Equalizer", d->eq_, 0, 0, "%u,%u,%s,%s,%s,%s,%s,%s,%s,%s,%u", ll, dl, idp(d->p3_, b[0]), idp(d->p4_, b[1]), idp(d->dem_, b[2]), idp(d->ce_, b[3]), idp(&d->sv_params_, b[4]), idp(w->par, b[5]),
          idp(&d->rs_vptr_, b[6]), idp(&d->pf_fir_coef_, b[7]), mode);
    env_k("V90TRN2Designer", d->trn2d_, 0, 0, "%s,%s", idp(w->par, b[0]), idp(&d->cpow_tab_, b[1]));
    env_k("V90ConstellationDesigner", d->cdes_, 0, 0, "%s,%s,%s", idp(w->par, b[0]), idp(&d->pf_fir_coef_, b[1]), idp(&d->cpow_tab_, b[2])); }
  g_snap_d = NULL;
  sh.cnt23_ = 0; sh.cnt26_ = 0; sh.cnt1f_ = 0; sh.w278_ = 0; sh.designed_ = 0; sh.quietCnt_ = 0;
  { int i;     /* the final state: every non-pointer member equals the shadow */
    for (i = 0; i < g_nfd; i++) {
      size_t sz = 0;
      switch (g_fd[i].kind) { case 'b': sz = 1; break; case 'h': sz = 2; break; case 'u': case 'i': case 'f': sz = 4; break; case 'd': sz = 8; break; case 'x': sz = g_fd[i].count; break; default: break; }
      if (sz) { size_t tot = sz * (g_fd[i].kind != 'x' && g_fd[i].count > 1 ? g_fd[i].count : 1u); if (memcmp((uint8_t *)d + g_fd[i].off, (uint8_t *)&sh + g_fd[i].off, tot)) h_die3("constructor: member %s differs from the model", g_fd[i].name); }
    } }
}
static void run_ctor(void) {
  W *w = &g_w;
  T *d = w->d, init;
  unsigned n = pick_n(), codec = h_rpct(60) ? h_rmod(9) : h_rnd(), mode = h_rpct(70) ? h_rmod(3) : h_rnd(), session = h_rpct(50) ? 0u : (h_rpct(70) ? 1u : h_rnd());
  HeapMember hm[13];
  HAl *a;
  unsigned i, k;
  uint32_t lastid = 0;
  w->par->linearEquLength = (int32_t)h_rmod(40); w->par->dfeLength = (int32_t)h_rmod(40);
  fill_flds(d, f_DmdObj, NF_DmdObj);
  zero_ptrs(d);
  memcpy(&init, d, SZ);
  g_foreign = 0; g_blocks = 0; g_nopoke = 1;
  gen_script();
  g_show = h_sample(&g_symctr, 12, 250) || g_showall;
  g_log = &g_il; g_il.n = 0; memset(g_cur, 0, sizeof g_cur); cov_clear();
  h_emit(g_show, "ctor n=%u codec=%u mode=%u session=%u", n, codec, mode, session);
  H_CTOR_PROBE(d, f_DmdObj, NF_DmdObj, SZ, f_ctor(d, n, w->info, w->jd, w->jd92, w->dil, w->mp1, w->mp2, w->cpi, w->cp, w->mp, codec, w->par, mode, session));
  __asm__ volatile("" ::: "memory");
  /* the owned blocks: sizes, allocation order, stamps */
  { T tmp; (void)tmp; }
  {
    void **order[13];
    unsigned long sizes[13];
    const char *nm[13] = { "inBuf_", "rsBuf_", "decBuf_", "eqBuf_", "byteBuf_", "adid_", "ce_", "p3_", "dem_", "p4_", "eq_", "trn2d_", "cdes_" };
    heap_members(d, hm, n);   /* hm: eq p3 p4 dem trn2d cdes adid ce bufs */
    order[0] = &d->inBuf_; order[1] = &d->rsBuf_; order[2] = &d->decBuf_; order[3] = &d->eqBuf_; order[4] = &d->byteBuf_;
    order[5] = &d->adid_; order[6] = &d->ce_; order[7] = &d->p3_; order[8] = &d->dem_; order[9] = &d->p4_; order[10] = &d->eq_; order[11] = &d->trn2d_; order[12] = &d->cdes_;
    sizes[0] = 4ul * n; sizes[1] = 12ul * n; sizes[2] = 4ul * n; sizes[3] = 8ul * n; sizes[4] = 8ul * n;
    sizes[5] = sizeof(AdidObj); sizes[6] = sizeof(EvObj); sizes[7] = sizeof(Ph3DemObj); sizes[8] = sizeof(DemObj); sizes[9] = sizeof(Ph4DemObj); sizes[10] = sizeof(EquObj); sizes[11] = sizeof(TrnObj); sizes[12] = sizeof(CdObj);
    for (i = 0; i < 13; i++) {
      a = h_find_al(*order[i], 1);
      if (!a || a->p != (uint8_t *)*order[i]) h_die3("constructor: member %s is not an allocated block", nm[i]);
      if (a->size != sizes[i]) h_die3("constructor: block %s has %u bytes, expected %lu", nm[i], a->size, sizes[i]);
      if (a->id <= lastid && i) h_die3("constructor: block %s allocated out of order", nm[i]);
      lastid = a->id;
      if (i >= 5) { unsigned cls[8] = { CL_ADID, CL_CE, CL_P3, CL_DEM, CL_P4, CL_EQ, CL_TRN, CL_CD }; stamp_chk(*order[i], cls[i - 5]); }
    }
  }
  /* the arguments */
  if (d->params_ != w->par || d->info_ != w->info || d->jd_ != w->jd || d->jd92_ != w->jd92 || d->dil_ != w->dil || d->mp1_ != w->mp1 || d->mp2_ != w->mp2 || d->cpInfo_ != w->cpi || d->cp_ != w->cp || d->mp_ != w->mp)
    h_die3("constructor: an argument pointer was stored wrongly");
  g_log = &g_ml; g_ml.n = 0;
  exp_ctor_log(&init, n, codec, mode, session);
  log_cmp();
  cov_flush();
  if (n == 0) CV(G_S1, "ctor: n = 0"); else CV(G_S1, "ctor: n > 0"); if (session) CV(G_S1, "ctor: session != 0"); else CV(G_S1, "ctor: session 0"); CV(G_S1, "ctor: record list checked");
  cov_flush();
  g_log = &g_il;
  for (k = 0; k < g_il.n && k < LOGMAX; k++) h_emit(g_show, "%s", g_il.r[k].t);
  h_dump_obj(g_show, "D", d, g_fdp, g_nfdp);
  for (k = 0; k < CAN; k++) if (g_can[k] != 0x5a) h_die3("canary byte %u behind the object was overwritten", k);
  g_foreign = 1; g_blocks = 1; g_nopoke = 0;
  g_ncalls++;
}
static void new_ctor_world(void) {
  W *w;
  uint8_t *raw = (uint8_t *)h_obj_new(SZ + CAN);
  memset(&g_w, 0, sizeof g_w);
  w = &g_w;
  memset(raw + SZ, 0x5a, CAN);
  w->d = (T *)raw; g_can = raw + SZ;
  new_args(1);
  w->ucode = (unsigned char *)obj_blk(0, 32);
  w->n = NMAX;
}
static void free_ctor_args(void) {
  W *w = &g_w;
  void *a[] = { w->par, w->mpar, w->mp1, w->mp2, w->cpi, w->info, w->mp, w->jd, w->jd92, w->dil, w->cp };
  unsigned i;
  for (i = 0; i < sizeof a / sizeof a[0]; i++) sysdep_free(a[i]);
}
static void free_heap_members(void) {
  HeapMember hm[13];
  unsigned i;
  heap_members(g_w.d, hm, 1);
  for (i = 0; i < 13; i++) if (*hm[i].pp) { sysdep_free(*hm[i].pp); *hm[i].pp = NULL; }
}

/* the lifecycle: the heap objects that the constructor built become the world of the methods */
static void life_init(void) {
  W *w = &g_w;
  T *d = w->d;
  w->ce = (EvObj *)d->ce_; w->p3 = (Ph3DemObj *)d->p3_; w->adid = (AdidObj *)d->adid_; w->dem = (DemObj *)d->dem_; w->p4 = (Ph4DemObj *)d->p4_; w->eq = (EquObj *)d->eq_; w->cd = (CdObj *)d->cdes_;
  w->trn2d = d->trn2d_;
  memset(w->ce, 0, sizeof(EvObj)); memset(w->p3, 0, sizeof(Ph3DemObj)); memset(w->adid, 0, sizeof(AdidObj)); memset(w->dem, 0, sizeof(DemObj)); memset(w->p4, 0, sizeof(Ph4DemObj));
  memset(w->eq, 0, sizeof(EquObj)); memset(w->cd, 0, sizeof(CdObj));
  stamp_put(w->adid, CL_ADID); stamp_put(w->ce, CL_CE); stamp_put(w->p3, CL_P3); stamp_put(w->dem, CL_DEM); stamp_put(w->p4, CL_P4); stamp_put(w->eq, CL_EQ); stamp_put(w->trn2d, CL_TRN); stamp_put(w->cd, CL_CD);
}

/* ------------------------------------------------------------------------------------------------ trials */
static void trial_methods(int sc) {
  static const int s2[] = { MT_EP4, MT_STEADY, MT_RRN, MT_FPE, MT_SETSESS, MT_EP3, MT_BITRATE, MT_RBS, MT_INDIC, MT_ATUD, MT_EDP, MT_REINIT };
  static const int s3[] = { MT_EXIT3, MT_RESET, MT_ECV };
  unsigned nc = rnd_range(1, 3), k;
  new_world();
  for (k = 0; k < nc; k++) {
    int mt = sc == 0 ? MT_SESS : sc == 2 ? s2[h_rmod(sizeof s2 / sizeof s2[0])] : sc == 3 ? s3[h_rmod(3)] : (int)h_rmod(MT_N);
    do_method(mt);
  }
  h_trial_end("p44");
}
static void trial_dtor(void) {
  new_dtor_world();
  run_dtor();
  h_trial_end("p44");
}
static void trial_ctor(void) {
  new_ctor_world();
  run_ctor();
  free_heap_members();
  free_ctor_args();
  h_trial_end("p44");
}
static void trial_life(void) {
  unsigned nc = rnd_range(1, 4), k;
  new_ctor_world();
  run_ctor();
  life_init();
  g_w.diag = NULL; g_w.rbs = NULL;
  for (k = 0; k < nc; k++) { set_inputs(); do_method((int)h_rmod(MT_N)); }
  CV(G_S4, "life: methods on the built object"); cov_flush();
  /* the destructor: the object is whatever the methods left */
  g_w.d->phase_ = h_rpct(50) ? 3u : g_w.d->phase_;
  run_dtor();
  free_ctor_args();
  h_trial_end("p44");
}

static void alarm_handler(int sig) { (void)sig; static const char m[] = "harness: watchdog (the blob or the port did not terminate)\n"; if (write(2, m, sizeof m - 1) < 0) {} _exit(3); }

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 0, tr, ntr, i;
  const char *seed = getenv("P44_SEED");
  if (seed) h_seed_xor((unsigned int)strtoul(seed, NULL, 0));
  if (getenv("P44_ALL")) g_showall = 1;
  if (getenv("P44_SNAN")) g_snan = 1;
  p07_layout_check(); p09_layout_check(); p11_layout_check(); p17_layout_check(); p18_layout_check(); p24_layout_check(); p26_layout_check(); p28_layout_check(); p34_layout_check(); p37_layout_check(); p40_layout_check();
  p43_layout_check(); p44_layout_check();
  for (i = 0; i < NF_DmdObj; i++) {
    const char *nm = f_DmdObj[i].name;
    char k = f_DmdObj[i].kind;
    int keep = !(k == 'p' || k == 'o' || k == 'e' || k == 'v');
    if (k == 'o' && (!strcmp(nm, "inBuf_") || !strcmp(nm, "rsBuf_") || !strcmp(nm, "decBuf_") || !strcmp(nm, "eqBuf_") || !strcmp(nm, "byteBuf_"))) keep = 1;
    if (keep) g_fd[g_nfd++] = f_DmdObj[i];
    {
      static const char *pn[] = { "info_", "jd_", "jd92_", "dil_", "mp1_", "mp2_", "trn2d_", "cpInfo_", "cp_", "mp_", "params_", "eq_", "p3_", "p4_", "dem_", "cdes_", "ce_", "adid_", "inBuf_", "rsBuf_", "decBuf_", "eqBuf_", "byteBuf_" };
      unsigned j;
      int isp = 0;
      for (j = 0; j < sizeof pn / sizeof pn[0]; j++) if (!strcmp(nm, pn[j])) isp = 1;
      if (isp) { g_fdp[g_nfdp] = f_DmdObj[i]; g_fdp[g_nfdp++].kind = 'p'; }
      else if (!(k == 'p' || k == 'o' || k == 'e' || k == 'v')) g_fdp[g_nfdp++] = f_DmdObj[i];
    }
  }
  h_x87_check();
  h_arena_init();
  h_rz_by_id = 1;
#ifndef __i386__
  /* the 64-bit sizeof of the eight heap classes is logged as the i386 size (the golden is the 32-bit blob's) */
  h_log_map_from = 0; h_log_map_to = 0;
  h_log_map_add((unsigned)sizeof(AdidObj), 0xa9b0u); h_log_map_add((unsigned)sizeof(EvObj), 0xbcu); h_log_map_add((unsigned)sizeof(Ph3DemObj), 0x42cu); h_log_map_add((unsigned)sizeof(DemObj), 0x1eb8u);
  h_log_map_add((unsigned)sizeof(Ph4DemObj), 0x351cu); h_log_map_add((unsigned)sizeof(EquObj), 0x150u); h_log_map_add((unsigned)sizeof(TrnObj), 8u); h_log_map_add((unsigned)sizeof(CdObj), 0x54u);
#endif
  signal(SIGALRM, alarm_handler);
  alarm(280);
  h_scenario_begin(1);
  h_covreq[H_COV_STRADDLE] = 0; h_covreq[H_COV_DTOR_NULL] = 0;
  ntr = getenv("P44_TRIALS") ? atoi(getenv("P44_TRIALS")) : 4000;
  if (sc < 0 || sc > 4) { fprintf(stderr, "unknown scenario %d\n", sc); return 2; }
  for (tr = 0; tr < ntr; tr++) {
    if (sc == 0) { if (h_rpct(50)) trial_methods(0); else trial_dtor(); }
    else if (sc == 1) trial_ctor();
    else if (sc == 4) { if (h_rpct(75)) trial_life(); else trial_methods(4); }
    else trial_methods(sc);
  }
  h_emit(1, "calls: malloc %lu free %lu calls %lu trials %d", h_nmalloc, h_nfree, g_ncalls, ntr);
  { CovSite *c; int bad = 0;
    for (c = __start_covsites; c < __stop_covsites; c++) {
      if (!(c->grp & (1u << sc))) continue;
      h_emit(1, "COV %s %lu", c->name, c->n);
      if (c->n == 0) { fprintf(stderr, "COVERAGE FAIL: site \"%s\" was never reached\n", c->name); bad = 1; }
    }
    h_cov_finish();
    return (bad && !getenv("P44_NOCOV")) ? 2 : 0; }
}
