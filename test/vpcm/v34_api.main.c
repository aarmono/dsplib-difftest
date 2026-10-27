/*
 * Differential test harness for V.34 "tier 10a" (src/vpcm/v34_api.c): the 39 VPcm* / V34XF_* wrapper and accessor
 * functions (see the port header for the list).
 *
 * One source, two binaries (convention of test/vpcm/v34_handshake_init.main.c):
 *   v34_tier10a_test_blob - linked against the real dsplibs.o (-DTEST_BLOB) in which the CALLEES the wrappers use are
 *                           WEAK (objcopy --weaken-symbol: v34handshakinit, preinitdigital, v34setuptxmit, edprintf)
 *   v34_tier10a_test_port - linked against src/vpcm/v34_api.c + src/v34/v34_api_tables.c only
 * Each scenario runs in its OWN process (argv[1]).  The four callees are RECORDERS defined here in BOTH binaries (the
 * strong definitions win for every reference): a recorder logs its arguments (pointers normalised), the formatted text
 * (edprintf), the hash of the state DIFF at entry (so the store-vs-call ORDER of every field is compared), and then
 * SCRIBBLES deterministic values over the fields the caller reads after the call (worst-case callee).  Their real
 * implementations are verified separately (tiers 3ab/3c/3d/9a).
 *
 * Function names are reached through __asm__ labels: five blob functions are C++-mangled free functions
 * (_Z20resetRetrainDetectorP17tag_retrainReqDets, _Z19interpretMohTimeouts, _Z25SetUpstreamModulationInfoP12tagV34Object,
 * _Z25VPcmV34SetMinimumSigLevelP12tagV34Object, _Z17VPcmV34SetTimeOutP12tagV34Objecti); the port uses plain names.
 * Small integer arguments are passed as full `int`s with GARBAGE in the upper 16 / 24 bits (the blob reads movswl / movzbl).
 *
 * State: a random fake object (OBJSZ 0xb400 B) plus an ARENA of fake sub-objects (ctx, cfg, P, the V.90 engine chain nodes,
 * out buffers) are re-filled with random bytes before every call; the pointer slots between them are then set to valid
 * arena addresses.  A per-call snapshot is taken; the call's effect is hashed as the DIFF (word index + new value) of
 * object + arena, so a stray write anywhere is caught and raw pointers are never compared (slots are unchanged by every
 * function under test: asserted, exit 3).  Returned pointers are normalised to (ret - obj).
 *
 * INDEPENDENT MODELS (written from the disassembly facts, not from the port source): VPcmV34GetCurrentTxBitRate (exact
 * integer arithmetic instead of x87: trunc(v*m*2^-k + 0.5) = (v*m + 2^(k-1)) >> k) and VPcmV34InterpretMohMessageBits
 * (table-driven; stores and the ordered list of printed lines are compared after every call, exit 3 on a mismatch).
 *
 * Scenarios: 0 simple setters / indicators / pointer returns, 1 interpretMohTimeout + SetMinimumSigLevel (table),
 * 2 state getters + GetSNR, 3 GetCurrentTxBitRate (x87), 4 edprintf group (NotifyDP, Get(Max)UpstreamRateIndex x2,
 * RequestDPNotification), 5 InterpretMohMessageBits, 6 Indicate* / SetV90RateReneg / K56Flex*, 7 Initiate* (v34handshakinit recorder).
 * Every scenario keeps COVERAGE COUNTERS derived from the INPUTS and exits 2 if a required class was never reached.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../v34/v34_img.h" /* T06: x86-64 image <-> native marshalling (V34Obj, ctx, cfg, engine chain nodes); empty at -m32 */

extern unsigned int dsplibs_debug_level;

/* ---- functions under test (asm labels: blob = mangled where needed, port = plain) ---- */
#ifdef TEST_BLOB
#define LBL(plain, mangled) __asm__(mangled)
#else
#define LBL(plain, mangled) __asm__(plain)
#endif
extern void T_resetRetrainDetector(void *, int) LBL("resetRetrainDetector", "_Z20resetRetrainDetectorP17tag_retrainReqDets");
extern int T_interpretMohTimeout(int) LBL("interpretMohTimeout", "_Z19interpretMohTimeouts");
extern void T_SetUpstreamModulationInfo(void *) LBL("SetUpstreamModulationInfo", "_Z25SetUpstreamModulationInfoP12tagV34Object");
extern void T_VPcmV34SetMinimumSigLevel(void *) LBL("VPcmV34SetMinimumSigLevel", "_Z25VPcmV34SetMinimumSigLevelP12tagV34Object");
extern void T_VPcmV34SetTimeOut(void *, int) LBL("VPcmV34SetTimeOut", "_Z17VPcmV34SetTimeOutP12tagV34Objecti");
extern int T_VPcmV34Delete(void) LBL("VPcmV34Delete", "VPcmV34Delete");
extern void T_VPcmV34SetMaxBlockLength(void *, int) LBL("VPcmV34SetMaxBlockLength", "VPcmV34SetMaxBlockLength");
extern void T_VPcmV34InitiateRateRenegotiation(void *, int) LBL("VPcmV34InitiateRateRenegotiation", "VPcmV34InitiateRateRenegotiation");
extern void T_VPcmV34InitiateHangUp(void *) LBL("VPcmV34InitiateHangUp", "VPcmV34InitiateHangUp");
extern int T_VPcmV34GetCurrentSessionDP(void *) LBL("VPcmV34GetCurrentSessionDP", "VPcmV34GetCurrentSessionDP");
extern int T_VPcmV34GetQuickConnectIndication(void *) LBL("VPcmV34GetQuickConnectIndication", "VPcmV34GetQuickConnectIndication");
extern int T_VPcmV34GetCurrentTxBitRate(void *) LBL("VPcmV34GetCurrentTxBitRate", "VPcmV34GetCurrentTxBitRate");
extern int T_VPcmV34GetCurrentRxBaudRate(void *) LBL("VPcmV34GetCurrentRxBaudRate", "VPcmV34GetCurrentRxBaudRate");
extern int T_VPcmV34GetCurrentTxBaudRate(void *) LBL("VPcmV34GetCurrentTxBaudRate", "VPcmV34GetCurrentTxBaudRate");
extern int T_VPcmV34GetCurrentRxCarrier(void *) LBL("VPcmV34GetCurrentRxCarrier", "VPcmV34GetCurrentRxCarrier");
extern int T_VPcmV34GetCurrentTxCarrier(void *) LBL("VPcmV34GetCurrentTxCarrier", "VPcmV34GetCurrentTxCarrier");
extern int T_VPcmV34GetSNR(void *) LBL("VPcmV34GetSNR", "VPcmV34GetSNR");
extern int T_getTimingOffset(void *) LBL("getTimingOffset", "getTimingOffset");
extern int T_getTimingPhase(void *) LBL("getTimingPhase", "getTimingPhase");
extern void *T_VPcmV34GetCleanedSamples(void *, int *) LBL("VPcmV34GetCleanedSamples", "VPcmV34GetCleanedSamples");
extern void T_VPcmV34NotifyDP(void *, int) LBL("VPcmV34NotifyDP", "VPcmV34NotifyDP");
extern int T_VPcmV34RequestDPNotification(void *, int *, int *, int *) LBL("VPcmV34RequestDPNotification", "VPcmV34RequestDPNotification");
extern int T_VPcmV34GetMaxUpstreamRateIndex(void *) LBL("VPcmV34GetMaxUpstreamRateIndex", "VPcmV34GetMaxUpstreamRateIndex");
extern int T_V34XF_GetMaxUpstreamRateIndex(void *) LBL("V34XF_GetMaxUpstreamRateIndex", "V34XF_GetMaxUpstreamRateIndex");
extern void T_VPcmV34ReportStartOfEchoAdapt(void) LBL("VPcmV34ReportStartOfEchoAdapt", "VPcmV34ReportStartOfEchoAdapt");
extern void T_VPcmV34ReportMiddleOfEchoAdapt(void) LBL("VPcmV34ReportMiddleOfEchoAdapt", "VPcmV34ReportMiddleOfEchoAdapt");
extern void *T_V34XF_GetInfo0BitsPtr(void *) LBL("V34XF_GetInfo0BitsPtr", "V34XF_GetInfo0BitsPtr");
extern void *T_V34XF_GetProbeResultsPtr(void *) LBL("V34XF_GetProbeResultsPtr", "V34XF_GetProbeResultsPtr");
extern int T_V34XF_GetRTD(void *) LBL("V34XF_GetRTD", "V34XF_GetRTD");
extern void T_VPcmV34InterpretMohMessageBits(void *, const uint16_t *) LBL("VPcmV34InterpretMohMessageBits", "VPcmV34InterpretMohMessageBits");
extern void T_VPcmV34SetV90RateReneg(void *, int, int) LBL("VPcmV34SetV90RateReneg", "VPcmV34SetV90RateReneg");
extern void T_V34XF_IndicateJdReceived(void *, int, int) LBL("V34XF_IndicateJdReceived", "V34XF_IndicateJdReceived");
extern void T_V34XF_IndicateDilReceived(void *, int) LBL("V34XF_IndicateDilReceived", "V34XF_IndicateDilReceived");
extern void T_V34XF_IndicateTrn2dReceived(void *) LBL("V34XF_IndicateTrn2dReceived", "V34XF_IndicateTrn2dReceived");
extern void T_V34XF_IndicateK56FlexJdReceived(void *, int) LBL("V34XF_IndicateK56FlexJdReceived", "V34XF_IndicateK56FlexJdReceived");
extern void T_V34XF_IndicateK56FlexRateDetermined(void *) LBL("V34XF_IndicateK56FlexRateDetermined", "V34XF_IndicateK56FlexRateDetermined");
extern void T_VPcmV34IndicateLocalRRN(void *) LBL("VPcmV34IndicateLocalRRN", "VPcmV34IndicateLocalRRN");
extern void T_VPcmV34IndicateRemoteRRN(void *) LBL("VPcmV34IndicateRemoteRRN", "VPcmV34IndicateRemoteRRN");
extern void T_VPcmV34SetIndicationOfRemoteRetrain(void *) LBL("VPcmV34SetIndicationOfRemoteRetrain", "VPcmV34SetIndicationOfRemoteRetrain");
/* V34SHIMS BEGIN (generated by tools/tier_rewrite.py shims; do not edit) */
#if defined(__x86_64__)
static void V34SH_T_resetRetrainDetector(void *a0, int a1) { v34i_enter(); T_resetRetrainDetector((void *)v34i_in((void *)a0), a1); v34i_leave(); }
static void V34SH_T_SetUpstreamModulationInfo(void *a0) { v34i_enter(); T_SetUpstreamModulationInfo((void *)v34i_in((void *)a0)); v34i_leave(); }
static void V34SH_T_VPcmV34SetMinimumSigLevel(void *a0) { v34i_enter(); T_VPcmV34SetMinimumSigLevel((void *)v34i_in((void *)a0)); v34i_leave(); }
static void V34SH_T_VPcmV34SetTimeOut(void *a0, int a1) { v34i_enter(); T_VPcmV34SetTimeOut((void *)v34i_in((void *)a0), a1); v34i_leave(); }
static void V34SH_T_VPcmV34SetMaxBlockLength(void *a0, int a1) { v34i_enter(); T_VPcmV34SetMaxBlockLength((void *)v34i_in((void *)a0), a1); v34i_leave(); }
static void V34SH_T_VPcmV34InitiateRateRenegotiation(void *a0, int a1) { v34i_enter(); T_VPcmV34InitiateRateRenegotiation((void *)v34i_in((void *)a0), a1); v34i_leave(); }
static void V34SH_T_VPcmV34InitiateHangUp(void *a0) { v34i_enter(); T_VPcmV34InitiateHangUp((void *)v34i_in((void *)a0)); v34i_leave(); }
static int V34SH_T_VPcmV34GetCurrentSessionDP(void *a0) { int r; v34i_enter(); r = T_VPcmV34GetCurrentSessionDP((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static int V34SH_T_VPcmV34GetQuickConnectIndication(void *a0) { int r; v34i_enter(); r = T_VPcmV34GetQuickConnectIndication((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static int V34SH_T_VPcmV34GetCurrentTxBitRate(void *a0) { int r; v34i_enter(); r = T_VPcmV34GetCurrentTxBitRate((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static int V34SH_T_VPcmV34GetCurrentRxBaudRate(void *a0) { int r; v34i_enter(); r = T_VPcmV34GetCurrentRxBaudRate((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static int V34SH_T_VPcmV34GetCurrentTxBaudRate(void *a0) { int r; v34i_enter(); r = T_VPcmV34GetCurrentTxBaudRate((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static int V34SH_T_VPcmV34GetCurrentRxCarrier(void *a0) { int r; v34i_enter(); r = T_VPcmV34GetCurrentRxCarrier((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static int V34SH_T_VPcmV34GetCurrentTxCarrier(void *a0) { int r; v34i_enter(); r = T_VPcmV34GetCurrentTxCarrier((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static int V34SH_T_VPcmV34GetSNR(void *a0) { int r; v34i_enter(); r = T_VPcmV34GetSNR((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static int V34SH_T_getTimingOffset(void *a0) { int r; v34i_enter(); r = T_getTimingOffset((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static int V34SH_T_getTimingPhase(void *a0) { int r; v34i_enter(); r = T_getTimingPhase((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static void * V34SH_T_VPcmV34GetCleanedSamples(void *a0, int *a1) { void * r; v34i_enter(); r = T_VPcmV34GetCleanedSamples((void *)v34i_in((void *)a0), (int *)v34i_in((void *)a1)); v34i_leave(); return (void *)v34i_outp((void *)r);}
static void V34SH_T_VPcmV34NotifyDP(void *a0, int a1) { v34i_enter(); T_VPcmV34NotifyDP((void *)v34i_in((void *)a0), a1); v34i_leave(); }
static int V34SH_T_VPcmV34RequestDPNotification(void *a0, int *a1, int *a2, int *a3) { int r; v34i_enter(); r = T_VPcmV34RequestDPNotification((void *)v34i_in((void *)a0), (int *)v34i_in((void *)a1), (int *)v34i_in((void *)a2), (int *)v34i_in((void *)a3)); v34i_leave(); return r;}
static int V34SH_T_VPcmV34GetMaxUpstreamRateIndex(void *a0) { int r; v34i_enter(); r = T_VPcmV34GetMaxUpstreamRateIndex((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static int V34SH_T_V34XF_GetMaxUpstreamRateIndex(void *a0) { int r; v34i_enter(); r = T_V34XF_GetMaxUpstreamRateIndex((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static void * V34SH_T_V34XF_GetInfo0BitsPtr(void *a0) { void * r; v34i_enter(); r = T_V34XF_GetInfo0BitsPtr((void *)v34i_in((void *)a0)); v34i_leave(); return (void *)v34i_outp((void *)r);}
static void * V34SH_T_V34XF_GetProbeResultsPtr(void *a0) { void * r; v34i_enter(); r = T_V34XF_GetProbeResultsPtr((void *)v34i_in((void *)a0)); v34i_leave(); return (void *)v34i_outp((void *)r);}
static int V34SH_T_V34XF_GetRTD(void *a0) { int r; v34i_enter(); r = T_V34XF_GetRTD((void *)v34i_in((void *)a0)); v34i_leave(); return r;}
static void V34SH_T_VPcmV34InterpretMohMessageBits(void *a0, const uint16_t *a1) { v34i_enter(); T_VPcmV34InterpretMohMessageBits((void *)v34i_in((void *)a0), (const uint16_t *)v34i_in((void *)a1)); v34i_leave(); }
static void V34SH_T_VPcmV34SetV90RateReneg(void *a0, int a1, int a2) { v34i_enter(); T_VPcmV34SetV90RateReneg((void *)v34i_in((void *)a0), a1, a2); v34i_leave(); }
static void V34SH_T_V34XF_IndicateJdReceived(void *a0, int a1, int a2) { v34i_enter(); T_V34XF_IndicateJdReceived((void *)v34i_in((void *)a0), a1, a2); v34i_leave(); }
static void V34SH_T_V34XF_IndicateDilReceived(void *a0, int a1) { v34i_enter(); T_V34XF_IndicateDilReceived((void *)v34i_in((void *)a0), a1); v34i_leave(); }
static void V34SH_T_V34XF_IndicateTrn2dReceived(void *a0) { v34i_enter(); T_V34XF_IndicateTrn2dReceived((void *)v34i_in((void *)a0)); v34i_leave(); }
static void V34SH_T_V34XF_IndicateK56FlexJdReceived(void *a0, int a1) { v34i_enter(); T_V34XF_IndicateK56FlexJdReceived((void *)v34i_in((void *)a0), a1); v34i_leave(); }
static void V34SH_T_V34XF_IndicateK56FlexRateDetermined(void *a0) { v34i_enter(); T_V34XF_IndicateK56FlexRateDetermined((void *)v34i_in((void *)a0)); v34i_leave(); }
static void V34SH_T_VPcmV34IndicateLocalRRN(void *a0) { v34i_enter(); T_VPcmV34IndicateLocalRRN((void *)v34i_in((void *)a0)); v34i_leave(); }
static void V34SH_T_VPcmV34IndicateRemoteRRN(void *a0) { v34i_enter(); T_VPcmV34IndicateRemoteRRN((void *)v34i_in((void *)a0)); v34i_leave(); }
static void V34SH_T_VPcmV34SetIndicationOfRemoteRetrain(void *a0) { v34i_enter(); T_VPcmV34SetIndicationOfRemoteRetrain((void *)v34i_in((void *)a0)); v34i_leave(); }
static void *const V34REAL_T_resetRetrainDetector = (void *)T_resetRetrainDetector; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_SetUpstreamModulationInfo = (void *)T_SetUpstreamModulationInfo; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34SetMinimumSigLevel = (void *)T_VPcmV34SetMinimumSigLevel; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34SetTimeOut = (void *)T_VPcmV34SetTimeOut; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34SetMaxBlockLength = (void *)T_VPcmV34SetMaxBlockLength; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34InitiateRateRenegotiation = (void *)T_VPcmV34InitiateRateRenegotiation; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34InitiateHangUp = (void *)T_VPcmV34InitiateHangUp; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34GetCurrentSessionDP = (void *)T_VPcmV34GetCurrentSessionDP; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34GetQuickConnectIndication = (void *)T_VPcmV34GetQuickConnectIndication; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34GetCurrentTxBitRate = (void *)T_VPcmV34GetCurrentTxBitRate; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34GetCurrentRxBaudRate = (void *)T_VPcmV34GetCurrentRxBaudRate; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34GetCurrentTxBaudRate = (void *)T_VPcmV34GetCurrentTxBaudRate; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34GetCurrentRxCarrier = (void *)T_VPcmV34GetCurrentRxCarrier; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34GetCurrentTxCarrier = (void *)T_VPcmV34GetCurrentTxCarrier; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34GetSNR = (void *)T_VPcmV34GetSNR; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_getTimingOffset = (void *)T_getTimingOffset; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_getTimingPhase = (void *)T_getTimingPhase; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34GetCleanedSamples = (void *)T_VPcmV34GetCleanedSamples; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34NotifyDP = (void *)T_VPcmV34NotifyDP; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34RequestDPNotification = (void *)T_VPcmV34RequestDPNotification; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34GetMaxUpstreamRateIndex = (void *)T_VPcmV34GetMaxUpstreamRateIndex; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_V34XF_GetMaxUpstreamRateIndex = (void *)T_V34XF_GetMaxUpstreamRateIndex; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_V34XF_GetInfo0BitsPtr = (void *)T_V34XF_GetInfo0BitsPtr; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_V34XF_GetProbeResultsPtr = (void *)T_V34XF_GetProbeResultsPtr; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_V34XF_GetRTD = (void *)T_V34XF_GetRTD; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34InterpretMohMessageBits = (void *)T_VPcmV34InterpretMohMessageBits; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34SetV90RateReneg = (void *)T_VPcmV34SetV90RateReneg; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_V34XF_IndicateJdReceived = (void *)T_V34XF_IndicateJdReceived; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_V34XF_IndicateDilReceived = (void *)T_V34XF_IndicateDilReceived; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_V34XF_IndicateTrn2dReceived = (void *)T_V34XF_IndicateTrn2dReceived; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_V34XF_IndicateK56FlexJdReceived = (void *)T_V34XF_IndicateK56FlexJdReceived; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_V34XF_IndicateK56FlexRateDetermined = (void *)T_V34XF_IndicateK56FlexRateDetermined; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34IndicateLocalRRN = (void *)T_VPcmV34IndicateLocalRRN; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34IndicateRemoteRRN = (void *)T_VPcmV34IndicateRemoteRRN; /* the real address (the name is a macro for the shim below) */
static void *const V34REAL_T_VPcmV34SetIndicationOfRemoteRetrain = (void *)T_VPcmV34SetIndicationOfRemoteRetrain; /* the real address (the name is a macro for the shim below) */
#define T_resetRetrainDetector V34SH_T_resetRetrainDetector
#define T_SetUpstreamModulationInfo V34SH_T_SetUpstreamModulationInfo
#define T_VPcmV34SetMinimumSigLevel V34SH_T_VPcmV34SetMinimumSigLevel
#define T_VPcmV34SetTimeOut V34SH_T_VPcmV34SetTimeOut
#define T_VPcmV34SetMaxBlockLength V34SH_T_VPcmV34SetMaxBlockLength
#define T_VPcmV34InitiateRateRenegotiation V34SH_T_VPcmV34InitiateRateRenegotiation
#define T_VPcmV34InitiateHangUp V34SH_T_VPcmV34InitiateHangUp
#define T_VPcmV34GetCurrentSessionDP V34SH_T_VPcmV34GetCurrentSessionDP
#define T_VPcmV34GetQuickConnectIndication V34SH_T_VPcmV34GetQuickConnectIndication
#define T_VPcmV34GetCurrentTxBitRate V34SH_T_VPcmV34GetCurrentTxBitRate
#define T_VPcmV34GetCurrentRxBaudRate V34SH_T_VPcmV34GetCurrentRxBaudRate
#define T_VPcmV34GetCurrentTxBaudRate V34SH_T_VPcmV34GetCurrentTxBaudRate
#define T_VPcmV34GetCurrentRxCarrier V34SH_T_VPcmV34GetCurrentRxCarrier
#define T_VPcmV34GetCurrentTxCarrier V34SH_T_VPcmV34GetCurrentTxCarrier
#define T_VPcmV34GetSNR V34SH_T_VPcmV34GetSNR
#define T_getTimingOffset V34SH_T_getTimingOffset
#define T_getTimingPhase V34SH_T_getTimingPhase
#define T_VPcmV34GetCleanedSamples V34SH_T_VPcmV34GetCleanedSamples
#define T_VPcmV34NotifyDP V34SH_T_VPcmV34NotifyDP
#define T_VPcmV34RequestDPNotification V34SH_T_VPcmV34RequestDPNotification
#define T_VPcmV34GetMaxUpstreamRateIndex V34SH_T_VPcmV34GetMaxUpstreamRateIndex
#define T_V34XF_GetMaxUpstreamRateIndex V34SH_T_V34XF_GetMaxUpstreamRateIndex
#define T_V34XF_GetInfo0BitsPtr V34SH_T_V34XF_GetInfo0BitsPtr
#define T_V34XF_GetProbeResultsPtr V34SH_T_V34XF_GetProbeResultsPtr
#define T_V34XF_GetRTD V34SH_T_V34XF_GetRTD
#define T_VPcmV34InterpretMohMessageBits V34SH_T_VPcmV34InterpretMohMessageBits
#define T_VPcmV34SetV90RateReneg V34SH_T_VPcmV34SetV90RateReneg
#define T_V34XF_IndicateJdReceived V34SH_T_V34XF_IndicateJdReceived
#define T_V34XF_IndicateDilReceived V34SH_T_V34XF_IndicateDilReceived
#define T_V34XF_IndicateTrn2dReceived V34SH_T_V34XF_IndicateTrn2dReceived
#define T_V34XF_IndicateK56FlexJdReceived V34SH_T_V34XF_IndicateK56FlexJdReceived
#define T_V34XF_IndicateK56FlexRateDetermined V34SH_T_V34XF_IndicateK56FlexRateDetermined
#define T_VPcmV34IndicateLocalRRN V34SH_T_VPcmV34IndicateLocalRRN
#define T_VPcmV34IndicateRemoteRRN V34SH_T_VPcmV34IndicateRemoteRRN
#define T_VPcmV34SetIndicationOfRemoteRetrain V34SH_T_VPcmV34SetIndicationOfRemoteRetrain
#endif
/* V34SHIMS END */
#if defined(__x86_64__)
/* T06: resetRetrainDetector works on a standalone 0x20-byte record (tag_retrainReqDets: no V34Obj member, no pointer) which this harness places at an
   arbitrary offset of the object image, across pointer members; it runs on the image directly, without marshalling */
#define T_resetRetrainDetector_raw ((void (*)(void *, int))V34REAL_T_resetRetrainDetector)
#else
#define T_resetRetrainDetector_raw T_resetRetrainDetector
#endif

/* the table (port copy; the blob's own table is checked THROUGH the blob's SetMinimumSigLevel, scenario 1) */
extern const int V34DisconnectThreshTable[8];

/* ---- accessors ---- */
#define OB(o, off) ((unsigned char *)(o) + (off))
#define U8(o, off) (*(uint8_t *)OB(o, off))
#define S16(o, off) (*(int16_t *)OB(o, off))
#define U16(o, off) (*(uint16_t *)OB(o, off))
#define S32(o, off) (*(int32_t *)OB(o, off))
#define U32(o, off) (*(uint32_t *)OB(o, off))
#define PTRW(o, off) (*(uint32_t *)OB(o, off)) /* 4-byte pointer slot (-m32) */
/* T06: the harness model of the object is the i386 IMAGE (test/v34/v34_img.h marshals around every call of a function under test), so these are plain image offsets
   (T02 had made them native-typed through src/v34/v34_object_view.h; that cannot reproduce the 32-bit goldens on x86-64) */
#define U8T(o, off) (*(uint8_t *)OB(o, off))
#define S16T(o, off) (*(int16_t *)OB(o, off))
#define S32T(o, off) (*(int32_t *)OB(o, off))
#define U16T(o, off) (*(uint16_t *)OB(o, off))
#define U32T(o, off) (*(uint32_t *)OB(o, off))

#define OBJSZ 0xb400u
#define ARSZ 0x7200u
/* arena layout (offsets) */
#define A_CTX 0x0000u  /* 0x6200 */
#define A_CFG 0x6200u  /* 0x600 */
#define A_P 0x6800u    /* 0x100 */
#define A_EB 0x6900u   /* [ctx+0x1758] */
#define A_QB 0x6a00u
#define A_RB 0x6a40u
#define A_EA 0x6a80u   /* [ctx+0x6124] */
#define A_QA 0x6b80u
#define A_RA 0x6bc0u
#define A_EH 0x6c00u   /* [ctx+0x175c], 0x300 B */
#define A_HB 0x6f00u   /* [EH+0x20c] */
#define A_OUT 0x7000u  /* out ints, 0x100 B */
#define A_MSG 0x7100u  /* message words */

static unsigned char *g_obj, *g_ar, *g_osnap, *g_asnap;

/* ---- deterministic LCG with a bijective finaliser ---- */
static unsigned int g_lcg = 0x4c9e2b71u;
static unsigned int lcg_next(void) {
  unsigned int x;
  g_lcg = g_lcg * 1103515245u + 12345u;
  x = g_lcg;
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}
static unsigned int lcg_mod(unsigned int m) { return (lcg_next() >> 8) % m; }
static int lcg_range(int lo, int hi) { return lo + (int)lcg_mod((unsigned int)(hi - lo + 1)); }
static int lcg_pct(int pct) { return (int)lcg_mod(100) < pct; }

static unsigned int fnv(unsigned int h, const void *p, size_t n) {
  const unsigned char *b = (const unsigned char *)p;
  size_t i;
  for (i = 0; i < n; i++) {
    h ^= b[i];
    h *= 16777619u;
  }
  return h;
}
#define FNV0 2166136261u

static void fill_random(void *p, size_t n) {
  unsigned char *b = (unsigned char *)p;
  uint64_t x = ((uint64_t)lcg_next() << 32) | lcg_next() | 1u;
  size_t i;
  for (i = 0; i + 8 <= n; i += 8) {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    memcpy(b + i, &x, 8);
  }
  for (; i < n; i++) {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    b[i] = (unsigned char)(x >> 24);
  }
}

static int g_fail = 0;
static void need(const char *name, long v) {
  printf("  coverage %s = %ld\n", name, v);
  if (v <= 0) {
    fprintf(stderr, "COVERAGE FAILURE: %s never reached\n", name);
    g_fail = 1;
  }
}
static void die(const char *msg) {
  fflush(stdout);
  fprintf(stderr, "HARNESS ERROR: %s\n", msg);
  exit(3);
}

/* ---- pointer slots ---- */
typedef struct {
  int in_obj;
  unsigned int off;
  unsigned int target; /* arena offset */
} slot_t;
static const slot_t k_slots[] = {
    {1, 0x3548, A_CTX}, {1, 0xac3c, A_P},          {0, A_CTX + 0x1758, A_EB}, {0, A_CTX + 0x175c, A_EH},
    {0, A_CTX + 0x6124, A_EA}, {0, A_CTX + 0x610c, A_CFG}, {0, A_EB + 0x40, A_QB}, {0, A_QB + 0, A_RB},
    {0, A_EA + 0x4c, A_QA},    {0, A_QA + 0, A_RA},        {0, A_EH + 0x20c, A_HB},
};
#define NSLOT (sizeof k_slots / sizeof k_slots[0])
static void set_slots(void) {
  size_t i;
  for (i = 0; i < NSLOT; i++) {
    unsigned char *b = k_slots[i].in_obj ? g_obj : g_ar;
    PTRW(b, k_slots[i].off) = (uint32_t)(uintptr_t)(g_ar + k_slots[i].target);
  }
}
static void check_slots(const char *what) {
  size_t i;
  for (i = 0; i < NSLOT; i++) {
    unsigned char *b = k_slots[i].in_obj ? g_obj : g_ar;
    if (PTRW(b, k_slots[i].off) != (uint32_t)(uintptr_t)(g_ar + k_slots[i].target)) {
      char m[120];
      snprintf(m, sizeof m, "%s: pointer slot %zu was modified", what, i);
      die(m);
    }
  }
}

/* fresh random state + valid slots + snapshot */
static void fresh(void) {
  fill_random(g_obj, OBJSZ);
  fill_random(g_ar, ARSZ);
  set_slots();
}
static void snap(void) {
  memcpy(g_osnap, g_obj, OBJSZ);
  memcpy(g_asnap, g_ar, ARSZ);
}

/* hash of the DIFF (word index + new value) of object + arena against the snapshot */
static unsigned int diff_state(void) {
  unsigned int h = FNV0;
  size_t i, j;
  V34I_SYNC_IMG(); /* x86-64: callbacks (recorders, print hooks) run while the port works on SHADOWS: bring the images up to date first (no-op outside a call) */
  for (i = 0; i < OBJSZ / 4; i += 16) {
    if (!memcmp(g_obj + 4 * i, g_osnap + 4 * i, 64)) continue;
    for (j = i; j < i + 16; j++) {
      uint32_t a, b;
      memcpy(&a, g_obj + 4 * j, 4);
      memcpy(&b, g_osnap + 4 * j, 4);
      if (a != b) {
        uint32_t idx = (uint32_t)j;
        h = fnv(h, &idx, 4);
        h = fnv(h, &a, 4);
      }
    }
  }
  for (i = 0; i < ARSZ / 4; i += 16) {
    if (!memcmp(g_ar + 4 * i, g_asnap + 4 * i, 64)) continue;
    for (j = i; j < i + 16 && j < ARSZ / 4; j++) {
      uint32_t a, b;
      memcpy(&a, g_ar + 4 * j, 4);
      memcpy(&b, g_asnap + 4 * j, 4);
      if (a != b) {
        uint32_t idx = (uint32_t)j | 0x10000u;
        h = fnv(h, &idx, 4);
        h = fnv(h, &a, 4);
      }
    }
  }
  return h;
}

/* ---- per-scenario rolling log ---- */
typedef struct {
  const char *name;
  unsigned int all;
  long calls;
  long every;
} logger_t;
static void log_call(logger_t *l, unsigned int h, long a, long b) {
  l->all = fnv(l->all, &h, 4);
  l->all = fnv(l->all, &a, 4);
  if (l->calls % l->every == 0) printf("%s c=%ld a=%ld b=%ld hash=%08x\n", l->name, l->calls, a, b, h);
  l->calls++;
}
static void log_done(logger_t *l) { printf("%s calls=%ld all=%08x\n", l->name, l->calls, l->all); }

/* ============================================================================
 * per-call recording: debug lines + state hash at each print, callee trace
 * ========================================================================== */
#define MAXL 24
typedef struct {
  char line[MAXL][240];
  unsigned int ph[MAXL];
  int nl;
} plog_t;
static plog_t g_pl;
static int g_hook_k = -1;      /* after the k-th print of a call ... */
static unsigned int g_hook_lvl; /* ... set the level to this */

static void plog_add(const char *line) {
  if (g_pl.nl >= MAXL) die("too many prints in one call");
  strncpy(g_pl.line[g_pl.nl], line, sizeof g_pl.line[0] - 1);
  g_pl.line[g_pl.nl][sizeof g_pl.line[0] - 1] = 0;
  g_pl.ph[g_pl.nl] = diff_state();
  if (g_hook_k >= 0 && g_pl.nl == g_hook_k) dsplibs_debug_level = g_hook_lvl;
  g_pl.nl++;
}
int dsplibs_debug_printf(const char *fmt, ...) {
  char buf[240];
  va_list ap;
  int len;
  va_start(ap, fmt);
  len = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  plog_add(buf);
  return len;
}
/* edprintf recorder: the FORMATTED text and the state at the call (level-independent: logged at every level) */
void edprintf(const char *fmt, ...) {
  char buf[240];
  char tag[260];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  snprintf(tag, sizeof tag, "ED:%s", buf);
  plog_add(tag);
}
static unsigned int hash_plog(void) {
  unsigned int h = FNV0;
  int i;
  for (i = 0; i < g_pl.nl; i++) {
    h = fnv(h, g_pl.line[i], strlen(g_pl.line[i]) + 1);
    h = fnv(h, &g_pl.ph[i], 4);
  }
  h = fnv(h, &g_pl.nl, 4);
  return h;
}

/* ---- the scribbling recorders (callees) ---- */
static unsigned int g_cseed;
static int g_occ[4];
typedef struct {
  int id;
  uint32_t a0;
  uint32_t a1;
  unsigned int dh;
} tr_t;
static tr_t g_tr[8];
static int g_ntr;
static uint64_t xs_next(uint64_t *s) {
  uint64_t x = *s;
  x ^= x << 13;
  x ^= x >> 7;
  x ^= x << 17;
  *s = x;
  return x;
}
static uint32_t norm_ptr(const void *p) {
  uintptr_t a = (uintptr_t)p;
  if (a >= (uintptr_t)g_obj && a < (uintptr_t)g_obj + OBJSZ) return 0x80000000u | (uint32_t)(a - (uintptr_t)g_obj);
  return (uint32_t)a;
}
static void rec(int id, void *obj, uint32_t a1) {
  tr_t *t;
  uint64_t s = (((uint64_t)(g_cseed + 0x9e3779b9u * (unsigned int)(g_occ[id]++)) << 32) | 0x5bd1e995u) ^
               (0x9e3779b97f4a7c15ull * (uint64_t)(id + 1));
  int i;
  if (g_ntr >= 8) die("too many callee calls");
  t = &g_tr[g_ntr++];
  t->id = id;
  obj = V34I_IMGP(obj); /* x86-64: the port passes the SHADOW of the object */
  t->a0 = norm_ptr(obj);
  t->a1 = a1;
  t->dh = diff_state();
  if (!s) s = 1;
  for (i = 0; i < 4; i++) xs_next(&s);
  /* scribble the fields the callers read AFTER the call */
  if (id == 1) { /* v34handshakinit: VPcmV34InitiateRateRenegotiation re-reads 0xac0e after the call */
    U16T(obj, 0xac0e) = (uint16_t)(xs_next(&s) >> 30);
    U16T(obj, 0x4bc) = (uint16_t)(xs_next(&s) >> 30);
    S32T(obj, 0x4) = (int32_t)(xs_next(&s) >> 30);
    S32T(obj, 0x2218) = (int32_t)(xs_next(&s) >> 30);
  } else if (id == 3) { /* v34setuptxmit: K56FlexJd re-reads 0x3596 and 0x35a4 after the call */
    unsigned int r = (unsigned int)(xs_next(&s) >> 20) % 100;
    if (r < 40) U16T(obj, 0x3596) = 0x12;
    else U16T(obj, 0x3596) = (uint16_t)(xs_next(&s) >> 30);
    U16T(obj, 0x35a4) = (uint16_t)(xs_next(&s) >> 30);
    U16T(obj, 0x25c0) = (uint16_t)(xs_next(&s) >> 30);
    U32T(obj, 0x258) = (uint32_t)(xs_next(&s) >> 30);
  } else if (id == 2) { /* preinitdigital: SetV90RateReneg stores 0x2aa2 / 0x382 / 0x238 ... afterwards; scribble the pre-stored ones */
    U16T(obj, 0x25c2) = (uint16_t)(xs_next(&s) >> 30);
    S32T(obj, 0x24c) = (int32_t)(xs_next(&s) >> 30);
    U16T(obj, 0x2aa2) = (uint16_t)(xs_next(&s) >> 30);
    U32T(obj, 0x248) = (uint32_t)(xs_next(&s) >> 30);
  }
  V34I_SYNC_NAT(); /* x86-64: push the scribbled image fields into the shadow the port goes on with */
}
int v34handshakinit(void *obj, unsigned int mode) {
  rec(1, obj, mode);
  return 0;
}
void preinitdigital(void *obj) { rec(2, obj, 0); }
void v34setuptxmit(void *obj) { rec(3, obj, 0); }
static unsigned int hash_trace(void) {
  unsigned int h = FNV0;
  int i;
  for (i = 0; i < g_ntr; i++) h = fnv(fnv(fnv(fnv(h, &g_tr[i].id, 4), &g_tr[i].a0, 4), &g_tr[i].a1, 4), &g_tr[i].dh, 4);
  return fnv(h, &g_ntr, 4);
}

/* per-call reset of the recording state */
static void call_begin(void) {
  g_pl.nl = 0;
  g_ntr = 0;
  memset(g_occ, 0, sizeof g_occ);
  g_cseed = lcg_next();
  snap();
}
/* common epilogue: hash = state diff + prints + trace; asserts the pointer slots are intact */
static unsigned int call_end(const char *what) {
  unsigned int h;
  check_slots(what);
  h = diff_state();
  h = fnv(h, &(unsigned int){hash_plog()}, 4);
  h = fnv(h, &(unsigned int){hash_trace()}, 4);
  return h;
}

/* levels used for the prints */
static unsigned int pick_level(void) {
  static const unsigned int lv[] = {0, 1, 2, 3, 0x80000000u, 0xffffffffu, 2, 2, 0, 5};
  return lv[lcg_mod(sizeof lv / sizeof lv[0])];
}
static void pick_hook(void) {
  g_hook_k = -1;
  if (lcg_pct(30)) {
    static const unsigned int hl[] = {0, 1, 2, 3, 0x80000000u};
    g_hook_k = (int)lcg_mod(4);
    g_hook_lvl = hl[lcg_mod(5)];
  }
}
/* garbage in the upper bits of a narrow argument */
static int garb16(int v) { return (int)(((unsigned int)lcg_next() & 0xffff0000u) | ((unsigned int)v & 0xffffu)); }
static int garb8(int v) { return (int)(((unsigned int)lcg_next() & 0xffffff00u) | ((unsigned int)v & 0xffu)); }

/* steered 32-bit "interesting" values */
static int32_t wild32(void) {
  static const int32_t sp[] = {0, 1, -1, 2, 3, 5, 6, 0x7fffffff, (int32_t)0x80000000, 0x7fff, 0x8000, -0x8000, 0xffff, 0x10000,
                               0x2580, -0x2580, 0x960, 100, -100};
  if (lcg_pct(35)) return sp[lcg_mod(sizeof sp / sizeof sp[0])];
  return (int32_t)lcg_next();
}

/* ============================================================================
 * scenario 0: simple setters / indicators / pointer returns
 * ========================================================================== */
static void scen0(void) {
  logger_t L = {"simple", FNV0, 0, 211};
  long n_mode65 = 0, n_mode66 = 0, n_modeo = 0, n_neg = 0, n_wrap = 0, n_alias = 0, n_lv = 0, n_ret = 0, n_bnd = 0;
  int i, f;
  for (i = 0; i < 40000; i++) {
    unsigned int h;
    long ra = 0, rb = 0;
    f = i % 20;
    fresh();
    dsplibs_debug_level = pick_level();
    pick_hook();
    call_begin();
    switch (f) {
      case 0: { /* resetRetrainDetector(r, mode): r = 0x20-byte record at a random obj offset */
        unsigned int off = (lcg_mod(0xb000u) & ~3u) + 0;
        int m;
        if (off + 0x20 > 0x3548 && off < 0x354c) off = 0x3600; /* keep the 0x20-byte record off the pointer slots */
        if (off + 0x20 > 0xac3c && off < 0xac40) off = 0x9000;
        unsigned int c = lcg_mod(10);
        m = (c < 3) ? 0x65 : (c < 6) ? 0x66 : (c < 7) ? 0x64 : (c < 8) ? 0x67 : (c < 9) ? 0 : (int)(lcg_next() & 0xffff);
        if (c < 3) n_mode65++; else if (c < 6) n_mode66++; else n_modeo++;
        T_resetRetrainDetector_raw(g_obj + off, garb16(m));
        ra = m;
        break;
      }
      case 1: T_SetUpstreamModulationInfo(g_obj); break;
      case 2: ra = T_VPcmV34Delete(); break;
      case 3: { /* SetMaxBlockLength: level read before the store, then the print */
        int len = wild32();
        T_VPcmV34SetMaxBlockLength(g_obj, len);
        ra = len;
        n_lv += dsplibs_debug_level > 1u;
        break;
      }
      case 4: {
        int t = wild32();
        T_VPcmV34SetTimeOut(g_obj, t);
        ra = t;
        if (t < 0) n_neg++;
        if ((int64_t)t * 0x2580 != (int32_t)((uint32_t)t * 0x2580u)) n_wrap++;
        break;
      }
      case 5:
        if (lcg_pct(25)) U16T(g_obj, 0xac0e) = 0xffff; /* 16-bit wrap */
        if (U16T(g_obj, 0xac0e) == 0xffff) n_wrap++;
        T_VPcmV34IndicateLocalRRN(g_obj);
        break;
      case 6:
        if (lcg_pct(25)) U16T(g_obj, 0xac10) = 0xffff;
        if (U16T(g_obj, 0xac10) == 0xffff) n_wrap++;
        T_VPcmV34IndicateRemoteRRN(g_obj);
        break;
      case 7: T_VPcmV34SetIndicationOfRemoteRetrain(g_obj); break;
      case 8: ra = T_getTimingOffset(g_obj); break;
      case 9: ra = T_getTimingPhase(g_obj); break;
      case 10: ra = (long)((unsigned char *)T_V34XF_GetInfo0BitsPtr(g_obj) - g_obj); n_ret++; break;
      case 11: ra = (long)((unsigned char *)T_V34XF_GetProbeResultsPtr(g_obj) - g_obj); n_ret++; break;
      case 12:
        if (lcg_pct(30)) U16T(g_obj, 0xaa7e) = (uint16_t)(0xfe20 + lcg_mod(0x40)); /* sum crosses 16 bits */
        ra = T_V34XF_GetRTD(g_obj);
        if (ra < 0) n_neg++;
        break;
      case 13: case 14: { /* GetCleanedSamples: normal out buffer, or an out pointer ALIASING obj+0x2aa6 (store order) */
        int *out = (int *)(g_ar + A_OUT);
        int alias = lcg_pct(12) && f == 13;
        void *r;
        if (alias) { out = (int *)(g_obj + 0x2aa6); n_alias++; }
        r = T_VPcmV34GetCleanedSamples(g_obj, out);
        ra = (long)((unsigned char *)r - g_obj);
        rb = alias ? 0 : *out;
        n_ret++;
        break;
      }
      case 15: T_VPcmV34ReportStartOfEchoAdapt(); n_lv += dsplibs_debug_level > 1u; break;
      case 16: T_VPcmV34ReportMiddleOfEchoAdapt(); n_lv += dsplibs_debug_level > 1u; break;
      default: { /* RequestDPNotification, random: valid (>=0) / invalid (<0), distinct out pointers in the arena */
        int *a = (int *)(g_ar + A_OUT), *b = (int *)(g_ar + A_OUT + 16), *c = (int *)(g_ar + A_OUT + 32);
        if (lcg_pct(50)) S32T(g_obj, 0xac40) = (int32_t)(lcg_next() & 0x7fffffffu);
        if (lcg_pct(10)) S32T(g_obj, 0xac40) = -1;
        if (lcg_pct(30)) { /* the sign boundary: 0, 1, 2 are VALID, -1 / INT_MIN are not */
          static const int32_t sb[] = {0, 1, 2, -1, -2, 0x7fffffff, (int32_t)0x80000000};
          S32T(g_obj, 0xac40) = sb[lcg_mod(7)];
          n_bnd++;
        }
        ra = T_VPcmV34RequestDPNotification(g_obj, a, b, c);
        rb = *a ^ *b ^ *c;
        n_ret += (int)ra;
        break;
      }
    }
    h = call_end("scen0");
    log_call(&L, h, ra, rb);
  }
  log_done(&L);
  need("resetRetrainDetector mode 0x65", n_mode65);
  need("resetRetrainDetector mode 0x66", n_mode66);
  need("resetRetrainDetector other modes", n_modeo);
  need("SetTimeOut negative", n_neg);
  need("SetTimeOut / counter wrap", n_wrap);
  need("GetCleanedSamples aliased out pointer", n_alias);
  need("print enabled (level > 1)", n_lv);
  need("pointer / notification returns", n_ret);
  need("RequestDPNotification d at the sign boundary (0, 1, 2, -1 ...)", n_bnd);
}

/* ============================================================================
 * scenario 1: interpretMohTimeout (exhaustive) + SetMinimumSigLevel (table through the blob)
 * ========================================================================== */
static void scen1(void) {
  logger_t L = {"moh_timeout", FNV0, 0, 4099}, L2 = {"min_sig_level", FNV0, 0, 509};
  long cnt[16], n_inrange = 0, n_hi = 0, n_hi2 = 0, idxhit[8] = {0};
  int v, i, k;
  memset(cnt, 0, sizeof cnt);
  for (k = 0; k < 4; k++)
    for (v = 0; v < 65536; v++) {
      int arg = (k == 0) ? v : (k == 1) ? (int)(0xffff0000u | (unsigned)v) : (k == 2) ? (int)(0x5a5a0000u | (unsigned)v) : garb16(v);
      int r = T_interpretMohTimeout(arg);
      log_call(&L, (unsigned int)r, v, r);
      if (v <= 13) cnt[v]++;
      else n_hi++;
    }
  log_done(&L);
  for (i = 0; i <= 13; i++) {
    char nm[40];
    snprintf(nm, sizeof nm, "interpretMohTimeout code %d", i);
    need(nm, cnt[i]);
  }
  need("interpretMohTimeout > 13 (low word)", n_hi);
  (void)n_inrange;
  /* SetMinimumSigLevel: index = (unsigned)([P+0x60]+0x30); steer around 0..7 and the unsigned wrap */
  for (i = 0; i < 24000; i++) {
    int pv;
    unsigned int h;
    int idx;
    unsigned int c = lcg_mod(10);
    fresh();
    dsplibs_debug_level = pick_level();
    pick_hook();
    if (c < 6) pv = (int)lcg_range(-0x30 - 3, 12 - 0x30);
    else if (c < 7) pv = -0x30 + 7 + 1;
    else if (c < 8) pv = (int)0x7fffffd0 + (int)lcg_range(-3, 3);
    else pv = (int)lcg_next();
    S32(g_ar, A_P + 0x60) = pv;
    call_begin();
    T_VPcmV34SetMinimumSigLevel(g_obj);
    h = call_end("scen1");
    idx = (int)((unsigned int)pv + 0x30u);
    if ((unsigned int)idx <= 7u) idxhit[idx]++;
    else n_hi2++;
    /* the blob's (or the port's) own table through the function: S32(obj,0x230) must equal the extracted copy */
    {
      unsigned int u = (unsigned int)idx;
      int expect = V34DisconnectThreshTable[u > 7u ? 3 : u];
      if (S32T(g_obj, 0x230) != expect) die("SetMinimumSigLevel result differs from the extracted V34DisconnectThreshTable");
      if (S32T(g_obj, 0x234) != 0) die("SetMinimumSigLevel [0x234] != 0");
    }
    log_call(&L2, h, pv, S32T(g_obj, 0x230));
  }
  log_done(&L2);
  for (i = 0; i < 8; i++) {
    char nm[40];
    snprintf(nm, sizeof nm, "SetMinimumSigLevel table index %d", i);
    need(nm, idxhit[i]);
  }
  need("SetMinimumSigLevel unsigned index > 7 (-> 3)", n_hi2);
}

/* ============================================================================
 * scenario 2: state getters and GetSNR
 * ========================================================================== */
static int steer_mode(void) {
  static const int32_t sp[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, -1, -2, 0x7fffffff, (int32_t)0x80000000, 0x100, 0x101};
  if (lcg_pct(85)) return sp[lcg_mod(sizeof sp / sizeof sp[0])];
  return (int32_t)lcg_next();
}
static void scen2(void) {
  logger_t L = {"getters", FNV0, 0, 997};
  long m_dp[4] = {0}, qc[3] = {0}, n66 = 0, nn66 = 0, n_lowword66 = 0, ret8000 = 0, ret0 = 0, retv = 0;
  long snr_div_le0 = 0, snr_q_le0 = 0, snr_loop1 = 0, snr_loop2 = 0, snr_neg_div = 0, snr_big = 0;
  int i, f;
  for (i = 0; i < 90000; i++) {
    unsigned int h;
    long ra;
    f = i % 9;
    fresh();
    dsplibs_debug_level = 0;
    S32T(g_obj, 0x0) = steer_mode();
    {
      unsigned int c = lcg_mod(10);
      if (c < 4) U16T(g_obj, 0x359c) = 0x66;
      else if (c < 6) U16T(g_obj, 0x359c) = 0x65;
      else if (c < 7) U32T(g_obj, 0x359c) = 0xabcd0066u; /* garbage above the compared word */
      else if (c < 8) U16T(g_obj, 0x359c) = 0x166;       /* low byte 0x66 only */
      if (U16T(g_obj, 0x359c) == 0x66) n66++; else nn66++;
      if (U32T(g_obj, 0x359c) != 0x66 && U16T(g_obj, 0x359c) == 0x66) n_lowword66++;
    }
    if (lcg_pct(40)) U16T(g_obj, 0xabc6) = 0;
    if (lcg_pct(40)) U16T(g_obj, 0xabc8) = 0;
    if (lcg_pct(30)) { U16T(g_obj, 0xabc6) = 0; U16T(g_obj, 0xabc8) = 0; }
    if (f == 8) { /* SNR: steer divisor and dividend */
      unsigned int c = lcg_mod(10);
      if (c < 1) S16T(g_obj, 0x47e) = 0;
      else if (c < 2) S16T(g_obj, 0x47e) = (int16_t)lcg_range(-300, -1);
      else if (c < 3) S16T(g_obj, 0x47e) = (int16_t)0x8000;
      else if (c < 6) S16T(g_obj, 0x47e) = (int16_t)lcg_range(1, 4);
      else S16T(g_obj, 0x47e) = (int16_t)(lcg_next() & 0x7fff);
      c = lcg_mod(10);
      if (c < 2) S32T(g_obj, 0x4ac) = (int32_t)lcg_range(-5, 5);
      else if (c < 4) S32T(g_obj, 0x4ac) = (int32_t)0x7fffffff - (int32_t)lcg_range(0, 5);
      else if (c < 5) S32T(g_obj, 0x4ac) = (int32_t)(0x80000000u + lcg_range(0, 5));
      else if (c < 8) S32T(g_obj, 0x4ac) = (int32_t)(lcg_next() & 0xfffffu) * (int32_t)lcg_range(1, 40);
    }
    call_begin();
    switch (f) {
      case 0: ra = T_VPcmV34GetCurrentSessionDP(g_obj); break;
      case 1: ra = T_VPcmV34GetQuickConnectIndication(g_obj); break;
      case 2: ra = T_VPcmV34GetCurrentRxBaudRate(g_obj); break;
      case 3: ra = T_VPcmV34GetCurrentTxBaudRate(g_obj); break;
      case 4: ra = T_VPcmV34GetCurrentRxCarrier(g_obj); break;
      case 5: ra = T_VPcmV34GetCurrentTxCarrier(g_obj); break;
      case 6: ra = T_VPcmV34GetCurrentRxBaudRate(g_obj) ^ T_VPcmV34GetCurrentTxCarrier(g_obj); break;
      case 7: ra = T_VPcmV34GetCurrentTxBaudRate(g_obj) ^ T_VPcmV34GetCurrentRxCarrier(g_obj); break;
      default: {
        int16_t d = S16T(g_obj, 0x47e);
        int32_t dv = S32T(g_obj, 0x4ac);
        ra = T_VPcmV34GetSNR(g_obj);
        if (d <= 0) snr_div_le0++;
        else {
          int32_t q = dv / d;
          if (q <= 0) snr_q_le0++;
          else {
            int32_t p = (int32_t)((uint32_t)q * 0x1013u) >> 14;
            if (p > 0) snr_loop1++;
            else if (ra > 0) snr_loop2++;
            if (q > 0x100000) snr_big++;
          }
        }
        if (dv < 0) snr_neg_div++;
        break;
      }
    }
    if (f == 0) { int32_t m = S32T(g_obj, 0x0); m_dp[m == 1 ? 0 : m == 2 ? 1 : m == 3 ? 2 : 3]++; }
    if (f == 1) { uint32_t m = U32T(g_obj, 0x0); qc[m > 10 ? 2 : ((1u << m) & 0xe7u) ? 0 : 1]++; }
    if (f >= 2 && f <= 7) { if (ra == 8000) ret8000++; else if (ra == 0) ret0++; else retv++; }
    h = call_end("scen2");
    log_call(&L, h, ra, f);
  }
  log_done(&L);
  need("SessionDP mode 1", m_dp[0]);
  need("SessionDP mode 2", m_dp[1]);
  need("SessionDP mode 3", m_dp[2]);
  need("SessionDP other", m_dp[3]);
  need("QuickConnect mode in {0,1,2,5,6,7}", qc[0]);
  need("QuickConnect other mode <= 10", qc[1]);
  need("QuickConnect mode > 10", qc[2]);
  need("0x359c == 0x66", n66);
  need("0x359c != 0x66", nn66);
  need("0x359c low word 0x66 with garbage above", n_lowword66);
  need("baud/carrier result 8000", ret8000);
  need("baud/carrier result 0", ret0);
  need("baud/carrier result from the object", retv);
  need("SNR divisor <= 0", snr_div_le0);
  need("SNR quotient <= 0", snr_q_le0);
  need("SNR first loop ran", snr_loop1);
  need("SNR only second loop", snr_loop2);
  need("SNR negative dividend", snr_neg_div);
  need("SNR large quotient", snr_big);
}

/* ============================================================================
 * scenario 3: GetCurrentTxBitRate (x87), independent integer model
 * ========================================================================== */
static int model_txrate(const unsigned char *obj, const unsigned char *ar) {
  int32_t mode = S32T(obj, 0x0);
  int chainB = 0, chainA = 0;
  uint32_t v;
  uint64_t acc;
  const unsigned char *e, *q, *r;
  (void)ar;
  if (U16T(obj, 0x359c) == 0x66) {
    if (mode == 2) chainA = 1;
    else if (mode == 3) return 30000;
  } else if ((uint32_t)(mode - 1) <= 1u)
    chainB = 1;
  if (!chainA && !chainB) return (int)S16T(obj, 0xaa88) * 0x960;
  e = (const unsigned char *)(uintptr_t)PTRW((unsigned char *)ar, A_CTX + (chainA ? 0x6124 : 0x1758));
  if (S32((unsigned char *)e, 0x2c) != 3) return 0;
  q = (const unsigned char *)(uintptr_t)PTRW((unsigned char *)e, chainA ? 0x4c : 0x40);
  r = (const unsigned char *)(uintptr_t)PTRW((unsigned char *)q, 0);
  v = (uint32_t)((uint64_t)(uint32_t)S32((unsigned char *)r, 4) * 8000u);
  acc = (uint64_t)v * 0xaaaaabull; /* exact: v * m, m = 0xaaaaab (24-bit mantissa of 1/6 and 1/12) */
  if (chainB) return (int)(uint32_t)((acc + (1ull << 25)) >> 26); /* 1/6f  = m * 2^-26, + 0.5 */
  return (int)(uint32_t)((acc + (1ull << 26)) >> 27);              /* 1/12f = m * 2^-27, + 0.5 */
}
static void scen3(void) {
  logger_t L = {"txbitrate", FNV0, 0, 997};
  long nA = 0, nB = 0, n30000 = 0, nflt = 0, nstate = 0, nzero = 0, nuns = 0, nhalf = 0, nbig = 0, nhalf2 = 0;
  int i;
  for (i = 0; i < 160000; i++) {
    unsigned int h;
    int r, mr;
    unsigned int c;
    fresh();
    dsplibs_debug_level = 0;
    c = lcg_mod(10);
    S32T(g_obj, 0x0) = (c < 3) ? 2 : (c < 5) ? 1 : (c < 6) ? 3 : (c < 7) ? 4 : (int32_t)steer_mode();
    c = lcg_mod(10);
    if (c < 5) U16T(g_obj, 0x359c) = 0x66;
    else if (c < 7) U32T(g_obj, 0x359c) = 0x12340066u;
    else if (c < 8) U16T(g_obj, 0x359c) = 0x67;
    /* chain nodes: [e+0x2c] == 3 most of the time */
    S32(g_ar, A_EB + 0x2c) = lcg_pct(80) ? 3 : (int32_t)lcg_range(0, 4);
    S32(g_ar, A_EA + 0x2c) = lcg_pct(80) ? 3 : (int32_t)lcg_range(0, 4);
    {
      int32_t pv;
      c = lcg_mod(10);
      if (c < 3) pv = (int32_t)lcg_range(0, 40000);                 /* typical rates */
      else if (c < 5) pv = (int32_t)lcg_range(-40000, 0);            /* product * 8000 negative: UNSIGNED view */
      else if (c < 6) pv = (int32_t)(0x7fffffff / 8000 + lcg_range(-3, 3));
      else if (c < 7) pv = (int32_t)(0x80000000u / 8000 + lcg_range(-3, 3));
      else if (c < 8) pv = (int32_t)(6 * lcg_range(0, 5000) + lcg_range(-1, 1)); /* near multiples: rounding boundaries */
      else pv = (int32_t)lcg_next();
      if (lcg_pct(12)) { /* EXACT x.5 boundary: v = u*8000 (mod 2^32) == 2^25 (mod 2^26) <=> u = (2t+1)<<19 + k<<26; +-1 on the sides */
        pv = (int32_t)((((uint32_t)lcg_range(0, 63) * 2u + 1u) << 19) + ((uint32_t)lcg_range(0, 63) << 26));
        if (lcg_pct(30)) pv += lcg_range(-1, 1);
        nhalf2++;
      }
      S32(g_ar, A_RB + 4) = pv;
      c = lcg_mod(10);
      if (c < 3) pv = (int32_t)lcg_range(0, 40000);
      else if (c < 5) pv = (int32_t)lcg_range(-40000, 0);
      else if (c < 8) pv = (int32_t)(12 * lcg_range(0, 3000) + lcg_range(-1, 1));
      else pv = (int32_t)lcg_next();
      if (lcg_pct(12)) { /* EXACT x.5 boundary for 1/12: v == 2^26 (mod 2^27) <=> u = (2t+1)<<20 + k<<27 */
        pv = (int32_t)((((uint32_t)lcg_range(0, 31) * 2u + 1u) << 20) + ((uint32_t)lcg_range(0, 31) << 27));
        if (lcg_pct(30)) pv += lcg_range(-1, 1);
        nhalf2++;
      }
      S32(g_ar, A_RA + 4) = pv;
    }
    call_begin();
    r = T_VPcmV34GetCurrentTxBitRate(g_obj);
    mr = model_txrate(g_obj, g_ar);
    if (r != mr) {
      char m[160];
      snprintf(m, sizeof m, "txbitrate: result %d differs from the independent model %d (call %d)", r, mr, i);
      die(m);
    }
    {
      int32_t mode = S32T(g_obj, 0x0);
      if (U16T(g_obj, 0x359c) == 0x66) {
        if (mode == 2) { nA++; if (S32(g_ar, A_EA + 0x2c) == 3) nstate++; else nzero++; }
        else if (mode == 3) n30000++;
        else nflt++;
      } else if ((uint32_t)(mode - 1) <= 1u) {
        nB++;
        if (S32(g_ar, A_EB + 0x2c) == 3) nstate++; else nzero++;
      } else nflt++;
      if (r > 100000) nbig++;
      if (((uint64_t)(uint32_t)((uint32_t)S32(g_ar, A_RB + 4) * 8000u)) > 0x7fffffffull) nuns++;
      if ((r & 1) == 0 && r != 0) nhalf++;
    }
    h = call_end("scen3");
    log_call(&L, h, r, 0);
  }
  log_done(&L);
  need("chain A (0x66, mode 2)", nA);
  need("chain B (mode 1/2)", nB);
  need("mode 3 -> 30000", n30000);
  need("fallback (short)[0xaa88]*0x960", nflt);
  need("chain state == 3", nstate);
  need("chain state != 3 -> 0", nzero);
  need("unsigned (>= 2^31) product", nuns);
  need("result > 100000", nbig);
  need("even results", nhalf);
  need("exact x.5 boundary inputs steered", nhalf2);
}

/* ============================================================================
 * scenario 4: edprintf group
 * ========================================================================== */
static void scen4(void) {
  logger_t L = {"edprintf_group", FNV0, 0, 997};
  long n_code[5] = {0}, n_sens = 0, n_reg = 0, n_clamp = 0, n_noclamp = 0, n_ovf = 0, n_lazy = 0, n_le1 = 0, n_cfg0 = 0;
  int i;
  for (i = 0; i < 80000; i++) {
    unsigned int h;
    long ra = 0;
    int f = i % 5;
    fresh();
    dsplibs_debug_level = pick_level();
    pick_hook();
    if (f == 0) {
      int code = (int)lcg_range(-2, 5);
      if (lcg_pct(10)) code = (int)lcg_next();
      if (lcg_pct(25)) S32T(g_obj, 0xabdc) = (int32_t)(0x7fffffff - lcg_range(0, 0xbb80 + 2)); /* + 48000 wraps */
      call_begin();
      T_VPcmV34NotifyDP(g_obj, code);
      n_code[code >= 0 && code <= 3 ? code : 4]++;
      ra = code;
    } else {
      int twin = (f == 2);
      /* Get(Max)UpstreamRateIndex */
      S32(g_ar, A_CTX + 0x6120) = lcg_pct(70) ? (int32_t)lcg_range(1, 3) : 0;
      {
        int32_t m = S32T(g_obj, 0x24c);
        unsigned int c = lcg_mod(10);
        S32T(g_obj, 0x24c) = (c < 3) ? 2 : (c < 5) ? 1 : (c < 6) ? 0 : (c < 7) ? (int32_t)0x80000000 : (c < 8) ? 3 : m;
      }
      S32(g_ar, A_CFG + 0x4f8) = lcg_pct(75) ? (int32_t)lcg_range(1, 9) : 0;
      {
        unsigned int c = lcg_mod(10);
        int32_t k;
        if (c < 4) k = (int32_t)lcg_range(0, 40);
        else if (c < 6) k = (int32_t)lcg_range(-3, 3);
        else if (c < 8) k = (int32_t)(0x7fffffff / 0x960 + lcg_range(-2, 2));
        else k = (int32_t)lcg_next();
        S32(g_ar, A_CFG + 0x4fc) = k;
        c = lcg_mod(10);
        {
          int32_t lim = (int32_t)((uint32_t)k * 0x960u);
          int32_t rate = (c < 3) ? lim : (c < 5) ? lim + 1 : (c < 7) ? lim - 1 : (int32_t)lcg_next();
          S32(g_ar, A_P + 0x3c) = rate;
        }
      }
      call_begin();
      ra = (f == 1 || f == 3) ? T_VPcmV34GetMaxUpstreamRateIndex(g_obj) : twin ? T_V34XF_GetMaxUpstreamRateIndex(g_obj)
                                                                               : T_VPcmV34GetMaxUpstreamRateIndex(g_obj);
      if (f == 4) ra ^= T_V34XF_GetMaxUpstreamRateIndex(g_obj);
      {
        int sens = S32(g_ar, A_CTX + 0x6120) != 0 && S32T(g_obj, 0x24c) > 1;
        if (sens && S32(g_ar, A_CFG + 0x4f8) != 0) {
          int32_t lim = (int32_t)((uint32_t)S32(g_ar, A_CFG + 0x4fc) * 0x960u);
          n_sens++;
          if (S32(g_ar, A_P + 0x3c) > lim) n_clamp++; else n_noclamp++;
          if ((int64_t)S32(g_ar, A_CFG + 0x4fc) * 0x960 != lim) n_ovf++;
        } else n_reg++;
        if (!(S32(g_ar, A_CTX + 0x6120) != 0)) n_lazy++;
        if (S32(g_ar, A_CTX + 0x6120) != 0 && S32T(g_obj, 0x24c) <= 1) n_le1++;
        if (S32(g_ar, A_CTX + 0x6120) != 0 && S32T(g_obj, 0x24c) > 1 && S32(g_ar, A_CFG + 0x4f8) == 0) n_cfg0++;
      }
    }
    h = call_end("scen4");
    log_call(&L, h, ra, f);
  }
  log_done(&L);
  need("NotifyDP code 0", n_code[0]);
  need("NotifyDP code 1", n_code[1]);
  need("NotifyDP code 2", n_code[2]);
  need("NotifyDP code 3", n_code[3]);
  need("NotifyDP other code (no effect)", n_code[4]);
  need("sensitive ISP path", n_sens);
  need("regular ISP path", n_reg);
  need("clamp taken", n_clamp);
  need("clamp not taken", n_noclamp);
  need("imul overflow in the limit", n_ovf);
  need("ctx+0x6120 == 0", n_lazy);
  need("[0x24c] <= 1 with ctx flag", n_le1);
  need("cfg[0x4f8] == 0", n_cfg0);
}

/* ============================================================================
 * scenario 5: InterpretMohMessageBits with an independent model
 * ========================================================================== */
typedef struct {
  int32_t code;       /* [0xabf4] */
  int has_e0, has_e2, has_fa;
  uint16_t e0, e2;
  uint8_t fa;
  int nprint;
  const char *pr[6];
  unsigned int parg[6]; /* argument of the print (when it has one) */
} moh_model_t;
static const char *const k_s_req = "MHreq message detected !\r\n";
static const char *const k_s_frr = "MHfrr message detected !\r\n";
static const char *const k_s_cda = "MHcda message detected !\r\n";
static const char *const k_s_ack = "MHack message detected ! , Time out period code = %d\r\n";
static const char *const k_s_nackm = "MHnack message detected (modem may init MOH in the future) !\r\n";
static const char *const k_s_nackn = "MHnack message detected (modem may NOT init MOH in the future) !\r\n";
static const char *const k_s_in = "MHcld message detected ! , disconnect reason is incoming call\r\n";
static const char *const k_s_out = "MHcld message detected ! , disconnect reason is outgoing call\r\n";
static const char *const k_s_oth = "MHcld message detected ! , disconnect reason is other reason\r\n";
static const char *const k_s_res = "MHcld message detected ! , reserved disconnect reason (assume other reason)\r\n";
static const char *const k_s_eq = "=========================================================================\r\n";
static const char *const k_s_ill = "Illegal MH message detected !!! (rxsq[0] = %X)\r\n";
static const char *const k_s_frc = "forcing message type to MH NACK\r\n";
/* the model sees the level sequence the real code sees: `levels` = what dsplibs_debug_level is at each read, in order; since
 * the hook may change the level after print k, the model replays it */
static void model_moh(unsigned int w, moh_model_t *m, unsigned int lv0, int hook_k, unsigned int hook_lvl) {
  unsigned int lv = lv0;
  int np = 0;
  memset(m, 0, sizeof *m);
#define MPRINT(S, A)                                                         \
  do {                                                                       \
    m->pr[np] = (S);                                                         \
    m->parg[np] = (A);                                                       \
    if (hook_k >= 0 && np == hook_k) lv = hook_lvl;                          \
    np++;                                                                    \
  } while (0)
  if (w == 0x33) { if (lv > 1) MPRINT(k_s_req, 0); m->code = 0; }
  else if ((w & 0xf0) == 0x50) { m->has_e0 = 1; m->e0 = (uint16_t)(w & 0xf); if (lv > 1) MPRINT(k_s_ack, w & 0xf); m->code = 4; }
  else if (w == 0x77) { if (lv > 1) MPRINT(k_s_nackm, 0); m->code = 5; }
  else if (w == 0x75) { if (lv > 1) MPRINT(k_s_nackn, 0); m->has_e2 = 1; m->e2 = 3; m->code = 5; }
  else if ((w & 0xf0) == 0x90) {
    unsigned int c = w & 0xf;
    m->has_fa = 1;
    if (c == 5) { m->fa = 0; if (lv > 1) MPRINT(k_s_in, 0); }
    else if (c == 6) { m->fa = 1; if (lv > 1) MPRINT(k_s_out, 0); }
    else if (c == 10) { m->fa = 2; if (lv > 1) MPRINT(k_s_oth, 0); }
    else { m->fa = 2; if (lv > 1) MPRINT(k_s_res, 0); }
    m->code = 2;
  } else if (w == 0xbb) { if (lv > 1) MPRINT(k_s_cda, 0); m->code = 3; }
  else if (w == 0xdd) { if (lv > 1) MPRINT(k_s_frr, 0); m->code = 1; }
  else {
    /* each print is guarded by the level read right after the previous one (the hook acts inside MPRINT) */
    if (lv > 1) {
      MPRINT(k_s_eq, 0);
      if (lv > 1) {
        MPRINT(k_s_ill, w);
        if (lv > 1) MPRINT(k_s_frc, 0);
      }
    }
    m->code = 5;
    if (lv > 1) MPRINT(k_s_eq, 0);
  }
  m->nprint = np;
#undef MPRINT
}
static void scen5(void) {
  logger_t L = {"moh_interpret", FNV0, 0, 4093};
  long arm[10] = {0}, n_lv = 0, n_hook = 0, n_aliasmsg = 0;
  int i;
  for (i = 0; i < 140000; i++) {
    unsigned int h, w, lv;
    uint16_t *msg = (uint16_t *)(g_ar + A_MSG);
    moh_model_t m;
    int k;
    fresh();
    lv = pick_level();
    dsplibs_debug_level = lv;
    pick_hook();
    /* message word: exhaustive-ish: the table arms plus the 0x5x / 0x9x families with any upper byte, plus random */
    {
      unsigned int c = lcg_mod(20);
      if (c < 2) w = 0x33;
      else if (c < 5) w = 0x50 | (lcg_next() & 0x1f0f) | ((lcg_next() & 0x1f00) ? 0 : 0);
      else if (c < 6) w = 0x77;
      else if (c < 7) w = 0x75;
      else if (c < 11) w = 0x90 | (lcg_next() & 0xf) | ((lcg_next() & 1) ? (lcg_next() & 0xff00) : 0);
      else if (c < 12) w = 0xbb;
      else if (c < 13) w = 0xdd;
      else if (c < 15) w = 0x90 | (unsigned)lcg_range(0, 15);
      else w = (unsigned)(lcg_next() & 0xffff);
      if (c >= 2 && c < 5) w = (w & 0xff00u) ? w : (0x50u | (unsigned)lcg_range(0, 15));
    }
    msg[0] = (uint16_t)w;
    msg[1] = (uint16_t)lcg_next(); /* neighbour word: must not matter */
    if (lcg_pct(5)) { /* the message lives INSIDE the object: a print hook / store could clobber it */
      memcpy(g_obj + 0xabe0, &(uint16_t){(uint16_t)w}, 2);
      msg = (uint16_t *)(g_obj + 0xabe0); /* aliases [0xabe0], which the 0x5x arm writes */
      n_aliasmsg++;
    }
    call_begin();
    T_VPcmV34InterpretMohMessageBits(g_obj, msg);
    h = call_end("scen5");
    model_moh(w, &m, lv, g_hook_k, g_hook_lvl);
    /* with the aliased message, the 0x5x arm rewrites the word before the illegal-chain could re-read it; only 0x5x writes it */
    if (S32T(g_obj, 0xabf4) != m.code) die("moh: [0xabf4] differs from the model");
    if (m.has_e0 && U16T(g_obj, 0xabe0) != m.e0) die("moh: [0xabe0] differs from the model");
    if (m.has_e2 && U16T(g_obj, 0xabe2) != m.e2) die("moh: [0xabe2] differs from the model");
    if (m.has_fa && U8T(g_obj, 0xabfa) != m.fa) die("moh: [0xabfa] differs from the model");
    if (g_pl.nl != m.nprint) {
      char b[120];
      snprintf(b, sizeof b, "moh: %d prints, model %d (w=%04x lv=%x hook=%d/%x)", g_pl.nl, m.nprint, w, lv, g_hook_k, g_hook_lvl);
      die(b);
    }
    for (k = 0; k < m.nprint; k++) {
      char exp[240];
      snprintf(exp, sizeof exp, m.pr[k], m.parg[k]);
      if (strcmp(exp, g_pl.line[k]) != 0) die("moh: print text differs from the model");
    }
    {
      int a = (w == 0x33) ? 0 : ((w & 0xf0) == 0x50) ? 1 : (w == 0x77) ? 2 : (w == 0x75) ? 3 : ((w & 0xf0) == 0x90) ? 4 : (w == 0xbb) ? 5 : (w == 0xdd) ? 6 : 7;
      arm[a]++;
      if ((w & 0xf0) == 0x90) arm[(w & 0xf) == 5 ? 8 : (w & 0xf) == 6 ? 9 : 4]++;
      if (lv > 1) n_lv++;
      if (g_hook_k >= 0 && g_pl.nl > g_hook_k) n_hook++;
    }
    log_call(&L, h, w, m.code);
  }
  log_done(&L);
  need("0x33", arm[0]);
  need("0x5x family", arm[1]);
  need("0x77", arm[2]);
  need("0x75", arm[3]);
  need("0x9x family", arm[4]);
  need("0xbb", arm[5]);
  need("0xdd", arm[6]);
  need("illegal message", arm[7]);
  need("0x9x code 5", arm[8]);
  need("0x9x code 6", arm[9]);
  need("level > 1", n_lv);
  need("level hook fired", n_hook);
  need("message aliasing the object", n_aliasmsg);
}

/* ============================================================================
 * scenario 6: Indicate* / SetV90RateReneg / K56Flex*  (preinitdigital / v34setuptxmit recorders)
 * ========================================================================== */
static void scen6(void) {
  logger_t L = {"indicate", FNV0, 0, 997};
  long n_f[10] = {0}, n_trn[5] = {0}, n_k10 = 0, n_knot10 = 0, n_neg35a4 = 0, n_rrn0 = 0, n_rrnn = 0, n_sz0 = 0, n_szn = 0;
  int i;
  for (i = 0; i < 100000; i++) {
    unsigned int h;
    int f = i % 7;
    long ra = 0, rb = 0;
    fresh();
    dsplibs_debug_level = pick_level();
    pick_hook();
    switch (f) {
      case 0: { /* SetV90RateReneg(rrn, size) */
        int rrn = lcg_pct(30) ? 0 : (int)(lcg_next() & 0xffff);
        int sz = lcg_pct(30) ? 0 : (int)(lcg_next() & 0xff);
        if (lcg_pct(10)) { rrn = 0x10000; sz = 0x100; }
        call_begin();
        T_VPcmV34SetV90RateReneg(g_obj, garb16(rrn), garb8(sz));
        ra = rrn; rb = sz;
        if ((rrn & 0xffff) == 0) n_rrn0++; else n_rrnn++;
        if ((sz & 0xff) == 0) n_sz0++; else n_szn++;
        break;
      }
      case 1: {
        int a = lcg_pct(30) ? 0 : (int)(lcg_next() & 0xff), b = lcg_pct(40) ? 0 : (int)(lcg_next() & 0xff);
        call_begin();
        T_V34XF_IndicateJdReceived(g_obj, garb8(a), garb8(b));
        ra = a; rb = b;
        break;
      }
      case 2: {
        int a = lcg_pct(40) ? 0 : (int)(lcg_next() & 0xff);
        call_begin();
        T_V34XF_IndicateDilReceived(g_obj, garb8(a));
        ra = a;
        break;
      }
      case 3: { /* Trn2d: steer around 10, 14, 17 */
        static const int32_t sp[] = {9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 0, -1, 0x7fffffff, (int32_t)0x80000000, 5, 12};
        int32_t v = lcg_pct(80) ? sp[lcg_mod(sizeof sp / sizeof sp[0])] : (int32_t)lcg_next();
        S32T(g_obj, 0x24c) = v;
        call_begin();
        T_V34XF_IndicateTrn2dReceived(g_obj);
        ra = v; rb = S32T(g_obj, 0x24c);
        n_trn[v < 10 ? 0 : v < 15 ? 1 : v < 18 ? 2 : 3]++;
        break;
      }
      case 4: {
        int a = lcg_pct(40) ? 0x10 : lcg_pct(30) ? 0x11 : (int)(lcg_next() & 0xff);
        if (lcg_pct(10)) a = 0x110;
        call_begin();
        T_V34XF_IndicateK56FlexJdReceived(g_obj, garb8(a));
        ra = a;
        if ((a & 0xff) == 0x10) n_k10++; else n_knot10++;
        break;
      }
      case 5:
        call_begin();
        T_V34XF_IndicateK56FlexRateDetermined(g_obj);
        break;
      default: { /* K56FlexJd with extreme [0x35a4] / 0x3596 (the recorder scribbles them again) */
        S16T(g_obj, 0x35a4) = (int16_t)(lcg_pct(50) ? (int)lcg_range(-300, 300) : (int)(lcg_next() & 0xffff));
        if (S16T(g_obj, 0x35a4) < 0) n_neg35a4++;
        U16T(g_obj, 0x3596) = lcg_pct(40) ? 0x12 : (uint16_t)lcg_next();
        call_begin();
        T_V34XF_IndicateK56FlexJdReceived(g_obj, 0x10);
        break;
      }
    }
    n_f[f]++;
    h = call_end("scen6");
    log_call(&L, h, ra, rb);
  }
  log_done(&L);
  need("SetV90RateReneg", n_f[0]);
  need("rrn == 0", n_rrn0);
  need("rrn != 0", n_rrnn);
  need("constel size == 0", n_sz0);
  need("constel size != 0", n_szn);
  need("IndicateJdReceived", n_f[1]);
  need("IndicateDilReceived", n_f[2]);
  need("Trn2d < 10", n_trn[0]);
  need("Trn2d 10..14", n_trn[1]);
  need("Trn2d 15..17", n_trn[2]);
  need("Trn2d > 17", n_trn[3]);
  need("K56FlexJd a == 0x10", n_k10);
  need("K56FlexJd a != 0x10", n_knot10);
  need("K56FlexRateDetermined", n_f[5]);
  need("negative [0x35a4]", n_neg35a4);
}

/* ============================================================================
 * scenario 7: InitiateRateRenegotiation / InitiateHangUp (v34handshakinit recorder)
 * ========================================================================== */
static void scen7(void) {
  logger_t L = {"initiate", FNV0, 0, 997};
  long n_code[8] = {0}, n_m12 = 0, n_other = 0, n_hu12 = 0, n_hu_other = 0, n_stored = 0, n_notstored = 0, n_hi = 0;
  int i;
  for (i = 0; i < 100000; i++) {
    unsigned int h;
    int f = i % 3;
    long ra = 0;
    fresh();
    dsplibs_debug_level = pick_level();
    pick_hook();
    {
      unsigned int c = lcg_mod(10);
      S32T(g_obj, 0x0) = (c < 3) ? 1 : (c < 5) ? 2 : (c < 6) ? 0 : (c < 8) ? (int32_t)lcg_range(3, 7) : (int32_t)steer_mode();
    }
    if (f == 2) { /* HangUp */
      call_begin();
      T_VPcmV34InitiateHangUp(g_obj);
      if ((uint32_t)(S32T(g_obj, 0x0) - 1) <= 1u || (g_ntr == 0)) n_hu12++; else n_hu_other++;
      if (dsplibs_debug_level > 1u) n_hi++;
    } else {
      int code;
      unsigned int c = lcg_mod(10);
      code = (c < 7) ? (int)lcg_range(0, 6) : (c < 8) ? -1 : (int)lcg_next();
      /* bounds [0x220] <= [0x228]-1 etc. : steer around the three compare boundaries */
      {
        int32_t cur = (int32_t)lcg_range(-5, 5);
        S32T(g_obj, 0x228) = cur;
        S32T(g_obj, 0x220) = cur - 1 + (int32_t)lcg_range(-1, 1);
        S32T(g_obj, 0x224) = cur + 1 + (int32_t)lcg_range(-1, 1);
        if (lcg_pct(10)) { S32T(g_obj, 0x228) = (int32_t)0x80000000; }
        if (lcg_pct(10)) { S32T(g_obj, 0x228) = 0x7fffffff; }
      }
      {
        int32_t before = S32T(g_obj, 0x22c);
        call_begin();
        T_VPcmV34InitiateRateRenegotiation(g_obj, code);
        if (S32T(g_obj, 0x22c) != before) n_stored++; else n_notstored++;
      }
      if ((uint32_t)(S32(g_osnap, 0) - 1) <= 1u) n_m12++; else { n_other++; n_code[code >= 0 && code <= 5 ? code : 6]++; }
      ra = code;
    }
    h = call_end("scen7");
    log_call(&L, h, ra, f);
  }
  log_done(&L);
  need("rate-reneg mode 1/2 (ctx chain write)", n_m12);
  need("rate-reneg other modes", n_other);
  for (i = 0; i < 7; i++) {
    char nm[40];
    snprintf(nm, sizeof nm, "rate-reneg code %d", i);
    need(nm, n_code[i]);
  }
  need("rate-reneg [0x22c] stored", n_stored);
  need("rate-reneg [0x22c] not stored", n_notstored);
  need("hang-up mode 1/2 (ctx chain write)", n_hu12);
  need("hang-up other modes (v34handshakinit)", n_hu_other);
  need("hang-up print enabled", n_hi);
}

int main(int argc, char **argv) {
  int s = (argc > 1) ? atoi(argv[1]) : 0;
  g_obj = (unsigned char *)calloc(1, OBJSZ);
  g_osnap = (unsigned char *)calloc(1, OBJSZ);
  g_ar = (unsigned char *)calloc(1, ARSZ);
  g_asnap = (unsigned char *)calloc(1, ARSZ);
  if (!g_obj || !g_osnap || !g_ar || !g_asnap) return 4;
  /* T06: the arena pieces the port dereferences are images of typed classes (ctx / cfg of the V.90 modem object, the engine chain nodes of src/v34/v34_engine_views.h) */
  V34I_REGC(g_ar + A_CTX, 0x6200u, ctx);
  V34I_REGC(g_ar + A_CFG, 0x600u, cfg);
  V34I_REGC(g_ar + A_EB, 0x100u, e90);
  V34I_REGC(g_ar + A_QB, 0x40u, ers);
  V34I_REGC(g_ar + A_RB, 0x40u, erc90); /* RS1 / M3: the V.90 chain ends in V90Mapper, the V.92 one in V92Transmitter (different x86-64 layouts) */
  V34I_REGC(g_ar + A_EA, 0x100u, e92);
  V34I_REGC(g_ar + A_QA, 0x40u, ers);
  V34I_REGC(g_ar + A_RA, 0x40u, erc92);
  V34I_REGC(g_ar + A_EH, 0x300u, edm);
  V34I_REGC(g_ar + A_HB, 0x100u, emg);
  g_lcg ^= (unsigned int)s * 0x9e3779b9u;
  {
    const char *sd = getenv("T10A_SEED");
    if (sd) g_lcg ^= (unsigned int)atoi(sd) * 0x85ebca6bu;
  }
  printf("scenario %d\n", s);
  switch (s) {
    case 0: scen0(); break;
    case 1: scen1(); break;
    case 2: scen2(); break;
    case 3: scen3(); break;
    case 4: scen4(); break;
    case 5: scen5(); break;
    case 6: scen6(); break;
    case 7: scen7(); break;
    default: fprintf(stderr, "bad scenario\n"); return 4;
  }
  fflush(stdout);
  return g_fail ? 2 : 0;
}
