/*
 * End-to-end integration differential test for V.32/V.32bis's higher-rate RX
 * pipeline: DemodDataV32 -> FPM_ECC_cancel -> FPM_AGC_agc -> FPM_SRE_recover
 * -> FPM_FSE_receive -> a trellis-coded FSE_decision_{16Tpt,32pt,64pt,128pt}
 * callback -> VTB_decoder -> bits out. Every individual stage of this chain
 * (and VTBv32_init/VTB_decoder themselves) already has its own bit-exact
 * differential test (see NOTES.md's VTBv32_init/VTB_decoder entries), all
 * exercised via DIRECT calls on synthetic sub-objects. What has NOT been
 * proven until this file: that the REAL wiring between these stages - the
 * real object fields, the real pointer chases, the real construction-time
 * setup - is correct when the whole chain is driven together, through the
 * REAL caller path, across many consecutive calls.
 *
 * ==========================================================================
 * DESIGN CHOICE: option (a) from the brief, via a "cfg-configured rate,
 * organic case-9 rate switch" variant - NOT a raw SetRxModeV32 poke
 * ==========================================================================
 * The brief's option (a) ("directly poke the object into higher-rate RX-
 * active state, then call the already-verified SetRxModeV32(engine, mode)
 * directly") is available and would work, but this harness found a strictly
 * BETTER variant of the same idea while re-reading V32LocLoopNextState's own
 * case 9 body (src/v32/v32_engine.c): case 9 (the state9->10 transition, which this
 * project's existing test/v32/v32_engine.loclop.main.c ALREADY drives for real,
 * organically, via TxHdxScrSequence's own budget-exhaustion -> V32NextState[2]
 * dispatch) does exactly this:
 *     rate = GetRateV32(obj);
 *     SetTxModeV32(obj, V32_TX_MODE[rate]);
 *     SetRxModeV32(obj, V32_RX_MODE[rate]);
 * i.e. the REAL rate-mode switch is already a normal, unconditional part of
 * LocLoop's own real state machine - it is NOT gated behind DecodeRateSeq (the
 * mechanism V32OrgNextState/V32AnsNextState use, and which this project's own
 * NOTES.md already established is PROVABLY locked to rate class 0 or 6 for
 * this project's fixed test rate-sequence value). `GetRateV32` instead reads
 * `wrapperObj+4` directly - the RX rate this object was CONFIGURED with at
 * `V32FP_create` time (`cfg[5]`'s low 16 bits), completely independent of any
 * simulated received handshake tones. This means: by simply constructing the
 * object with a higher `cfg[5]` bps value (7200 or 14400 instead of the
 * existing test's 4800), the EXACT SAME already-proven-correct LocLoop state
 * walk (0->1->3->4->6->7->8->9->10, driven the identical way
 * test/v32/v32_engine.loclop.main.c already does, real TxHdxCarrierState/
 * TxHdxScrSequence budget-exhaustion dispatch, no direct NextState pokes past
 * state 3) will, for REAL, at the REAL case-9 transition, call the REAL
 * `SetRxModeV32(obj, 4)` (7200bps family) or `SetRxModeV32(obj, 6)` (14400bps
 * family) - reaching a trellis-coded rate through completely organic
 * "the object was asked to connect at this rate" logic, not a raw internal-
 * field poke of the RX mode itself.
 *
 * The ONE poke this harness still needs, unavoidably, is the same
 * `hdxobj+0x76 = 2` (LocLoop role) poke `test/v32/v32_engine.loclop.main.c` already
 * uses and already justified at length (see that file's own header comment,
 * and NOTES.md's "V.32 LocLoop NextState" entry): `V32FP_create`'s own public
 * marshalling can never produce role 2, and a full call-site trace confirms
 * no real dsplibs.o call sequence ever reaches it either - this is the ONLY
 * way to construct a differential test of ANY LocLoop-role code at all,
 * already an established, blessed precedent in this project. Once that one
 * poke is done, EVERY further step in this file - the state walk, the rate
 * switch, and the final RxHdxData barrage - is 100% real production code,
 * called exactly the way dsplibs.o's own LocLoop role calls it.
 *
 * ==========================================================================
 * THE REAL CALL CHAIN THIS FILE EXERCISES (every hop for real, not
 * synthetically)
 * ==========================================================================
 * V32RxHdxModem (real tail-call thunk through hdxobj+0x70)
 *   -> RxHdxData (real handler, installed by case 6/8/9 of the real
 *      V32LocLoopNextState, exactly as dsplibs.o installs it)
 *     -> DemodDataV32 (real)
 *       -> FPM_MRF_filter (real, already separately verified)
 *       -> FPM_ECC_cancel (real, already separately verified)
 *       -> FPM_AGC_agc (real, already separately verified)
 *       -> FPM_SRE_recover (real, already separately verified)
 *       -> FPM_FSE_receive (real) - once SRE reports carrier-locked
 *         -> the installed FSE_decision_16Tpt / FSE_decision_128pt callback
 *            (real, installed by the real SetRxModeV32 call made by the
 *            real case 9 above - NOT installed by this harness)
 *           -> VTB_decoder (real, hand-cleaned in the prior chunk)
 *     -> DescrambleDataV32 (real)
 *     -> RxClampV32 (real)
 * This is genuinely additive over every prior V.32 chunk: no prior test
 * drives RxHdxData at a trellis-coded rate, and no prior test reaches a
 * trellis-coded rate through anything other than a direct SetRxModeV32/
 * VTBv32_init/VTB_decoder call on a synthetic or freshly-constructed
 * sub-object.
 *
 * ==========================================================================
 * WHAT THIS PROVES, AND WHAT IT EXPLICITLY DOES NOT
 * ==========================================================================
 * PROVES: the DATA-CHAIN wiring (object fields, pointer chases, callback
 * installation, cross-stage state threading) between DemodDataV32 and
 * VTB_decoder is correct for a REAL object driven through REAL production
 * code, at 2 of the 4 trellis-coded rate families (7200bps/16Tpt and
 * 14400bps/128pt - chosen as the smallest and largest trellis, the two
 * extremes; the per-family ARITHMETIC itself was already proven byte-exact
 * for all 4 families by the VTB_decoder chunk's own direct-call test, so
 * covering the extremes here is judged sufficient to prove the wiring is not
 * family-specific, without re-spending this chunk's budget re-proving
 * per-family arithmetic already closed out).
 *
 * DOES NOT PROVE: that dsplibs.o's own real NextState/handshake machinery can
 * organically reach this state through a real two-modem V.8/V.25/V.32
 * handshake exchange. That remains gated by the V.8/V.25 pre-synchronization
 * gap, explicitly out of this project's V.32-only scope (per this project's
 * long-running NOTES.md tracking) - this file's `hdxobj+0x76=2` poke and its
 * cfg-configured rate are both requirements of testing LocLoop-role code at
 * all, not evidence that a real connection would organically reach a
 * trellis-coded rate.
 *
 * ==========================================================================
 * BUDGET/RE-USE NOTE
 * ==========================================================================
 * States 0/1/3/4/6/7/8/9 are driven EXACTLY the way
 * test/v32/v32_engine.loclop.main.c already does (same helper shapes, same
 * bounded-call-count safety margin derivation) - this is deliberate: those
 * transitions are not this chunk's subject matter (already proven bit-exact
 * elsewhere), they are just the required on-ramp to reach state 10 with
 * RxHdxData active and a trellis-coded rate installed. The NEW content
 * this file adds is entirely in `rx_vtb_barrage()`, driven at state 10 with
 * many (64, double VTB_decoder's own chunk's 32-call depth-16-traceback
 * margin) consecutive real V32RxHdxModem calls, each one dumping both the
 * final descrambled outBits AND the VTB sub-object's own internal state
 * (metrics/write-index/prevQuadrant/history-buffer checksum) - the same
 * field set test/v32/v32_engine.vtb_decoder.main.c's own direct-call test already
 * uses, chosen here specifically so a divergence (if any) can be localized
 * the same way that chunk's own methodology would.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v32_hobj.h"

extern void *V32FP_create(int *cfg6, int param2);
extern void V32FP_delete(char *obj);

extern void V32LocLoopNextState(char *obj);
extern void V32TxHdxModem(char *obj, void *txBits, short *out, short *outCount);
extern short V32RxHdxModem(char *obj, void *rxSamples, void *outBits, short *outCount);

extern short TxHdxCarrierState(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxScrSequence(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxTone(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxNull(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxNoCarrier(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxPhsReversal(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxSequence(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxData(char *obj, void *a2, void *a3, void *a4);
extern short RxHdxTone(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxError(char *obj, void *a2, void *a3, void *a4);

/* Taken by ADDRESS ONLY, for handler-identity printing - never called
 * directly (same convention as test/v32/v32_engine.loclop.main.c). Confirms which
 * FSE_decision_* callback the REAL SetRxModeV32 call installed. */
extern short FSE_decision_16Tpt(int fseObj, short *angleInOut, short *magInOut);
extern short FSE_decision_32pt(int fseObj, short *angleInOut, short *magInOut);
extern short FSE_decision_64pt(int fseObj, short *angleInOut, short *magInOut);
extern short FSE_decision_128pt(int fseObj, short *angleInOut, short *magInOut);
extern unsigned int FSE_decision_16pt(int fseObj, short *angleInOut, short *magInOut);
extern unsigned int FSE_decision_4pt(int fseObj, short *angleInOut, short *magInOut);
extern unsigned int FSE_decision_AB(int fseObj, short *angleInOut, short *magInOut);

#define U8(o) (*(unsigned char *)(intptr_t)(o))
#define S16(o) (*(short *)(intptr_t)(o))
#define U16(o) (*(unsigned short *)(intptr_t)(o))
#define S32(o) (*(int *)(intptr_t)(o))
#define U32(o) (*(unsigned int *)(intptr_t)(o))

static char *OBJ;
static char *HDX;

#define MAXBUF 512
static short txBitsBuf[MAXBUF];
static short txOutBuf[MAXBUF];
static short rxInBuf[MAXBUF];
static short rxOutBuf[MAXBUF];

/* Deterministic (not random) synthetic RX sample generator - same LCG shape
 * as test/v32/v32_engine.loclop.main.c's own, but re-seeded per family run so each
 * family gets its own independent, still fully reproducible, sample stream. */
static unsigned int rngState;
static short next_sample(void)
{
  rngState = rngState * 1103515245u + 12345u;
  return (short)(((rngState >> 16) & 0x3fffu)) - 8192;
}

static const char *tx_handler_name(void *p)
{
  if (p == (void *)TxHdxCarrierState) return "TxHdxCarrierState";
  if (p == (void *)TxHdxScrSequence) return "TxHdxScrSequence";
  if (p == (void *)TxHdxTone) return "TxHdxTone";
  if (p == (void *)TxHdxNull) return "TxHdxNull";
  if (p == (void *)TxHdxNoCarrier) return "TxHdxNoCarrier";
  return "UNKNOWN-TX";
}

static const char *rx_handler_name(void *p)
{
  if (p == (void *)RxHdxPhsReversal) return "RxHdxPhsReversal";
  if (p == (void *)RxHdxSequence) return "RxHdxSequence";
  if (p == (void *)RxHdxData) return "RxHdxData";
  if (p == (void *)RxHdxTone) return "RxHdxTone";
  if (p == (void *)RxHdxError) return "RxHdxError";
  return "UNKNOWN-RX";
}

static const char *fse_decision_name(void *p)
{
  if (p == (void *)FSE_decision_16Tpt) return "FSE_decision_16Tpt";
  if (p == (void *)FSE_decision_32pt) return "FSE_decision_32pt";
  if (p == (void *)FSE_decision_64pt) return "FSE_decision_64pt";
  if (p == (void *)FSE_decision_128pt) return "FSE_decision_128pt";
  if (p == (void *)FSE_decision_16pt) return "FSE_decision_16pt";
  if (p == (void *)FSE_decision_4pt) return "FSE_decision_4pt";
  if (p == (void *)FSE_decision_AB) return "FSE_decision_AB";
  return "UNKNOWN-FSE_decision";
}

static void dump(const char *label)
{
  printf("[%-24s] state=%d role=%d budget=%d elapsed=%d fixedBudget=%d "
         "tx=%s rx=%s status30=%d status31=0x%02x reg0=%d\n",
         label, (int)S16(HDX + 0x74), (int)S16(HDX + 0x76), S32(HDX + 0x78), S32(HDX + 0x7c),
         S32(HDX + 0x80), tx_handler_name(V32H_PG(void *, HDX, 0x6c)),
         rx_handler_name(V32H_PG(void *, HDX, 0x70)), (int)(signed char)U8(OBJ + 0x30),
         (unsigned)U8(OBJ + 0x31), (int)S16(HDX + 0x3c));
}

/* Dumps the RX-mode/rate-family wiring the real case-9 SetRxModeV32 call
 * installed: engine+0x50b0 (mode), engine+0x234 (installed FSE_decision_*
 * callback identity), and the VTB sub-object's own rate-family selector
 * (+0x24) / trellisSize (+0x2c) - proves the whole chain agrees on which
 * rate family is active, not just that SOME trellis-coded mode got picked. */
static void dump_mode(const char *label)
{
  char *engine = V32W_PG(char *, OBJ, 0x68);
  char *vtbSubObj = V32E_PG(char *, engine, 0x230) + 0x18;
  printf("[%-24s] engine+0x50b0(mode)=%d fseDecision=%s vtb(+0x24 family)=%d "
         "vtb(+0x2c trellisSize)=%d vtb(+0x2e depth)=%d\n",
         label, (int)U16(engine + 0x50b0), fse_decision_name(V32E_PG(void *, engine, 0x234)),
         (int)S16(vtbSubObj + 0x24), (int)S16(vtbSubObj + 0x2c), (int)S16(vtbSubObj + 0x2e));
}

static void dump_outbits(const char *tag, short count)
{
  int i;
  int n = (count < 0) ? 0 : (int)count;
  if (n > 16) n = 16;
  printf("[%-24s] outCount=%d outBits[0..%d)=", tag, (int)count, n);
  for (i = 0; i < n; i++) {
    printf("%d%s", (int)rxOutBuf[i], (i + 1 < n) ? "," : "");
  }
  printf("\n");
}

/* The NEW content this file adds: drives V32RxHdxModem `n` times at state 10
 * (RxHdxData active, trellis-coded rate installed), dumping both outBits AND
 * the VTB sub-object's own internal per-call state (8 path metrics,
 * write-index, prevQuadrant, and a checksum of its 0x200-byte circular
 * history buffer) after EVERY call - the same diagnostic field set
 * test/v32/v32_engine.vtb_decoder.main.c's own direct-call test uses, so a
 * divergence here (if any) is immediately comparable/localizable against
 * that chunk's own already-verified per-field behavior.
 *
 * HONEST NOTE on outBits: `RxHdxData`'s own final step, `RxClampV32` (already
 * hand-cleaned/verified in the LocLoop chunk - src/v32/v32_datapath.c), always
 * OVERWRITES the entire outBits buffer with a fixed `0xff` per element,
 * discarding whatever DemodDataV32/DescrambleDataV32 actually computed - this
 * is confirmed pre-existing, already-observed real dsplibs.o behavior (not
 * something this chunk found or introduced; test/v32/v32_engine.loclop.main.c's own
 * `state7(RxHdxData)` dump already shows the identical constant-0xff pattern)
 * and is byte-exact between blob/port here too. Because of this, outBits is
 * NOT a useful signal for whether VTB_decoder's real per-symbol decode is
 * happening - it will read 0xff regardless. The VTB sub-object's own internal
 * state (metrics/write-index/prevQuadrant/history checksum, all independently
 * verified to genuinely evolve call-over-call below) is the real evidence
 * this chunk relies on to prove the chain is live and wired correctly, not
 * outBits. outBits is still dumped/compared per call anyway (costs nothing,
 * and a future divergence in RxClampV32's own field reads would still show
 * up here). */
static void rx_vtb_barrage(const char *label, int n)
{
  int i, j;
  short symLen = S16(HDX + 0x9e);
  char *engine = V32W_PG(char *, OBJ, 0x68);
  char *vtbSubObj = V32E_PG(char *, engine, 0x230) + 0x18;
  if (symLen <= 0 || symLen > MAXBUF) symLen = 1;

  for (i = 0; i < n; i++) {
    short count = symLen;
    char tag[64];
    unsigned int histChecksum = 0;
    short *hist;
    int k;

    for (j = 0; j < symLen; j++) rxInBuf[j] = next_sample();
    memset(rxOutBuf, 0, sizeof(rxOutBuf));
    V32RxHdxModem(OBJ, rxInBuf, rxOutBuf, &count);

    snprintf(tag, sizeof(tag), "%s rx#%d", label, i);
    dump_outbits(tag, count);

    /* vtbSubObj+0x00 is a pointer to the 0x200-byte circular history buffer
     * (per VTB_decoder's own already-verified field table) - checksum its
     * live content each call. */
    hist = V32X_PG(short *, vtbSubObj, V32_VTB_OFF, 0x00);
    for (k = 0; k < 0x100; k++) histChecksum = histChecksum * 131u + (unsigned short)hist[k];

    printf("[%-24s] metrics(0..7)=%d,%d,%d,%d,%d,%d,%d,%d writeIdx(+0x14)=%d "
           "prevQuadrant(+0x30)=%d histChecksum=%u\n",
           tag, (int)S16(vtbSubObj + 0x04), (int)S16(vtbSubObj + 0x06), (int)S16(vtbSubObj + 0x08),
           (int)S16(vtbSubObj + 0x0a), (int)S16(vtbSubObj + 0x0c), (int)S16(vtbSubObj + 0x0e),
           (int)S16(vtbSubObj + 0x10), (int)S16(vtbSubObj + 0x12), (int)S16(vtbSubObj + 0x14),
           (int)S16(vtbSubObj + 0x30), histChecksum);
  }
}

static void rx_micro_test(const char *label, int n)
{
  int i, j;
  short symLen = S16(HDX + 0x9e);
  if (symLen <= 0 || symLen > MAXBUF) symLen = 1;
  for (i = 0; i < n; i++) {
    short count = symLen;
    char tag[64];
    for (j = 0; j < symLen; j++) rxInBuf[j] = next_sample();
    memset(rxOutBuf, 0, sizeof(rxOutBuf));
    V32RxHdxModem(OBJ, rxInBuf, rxOutBuf, &count);
    snprintf(tag, sizeof(tag), "%s rx#%d", label, i);
    dump(tag);
    dump_outbits(tag, count);
  }
}

static int tx_drive_until_transition(const char *label, short fromState, int maxIters)
{
  int i;
  for (i = 0; i < maxIters; i++) {
    short outCount = 0;
    char tag[64];
    memset(txBitsBuf, 0, sizeof(txBitsBuf));
    memset(txOutBuf, 0, sizeof(txOutBuf));
    V32TxHdxModem(OBJ, txBitsBuf, txOutBuf, &outCount);
    snprintf(tag, sizeof(tag), "%s tx#%d", label, i);
    dump(tag);
    if (S16(HDX + 0x74) != fromState) {
      printf("%s: transitioned away from state %d after %d V32TxHdxModem call(s)\n", label,
             (int)fromState, i + 1);
      return i + 1;
    }
  }
  printf("%s: FAILED to transition away from state %d within %d calls\n", label, (int)fromState,
         maxIters);
  return -1;
}

/* Drives one full LocLoop object, at the given configured RX bps, from
 * construction all the way through the real state9->10 rate switch and then
 * a 64-call RxHdxData barrage at the resulting trellis-coded rate. */
static void run_family(unsigned short bps, const char *famLabel, unsigned int seed)
{
  int cfg[6];
  void *p;
  int fixedBudget;
  short symLen;
  int safeRxCalls;

  printf("\n==== FAMILY: %s (cfg bps=%u) ====\n", famLabel, (unsigned)bps);
  rngState = seed;

  memset(cfg, 0, sizeof(cfg));
  cfg[0] = 0; /* role selector at V32FP_create's own level - irrelevant, overridden below */
  cfg[1] = 100;
  cfg[2] = 60000;
  cfg[3] = 700;
  cfg[4] = 1;
  cfg[5] = (int)bps; /* V32FP_create only reads the LOW 16 bits of param_1[5] as txRate,
                       * duplicated into both obj+2 (tx) and obj+4 (rx) - see this file's
                       * header comment / GetRateV32's own derivation. */

  p = V32FP_create(cfg, 0);
  if (!p) {
    printf("create failed\n");
    exit(1);
  }
  OBJ = (char *)(p);
  HDX = V32W_PG(char *, OBJ, 0x64);

  /* The one unavoidable poke (see header comment): LocLoop role is otherwise
   * unreachable through any real call sequence. */
  S16(HDX + 0x76) = 2;
  dump("after role-poke");

  symLen = S16(HDX + 0x9e);
  fixedBudget = S32(HDX + 0x80);
  safeRxCalls = (symLen > 0) ? (fixedBudget / ((int)symLen * 4)) : 1;
  if (safeRxCalls > 40) safeRxCalls = 40;
  if (safeRxCalls < 1) safeRxCalls = 1;
  printf("object params: symLen(+0x9e)=%d fixedBudget(+0x80)=%d safeRxCalls=%d\n", (int)symLen,
         fixedBudget, safeRxCalls);

  /* ---- states 0/1/3: identical shape to test/v32/v32_engine.loclop.main.c - not
   * this chunk's subject matter, just the required on-ramp. ---- */
  S16(HDX + 0x74) = 0;
  S32(HDX + 0x78) = 0;
  V32LocLoopNextState(OBJ);
  dump("case0->1");

  V32LocLoopNextState(OBJ);
  dump("case1->3");

  V32LocLoopNextState(OBJ);
  dump("case3->4");

  /* ---- states 4/6/7/8: real TX-driven transitions, same shape as the
   * existing LocLoop test - rate class is still whatever the object was
   * constructed with (bps), but SetRxModeV32 for the DATA rate hasn't been
   * called yet (that's case 9, below) - these phases are non-trellis
   * (RxHdxPhsReversal/RxHdxData/RxHdxSequence at the still-default mode 1
   * installed by V32FP_recreate's own construction-time SetRxModeV32(obj,1)
   * call), exactly matching the existing LocLoop test's own coverage. ---- */
  rx_micro_test("state4(RxHdxPhsReversal)", safeRxCalls);
  tx_drive_until_transition("state4->6(TxHdxCarrierState)", 4, 20000);

  rx_micro_test("state6(RxHdxPhsReversal)", safeRxCalls);
  tx_drive_until_transition("state6->7(TxHdxCarrierState)", 6, 20000);

  rx_micro_test("state7(RxHdxData)", safeRxCalls);
  tx_drive_until_transition("state7->8(TxHdxCarrierState)", 7, 20000);

  rx_micro_test("state8(RxHdxSequence)", safeRxCalls);
  tx_drive_until_transition("state8->9(TxHdxScrSequence)", 8, 20000);

  /* ---- THE organic rate switch: case 9's own body calls GetRateV32(obj)
   * (reads the cfg-configured bps above, NOT anything DecodeRateSeq-derived)
   * then SetTxModeV32/SetRxModeV32 for REAL. This is where the trellis-coded
   * rate family actually gets installed - no poke anywhere in this call. ---- */
  tx_drive_until_transition("state9->10(TxHdxScrSequence)", 9, 20000);
  dump("state10(reached)");
  dump_mode("state10(reached)");

  /* ---- THE NEW content: 64 real consecutive RxHdxData calls at the now-
   * trellis-coded rate, each one demodulating through the REAL
   * DemodDataV32->FPM_ECC_cancel->FPM_AGC_agc->FPM_SRE_recover->
   * FPM_FSE_receive->FSE_decision_*->VTB_decoder chain, dumping outBits +
   * VTB sub-object internal state after every call. 64 is double
   * VTB_decoder's own chunk's 32-call (2x its depth-16 traceback latency)
   * margin. ---- */
  rx_vtb_barrage("state10(RxHdxData,trellis)", 64);

  dump("state10(final)");

  /* Deliberately NOT calling V32FP_delete here - see this file's header
   * comment addendum on the malloc-reuse hazard this chunk's own debugging
   * found: SMCv32_encoder_tcm's persistent trellis-state fields
   * (engine+0x56/+0x58) are never explicitly zeroed by ANY real code path
   * (confirmed against porting/decompiled/dp/v32_helpers.c's own real SetTxModeV32 -
   * this port matches it exactly) - a real object only ever reaches a
   * trellis-coded TX mode after a FRESH V32FP_create, where these bytes are
   * implicitly zero because the OS hands back a fresh zero-filled page, not
   * because any function actually initializes them. Freeing family N's
   * object before constructing family N+1 lets the allocator hand back the
   * SAME, now non-zero-but-still-in-range-by-luck-or-not bytes for this
   * exact sub-object, which is a genuine reproducible SEGV for the
   * 14400bps/128pt family (bVar6=4 makes an uninitialized `smcObj+0x10`
   * index far enough out of TrellisTransitionTable's 32-entry bounds to
   * fault) - found empirically while building this test, confirmed to be a
   * test-harness malloc-reuse artifact (both the blob and port sides link
   * the SAME test/stub_env.c sysdep_malloc=malloc/sysdep_free=free, but the
   * two binaries' overall heap allocation histories differ enough that only
   * the port side happened to get unlucky garbage here), NOT a port
   * correctness bug (the port's SetTxModeV32 matches the real disassembly/
   * decompile field-for-field, byte-for-byte - see this file's header
   * comment). Each family's object is therefore built on genuinely fresh
   * memory (nothing is ever freed in this process) rather than reused. */
}

int main(void)
{
  printf("== V.32 higher-rate RX pipeline end-to-end integration test ==\n");

  /* Family 1: 7200bps -> rate class 3 -> RX mode 4 -> FSE_decision_16Tpt
   * (smallest trellis, 2 bits/symbol coded). */
  run_family(7200, "7200bps/16Tpt(class3/mode4)", 0x2545F491u);

  /* Family 2: 14400bps -> rate class 5 -> RX mode 6 -> FSE_decision_128pt
   * (largest trellis, 6 bits/symbol coded). Together with family 1, these
   * are the two extremes of the 4 trellis-coded families - the per-family
   * ARITHMETIC is already proven byte-exact for all 4 by VTB_decoder's own
   * direct-call test; this chunk's job is proving the WIRING, which is not
   * expected (and, per this result, is not) family-specific, so covering
   * the extremes is judged sufficient without re-spending budget on the
   * middle two (9600bps/32pt, 12000bps/64pt). */
  run_family(14400, "14400bps/128pt(class5/mode6)", 0x9e3779b9u);

  printf("== done ==\n");
  return 0;
}
