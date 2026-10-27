/*
 * Differential test harness for V.32/V.32bis's mid-call rate-renegotiation
 * pair - V32RngInitNextState (initiator, 12 real cases) and
 * V32RngRespNextState (responder, 11 real cases), the last 2 of V.32's 5
 * role-transition NextState functions - plus the THREE genuinely new Hdx
 * handlers this chunk hand-cleans in src/v32/v32_engine.c: TxHdxData (RngResp case 0
 * ONLY), RxHdxSequenceE (RngInit case 4 conditional / RngResp case 1),
 * RxHdxToneData (RngInit case 0 ONLY). Every other handler/helper these two
 * functions call (TxHdxCarrierState, TxHdxScrSequence, RxHdxSequence,
 * RxHdxData, CodeRateSeq/CodeESeq/DecodeRateSeq/RateToSeq/InitGenSequence/
 * InitDetSequence/SetToneDetect/StoreReg/LoadReg/GetSequence) is already
 * hand-cleaned/verified by the LocLoop/Org/Ans chunks and reused as-is.
 *
 * Same one-source/two-binary pattern as every other *_diff_main.c in this
 * project (test/v32/v32_engine.rng.sh): one linked directly against the
 * real dsplibs.o (every symbol under test is a globally-exported ELF symbol,
 * confirmed via `readelf -sW ../dsplibs.o`), one against src/v32/v32_engine.c + its
 * already-verified dependencies.
 *
 * ==========================================================================
 * WHY THIS TEST STAYS AT RATE CLASS 0 AND WHAT THAT MEANS FOR REACHABLE
 * DecodeRateSeq CLASSES (derived from V32_RATE_SEQ[0], not assumed)
 * ==========================================================================
 * This project's whole V.32 hand-clean effort deliberately stays at rate
 * class 0 (4800bps) to avoid the still-unported Viterbi decoder
 * (VTBv32_init/VTB_decoder), reachable from SetRxModeV32's mode 2-6 branches.
 * `V32_RATE_SEQ[0]` (this object's own "locally configured capability word"
 * every CodeRateSeq/CodeESeq/CodeFinalRateSeq/DecodeRateSeq call intersects
 * against) is `3345` = `0xD11` = bits {0,4,8,10,11} set - i.e. bits 0x1, 0x10,
 * 0x100, 0x400, 0x800. NONE of the 5 bits DecodeRateSeq's cascade actually
 * TESTS (8, 0x20, 0x200, 0x40, 0x80) are set in this word EXCEPT 0x400's own
 * sibling test - working through the cascade by hand with local=0xD11 shows
 * the ONLY two class values reachable at rate class 0 are:
 *   - class 0 (the "4800bps, compatible" case) when the far-end/captured
 *     rate-sequence word being decoded has bit 0x400 set,
 *   - class 6 (the "incompatible" sentinel) otherwise (in particular,
 *     whenever the word is 0 - i.e. whenever `GetSequence()`'s own
 *     `engine+0x68` field is still at its `InitDetSequence`-zeroed default,
 *     which is exactly what happens under this harness's synthetic RX noise,
 *     since `DetSequence` never organically matches - the SAME "no organic
 *     match under synthetic noise" finding already established by every
 *     earlier V.32 NextState chunk's own test).
 * `V32_TX_MODE[0]`/`V32_RX_MODE[0]` are both `1` (the safe, VTB-free mode);
 * `V32_TX_MODE[6]`/`V32_RX_MODE[6]` are both `7`. Mode 7 is confirmed SAFE on
 * the TX side (`SetTxModeV32`'s own switch has no `case 7`, so it falls to
 * the already-differentially-verified `default:` branch - src/v32/
 * v32_datapath.c, "all 8 branches... none of these reach VTBv32_init").
 * This harness therefore lets class 6 occur freely wherever `SetTxModeV32`
 * is the consumer, but ALWAYS pins class 0 (via a direct `StoreReg`/field
 * poke of the relevant rate-sequence-word field to `0x400`, the exact same
 * "pin rate class 0" technique `test/v32/v32_engine.org.main.c`/
 * `test/v32/v32_engine.ans.main.c`'s own final phase-groups already established)
 * wherever `SetRxModeV32` is the consumer - so this test NEVER calls
 * SetRxModeV32 with anything other than mode 1, honoring the project's rate-
 * class-0 constraint strictly rather than relying on mode 7 also happening
 * to be safe there (not checked for SetRxModeV32 specifically - SetRxModeV32
 * cases 2-6 are the documented VTBv32_init range, and 7 was never verified
 * either way by the earlier primitives chunk's own test scope, which only
 * exercised modes 0/1/default).
 *
 * ==========================================================================
 * WHY DIRECT FIELD POKES, NOT A "POKE THE ROLE FIELD" HACK
 * ==========================================================================
 * Unlike V32LocLoopNextState's own role value (2, genuinely unreachable
 * through any real call sequence), roles 4 (RngInit) and 5 (RngResp) ARE
 * reachable in principle - via V32FP_control's own RngInit/RngResp
 * "reconnect" branch (`ctlOptByte` bit 0x8 set, bit 0x4 clear) - but that
 * branch is itself still an explicit, out-of-this-chunk's-scope
 * `v32_unported()` stub in `src/v32/v32_engine.c` (a separate, smaller follow-up item,
 * not part of this chunk's brief, which is specifically the two NextState
 * functions themselves and their own new Hdx handlers). This harness
 * therefore uses the SAME already-blessed "build a real object via
 * V32FP_create, then directly poke the role/state field" precedent
 * `test/v32/v32_engine.loclop.main.c` established for role 2 - the only difference
 * being that here it is a scoping convenience (the real entry path exists but
 * isn't hand-cleaned yet), not a structural impossibility.
 *
 * ==========================================================================
 * DRIVING STRATEGY: ORGANIC TX WHERE POSSIBLE, DIRECT CALLS FOR BRANCH PAIRS
 * ==========================================================================
 * Once role/state are poked, EVERY handler either function's own organic
 * path can install is already real (TxHdxCarrierState/TxHdxScrSequence/
 * TxHdxData/RxHdxSequence/RxHdxData/RxHdxSequenceE/RxHdxToneData - the last
 * three hand-cleaned THIS chunk), so - unlike the old LocLoop-era caution
 * about landing in an abort()-stubbed error transition - it is now safe to
 * drive TX organically throughout (`tx_drive_until_transition`, same helper
 * as the Org/Ans/LocLoop harnesses) and let real budget-exhaustion dispatch
 * calls exercise the MAIN-LINE branch of every case for real. Two genuine
 * structural wrinkles, both confirmed by reading the actual case bodies (not
 * assumed by analogy to Org/Ans):
 *   1. RngInit's cases 1 and 3 each have their OWN internal "is the budget
 *      REALLY exhausted yet" re-check (`if (0 < budget) { ...still waiting...
 *      } else { ...advance... }`) - a genuine, disassembly-confirmed
 *      structural difference from every other NextState case in this
 *      project, because `RxHdxToneData` (installed through states 0-3) can
 *      ALSO trigger a dispatch into these cases via its OWN
 *      `FPM_TONE_detect`-based condition, independent of the TX budget. Pure
 *      organic TX driving can only ever reach the "exhausted" branch (since
 *      this harness never drives RX enough to trigger `RxHdxToneData`'s own
 *      early dispatch path) - the "still waiting" branch is exercised via an
 *      explicit direct call with the budget field temporarily forced
 *      positive.
 *   2. Both functions' rate-class-derived branches (RngInit cases 6/7/8/0x20,
 *      RngResp cases 7/8/9/0x20) read a rate-sequence-word field
 *      (`LoadReg(1)`/`GetSequence()`-captured `+0x44`) that NEVER gets
 *      written to anything but 0 by this harness's own organic path (see the
 *      rate-class note above) - so organic driving alone exercises the
 *      class-6 ("incompatible") branch of every one of these all the way to
 *      the respective terminal/CONNECT state, and the class-0 ("compatible")
 *      happy path is separately exercised via a direct reg/field poke plus a
 *      rewind to the relevant case, exactly mirroring
 *      `test/v32/v32_engine.ans.main.c`'s own phase-group-4 precedent for its
 *      analogous reg4 poke.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v32_hobj.h"

extern void *V32FP_create(int *cfg6, int param2);
extern void V32FP_delete(char *obj);

extern void V32RngInitNextState(char *obj);
extern void V32RngRespNextState(char *obj);
extern void V32TxHdxModem(char *obj, void *txBits, short *out, short *outCount);
extern short V32RxHdxModem(char *obj, void *rxSamples, void *outBits, short *outCount);

extern short TxHdxCarrierState(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxScrSequence(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxData(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxNoCarrier(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxTone(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxNull(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxToneData(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxSequence(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxSequenceE(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxData(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxError(char *obj, void *a2, void *a3, void *a4);
extern short RxHdxTone(char *obj, void *a2, void *a3, void *a4);

extern int LoadReg(char *obj, unsigned short idx);
extern void StoreReg(char *obj, short value, unsigned short idx);
extern unsigned int GetSequence(char *obj);

/* Real GLOBAL ELF symbols (readelf -sW) used to compute EXPECTED values
 * rather than hardcoding bare literals - same precedent as
 * test/v32/v32_engine.org.main.c/v32_ans_diff_main.c's own final phase-groups. */
extern const short V32_CONNECT[7];
extern const short V32_TX_MODE[7];
extern const short V32_RX_MODE[7];

#define U8(o) (*(unsigned char *)(intptr_t)(o))
#define S16(o) (*(short *)(intptr_t)(o))
#define U16(o) (*(unsigned short *)(intptr_t)(o))
#define S32(o) (*(int *)(intptr_t)(o))
#define U32(o) (*(unsigned int *)(intptr_t)(o))

static char *OBJ;
static char *HDX;
static char *BIGBUF;

#define MAXBUF 512
static short txBitsBuf[MAXBUF];
static short txOutBuf[MAXBUF];
static short rxInBuf[MAXBUF];
static short rxOutBuf[MAXBUF];

/* Deterministic (not random) synthetic RX sample generator - same
 * generator/seed as every earlier V.32 chunk's own harness. */
static unsigned int rngState = 0x2545F491u;
static short next_sample(void)
{
  rngState = rngState * 1103515245u + 12345u;
  return (short)(((rngState >> 16) & 0x3fffu)) - 8192;
}

static const char *tx_handler_name(void *p)
{
  if (p == (void *)TxHdxCarrierState) return "TxHdxCarrierState";
  if (p == (void *)TxHdxScrSequence) return "TxHdxScrSequence";
  if (p == (void *)TxHdxData) return "TxHdxData";
  if (p == (void *)TxHdxNoCarrier) return "TxHdxNoCarrier";
  if (p == (void *)TxHdxTone) return "TxHdxTone";
  if (p == (void *)TxHdxNull) return "TxHdxNull";
  return "UNKNOWN-TX";
}

static const char *rx_handler_name(void *p)
{
  if (p == (void *)RxHdxToneData) return "RxHdxToneData";
  if (p == (void *)RxHdxSequence) return "RxHdxSequence";
  if (p == (void *)RxHdxSequenceE) return "RxHdxSequenceE";
  if (p == (void *)RxHdxData) return "RxHdxData";
  if (p == (void *)RxHdxError) return "RxHdxError";
  if (p == (void *)RxHdxTone) return "RxHdxTone";
  return "UNKNOWN-RX";
}

static void dump(const char *label)
{
  printf("[%-28s] state=%d role=%d budget=%d elapsed=%d fixedBudget=%d "
         "reg0=%d reg1=%d reg2=%d rateword44=%d hold48=%d tx=%s rx=%s "
         "status30=%d status31=0x%02x bigbuf0=%d bigbuf50=%d bigbuf58=%d\n",
         label, (int)S16(HDX + 0x74), (int)S16(HDX + 0x76), S32(HDX + 0x78), S32(HDX + 0x7c),
         S32(HDX + 0x80), (int)S16(HDX + 0x3c), (int)S16(HDX + 0x3e), (int)S16(HDX + 0x40),
         (int)U16(HDX + 0x44), (int)S16(HDX + 0x48),
         tx_handler_name(V32H_PG(void *, HDX, 0x6c)),
         rx_handler_name(V32H_PG(void *, HDX, 0x70)), (int)(signed char)U8(OBJ + 0x30),
         (unsigned)U8(OBJ + 0x31), S32(BIGBUF + 0), (int)U16(BIGBUF + 0x50),
         (int)U16(BIGBUF + 0x58));
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
  }
}

/* Drives V32TxHdxModem `n` times WITHOUT any transition expectation - used
 * for a plain per-call smoke test of a handler whose own budget is
 * deliberately left large enough that a transition isn't imminent. */
static void tx_smoke_test(const char *label, int n)
{
  int i;
  for (i = 0; i < n; i++) {
    short outCount = 0;
    char tag[64];
    memset(txBitsBuf, 0, sizeof(txBitsBuf));
    memset(txOutBuf, 0, sizeof(txOutBuf));
    V32TxHdxModem(OBJ, txBitsBuf, txOutBuf, &outCount);
    snprintf(tag, sizeof(tag), "%s tx#%d", label, i);
    dump(tag);
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

/* ==========================================================================
 * V32RngInitNextState - initiator side, 12 real cases.
 * ========================================================================== */
static void test_rng_init(void)
{
  int cfg[6];
  void *p;

  memset(cfg, 0, sizeof(cfg));
  cfg[0] = 0; /* obj+0 role bool: Originate side initiates renegotiation */
  cfg[1] = 100;
  cfg[2] = 60000;
  cfg[3] = 700;
  cfg[4] = 1;
  *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16); /* 4800bps, rate class 0 */

  p = V32FP_create(cfg, 0);
  if (!p) {
    printf("create failed (RngInit)\n");
    exit(1);
  }
  OBJ = (char *)(p);
  HDX = V32W_PG(char *, OBJ, 0x64);
  BIGBUF = V32W_PG(char *, OBJ, 0x68);
  /* bigbuf+0x58 and bigbuf+0x50 are genuinely never initialized to a known
   * value by V32FP_recreate (real dsplibs.o's own construction path never
   * touches either - +0x58 only gets written by case 6's/RngResp case 7's
   * own "cls>2" branch, +0x50 only by case 3's/case 4's own role-based
   * write, both conditionally and only once this test's own flow actually
   * reaches them) - a harness-only normalization, same precedent as
   * test/v23_diff_main.c's own uninitialized-txBitSrc fix: without this,
   * `dump()`'s own bigbuf50/bigbuf58 fields read whatever leftover heap
   * bytes this fresh sysdep_malloc(0x50dc) happened to return, which is NOT
   * guaranteed identical between the separately-linked blob/port binaries
   * (confirmed empirically - both diverge for the RngResp object below,
   * before its own test reaches the branches that write them for real). */
  U16(BIGBUF + 0x58) = 0;
  U16(BIGBUF + 0x50) = 0;

  printf("== V.32 RngInit (renegotiation initiator) ==\n");

  /* Resolve the role-entry-path scoping convenience (see header comment):
   * poke curRole=4 directly on a normally-constructed object. */
  S16(HDX + 0x76) = 4;
  S16(HDX + 0x74) = 0;
  dump("after role-poke");

  /* ---- case 0: unconditional, installs TxHdxCarrierState/RxHdxToneData,
   * sets bigbuf+0=1 (carrier-present flag), SetToneDetect ---- */
  V32RngInitNextState(OBJ);
  dump("case0");

  /* Genuine per-call exercise of the newly-hand-cleaned RxHdxToneData while
   * it's the real active RX handler (state 0/1's construction-time-adjacent
   * phase) - whatever FPM_TONE_detect decides for this synthetic noise, both
   * binaries do the identical thing. */
  rx_micro_test("state0-1(RxHdxToneData)", 3);

  /* ---- case 1, "not yet" branch (budget>0): direct-poked, since organic TX
   * driving can only ever reach the "exhausted" branch (see header
   * comment) ---- */
  S16(HDX + 0x74) = 1;
  S32(HDX + 0x78) = 999;
  V32RngInitNextState(OBJ);
  dump("case1(not yet)");

  /* ---- case 1, exhausted branch + case 3, exhausted branch (bit4 clear):
   * driven organically for real via TxHdxCarrierState's own budget
   * exhaustion ---- */
  S32(HDX + 0x78) = 5; /* small, so the organic drive below completes fast */
  tx_drive_until_transition("state1->3(TxHdxCarrierState)", 1, 20000);

  /* ---- case 3, "not yet" branch (budget<1 false): direct-poked ---- */
  {
    short savedState = S16(HDX + 0x74);
    int savedBudget = S32(HDX + 0x78);
    S32(HDX + 0x78) = 999;
    V32RngInitNextState(OBJ);
    dump("case3(not yet)");
    S16(HDX + 0x74) = savedState;
    S32(HDX + 0x78) = savedBudget;
  }

  tx_drive_until_transition("state3->4(TxHdxCarrierState)", 3, 20000);

  /* ---- case 4, "else" branch (bit4 already set): direct-poked, since the
   * organic path always finds bit4 clear (RxHdxSequenceE's own settle path
   * never organically runs long enough to set it) ---- */
  {
    short savedState = S16(HDX + 0x74);
    unsigned char savedFlags = U8(OBJ + 0x31);
    U8(OBJ + 0x31) = (unsigned char)(savedFlags | 4);
    V32RngInitNextState(OBJ);
    dump("case4(bit4 set)");
    S16(HDX + 0x74) = savedState;
    U8(OBJ + 0x31) = savedFlags;
  }

  /* ---- case 4, "if" branch (bit4 clear), BOTH adjust sub-branches:
   * direct-poked (this is genuinely a fresh sub-branch inside the "if" body,
   * not reachable by choosing bit4 alone) ---- */
  {
    short savedState = S16(HDX + 0x74);
    int savedBudget = S32(HDX + 0x78), savedFixed = S32(HDX + 0x80);
    unsigned char savedFlags = (unsigned char)(U8(OBJ + 0x31) & (unsigned char)~4u);
    U8(OBJ + 0x31) = savedFlags;
    S32(HDX + 0x80) = 0x100;
    S32(HDX + 0x78) = 0x50; /* adjust = 0x100-0x50 = 0xb0 > 0x3f */
    V32RngInitNextState(OBJ);
    dump("case4(if, adjust>0x3f)");

    S16(HDX + 0x74) = 4;
    U8(OBJ + 0x31) = savedFlags;
    S32(HDX + 0x80) = 0x100;
    S32(HDX + 0x78) = 0xe0; /* adjust = 0x100-0xe0 = 0x20 <= 0x3f */
    V32RngInitNextState(OBJ);
    dump("case4(if, adjust<=0x3f)");

    S16(HDX + 0x74) = savedState;
    S32(HDX + 0x78) = savedBudget;
    S32(HDX + 0x80) = savedFixed;
  }

  /* Real case4->5 transition: state/budget/flags were all restored to their
   * exact post-organic-transition values by the two side-test blocks above
   * (each saves and restores), so this direct call exercises case4's own
   * "if" (hunting, bit4 clear) path exactly as the real object would from
   * here - the SAME branch the earlier organic "state3->4" drive already
   * left it primed for. */
  V32RngInitNextState(OBJ);
  dump("case4->5");

  /* Genuine per-call exercise of RxHdxSequenceE's own "hunting" per-call body
   * while it's the real active RX handler. */
  rx_micro_test("state5(RxHdxSequenceE)", 3);

  tx_drive_until_transition("state5->6(TxHdxScrSequence)", 5, 20000);

  /* ---- case 6, class-0 happy path: direct reg1 poke + rewind (organic path
   * already gave class 6, see below) ---- */
  {
    short savedState = S16(HDX + 0x74);
    int savedBudget = S32(HDX + 0x78);
    StoreReg(OBJ, 0x400, 1);
    S16(HDX + 0x74) = 6;
    V32RngInitNextState(OBJ);
    dump("case6(class0, reg1=0x400)");
    if (S16(HDX + 0x76) == 4 /* still RngInit's own object, sanity check */) {
      /* leave the class-0 result in place; rewind state back so the organic
       * "class 6" pass below starts from a known state again */
    }
    S16(HDX + 0x74) = savedState;
    S32(HDX + 0x78) = savedBudget;
    StoreReg(OBJ, 0, 1);
  }

  /* ---- case 6/7/8/0x20, driven fully organically with class 6 throughout
   * (reg1 stays 0, per the header comment's own finding) ----
   * IMPORTANT CORRECTION vs. an earlier draft of this test: unlike
   * V32RngRespNextState's own case 7 (which DOES short-circuit to a terminal
   * "incompatible" sub-state 0x1f when cls==6 - see below), RE-CHECKED
   * directly against porting/decompiled/dp/v32_nextstate.c's own text:
   * V32RngInitNextState's cases 6 AND 7 have NO such short-circuit at all -
   * case 6 only conditionally clears bigbuf+0x58 when cls>2 (true for
   * cls=6), and case 7's own +0x44==0/!=0 branch is completely independent
   * of cls (its own SetRxModeV32 call there is hardcoded to mode 1, not
   * derived from cls at all). So class 6 flows harmlessly all the way
   * through to case 8's own SetRxModeV32(V32_RX_MODE[6]=7) call - confirmed
   * SAFE (src/v32/v32_datapath.c's SetRxModeV32: mode 7 falls to the
   * already-differentially-verified `default:` branch, same as every other
   * non-2..6 mode - it does NOT reach VTBv32_init, so no rate-class-0
   * poking is actually needed here for safety, only for the SEPARATE
   * class-0 happy-path coverage below). */
  tx_drive_until_transition("state6->7(TxHdxScrSequence)", 6, 20000);
  tx_drive_until_transition("state7->8(TxHdxScrSequence)", 7, 20000);
  dump("case8(organic, class6)");

  /* case 8 always transitions straight to 0x20 (RngInit's OWN terminal
   * connect state) - no further TX-budget-driven step needed. */
  dump("case0x20(organic, class6/far-end-field=0 -> error branch)");

  /* ---- case 6/7/8/0x20, class-0 happy path: direct reg1/rateword pokes +
   * rewind, exercised end to end via direct calls (the organic run above
   * already consumed this object's terminal state) ---- */
  StoreReg(OBJ, 0x400, 1);
  S16(HDX + 0x74) = 6;
  S32(HDX + 0x78) = 0x18;
  S32(HDX + 0x7c) = 0;
  V32RngInitNextState(OBJ); /* case6: cls=0 -> SetTxModeV32(mode1),
                             * bigbuf+0x58 untouched (cls<=2) */
  dump("case6(class0)");

  U16(HDX + 0x44) = 0;
  V32RngInitNextState(OBJ); /* case7: cls=0 -> +0x44==0 -> installs
                             * RxHdxSequence, state=8, budget=+0x80,
                             * SetRxModeV32(mode 1, hardcoded) */
  dump("case7(class0, +0x44==0)");

  /* ---- case 7, "else" branch (+0x44 != 0): direct-poked ---- */
  {
    int savedBudget = S32(HDX + 0x78);
    U16(HDX + 0x44) = 0x400;
    S16(HDX + 0x74) = 7;
    V32RngInitNextState(OBJ);
    dump("case7(+0x44!=0)");
    U16(HDX + 0x44) = 0;
    S32(HDX + 0x78) = savedBudget;
  }

  /* ---- case 8, class-0 path: GetSequence() reads engine+0x68 - poke it so
   * the captured +0x44 (and thus cls) is 0 here too ---- */
  S16(HDX + 0x74) = 8;
  U32(HDX + 0x68) = 0x400;
  V32RngInitNextState(OBJ);
  dump("case8(class0)");

  /* ---- case 0x20, success branch: both rate-sequence fields already read
   * as class-0-compatible (+0x44 was just set to 0x400 by case8 above; +0x3e
   * needs its own poke since nothing else in this harness ever writes it)
   * ---- */
  U16(HDX + 0x3e) = 0x400;
  V32RngInitNextState(OBJ); /* state is already 0x20, set by case8 above */
  dump("case0x20(success, class0/class0)");

  /* ---- case 0x21: universal budget-refresh ---- */
  S16(HDX + 0x74) = 0x21;
  S32(HDX + 0x78) = -999;
  V32RngInitNextState(OBJ);
  dump("case0x21(direct)");

  V32FP_delete(OBJ);
}

/* ==========================================================================
 * V32RngRespNextState - responder side, 11 real cases.
 * ========================================================================== */
static void test_rng_resp(void)
{
  int cfg[6];
  void *p;

  memset(cfg, 0, sizeof(cfg));
  cfg[0] = 1; /* obj+0 role bool: Answer side responds to renegotiation */
  cfg[1] = 100;
  cfg[2] = 60000;
  cfg[3] = 700;
  cfg[4] = 1;
  *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16); /* 4800bps, rate class 0 */

  p = V32FP_create(cfg, 0);
  if (!p) {
    printf("create failed (RngResp)\n");
    exit(1);
  }
  OBJ = (char *)(p);
  HDX = V32W_PG(char *, OBJ, 0x64);
  BIGBUF = V32W_PG(char *, OBJ, 0x68);
  U16(BIGBUF + 0x58) = 0; /* see test_rng_init()'s own comment on these pokes */
  U16(BIGBUF + 0x50) = 0;

  printf("== V.32 RngResp (renegotiation responder) ==\n");

  S16(HDX + 0x76) = 5;
  S16(HDX + 0x74) = 0;
  dump("after role-poke");

  /* ---- case 0: unconditional, installs TxHdxData/RxHdxSequence,
   * bigbuf+0=1 ---- */
  V32RngRespNextState(OBJ);
  dump("case0");

  /* Genuine per-call exercise of the newly-hand-cleaned TxHdxData while it's
   * the real active TX handler (budget is comfortably large here - set to
   * +0x80 by case0 above - so this smoke test has no risk of also
   * triggering the transition tested separately right below). */
  tx_smoke_test("state0(TxHdxData)-smoke", 3);

  /* Real organic case0->1 transition (TxHdxData's own budget exhaustion). */
  S32(HDX + 0x78) = 5;
  tx_drive_until_transition("state0->1(TxHdxData)", 0, 20000);

  /* ---- case 1: unconditional, installs TxHdxCarrierState/RxHdxSequenceE
   * ---- */
  /* (case 1 already ran organically above as part of the state0->1
   * transition dispatch - no separate direct call needed.) */

  rx_micro_test("state1-3(RxHdxSequenceE)", 3);

  tx_drive_until_transition("state1->3(TxHdxCarrierState)", 1, 20000);

  /* ---- case 3: unconditional (no internal branch, unlike RngInit's own
   * case 3 - confirmed via disassembly, see src/v32/v32_engine.c's own header
   * comment) ---- */
  tx_drive_until_transition("state3->4(TxHdxScrSequence)", 3, 20000);

  tx_drive_until_transition("state4->6(TxHdxScrSequence)", 4, 20000);

  /* ---- case 6: unconditional ---- */
  tx_drive_until_transition("state6->7(TxHdxScrSequence)", 6, 20000);

  /* ---- case 7, class-0 happy path: direct reg1 poke + rewind (organic path
   * below gives class 6) ---- */
  {
    int savedBudget = S32(HDX + 0x78);
    StoreReg(OBJ, 0x400, 1);
    S16(HDX + 0x74) = 7;
    V32RngRespNextState(OBJ);
    dump("case7(class0, reg1=0x400)");
    S16(HDX + 0x74) = 7;
    S32(HDX + 0x78) = savedBudget;
    StoreReg(OBJ, 0, 1);
  }

  /* ---- case 7 organic (class 6, reg1 naturally 0) -> error branch fires,
   * terminal sub-state 0x1f; also exercises the "else" branch's own
   * "cls>2 -> bigbuf+0x58=0" step is NOT taken here since the error branch
   * short-circuits before reaching it - see the direct class-0 test above
   * for that step's own coverage (cls=0 is not >2 either, so bigbuf+0x58 is
   * untouched there too; both are provably safe/inert at rate class 0,
   * ported faithfully regardless). ---- */
  tx_drive_until_transition("state7(TxHdxScrSequence)-organic", 7, 20000);
  dump("case7(organic, class6 -> terminal 0x1f)");

  /* ---- case 8, "if" branch (+0x44==0) + case 9 + case 0x20 success:
   * direct-poked end to end. ---- */
  StoreReg(OBJ, 0x400, 1);
  S16(HDX + 0x74) = 8;
  S32(HDX + 0x78) = 8;
  S32(HDX + 0x7c) = 0;
  U16(HDX + 0x44) = 0;
  V32RngRespNextState(OBJ);
  dump("case8(class0, +0x44==0)");

  /* ---- case 8, "else" branch (+0x44 != 0): direct-poked ---- */
  {
    int savedBudget = S32(HDX + 0x78);
    U16(HDX + 0x44) = 0x400;
    S16(HDX + 0x74) = 8;
    V32RngRespNextState(OBJ);
    dump("case8(+0x44!=0)");
    U16(HDX + 0x44) = 0;
    S32(HDX + 0x78) = savedBudget;
  }

  /* ---- case 9: pin class 0 via engine+0x68 ---- */
  S16(HDX + 0x74) = 9;
  U32(HDX + 0x68) = 0x400;
  V32RngRespNextState(OBJ);
  dump("case9(class0)");

  /* ---- case 0x20 (RngResp's simpler, no-error-branch mirror of RngInit's
   * own case 0x20): +0x44 is already 0x400 (class 0) from case9's own
   * capture above. ---- */
  V32RngRespNextState(OBJ);
  dump("case0x20(class0)");

  /* ---- case 0x21 ---- */
  S16(HDX + 0x74) = 0x21;
  S32(HDX + 0x78) = -999;
  V32RngRespNextState(OBJ);
  dump("case0x21(direct)");

  V32FP_delete(OBJ);
}

int main(void)
{
  test_rng_init();
  test_rng_resp();
  printf("== done ==\n");
  return 0;
}
