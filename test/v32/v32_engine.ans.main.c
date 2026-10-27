/*
 * Differential test harness for V.32/V.32bis's Answer ("role 1") cold-start
 * NextState FSM - phase-group 1 (src/v32/v32_engine.c's first V32AnsNextState hand-
 * clean chunk): V32AnsNextState cases 0-4 + the universal 0x21 budget-
 * refresh case, plus the one genuinely NEW Hdx handler this range needs,
 * `TxHdxTone` (the Answer-role counterpart to role 0's own `TxHdxNull`,
 * previously a project-wide stub since only role-0 objects had exercised
 * this layer so far). `TxHdxCarrierState`/`RxHdxPhsReversal`/`TxHdxNoCarrier`/
 * `RxHdxTone` (already hand-cleaned/verified by the LocLoop/Org chunks) are
 * reused as-is. See src/v32/v32_engine.c's own large header comment (right before
 * `V32AnsNextState`'s definition) for the full case-by-case derivation and
 * the disassembly-confirmed verification of every one of these 5 cases
 * against the real dsplibs.o machine code.
 *
 * PHASE-GROUP 2 (see the "==== PHASE-GROUP 2 ====" section further down):
 * V32AnsNextState cases 6-9, plus `RxHdxNoSignal` (the one genuinely NEW Hdx
 * handler this range needs - and the FIRST time in this whole project
 * `RxHdxNoSignal` is exercised as a real, functioning handler rather than an
 * installed-but-never-called value). See src/v32/v32_engine.c's own large header
 * comment for the full case-by-case derivation, the `FPM_AGC_agc` 4-vs-3-
 * argument resolution, and exactly how cases 6-9 are driven (organic
 * TX-budget dispatch for case 6/8->9, an organic `RxHdxNoSignal` batch +
 * direct `V32AnsNextState(OBJ)` guard-check for case 7 - mirroring case 3/4's
 * own established technique - and a direct substate=8 poke to skip state
 * 0x1d's own explicitly-out-of-scope body).
 *
 * PHASE-GROUP 3 (this chunk's own extension, see the "==== PHASE-GROUP 3
 * ===="  section further down): V32AnsNextState cases 0xa-0x12 (the real
 * TRN/rate-encode/LISTEN segment - mirrors V32OrgNextState's own cases
 * 0xc-0x12 almost exactly, per the case map), plus `RateToSeq` (the one
 * genuinely NEW shared helper this range needs - the 4th and last member of
 * the CodeRateSeq/CodeESeq/CodeFinalRateSeq/DecodeRateSeq rate-negotiation-
 * cascade family). Unlike V32OrgNextState's own phase-group 3, NONE of
 * cases 0xa-0x12 share Org's own case-0x10 "doesn't reset budget" hazard
 * (disassembly-confirmed this chunk: EVERY case in this range explicitly
 * writes a fresh, small, non-negative budget as part of its own transition),
 * so this whole phase-group is driven organically via `tx_drive_until_transition`
 * throughout, no direct-call crash workaround needed. `TxHdxTRN`/
 * `TxHdxScrSequence`/`RxHdxSTone`/`RxHdxEpoch`/`RxHdxRateSequence`/`RxHdxData`
 * (already hand-cleaned/verified by the Org/LocLoop chunks) are reused as-is.
 *
 * PHASE-GROUP 4 (see the "PHASE-GROUP 4" section further down):
 * V32AnsNextState cases 0x15-0x1e, plus `CodeFinalRateSeq` - the
 * rate-negotiation ack/CONNECT tail. At the time this phase-group was
 * originally written, this completed V32AnsNextState's main connect path
 * except for case 0x1d's own loop-back body (needing `TxHdxFinishFrame`,
 * not yet ported then) - **that gap is now closed by PHASE-GROUP 5, see
 * below**. EVERY case in this range is unconditional, so the whole stretch
 * is driven organically via `tx_drive_until_transition`, exactly like
 * phase-group 3. Empirically confirmed (not assumed): `GetSequence()` never
 * organically matches under this test's synthetic noise, so the organic
 * drive alone already exercises the INCOMPATIBLE-rate (class 6) branch all
 * the way to the real terminal CONNECT state. The COMPATIBLE (class 0) happy
 * path is then separately exercised via a direct `reg4` poke (`StoreReg(OBJ,
 * 0x400, 4)`) and a rewind to state 0x1a, the exact same technique
 * `test/v32/v32_engine.org.main.c`'s own final phase-group already established for
 * reg3 - both branches are proven bit-exact against dsplibs.o.
 *
 * PHASE-GROUP 5 (THIS CHUNK, see the "PHASE-GROUP 5" section further down,
 * inserted chronologically BETWEEN the old phase-group 2's case 7->0x1d
 * transition and its own former case-8 direct-poke workaround): case 0x1d's
 * own "local retrain" loop-back body, now hand-cleaned along with
 * `TxHdxFinishFrame` (the ONE handler this case needs, previously not even
 * stubbed). Drives the REAL organic path state 0x1d -> (TxHdxFinishFrame
 * installed, tiny one-symbol budget) -> state 8 -> (case 8's own body fires)
 * -> state 9, replacing the prior chunk's own direct `S16(HDX+0x74)=8` poke
 * that skipped case 0x1d's body entirely. Confirms the loop-back
 * RECONVERGES into the exact same state-8-onward path (TxHdxCarrierState
 * reinstalled, accumulator reset, budget<=0x100, the SAME reg0 register
 * round-trip) that this file's own phase-group 2 already established for
 * cases 8/9 - not just that neither endpoint crashes.
 *
 * ==========================================================================
 * WHY THIS TEST NEEDS NO "POKE THE ROLE FIELD" HACK
 * ==========================================================================
 * Role 1 (Answer) is reached through the REAL, un-hacked `V32FP_create` entry
 * point directly (`cfg[0] = 1`) - `V32FP_create`'s own public marshalling
 * collapses ANY nonzero role argument to a plain boolean (`cb[0] =
 * (param_1[0] != 0)`), so `cfg[0]=1` is exactly what a real Answer-side
 * `v32_create` call passes, no field poke needed anywhere (same "no hack
 * needed" situation the Org chunk's own harness already established for role
 * 0). Per src/v32/v32_engine.c's own V32FP_recreate section, the SAME skip-bit
 * (`obj+0x11 & 4`, always set for any object built via this real entry
 * point, since the caller's own enable-word bit0 is always set - matching
 * `v32_create`'s own real convention) that made role 0's inline construction
 * shortcut provably unreachable does the SAME for role 1's own inline
 * shortcut - so a real Answer object genuinely starts AT substate 0, with
 * `TxHdxTone`/`RxHdxTone` (construction-time defaults, NOT installed by
 * V32AnsNextState itself) as its active handlers and budget(+0x78)=0x1ef0
 * (a role-1-specific construction-time constant, NOT phaseInc - confirmed by
 * reading src/v32/v32_engine.c's own V32FP_recreate "else" branch, the role!=0
 * counterpart to role 0's own `phaseInc` initial budget).
 *
 * ==========================================================================
 * A REAL FINDING WHILE DESIGNING THIS TEST: case 1's own budget-choice guard
 * (`obj+0x11 & 2`) is ALWAYS TRUE for any object built through this real
 * entry point - confirmed by reading the table, not guessed
 * ==========================================================================
 * `V32FP_create` only ever overrides `obj+0x11`'s bit2 (the "skip inline
 * construction shortcut" bit, from the caller's own enable-word bit0) -
 * bit1 (mask 2, the ONE case 1 itself tests: `((obj+0x11 & 2) != 0) ? 0xa60 :
 * 0x100`) is left exactly as `V32_CFG_raw`'s own default value copies it in
 * (src/v32/v32_tables.c: word4 = 0x68b = little-endian bytes {0x8b, 0x06, 0,
 * 0}, so byte17=0x06=0b110 - bit1 IS set in the default table itself). Since
 * nothing in `V32FP_create`/`V32FP_recreate` ever touches bit1 again, EVERY
 * object built through the real public entry point takes case 1's `0xa60`
 * (2656) branch, never the `0x100` (256) one - the `0x100` branch is
 * presumably only reachable if some OTHER real caller ever supplied a
 * different raw cfg table with bit1 clear (never observed in this project's
 * own reverse-engineering so far). This test's own dump() output confirms
 * budget=0xa60 after case 1 fires, rather than asserting either value blind.
 *
 * ==========================================================================
 * A SECOND REAL FINDING: case 2's "else"/continue branch (and therefore
 * cases 3/4/6+) is STRUCTURALLY UNREACHABLE via ANY natural TX-exhaustion-
 * triggered dispatch, confirmed by tracing the shared budget field, not
 * merely a "no natural correlator match" precedent
 * ==========================================================================
 * `TxHdxCarrierState` (the active TX handler throughout substate 2) only
 * ever calls `V32NextState` at the EXACT moment its own budget (+0x78, the
 * SAME field case 2's guard reads) has just gone `<=0` - so ANY natural,
 * TX-exhaustion-triggered arrival at case 2 is GUARANTEED to see
 * `budget<1` true, taking the give-up branch (install `TxHdxNoCarrier`/
 * `RxHdxError`, `status31|=2`, `status30=0x16`, substate=0x21) every single
 * time, unconditionally - this isn't a "synthetic noise doesn't trigger a
 * correlator" probabilistic argument (this project's usual `DetSequence`/
 * `FPM_TONE_find_rev` precedent), it's a structural guarantee following
 * directly from the shared-field invariant. The ONLY way the real protocol
 * ever takes the "else" branch is a genuine EARLY dispatch - some Rx event
 * (a real tone-detect result via `RxHdxTone`, still active RX handler
 * through substate 2) calling `V32NextState` WHILE budget is still positive,
 * before the timeout countdown finishes. Since `RxHdxTone` only ever
 * dispatches on `FPM_TONE_detect` returning EXACTLY 0 - per the Org chunk's
 * own already-confirmed finding that synthetic LCG-noise/square-wave input
 * never produces that result - this test's own organic drive proves the
 * REAL, common give-up path (a real Answer line that never sees a confirmed
 * tone within the window), and this file separately proves case 2's "else"
 * branch (plus cases 3/4) via the established "poke the field, call
 * NextState directly" technique - the ONLY way to exercise that branch at
 * all under synthetic input, exactly like V32OrgNextState's own case 1 guard
 * was proven "both sides straddled" via direct pokes. Before starting that
 * poke sequence, this test also restores TX's own handler pointer back to
 * `TxHdxCarrierState` (which the organic give-up path just replaced with
 * `TxHdxNoCarrier`) - simulating "as if case 2's real early-detect else
 * branch had fired instead of the timeout", which is exactly what the real
 * protocol would have left in place (case 2's own else branch never touches
 * +0x6c at all, so TX would still be whatever case 1 originally installed).
 *
 * Same pattern as every other *_diff_main.c in this project: this ONE source
 * compiles into two binaries (test/v32/v32_engine.ans.sh) - one linked
 * directly against the real dsplibs.o, one against src/v32/v32_engine.c + its
 * already-verified dependencies.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v32_hobj.h"

extern void *V32FP_create(int *cfg6, int param2);
extern void V32FP_delete(char *obj);

extern void V32AnsNextState(char *obj);
extern void V32TxHdxModem(char *obj, void *txBits, short *out, short *outCount);
extern short V32RxHdxModem(char *obj, void *rxSamples, void *outBits, short *outCount);

extern short TxHdxCarrierState(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxTone(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxNoCarrier(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxTRN(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxScrSequence(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxFinishFrame(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxPhsReversal(char *obj, void *a2, void *a3, void *a4);
extern short RxHdxTone(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxNoSignal(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxError(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxData(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxSTone(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxEpoch(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxRateSequence(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxSequence(char *obj, void *a2, void *a3, void *a4);
extern int LoadReg(char *obj, unsigned short idx);
extern void StoreReg(char *obj, short value, unsigned short idx);

/* Real GLOBAL ELF symbol (readelf -sW), used by phase-group 4's own test
 * logic: V32_CONNECT to look up the EXPECTED status30 code for the pinned
 * class-0 happy-path scenario rather than hardcoding a bare literal (same
 * precedent as test/v32/v32_engine.org.main.c's own final phase-group). */
extern const short V32_CONNECT[7];

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

/* Deterministic (not random) synthetic RX sample generator - same
 * generator/seed as the LocLoop/Org chunks' own harnesses. */
static unsigned int rngState = 0x2545F491u;
static short next_sample(void)
{
  rngState = rngState * 1103515245u + 12345u;
  return (short)(((rngState >> 16) & 0x3fffu)) - 8192;
}

static const char *tx_handler_name(void *p)
{
  if (p == (void *)TxHdxCarrierState) return "TxHdxCarrierState";
  if (p == (void *)TxHdxTone) return "TxHdxTone";
  if (p == (void *)TxHdxNoCarrier) return "TxHdxNoCarrier";
  if (p == (void *)TxHdxTRN) return "TxHdxTRN";
  if (p == (void *)TxHdxScrSequence) return "TxHdxScrSequence";
  if (p == (void *)TxHdxFinishFrame) return "TxHdxFinishFrame";
  return "UNKNOWN-TX";
}

static const char *rx_handler_name(void *p)
{
  if (p == (void *)RxHdxPhsReversal) return "RxHdxPhsReversal";
  if (p == (void *)RxHdxTone) return "RxHdxTone";
  if (p == (void *)RxHdxNoSignal) return "RxHdxNoSignal";
  if (p == (void *)RxHdxError) return "RxHdxError";
  if (p == (void *)RxHdxData) return "RxHdxData";
  if (p == (void *)RxHdxSTone) return "RxHdxSTone";
  if (p == (void *)RxHdxEpoch) return "RxHdxEpoch";
  if (p == (void *)RxHdxRateSequence) return "RxHdxRateSequence";
  if (p == (void *)RxHdxSequence) return "RxHdxSequence";
  return "UNKNOWN-RX";
}

static void dump(const char *label)
{
  FpmTone *toneObj = V32H_TONE0(HDX);
  /* bigbuf (obj+0x68) - only needed here for the bigbuf+0x2c/+0x2e
   * "currently CONNECTED tx/rx rate class" check added below (NOT the same
   * struct as hdxobj+0x2c/toneObj above - see this file's/NOTES.md's own
   * "bigbuf+0x2c" writeup for why that's a real, previously-confused
   * offset collision between two distinct structs). */
  char *bigbuf = V32W_PG(char *, OBJ, 0x68);
  printf("[%-28s] state=%d role=%d budget=%d elapsed=%d fixedBudget=%d "
         "latch90=%d revcnt_a8=%d nosig_aa=%d reg0=%d reg1=%d reg2=%d reg4=%d tx=%s rx=%s status30=%d "
         "status31=0x%02x status32=0x%02x "
         "tone(6,8,10)=%u,%u,%d gen(4a,4c,4e,50,52)=%d,%d,%d,%d,%u "
         "bigbuf(2c,2e)=%d,%d\n",
         label, (int)S16(HDX + 0x74), (int)S16(HDX + 0x76), S32(HDX + 0x78), S32(HDX + 0x7c),
         S32(HDX + 0x80), S32(HDX + 0x90), (int)S16(HDX + 0xa8), (int)S16(HDX + 0xaa),
         (int)S16(HDX + 0x3c), (int)S16(HDX + 0x3e), (int)S16(HDX + 0x40), (int)S16(HDX + 0x44),
         tx_handler_name(V32H_PG(void *, HDX, 0x6c)),
         rx_handler_name(V32H_PG(void *, HDX, 0x70)), (int)(signed char)U8(OBJ + 0x30),
         (unsigned)U8(OBJ + 0x31), (unsigned)U8(OBJ + 0x32),
         (unsigned)H_TONE_US(toneObj, 6), (unsigned)H_TONE_US(toneObj, 8),
         (int)H_TONE_S(toneObj, 10), (int)S16(HDX + 0x4a), (int)S16(HDX + 0x4c), (int)S16(HDX + 0x4e),
         (int)S16(HDX + 0x50), (unsigned)U16(HDX + 0x52),
         /* bigbuf+0x2c/+0x2e - written by SetTxModeV32/SetRxModeV32
          * (src/v32/v32_datapath.c, disassembly-confirmed @0x81680/@0x81890
          * in dsplibs.o: `movw $X,0x2c(%edx)`/`movw $X,0x2e(%eax)` where
          * edx/eax == *(obj+0x68) == bigbuf). Both are called repeatedly
          * throughout this test's own cases (0/1's initial mode-0/1 calls,
          * and cases 0x1a/0x1b's own final rate-class SetRxModeV32/
          * SetTxModeV32 switch just before case 0x1c's terminal CONNECT) -
          * this field pair was never compared by ANY
          * prior V.32 differential test before this chunk. */
         (int)S16(bigbuf + 0x2c), (int)S16(bigbuf + 0x2e));
}

/* Drives V32RxHdxModem `n` times with varying synthetic samples at the
 * object's own natural per-call sample count, dumping every call - used for
 * state 4's own small, deliberately-bounded RxHdxPhsReversal batch (same
 * technique/threshold-safety-margin as the Org chunk's own case 4). */
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

/* Drives V32TxHdxModem repeatedly until hdxobj+0x74 (state) changes away
 * from `fromState`. Safe to use through EVERY transition in this chunk's own
 * scope (unlike the Org chunk's own case-4 caution): every handler this
 * range's organic path can possibly install (TxHdxTone, TxHdxCarrierState,
 * TxHdxNoCarrier) is ALREADY hand-cleaned/real, so even a mid-block
 * continuation handing a leftover remainder to a just-installed handler
 * (V32TxHdxModem's own documented behavior) never risks an abort() stub. */
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

int main(void)
{
  int cfg[6];
  void *p;

  memset(cfg, 0, sizeof(cfg));
  cfg[0] = 1; /* role 1 = Answer - the REAL role, no poke needed (collapsed to
               * a plain boolean by V32FP_create's own marshalling). */
  cfg[1] = 100;
  cfg[2] = 60000;
  cfg[3] = 700;
  cfg[4] = 1; /* enable bit0 - matches v32_create's own real convention (sets
               * the skip-bit, per src/v32/v32_engine.c's header comment, so
               * V32FP_recreate's dead role==1 inline shortcut is skipped -
               * the object genuinely starts at substate 0). */
  *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16); /* 4800bps, rate class 0 */

  p = V32FP_create(cfg, 0);
  if (!p) {
    printf("create failed\n");
    return 1;
  }
  OBJ = (char *)(p);
  HDX = V32W_PG(char *, OBJ, 0x64);

  printf("== V.32 Answer NextState, phase-group 1 (cases 0-4 + 0x21) ==\n");
  dump("after create");
  if (S16(HDX + 0x74) != 0 || S32(HDX + 0x78) != 0x1ef0) {
    printf("UNEXPECTED: construction-time state/budget not (0, 0x1ef0) as "
           "expected (state=%d, budget=%d)\n", (int)S16(HDX + 0x74), S32(HDX + 0x78));
  }

  /* ---- case 0 (substate 0 -> 1), driven for real via TxHdxTone's own fixed
   * per-call budget countdown - purely TX-tick-count-driven, no RX/
   * correlator involvement needed at all (see this file's own header
   * comment). ~0x1ef0/12=660 TxHdxTone calls. ---- */
  tx_drive_until_transition("state0->1(TxHdxTone)", 0, 2000);
  if (S16(HDX + 0x74) == 1 && S32(HDX + 0x78) != 0xb4) {
    printf("NOTE: state is 1 but budget=%d, not the expected 0xb4\n", S32(HDX + 0x78));
  }

  /* ---- case 1 (substate 1 -> 2), still driven by TxHdxTone (case 0 does
   * NOT install a new TX handler) - installs TxHdxCarrierState, resets
   * accumulator, InitGenSequence(3,4,2). See this file's own header comment
   * for why budget lands on 0xa60, not 0x100. ---- */
  tx_drive_until_transition("state1->2(TxHdxTone)", 1, 500);
  if (S16(HDX + 0x74) == 2) {
    printf("NOTE: case 1's own budget-choice guard (obj+0x11 & 2) resolved "
           "to budget=%d this run (see this file's header comment for the "
           "expected value and why)\n", S32(HDX + 0x78));
  }

  /* ---- case 2's organic path: TxHdxCarrierState is now the active TX
   * handler. Per this file's own header comment, this is STRUCTURALLY
   * guaranteed to take the give-up branch (install TxHdxNoCarrier/
   * RxHdxError, status31|=2, status30=0x16, substate=0x21) - not a
   * probabilistic "no natural match" situation, a hard guarantee from the
   * shared-budget-field invariant. ---- */
  tx_drive_until_transition("state2->giveup(TxHdxCarrierState)", 2, 2000);
  if ((int)(signed char)U8(OBJ + 0x30) == 0x16 && (U8(OBJ + 0x31) & 2) != 0 &&
      V32H_PG(void *, HDX, 0x6c) == (void *)TxHdxNoCarrier &&
      V32H_PG(void *, HDX, 0x70) == (void *)RxHdxError) {
    printf("CONFIRMED: case 2's organic give-up/timeout path fired for real "
           "(status30=0x16, status31 bit1 set, TxHdxNoCarrier/RxHdxError "
           "installed)\n");
  } else {
    printf("NOTE: case 2's give-up path did not fire as expected "
           "(status30=%d, status31=0x%02x, tx=%s, rx=%s)\n",
           (int)(signed char)U8(OBJ + 0x30), (unsigned)U8(OBJ + 0x31),
           tx_handler_name(V32H_PG(void *, HDX, 0x6c)),
           rx_handler_name(V32H_PG(void *, HDX, 0x70)));
  }
  dump("state2(giveup, settled)");

  /* ==========================================================================
   * DIRECT-POKE BRANCH: case 2's "else" branch + cases 3/4 - see this file's
   * own header comment for why this is the ONLY way to reach them at all.
   * Reuses the SAME already-initialized object (its own sub-objects/config
   * are all still valid and untouched by anything below), exactly like the
   * Org chunk's own rate-class-0 pin technique.
   * ========================================================================== */
  printf("== direct-poke branch: case 2's else + cases 3/4 (the only way to "
         "reach them under synthetic input - see header comment) ==\n");

  /* Restore TX's own handler pointer to TxHdxCarrierState (what the give-up
   * path just replaced with TxHdxNoCarrier) - simulating "as if case 2's
   * real early-detect else branch had fired instead of the timeout", which
   * is exactly what the real protocol would have left in place (case 2's
   * own else branch never touches +0x6c at all). */
  V32H_PS(void *, HDX, 0x6c, (void *)TxHdxCarrierState);

  S16(HDX + 0x74) = 2;
  S32(HDX + 0x78) = 0x50; /* any positive value - guard is budget<1 */
  V32AnsNextState(OBJ);
  dump("case2->3(direct, else branch)");
  if (S16(HDX + 0x74) != 3 || S32(HDX + 0x90) != 1 || S32(HDX + 0x7c) != 0) {
    printf("UNEXPECTED: case 2's else branch did not produce the expected "
           "state=3/latch90=1/accum=0\n");
  }

  /* ---- case 3, "guard not yet met" branch: +0x7c at the 0x8b=139 boundary
   * itself (NOT exceeded) - must be a pure no-op (still substate 3),
   * straddling the boundary exactly like the Org chunk's own case 1 test. ---- */
  S32(HDX + 0x7c) = 0x8b;
  V32AnsNextState(OBJ);
  dump("case3(not yet, direct)");
  if (S16(HDX + 0x74) != 3) {
    printf("UNEXPECTED: case 3 guard fired at the boundary value 0x8b itself "
           "(should need STRICTLY greater)\n");
  }

  /* ---- case 3, "guard passed" branch: +0x7c just ABOVE 0x8b - installs
   * RxHdxPhsReversal, resets accumulator, budget=phaseInc, zeroes +0xa8 for
   * case 4's own guard. ---- */
  S32(HDX + 0x7c) = 0x8c;
  V32AnsNextState(OBJ);
  dump("case3->4(direct)");
  if (S16(HDX + 0x74) != 4 || V32H_PG(void *, HDX, 0x70) != (void *)RxHdxPhsReversal) {
    printf("UNEXPECTED: case 3's guard-pass did not produce state=4 with "
           "RxHdxPhsReversal installed\n");
  }

  /* ---- state 4: RxHdxPhsReversal is now the active RX handler - grow
   * +0xa8 via a small, bounded batch (8 calls - past the 7-call guard
   * threshold, comfortably clear of the not-yet-root-caused 25+-call risk
   * zone flagged by the LocLoop/Org chunks), then a direct V32AnsNextState
   * call re-checks the now-satisfied guard (case 4 has no other
   * precondition - direct-callable safely regardless of how substate 4 was
   * reached, exactly like every other guard-only case in this project). ---- */
  rx_micro_test("state4(RxHdxPhsReversal)", 8);
  V32AnsNextState(OBJ);
  dump("case4->6(direct)");
  if (S16(HDX + 0x74) == 6) {
    if (V32H_PG(void *, HDX, 0x70) == (void *)RxHdxNoSignal &&
        V32H_PG(void *, HDX, 0x6c) == (void *)TxHdxCarrierState) {
      printf("CONFIRMED: case 4's guard-pass installed RxHdxNoSignal at "
             "+0x70 WITHOUT touching +0x6c (TX stays TxHdxCarrierState) - "
             "the confirmed structural difference from V32OrgNextState's own "
             "case 4\n");
    } else {
      printf("NOTE: case 4 reached state 6 but handler installs don't match "
             "expectations (tx=%s, rx=%s)\n",
             tx_handler_name(V32H_PG(void *, HDX, 0x6c)),
             rx_handler_name(V32H_PG(void *, HDX, 0x70)));
    }
  } else {
    printf("UNEXPECTED: substate is %d, not 6, after case 4's guard-pass "
           "direct call\n", (int)S16(HDX + 0x74));
  }

  /* ==========================================================================
   * PHASE-GROUP 2 (this chunk): cases 6-9, plus RxHdxNoSignal - see this
   * file's own header comment and src/v32/v32_engine.c's large header comment (right
   * before V32AnsNextState's own definition) for the full derivation.
   * ========================================================================== */
  printf("== V.32 Answer NextState, phase-group 2 (cases 6-9) ==\n");

  /* ---- case 6 (substate 6 -> 7), driven fully organically: TxHdxCarrierState
   * (still the active TX handler, unchanged since case 1 - the earlier
   * direct-poke branch restored it after case 2's organic give-up path)
   * decrements the fresh budget case 4 just set (round-up-to-even of the old
   * accumulator) every TX tick, dispatching to V32NextState the instant it
   * hits <=0 - the SAME organic TX-budget-exhaustion technique already used
   * for cases 0->1/1->2. ---- */
  tx_drive_until_transition("state6->7(TxHdxCarrierState)", 6, 2000);
  if (S16(HDX + 0x74) == 7) {
    if (S32(HDX + 0x78) == S32(HDX + 0x80) && S16(HDX + 0xaa) == 0) {
      printf("CONFIRMED: case 6's guard-pass organically fired (budget "
             "refreshed to fixedBudget, +0xaa zeroed for case 7's own "
             "guard)\n");
    } else {
      printf("NOTE: case 6 reached state 7 but budget/+0xaa don't match "
             "expectations (budget=%d fixedBudget=%d nosig_aa=%d)\n",
             S32(HDX + 0x78), S32(HDX + 0x80), (int)S16(HDX + 0xaa));
    }
  } else {
    printf("UNEXPECTED: substate is %d, not 7, after driving case 6\n",
           (int)S16(HDX + 0x74));
  }

  /* ---- state 7: RxHdxNoSignal is now the active RX handler (installed by
   * case 4, first genuinely EXERCISED for real starting here) - grow its own
   * +0xaa via a bounded, organic batch of real V32RxHdxModem calls (SAME
   * technique as state 4's own RxHdxPhsReversal batch). symLen=12/call, so 7
   * calls lands at 84<=95 (guard boundary, NOT exceeded - checked explicitly,
   * straddling it exactly like case 3's own 0x8b boundary test) and an 8th
   * call lands at 96>95 (guard exceeded). Since nothing here drives a TX tick
   * while in state 7 (TX budget was just freshly reset to
   * fixedBudget=~12922 by case 6's own transition - far too large to exhaust
   * organically in a bounded test) and RxHdxNoSignal itself only calls
   * V32NextState when a tone is actually detected (never true under
   * synthetic input, per this project's established precedent), a direct
   * V32AnsNextState(OBJ) call is needed to actually CHECK the guard - the
   * SAME "grow the real accumulator for real, then call NextState directly to
   * evaluate the guard" technique already established for cases 3/4. ---- */
  rx_micro_test("state7(RxHdxNoSignal,pre-boundary)", 7);
  V32AnsNextState(OBJ);
  dump("case7(not yet, direct)");
  if (S16(HDX + 0x74) != 7) {
    printf("UNEXPECTED: case 7's guard fired early, at +0xaa=%d (should need "
           "STRICTLY greater than 0x5f=95)\n", (int)S16(HDX + 0xaa));
  }

  rx_micro_test("state7(RxHdxNoSignal,past-boundary)", 1);
  /* A REAL FINDING: unlike RxHdxTone's own well-established "detected==0
   * (tone lost) never organically fires under synthetic LCG-noise input"
   * precedent, RxHdxNoSignal's own inner dispatch is gated on the OPPOSITE
   * condition (`detected != 0`, i.e. "signal has returned") - and this
   * chunk's own testing found THIS condition DOES organically fire under
   * synthetic input once elapsed exceeds 0x3c=60, so the guard-pass above
   * (+0xaa>0x5f) is very likely to have ALREADY been organically evaluated
   * and dispatched from INSIDE the last RxHdxNoSignal call itself (state may
   * already be 0x1d here) - only fall back to the direct-call guard-check
   * technique if that didn't happen. Either way, both the port and the real
   * dsplibs.o see the IDENTICAL deterministic synthetic sample sequence, so
   * this is safe and still bit-exact-comparable regardless of which path
   * actually fires. */
  if (S16(HDX + 0x74) == 7) {
    V32AnsNextState(OBJ);
  }
  dump("case7->0x1d(direct-or-organic)");
  if (S16(HDX + 0x74) == 0x1d) {
    if (V32H_PG(void *, HDX, 0x6c) == (void *)TxHdxNoCarrier &&
        V32H_PG(void *, HDX, 0x70) == (void *)RxHdxData &&
        (int)(signed char)U8(OBJ + 0x30) == 0xf &&
        (U8(OBJ + 0x31) & 0x20) != 0 && (U8(OBJ + 0x32) & 8) != 0) {
      printf("CONFIRMED: case 7's guard-pass organically fired (status "
             "pattern written, state=0x1d, TxHdxNoCarrier/RxHdxData "
             "installed) - state 0x1d's own body is driven for real in "
             "phase-group 5 below (see header comment)\n");
    } else {
      printf("NOTE: case 7 reached state 0x1d but status/handler installs "
             "don't match expectations (status30=%d status31=0x%02x "
             "status32=0x%02x tx=%s rx=%s)\n",
             (int)(signed char)U8(OBJ + 0x30), (unsigned)U8(OBJ + 0x31),
             (unsigned)U8(OBJ + 0x32),
             tx_handler_name(V32H_PG(void *, HDX, 0x6c)),
             rx_handler_name(V32H_PG(void *, HDX, 0x70)));
    }
  } else {
    printf("UNEXPECTED: substate is %d, not 0x1d, after case 7's guard-pass "
           "direct call\n", (int)S16(HDX + 0x74));
  }

  /* ==========================================================================
   * PHASE-GROUP 5 (THIS CHUNK): case 0x1d's own "local retrain" loop-back
   * body - now hand-cleaned (needed TxHdxFinishFrame, ported this chunk).
   * Supersedes the prior chunk's own direct-poke workaround (which skipped
   * straight from state 0x1d to state 8, bypassing case 0x1d's own body
   * entirely because TxHdxFinishFrame didn't exist yet) - this drives the
   * REAL organic path all the way through instead.
   *
   * State is currently 0x1d (from case 7's own guard-pass above), with
   * TxHdxNoCarrier active and budget FIXED at 0x10 (16, set by case 7's own
   * transition, not the object's usual fixedBudget/+0x80 field). Every
   * V32TxHdxModem call requests exactly symLen (+0x9e) samples (see
   * src/v32/v32_engine.c's own V32TxHdxModem definition: `remaining =
   * *(short*)(hdxobj+0x9e)` at the top of every call) - the SAME small value
   * case 0x1d itself installs as TxHdxFinishFrame's own fresh budget. Since
   * TxHdxNoCarrier's own budget decrement uses the FULL per-call requested
   * count, not the budget-clamped produced amount (src/v32/v32_engine.c's own
   * TxHdxNoCarrier header comment), and TxHdxFinishFrame itself does not
   * clamp AT ALL (src/v32/v32_engine.c's own TxHdxFinishFrame header comment), exactly
   * how many V32TxHdxModem calls this takes - and whether case 0x1d's own
   * dispatch fires with leftover `remaining` samples that
   * V32TxHdxModem's documented "mid-block continuation" idiom immediately
   * hands to the just-installed TxHdxFinishFrame WITHIN THE SAME CALL, quite
   * possibly driving TxHdxFinishFrame's own tiny budget negative too and
   * dispatching case 8's body in that SAME call - depends on this object's
   * actual symLen value, not hardcoded/assumed here. Both outcomes (state
   * lands observably at 8 first, or collapses straight through to 9 in one
   * call) are handled below; either way it's the DIFFERENTIAL DIFF itself,
   * not a hardcoded prediction, that proves bit-exact behavior against
   * dsplibs.o through whichever path the real numbers take.
   * ========================================================================== */
  printf("== V.32 Answer NextState, phase-group 5 (case 0x1d local-retrain "
         "loop-back, real TxHdxFinishFrame) ==\n");

  tx_drive_until_transition("state0x1d(TxHdxNoCarrier)->?(case0x1d fires)", 0x1d, 50);

  if (S16(HDX + 0x74) == 8) {
    printf("CONFIRMED: case 0x1d's own body fired for real, landing at state "
           "8 with TxHdxFinishFrame=%s installed (budget=%d) - now driving "
           "TxHdxFinishFrame's own tiny budget to the case 8 dispatch\n",
           (V32H_PG(void *, HDX, 0x6c) == (void *)TxHdxFinishFrame) ? "yes"
                                                                            : "NO(UNEXPECTED)",
           S32(HDX + 0x78));
    tx_drive_until_transition("state8(TxHdxFinishFrame)->9(case8 fires)", 8, 50);
  } else if (S16(HDX + 0x74) == 9) {
    printf("CONFIRMED: case 0x1d's own body AND case 8's own body both fired "
           "within the SAME V32TxHdxModem call (TxHdxFinishFrame's own "
           "leftover budget was immediately exhausted by a mid-block-"
           "continuation sample) - state 8 was never independently "
           "observable as its own dump line for this object's symLen value, "
           "landing straight at state 9\n");
  } else {
    printf("UNEXPECTED: substate is %d, neither 8 nor 9, after driving "
           "through case 0x1d's own body\n", (int)S16(HDX + 0x74));
  }

  if (S16(HDX + 0x74) == 9) {
    if (V32H_PG(void *, HDX, 0x6c) == (void *)TxHdxCarrierState &&
        S32(HDX + 0x78) <= 0x100 && S32(HDX + 0x7c) == 0) {
      printf("CONFIRMED: the local-retrain loop-back RECONVERGED into the "
             "exact same state-8-onward path the cold-start's own case "
             "8->9 transition uses - TxHdxCarrierState reinstalled, "
             "accumulator reset, budget<=0x100 - LoadReg(0)->"
             "SetECRndTripDelayV32 read back reg0=%d (the SAME field case "
             "7's own StoreReg(obj,engine[0x96],0) wrote before entering the "
             "loop-back)\n", (int)S16(HDX + 0x3c));
    } else {
      printf("NOTE: reached state 9 via the loop-back but handler/budget/"
             "accum don't match case 8's own expected reconvergence values "
             "(tx=%s budget=%d accum=%d)\n",
             tx_handler_name(V32H_PG(void *, HDX, 0x6c)), S32(HDX + 0x78),
             S32(HDX + 0x7c));
    }
  } else {
    printf("UNEXPECTED: did not reach state 9 after driving through the full "
           "case 0x1d -> case 8 loop-back sequence (state=%d)\n",
           (int)S16(HDX + 0x74));
  }

  /* ---- case 9 (substate 9 -> 10), driven fully organically again: case 8
   * just reinstalled TxHdxCarrierState with a fresh budget=0x100, so TX ticks
   * resume naturally decrementing it - once exhausted, case 9 fires (shares
   * case 0x16's own literal tail, see header comment). Note: case 9's own
   * budget=0x10 write is immediately PARTIALLY re-consumed within the SAME
   * V32TxHdxModem call, per this project's already-documented
   * "mid-block-continuation" precedent (V32TxHdxModem hands any leftover
   * requested-sample remainder straight to the just-installed handler/budget
   * before returning) - so budget is checked as "<=0x10", not "==0x10". ---- */
  tx_drive_until_transition("state9->10(TxHdxCarrierState)", 9, 200);
  if (S16(HDX + 0x74) == 10) {
    if (S32(HDX + 0x78) <= 0x10 && S32(HDX + 0x7c) == 0 &&
        V32H_PG(void *, HDX, 0x6c) == (void *)TxHdxCarrierState) {
      printf("CONFIRMED: case 9's organic transition to state 10 (budget<="
             "0x10 after V32TxHdxModem's own mid-block-continuation, "
             "accumulator=0, TxHdxCarrierState reinstalled) - state 0xa/10 "
             "onward (TxHdxTRN/the real training-sequence segment) is NOT "
             "this chunk's scope, see the proposed next phase-group in "
             "NOTES.md\n");
    } else {
      printf("NOTE: case 9 reached state 10 but budget/handler don't match "
             "expectations (budget=%d accum=%d tx=%s)\n",
             S32(HDX + 0x78), S32(HDX + 0x7c),
             tx_handler_name(V32H_PG(void *, HDX, 0x6c)));
    }
  } else {
    printf("UNEXPECTED: substate is %d, not 10, after driving case 9\n",
           (int)S16(HDX + 0x74));
  }

  /* ==========================================================================
   * PHASE-GROUP 3 (this chunk): cases 0xa-0x12, plus RateToSeq - see this
   * file's own header comment and src/v32/v32_engine.c's large header comment (right
   * before V32AnsNextState's own definition) for the full case-by-case
   * derivation. Unlike V32OrgNextState's own phase-group 3, EVERY case in
   * this range explicitly resets budget to a fresh, non-negative value as
   * part of its own transition (disassembly-confirmed) - none of them share
   * Org's own case-0x10 "doesn't reset budget" hazard, so this whole
   * phase-group is driven organically via tx_drive_until_transition
   * throughout, no direct-call crash workaround needed.
   * ========================================================================== */
  printf("== V.32 Answer NextState, phase-group 3 (cases 0xa-0x12) ==\n");

  /* ---- state 10 -> 11 (case 0xa): installs TxHdxTRN - THE new handler this
   * phase-group actually exercises the body of (RX stays RxHdxData, already
   * verified); InitGenSequence(0xf,4,2); SeedScramblerV32(0). ---- */
  tx_drive_until_transition("state10->11(TxHdxCarrierState)", S16(HDX + 0x74), 20000);
  if (S16(HDX + 0x74) == 11 && V32H_PG(void *, HDX, 0x6c) != (void *)TxHdxTRN) {
    printf("UNEXPECTED: substate is 11 but TX handler is %s, not TxHdxTRN\n",
           tx_handler_name(V32H_PG(void *, HDX, 0x6c)));
  }

  /* ---- state 11 -> 12 (case 0xb): installs TxHdxScrSequence; the substate-12
   * TX ticks immediately below are what actually exercise TxHdxTRN's own
   * per-call body (GenSequence/ScrambleDataV32/the dibit-remap
   * lookup/ModDataV32) for real, budget=0x400 (1024 samples) worth of
   * calls. ---- */
  tx_drive_until_transition("state11->12(TxHdxTRN)", S16(HDX + 0x74), 20000);

  /* ---- state 12 -> 13 (case 0xc): TX handler NOT touched (stays
   * TxHdxScrSequence from case 0xb - confirmed via disassembly), budget=
   * 0x1a6c(6764, the long bulk-TRN segment). ---- */
  tx_drive_until_transition("state12->13(TxHdxScrSequence)", S16(HDX + 0x74), 20000);

  /* ---- state 13 -> 14 (case 0xd): the "big" rate-encoding case -
   * GetRateV32->RateToSeq->CodeRateSeq->StoreReg(reg1) (RateToSeq is the ONE
   * new shared helper this range needs), installs RxHdxSTone,
   * FPM_MTD_create(mtdObj,mtdObj) self-referential reset, InitGenSequence
   * with the coded rate, SetTxModeV32(1). TX stays TxHdxScrSequence
   * (confirmed via disassembly - no write to +0x6c anywhere in this case's
   * own compiled body). ---- */
  tx_drive_until_transition("state13->14(TxHdxScrSequence)", S16(HDX + 0x74), 20000);
  if (S16(HDX + 0x74) == 14) {
    printf("CONFIRMED: case 0xd's GetRateV32->RateToSeq->CodeRateSeq cascade "
           "coded reg1=%d (StoreReg) for this object's own rate-class-0 "
           "configuration - RxHdxSTone installed=%s\n",
           (int)S16(HDX + 0x3e),
           (V32H_PG(void *, HDX, 0x70) == (void *)RxHdxSTone) ? "yes" : "NO (UNEXPECTED)");
  } else {
    printf("UNEXPECTED: substate is %d, not 14, after driving case 0xd\n",
           (int)S16(HDX + 0x74));
  }

  /* ---- state 14 -> 15 (case 0xe): budget = LoadReg(0) + TurnAroundDelay (a
   * simpler formula shape than Org's own case 0xa - no +0x100/rounding
   * term, confirmed via disassembly); installs TxHdxNoCarrier/RxHdxData
   * (both already real). ---- */
  tx_drive_until_transition("state14->15(TxHdxScrSequence)", S16(HDX + 0x74), 20000);
  if (S16(HDX + 0x74) == 15 &&
      (V32H_PG(void *, HDX, 0x6c) != (void *)TxHdxNoCarrier ||
       V32H_PG(void *, HDX, 0x70) != (void *)RxHdxData)) {
    printf("UNEXPECTED: substate is 15 but handlers don't match (tx=%s rx=%s)\n",
           tx_handler_name(V32H_PG(void *, HDX, 0x6c)),
           rx_handler_name(V32H_PG(void *, HDX, 0x70)));
  }

  /* ---- state 15 -> 16 (case 0xf): budget=phaseInc; SetToneDetect(obj,
   * 0x708); installs RxHdxSTone (reinstall); tone object #1's own +6 field
   * set to 0x747a (disassembly-CONFIRMED this chunk a physically SEPARATE
   * write site from cases 0/1's own shared tail, not a literal shared
   * tail with them - resolving the flag phase-group 1 left open). TX stays
   * TxHdxNoCarrier. ---- */
  tx_drive_until_transition("state15->16(TxHdxNoCarrier)", S16(HDX + 0x74), 20000);

  /* ---- state 16 -> 17 (case 0x10): installs RxHdxEpoch (already real);
   * budget=phaseInc; SetRxModeV32(0); SetRxLoopsV32(2). TX stays
   * TxHdxNoCarrier. ---- */
  tx_drive_until_transition("state16->17(TxHdxNoCarrier)", S16(HDX + 0x74), 20000);

  /* ---- state 17 -> 18 (case 0x11): budget=0x610(1552); installs
   * TxHdxNoCarrier/RxHdxData (reinstall); SetAdaptEqV32(2). **Confirmed via
   * disassembly this chunk: unlike V32OrgNextState's own structurally-
   * similar case 0x11, this case does NOT touch the bigbuf+0x1e4/+0x1e8 AGC
   * alpha/beta pointers at all** - a genuine, checked-not-assumed structural
   * difference from Org. ---- */
  tx_drive_until_transition("state17->18(TxHdxNoCarrier)", S16(HDX + 0x74), 20000);

  /* ---- state 18 -> 21/0x15 (case 0x12): substate JUMPS to 0x15, NOT 0x13
   * (Ans's own case numbers are not contiguous through this stretch,
   * confirmed via the jump-table decode); installs RxHdxRateSequence
   * (already real); budget=phaseInc; SetRxModeV32(1); SetAdaptEqV32(3);
   * InitDetSequence(0x1110111,0xf111f111,0xffffffff,2) - begins the real
   * rate-negotiation LISTEN phase. TX stays TxHdxNoCarrier. This is the
   * last case in this chunk's own scope - cases 0x15-0x1e (the
   * rate-negotiation ack/CONNECT tail, needing CodeFinalRateSeq) are the
   * proposed NEXT phase-group, see NOTES.md. ---- */
  tx_drive_until_transition("state18->21(TxHdxNoCarrier)", S16(HDX + 0x74), 20000);
  if (S16(HDX + 0x74) == 0x15) {
    if (V32H_PG(void *, HDX, 0x70) == (void *)RxHdxRateSequence &&
        V32H_PG(void *, HDX, 0x6c) == (void *)TxHdxNoCarrier) {
      printf("CONFIRMED: case 0x12's own non-contiguous jump to state 0x15 "
             "fired organically, RxHdxRateSequence installed (TX stays "
             "TxHdxNoCarrier) - cases 0x15-0x1e are NOT this chunk's scope, "
             "see the proposed next phase-group in NOTES.md\n");
    } else {
      printf("NOTE: reached state 0x15 but handlers don't match expectations "
             "(tx=%s rx=%s)\n",
             tx_handler_name(V32H_PG(void *, HDX, 0x6c)),
             rx_handler_name(V32H_PG(void *, HDX, 0x70)));
    }
  } else {
    printf("UNEXPECTED: substate is %d, not 0x15, after driving case 0x12\n",
           (int)S16(HDX + 0x74));
  }

  /* ==========================================================================
   * PHASE-GROUP 4: cases 0x15-0x1e, plus CodeFinalRateSeq - the
   * rate-negotiation ack/CONNECT tail. At the time this was written, this
   * completed V32AnsNextState's main connect path except for case 0x1d's own
   * loop-back body (needing TxHdxFinishFrame) - **now closed by
   * PHASE-GROUP 5, further down**. See src/v32/v32_engine.c's own large header comment
   * (right before V32AnsNextState's own definition) for the full
   * case-by-case derivation.
   *
   * EVERY case in this range (0x15/0x16/0x17/0x18/0x19/0x1e/0x1a/0x1b/0x1c)
   * is UNCONDITIONAL - none of them have their own guard - so, exactly like
   * phase-group 3, the whole stretch is driven organically via
   * tx_drive_until_transition: whichever TX handler is active
   * (TxHdxCarrierState/TxHdxTRN/TxHdxScrSequence, all already real) just
   * keeps decrementing its own budget field and dispatches to V32NextState
   * the instant it hits <=0, with no RX/correlator involvement needed to
   * reach any of them.
   *
   * EMPIRICALLY CONFIRMED (not assumed) this chunk, exactly as the previous
   * chunk's own proposed-next-step predicted: `GetSequence()` (the
   * DetSequence correlator's own exposed match register, hdxobj+0x68) never
   * organically matches under this test's synthetic LCG noise - the SAME
   * "GetSequence never organically matches" precedent already established
   * for V32OrgNextState's own final phase-group - so case 0x15's reg2
   * capture and case 0x1e's reg4 capture both naturally read 0, and
   * DecodeRateSeq(0) against this object's own local=3345 (V32_RATE_SEQ[0],
   * the SAME value phase-group 3 already independently confirmed) lands on
   * class 6 ("incompatible") throughout - so simply driving this whole
   * range organically, with NO poke at all, already exercises the
   * INCOMPATIBLE-RATE branch for real, for cases 0x15/0x1a/0x1b/0x1c's own
   * DecodeRateSeq/CodeESeq/CodeFinalRateSeq calls.
   * ========================================================================== */
  printf("== V.32 Answer NextState, phase-group 4 (cases 0x15-0x1e, "
         "incompatible-rate branch) ==\n");

  /* ---- state 0x15 -> 0x16: SetTxModeV32(0); InitGenSequence(1,4,2);
   * GetSequence()->StoreReg(reg2); LoadReg(2)->DecodeRateSeq() - class 6
   * expected (see header comment above), sets status31|=2/status30=0x17. TX
   * switches from TxHdxNoCarrier to TxHdxCarrierState here. ---- */
  tx_drive_until_transition("state0x15->0x16(TxHdxNoCarrier)", S16(HDX + 0x74), 20000);
  if (S16(HDX + 0x74) == 0x16) {
    if (S16(HDX + 0x40) == 0 && (int)(signed char)U8(OBJ + 0x30) == 0x17 &&
        (U8(OBJ + 0x31) & 2) != 0) {
      printf("CONFIRMED: case 0x15's reg2 capture naturally read 0 "
             "(GetSequence's correlator never organically matches), "
             "DecodeRateSeq(0)->class 6 fired the incompatible fork "
             "(status31 bit1 set, status30=0x17)\n");
    } else {
      printf("NOTE: reached state 0x16 but reg2/status don't match "
             "expectations (reg2=%d status30=%d status31=0x%02x)\n",
             (int)S16(HDX + 0x40), (int)(signed char)U8(OBJ + 0x30),
             (unsigned)U8(OBJ + 0x31));
    }
  } else {
    printf("UNEXPECTED: substate is %d, not 0x16, after driving case 0x15\n",
           (int)S16(HDX + 0x74));
  }

  /* ---- state 0x16 -> 0x17: shares its own tail (real address 0x85ad8) with
   * case 9 - InitGenSequence(0xe,4,2); TX reinstalls TxHdxCarrierState. ---- */
  tx_drive_until_transition("state0x16->0x17(TxHdxCarrierState)", S16(HDX + 0x74), 20000);

  /* ---- state 0x17 -> 0x18: installs TxHdxTRN (a SECOND, acknowledgment TRN
   * segment); InitGenSequence(0xf,4,2); SeedScramblerV32(0);
   * SetAdaptEcV32(1). ---- */
  tx_drive_until_transition("state0x17->0x18(TxHdxCarrierState)", S16(HDX + 0x74), 20000);
  if (S16(HDX + 0x74) == 0x18 && V32H_PG(void *, HDX, 0x6c) != (void *)TxHdxTRN) {
    printf("UNEXPECTED: substate is 0x18 but TX handler is %s, not TxHdxTRN\n",
           tx_handler_name(V32H_PG(void *, HDX, 0x6c)));
  }

  /* ---- state 0x18 -> 0x19: budget=0x1bbc(7100, the second long
   * bulk-TRN-ack segment) - the TX ticks below actually exercise TxHdxTRN's
   * own per-call body a second time, for real. TX stays TxHdxTRN throughout
   * (installed by case 0x17, reinstalled again only at case 0x19/0x1a as
   * TxHdxScrSequence). ---- */
  tx_drive_until_transition("state0x18->0x19(TxHdxTRN)", S16(HDX + 0x74), 20000);

  /* ---- state 0x19 -> 0x1e: installs RxHdxSequence (already real, from
   * LocLoop); LoadReg(reg2)->CodeFinalRateSeq(obj,seq)->
   * InitGenSequence(codedFinalRate,0x10,2) - THE call that actually
   * exercises CodeFinalRateSeq for the first time in this project;
   * SetTxModeV32(1); InitDetSequence(...); SetAdaptEqV32(1). TX switches to
   * TxHdxScrSequence here. ---- */
  tx_drive_until_transition("state0x19->0x1e(TxHdxTRN)", S16(HDX + 0x74), 20000);
  if (S16(HDX + 0x74) == 0x1e) {
    if (V32H_PG(void *, HDX, 0x70) == (void *)RxHdxSequence &&
        V32H_PG(void *, HDX, 0x6c) == (void *)TxHdxScrSequence) {
      printf("CONFIRMED: case 0x19's CodeFinalRateSeq(reg2=0)->0x111 sentinel "
             "(class 6) fed InitGenSequence without crashing, RxHdxSequence "
             "installed, TX switched to TxHdxScrSequence\n");
    } else {
      printf("NOTE: reached state 0x1e but handlers don't match expectations "
             "(tx=%s rx=%s)\n",
             tx_handler_name(V32H_PG(void *, HDX, 0x6c)),
             rx_handler_name(V32H_PG(void *, HDX, 0x70)));
    }
  } else {
    printf("UNEXPECTED: substate is %d, not 0x1e, after driving case 0x19\n",
           (int)S16(HDX + 0x74));
  }

  /* ---- state 0x1e -> 0x1a: budget = 8-((phaseInc-curBudget)&7) (a TINY
   * value, 0-8) - GetSequence()->StoreReg(reg4). Neither TX nor RX handler
   * is touched by case 0x1e itself (confirmed via disassembly - see
   * src/v32/v32_engine.c's own header comment), so TX stays TxHdxScrSequence and this
   * transition fires almost immediately. ---- */
  tx_drive_until_transition("state0x1e->0x1a(TxHdxScrSequence)", S16(HDX + 0x74), 20000);
  if (S16(HDX + 0x74) == 0x1a && S16(HDX + 0x44) != 0) {
    printf("NOTE: reg4=%d after case 0x1e's own capture, expected 0 (see "
           "header comment)\n", (int)S16(HDX + 0x44));
  }

  /* ---- state 0x1a -> 0x1b: LoadReg(reg4)->DecodeRateSeq()->
   * SetRxModeV32(V32_RX_MODE[cls]); a SEPARATE LoadReg(reg4)->CodeESeq()->
   * InitGenSequence(...); FPM_AGC_Freeze(bigbuf+0x1d8). budget=8 (fixed,
   * tiny) - accumulator is NOT reset by this case (see header comment), so
   * this transition also fires almost immediately. ---- */
  tx_drive_until_transition("state0x1a->0x1b(TxHdxScrSequence)", S16(HDX + 0x74), 20000);

  /* ---- state 0x1b -> 0x1c: InitGenSequence(0xffff,0x10,8) BEFORE the state
   * write; LoadReg(reg4)->DecodeRateSeq()->SetTxModeV32(V32_TX_MODE[cls]).
   * budget=0x80(128). ---- */
  tx_drive_until_transition("state0x1b->0x1c(TxHdxScrSequence)", S16(HDX + 0x74), 20000);

  /* ---- state 0x1c: the terminal self-loop - status31|=0x19;
   * LoadReg(reg4)->DecodeRateSeq()->status30=V32_CONNECT[cls];
   * SetAdaptEqV32(3); SetRxLoopsV32(3) (the confirmed difference from Org's
   * own case 0x17); clears fseObj+0x62. budget=phaseInc (large), so this
   * stays put - re-exercise it a couple more times via direct calls (always
   * safe, self-loop, no other precondition) to confirm it's genuinely
   * stable. ---- */
  tx_drive_until_transition("state0x1b->0x1c(land)", S16(HDX + 0x74), 20000);
  if (S16(HDX + 0x74) == 0x1c) {
    if ((int)(signed char)U8(OBJ + 0x30) == (int)V32_CONNECT[6] &&
        (U8(OBJ + 0x31) & 0x19) == 0x19) {
      printf("CONFIRMED: reached the terminal state 0x1c with the "
             "INCOMPATIBLE-rate class 6 CONNECT code (status30=%d="
             "V32_CONNECT[6], status31 0x19 flags set) - the organic, "
             "no-poke path\n", (int)V32_CONNECT[6]);
    } else {
      printf("NOTE: reached state 0x1c but status30/status31 don't match the "
             "expected incompatible-class values (status30=%d status31=0x%02x)\n",
             (int)(signed char)U8(OBJ + 0x30), (unsigned)U8(OBJ + 0x31));
    }
    V32AnsNextState(OBJ);
    V32AnsNextState(OBJ);
    dump("state0x1c(self-loop re-exercised, incompatible)");
  } else {
    printf("UNEXPECTED: substate is %d, not 0x1c, after driving through the "
           "whole incompatible-rate tail\n", (int)S16(HDX + 0x74));
  }

  /* ---- one final small RxHdxData confirmation batch at the terminal state
   * (RxHdxData never calls NextState, so this is always safe regardless of
   * budget) ---- */
  if (S16(HDX + 0x74) == 0x1c) {
    rx_micro_test("state0x1c(RxHdxData, incompatible)", 6);
  }

  /* ==========================================================================
   * THE COMPATIBLE-RATE (CLASS 0) HAPPY PATH - pinned via a direct reg4
   * poke, the SAME "poke the field, call NextState directly" technique
   * test/v32/v32_engine.org.main.c's own final phase-group already established.
   * reg4 is captured ONLY by case 0x1e's own GetSequence()->StoreReg(4) -
   * cases 0x1a/0x1b/0x1c all just LoadReg(4) WITHOUT re-capturing it - so
   * rewinding the substate to 0x1a (the first case that just READS reg4,
   * mirroring Org's own rewind-to-0x14 precedent) and poking reg4 first
   * pins the value for all three remaining cases. Reuses the SAME object.
   * ========================================================================== */
  printf("== pinning rate class 0 (compatible) via a direct reg4 poke, "
         "re-driving cases 0x1a-0x1c ==\n");
  StoreReg(OBJ, 0x400, 4); /* bit 0x400 is the ONLY bit DecodeRateSeq tests
                             * against this object's own local=3345 value -
                             * see test/v32/v32_engine.org.main.c's own header
                             * comment for the full derivation (independently
                             * re-confirmed for this object by phase-group
                             * 3's own reg1=3345 finding). */
  S16(HDX + 0x74) = 0x1a;

  V32AnsNextState(OBJ); /* case 0x1a: DecodeRateSeq(0x400)->class 0 ->
                          * SetRxModeV32(V32_RX_MODE[0]); CodeESeq(0x400)
                          * (also class 0, not the 0xf111 sentinel this
                          * time). */
  dump("case0x1a->0x1b(direct, class0 pin)");

  V32AnsNextState(OBJ); /* case 0x1b: DecodeRateSeq(0x400)->class 0 ->
                          * SetTxModeV32(V32_TX_MODE[0]). */
  dump("case0x1b->0x1c(direct, class0 pin)");

  V32AnsNextState(OBJ); /* case 0x1c: the terminal CONNECT transition, this
                          * time landing the REAL class-0 CONNECT code. */
  dump("case0x1c(direct, class0 pin - CONNECTED)");

  if (S16(HDX + 0x74) == 0x1c) {
    if ((int)(signed char)U8(OBJ + 0x30) == (int)V32_CONNECT[0] &&
        (U8(OBJ + 0x31) & 0x19) == 0x19) {
      printf("CONFIRMED: pinned rate class 0 reached the REAL CONNECTED "
             "state (status30=%d=V32_CONNECT[0], status31 0x19 flags set) - "
             "this is the actual 4800bps-connect code slmodemd surfaces to "
             "userspace, and completes V32AnsNextState's entire main connect "
             "path\n", (int)V32_CONNECT[0]);
    } else {
      printf("NOTE: pinned class-0 path reached 0x1c but status30/status31 "
             "don't match the expected class-0 CONNECT values (status30=%d, "
             "status31=0x%02x, expected status30=%d)\n",
             (int)(signed char)U8(OBJ + 0x30), (unsigned)U8(OBJ + 0x31),
             (int)V32_CONNECT[0]);
    }
  } else {
    printf("UNEXPECTED: substate is %d, not 0x1c, after the pinned class-0 "
           "direct-call sequence\n", (int)S16(HDX + 0x74));
  }

  /* ---- one final small RxHdxData confirmation batch at the terminal state,
   * this time in the real class-0-connected configuration ---- */
  if (S16(HDX + 0x74) == 0x1c) {
    rx_micro_test("state0x1c(RxHdxData, class0 pin)", 6);
  }

  /* ---- explicit direct-call coverage of the universal case 0x21 ---- */
  S16(HDX + 0x74) = 0x21;
  S32(HDX + 0x78) = -999;
  V32AnsNextState(OBJ);
  dump("case0x21(direct)");
  if (S32(HDX + 0x78) != S32(HDX + 0x80)) {
    printf("UNEXPECTED: case 0x21 did not refresh budget to fixedBudget\n");
  }

  V32FP_delete(OBJ);
  printf("== done ==\n");
  return 0;
}
