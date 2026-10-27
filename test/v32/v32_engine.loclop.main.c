/*
 * Differential test harness for V.32/V.32bis's LocLoop ("role 2") NextState
 * FSM + its own Hdx handler subset (src/v32/v32_engine.c's newest hand-clean chunk):
 * V32LocLoopNextState, TxHdxCarrierState, TxHdxScrSequence,
 * RxHdxPhsReversal, RxHdxSequence, RxHdxData, and the small
 * InitGenSequence/InitDetSequence/SetToneDetect/GenSequence/DetSequence/
 * StoreReg helpers they depend on.
 *
 * Same pattern as every other *_diff_main.c in this project: this ONE
 * source compiles into two binaries (test/v32/v32_engine.loclop.sh) -
 * one linked directly against the real dsplibs.o (every symbol under test
 * is globally exported per `nm ../dsplibs.o` - V32LocLoopNextState,
 * TxHdxCarrierState, TxHdxScrSequence, RxHdxPhsReversal, RxHdxSequence,
 * RxHdxData, TxHdxTone/TxHdxNull/RxHdxTone/TxHdxNoCarrier/RxHdxError [the
 * last 5 taken by ADDRESS ONLY, for handler-identity printing below, never
 * called - see the "risk" section below]), one against src/v32/v32_engine.c + its
 * already-verified dependencies.
 *
 * ==========================================================================
 * THE LOCLOOP-ENTRY-PATH QUESTION (see src/v32/v32_engine.c's own header comment,
 * right before V32LocLoopNextState's definition, for the full derivation)
 * ==========================================================================
 * `V32FP_create`'s own public marshalling collapses any nonzero role
 * argument to a plain boolean, so `hdxobj+0x76` (the role field
 * V32LocLoopNextState/the 5 handlers all read) can NEVER become 2 through
 * that entry point. A full `get_function_xrefs` trace of `V32FP_recreate`
 * (the only place that role-branch logic lives) turns up exactly 3 call
 * sites in the whole object - `V32FP_create` and two in-place
 * "V32FP_recreate(obj,obj,0)" reinit calls inside `V32FP_control`, neither
 * of which can produce word0==2 either. A full grep of every `+0x76) =`
 * write site across the whole decompiled V.32 handshake layer never
 * produces 2. **LocLoop mode is genuinely unreachable through any real call
 * sequence dsplibs.o itself performs** - this is DIFFERENT from (stronger
 * than) B103's own LocLoop precedent, where LocLoop crashed the real blob on
 * first call but WAS at least reachable. This harness therefore uses this
 * project's own established, already-blessed "poke the field directly"
 * precedent (used previously for role 6/v32_data testing): build a real
 * object via `V32FP_create` normally, then directly write `hdxobj+0x76 = 2`.
 * This is the ONLY way to construct a byte-exact differential test of this
 * code at all, since no real call sequence reaches it.
 *
 * ==========================================================================
 * WHY THIS HARNESS DOES NOT DRIVE ONE FULLY ORGANIC END-TO-END RUN
 * ==========================================================================
 * `RxHdxPhsReversal`/`RxHdxSequence` both carry the generic Rx-side "budget
 * exceeded" error idiom, forcing +0x6c/+0x70 to `TxHdxNoCarrier`/
 * `RxHdxError` (explicitly OUT of this chunk's scope, left `abort()`-
 * stubbed). For synthetic (non-real-tone) input, neither
 * `FPM_TONE_find_rev` nor `DetSequence` will ever naturally signal a real
 * match, so the ONLY way any of these phases ever advances for synthetic
 * data is via the Tx side's own budget exhaustion (`TxHdxCarrierState`/
 * `TxHdxScrSequence`, which is purely count-based, not data-dependent).
 * Meanwhile the Rx side accumulates its OWN parallel "elapsed" counter
 * (+0x7c) against the exact SAME fixed threshold (+0x80) the Tx side resets
 * itself to at every phase transition. If both were driven at the same
 * cadence (e.g. through `v32_handshake`, which calls RX before TX every
 * block), RX's error-check would very likely win the race and force the
 * object into the out-of-scope TxHdxNoCarrier/RxHdxError pair before TX ever
 * got a chance - and since those are `abort()` stubs on the port side, that
 * would crash this test, not fail it cleanly.
 *
 * This harness avoids that deliberately: TX and RX are driven
 * INDEPENDENTLY (never through v32_handshake). For every RxHdxPhsReversal/
 * RxHdxSequence-active phase, only a small, bounded number of RX calls are
 * made (computed from the object's own real +0x80/+0x9e values, read back at
 * runtime, kept under 25% of the real threshold with a printed safety
 * margin) to validate their own per-call processing logic bit-exact,
 * without ever letting +0x7c reach +0x80. Separately, TX is looped for real
 * (varying nothing about the input - GenSequence overwrites the tx bit
 * buffer unconditionally before it's ever read, so the caller-supplied
 * content is provably inert for these two handlers) until its OWN budget
 * naturally exhausts and it calls this role's REAL NextState function
 * itself - i.e. every state transition from 3 through 10 is driven by the
 * REAL handler -> V32NextState[2] -> V32LocLoopNextState call chain, not by
 * this harness calling V32LocLoopNextState directly. Only states 0 and 1
 * (whose real active handlers, TxHdxTone/RxHdxTone, are out of this chunk's
 * scope - inherited from construction, never installed by
 * V32LocLoopNextState itself) and the case-0x21/case-0-"not yet" diagnostics
 * are driven via a direct V32LocLoopNextState(obj) call instead.
 *
 * If this ever turns out wrong (i.e. if some future chunk's investigation
 * finds a way LocLoop mode really was reachable, or a way to safely drive
 * RxHdxPhsReversal/RxHdxSequence all the way to a genuine match), this
 * harness's design should be revisited - it is deliberately conservative
 * given what's provable today.
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

/* Deterministic (not random - reproducible across blob/port runs, which
 * both read the SAME sequence of values since they're generated by this
 * harness, not by the code under test) synthetic RX sample generator. */
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

static void dump(const char *label)
{
  printf("[%-18s] state=%d role=%d budget=%d elapsed=%d fixedBudget=%d "
         "tx=%s rx=%s status30=%d status31=0x%02x reg0=%d "
         "gen(4a,4c,4e,50,52)=%d,%d,%d,%d,%u "
         "det(54,58,5c,60,64,68)=%u,%u,%u,%u,%u,%u\n",
         label, (int)S16(HDX + 0x74), (int)S16(HDX + 0x76), S32(HDX + 0x78), S32(HDX + 0x7c),
         S32(HDX + 0x80), tx_handler_name(V32H_PG(void *, HDX, 0x6c)),
         rx_handler_name(V32H_PG(void *, HDX, 0x70)), (int)(signed char)U8(OBJ + 0x30),
         (unsigned)U8(OBJ + 0x31), (int)S16(HDX + 0x3c), (int)S16(HDX + 0x4a), (int)S16(HDX + 0x4c),
         (int)S16(HDX + 0x4e), (int)S16(HDX + 0x50), (unsigned)U16(HDX + 0x52), U16(HDX + 0x54),
         U32(HDX + 0x58), U32(HDX + 0x5c), U32(HDX + 0x60), U32(HDX + 0x64), U32(HDX + 0x68));
}

/* Drives V32RxHdxModem `n` times with varying synthetic samples, dumping
 * state after each call. Caller is responsible for keeping `n` safely under
 * whatever budget threshold currently applies (see main()). */
static void dump_outbits(const char *tag, short count)
{
  int i;
  int n = (count < 0) ? 0 : (int)count;
  if (n > 16) n = 16;
  printf("[%-18s] outCount=%d outBits[0..%d)=", tag, (int)count, n);
  for (i = 0; i < n; i++) {
    printf("%d%s", (int)rxOutBuf[i], (i + 1 < n) ? "," : "");
  }
  printf("\n");
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

/* Same as rx_micro_test, but with an explicit, caller-chosen per-call sample
 * count instead of the natural symLen - see main()'s "ECC LANDMINE" comment
 * for why this exists and why it must be used (instead of rx_micro_test)
 * for every RxHdxData/RxHdxSequence-active phase. */
static void rx_micro_test_n(const char *label, int calls, int perCall)
{
  int i, j;
  if (perCall <= 0 || perCall > MAXBUF) perCall = 1;
  for (i = 0; i < calls; i++) {
    short count = (short)perCall;
    char tag[64];
    for (j = 0; j < perCall; j++) rxInBuf[j] = next_sample();
    memset(rxOutBuf, 0, sizeof(rxOutBuf));
    V32RxHdxModem(OBJ, rxInBuf, rxOutBuf, &count);
    snprintf(tag, sizeof(tag), "%s rx#%d", label, i);
    dump(tag);
  }
}

/* Drives V32TxHdxModem repeatedly until `hdxobj+0x74` (state) changes away
 * from `fromState`, dumping state after each call. This is what actually
 * exercises the real TxHdxCarrierState/TxHdxScrSequence -> V32NextState[2]
 * -> V32LocLoopNextState dispatch chain for real, per this file's header
 * comment. Returns the number of V32TxHdxModem calls made. */
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
  short symLen;
  int fixedBudget;
  int safeRxCalls;

  memset(cfg, 0, sizeof(cfg));
  cfg[0] = 0; /* role selector at V32FP_create's own level - irrelevant, overridden below */
  cfg[1] = 100;
  cfg[2] = 60000;
  cfg[3] = 700;
  cfg[4] = 1; /* enable bit0 - matches v32_create's own real convention */
  *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16); /* 4800bps, rate class 0 */

  p = V32FP_create(cfg, 0);
  if (!p) {
    printf("create failed\n");
    return 1;
  }
  OBJ = (char *)(p);
  HDX = V32W_PG(char *, OBJ, 0x64);

  printf("== V.32 LocLoop NextState + Hdx handler subset ==\n");

  /* Resolve the LocLoop-entry-path question the only way it CAN be resolved
   * (see this file's header comment): directly poke the role field. */
  S16(HDX + 0x76) = 2;
  dump("after role-poke");

  symLen = S16(HDX + 0x9e);
  fixedBudget = S32(HDX + 0x80);
  printf("object params: symLen(+0x9e)=%d fixedBudget(+0x80)=%d\n", (int)symLen, fixedBudget);
  /* Keep every bounded RX micro-test comfortably under the real per-phase
   * budget threshold (this cap is now 40, not just 4x-margin-derived - see
   * below for why 40 specifically).
   *
   * RESOLVED (2026-09-29 follow-up): the previous version of this comment
   * recorded a "tried and reverted" experiment - raising this cap from 5 to
   * 25 surfaced a divergence at rx#13 within state4's own RxHdxPhsReversal
   * loop (blob transitions to state 6 while the port was still in state 4,
   * ~168 elapsed) - and left it as an open, not-yet-root-caused finding.
   * It has now been root-caused and FIXED, via the exact same "call-by-call
   * field trace, narrow to the exact function, then disassembly-confirm"
   * methodology already used for the FPM_atan and RX-descrambler bugs (see
   * NOTES.md's "RxHdxPhsReversal 25+-call divergence RESOLVED" entry for the
   * full writeup): a field-level-instrumented probe (dumping
   * RxHdxPhsReversal's own +0x90/+0x94.../+0x9c/+0xa8/+0xac fields AND a
   * checksum of the post-call `inSamples` buffer - which RxHdxPhsReversal
   * mutates in place via FPM_AGC_agc/FPM_TONE_kill/FPM_TONE_find_rev's own
   * internal pre-filter - after every single call) showed the transformed
   * buffer already differed between dsplibs.o and the port at call #0, long
   * before the visible state-transition symptom at call ~13-16 (a classic
   * "matches for N calls then diverges" accumulation shape). Isolating each
   * of the 3 stages RxHdxPhsReversal chains (FPM_AGC_agc, then 2x
   * FPM_TONE_kill, then FPM_TONE_find_rev's own internal IIR pre-filter) via
   * direct calls on the real hdxobj's real sub-objects showed the first two
   * stages match byte-for-byte, isolating the bug to FPM_TONE_find_rev's own
   * internal filter. Its own coefficient array - built once, at
   * FPM_TONE_create construction time, only ever used by find_rev - had a
   * genuinely wrong coefficient: dsplibs.o's `-31457` vs. the port's `-1`.
   * Root cause: `src/fpm/fpm_tone.c`'s `FPM_TONE_create` had a SPURIOUS
   * intermediate `(short)` truncation-before-shift in
   * `puVar2[4] = (unsigned short)((short)(local_24[1] * -0x7ae1) >> 0xe)` -
   * the exact same bug CLASS already found/fixed twice before in this
   * project (`FPM_atan`, `FPM_log10`), now found a third time, disassembly-
   * confirmed (`imul eax,edi,0xffff851f; sar eax,0xe` - the multiply/shift
   * pair run on the FULL 32-bit product, truncated to 16 bits only once, at
   * the final store - NOT truncated to 16 bits before the shift like the
   * port's extra cast did). Fixed in `src/fpm/fpm_tone.c` (see that file's own
   * updated comment on this exact line). With the fix, this cap was raised
   * to 40 (comfortably clearing the old ~13-16-call divergence zone with
   * margin) and re-verified byte-for-byte identical against dsplibs.o at
   * that count - see the "40-call RxHdxPhsReversal regression coverage"
   * paragraph in NOTES.md. */
  safeRxCalls = (symLen > 0) ? (fixedBudget / ((int)symLen * 4)) : 1;
  if (safeRxCalls > 40) safeRxCalls = 40;
  if (safeRxCalls < 1) safeRxCalls = 1;
  printf("chosen safe RX micro-test call count per phase: %d\n", safeRxCalls);

  /* ---- case 0, "not yet" branch: budget>0 must be a pure no-op ---- */
  S16(HDX + 0x74) = 0;
  S32(HDX + 0x78) = 50;
  V32LocLoopNextState(OBJ);
  dump("case0 (not yet)");

  /* ---- case 0, real transition branch (0 -> 1) ---- */
  S32(HDX + 0x78) = 0;
  V32LocLoopNextState(OBJ);
  dump("case0->1");

  /* ---- case 1 (1 -> 3): installs TxHdxCarrierState ---- */
  V32LocLoopNextState(OBJ);
  dump("case1->3");

  /* States 0/1/3 all have RxHdxTone/TxHdxTone as their real active
   * handlers (inherited from construction, not installed by
   * V32LocLoopNextState - see this file's header comment) - out of this
   * chunk's scope, so state 3 is left via a direct NextState call too
   * (case 3 itself does not depend on any handler having run). */

  /* ---- case 3 (3 -> 4): installs RxHdxPhsReversal ---- */
  V32LocLoopNextState(OBJ);
  dump("case3->4");

  /* From here on, TX=TxHdxCarrierState/TxHdxScrSequence is the real active
   * handler continuously (see this file's header comment) - drive every
   * further transition for real through it. */

  rx_micro_test("state4(RxHdxPhsReversal)", safeRxCalls);
  tx_drive_until_transition("state4->6(TxHdxCarrierState)", 4, 20000);

  rx_micro_test("state6(RxHdxPhsReversal)", safeRxCalls);
  tx_drive_until_transition("state6->7(TxHdxCarrierState)", 6, 20000);

  /* ---- FORMER "ECC LANDMINE" CRASH, RESOLVED (2026-09-29 follow-up) ----
   * RxHdxData/RxHdxSequence both call DemodDataV32 -> FPM_ECC_cancel, whose
   * "every 3rd sample" tap-injection step chases a two-level pointer through
   * ECCv32_IMAP/ECCv32_QMAP. This used to CRASH the port deterministically
   * the moment 3 cumulative samples had been fed through any real object's
   * DemodDataV32 path (see git history / NOTES.md for the original writeup),
   * while the real dsplibs.o never crashed under the identical call
   * sequence. Root-caused: `src/v32/v32_tables.c` previously captured
   * ECCv32_IMAP/ECCv32_QMAP as byte-exact ALL-ZERO flat data, but
   * `readelf -r --wide dsplibs.o` (cross-checked with gdb against
   * `test/.build/v32/v32_engine.loclop_test_blob`'s own live memory) shows both tables are
   * really 6-slot arrays of R_386_32-relocated SUB-TABLE POINTERS
   * (SMCv32_QMAP16/IMAP16 + VTBv32_{Q,I}MAP{32,16T,64,128}) - the real
   * dsplibs.o's copies get these relocations resolved by the final link
   * step, but the port's separately-compiled, statically-all-zero copies
   * did not. Fixed in `src/v32/v32_tables.c`/`.h` (now real 6-pointer arrays)
   * and proven bit-exact in isolation by `test/v32/v32_engine.primitives.main.c`
   * scenario 9 (`test_ecc_real_tables`, count=9, forces the tap-injection
   * branch 3 times in a SINGLE FPM_ECC_cancel call) - byte-for-byte
   * identical to dsplibs.o, no crash.
   *
   * ---- SEPARATE, NOW ALSO RESOLVED (2026-09-29 second follow-up): the
   * cross-call MRF/ECC/SRE/FSE divergence ----
   * With the crash fixed, this harness was experimentally changed to drive
   * RxHdxData/RxHdxSequence at their NATURAL per-call sample count (`symLen`,
   * via `rx_micro_test`) instead of a tiny 1-sample-per-call workaround,
   * across multiple DemodDataV32 calls. That experiment found: raw RX
   * samples entering the chain were byte-identical between dsplibs.o and the
   * port (gdb-confirmed), yet after several real calls the DEMODULATED bit
   * content `DetSequence` receives started to differ (dsplibs.o: `{3,1,2,1}`,
   * port: `{2,1,1,1}` at the first divergence point) - i.e. a genuine
   * cross-call PERSISTENT STATE bug somewhere in the MRF/ECC/SRE/FSE chain,
   * not in this harness's sample generation and not the IMAP/QMAP crash
   * above (independently proven bit-exact in isolation for a multi-tick
   * single call).
   *
   * ROOT CAUSE (confirmed via porting/decompiled/dp/v32_state.c, not just theory):
   * `V32FP_recreate` (the real object constructor `src/v32/v32_engine.c`'s "--- FPM_ECC_
   * init ---" block ports) does THREE writes immediately after calling
   * `FPM_ECC_init`, at porting/decompiled/dp/v32_state.c lines 1591-1593:
   *     *(undefined2 *)(iVar11 + 0x114) = 0x30;   // eccObj+0x34
   *     *(undefined2 *)(iVar11 + 0x110) = 0;      // eccObj+0x30
   *     *(undefined2 *)(iVar11 + 0x112) = 0;      // eccObj+0x32
   * i.e. an unconditional fixed-triple override of the "every 3rd sample"
   * tap-injection's circular read-index pair (`eccObj+0x30`/`+0x32`, aka
   * FPM_ECC_cancel's `local_48`/`local_4c`) and their shared modulus bound
   * (`eccObj+0x34`, aka `sVar6`) - discarding FPM_ECC_init's OWN cfg-derived
   * computation for these 3 fields. `src/v32/v32_echo_canceller.c`'s header comment already
   * documented that this override happens at "both real call sites" - but
   * `src/v32/v32_engine.c`'s own `V32FP_recreate` translation never actually PERFORMED
   * it: only the (unrelated) `eccObj+0x64`/`+0x66` pre-seed writes were
   * present, the 3-field post-init override was silently missing. Without
   * it, `FPM_ECC_cancel` ran with FPM_ECC_init's own internally-computed
   * (wrong, in-real-practice-always-discarded) values for these 3 fields
   * instead of the real object's actual runtime values - the two objects'
   * "every 3rd sample" circular buffer indices march out of sync from the
   * very first tap-injection tick onward, one field write at a time, which
   * is invisible in a short/isolated test (bounded state, no chance to
   * accumulate a visible difference) but produces materially different
   * demodulated bit content once enough real ticks have accumulated across
   * several real `DemodDataV32` calls - exactly the "matches for N calls
   * then diverges" shape this bug class predicts. This was NOT caught by
   * `test/v32/v32_engine.primitives.main.c`'s own isolated ECC scenarios (0-7,9)
   * because those scenarios manually pre-set this exact fixed triple in the
   * test harness itself before calling `FPM_ECC_cancel` directly (per
   * `src/v32/v32_echo_canceller.c`'s header note) - they never exercised the REAL
   * `V32FP_recreate` construction path that was supposed to (but didn't)
   * perform this override.
   *
   * FIX: `src/v32/v32_engine.c`'s FPM_ECC_init call site now performs the same 3-field
   * fixed-triple override (`eccObj+0x34=0x30`, `eccObj+0x30=0`,
   * `eccObj+0x32=0`) immediately after `FPM_ECC_init`, matching
   * porting/decompiled/dp/v32_state.c lines 1591-1593 exactly.
   *
   * This harness now drives RxHdxData/RxHdxSequence at their natural
   * per-call `symLen` sample count, `safeRxCalls` times each (same budget-
   * safety margin already computed above for states 4/6), to prove the fix
   * holds across multiple real calls - not just the crash-avoidance smoke
   * test the tiny 1-sample workaround used to provide. */
  rx_micro_test("state7(RxHdxData)", safeRxCalls);
  tx_drive_until_transition("state7->8(TxHdxCarrierState)", 7, 20000);

  rx_micro_test("state8(RxHdxSequence)", safeRxCalls);
  tx_drive_until_transition("state8->9(TxHdxScrSequence)", 8, 20000);

  tx_drive_until_transition("state9->10(TxHdxScrSequence)", 9, 20000);

  /* State 10 is terminal (case 10 never advances +0x74) - a natural TX
   * budget exhaustion here just re-enters case 10 (budget refresh only).
   * Drive it for real once to prove that too, then stop (it would loop
   * forever otherwise). */
  {
    int i;
    for (i = 0; i < 20000; i++) {
      short outCount = 0;
      char tag[64];
      int budgetBefore = S32(HDX + 0x78);
      memset(txBitsBuf, 0, sizeof(txBitsBuf));
      memset(txOutBuf, 0, sizeof(txOutBuf));
      V32TxHdxModem(OBJ, txBitsBuf, txOutBuf, &outCount);
      snprintf(tag, sizeof(tag), "state10(terminal) tx#%d", i);
      dump(tag);
      if (S32(HDX + 0x78) > budgetBefore) {
        printf("state10(terminal): observed a real natural budget refresh "
               "(case 10 re-entered) after %d calls\n",
               i + 1);
        break;
      }
    }
  }

  /* ---- explicit direct-call coverage of case 10 and case 0x21, the two
   * remaining case bodies not otherwise naturally exercised in a controlled
   * way above ---- */
  S32(HDX + 0x78) = 1;
  S32(HDX + 0x7c) = 777;
  V32LocLoopNextState(OBJ);
  dump("case10(direct)");

  S16(HDX + 0x74) = 0x21;
  S32(HDX + 0x78) = -999;
  V32LocLoopNextState(OBJ);
  dump("case0x21(direct)");

  V32FP_delete(OBJ);
  printf("== done ==\n");
  return 0;
}
