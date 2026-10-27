/*
 * Differential test harness for V.32/V.32bis's Originate ("role 0")
 * cold-start NextState FSM - phase-groups 1, 2, 3 AND 4 (src/v32/v32_engine.c's four
 * hand-clean chunks - V32OrgNextState is now COMPLETE): V32OrgNextState
 * cases 0-0x17 + the universal 0x21 budget-refresh case, plus the Hdx
 * handlers these four phase-groups need: RxHdxTone (phase-group 1),
 * TxHdxNoCarrier/RxHdxSTone/RxHdxEpoch/RxHdxRateSequence/RxHdxNull
 * (phase-group 2), TxHdxTRN (phase-group 3, cases 0xa-0x11), and
 * `DecodeRateSeq`/`CodeESeq` (phase-group 4, this pass - cases 0x12-0x17,
 * the "rate-sequence detect/decode/ack exchange through the terminal
 * CONNECTED state" stretch - no NEW Hdx handler needed, `RxHdxSequence` is
 * the only one this range touches and it's already real).
 * TxHdxCarrierState/TxHdxScrSequence/RxHdxPhsReversal/RxHdxSequence/
 * RxHdxData (already hand-cleaned/verified by earlier chunks) are reused as
 * is.
 *
 * ==========================================================================
 * PHASE-GROUP 4 (this pass): cases 0x12-0x17, `DecodeRateSeq`/`CodeESeq` -
 * see src/v32/v32_engine.c's own large header comment (right before V32OrgNextState's
 * definition) for the full case-by-case derivation and the disassembly-
 * confirmed verification of every one of these 6 cases plus both new
 * helpers against the real dsplibs.o machine code.
 *
 * **THE RATE-CLASS QUESTION, RESOLVED EMPIRICALLY**: `DecodeRateSeq`'s
 * result depends entirely on what `GetSequence()` returns at the moment
 * cases 0x13/0x16/0x17 read it (via reg3, captured through `LoadReg`/
 * `StoreReg`) - and `GetSequence()` just reads back `hdxobj+0x68`, a field
 * `InitDetSequence` explicitly zeroes and only `DetSequence` ever writes, ON
 * A MATCH. Per this project's own repeatedly-confirmed finding (every other
 * `DetSequence`-based handler in this file, going back to the LocLoop
 * chunk), synthetic noise never produces a real correlator match - and this
 * test's own `RxHdxSequence` batch at state 0x13 (6 calls of synthetic
 * noise, the only place a match could occur before case 0x13 reads reg3) is
 * no exception. So for THIS test's own scenario, `GetSequence()` reads back
 * `0` every time, and (worked out by hand against `V32_RATE_SEQ[0]=3345`'s
 * own bit pattern, then CONFIRMED by observing the actual dump() output
 * below) `DecodeRateSeq(0)` returns class 6 ("incompatible") - NOT class 0 -
 * for this rate-class-0-configured object. This gives the INCOMPATIBLE-rate
 * branch organically, for free, without any poke: case 0x14's non-fatal
 * fork (status31|=2/status30=0x17) fires for real, and cases 0x15/0x16/0x17
 * exercise `V32_TX_MODE[6]`/`V32_RX_MODE[6]`/`V32_CONNECT[6]` - all
 * confirmed SAFE (mode 7 falls into `SetTxModeV32`/`SetRxModeV32`'s own
 * generic "unrecognized mode" default branch, src/v32/v32_datapath.c - NOT
 * one of the `VTBv32_init`-calling cases 3-6, which only trigger for modes
 * 3-6, i.e. rate classes 2-5 - so this test's incompatible-class-6 path
 * never risks the `VTBv32_init` abort() stub either).
 *
 * To ALSO exercise the real, compatible/happy path (class 0, the actual
 * `V32_CONNECT[0]=4` "4800bps connected" code, `SetTxModeV32`/
 * `SetRxModeV32` mode 1 - both provably safe, no `VTBv32_init` call), this
 * test PINS class 0 by directly poking reg3 (`StoreReg(OBJ, 0x400, 3)` -
 * bit 0x400 is the ONLY bit `DecodeRateSeq` actually tests against this
 * object's own `local=V32_RATE_SEQ[0]=3345` value, per the case-by-case
 * derivation in src/v32/v32_engine.c's header comment - every other decision branch in
 * the shared cascade is foreclosed by `local`'s own zero bits at
 * 0x8/0x20/0x40/0x200) after rewinding the substate field back to 0x14 and
 * re-driving cases 0x14-0x17 via direct `V32OrgNextState(OBJ)` calls - safe
 * and fully faithful because every one of these 4 cases is unconditional and
 * guard-free (per the case map), so a direct call exercises each one
 * completely regardless of how the state was reached, exactly like this
 * project's established "poke the field, call NextState directly" technique
 * (LocLoop chunk onward). Reusing the SAME object (already fully
 * initialized by the real handshake) rather than constructing a second one
 * is safe here since none of cases 0x14-0x17 depend on anything this rewind
 * disturbs (they only touch reg3, the state/budget/accumulator fields, and
 * already-initialized sub-objects like the +0x230 FSE block).
 *
 * ==========================================================================
 * PHASE-GROUP 3 (this pass): cases 0xa-0x11, TxHdxTRN - see src/v32/v32_engine.c's own
 * large header comment (right before V32OrgNextState's definition) for the
 * full case-by-case derivation and the disassembly-diff confirmation that
 * case 0x11 shares case 7's own physical machine-code tail. Continuing the
 * same handler-install-timing rule established by the prior two
 * phase-groups (case N's body fires WHILE at substate N, installing the
 * handler(s) active for substate N+1):
 *   substate 10: TX=TxHdxNoCarrier, RX=RxHdxNull (already exercised by
 *               phase-group 2's own batch) - drive 10->11 via TX ticking.
 *   substate 11: TX=TxHdxCarrierState, RX=RxHdxData become active HERE
 *               (case 0xa's own transition INTO 11) - both already verified
 *               elsewhere, just drive forward.
 *   substate 12: handlers UNCHANGED from substate 11 (case 0xb doesn't touch
 *               either) - just drive forward.
 *   substate 13: TX=TxHdxTRN becomes active HERE (case 0xc's own transition
 *               INTO 13) - THE new handler this phase-group hand-cleans.
 *               RX=RxHdxData stays unchanged (case 0xc never touches it).
 *               Driven via `tx_drive_until_transition` (through
 *               V32TxHdxModem, not a direct call) - SAFE here even though
 *               TxHdxTRN is brand new, because the target this transition
 *               installs (TxHdxScrSequence, case 0xd) is ALREADY verified,
 *               unlike the prior chunk's state4->5 workaround where the
 *               target was still a stub.
 *   substate 14/15: TX=TxHdxScrSequence (already verified) - just drive
 *               forward through cases 0xd/0xe's own budget-only transitions.
 *   substate 16 (0x10): RX=RxHdxSTone (reinstall, case 0xf's own transition
 *               INTO 0x10) - already verified/reused, no new batch needed.
 *   substate 17 (0x11): RX=RxHdxEpoch (reinstall, case 0x10's own transition
 *               INTO 0x11) - already verified/reused; drive forward via TX
 *               to exercise case 0x11's own transition, the last one in this
 *               phase-group's scope.
 *   substate 18 (0x12): STOP here - case 0x12 is the NEXT phase-group's own
 *               scope (still `v32_unported()` on the port side). RX=RxHdxData
 *               is active (case 0x11's own install) - safe for one final
 *               small confirmation batch (RxHdxData never calls NextState),
 *               but this test never ticks TX again past this point.
 *
 * `CodeRateSeq`'s own rate-class cascade (case 0xf) is NOT forced with a
 * poked register value - per this project's "stay at a confirmed
 * 4800bps/rate-class-0 outcome, don't guess at bit patterns" discipline, its
 * INPUT (reg1, captured by case 0xa's own `StoreReg(...,1)` from whatever
 * `GetSequence` returned) is whatever the blob/port's own byte-exact-so-far
 * bit-serializer state produces - since both sides have been running
 * byte-identically through every prior state, this input is GUARANTEED
 * identical between blob and port without needing a poke, so a faithful
 * `CodeRateSeq` translation is proven bit-exact for whatever value actually
 * occurs, regardless of what rate class it happens to decode to (case 0xf
 * only ENCODES this object's own already-configured rate-class-0 capability
 * into a transmitted code word - it never decodes/branches on a FAR-END
 * class, so there is no risk of landing on an untested negotiated rate).
 *
 * ==========================================================================
 * THIS PASS'S OWN EXTENSION (cases 5-9 + TxHdxNoCarrier/RxHdxSTone/
 * RxHdxEpoch/RxHdxRateSequence/RxHdxNull) - CRITICAL ORDERING CONSTRAINT
 * ==========================================================================
 * Handler-install timing, worked out case-by-case from the raw decompile
 * (matches src/v32/v32_engine.c's own big header comment): case N's body fires WHILE
 * the object is AT substate N, and installs the handler(s) active for
 * substate N+1. So:
 *   substate 5: TX=TxHdxNoCarrier (real), RX=RxHdxNoSignal (STILL a stub -
 *               installed by case 4's own guard-pass, NOT case 5). Calling
 *               V32RxHdxModem/rx_micro_test while AT substate 5 would invoke
 *               RxHdxNoSignal and ABORT the port build - this test MUST
 *               drive substate 5->6 via TX-side ticking ONLY.
 *   substate 6: RX=RxHdxSTone becomes active HERE (installed by case 5's own
 *               transition INTO 6) - safe to batch-test now.
 *   substate 7: RX=RxHdxEpoch becomes active HERE (installed by case 6's
 *               transition INTO 7).
 *   substate 8: RX=RxHdxData (reinstall, already verified) - case 7's own
 *               transition INTO 8, plus the AGC alpha/beta pointer retune.
 *   substate 9: RX=RxHdxRateSequence becomes active HERE (installed by case
 *               8's transition INTO 9) - budget=phaseInc, so this state
 *               lingers as long as any other.
 *   substate 10: RX=RxHdxNull becomes active HERE (installed by case 9's
 *               transition INTO 10) - case 9's own TX budget is deliberately
 *               tiny (=symLen, not phaseInc), so 9->10 fires almost
 *               immediately once TX ticks at all. Case 0xa (next
 *               phase-group) is still `v32_unported()` -> abort(), so this
 *               test STOPS here and never ticks TX again past substate 10.
 * Since `V32TxHdxModem` (src/v32/v32_engine.c) only ever reads/writes +0x6c, never
 * +0x70, and TX stays `TxHdxNoCarrier` continuously from substate 4 through
 * 10 (every one of cases 5-9's own TX-handler writes is a same-value
 * reinstall, not a real change), driving forward via the EXISTING
 * `tx_drive_until_transition` helper (through `V32TxHdxModem`, not a direct
 * call) is safe for every one of these transitions - unlike the prior
 * chunk's state4->5 transition, there is no "hand a mid-block remainder to a
 * brand-new unported handler" risk here, since the TX handler never changes.
 *
 * `RxHdxRateSequence`'s own dispatch (a `DetSequence` correlator match, plus
 * a tautological self-consistency double-check on `GetSequence`'s packed
 * value - see src/v32/v32_engine.c's own header comment) is a data-dependent
 * condition this test does NOT attempt to force with synthetic noise (per
 * this project's own "stay at a confirmed 4800bps outcome, don't guess at
 * bit patterns" discipline) - its own per-call demod/descramble/correlate
 * logic is proven bit-exact via a bounded direct batch regardless of whether
 * a match happens to fire; the substate 9->10 transition is driven for real
 * via TX-side budget exhaustion instead, exactly like every other
 * unconditional case in this stretch.
 *
 * Same pattern as every other *_diff_main.c in this project: this ONE
 * source compiles into two binaries (test/v32/v32_engine.org.sh) - one
 * linked directly against the real dsplibs.o, one against src/v32/v32_engine.c + its
 * already-verified dependencies.
 *
 * ==========================================================================
 * WHY THIS TEST NEEDS NO "POKE THE ROLE FIELD" HACK (unlike the LocLoop
 * chunk's own harness)
 * ==========================================================================
 * Role 0 (Originate) is reached through the REAL, un-hacked `V32FP_create`
 * entry point directly (`cfg[0] = 0`) - no field poke needed anywhere. Per
 * `src/v32/v32_engine.c`'s own header comment (top-level-object-lifecycle chunk):
 * `V32FP_recreate`'s "if role==0 and skip-bit clear, do the case-0/1-
 * equivalent transition inline" branch is PROVABLY unreachable through any
 * real `V32FP_create` call whose caller's enable-word has bit0 set (which
 * `v32_create`'s own real convention, and this harness's own cfg below,
 * both do) - so a real role-0 object genuinely starts AT substate 0, with
 * `TxHdxNull`/`RxHdxTone` (construction-time defaults) as its active
 * handlers, exactly the starting point V32OrgNextState's cases 0/1 expect.
 *
 * ==========================================================================
 * A REAL FINDING WHILE DESIGNING THIS TEST: synthetic RX input essentially
 * NEVER makes RxHdxTone dispatch to V32OrgNextState on its own
 * ==========================================================================
 * `RxHdxTone` only calls `V32NextState` when `FPM_TONE_detect` returns
 * EXACTLY 0 - which (per that function's own arithmetic, already verified
 * elsewhere in `src/fpm/fpm_tone.c`) means "signal level above the noise floor
 * AND high ripple relative to level" (0=noisy-but-present, 1=clean/tone-
 * like, 2=too weak to judge). Empirically instrumenting the port side (a
 * temporary debug build, not checked in) against BOTH a deterministic LCG
 * noise generator and a deliberate alternating full-scale square wave
 * showed `detected` staying at 1 (occasionally 2 during the initial
 * IIR-average warm-up) for THOUSANDS of calls with either input - i.e.
 * within this test's practical reach, no plausible synthetic RX content
 * reliably produces the specific "0" result cases 0/1 wait for. This is
 * plausibly a real reflection of a real-world constraint too (a live
 * originate line before any real far-end tone appears may very well sit in
 * `RxHdxTone`'s "1" or "2" results for a long time, with the OTHER exit -
 * `RxHdxTone`'s own +0x7c>=+0x80 budget-exceeded idiom, forcing the
 * terminal give-up state 0x21 - being the actual common real-world path out
 * of states 0/1) - not something this chunk's own hand-clean pass is
 * positioned to resolve (that would need real protocol/telephony-level
 * research, out of scope for a faithful-translation pass).
 *
 * Given that, this test's own coverage strategy for cases 0/1 is split in
 * two, each proving something different and together adding up to real
 * coverage of everything this chunk touches:
 *   - `RxHdxTone`'s OWN per-call logic (accumulator growth, AGC, the
 *     tone-object correlator's internal state, the RxClampV32 tail) is
 *     proven bit-exact via a real, bounded batch of actual `V32RxHdxModem`
 *     calls with deterministic synthetic input - genuine coverage of the
 *     one new Hdx handler this chunk adds, independent of whether it
 *     happens to trigger a state transition.
 *   - `V32OrgNextState`'s cases 0/1 OWN transition logic (the exact field
 *     writes, and case 1's exact `0x95f` guard boundary) is proven via
 *     direct `V32OrgNextState(OBJ)` calls with `+0x7c` set to controlled
 *     values straddling the boundary - the same "poke the field, call the
 *     NextState function directly" technique the LocLoop chunk's own
 *     harness already established for states its real handlers couldn't
 *     practically drive to. This is MORE precise than an organic drive
 *     would be anyway (an organic run's +0x7c only lands on multiples of
 *     +0x9e=12, never testing the exact 2399/2400 boundary values
 *     directly).
 *
 * ==========================================================================
 * HOW THIS TEST DRIVES EVERY OTHER TRANSITION FOR REAL (see src/v32/v32_engine.c's
 * own large header comment right before V32OrgNextState's definition for
 * the full derivation - summarized here)
 * ==========================================================================
 *   - States 2/3: single-tick unconditional pass-throughs, driven by
 *     `TxHdxCarrierState`'s (already-verified) own independent per-tick
 *     budget exhaustion - exactly the LocLoop chunk's own established "TX
 *     drives transitions" test design (`tx_drive_until_transition` below,
 *     copied from that chunk's harness).
 *   - State 4: guarded on RxHdxPhsReversal's own +0xa8 phase-reversal-count
 *     accumulator (grown UNCONDITIONALLY every RxHdxPhsReversal call,
 *     match or no match) exceeding 0x48=72 - only 7 RX calls needed, well
 *     under the 25+-call zone NOTES.md's most recent V.32 LocLoop entry
 *     flags as having an unresolved RxHdxPhsReversal-own-transition-timing
 *     divergence (not yet root-caused). This test uses 8 RX calls (a hair
 *     over the minimum, comfortably clear of both the guard threshold and
 *     the risk zone), then TX-ticks until the now-satisfied guard fires.
 *   - Case 4's guard-pass installs `TxHdxNoCarrier`/`RxHdxNoSignal` - BOTH
 *     still genuinely out of scope (next phase-group's own work) - this
 *     test stops the INSTANT it observes substate becoming 5 and never
 *     calls either handler (both `abort()` on the port side).
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v32_hobj.h"

extern void *V32FP_create(int *cfg6, int param2);
extern void V32FP_delete(char *obj);

/* SetTxModeV32 - real GLOBAL exported symbol in both dsplibs.o and
 * src/v32/v32_datapath.c (readelf -sW dsplibs.o: FUNC GLOBAL @0x81680).
 * Declared here only for this file's own extra bigbuf+0x2c coverage sweep
 * at the end of main() - see the comment there for why. */
extern void SetTxModeV32(char *obj, short mode);

extern void V32OrgNextState(char *obj);
extern void V32TxHdxModem(char *obj, void *txBits, short *out, short *outCount);
extern short V32RxHdxModem(char *obj, void *rxSamples, void *outBits, short *outCount);

extern short TxHdxCarrierState(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxTone(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxNull(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxNoCarrier(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxScrSequence(char *obj, void *a2, void *a3, void *a4);
extern short TxHdxTRN(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxPhsReversal(char *obj, void *a2, void *a3, void *a4);
extern short RxHdxTone(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxNoSignal(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxError(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxData(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxSTone(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxEpoch(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxRateSequence(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxNull(char *obj, void *a2, void *a3, void *a4);
extern void RxHdxSequence(char *obj, void *a2, void *a3, void *a4);

/* Real GLOBAL ELF symbol (readelf -sW: "OBJECT GLOBAL DEFAULT 129
 * V32_S_DATA_COEF"), not one of this project's disambiguated-rename cases -
 * usable directly against both the blob and port builds. Used here only for
 * RxHdxSTone's own dump()-side "is the MTD coefficient pointer the
 * connected-data-phase table" flag (a portable boolean, not a raw address -
 * see dump()'s own comment). */
extern const short V32_S_DATA_COEF[15];

/* Real GLOBAL ELF symbols (readelf -sW - both are real exported functions in
 * dsplibs.o, not this project's own port-only names), used directly by this
 * phase-group's own test logic: `StoreReg` to PIN reg3 for the compatible-
 * rate-class-0 scenario (see this file's own header comment), `V32_CONNECT`
 * to look up the EXPECTED status30 code for that scenario rather than
 * hardcoding a bare "4" in the assertion below. */
extern void StoreReg(char *obj, short value, unsigned short idx);
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

/* Deterministic (not random - reproducible across blob/port runs, since
 * both read the SAME generated sequence, not something the code under test
 * produces) synthetic RX sample generator - same generator/seed as the
 * LocLoop chunk's own harness. */
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
  if (p == (void *)TxHdxNull) return "TxHdxNull";
  if (p == (void *)TxHdxNoCarrier) return "TxHdxNoCarrier";
  if (p == (void *)TxHdxScrSequence) return "TxHdxScrSequence";
  if (p == (void *)TxHdxTRN) return "TxHdxTRN";
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
  if (p == (void *)RxHdxNull) return "RxHdxNull";
  if (p == (void *)RxHdxSequence) return "RxHdxSequence";
  return "UNKNOWN-RX";
}

static void dump(const char *label)
{
  FpmTone *toneObj = V32H_TONE0(HDX);
  char *bigbuf = V32W_PG(char *, OBJ, 0x68);
  FpmMtd *mtdObj = V32H_MTD(HDX);
  int coefIsSData = H_MTD_COEF_IS(mtdObj, V32_S_DATA_COEF);
  /* AGC alpha/beta pointer fields (case 7's own bigbuf+0x1e4/+0x1e8 retune) -
   * dereferenced to a portable SHORT VALUE, never the raw pointer itself
   * (the blob/port binaries' own V32_AGC_DEF_ALPHA/BETA table addresses
   * differ, so comparing raw addresses would be a false mismatch). */
  short agcAlpha = H_AGC_FIRST(bigbuf, 0x1e4, alpha);
  short agcBeta = H_AGC_FIRST(bigbuf, 0x1e8, beta);
  printf("[%-24s] state=%d role=%d budget=%d elapsed=%d fixedBudget=%d "
         "e84=%d latch90=%d revcnt_a8=%d tx=%s rx=%s status30=%d "
         "status31=0x%02x tone(6,8,10)=%u,%u,%d "
         "gen(4a,4c,4e,50,52)=%d,%d,%d,%d,%u "
         "det(64,68)=%u,%u mtd(coefIsSData,10,12,14,16)=%d,%d,%d,%d,%d "
         "agc(alpha,beta)=%d,%d reg(0,1,2,3,4)=%d,%d,%d,%d,%d delta46=%d "
         "fse62=%d bigbuf(2c,2e)=%d,%d\n",
         label, (int)S16(HDX + 0x74), (int)S16(HDX + 0x76), S32(HDX + 0x78), S32(HDX + 0x7c),
         S32(HDX + 0x80), (int)S16(HDX + 0x84), S32(HDX + 0x90), (int)S16(HDX + 0xa8),
         tx_handler_name(V32H_PG(void *, HDX, 0x6c)),
         rx_handler_name(V32H_PG(void *, HDX, 0x70)), (int)(signed char)U8(OBJ + 0x30),
         (unsigned)U8(OBJ + 0x31), (unsigned)H_TONE_US(toneObj, 6), (unsigned)H_TONE_US(toneObj, 8),
         (int)H_TONE_S(toneObj, 10), (int)S16(HDX + 0x4a), (int)S16(HDX + 0x4c), (int)S16(HDX + 0x4e),
         (int)S16(HDX + 0x50), (unsigned)U16(HDX + 0x52),
         U32(HDX + 0x64), U32(HDX + 0x68),
         coefIsSData, (int)H_MTD_S(mtdObj, 0x10), (int)H_MTD_S(mtdObj, 0x12), (int)H_MTD_S(mtdObj, 0x14),
         (int)H_MTD_S(mtdObj, 0x16),
         (int)agcAlpha, (int)agcBeta,
         /* reg0/reg1/reg2 (hdxobj+0x3c/0x3e/0x40) - captured/consumed by
          * cases 0xa (reg0 read, reg1 write)/0xb (reg0 read)/0xf (reg1
          * read, reg2 write); reg3 (hdxobj+0x42) - captured by case 0x13,
          * consumed/decoded by cases 0x14/0x15/0x17 (this phase-group's own
          * new coverage, INCLUDING the direct poke used to pin rate class 0
          * - see this file's own header comment); reg4 (hdxobj+0x44) -
          * captured by case 0x16 (also new this phase-group) - and the
          * +0x46 timing-compensation delta case 0x11 writes. */
         (int)S16(HDX + 0x3c), (int)S16(HDX + 0x3e), (int)S16(HDX + 0x40), (int)S16(HDX + 0x42),
         (int)S16(HDX + 0x44),
         (int)S16(HDX + 0x46),
         /* fseObj+0x62 (bigbuf+0x230's own sub-object) - unconditionally
          * cleared by case 0x17's own terminal transition (this
          * phase-group's own new coverage; also read/cleared piecemeal by
          * RetrainDetectV32/RenegotiateDetectV32, out of this file's scope). */
         (int)U16(V32E_PG(char *, bigbuf, 0x230) + 0x62),
         /* bigbuf+0x2c/+0x2e - the "currently CONNECTED tx/rx rate class"
          * fields (as opposed to bigbuf+0x28/+0x2a, the TARGET/negotiated
          * rate class V32FP_recreate writes - see NOTES.md's "bigbuf+0x2c"
          * writeup). Writer confirmed via disassembly to be
          * SetTxModeV32/SetRxModeV32 (src/v32/v32_datapath.c, @0x81680/
          * @0x81890 in dsplibs.o) - both already called throughout this
          * test's own case 0/1/0x15/0x16/0x17 (or their RngInit/RngResp/Ans
          * counterparts). Added here so every dump() call in this test -
          * including the incompatible-class-6 CONNECT and the
          * compatible-class-0 CONNECT below - actually compares this field
          * pair between blob and port, closing the "never compared"
          * coverage gap NOTES.md flagged. */
         (int)S16(bigbuf + 0x2c), (int)S16(bigbuf + 0x2e));
}

/* Drives V32RxHdxModem `n` times with varying synthetic samples at the
 * object's own natural per-call sample count, dumping every call - used for
 * state 4's own small, deliberately-bounded RxHdxPhsReversal batch. */
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
 * from `fromState` - this is what exercises the real TxHdxCarrierState ->
 * V32NextState[0] -> V32OrgNextState dispatch chain for real. Copied from
 * the LocLoop chunk's own already-established harness pattern. */
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

/* Drives `TxHdxCarrierState` DIRECTLY (NOT through `V32TxHdxModem`'s own
 * wrapping loop) until hdxobj+0x74 changes away from `fromState`.
 *
 * WHY THIS EXISTS, NOT JUST `tx_drive_until_transition` AGAIN (a real
 * finding from building this test): `V32TxHdxModem` re-reads hdxobj+0x6c
 * and calls it AGAIN, WITHIN THE SAME V32TxHdxModem call, whenever a
 * handler's own budget-exhaustion transition consumes FEWER samples than
 * the block still has remaining (`TxHdxCarrierState` decrements +0x78 by
 * the FULL requested count but only PRODUCES `min(budget,requested)`
 * samples - see its own header comment - so a transition landing mid-block
 * leaves a nonzero remainder that `V32TxHdxModem`'s loop immediately hands
 * to whatever handler the transition JUST installed). For every OTHER
 * transition in this test, that target handler is already hand-cleaned
 * (TxHdxCarrierState itself, unchanged across cases 2/3), so this is
 * harmless. But case 4's own guard-pass installs `TxHdxNoCarrier` - still a
 * genuine `abort()` stub - and `12922 (this object's own +0x80) % 12
 * (+0x9e) != 0`, so the state4->5 transition WILL generally land mid-block
 * and `V32TxHdxModem` WOULD hand the leftover samples straight to the
 * brand-new `TxHdxNoCarrier`, aborting the port build. Calling
 * `TxHdxCarrierState` directly avoids this entirely: this helper's own
 * loop (not `V32TxHdxModem`'s) decides whether to call again, and it stops
 * the instant it observes the transition, before ever touching whatever
 * got installed. */
static int tx_drive_carrierstate_until_transition(const char *label, short fromState, int maxIters)
{
  int i;
  short symLen = S16(HDX + 0x9e);
  if (symLen <= 0 || symLen > MAXBUF) symLen = 1;
  for (i = 0; i < maxIters; i++) {
    short count = symLen;
    char tag[64];
    memset(txBitsBuf, 0, sizeof(txBitsBuf));
    memset(txOutBuf, 0, sizeof(txOutBuf));
    TxHdxCarrierState(OBJ, txBitsBuf, txOutBuf, &count);
    snprintf(tag, sizeof(tag), "%s tx#%d", label, i);
    dump(tag);
    if (S16(HDX + 0x74) != fromState) {
      printf("%s: transitioned away from state %d after %d TxHdxCarrierState call(s)\n", label,
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
  cfg[0] = 0; /* role 0 = Originate - the REAL role, no poke needed */
  cfg[1] = 100;
  cfg[2] = 60000;
  cfg[3] = 700;
  cfg[4] = 1; /* enable bit0 - matches v32_create's own real convention (sets
               * the skip-bit, per src/v32/v32_engine.c's header comment, so
               * V32FP_recreate's dead role==0 inline shortcut is skipped -
               * the object genuinely starts at substate 0). */
  *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16); /* 4800bps, rate class 0 */

  p = V32FP_create(cfg, 0);
  if (!p) {
    printf("create failed\n");
    return 1;
  }
  OBJ = (char *)(p);
  HDX = V32W_PG(char *, OBJ, 0x64);

  printf("== V.32 Originate NextState, phase-groups 1+2 (cases 0-9 + 0x21) ==\n");
  dump("after create");

  /* ---- RxHdxTone's OWN per-call logic, proven via a real, bounded batch
   * of actual calls (see this file's header comment for why this does NOT
   * rely on it naturally triggering a state transition) ---- */
  rx_micro_test("state0(RxHdxTone)", 12);

  /* ---- case 0 (substate 0 -> 1), driven via a direct V32OrgNextState
   * call - case 0 has NO internal guard, so a direct call exercises it
   * completely and faithfully regardless of how it's reached in practice.
   * (Substate is still 0 here - the rx_micro_test batch above never
   * naturally triggered a transition, per this file's header comment.) ---- */
  if (S16(HDX + 0x74) != 0) {
    printf("UNEXPECTED: substate is already %d before the direct case-0 call "
           "(rx_micro_test triggered a natural transition after all)\n",
           (int)S16(HDX + 0x74));
  }
  V32OrgNextState(OBJ);
  dump("case0->1(direct)");

  /* ---- case 1, "guard not yet met" branch: +0x7c just BELOW the 0x95f
   * threshold - must be a pure no-op (still substate 1) ---- */
  S32(HDX + 0x7c) = 0x900;
  V32OrgNextState(OBJ);
  dump("case1(not yet, direct)");

  /* ---- case 1, "guard passed" branch: +0x7c just ABOVE the 0x95f
   * threshold - installs TxHdxCarrierState/RxHdxPhsReversal, re-tunes the
   * tone object, resets budget/accumulator ---- */
  S32(HDX + 0x7c) = 0x960;
  V32OrgNextState(OBJ);
  dump("case1->2(direct)");

  /* From here on TxHdxCarrierState/RxHdxPhsReversal (installed by case 1's
   * own guard-pass body) are the real active handlers - every further
   * transition is driven for real through them, not by direct calls. */

  /* ---- state 2 -> 3, single-tick pass-through, driven by TX ticking ---- */
  tx_drive_until_transition("state2->3(TxHdxCarrierState)", 2, 20000);

  /* ---- state 3 -> 4, single-tick pass-through, driven by TX ticking ---- */
  tx_drive_until_transition("state3->4(TxHdxCarrierState)", 3, 20000);

  /* ---- state 4 -> 5: grow +0xa8 via a small, bounded RxHdxPhsReversal
   * batch (8 calls - past the 7-call guard threshold, comfortably clear of
   * the not-yet-root-caused 25+-call risk zone), then let TX ticking
   * re-invoke V32OrgNextState until it observes the guard has passed ---- */
  rx_micro_test("state4(RxHdxPhsReversal)", 8);
  tx_drive_carrierstate_until_transition("state4->5(TxHdxCarrierState)", 4, 20000);

  /* ==========================================================================
   * PHASE-GROUP 2 (this pass): cases 5-9, TxHdxNoCarrier/RxHdxSTone/
   * RxHdxEpoch/RxHdxRateSequence/RxHdxNull. See this file's own header
   * comment for the critical substate/handler-install timing this section
   * depends on - substate 5's own RX handler (RxHdxNoSignal) is STILL a
   * stub, so this test drives 5->6 via TX ticking ONLY, never calling
   * V32RxHdxModem while substate==5.
   * ========================================================================== */

  /* ---- state 5 -> 6: TX-only ticking (TxHdxNoCarrier's own budget
   * exhaustion) - RxHdxNoSignal (still out of scope) must never be called ---- */
  if (S16(HDX + 0x74) != 5) {
    printf("UNEXPECTED: substate is %d, not 5, before the state5->6 TX-only "
           "drive\n", (int)S16(HDX + 0x74));
  }
  tx_drive_until_transition("state5->6(TxHdxNoCarrier)", 5, 20000);

  /* ---- state 6: RxHdxSTone is now the active RX handler (installed by
   * case 5's own transition into state 6) - safe to batch-test ---- */
  if (S16(HDX + 0x74) == 6) {
    rx_micro_test("state6(RxHdxSTone)", 6);
  } else {
    printf("NOTE: substate is %d, not 6, before state6's own RxHdxSTone "
           "batch\n", (int)S16(HDX + 0x74));
  }
  tx_drive_until_transition("state6->7(TxHdxNoCarrier)", S16(HDX + 0x74), 20000);

  /* ---- state 7: RxHdxEpoch is now the active RX handler (installed by
   * case 6's own transition into state 7) - safe to batch-test ---- */
  if (S16(HDX + 0x74) == 7) {
    rx_micro_test("state7(RxHdxEpoch)", 6);
  } else {
    printf("NOTE: substate is %d, not 7, before state7's own RxHdxEpoch "
           "batch\n", (int)S16(HDX + 0x74));
  }
  tx_drive_until_transition("state7->8(TxHdxNoCarrier)", S16(HDX + 0x74), 20000);

  /* ---- state 8: RxHdxData (reinstall, already verified elsewhere) plus the
   * AGC alpha/beta pointer retune (case 7's own transition into state 8) -
   * just drive TX forward, no new handler to batch-test here ---- */
  tx_drive_until_transition("state8->9(TxHdxNoCarrier)", S16(HDX + 0x74), 20000);

  /* ---- state 9: RxHdxRateSequence is now the active RX handler (installed
   * by case 8's own transition into state 9) - batch-test its own per-call
   * logic (demod/descramble/DetSequence/the two GetSequence calls), WITHOUT
   * relying on a real correlator match firing (see this file's header
   * comment) - then drive TX forward (case 9's own tiny symLen budget means
   * this should fire almost immediately) ---- */
  if (S16(HDX + 0x74) == 9) {
    rx_micro_test("state9(RxHdxRateSequence)", 6);
  } else {
    printf("NOTE: substate is %d, not 9, before state9's own RxHdxRateSequence "
           "batch\n", (int)S16(HDX + 0x74));
  }
  tx_drive_until_transition("state9->10(TxHdxNoCarrier)", S16(HDX + 0x74), 20000);

  /* ---- state 10: RxHdxNull is now the active RX handler (installed by
   * case 9's own transition into state 10) - batch-test its own per-call
   * logic (accumulator growth + budget check, no DemodDataV32 call at all). */
  if (S16(HDX + 0x74) == 10) {
    rx_micro_test("state10(RxHdxNull)", 6);
  } else {
    printf("NOTE: substate is %d, not 10, before state10's own RxHdxNull "
           "batch\n", (int)S16(HDX + 0x74));
  }

  /* ==========================================================================
   * PHASE-GROUP 3 (this pass): cases 0xa-0x11, TxHdxTRN. See this file's own
   * header comment for the substate/handler-install timing this section
   * depends on. TX ticking alone (via `tx_drive_until_transition`, through
   * `V32TxHdxModem`, not a direct call - safe throughout this whole stretch
   * since every transition target in it is already hand-cleaned/verified by
   * the time this test drives into it) walks every one of these
   * transitions forward, exactly like phase-groups 1/2's own established
   * design.
   * ========================================================================== */

  /* ---- state 10 -> 11 (case 0xa): installs TxHdxCarrierState/RxHdxData,
   * both already verified elsewhere - no new batch needed, just drive
   * forward ---- */
  tx_drive_until_transition("state10->11(TxHdxNoCarrier)", S16(HDX + 0x74), 20000);

  /* ---- state 11 -> 12 (case 0xb): handlers UNCHANGED (TxHdxCarrierState/
   * RxHdxData) - InitGenSequence/SetECRndTripDelayV32 only ---- */
  tx_drive_until_transition("state11->12(TxHdxCarrierState)", S16(HDX + 0x74), 20000);

  /* ---- state 12 -> 13 (case 0xc): installs TxHdxTRN - THE new handler
   * this phase-group hand-cleans. Driving 12->13 exercises case 0xc's own
   * transition; the substate-13 TX ticks immediately below are what
   * actually exercise TxHdxTRN's own per-call body (GenSequence/
   * ScrambleDataV32/the dibit-remap loop/ModDataV32) for real, budget=0x100
   * (256 samples) worth of calls ---- */
  tx_drive_until_transition("state12->13(TxHdxCarrierState)", S16(HDX + 0x74), 20000);
  if (S16(HDX + 0x74) == 13) {
    if (V32H_PG(void *, HDX, 0x6c) != (void *)TxHdxTRN) {
      printf("UNEXPECTED: substate is 13 but TX handler is %s, not TxHdxTRN\n",
             tx_handler_name(V32H_PG(void *, HDX, 0x6c)));
    }
  } else {
    printf("NOTE: substate is %d, not 13, before TxHdxTRN's own drive\n",
           (int)S16(HDX + 0x74));
  }

  /* ---- state 13 -> 14 (case 0xd): installs TxHdxScrSequence (already
   * verified) - this transition is what proves TxHdxTRN's own budget
   * exhaustion/NextState dispatch is bit-exact ---- */
  tx_drive_until_transition("state13->14(TxHdxTRN)", S16(HDX + 0x74), 20000);

  /* ---- state 14 -> 15 (case 0xe): TX stays TxHdxScrSequence, budget=0x400 ---- */
  tx_drive_until_transition("state14->15(TxHdxScrSequence)", S16(HDX + 0x74), 20000);

  /* ---- state 15 -> 16 (case 0xf): the big one - CodeRateSeq/StoreReg,
   * RxHdxSTone reinstall, FPM_MTD_create(mtdObj,mtdObj), InitGenSequence
   * with the coded rate, InitDetSequence, SetTxModeV32(1),
   * FPM_AGC_Release, AGC alpha/beta -> index [0] ---- */
  tx_drive_until_transition("state15->16(TxHdxScrSequence)", S16(HDX + 0x74), 20000);

  /* ==========================================================================
   * A REAL, CONFIRMED CRASH FOUND WHILE DESIGNING THIS SECTION - cases 0x10/
   * 0x11 are driven via DIRECT V32OrgNextState() calls, NOT organic TX
   * ticking, and here is why
   * ==========================================================================
   * Case 0x10's own body (see src/v32/v32_engine.c's case 0x10 comment) deliberately
   * does NOT reset budget (+0x78) - disassembly-confirmed absent - so
   * whatever's left of case 0xf's own `budget=phaseInc` (=12922 for this
   * object) after TxHdxScrSequence has been ticking it down by symLen(=12)
   * each call carries UNCHANGED into state 17. Since 12922 is NOT an exact
   * multiple of 12 (12922 % 12 = 10 - already flagged as a known
   * non-dividing pair by an earlier chunk's own TxHdxNoCarrier
   * investigation), the tick that finally exhausts it leaves budget at a
   * small NEGATIVE residual (e.g. -2) rather than exactly 0 - and case 0x11
   * is DESIGNED to read that exact residual (the whole point of its own
   * `+0x46 = phaseInc - oldBudget` timing-compensation calculation, per
   * src/v32/v32_engine.c's own case 0x11 comment). The problem: once +0x78 is
   * negative, `TxHdxScrSequence`'s own budget/produced-count clamp
   * (`if ((int)budget <= (int)produced) produced = budget & 0xffff;`)
   * reinterprets that small negative int as a HUGE unsigned 16-bit count
   * (e.g. -2 -> 65534) the FIRST time it's called again with budget already
   * negative - corrupting GenSequence/ScrambleDataV32/ModDataV32's own
   * buffer writes far past any reasonable bound.
   *
   * An EARLIER version of this test drove state 16->17 (and onward) via the
   * ordinary `tx_drive_until_transition` helper, same as every other
   * transition in this file - and it segfaulted, identically, on BOTH the
   * dsplibs.o-linked blob binary AND the port binary (same call count, same
   * budget-decay pattern leading up to it: `budget` counting down by 12
   * every call, crashing the instant it would have gone negative). This
   * CONFIRMS it is a genuine property of the real object under this exact
   * synthetic scenario, NOT a port bug - and, per this project's own
   * "differential test doesn't imply the port is correct if BOTH sides do
   * the same wrong thing" caveat, this specific negative-budget-clamp
   * interaction is exactly the kind of "organically driving a case well
   * past what its real design intends" risk the LocLoop chunk's own header
   * comment already warned about: `RxHdxEpoch`'s own `EpochDetectV32` match
   * is almost certainly the REAL, intended way this state normally advances
   * in practice (a real symbol-timing lock, reached well before a
   * multi-thousand-sample TX budget could ever run out) - synthetic,
   * signal-free RX input can't organically trigger that (same "no natural
   * match" property every earlier phase-group's own header comment already
   * established for `FPM_TONE_find_rev`/`DetSequence`). So, per this
   * project's own "poke the field, call NextState directly" technique
   * (already used for cases 0/1/0x21 above), cases 0x10/0x11's OWN
   * transition logic is proven via direct calls instead - genuine coverage
   * of the translation, without organically manufacturing a scenario the
   * real design doesn't intend and that crashes BOTH binaries alike. */

  /* ---- state 16 (0x10): RxHdxSTone is the active RX handler (reinstall,
   * already verified/reused - no new batch needed) - case 0x10's own
   * transition has no guard at all, so a direct call exercises it
   * completely and faithfully regardless of the current (unmodified)
   * budget value. ---- */
  if (S16(HDX + 0x74) != 0x10) {
    printf("UNEXPECTED: substate is %d, not 0x10, before the direct case-0x10 "
           "call\n", (int)S16(HDX + 0x74));
  }
  V32OrgNextState(OBJ);
  dump("case0x10->0x11(direct)");

  /* ---- state 17 (0x11): RxHdxEpoch is the active RX handler (reinstall,
   * already verified/reused) - a small bounded RX batch for extra coverage
   * (always safe regardless of +0x78's own value - RxHdxEpoch never reads
   * or writes it), then case 0x11's OWN transition (the last one in this
   * phase-group's scope - saves the +0x46 timing delta, installs
   * RxHdxData, SetAdaptEqV32(2), AGC alpha/beta -> index [1], the
   * disassembly-CONFIRMED case-7-shared tail) is likewise proven via a
   * direct call. ---- */
  if (S16(HDX + 0x74) == 0x11) {
    rx_micro_test("state0x11(RxHdxEpoch)", 6);
  } else {
    printf("NOTE: substate is %d, not 0x11, before state0x11's own RxHdxEpoch "
           "batch\n", (int)S16(HDX + 0x74));
  }
  V32OrgNextState(OBJ);
  dump("case0x11->0x12(direct)");

  /* ---- state 18 (0x12): RxHdxData is active (case 0x11's own install) -
   * safe for one small confirmation batch, since RxHdxData never calls
   * NextState ---- */
  if (S16(HDX + 0x74) == 0x12) {
    rx_micro_test("state0x12(RxHdxData)", 6);
  } else {
    printf("NOTE: substate is %d, not 0x12, before the state0x12 "
           "RxHdxData batch\n", (int)S16(HDX + 0x74));
  }

  /* ==========================================================================
   * PHASE-GROUP 4 (this pass): cases 0x12-0x17, DecodeRateSeq/CodeESeq. See
   * this file's own header comment for the full rate-class derivation this
   * section depends on. TX ticking alone (via `tx_drive_until_transition`,
   * through `V32TxHdxModem`, not a direct call) walks every transition in
   * this stretch forward, exactly like phase-group 3's own established
   * design - EVERY case in this range explicitly resets budget to a fresh,
   * small, non-negative value as part of its own transition (case map:
   * 0x12=phaseInc, 0x13=8-((...)&7) [range 1-8], 0x14=8, 0x15=phaseInc,
   * 0x16=0x80, 0x17=phaseInc), so none of them share case 0x10's own
   * "doesn't reset budget" property - the earlier chunk's confirmed crash
   * risk (a stale NEGATIVE budget residual reinterpreted as a huge unsigned
   * count by TxHdxScrSequence's own clamp) does NOT apply anywhere in this
   * phase-group.
   * ========================================================================== */

  /* ---- state 18 -> 19 (case 0x12): installs RxHdxSequence - the ONE Hdx
   * handler this phase-group needs, already hand-cleaned/verified by the
   * LocLoop chunk, reused as-is. ---- */
  tx_drive_until_transition("state0x12->0x13(TxHdxScrSequence)", S16(HDX + 0x74), 20000);

  /* ---- state 19: RxHdxSequence is now the active RX handler - batch-test
   * its own per-call logic in THIS call context (demod/descramble/
   * DetSequence), same "don't rely on a real correlator match firing"
   * discipline as every earlier RxHdx* batch in this file - this is also
   * what empirically establishes that GetSequence() reads back 0 at case
   * 0x13 below (no match ever occurs). ---- */
  if (S16(HDX + 0x74) == 0x13) {
    rx_micro_test("state0x13(RxHdxSequence)", 6);
  } else {
    printf("NOTE: substate is %d, not 0x13, before state0x13's own "
           "RxHdxSequence batch\n", (int)S16(HDX + 0x74));
  }

  /* ---- state 19 -> 20 (case 0x13): the +0x46-delta budget formula, and
   * reg3's own first capture (GetSequence(), expected 0 per the header
   * comment's own derivation - confirmed below once case 0x14 has run). ---- */
  tx_drive_until_transition("state0x13->0x14(TxHdxScrSequence)", S16(HDX + 0x74), 20000);

  /* ---- state 20 -> 21 (case 0x14): the crucial rate-negotiation DECODE
   * step - DecodeRateSeq(reg3) on the naturally-captured reg3=0 value
   * decodes to class 6 ("incompatible"), so this drive exercises the
   * non-fatal status31|=2/status30=0x17 fork FOR REAL, organically (not
   * forced) - confirmed by inspecting status30/status31 in the dump below,
   * right after this transition fires. ---- */
  tx_drive_until_transition("state0x14->0x15(TxHdxScrSequence)", S16(HDX + 0x74), 20000);
  if ((int)(signed char)U8(OBJ + 0x30) == 0x17 && (U8(OBJ + 0x31) & 2) != 0) {
    printf("CONFIRMED: case 0x14's non-fatal 'incompatible rate' branch fired "
           "organically (status30=0x17, status31 bit1 set)\n");
  } else {
    printf("NOTE: case 0x14's 'incompatible rate' branch did NOT fire "
           "(status30=%d, status31=0x%02x) - reg3 must not have been 0 after "
           "all; see this file's own header comment for the expected "
           "derivation\n",
           (int)(signed char)U8(OBJ + 0x30), (unsigned)U8(OBJ + 0x31));
  }

  /* ---- state 21 -> 22 (case 0x15): DecodeRateSeq(reg3)->SetTxModeV32 -
   * V32_TX_MODE[6]=7 falls into SetTxModeV32's own safe "unrecognized mode"
   * default branch (src/v32/v32_datapath.c), so this is safe to drive
   * organically for the incompatible-class-6 path without any special
   * pinning. ---- */
  tx_drive_until_transition("state0x15->0x16(TxHdxScrSequence)", S16(HDX + 0x74), 20000);

  /* ---- state 22 -> 23 (case 0x16): reg4's own capture, DecodeRateSeq(reg3
   * captured via GetSequence)->SetRxModeV32 - V32_RX_MODE[6]=7 likewise
   * falls into SetRxModeV32's own safe default branch (NOT one of the
   * VTBv32_init-calling cases 3-6/modes 3-6). ---- */
  tx_drive_until_transition("state0x16->0x17(TxHdxScrSequence)", S16(HDX + 0x74), 20000);

  /* ---- state 23 (0x17): the TERMINAL "connected" self-loop has been
   * reached for the first time - dump immediately, then confirm the
   * incompatible-class-6 CONNECT code (V32_CONNECT[6]=14) landed in
   * status30, and that status31's final "connected" flags (0x19) are set. ---- */
  dump("state0x17(reached, incompatible)");
  if (S16(HDX + 0x74) == 0x17) {
    if ((int)(signed char)U8(OBJ + 0x30) == (int)V32_CONNECT[6] &&
        (U8(OBJ + 0x31) & 0x19) == 0x19) {
      printf("CONFIRMED: case 0x17's terminal transition landed the "
             "incompatible-class-6 CONNECT code (status30=%d=V32_CONNECT[6], "
             "status31 0x19 flags set)\n",
             (int)V32_CONNECT[6]);
    } else {
      printf("NOTE: case 0x17 reached but status30/status31 don't match the "
             "expected incompatible-class-6 values (status30=%d, "
             "status31=0x%02x, expected status30=%d)\n",
             (int)(signed char)U8(OBJ + 0x30), (unsigned)U8(OBJ + 0x31),
             (int)V32_CONNECT[6]);
    }
  } else {
    printf("UNEXPECTED: substate is %d, not 0x17, after the state0x16->0x17 "
           "drive\n", (int)S16(HDX + 0x74));
  }

  /* ---- re-exercise the SELF-LOOP for real (not just observe its first
   * arrival): tick TX enough times to force AT LEAST one more full
   * budget-exhaustion cycle (budget=phaseInc=~12922, symLen=12, so ~1078
   * calls) and confirm the state stays 0x17 throughout - proving case
   * 0x17's own re-entrant body (accumulator=0/budget=phaseInc/
   * DecodeRateSeq/status writes/fseObj+0x62 clear/SetAdaptEqV32/
   * SetAdaptEcV32, all over again) is bit-exact on a SECOND firing, not just
   * the first. ---- */
  {
    int i;
    int selfLoopIters = 1200;
    int leftState = 0;
    for (i = 0; i < selfLoopIters; i++) {
      short outCount = 0;
      memset(txBitsBuf, 0, sizeof(txBitsBuf));
      memset(txOutBuf, 0, sizeof(txOutBuf));
      V32TxHdxModem(OBJ, txBitsBuf, txOutBuf, &outCount);
      if (S16(HDX + 0x74) != 0x17) {
        printf("UNEXPECTED: state left 0x17 during the self-loop "
               "re-exercise batch (i=%d, state=%d)\n", i, (int)S16(HDX + 0x74));
        leftState = 1;
        break;
      }
    }
    if (!leftState) {
      printf("state0x17: self-loop re-exercised for %d V32TxHdxModem calls, "
             "state stayed 0x17 throughout\n", selfLoopIters);
    }
    dump("state0x17(self-loop re-exercised, incompatible)");
  }

  /* ---- one final small RxHdxData confirmation batch at the terminal state
   * (RxHdxData never calls NextState, so this is always safe regardless of
   * budget) ---- */
  if (S16(HDX + 0x74) == 0x17) {
    rx_micro_test("state0x17(RxHdxData, incompatible)", 6);
  }

  /* ==========================================================================
   * THE COMPATIBLE-RATE (CLASS 0) HAPPY PATH - pinned via a direct reg3
   * poke, per this file's own header comment. Reuses the SAME object (its
   * own real handshake state, sub-objects, etc. are all already valid and
   * untouched by anything below) - rewinds the substate field back to 0x14
   * and re-drives cases 0x14-0x17 via direct, guard-free V32OrgNextState()
   * calls.
   * ========================================================================== */
  printf("== pinning rate class 0 (compatible) via a direct reg3 poke, "
         "re-driving cases 0x14-0x17 ==\n");
  StoreReg(OBJ, 0x400, 3); /* bit 0x400 is the ONLY bit DecodeRateSeq tests
                             * against this object's own local=3345 value -
                             * see this file's own header comment. */
  S16(HDX + 0x74) = 0x14;
  S32(HDX + 0x7c) = 0;

  V32OrgNextState(OBJ); /* case 0x14: DecodeRateSeq(0x400) -> class 0, the
                          * compatible branch (status31/status30 NOT
                          * touched this time - case 0x14 only writes them
                          * on class==6). */
  dump("case0x14->0x15(direct, class0 pin)");

  V32OrgNextState(OBJ); /* case 0x15: SetTxModeV32(V32_TX_MODE[0]) - the
                          * REAL, safe mode-1 branch (not the generic
                          * default branch the incompatible path above
                          * exercised). */
  dump("case0x15->0x16(direct, class0 pin)");

  V32OrgNextState(OBJ); /* case 0x16: SetRxModeV32(V32_RX_MODE[0]) - same
                          * REAL mode-1 branch RX-side. */
  dump("case0x16->0x17(direct, class0 pin)");

  V32OrgNextState(OBJ); /* case 0x17: the terminal CONNECT transition, this
                          * time landing the REAL class-0 CONNECT code. */
  dump("case0x17(direct, class0 pin - CONNECTED)");

  if (S16(HDX + 0x74) == 0x17) {
    if ((int)(signed char)U8(OBJ + 0x30) == (int)V32_CONNECT[0] &&
        (U8(OBJ + 0x31) & 0x19) == 0x19) {
      printf("CONFIRMED: pinned rate class 0 reached the REAL CONNECTED "
             "state (status30=%d=V32_CONNECT[0]=V32_CONNECT class 0, "
             "status31 0x19 flags set) - this is the actual "
             "4800bps-connect code slmodemd surfaces to userspace\n",
             (int)V32_CONNECT[0]);
    } else {
      printf("NOTE: pinned class-0 path reached 0x17 but status30/status31 "
             "don't match the expected class-0 CONNECT values (status30=%d, "
             "status31=0x%02x, expected status30=%d)\n",
             (int)(signed char)U8(OBJ + 0x30), (unsigned)U8(OBJ + 0x31),
             (int)V32_CONNECT[0]);
    }
  } else {
    printf("UNEXPECTED: substate is %d, not 0x17, after the pinned "
           "class-0 direct-call sequence\n", (int)S16(HDX + 0x74));
  }

  /* ---- one final small RxHdxData confirmation batch at the terminal state,
   * this time in the real class-0-connected configuration ---- */
  if (S16(HDX + 0x74) == 0x17) {
    rx_micro_test("state0x17(RxHdxData, class0 pin)", 6);
  }

  /* ---- explicit direct-call coverage of the universal case 0x21, not
   * otherwise naturally exercised in a controlled way above (same
   * precedent as the LocLoop chunk's own harness) ---- */
  S16(HDX + 0x74) = 0x21;
  S32(HDX + 0x78) = -999;
  V32OrgNextState(OBJ);
  dump("case0x21(direct)");

  /* ---- bigbuf+0x2c direct-call sweep -----------------------------------
   * Everything above only ever drives SetTxModeV32/SetRxModeV32 with mode
   * 0/1/7(default) - this object's own rate-sequence value (V32_RATE_SEQ[0])
   * makes DecodeRateSeq organically return ONLY class 0 or class 6 (see
   * DecodeRateSeq's own header comment in src/v32/v32_engine.c: every OTHER return
   * path is provably dead for this object's fixed `local` bit pattern), so
   * bigbuf+0x2c/+0x2e can never go non-zero via the natural/pinned
   * NextState path alone - modes 0/1 both write bigbuf+0x2c=0, and mode 7
   * (class 6, "incompatible") hits SetTxModeV32's generic default branch,
   * which does NOT touch bigbuf+0x2c at all (disassembly-confirmed @0x81697
   * dsplibs.o - the default case only sets the status bytes, then falls
   * into the shared tail without a `movw ...,0x2c(%edx)`). Reaching a
   * REAL non-zero bigbuf+0x2c write would need rate class 1-5, which only
   * happens through SetRxModeV32's OWN modes 2-6 - and those call
   * VTBv32_init (still an explicit out-of-scope stub - see NOTES.md item
   * (e)) on the RX side, so driving there through the real handshake is
   * unsafe/out of scope for this chunk.
   *
   * SetTxModeV32 itself, though, is already a fully real, disassembly-
   * verified function - and its own modes 2-6 do NOT call VTBv32_init at
   * all (confirmed: no `call VTBv32_init` anywhere in SetTxModeV32's
   * disassembly, unlike its SetRxModeV32 sibling - only SMCv32/VTBv32
   * constellation MAP table pointer stores). So calling SetTxModeV32(OBJ, mode) directly
   * for mode=2..6 here is safe in BOTH binaries (same "call a proven-real
   * primitive directly, sidestepping only the out-of-scope stub"
   * precedent as StoreReg's own reg3 poke above) and gives this chunk's
   * required bigbuf+0x2c coverage a genuinely NON-trivial value in each
   * case (1/1/3/4/5 per SetTxModeV32's own case map), not just the "stays
   * 0" trivial value every case above produces - closing the "bigbuf+0x2c
   * literally never compared" gap with real, non-default content, not only
   * a default-value coincidence. bigbuf+0x2e is intentionally left
   * untouched by this sweep (still whatever the class-0 CONNECT path above
   * left it at) - only TX mode is swept, precisely because RX mode 2-6
   * is NOT safe to call (VTBv32_init). */
  {
    short mode;
    for (mode = 2; mode <= 6; mode++) {
      char tag[32];
      SetTxModeV32(OBJ, mode);
      snprintf(tag, sizeof(tag), "SetTxModeV32(direct,mode=%d)", (int)mode);
      dump(tag);
    }
  }

  V32FP_delete(OBJ);
  printf("== done ==\n");
  return 0;
}
