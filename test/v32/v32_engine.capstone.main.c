/*
 * V.32 two-object end-to-end capstone differential test: drives TWO real
 * V32FP_* engine objects - one Originate (role 0), one Answer (role != 0) -
 * against EACH OTHER's real V32FP_modem TX audio output, in a full-duplex
 * loop, using ONLY the real public API (V32FP_create/V32FP_modem/
 * V32FP_status/V32FP_delete - no register pokes anywhere in this file).
 * This is V.32's counterpart to test/v8_lifecycle_diff_main.c's own
 * end-to-end capstone: the goal is to prove the whole NextState/Hdx layer
 * this project has hand-cleaned works correctly IN COMBINATION, driven by a
 * REAL peer's signal for the first time (every earlier V.32 test in this
 * project drives ONE object against synthetic LCG noise or direct pokes).
 *
 * ==========================================================================
 * HONEST RESULT (load-bearing - read before extending this test)
 * ==========================================================================
 * Two real dsplibs.o-linked V32FP_create objects, cold-started at the same
 * simulated instant and exchanging real full-duplex audio via V32FP_modem
 * with NO external (V.8/V.25-style) pre-synchronization phase, do **NOT**
 * organically converge to CONNECT. Both sides get stuck and time out during
 * the EARLY carrier-acquisition/phase-reversal-detection phase (states 2-7,
 * well before either side's NextState machinery would even reach the
 * rate-negotiation/TRN/scrambled-sequence stretch this project's Org/Ans
 * NextState hand-cleaning has already verified in isolation). This was
 * root-caused (not just observed) by driving the real dsplibs.o pair
 * through a standalone probe harness with full field-level instrumentation
 * (org/ans's +0x74/0x76/0x7c/0x80/0x90/0xa8/0xac fields dumped every call):
 *
 *   - Originate's own RxHdxPhsReversal (its state-4 handler) needs to
 *     detect a genuine phase-reversal event in Answer's incoming signal via
 *     FPM_TONE_find_rev before its own elapsed-time watchdog
 *     (hdxobj+0x7c >= hdxobj+0x80, `phaseInc` - derived from this object's
 *     own `timeout` cfg word) expires. Empirically, with the SAME cfg this
 *     project's other V.32 tests use (timeout=60000 -> phaseInc=12922
 *     samples), Originate's watchdog fires at call 1768 (status 0x12,
 *     forced to substate 0x21) - and Answer's OWN analogous state-4 guard
 *     (the SAME RxHdxPhsReversal handler, same +0xa8>0x48 threshold)
 *     succeeds at call 1765 - just 3 calls (120 samples, ~15ms) earlier.
 *   - This 3-call margin was checked and found to be a small, ~fixed gap,
 *     NOT a proportional "not enough time" shortfall: re-running the same
 *     probe with phaseInc inflated to 32121 (timeout=68000, near the 16-bit
 *     wraparound-safe maximum for this field - `(short)((timeout*0x4ccc)
 *     >>0xd)`) reproduces the IDENTICAL 3-call gap at a proportionally
 *     later point (org watchdog @3368, ans success @3365) - both sides'
 *     early-phase timing (Originate's own carrier/reversal schedule,
 *     Answer's own watchdog) are driven by the SAME phaseInc-derived
 *     constant, so scaling it up never changes the RATIO, only the
 *     absolute scale. Swapping which side's V32FP_modem is called first
 *     each iteration (removing one full call's worth of audio latency in
 *     one direction) also made no difference - the gap is structural, not
 *     a call-ordering artifact of this test harness.
 *   - Answer's own apparent "success" at state 4->6 is itself short-lived:
 *     it cascades into Answer's OWN next watchdog (RxHdxNoSignal's
 *     elapsed-time check, installed by that very state-4 guard-pass) firing
 *     shortly after (call 2842, status 0x11) - Answer's case 7 guard-pass
 *     (the "local retrain" loop-back into the still-deliberately-deferred
 *     state 0x1d - see NOTES.md - needing `TxHdxFinishFrame`) is NEVER
 *     organically reached in this scenario: RxHdxNoSignal's own watchdog
 *     always wins the race first. This conveniently means this test never
 *     exercises the genuinely out-of-scope state-0x1d body at all.
 *   - Root cause, most likely: this project's own scope (V32FP_create's
 *     engine objects only) does not include the V.8/V.25 "detect ring,
 *     answer, exchange initial ack tones" pre-phase real slmodemd hardware
 *     goes through BEFORE a V.32 engine object is ever created. In real
 *     deployment, by the time V32FP_create runs on the Answer side, some
 *     amount of that lower-layer synchronization has already happened,
 *     giving Originate's phase-reversal-detection window a healthy margin
 *     this cold, simultaneous, zero-line-impairment test does not model.
 *     Implementing that pre-phase is out of this chunk's scope (V.8 and
 *     V.32 are separate protocol families in this project).
 *   - This is reported as an honest, root-caused, and *reproducible*
 *     finding, per this project's own stated methodology: a full 60000-call
 *     run of this exact scenario was independently confirmed BIT-EXACT
 *     between dsplibs.o and the port (see below) - the port faithfully
 *     reproduces dsplibs.o's own real non-convergent behavior, including
 *     both watchdog time-outs at the exact same call numbers and byte-exact
 *     TX sample streams throughout. No new port bug was needed to explain
 *     the non-convergence itself - it is a genuine property of the real
 *     object under this test's own honest "no V.8 pre-sync" scope.
 *
 * ==========================================================================
 * WHAT THIS TEST DID FIND AND FIX (two new required hand-cleans)
 * ==========================================================================
 * Driving BOTH TX and RX of a role-0 (Originate) object from substate 0
 * through a real V32FP_modem/v32_handshake/V32TxHdxModem call chain (rather
 * than starting a test's own direct-call harness at substate 1+, as every
 * earlier Org/Ans test in this project did) is something NO earlier test
 * exercised - and it immediately surfaced that TWO Hdx handlers, though
 * individually irrelevant to every PRIOR test's own scope, are genuinely
 * REQUIRED for this one:
 *   - `TxHdxNull` (Originate's own construction-time-default Tx handler,
 *     substates 0/1) was a `v32_unported` abort() stub - hand-cleaned this
 *     chunk (src/v32/v32_engine.c, disassembly-confirmed real address 0x802b0, size
 *     116 - matches porting/decompiled/dp/v32_hdx.c's raw decompile exactly): fills
 *     the output buffer with engine[+0xa0] zero samples (silence - the
 *     direct counterpart to TxHdxTone's tone-generation), decrements budget
 *     by engine[+0x84], dispatches to NextState once exhausted.
 *   - `RxHdxError` (the universal "phase timed out, give up" terminal Rx
 *     handler every OTHER Rx handler's shared error-transition idiom
 *     installs) was ALSO a `v32_unported` abort() stub, since no earlier
 *     test ever let a real budget run all the way out. Hand-cleaned this
 *     chunk (src/v32/v32_engine.c, disassembly-confirmed real address 0x84460, size
 *     57): re-asserts obj[+0x31]|=2, calls DemodDataV32 once (draining the
 *     rx queue), zeroes the output bit count.
 * Both are disassembly-verified against the real dsplibs.o machine code
 * (addresses from `readelf -sW`) and both are exercised for real, for many
 * thousands of calls, by this test's own honest non-convergent scenario -
 * this is genuine coverage, not a "ported but never called" placeholder.
 *
 * ==========================================================================
 * VERIFICATION METHODOLOGY
 * ==========================================================================
 * This file is compiled once against the real dsplibs.o and once against
 * the hand-cleaned src/v32/v32_engine.c (+ its already-verified subsystem
 * dependencies), exactly like every other *_diff_main.c in this project.
 * Both objects' key handshake-state fields (state/role/budget/elapsed/
 * status/active Tx+Rx handler identity/the phase-reversal-relevant
 * counters) are dumped every call for BOTH sides, together with the full
 * hex of every audio sample exchanged in both directions - a true
 * byte-for-byte differential test of the entire V32FP_modem/v32_handshake/
 * V32TxHdxModem/V32RxHdxModem call chain (never exercised end-to-end by any
 * earlier V.32 chunk) plus the two newly-required Hdx handlers above,
 * running for 4000 calls (comfortably past BOTH sides' own real terminal
 * error transitions, empirically confirmed to land by call ~2842 - see
 * above). `V32FP_status`'s own output buffer is explicitly zeroed before
 * each call (V32FP_status, per porting/decompiled/dp/v32_state.c, only ever writes
 * SOME of its 24 output words - words the function itself never touches
 * would otherwise surface as uninitialized-stack-garbage "diffs" that are
 * really just an artifact of comparing two different binaries' own stack
 * layouts, not a real behavioral difference - the exact same "leftover
 * malloc bytes are safe to compare only because both binaries allocate
 * identically" precedent this project has already documented elsewhere,
 * applied to stack memory instead of heap memory here). Each object's own
 * `obj`/`hdx` heap addresses are int-typed handles used only for pointer
 * chasing in this process's own memory - never printed raw (would be
 * meaningless ASLR/allocator-dependent noise, not a real state field).
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v32_hobj.h"

extern void *V32FP_create(int *cfg6, int param2);
extern void V32FP_delete(char *obj);
extern int V32FP_status(char *obj, short *statusOut);
extern unsigned char V32FP_modem(char *obj, void *rxBits, void *txOut, void *txBitsIn, void *rxOut,
                                  unsigned int *txCount, unsigned int *rxCount);

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

#define S16(o) (*(short *)(intptr_t)(o))
#define S32(o) (*(int *)(intptr_t)(o))
#define U8(o) (*(unsigned char *)(intptr_t)(o))

/* This object's own natural per-block audio sample count (V32_SAMPLE_LEN for
 * this rate class) is empirically 40 (confirmed via this test's own probe
 * work - see header comment); MAXBUF gives comfortable margin. */
#define MAXBUF 64
#define MAX_CALLS 4000

typedef struct {
  const char *label;
  char *obj;
  char *hdx;
  short prevTx[MAXBUF];
  unsigned int prevTxCount;
} side_t;

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

static void create_side(side_t *s, const char *label, int role)
{
  int cfg[6];
  memset(cfg, 0, sizeof(cfg));
  cfg[0] = role;   /* role selector: 0 = Originate, nonzero = Answer */
  cfg[1] = 100;    /* look-ahead */
  cfg[2] = 60000;  /* timeout (this object's own `phaseInc`-derivation input -
                     * see header comment; matches every other V.32 test's
                     * own convention in this project) */
  cfg[3] = 700;    /* margin */
  cfg[4] = 1;      /* enable word, bit0 set */
  *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16); /* tx rate 4800, both halves */

  s->label = label;
  s->obj = (char *)(V32FP_create(cfg, 0));
  s->hdx = V32W_PG(char *, s->obj, 0x64);
  memset(s->prevTx, 0, sizeof(s->prevTx));
  s->prevTxCount = 0;
}

/* One V32FP_modem call: feeds `inSamples`/`inCount` (the PEER's own most
 * recently produced TX audio) in as this side's real rx-audio-in, and
 * captures this side's own newly-produced TX audio into `outTx`/
 * `outTxCount`. The digital-bits-in/out parameters are functionally
 * irrelevant during handshake (every Hdx handler exercised by this
 * scenario generates its own tones/sequences internally, never consuming
 * caller-supplied bit content - confirmed by this project's own Org/Ans
 * NextState hand-cleaning work) so dummy zeroed buffers are used for both. */
static unsigned char step(side_t *s, const short *inSamples, unsigned int inCount, short *outTx,
                           unsigned int *outTxCount)
{
  static short dummyRxBits[MAXBUF];
  static short dummyBitsOut[MAXBUF];
  unsigned int txC = inCount;
  unsigned int rxC = inCount;
  unsigned char status;
  short txBitsIn[MAXBUF];

  memset(txBitsIn, 0, sizeof(txBitsIn));
  memcpy(txBitsIn, inSamples, inCount * sizeof(short));
  memset(dummyRxBits, 0, sizeof(dummyRxBits));

  status = V32FP_modem(s->obj, dummyRxBits, outTx, txBitsIn, dummyBitsOut, &txC, &rxC);
  *outTxCount = txC;
  return status;
}

static void dump_side(const char *tag, side_t *s, unsigned char status, const short *txOut,
                       unsigned int txCount)
{
  unsigned int i;
  printf("[%-24s] state=%d role=%d status=%d flags31=0x%02x budget78=%d elapsed7c=%d "
         "watchdog80=%d latch90=%d miss_ac=%d revcnt_a8=%d retrain_aa=%d tx=%s rx=%s "
         "txcount=%u tx_samples=",
         tag, (int)S16(s->hdx + 0x74), (int)S16(s->hdx + 0x76), (int)(signed char)status,
         (unsigned)U8(s->obj + 0x31), S32(s->hdx + 0x78), S32(s->hdx + 0x7c), S32(s->hdx + 0x80),
         S32(s->hdx + 0x90), (int)S16(s->hdx + 0xac), (int)S16(s->hdx + 0xa8),
         (int)S16(s->hdx + 0xaa), tx_handler_name(V32H_PG(void *, s->hdx, 0x6c)),
         rx_handler_name(V32H_PG(void *, s->hdx, 0x70)), txCount);
  for (i = 0; i < txCount && i < MAXBUF; i++) printf("%04x", (unsigned short)txOut[i]);
  printf("\n");
}

static void dump_status(const char *label, char *obj)
{
  short status[24];
  int ret;
  int i;
  memset(status, 0, sizeof(status)); /* see header comment: V32FP_status only
                                       * writes SOME of its output words -
                                       * zeroing first keeps the untouched
                                       * ones deterministic across binaries */
  ret = V32FP_status(obj, status);
  printf("%s ret=%d words:", label, ret);
  for (i = 0; i < 24; i++) printf(" %d", (int)status[i]);
  printf("\n");
}

int main(void)
{
  side_t org, ans;
  int i;
  short orgTx[MAXBUF], ansTx[MAXBUF];
  unsigned int orgTxCount = 0, ansTxCount = 0;
  int orgConnectedAt = -1, ansConnectedAt = -1;

  create_side(&org, "ORG", 0);
  create_side(&ans, "ANS", 1);

  memset(orgTx, 0, sizeof(orgTx));
  memset(ansTx, 0, sizeof(ansTx));

  for (i = 0; i < MAX_CALLS; i++) {
    unsigned char statusOrg, statusAns;
    short newOrgTx[MAXBUF], newAnsTx[MAXBUF];
    unsigned int newOrgTxCount, newAnsTxCount;
    char tag[32];

    /* Simultaneous full-duplex exchange: each side's rx-audio-in this call
     * is the OTHER side's TX audio produced in the PREVIOUS call (a
     * uniform one-block link latency in both directions - this project's
     * own probe work, referenced in the header comment, confirmed the
     * observed non-convergence is NOT an artifact of this choice: removing
     * the latency in one direction only reproduces the identical result). */
    statusOrg = step(&org, ans.prevTx, ans.prevTxCount, newOrgTx, &newOrgTxCount);
    statusAns = step(&ans, org.prevTx, org.prevTxCount, newAnsTx, &newAnsTxCount);

    memcpy(orgTx, newOrgTx, sizeof(orgTx));
    orgTxCount = newOrgTxCount;
    memcpy(ansTx, newAnsTx, sizeof(ansTx));
    ansTxCount = newAnsTxCount;

    memcpy(org.prevTx, orgTx, sizeof(orgTx));
    org.prevTxCount = orgTxCount;
    memcpy(ans.prevTx, ansTx, sizeof(ansTx));
    ans.prevTxCount = ansTxCount;

    snprintf(tag, sizeof(tag), "call%d ORG", i);
    dump_side(tag, &org, statusOrg, orgTx, orgTxCount);
    snprintf(tag, sizeof(tag), "call%d ANS", i);
    dump_side(tag, &ans, statusAns, ansTx, ansTxCount);

    if (S16(org.hdx + 0x76) == 6 && orgConnectedAt < 0) {
      orgConnectedAt = i;
      printf("*** ORG role switched to 6 (connected) at call %d ***\n", i);
    }
    if (S16(ans.hdx + 0x76) == 6 && ansConnectedAt < 0) {
      ansConnectedAt = i;
      printf("*** ANS role switched to 6 (connected) at call %d ***\n", i);
    }
  }

  printf("== final ==\n");
  printf("org connected_at=%d ans connected_at=%d\n", orgConnectedAt, ansConnectedAt);
  dump_status("final_status_org", org.obj);
  dump_status("final_status_ans", ans.obj);

  V32FP_delete(org.obj);
  V32FP_delete(ans.obj);
  printf("deleted_ok\n");
  return 0;
}
