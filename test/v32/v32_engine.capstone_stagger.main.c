/*
 * V.32 two-object end-to-end capstone differential test, REALISTIC-STAGGER
 * variant: same two real V32FP_* engine objects (Originate role 0, Answer
 * role != 0) and the same real full-duplex V32FP_modem exchange loop as
 * test/v32/v32_engine.capstone.main.c, but with one change - Answer's engine gets
 * a realistic HEAD START before Originate's engine is even created, instead
 * of both objects being cold-started at the exact same simulated instant.
 *
 * ==========================================================================
 * WHY THIS SCENARIO EXISTS (read test/v32/v32_engine.capstone.main.c's own header
 * first - this file does not repeat that finding, it tests a hypothesis
 * about its root cause)
 * ==========================================================================
 * The original capstone test's "simultaneous cold start" scenario found
 * (and root-caused) that two real V32FP_create objects, started at the same
 * instant, do NOT organically converge: Originate's RxHdxPhsReversal
 * watchdog fires at call 1768, just 3 calls (120 samples) AFTER Answer's own
 * analogous guard succeeds at call 1765 - a small, fixed structural margin
 * a synchronous test can never clear. That entry's own hypothesis was that
 * a real deployment's V.8/V.25 pre-synchronization phase (out of this
 * project's V.32-only scope) gives Answer's engine a head start a
 * synchronous test doesn't model.
 *
 * That hypothesis has now been checked against REAL evidence, not just
 * argued: a real back-to-back call between two real dsplibs.o-linked
 * slmodemd instances (AT+MS=132,0,4800,9600 on both sides - V.32bis,
 * automode=0, so V.8 is never invoked at all per slmodemd/modem.c's own
 * IS_FAST_DP macro) was captured in nulltest-logs/{answer,call}-slmodemd.log
 * on 2026-09-29. The real logs show:
 *
 *   answer-slmodemd.log:36:  <401.094314> v32: create...
 *   call-slmodemd.log:96:    <402.274871> Found 2100
 *   call-slmodemd.log:99:    <402.274901> v32: create...
 *
 * i.e. Answer's real V32FP_create fires 1.180587s BEFORE Originate's -
 * caused entirely by the Originate/caller side's own real call-progress
 * dialing overhead (busy/dial-tone/ringback detection, "Found 2100" =
 * answer-tone detection) that happens BEFORE Originate's V32 engine object
 * is even constructed - NOT by any V.8/V.25 protocol exchange (there is
 * none in this call, confirmed by protocol=132/automode=0). Both sides then
 * organically walked dsplibs.o's own STATE_B..STATE_V log all the way to a
 * real CONNECT 9600 (call-slmodemd.log:455, answer-slmodemd.log:236).
 *
 * At the real 9600Hz sample rate and this object's own real 40-sample
 * block size (V32_SAMPLE_LEN for this rate class - see the other capstone
 * file's own header comment), 1.180587s of head start is:
 *     1.180587s * 9600 samples/s = 11333.6 samples
 *     11333.6 samples / 40 samples/call = 283.34 calls
 * This test rounds DOWN to HEAD_START_CALLS=283 (11320 samples, 1.17917s -
 * 1.4ms/one-third of a call block short of the observed real value, an
 * immaterial difference given the mere 3-call/120-sample margin the
 * original scenario came up short by). This is the REAL observed value,
 * not the bare minimum needed (which the original entry already showed is
 * just 3 calls/120 samples) - this chunk is modeling reality, not just
 * patching over the gap.
 *
 * ==========================================================================
 * WHAT'S DIFFERENT FROM v32_capstone_diff_main.c's OWN LOOP
 * ==========================================================================
 * Phase 1 (head start, HEAD_START_CALLS iterations): ONLY the Answer object
 * exists. It is stepped alone, once per call, fed genuine silence (zero rx
 * samples in - the direct counterpart to a real phone line carrying no
 * modem signal from the far end before the far end's own engine starts
 * transmitting). Its own TX output is captured each call but has no peer to
 * receive it (matching reality: Originate's engine does not exist yet).
 *
 * Phase 2 (joint full-duplex, JOINT_CALLS iterations): Originate is created
 * NOW (matching the real log's own "v32: create..." ordering) and the
 * normal two-sided exchange begins exactly as in the original capstone
 * test - each side's rx-audio-in this call is the OTHER side's TX audio
 * from the PREVIOUS call. Answer's very first rx-input in this phase is
 * whatever silence-fed TX Answer itself produced on its own last head-start
 * call (continuity of Answer's own real audio stream - Answer has been
 * transmitting the whole time, per real modem behavior; it's Originate's
 * receiver that was simply not there to hear it yet). Originate's very
 * first rx-input is silence (0 samples), since Answer has not yet heard
 * anything from Originate to echo back at the moment Originate's own
 * engine is created - a real one-block link latency, same convention as
 * the original test.
 *
 * This file does NOT modify or replace the original "simultaneous cold
 * start" scenario (test/v32/v32_engine.capstone.main.c) - that remains a
 * separate, valuable, honest finding in its own right. This is an
 * additional scenario testing a specific, now evidence-backed hypothesis
 * about what a synchronous test was missing.
 *
 * See NOTES.md's dated entry for this chunk for the actual result.
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
 * this rate class) is empirically 40 (confirmed via the original capstone
 * test's own probe work); MAXBUF gives comfortable margin. */
#define MAXBUF 64

/* Realistic Answer-side head start, derived from a real back-to-back
 * slmodemd call's own logged V32FP_create timestamps - see header comment.
 * 283 calls * 40 samples/call = 11320 samples = 1.17917s at 9600Hz, vs. the
 * observed real 1.180587s gap (283.34 calls) - rounded down, ~1.4ms short
 * of the real value, immaterial next to the mere 3-call/120-sample margin
 * the synchronous scenario came up short by. */
#define HEAD_START_CALLS 283

/* Joint full-duplex phase length once Originate is created: comfortably
 * past both sides' own original (synchronous-scenario) terminal watchdog
 * calls (1768/1765/2842) AND long enough afterward to check for continued
 * post-connect stability rather than a one-call touch-and-diverge. */
#define JOINT_CALLS 6000

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
                     * matches every other V.32 test's own convention) */
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
 * recently produced TX audio, or 0/silence when there is no peer yet) in as
 * this side's real rx-audio-in, and captures this side's own newly-produced
 * TX audio into `outTx`/`outTxCount`. Digital-bits-in/out buffers are dummy
 * zeroed buffers - functionally irrelevant during handshake (see the
 * original capstone file's own header comment). */
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
  if (inCount) memcpy(txBitsIn, inSamples, inCount * sizeof(short));
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
  memset(status, 0, sizeof(status));
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
  int orgDisconnectedAfter = -1, ansDisconnectedAfter = -1;
  short silence[MAXBUF];

  memset(silence, 0, sizeof(silence));
  memset(orgTx, 0, sizeof(orgTx));
  memset(ansTx, 0, sizeof(ansTx));

  /* ---- Phase 1: Answer-only head start (realistic pre-Originate window) ---- */
  create_side(&ans, "ANS", 1);

  for (i = 0; i < HEAD_START_CALLS; i++) {
    unsigned char statusAns;
    short newAnsTx[MAXBUF];
    unsigned int newAnsTxCount;
    char tag[40];

    /* Answer hears nothing yet - Originate's engine does not exist. */
    statusAns = step(&ans, silence, 0, newAnsTx, &newAnsTxCount);

    memcpy(ansTx, newAnsTx, sizeof(ansTx));
    ansTxCount = newAnsTxCount;
    memcpy(ans.prevTx, ansTx, sizeof(ansTx));
    ans.prevTxCount = ansTxCount;

    snprintf(tag, sizeof(tag), "headstart%d ANS", i);
    dump_side(tag, &ans, statusAns, ansTx, ansTxCount);
  }

  printf("== head start complete: %d calls, %u samples (%.5fs @9600Hz) ==\n", HEAD_START_CALLS,
         (unsigned)HEAD_START_CALLS * 40u, (HEAD_START_CALLS * 40.0) / 9600.0);

  /* ---- Phase 2: Originate created now, real full-duplex exchange begins ---- */
  create_side(&org, "ORG", 0);
  memset(org.prevTx, 0, sizeof(org.prevTx));
  org.prevTxCount = 0;
  /* ans.prevTx/prevTxCount already hold Answer's own last head-start TX -
   * Answer keeps transmitting continuously, per real modem behavior; it is
   * only Originate's receiver that was previously absent to hear it. */

  for (i = 0; i < JOINT_CALLS; i++) {
    unsigned char statusOrg, statusAns;
    short newOrgTx[MAXBUF], newAnsTx[MAXBUF];
    unsigned int newOrgTxCount, newAnsTxCount;
    char tag[40];

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

    snprintf(tag, sizeof(tag), "joint%d ORG", i);
    dump_side(tag, &org, statusOrg, orgTx, orgTxCount);
    snprintf(tag, sizeof(tag), "joint%d ANS", i);
    dump_side(tag, &ans, statusAns, ansTx, ansTxCount);

    if (S16(org.hdx + 0x76) == 6 && orgConnectedAt < 0) {
      orgConnectedAt = i;
      printf("*** ORG role switched to 6 (connected) at joint call %d ***\n", i);
    }
    if (S16(ans.hdx + 0x76) == 6 && ansConnectedAt < 0) {
      ansConnectedAt = i;
      printf("*** ANS role switched to 6 (connected) at joint call %d ***\n", i);
    }
    if (orgConnectedAt >= 0 && S16(org.hdx + 0x76) != 6 && orgDisconnectedAfter < 0) {
      orgDisconnectedAfter = i;
      printf("*** ORG role LEFT 6 (dropped) at joint call %d ***\n", i);
    }
    if (ansConnectedAt >= 0 && S16(ans.hdx + 0x76) != 6 && ansDisconnectedAfter < 0) {
      ansDisconnectedAfter = i;
      printf("*** ANS role LEFT 6 (dropped) at joint call %d ***\n", i);
    }
  }

  printf("== final ==\n");
  printf("org connected_at=%d ans connected_at=%d org_dropped_after=%d ans_dropped_after=%d\n",
         orgConnectedAt, ansConnectedAt, orgDisconnectedAfter, ansDisconnectedAfter);
  dump_status("final_status_org", org.obj);
  dump_status("final_status_ans", ans.obj);

  V32FP_delete(org.obj);
  V32FP_delete(ans.obj);
  printf("deleted_ok\n");
  return 0;
}
