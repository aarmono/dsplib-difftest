/*
 * Differential test harness for the Bell 103/V.21 dp_operations object:
 * B103FP_create/delete/modem.
 *
 * Same convention as ../test/rd_diff_main.c and ../test/fpm_diff_main.c:
 * this ONE source file is compiled into two binaries (one linked against
 * the real dsplibs.o, one against ../src/fsk/v21_b103.c + its table/FPM_*
 * dependencies); both expose identically-named B103FP_* symbols, so the
 * harness itself never changes - only `diff`-ing the two runs' stdout
 * tells you anything.
 *
 * Deterministic synthetic input: a simple LCG generates both the TX bit
 * stream and the RX "audio" sample stream, seeded identically for a given
 * scenario in both binaries (the seed lives in this harness, not exchanged
 * over any protocol), so byte-for-byte reproducibility doesn't depend on
 * anything except both binaries executing the same algorithm on the same
 * inputs - exactly like the RingDetector_* and FPM_* tests' synthetic
 * signals, not a real V.21 handshake.
 *
 * ONE SCENARIO PER PROCESS (argv[1] selects which - see SCENARIOS below,
 * driven by test/build_and_diff_b103.sh): running multiple B103FP_create/
 * modem/delete sequences back-to-back in a single process was tried first
 * and found to make the REAL dsplibs.o eventually segfault deep into a
 * later scenario (confirmed independent of feedRxSamples/RX_IN_CAP - an
 * identical scenario run alone in its own process never crashes at all).
 * This points at some B103-internal cross-instance state in dsplibs.o
 * itself (not this port, which was never even reached when it happened;
 * the crash was inside the blob binary) that real slmodemd's actual usage
 * pattern - one live B103 instance per call, not several accumulated in
 * one process - evidently never exercises either. Isolating each scenario
 * into its own process/heap sidesteps it entirely and is arguably the more
 * faithful test methodology anyway.
 *
 * mode==LocLoop (cfg[0]==2, also dsplibs.o's own B103_CFG_raw default, i.e.
 * what B103FP_create(x, NULL) gives you) is NOT exercised here at all - it
 * crashes the real dsplibs.o unconditionally, on literally the first
 * B103FP_modem() call, independent of any input choice:
 * B103LocLoopNextState's sub-state-0 transition (the one TxHdxStartB103's
 * very first B103NextState[...] dispatch lands on) sets the TX handler
 * (Hdx+8) but never sets the RX handler (Hdx+0x10) - and unlike Originate/
 * Answer, no other code path sets it either for this mode, so it's stuck
 * at its create()-time value of 0/uninitialized. B103FP_modem's RX handler
 * call is a `do { ... } while (remaining != 0)` - it always runs at least
 * once even when the caller passes an RX sample count of 0 - so the very
 * first modem() call always calls through that unset pointer and crashes
 * (confirmed with gdb: EIP=0, called from B103FP_modem). This is a genuine
 * latent bug/dead code path in dsplibs.o itself, not a port issue - real
 * slmodemd's dp_b103_init evidently never relies on the LocLoop/default-cfg
 * path in practice, and this differential test can't meaningfully validate
 * a code path that doesn't survive its first call in the ground truth
 * either. Only cfg[0]==0 (Originate) and cfg[0]==1 (Answer) - the two modes
 * a real call actually uses - are exercised below.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* B103Obj the typed instance (src/fsk/v21_b103.h); RL6: the engine entry points take pointers (the blob's int addresses are ABI-identical at -m32) */
#include "../src/fsk/v21_b103.h"
extern B103Obj *B103FP_create(B103Obj *self, int *cfg);
extern void B103FP_delete(B103Obj *self);
extern unsigned int B103FP_modem(B103Obj *self, int *txBits, short *txOut, short *rxIn, int *rxOut,
                                  unsigned short *txCount, unsigned short *rxCount);

#define MAX_TX_BITS 6
#define TX_OUT_CAP 8192
/*
 * DemodDataB103's own on-stack scratch buffer (`short local_15c[166]`, used
 * for FPM_TONE_generate_demod's per-sample output) has no bounds check
 * against the RX sample count a single B103FP_modem call passes through -
 * exceeding it smashes the stack in the REAL dsplibs.o too (confirmed: the
 * blob binary crashed inside DemodDataB103 before this cap was added).
 * Real slmodemd evidently never calls B103FP_modem with a chunk anywhere
 * near that large in one shot; kept safely under 166 here.
 */
#define RX_IN_CAP 15
#define RX_OUT_CAP 8192

static unsigned int lcg_state;

static unsigned int lcg_next(void)
{
  lcg_state = lcg_state * 1103515245u + 12345u;
  return (lcg_state >> 16) & 0x7fffu;
}

static void run_iterations(const char *label, int *cfg, int feedRxSamples, int iterations,
                            unsigned int seed)
{
  B103Obj *obj;
  int iter;
  static int txBits[MAX_TX_BITS];
  static short txOut[TX_OUT_CAP];
  static short rxIn[RX_IN_CAP];
  static int rxOut[RX_OUT_CAP];
  unsigned short txCount, rxCount;

  lcg_state = seed;

  printf("== %s ==\n", label);
  obj = B103FP_create(0, cfg);
  if (obj == 0) {
    printf("create failed\n");
    return;
  }

  for (iter = 0; iter < iterations; iter++) {
    unsigned int ret;
    int i;
    int nTxBits = (int)(lcg_next() % (MAX_TX_BITS + 1));
    int nRxSamples = feedRxSamples ? (int)(lcg_next() % (RX_IN_CAP + 1)) : 0;

    for (i = 0; i < nTxBits; i++) {
      txBits[i] = (int)(lcg_next() & 1u);
    }
    for (i = 0; i < nRxSamples; i++) {
      /* small-amplitude synthetic "line noise", not full-range random
       * garbage - see file header comment: pure +-15000 white noise drove
       * the AGC/FSD internal state into corners even the real dsplibs.o
       * doesn't handle robustly (confirmed: it crashed too, not just the
       * port), which isn't useful for a *behavioral-equivalence* test. */
      rxIn[i] = (short)((lcg_next() % 4000u) - 2000);
    }
    memset(txOut, 0, sizeof(txOut));
    memset(rxOut, 0, sizeof(rxOut));

    txCount = (unsigned short)nTxBits;
    rxCount = (unsigned short)nRxSamples;

    ret = B103FP_modem(obj, txBits, txOut, rxIn, rxOut, &txCount, &rxCount);

    printf("iter %d: ret=%u nTxBits=%d txCount=%u nRxSamples=%d rxCount=%u\n", iter, ret, nTxBits,
           txCount, nRxSamples, rxCount);

    {
      unsigned int chk = 0;
      int n = txCount < TX_OUT_CAP ? txCount : TX_OUT_CAP;
      for (i = 0; i < n; i++) chk = chk * 131u + (unsigned short)txOut[i];
      printf("  txOutChk=%u\n", chk);
    }
    {
      unsigned int chk = 0;
      int n = rxCount < RX_OUT_CAP ? (int)rxCount : RX_OUT_CAP;
      for (i = 0; i < n; i++) chk = chk * 131u + (unsigned int)rxOut[i];
      printf("  rxOutChk=%u\n", chk);
    }
    if (nRxSamples > 0) {
      /* the RX correlator modifies rxIn in place before demod consumes it */
      unsigned int chk = 0;
      for (i = 0; i < nRxSamples; i++) chk = chk * 131u + (unsigned short)rxIn[i];
      printf("  rxInChk=%u\n", chk);
    }
  }

  B103FP_delete(obj);
  printf("== done ==\n");
}

struct scenario {
  const char *label;
  int mode, w1, w2, w4;
  int useDefaultCfg; /* cfg==NULL -> B103_CFG_raw */
  int feedRxSamples;
  unsigned int seedXor;
};

static const struct scenario SCENARIOS[] = {
  /* mode 0 = Originate: cross cfg[1] (freq selector) x cfg[4] (Hdx+0x18 path) */
  {"mode0(Originate) cfg1=0 cfg4=0", 0, 0, 0, 0, 0, 1, 0u},
  {"mode0(Originate) cfg1=1 cfg4=0", 0, 1, 0, 0, 0, 1, 1u},
  {"mode0(Originate) cfg1=0 cfg4=1", 0, 0, 0, 1, 0, 1, 2u},
  {"mode0(Originate) cfg1=1 cfg4=1", 0, 1, 0, 1, 0, 1, 3u},
  /* mode 1 = Answer: cross cfg[1] (freq selector) */
  {"mode1(Answer) cfg1=0", 1, 0, 0, 0, 0, 1, 4u},
  {"mode1(Answer) cfg1=1", 1, 1, 0, 0, 0, 1, 5u},
  /* mode 2 (LocLoop) and the cfg==NULL default are NOT tested - see file
   * header comment: they crash the real dsplibs.o on the first call. */
};
#define NUM_SCENARIOS (int)(sizeof(SCENARIOS) / sizeof(SCENARIOS[0]))
#define ITERATIONS 400

int main(int argc, char **argv)
{
  int idx;
  const struct scenario *sc;
  int cfg[7];

  if (argc != 2) {
    fprintf(stderr, "usage: %s <scenario-index 0..%d>\n", argv[0], NUM_SCENARIOS - 1);
    return 2;
  }
  idx = atoi(argv[1]);
  if (idx < 0 || idx >= NUM_SCENARIOS) {
    fprintf(stderr, "scenario index out of range\n");
    return 2;
  }
  sc = &SCENARIOS[idx];

  if (sc->useDefaultCfg) {
    run_iterations(sc->label, 0, sc->feedRxSamples, ITERATIONS, 0xC0FFEEu ^ sc->seedXor);
  } else {
    cfg[0] = sc->mode;
    cfg[1] = sc->w1;
    cfg[2] = sc->w2;
    cfg[3] = 0x36b0;
    cfg[4] = sc->w4;
    cfg[5] = 0;
    cfg[6] = 0xc80;
    run_iterations(sc->label, cfg, sc->feedRxSamples, ITERATIONS, 0xC0FFEEu ^ sc->seedXor);
  }

  return 0;
}
