/*
 * Differential test harness for the V.23 top-level modem object:
 * CreateV23Modem/DeleteV23Modem/V23ModemMain (analogous to
 * test/fsk/v21_b103.main.c for B103FP_create/delete/modem).
 *
 * Same convention: this ONE source file is compiled into two binaries (one
 * linked against the real dsplibs.o, one against ../src/fsk/v23.c + its
 * table/FPM_* dependencies); both expose identically-named CreateV23Modem/
 * DeleteV23Modem/V23ModemMain symbols, so the harness itself never changes.
 *
 * ONE SCENARIO PER PROCESS (argv[1] selects which), for the same reason
 * documented in test/fsk/v21_b103.main.c's header: isolating each scenario
 * into its own process/heap sidesteps any cross-instance state the real
 * dsplibs.o might not tolerate accumulating within one process.
 *
 * A single fixed CHUNK_SIZE is used for every V23ModemMain call (both
 * during the initial answer-tone states 0/1 AND steady state) because
 * V23ModemMain's own states 0/1 branch reuses the RX sample count as the TX
 * generate/silence length (see src/fsk/v23.c's header comment on this) - real
 * callers always pass matching tx/rx counts per block, so this harness does
 * too rather than exercising a combination nothing in the real call graph
 * ever produces.
 *
 * 4 scenarios cover every {rxBackend, answerTone} combination
 * CreateV23Modem accepts, even though real slmodemd usage always pairs a
 * main-channel-RX/answerTone=1 instance with a backward-channel-RX/
 * answerTone=0 instance (or vice versa) - testing all 4 independently is a
 * strictly more thorough check of CreateV23Modem's own branching, matching
 * the spirit of test/fsk/v21_b103.main.c's cross-cfg-word testing.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* U04: V23Modem is the typed instance (src/fsk/v23.h) */
#include "../src/fsk/v23.h"
extern V23Modem *CreateV23Modem(V23Modem *self, short rxBackend, const char *cfg3);
extern void DeleteV23Modem(V23Modem *self);
extern int V23ModemMain(V23Modem *self, int *txBitSrc, int *txBitCount, short *txAudioOut,
                         int txSampleCount, short *rxAudioIn, int rxSampleCount,
                         unsigned int *rxBitsOut, int *nRxBitsOut);

#define CHUNK_SIZE 20
#define TX_AUDIO_CAP 8192
#define RX_BITS_CAP 64

static unsigned int lcg_state;

static unsigned int lcg_next(void)
{
  lcg_state = lcg_state * 1103515245u + 12345u;
  return (lcg_state >> 16) & 0x7fffu;
}

/* cfg3 layout CreateV23Modem expects: byte0 answerTone flag, bytes4-7 rate
 * (real usage: 8000), bytes8-11 margin threshold (real usage: 700) - see
 * src/fsk/v23.c's CreateV23Modem block comment. */
struct v23_cfg3 {
  char answerTone;
  char pad[3];
  int rate;
  int marginThreshold;
};

static void run_iterations(const char *label, short rxBackend, int answerTone, int steadyCalls,
                            unsigned int seed)
{
  V23Modem *obj;
  struct v23_cfg3 cfg3;
  int txBitSrc[CHUNK_SIZE] = {0};
  static short txAudioOut[TX_AUDIO_CAP];
  static short rxAudioIn[CHUNK_SIZE];
  static unsigned int rxBitsOut[RX_BITS_CAP];
  int iter;

  lcg_state = seed;

  memset(&cfg3, 0, sizeof(cfg3));
  cfg3.answerTone = (char)answerTone;
  cfg3.rate = 8000;
  cfg3.marginThreshold = 700;

  printf("== %s ==\n", label);
  obj = CreateV23Modem(0, rxBackend, (const char *)&cfg3);
  if (obj == 0) {
    printf("create failed\n");
    return;
  }

  /* Drive through answer-tone states 0/1 (if answerTone) into steady state.
   * State 0 needs rate*3=24000 samples, state 1 needs rate/20=400 - at
   * CHUNK_SIZE=20 that's 1200+20 calls; cap generously higher in case a
   * config doesn't transition as expected, so a stuck scenario is visible
   * as a bounded, diagnosable loop rather than hanging. */
  {
    int i;
    int txBitCount = 0;
    /* U04: was `int nRxBits;` UNINITIALIZED.  v23FP_rx_progress does not set it while the receiver is still detecting the tone (main channel, states < 10), so
     * the `nRxBits != 0` test below read a stack leftover: a stale stack word, nonzero in every recorded blob run (scenarios 0, 1, 3 left the warm-up loop after its
     * FIRST call: that is what the golden contains), 0 on a 64-bit build (a different stack) - which changed how many warm-up calls ran and so the phase of the
     * transmit tone.  Initialised to a nonzero value that reproduces the recorded behaviour on every build (scenario 2: BwChDem_Progress stores 0 itself and the loop
     * leaves on txBitCount). */
    int nRxBits = -1;
    for (i = 0; i < 1300; i++) {
      int ret = V23ModemMain(obj, txBitSrc, &txBitCount, txAudioOut, CHUNK_SIZE, rxAudioIn,
                              CHUNK_SIZE, rxBitsOut, &nRxBits);
      if (ret != 1 && ret != 0 && ret != 2) break; /* unexpected status, stop early */
      /* once V23ModemMain starts returning something other than the plain
       * "still transitioning" 1, or produces/consumes real bit traffic,
       * treat it as having reached steady state */
      if (nRxBits != 0 || txBitCount != 0) break;
    }
  }

  for (iter = 0; iter < steadyCalls; iter++) {
    int i;
    int txBitCount = CHUNK_SIZE;
    int nRxBits = 0;
    int ret;

    for (i = 0; i < CHUNK_SIZE; i++) {
      txBitSrc[i] = (int)(lcg_next() & 1u);
    }
    for (i = 0; i < CHUNK_SIZE; i++) {
      /* small-amplitude synthetic "line noise" - see test/fsk/v21_b103.main.c's
       * header comment for why full-range noise isn't useful here either. */
      rxAudioIn[i] = (short)((lcg_next() % 4000u) - 2000);
    }
    memset(txAudioOut, 0, sizeof(txAudioOut));
    memset(rxBitsOut, 0, sizeof(rxBitsOut));

    ret = V23ModemMain(obj, txBitSrc, &txBitCount, txAudioOut, CHUNK_SIZE, rxAudioIn, CHUNK_SIZE,
                        rxBitsOut, &nRxBits);

    if (nRxBits < 0) nRxBits = 0;
    if (nRxBits > RX_BITS_CAP) nRxBits = RX_BITS_CAP;

    printf("iter %d: ret=%d txBitCount=%d nRxBits=%d\n", iter, ret, txBitCount, nRxBits);
    {
      unsigned int chk = 0;
      int n = CHUNK_SIZE;
      for (i = 0; i < n; i++) chk = chk * 131u + (unsigned short)txAudioOut[i];
      printf("  txAudioChk=%u\n", chk);
    }
    {
      unsigned int chk = 0;
      for (i = 0; i < nRxBits; i++) chk = chk * 131u + rxBitsOut[i];
      printf("  rxBitsChk=%u\n", chk);
    }
  }

  DeleteV23Modem(obj);
  printf("== done ==\n");
}

struct scenario {
  const char *label;
  short rxBackend;
  int answerTone;
  unsigned int seedXor;
};

static const struct scenario SCENARIOS[] = {
  {"mainChannelRx answerTone=0", 0, 0, 0u},
  {"mainChannelRx answerTone=1", 0, 1, 1u},
  {"backwardChannelRx answerTone=0", 1, 0, 2u},
  {"backwardChannelRx answerTone=1", 1, 1, 3u},
};
#define NUM_SCENARIOS (int)(sizeof(SCENARIOS) / sizeof(SCENARIOS[0]))
#define STEADY_CALLS 200

int main(int argc, char **argv)
{
  int idx;
  const struct scenario *sc;

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
  run_iterations(sc->label, sc->rxBackend, sc->answerTone, STEADY_CALLS, 0xC0FFEEu ^ sc->seedXor);
  return 0;
}
