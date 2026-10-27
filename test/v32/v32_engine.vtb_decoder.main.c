/*
 * Differential test harness for V.32/V.32bis's `VTB_decoder` (per-symbol
 * Viterbi trellis decode step, src/v32/v32_datapath.c) - the LAST remaining
 * stub in V.32's Viterbi decoder subsystem (see NOTES.md's "V.32 VTB
 * Viterbi decoder decompile-archaeology", "V.32 VTB table extraction", and
 * "V.32 VTBv32_init hand-clean" entries for the 3 prior chunks this one
 * builds on).
 *
 * ==========================================================================
 * WHY A DIRECT CALL SEQUENCE, NOT A SINGLE CALL
 * ==========================================================================
 * `VTB_decoder` implements a FIXED-LAG (depth-16) traceback: each call
 * advances a 16-slot circular history buffer by one slot and decodes
 * whatever ACS winner was recorded 16 calls ago at that same slot position.
 * A single isolated call would only exercise the (harmless, always-zero,
 * VTBv32_init-initialized) INITIAL buffer content - a "vacuous pass" per
 * this project's own explicit anti-pattern warning (see NOTES.md's
 * `FPM_TONE_create`/`FPM_atan`/`FPM_log10` precedents for the same class of
 * mistake: a test that "passes" without ever exercising the code path that
 * could actually diverge). This test instead drives EACH of the 4 trellis
 * rate families through a 32-call sequence of varied, non-trivial (I,Q)
 * sample pairs - double the fixed-lag depth, so the traceback is fully
 * "primed" (every slot in the circular buffer has been written at least
 * once, and the SECOND half of the sequence exercises genuine steady-state
 * traceback/differential-decode behavior, not just the initial fill).
 *
 * ==========================================================================
 * OBJECT CONSTRUCTION - SAME PATTERN AS test/v32/v32_engine.vtb.main.c
 * ==========================================================================
 * `vtbSubObj` is obtained the same way the real object obtains it -
 * `*(int*)(engine+0x230)+0x18` off a real, fully-constructed `V32FP_create`
 * engine object - then `VTBv32_init(vtbSubObj, rate, 0)` is called for real
 * (now a hand-cleaned, already-differentially-verified function, per the
 * prior chunk) to put the object into the exact per-rate state
 * `VTB_decoder` expects, before this test's own (I,Q) sequence is fed in.
 *
 * ==========================================================================
 * WHAT IS COMPARED, PER CALL
 * ==========================================================================
 * After every one of the 32 calls per rate family:
 *   1. `outBits` - the decoded symbol `VTB_decoder` writes through its 4th
 *      argument.
 *   2. The full internal ACS state: all 8 per-state accumulated path
 *      metrics (vtbSubObj+0x04..+0x13), the circular write-index
 *      (+0x14), and the differential decoder's own carried "prevQuadrant"
 *      state (+0x30) - together these are everything that persists across
 *      calls, so any divergence in the ACS/traceback/differential-decode
 *      logic will show up here even if it happens to not (yet) affect
 *      `outBits` on that particular call.
 *   3. A checksum of the entire 0x200-byte circular history buffer
 *      (vtbSubObj+0x00's pointee) - catches any divergence in the ACS
 *      butterfly writes (predecessor-state index / encoded-symbol fields)
 *      that might not yet be visible via outBits or the metrics alone.
 *
 * Any single byte of divergence anywhere in this per-call dump, for any of
 * the 4 rate families, at any of the 32 calls, is treated as a genuine
 * mismatch - `diff -u` on the two sides' full stdout is the pass/fail
 * criterion, matching every other differential test in this project.
 *
 * ==========================================================================
 * (I,Q) SEQUENCE DESIGN
 * ==========================================================================
 * A simple deterministic LCG (avoids any libc RNG-portability concerns
 * between the two build sides) generates 32 (I,Q) pairs per rate family,
 * scaled into a realistic constellation-sample range (roughly
 * +-24000, comparable to the magnitude of real IMAP/QMAP constellation
 * point values, e.g. DECv32_QMAP4's 12288/-4096) - NOT all-zero, NOT a
 * single repeated value, and large enough in magnitude to exercise both
 * signs of I and Q (all 4 coarse quadrants) and a range of "region"
 * quantization buckets. A different LCG seed per rate family (seeded from
 * the rate value itself) means the 4 families see genuinely different
 * sequences, not the same one 4 times.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v32_hobj.h"

extern void *V32FP_create(int *cfg6, int param2);
extern void V32FP_delete(char *obj);
extern void VTBv32_init(char *vtbSubObj, unsigned int rate, int param3);
extern void VTB_decoder(char *vtbSubObj, short i, short q, short *outBits);

#define S16(o) (*(short *)(intptr_t)(o))
#define U16(o) (*(unsigned short *)(intptr_t)(o))
#define S32(o) (*(int *)(intptr_t)(o))

#define NUM_CALLS 32

struct rate_info {
  unsigned int rate;
  const char *label;
};

static const struct rate_info RATES[] = {
  { 3, "7200/16T" },
  { 2, "9600/32pt" },
  { 4, "12000/64pt" },
  { 5, "14400/128pt" },
};
#define NUM_RATES (int)(sizeof(RATES) / sizeof(RATES[0]))

/* Simple deterministic LCG, scaled to a realistic constellation-sample
 * magnitude (+-24000ish) - portable across both build sides (avoids any
 * libc-rand-implementation difference). */
static unsigned int lcg_state;
static short next_sample(void)
{
  lcg_state = lcg_state * 1103515245u + 12345u;
  /* top 16 bits, reinterpreted as signed short, scaled down slightly so we
   * stay comfortably within a realistic constellation-sample range rather
   * than the full short range. */
  short raw = (short)(lcg_state >> 16);
  return (short)(raw - (raw / 3)); /* ~2/3 scale, still wide-ranging */
}

static unsigned int history_checksum(const short *histPtr)
{
  unsigned int sum = 0;
  int i;
  for (i = 0; i < 0x100 /* 0x200 bytes / 2 */; i++) {
    sum = sum * 131u + (unsigned short)histPtr[i];
  }
  return sum;
}

static void dump_state(const char *label, char *vtbSubObj, int callIdx, short i, short q, short outBits)
{
  const short *hist = V32X_PG(const short *, vtbSubObj, V32_VTB_OFF, 0x00);
  printf("[%-14s call=%2d] I=%6d Q=%6d outBits=%6d "
         "metrics(0..7)=%d,%d,%d,%d,%d,%d,%d,%d wIdx=%d prevQuad=%d hist_csum=%u\n",
         label, callIdx, (int)i, (int)q, (int)outBits,
         (int)S16(vtbSubObj + 0x04), (int)S16(vtbSubObj + 0x06),
         (int)S16(vtbSubObj + 0x08), (int)S16(vtbSubObj + 0x0a),
         (int)S16(vtbSubObj + 0x0c), (int)S16(vtbSubObj + 0x0e),
         (int)S16(vtbSubObj + 0x10), (int)S16(vtbSubObj + 0x12),
         (int)S16(vtbSubObj + 0x14), (int)S16(vtbSubObj + 0x30),
         history_checksum(hist));
}

int main(void)
{
  int cfg[6];
  void *p;
  char *obj, *engine, *vtbSubObj;
  int r;

  memset(cfg, 0, sizeof(cfg));
  cfg[0] = 0;               /* role: Originate */
  cfg[1] = 100;              /* look-ahead */
  cfg[2] = 60000;            /* timeout */
  cfg[3] = 700;              /* margin */
  cfg[4] = 1;                /* enable word, bit0 set */
  *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16); /* tx rate 4800, both halves */

  p = V32FP_create(cfg, 0);
  if (!p) {
    printf("create failed\n");
    return 1;
  }
  obj = (char *)(p);
  engine = V32W_PG(char *, obj, 0x68);
  vtbSubObj = V32E_PG(char *, engine, 0x230) + 0x18;

  printf("== VTB_decoder direct-call sequence (4 trellis rate families x %d calls) ==\n", NUM_CALLS);

  for (r = 0; r < NUM_RATES; r++) {
    const struct rate_info *ri = &RATES[r];
    char tag[64];
    int c;

    VTBv32_init(vtbSubObj, ri->rate, 0);
    snprintf(tag, sizeof(tag), "rate=%u(%s)", ri->rate, ri->label);

    lcg_state = 0x9e3779b1u ^ (ri->rate * 0x1000193u);

    for (c = 0; c < NUM_CALLS; c++) {
      short i = next_sample();
      short q = next_sample();
      short outBits = 0;

      VTB_decoder(vtbSubObj, i, q, &outBits);
      dump_state(tag, vtbSubObj, c, i, q, outBits);
    }
  }

  V32FP_delete(obj);
  printf("== done ==\n");
  return 0;
}
