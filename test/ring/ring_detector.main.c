/*
 * Differential test harness for RingDetector_*.
 *
 * This ONE source file is compiled into two separate binaries:
 *   rd_test_blob - linked against the real dsplibs.o
 *   rd_test_port - linked against ../src/ring/ring_detector.c
 * Both provide identically-named RingDetector_Create/Process/GetLastRing/
 * Delete symbols, so the harness itself never changes between runs - only
 * the implementation behind those symbols does. That guarantees the two
 * runs are driven by byte-identical input and comparison logic; only
 * `diff`-ing their stdout tells you anything.
 *
 * Synthetic input: a deterministic square wave alternating between "ring
 * on" bursts (an oscillating +-16000 tone approximating a ~22.5 Hz ring
 * cadence tone at 8 kHz) and "ring off" gaps (near-zero noise floor), fed
 * through RingDetector_Process in varying chunk sizes to also exercise
 * call-boundary handling.
 */

#include <stdio.h>

extern int *RingDetector_Create(int *cfg);
extern void RingDetector_Delete(int *rd);
extern int RingDetector_Process(int *rd, short *samples, unsigned int count);
extern void RingDetector_GetLastRing(int *rd, int *freq, int *duration);

#define FS 8000
#define HALF_PERIOD 178 /* ~22.5 Hz square wave at 8 kHz */
#define ON_SAMPLES (2 * FS)
#define OFF_SAMPLES (4 * FS)
#define CYCLES 3
#define TOTAL_SAMPLES (CYCLES * (ON_SAMPLES + OFF_SAMPLES))

static short g_samples[TOTAL_SAMPLES];

static void gen_samples(void)
{
  int i, cyc, phase;
  int idx = 0;
  for (cyc = 0; cyc < CYCLES; cyc++) {
    for (i = 0; i < ON_SAMPLES; i++) {
      phase = (i / HALF_PERIOD) & 1;
      g_samples[idx++] = phase ? -16000 : 16000;
    }
    for (i = 0; i < OFF_SAMPLES; i++) {
      /* small deterministic "noise floor", well below any threshold */
      g_samples[idx++] = (short)(((i * 37) % 41) - 20);
    }
  }
}

static void run_config(int fs, int minFreq, int maxFreq, int minOnDur,
                        int minOffDur, int threshold)
{
  int cfg[6];
  int *rd;
  unsigned int pos = 0;
  static const unsigned int chunk_sizes[] = {37, 101, 4096, 1, 800};
  unsigned int chunk_i = 0;
  int call_no = 0;

  cfg[0] = fs;
  cfg[1] = minFreq;
  cfg[2] = maxFreq;
  cfg[3] = minOnDur;
  cfg[4] = minOffDur;
  cfg[5] = threshold;

  printf("== config fs=%d minFreq=%d maxFreq=%d minOnDur=%d minOffDur=%d threshold=%d ==\n",
         fs, minFreq, maxFreq, minOnDur, minOffDur, threshold);

  rd = RingDetector_Create(cfg);
  if (rd == 0) {
    printf("RingDetector_Create returned NULL\n");
    return;
  }

  while (pos < TOTAL_SAMPLES) {
    unsigned int chunk = chunk_sizes[chunk_i % (sizeof(chunk_sizes) / sizeof(chunk_sizes[0]))];
    chunk_i++;
    if (chunk > TOTAL_SAMPLES - pos) {
      chunk = TOTAL_SAMPLES - pos;
    }
    {
      int ev = RingDetector_Process(rd, &g_samples[pos], chunk);
      call_no++;
      if (ev != 0) {
        int freq = 0, dur = 0;
        RingDetector_GetLastRing(rd, &freq, &dur);
        printf("call %d pos %u: event freq=%d duration=%d\n", call_no, pos, freq, dur);
      }
    }
    pos += chunk;
  }

  RingDetector_Delete(rd);
  printf("== done ==\n");
}

int main(void)
{
  gen_samples();

  /* default (non-country-specific) threshold, negative sentinel like
   * modem_get_param(6) default case in RD_create (see entry_points.c) */
  run_config(FS, 15, 80, 120, 120, -3000);

  /* a positive, smaller-magnitude country-specific threshold */
  run_config(FS, 15, 80, 120, 120, 850);

  return 0;
}
